using System;
using System.Collections.Generic;
using System.IO;
using Orbbec;
using PointCloud;
using UnityEditor;
using UnityEngine;

namespace Calibration.EditorTools
{
    /// <summary>
    /// Editor UI for issue #9 / Phase E multi-camera extrinsic calibration.
    /// Subscribes to each PointCloudRenderer in the active scene, latches the latest
    /// color frame per camera, and on Capture runs MarkerPoseEstimator per camera +
    /// atomic skew gating. On Solve, runs centroid-based world solve and writes
    /// <c>extrinsics.yaml</c> via PointCloudRecording.WriteExtrinsicsYaml.
    /// </summary>
    public class CalibrationWindow : EditorWindow
    {
        [MenuItem("Window/Calibration/Multi-Camera Extrinsic")]
        public static void Open()
        {
            var w = GetWindow<CalibrationWindow>("Calibration");
            w.minSize = new Vector2(420, 360);
            w.Show();
        }

        // -------- Inspector-style fields (serialized so the user can leave dialog open) --------
        [SerializeField] private CharucoBoardSpec _boardSpec;
        [SerializeField] private float _maxSkewMs = 16f;
        [SerializeField] private string _extrinsicsRoot = string.Empty;

        // -------- Runtime state --------
        private PointCloudCameraManager _manager;
        private readonly Dictionary<PointCloudRenderer, LatestFrame> _latest = new Dictionary<PointCloudRenderer, LatestFrame>();
        private readonly Dictionary<PointCloudRenderer, Action<PointCloudRenderer, RawFrameData>> _handlers = new Dictionary<PointCloudRenderer, Action<PointCloudRenderer, RawFrameData>>();
        private MarkerPoseEstimator _estimator;
        private CharucoBoardSpec _estimatorSpecCache;
        private readonly List<CaptureSample> _samples = new List<CaptureSample>();
        private string _status = "";

        private struct LatestFrame
        {
            public byte[] Rgb8;
            public int Width;
            public int Height;
            public ulong TimestampUs;
            public bool HasFrame;
        }

        private struct CameraResult
        {
            public string Serial;
            public bool Detected;
            public int Markers;
            public int Corners;
            public Rigid3d CamTrMarker;
        }

        private struct CaptureSample
        {
            public DateTime CapturedAtUtc;
            public double SkewMs;
            public bool AcceptedAllCameras;
            public List<CameraResult> Cameras;
        }

        private void OnEnable()
        {
            EditorApplication.playModeStateChanged += OnPlayModeStateChanged;
            EditorApplication.update += OnEditorUpdate;
        }

        private void OnDisable()
        {
            EditorApplication.playModeStateChanged -= OnPlayModeStateChanged;
            EditorApplication.update -= OnEditorUpdate;
            Unsubscribe();
            _estimator?.Dispose();
            _estimator = null;
        }

        private void OnPlayModeStateChanged(PlayModeStateChange change)
        {
            if (change == PlayModeStateChange.ExitingPlayMode || change == PlayModeStateChange.ExitingEditMode)
                Unsubscribe();
        }

        private void OnEditorUpdate()
        {
            if (!Application.isPlaying) return;
            if (_manager == null || _manager.Renderers == null || _manager.Renderers.Count == 0)
            {
                _manager = FindFirstObjectByType<PointCloudCameraManager>();
                if (_manager != null) Subscribe();
            }
            else
            {
                // Detect renderer list changes (manager re-spawned). Cheap diff.
                if (_handlers.Count != _manager.Renderers.Count) Subscribe();
            }
        }

        // -------- GUI --------

        private void OnGUI()
        {
            EditorGUILayout.LabelField("Multi-Camera Extrinsic Calibration", EditorStyles.boldLabel);
            EditorGUILayout.Space();

            using (new EditorGUI.DisabledScope(_samples.Count > 0))
            {
                _boardSpec = (CharucoBoardSpec)EditorGUILayout.ObjectField(
                    new GUIContent("Board spec",
                        "Scanned Reality DIN-A3 ChArUco PDF spec. Confirm dictionary against the actual PDF."),
                    _boardSpec, typeof(CharucoBoardSpec), allowSceneObjects: false);
            }

            _maxSkewMs = EditorGUILayout.FloatField(
                new GUIContent("Max skew (ms)",
                    "Atomic gate: if any camera's color frame timestamp differs from the rest by more " +
                    "than this, the entire capture set is discarded."),
                _maxSkewMs);

            _extrinsicsRoot = EditorGUILayout.TextField(
                new GUIContent("Extrinsics root",
                    "Root for calibration/extrinsics.yaml. Empty = Application.persistentDataPath/Recordings/recording. " +
                    "Same convention as PointCloudRecorder.folderPath / PointCloudCameraManager.extrinsicsRoot."),
                _extrinsicsRoot);

            EditorGUILayout.Space();

            using (new EditorGUI.DisabledScope(!Application.isPlaying))
            {
                if (_manager == null)
                {
                    EditorGUILayout.HelpBox(
                        "Enter Play mode and ensure a PointCloudCameraManager is in the scene.",
                        MessageType.Info);
                }
                else
                {
                    EditorGUILayout.LabelField($"Manager: {_manager.name}  |  renderers: {_manager.Renderers.Count}");

                    using (new EditorGUI.DisabledScope(_boardSpec == null))
                    {
                        if (GUILayout.Button("Capture")) DoCapture();
                    }
                    using (new EditorGUI.DisabledScope(_samples.Count == 0))
                    {
                        if (GUILayout.Button($"Solve & Write extrinsics.yaml ({_samples.Count} sample(s))")) DoSolve();
                    }
                    if (GUILayout.Button("Reset (write identity for all detected serials)")) DoReset();
                }
            }

            EditorGUILayout.Space();
            EditorGUILayout.LabelField("Samples", EditorStyles.boldLabel);
            for (int i = 0; i < _samples.Count; i++)
            {
                var s = _samples[i];
                EditorGUILayout.LabelField(
                    $"#{i} {s.CapturedAtUtc:HH:mm:ss}  skew={s.SkewMs:0.0}ms  " +
                    (s.AcceptedAllCameras ? "OK" : "REJECTED"));
                if (s.Cameras != null)
                {
                    foreach (var c in s.Cameras)
                        EditorGUILayout.LabelField($"   {c.Serial}: " +
                            (c.Detected ? $"detected (markers={c.Markers}, corners={c.Corners})" : "no detection"));
                }
            }

            if (!string.IsNullOrEmpty(_status))
            {
                EditorGUILayout.Space();
                EditorGUILayout.HelpBox(_status, MessageType.Info);
            }
        }

        // -------- Subscription --------

        private void Subscribe()
        {
            Unsubscribe();
            if (_manager == null) return;
            foreach (var r in _manager.Renderers)
            {
                if (r == null) continue;
                Action<PointCloudRenderer, RawFrameData> h = OnFrame;
                r.OnRawFramesReady += h;
                _handlers[r] = h;
                _latest[r] = default;
            }
        }

        private void Unsubscribe()
        {
            foreach (var kv in _handlers)
            {
                if (kv.Key != null) kv.Key.OnRawFramesReady -= kv.Value;
            }
            _handlers.Clear();
            _latest.Clear();
        }

        private void OnFrame(PointCloudRenderer src, RawFrameData raw)
        {
            if (raw.ColorByteCount <= 0 || raw.ColorBytes == null) return;
            // Copy out (the raw buffer is pooled and may be overwritten next frame).
            var copy = new byte[raw.ColorByteCount];
            Buffer.BlockCopy(raw.ColorBytes, 0, copy, 0, raw.ColorByteCount);
            _latest[src] = new LatestFrame
            {
                Rgb8 = copy,
                Width = raw.ColorWidth,
                Height = raw.ColorHeight,
                TimestampUs = raw.TimestampUs,
                HasFrame = true,
            };
        }

        // -------- Capture / Solve / Reset --------

        private void DoCapture()
        {
            try
            {
                EnsureEstimator();

                if (_manager.Renderers.Count == 0)
                {
                    SetStatus("No renderers in scene.", warn: true);
                    return;
                }

                // Snapshot all latest frames + timestamps in a single pass.
                var snapshots = new List<(PointCloudRenderer renderer, LatestFrame frame, ObCameraIntrinsic intr, ObCameraDistortion dist)>();
                ulong tMin = ulong.MaxValue, tMax = 0;
                foreach (var r in _manager.Renderers)
                {
                    if (r == null) continue;
                    if (!_latest.TryGetValue(r, out var f) || !f.HasFrame)
                    {
                        SetStatus($"Capture aborted: no frame yet for {r.deviceSerial}.", warn: true);
                        return;
                    }
                    if (!r.CameraParam.HasValue)
                    {
                        SetStatus($"Capture aborted: {r.deviceSerial} has no CameraParam (pipeline not started?).", warn: true);
                        return;
                    }
                    var p = r.CameraParam.Value;
                    snapshots.Add((r, f, p.RgbIntrinsic, p.RgbDistortion));
                    if (f.TimestampUs < tMin) tMin = f.TimestampUs;
                    if (f.TimestampUs > tMax) tMax = f.TimestampUs;
                }

                double skewMs = (tMax - tMin) / 1000.0;
                bool skewOk = skewMs <= _maxSkewMs;

                var sample = new CaptureSample
                {
                    CapturedAtUtc = DateTime.UtcNow,
                    SkewMs = skewMs,
                    AcceptedAllCameras = skewOk,
                    Cameras = new List<CameraResult>(snapshots.Count),
                };

                if (!skewOk)
                {
                    foreach (var s in snapshots)
                        sample.Cameras.Add(new CameraResult { Serial = s.renderer.deviceSerial, Detected = false });
                    _samples.Add(sample);
                    SetStatus($"Capture rejected: skew {skewMs:0.0}ms > {_maxSkewMs:0.0}ms (atomic gate).", warn: true);
                    return;
                }

                bool allDetected = true;
                foreach (var s in snapshots)
                {
                    var distArr = new double[]
                    {
                        s.dist.K1, s.dist.K2, s.dist.P1, s.dist.P2, s.dist.K3,
                    };
                    MarkerPoseEstimator.Result res = _estimator.Estimate(
                        s.frame.Rgb8, s.frame.Width, s.frame.Height,
                        s.intr.Fx, s.intr.Fy, s.intr.Cx, s.intr.Cy,
                        distArr);
                    if (!res.Success)
                    {
                        allDetected = false;
                        sample.Cameras.Add(new CameraResult
                        {
                            Serial = s.renderer.deviceSerial,
                            Detected = false,
                            Markers = res.DetectedMarkerCount,
                            Corners = res.InterpolatedCornerCount,
                        });
                        continue;
                    }
                    sample.Cameras.Add(new CameraResult
                    {
                        Serial = s.renderer.deviceSerial,
                        Detected = true,
                        Markers = res.DetectedMarkerCount,
                        Corners = res.InterpolatedCornerCount,
                        CamTrMarker = new Rigid3d(res.Rotation, res.Translation),
                    });
                }
                sample.AcceptedAllCameras = allDetected;
                _samples.Add(sample);
                SetStatus(allDetected
                    ? $"Captured #{_samples.Count - 1}: {snapshots.Count} cameras detected (skew {skewMs:0.0}ms)."
                    : $"Captured #{_samples.Count - 1}: not all cameras detected (skew {skewMs:0.0}ms). Solve will use later samples.");
            }
            catch (Exception e)
            {
                SetStatus($"Capture failed: {e.Message}", warn: true);
                Debug.LogException(e);
            }
        }

        private void DoSolve()
        {
            try
            {
                // Find the most recent sample where all cameras detected. Multi-sample
                // averaging is deferred (open issue 3 in plan).
                CaptureSample? chosen = null;
                for (int i = _samples.Count - 1; i >= 0; i--)
                {
                    if (_samples[i].AcceptedAllCameras) { chosen = _samples[i]; break; }
                }
                if (chosen == null)
                {
                    SetStatus("Solve aborted: no fully-detected sample yet. Capture again.", warn: true);
                    return;
                }
                var s = chosen.Value;
                var camTrMarker = new Rigid3d[s.Cameras.Count];
                for (int i = 0; i < s.Cameras.Count; i++) camTrMarker[i] = s.Cameras[i].CamTrMarker;

                var globalTrColor = CentroidCalibrationMath.SolveGlobalTrColorCamera(camTrMarker);

                // Build calibration entries by pulling intrinsics + D2C from the live
                // renderers (matched by serial).
                var calibrations = new List<PointCloudRecording.DeviceCalibration>(s.Cameras.Count);
                for (int i = 0; i < s.Cameras.Count; i++)
                {
                    var renderer = FindRenderer(s.Cameras[i].Serial);
                    if (renderer == null || !renderer.CameraParam.HasValue)
                    {
                        SetStatus($"Solve aborted: {s.Cameras[i].Serial} no longer in scene.", warn: true);
                        return;
                    }
                    var p = renderer.CameraParam.Value;
                    calibrations.Add(new PointCloudRecording.DeviceCalibration
                    {
                        Serial = s.Cameras[i].Serial,
                        ColorIntrinsic = p.RgbIntrinsic,
                        DepthIntrinsic = p.DepthIntrinsic,
                        ColorDistortion = p.RgbDistortion,
                        DepthDistortion = p.DepthDistortion,
                        DepthToColor = p.Transform,
                        GlobalTrColorCamera = ToObExtrinsicMm(globalTrColor[i]),
                    });
                }

                string root = ResolveRoot();
                PointCloudRecording.WriteExtrinsicsYaml(root, calibrations);
                string outPath = Path.Combine(PointCloudRecording.CalibrationDir(root), "extrinsics.yaml");
                SetStatus($"Wrote {calibrations.Count} entries to {outPath}.");
            }
            catch (Exception e)
            {
                SetStatus($"Solve failed: {e.Message}", warn: true);
                Debug.LogException(e);
            }
        }

        private void DoReset()
        {
            try
            {
                if (_manager == null || _manager.Renderers.Count == 0)
                {
                    SetStatus("Reset aborted: no renderers in scene.", warn: true);
                    return;
                }
                var calibrations = new List<PointCloudRecording.DeviceCalibration>();
                foreach (var r in _manager.Renderers)
                {
                    if (r == null || !r.CameraParam.HasValue) continue;
                    var p = r.CameraParam.Value;
                    calibrations.Add(new PointCloudRecording.DeviceCalibration
                    {
                        Serial = r.deviceSerial,
                        ColorIntrinsic = p.RgbIntrinsic,
                        DepthIntrinsic = p.DepthIntrinsic,
                        ColorDistortion = p.RgbDistortion,
                        DepthDistortion = p.DepthDistortion,
                        DepthToColor = p.Transform,
                        GlobalTrColorCamera = PointCloudRecording.Identity,
                    });
                }
                if (calibrations.Count == 0)
                {
                    SetStatus("Reset aborted: no renderer has CameraParam yet.", warn: true);
                    return;
                }
                string root = ResolveRoot();
                PointCloudRecording.WriteExtrinsicsYaml(root, calibrations);
                _samples.Clear();
                SetStatus($"Reset: identity extrinsics for {calibrations.Count} device(s).");
            }
            catch (Exception e)
            {
                SetStatus($"Reset failed: {e.Message}", warn: true);
                Debug.LogException(e);
            }
        }

        // -------- Helpers --------

        private void EnsureEstimator()
        {
            if (_estimator != null && _estimatorSpecCache == _boardSpec) return;
            _estimator?.Dispose();
            _estimator = new MarkerPoseEstimator(_boardSpec);
            _estimatorSpecCache = _boardSpec;
        }

        private PointCloudRenderer FindRenderer(string serial)
        {
            if (_manager == null) return null;
            foreach (var r in _manager.Renderers)
                if (r != null && r.deviceSerial == serial) return r;
            return null;
        }

        private string ResolveRoot()
        {
            string p = _extrinsicsRoot;
            if (string.IsNullOrWhiteSpace(p))
                p = Path.Combine(Application.persistentDataPath, "Recordings", "recording");
            else if (!Path.IsPathRooted(p))
                p = Path.Combine(Application.persistentDataPath, p);
            return p;
        }

        private void SetStatus(string msg, bool warn = false)
        {
            _status = msg;
            if (warn) Debug.LogWarning($"[CalibrationWindow] {msg}");
            else Debug.Log($"[CalibrationWindow] {msg}");
            Repaint();
        }

        private static ObExtrinsic ToObExtrinsicMm(Rigid3d r)
        {
            // Rigid3d translation is meters; ObExtrinsic.Trans is mm (Orbbec convention).
            float[] rot = new float[9];
            for (int i = 0; i < 9; i++) rot[i] = (float)r.Rotation[i];
            return new ObExtrinsic
            {
                Rot = rot,
                Trans = new[] { (float)(r.Translation[0] * 1000.0), (float)(r.Translation[1] * 1000.0), (float)(r.Translation[2] * 1000.0) },
            };
        }
    }
}
