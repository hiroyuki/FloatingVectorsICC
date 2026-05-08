using System;
using System.Collections.Generic;
using System.IO;
using System.Text;
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

        // Body-tracking components (k4abt) crash on destroy in this project (issue #7
        // adjacent). The body-tracking machinery isn't relevant for calibration, so
        // the calibration window auto-disables it while open and restores on close.
        private static readonly string[] BodyTrackingTypeNames = {
            "BodyTracking.BodyTrackingLive, Assembly-CSharp",
            "BodyTracking.BodyTrackingPlayback, Assembly-CSharp",
        };
        private readonly Dictionary<Behaviour, bool> _suspendedBodyTracking = new Dictionary<Behaviour, bool>();

        private Vector2 _samplesScroll;
        private int _samplesShownCount = -1;

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
            SuspendBodyTracking();
        }

        private void OnDisable()
        {
            EditorApplication.playModeStateChanged -= OnPlayModeStateChanged;
            EditorApplication.update -= OnEditorUpdate;
            Unsubscribe();
            _estimator?.Dispose();
            _estimator = null;
            RestoreBodyTracking();
        }

        /// <summary>
        /// Find every active <c>BodyTrackingLive</c> / <c>BodyTrackingPlayback</c>
        /// component, save its <c>enabled</c> state, and disable it. Reverts on
        /// <see cref="RestoreBodyTracking"/>. We use reflection so the calibration
        /// asmdef stays decoupled from the BodyTracking code in Assembly-CSharp.
        /// </summary>
        private void SuspendBodyTracking()
        {
            foreach (var typeName in BodyTrackingTypeNames)
            {
                var t = Type.GetType(typeName);
                if (t == null) continue;
                var found = UnityEngine.Object.FindObjectsByType(
                    t, FindObjectsInactive.Include, FindObjectsSortMode.None);
                foreach (var o in found)
                {
                    if (o is Behaviour b && !_suspendedBodyTracking.ContainsKey(b))
                    {
                        _suspendedBodyTracking[b] = b.enabled;
                        b.enabled = false;
                    }
                }
            }
            if (_suspendedBodyTracking.Count > 0)
                Debug.Log($"[CalibrationWindow] auto-disabled {_suspendedBodyTracking.Count} body-tracking " +
                          "component(s) to avoid the k4abt destroy crash. Will restore on window close.");
        }

        private void RestoreBodyTracking()
        {
            int restored = 0;
            foreach (var kv in _suspendedBodyTracking)
            {
                if (kv.Key != null)
                {
                    kv.Key.enabled = kv.Value;
                    restored++;
                }
            }
            _suspendedBodyTracking.Clear();
            if (restored > 0)
                Debug.Log($"[CalibrationWindow] restored {restored} body-tracking component(s).");
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
                    if (_suspendedBodyTracking.Count > 0)
                    {
                        EditorGUILayout.HelpBox(
                            $"Body tracking is auto-disabled while this window is open " +
                            $"({_suspendedBodyTracking.Count} component(s) suspended). " +
                            "Closing the window restores them.",
                            MessageType.Info);
                    }

                    using (new EditorGUI.DisabledScope(_boardSpec == null))
                    {
                        if (GUILayout.Button("Capture")) DoCapture();
                    }
                    using (new EditorGUI.DisabledScope(_samples.Count == 0))
                    {
                        if (GUILayout.Button($"Solve & Write extrinsics.yaml ({_samples.Count} sample(s))")) DoSolve();
                    }
                    if (GUILayout.Button("Reset (write identity for all detected serials)")) DoReset();
                    if (GUILayout.Button("Dump Latest Frames (debug)")) DoDumpFrames();
                }
            }

            EditorGUILayout.Space();
            using (new EditorGUILayout.HorizontalScope())
            {
                EditorGUILayout.LabelField($"Samples  ({_samples.Count})", EditorStyles.boldLabel);
                GUILayout.FlexibleSpace();
                using (new EditorGUI.DisabledScope(_samples.Count == 0))
                {
                    if (GUILayout.Button("Clear", GUILayout.Width(70))) _samples.Clear();
                }
            }

            // Auto-scroll to bottom whenever a new sample is appended so the latest is visible.
            if (_samplesShownCount != _samples.Count)
            {
                _samplesScroll.y = float.MaxValue;
                _samplesShownCount = _samples.Count;
            }
            _samplesScroll = EditorGUILayout.BeginScrollView(_samplesScroll,
                GUILayout.MinHeight(120), GUILayout.MaxHeight(360));
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
            EditorGUILayout.EndScrollView();

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

                LogSolveDebug(s, camTrMarker, globalTrColor);

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

        /// <summary>
        /// Save each camera's latest RGB color frame as a PNG to
        /// <c>&lt;extrinsicsRoot&gt;/calibration/_dump/&lt;serial&gt;.png</c>. Lets us
        /// inspect the actual pixels the detector saw when Capture was pressed
        /// (especially useful when one camera "should" detect but doesn't).
        /// </summary>
        private void DoDumpFrames()
        {
            try
            {
                if (_manager == null || _manager.Renderers.Count == 0)
                {
                    SetStatus("Dump aborted: no renderers in scene.", warn: true);
                    return;
                }
                string root = ResolveRoot();
                string dumpDir = Path.Combine(PointCloudRecording.CalibrationDir(root), "_dump");
                Directory.CreateDirectory(dumpDir);
                int saved = 0;
                foreach (var r in _manager.Renderers)
                {
                    if (r == null) continue;
                    if (!_latest.TryGetValue(r, out var f) || !f.HasFrame) continue;

                    // The raw buffer is row-major top-to-bottom (standard image convention),
                    // but Unity's Texture2D treats the first byte as the BOTTOM-left pixel.
                    // Flip rows so the saved PNG matches what the camera actually sees.
                    int rowBytes = f.Width * 3;
                    byte[] flipped = new byte[f.Rgb8.Length];
                    for (int y = 0; y < f.Height; y++)
                    {
                        Buffer.BlockCopy(f.Rgb8, y * rowBytes,
                                         flipped, (f.Height - 1 - y) * rowBytes, rowBytes);
                    }
                    var tex = new Texture2D(f.Width, f.Height, TextureFormat.RGB24, mipChain: false);
                    tex.SetPixelData(flipped, 0);
                    tex.Apply(updateMipmaps: false, makeNoLongerReadable: false);

                    string outPath = Path.Combine(dumpDir, $"{Sanitize(r.deviceSerial)}.png");
                    File.WriteAllBytes(outPath, tex.EncodeToPNG());
                    UnityEngine.Object.DestroyImmediate(tex);
                    saved++;
                }
                SetStatus($"Dumped {saved} frame(s) to {dumpDir}");
            }
            catch (Exception e)
            {
                SetStatus($"Dump failed: {e.Message}", warn: true);
                Debug.LogException(e);
            }
        }

        private static string Sanitize(string s)
        {
            if (string.IsNullOrEmpty(s)) return "unknown";
            foreach (char bad in Path.GetInvalidFileNameChars()) s = s.Replace(bad, '_');
            return s;
        }

        /// <summary>
        /// Verbose log of every step in the centroid solve, plus what the live
        /// renderer transforms end up at after applying the result. Written to
        /// console AND <c>&lt;extrinsicsRoot&gt;/calibration/_dump/solve_debug.log</c>
        /// for offline inspection. Sanity-check landmarks:
        ///   - cam0 should land at world origin with identity rotation
        ///   - sum of cameras' world translations should be ~0 (centroid invariant)
        ///   - cam-pair distances in world should match cam-pair distances in marker frame
        /// </summary>
        private void LogSolveDebug(CaptureSample s, Rigid3d[] camTrMarker, Rigid3d[] globalTrColor)
        {
            var sb = new StringBuilder();
            sb.AppendLine($"=== Solve debug @ {DateTime.UtcNow:o} ===");
            sb.AppendLine($"Sample skew={s.SkewMs:0.0}ms cameras={s.Cameras.Count}");
            sb.AppendLine();

            sb.AppendLine("--- inputs (cam_tr_marker, OpenCV camera frame, meters) ---");
            for (int i = 0; i < s.Cameras.Count; i++)
            {
                var c = s.Cameras[i];
                var m = camTrMarker[i];
                sb.AppendLine($"  [{i}] {c.Serial}");
                sb.AppendLine($"      markers={c.Markers}, corners={c.Corners}");
                sb.AppendLine($"      t = {V(m.Translation)}");
                sb.AppendLine($"      euler(deg, ZYX) = {Eul(m.Rotation)}");
            }

            // marker_tr_cam (camera position seen from the board)
            sb.AppendLine();
            sb.AppendLine("--- marker_tr_cam (= inverse: camera positions in marker frame) ---");
            double cx = 0, cy = 0, cz = 0;
            var markerTrCam = new Rigid3d[camTrMarker.Length];
            for (int i = 0; i < camTrMarker.Length; i++)
            {
                markerTrCam[i] = camTrMarker[i].Inverse();
                cx += markerTrCam[i].Translation[0];
                cy += markerTrCam[i].Translation[1];
                cz += markerTrCam[i].Translation[2];
                sb.AppendLine($"  [{i}] t_in_marker = {V(markerTrCam[i].Translation)}");
            }
            int n = camTrMarker.Length;
            sb.AppendLine($"  centroid_in_marker = ({cx / n:0.000}, {cy / n:0.000}, {cz / n:0.000})");

            sb.AppendLine();
            sb.AppendLine("--- output global_tr_colorCamera (cam0 should be identity) ---");
            for (int i = 0; i < globalTrColor.Length; i++)
            {
                sb.AppendLine($"  [{i}] {s.Cameras[i].Serial}");
                sb.AppendLine($"      t (m) = {V(globalTrColor[i].Translation)}");
                sb.AppendLine($"      euler(deg, ZYX) = {Eul(globalTrColor[i].Rotation)}");
            }

            // Pair-wise distances: invariant should hold (rigid transform preserves distances).
            sb.AppendLine();
            sb.AppendLine("--- pairwise distance check (marker frame vs world; must match) ---");
            for (int i = 0; i < n; i++)
                for (int j = i + 1; j < n; j++)
                {
                    double dM = Dist(markerTrCam[i].Translation, markerTrCam[j].Translation);
                    double dW = Dist(globalTrColor[i].Translation, globalTrColor[j].Translation);
                    sb.AppendLine($"  [{i}]↔[{j}]  marker-frame {dM:0.000}m   world {dW:0.000}m   delta {dW - dM:+0.0000;-0.0000;0}");
                }

            // After-apply Unity transforms: where the renderer GO actually ends up after
            // ExtrinsicsApply. Useful to see the basis-change result.
            sb.AppendLine();
            sb.AppendLine("--- after-apply Unity transforms (renderer.transform localPos/Rot) ---");
            for (int i = 0; i < globalTrColor.Length; i++)
            {
                var ocv = ToObExtrinsicMm(globalTrColor[i]);
                ExtrinsicsApply.ToUnityLocal(in ocv, out var pos, out var rot);
                var euler = rot.eulerAngles;
                sb.AppendLine($"  [{i}] {s.Cameras[i].Serial}");
                sb.AppendLine($"      Unity localPosition  (m) = ({pos.x:0.000}, {pos.y:0.000}, {pos.z:0.000})");
                sb.AppendLine($"      Unity localEulerAngles(°) = ({euler.x:0.0}, {euler.y:0.0}, {euler.z:0.0})");
            }

            string text = sb.ToString();
            Debug.Log("[CalibrationWindow] " + text);
            try
            {
                string root = ResolveRoot();
                string dir = Path.Combine(PointCloudRecording.CalibrationDir(root), "_dump");
                Directory.CreateDirectory(dir);
                File.WriteAllText(Path.Combine(dir, "solve_debug.log"), text);
            }
            catch (Exception e) { Debug.LogWarning("debug log write failed: " + e.Message); }
        }

        private static string V(double[] t) => $"({t[0]:+0.000;-0.000;0}, {t[1]:+0.000;-0.000;0}, {t[2]:+0.000;-0.000;0})";

        private static double Dist(double[] a, double[] b)
        {
            double dx = a[0] - b[0], dy = a[1] - b[1], dz = a[2] - b[2];
            return Math.Sqrt(dx * dx + dy * dy + dz * dz);
        }

        // Row-major 3x3 → ZYX intrinsic Euler angles in degrees (yaw, pitch, roll style).
        private static string Eul(double[] r)
        {
            double sy = Math.Sqrt(r[0] * r[0] + r[3] * r[3]);
            double yaw, pitch, roll;
            if (sy > 1e-6)
            {
                roll = Math.Atan2(r[7], r[8]);
                pitch = Math.Atan2(-r[6], sy);
                yaw = Math.Atan2(r[3], r[0]);
            }
            else
            {
                roll = Math.Atan2(-r[5], r[4]);
                pitch = Math.Atan2(-r[6], sy);
                yaw = 0;
            }
            const double D = 180.0 / Math.PI;
            return $"(roll={roll * D:+0.0;-0.0}, pitch={pitch * D:+0.0;-0.0}, yaw={yaw * D:+0.0;-0.0})";
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
