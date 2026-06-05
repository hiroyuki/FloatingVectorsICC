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
            w.minSize = new Vector2(520, 520);
            w.Show();
        }

        // -------- Inspector-style fields (serialized so the user can leave dialog open) --------
        [SerializeField] private CharucoBoardSpec _boardSpec;
        [SerializeField] private float _maxSkewMs = 50f;
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
            "BodyTracking.BodyTrackingMultiLive, Assembly-CSharp",
            "BodyTracking.BodyTrackingPlayback, Assembly-CSharp",
        };
        private readonly Dictionary<Behaviour, bool> _suspendedBodyTracking = new Dictionary<Behaviour, bool>();

        private Vector2 _samplesScroll;
        private int _samplesShownCount = -1;

        // -------- Oversized GUI styles (built lazily inside OnGUI; GUI.skin is null in OnEnable) --------
        private GUIStyle _bigButton;
        private GUIStyle _bigHeader;
        private GUIStyle _bigSampleHeader;
        private GUIStyle _bigSampleLine;

        private void EnsureStyles()
        {
            if (_bigButton != null) return;
            _bigButton = new GUIStyle(GUI.skin.button)
            {
                fontSize = 22,
                fontStyle = FontStyle.Bold,
                fixedHeight = 56,
                wordWrap = true,
            };
            _bigHeader = new GUIStyle(EditorStyles.boldLabel)
            {
                fontSize = 24,
                fontStyle = FontStyle.Bold,
            };
            _bigSampleHeader = new GUIStyle(EditorStyles.boldLabel)
            {
                fontSize = 18,
                fontStyle = FontStyle.Bold,
            };
            _bigSampleLine = new GUIStyle(EditorStyles.label)
            {
                fontSize = 16,
                wordWrap = true,
            };
        }

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
            // Only the skew gate hard-rejects samples now. Partial detection (some
            // cameras saw the board, others didn't) is FINE — Solve stitches edges
            // across samples via PairwiseCalibrationMath. See DoSolve.
            public bool SkewOk;
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
        /// Find every active <c>BodyTrackingMultiLive</c> / <c>BodyTrackingPlayback</c>
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
            EnsureStyles();
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
                        if (GUILayout.Button("Capture", _bigButton)) DoCapture();
                    }
                    // Solve / Reset / Dump are secondary — keep them small in a single row.
                    using (new EditorGUILayout.HorizontalScope())
                    {
                        using (new EditorGUI.DisabledScope(_samples.Count == 0))
                        {
                            if (GUILayout.Button($"Solve ({_samples.Count})")) DoSolve();
                        }
                        if (GUILayout.Button("Reset")) DoReset();
                        if (GUILayout.Button("Dump")) DoDumpFrames();
                    }
                }
            }

            EditorGUILayout.Space();
            using (new EditorGUILayout.HorizontalScope())
            {
                EditorGUILayout.LabelField($"Samples  ({_samples.Count})", _bigHeader, GUILayout.Height(34));
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
                GUILayout.MinHeight(220), GUILayout.MaxHeight(600));
            for (int i = 0; i < _samples.Count; i++)
            {
                var s = _samples[i];
                int detectedCount = 0;
                if (s.Cameras != null)
                    foreach (var c in s.Cameras) if (c.Detected) detectedCount++;
                int total = s.Cameras != null ? s.Cameras.Count : 0;
                string header = s.SkewOk
                    ? $"#{i} {s.CapturedAtUtc:HH:mm:ss}  skew={s.SkewMs:0.0}ms  detected {detectedCount}/{total}"
                    : $"#{i} {s.CapturedAtUtc:HH:mm:ss}  skew={s.SkewMs:0.0}ms  REJECTED (skew)";
                EditorGUILayout.LabelField(header, _bigSampleHeader);
                if (s.Cameras != null)
                {
                    foreach (var c in s.Cameras)
                        EditorGUILayout.LabelField($"   {c.Serial}: " +
                            (c.Detected ? $"OK (markers={c.Markers}, corners={c.Corners})" : $"no detection (markers={c.Markers})"),
                            _bigSampleLine);
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
                    SkewOk = skewOk,
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

                int detected = 0;
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
                        sample.Cameras.Add(new CameraResult
                        {
                            Serial = s.renderer.deviceSerial,
                            Detected = false,
                            Markers = res.DetectedMarkerCount,
                            Corners = res.InterpolatedCornerCount,
                        });
                        continue;
                    }
                    detected++;
                    sample.Cameras.Add(new CameraResult
                    {
                        Serial = s.renderer.deviceSerial,
                        Detected = true,
                        Markers = res.DetectedMarkerCount,
                        Corners = res.InterpolatedCornerCount,
                        CamTrMarker = new Rigid3d(res.Rotation, res.Translation),
                    });
                }
                _samples.Add(sample);
                SetStatus($"Captured #{_samples.Count - 1}: {detected}/{snapshots.Count} cameras detected " +
                          $"(skew {skewMs:0.0}ms). Pairs in this sample contribute edges to the solve graph.");
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
                // Stable camera index = current renderer order. cam0 (the first renderer
                // in the manager's list) becomes world origin via PairwiseCalibrationMath.
                var serials = new List<string>();
                foreach (var r in _manager.Renderers)
                {
                    if (r == null || string.IsNullOrEmpty(r.deviceSerial)) continue;
                    serials.Add(r.deviceSerial);
                }
                if (serials.Count == 0)
                {
                    SetStatus("Solve aborted: no renderers with serials in scene.", warn: true);
                    return;
                }
                var serialToIdx = new Dictionary<string, int>();
                for (int i = 0; i < serials.Count; i++) serialToIdx[serials[i]] = i;

                // Flatten every detected observation across every skew-accepted sample.
                // Partial-detection samples are FINE — pairs whose two endpoints both
                // detected in the same sample become an edge.
                var observations = new List<PairwiseCalibrationMath.Observation>();
                for (int si = 0; si < _samples.Count; si++)
                {
                    var s = _samples[si];
                    if (!s.SkewOk || s.Cameras == null) continue;
                    foreach (var c in s.Cameras)
                    {
                        if (!c.Detected) continue;
                        if (!serialToIdx.TryGetValue(c.Serial, out var idx)) continue;
                        observations.Add(new PairwiseCalibrationMath.Observation
                        {
                            SampleIndex = si,
                            CameraIndex = idx,
                            CamTrMarker = c.CamTrMarker,
                        });
                    }
                }
                if (observations.Count == 0)
                {
                    SetStatus("Solve aborted: no detections in any sample. Capture again with the board visible.", warn: true);
                    return;
                }

                var solve = PairwiseCalibrationMath.Solve(serials.Count, observations);

                var unreachable = new List<string>();
                for (int i = 0; i < serials.Count; i++)
                    if (!solve.Reachable[i]) unreachable.Add(serials[i]);
                if (unreachable.Count > 0)
                {
                    SetStatus(
                        $"Solve aborted: camera(s) [{string.Join(", ", unreachable)}] have no path to cam0 ({serials[0]}). " +
                        "Capture additional sample(s) where each of them shares the board view with the connected set " +
                        $"({SerialsOfReachable(serials, solve.Reachable)}).",
                        warn: true);
                    return;
                }

                LogSolveDebug(serials, observations, solve);

                var calibrations = new List<PointCloudRecording.DeviceCalibration>(serials.Count);
                for (int i = 0; i < serials.Count; i++)
                {
                    var renderer = FindRenderer(serials[i]);
                    if (renderer == null || !renderer.CameraParam.HasValue)
                    {
                        SetStatus($"Solve aborted: {serials[i]} no longer in scene.", warn: true);
                        return;
                    }
                    var p = renderer.CameraParam.Value;
                    calibrations.Add(new PointCloudRecording.DeviceCalibration
                    {
                        Serial = serials[i],
                        ColorIntrinsic = p.RgbIntrinsic,
                        DepthIntrinsic = p.DepthIntrinsic,
                        ColorDistortion = p.RgbDistortion,
                        DepthDistortion = p.DepthDistortion,
                        DepthToColor = p.Transform,
                        GlobalTrColorCamera = ToObExtrinsicMm(solve.GlobalTrCamera[i]),
                    });
                }

                string root = ResolveRoot();
                PointCloudRecording.WriteExtrinsicsYaml(root, calibrations);
                string outPath = Path.Combine(PointCloudRecording.CalibrationDir(root), "extrinsics.yaml");

                // Auto-apply to the live scene. PointCloudCameraManager.ApplyExtrinsicsToLive
                // only runs once at Start, so without this the renderers keep showing the
                // PREVIOUS calibration until the user re-enters Play. We only apply when the
                // manager reads from the same root we just wrote to — otherwise we'd push a
                // different (stale) yaml onto the cameras. We also honor the manager's
                // applyExtrinsics flag: when it is off the renderers are intentionally kept
                // at the local origin, so auto-applying here would be a regression.
                string applyNote;
                if (_manager == null)
                {
                    applyNote = " Manager not found — re-Play to load.";
                }
                else if (!_manager.applyExtrinsics)
                {
                    applyNote = " NOT auto-applied: manager.applyExtrinsics is off (renderers stay at local origin).";
                }
                else
                {
                    string mgrRoot = _manager.ResolveExtrinsicsRoot();
                    bool sameRoot = string.Equals(
                        Path.GetFullPath(root), Path.GetFullPath(mgrRoot), StringComparison.OrdinalIgnoreCase);
                    if (sameRoot)
                    {
                        _manager.ApplyExtrinsicsToLive();
                        applyNote = " Applied to live renderers.";
                    }
                    else
                    {
                        applyNote = $" NOT auto-applied: manager reads a different root ({mgrRoot}); re-Play or align the roots.";
                    }
                }
                SetStatus($"Wrote {calibrations.Count} entries to {outPath}.{applyNote}");
            }
            catch (Exception e)
            {
                SetStatus($"Solve failed: {e.Message}", warn: true);
                Debug.LogException(e);
            }
        }

        private static string SerialsOfReachable(List<string> serials, bool[] reachable)
        {
            var sb = new StringBuilder();
            for (int i = 0; i < serials.Count; i++)
            {
                if (!reachable[i]) continue;
                if (sb.Length > 0) sb.Append(", ");
                sb.Append(serials[i]);
            }
            return sb.ToString();
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

                // Reuse the same estimator we'd use for Capture so the overlay reflects
                // the actual detector pipeline (gray + equalizeHist + the configured
                // CharucoBoardSpec). When _boardSpec is null we still dump the raw frame.
                bool canAnnotate = _boardSpec != null;
                if (canAnnotate) EnsureEstimator();

                int saved = 0;
                foreach (var r in _manager.Renderers)
                {
                    if (r == null) continue;
                    if (!_latest.TryGetValue(r, out var f) || !f.HasFrame) continue;

                    string serial = Sanitize(r.deviceSerial);
                    WriteRgb8Png(Path.Combine(dumpDir, $"{serial}.png"), f.Rgb8, f.Width, f.Height);

                    if (canAnnotate)
                    {
                        var det = _estimator.DetectAndAnnotate(f.Rgb8, f.Width, f.Height);
                        WriteRgb8Png(Path.Combine(dumpDir, $"{serial}_annotated.png"),
                            det.AnnotatedRgb8, f.Width, f.Height);
                        string ids = det.MarkerIds.Length == 0 ? "none" : string.Join(",", det.MarkerIds);
                        Debug.Log($"[CalibrationWindow] {r.deviceSerial}: markers={det.DetectedMarkerCount} " +
                                  $"interpolated={det.InterpolatedCornerCount} ids=[{ids}]");
                    }
                    saved++;
                }
                string suffix = canAnnotate
                    ? " (raw + _annotated.png — see console for per-camera marker counts)"
                    : " (no Board spec set → raw only; assign one to also dump annotated)";
                SetStatus($"Dumped {saved} frame(s) to {dumpDir}{suffix}");
            }
            catch (Exception e)
            {
                SetStatus($"Dump failed: {e.Message}", warn: true);
                Debug.LogException(e);
            }
        }

        // Save row-major top-to-bottom RGB8 bytes as a PNG, flipping rows to match
        // Texture2D's bottom-up convention.
        private static void WriteRgb8Png(string outPath, byte[] rgb8, int width, int height)
        {
            int rowBytes = width * 3;
            byte[] flipped = new byte[rgb8.Length];
            for (int y = 0; y < height; y++)
            {
                Buffer.BlockCopy(rgb8, y * rowBytes,
                                 flipped, (height - 1 - y) * rowBytes, rowBytes);
            }
            var tex = new Texture2D(width, height, TextureFormat.RGB24, mipChain: false);
            tex.SetPixelData(flipped, 0);
            tex.Apply(updateMipmaps: false, makeNoLongerReadable: false);
            File.WriteAllBytes(outPath, tex.EncodeToPNG());
            UnityEngine.Object.DestroyImmediate(tex);
        }

        private static string Sanitize(string s)
        {
            if (string.IsNullOrEmpty(s)) return "unknown";
            foreach (char bad in Path.GetInvalidFileNameChars()) s = s.Replace(bad, '_');
            return s;
        }

        /// <summary>
        /// Verbose log of the pair-wise solve: which sample bridged each pair,
        /// the BFS chain from cam0 to each camera, and the final Unity transforms
        /// each renderer GO will land at after ExtrinsicsApply. Written to console
        /// AND <c>&lt;extrinsicsRoot&gt;/calibration/_dump/solve_debug.log</c> for
        /// offline inspection. Sanity-check landmarks:
        ///   - cam0 should be identity (t=0, rot=0)
        ///   - long chains (>2 hops) accumulate error — prefer adding direct samples
        /// </summary>
        private void LogSolveDebug(List<string> serials,
                                    List<PairwiseCalibrationMath.Observation> observations,
                                    PairwiseCalibrationMath.SolveResult solve)
        {
            var sb = new StringBuilder();
            sb.AppendLine($"=== Pairwise solve debug @ {DateTime.UtcNow:o} ===");
            sb.AppendLine($"cameras={serials.Count}  observations={observations.Count}");
            sb.AppendLine($"world frame = cam0 = {serials[0]} (cam0 pinned to identity, world axes = cam0 color-camera axes)");
            sb.AppendLine();

            sb.AppendLine("--- per-camera path from cam0 (BFS, shortest hops) ---");
            for (int i = 0; i < serials.Count; i++)
            {
                var path = solve.PathFromCam0[i];
                string hops = path.Count == 0 ? "<unreachable>" : string.Join(" → ", path.ConvertAll(idx => $"[{idx}]"));
                sb.AppendLine($"  [{i}] {serials[i]}: {hops}");
            }

            sb.AppendLine();
            sb.AppendLine("--- pair edges used (a→b, sample that contributed the cam_a_tr_cam_b) ---");
            foreach (var kv in solve.EdgeSampleIndex)
            {
                if (kv.Key.from >= kv.Key.to) continue; // print undirected once
                sb.AppendLine($"  [{kv.Key.from}] {serials[kv.Key.from]} ↔ [{kv.Key.to}] {serials[kv.Key.to]}  (from sample #{kv.Value})");
            }

            sb.AppendLine();
            sb.AppendLine("--- output global_tr_colorCamera ---");
            for (int i = 0; i < serials.Count; i++)
            {
                var g = solve.GlobalTrCamera[i];
                sb.AppendLine($"  [{i}] {serials[i]}{(solve.Reachable[i] ? "" : "  <unreachable>")}");
                sb.AppendLine($"      t (m) = {V(g.Translation)}");
                sb.AppendLine($"      euler(deg, ZYX) = {Eul(g.Rotation)}");
            }

            sb.AppendLine();
            sb.AppendLine("--- after-apply Unity transforms (renderer.transform localPos/Rot) ---");
            for (int i = 0; i < serials.Count; i++)
            {
                var ocv = ToObExtrinsicMm(solve.GlobalTrCamera[i]);
                ExtrinsicsApply.ToUnityLocal(in ocv, out var pos, out var rot);
                var euler = rot.eulerAngles;
                sb.AppendLine($"  [{i}] {serials[i]}");
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
