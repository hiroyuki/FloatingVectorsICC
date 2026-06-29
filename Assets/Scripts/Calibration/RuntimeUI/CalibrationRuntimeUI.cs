// In-app (Game view) multi-camera extrinsic calibration UI, keyboard-driven.
//
// This is the runtime sibling of Calibration.EditorTools.CalibrationWindow. It
// runs the exact same pipeline — subscribe to every PointCloudRenderer, latch
// the latest color frame, on Capture run MarkerPoseEstimator per camera with
// an atomic skew gate, on Solve run PairwiseCalibrationMath and write
// extrinsics.yaml — but renders live camera previews with the ChArUco detection
// overlay into the Game view so an operator can calibrate "while watching" with
// no Editor window and no mouse (all actions are key-bound).
//
// Why a separate MonoBehaviour instead of reusing CalibrationWindow: the window
// is an EditorWindow (IMGUI in the Editor chrome, not the build). The operator
// runs the actual app, so the controls have to live in the running scene.

using System;
using System.Collections.Generic;
using System.IO;
using System.Text;
using Orbbec;
using PointCloud;
using UnityEngine;

namespace Calibration.RuntimeUI
{
    [AddComponentMenu("Calibration/Calibration Runtime UI")]
    public class CalibrationRuntimeUI : MonoBehaviour
    {
        [Header("Board / gating")]
        [Tooltip("ChArUco board spec asset (same one the Editor CalibrationWindow uses). " +
                 "Required before Capture works.")]
        public CharucoBoardSpec boardSpec;
        [Tooltip("Atomic skew gate: if any camera's color frame timestamp differs from the " +
                 "rest by more than this, the whole Capture set is rejected.")]
        public float maxSkewMs = 50f;
        [Tooltip("Root for calibration/extrinsics.yaml. Empty = same default as " +
                 "PointCloudCameraManager (persistentDataPath/Recordings/recording).")]
        public string extrinsicsRoot = string.Empty;

        [Header("Preview")]
        [Tooltip("Seconds between detection-overlay refreshes. Cameras are annotated " +
                 "round-robin (one per tick) so the OpenCV cost is spread across frames.")]
        [Min(0.02f)]
        public float detectIntervalSec = 0.12f;
        [Tooltip("Show the on-screen UI. Toggle at runtime with the Toggle UI key.")]
        public bool showUI = true;

        [Header("Keys")]
        public KeyCode captureKey = KeyCode.C;
        public KeyCode solveKey = KeyCode.S;
        public KeyCode resetKey = KeyCode.R;
        public KeyCode dumpKey = KeyCode.D;
        public KeyCode toggleApplyKey = KeyCode.A;
        public KeyCode clearSamplesKey = KeyCode.Backspace;
        public KeyCode toggleUiKey = KeyCode.H;

        [Header("Interfering components")]
        [Tooltip("Disable these components while this UI is enabled, restore on disable. " +
                 "Two reasons: (1) k4abt body-tracking components crash on destroy (issue #7); " +
                 "(2) the TSDF debug components bind the SAME keys as calibration — e.g. their " +
                 "'C' triggers recorder playback, which calls DestroyAllRenderers() and kills " +
                 "the live cameras mid-calibration. Suspending them makes calibration a clean, " +
                 "non-destructive mode. Assembly-qualified type names ('Namespace.Type, Assembly').")]
        public bool suspendInterferingComponents = true;
        public string[] suspendComponentTypeNames = {
            "BodyTracking.SkeletonMerger, Assembly-CSharp",
            "BodyTracking.BodyTrackingPlayback, Assembly-CSharp",
            "TSDF.DebugTools.TSDFDebugSession, TSDF",
            "TSDF.MeshCumulative, TSDF",
        };

        // -------- Runtime state --------
        private PointCloudCameraManager _manager;
        private MarkerPoseEstimator _estimator;
        private CharucoBoardSpec _estimatorSpecCache;
        private string _status = "";

        private readonly Dictionary<PointCloudRenderer, LatestFrame> _latest =
            new Dictionary<PointCloudRenderer, LatestFrame>();
        private readonly Dictionary<PointCloudRenderer, Action<PointCloudRenderer, RawFrameData>> _handlers =
            new Dictionary<PointCloudRenderer, Action<PointCloudRenderer, RawFrameData>>();
        private readonly Dictionary<PointCloudRenderer, Preview> _previews =
            new Dictionary<PointCloudRenderer, Preview>();
        private readonly List<CaptureSample> _samples = new List<CaptureSample>();

        private float _detectTimer;
        private int _annotateCursor;
        private byte[] _flipBuffer;

        // Suspend bookkeeping (reflection — keeps this asmdef decoupled from
        // BodyTracking / TSDF, mirrors CalibrationWindow's body-tracking workaround).
        private readonly Dictionary<Behaviour, bool> _suspended = new Dictionary<Behaviour, bool>();

        private GUIStyle _label;
        private GUIStyle _hud;

        private struct LatestFrame
        {
            public byte[] Rgb8;
            public int Width;
            public int Height;
            public ulong TimestampUs;
            public bool HasFrame;
        }

        // Per-camera preview texture plus the most recent detection counts.
        private class Preview
        {
            public Texture2D Tex;
            public int Width;
            public int Height;
            public int Markers;
            public int Corners;
            public bool Detected;
            public bool HasDetectionPass;
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
            public bool SkewOk;
            public List<CameraResult> Cameras;
        }

        private void OnEnable()
        {
            if (suspendInterferingComponents) SuspendInterfering();
        }

        private void OnDisable()
        {
            Unsubscribe();
            _estimator?.Dispose();
            _estimator = null;
            foreach (var p in _previews.Values)
                if (p.Tex != null) Destroy(p.Tex);
            _previews.Clear();
            RestoreInterfering();
        }

        private void Update()
        {
            // (Re)acquire the manager and keep our subscription list in sync with
            // the renderer set (manager spawns renderers in its own Start()).
            if (_manager == null || _manager.Renderers == null || _manager.Renderers.Count == 0)
            {
                var found = FindFirstObjectByType<PointCloudCameraManager>();
                if (found != _manager) { _manager = found; Subscribe(); }
            }
            else if (_handlers.Count != _manager.Renderers.Count)
            {
                Subscribe();
            }

            HandleInput();
            UpdatePreviews();
        }

        private void HandleInput()
        {
            if (Input.GetKeyDown(toggleUiKey)) showUI = !showUI;
            if (Input.GetKeyDown(captureKey)) DoCapture();
            if (Input.GetKeyDown(solveKey)) DoSolve();
            if (Input.GetKeyDown(resetKey)) DoReset();
            if (Input.GetKeyDown(dumpKey)) DoDumpFrames();
            if (Input.GetKeyDown(clearSamplesKey)) { _samples.Clear(); SetStatus("Samples cleared."); }
            if (Input.GetKeyDown(toggleApplyKey)) ToggleApply();
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
                if (kv.Key != null) kv.Key.OnRawFramesReady -= kv.Value;
            _handlers.Clear();
            _latest.Clear();
        }

        private void OnFrame(PointCloudRenderer src, RawFrameData raw)
        {
            if (raw.ColorByteCount <= 0 || raw.ColorBytes == null) return;
            // The raw buffer is pooled and may be overwritten next frame — copy out.
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

        // -------- Preview (live overlay) --------

        // Annotate one camera per tick (round-robin) and upload it to that camera's
        // preview texture. When no board spec is set we still show the raw frame.
        private void UpdatePreviews()
        {
            // The overlay only matters while the UI is shown — skip the OpenCV
            // detection pass entirely when hidden so the running show pays ~nothing.
            if (!showUI) return;
            if (_manager == null || _manager.Renderers.Count == 0) return;
            _detectTimer += Time.unscaledDeltaTime;
            if (_detectTimer < detectIntervalSec) return;
            _detectTimer = 0f;

            int count = _manager.Renderers.Count;
            for (int probe = 0; probe < count; probe++)
            {
                int idx = (_annotateCursor + probe) % count;
                var r = _manager.Renderers[idx];
                if (r == null) continue;
                if (!_latest.TryGetValue(r, out var f) || !f.HasFrame) continue;

                _annotateCursor = (idx + 1) % count;
                RefreshPreview(r, f);
                return; // one camera per tick
            }
        }

        private void RefreshPreview(PointCloudRenderer r, LatestFrame f)
        {
            if (!_previews.TryGetValue(r, out var p))
            {
                p = new Preview();
                _previews[r] = p;
            }

            byte[] pixels = f.Rgb8;
            if (boardSpec != null)
            {
                try
                {
                    EnsureEstimator();
                    var det = _estimator.DetectAndAnnotate(f.Rgb8, f.Width, f.Height);
                    pixels = det.AnnotatedRgb8;
                    p.Markers = det.DetectedMarkerCount;
                    p.Corners = det.InterpolatedCornerCount;
                    p.Detected = det.DetectedMarkerCount > 0;
                    p.HasDetectionPass = true;
                }
                catch (Exception e)
                {
                    SetStatus($"Detect failed for {r.deviceSerial}: {e.Message}", warn: true);
                }
            }

            UploadFlipped(p, pixels, f.Width, f.Height);
        }

        // Sensor RGB8 is row-major top-to-bottom; Texture2D is bottom-up. Flip rows
        // so the preview shows upright (same flip the PNG dump uses).
        private void UploadFlipped(Preview p, byte[] rgb8, int width, int height)
        {
            int rowBytes = width * 3;
            int need = rowBytes * height;
            if (_flipBuffer == null || _flipBuffer.Length < need) _flipBuffer = new byte[need];
            for (int y = 0; y < height; y++)
                Buffer.BlockCopy(rgb8, y * rowBytes, _flipBuffer, (height - 1 - y) * rowBytes, rowBytes);

            if (p.Tex == null || p.Width != width || p.Height != height)
            {
                if (p.Tex != null) Destroy(p.Tex);
                p.Tex = new Texture2D(width, height, TextureFormat.RGB24, mipChain: false);
                p.Width = width;
                p.Height = height;
            }
            p.Tex.SetPixelData(_flipBuffer, 0, 0);
            p.Tex.Apply(updateMipmaps: false);
        }

        // -------- GUI --------

        private void OnGUI()
        {
            if (!showUI) return;
            EnsureStyles();

            if (_manager == null || _manager.Renderers.Count == 0)
            {
                GUI.Label(new Rect(10, 10, 800, 30),
                    "Calibration: waiting for PointCloudCameraManager / renderers...", _hud);
                return;
            }

            DrawPreviewGrid();
            DrawHud();
        }

        private void DrawPreviewGrid()
        {
            int n = _manager.Renderers.Count;
            int cols = Mathf.CeilToInt(Mathf.Sqrt(n));
            int rows = Mathf.CeilToInt(n / (float)cols);
            float hudH = 92f;
            float cellW = Screen.width / (float)cols;
            float cellH = (Screen.height - hudH) / rows;

            for (int i = 0; i < n; i++)
            {
                var r = _manager.Renderers[i];
                int cx = i % cols;
                int cy = i / cols;
                var cell = new Rect(cx * cellW, cy * cellH, cellW - 2, cellH - 2);

                _previews.TryGetValue(r, out var p);
                if (p != null && p.Tex != null)
                {
                    var img = FitAspect(cell, p.Width, p.Height);
                    GUI.DrawTexture(img, p.Tex, ScaleMode.StretchToFill, false);
                }
                else
                {
                    GUI.Box(cell, "no frame");
                }

                string serial = r != null ? r.deviceSerial : "?";
                string det;
                if (p == null || !p.HasDetectionPass)
                    det = boardSpec == null ? "no board spec" : "...";
                else
                    det = (p.Detected ? "● " : "✕ ") + $"M={p.Markers} C={p.Corners}";
                GUI.Label(new Rect(cell.x + 6, cell.y + 4, cell.width - 12, 26),
                    $"[{i}] {serial}   {det}", _label);
            }
        }

        private void DrawHud()
        {
            float y = Screen.height - 88f;
            GUI.Box(new Rect(0, y, Screen.width, 88), GUIContent.none);

            string apply = _manager != null && _manager.applyExtrinsics ? "ON" : "OFF";
            string keys =
                $"[{captureKey}] Capture ({_samples.Count})   " +
                $"[{solveKey}] Solve   " +
                $"[{resetKey}] Reset   " +
                $"[{toggleApplyKey}] Apply:{apply}   " +
                $"[{dumpKey}] Dump   " +
                $"[{clearSamplesKey}] Clear   " +
                $"[{toggleUiKey}] Hide UI";
            GUI.Label(new Rect(10, y + 6, Screen.width - 20, 28), keys, _hud);

            int accepted = 0, detectedTotal = 0;
            foreach (var s in _samples) if (s.SkewOk) { accepted++; }
            GUI.Label(new Rect(10, y + 34, Screen.width - 20, 24),
                $"samples: {_samples.Count} (skew-ok {accepted})   board: {(boardSpec != null ? boardSpec.name : "<none>")}",
                _hud);

            if (!string.IsNullOrEmpty(_status))
                GUI.Label(new Rect(10, y + 58, Screen.width - 20, 26), _status, _hud);
            _ = detectedTotal;
        }

        private static Rect FitAspect(Rect cell, int texW, int texH)
        {
            if (texW <= 0 || texH <= 0) return cell;
            float ar = texW / (float)texH;
            float cellAr = cell.width / cell.height;
            float w, h;
            if (ar > cellAr) { w = cell.width; h = w / ar; }
            else { h = cell.height; w = h * ar; }
            float x = cell.x + (cell.width - w) * 0.5f;
            float yy = cell.y + (cell.height - h) * 0.5f;
            return new Rect(x, yy, w, h);
        }

        private void EnsureStyles()
        {
            if (_label != null) return;
            _label = new GUIStyle(GUI.skin.label)
            {
                fontSize = 16,
                fontStyle = FontStyle.Bold,
                normal = { textColor = Color.white },
            };
            _hud = new GUIStyle(GUI.skin.label)
            {
                fontSize = 16,
                normal = { textColor = Color.white },
                wordWrap = false,
            };
        }

        // -------- Capture / Solve / Reset / Dump (ported from CalibrationWindow) --------

        private void DoCapture()
        {
            try
            {
                if (boardSpec == null) { SetStatus("Capture aborted: no board spec assigned.", warn: true); return; }
                EnsureEstimator();
                if (_manager == null || _manager.Renderers.Count == 0)
                {
                    SetStatus("Capture aborted: no renderers.", warn: true); return;
                }

                var snapshots = new List<(PointCloudRenderer renderer, LatestFrame frame, ObCameraIntrinsic intr, ObCameraDistortion dist)>();
                ulong tMin = ulong.MaxValue, tMax = 0;
                foreach (var r in _manager.Renderers)
                {
                    if (r == null) continue;
                    if (!_latest.TryGetValue(r, out var f) || !f.HasFrame)
                    {
                        SetStatus($"Capture aborted: no frame yet for {r.deviceSerial}.", warn: true); return;
                    }
                    if (!r.CameraParam.HasValue)
                    {
                        SetStatus($"Capture aborted: {r.deviceSerial} has no CameraParam.", warn: true); return;
                    }
                    var p = r.CameraParam.Value;
                    snapshots.Add((r, f, p.RgbIntrinsic, p.RgbDistortion));
                    if (f.TimestampUs < tMin) tMin = f.TimestampUs;
                    if (f.TimestampUs > tMax) tMax = f.TimestampUs;
                }

                double skewMs = (tMax - tMin) / 1000.0;
                bool skewOk = skewMs <= maxSkewMs;

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
                    SetStatus($"Capture rejected: skew {skewMs:0.0}ms > {maxSkewMs:0.0}ms.", warn: true);
                    return;
                }

                int detected = 0;
                foreach (var s in snapshots)
                {
                    var distArr = new double[] { s.dist.K1, s.dist.K2, s.dist.P1, s.dist.P2, s.dist.K3 };
                    var res = _estimator.Estimate(
                        s.frame.Rgb8, s.frame.Width, s.frame.Height,
                        s.intr.Fx, s.intr.Fy, s.intr.Cx, s.intr.Cy, distArr);
                    if (!res.Success)
                    {
                        sample.Cameras.Add(new CameraResult
                        {
                            Serial = s.renderer.deviceSerial, Detected = false,
                            Markers = res.DetectedMarkerCount, Corners = res.InterpolatedCornerCount,
                        });
                        continue;
                    }
                    detected++;
                    sample.Cameras.Add(new CameraResult
                    {
                        Serial = s.renderer.deviceSerial, Detected = true,
                        Markers = res.DetectedMarkerCount, Corners = res.InterpolatedCornerCount,
                        CamTrMarker = new Rigid3d(res.Rotation, res.Translation),
                    });
                }
                _samples.Add(sample);
                SetStatus($"Captured #{_samples.Count - 1}: {detected}/{snapshots.Count} cameras (skew {skewMs:0.0}ms).");
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
                if (_manager == null) { SetStatus("Solve aborted: no manager.", warn: true); return; }
                var serials = new List<string>();
                foreach (var r in _manager.Renderers)
                {
                    if (r == null || string.IsNullOrEmpty(r.deviceSerial)) continue;
                    serials.Add(r.deviceSerial);
                }
                if (serials.Count == 0) { SetStatus("Solve aborted: no renderers with serials.", warn: true); return; }

                var serialToIdx = new Dictionary<string, int>();
                for (int i = 0; i < serials.Count; i++) serialToIdx[serials[i]] = i;

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
                            SampleIndex = si, CameraIndex = idx, CamTrMarker = c.CamTrMarker,
                        });
                    }
                }
                if (observations.Count == 0)
                {
                    SetStatus("Solve aborted: no detections in any sample.", warn: true); return;
                }

                var solve = PairwiseCalibrationMath.Solve(serials.Count, observations);

                var unreachable = new List<string>();
                for (int i = 0; i < serials.Count; i++)
                    if (!solve.Reachable[i]) unreachable.Add(serials[i]);
                if (unreachable.Count > 0)
                {
                    SetStatus(
                        $"Solve aborted: [{string.Join(", ", unreachable)}] have no path to cam0 ({serials[0]}). " +
                        "Capture more samples where they share the board with the connected set.",
                        warn: true);
                    return;
                }

                var calibrations = new List<PointCloudRecording.DeviceCalibration>(serials.Count);
                for (int i = 0; i < serials.Count; i++)
                {
                    var renderer = FindRenderer(serials[i]);
                    if (renderer == null || !renderer.CameraParam.HasValue)
                    {
                        SetStatus($"Solve aborted: {serials[i]} no longer in scene.", warn: true); return;
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

                // Auto-apply to the live scene if the manager reads the same root and
                // applyExtrinsics is on (mirrors CalibrationWindow).
                string applyNote;
                if (!_manager.applyExtrinsics)
                {
                    applyNote = " (apply OFF; press " + toggleApplyKey + " to see it)";
                }
                else
                {
                    string mgrRoot = _manager.ResolveExtrinsicsRoot();
                    bool sameRoot = string.Equals(
                        Path.GetFullPath(root), Path.GetFullPath(mgrRoot), StringComparison.OrdinalIgnoreCase);
                    if (sameRoot) { _manager.ApplyExtrinsicsToLive(); applyNote = " (applied to live)"; }
                    else applyNote = $" (manager root differs: {mgrRoot})";
                }
                SetStatus($"Solved. Wrote {calibrations.Count} entries to {outPath}.{applyNote}");
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
                    SetStatus("Reset aborted: no renderers.", warn: true); return;
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
                    SetStatus("Reset aborted: no renderer has CameraParam yet.", warn: true); return;
                }
                string root = ResolveRoot();
                PointCloudRecording.WriteExtrinsicsYaml(root, calibrations);
                _samples.Clear();
                if (_manager.applyExtrinsics) _manager.ApplyExtrinsicsToLive();
                SetStatus($"Reset: identity extrinsics for {calibrations.Count} device(s).");
            }
            catch (Exception e)
            {
                SetStatus($"Reset failed: {e.Message}", warn: true);
                Debug.LogException(e);
            }
        }

        private void DoDumpFrames()
        {
            try
            {
                if (_manager == null || _manager.Renderers.Count == 0)
                {
                    SetStatus("Dump aborted: no renderers.", warn: true); return;
                }
                string root = ResolveRoot();
                string dumpDir = Path.Combine(PointCloudRecording.CalibrationDir(root), "_dump");
                Directory.CreateDirectory(dumpDir);

                bool canAnnotate = boardSpec != null;
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
                    }
                    saved++;
                }
                SetStatus($"Dumped {saved} frame(s) to {dumpDir}.");
            }
            catch (Exception e)
            {
                SetStatus($"Dump failed: {e.Message}", warn: true);
                Debug.LogException(e);
            }
        }

        private void ToggleApply()
        {
            if (_manager == null) { SetStatus("No manager.", warn: true); return; }
            _manager.applyExtrinsics = !_manager.applyExtrinsics;
            if (_manager.applyExtrinsics) _manager.ApplyExtrinsicsToLive();
            else ResetRendererTransformsToOrigin();
            SetStatus($"applyExtrinsics = {_manager.applyExtrinsics}.");
        }

        // When turning apply OFF, snap renderers back to the manager's local origin
        // so the "before" view matches the pre-calibration layout.
        private void ResetRendererTransformsToOrigin()
        {
            foreach (var r in _manager.Renderers)
            {
                if (r == null) continue;
                r.transform.localPosition = Vector3.zero;
                r.transform.localRotation = Quaternion.identity;
            }
        }

        // -------- Helpers (ported) --------

        private void EnsureEstimator()
        {
            if (_estimator != null && _estimatorSpecCache == boardSpec) return;
            _estimator?.Dispose();
            _estimator = new MarkerPoseEstimator(boardSpec);
            _estimatorSpecCache = boardSpec;
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
            string p = extrinsicsRoot;
            if (string.IsNullOrWhiteSpace(p))
                p = Path.Combine(Application.persistentDataPath, "Recordings", "recording");
            else if (!Path.IsPathRooted(p))
                p = Path.Combine(Application.persistentDataPath, p);
            return p;
        }

        private void SetStatus(string msg, bool warn = false)
        {
            _status = msg;
            if (warn) Debug.LogWarning($"[CalibrationRuntimeUI] {msg}");
            else Debug.Log($"[CalibrationRuntimeUI] {msg}");
        }

        private static ObExtrinsic ToObExtrinsicMm(Rigid3d r)
        {
            float[] rot = new float[9];
            for (int i = 0; i < 9; i++) rot[i] = (float)r.Rotation[i];
            return new ObExtrinsic
            {
                Rot = rot,
                Trans = new[]
                {
                    (float)(r.Translation[0] * 1000.0),
                    (float)(r.Translation[1] * 1000.0),
                    (float)(r.Translation[2] * 1000.0),
                },
            };
        }

        private static void WriteRgb8Png(string outPath, byte[] rgb8, int width, int height)
        {
            int rowBytes = width * 3;
            byte[] flipped = new byte[rgb8.Length];
            for (int y = 0; y < height; y++)
                Buffer.BlockCopy(rgb8, y * rowBytes, flipped, (height - 1 - y) * rowBytes, rowBytes);
            var tex = new Texture2D(width, height, TextureFormat.RGB24, mipChain: false);
            tex.SetPixelData(flipped, 0);
            tex.Apply(updateMipmaps: false, makeNoLongerReadable: false);
            File.WriteAllBytes(outPath, tex.EncodeToPNG());
            Destroy(tex);
        }

        private static string Sanitize(string s)
        {
            if (string.IsNullOrEmpty(s)) return "unknown";
            foreach (char bad in Path.GetInvalidFileNameChars()) s = s.Replace(bad, '_');
            return s;
        }

        // -------- Interfering-component suspend (reflection, mirrors CalibrationWindow) --------

        private void SuspendInterfering()
        {
            if (suspendComponentTypeNames == null) return;
            foreach (var typeName in suspendComponentTypeNames)
            {
                if (string.IsNullOrEmpty(typeName)) continue;
                var t = Type.GetType(typeName);
                if (t == null) continue;
                var found = FindObjectsByType(t, FindObjectsInactive.Include, FindObjectsSortMode.None);
                foreach (var o in found)
                {
                    if (o is Behaviour b && !_suspended.ContainsKey(b))
                    {
                        _suspended[b] = b.enabled;
                        b.enabled = false;
                    }
                }
            }
            if (_suspended.Count > 0)
                Debug.Log($"[CalibrationRuntimeUI] auto-disabled {_suspended.Count} interfering " +
                          "component(s) (body-tracking / TSDF debug) so their keybindings don't fight " +
                          "calibration. Restored on disable.");
        }

        private void RestoreInterfering()
        {
            int restored = 0;
            foreach (var kv in _suspended)
            {
                if (kv.Key != null) { kv.Key.enabled = kv.Value; restored++; }
            }
            _suspended.Clear();
            if (restored > 0)
                Debug.Log($"[CalibrationRuntimeUI] restored {restored} interfering component(s).");
        }
    }
}
