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
//
// Calibration mode is exclusive: on enter it stops recorder playback, disables
// the recorder components, and hides every point-cloud mesh (live + _Playback_*)
// — only raw color matters while aiming cameras. The operator HUD/grid stays on
// display 0; displays 1 and 2 (OS "Display 2/3") get full-rate raw color feeds,
// cams id0/id1 and id2/id3 stacked vertically. All of it is restored on exit
// (playback itself stays stopped — press Play again).

using System;
using System.Collections.Generic;
using System.IO;
using System.Text;
using Orbbec;
using PointCloud;
using UnityEngine;
using UnityEngine.UI;

namespace Calibration.RuntimeUI
{
    [AddComponentMenu("Calibration/Calibration Runtime UI")]
    public class CalibrationRuntimeUI : MonoBehaviour, Shared.IViewToggle
    {
        // ---- IViewToggle ("Calibration mode" in the Control Panel Views list) ----
        // Same lever as toggleActiveKey (F1): entering suspends interfering
        // components and shows the camera grid; exiting restores them.
        // Edit mode is side-effect free: _active is only meaningful after Awake
        // normalizes it, and SetActive would flip other components' enabled
        // flags in the saved scene (no undo). Outside Play the getter reflects
        // startActive and the setter is a no-op (flip startActive in the
        // Inspector to change the launch state).
        public string ViewLabel => "Calibration mode";
        public bool Visible
        {
            get => Application.isPlaying ? _active : startActive;
            set { if (Application.isPlaying) SetActive(value); }
        }

        [Header("Board / gating")]
        [Tooltip("ChArUco board spec asset (same one the Editor CalibrationWindow uses). " +
                 "Required before Capture works.")]
        public CharucoBoardSpec boardSpec;
        [Tooltip("Atomic skew gate: if any camera's color frame timestamp differs from the " +
                 "rest by more than this, the whole Capture set is rejected.")]
        public float maxSkewMs = 50f;
        [Tooltip("Root for calibration/extrinsics.yaml. Empty = same default as " +
                 "SensorManager (persistentDataPath/Recordings/recording).")]
        public string extrinsicsRoot = string.Empty;

        [Header("Preview")]
        [Tooltip("Seconds between detection-overlay refreshes. Cameras are annotated " +
                 "round-robin (one per tick) so the OpenCV cost is spread across frames.")]
        [Min(0.02f)]
        public float detectIntervalSec = 0.12f;
        [Tooltip("Show the on-screen UI. Toggle at runtime with the Toggle UI key.")]
        public bool showUI = true;

        [Header("Keys")]
        [Tooltip("Master enable/disable for the whole calibration UI. When disabled the overlay " +
                 "hides, frame subscription / detection stop, and the suspended body-tracking / " +
                 "TSDF components are restored so the normal show runs. Toggling back on re-suspends " +
                 "them. The component stays enabled so this key keeps working while disabled.")]
        public KeyCode toggleActiveKey = KeyCode.F1;
        [Tooltip("Master active state when the component starts. OFF = the normal show keeps " +
                 "running and nothing is suspended; press the toggle key to enter calibration " +
                 "mode. ON = calibration mode from the first frame (legacy behavior).")]
        public bool startActive = false;
        public KeyCode captureKey = KeyCode.C;
        public KeyCode solveKey = KeyCode.S;
        public KeyCode resetKey = KeyCode.R;
        public KeyCode dumpKey = KeyCode.D;
        public KeyCode clearSamplesKey = KeyCode.Backspace;
        public KeyCode toggleUiKey = KeyCode.H;
        [Tooltip("Enter/exit camera-id assign mode (also Esc to exit). In assign mode the action " +
                 "keys are suspended and the arrow keys reassign ids / pick the world origin.")]
        public KeyCode assignModeKey = KeyCode.I;
        [Tooltip("In assign mode: pin the selected camera as the world origin (cam0).")]
        public KeyCode setOriginKey = KeyCode.O;

        [Header("Calibration mode suppression")]
        [Tooltip("Hide every point-cloud mesh (live renderers AND the recorder's _Playback_* " +
                 "objects) while calibration mode is active; restored on exit. Color frames " +
                 "keep flowing — only the depth visuals go dark. Enforced every frame so " +
                 "late-spawned renderers are caught too.")]
        public bool hidePointClouds = true;
        [Tooltip("Stop SensorRecorder playback on entering calibration mode and keep the " +
                 "recorder components disabled while it is active. Playback does NOT auto-" +
                 "resume on exit — press the recorder's Play again.")]
        public bool stopRecorderPlayback = true;
        [Tooltip("When entering calibration mode with no live renderers (the scene started " +
                 "playbackOnly, or playback freed the cameras), reconnect the live rig via " +
                 "SensorRecorder.SwitchToLive() — calibration needs real cameras. Device " +
                 "re-enumeration takes ~15 s; the previews fill in as streams come up.")]
        public bool switchToLiveOnEnter = true;

        [Header("External color displays")]
        [Tooltip("Mirror the raw color feeds onto extra displays while calibration mode is " +
                 "active: cams id0/id1 stacked vertically on displayCam01, id2/id3 on " +
                 "displayCam23 (camera-id order = cameras.yaml). The operator HUD stays " +
                 "on display 0.")]
        public bool externalColorDisplays = true;
        [Tooltip("0-based display index for cams 0+1 (1 = the OS's 'Display 2').")]
        public int displayCam01 = 1;
        [Tooltip("0-based display index for cams 2+3 (2 = the OS's 'Display 3').")]
        public int displayCam23 = 2;

        [Header("Interfering components")]
        [Tooltip("Disable these components while this UI is enabled, restore on disable. " +
                 "Two reasons: (1) k4abt body-tracking components crash on destroy (issue #7); " +
                 "(2) the TSDF debug components bind the SAME keys as calibration — e.g. their " +
                 "'C' triggers recorder playback, which calls DestroyAllRenderers() and kills " +
                 "the live cameras mid-calibration. Suspending them makes calibration a clean, " +
                 "non-destructive mode. Assembly-qualified type names ('Namespace.Type, Assembly').")]
        public bool suspendInterferingComponents = true;
        public string[] suspendComponentTypeNames = {
            "BodyTracking.SkeletonMerger, BodyTracking",
            "BodyTracking.BodyTrackingPlayback, BodyTracking",
            "TSDF.DebugTools.TSDFDebugSession, TSDF",
            "TSDF.MeshCumulative, TSDF",
        };

        // -------- Runtime state --------
        private SensorManager _manager;
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

        // Master active state (toggled by toggleActiveKey). The component stays enabled
        // so Update keeps polling the toggle key even while the UI is "off".
        private bool _active = true;

        // -------- Camera-id map (calibration/cameras.yaml) --------
        // _camOrder index IS the id (entry 0 = id 0). Disconnected serials are kept so
        // their id survives a replug. _originSerial designates the world-origin camera.
        private readonly List<string> _camOrder = new List<string>();
        private string _originSerial = string.Empty;
        private bool _camMapLoaded;
        private bool _camMapDirty;
        private bool _assignMode;
        private int _assignCursor;
        // Solo view (normal mode): -1 = show all in a grid, >=0 = show only that id full-screen.
        private int _soloId = -1;

        // Suspend bookkeeping (reflection — keeps this asmdef decoupled from
        // BodyTracking / TSDF, mirrors CalibrationWindow's body-tracking workaround).
        private readonly Dictionary<Behaviour, bool> _suspended = new Dictionary<Behaviour, bool>();

        // Scene suppression bookkeeping (hidePointClouds / stopRecorderPlayback).
        private readonly Dictionary<MeshRenderer, bool> _hiddenMeshes = new Dictionary<MeshRenderer, bool>();
        private readonly Dictionary<SensorRecorder, bool> _disabledRecorders = new Dictionary<SensorRecorder, bool>();
        private readonly List<SensorRecorder> _sceneRecorders = new List<SensorRecorder>();

        // External color displays (built on enter, destroyed on exit).
        private ExternalView[] _externalViews;
        private readonly List<GameObject> _externalCanvases = new List<GameObject>();

        // One stacked cell on an external display: raw color + crosshair + label.
        private class ExternalView
        {
            public RawImage Raw;
            public AspectRatioFitter Fitter;
            public Text Label;
            public Texture2D Tex;
            public ulong LastTs;
        }

        private GUIStyle _label;
        private GUIStyle _hud;

        // Hollow red center reticle drawn over each preview. Built once, in pixels.
        private Texture2D _centerRing;
        private const int CenterRingPixels = 28;

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

        private void Awake()
        {
            _active = startActive;
        }

        private void OnEnable()
        {
            if (_active)
            {
                if (suspendInterferingComponents) SuspendInterfering();
                SuppressScene();
                BuildExternalDisplays();
            }
        }

        private void OnDisable()
        {
            Unsubscribe();
            _estimator?.Dispose();
            _estimator = null;
            foreach (var p in _previews.Values)
                if (p.Tex != null) Destroy(p.Tex);
            _previews.Clear();
            if (_centerRing != null) { Destroy(_centerRing); _centerRing = null; }
            DestroyExternalDisplays();
            RestoreScene();
            RestoreInterfering();
        }

        private void Update()
        {
            // The master toggle is polled even while disabled so it can turn back on.
            if (Input.GetKeyDown(toggleActiveKey)) SetActive(!_active);
            if (!_active) return;

            // (Re)acquire the manager and keep our subscription list in sync with
            // the renderer set (manager spawns renderers in its own Start()).
            if (_manager == null || _manager.Renderers == null || _manager.Renderers.Count == 0)
            {
                var found = FindFirstObjectByType<SensorManager>();
                if (found != _manager) { _manager = found; Subscribe(); }
            }
            else if (_handlers.Count != _manager.Renderers.Count)
            {
                Subscribe();
            }

            if (_manager != null && _manager.Renderers != null && _manager.Renderers.Count > 0)
                ReconcileCameraMap();

            HandleInput();
            UpdatePreviews();
            EnforceHiddenPointClouds(); // every frame — catches late-spawned renderers
            UpdateExternalDisplays();
        }

        // Master enable/disable. Off: stop subscription/detection, restore the suspended
        // body-tracking / TSDF components, free preview textures. On: re-suspend them
        // (Update re-subscribes on the next tick).
        private void SetActive(bool value)
        {
            if (value == _active) return;
            _active = value;
            if (_active)
            {
                if (suspendInterferingComponents) SuspendInterfering();
                SuppressScene();
                BuildExternalDisplays();
                SetStatus("Calibration UI enabled.");
            }
            else
            {
                _assignMode = false;
                _soloId = -1;
                Unsubscribe();
                _estimator?.Dispose();
                _estimator = null;
                foreach (var p in _previews.Values)
                    if (p.Tex != null) Destroy(p.Tex);
                _previews.Clear();
                DestroyExternalDisplays();
                RestoreScene();
                RestoreInterfering();
                SetStatus("Calibration UI disabled (normal show restored).");
            }
        }

        private void HandleInput()
        {
            if (Input.GetKeyDown(toggleUiKey)) showUI = !showUI;

            // Assign mode swallows the action keys so arrow/origin/save edits the id map
            // instead of firing Capture/Solve/etc.
            if (_assignMode) { HandleAssignInput(); return; }
            if (Input.GetKeyDown(assignModeKey)) { EnterAssignMode(); return; }

            if (Input.GetKeyDown(captureKey)) DoCapture();
            if (Input.GetKeyDown(solveKey)) DoSolve();
            if (Input.GetKeyDown(resetKey)) DoReset();
            if (Input.GetKeyDown(dumpKey)) DoDumpFrames();
            if (Input.GetKeyDown(clearSamplesKey)) { _samples.Clear(); SetStatus("Samples cleared."); }
            HandleSoloInput();
        }

        // Normal mode: a digit shows only that id full-screen; the same digit again
        // (or 0 on an empty rig) returns to the grid.
        private void HandleSoloInput()
        {
            int count = PresentSerialsInOrder().Count;
            for (int d = 0; d < 10; d++)
            {
                if (!Input.GetKeyDown(KeyCode.Alpha0 + d) && !Input.GetKeyDown(KeyCode.Keypad0 + d)) continue;
                if (d >= count) { SetStatus($"No camera with id {d}."); return; }
                _soloId = (_soloId == d) ? -1 : d;
                SetStatus(_soloId < 0 ? "Showing all cameras." : $"Showing only id {d}.");
                return;
            }
        }

        // -------- Camera-id map --------

        // Lazily load cameras.yaml, then fold in any present serials not yet in the map
        // (appended at the end = next free id) and pick a sane default origin.
        private void ReconcileCameraMap()
        {
            if (!_camMapLoaded)
            {
                try
                {
                    var map = PointCloudRecording.ReadCamerasYaml(ResolveRoot());
                    _camOrder.Clear();
                    if (map != null)
                    {
                        foreach (var e in map.Cameras)
                            if (e != null && !string.IsNullOrEmpty(e.Serial) && !_camOrder.Contains(e.Serial))
                                _camOrder.Add(e.Serial);
                        _originSerial = map.OriginSerial ?? string.Empty;
                    }
                }
                catch (Exception e)
                {
                    SetStatus($"cameras.yaml load failed: {e.Message}", warn: true);
                    _camOrder.Clear();
                    _originSerial = string.Empty;
                }
                _camMapLoaded = true;
            }

            foreach (var r in _manager.Renderers)
            {
                if (r == null || string.IsNullOrEmpty(r.deviceSerial)) continue;
                if (!_camOrder.Contains(r.deviceSerial)) { _camOrder.Add(r.deviceSerial); _camMapDirty = true; }
            }

            // Default origin = lowest-id present camera if none chosen / chosen one absent.
            if (string.IsNullOrEmpty(_originSerial) || !IsPresent(_originSerial))
            {
                var present = PresentSerialsInOrder();
                if (present.Count > 0) _originSerial = present[0];
            }
        }

        private void EnterAssignMode()
        {
            _assignMode = true;
            _assignCursor = 0;
            SetStatus("Assign mode: arrows select  0-9 set id  " +
                      $"[{setOriginKey}] origin  Enter save  [{assignModeKey}]/Esc exit");
        }

        private void ExitAssignMode()
        {
            _assignMode = false;
            SetStatus(_camMapDirty ? "Assign mode closed (unsaved changes — Enter in assign mode saves)."
                                   : "Assign mode closed.");
        }

        private void HandleAssignInput()
        {
            if (Input.GetKeyDown(assignModeKey) || Input.GetKeyDown(KeyCode.Escape)) { ExitAssignMode(); return; }

            var present = PresentSerialsInOrder();
            if (present.Count == 0) return;
            _assignCursor = Mathf.Clamp(_assignCursor, 0, present.Count - 1);

            // Arrows only move the selection (focus) — they never change ids. The grid
            // is laid out row-major with this many columns, so ↑/↓ jump a whole row.
            int cols = Mathf.CeilToInt(Mathf.Sqrt(present.Count));
            if (Input.GetKeyDown(KeyCode.RightArrow)) { _assignCursor = (_assignCursor + 1) % present.Count; return; }
            if (Input.GetKeyDown(KeyCode.LeftArrow)) { _assignCursor = (_assignCursor - 1 + present.Count) % present.Count; return; }
            if (Input.GetKeyDown(KeyCode.UpArrow)) { int tgt = _assignCursor - cols; if (tgt >= 0) _assignCursor = tgt; return; }
            if (Input.GetKeyDown(KeyCode.DownArrow)) { int tgt = _assignCursor + cols; if (tgt < present.Count) _assignCursor = tgt; return; }
            if (Input.GetKeyDown(setOriginKey))
            {
                _originSerial = present[_assignCursor];
                _camMapDirty = true;
                SetStatus($"origin = id {CamId(_originSerial)} ({_originSerial}) (unsaved; Enter to save).");
                return;
            }
            if (Input.GetKeyDown(KeyCode.Return) || Input.GetKeyDown(KeyCode.KeypadEnter)) { SaveCameraMap(); return; }

            // Direct id entry: type a digit to assign that id to the selected camera
            // (the others shift to keep ids contiguous).
            for (int d = 0; d < 10; d++)
            {
                if (Input.GetKeyDown(KeyCode.Alpha0 + d) || Input.GetKeyDown(KeyCode.Keypad0 + d))
                {
                    SetSelectedId(d);
                    return;
                }
            }
        }

        // Move the selected (present) camera to id == target (clamped to the present
        // range), shifting the others so ids stay contiguous 0..n-1.
        private void SetSelectedId(int target)
        {
            var present = PresentSerialsInOrder();
            if (present.Count == 0) return;
            int from = Mathf.Clamp(_assignCursor, 0, present.Count - 1);
            target = Mathf.Clamp(target, 0, present.Count - 1);
            string serial = present[from];
            if (target != from)
            {
                present.RemoveAt(from);
                present.Insert(target, serial);
                ApplyPresentOrder(present);
                _camMapDirty = true;
            }
            _assignCursor = target;
            SetStatus($"set {serial} -> id {target} (unsaved; Enter to save).");
        }

        // Rewrite _camOrder so the present serials appear in newPresent order while any
        // disconnected serials keep their absolute slots (so their id survives a replug).
        private void ApplyPresentOrder(List<string> newPresent)
        {
            var rebuilt = new List<string>(_camOrder.Count);
            int p = 0;
            foreach (var s in _camOrder)
            {
                if (IsPresent(s)) { if (p < newPresent.Count) rebuilt.Add(newPresent[p++]); }
                else rebuilt.Add(s);
            }
            while (p < newPresent.Count) rebuilt.Add(newPresent[p++]);
            _camOrder.Clear();
            _camOrder.AddRange(rebuilt);
        }

        private void SaveCameraMap()
        {
            try
            {
                var map = new PointCloudRecording.CameraIdMap { OriginSerial = _originSerial };
                foreach (var serial in _camOrder)
                    map.Cameras.Add(new PointCloudRecording.CameraIdEntry { Serial = serial });
                PointCloudRecording.WriteCamerasYaml(ResolveRoot(), map);
                _camMapDirty = false;
                string outPath = Path.Combine(PointCloudRecording.CalibrationDir(ResolveRoot()), "cameras.yaml");
                SetStatus($"Saved {map.Cameras.Count} camera id(s), origin=id {CamId(_originSerial)} to {outPath}.");
            }
            catch (Exception e)
            {
                SetStatus($"Save cameras.yaml failed: {e.Message}", warn: true);
                Debug.LogException(e);
            }
        }

        private List<string> PresentSerialsInOrder()
        {
            var present = new List<string>();
            foreach (var serial in _camOrder)
                if (IsPresent(serial)) present.Add(serial);
            return present;
        }

        private bool IsPresent(string serial) => FindRenderer(serial) != null;

        // Displayed id == position among present cameras (contiguous 0..n-1), so what the
        // operator types in assign mode matches the label on screen.
        private int CamId(string serial) => PresentSerialsInOrder().IndexOf(serial);

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
            p.Tex = UploadFlippedTo(p.Tex, rgb8, width, height);
            p.Width = width;
            p.Height = height;
        }

        // Shared flip+upload into an arbitrary texture (recreated on size change).
        private Texture2D UploadFlippedTo(Texture2D tex, byte[] rgb8, int width, int height)
        {
            int rowBytes = width * 3;
            int need = rowBytes * height;
            if (_flipBuffer == null || _flipBuffer.Length < need) _flipBuffer = new byte[need];
            for (int y = 0; y < height; y++)
                Buffer.BlockCopy(rgb8, y * rowBytes, _flipBuffer, (height - 1 - y) * rowBytes, rowBytes);

            if (tex == null || tex.width != width || tex.height != height)
            {
                if (tex != null) Destroy(tex);
                tex = new Texture2D(width, height, TextureFormat.RGB24, mipChain: false);
            }
            tex.SetPixelData(_flipBuffer, 0, 0);
            tex.Apply(updateMipmaps: false);
            return tex;
        }

        // -------- GUI --------

        private void OnGUI()
        {
            if (!_active || !showUI) return;
            EnsureStyles();

            if (_manager == null || _manager.Renderers.Count == 0)
            {
                // Bottom-left: the top-left corner belongs to MultiCameraDebugView's
                // header, and the bottom HUD box only exists once renderers are up.
                GUI.Label(new Rect(10, Screen.height - 34, 800, 26),
                    "Calibration: waiting for SensorManager / renderers...", _hud);
                return;
            }

            DrawPreviewGrid();
            DrawHud();
        }

        private void DrawPreviewGrid()
        {
            // Lay out cells in camera-id order (cameras.yaml), not device-enumeration order.
            var order = PresentSerialsInOrder();
            int n = order.Count;
            if (n == 0) { GUI.Box(new Rect(10, 10, 300, 30), "no cameras"); return; }
            float hudH = 92f;

            // Solo view (normal mode only): one camera filling the whole preview area.
            if (!_assignMode && _soloId >= 0 && _soloId < n)
            {
                DrawCameraCell(new Rect(0, 0, Screen.width, Screen.height - hudH), order[_soloId], _soloId, selected: false);
                return;
            }

            int cols = Mathf.CeilToInt(Mathf.Sqrt(n));
            int rows = Mathf.CeilToInt(n / (float)cols);
            float cellW = Screen.width / (float)cols;
            float cellH = (Screen.height - hudH) / rows;

            for (int i = 0; i < n; i++)
            {
                int cx = i % cols;
                int cy = i / cols;
                var cell = new Rect(cx * cellW, cy * cellH, cellW - 2, cellH - 2);
                DrawCameraCell(cell, order[i], i, selected: _assignMode && i == _assignCursor);
            }
        }

        // Draws one camera's preview + id/origin/detection label into the given rect.
        private void DrawCameraCell(Rect cell, string serial, int id, bool selected)
        {
            var r = FindRenderer(serial);
            Preview p = null;
            if (r != null) _previews.TryGetValue(r, out p);
            Rect imgRect = cell;
            if (p != null && p.Tex != null)
            {
                imgRect = FitAspect(cell, p.Width, p.Height);
                GUI.DrawTexture(imgRect, p.Tex, ScaleMode.StretchToFill, false);
            }
            else
            {
                GUI.Box(cell, "no frame");
            }

            if (selected) DrawCellBorder(cell, Color.yellow);

            // Hollow red center reticle at the image center.
            DrawCenterRing(imgRect);

            string det;
            if (p == null || !p.HasDetectionPass)
                det = boardSpec == null ? "no board spec" : "...";
            else
                det = (p.Detected ? "● " : "✕ ") + $"M={p.Markers} C={p.Corners}";

            string idTag = $"ID {id}";
            if (serial == _originSerial) idTag += " ★origin";
            if (selected) idTag = "▶ " + idTag;

            _label.normal.textColor = selected ? Color.yellow : Color.white;
            GUI.Label(new Rect(cell.x + 6, cell.y + 4, cell.width - 12, 26),
                $"[{idTag}] {serial}   {det}", _label);
            _label.normal.textColor = Color.white;
        }

        // Draws the 1px red center crosshair (full width/height of the image) plus the
        // hollow red reticle, centered on the given (image) rect.
        private void DrawCenterRing(Rect img)
        {
            float cx = img.x + img.width * 0.5f;
            float cy = img.y + img.height * 0.5f;

            var prev = GUI.color;
            GUI.color = Color.red;
            // Vertical line through the horizontal center, horizontal line through the
            // vertical center. 1px each.
            GUI.DrawTexture(new Rect(cx - 0.5f, img.y, 1f, img.height), Texture2D.whiteTexture);
            GUI.DrawTexture(new Rect(img.x, cy - 0.5f, img.width, 1f), Texture2D.whiteTexture);
            GUI.color = prev;

            EnsureCenterRing();
            float s = CenterRingPixels;
            GUI.DrawTexture(new Rect(cx - s * 0.5f, cy - s * 0.5f, s, s),
                _centerRing, ScaleMode.StretchToFill, alphaBlend: true);
        }

        // Builds a transparent texture with a thin anti-aliased red ring (not filled).
        private void EnsureCenterRing()
        {
            if (_centerRing != null) return;
            const int N = 32;
            const float rc = 11f;       // band center radius
            const float halfThk = 1.4f; // half ring thickness (px)
            var tex = new Texture2D(N, N, TextureFormat.RGBA32, mipChain: false);
            tex.filterMode = FilterMode.Bilinear;
            tex.wrapMode = TextureWrapMode.Clamp;
            float center = (N - 1) * 0.5f;
            var px = new Color32[N * N];
            for (int y = 0; y < N; y++)
            {
                for (int x = 0; x < N; x++)
                {
                    float dx = x - center, dy = y - center;
                    float d = Mathf.Sqrt(dx * dx + dy * dy);
                    // 1px anti-aliased band around radius rc.
                    float a = Mathf.Clamp01(halfThk - Mathf.Abs(d - rc) + 0.5f);
                    px[y * N + x] = new Color32(255, 0, 0, (byte)Mathf.RoundToInt(a * 255f));
                }
            }
            tex.SetPixels32(px);
            tex.Apply(updateMipmaps: false);
            _centerRing = tex;
        }

        // Four thin filled rects forming a border around the selected cell.
        private static void DrawCellBorder(Rect cell, Color color)
        {
            const float t = 3f;
            var prev = GUI.color;
            GUI.color = color;
            GUI.DrawTexture(new Rect(cell.x, cell.y, cell.width, t), Texture2D.whiteTexture);
            GUI.DrawTexture(new Rect(cell.x, cell.yMax - t, cell.width, t), Texture2D.whiteTexture);
            GUI.DrawTexture(new Rect(cell.x, cell.y, t, cell.height), Texture2D.whiteTexture);
            GUI.DrawTexture(new Rect(cell.xMax - t, cell.y, t, cell.height), Texture2D.whiteTexture);
            GUI.color = prev;
        }

        private void DrawHud()
        {
            float y = Screen.height - 88f;
            GUI.Box(new Rect(0, y, Screen.width, 88), GUIContent.none);

            string keys;
            if (_assignMode)
            {
                keys = "ASSIGN MODE   arrows select   0-9 set id   " +
                       $"[{setOriginKey}] set origin   Enter save   [{assignModeKey}]/Esc exit";
            }
            else
            {
                string solo = _soloId >= 0 ? $"id {_soloId}" : "all";
                keys =
                    $"[{captureKey}] Capture ({_samples.Count})   " +
                    $"[{solveKey}] Solve   " +
                    $"[{resetKey}] Reset   " +
                    $"[{dumpKey}] Dump   " +
                    $"[{clearSamplesKey}] Clear   " +
                    $"[{assignModeKey}] Assign-ID   " +
                    $"0-9 Solo:{solo}   " +
                    $"[{toggleUiKey}] Hide UI   " +
                    $"[{toggleActiveKey}] Disable";
            }
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
                // Build the solver's camera list in camera-id order, then pin the
                // designated origin camera to index 0 (cam0 = world origin in
                // PairwiseCalibrationMath). This makes the world frame depend on the
                // chosen origin serial, not device-enumeration order.
                var serials = PresentSerialsInOrder();
                if (serials.Count == 0) { SetStatus("Solve aborted: no renderers with serials.", warn: true); return; }
                if (!string.IsNullOrEmpty(_originSerial))
                {
                    int oi = serials.IndexOf(_originSerial);
                    if (oi > 0) { serials.RemoveAt(oi); serials.Insert(0, _originSerial); }
                }

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
                    applyNote = " (manager applyExtrinsics is OFF; not applied to live)";
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

        // -------- Scene suppression (depth visuals + recorder playback) --------

        // Entering calibration mode: stop any recorder playback and disable the
        // recorder components so nothing restarts it. Mesh hiding is continuous
        // (EnforceHiddenPointClouds) because renderers spawn after manager Start().
        private void SuppressScene()
        {
            _sceneRecorders.Clear();
            _sceneRecorders.AddRange(FindObjectsByType<SensorRecorder>(
                FindObjectsInactive.Include, FindObjectsSortMode.None));

            // No live renderers (playbackOnly startup, or a Load freed the
            // cameras) → calibration has no color source at all; reconnect the
            // live rig. SwitchToLive stops playback and clears the loaded tracks
            // itself, then SensorManager.StartLive re-enumerates (~15 s).
            if (switchToLiveOnEnter && Application.isPlaying)
            {
                var mgr = _manager != null ? _manager : FindFirstObjectByType<SensorManager>();
                bool noLive = mgr == null || mgr.Renderers == null || mgr.Renderers.Count == 0;
                if (noLive && _sceneRecorders.Count > 0)
                {
                    SetStatus("No live cameras — switching to LIVE for calibration (~15 s)…");
                    _sceneRecorders[0].SwitchToLive();
                }
            }

            if (!stopRecorderPlayback) return;
            foreach (var rec in _sceneRecorders)
            {
                if (_disabledRecorders.ContainsKey(rec)) continue;
                if (rec.IsPlaying)
                {
                    rec.TogglePlay(); // stop — does NOT auto-resume on exit
                    SetStatus("Recorder playback stopped for calibration (press Play again after exit).");
                }
                _disabledRecorders[rec] = rec.enabled;
                rec.enabled = false;
            }
        }

        // Hide every point-cloud mesh: live renderers plus the recorders'
        // _Playback_* children. Runs every frame while active so late spawns are
        // caught; each mesh's prior state is saved once and restored on exit.
        private void EnforceHiddenPointClouds()
        {
            if (!hidePointClouds) return;

            if (_manager != null && _manager.Renderers != null)
            {
                foreach (var r in _manager.Renderers)
                {
                    if (r == null) continue;
                    if (r.TryGetComponent(out MeshRenderer mr) && mr.enabled)
                    {
                        if (!_hiddenMeshes.ContainsKey(mr)) _hiddenMeshes[mr] = true;
                        mr.enabled = false;
                    }
                }
            }

            foreach (var rec in _sceneRecorders)
            {
                if (rec == null) continue;
                foreach (Transform child in rec.transform)
                {
                    if (!child.name.StartsWith("_Playback_")) continue;
                    if (child.TryGetComponent(out MeshRenderer pmr) && pmr.enabled)
                    {
                        if (!_hiddenMeshes.ContainsKey(pmr)) _hiddenMeshes[pmr] = true;
                        pmr.enabled = false;
                    }
                }
            }
        }

        private void RestoreScene()
        {
            int meshes = 0;
            foreach (var kv in _hiddenMeshes)
                if (kv.Key != null) { kv.Key.enabled = kv.Value; meshes++; }
            _hiddenMeshes.Clear();

            int recs = 0;
            foreach (var kv in _disabledRecorders)
                if (kv.Key != null) { kv.Key.enabled = kv.Value; recs++; }
            _disabledRecorders.Clear();
            _sceneRecorders.Clear();

            if (meshes > 0 || recs > 0)
                Debug.Log($"[CalibrationRuntimeUI] restored {meshes} point-cloud mesh(es), " +
                          $"{recs} recorder component(s). Playback stays stopped — press Play to resume.");
        }

        // -------- External color displays (cams 0/1 and 2/3, stacked vertically) --------

        private void BuildExternalDisplays()
        {
            if (!externalColorDisplays || _externalCanvases.Count > 0) return;
            _externalViews = new ExternalView[4];
            BuildExternalCanvas(displayCam01, firstCamId: 0);
            BuildExternalCanvas(displayCam23, firstCamId: 2);
        }

        // One overlay canvas on the given display with two vertically stacked
        // cells (top = firstCamId, bottom = firstCamId+1) over a black backdrop.
        private void BuildExternalCanvas(int displayIndex, int firstCamId)
        {
            if (displayIndex < 0) return;
#if !UNITY_EDITOR
            // Secondary displays start inactive in builds. Activation is one-way,
            // which is fine — the show's visitor displays use them anyway.
            if (displayIndex < Display.displays.Length && !Display.displays[displayIndex].active)
                Display.displays[displayIndex].Activate();
#endif
            var go = new GameObject($"_CalibColorDisplay{displayIndex}");
            go.transform.SetParent(transform, false);
            var canvas = go.AddComponent<Canvas>();
            canvas.renderMode = RenderMode.ScreenSpaceOverlay;
            canvas.targetDisplay = displayIndex;
            canvas.sortingOrder = 100; // above any visitor UI left on that display
            var scaler = go.AddComponent<CanvasScaler>();
            scaler.uiScaleMode = CanvasScaler.ScaleMode.ScaleWithScreenSize;
            scaler.referenceResolution = new Vector2(1920f, 1080f);
            scaler.matchWidthOrHeight = 0.5f;
            _externalCanvases.Add(go);

            var backdrop = new GameObject("Backdrop", typeof(RectTransform)).AddComponent<Image>();
            backdrop.transform.SetParent(go.transform, false);
            backdrop.color = Color.black;
            backdrop.raycastTarget = false;
            Stretch((RectTransform)backdrop.transform, Vector2.zero, Vector2.one);

            for (int slot = 0; slot < 2; slot++)
            {
                int camId = firstCamId + slot;
                var cell = new GameObject($"Cam{camId}", typeof(RectTransform));
                cell.transform.SetParent(go.transform, false);
                // Top cell = lower id, bottom cell = higher id.
                Stretch((RectTransform)cell.transform,
                        new Vector2(0f, slot == 0 ? 0.5f : 0f),
                        new Vector2(1f, slot == 0 ? 1f : 0.5f));

                var img = new GameObject("Image", typeof(RectTransform)).AddComponent<RawImage>();
                img.transform.SetParent(cell.transform, false);
                img.raycastTarget = false;
                img.color = Color.black; // black until the first frame arrives
                var fit = img.gameObject.AddComponent<AspectRatioFitter>();
                fit.aspectMode = AspectRatioFitter.AspectMode.FitInParent;
                fit.aspectRatio = 16f / 9f;

                // Center crosshair (spans the fitted image, so it tracks aspect).
                MakeLine(img.transform, "CrossV", new Vector2(0.5f, 0f), new Vector2(0.5f, 1f), new Vector2(2f, 0f));
                MakeLine(img.transform, "CrossH", new Vector2(0f, 0.5f), new Vector2(1f, 0.5f), new Vector2(0f, 2f));

                var label = new GameObject("Label", typeof(RectTransform)).AddComponent<Text>();
                label.transform.SetParent(cell.transform, false);
                label.font = Resources.GetBuiltinResource<Font>("LegacyRuntime.ttf");
                label.fontSize = 28;
                label.fontStyle = FontStyle.Bold;
                label.color = Color.white;
                label.raycastTarget = false;
                var lr = (RectTransform)label.transform;
                lr.anchorMin = lr.anchorMax = lr.pivot = new Vector2(0f, 1f);
                lr.anchoredPosition = new Vector2(12f, -8f);
                lr.sizeDelta = new Vector2(1400f, 36f);

                _externalViews[camId] = new ExternalView { Raw = img, Fitter = fit, Label = label };
            }
        }

        private static void MakeLine(Transform parent, string name, Vector2 anchorMin, Vector2 anchorMax, Vector2 size)
        {
            var line = new GameObject(name, typeof(RectTransform)).AddComponent<Image>();
            line.transform.SetParent(parent, false);
            line.color = new Color(1f, 0f, 0f, 0.8f);
            line.raycastTarget = false;
            var rt = (RectTransform)line.transform;
            rt.anchorMin = anchorMin;
            rt.anchorMax = anchorMax;
            rt.sizeDelta = size;
            rt.anchoredPosition = Vector2.zero;
        }

        private static void Stretch(RectTransform rt, Vector2 anchorMin, Vector2 anchorMax)
        {
            rt.anchorMin = anchorMin;
            rt.anchorMax = anchorMax;
            rt.offsetMin = rt.offsetMax = Vector2.zero;
        }

        // Raw feed at frame rate (independent of the round-robin detection pass —
        // the wall displays are for aiming, so they must be smooth). Labels carry
        // the id/origin/detection info instead of baking the overlay into pixels.
        //
        // Slots map to the STABLE cameras.yaml order (_camOrder keeps disconnected
        // serials), NOT the present-compacted ids the HUD grid shows — while the
        // operator re-plugs cables, a disconnect must not shift the remaining
        // cameras onto different displays.
        private void UpdateExternalDisplays()
        {
            if (_externalViews == null) return;
            for (int id = 0; id < _externalViews.Length; id++)
            {
                var v = _externalViews[id];
                if (v == null || v.Raw == null) continue;
                string serial = id < _camOrder.Count ? _camOrder[id] : null;
                var r = serial != null ? FindRenderer(serial) : null;
                if (r == null)
                {
                    v.Raw.texture = null;
                    v.Raw.color = Color.black;
                    v.LastTs = 0; // force a re-upload when the camera comes back
                    v.Label.text = serial == null
                        ? $"ID {id}   no camera"
                        : $"[ID {id}] {serial}   (disconnected)";
                    continue;
                }

                if (_latest.TryGetValue(r, out var f) && f.HasFrame && f.TimestampUs != v.LastTs)
                {
                    v.Tex = UploadFlippedTo(v.Tex, f.Rgb8, f.Width, f.Height);
                    v.LastTs = f.TimestampUs;
                    v.Raw.texture = v.Tex;
                    v.Raw.color = Color.white;
                    v.Fitter.aspectRatio = f.Width / (float)f.Height;
                }

                _previews.TryGetValue(r, out var p);
                string det;
                if (p == null || !p.HasDetectionPass)
                    det = boardSpec == null ? "" : "...";
                else
                    det = (p.Detected ? "● " : "✕ ") + $"M={p.Markers} C={p.Corners}";
                string idTag = $"ID {id}";
                if (serial == _originSerial) idTag += " ★origin";
                v.Label.text = $"[{idTag}] {serial}   {det}";
            }
        }

        private void DestroyExternalDisplays()
        {
            if (_externalViews != null)
                foreach (var v in _externalViews)
                    if (v?.Tex != null) Destroy(v.Tex);
            _externalViews = null;
            foreach (var go in _externalCanvases)
                if (go != null) Destroy(go);
            _externalCanvases.Clear();
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
