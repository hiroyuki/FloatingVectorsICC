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
// Display 1; Displays 2 and 3 get full-rate raw color feeds, cams id0/id1 and
// id2/id3 stacked vertically. All of it is restored on exit (playback itself
// stays stopped — press Play again).
//
// Display numbers are 1-origin throughout this file (project convention,
// matching the Unity Inspector and the OS). Unity's targetDisplay API is
// 0-based, so Display N == targetDisplay N-1; the displayCam* fields below hold
// raw API values and their tooltips spell out both.

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
        [Tooltip("Capture the floor-anchor sample: the board lying FLAT on the physical " +
                 "floor. At Solve the board plane becomes world y=0 with +Y up — without " +
                 "it the world inherits the origin camera's tilt and the rebase/floor " +
                 "grid/sensing area are all skewed.")]
        public KeyCode floorSampleKey = KeyCode.F;
        public KeyCode toggleUiKey = KeyCode.H;
        [Tooltip("Re-fit the sensing box to the current camera rig and save it to " +
                 "calibration/sensing_area.yaml. Solve does this too; this key is for " +
                 "when the cameras were nudged and a full re-solve is not warranted.")]
        public KeyCode rebuildSensingAreaKey = KeyCode.B;
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
                 "on Display 1.")]
        public bool externalColorDisplays = true;
        [Tooltip("Raw Unity targetDisplay index (0-based) for cams 0+1. Default 1 = Display 2.")]
        public int displayCam01 = 1;
        [Tooltip("Raw Unity targetDisplay index (0-based) for cams 2+3. Default 2 = Display 3.")]
        public int displayCam23 = 2;

        [Header("Calibration color resolution")]
        [Tooltip("Restart the live rig with a higher color resolution while calibration " +
                 "mode is active — AprilTag decode range scales ~linearly with pixels on " +
                 "the marker, so 1920x1080 roughly reaches 1.5x the distance of 1280x960. " +
                 "The show resolution is restored on exit. Each switch restarts the " +
                 "camera pipelines (~15 s).")]
        public bool boostColorResolution = true;
        public uint calibColorWidth = 1920;
        public uint calibColorHeight = 1080;

        [Header("Floor anchor")]
        [Tooltip("How Solve levels the world. BoardSample: the [floorSampleKey] shot of the " +
                 "board flat on the floor defines y=0 (accurate but needs a steep view of the " +
                 "board — a grazing-angle shot tilts the whole world). CameraPlaneAssumeLevel: " +
                 "assume the 4 cameras are mounted level and coplanar, make their best-fit " +
                 "plane horizontal at cameraHeightMeters (well-conditioned across the 4.6 m " +
                 "span; level the physical rig instead of the software).")]
        public FloorAnchorMode floorAnchor = FloorAnchorMode.CameraPlaneAssumeLevel;
        [Tooltip("CameraPlaneAssumeLevel only: physical height of the camera plane above the floor (m).")]
        public float cameraHeightMeters = 1.0f;

        [Tooltip("Floor-tune mode: puts a top-down camera on Display 1 showing the live point " +
                 "clouds clipped to the sensing box. Press Enter to fit the floor from the " +
                 "whole cloud and level it horizontal at y=0 — that fixes tilt and height " +
                 "together, and is the lever to reach for. The arrow keys only trim height " +
                 "afterwards. NOBODY may stand in the sensing area during the fit.")]
        public KeyCode floorTuneKey = KeyCode.G;
        [Tooltip("Floor-tune step per arrow press (m). Hold Shift for a tenth of it. " +
                 "This is the raise-the-y=0-plane lever, so its sign is inverted from " +
                 "what you see: Up moves the FLOOR DOWN by one step, Down moves it up. " +
                 "Nudges are saved to floor.yaml on Enter, not on every press.")]
        public float floorTuneStep = 0.01f;
        [Tooltip("Floor fit: half-height of the band around the located floor that is " +
                 "fed to the plane fit (m). Wide enough to cover the depth noise and a " +
                 "real slope, tight enough to exclude furniture.")]
        public float floorFitBandMeters = 0.12f;
        [Tooltip("Floor fit: how far inside the sensing box the sampled floor patch " +
                 "stops (m). The box edges collect skirting, stand feet and the " +
                 "sparsest, noisiest floor returns — and being furthest from the " +
                 "centroid they have the most leverage on the fitted tilt.")]
        public float floorFitInsetMeters = 0.4f;
        [Tooltip("Floor-tune view: how far down the temporary Display 1 camera looks " +
                 "(degrees below horizontal), so the operator can see the floor as a " +
                 "surface while judging the fit. 90 = straight down.")]
        [Range(15f, 89f)] public float floorTunePitchDeg = 55f;
        [Tooltip("Floor fit: refuse to apply a fit whose normal is more than " +
                 "this far from vertical (degrees) — that plane is a wall, not a floor.")]
        [Range(5f, 80f)] public float floorMaxTiltDeg = 40f;

        public enum FloorAnchorMode { BoardSample, CameraPlaneAssumeLevel }

        [Header("After solve")]
        [Tooltip("After a Solve that applied to the live rig: fit the sensing area " +
                 "(ExperienceSpaceBuilder.Apply → BoundingVolume reshaped, floor grid " +
                 "attached) and turn on every camera's frustum marker, so Display 1 " +
                 "immediately shows where the cameras sit in the solved world.")]
        public bool applySensingAidsOnSolve = true;
        [Tooltip("Sensing-area builder to Apply after Solve. Empty → found in scene.")]
        public Experience.ExperienceSpaceBuilder spaceBuilder;

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
            public DetectionOverlay Overlay;
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
            // Latest detection geometry (image px) for the external display overlay.
            public float[][] MarkerQuads;
            public float[] CharucoCornerPoints;
            public int DetWidth;
            public int DetHeight;
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
                // Before SuppressScene: its SwitchToLive path spawns renderers from
                // the manager's current color fields, which Boost just overrode.
                BoostColorResolutionForCalibration();
                SuppressScene();
                BuildExternalDisplays();
            }
        }

        private void OnDisable()
        {
            // Fields only, no camera restart: OnDisable also fires on teardown /
            // playmode exit, where a ~15 s device re-open would just block. The
            // next StartLive picks up the restored show resolution.
            RestoreColorResolution(restartLive: false);
            AbandonFloorTune();
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

            // Floor tune is independent of calibration mode: it needs the live point
            // clouds and the arrow keys, none of what calibration sets up (marker
            // detection, boosted color, suspended BT/TSDF, stopped playback). Runs
            // whether or not the UI is active — that is the whole point of the
            // "calibrate, restart the app, then tune the floor" flow.
            if (_manager == null) _manager = FindFirstObjectByType<SensorManager>();
            if (Input.GetKeyDown(floorTuneKey)) ToggleFloorTune();
            if (_floorTune) { HandleFloorTuneInput(); FlushFloorNudgeSave(); if (!_active) return; }

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
                BoostColorResolutionForCalibration();
                SuppressScene();
                BuildExternalDisplays();
                SetStatus("Calibration UI enabled.");
            }
            else
            {
                _assignMode = false;
                _soloId = -1;
                AbandonFloorTune();
                Unsubscribe();
                _estimator?.Dispose();
                _estimator = null;
                foreach (var p in _previews.Values)
                    if (p.Tex != null) Destroy(p.Tex);
                _previews.Clear();
                DestroyExternalDisplays();
                RestoreScene();
                RestoreInterfering();
                RestoreColorResolution(restartLive: true);
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
            if (Input.GetKeyDown(rebuildSensingAreaKey))
            {
                SetStatus("Sensing area" + ApplySensingAids());
                return;
            }

            if (_floorTune) return; // Update already handled the nudge keys

            if (Input.GetKeyDown(captureKey)) DoCapture();
            if (Input.GetKeyDown(floorSampleKey)) DoCaptureFloor();
            if (Input.GetKeyDown(solveKey)) DoSolve();
            if (Input.GetKeyDown(resetKey)) DoReset();
            if (Input.GetKeyDown(dumpKey)) DoDumpFrames();
            if (Input.GetKeyDown(clearSamplesKey))
            {
                _samples.Clear();
                _hasFloorSample = false;
                SetStatus("Samples + floor sample cleared.");
            }
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
                    p.MarkerQuads = det.MarkerQuads;
                    p.CharucoCornerPoints = det.CharucoCornerPoints;
                    p.DetWidth = f.Width;
                    p.DetHeight = f.Height;
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
            if ((!_active && !_floorTune) || !showUI) return;
            EnsureStyles();

            // Floor tune runs without calibration mode, so it draws its own readout
            // and must not fall into the "waiting for renderers" early-out below.
            if (_floorTune) { DrawHud(); return; }

            if (_manager == null || _manager.Renderers.Count == 0)
            {
                // Bottom-left: the top-left corner belongs to MultiCameraDebugView's
                // header, and the bottom HUD box only exists once renderers are up.
                GUI.Label(new Rect(10, Screen.height - 34, 800, 26),
                    "Calibration: waiting for SensorManager / renderers...", _hud);
                return;
            }

            // Floor tune is a 3D judgement — the color grid would cover the very
            // point cloud being lined up, so only the HUD line is drawn.
            if (!_floorTune) DrawPreviewGrid();
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
                keys = _floorTune
                    ? $"FLOOR TUNE  ↑ floor down / ↓ up ({floorTuneStep * 100f:0.#} cm, Shift 1/10)   " +
                      "Enter=fit floor (area must be EMPTY)   " +
                      $"y {(_manager != null ? _manager.rebaseFloorY : 0f):0.###}   " +
                      $"[{floorTuneKey}]/Esc exit"
                    :
                    $"[{captureKey}] Capture ({_samples.Count})   " +
                    $"[{floorSampleKey}] Floor ({(_hasFloorSample ? "set" : "NONE")})   " +
                    $"[{floorTuneKey}] Floor-tune   " +
                    $"[{solveKey}] Solve   " +
                    $"[{resetKey}] Reset   " +
                    $"[{dumpKey}] Dump   " +
                    $"[{clearSamplesKey}] Clear   " +
                    $"[{assignModeKey}] Assign-ID   " +
                    $"[{rebuildSensingAreaKey}] Sensing-area   " +
                    $"0-9 Solo:{solo}   " +
                    $"[{toggleUiKey}] Hide UI   " +
                    $"[{toggleActiveKey}] Disable";
            }
            GUI.Label(new Rect(10, y + 6, Screen.width - 20, 28), keys, _hud);

            int accepted = 0, detectedTotal = 0;
            foreach (var s in _samples) if (s.SkewOk) { accepted++; }
            GUI.Label(new Rect(10, y + 34, Screen.width - 20, 24),
                $"samples: {_samples.Count} (skew-ok {accepted})   floor: {(_hasFloorSample ? "set" : "NONE — press " + floorSampleKey)}   " +
                $"board: {(boardSpec != null ? boardSpec.name : "<none>")}",
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

        // -------- Floor tune (visual y=0 nudge) --------

        private bool _floorTune;
        private BoundingVolume.FilterMode _savedFilterMode;
        private bool _savedFilterModeValid;
        private PointCloudView _tunedView;
        private bool _savedShowPointClouds;

        // Display 1 (the operator screen) borrowed for the pick — see BorrowOperatorDisplay.
        private GameObject _floorTuneCamGo;
        private readonly List<Canvas> _hiddenPickCanvases = new List<Canvas>();

        // Arrow-key height nudges coalesce into one floor.yaml write this long
        // after the last press (0 = nothing pending).
        private const float kFloorNudgeSaveDelay = 0.75f;
        private float _floorSaveDueAt;

        // Shows the live clouds clipped to the sensing box so the operator can park
        // y=0 exactly where the floor points disappear. The lever is
        // SensorManager.rebaseFloorY (the calibration-frame height that becomes the
        // new y=0), re-applied through ApplyExtrinsicsToLive — cheap, and it writes
        // the same path the show reads.
        private void ToggleFloorTune()
        {
            _floorTune = !_floorTune;
            var box = _manager != null ? _manager.defaultBoundingBox : null;
            if (_floorTune)
            {
                // Two separate hiders must both stand down. EnforceHiddenPointClouds
                // already disabled these meshes, so skipping it from now on is not
                // enough — switch them back on here. And PointCloudView re-asserts
                // showPointClouds over every registered MeshRenderer each frame, so
                // without this the clouds flick straight back off.
                RestoreHiddenMeshesForTuneExit();
                // The wall canvases are full-screen black backdrops on the very
                // displays the scene cameras render to — leaving them up hides the
                // point cloud completely, which is the only thing floor tune shows.
                SetExternalCanvasesVisible(false);
                // Bring the cloud to the operator's own screen so the click and the
                // projection share a coordinate space, and clear that screen.
                BorrowOperatorDisplay();
                _tunedView = _manager != null ? _manager.view : null;
                if (_tunedView != null)
                {
                    _savedShowPointClouds = _tunedView.showPointClouds;
                    _tunedView.showPointClouds = true;
                }
                if (box != null)
                {
                    _savedFilterMode = box.filterMode;
                    _savedFilterModeValid = true;
                    box.filterMode = BoundingVolume.FilterMode.KeepInside;
                }
                // Deliberately does NOT touch the color resolution: switching it
                // restarts the pipelines, and the point clouds do not resume
                // updating afterwards (they freeze on the last mesh — looks like
                // leftover buffer garbage). Floor tuning is a separate session:
                // finish marker calibration, restart the app, then tune.
                if (_manager != null && !_manager.applyWorldRebase)
                    SetStatus("Floor tune: SensorManager.applyWorldRebase is OFF — rebaseFloorY has no effect.", warn: true);
                else
                    SetStatus("Floor tune ON: clear the area of people, then Enter = fit the floor " +
                              "from the whole cloud & level it (fixes tilt AND height — do this " +
                              "first). " +
                              $"↑/↓ then trims height only, {floorTuneStep * 100f:0.#} cm a press " +
                              "(Shift 1/10) — ↑ lowers the floor. " +
                              $"[{floorTuneKey}]/Esc exit.");
            }
            else
            {
                if (box != null && _savedFilterModeValid) box.filterMode = _savedFilterMode;
                _savedFilterModeValid = false;
                if (_tunedView != null) { _tunedView.showPointClouds = _savedShowPointClouds; _tunedView = null; }
                ReturnOperatorDisplay();
                SetExternalCanvasesVisible(true);
                RestoreHiddenMeshesForTuneExit();
                FlushFloorNudgeSaveNow();
                SetStatus($"Floor tune OFF. rebaseFloorY  {(_manager != null ? _manager.rebaseFloorY : 0f):0.####}" +
                          "  (saved in calibration/floor.yaml — reloaded on next start)");
            }
        }

        // Put every mesh back to its pre-calibration state and forget the record:
        // on tune ENTER that reveals the clouds, on EXIT it hands them back to
        // EnforceHiddenPointClouds with a clean slate (it only saves a mesh's prior
        // state the first time it hides it).
        private void RestoreHiddenMeshesForTuneExit()
        {
            foreach (var kv in _hiddenMeshes)
                if (kv.Key != null) kv.Key.enabled = kv.Value;
            _hiddenMeshes.Clear();
        }

        // Give back what floor tune borrowed, without ToggleFloorTune's side effects
        // (re-boosting the color resolution would restart the cameras during a
        // teardown / mode exit).
        private void AbandonFloorTune()
        {
            if (!_floorTune) return;
            _floorTune = false;
            var box = _manager != null ? _manager.defaultBoundingBox : null;
            if (box != null && _savedFilterModeValid) box.filterMode = _savedFilterMode;
            _savedFilterModeValid = false;
            if (_tunedView != null) { _tunedView.showPointClouds = _savedShowPointClouds; _tunedView = null; }
            ReturnOperatorDisplay();
            SetExternalCanvasesVisible(true);
            FlushFloorNudgeSaveNow();
        }

        private void SetExternalCanvasesVisible(bool visible)
        {
            foreach (var go in _externalCanvases)
                if (go != null) go.SetActive(visible);
        }

        // Floor picking turns a 2D mouse position into a world point by projecting
        // the cloud through a camera, so two things must hold at once:
        //
        //  1. The view the operator clicks in and the camera used to project must
        //     be the SAME camera. Input.mousePosition only ever speaks one
        //     display's coordinates (the primary, Display 1), while the stage
        //     camera renders to Display 2.
        //  2. That camera must LOOK DOWN at the floor. TryPick keeps the frontmost
        //     vertex under the cursor; from the stage camera's horizontal eye-level
        //     view the floor is edge-on and every ray hits a wall or the visitor
        //     first, so the "floor" picks come off a vertical surface and the fit
        //     levels the world to a wall (observed: an 82-degree roll).
        //
        // Both are satisfied by standing up a throwaway camera on Display 1 —
        // the operator's own screen, where the HUD already is — framed looking
        // down at the sensing box. Deliberately a NEW camera rather than
        // re-aiming the stage one: Main Camera carries a CameraOrbitController
        // whose idle auto-orbit rewrites the pose every frame, so a borrowed
        // camera would drift out from under the operator mid-pick. Leaving the
        // stage camera alone also means Display 2 and the visitor canvases
        // riding it never need touching.
        //
        // Display numbers below are 1-origin (project convention); Unity's
        // targetDisplay API is 0-based, hence kOperatorTargetDisplay.
        private const int kOperatorTargetDisplay = 0;   // Display 1

        private void BorrowOperatorDisplay()
        {
            Shared.OperatorOverlayGate.FloorTuneActive = true;

            var src = Camera.main;
            var go = new GameObject("_FloorTuneCamera");
            _floorTuneCamGo = go;
            var cam = go.AddComponent<Camera>();
            if (src != null) cam.CopyFrom(src);          // clear flags, mask, clip planes, URP data
            cam.targetDisplay = kOperatorTargetDisplay;
            cam.depth = 100f;                            // above the Display 1 black backdrop
            AimFloorTuneCamera(cam, src);

            // Any screen-space canvas already on Display 1 would cover the cloud.
            var canvases = FindObjectsByType<Canvas>(FindObjectsInactive.Exclude, FindObjectsSortMode.None);
            for (int i = 0; i < canvases.Length; i++)
            {
                var c = canvases[i];
                if (c == null || !c.enabled) continue;
                bool onOperatorDisplay = c.renderMode == RenderMode.ScreenSpaceCamera
                    ? c.worldCamera != null && c.worldCamera.targetDisplay == kOperatorTargetDisplay
                    : c.renderMode == RenderMode.ScreenSpaceOverlay && c.targetDisplay == kOperatorTargetDisplay;
                if (!onOperatorDisplay) continue;
                c.enabled = false;
                _hiddenPickCanvases.Add(c);
            }
        }

        // Look down at the floor of the sensing box from floorTunePitchDeg, far
        // enough back that the box's footprint fits the frame. Yaw is inherited
        // from the stage camera so left/right still means what the operator just
        // saw. Falls back to a fixed 4 m framing of the world origin when there is
        // no bounding box to aim at.
        private void AimFloorTuneCamera(Camera cam, Camera src)
        {
            var box = _manager != null ? _manager.defaultBoundingBox : null;
            Vector3 target = Vector3.zero;
            float radius = 2f;
            if (box != null)
            {
                var s = box.transform.lossyScale;
                target = new Vector3(box.transform.position.x, 0f, box.transform.position.z);
                radius = new Vector2(s.x, s.z).magnitude * 0.5f;
            }
            float yaw = src != null ? src.transform.eulerAngles.y : 0f;
            float pitch = Mathf.Clamp(floorTunePitchDeg, 15f, 89f);
            // Distance that fits `radius` in the narrower of the two half-FOVs,
            // with margin so the floor is not jammed against the frame edge.
            float halfV = cam.fieldOfView * 0.5f * Mathf.Deg2Rad;
            float halfH = Mathf.Atan(Mathf.Tan(halfV) * Mathf.Max(0.1f, cam.aspect));
            float dist = radius / Mathf.Tan(Mathf.Max(0.05f, Mathf.Min(halfV, halfH))) * 1.25f;
            var rot = Quaternion.Euler(pitch, yaw, 0f);
            cam.transform.SetPositionAndRotation(target - rot * Vector3.forward * dist, rot);
        }

        // Hand Display 1 back exactly as it was. Safe to call when nothing was
        // borrowed — both tune exits and the teardown path funnel through here.
        private void ReturnOperatorDisplay()
        {
            Shared.OperatorOverlayGate.FloorTuneActive = false;

            if (_floorTuneCamGo != null) Destroy(_floorTuneCamGo);
            _floorTuneCamGo = null;

            foreach (var c in _hiddenPickCanvases)
                if (c != null) c.enabled = true;
            _hiddenPickCanvases.Clear();
        }

        private void HandleFloorTuneInput()
        {
            if (Input.GetKeyDown(KeyCode.Escape)) { ToggleFloorTune(); return; }
            if (_manager == null) return;

            // Enter = fit the floor from the whole cloud and level to it, then save.
            if (Input.GetKeyDown(KeyCode.Return) || Input.GetKeyDown(KeyCode.KeypadEnter))
            { ApplyFloorLeveling(); return; }
            // No "reset tilt" key: the levelling Pose carries the floor HEIGHT as
            // well as its tilt, so resetting it to identity drops the floor by
            // whatever height it was holding (1.6 m, in the session that motivated
            // removing it). Enter always computes an absolute fit, so there is
            // never a reason to pass through an identity state to get a good floor.

            float step = floorTuneStep *
                         ((Input.GetKey(KeyCode.LeftShift) || Input.GetKey(KeyCode.RightShift)) ? 0.1f : 1f);
            float delta = 0f;
            if (Input.GetKeyDown(KeyCode.UpArrow)) delta = step;
            else if (Input.GetKeyDown(KeyCode.DownArrow)) delta = -step;
            if (delta == 0f) return;
            _manager.rebaseFloorY += delta;
            _manager.ApplyExtrinsicsToLive();
            // Coalesce the write instead of saving per press: a held arrow key
            // walked rebaseFloorY 1.4 m in one session and persisted every 1 cm of
            // it. The nudge still sticks — it is just written once the operator
            // stops moving.
            _floorSaveDueAt = Time.unscaledTime + kFloorNudgeSaveDelay;
            SetStatus($"rebaseFloorY  {_manager.rebaseFloorY:0.####}");
        }

        // Debounced save for the arrow-key nudges (see HandleFloorTuneInput).
        private void FlushFloorNudgeSave()
        {
            if (_floorSaveDueAt <= 0f || Time.unscaledTime < _floorSaveDueAt) return;
            FlushFloorNudgeSaveNow();
        }

        // Write a pending nudge immediately — on tune exit / teardown, so leaving
        // within the debounce window never loses the height the operator dialled in.
        private void FlushFloorNudgeSaveNow()
        {
            if (_floorSaveDueAt <= 0f || _manager == null) { _floorSaveDueAt = 0f; return; }
            _floorSaveDueAt = 0f;
            SaveFloor($"rebaseFloorY  {_manager.rebaseFloorY:0.####}");
        }

        // Persist both the floor height and the levelling Pose together — either
        // lever writes the whole floor.yaml, so nudging Y never drops the tilt and
        // levelling never drops Y. The Inspector values die with Play mode, so this
        // file is the whole point of the tune session.
        private void SaveFloor(string okPrefix)
        {
            var lv = _manager.rebaseFloorLeveling;
            try
            {
                PointCloudRecording.WriteFloor(
                    _manager.ResolveExtrinsicsRoot(), _manager.rebaseFloorY,
                    new[] { lv.position.x, lv.position.y, lv.position.z },
                    new[] { lv.rotation.x, lv.rotation.y, lv.rotation.z, lv.rotation.w });
                SetStatus($"{okPrefix}   (saved to floor.yaml)");
            }
            catch (Exception e)
            {
                SetStatus($"{okPrefix}   ⚠ save failed: {e.Message}", warn: true);
            }
        }

        // Fit the floor from every visible floor point and level the world to it.
        // Runs twice: the first pass pulls the floor to horizontal at y=0 from
        // wherever it was, the second re-fits in the corrected world so the band
        // now brackets the real floor and the residual tilt is squeezed out.
        // Assumes an empty sensing area — see FloorCloudFitter's header.
        private void ApplyFloorLeveling()
        {
            if (_manager == null) return;
            if (!_manager.applyWorldRebase)
            { SetStatus("Floor level: SensorManager.applyWorldRebase is OFF — levelling has no effect.", warn: true); return; }

            var box = _manager.defaultBoundingBox;
            FloorCloudFitter.Fit fit = default;
            bool any = false;
            for (int pass = 0; pass < 2; pass++)
            {
                if (!FloorCloudFitter.TryFit(box, floorFitBandMeters, floorFitInsetMeters, out fit))
                {
                    if (any) break;  // pass 1 worked, pass 2 found nothing — keep it
                    SetStatus("Floor fit: not enough floor points in the sensing area — " +
                              "is the rig live and the area clear?", warn: true);
                    return;
                }
                // A real floor is near horizontal. A steep fit means the samples are
                // a wall (or a person filling the area), and applying it would stand
                // the world on its side — refuse rather than destroy a good calibration.
                if (fit.TiltDeg > floorMaxTiltDeg)
                {
                    SetStatus($"Floor fit: plane is {fit.TiltDeg:0.0}° off horizontal (limit " +
                              $"{floorMaxTiltDeg:0}°) — that is a wall, not the floor. Not applied.", warn: true);
                    return;
                }
                // The fit is measured in the CURRENT (already-levelled) world, so the
                // delta refines toward horizontal; compose it onto the stored total.
                Pose delta = FloorPlaneMath.ComputeLeveling(fit.Point, fit.Normal);
                _manager.rebaseFloorLeveling =
                    FloorPlaneMath.ComposeWorld(delta, _manager.rebaseFloorLeveling);
                _manager.ApplyExtrinsicsToLive();
                any = true;
            }

            _floorSaveDueAt = 0f;   // this save supersedes any pending nudge write
            SaveFloor($"Floor fitted: {fit.SampleCount} pts, residual σ {fit.SdY * 1000f:0.0} mm " +
                      $"(corrected {fit.TiltDeg:0.00}° tilt)");
        }

        private CaptureSample _floorSample;
        private bool _hasFloorSample;

        // Floor anchor: one shot of the board lying FLAT on the physical floor.
        // Unlike DoCapture it needs no skew gate and no full-rig coverage — the
        // board is static, one camera's view fixes the plane.
        private void DoCaptureFloor()
        {
            try
            {
                if (boardSpec == null) { SetStatus("Floor sample aborted: no board spec assigned.", warn: true); return; }
                EnsureEstimator();
                if (_manager == null || _manager.Renderers.Count == 0)
                {
                    SetStatus("Floor sample aborted: no renderers.", warn: true); return;
                }

                var sample = new CaptureSample
                {
                    CapturedAtUtc = DateTime.UtcNow,
                    SkewMs = 0,
                    SkewOk = true,
                    Cameras = new List<CameraResult>(),
                };
                int detected = 0;
                foreach (var r in _manager.Renderers)
                {
                    if (r == null || !r.CameraParam.HasValue) continue;
                    if (!_latest.TryGetValue(r, out var f) || !f.HasFrame) continue;
                    var p = r.CameraParam.Value;
                    var distArr = new double[] { p.RgbDistortion.K1, p.RgbDistortion.K2, p.RgbDistortion.P1, p.RgbDistortion.P2, p.RgbDistortion.K3 };
                    var res = _estimator.Estimate(
                        f.Rgb8, f.Width, f.Height,
                        p.RgbIntrinsic.Fx, p.RgbIntrinsic.Fy, p.RgbIntrinsic.Cx, p.RgbIntrinsic.Cy, distArr);
                    if (!res.Success) continue;
                    detected++;
                    sample.Cameras.Add(new CameraResult
                    {
                        Serial = r.deviceSerial, Detected = true,
                        Markers = res.DetectedMarkerCount, Corners = res.InterpolatedCornerCount,
                        CamTrMarker = new Rigid3d(res.Rotation, res.Translation),
                    });
                }
                if (detected == 0)
                {
                    SetStatus("Floor sample: board not detected by any camera — lay it flat on the floor in view.", warn: true);
                    return;
                }
                _floorSample = sample;
                _hasFloorSample = true;
                SetStatus($"Floor sample set ({detected} camera(s) see the board). Solve will anchor the world to it.");
            }
            catch (Exception e)
            {
                SetStatus($"Floor sample failed: {e.Message}", warn: true);
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

                // Floor anchor: re-express the cam0-anchored solution in a
                // gravity-aligned frame. Without it the world inherits the origin
                // camera's tilt and the rebase gate / floor grid are skewed.
                string floorNote;
                bool gravityAligned = false;
                if (floorAnchor == FloorAnchorMode.CameraPlaneAssumeLevel)
                {
                    string planeErr = serials.Count != 4 ? $"need 4 cameras, have {serials.Count}" : null;
                    if (planeErr == null && TryAnchorToCameraPlane(ref solve, serials.Count, out planeErr))
                    {
                        gravityAligned = true;
                        floorNote = $" (camera-plane anchored, cams at {cameraHeightMeters:0.00} m)";
                    }
                    else
                    {
                        floorNote = $" ⚠ camera-plane anchor failed ({planeErr}) — world stays camera-tilted";
                    }
                }
                else if (_hasFloorSample)
                {
                    int fi = -1;
                    Rigid3d camTrMarker = Rigid3d.Identity;
                    foreach (var c in _floorSample.Cameras)
                    {
                        if (!c.Detected || !serialToIdx.TryGetValue(c.Serial, out var idx)) continue;
                        fi = idx; camTrMarker = c.CamTrMarker; break;
                    }
                    if (fi < 0)
                    {
                        floorNote = " ⚠ floor sample's cameras aren't in this solve — world stays camera-tilted";
                    }
                    else
                    {
                        var globalTrMarker = Rigid3d.Compose(solve.GlobalTrCamera[fi], camTrMarker);
                        // Marker→world axis shuffle: board X stays x, board Z (out of
                        // the pattern, i.e. up off the floor) becomes -y (down in the
                        // OpenCV convention this file stores = up in Unity), board Y
                        // becomes z. Determinant +1.
                        var axis = new Rigid3d(
                            new[] { 1.0, 0, 0, 0, 0, -1.0, 0, 1.0, 0 },
                            new[] { 0.0, 0, 0 });
                        var worldTrGlobal = Rigid3d.Compose(axis, globalTrMarker.Inverse());
                        for (int i = 0; i < solve.GlobalTrCamera.Length; i++)
                            solve.GlobalTrCamera[i] = Rigid3d.Compose(worldTrGlobal, solve.GlobalTrCamera[i]);
                        floorNote = " (floor-anchored)";
                        gravityAligned = true;
                    }
                }
                else
                {
                    floorNote = $" ⚠ no floor sample — lay the board flat on the floor and press [{floorSampleKey}] " +
                                "before Solve, or the world stays camera-tilted";
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

                // Auto-derive the rig perimeter order from the solved layout so the
                // rebase never fails on a hand-entered camera-id order. Only
                // meaningful in a gravity-aligned world.
                string orderNote = "";
                if (gravityAligned && serials.Count == 4)
                {
                    var posBySerial = new Dictionary<string, Vector3>();
                    for (int i = 0; i < serials.Count; i++)
                    {
                        ExtrinsicsApply.ToUnityLocal(ToObExtrinsicMm(solve.GlobalTrCamera[i]), out var upos, out _);
                        posBySerial[serials[i]] = upos;
                    }
                    var perim = FindPerimeterOrder(posBySerial, serials[0]);
                    if (perim != null)
                    {
                        ApplyPresentOrder(perim);
                        SaveCameraMap();
                        orderNote = " rig order auto-set: " +
                                    string.Join("→", perim.ConvertAll(s => s.Substring(s.Length - 2)));
                    }
                    else orderNote = " ⚠ no valid rig perimeter order found from solved positions";
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
                    if (sameRoot)
                    {
                        _manager.ApplyExtrinsicsToLive();
                        applyNote = " (applied to live)";
                        // Camera poses are now solved-world; safe to fit the box/grid
                        // and show the frustums.
                        if (applySensingAidsOnSolve) applyNote += ApplySensingAids();
                    }
                    else applyNote = $" (manager root differs: {mgrRoot})";
                }
                SetStatus($"Solved{floorNote}. Wrote {calibrations.Count} entries to {outPath}.{orderNote}{applyNote}");
            }
            catch (Exception e)
            {
                SetStatus($"Solve failed: {e.Message}", warn: true);
                Debug.LogException(e);
            }
        }

        // CameraPlaneAssumeLevel anchor: make the best-fit plane through the four
        // (near-coplanar) camera positions horizontal, with the plane at
        // cameraHeightMeters above the new floor. Works in the solver's OpenCV
        // frame (+y down); the down direction is disambiguated by the cameras'
        // own +y (image-down) axes, which point at the physical floor for any
        // upright-mounted camera. Far better conditioned than a distant board
        // shot (4.6 m baseline vs a 0.7 m board patch at grazing angle).
        private bool TryAnchorToCameraPlane(ref PairwiseCalibrationMath.SolveResult solve, int count, out string err)
        {
            err = null;
            var p = new UnityEngine.Vector3[count];
            for (int i = 0; i < count; i++)
            {
                var t = solve.GlobalTrCamera[i].Translation;
                p[i] = new Vector3((float)t[0], (float)t[1], (float)t[2]);
            }
            Vector3 c = (p[0] + p[1] + p[2] + p[3]) * 0.25f;

            // Plane normal: average of the two triangle normals, sign-aligned.
            Vector3 n1 = Vector3.Cross(p[1] - p[0], p[2] - p[0]);
            Vector3 n2 = Vector3.Cross(p[2] - p[0], p[3] - p[0]);
            if (n1.magnitude < 1e-6f || n2.magnitude < 1e-6f) { err = "degenerate camera layout"; return false; }
            if (Vector3.Dot(n1, n2) < 0f) n2 = -n2;
            Vector3 n = (n1.normalized + n2.normalized);
            if (n.magnitude < 1e-6f) { err = "degenerate camera layout"; return false; }
            n.Normalize();

            // Physical down = average image-down (+y) axis of the cameras
            // (rotation column 1 of global_tr_cam maps camera +y into global).
            Vector3 downAvg = Vector3.zero;
            for (int i = 0; i < count; i++)
            {
                var r = solve.GlobalTrCamera[i].Rotation;
                downAvg += new Vector3((float)r[1], (float)r[4], (float)r[7]);
            }
            if (downAvg.magnitude < 1e-6f) { err = "cannot resolve down direction"; return false; }
            if (Vector3.Dot(n, downAvg) < 0f) n = -n; // n = down in the OpenCV frame

            // Orthonormal basis: y' = n (down), x' = first edge projected onto the
            // plane (yaw is arbitrary here — the rebase re-yaws to camera1→2),
            // z' = x' × y' (right-handed).
            Vector3 xp = (p[1] - p[0]) - Vector3.Dot(p[1] - p[0], n) * n;
            if (xp.magnitude < 1e-6f) { err = "degenerate camera layout"; return false; }
            xp.Normalize();
            Vector3 zp = Vector3.Cross(xp, n);

            // world_tr_global: rows = new axes; centroid maps to (0, -h, 0) so the
            // camera plane sits h metres above the floor (OpenCV y is down; the
            // Unity conversion flips it to +h up).
            var rot = new double[]
            {
                xp.x, xp.y, xp.z,
                n.x,  n.y,  n.z,
                zp.x, zp.y, zp.z,
            };
            var rc = new Vector3(
                (float)(rot[0] * c.x + rot[1] * c.y + rot[2] * c.z),
                (float)(rot[3] * c.x + rot[4] * c.y + rot[5] * c.z),
                (float)(rot[6] * c.x + rot[7] * c.y + rot[8] * c.z));
            var trans = new double[] { -rc.x, -cameraHeightMeters - rc.y, -rc.z };
            var worldTrGlobal = new Rigid3d(rot, trans);
            for (int i = 0; i < solve.GlobalTrCamera.Length; i++)
                solve.GlobalTrCamera[i] = Rigid3d.Compose(worldTrGlobal, solve.GlobalTrCamera[i]);
            return true;
        }

        // Perimeter walk that satisfies the WorldFrameRebase gate: camera1→2 is
        // the X axis, camera2→3 must agree with X̂ × up within its 10° tolerance.
        // Starting camera fixed to the origin serial; tries the 6 orderings of
        // the remaining three and returns the first valid winding.
        private static List<string> FindPerimeterOrder(Dictionary<string, Vector3> pos, string first)
        {
            var rest = new List<string>(pos.Keys);
            rest.Remove(first);
            if (rest.Count != 3) return null;
            for (int a = 0; a < 3; a++)
            {
                for (int b = 0; b < 3; b++)
                {
                    if (b == a) continue;
                    int c = 3 - a - b;
                    var ord = new List<string> { first, rest[a], rest[b], rest[c] };
                    Vector3 x = pos[ord[1]] - pos[ord[0]]; x.y = 0f;
                    Vector3 z = pos[ord[2]] - pos[ord[1]]; z.y = 0f;
                    if (x.magnitude < 0.01f || z.magnitude < 0.01f) continue;
                    var zAxis = Vector3.Cross(x.normalized, Vector3.up);
                    if (Vector3.Dot(zAxis, z.normalized) >= Mathf.Cos(10f * Mathf.Deg2Rad)) return ord;
                }
            }
            return null;
        }

        // Post-solve scene aids on Display 1: sensing-area box + floor grid via
        // ExperienceSpaceBuilder, camera frustum markers via SensorManager (the
        // point clouds stay hidden in calibration mode, so the frustums + grid
        // are what shows the solved rig). Returns a note for the status line.
        private string ApplySensingAids()
        {
            if (_manager != null) _manager.showCameraMarkers = true;
            var builder = spaceBuilder != null
                ? spaceBuilder
                : FindFirstObjectByType<Experience.ExperienceSpaceBuilder>(FindObjectsInactive.Include);
            if (builder == null) builder = _ownedSpaceBuilder != null ? _ownedSpaceBuilder : CreateSpaceBuilderFromDirector();
            if (builder == null)
                return " — no ExperienceSpaceBuilder and no wired ExperienceDirector in scene, sensing area NOT fitted";
            builder.Apply(); // warns on its own when the layout is degenerate
            if (builder.floorOrigin != null)
            {
                builder.floorOrigin.showGrid = true;
                builder.floorOrigin.fitToBoundingBox = true;
            }
            return " + sensing area fitted, floor grid on, camera frustums shown";
        }

        // -------- Calibration-time color resolution boost --------

        private uint _savedColorW, _savedColorH;
        private bool _colorBoosted;

        private void BoostColorResolutionForCalibration()
        {
            if (!boostColorResolution || _colorBoosted) return;
            var mgr = _manager != null ? _manager : FindFirstObjectByType<SensorManager>();
            if (mgr == null) return;
            if (mgr.colorWidth == calibColorWidth && mgr.colorHeight == calibColorHeight) return;
            _savedColorW = mgr.colorWidth;
            _savedColorH = mgr.colorHeight;
            mgr.colorWidth = calibColorWidth;
            mgr.colorHeight = calibColorHeight;
            _colorBoosted = true;
            RestartLiveRenderers(mgr, $"color {calibColorWidth}x{calibColorHeight} for calibration");
        }

        private void RestoreColorResolution(bool restartLive)
        {
            if (!_colorBoosted) return;
            var mgr = _manager != null ? _manager : FindFirstObjectByType<SensorManager>();
            // Clear the flag only once the fields are actually restorable —
            // forgetting the boost with the manager gone would let the next Boost
            // save the CALIBRATION size as the show size and pin it there.
            if (mgr == null) return;
            _colorBoosted = false;
            mgr.colorWidth = _savedColorW;
            mgr.colorHeight = _savedColorH;
            if (restartLive) RestartLiveRenderers(mgr, $"color restored to {_savedColorW}x{_savedColorH}");
        }

        // The stream profile is fixed at pipeline start, so a resolution change
        // means destroy + re-enumerate (~15 s). No-op with zero live renderers —
        // the next StartLive/SwitchToLive spawns from the already-updated fields.
        private void RestartLiveRenderers(SensorManager mgr, string why)
        {
            if (mgr.Renderers == null || mgr.Renderers.Count == 0) return;
            mgr.DestroyAllRenderers();
            mgr.StartLive();
            // Same count in, same count out — Update's count-based re-subscribe
            // check cannot see that every renderer is a NEW object, so the frame
            // handlers would stay bound to the destroyed ones and no preview,
            // external display or capture would ever get a frame again.
            PurgeDeadFrameCaches();
            Subscribe();
            SetStatus($"Restarting live cameras: {why} (~15 s)…");
        }

        // Drop _latest/_previews entries for renderers that are no longer the
        // manager's, so a rig restart doesn't leak textures or keep stale frames.
        // Membership, not a null check: Destroy is deferred to end of frame, so
        // just-destroyed renderers still compare non-null in this same frame.
        private void PurgeDeadFrameCaches()
        {
            var live = new HashSet<PointCloudRenderer>();
            if (_manager != null && _manager.Renderers != null)
                foreach (var r in _manager.Renderers)
                    if (r != null) live.Add(r);

            var deadPrev = new List<PointCloudRenderer>();
            foreach (var kv in _previews)
                if (kv.Key == null || !live.Contains(kv.Key))
                { if (kv.Value.Tex != null) Destroy(kv.Value.Tex); deadPrev.Add(kv.Key); }
            foreach (var k in deadPrev) _previews.Remove(k);

            var deadLatest = new List<PointCloudRenderer>();
            foreach (var kv in _latest)
                if (kv.Key == null || !live.Contains(kv.Key)) deadLatest.Add(kv.Key);
            foreach (var k in deadLatest) _latest.Remove(k);
        }

        private Experience.ExperienceSpaceBuilder _ownedSpaceBuilder;

        // The scene keeps no persistent ExperienceSpaceBuilder — ExperienceDirector
        // AddComponents its own on experience start. Mirror that here, borrowing the
        // director's serialized refs so calibration fits the same box/grid the show
        // will use. The component lives on this GO for the rest of the play session;
        // the fitted volume intentionally survives calibration-mode exit.
        private Experience.ExperienceSpaceBuilder CreateSpaceBuilderFromDirector()
        {
            var dir = FindFirstObjectByType<Experience.ExperienceDirector>(FindObjectsInactive.Include);
            if (dir == null || dir.boundingVolume == null) return null;
            _ownedSpaceBuilder = gameObject.AddComponent<Experience.ExperienceSpaceBuilder>();
            _ownedSpaceBuilder.boundingVolume = dir.boundingVolume;
            _ownedSpaceBuilder.floorOrigin = dir.floorOrigin;
            _ownedSpaceBuilder.sensorManager = dir.sensorManager != null ? dir.sensorManager : _manager;
            _ownedSpaceBuilder.sensorRecorder = dir.sensorRecorder;
            if (dir.config != null) _ownedSpaceBuilder.floorY = dir.config.floorY;
            return _ownedSpaceBuilder;
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
            if (!hidePointClouds || _floorTune) return; // floor tune needs to SEE the clouds

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

                // Detection overlay (marker quads + ChArUco corners) — child of the
                // fitted image so its rect matches the displayed pixels exactly.
                var overlay = new GameObject("DetectOverlay", typeof(RectTransform), typeof(CanvasRenderer))
                    .AddComponent<DetectionOverlay>();
                overlay.transform.SetParent(img.transform, false);
                overlay.raycastTarget = false;
                Stretch((RectTransform)overlay.transform, Vector2.zero, Vector2.one);

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

                _externalViews[camId] = new ExternalView { Raw = img, Fitter = fit, Label = label, Overlay = overlay };
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
            // Sample tallies for the wall labels: global count + skew-ok (same
            // numbers as the HUD status bar) plus per-camera detected count, so
            // the operator sees capture coverage without walking back to Display 1.
            int sampleTotal = _samples.Count, sampleSkewOk = 0;
            foreach (var s in _samples) if (s.SkewOk) sampleSkewOk++;
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
                    if (v.Overlay != null) v.Overlay.SetDetections(null, null, 0, 0);
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
                int camSamples = 0;
                foreach (var s in _samples)
                {
                    if (s.Cameras == null) continue;
                    foreach (var c in s.Cameras)
                        if (c.Detected && c.Serial == serial) { camSamples++; break; }
                }
                v.Label.text = $"[{idTag}] {serial}   {det}   " +
                               $"smp {camSamples}/{sampleTotal} (skew-ok {sampleSkewOk})";

                // Detection geometry rides the same round-robin pass as the HUD
                // previews; showUI off stops that pass, so blank the overlay
                // rather than freezing a stale one over the live feed.
                if (v.Overlay != null)
                {
                    if (p != null && p.HasDetectionPass && showUI)
                        v.Overlay.SetDetections(p.MarkerQuads, p.CharucoCornerPoints, p.DetWidth, p.DetHeight);
                    else
                        v.Overlay.SetDetections(null, null, 0, 0);
                }
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
