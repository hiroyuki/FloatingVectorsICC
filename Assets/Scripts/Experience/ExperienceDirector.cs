// The show's single owner (Phase 5, Plans/phase5-director-plan.md): drives the
// ExperienceStateMachine, owns every runtime object the experience spawns
// (visitor UI, presence detector, spheres, snapshot miniatures), snapshots the
// dev-mode values it touches on Enter and restores them all on Exit — Dev mode
// must come back bit-identical.
//
// Enter order: snapshot -> world rebase on -> sensing area -> spawn UI/presence
// -> FSM starts at Attract. Exit runs the exact reverse (cancel any publish
// task first and OBSERVE it — no fire-and-forget).
//
// Capture protocol per prompt (parent-plan contract): countdown -> set
// historySamples=15 -> wait for PointCloudMotionCurves.BuildVersion to advance
// by 2 (a full rebuild with the new window) with a 2 s timeout (curves may
// legitimately have nothing to build) -> TSDFSnapshotBuilder.Capture (sync,
// non-destructive) -> restore historySamples -> CaptureDone.

using System;
using System.Collections;
using System.IO;
using System.Threading;
using System.Threading.Tasks;
using BodyTracking;
using Experience.Publishing;
using PointCloud;
using TSDF;
using UnityEngine;

namespace Experience
{
    [DisallowMultipleComponent]
    public class ExperienceDirector : MonoBehaviour, Shared.IViewToggle
    {
        [Tooltip("Experience configuration asset. A default instance is created " +
                 "at runtime when left empty (dev convenience).")]
        public ExperienceConfig config;

        [Header("Scene references (auto-resolved when empty)")]
        public SensorManager sensorManager;
        public SensorRecorder sensorRecorder;
        public SkeletonMerger merger;
        public BoundingVolume boundingVolume;
        public FloorOrigin floorOrigin;
        public TSDFVolume tsdfVolume;
        public PointCloudMotionCurves motionCurves;
        public BonePoseHistory poseHistory;
        public TSDFPrintExporter printExporter; // capture/export settings source
        public CameraHealthMonitor healthMonitor; // fault source (auto-resolved)

        [Header("Debug")]
        [Tooltip("Force the Fault state (full-screen red alert) regardless of the " +
                 "health monitor.")]
        public bool debugForceFault;

        // ---- IViewToggle ("Experience mode" in the Views panel) ----
        public string ViewLabel => "Experience mode";
        public bool Visible
        {
            get => _active;
            set { if (value && !_active) EnterMode(); else if (!value && _active) ExitMode(); }
        }

        public ExperienceState CurrentState => _fsm?.State ?? ExperienceState.Attract;
        public bool IsActive => _active;

        private bool _active;
        private ExperienceStateMachine _fsm;

        // spawned per mode session
        private VisitorMessageUI _ui;
        private PresenceDetector _presence;
        private ExperienceSpaceBuilder _space;
        private readonly DwellSphere[] _spheres = new DwellSphere[3];
        private readonly GameObject[] _miniatures = new GameObject[3];
        private Material _miniatureMat;

        // dev-value snapshot (restored on Exit)
        private int _savedHistorySamples;
        private bool _savedCurvesVisible, _savedCurvesFreeze;
        private bool _savedMgrRebase, _savedRecRebase;
        private string[] _savedMgrOrder, _savedRecOrder;
        private bool _savedKeepLive;
        private string _savedPlaybackFolder;
        private bool _savedCumulativeNoErase;
        private PointCloudCumulative _cumulative;
        private readonly System.Collections.Generic.Dictionary<string, bool> _savedLiveVisible =
            new System.Collections.Generic.Dictionary<string, bool>();
        private readonly System.Collections.Generic.Dictionary<string, bool> _savedLiveSuppress =
            new System.Collections.Generic.Dictionary<string, bool>();

        // attract playback (Phase 6)
        private AttractPlaybackController _attract;
        private bool _attractOwnsPlayback;
        private bool _devFallbackLogged;

        // per-run state
        private readonly TSDFSnapshot[] _snapshots = new TSDFSnapshot[3];
        private int _captureIndex;
        private bool _captureDone;
        private Coroutine _captureRoutine;
        private int _selectedIndex = -1;
        private bool _exportDone, _exportFailed;
        private Coroutine _exportRoutine;
        private CancellationTokenSource _cts;
        private Task<PublishResult> _publishTask;
        private string _qrUrl;
        private bool _crowdShowing;

        // ------------------------------------------------ lifecycle ----

        private void OnEnable()
        {
            if (sensorManager == null) sensorManager = FindFirstObjectByType<SensorManager>();
            if (sensorRecorder == null) sensorRecorder = FindFirstObjectByType<SensorRecorder>();
            if (merger == null) merger = FindFirstObjectByType<SkeletonMerger>();
            if (boundingVolume == null) boundingVolume = FindFirstObjectByType<BoundingVolume>();
            if (floorOrigin == null) floorOrigin = FindFirstObjectByType<FloorOrigin>();
            if (tsdfVolume == null) tsdfVolume = FindFirstObjectByType<TSDFVolume>();
            if (motionCurves == null) motionCurves = FindFirstObjectByType<PointCloudMotionCurves>();
            if (poseHistory == null) poseHistory = FindFirstObjectByType<BonePoseHistory>();
            if (printExporter == null) printExporter = FindFirstObjectByType<TSDFPrintExporter>();
            if (healthMonitor == null) healthMonitor = FindFirstObjectByType<CameraHealthMonitor>();
        }

        private bool HasLiveRenderers()
        {
            if (sensorManager == null) return false;
            foreach (var r in sensorManager.Renderers)
                if (r != null) return true;
            return false;
        }

        private void OnDisable()
        {
            if (_active) ExitMode();
        }

        private void Update()
        {
            if (!_active) return;

            var inputs = new ExperienceInputs
            {
                Present = _presence != null && _presence.IsPresent,
                Fault = debugForceFault || (healthMonitor != null && !healthMonitor.IsHealthy),
                CaptureDone = _captureDone,
                SelectedIndex = _selectedIndex,
                ExportDone = _exportDone,
                ExportFailed = _exportFailed,
            };
            _fsm.Tick(Time.deltaTime, in inputs, config.timings);
            UpdateCrowdNotice();
        }

        // ------------------------------------------------ enter / exit ----

        private void EnterMode()
        {
            if (config == null)
            {
                config = ScriptableObject.CreateInstance<ExperienceConfig>();
                Debug.LogWarning($"[{nameof(ExperienceDirector)}] no config assigned — using defaults.", this);
            }

            // 1) snapshot dev values.
            if (poseHistory != null) _savedHistorySamples = poseHistory.historySamples;
            if (motionCurves != null)
            {
                _savedCurvesVisible = motionCurves.visible;
                _savedCurvesFreeze = motionCurves.freeze;
                // Either flag would stall the capture-readiness wait (curves
                // Update early-returns) — force them sane for the session.
                motionCurves.visible = true;
                motionCurves.freeze = false;
            }

            // 2) world rebase on (single reversible hook, Phase 2).
            if (sensorManager != null)
            {
                _savedMgrRebase = sensorManager.applyWorldRebase;
                _savedMgrOrder = sensorManager.rigSerialOrder;
                sensorManager.applyWorldRebase = true;
                sensorManager.rigSerialOrder = config.rigSerialOrder;
                sensorManager.ApplyExtrinsicsToLive();
            }
            if (sensorRecorder != null)
            {
                _savedRecRebase = sensorRecorder.applyWorldRebase;
                _savedRecOrder = sensorRecorder.rigSerialOrder;
                sensorRecorder.applyWorldRebase = true;
                sensorRecorder.rigSerialOrder = config.rigSerialOrder;
                sensorRecorder.ReapplyExtrinsics();
            }

            // 3) sensing area (TSDF volume + floor grid follow automatically).
            _space = gameObject.AddComponent<ExperienceSpaceBuilder>();
            _space.boundingVolume = boundingVolume;
            _space.floorOrigin = floorOrigin;
            _space.sensorManager = sensorManager;
            _space.sensorRecorder = sensorRecorder;
            _space.rigSerialOrder = config.rigSerialOrder;
            _space.floorY = config.floorY;
            _space.Apply();

            // 4) spawned objects.
            _ui = new GameObject("_ExperienceUI").AddComponent<VisitorMessageUI>();
            _ui.transform.SetParent(transform, false);
            _presence = new GameObject("_ExperiencePresence").AddComponent<PresenceDetector>();
            _presence.transform.SetParent(transform, false);
            _presence.merger = merger;
            _presence.sensingVolume = boundingVolume;
            _presence.sensorManager = sensorManager;
            _presence.insetMeters = config.insetMeters;
            _presence.yBandMin = config.yBandMin;
            _presence.yBandMax = config.yBandMax;
            _presence.occupancyThreshold = config.occupancyThreshold;
            SpawnSpheres();

            // 4b) attract coexistence (Phase 6): keep the live rig through
            // playback loads, hide the raw clouds (the TSDF mesh is the star),
            // pause cumulative snapshots, spawn the attract controller.
            if (sensorRecorder != null)
            {
                _savedKeepLive = sensorRecorder.keepLiveRenderersOnLoad;
                _savedPlaybackFolder = sensorRecorder.playbackFolderPath;
                sensorRecorder.keepLiveRenderersOnLoad = true;
            }
            _savedLiveVisible.Clear();
            _savedLiveSuppress.Clear();
            if (sensorManager != null)
            {
                foreach (var r in sensorManager.Renderers)
                {
                    if (r == null) continue;
                    if (r.TryGetComponent(out MeshRenderer mr))
                        _savedLiveVisible[r.deviceSerial] = mr.enabled;
                    _savedLiveSuppress[r.deviceSerial] = r.suppressAsSource;
                }
                sensorManager.SetLiveVisualsVisible(false);
            }
            _cumulative = FindFirstObjectByType<PointCloudCumulative>();
            if (_cumulative != null)
            {
                _savedCumulativeNoErase = _cumulative.noErase;
                _cumulative.noErase = false;
            }
            if (!string.IsNullOrEmpty(config.attractRecordingRoot) && sensorRecorder != null)
            {
                _attract = gameObject.AddComponent<AttractPlaybackController>();
                _attract.sensorRecorder = sensorRecorder;
                _attract.attractRootPath = config.attractRecordingRoot;
            }
            _attractOwnsPlayback = false;
            _devFallbackLogged = false;

            // 5) run.
            ResetRunState();
            _fsm = new ExperienceStateMachine();
            _fsm.Changed += OnStateChanged;
            _fsm.ResetTo(ExperienceState.Attract);
            _active = true;
            ApplyStateSideEffects(ExperienceState.Attract);
            Debug.Log($"[{nameof(ExperienceDirector)}] experience mode ON.", this);
        }

        private void ExitMode()
        {
            _active = false;

            if (_captureRoutine != null) { StopCoroutine(_captureRoutine); _captureRoutine = null; }
            if (_exportRoutine != null) { StopCoroutine(_exportRoutine); _exportRoutine = null; }
            CancelPublish();
            RestoreHistorySamples(); // in case a capture was mid-flight

            if (_fsm != null) { _fsm.Changed -= OnStateChanged; _fsm = null; }

            for (int i = 0; i < 3; i++)
            {
                if (_miniatures[i] != null) { Destroy(_miniatures[i]); _miniatures[i] = null; }
                if (_spheres[i] != null) { Destroy(_spheres[i].gameObject); _spheres[i] = null; }
                _snapshots[i] = null;
            }
            if (_miniatureMat != null) { Destroy(_miniatureMat); _miniatureMat = null; }
            if (_ui != null) { _ui.ClearEverything(); Destroy(_ui.gameObject); _ui = null; }
            if (_presence != null) { Destroy(_presence.gameObject); _presence = null; }

            // attract coexistence teardown (before space/rebase restore)
            if (_attract != null)
            {
                if (_attractOwnsPlayback) _attract.Stop();
                Destroy(_attract);
                _attract = null;
                _attractOwnsPlayback = false;
            }
            if (sensorRecorder != null)
            {
                sensorRecorder.keepLiveRenderersOnLoad = _savedKeepLive;
                sensorRecorder.playbackFolderPath = _savedPlaybackFolder;
            }
            if (sensorManager != null)
            {
                foreach (var r in sensorManager.Renderers)
                {
                    if (r == null) continue;
                    // Per-renderer restore of the saved dev state; renderers born
                    // during the session (no saved entry) fall back to defaults.
                    r.suppressAsSource = _savedLiveSuppress.TryGetValue(r.deviceSerial, out bool sup) && sup;
                    if (r.TryGetComponent(out MeshRenderer mr) &&
                        _savedLiveVisible.TryGetValue(r.deviceSerial, out bool vis))
                        mr.enabled = vis;
                }
            }
            if (_cumulative != null)
            {
                _cumulative.noErase = _savedCumulativeNoErase;
                _cumulative = null;
            }

            if (_space != null) { _space.Restore(); Destroy(_space); _space = null; }

            if (sensorManager != null)
            {
                sensorManager.applyWorldRebase = _savedMgrRebase;
                sensorManager.rigSerialOrder = _savedMgrOrder;
                sensorManager.ApplyExtrinsicsToLive();
            }
            if (sensorRecorder != null)
            {
                sensorRecorder.applyWorldRebase = _savedRecRebase;
                sensorRecorder.rigSerialOrder = _savedRecOrder;
                sensorRecorder.ReapplyExtrinsics();
            }

            if (motionCurves != null)
            {
                motionCurves.visible = _savedCurvesVisible;
                motionCurves.freeze = _savedCurvesFreeze;
            }
            if (poseHistory != null) poseHistory.historySamples = _savedHistorySamples;

            Debug.Log($"[{nameof(ExperienceDirector)}] experience mode OFF (dev values restored).", this);
        }

        private void ResetRunState()
        {
            _captureIndex = 0;
            _captureDone = false;
            _selectedIndex = -1;
            _exportDone = _exportFailed = false;
            _qrUrl = null;
            _crowdShowing = false;
            for (int i = 0; i < 3; i++) _snapshots[i] = null;
        }

        // ------------------------------------------------ state side effects ----

        private void OnStateChanged(ExperienceState from, ExperienceState to)
        {
            // Leaving a prompt mid-capture (visitor walked away / fault).
            if (_captureRoutine != null && !IsPrompt(to))
            {
                StopCoroutine(_captureRoutine);
                _captureRoutine = null;
                RestoreHistorySamples();
            }
            if (from == ExperienceState.Exporting && to == ExperienceState.Attract)
                CancelPublish(); // fail path or skip; success path already completed

            // Attract → visitor handoff: the ghost stops, live becomes the
            // sculpture source, k4abt workers restart (clock jump guard). Only
            // when a live rig exists — the dev/Mac fallback keeps the playback
            // running as the only source (Phase 5 E2E behaviour).
            if (from == ExperienceState.Attract && to == ExperienceState.Welcome)
            {
                if (HasLiveRenderers())
                {
                    _attract?.Stop();
                    _attractOwnsPlayback = false;
                    sensorManager?.SetLiveSuppressedAsSource(false);
                    if (merger != null) merger.RestartWorkers();
                }
                else
                {
                    // Dev/Mac fallback: the ghost stays the only sculpture source
                    // — keep it playing but freeze take rotation until the next
                    // Attract (a mid-experience switch would yank the source).
                    if (_attract != null) _attract.RotationEnabled = false;
                    if (!_devFallbackLogged)
                    {
                        Debug.Log($"[{nameof(ExperienceDirector)}] no live rig — attract playback keeps " +
                                  "running through the experience (dev fallback).", this);
                        _devFallbackLogged = true;
                    }
                }
            }

            if (to == ExperienceState.Attract || from == ExperienceState.QrShow)
                ResetRunState();

            ApplyStateSideEffects(to);
        }

        private static bool IsPrompt(ExperienceState s) =>
            s == ExperienceState.PromptAnimal || s == ExperienceState.PromptMantis ||
            s == ExperienceState.PromptFree;

        // ONE-SHOT state-entry side effects (coroutines, spawning). Message
        // repaint lives in ShowStateMessage so the crowd notice can restore the
        // text WITHOUT re-entering the state (a re-entered prompt would start a
        // second capture routine and clobber _captureIndex).
        private void ApplyStateSideEffects(ExperienceState state)
        {
            switch (state)
            {
                case ExperienceState.Attract:
                    _ui.ClearAll();
                    SetSpheresActive(false);
                    SetMiniaturesActive(false);
                    StartAttractPlayback();
                    break;
                case ExperienceState.PromptAnimal:
                    StartPrompt(0, config.promptAnimalText);
                    break;
                case ExperienceState.PromptMantis:
                    StartPrompt(1, Pick(config.promptMantisVariants));
                    break;
                case ExperienceState.PromptFree:
                    StartPrompt(2, Pick(config.promptFreeVariants));
                    break;
                case ExperienceState.Select:
                    ShowSelection();
                    break;
                case ExperienceState.Exporting:
                    SetSpheresActive(false);
                    _exportRoutine = StartCoroutine(ExportAndPublish());
                    break;
                case ExperienceState.Fault:
                    string fault = healthMonitor != null ? healthMonitor.FaultAlertText : "";
                    _ui.ShowAlert(string.IsNullOrEmpty(fault) ? "カメラが異常です" : fault);
                    break;
            }
            if (state != ExperienceState.Fault) _ui.ClearAlert();
            ShowStateMessage(state);
        }

        // Attract ghost: start only when the recorder is idle (a dev playback
        // session already running is left untouched) or when the attract
        // controller already owns the playback (loop re-roll case).
        private void StartAttractPlayback()
        {
            if (_attract == null || sensorRecorder == null) return;
            _attract.RotationEnabled = true;
            if (sensorRecorder.IsPlaying && !_attractOwnsPlayback) return;
            if (HasLiveRenderers()) sensorManager?.SetLiveSuppressedAsSource(true);
            _attract.PlayRandomTake();
            _attractOwnsPlayback = _attract.IsRunning;
        }

        // Idempotent message repaint for the current state (also used when the
        // crowd notice clears).
        private void ShowStateMessage(ExperienceState state)
        {
            switch (state)
            {
                case ExperienceState.Attract: _ui.ShowMessage(config.attractText); break;
                case ExperienceState.Welcome: _ui.ShowMessage(config.welcomeText); break;
                case ExperienceState.FreePlay: _ui.ShowMessage(config.freePlayText); break;
                case ExperienceState.Ready: _ui.ShowMessage(config.readyText); break;
                case ExperienceState.PromptAnimal:
                case ExperienceState.PromptMantis:
                case ExperienceState.PromptFree:
                    _ui.ShowMessage(_currentPromptText);
                    break;
                case ExperienceState.Select: _ui.ShowMessage(config.selectText); break;
                case ExperienceState.Exporting:
                    _ui.ShowMessage(_exportFailed ? config.exportFailedText : config.exportingText);
                    break;
                case ExperienceState.QrShow: ShowQr(); break;
                case ExperienceState.Fault: break; // the alert owns the screen
            }
        }

        private static string Pick(string[] variants) =>
            variants == null || variants.Length == 0
                ? "" : variants[UnityEngine.Random.Range(0, variants.Length)];

        // ------------------------------------------------ capture ----

        private string _currentPromptText = "";

        private void StartPrompt(int captureIndex, string text)
        {
            _captureIndex = captureIndex;
            _captureDone = false; // per-prompt reset (double defense with the FSM latch)
            _currentPromptText = text;
            _captureRoutine = StartCoroutine(PromptCaptureRoutine());
        }

        private IEnumerator PromptCaptureRoutine()
        {
            var t = config.timings;
            float captureAt = Mathf.Max(0f, t.promptSeconds - 0.5f);
            float countdownFrom = Mathf.Max(0f, captureAt - config.countdownSeconds);

            while (_fsm.TimeInState < countdownFrom) yield return null;
            int shown = -1;
            while (_fsm.TimeInState < captureAt)
            {
                int remain = Mathf.Max(1, Mathf.CeilToInt(captureAt - _fsm.TimeInState));
                if (remain != shown) { _ui.ShowCountdown(remain); shown = remain; }
                yield return null;
            }

            // Freeze the 15-frame trail window and wait for a full curve rebuild.
            int saved = poseHistory != null ? poseHistory.historySamples : 0;
            if (poseHistory != null) poseHistory.historySamples = 15;
            if (motionCurves != null)
            {
                int target = motionCurves.BuildVersion + 2;
                float deadline = Time.realtimeSinceStartup + 2f;
                while (motionCurves.BuildVersion < target &&
                       Time.realtimeSinceStartup < deadline)
                    yield return null;
                if (motionCurves.BuildVersion < target)
                    Debug.LogWarning($"[{nameof(ExperienceDirector)}] curve rebuild wait timed out — " +
                                     "capturing anyway (mesh-only capture is valid).", this);
            }

            var snap = TSDFSnapshotBuilder.Capture(tsdfVolume, motionCurves, CaptureOptions(), out string err);
            if (poseHistory != null) poseHistory.historySamples = saved;
            if (snap == null)
                Debug.LogWarning($"[{nameof(ExperienceDirector)}] capture {_captureIndex} failed: {err} " +
                                 "— continuing without it.", this);
            else
                _snapshots[_captureIndex] = snap;

            _captureDone = true;
            _captureRoutine = null;
        }

        private void RestoreHistorySamples()
        {
            if (poseHistory != null && _savedHistorySamples > 0)
                poseHistory.historySamples = _savedHistorySamples;
        }

        private TSDFSnapshotBuilder.CaptureOptions CaptureOptions()
        {
            if (printExporter != null)
                return new TSDFSnapshotBuilder.CaptureOptions
                {
                    smoothIterations = printExporter.smoothIterations,
                    triangleBudgetPerSlab = printExporter.triangleBudgetPerSlab,
                    meshTargetTris = printExporter.webMeshTargetTris,
                    includeCurves = printExporter.webIncludeCurves,
                    curveStride = printExporter.webCurveStride,
                    curveSides = printExporter.webCurveSides,
                    curveTipTaper = printExporter.webCurveTipTaper,
                    curveTolerance = printExporter.webCurveTolerance,
                };
            return new TSDFSnapshotBuilder.CaptureOptions
            {
                smoothIterations = 10,
                triangleBudgetPerSlab = 2_000_000,
                meshTargetTris = 150_000,
                includeCurves = true,
                curveStride = 40,
                curveSides = 4,
                curveTipTaper = 0.25f,
                curveTolerance = 0.0015f,
            };
        }

        // ------------------------------------------------ selection ----

        private void SpawnSpheres()
        {
            for (int i = 0; i < 3 && i < config.sphereXOffsets.Length; i++)
            {
                var go = new GameObject($"_DwellSphere_{i}");
                go.transform.SetParent(transform, false);
                go.transform.position = new Vector3(
                    config.sphereXOffsets[i],
                    config.floorY + config.sphereHeight,
                    config.sphereZ);
                var sphere = go.AddComponent<DwellSphere>();
                sphere.presence = _presence;
                sphere.radius = config.sphereRadius;
                sphere.dwellSeconds = config.sphereDwellSeconds;
                int index = i;
                sphere.OnSelected += _ => OnSphereSelected(index);
                go.SetActive(false);
                _spheres[i] = sphere;
            }
        }

        private void OnSphereSelected(int index)
        {
            if (_snapshots[index] == null) return; // empty slot never counts
            _selectedIndex = index;
        }

        private void ShowSelection()
        {
            _selectedIndex = -1;
            if (_miniatureMat == null)
            {
                var shader = Resources.Load<Shader>("SnapshotVertexColor");
                if (shader != null)
                    _miniatureMat = new Material(shader)
                    { name = "Snapshot miniature (auto)", hideFlags = HideFlags.DontSave };
            }
            for (int i = 0; i < 3; i++)
            {
                bool has = _snapshots[i] != null && _spheres[i] != null;
                if (_spheres[i] != null)
                {
                    _spheres[i].gameObject.SetActive(has);
                    if (has) _spheres[i].ResetSphere();
                }
                if (!has) continue;
                BuildMiniature(i);
            }
        }

        private void BuildMiniature(int i)
        {
            if (_miniatures[i] != null) Destroy(_miniatures[i]);
            TSDFSnapshotBuilder.BuildDisplayMeshes(_snapshots[i], out var surface, out var tubes);

            var root = new GameObject($"_Miniature_{i}");
            root.transform.SetParent(transform, false);
            float s = config.displayMiniatureScale;
            root.transform.localScale = Vector3.one * s;
            // Centre the world-space mesh on the sphere, floating above it.
            Vector3 centre = surface.bounds.center;
            Vector3 anchor = _spheres[i].transform.position + Vector3.up * (config.sphereRadius + 0.35f);
            root.transform.position = anchor - centre * s;

            AddMeshChild(root.transform, surface, "surface");
            if (tubes != null) AddMeshChild(root.transform, tubes, "tubes");
            _miniatures[i] = root;
        }

        private void AddMeshChild(Transform parent, Mesh mesh, string name)
        {
            var go = new GameObject(name);
            go.transform.SetParent(parent, false);
            go.AddComponent<MeshFilter>().sharedMesh = mesh;
            var mr = go.AddComponent<MeshRenderer>();
            mr.sharedMaterial = _miniatureMat;
            mr.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
        }

        private void SetSpheresActive(bool active)
        {
            foreach (var s in _spheres)
                if (s != null) s.gameObject.SetActive(active && s.gameObject.activeSelf);
            if (!active) SetMiniaturesActive(false);
        }

        private void SetMiniaturesActive(bool active)
        {
            foreach (var m in _miniatures)
                if (m != null) m.SetActive(active);
        }

        // ------------------------------------------------ export / publish ----

        private IEnumerator ExportAndPublish()
        {
            _exportDone = _exportFailed = false;
            var snap = _selectedIndex >= 0 ? _snapshots[_selectedIndex] : null;
            if (snap == null) { _exportFailed = true; ShowExportFailed(); yield break; }

            string dir = Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.UserProfile),
                                      "Documents", "FloatingVectorsPrints");
            string stamp = DateTime.Now.ToString("yyyyMMdd_HHmmss");
            string glbPath = Path.Combine(dir, $"exp_{stamp}.glb");
            string usdzPath = Path.Combine(dir, $"exp_{stamp}.usdz");
            string usdPython = printExporter != null ? printExporter.usdPythonPath : "";

            // Synchronous local write (accepted for Phase 5 — atomic tmp+rename,
            // at most one export blocks a mode exit; revisit Task.Run in Phase 7).
            if (!TSDFSnapshotBuilder.ExportFiles(snap, glbPath, usdzPath, usdPython,
                                                 out _, out string err))
            {
                Debug.LogError($"[{nameof(ExperienceDirector)}] export failed: {err}", this);
                _exportFailed = true;
                ShowExportFailed();
                yield break;
            }

            ISculptureResultPublisher publisher;
            if (config.dryRunPublish)
            {
                publisher = new DryRunPublisher(config.dryRunDelaySeconds);
            }
            else
            {
                string token = ResolveLfksToken();
                if (string.IsNullOrEmpty(token))
                {
                    Debug.LogError($"[{nameof(ExperienceDirector)}] no LFKS token " +
                                   "(persistentDataPath/lfks-token.txt or LFKS_TOKEN env) — " +
                                   $"files kept locally: {glbPath}", this);
                    _exportFailed = true;
                    ShowExportFailed();
                    _exportRoutine = null;
                    yield break;
                }
                publisher = new LfksUploadPublisher(
                    Path.Combine(Application.streamingAssetsPath, "lfks", "upload.ps1"),
                    config.uploadScriptSha256, token,
                    config.lfksRemoteDirectory, config.publishTimeoutSeconds);
            }
            _cts = new CancellationTokenSource();
            _publishTask = publisher.PublishAsync(glbPath, usdzPath, _cts.Token);
            while (!_publishTask.IsCompleted) yield return null;

            if (_publishTask.IsCanceled) { _exportRoutine = null; yield break; }
            if (_publishTask.IsFaulted || !_publishTask.Result.Success)
            {
                string why = _publishTask.IsFaulted
                    ? _publishTask.Exception?.GetBaseException().Message
                    : _publishTask.Result.Error;
                Debug.LogError($"[{nameof(ExperienceDirector)}] publish failed: {why} " +
                               $"(files kept for manual upload: {glbPath})", this);
                _exportFailed = true;
                ShowExportFailed();
            }
            else
            {
                var r = _publishTask.Result;
                _qrUrl = config.qrUrlKind switch
                {
                    QrUrlKind.Glb => r.GlbUrl,
                    QrUrlKind.Usdz => r.UsdzUrl,
                    _ => string.IsNullOrEmpty(r.UsdzUrl) ? r.GlbUrl : r.UsdzUrl,
                };
                _exportDone = true;
            }
            _exportRoutine = null;
        }

        private void ShowExportFailed() => _ui.ShowMessage(config.exportFailedText);

        // Token lives OUTSIDE the repo/assets: a user-local file first, the
        // environment second. Trimmed; empty = not configured.
        private static string ResolveLfksToken()
        {
            try
            {
                string path = Path.Combine(Application.persistentDataPath, "lfks-token.txt");
                if (File.Exists(path))
                {
                    string fromFile = File.ReadAllText(path).Trim();
                    if (fromFile.Length > 0) return fromFile;
                }
            }
            catch (Exception e)
            {
                Debug.LogWarning($"[{nameof(ExperienceDirector)}] token file read failed: {e.Message}");
            }
            return Environment.GetEnvironmentVariable("LFKS_TOKEN")?.Trim();
        }

        private void CancelPublish()
        {
            if (_cts != null)
            {
                _cts.Cancel();
                _cts.Dispose();
                _cts = null;
            }
            // Observe the task so a cancellation/fault never goes unobserved.
            _publishTask?.ContinueWith(t => { _ = t.Exception; },
                TaskContinuationOptions.OnlyOnFaulted);
            _publishTask = null;
        }

        // ------------------------------------------------ QR / crowd ----

        private void ShowQr()
        {
            var tex = new QrUrlPresenter().Present(_qrUrl);
            string caption = string.IsNullOrEmpty(config.qrCaption) ? "" : config.qrCaption;
            if (tex != null) _ui.ShowQr(tex, caption);
            else _ui.ShowMessage(_qrUrl ?? "");
        }

        private void UpdateCrowdNotice()
        {
            if (_fsm == null || _ui == null) return;
            var s = _fsm.State;
            bool eligible = s != ExperienceState.Attract && s != ExperienceState.Fault &&
                            s != ExperienceState.QrShow && s != ExperienceState.Exporting;
            bool crowd = eligible && _presence != null && _presence.CrowdActive;
            if (crowd && !_crowdShowing)
            {
                _ui.ShowMessage(config.crowdText);
                _crowdShowing = true;
            }
            else if (!crowd && _crowdShowing)
            {
                _crowdShowing = false;
                ShowStateMessage(s); // repaint only — never re-enter the state
            }
        }
    }
}
