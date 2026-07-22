// The show's single owner: drives the one-second-take ExperienceStateMachine
// (Idle → Calibrate → FreeMove → Shoot → Processing → ResultShow → QrShow —
// Idle shows nothing but the floor grid while unattended), owns
// every runtime object the experience spawns (visitor UI, presence detector,
// live skeleton feed), snapshots the dev-mode values it touches on Enter and
// restores them all on Exit — Dev mode must come back bit-identical.
//
// Sequence responsibilities per state:
//   Calibrate  — star-pose guide + window countdown; while the pose is held,
//                per-camera bone lengths are sampled (LiveFusedBodySource)
//                and become the PER-VISITOR fusion profile — applied to the
//                live fusion immediately and to the take conversion later.
//                Window expiry proceeds with the default profile.
//   FreeMove   — free movement with the (now personally calibrated) live
//                sculpture.
//   Shoot      — cue text → countdown (recording starts WITH the countdown so
//                the fusion has warm-up) → at zero ONE second is the take →
//                recording stops.
//   Processing — FusedTakeConverter (v11s, per-visitor profile) on a worker
//                thread with a progress bar, then a single play-through of
//                the converted take (loop off) and the final capture over the
//                one-second trail window. Failure at any step falls back so
//                the visitor is never dead-ended.
//   ResultShow — the sculpture stays frozen at the captured moment (that IS
//                the exported shape) while glb/usdz export + upload run.
//   QrShow     — QR + scan prompt over the frozen result.
//
// Capture protocol (carried over): set historySamples to the one-second
// window -> wait for PointCloudMotionCurves.BuildVersion +2 (2 s timeout;
// the ring always holds MaxK frames, so this works while stopped) ->
// TSDFSnapshotBuilder.Capture (sync, non-destructive). The window stays
// applied through ResultShow/QrShow so the screen keeps showing exactly what
// was exported; it is restored on the next Idle.

using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Threading;
using System.Threading.Tasks;
using BodyTracking;
using BodyTracking.Eval.Rtmpose;
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
        public TSDFIntegrator tsdfIntegrator; // frozen at capture so the result stops recomputing
        public TSDFView tsdfView;             // hidden during Processing (see SetSculptureVisible)
        public PointCloudMotionCurves motionCurves;
        public BonePoseHistory poseHistory;
        public TSDFPrintExporter printExporter; // capture/export settings source
        public CameraHealthMonitor healthMonitor; // fault source (auto-resolved)

        [Tooltip("Live CUDA fusion source (RTMPose). When an enabled instance exists " +
                 "it IS the live skeleton pipeline: fused bodies drive the merger " +
                 "during live phases and the calibration pose samples the per-visitor " +
                 "bone profile from its per-camera skeletons. The k4abt " +
                 "LiveSkeletonFeed stays dormant. Auto-resolves when empty; absent = " +
                 "k4abt live pipeline (no per-visitor profile).")]
        public LiveFusedBodySource liveFusedSource;

        [Header("Startup")]
        [Tooltip("Enter experience mode automatically, one frame into Play, so the " +
                 "exhibition machine comes up showing the visitor experience with no " +
                 "operator action. Deliberately waits a frame: every other component's " +
                 "Start() (SensorManager going live, recorder, fusion) must have run " +
                 "before EnterMode snapshots their state. Turning the mode off by hand " +
                 "afterwards sticks — this only ever fires once per Play.")]
        public bool activateOnPlay;

        [Header("Debug")]
        [Tooltip("Force the Fault state (full-screen red alert) regardless of the " +
                 "health monitor.")]
        public bool debugForceFault;

        [Tooltip("Log the star-pose conditions and their measured values twice a " +
                 "second while Calibrate has not matched — for tuning the star* " +
                 "thresholds against a real visitor. A '!' marks the failing term.")]
        public bool debugPoseDiagnostics;

        [Tooltip("Snapshot the LIVE curve the instant Processing begins, then compare " +
                 "it against the v11s curve the capture actually exports, and log the " +
                 "difference. The two are built from the same second of movement — the " +
                 "live one from the fusion as it ran, the exported one from replaying " +
                 "the take through the re-converted bodies — so they should differ only " +
                 "in precision. They visibly do not, and this is how that gets pinned " +
                 "down. OFF by default because it is not free: TryReadCurvePolylines " +
                 "does a synchronous GPU readback per sampled seed, i.e. hundreds of " +
                 "blocking readbacks right at the start of Processing.")]
        public bool compareCurveSources;

        // ---- IViewToggle ("Experience mode" in the Views panel) ----
        public string ViewLabel => "Experience mode";
        public bool Visible
        {
            get => _active;
            set { if (value && !_active) EnterMode(); else if (!value && _active) ExitMode(); }
        }

        public ExperienceState CurrentState => _fsm?.State ?? ExperienceState.Idle;
        public bool IsActive => _active;
        /// <summary>Session body metrics (defaults until the star pose measured them).</summary>
        public VisitorBodyMetrics CurrentMetrics => _metrics;
        /// <summary>True once the star pose actually measured the visitor.</summary>
        public bool MetricsMeasured => _metricsMeasured;
        /// <summary>True once the per-visitor bone profile was measured and applied.</summary>
        public bool ProfileMeasured => _visitorProfile != null;

        private bool _active;
        private ExperienceStateMachine _fsm;

        // spawned per mode session
        private VisitorMessageUI _ui;
        private bool _uiSpawned; // false = scene-authored instance (kept on exit)
        private PresenceDetector _presence;
        private LiveSkeletonFeed _liveFeed;
        private ExperienceSpaceBuilder _space;
        private AudioSource _audio;
        private Texture2D _starGuideGenerated;

        // dev-value snapshot (restored on Exit)
        private int _savedHistorySamples;
        private bool _savedCurvesVisible, _savedCurvesFreeze;
        // The finished sculpture is frozen at capture (integrator gate closed +
        // curves frozen) so ResultShow/QrShow stop recomputing it — the model the
        // visitor sees is fixed the instant it is generated. Restored on the next
        // Idle (live-follow again) and on mode exit.
        private bool _sculptureFrozen;
        private bool _savedMgrRebase, _savedRecRebase;
        private float _lastPoseDiagAt;
        // The visitor-facing curve as it stood the instant Processing began — i.e.
        // the ribbons they were actually watching. Stage 2 replays the take and
        // rebuilds the ring from the v11s bodies, so this is the only chance to
        // keep the live one. Kept purely to compare the two (compareCurveSources).
        private readonly List<Vector3[]> _liveCurveLines = new List<Vector3[]>();
        private readonly List<Vector3> _liveCurveColors = new List<Vector3>();
        private bool _savedKeepLive;
        private string _savedPlaybackFolder;
        private bool _savedWasPlaying;
        private bool _savedWasPaused;
        private bool _savedWasLoaded;
        private bool _savedLiveFrozen;
        private bool _savedCumulativeNoErase;
        private PointCloudCumulative _cumulative;
        private readonly System.Collections.Generic.Dictionary<string, bool> _savedLiveVisible =
            new System.Collections.Generic.Dictionary<string, bool>();
        private readonly System.Collections.Generic.Dictionary<string, bool> _savedLiveSuppress =
            new System.Collections.Generic.Dictionary<string, bool>();

        // visitor recording (Shoot)
        private bool _visitorRecordingActive;
        private string _savedRecFolderPath;
        private bool _savedRecAutoTimestamp;

        // visitor playback (the Processing play-through; stays switched through
        // ResultShow/QrShow so the frozen result keeps its transport)
        private bool _visitorPlaybackActive;
        private bool _takeHasBodies = true;
        private int _savedRenderDelay;
        private bool _savedLoop;

        // body-source snapshot (EnterMode) — restored on Exit
        private bool _savedIgnoreRecorded;
        private bool _savedUseExternal;
        private bool _savedLfbsSubmit;
        private bool _savedLfbsLiveOnly;

        // per-run state
        private VisitorBodyMetrics _metrics = VisitorBodyMetrics.Default;
        private bool _metricsMeasured;
        private BodyProfile _visitorProfile;
        private BodyProfile _defaultProfile; // the fusion's profile before any visitor calibration
        private bool _profileSamplingActive;
        private PoseHoldDetector _starHold;
        private bool _calibrationDone;
        private bool _recordingDone;
        private bool _processingDone, _processingFailed;
        private string _takeRoot;
        private FusedTakeConverter _converter;
        private TSDFSnapshot _snapshot;
        private bool _exportDone, _exportFailed;
        private Coroutine _calibrateRoutine, _shootRoutine, _processingRoutine, _exportRoutine;
        private CancellationTokenSource _cts;
        private Task<PublishResult> _publishTask;
        private string _qrUrl;
        private bool _crowdShowing;
        private bool _autoActivated; // activateOnPlay latch (once per Play)
        private Coroutine _recoveryRoutine; // camera auto-recovery, Fault only
        private bool _savedHadLiveRig;      // live renderers existed when the show began

        // ------------------------------------------------ lifecycle ----

        private void OnEnable()
        {
            if (sensorManager == null) sensorManager = FindFirstObjectByType<SensorManager>();
            if (sensorRecorder == null) sensorRecorder = FindFirstObjectByType<SensorRecorder>();
            if (merger == null) merger = FindFirstObjectByType<SkeletonMerger>();
            if (boundingVolume == null) boundingVolume = FindFirstObjectByType<BoundingVolume>();
            if (floorOrigin == null) floorOrigin = FindFirstObjectByType<FloorOrigin>();
            if (tsdfVolume == null) tsdfVolume = FindFirstObjectByType<TSDFVolume>();
            if (tsdfIntegrator == null) tsdfIntegrator = FindFirstObjectByType<TSDFIntegrator>();
            if (tsdfView == null) tsdfView = FindFirstObjectByType<TSDFView>();
            if (motionCurves == null) motionCurves = FindFirstObjectByType<PointCloudMotionCurves>();
            if (poseHistory == null) poseHistory = FindFirstObjectByType<BonePoseHistory>();
            if (printExporter == null) printExporter = FindFirstObjectByType<TSDFPrintExporter>();
            if (healthMonitor == null) healthMonitor = FindFirstObjectByType<CameraHealthMonitor>();
            if (liveFusedSource == null) liveFusedSource = FindFirstObjectByType<LiveFusedBodySource>();
        }

        /// <summary>The live CUDA fusion pipeline is present and running.</summary>
        private bool LfbsActive => liveFusedSource != null && liveFusedSource.isActiveAndEnabled;

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
            // Exhibition auto-start: fires on the first Update (not Start) so every
            // other component's Start() has already run — EnterMode snapshots their
            // state, and a snapshot taken mid-startup restores the wrong values on
            // exit. Latched: an operator who turns the mode off keeps it off.
            if (activateOnPlay && !_autoActivated)
            {
                _autoActivated = true;
                if (!_active) Visible = true;
            }

            if (!_active) return;

            var inputs = new ExperienceInputs
            {
                Present = ComputePresent(),
                Fault = debugForceFault || (healthMonitor != null && !healthMonitor.IsHealthy),
                CalibrationDone = _calibrationDone,
                RecordingDone = _recordingDone,
                ProcessingDone = _processingDone,
                ProcessingFailed = _processingFailed,
                ExportDone = _exportDone,
                ExportFailed = _exportFailed,
            };
            _fsm.Tick(Time.deltaTime, in inputs, config.timings);
            UpdateCrowdNotice();

            // Fused source died mid-session (CUDA fault, component disabled):
            // bring up the k4abt fallback feed so calibration/presence keep a
            // live-skeleton provider. On recovery, retire the fallback again —
            // two live pipelines would fight for the GPU — and reassert the
            // current state's body-source mode (the recovered component
            // re-enabled with its own serialized flags).
            if (_liveFeed == null && !LfbsActive)
            {
                Debug.LogWarning($"[{nameof(ExperienceDirector)}] live fused source inactive — " +
                                 "spawning the k4abt LiveSkeletonFeed fallback.", this);
                SpawnLiveFeed();
            }
            else if (_liveFeed != null && LfbsActive)
            {
                Debug.Log($"[{nameof(ExperienceDirector)}] live fused source recovered — " +
                          "retiring the k4abt fallback feed.", this);
                Destroy(_liveFeed.gameObject);
                _liveFeed = null;
                ReapplyBodySourceForState(_fsm.State);
            }
        }

        private void ReapplyBodySourceForState(ExperienceState s)
        {
            switch (s)
            {
                case ExperienceState.Idle:
                case ExperienceState.Consent:
                case ExperienceState.Welcome:
                case ExperienceState.Calibrate:
                case ExperienceState.FreeMove:
                case ExperienceState.Shoot:
                    ApplyBodySource(BodySource.Live);
                    break;
                case ExperienceState.Processing:
                    // Live until the play-through switches the transport.
                    if (_visitorPlaybackActive) ApplyBodySource(BodySource.VisitorPlayback, _takeHasBodies);
                    else ApplyBodySource(BodySource.Live);
                    break;
                case ExperienceState.ResultShow:
                case ExperienceState.QrShow:
                    ApplyBodySource(BodySource.VisitorPlayback, _takeHasBodies);
                    break;
                    // Fault: leave the transport as the preceding state set it —
                    // the alert owns the screen, recovery goes through Idle.
            }
        }

        private void SpawnLiveFeed()
        {
            _liveFeed = new GameObject("_ExperienceLiveFeed").AddComponent<LiveSkeletonFeed>();
            _liveFeed.transform.SetParent(transform, false);
            _liveFeed.workerHost = merger != null ? merger.workerHost : null;
            _liveFeed.merger = merger;
            _liveFeed.sensorManager = sensorManager;
            _liveFeed.recorder = sensorRecorder;
            _liveFeed.trackingVolume = boundingVolume;
        }

        // While the visitor take's recorded bodies drive the merge (Processing
        // play-through onward) the merged-BT presence path sees the GHOST —
        // presence must come from live-only signals: GPU occupancy, the fused
        // live skeleton, or the k4abt live feed. In the no-bodies fallback the
        // merger re-runs k4abt on the playback — that "ghost" IS the visitor's
        // own take, so the merged path stays valid there (overstay risk is
        // bounded by the QR timeout). Idle has no ghost at all: the normal
        // presence blend fires the moment someone steps in.
        private bool ComputePresent()
        {
            if (_presence == null) return false;
            bool ghostDrivesMerge = _visitorPlaybackActive && _takeHasBodies;
            if (!ghostDrivesMerge) return _presence.IsPresent;
            return _presence.OccupancyActive
                || (LfbsActive && liveFusedSource.HasRecentFused)
                || (_liveFeed != null && _liveFeed.HasRecentPerson);
        }

        // ------------------------------------------------ enter / exit ----

        private void EnterMode()
        {
            if (config == null)
            {
                config = ScriptableObject.CreateInstance<ExperienceConfig>();
                Debug.LogWarning($"[{nameof(ExperienceDirector)}] no config assigned — using defaults.", this);
            }

            // Start each session with the recovery slot free. Unity stops coroutines on
            // disable without running their cleanup, so a handle left non-null there
            // would fail the `_recoveryRoutine == null` start guard forever and
            // silently disable camera recovery for the rest of the run.
            _recoveryRoutine = null;

            // 0) tear down the bone-verify diagnostic (F7/F8) if it is still up — it
            // forces showBones, curve visibility, a colour-grid overlay on a visitor
            // display and camera suppression, none of which may bleed into the
            // experience. Exit() restores all of that BEFORE we snapshot below, so the
            // experience takes over from the clean baseline, not the diagnostic's.
            var boneVerify = FindFirstObjectByType<BodyTracking.Eval.Rtmpose.BoneVerifyController>();
            if (boneVerify != null && boneVerify.CurrentMode != BodyTracking.Eval.Rtmpose.BoneVerifyController.Mode.Off)
                boneVerify.Exit();

            // 1) snapshot dev values.
            if (poseHistory != null) _savedHistorySamples = poseHistory.historySamples;
            if (motionCurves != null)
            {
                _savedCurvesVisible = motionCurves.visible;
                _savedCurvesFreeze = motionCurves.freeze;
                // freeze would stall the capture-readiness wait (curves Update
                // early-returns) — force it sane for the session. `visible` is
                // owned per-state by ApplyCurvesVisibility (off until Calibrate
                // is done), which the first state entry applies.
                motionCurves.freeze = false;
            }

            // 2) world rebase on (single reversible hook).
            if (sensorManager != null)
            {
                // Only the rebase flag is ours to force. rigSerialOrder is left
                // alone: each component resolves it from the machine-local
                // calibration/cameras.yaml, and this director used to overwrite
                // that with a git-synced config value — i.e. the other rig's
                // serials on one of the two machines.
                _savedMgrRebase = sensorManager.applyWorldRebase;
                sensorManager.applyWorldRebase = true;
                sensorManager.ApplyExtrinsicsToLive();
            }
            if (sensorRecorder != null)
            {
                _savedRecRebase = sensorRecorder.applyWorldRebase;
                sensorRecorder.applyWorldRebase = true;
                sensorRecorder.ReapplyExtrinsics();
            }

            // 3) sensing area (TSDF volume + floor grid follow automatically).
            _space = gameObject.AddComponent<ExperienceSpaceBuilder>();
            _space.boundingVolume = boundingVolume;
            _space.floorOrigin = floorOrigin;
            _space.sensorManager = sensorManager;
            _space.sensorRecorder = sensorRecorder;
            // rigSerialOrder left empty — ExperienceSpaceBuilder resolves it from
            // cameras.yaml (see the note on the rebase hook above).
            _space.floorY = config.floorY;
            _space.Apply();

            // 4) spawned objects.
            // Prefer a scene-authored UI (layout tunable + persisted on the GO);
            // spawn a throwaway one only when the scene has none.
            _ui = FindFirstObjectByType<VisitorMessageUI>(FindObjectsInactive.Include);
            _uiSpawned = _ui == null;
            if (_uiSpawned)
            {
                _ui = new GameObject("_ExperienceUI").AddComponent<VisitorMessageUI>();
                _ui.transform.SetParent(transform, false);
            }
            else _ui.gameObject.SetActive(true);
            _audio = _ui.GetComponent<AudioSource>();
            if (_audio == null) _audio = _ui.gameObject.AddComponent<AudioSource>();
            _audio.playOnAwake = false;
            _presence = new GameObject("_ExperiencePresence").AddComponent<PresenceDetector>();
            _presence.transform.SetParent(transform, false);
            _presence.merger = merger;
            _presence.sensingVolume = boundingVolume;
            _presence.sensorManager = sensorManager;
            _presence.insetMeters = config.insetMeters;
            _presence.yBandMin = config.yBandMin;
            _presence.yBandMax = config.yBandMax;
            _presence.occupancyThreshold = config.occupancyThreshold;
            // k4abt live-skeleton feed: only when no live CUDA fusion runs — with
            // LiveFusedBodySource active, pose detection reads the fused skeleton
            // and spawning k4abt workers would just fight it for the GPU. Update()
            // watches for a mid-session fused-source death and spawns the feed
            // then, so the k4abt fallback branches always have a provider.
            if (!LfbsActive) SpawnLiveFeed();
            else Debug.Log($"[{nameof(ExperienceDirector)}] live fused source active — " +
                           "k4abt LiveSkeletonFeed dormant unless the fusion stops.", this);

            // body-source snapshot (restored on Exit)
            if (merger != null)
            {
                _savedIgnoreRecorded = merger.ignoreRecordedBodies;
                _savedUseExternal = merger.useExternalBodies;
            }
            if (liveFusedSource != null)
            {
                _savedLfbsSubmit = liveFusedSource.submitToMerger;
                _savedLfbsLiveOnly = liveFusedSource.liveFramesOnly;
                // the session default (json-loaded or null) — restored between
                // visitors so one visitor's bones never shape the next
                _defaultProfile = liveFusedSource.CurrentBodyProfile;
            }

            _starHold = new PoseHoldDetector(config.starHoldSeconds, config.poseHoldDropoutSeconds);

            // 4b) playback coexistence: keep the live rig through playback loads,
            // hide the raw clouds (the TSDF mesh is the star), pause cumulative
            // snapshots.
            if (sensorRecorder != null)
            {
                _savedKeepLive = sensorRecorder.keepLiveRenderersOnLoad;
                _savedPlaybackFolder = sensorRecorder.playbackFolderPath;
                _savedWasPlaying = sensorRecorder.IsPlaying;             // Idle stops it; Exit resumes it
                _savedWasPaused = sensorRecorder.IsPaused;               // resume paused, not advancing
                _savedWasLoaded = sensorRecorder.RecordedFrameCount > 0; // loaded-but-stopped: Exit reloads
                // A live freeze (Space) must not carry into the show — every
                // renderer would keep holdLiveFrame and the whole experience
                // (sculpture, BT, curves) would stay frozen. Exit re-freezes.
                _savedLiveFrozen = !sensorRecorder.IsPlaying && sensorRecorder.IsPaused;
                if (_savedLiveFrozen) sensorRecorder.ResumeFrames();
                sensorRecorder.keepLiveRenderersOnLoad = true;
            }
            _savedLiveVisible.Clear();
            _savedLiveSuppress.Clear();
            // Did the operator have a live rig before the show? Only then may an
            // interrupted camera recovery put one back on exit — entering from a
            // playback-only dev session must come out of the show still playback-only.
            _savedHadLiveRig = sensorManager != null && sensorManager.Renderers.Count > 0;
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
            // 5) run.
            ResetRunState();
            _fsm = new ExperienceStateMachine();
            _fsm.Changed += OnStateChanged;
            _fsm.ResetTo(ExperienceState.Idle);
            _active = true;
            ApplyStateSideEffects(ExperienceState.Idle);
            Debug.Log($"[{nameof(ExperienceDirector)}] experience mode ON.", this);
        }

        private void ExitMode()
        {
            _active = false;

            StopRoutine(ref _calibrateRoutine);
            StopRoutine(ref _shootRoutine);
            StopRoutine(ref _processingRoutine);
            StopRoutine(ref _exportRoutine);
            // Camera recovery must not outlive the mode: left running it would call
            // StartLive() partway through the dev/playback restore below.
            if (_recoveryRoutine != null)
            {
                StopRoutine(ref _recoveryRoutine);
                // It may have been stopped between DestroyAllRenderers() and its own
                // StartLive(), which would strand the rig with no cameras. Re-opening
                // HERE is not safe though: DestroyAllRenderers clears the list at once
                // while Unity defers the actual Destroy() (and the OnDestroy that joins
                // the capture thread and disposes the Orbbec pipeline) to end of frame,
                // so a same-frame StartLive would re-open devices whose old pipelines
                // still hold them — the exact start-up race this recovery exists to
                // undo. Re-open on a delay instead.
                // ...and only when the operator actually had a live rig before the show:
                // entering from a playback-only dev session restores keepLiveRenderersOnLoad
                // =false and reloads the take below, which deliberately leaves live
                // renderers absent. Re-opening there would fight that restore and drag
                // CameraHealthMonitor out of its playback suppression path.
                if (_savedHadLiveRig && sensorManager != null && sensorManager.Renderers.Count == 0)
                {
                    if (isActiveAndEnabled) StartCoroutine(ReopenLiveAfterReleaseRoutine());
                    else
                        Debug.LogError($"[{nameof(ExperienceDirector)}] mode exited via disable " +
                                       "while camera recovery was mid-teardown — the live rig is " +
                                       "down and cannot be re-opened from OnDisable. Re-enter " +
                                       "experience mode (or restart Play) to bring it back.", this);
                }
            }
            CancelPublish();
            CancelProfileSampling();
            _converter?.Abort();
            _converter = null;
            AbortVisitorRecording();
            StopVisitorPlayback();
            RestoreHistorySamples(); // in case a capture was mid-flight

            // body-source restore (mirror of the EnterMode snapshot). The
            // useExternalBodies write comes LAST — SetSubmitToMerger may touch
            // the flag, and Dev mode must come back bit-identical even when the
            // pre-session combination was unusual (e.g. submit off, flag on).
            if (liveFusedSource != null)
            {
                liveFusedSource.SetSubmitToMerger(_savedLfbsSubmit);
                liveFusedSource.liveFramesOnly = _savedLfbsLiveOnly;
                if (_visitorProfile != null)
                    liveFusedSource.ApplyBodyProfile(_defaultProfile);
            }
            if (merger != null)
            {
                merger.ignoreRecordedBodies = _savedIgnoreRecorded;
                merger.muteWorkerIngest = false;
                merger.useExternalBodies = _savedUseExternal;
            }

            if (_fsm != null) { _fsm.Changed -= OnStateChanged; _fsm = null; }

            if (_ui != null)
            {
                _ui.ClearEverything();
                if (_uiSpawned) Destroy(_ui.gameObject);
                _ui = null;
            }
            _audio = null; // lived on the UI object
            if (_presence != null) { Destroy(_presence.gameObject); _presence = null; }
            if (_liveFeed != null) { Destroy(_liveFeed.gameObject); _liveFeed = null; }
            if (_starGuideGenerated != null) { Destroy(_starGuideGenerated); _starGuideGenerated = null; }
            _snapshot = null;

            if (sensorRecorder != null)
            {
                sensorRecorder.keepLiveRenderersOnLoad = _savedKeepLive;
                sensorRecorder.playbackFolderPath = _savedPlaybackFolder;
                // A dev playback session existed when the mode was entered
                // (Idle unloaded it) — bring it back in the same transport
                // state: reload when it was merely loaded, resume when it was
                // playing, re-pause when it was paused. The playhead restarts
                // from the beginning. An empty playbackFolderPath is fine —
                // Load() resolves the recorder's default root itself.
                if ((_savedWasPlaying || _savedWasLoaded) && !sensorRecorder.IsPlaying)
                {
                    sensorRecorder.Load();
                    if (_savedWasPlaying)
                    {
                        sensorRecorder.TogglePlay();
                        if (_savedWasPaused) sensorRecorder.PausePlayback();
                    }
                }
                // Reapply the live freeze the operator had before the show.
                if (_savedLiveFrozen && !sensorRecorder.IsPlaying && !sensorRecorder.IsPaused)
                    sensorRecorder.HoldFrames();
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
                sensorManager.ApplyExtrinsicsToLive();
            }
            if (sensorRecorder != null)
            {
                sensorRecorder.applyWorldRebase = _savedRecRebase;
                sensorRecorder.ReapplyExtrinsics();
            }

            if (motionCurves != null)
            {
                motionCurves.visible = _savedCurvesVisible;
                motionCurves.freeze = _savedCurvesFreeze;
            }
            // A result freeze must not leak past the show — reopen the integrator
            // gate and reset its batch state (curves already restored above to the
            // operator's own value, so this does not touch motionCurves.freeze).
            if (_sculptureFrozen)
            {
                ResumeIntegratorLiveFollow();
                _sculptureFrozen = false;
            }
            if (poseHistory != null) poseHistory.historySamples = _savedHistorySamples;
            // Leaving mode mid-Processing must not strand the sculpture hidden.
            SetSculptureVisible(true);

            Debug.Log($"[{nameof(ExperienceDirector)}] experience mode OFF (dev values restored).", this);
        }

        private void StopRoutine(ref Coroutine routine)
        {
            if (routine != null) { StopCoroutine(routine); routine = null; }
        }

        private void ResetRunState()
        {
            // Undo the previous visitor's fusion calibration BEFORE clearing the
            // reference — the live fusion must start every run on the session
            // default (or its absence), never on the last visitor's bones.
            if (_visitorProfile != null && liveFusedSource != null)
                liveFusedSource.ApplyBodyProfile(_defaultProfile);
            _metrics = VisitorBodyMetrics.Default;
            _metricsMeasured = false;
            _visitorProfile = null;
            _calibrationDone = false;
            _recordingDone = false;
            _processingDone = _processingFailed = false;
            _takeRoot = null;
            _snapshot = null;
            _exportDone = _exportFailed = false;
            _takeHasBodies = true;
            _qrUrl = null;
            _crowdShowing = false;
            _starHold?.Reset();
            // Release AND clear the frozen ribbons: the next visitor must draw their
            // own, not append to the previous one's curve.
            if (poseHistory != null) poseHistory.ReleaseHoldAndClear();
        }

        // ------------------------------------------------ state side effects ----

        private void OnStateChanged(ExperienceState from, ExperienceState to)
        {
            // Routine / device cleanup when a state is left by any path
            // (advance, walked away, fault).
            if (from == ExperienceState.Calibrate)
            {
                StopRoutine(ref _calibrateRoutine);
                // Timeout path (advancing without a held star pose): don't throw
                // the window's samples away. Bone lengths are pose-invariant, so
                // whatever the cameras collected over the window still yields a
                // usable best-effort profile — the star pose only makes the
                // samples cleaner. Walked-away/fault paths discard as before.
                if (to == ExperienceState.FreeMove) FinalizeProfileSamplingBestEffort();
                else CancelProfileSampling();
            }
            if (from == ExperienceState.Shoot)
            {
                StopRoutine(ref _shootRoutine);
                if (to != ExperienceState.Processing) AbortVisitorRecording();
            }
            if (from == ExperienceState.Processing && _processingRoutine != null)
            {
                StopRoutine(ref _processingRoutine);
                _converter?.Abort();
                RestoreHistorySamples(); // the capture may have been mid-flight
            }
            if (from == ExperienceState.ResultShow && to == ExperienceState.Idle)
                CancelPublish(); // fail path or skip; success path already completed

            // Idle → visitor handoff (k4abt pipeline only): restart the workers
            // across the clock jump left by the previous run's playback. The
            // fused source has no workers to restart. Consent is the first
            // interactive state now, so the restart happens on entering it.
            if (from == ExperienceState.Idle && to == ExperienceState.Consent)
            {
                if (merger != null && !LfbsActive && HasLiveRenderers()) merger.RestartWorkers();
            }

            if (to == ExperienceState.Idle || from == ExperienceState.QrShow)
                ResetRunState();

            ApplyStateSideEffects(to);
        }

        // ONE-SHOT state-entry side effects (coroutines, transport switching).
        // Message repaint lives in ShowStateMessage so the crowd notice can
        // restore the text WITHOUT re-entering the state.
        private void ApplyStateSideEffects(ExperienceState state)
        {
            var t = config.timings;
            switch (state)
            {
                case ExperienceState.Idle:
                    // Unattended: nothing on the screens but the scene itself —
                    // the floor grid (ExperienceSpaceBuilder) and, as someone
                    // steps in, their live sculpture.
                    UnfreezeSculpture(); // drop the previous run's frozen result → live-follow
                    StopVisitorPlayback();
                    // A playback session that predates Experience mode (dev
                    // session running, or loaded-and-stopped with its _Playback_*
                    // objects still showing the last frame) must not keep
                    // painting the stage — Idle owns it now. playbackFolderPath
                    // is restored on Exit.
                    if (sensorRecorder != null
                        && (sensorRecorder.IsPlaying || sensorRecorder.RecordedFrameCount > 0))
                        sensorRecorder.StopAndUnload();
                    RestoreHistorySamples(); // drop the one-second window of the previous run
                    _ui.ClearAll();
                    ApplyBodySource(BodySource.Live);
                    if (HasLiveRenderers()) sensorManager?.SetLiveSuppressedAsSource(false);
                    break;
                case ExperienceState.Consent:
                    break; // silent privacy notice — message only (ShowStateMessage)
                case ExperienceState.Welcome:
                    break; // greeting cue handled by PlayStateEnterSe below
                case ExperienceState.Calibrate:
                    if (!t.skipCalibrate)
                        _calibrateRoutine = StartCoroutine(CalibrateRoutine());
                    break;
                case ExperienceState.FreeMove:
                    break; // message only — the live sculpture is the content
                case ExperienceState.Shoot:
                    if (t.skipShoot)
                    {
                        _takeRoot = config.devCannedTakeRoot; // pre-recorded dev take
                    }
                    else
                    {
                        _shootRoutine = StartCoroutine(ShootRoutine());
                    }
                    break;
                case ExperienceState.Processing:
                    _processingRoutine = StartCoroutine(ProcessingRoutine());
                    break;
                case ExperienceState.ResultShow:
                    _exportRoutine = StartCoroutine(ExportAndPublish());
                    break;
                case ExperienceState.QrShow:
                    break; // QR cue handled by PlayStateEnterSe below
                case ExperienceState.Fault:
                    string fault = healthMonitor != null ? healthMonitor.FaultAlertText : "";
                    _ui.ShowAlert(string.IsNullOrEmpty(fault) ? "カメラが異常です" : fault);
                    // Try to bring the rig back by ourselves. Skipped when the operator
                    // forced the fault (debugForceFault) — there is no camera to fix.
                    if (config.autoRecoverCameras && !debugForceFault && _recoveryRoutine == null)
                        _recoveryRoutine = StartCoroutine(RecoverCamerasRoutine());
                    break;
            }
            if (state != ExperienceState.Fault)
            {
                _ui.ClearAlert();
                PlayStateEnterSe(state); // one cue per screen, on entry only
            }
            ApplyCurvesVisibility(state);
            // Sculpture hidden for the whole of Processing, revealed on entering
            // ResultShow. Driven off the state (not the routine) so every exit path —
            // fault, visitor walked off, processing failure — brings it back.
            SetSculptureVisible(state != ExperienceState.Processing);
            ShowStateMessage(state);
        }

        // The per-screen audio cue. Fires once as each state is entered (from
        // ApplyStateSideEffects, NOT the crowd-notice repaint), so a clip plays
        // exactly when its screen appears. Idle is silent by design. The pose-
        // matched / countdown / shutter cues are sub-state events fired from the
        // routines, not here.
        private void PlayStateEnterSe(ExperienceState state)
        {
            switch (state)
            {
                case ExperienceState.Consent: PlaySe(config.consentSe); break;
                case ExperienceState.Welcome: PlaySe(config.welcomeSe); break;
                case ExperienceState.Calibrate: PlaySe(config.calibrateSe); break;
                case ExperienceState.FreeMove: PlaySe(config.freeMoveSe); break;
                case ExperienceState.Shoot: PlaySe(config.shootCueSe); break;
                case ExperienceState.Processing: PlaySe(config.processingSe); break;
                case ExperienceState.ResultShow: PlaySe(config.resultSe); break;
                case ExperienceState.QrShow: PlaySe(config.qrShowSe); break;
            }
        }

        /// <summary>
        /// The ribbons stay OFF until the star pose is done. Idle..Calibrate are
        /// read-the-screen stages (notice, greeting, pose guide) and curves drawn
        /// from the approach walk are noise the visitor has no way to read; the
        /// sculpture should appear the moment it becomes theirs to move. From
        /// FreeMove on they are the content, and Processing/ResultShow need them
        /// visible because the capture builds from them.
        /// </summary>
        private void ApplyCurvesVisibility(ExperienceState state)
        {
            if (motionCurves == null) return;
            // The moment the star pose lands, the sculpture is theirs — show the
            // ribbons on the はかれたよ！ beat rather than one state later, so the
            // reveal is tied to the visitor's own action.
            if (state == ExperienceState.Calibrate && _calibrationDone)
            { motionCurves.visible = true; return; }
            motionCurves.visible = state is ExperienceState.FreeMove
                                        or ExperienceState.Shoot
                                        or ExperienceState.Processing
                                        or ExperienceState.ResultShow
                                        or ExperienceState.QrShow;
        }

        /// <summary>
        /// Hide the sculpture itself (TSDF mesh + ribbons) for the whole of Processing,
        /// so the finished piece is a REVEAL at ResultShow instead of something the
        /// visitor watched being rebuilt. Processing replays the take to rebuild the
        /// sculpture from the v11s bodies, and that replay is visibly noisy — measured
        /// on the rig, the TSDF swings 664k..825k vertices (~20% of the mesh, 31
        /// distinct values) over the ~2 s play-through. Only the progress bar shows
        /// during that.
        ///
        /// Draw-only on both halves, via each component's suppressDraw — the visible/
        /// showMesh flags are deliberately NOT used, because both gate production, not
        /// just drawing, and the capture consumes what they produce:
        ///   - curves.visible early-outs before the BUILD, and the capture reads the
        ///     built curves.
        ///   - tsdfView.showMesh gates TrianglesReady, which is what
        ///     PointCloudMotionCurves.CollectSeeds uses to seed ribbons off the TSDF
        ///     surface — dropping it would change which curves exist, and so change
        ///     what gets exported.
        /// suppressDraw on both leaves every buffer being produced exactly as normal
        /// and only skips the draw calls, so the export is bit-for-bit what it would
        /// have been with the sculpture on screen.
        /// </summary>
        private void SetSculptureVisible(bool shown)
        {
            if (tsdfView != null) tsdfView.suppressDraw = !shown;
            if (motionCurves != null) motionCurves.suppressDraw = !shown;
        }

        // ---------------- body-source switching ----------------
        // Two skeleton contexts, one switchboard. The LIVE pipeline is either
        // LiveFusedBodySource (CUDA RTMPose fusion → merger via external bodies)
        // or the k4abt workers; recorded bodies_main serves the visitor take
        // from the Processing play-through onward.
        private enum BodySource { Live, VisitorPlayback }

        private void ApplyBodySource(BodySource mode, bool takeHasBodies = true)
        {
            bool lfbs = LfbsActive;
            switch (mode)
            {
                case BodySource.Live:
                    if (lfbs)
                    {
                        liveFusedSource.SetSubmitToMerger(true);
                        liveFusedSource.liveFramesOnly = false;
                    }
                    if (merger != null)
                    {
                        merger.ignoreRecordedBodies = _savedIgnoreRecorded;
                        merger.muteWorkerIngest = false;
                    }
                    break;

                case BodySource.VisitorPlayback:
                    if (lfbs)
                    {
                        liveFusedSource.SetSubmitToMerger(false); // the take's bodies own the merge
                        liveFusedSource.liveFramesOnly = true;    // fuse the stage, not the replay
                    }
                    if (merger != null)
                    {
                        if (takeHasBodies)
                        {
                            merger.ignoreRecordedBodies = false;
                            merger.muteWorkerIngest = true; // and no worker spawn under the frozen result
                        }
                        else
                        {
                            // no bodies_main at all: the merger's re-run-k4abt-on-
                            // playback path is the only skeleton source — leave it on.
                            merger.ignoreRecordedBodies = _savedIgnoreRecorded;
                            merger.muteWorkerIngest = false;
                        }
                    }
                    break;
            }
        }

        // Idempotent message repaint for the current state (also used when the
        // crowd notice clears).
        private void ShowStateMessage(ExperienceState state)
        {
            switch (state)
            {
                case ExperienceState.Idle: _ui.ClearAll(); break; // floor grid only
                // Framed box, small body text: this one is READ, not glanced at.
                case ExperienceState.Consent: _ui.ShowNotice(config.consentText); break;
                case ExperienceState.Welcome: _ui.ShowMessage(config.welcomeText); break;
                case ExperienceState.Calibrate:
                    if (_calibrationDone) _ui.ShowMessage(config.calibrateMatchedText);
                    else _ui.ShowPoseGuide(StarGuide(), config.calibrateText);
                    break;
                case ExperienceState.FreeMove: _ui.ShowMessage(config.freeMoveText); break;
                case ExperienceState.Shoot:
                    // the routine owns the cue → countdown → shooting sequence;
                    // repaint just restores the cue text
                    _ui.ShowMessage(config.shootCueText);
                    break;
                case ExperienceState.Processing:
                    _ui.ShowProgress(_converter?.Progress ?? 0f, config.processingText);
                    break;
                case ExperienceState.ResultShow:
                    // Same layout as the QR screen (headline already in place) so
                    // ResultShow→QrShow reads as ONE scene that gains the QR.
                    if (_exportFailed) _ui.ShowMessage(config.exportFailedText);
                    else _ui.ShowResult(config.resultText);
                    break;
                case ExperienceState.QrShow: ShowQr(); break;
                case ExperienceState.Fault: break; // the alert owns the screen
            }
        }

        private Texture2D StarGuide() =>
            config.poseGuideTexture != null
                ? config.poseGuideTexture
                : _starGuideGenerated ??= StickFigureTexture.DrawStarPose();

        private void PlaySe(AudioClip clip)
        {
            if (clip != null && _audio != null) _audio.PlayOneShot(clip);
        }

        // ------------------------------------------------ Calibrate ----

        private IEnumerator CalibrateRoutine()
        {
            var t = config.timings;
            int shown = -1;
            // visitor-size logging while the star pose is held
            float armSum = 0f, spanSum = 0f, shoulderSum = 0f, heightSum = 0f;
            int sampleCount = 0;

            // Per-visitor bone profile: the fused source samples per-camera raw
            // skeletons while the pose is held (per-camera lengths avoid both the
            // fused output's length-projection circularity and the depth-lift Z
            // error). Without the fused source there is nothing to calibrate —
            // k4abt doesn't take a profile.
            if (LfbsActive)
            {
                liveFusedSource.BeginBodyProfileSampling();
                _profileSamplingActive = true;
            }

            while (_active && _fsm.State == ExperienceState.Calibrate)
            {
                // Digits only over the last countdownSeconds of the window; until
                // then the pose guide stands alone. A detected pose ends the
                // state early (FSM latch), skipping whatever countdown is left.
                int remain = Mathf.CeilToInt(t.calibrateSeconds - _fsm.TimeInState);
                if (remain > 0 && remain <= config.countdownSeconds && remain != shown)
                {
                    _ui.ShowCountdown(remain);
                    shown = remain;
                }

                bool star = false;
                if (TryGetLiveSkeleton(out var joints, out var valid))
                {
                    star = PoseClassifiers.IsStarPose(joints, valid,
                        config.starArmAngleFromUpMaxDeg, config.starLegAngleFromDownMinDeg);
                    if (debugPoseDiagnostics && !star
                        && Time.realtimeSinceStartup - _lastPoseDiagAt > 0.5f)
                    {
                        _lastPoseDiagAt = Time.realtimeSinceStartup;
                        Debug.Log("[star] " + PoseClassifiers.DescribeStarPose(joints, valid,
                            config.starArmAngleFromUpMaxDeg,
                            config.starLegAngleFromDownMinDeg), this);
                    }
                    if (star && PoseClassifiers.TryMeasureMetrics(joints, valid, out var m))
                    {
                        armSum += m.ArmLengthMeters;
                        spanSum += m.ArmSpanMeters;
                        shoulderSum += m.ShoulderWidthMeters;
                        heightSum += m.StandingHeightMeters;
                        sampleCount++;
                    }
                }

                if (_starHold.Update(star, Time.realtimeSinceStartup) && sampleCount > 0)
                {
                    _metrics = new VisitorBodyMetrics
                    {
                        ArmLengthMeters = armSum / sampleCount,
                        ArmSpanMeters = spanSum / sampleCount,
                        ShoulderWidthMeters = shoulderSum / sampleCount,
                        StandingHeightMeters = heightSum / sampleCount,
                    };
                    _metricsMeasured = true;

                    if (_profileSamplingActive)
                    {
                        _profileSamplingActive = false;
                        if (liveFusedSource.EndBodyProfileSampling(
                                config.profileMinSamplesPerBone, out var profile, out string summary))
                        {
                            _visitorProfile = profile;
                            liveFusedSource.ApplyBodyProfile(profile);
                            Debug.Log($"[{nameof(ExperienceDirector)}] per-visitor bone profile " +
                                      $"applied: {summary}", this);
                        }
                        else
                        {
                            Debug.LogWarning($"[{nameof(ExperienceDirector)}] bone-profile sampling " +
                                             $"insufficient ({summary}) — default profile stays.", this);
                        }
                    }

                    _calibrationDone = true;
                    ApplyCurvesVisibility(_fsm.State); // ribbons appear on the match
                    PlaySe(config.poseMatchedSe);
                    _ui.ShowMessage(config.calibrateMatchedText);
                    Debug.Log($"[{nameof(ExperienceDirector)}] star pose matched: arm " +
                              $"{_metrics.ArmLengthMeters:0.00}m span {_metrics.ArmSpanMeters:0.00}m " +
                              $"height {_metrics.StandingHeightMeters:0.00}m ({sampleCount} samples).", this);
                    break;
                }
                yield return null;
            }
            _calibrateRoutine = null;
        }

        /// <summary>Stop a still-running profile collection (leave / fault /
        /// exit) without applying anything.</summary>
        private void CancelProfileSampling()
        {
            if (!_profileSamplingActive) return;
            _profileSamplingActive = false;
            if (liveFusedSource != null)
                liveFusedSource.EndBodyProfileSampling(int.MaxValue, out _, out _);
        }

        /// <summary>Calibrate window expired without a held star pose: build the
        /// profile from whatever the window collected anyway (the hold path
        /// already finished sampling itself, so this is a no-op then).</summary>
        private void FinalizeProfileSamplingBestEffort()
        {
            if (!_profileSamplingActive) return;
            _profileSamplingActive = false;
            if (liveFusedSource == null) return;
            if (liveFusedSource.EndBodyProfileSampling(
                    config.profileMinSamplesPerBone, out var profile, out string summary))
            {
                _visitorProfile = profile;
                liveFusedSource.ApplyBodyProfile(profile);
                Debug.Log($"[{nameof(ExperienceDirector)}] calibration window expired — " +
                          $"best-effort per-visitor profile applied: {summary}", this);
            }
            else
            {
                Debug.LogWarning($"[{nameof(ExperienceDirector)}] calibration window expired and " +
                                 $"sampling was insufficient ({summary}) — default profile stays.", this);
            }
        }

        // Live skeleton during LIVE phases: the merged output IS the live person
        // there — source-agnostic (fused-submitted or k4abt).
        private bool TryGetLiveSkeleton(out Vector3[] joints, out bool[] valid)
        {
            if (merger != null && merger.TryGetPrimarySkeleton(out joints, out valid)) return true;
            if (_liveFeed != null && _liveFeed.TryGetBestSkeleton(out joints, out valid)) return true;
            joints = null;
            valid = null;
            return false;
        }

        // ------------------------------------------------ Shoot (recording) ----

        private void StartVisitorRecording()
        {
            if (sensorRecorder == null) return;
            _savedRecFolderPath = sensorRecorder.folderPath;
            _savedRecAutoTimestamp = sensorRecorder.autoTimestampFolder;
            if (!string.IsNullOrEmpty(config.visitorRecordingRoot))
                sensorRecorder.folderPath = config.visitorRecordingRoot;
            sensorRecorder.autoTimestampFolder = true;
            sensorRecorder.StartRecording();
            _visitorRecordingActive = sensorRecorder.CurrentState == SensorRecorder.State.Recording;
            if (!_visitorRecordingActive)
            {
                Debug.LogError($"[{nameof(ExperienceDirector)}] recording failed to start " +
                               "(no live renderers?) — the take will be empty.", this);
                RestoreRecorderFolderConfig();
            }
        }

        // Cue text → countdown → zero → captureSeconds of the actual take → stop.
        //
        // Recording covers the CAPTURE WINDOW ONLY (plus recordPreRollSeconds).
        // It used to start with the countdown, to warm the fusion up before the
        // second that counts — but the live RTMPose fusion runs continuously
        // through FreeMove, so it is already warm when Shoot begins. Recording
        // the countdown only produced ~3 extra seconds per camera that the
        // v11s conversion then paid full inference on and the capture threw
        // away (the pose history keeps the capture window).
        private IEnumerator ShootRoutine()
        {
            var t = config.timings;
            float cueEnd = config.shootCueSeconds;
            float zeroAt = cueEnd + config.countdownSeconds;
            // Recording runs past the kept second by the reaction offset, because
            // the capture takes the LAST captureSeconds of the take — extending the
            // tail is what slides the kept window off the shutter and onto the
            // movement the visitor actually performs.
            float shootEnd = zeroAt + config.captureStartOffsetSeconds + config.captureSeconds;
            // Opening four RCSV writers is not instant; start that much early so
            // the first frames OF THE WINDOW are on disk. 0 = start exactly at zero.
            float recStart = Mathf.Max(cueEnd, zeroAt - config.recordPreRollSeconds);

            while (_active && _fsm.State == ExperienceState.Shoot && _fsm.TimeInState < cueEnd)
                yield return null;
            if (!_active || _fsm.State != ExperienceState.Shoot) yield break;

            int shown = -1;
            bool recStarted = false;
            while (_active && _fsm.State == ExperienceState.Shoot && _fsm.TimeInState < zeroAt)
            {
                if (!recStarted && _fsm.TimeInState >= recStart)
                {
                    StartVisitorRecording();
                    recStarted = true;
                }
                int remain = Mathf.Max(1, Mathf.CeilToInt(zeroAt - _fsm.TimeInState));
                if (remain != shown)
                {
                    _ui.ShowCountdown(remain);
                    PlaySe(config.countdownTickSe);
                    shown = remain;
                }
                yield return null;
            }
            if (!_active || _fsm.State != ExperienceState.Shoot) yield break; // exit cleanup aborts

            // recordPreRollSeconds 0 (or a countdown short enough that the loop
            // never ticked past recStart): start exactly at zero.
            if (!recStarted) StartVisitorRecording();

            PlaySe(config.recordEndSe); // shutter at zero
            _ui.ShowMessage(config.shootingText);
            while (_active && _fsm.State == ExperienceState.Shoot && _fsm.TimeInState < shootEnd)
                yield return null;
            if (!_active || _fsm.State != ExperienceState.Shoot) yield break;

            if (_visitorRecordingActive && sensorRecorder != null)
            {
                sensorRecorder.StopRecording();
                _takeRoot = sensorRecorder.LastRecordingRoot;
                _visitorRecordingActive = false;
                RestoreRecorderFolderConfig();
            }
            PlaySe(config.recordEndSe);
            _recordingDone = true;
            _shootRoutine = null;
        }

        /// <summary>Visitor walked away / fault mid-recording: stop and DISCARD
        /// (the take stays on disk but is not used).</summary>
        private void AbortVisitorRecording()
        {
            if (!_visitorRecordingActive) return;
            _visitorRecordingActive = false;
            if (sensorRecorder != null
                && sensorRecorder.CurrentState == SensorRecorder.State.Recording)
                sensorRecorder.StopRecording();
            RestoreRecorderFolderConfig();
        }

        private void RestoreRecorderFolderConfig()
        {
            if (sensorRecorder == null) return;
            sensorRecorder.folderPath = _savedRecFolderPath;
            sensorRecorder.autoTimestampFolder = _savedRecAutoTimestamp;
        }

        // -------------------------- Processing (v11s + play-through + capture) ----

        private IEnumerator ProcessingRoutine()
        {
            var t = config.timings;
            if (string.IsNullOrEmpty(_takeRoot) || !Directory.Exists(_takeRoot))
            {
                Debug.LogError($"[{nameof(ExperienceDirector)}] no take to process ('{_takeRoot}').", this);
                _processingFailed = true;
                _ui.ShowMessage(config.exportFailedText);
                _processingRoutine = null;
                yield break;
            }

            // Grab the live ribbons BEFORE anything replays over them — from here
            // on the pose ring belongs to the take play-through.
            _liveCurveLines.Clear();
            _liveCurveColors.Clear();
            if (compareCurveSources && motionCurves != null)
            {
                if (!motionCurves.TryReadCurvePolylines(CaptureOptions().curveStride,
                                                        _liveCurveLines, _liveCurveColors, out string curveWhy))
                    Debug.LogWarning($"[{nameof(ExperienceDirector)}] live curve snapshot skipped: {curveWhy}", this);
            }

            // ---- stage 1: v11s conversion (skippable) ----
            // Canned-take protection: the conversion rewrites bodies_main IN PLACE.
            // A dev E2E run must never mutate the canonical recording unless the
            // operator explicitly opted in with a disposable copy.
            bool convert = !t.skipProcessing
                && !(t.skipShoot && !config.allowCannedTakeConversion);
            if (t.skipShoot && !config.allowCannedTakeConversion && !t.skipProcessing)
                Debug.Log($"[{nameof(ExperienceDirector)}] canned take — skipping v11s conversion " +
                          "(allowCannedTakeConversion is off); playing existing bodies_main.", this);

            if (convert)
            {
                _converter = new FusedTakeConverter();
                bool started = _converter.Start(_takeRoot, new FusedTakeConverter.Options
                {
                    ModelsDir = config.conversionModelsDir,
                    BodyProfilePath = config.conversionBodyProfilePath,
                    ProfileOverride = _visitorProfile, // per-visitor calibration, if measured
                    Provider = config.conversionProvider,
                    ConfThreshold = config.conversionConfThreshold,
                    RunCatchupSmooth = config.runCatchupSmooth,
                });
                if (!started)
                {
                    Debug.LogWarning($"[{nameof(ExperienceDirector)}] v11s conversion could not start " +
                                     $"({_converter.Error}) — falling back to the take's own bodies.", this);
                }
                else
                {
                    float deadline = Time.realtimeSinceStartup + t.processingTimeoutSeconds;
                    float abandonAt = float.PositiveInfinity;
                    while (_converter.Status == FusedTakeConverter.ConvertStatus.Running)
                    {
                        float now = Time.realtimeSinceStartup;
                        if (float.IsInfinity(abandonAt) && now > deadline)
                        {
                            Debug.LogWarning($"[{nameof(ExperienceDirector)}] v11s conversion exceeded " +
                                             $"{t.processingTimeoutSeconds:0}s — aborting, falling back " +
                                             "to the take's own bodies.", this);
                            _converter.Abort();
                            abandonAt = now + 5f; // grace for the cooperative stop
                        }
                        // Abort() is cooperative — a worker stuck inside a native ORT
                        // call can't observe it. Don't hold the visitor hostage:
                        // abandon the thread after the grace window. Abort() locked
                        // the converter's finalize gate, so an abandoned thread can
                        // never rename/smooth bodies_main under the playback we
                        // start next — unless it already ENTERED finalize (bounded
                        // file I/O), which we wait out instead of racing.
                        if (now > abandonAt && !_converter.IsFinalizing)
                        {
                            Debug.LogWarning($"[{nameof(ExperienceDirector)}] converter did not stop " +
                                             "within the grace window — abandoning it.", this);
                            break;
                        }
                        _ui.ShowProgress(_converter.Progress * 0.8f, config.processingText);
                        yield return null;
                    }
                    if (_converter.Status != FusedTakeConverter.ConvertStatus.Done)
                        Debug.LogWarning($"[{nameof(ExperienceDirector)}] v11s conversion ended " +
                                         $"{_converter.Status} ({_converter.Error}) — playing the take's " +
                                         "own bodies.", this);
                }
            }

            // ---- stage 2: single play-through of the take (loop off) ----
            StartVisitorPlayback(_takeRoot);
            if (!_visitorPlaybackActive || sensorRecorder == null || !sensorRecorder.IsPlaying)
            {
                Debug.LogError($"[{nameof(ExperienceDirector)}] take play-through failed to start.", this);
                _processingFailed = true;
                _ui.ShowMessage(config.exportFailedText);
                _processingRoutine = null;
                yield break;
            }
            double duration = sensorRecorder.PlaybackDurationSeconds;
            float playDeadline = Time.realtimeSinceStartup + (float)duration + 10f;
            while (sensorRecorder.CurrentState == SensorRecorder.State.Playing
                   && Time.realtimeSinceStartup < playDeadline)
            {
                float frac = duration > 0.01
                    ? Mathf.Clamp01((float)(sensorRecorder.CurrentPlayheadSeconds / duration)) : 1f;
                _ui.ShowProgress(0.8f + 0.18f * frac, config.processingText);
                yield return null;
            }
            // loop=false → the recorder stopped itself at the last frame; the
            // scene now holds the end-of-take state (shell + pose ring). Deadline
            // expiry (slow disk, stalled clock): freeze the transport HERE so the
            // capture and the ResultShow screen agree — a still-running playback
            // would drift past whatever we export.
            if (sensorRecorder.CurrentState == SensorRecorder.State.Playing)
            {
                Debug.LogWarning($"[{nameof(ExperienceDirector)}] play-through did not finish within " +
                                 $"{duration + 10:0}s — pausing at the current frame and capturing there.", this);
                sensorRecorder.PausePlayback();
            }

            // ---- stage 3: capture the one-second window ----
            int trailFrames = Mathf.Clamp(Mathf.RoundToInt(config.captureSeconds * 30f), 2, 32);
            if (poseHistory != null) poseHistory.historySamples = trailFrames;
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

            // Freeze BEFORE the capture, not after the decimation wait: the take has
            // already stopped (stopped, not paused), so every frame from here on is
            // one where the staleness clear could empty the ring — and the wait below
            // yields for seconds. What the capture reads and what ResultShow shows
            // must be the same ribbons, so the hold has to start earlier than both.
            if (poseHistory != null) poseHistory.hold = true;

            var capOpt = CaptureOptions();
            capOpt.deferDecimation = true; // longest phase — off the main thread, below
            var snap = TSDFSnapshotBuilder.Capture(tsdfVolume, motionCurves, capOpt, out string err);
            if (snap == null)
            {
                Debug.LogError($"[{nameof(ExperienceDirector)}] final capture failed: {err}", this);
                _processingFailed = true;
                _ui.ShowMessage(config.exportFailedText);
                _processingRoutine = null;
                yield break;
            }

            // Everything above ran on the main thread — the frame is frozen for
            // exactly this long. Reported because "it freezes at the end of the
            // progress bar" is only actionable with the per-phase split (the
            // Marching Cubes slab readbacks and the weld are the parts that
            // cannot simply move to a worker).
            Debug.Log($"[{nameof(ExperienceDirector)}] capture blocked the main thread for " +
                      $"{snap.mcMs + snap.weldMs + snap.smoothMs} ms " +
                      $"(MC {snap.mcMs} ms / {snap.mcTris} tris, weld {snap.weldMs} ms, " +
                      $"smooth {snap.smoothMs} ms) — decimating {snap.trisBeforeDecimate} tris " +
                      "off-thread next.", this);

            // Decimation is seconds of single-threaded array math. Run it on a
            // worker thread and keep ticking here, so the sculpture stays live
            // and the progress bar keeps moving instead of the whole view
            // freezing between the bar filling and できたよ！ appearing.
            int targetTris = capOpt.meshTargetTris;
            var decimateTask = System.Threading.Tasks.Task.Run(
                () => TSDFSnapshotBuilder.Decimate(snap, targetTris));
            while (!decimateTask.IsCompleted)
            {
                _ui.ShowProgress(0.98f, config.processingText);
                yield return null;
            }
            if (decimateTask.IsFaulted)
            {
                // The undecimated mesh is still a valid capture — heavier to
                // export, but the visitor gets their sculpture.
                Debug.LogWarning($"[{nameof(ExperienceDirector)}] decimation failed " +
                                 $"({decimateTask.Exception?.GetBaseException().Message}) — " +
                                 "exporting the full-resolution mesh.", this);
            }

            if (compareCurveSources) LogCurveComparison(snap);

            // Freeze the sculpture the instant it is generated: close the TSDF
            // integrator gate and freeze the curves so ResultShow/QrShow stop
            // recomputing the model. Without this the integrator stays in
            // live-follow (integrationEnabled=true) and the curves keep rebuilding
            // every frame — the frozen result visibly flickers. Restored on the
            // next Idle (live-follow again) and on mode exit.
            FreezeSculpture();

            _snapshot = snap;
            _ui.ShowProgress(1f, config.processingText);
            // historySamples intentionally stays at the one-second window: the
            // frozen curves on screen ARE the exported shape. Restored on Idle.
            _processingDone = true;
            _processingRoutine = null;
        }

        /// <summary>
        /// Report how far the exported (v11s, replayed) curve sits from the live one
        /// the visitor was watching when Processing began. Both describe the same
        /// second of the same movement, so a small spread is precision and a large
        /// one means they are not the same motion at all. Deliberately reports shape
        /// AND placement separately: a curve that is the right shape in the wrong
        /// place is a frame/timing bug, while a different shape is a tracking one.
        /// </summary>
        private void LogCurveComparison(TSDFSnapshot snap)
        {
            int liveN = _liveCurveLines.Count, expN = snap.curveLines.Count;
            if (liveN == 0 || expN == 0)
            {
                Debug.Log($"[{nameof(ExperienceDirector)}] curve compare: nothing to compare " +
                          $"(live {liveN} polylines, exported {expN}).", this);
                return;
            }

            Vector3 liveC, expC; float liveLen, expLen;
            Bounds liveB = CurveStats(_liveCurveLines, out liveC, out liveLen);
            Bounds expB = CurveStats(snap.curveLines, out expC, out expLen);

            Debug.Log($"[{nameof(ExperienceDirector)}] curve compare — " +
                      $"live: {liveN} lines, total length {liveLen:0.00} m, centroid {liveC:F3}, " +
                      $"size {liveB.size:F3} | " +
                      $"exported(v11s): {expN} lines, total length {expLen:0.00} m, centroid {expC:F3}, " +
                      $"size {expB.size:F3} | " +
                      $"centroid offset {Vector3.Distance(liveC, expC):0.000} m, " +
                      $"length ratio {(liveLen > 1e-4f ? expLen / liveLen : 0f):0.00}", this);
        }

        // Bounds + centroid + summed polyline length over a curve set.
        private static Bounds CurveStats(List<Vector3[]> lines, out Vector3 centroid, out float totalLength)
        {
            var b = new Bounds();
            bool first = true;
            Vector3 sum = Vector3.zero;
            int n = 0;
            totalLength = 0f;
            foreach (var line in lines)
            {
                if (line == null) continue;
                for (int i = 0; i < line.Length; i++)
                {
                    if (first) { b = new Bounds(line[i], Vector3.zero); first = false; }
                    else b.Encapsulate(line[i]);
                    sum += line[i];
                    n++;
                    if (i > 0) totalLength += Vector3.Distance(line[i - 1], line[i]);
                }
            }
            centroid = n > 0 ? sum / n : Vector3.zero;
            return b;
        }

        // ------------------------------------------------ visitor playback ----

        // Single play-through transport for the converted take (loop OFF — the
        // recorder stops itself at the end and the scene freezes on the final
        // state, which ResultShow/QrShow keep on screen).
        private void StartVisitorPlayback(string takeRoot)
        {
            if (sensorRecorder == null) return;
            if (string.IsNullOrEmpty(takeRoot) || !Directory.Exists(takeRoot))
            {
                Debug.LogError($"[{nameof(ExperienceDirector)}] no playable take ('{takeRoot}').", this);
                return;
            }

            // The visitor take replaces any playback a dev session left running.
            if (sensorRecorder.IsPlaying) sensorRecorder.StopAndUnload();

            _savedRenderDelay = sensorRecorder.playbackRenderDelayFrames;
            sensorRecorder.playbackRenderDelayFrames = config.playbackRenderDelayFrames;
            _savedLoop = sensorRecorder.loop;
            sensorRecorder.loop = false;
            if (HasLiveRenderers()) sensorManager?.SetLiveSuppressedAsSource(true);

            sensorRecorder.playbackFolderPath = takeRoot;
            sensorRecorder.Load();

            // Skeleton source for the playback. Normal path: the take carries
            // bodies_main (v11s-converted) → merger consumes it. Edge: NO track
            // has bodies (conversion failed AND live ran in external mode, so
            // nothing was recorded) → the merger's re-run-k4abt-on-playback
            // path stays the only skeleton source.
            _takeHasBodies = false;
            foreach (var (serial, _) in PointCloudRecording.EnumerateDevices(takeRoot))
                if (sensorRecorder.HasRecordedBodies(serial)) { _takeHasBodies = true; break; }
            if (!_takeHasBodies)
                Debug.LogWarning($"[{nameof(ExperienceDirector)}] take has no recorded bodies — " +
                                 "keeping live-k4abt-on-playback as the skeleton source.", this);
            ApplyBodySource(BodySource.VisitorPlayback, _takeHasBodies);

            if (!sensorRecorder.IsPlaying) sensorRecorder.TogglePlay();
            _visitorPlaybackActive = true;
        }

        // Transport teardown only — the body source is (re)applied by whichever
        // context follows (Idle side effects, or the ExitMode restore).
        private void StopVisitorPlayback()
        {
            if (!_visitorPlaybackActive) return;
            _visitorPlaybackActive = false;
            if (sensorRecorder != null)
            {
                sensorRecorder.StopAndUnload();
                sensorRecorder.playbackRenderDelayFrames = _savedRenderDelay;
                sensorRecorder.loop = _savedLoop;
            }
        }

        // ---- camera auto-recovery (Fault only) ----

        /// <summary>
        /// Re-open the live rig that a stopped recovery left torn down, honouring the
        /// same release delay the recovery uses. Frame yield first: DestroyAllRenderers
        /// clears the list immediately but Unity runs the actual Destroy() — and the
        /// PointCloudRenderer.OnDestroy that joins the capture thread and disposes the
        /// Orbbec pipeline — at end of frame, so the device is not free until after it.
        /// Deliberately NOT gated on _active: it exists to clean up after mode exit.
        /// </summary>
        private IEnumerator ReopenLiveAfterReleaseRoutine()
        {
            yield return null; // let the deferred Destroy()/OnDestroy actually run
            yield return new WaitForSecondsRealtime(config.cameraReleaseSeconds);
            sensorManager?.StartLive();
            Debug.Log($"[{nameof(ExperienceDirector)}] re-opened the live rig that an " +
                      "interrupted camera recovery had torn down.", this);
        }

        /// <summary>
        /// Re-init the live rig while the show is stuck in Fault. The dropout this
        /// answers is a start-up race, not a USB fault (Docs/camera-dropout-
        /// investigation.md: a different camera each time, no OS-level USB error,
        /// fixed by re-entering Play), so tearing the renderers down and re-opening
        /// them is the actual remedy. Every consumer re-binds on its own —
        /// TSDFIntegrator.Update re-scans via BindAllSources, LiveFusedBodySource
        /// re-subscribes for late-joining renderers, CameraHealthMonitor re-hooks —
        /// so nothing else needs poking.
        ///
        /// Runs ONLY in Fault, where the FSM has already aborted the run, so it can
        /// never interrupt a visitor. A teardown is always followed by its StartLive
        /// in the same iteration — the loop only re-checks state before tearing down,
        /// never between, so we can't leave the rig destroyed.
        /// </summary>
        private IEnumerator RecoverCamerasRoutine()
        {
            int attempts = Mathf.Max(1, config.cameraRecoverAttempts);
            for (int attempt = 1; attempt <= attempts; attempt++)
            {
                // Left Fault between attempts (recovered, or the operator took over):
                // done, and NOT a give-up — fall out quietly rather than logging an error.
                if (!_active || _fsm == null || _fsm.State != ExperienceState.Fault)
                {
                    _recoveryRoutine = null;
                    yield break;
                }

                Debug.LogWarning($"[{nameof(ExperienceDirector)}] camera auto-recovery: " +
                                 $"re-init attempt {attempt}/{attempts} " +
                                 $"({healthMonitor?.FaultAlertText})", this);

                sensorManager?.DestroyAllRenderers();
                yield return new WaitForSecondsRealtime(config.cameraReleaseSeconds);
                sensorManager?.StartLive();

                // POLL for health rather than judging after a flat wait. StartLive
                // returns as soon as the renderers are spawned, but the devices need
                // roughly ten more seconds before frames actually flow — and until they
                // do, CameraHealthMonitor still reports stalled (staleFrameSeconds=3).
                // A fixed wait that expires first tears the rig down again MID-STARTUP,
                // which is strictly worse than doing nothing: measured on the rig, the
                // renderers came back 4/4 every attempt and were killed each time.
                // Polling exits the moment the rig is healthy and otherwise gives it
                // the whole window.
                float deadline = Time.realtimeSinceStartup + config.cameraSettleSeconds;
                while (Time.realtimeSinceStartup < deadline
                       && healthMonitor != null && !healthMonitor.IsHealthy)
                    yield return new WaitForSecondsRealtime(0.5f);

                if (healthMonitor == null || healthMonitor.IsHealthy)
                {
                    Debug.Log($"[{nameof(ExperienceDirector)}] camera auto-recovery: rig healthy " +
                              $"again after attempt {attempt} — the show resumes from Idle.", this);
                    _recoveryRoutine = null;
                    yield break;
                }
            }

            Debug.LogError($"[{nameof(ExperienceDirector)}] camera auto-recovery gave up after " +
                           $"{attempts} attempt(s) — the alert stays up for the operator. " +
                           $"({healthMonitor?.FaultAlertText})", this);
            _recoveryRoutine = null;
        }

        private void RestoreHistorySamples()
        {
            if (poseHistory != null && _savedHistorySamples > 0)
                poseHistory.historySamples = _savedHistorySamples;
        }

        // ---- sculpture freeze (final result holds fixed, no per-frame churn) ----

        // Close the integrator gate and freeze the curves so the FRONT buffer — the
        // exact mesh TSDFSnapshotBuilder.Capture just read — holds on screen.
        // Deliberately does NOT Publish(): unlike TSDFTrailBaker.StopCapture (single-
        // buffered accumulate, where write == front), the live-follow volume is
        // double-buffered and its hidden WRITE buffer may be a partial/cleared
        // in-progress batch — swapping that to the front would re-introduce the very
        // flicker this freezes out. Idempotent.
        private void FreezeSculpture()
        {
            if (_sculptureFrozen) return;
            if (tsdfIntegrator != null) tsdfIntegrator.integrationEnabled = false;
            if (motionCurves != null) motionCurves.freeze = true;
            _sculptureFrozen = true;
        }

        // Back to live-follow for the next visitor (called on Idle) — the curves
        // rebuild and the integrator folds live frames again. Exit restores the
        // operator's own freeze via _savedCurvesFreeze instead.
        private void UnfreezeSculpture()
        {
            if (!_sculptureFrozen) return;
            ResumeIntegratorLiveFollow();
            if (motionCurves != null) motionCurves.freeze = false;
            _sculptureFrozen = false;
        }

        // Re-open the integrator gate AND reset its batch state. Resuming after a
        // gated freeze without clearing the in-flight batch lets the batch state
        // machine believe it is mid-batch and publish a partial first batch
        // (TSDFIntegrator.BeginFreshBatch documents exactly this hazard).
        // BeginFreshBatch clears the batch serial set and the write buffer so the
        // next live/playback batch starts clean.
        private void ResumeIntegratorLiveFollow()
        {
            if (tsdfIntegrator == null) return;
            tsdfIntegrator.integrationEnabled = true;
            tsdfIntegrator.BeginFreshBatch();
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

        // ------------------------------------------------ export / publish ----

        private IEnumerator ExportAndPublish()
        {
            _exportDone = _exportFailed = false;
            var snap = _snapshot;
            if (snap == null) { _exportFailed = true; ShowExportFailed(); yield break; }

            string dir = Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.UserProfile),
                                      "Documents", "FloatingVectorsPrints");
            string stamp = DateTime.Now.ToString("yyyyMMdd_HHmmss");
            string glbPath = Path.Combine(dir, $"exp_{stamp}.glb");
            string usdzPath = Path.Combine(dir, $"exp_{stamp}.usdz");
            string usdPython = printExporter != null ? printExporter.usdPythonPath : "";

            // Let the ResultShow frame finish before blocking on the write.
            // StartCoroutine runs a coroutine synchronously up to its first yield, and
            // ResultShow's own side effects — revealing the sculpture, painting
            // できたよ！ — are applied later in the SAME ApplyStateSideEffects call.
            // uGUI only renders at end of frame, so without this yield the export
            // (measured ~3 s with the USDZ step) stalls the frame that was supposed to
            // present the reveal: the visitor keeps staring at the blank Processing
            // stage for the whole write and the result appears only after it. Yielding
            // once lets that frame present first, which is exactly the "frozen result
            // view absorbs the hitch" the write below assumes.
            yield return null;

            // A frame passed — re-check that the show still wants this write. Fault
            // preempts everything (ExperienceStateMachine.Tick) and mode exit can land
            // here too; either way the state that replaced ResultShow has already
            // painted, and blocking ~3 s now would stall the very frame meant to
            // present it — the red alert would stay off-screen for the whole write.
            // Skip the export instead: the run is being torn down regardless.
            if (!_active || _fsm == null || _fsm.State != ExperienceState.ResultShow)
            {
                Debug.LogWarning($"[{nameof(ExperienceDirector)}] left ResultShow before the " +
                                 "export began — skipping export/upload for this run.", this);
                _exportRoutine = null;
                yield break;
            }

            // Synchronous local write (atomic tmp+rename; the frozen result view
            // absorbs the hitch frame).
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
                string token = LfksToken.Resolve();
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
                    config.lfksRemoteDirectory, config.publishTimeoutSeconds, config.lfksApiUrl);
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
            string caption = !string.IsNullOrEmpty(config.qrScanText) ? config.qrScanText
                           : (config.qrCaption ?? "");
            // The result cheer and the QR share one screen (headline above the code).
            string headline = _exportFailed ? null : config.resultText;
            if (tex != null) _ui.ShowQr(tex, caption, headline);
            else _ui.ShowMessage(_qrUrl ?? "");
        }

        private void UpdateCrowdNotice()
        {
            if (_fsm == null || _ui == null) return;
            var s = _fsm.State;
            // Only during the live-interaction stages — playback/result states
            // read presence from occupancy where crowd counting is meaningless.
            bool eligible = s == ExperienceState.Welcome || s == ExperienceState.Calibrate
                || s == ExperienceState.FreeMove || s == ExperienceState.Shoot;
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
