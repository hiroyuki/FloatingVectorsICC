// The show's single owner: drives the one-second-take ExperienceStateMachine
// (Idle → Calibrate → TestMove1/2 → Shoot → Processing → ResultShow → QrShow —
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
//   TestMove1/2 — practice rounds: intro message(s) → countdown →
//                ONE second recorded → v11s conversion → できたよ！ +
//                playbackLoops looped play-throughs with the ribbons and the
//                orbit cameras. No capture, no export, no upload; the take
//                stays on disk unused. Ends in TestMoveDone.
//   Shoot      — ほんばん cue → countdown → at zero ONE second is
//                the take → recording stops.
//   Processing — FusedTakeConverter (v11s, per-visitor profile) on a worker
//                thread with a progress bar, then a single play-through of
//                the converted take (loop off) and the final capture over the
//                one-second trail window. Failure at any step falls back so
//                the visitor is never dead-ended.
//   ResultShow — the take replays playbackLoops times with the ribbons + the
//                orbit cameras (できたよ！) while glb/usdz export + upload run
//                in the background; the last loop freezes on the final frame.
//   QrShow     — QR + scan prompt on the right, the frozen line model centre
//                stage, orbit still running.
//
// The TSDF mesh is never drawn while the mode is on (suppressDraw held for
// the whole session): the visible content is the point cloud + the ribbons.
// Production still runs underneath — the capture/export consume the mesh.
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
    // Runs before every component whose Update ISSUES A DRAW, so a state change
    // and the draws it unsuppresses land in the same frame. At the default order
    // the Processing→ResultShow reveal split across two components: curves
    // (order 0, arbitrary vs this one) could already have skipped their draw
    // while TSDFView (order 10) always ran after and drew — one presented frame
    // of bare shell with no ribbons. And that frame is the one ExportAndPublish
    // then blocks on (~3 s), so the "one frame" sat on screen for seconds.
    // -20 keeps it after K4abtWorkerHost (-100), which is where the skeletons
    // this reads arrive from.
    [DefaultExecutionOrder(-20)]
    [DisallowMultipleComponent]
    public class ExperienceDirector : MonoBehaviour, Shared.IViewToggle, Shared.IStartupActivatable,
                                      Shared.IPanelTunable
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
        public PointCloudDecimater decimater; // presentation-look switch (auto-resolved)
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

        // ---- IStartupActivatable (Control Panel's big Activate On Play toggle) ----
        public string ActivateLabel => "Experience mode";
        public bool ActivateOnPlay
        {
            get => activateOnPlay;
            set => activateOnPlay = value;
        }

        // ---- IViewToggle ("Experience mode" in the Views panel) ----
        public string ViewLabel => "Experience mode";
        public bool Visible
        {
            get => _active;
            set { if (value && !_active) EnterMode(); else if (!value && _active) ExitMode(); }
        }

        // ---- IPanelTunable (Control Panel > Tuning) ----
        // Live sync is a genuine trade, not a fix, so it belongs on the panel rather than
        // buried in the config asset: ON aligns the cloud with the tracked skeleton at the
        // cost of showing everything ~150ms late; OFF is real-time but the skeleton
        // visibly trails the body during motion. Which one is right depends on what the
        // operator is judging, so let them flip it while looking at the result.
        public string TuningLabel => "Experience";
        public int TunableCount => 1;
        public string TunableName(int i) => "Live sync (0=real-time, 1=aligned)";
        public float TunableValue(int i) => config != null && config.liveRenderSync ? 1f : 0f;
        public void SetTunableValue(int i, float value)
        {
            if (config == null) return;
            config.liveRenderSync = value >= 0.5f;
            // Push it immediately: UpdateLiveRenderSync clears the delay on the next tick
            // when it goes off, but turning it ON should not wait for a state change.
            UpdateLiveRenderSync();
            if (_orbitOn) UpdatePresentationPivot();
        }
        public float TunableMin(int i) => 0f;
        public float TunableMax(int i) => 1f;
        public bool TunableIsInt(int i) => true;

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
        private string _savedPlaybackFolderMacOverride;
        private bool _savedWasPlaying;
        private bool _savedWasPaused;
        private bool _savedWasLoaded;
        private bool _savedLiveFrozen;
        private bool _savedCumulativeNoErase;
        private PointCloudCumulative _cumulative;
        private bool _savedMeshCumHotkeys;
        private TSDF.MeshCumulative _meshCumulative;
        private readonly System.Collections.Generic.Dictionary<string, bool> _savedLiveVisible =
            new System.Collections.Generic.Dictionary<string, bool>();
        private readonly System.Collections.Generic.Dictionary<string, bool> _savedLiveSuppress =
            new System.Collections.Generic.Dictionary<string, bool>();

        // visitor recording (Shoot / practice rounds)
        private bool _visitorRecordingActive;
        // The active recording is the playback TAP (rig-less machine: the
        // played entrance take is the only frame source — SensorRecorder
        // records its own playback stream into a new take).
        private bool _visitorRecordingTapped;
        // The current run's take came from the tap. Its bodies are passed
        // through from the already-converted source take, so the v11s
        // conversion is redundant (and on a CPU-only machine, prohibitive).
        private bool _takeIsTapped;
        private string _savedRecFolderPath;
        private bool _savedRecAutoTimestamp;
        // Every take recorded during the current run (practice rounds + the
        // real one). Visitor privacy + disk hygiene: they are all DELETED once
        // the QR is up (the QR only appears after export/upload completed, so
        // nothing reads them again) — the still-loaded take follows on unload.
        // Applies to dev AND production (user decision 2026-07-23). Kept across
        // ResetRunState so a failed delete (transient file lock) retries on the
        // next boundary.
        private readonly List<string> _runTakeRoots = new List<string>();

        // visitor playback (the Processing play-through; stays switched through
        // ResultShow/QrShow so the frozen result keeps its transport)
        private bool _visitorPlaybackActive;
        private bool _takeHasBodies = true;
        private int _savedRenderDelay;
        private bool _savedLoop;
        private float _savedPlaybackRate;

        // body-source snapshot (EnterMode) — restored on Exit
        private bool _savedIgnoreRecorded;
        private bool _savedUseExternal;
        private bool _savedShowBones;

        // point-cloud content (calibration finish → Shoot) + bottom-up reveal
        private PointCloudView _pcView;
        private bool _savedShowClouds;
        private bool _cloudsShown;
        private bool _cloudRevealed; // sweep finished — the bones hand over to the cloud
        private Coroutine _cloudRevealRoutine;
        private static readonly int PcRevealModeId = Shader.PropertyToID("_PcRevealMode");
        private static readonly int PcRevealYId = Shader.PropertyToID("_PcRevealY");
        private static readonly int PcFadeCullId = Shader.PropertyToID("_PcFadeCull");
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
        private bool _testMoveDone;    // practice mini-cycle finished (per TestMove state)
        // The practice presentation stretch (recording stopped → conversion →
        // load → looped playback): suspends the FSM's leave reset — see
        // ExperienceInputs.TestMovePresenting. Cleared on every exit path.
        private bool _testMovePresenting;
        private bool _resultShowDone;  // ResultShow playback loops finished
        // できたよ！ is withheld through the wireframe replay and painted only when
        // the replay finishes and the finished model is revealed (2026-07-24). Gates
        // ShowStateMessage(ResultShow) so any repaint (crowd notice clear) stays blank
        // until the routine flips it. Reset on every ResultShow entry / mode exit.
        private bool _deferResultText;
        private bool _recordingDone;
        private bool _processingDone, _processingFailed;
        private string _takeRoot;
        private FusedTakeConverter _converter;
        private TSDFSnapshot _snapshot;
        private bool _exportDone, _exportFailed;
        private Coroutine _calibrateRoutine, _shootRoutine, _processingRoutine, _exportRoutine;
        private Coroutine _testMoveRoutine, _resultShowRoutine;

        // Orbit cameras (visitor displays). Found via Shared.IOrbitOverride —
        // PauseOrbitGate lives in Assembly-CSharp, which this asmdef cannot
        // reference. The gates snapshot+restore the camera pose themselves
        // (RestorePoseOnDisable, forced on for the session).
        private struct OrbitGateSnap
        {
            public Shared.IOrbitOverride gate;
            public bool savedOverride;
            public bool savedRestore;
            public bool savedAutoOrbit;
            public bool savedSuppressTransport;
        }
        private readonly List<OrbitGateSnap> _orbitGates = new List<OrbitGateSnap>();
        private bool _orbitOn;
        private Coroutine _orbitEaseRoutine; // ResultShow yaw ease-in (0→full)
        // Person-tracking orbit anchor (chest↔waist midpoint of the merged
        // skeleton). The presentation camera looks here while it circles.
        private Transform _presentationPivot;
        private bool _pivotSeeded;
        private bool _savedTsdfSuppress; // mesh hidden for the whole session
        private Coroutine _gapPaintRoutine; // deferred state paint (stateGapSeconds)
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
            if (decimater == null) decimater = FindFirstObjectByType<PointCloudDecimater>();
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

            if (!_active)
            {
                // The jump keys stay live with the mode off — pressing one means
                // "turn the show on and go there" (same as the dev panel), so a
                // key press is never silently swallowed by a mode that something
                // (calibration suspend, HUD toggle, play restart) switched off.
                if (config != null && config.devStateJumpHotkeys) HandleDevJumpHotkeys();
                return;
            }

            var inputs = new ExperienceInputs
            {
                Present = ComputePresent(),
                Fault = debugForceFault || (healthMonitor != null && !healthMonitor.IsHealthy),
                CalibrationDone = _calibrationDone,
                TestMoveDone = _testMoveDone,
                TestMovePresenting = _testMovePresenting,
                RecordingDone = _recordingDone,
                ProcessingDone = _processingDone,
                ProcessingFailed = _processingFailed,
                ExportDone = _exportDone,
                ExportFailed = _exportFailed,
                ResultShowDone = _resultShowDone,
            };
            _fsm.Tick(Time.deltaTime, in inputs, config.timings);
            UpdateCrowdNotice();
            if (config.devStateJumpHotkeys) HandleDevJumpHotkeys();
            MaintainDevEntrancePlayback();

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

            UpdateLiveRenderSync();
            if (_orbitOn) UpdatePresentationPivot();
        }

        // --- live cloud <-> skeleton timestamp sync (curve-quality knob) ---
        // The live preview draws the cloud straight off the cameras while the
        // ribbons seed from a fused skeleton that runs ~130-166ms behind. During
        // motion the arm's points sit far from where the bones say the arm is, so
        // those seeds fail the bone-distance test and the ribbons thin out exactly
        // when the visitor moves. Matching the DISPLAYED cloud to the skeleton's
        // own timestamp puts both at the same instant. Display-only: BT and the
        // recording keep the newest frame, so the take behind ResultShow (which
        // has its own playbackRenderDelayFrames) is untouched. Cost: the preview
        // reads as a ~150ms-lagged mirror. Same mechanism BoneVerify's F8 live
        // mode uses; off by default until the lag trade is judged on the rig.
        private bool _liveSyncOn;

        private void UpdateLiveRenderSync()
        {
            // Every live state, not just the three that draw the sculpture. Restricting it
            // meant the visitor's first impression — walking up, Consent, Welcome — showed
            // the cloud in real time with the skeleton trailing ~150ms behind it, and the
            // mismatch then vanished the moment Calibrate began. A lag that appears and
            // disappears reads as a fault; a consistently aligned picture does not. The
            // cost is that the preview is a ~150ms-late mirror throughout, which is the
            // trade this project already accepted for the states that had it.
            // Visitor playback stays excluded: the take owns its own timing there
            // (SensorRecorder.playbackRenderDelayFrames).
            bool want = config != null && config.liveRenderSync && !_visitorPlaybackActive;
            if (!want)
            {
                if (_liveSyncOn) SetLiveRenderSync(false);
                return;
            }

            // No fresh skeleton (tracking gap, worker restart) → target 0 so the
            // cloud snaps back to real time instead of freezing on a stale target
            // once that frame falls out of the renderer's delay ring.
            ulong tsUs = 0UL;
            if (merger != null && merger.TryGetLatestSkeletonTimestampNs(out ulong tsNs))
                tsUs = tsNs / 1000UL;

            _liveSyncOn = true;
            if (sensorManager == null) return;
            foreach (var r in sensorManager.Renderers)
            {
                if (r == null) continue;
                r.renderDelayEnabled = true;
                r.targetDisplayTimestampUs = tsUs;
            }
        }

        private void SetLiveRenderSync(bool on)
        {
            _liveSyncOn = on;
            if (sensorManager == null) return;
            foreach (var r in sensorManager.Renderers)
            {
                if (r == null) continue;
                r.renderDelayEnabled = on;
                if (!on) r.targetDisplayTimestampUs = 0;
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
                case ExperienceState.Shoot:
                    ApplyBodySource(BodySource.Live);
                    break;
                case ExperienceState.TestMove1:
                case ExperienceState.TestMove2:
                case ExperienceState.Processing:
                    // Live until the play-through switches the transport (the
                    // TestMove states carry their own playback sub-phase).
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
            // Playback-driven rig (no live cameras): presence monitoring is OFF
            // by design — the stand-in visitor is a recording, and every
            // transport swap (practice takes, entrance reloads) opens multi-
            // second gaps in the fallback signals that reset the run for no
            // reason. The leave-grace / QR-departure flows are validated on the
            // real rig, where live renderers exist and the monitoring below
            // runs unchanged. Deliberately no toggle (user decision 2026-07-23).
            if (!HasLiveRenderers()) return true;
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
            // The TSDF mesh never draws during the show — the visible content is
            // the point cloud + the ribbons. Draw-only: integration/capture/export
            // still consume the mesh. Restored on Exit.
            if (tsdfView != null)
            {
                _savedTsdfSuppress = tsdfView.suppressDraw;
                tsdfView.suppressDraw = true;
            }
            // Orbit cameras: the show owns the override for the session (playback
            // and QR phases orbit the model) and forces pose-restore-on-disable so
            // the stage framing comes back for the next visitor / on exit.
            _orbitGates.Clear();
            _orbitOn = false;
            foreach (var mb in FindObjectsByType<MonoBehaviour>(FindObjectsSortMode.None))
            {
                if (mb is not Shared.IOrbitOverride gate) continue;
                _orbitGates.Add(new OrbitGateSnap
                {
                    gate = gate,
                    savedOverride = gate.OrbitOverride,
                    savedRestore = gate.RestorePoseOnDisable,
                    savedAutoOrbit = gate.AutoOrbit,
                    savedSuppressTransport = gate.SuppressTransportOrbit,
                });
                gate.OrbitOverride = false;
                gate.RestorePoseOnDisable = true;
                // The visitor has no mouse — an orbit phase must rotate by itself.
                gate.AutoOrbit = true;
                // The director owns the camera; the dev pause/bake auto-orbit must
                // not fire (freezing the model for the dissolve pauses the recorder).
                gate.SuppressTransportOrbit = true;
            }
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
            // Reuse a leftover builder ([DisallowMultipleComponent]): if an earlier
            // EnterMode died mid-way, AddComponent would return null here and every
            // future enter would NRE — the mode stays bricked until domain reload.
            if (!TryGetComponent(out _space))
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
                _savedShowBones = merger.showBones;
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
                _savedPlaybackFolderMacOverride = sensorRecorder.playbackFolderPathMacOverride;
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
            // Cloud visibility is per-state from here (ApplyCloudVisibility):
            // hidden through the bones-only intro, revealed bottom-up when
            // calibration finishes. PointCloudView is the single owner of every
            // cloud MR (live + playback), so the show drives that one knob.
            _pcView = FindFirstObjectByType<PointCloudView>();
            if (_pcView != null)
            {
                _savedShowClouds = _pcView.showPointClouds;
                _pcView.showPointClouds = false;
            }
            _cloudsShown = false;
            _cloudRevealed = false;
            Shader.SetGlobalFloat(PcRevealModeId, 0f);
            _cumulative = FindFirstObjectByType<PointCloudCumulative>();
            if (_cumulative != null)
            {
                _savedCumulativeNoErase = _cumulative.noErase;
                _cumulative.noErase = false;
            }
            // The show owns the freeze state — MeshCumulative's raw C/V/R
            // hotkeys must not fire mid-run (R doubles as a state-jump key).
            _meshCumulative = FindFirstObjectByType<TSDF.MeshCumulative>();
            if (_meshCumulative != null)
            {
                _savedMeshCumHotkeys = _meshCumulative.hotkeysEnabled;
                _meshCumulative.hotkeysEnabled = false;
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

            // Display-only, but it must not outlive the mode: a dev session left
            // with delayed live renderers looks like a stuck cloud.
            if (_liveSyncOn) SetLiveRenderSync(false);

            StopRoutine(ref _calibrateRoutine);
            StopRoutine(ref _testMoveRoutine);
            StopRoutine(ref _shootRoutine);
            StopRoutine(ref _processingRoutine);
            StopRoutine(ref _resultShowRoutine);
            StopRoutine(ref _exportRoutine);
            StopRoutine(ref _gapPaintRoutine);
            StopRoutine(ref _cloudRevealRoutine);
            Shader.SetGlobalFloat(PcRevealModeId, 0f); // never leak the clip into dev
            Shader.SetGlobalFloat(PcFadeCullId, 0f);   // nor an interrupted dissolve
            PointCloud.PointCloudDecimater.DecimFramePin = -1; // nor a pinned decimate seed
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
            DeleteRunTakes(includeLoaded: true); // visitor data never outlives the show
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
                merger.showBones = _savedShowBones;
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
            _snapshot = null;

            if (sensorRecorder != null)
            {
                sensorRecorder.keepLiveRenderersOnLoad = _savedKeepLive;
                sensorRecorder.playbackFolderPath = _savedPlaybackFolder;
                sensorRecorder.playbackFolderPathMacOverride = _savedPlaybackFolderMacOverride;
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
            // PointCloudView owns every cloud MR in dev; putting its flag back
            // makes its next Update reapply the operator's own visibility over
            // whatever per-state value the show left on the renderers.
            if (_pcView != null)
            {
                _pcView.showPointClouds = _savedShowClouds;
                _pcView = null;
            }
            if (_cumulative != null)
            {
                _cumulative.noErase = _savedCumulativeNoErase;
                _cumulative = null;
            }
            if (_meshCumulative != null)
            {
                _meshCumulative.hotkeysEnabled = _savedMeshCumHotkeys;
                _meshCumulative = null;
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
            // Leaving mode mid-Processing must not strand the ribbons hidden.
            SetSculptureVisible(true);
            // Session-long TSDF hide + orbit ownership — hand both back.
            // A wireframe replay interrupted mid-flight (visitor walked off, fault, dev
            // jump) must not strand the dev view as a wireframe or defer できたよ！.
            _wireframeReplay = false;
            _deferResultText = false;
            if (tsdfView != null) { tsdfView.wireframe = false; tsdfView.suppressDraw = _savedTsdfSuppress; }
            SetPresentationLook(false); // scene (live) values back before dev restore
            SetOrbit(false);
            foreach (var s in _orbitGates)
            {
                if (s.gate is not MonoBehaviour mb || mb == null) continue;
                // Off-edge NOW, while RestorePoseOnDisable is still the show's
                // forced true — the gate's own deferred Update would run only
                // after the dev flags below took effect, stranding the camera
                // at the orbit pose.
                s.gate.ReleaseOrbit();
                s.gate.OrbitOverride = s.savedOverride;
                s.gate.RestorePoseOnDisable = s.savedRestore;
                s.gate.AutoOrbit = s.savedAutoOrbit;
                s.gate.SuppressTransportOrbit = s.savedSuppressTransport;
            }
            _orbitGates.Clear();
            if (_presentationPivot != null)
            {
                Destroy(_presentationPivot.gameObject);
                _presentationPivot = null;
            }

            Debug.Log($"[{nameof(ExperienceDirector)}] experience mode OFF (dev values restored).", this);
        }

        private void StopRoutine(ref Coroutine routine)
        {
            if (routine != null) { StopCoroutine(routine); routine = null; }
        }

        // Dev (rig-less) quality-of-life: the entrance recording is the stand-in
        // visitor, but Idle's cleanup and the visitor play-through stop/unload
        // the recorder — after which the stage (and presence) stays empty until
        // the operator presses 入場 again. Whenever the recorder is free, bring
        // the entrance playback back, looping so the take never runs out.
        private float _entranceRetryAt;

        private void MaintainDevEntrancePlayback()
        {
            if (!config.devLoopEntrancePlayback || sensorRecorder == null) return;
            if (_visitorPlaybackActive || HasLiveRenderers()) return;
            if (_fsm.State == ExperienceState.Fault
                || _fsm.State == ExperienceState.Processing) return;
            if (sensorRecorder.CurrentState != SensorRecorder.State.Idle) return;
            if (Time.realtimeSinceStartup < _entranceRetryAt) return;
            _entranceRetryAt = Time.realtimeSinceStartup + 3f; // don't thrash a failing Load
            sensorRecorder.loop = true;
            sensorRecorder.TogglePlay();
            if (sensorRecorder.CurrentState == SensorRecorder.State.Playing)
                Debug.Log($"[{nameof(ExperienceDirector)}] entrance playback restarted " +
                          "(devLoopEntrancePlayback).", this);
        }

        // ------------------------------------------------ dev state jumping ----

        /// <summary>
        /// Dev hotkeys / panel: jump the running show straight to a state.
        /// Idle restarts the run (頭出し). Forward jumps clear only the flags that
        /// would fast-forward past the target, and keep the run's artifacts
        /// (take root, capture snapshot, QR) so ResultShow/QrShow can be checked
        /// right after a Processing jump. Processing with no take yet borrows the
        /// canned dev take.
        /// </summary>
        public void DevJumpTo(ExperienceState target)
        {
            if (target == ExperienceState.Fault) return;
            if (!_active) Visible = true; // dev jump implies the mode — enter first
            if (!_active || _fsm == null) return; // mode failed to enter
            if (target == ExperienceState.Idle)
            {
                ResetRunState();
                _fsm.ForceTransition(ExperienceState.Idle, "dev jump (頭出し)");
                return;
            }
            switch (target)
            {
                case ExperienceState.Calibrate: _calibrationDone = false; break;
                case ExperienceState.TestMove1:
                case ExperienceState.TestMove2:
                    _testMoveDone = false;
                    break;
                case ExperienceState.Shoot: _recordingDone = false; break;
                case ExperienceState.Processing:
                    _processingDone = _processingFailed = false;
                    if (string.IsNullOrEmpty(_takeRoot))
                    {
                        _takeRoot = config.DevCannedTakeRoot;
                        _takeIsTapped = false;
                    }
                    break;
                case ExperienceState.ResultShow:
                    _exportDone = _exportFailed = false;
                    _resultShowDone = false;
                    // A direct jump has no play-through behind it — borrow the
                    // canned take so the replay presentation can be checked.
                    if (string.IsNullOrEmpty(_takeRoot)) _takeRoot = config.DevCannedTakeRoot;
                    break;
            }
            Debug.Log($"[{nameof(ExperienceDirector)}] dev jump → {target}.", this);
            _fsm.ForceTransition(target, "dev jump");
        }

        // Digit row (and keypad), in state order: 0=Idle(頭出し) 1=Consent
        // 2=Welcome 3=Calibrate 4=TestMove1 5=TestMove2 6=Shoot 7=Processing
        // 8=ResultShow 9=QrShow. TSDFDebugSession's per-camera views moved to
        // Q/W/E/R (all cams: A) to free the digits for this.
        private void HandleDevJumpHotkeys()
        {
            if (!Input.anyKeyDown) return;
            for (int i = 0; i <= (int)ExperienceState.QrShow; i++)
                if (Input.GetKeyDown(KeyCode.Alpha0 + i) || Input.GetKeyDown(KeyCode.Keypad0 + i))
                {
                    DevJumpTo((ExperienceState)i);
                    return;
                }
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
            _testMoveDone = false;
            _testMovePresenting = false;
            _resultShowDone = false;
            _recordingDone = false;
            _processingDone = _processingFailed = false;
            _takeRoot = null;
            _takeIsTapped = false;
            _snapshot = null;
            _exportDone = _exportFailed = false;
            _takeHasBodies = true;
            _qrUrl = null;
            _crowdShowing = false;
            _starHold?.Reset();
            // Release AND clear the frozen ribbons: the next visitor must draw their
            // own, not append to the previous one's curve.
            if (poseHistory != null) poseHistory.ReleaseHoldAndClear();
            // Belt-and-suspenders: a shoot-end dissolve that never reached its reveal
            // (Processing failure, dev jump) can leave _PcFadeCull at 1 / the decimate
            // pinned / frames frozen. Clear it here — this runs on Idle entry BEFORE
            // ApplyCloudVisibility shows the live cloud, so the stage can't come up black.
            AbortShootEndDissolve();
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
                if (to == ExperienceState.TestMove1) FinalizeProfileSamplingBestEffort();
                else CancelProfileSampling();
            }
            if (from is ExperienceState.TestMove1 or ExperienceState.TestMove2
                && _testMoveRoutine != null)
            {
                // Abnormal exit (walked away, fault, dev jump) mid-practice: the
                // routine did not run its own cleanup — stop whatever sub-phase
                // was live. The playback transport is torn down by the Idle side
                // effects / the next StartVisitorPlayback.
                StopRoutine(ref _testMoveRoutine);
                AbortVisitorRecording();
                _converter?.Abort();
                _testMovePresenting = false;
                AbortShootEndDissolve(); // stopped mid dissolve: no half-dissolved/frozen leak
                // Stopped mid wireframe replay: undo it (hides the mesh, restores the
                // integrator) so the next state isn't left drawing a wireframe / with
                // integration off. SetWireframeReplay no-ops when it wasn't armed.
                SetWireframeReplay(false);
            }
            if (from == ExperienceState.Shoot)
            {
                StopRoutine(ref _shootRoutine);
                // Shoot -> Processing is the normal path; the dissolve there completed
                // and must keep the black (cull=1) through Processing until the ResultShow
                // reveal. Any OTHER exit was interrupted mid-dissolve — reset it.
                if (to != ExperienceState.Processing)
                {
                    AbortVisitorRecording();
                    AbortShootEndDissolve();
                }
            }
            if (from == ExperienceState.Processing)
            {
                if (_processingRoutine != null)
                {
                    StopRoutine(ref _processingRoutine);
                    _converter?.Abort();
                    RestoreHistorySamples(); // the capture may have been mid-flight
                }
                // MUST run even when the routine already nulled itself on a failure path
                // (no take / playback / capture failure): every Processing exit that
                // isn't the ResultShow reveal has to clear the shoot-end dissolve, or
                // _PcFadeCull stays at 1 and the next visible state comes up black.
                if (to != ExperienceState.ResultShow) AbortShootEndDissolve();
            }
            if (from == ExperienceState.ResultShow)
            {
                StopRoutine(ref _resultShowRoutine);
                // Left mid wireframe replay (walked off, fault, dev jump before the
                // reveal): undo it so the mesh stops drawing and the integrator is
                // restored. No-ops on the normal ResultShow→QrShow exit, where phase 2
                // already turned the wireframe off. Also drop the deferred-text gate so
                // a later ResultShow repaint isn't stuck blank.
                SetWireframeReplay(false);
                _deferResultText = false;
                if (to == ExperienceState.Idle)
                    CancelPublish(); // fail path or skip; success path already completed
            }

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

            // The SAME director flag drives both TestMove states in sequence —
            // entering the second with the first's TestMoveDone still up would
            // skip it entirely (the FSM clears its own latch on entry, but it
            // re-latches from this input on the very next tick). ResultShowDone
            // gets the same treatment for symmetry (dev jumps land here without
            // a run reset).
            if (to is ExperienceState.TestMove1 or ExperienceState.TestMove2)
                _testMoveDone = false;
            if (to == ExperienceState.ResultShow)
                _resultShowDone = false;

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
                    // Transport unloaded — the last run's still-loaded take can
                    // go now (QrShow already deleted the rest).
                    DeleteRunTakes(includeLoaded: true);
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
                case ExperienceState.TestMove1:
                    if (!t.skipTestMoves)
                        _testMoveRoutine = StartCoroutine(TestMoveRoutine(1));
                    break;
                case ExperienceState.TestMove2:
                    if (!t.skipTestMoves)
                        _testMoveRoutine = StartCoroutine(TestMoveRoutine(2));
                    break;
                case ExperienceState.Shoot:
                    if (t.skipShoot)
                    {
                        _takeRoot = config.DevCannedTakeRoot; // pre-recorded dev take
                        _takeIsTapped = false;
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
                    _resultShowRoutine = StartCoroutine(ResultShowRoutine());
                    break;
                case ExperienceState.QrShow:
                    // The frozen line model keeps orbiting under the QR (a dev
                    // jump straight here turns the orbit on too).
                    SetOrbit(true);
                    // The QR is up = export/upload done — the recorded takes
                    // (practice + real) have served their purpose. Delete them
                    // now; the loaded one (on screen) follows on unload.
                    DeleteRunTakes(includeLoaded: false);
                    break; // QR cue handled by PlayStateEnterSe below
                case ExperienceState.Fault:
                    string fault = healthMonitor != null ? healthMonitor.FaultAlertText : "";
                    _ui.ShowAlert(string.IsNullOrEmpty(fault) ? "カメラが異常です" : fault);
                    // Try to bring the rig back by ourselves. Skipped when the operator
                    // forced the fault (debugForceFault) — there is no camera to fix —
                    // and on sessions that never had a live rig (_savedHadLiveRig):
                    // there is nothing to re-open, and on a rig-less dev machine
                    // StartLive would throw DllNotFoundException (no OrbbecSDK
                    // native library on macOS).
                    if (config.autoRecoverCameras && !debugForceFault && _savedHadLiveRig
                        && _recoveryRoutine == null)
                        _recoveryRoutine = StartCoroutine(RecoverCamerasRoutine());
                    break;
            }
            if (state != ExperienceState.Fault) _ui.ClearAlert();
            // Orbit + the rich presentation look are playback-phase affairs: the
            // TestMove routines switch them on for their play-through and
            // ResultShow's routine does the same (they stay on through QrShow).
            // Every other state entry — including a TestMove entry, whose
            // routine re-enables them only once its own playback starts —
            // forces them off, so any abnormal exit (walked away, fault, dev
            // jump) restores the stage framing and the live look.
            if (state is not (ExperienceState.ResultShow or ExperienceState.QrShow))
            {
                SetOrbit(false);
                SetPresentationLook(false);
            }
            ApplyCurvesVisibility(state);
            // Cloud BEFORE bones: hiding the cloud clears _cloudRevealed, and the
            // bones read that flag — a jump from a revealed state back into the
            // intro must land with the bones already back on.
            ApplyCloudVisibility(state);
            ApplyBonesVisibility(state);
            // The ribbons draw EVERYWHERE the cloud is content — the live phases
            // after calibration (point cloud + curves, 2026-07-23 decision) and
            // every playback phase. Hidden only during Processing (the progress
            // bar owns that screen); the bones-only intro shows nothing anyway
            // because ApplyCurvesVisibility gates the BUILD until the match.
            // Draw-only: production keeps running underneath for the
            // capture/export. Driven off the state so every exit path — fault,
            // visitor walked off, processing failure — lands right.
            // (The TSDF mesh is session-suppressed in EnterMode and never shows.)
            SetSculptureVisible(state is not ExperienceState.Processing);

            // ResultShow opens on a BLANK stage (behaviour B, 2026-07-24). Processing
            // froze the FULL sculpture — the built curve buffer AND the pose ring — and
            // leaving it on screen through the inter-state beat read as "the finished
            // lines are already drawn and the body just retraces them". Clear the stale
            // curves + ring and hide the cloud NOW so the beat is truly empty;
            // ResultShowRoutine then runs the wireframe replay and reveals the finished
            // model FROM ZERO. Guarded on a replayable take so the no-take edge still
            // shows the frozen piece rather than a black screen. _deferResultText holds
            // できたよ！ back until the replay finishes (the routine paints it then).
            _deferResultText = false;
            if (state == ExperienceState.ResultShow
                && sensorRecorder != null
                && !string.IsNullOrEmpty(_takeRoot) && Directory.Exists(_takeRoot))
            {
                if (motionCurves != null) { motionCurves.freeze = false; motionCurves.suppressDraw = true; }
                if (poseHistory != null) poseHistory.ReleaseHoldAndClear();
                SetCloudRenderersVisible(false);
                _deferResultText = true;
            }

            // The beat between screens (stateGapSeconds): blank now, then the new
            // screen and its cue land together after the gap. Fault must alert
            // instantly and Idle is blank by definition — both paint immediately.
            StopRoutine(ref _gapPaintRoutine);
            float gap = t.stateGapSeconds / Mathf.Max(0.01f, t.timeMultiplier);
            if (gap <= 0f || state == ExperienceState.Fault || state == ExperienceState.Idle)
            {
                if (state != ExperienceState.Fault)
                    PlayStateEnterSe(state); // one cue per screen, on entry only
                ShowStateMessage(state);
            }
            else
            {
                _ui.ClearAll(); // hold the blank; late canvases stay blank too
                _gapPaintRoutine = StartCoroutine(GapPaintRoutine(state, gap));
            }
        }

        // Deferred half of ApplyStateSideEffects — the screen + audio cue for
        // `state`, landing stateGapSeconds after the transition blanked the UI.
        private IEnumerator GapPaintRoutine(ExperienceState state, float delay)
        {
            yield return new WaitForSeconds(delay);
            _gapPaintRoutine = null;
            if (!_active || _fsm.State != state) yield break; // a newer transition owns the screen
            if (_crowdShowing) yield break; // crowd warning owns it; repaint on clear
            PlayStateEnterSe(state);
            ShowStateMessage(state);
        }

        // The audio cue for the message each state paints on entry (from
        // ApplyStateSideEffects, NOT the crowd-notice repaint), so a clip plays
        // exactly when its message appears. Idle is silent by design. Mid-state
        // message changes (TestMove1's second intro, さつえいちゅう！, pose
        // matched, failures, countdown/shutter) fire their own cues from the
        // routines, not here.
        private void PlayStateEnterSe(ExperienceState state)
        {
            switch (state)
            {
                case ExperienceState.Consent: PlaySe(config.consentSe); break;
                case ExperienceState.Welcome: PlaySe(config.welcomeSe); break;
                case ExperienceState.Calibrate: PlaySe(config.calibrateSe); break;
                case ExperienceState.TestMove1: PlaySe(config.testMove1IntroSe); break;
                case ExperienceState.TestMove2: PlaySe(config.testMove2IntroSe); break;
                case ExperienceState.Shoot: PlaySe(config.shootCueSe); break;
                case ExperienceState.Processing: PlaySe(config.processingSe); break;
                // Deferred through the wireframe replay — the routine plays it on reveal.
                case ExperienceState.ResultShow: if (!_deferResultText) PlaySe(config.resultSe); break;
                case ExperienceState.QrShow: PlaySe(config.qrShowSe); break;
            }
        }

        /// <summary>
        /// The ribbons stay OFF until the star pose is done. Idle..Calibrate are
        /// read-the-screen stages (notice, greeting, pose guide) and curves drawn
        /// from the approach walk are noise the visitor has no way to read; the
        /// sculpture should appear the moment it becomes theirs to move. From
        /// the TestMoves on they are the content, and Processing/ResultShow need them
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
            motionCurves.visible = state is ExperienceState.TestMove1
                                        or ExperienceState.TestMove2
                                        or ExperienceState.Shoot
                                        or ExperienceState.Processing
                                        or ExperienceState.ResultShow
                                        or ExperienceState.QrShow;
        }

        /// <summary>
        /// The intro (Consent through the pose guide) shows the visitor as a bare
        /// skeleton — bone lines only, no TSDF mesh, no clouds — so the privacy
        /// notice and このポーズをとってね are read against exactly the body data
        /// being captured. The bones stay on while the point cloud rises, then
        /// HAND OVER: the instant the reveal sweep completes (_cloudRevealed) the
        /// bones go out and the cloud alone is the content through the TestMoves and
        /// Shoot. Off from Processing on (progress bar, then the frozen sculpture
        /// own those screens) and in Idle/Fault. Re-evaluated on every state entry
        /// AND on reveal completion (CloudRevealRoutine); a backward jump or run
        /// reset hides the cloud, which clears _cloudRevealed and brings the bones
        /// back. EnterMode snapshots the operator's showBones; ExitMode restores it.
        /// </summary>
        private void ApplyBonesVisibility(ExperienceState state)
        {
            if (merger == null) return;
            merger.showBones = state is ExperienceState.Consent
                                     or ExperienceState.Welcome
                                     or ExperienceState.Calibrate
                                     or ExperienceState.TestMove1
                                     or ExperienceState.TestMove2
                                     or ExperienceState.Shoot
                            && !_cloudRevealed;
        }

        /// <summary>
        /// The visitor's raw point cloud is the visible content from the
        /// はかれたよ！ beat through Shoot. Hidden during the bones-only intro;
        /// when calibration finishes it rises out of the floor over
        /// cloudRevealSeconds (a global shader clip on PointCloudUnlit sweeping
        /// _PcRevealY from the feet up — see the shader note on why globals).
        /// The window-expiry and skipCalibrate paths land here too, via the
        /// TestMove1 entry. Hidden during Processing. Display-only on both
        /// halves: PointCloudView toggles MeshRenderers and the clip lives in the
        /// vertex shader, so BT, TSDF integration, curves and the capture always
        /// consume the full cloud.
        /// </summary>
        private void ApplyCloudVisibility(ExperienceState state)
        {
            // Clouds carry every phase that has stage content now: the live body
            // from the はかれたよ！ beat through the practice rounds and the shoot,
            // the take replays (TestMove playback / ResultShow) and the frozen
            // final frame under the QR. Idle too — with the TSDF mesh session-
            // hidden, the raw cloud is what a walk-in visitor sees of themselves.
            // Hidden only for the bones-only intro (Consent..Calibrate pre-match)
            // and Processing (progress bar owns the screen).
            bool show = state is ExperienceState.Idle
                             or ExperienceState.TestMove1
                             or ExperienceState.TestMove2
                             or ExperienceState.Shoot
                             or ExperienceState.ResultShow
                             or ExperienceState.QrShow
                     || (state == ExperienceState.Calibrate && _calibrationDone);
            if (show == _cloudsShown)
            {
                // No visibility edge, but a state change during playback still
                // needs the live MRs' show/hide refreshed (StartVisitorPlayback
                // hides them under the shared PointCloudView flag).
                if (show && !_visitorPlaybackActive && HasLiveRenderers())
                    sensorManager?.SetLiveVisualsVisible(true);
                return;
            }
            _cloudsShown = show;
            _cloudRevealed = false; // hand-back: bones return until the next sweep lands
            StopRoutine(ref _cloudRevealRoutine);
            Shader.SetGlobalFloat(PcRevealModeId, 0f);
            SetCloudRenderersVisible(show);
            if (show && !_visitorPlaybackActive && HasLiveRenderers())
                sensorManager?.SetLiveVisualsVisible(true);
            if (show && isActiveAndEnabled)
                _cloudRevealRoutine = StartCoroutine(CloudRevealRoutine());
        }

        private void SetCloudRenderersVisible(bool visible)
        {
            if (_pcView != null) _pcView.showPointClouds = visible;
            else sensorManager?.SetLiveVisualsVisible(visible);
        }

        // Sweep the reveal plane from the floor to overhead, then drop the clip
        // entirely (mode 0) so the fully-shown cloud pays nothing per vertex.
        // Survives the Calibrate→TestMove1 transition because ApplyCloudVisibility
        // only restarts it on a hidden→shown edge. Completion is the bones'
        // exit cue: the skeleton that carried the intro hands the stage to the
        // fully-risen cloud (an interrupted sweep never sets _cloudRevealed, so
        // the bones stay up on every abort path).
        private IEnumerator CloudRevealRoutine()
        {
            float dur = config.cloudRevealSeconds
                        / Mathf.Max(0.01f, config.timings.timeMultiplier);
            if (dur > 0f)
            {
                Shader.SetGlobalFloat(PcRevealModeId, 1f);
                for (float e = 0f; e < dur; e += Time.deltaTime)
                {
                    Shader.SetGlobalFloat(PcRevealYId,
                        config.floorY + config.cloudRevealHeight * (e / dur));
                    yield return null;
                }
                Shader.SetGlobalFloat(PcRevealModeId, 0f);
            }
            _cloudRevealed = true;
            _cloudRevealRoutine = null;
            if (_fsm != null) ApplyBonesVisibility(_fsm.State);
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
            // Ribbons only — the TSDF mesh is suppressed for the whole session
            // (EnterMode): the show's visible content is point cloud + lines.
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
                // The routines own the intro → countdown → playback sequences;
                // repaint restores the first intro (same contract as Shoot's cue).
                case ExperienceState.TestMove1: _ui.ShowMessage(config.testMove1IntroText); break;
                case ExperienceState.TestMove2: _ui.ShowMessage(config.testMove2IntroText); break;
                case ExperienceState.Shoot:
                    // the routine owns the cue → countdown → shooting sequence;
                    // repaint just restores the cue text
                    _ui.ShowMessage(config.shootCueText);
                    break;
                case ExperienceState.Processing:
                    _ui.ShowProgress(_converter?.Progress ?? 0f, config.processingText);
                    break;
                case ExperienceState.ResultShow:
                    // Through the wireframe replay show ぶんせきちゅう (any repaint —
                    // crowd notice clear, gap paint — keeps it) until the routine
                    // finishes the replay and reveals the finished model with できたよ！.
                    if (_deferResultText) { _ui.ShowMessage(config.analyzingText); break; }
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
                : StickFigureTexture.StarPose();

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
                    ApplyCurvesVisibility(_fsm.State); // curve BUILD starts on the match
                    ApplyCloudVisibility(_fsm.State);  // the cloud rises from the feet
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

        /// <summary>The rig-less path can still record: the recorder is playing
        /// (the entrance stand-in) and its stream can be tapped into a new take.</summary>
        private bool CanTapRecord =>
            sensorRecorder != null && !HasLiveRenderers() && sensorRecorder.IsPlaying;

        private void StartVisitorRecording()
        {
            _visitorRecordingTapped = false;
            if (sensorRecorder == null) return;
            bool tap = CanTapRecord;
            // dummyShoot without a tappable source: the old no-recording path
            // (canned take substitutes). With a playing source the take is REAL
            // — recorded off the playback stream, production-length.
            if (config.timings.dummyShoot && !tap) return;
            _savedRecFolderPath = sensorRecorder.folderPath;
            _savedRecAutoTimestamp = sensorRecorder.autoTimestampFolder;
            if (!string.IsNullOrEmpty(config.visitorRecordingRoot))
                sensorRecorder.folderPath = config.visitorRecordingRoot;
            sensorRecorder.autoTimestampFolder = true;
            if (tap)
            {
                _visitorRecordingActive = sensorRecorder.StartPlaybackTapRecording();
                _visitorRecordingTapped = _visitorRecordingActive;
            }
            else
            {
                sensorRecorder.StartRecording();
                _visitorRecordingActive = sensorRecorder.CurrentState == SensorRecorder.State.Recording;
            }
            if (!_visitorRecordingActive)
            {
                Debug.LogError($"[{nameof(ExperienceDirector)}] recording failed to start " +
                               "(no live renderers and no playback to tap?) — the take will be empty.", this);
                RestoreRecorderFolderConfig();
            }
        }

        /// <summary>Stop whichever recording mode is active and return the
        /// finished take root (null when nothing was recorded). Sets
        /// _takeIsTapped for the conversion skip.</summary>
        private string FinishVisitorRecording()
        {
            if (!_visitorRecordingActive || sensorRecorder == null) return null;
            string root;
            if (_visitorRecordingTapped)
            {
                sensorRecorder.StopPlaybackTapRecording();
                root = sensorRecorder.LastTapRecordingRoot;
                _takeIsTapped = true;
            }
            else
            {
                sensorRecorder.StopRecording();
                root = sensorRecorder.LastRecordingRoot;
                _takeIsTapped = false;
            }
            _visitorRecordingActive = false;
            _visitorRecordingTapped = false;
            RestoreRecorderFolderConfig();
            if (!string.IsNullOrEmpty(root) && !_runTakeRoots.Contains(root))
                _runTakeRoots.Add(root);
            return root;
        }

        /// <summary>Delete the takes recorded during this run. Only paths that
        /// FinishVisitorRecording produced are ever in the list — the canned
        /// dev take and the entrance recording can never be deleted here. The
        /// take still loaded in the transport (frozen on the QR screen) is
        /// skipped unless includeLoaded — its RCSV files may be open for lazy
        /// reads; it is deleted after the unload on the next run boundary.</summary>
        private void DeleteRunTakes(bool includeLoaded)
        {
            for (int i = _runTakeRoots.Count - 1; i >= 0; i--)
            {
                string root = _runTakeRoots[i];
                if (string.IsNullOrEmpty(root)) { _runTakeRoots.RemoveAt(i); continue; }
                bool loaded = _visitorPlaybackActive && root == _takeRoot;
                if (!includeLoaded && loaded) continue;
                try
                {
                    if (Directory.Exists(root)) Directory.Delete(root, true);
                    _runTakeRoots.RemoveAt(i);
                    Debug.Log($"[{nameof(ExperienceDirector)}] deleted visitor take {root}.", this);
                }
                catch (Exception e)
                {
                    // Keep it in the list — the next boundary retries.
                    Debug.LogWarning($"[{nameof(ExperienceDirector)}] could not delete take " +
                                     $"{root}: {e.Message} — will retry.", this);
                }
            }
        }

        // Cue text → countdown → zero → captureSeconds of the actual take → stop.
        //
        // Recording covers the CAPTURE WINDOW ONLY (plus recordPreRollSeconds).
        // It used to start with the countdown, to warm the fusion up before the
        // second that counts — but the live RTMPose fusion runs continuously
        // through the practice rounds, so it is already warm when Shoot begins. Recording
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
            PlaySe(config.shootingSe);
            _ui.ShowMessage(config.shootingText);
            while (_active && _fsm.State == ExperienceState.Shoot && _fsm.TimeInState < shootEnd)
                yield return null;
            if (!_active || _fsm.State != ExperienceState.Shoot) yield break;

            if (_visitorRecordingActive && sensorRecorder != null)
            {
                _takeRoot = FinishVisitorRecording();
            }
            else if (t.dummyShoot)
            {
                _takeRoot = config.DevCannedTakeRoot; // canned take stands in for the shot
                _takeIsTapped = false;
            }
            PlaySe(config.recordEndSe);

            // Erase the shot model with the same freeze → hold → dissolve as the
            // practice rounds, so Processing's progress bar comes up on black instead
            // of the model blinking out. Recording is already finished above, so the
            // freeze only holds the live/playback DISPLAY.
            _ui.ClearAll();
            yield return DissolveShootEndToBlack();
            if (!_active || _fsm.State != ExperienceState.Shoot) yield break;

            _recordingDone = true;
            _shootRoutine = null;
        }

        /// <summary>Visitor walked away / fault mid-recording: stop and discard.
        /// The partial take is visitor data like any finished one — it joins
        /// _runTakeRoots so the next boundary (Idle / QR / mode exit) deletes
        /// it from disk.</summary>
        private void AbortVisitorRecording()
        {
            if (!_visitorRecordingActive) return;
            _visitorRecordingActive = false;
            if (sensorRecorder != null)
            {
                string root = null;
                if (_visitorRecordingTapped)
                {
                    sensorRecorder.StopPlaybackTapRecording();
                    root = sensorRecorder.LastTapRecordingRoot;
                }
                else if (sensorRecorder.CurrentState == SensorRecorder.State.Recording)
                {
                    sensorRecorder.StopRecording();
                    root = sensorRecorder.LastRecordingRoot;
                }
                if (!string.IsNullOrEmpty(root) && !_runTakeRoots.Contains(root))
                    _runTakeRoots.Add(root);
            }
            _visitorRecordingTapped = false;
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
            // Sit out the inter-state beat — this routine paints the progress bar
            // every frame, which would otherwise pop in during the blank that the
            // deferred state paint (GapPaintRoutine) owns.
            float beat = t.stateGapSeconds / Mathf.Max(0.01f, t.timeMultiplier);
            if (beat > 0f) yield return new WaitForSeconds(beat);
            if (string.IsNullOrEmpty(_takeRoot) || !Directory.Exists(_takeRoot))
            {
                Debug.LogError($"[{nameof(ExperienceDirector)}] no take to process ('{_takeRoot}').", this);
                _processingFailed = true;
                ShowExportFailed();
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

            // ---- stage 1: v11s conversion (skippable, shared with TestMove) ----
            bool cannedTake = t.skipShoot || t.dummyShoot;
            yield return ConvertTakeRoutine(_takeRoot, 0f, 0.8f);

            // ---- stage 2: single play-through of the take (loop off) ----
            // Apply the dense presentation look (seedCount 60000, presentation
            // decimate + tailAlpha) BEFORE the play-through so the seeds rebuild
            // and trace the motion at full density during it — the capture below
            // then exports exactly what ResultShow reveals. Without this the
            // export froze the scene's base seedCount (500) → sparse curves that
            // never matched the on-screen ribbons. Stays on through ResultShow
            // (which re-asserts it); restored on Idle / mode exit.
            SetPresentationLook(true);
            StartVisitorPlayback(_takeRoot);
            if (!_visitorPlaybackActive || sensorRecorder == null || !sensorRecorder.IsPlaying)
            {
                Debug.LogError($"[{nameof(ExperienceDirector)}] take play-through failed to start.", this);
                _processingFailed = true;
                ShowExportFailed();
                _processingRoutine = null;
                yield break;
            }
            double duration = sensorRecorder.PlaybackDurationSeconds;
            // A canned dev take is a full-length recording (a real visitor take
            // barely exceeds the capture window) — jump to its tail so the
            // play-through costs production-like seconds, not the whole take.
            if (cannedTake && config.devCannedTakeTailSeconds > 0f
                && duration > config.devCannedTakeTailSeconds)
            {
                if (sensorRecorder.SeekToPlayheadSeconds(duration - config.devCannedTakeTailSeconds))
                    sensorRecorder.ResumePlayback();
                Debug.Log($"[{nameof(ExperienceDirector)}] canned take: skipping to the last " +
                          $"{config.devCannedTakeTailSeconds:0}s of {duration:0}s.", this);
            }
            double remaining = duration - sensorRecorder.CurrentPlayheadSeconds;
            float playDeadline = Time.realtimeSinceStartup + (float)remaining + 10f;
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
                ShowExportFailed();
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

        // ---------------- orbit cameras (playback/QR phases) ----------------

        // ---- presentation look (できたよ！ replay → QrShow) ----
        // The live phases keep the scene's own light values (few seeds, heavy
        // decimation — the cloud is the star); the replay presentation switches
        // to the rich look from config and this restores the scene values on
        // the way back. historySamples is NOT part of this — the ring window
        // keeps its own switching (30-sample growth per loop).
        private bool _presentationLook;
        private int _savedSeedCount;
        private float _savedTailAlpha;
        private float _savedDecimate;

        private void SetPresentationLook(bool on)
        {
            if (_presentationLook == on) return;
            _presentationLook = on;
            if (on)
            {
                if (motionCurves != null)
                {
                    _savedSeedCount = motionCurves.seedCount;
                    _savedTailAlpha = motionCurves.tailAlpha;
                    motionCurves.seedCount = config.presentationSeedCount;
                    motionCurves.tailAlpha = config.presentationTailAlpha;
                }
                if (decimater != null)
                {
                    _savedDecimate = decimater.reductionPercent;
                    decimater.reductionPercent = config.presentationDecimatePercent;
                }
            }
            else
            {
                if (motionCurves != null)
                {
                    motionCurves.seedCount = _savedSeedCount;
                    motionCurves.tailAlpha = _savedTailAlpha;
                }
                if (decimater != null) decimater.reductionPercent = _savedDecimate;
            }
        }

        // Flip every gate's override together. The gates themselves restore the
        // camera pose when their orbit turns off (RestorePoseOnDisable, forced
        // on for the session in EnterMode).
        private void SetOrbit(bool on, bool easeIn = false)
        {
            if (_orbitOn == on) return;
            _orbitOn = on;
            StopRoutine(ref _orbitEaseRoutine);
            if (on)
            {
                // Seed the anchor at the person (or stage centre) BEFORE the
                // controllers switch to it, so the first orbit frame already
                // looks at the body and Update keeps it tracking.
                EnsurePresentationPivot();
                _pivotSeeded = false;
                UpdatePresentationPivot();
                // ResultShow eases the rotation IN from the replay-end framing: the
                // controllers switch on at 0°/s (camera holds where the replay left
                // it) and the routine ramps the yaw up. The routine owns the gates
                // while it runs, so return before the immediate apply below.
                if (easeIn && isActiveAndEnabled && config.presentationOrbitEaseInSeconds > 0f)
                {
                    _orbitEaseRoutine = StartCoroutine(EaseInOrbitRoutine());
                    return;
                }
            }
            ApplyOrbitToGates(on, on ? config.presentationOrbitYawSpeedDeg : 0f);
        }

        // Push the current orbit state + yaw speed to every gate. Repeated calls are
        // cheap: SetPresentationOrbit saves the dev values once (first non-null pivot)
        // and just updates the speed after, so the ease-in can call it per frame.
        private void ApplyOrbitToGates(bool on, float yawSpeedDeg)
        {
            foreach (var s in _orbitGates)
            {
                if (s.gate is not MonoBehaviour mb || mb == null) continue;
                s.gate.SetPresentationOrbit(on ? _presentationPivot : null,
                                            yawSpeedDeg,
                                            config.presentationOrbitBobMeters);
                s.gate.OrbitOverride = on;
            }
        }

        // Enable the orbit at 0°/s (the camera holds the replay-end framing), then
        // ramp the yaw speed 0→full over presentationOrbitEaseInSeconds (SmoothStep)
        // so the rotation eases in instead of snapping to speed.
        private IEnumerator EaseInOrbitRoutine()
        {
            float dur = Mathf.Max(0.01f, config.presentationOrbitEaseInSeconds);
            float target = config.presentationOrbitYawSpeedDeg;
            ApplyOrbitToGates(true, 0f);
            for (float e = 0f; e < dur; e += Time.deltaTime)
            {
                ApplyOrbitToGates(true, target * Mathf.SmoothStep(0f, 1f, e / dur));
                yield return null;
            }
            ApplyOrbitToGates(true, target);
            _orbitEaseRoutine = null;
        }

        // Shoot-end → できたよ！transition: freeze whatever the visitor was watching
        // (live cloud OR playback, plus the ribbons), hold it still for a beat, then
        // dissolve it away to black (global dither cull 0→1 across the cloud, shadow
        // and ribbon shaders). Leaves the stage black + hidden and the cull reset to 0
        // so the replay model later reveals at full. The caller owns the ring/curve
        // state for the growing replay that follows.
        private IEnumerator DissolveShootEndToBlack()
        {
            // Freeze the frame: HoldFrames pauses playback or freezes the live intake;
            // the ribbons hold their built buffer. Pin the decimate seed so the point
            // cloud's per-frame RANDOM drop pattern stops varying (same density, no
            // blink) — otherwise the kept points flicker/"revive" on the still image.
            sensorRecorder?.HoldFrames();
            PointCloud.PointCloudDecimater.DecimFramePin = Time.frameCount;
            if (motionCurves != null) motionCurves.freeze = true;
            if (poseHistory != null) poseHistory.hold = true;

            float mult = Mathf.Max(0.01f, config.timings.timeMultiplier);
            float hold = config.shootEndFreezeHoldSeconds / mult;
            if (hold > 0f) yield return new WaitForSeconds(hold);

#if UNITY_EDITOR
            // Freeze the Editor right at the dissolve start so it can be stepped frame
            // by frame (⏭). Each step advances one cull increment of the ramp below.
            if (config.debugPauseAtShootDissolve) UnityEditor.EditorApplication.isPaused = true;
#endif

            // Frame-based ramp (one SmoothStep increment PER FRAME), not time-based:
            // while frame-stepping the Editor (⏭) Time.deltaTime collapses to ~0, so a
            // time-driven ramp barely moves per step and looks stuck. Counting frames
            // makes every ⏭ advance exactly one visible cull step. ~30 fps target, so
            // normal playback still takes ≈ shootEndDissolveSeconds.
            float dur = Mathf.Max(0.01f, config.shootEndDissolveSeconds / mult);
            int totalFrames = Mathf.Max(1, Mathf.RoundToInt(dur * 30f));
            for (int f = 1; f <= totalFrames; f++)
            {
                Shader.SetGlobalFloat(PcFadeCullId, Mathf.SmoothStep(0f, 1f, (float)f / totalFrames));
                yield return null;
            }
            Shader.SetGlobalFloat(PcFadeCullId, 1f); // fully dissolved

            // Gone: hide the renderers + ribbons, un-freeze / release the hold and
            // resume the transport so DOWNSTREAM rebuilds cleanly (the Processing
            // capture needs the curves to rebuild, the replay reveal a fresh ring).
            // CRUCIAL: leave the cull at 1 (still fully dissolved = black). Resetting
            // it to 0 here flashed the FULL cloud for one frame, because ResumeFrames
            // re-shows the playback renderers and cull 0 means "no dither" = full
            // density. cull stays 1 (black) through the beat; each reveal site sets it
            // back to 0 the instant it shows the model. (The mode-exit / abort cleanup
            // also resets it, so it can never leak out.)
            SetCloudRenderersVisible(false);
            if (motionCurves != null) { motionCurves.suppressDraw = true; motionCurves.freeze = false; }
            if (poseHistory != null) poseHistory.hold = false;
            sensorRecorder?.ResumeFrames();
            PointCloud.PointCloudDecimater.DecimFramePin = -1; // back to per-frame
        }

        // Reset every transient the shoot-end dissolve leaves behind. The dissolve
        // deliberately holds _PcFadeCull at 1 (black) until a reveal site clears it, and
        // freezes the transport + pins the decimate seed + holds the curve/pose ring
        // while it runs — so any exit that ISN'T a normal reveal (walked away, fault,
        // dev jump, or a Processing failure that returns to Idle) must run this or the
        // next state inherits a black stage / frozen live view / pinned decimate.
        private void AbortShootEndDissolve()
        {
            Shader.SetGlobalFloat(PcFadeCullId, 0f);
            PointCloud.PointCloudDecimater.DecimFramePin = -1;
            if (motionCurves != null) motionCurves.freeze = false;
            if (poseHistory != null) poseHistory.hold = false;
            sensorRecorder?.ResumeFrames();
        }

        // TSDF wireframe replay (ResultShow + practice, 2026-07-24). The recorded
        // take replays as an edge net that RE-FORMS with the motion — the integrator
        // follows the playback (fresh batch, gate reopened) so the mesh tracks the
        // body, drawn wireframe with the point cloud and ribbons hidden. The ribbons
        // keep BUILDING underneath (suppressDraw hides only the draw) so the one-second
        // window is full by the time the solid model is revealed after the replay.
        // on=false re-freezes the sculpture and hides the mesh, leaving a black stage
        // for the できたよ！ + finished-model reveal.
        private bool _wireframeReplay;
        private void SetWireframeReplay(bool on)
        {
            if (_wireframeReplay == on) return;
            _wireframeReplay = on;
            if (on)
            {
                SetCloudRenderersVisible(false);
                if (motionCurves != null) { motionCurves.suppressDraw = true; motionCurves.freeze = false; }
                if (tsdfView != null) { tsdfView.wireframe = true; tsdfView.suppressDraw = false; }
                // Fold the replay so the net forms with the motion. BeginFreshBatch
                // clears the write buffer so it re-forms from empty instead of holding
                // the frozen final sculpture. (_sculptureFrozen stays set — the gate is
                // re-closed on hand-off below and the exit restore still reopens it.)
                if (tsdfIntegrator != null)
                {
                    tsdfIntegrator.integrationEnabled = true;
                    tsdfIntegrator.BeginFreshBatch();
                }
            }
            else
            {
                if (tsdfView != null) { tsdfView.wireframe = false; tsdfView.suppressDraw = true; }
                // Restore the integrator to whatever the freeze invariant dictates,
                // NOT unconditionally off: a frozen sculpture (Processing ran →
                // ResultShow) stays gated so the finished model holds; otherwise
                // (practice, or a dev jump straight into ResultShow with no Processing)
                // go back to live-follow so integration is never stranded off with
                // _sculptureFrozen == false (UnfreezeSculpture would no-op).
                if (tsdfIntegrator != null)
                {
                    if (_sculptureFrozen) tsdfIntegrator.integrationEnabled = false;
                    else ResumeIntegratorLiveFollow();
                }
            }
        }

        private void EnsurePresentationPivot()
        {
            if (_presentationPivot != null) return;
            var go = new GameObject("_PresentationPivot");
            go.transform.SetParent(transform, false);
            _presentationPivot = go.transform;
        }

        // Track the chest↔waist midpoint of whoever is on stage (live body or
        // the replay ghost — TryGetLiveSkeleton reads the merged output either
        // way). Smoothed so BT jitter doesn't shake the whole camera; when the
        // skeleton drops (frozen QR frame, tracking gap) the anchor simply
        // stays where it was.
        private void UpdatePresentationPivot()
        {
            if (_presentationPivot == null) return;
            const int Pelvis = 0, SpineChest = 2; // k4abt joint ids
            if (TryGetLiveSkeleton(out var joints, out var valid)
                && joints != null && joints.Length > SpineChest
                && valid[Pelvis] && valid[SpineChest])
            {
                Vector3 target = (joints[Pelvis] + joints[SpineChest]) * 0.5f;
                _presentationPivot.position = _pivotSeeded
                    ? Vector3.Lerp(_presentationPivot.position, target, Time.deltaTime * 5f)
                    : target;
                _pivotSeeded = true;
            }
            else if (!_pivotSeeded)
            {
                Vector3 fallback = boundingVolume != null
                    ? boundingVolume.transform.position : Vector3.zero;
                fallback.y = config.floorY + 1.0f;
                _presentationPivot.position = fallback;
            }
        }

        // -------------- shared v11s conversion (Processing + practice) --------------

        // Runs the FusedTakeConverter on takeRoot with the per-visitor profile,
        // mapping its progress onto [progressFrom..progressTo] of the progress
        // bar. Honours the canned-take guard and the processing timeout; every
        // failure path just returns — the caller plays the take's own bodies.
        //
        // Canned-take protection: the conversion rewrites bodies_main IN PLACE.
        // A dev E2E run must never mutate the canonical recording unless the
        // operator explicitly opted in with a disposable copy.
        private IEnumerator ConvertTakeRoutine(string takeRoot, float progressFrom, float progressTo)
        {
            var t = config.timings;
            if (_takeIsTapped)
            {
                Debug.Log($"[{nameof(ExperienceDirector)}] playback-sourced (tapped) take — its " +
                          "bodies passed through from the already-converted source; skipping " +
                          "v11s conversion.", this);
                yield break;
            }
            bool cannedTake = takeRoot == config.DevCannedTakeRoot;
            bool convert = !t.skipProcessing
                && !(cannedTake && !config.allowCannedTakeConversion);
            if (cannedTake && !config.allowCannedTakeConversion && !t.skipProcessing)
                Debug.Log($"[{nameof(ExperienceDirector)}] canned take — skipping v11s conversion " +
                          "(allowCannedTakeConversion is off); playing existing bodies_main.", this);
            if (!convert) yield break;

            _converter = new FusedTakeConverter();
            bool started = _converter.Start(takeRoot, new FusedTakeConverter.Options
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
                yield break;
            }

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
                _ui.ShowProgress(progressFrom + (progressTo - progressFrom) * _converter.Progress,
                                 config.processingText);
                yield return null;
            }
            if (_converter.Status != FusedTakeConverter.ConvertStatus.Done)
                Debug.LogWarning($"[{nameof(ExperienceDirector)}] v11s conversion ended " +
                                 $"{_converter.Status} ({_converter.Error}) — playing the take's " +
                                 "own bodies.", this);
        }

        // ---------------- looped playback (practice + result presentation) ----------------

        // Plays the loaded visitor take config.playbackLoops times and leaves
        // the transport STOPPED on the take's final frame: on entering the last
        // loop the recorder's loop flag is dropped, so the take's own end-stop
        // freezes the final frame + the completed ribbons on screen (a wrap
        // would flush the pose ring and land on frame 0). startAtSeconds > 0
        // replays only the take's tail (canned dev takes) — re-sought after
        // every wrap, deferred to the routine so the seek never re-enters the
        // recorder from inside its own loop event.
        private IEnumerator RunPlaybackLoops(ExperienceState owner, double startAtSeconds = 0)
        {
            if (sensorRecorder == null) yield break;
            int target = Mathf.Max(1, config.playbackLoops);
            int loops = 0;
            bool seekPending = startAtSeconds > 0;
            sensorRecorder.loop = target > 1;
            Action onLoop = () =>
            {
                loops++;
                if (loops >= target - 1) sensorRecorder.loop = false;
                if (startAtSeconds > 0) seekPending = true;
            };
            sensorRecorder.OnPlaybackLooped += onLoop;
            double duration = Math.Max(0.1, sensorRecorder.PlaybackDurationSeconds);
            float rate = Mathf.Max(0.05f, sensorRecorder.playbackRate);
            float deadline = Time.realtimeSinceStartup + (float)(duration * target / rate) + 15f;
            while (_active && _fsm != null && _fsm.State == owner
                   && sensorRecorder.IsPlaying
                   && Time.realtimeSinceStartup < deadline)
            {
                if (seekPending)
                {
                    seekPending = false;
                    if (sensorRecorder.SeekToPlayheadSeconds(startAtSeconds))
                        sensorRecorder.ResumePlayback();
                }
                yield return null;
            }
            sensorRecorder.OnPlaybackLooped -= onLoop;
            // Deadline expiry / owner-state exit with the transport still
            // rolling: freeze where we are so nothing advances under whatever
            // owns the screen next. Normal completion is already stopped.
            if (sensorRecorder.IsPlaying) sensorRecorder.PausePlayback();
        }

        // ---------------- TestMove (practice mini-cycle) ----------------

        // The whole practice round for TestMove1 (round=1) / TestMove2 (round=2):
        // intro message(s) → clear → countdown
        // (recording starts recordPreRollSeconds before zero) → ONE second
        // recorded → v11s conversion (progress bar, same pipeline as the real
        // take) → できたよ！ + playbackLoops looped play-throughs with the
        // ribbons + orbit → back to live. No capture, no export, no upload.
        // Ends by latching TestMoveDone; every failure path still gets there so
        // a practice hiccup never dead-ends the visitor.
        private IEnumerator TestMoveRoutine(int round)
        {
            var t = config.timings;
            var state = round == 1 ? ExperienceState.TestMove1 : ExperienceState.TestMove2;

            // Timeline on the state clock — TimeInState freezes while the
            // visitor is absent, so all of this measures real attendance.
            float introEnd = round == 1 ? config.testIntroSeconds * 2f : config.testIntroSeconds;
            float zeroAt = introEnd + config.countdownSeconds;
            float shootEnd = zeroAt + config.captureStartOffsetSeconds + config.captureSeconds;
            float recStart = Mathf.Max(introEnd, zeroAt - config.recordPreRollSeconds);
            bool canned = t.skipShoot || t.dummyShoot;

            // -- intro: the state paint showed the first message; TestMove1
            //    sequences its second one here --
            bool secondShown = false;
            while (Active() && _fsm.TimeInState < introEnd)
            {
                if (round == 1 && !secondShown && _fsm.TimeInState >= config.testIntroSeconds)
                {
                    PlaySe(config.testMove1Intro2Se);
                    _ui.ShowMessage(config.testMove1Intro2Text);
                    secondShown = true;
                }
                yield return null;
            }
            if (!Active()) yield break;
            _ui.ClearAll(); // 消してから — the countdown lands on a cleared screen

            // -- countdown (+ recording pre-roll) --
            int shown = -1;
            bool recStarted = false;
            while (Active() && _fsm.TimeInState < zeroAt)
            {
                if (!t.skipShoot && !recStarted && _fsm.TimeInState >= recStart)
                {
                    StartVisitorRecording(); // self-gates: dummyShoot without a tappable source is a no-op
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
            if (!Active()) yield break;
            if (!t.skipShoot && !recStarted) StartVisitorRecording();

            PlaySe(config.recordEndSe); // shutter at zero
            PlaySe(config.shootingSe);
            _ui.ShowMessage(config.shootingText);
            while (Active() && _fsm.TimeInState < shootEnd) yield return null;
            if (!Active()) yield break;

            // From here the visitor's part is done — the presentation stretch
            // (save/convert/load/replay) must not be reset by a presence flicker:
            // the ghost owns the merge, so occupancy/live-fusion are the only
            // presence signals and they blink across the conversion and load
            // gaps. try/finally so the flag can never outlive the stretch, even
            // if the routine dies on an exception or is stopped mid-yield
            // (StopCoroutine disposes the iterator, which runs the finally).
            _testMovePresenting = true;
            try
            {
                string takeRoot = null;
                if (_visitorRecordingActive && sensorRecorder != null)
                {
                    takeRoot = FinishVisitorRecording();
                }
                if (string.IsNullOrEmpty(takeRoot) && canned)
                {
                    // no recording ran (skipShoot, or dummyShoot without a
                    // tappable source) — the canned take stands in
                    takeRoot = config.DevCannedTakeRoot;
                    _takeIsTapped = false;
                }
                PlaySe(config.recordEndSe);

                // -- conversion (same pipeline as the real take; no capture) --
                bool haveTake = !string.IsNullOrEmpty(takeRoot) && Directory.Exists(takeRoot);
                if (haveTake)
                {
                    yield return ConvertTakeRoutine(takeRoot, 0f, 1f);
                    if (!Active()) yield break;
                }

                // -- shoot-end dissolve → black → TSDF wireframe replay → できたよ！ + model --
                //    Freeze the frozen model, hold, dissolve it away; then (order changed
                //    2026-07-24, same as ResultShow) the take replays FIRST as a TSDF
                //    wireframe that re-forms with the motion — no できたよ！, no curves,
                //    no cloud — and only when it finishes do we paint できたよ！ and reveal
                //    the finished curve model (history-30 ribbons that built underneath).
                bool played = false;
                float beat = config.resultRevealDelaySeconds
                             / Mathf.Max(0.01f, config.timings.timeMultiplier);
                if (haveTake)
                {
                    // 撮影中 off so the frozen model stands alone, then freeze → hold →
                    // dissolve it to black (ends with the cloud + ribbons hidden).
                    _ui.ClearAll();
                    yield return DissolveShootEndToBlack();
                    if (!Active()) yield break;

                    // Prep the ring for the growing replay: widen to the one-second
                    // window and start it EMPTY (the lines build as the take plays,
                    // flushed on every loop wrap). Restored to the dev value on hand-back.
                    if (poseHistory != null)
                    {
                        poseHistory.historySamples =
                            Mathf.Clamp(Mathf.RoundToInt(config.captureSeconds * 30f), 2, 32);
                        poseHistory.ReleaseHoldAndClear();
                    }

                    // Wireframe replay: hide cloud + ribbons, show the TSDF as an edge
                    // net that re-forms with the motion (integrator follows the replay).
                    SetWireframeReplay(true);
                    _ui.ShowMessage(config.analyzingText); // ぶんせきちゅう over the wireframe
                    StartVisitorPlayback(takeRoot, loop: true,
                                         rate: config.presentationPlaybackRate);
                    played = _visitorPlaybackActive && sensorRecorder != null && sensorRecorder.IsPlaying;
                }
                if (played)
                {
                    // Camera stays STILL through the replay. Practice hands straight back
                    // to live afterwards, so there is no post-replay orbit ease-in here
                    // (that belongs to ResultShow's held QR frame).
                    SetPresentationLook(true); // grades the ribbons that build for the reveal
                    // Canned dev take: replay only its tail — a full-length entrance
                    // recording would otherwise loop for minutes. (A recorded or
                    // tapped take is capture-window-sized and never triggers this.)
                    double startAt = 0;
                    double duration = sensorRecorder.PlaybackDurationSeconds;
                    if (takeRoot == config.DevCannedTakeRoot && config.devCannedTakeTailSeconds > 0f
                        && duration > config.devCannedTakeTailSeconds)
                        startAt = duration - config.devCannedTakeTailSeconds;
                    yield return RunPlaybackLoops(state, startAt);
                    if (!Active()) yield break;

                    // Replay done: hide the wireframe (re-freezes the sculpture), clear
                    // ぶんせきちゅう, hold a pure-black beat, then paint できたよ！ and
                    // reveal the finished curve model.
                    SetWireframeReplay(false);
                    _ui.ClearAll(); // black gap between ぶんせきちゅう and できたよ！
                    if (beat > 0f) yield return new WaitForSeconds(beat);
                    if (!Active()) yield break;
                    Shader.SetGlobalFloat(PcFadeCullId, 0f);
                    SetCloudRenderersVisible(true);
                    if (motionCurves != null) { motionCurves.freeze = true; motionCurves.suppressDraw = false; }
                    if (poseHistory != null) poseHistory.hold = true;
                    PlaySe(config.resultSe);
                    _ui.ShowMessage(config.resultText);
                    // Hold the finished できたよ！ model on screen before handing back.
                    yield return new WaitForSeconds(Mathf.Max(1f, beat * 3f));
                    if (!Active()) yield break;

                    // hand the stage back to the live body for the next round
                    // (the ribbons keep drawing — point cloud + curves is the
                    // live look now too, just with the scene's lighter values)
                    SetOrbit(false);
                    SetPresentationLook(false);
                    StopVisitorPlayback();
                    ApplyBodySource(BodySource.Live);
                    if (HasLiveRenderers())
                    {
                        sensorManager?.SetLiveSuppressedAsSource(false);
                        sensorManager?.SetLiveVisualsVisible(true);
                    }
                    RestoreHistorySamples(); // back to the dev ring size
                    if (poseHistory != null) poseHistory.ReleaseHoldAndClear();
                    // The finished-model reveal froze the curves + closed the integrator
                    // gate (via SetWireframeReplay). Practice folds live frames again, so
                    // undo both for the next round.
                    if (motionCurves != null) motionCurves.freeze = false;
                    ResumeIntegratorLiveFollow();
                }
                else
                {
                    Debug.LogWarning($"[{nameof(ExperienceDirector)}] practice round {round}: nothing " +
                                     $"to play back ('{takeRoot}') — moving on.", this);
                    // The wireframe replay was armed for a playback that never started —
                    // undo it (re-shows the cloud, reopens the integrator) and put the
                    // live round's window back (mirrors the success path).
                    if (haveTake)
                    {
                        SetWireframeReplay(false);
                        ResumeIntegratorLiveFollow();
                        Shader.SetGlobalFloat(PcFadeCullId, 0f);
                        SetCloudRenderersVisible(true);
                        if (motionCurves != null) { motionCurves.freeze = false; motionCurves.suppressDraw = false; }
                        if (_visitorPlaybackActive) StopVisitorPlayback();
                        RestoreHistorySamples();
                        if (poseHistory != null) poseHistory.ReleaseHoldAndClear();
                    }
                }
            }
            finally
            {
                _testMovePresenting = false;
            }

            _testMoveDone = true;
            _testMoveRoutine = null;

            bool Active() => _active && _fsm != null && _fsm.State == state;
        }

        // ---------------- ResultShow (looped replay + freeze) ----------------

        // Replays the finished take playbackLoops times with the ribbons and
        // the orbit cameras while ExportAndPublish runs in parallel, then
        // freezes on the take's final frame (that frame + the completed
        // ribbons ARE the QR screen's centre piece). Ends by latching
        // ResultShowDone — the FSM waits for it alongside ExportDone.
        private IEnumerator ResultShowRoutine()
        {
            // Sit out the inter-state beat (parity with the deferred paint).
            float beat = config.timings.stateGapSeconds
                         / Mathf.Max(0.01f, config.timings.timeMultiplier);
            if (beat > 0f) yield return new WaitForSeconds(beat);
            if (!_active || _fsm.State != ExperienceState.ResultShow)
            { _resultShowRoutine = null; yield break; }

            bool replayable = sensorRecorder != null
                && !string.IsNullOrEmpty(_takeRoot) && Directory.Exists(_takeRoot);
            if (!replayable)
            {
                _resultShowDone = true; // nothing to present — don't gate the QR
                _resultShowRoutine = null;
                yield break;
            }

            float revealDelay = config.resultRevealDelaySeconds
                                / Mathf.Max(0.01f, config.timings.timeMultiplier);

            // ---- phase 1: wireframe replay (no できたよ！, no curves, no cloud) ----
            // Order changed 2026-07-24: the recorded take replays FIRST as a TSDF
            // wireframe that re-forms with the motion (SetWireframeReplay drops the
            // cloud + ribbons and reopens the integrator). Only when it finishes do
            // we paint できたよ！ and reveal the finished curve model. The stage was
            // blanked at entry (cloud hidden, ribbons suppressed, ring cleared).
            SetWireframeReplay(true);
            _ui.ShowMessage(config.analyzingText); // ぶんせきちゅう over the wireframe
            if (poseHistory != null) poseHistory.ReleaseHoldAndClear(); // grow the ring from empty

            // Restart the take from the top, looping. Processing left the transport
            // loaded and stopped at the final frame; a dev jump straight here starts
            // from nothing — StartVisitorPlayback covers both (Load is idempotent).
            StartVisitorPlayback(_takeRoot, loop: true,
                                 rate: config.presentationPlaybackRate);
            if (!_visitorPlaybackActive || !sensorRecorder.IsPlaying)
            {
                Debug.LogWarning($"[{nameof(ExperienceDirector)}] result replay failed to start — " +
                                 "revealing the frozen model instead.", this);
                SetWireframeReplay(false);
                RevealFinishedResultModel();
                if (poseHistory != null) poseHistory.hold = true;
                if (motionCurves != null) motionCurves.freeze = true;
                // Paint できたよ！ now — the replay that would have painted it never ran,
                // and entry deferred the gap paint, so without this the screen stays blank.
                _deferResultText = false;
                PlayStateEnterSe(ExperienceState.ResultShow);
                ShowStateMessage(ExperienceState.ResultShow);
                SetOrbit(true, easeIn: true);
                _resultShowDone = true;
                _resultShowRoutine = null;
                yield break;
            }

            // The camera stays STILL through the replay (no SetOrbit here) — it
            // starts circling only once the finished model is revealed below.
            SetPresentationLook(true); // grades the ribbons that build for the reveal
            double startAt = 0;
            double duration = sensorRecorder.PlaybackDurationSeconds;
            bool canned = _takeRoot == config.DevCannedTakeRoot;
            if (canned && config.devCannedTakeTailSeconds > 0f
                && duration > config.devCannedTakeTailSeconds)
                startAt = duration - config.devCannedTakeTailSeconds;
            yield return RunPlaybackLoops(ExperienceState.ResultShow, startAt);
            if (!_active || _fsm.State != ExperienceState.ResultShow)
            { _resultShowRoutine = null; yield break; }

            // ---- phase 2: できたよ！ + finished model ----
            // Hide the wireframe (re-freezes the sculpture) AND clear ぶんせきちゅう, hold
            // a pure-black beat, then paint できたよ！ and reveal the finished curve model
            // (history-30 ring, frozen). Freeze BEFORE the orbit so the pivot reads the
            // settled skeleton. _deferResultText stays true across the gap so a repaint
            // (crowd notice clear) shows ぶんせきちゅう, not an early できたよ！.
            SetWireframeReplay(false);
            _ui.ClearAll(); // black gap between ぶんせきちゅう and できたよ！
            if (revealDelay > 0f) yield return new WaitForSeconds(revealDelay);
            if (!_active || _fsm.State != ExperienceState.ResultShow)
            { _resultShowRoutine = null; yield break; }

            RevealFinishedResultModel();
            if (poseHistory != null) poseHistory.hold = true;
            if (motionCurves != null) motionCurves.freeze = true;

            // できたよ！ lands with the finished model (deferred from the gap paint).
            _deferResultText = false;
            PlayStateEnterSe(ExperienceState.ResultShow);
            ShowStateMessage(ExperienceState.ResultShow);

            SetOrbit(true, easeIn: true);
            _resultShowDone = true;
            _resultShowRoutine = null;
        }

        // Reveal the finished point-cloud + ribbon model on the black stage: clear
        // any lingering shoot-end dissolve and draw the frozen final frame + the
        // history-30 ribbons that built underneath the wireframe replay. The caller
        // owns the freeze/hold that pins the model. Shared by the normal reveal and
        // the replay-failed fallback.
        private void RevealFinishedResultModel()
        {
            Shader.SetGlobalFloat(PcFadeCullId, 0f);
            SetCloudRenderersVisible(true);
            if (motionCurves != null) motionCurves.suppressDraw = false;
            // Optional solid-mesh final model (A/B knob, config.presentationSolidMesh):
            // by default the mesh stays suppressed and the finished model is cloud +
            // ribbons; when on, draw the solid shaded TSDF mesh (with self-shadow) inside
            // the ribbons. SetWireframeReplay(false) just set suppressDraw=true — override
            // it here so the choice sticks through the ResultShow/QrShow orbit. The exit
            // restore (EnterMode's _savedTsdfSuppress) puts it back for the next visitor.
            if (tsdfView != null && config.presentationSolidMesh)
            {
                tsdfView.wireframe = false;
                tsdfView.suppressDraw = false;
            }
        }

        // ------------------------------------------------ visitor playback ----

        // Playback transport for the converted take. Processing runs it loop OFF
        // (the recorder stops itself at the end and the scene freezes on the
        // final state); the practice/result play-throughs run loop ON and count
        // wraps via OnPlaybackLooped (RunPlaybackLoops).
        private void StartVisitorPlayback(string takeRoot, bool loop = false, float rate = 1f)
        {
            if (sensorRecorder == null) return;
            if (string.IsNullOrEmpty(takeRoot) || !Directory.Exists(takeRoot))
            {
                Debug.LogError($"[{nameof(ExperienceDirector)}] no playable take ('{takeRoot}').", this);
                return;
            }

            // The visitor take replaces any playback a dev session left running.
            if (sensorRecorder.IsPlaying) sensorRecorder.StopAndUnload();

            // Save the dev transport knobs only on the first activation of a
            // playback stretch — a re-activation (a second practice round after
            // the first restored them, or a restart) must not overwrite the
            // operator's values with the show's own.
            if (!_visitorPlaybackActive)
            {
                _savedRenderDelay = sensorRecorder.playbackRenderDelayFrames;
                _savedLoop = sensorRecorder.loop;
                _savedPlaybackRate = sensorRecorder.playbackRate;
            }
            sensorRecorder.playbackRenderDelayFrames = config.playbackRenderDelayFrames;
            sensorRecorder.loop = loop;
            sensorRecorder.playbackRate = Mathf.Clamp(rate, 0.05f, 2f);
            if (HasLiveRenderers())
            {
                sensorManager?.SetLiveSuppressedAsSource(true);
                // Hide the LIVE clouds while the replay owns the stage — with the
                // shared PointCloudView flag on (playback phases show clouds), the
                // live person would otherwise render alongside their own ghost.
                // Re-shown by ApplyCloudVisibility once the playback ends.
                sensorManager?.SetLiveVisualsVisible(false);
            }

            sensorRecorder.playbackFolderPath = takeRoot;
            // On Mac ResolvePlaybackRoot prefers the mac override (the operator's
            // entrance take) over playbackFolderPath — the visitor take must win
            // here or Processing replays the wrong recording. Restored on exit.
            sensorRecorder.playbackFolderPathMacOverride = takeRoot;
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
                sensorRecorder.playbackRate = _savedPlaybackRate;
                // playbackFolderPath stays until Exit (see Idle side effects), but
                // the mac override must revert NOW: it outranks playbackFolderPath
                // on Mac, and the next visitor's entrance playback (Mac dev loop)
                // would otherwise load this visitor's take instead of the
                // operator's entrance recording.
                sensorRecorder.playbackFolderPathMacOverride = _savedPlaybackFolderMacOverride;
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
            // Quitting started during the release delay: leave the rig closed. Re-opening
            // now would hand the staged teardown devices it has already let go of.
            if (Shared.AppShutdown.IsShuttingDown) yield break;
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
        /// never between, so we can't leave the rig destroyed. The single exception is
        /// a quit that begins during the release delay, where leaving it closed is the
        /// point.
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
                // Quitting: the rig is being torn down on purpose. Re-opening it here
                // would race the staged teardown and re-open devices it just closed.
                if (Shared.AppShutdown.IsShuttingDown)
                {
                    _recoveryRoutine = null;
                    yield break;
                }

                Debug.LogWarning($"[{nameof(ExperienceDirector)}] camera auto-recovery: " +
                                 $"re-init attempt {attempt}/{attempts} " +
                                 $"({healthMonitor?.FaultAlertText})", this);

                sensorManager?.DestroyAllRenderers();
                yield return new WaitForSecondsRealtime(config.cameraReleaseSeconds);
                // Quitting started during the release delay. This is the one path that
                // leaves the rig torn down without its StartLive — deliberately: the
                // process is on its way out and the devices are already free.
                if (Shared.AppShutdown.IsShuttingDown)
                {
                    _recoveryRoutine = null;
                    yield break;
                }
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
                // Wait on AllStreaming, NOT IsHealthy alone: for the first few seconds
                // after StartLive every camera is inside its staleFrameSeconds slack, so
                // IsHealthy reads true while nothing has actually delivered a frame.
                // Exiting on that declared success on a dead rig and dropped straight
                // back into Fault, forever — the settle window below was never used.
                float deadline = Time.realtimeSinceStartup + config.cameraSettleSeconds;
                while (Time.realtimeSinceStartup < deadline
                       && healthMonitor != null && !(healthMonitor.IsHealthy && healthMonitor.AllStreaming))
                    yield return new WaitForSecondsRealtime(0.5f);

                if (healthMonitor == null || (healthMonitor.IsHealthy && healthMonitor.AllStreaming))
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
            // Single source of truth: the exporter's own options, so the capture
            // reads ALL curves (stride 1) when a size budget is set and lets the
            // export's longest-first budget selection do the only thinning. The
            // seed-stride path here used to decimate 1-in-webCurveStride BEFORE
            // the budget saw them, so "keep the longest" only ranked the survivors.
            if (printExporter != null)
                return printExporter.WebCaptureOptions();
            return new TSDFSnapshotBuilder.CaptureOptions
            {
                smoothIterations = 10,
                triangleBudgetPerSlab = 2_000_000,
                meshTargetTris = 150_000,
                includeCurves = true,
                curveStride = 1, // the export's size budget picks from all curves
                curveSides = 4,
                curveTipTaper = 0.25f,
                curveTolerance = 0.0015f,
                includePointCloud = true,
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
            // absorbs the hitch frame — the AO bake adds to it, watch aoMs in the log).
            var webOpt = printExporter != null
                ? printExporter.WebExportOptions()
                : new TSDFSnapshotBuilder.WebFileOptions
                {
                    ao = new MeshAO.Options { strength = 0.6f, samples = 32, maxDistance = 0.15f },
                    omitMesh = true,
                    curveBudgetBytes = 9L * 1048576,
                    shortCurveShare = 0.15f,
                    curveRadiusScale = 1.5f,
                    includePointCloud = true,
                    pointCloudSize = 0.008f,
                };
            if (!TSDFSnapshotBuilder.ExportFiles(snap, glbPath, usdzPath, usdPython,
                                                 out _, out string err, webOpt))
            {
                Debug.LogError($"[{nameof(ExperienceDirector)}] export failed: {err}", this);
                _exportFailed = true;
                ShowExportFailed();
                yield break;
            }

            ISculptureResultPublisher publisher;
            if (config.DryRunPublish)
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

        // Every failure path (Processing and export/publish) lands here, so the
        // 文言 and its cue stay paired. The ResultShow repaint (ShowStateMessage)
        // re-shows the text without replaying the cue.
        private void ShowExportFailed()
        {
            PlaySe(config.exportFailedSe);
            _ui.ShowMessage(config.exportFailedText);
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
            // read presence from occupancy where crowd counting is meaningless
            // (the TestMove states count only while their live half is up).
            bool eligible = s == ExperienceState.Welcome || s == ExperienceState.Calibrate
                || s == ExperienceState.Shoot
                || ((s == ExperienceState.TestMove1 || s == ExperienceState.TestMove2)
                    && !_visitorPlaybackActive);
            bool crowd = eligible && _presence != null && _presence.CrowdActive;
            if (crowd && !_crowdShowing)
            {
                PlaySe(config.crowdSe); // rising edge only — not the per-frame hold
                _ui.ShowMessage(config.crowdText);
                _crowdShowing = true;
            }
            else if (!crowd && _crowdShowing)
            {
                _crowdShowing = false;
                // Mid-gap the blank is intentional — the pending GapPaintRoutine
                // will paint this state when the beat ends.
                if (_gapPaintRoutine == null)
                    ShowStateMessage(s); // repaint only — never re-enter the state
            }
        }
    }
}
