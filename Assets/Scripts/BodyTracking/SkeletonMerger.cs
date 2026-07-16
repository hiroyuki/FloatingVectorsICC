// Multi-camera body tracking via the per-camera k4abt worker fleet from
// issue #10. Spawns one worker per PointCloudRenderer in the cameraManager,
// collects skeletons from each, transforms them to world space using the
// renderer's extrinsic-applied transform, then clusters and merges them into
// a single set of BodyVisuals.
//
// Merge pipeline (per Update):
//   1. Drop snapshots older than maxSkewMs (temporal alignment).
//   2. Build a candidate list of (worker, body, pelvisWorld, maxConfidence).
//   3. Continuity pass: prior persons (sorted by their previous max
//      confidence, descending) each claim the nearest unconsumed candidate
//      within continuityRadiusMeters as a seed, then absorb other workers'
//      bodies within mergeRadiusMeters into the same cluster. A consumed
//      flag prevents double-counting in this pass and the next.
//   4. New-cluster pass: remaining unconsumed candidates form new clusters
//      seeded by the highest-confidence remaining body, again pulling in
//      other workers' bodies within mergeRadiusMeters.
//   5. For each cluster: per-joint confidence-weighted position average +
//      hemisphere-aligned weighted quaternion sum (normalized).
//   6. Push each merged skeleton through BodyVisual.UpdateFromSkeleton via
//      a synthetic camera-local mm encoding so the existing rendering and
//      jump-reset trail logic from issue #7 work unchanged.
//
// Phase 5a moved BodyVisual to its own top-level file. Phase 5b moved the
// per-body dict / eviction / GC pass into the shared BodyVisualPool helper
// (see BodyVisualPool.cs).

using System.Collections.Generic;
using BodyTracking.MultiCam;
using BodyTracking.Shared;
using Orbbec;
using PointCloud;
using UnityEngine;

namespace BodyTracking
{
    [DisallowMultipleComponent]
    public class SkeletonMerger : MonoBehaviour, global::Shared.IViewToggle
    {
        [Header("Sources")]
        [Tooltip("SensorManager whose Renderers we drive workers off. " +
                 "Spawned at Play time; this script binds when the list becomes non-empty.")]
        public SensorManager cameraManager;

        [Tooltip("K4abtWorkerHost used to spawn one worker per camera. Auto-found at " +
                 "OnEnable if left null. Required.")]
        public K4abtWorkerHost workerHost;

        [Tooltip("During playback, ignore any recorded bodies_main and run live k4abt on the " +
                 "played-back depth/IR instead (spawns a worker per camera, same as live capture). " +
                 "Only takes effect on Windows (Editor or player) — on other platforms the BT SDK " +
                 "is unavailable, so recorded bodies_main stays the playback skeleton source and " +
                 "this flag is inert. On Windows this is the intended default: playback should " +
                 "exercise the same live worker pipeline as capture; bodies_main is the fallback " +
                 "for platforms without k4abt (Mac).")]
        public bool ignoreRecordedBodies = true;

        // ignoreRecordedBodies, platform-gated: live k4abt exists only on Windows, so on any
        // other platform the flag must read as false or playback would have no skeleton source.
        private bool IgnoreRecordedActive =>
#if UNITY_EDITOR_WIN || UNITY_STANDALONE_WIN
            ignoreRecordedBodies;
#else
            false;
#endif

        [Tooltip("External body source mode (e.g. LiveFusedBodySource running the RTMPose " +
                 "fusion). While true, skeletons arrive exclusively through " +
                 "SubmitExternalBodies: k4abt workers are never spawned and recorded " +
                 "bodies_main is ignored. LiveFusedBodySource sets/clears this on its own " +
                 "enable/disable.")]
        public bool useExternalBodies = false;

        [Range(0f, 30f)]
        [Tooltip("Cap on how many frames per second are sent to each k4abt worker (0 = every frame). " +
                 "BT inference is the dominant GPU cost with multiple workers; lowering this trades " +
                 "skeleton update rate for host frame rate (the merger holds the latest pose between " +
                 "results). 30 = full sensor rate. Applies to live capture and live-k4abt-on-playback.")]
        public float btInferenceFps = 30f;

        [Range(0f, 1.5f)]
        [Tooltip("Live-k4abt-on-playback only: after a loop wrap, drop skeletons sourced from the " +
                 "first N seconds of the new loop. The playhead teleports backward at the seam, and " +
                 "each worker's k4abt tracker keeps PREDICTING the previous loop's last body for a " +
                 "few frames while processing the new loop's first frames — ingesting that burst " +
                 "re-seeds the old pose as a ghost person that squats for the whole loop. Judged on " +
                 "the SOURCE frame timestamp (not wall time), so worker latency doesn't matter.")]
        public float loopSeamBlackoutSeconds = 0.4f;

        // Set on the first loop wrap; the seam blackout only applies after a wrap (the very
        // first pass from 0 starts with clean trackers and needs no suppression).
        private bool _playbackLoopedOnce;

        [Header("Display")]
        [Tooltip("Master switch for skeleton rendering: ON draws the bone lines between " +
                 "joints (and the joint spheres). OFF hides everything.")]
        [UnityEngine.Serialization.FormerlySerializedAs("showSkeleton")]
        public bool showBones = true;

        // ---- Shared.IViewToggle (unified Views panel) ----
        public string ViewLabel => "BT skeleton";
        public bool Visible { get => showBones; set => showBones = value; }

        // Increments whenever a new body frame is ingested (playback or live worker). Consumers like
        // BonePoseHistory watch it to know a genuinely new pose arrived — so they update on play AND on
        // paused frame-stepping, but hold steady when the playhead is truly static (no repeated-sample
        // collapse). Independent of IsPaused, which stays true while stepping.
        private ulong _poseVersion;
        public ulong PoseVersion => _poseVersion;

        [Tooltip("Joint marker radius (m). Set to 0 to hide the joint spheres entirely " +
                 "(the bone lines stay visible while showBones is on).")]
        [Range(0f, 0.2f)] public float jointRadius = 0.05f;

        [Tooltip("Skeleton color. Bones inherit this; joints are slightly brighter.")]
        public Color skeletonColor = new Color(0.2f, 0.9f, 1f, 1f);

        [Header("Bones")]
        [Range(0.001f, 0.05f)]
        [Tooltip("Radius (m) of the tube mesh drawn for each anatomical bone segment. " +
                 "Independent from the joint sphere radius. Has no effect when Show " +
                 "Anatomical Bones is off.")]
        public float boneWidth = 0.005f;

        [Header("One-Euro filter (1€)")]
        [Tooltip("Speed-adaptive low-pass applied to each joint position in BodyVisual. " +
                 "When the joint is near-stationary cutoff is low (heavy smoothing kills the " +
                 "camera-swap jumps that produced the 'flying bones'); when the joint moves " +
                 "fast cutoff rises so the filter follows the real motion with low lag. " +
                 "Replaces the old mergeSmoothing / trailSmoothing flat EMAs.")]
        public bool useOneEuroFilter = true;
        [Range(0.1f, 10f)]
        [Tooltip("Cutoff frequency (Hz) when the joint is at rest. Lower = stronger smoothing " +
                 "when not moving. 1 Hz is a reasonable default for hand-scale motion.")]
        public float oneEuroMinCutoff = 1.0f;
        [Range(0f, 5f)]
        [Tooltip("How aggressively cutoff rises with joint speed (in Hz per m/s, roughly). " +
                 "Larger = filter releases sooner as the joint moves. Too large and noise leaks " +
                 "through during motion; too small and fast motion lags.")]
        public float oneEuroBeta = 0.5f;
        [Range(0.1f, 10f)]
        [Tooltip("Cutoff (Hz) of the velocity estimator inside the filter. Typically 1 Hz; " +
                 "rarely needs tuning.")]
        public float oneEuroDerivCutoff = 1.0f;

        [Header("Continuity gate")]
        [Tooltip("Down-weight per-camera joint samples that disagree with the merged joint's " +
                 "predicted position (previous merged position + smoothed velocity). Fixes the " +
                 "lifted-foot half-height collapse: occluded-side cameras report a floor-biased " +
                 "position at MEDIUM confidence, which a plain confidence-weighted mean cannot " +
                 "reject. Falls back to the ungated mean when the gate would starve (all cameras " +
                 "agree but the prediction is wrong, e.g. a fast kick).")]
        public bool enableContinuityGate = true;
        [Tooltip("Soft gate width (meters). A sample at this distance from the prediction keeps " +
                 "half its weight: g = 1/(1+(r/sigma)^2). Well below camera disagreement during " +
                 "occlusion (~0.3-0.5) but above BT jitter (~0.02-0.05).")]
        public float gateSigma = 0.12f;
        [Tooltip("EMA retention for the per-joint velocity estimate (0 = raw frame-to-frame " +
                 "velocity, higher = smoother but laggier prediction).")]
        [Range(0f, 0.95f)] public float gateVelocitySmoothing = 0.5f;
        [Tooltip("If the gated weight sum falls below this fraction of the ungated sum, the " +
                 "gate starves and the ungated mean is used instead. Rescues motion the " +
                 "prediction did not anticipate without letting a single outlier dominate.")]
        [Range(0f, 1f)] public float gateFallbackRatio = 0.2f;
        [Tooltip("Cap (seconds) on the prediction extrapolation window. BT samples arrive at " +
                 "~19 Hz; beyond this the prediction is stale and stops extrapolating further.")]
        public float gateMaxPredictionDt = 0.2f;
        [Tooltip("Escape valve for prediction lock-on: after this many consecutive FRESH merges " +
                 "where the gated mean sits more than 2*gateSigma from the ungated consensus, " +
                 "the joint's history resets and the consensus is accepted.")]
        public int gateDivergenceResetSamples = 12;
        [Tooltip("Log gate fallback / divergence-reset events (throttled to one line per second).")]
        public bool logGateDiagnostics = false;

        [Header("BigJump logging")]
        [Tooltip("When on, emit a [BIGJUMP] log line whenever any merged joint moves more " +
                 "than bigJumpLogThresholdMeters between frames. Each line lists the per-" +
                 "camera raw position + confidence so the cause can be classified.")]
        public bool logBigJumps = false;
        [Tooltip("Threshold (meters) for emitting a [BIGJUMP] log. Slightly tighter than " +
                 "BodyVisual.kTrailJumpResetMeters (0.4) so jumps just under that ceiling " +
                 "(which still cause visible 'arm flying' artifacts) are captured too.")]
        public float bigJumpLogThresholdMeters = 0.3f;

        [Header("Visual lifetime")]
        [Tooltip("Hard cap on cached BodyVisuals. K4abt body IDs can flap between frames when " +
                 "tracking is unstable; without a cap we'd allocate 32 GameObjects + Mesh + Material " +
                 "per id forever and freeze Unity. Single-person sessions rarely need more than 2.")]
        [Min(1)] public int maxBodies = 4;

        [Tooltip("Destroy a BodyVisual that hasn't been seen for this many Update ticks.")]
        [Min(1)] public int unseenFramesBeforeDestroy = 60;

        [Header("Merge")]
        [Tooltip("Greedy cluster radius (meters) used by the new-cluster pass. Worker bodies " +
                 "whose pelvis world position falls within this distance of a seed are pulled " +
                 "into the same merged person.")]
        [Range(0.05f, 1f)]
        public float mergeRadiusMeters = 0.3f;

        [Tooltip("Continuity radius (meters) used by the previous-frame proximity stabilizer. " +
                 "If a current detection's pelvis is within this distance of a prior merged " +
                 "person's pelvis, it carries over the prior persistent ID.")]
        [Range(0.05f, 1.5f)]
        public float continuityRadiusMeters = 0.5f;

        [Tooltip("Maximum staleness (ms) for a worker snapshot to participate in this frame's " +
                 "merge. Snapshots older than this are dropped to avoid temporal ghosting. " +
                 "Must comfortably exceed the slowest expected worker output interval — k4abt " +
                 "runs at the depth stream fps (typically 15–30 Hz = 33–66 ms), so 200 ms gives " +
                 "headroom for jitter while still rejecting genuinely stale frames.")]
        [Range(16, 500)]
        public int maxSkewMs = 200;

        [Tooltip("Minimum number of worker snapshots required for a cluster to emit a person. " +
                 "Set to 2 to require multi-camera agreement; 1 (default) accepts single-cam " +
                 "detections too — that's the right setting for a single-camera scene.")]
        [Min(1)]
        public int requireMinWorkerCount = 1;

        [Header("Tracking volume gate")]
        [Tooltip("Discard whole bodies whose pelvis lies outside the tracking volume before " +
                 "clustering. This installation is single-person and everything outside the " +
                 "capture volume must be dropped — without this gate a bystander standing " +
                 "meters outside the box is tracked, rendered, and counted (crowd alert / " +
                 "presence). Per-worker debug skeletons (showPerWorkerSkeletons) stay ungated " +
                 "so raw camera output remains inspectable.")]
        public bool enableVolumeGate = true;

        [Tooltip("OBB defining the tracking volume (same unit-cube convention as the point-" +
                 "cloud filter). Auto-found at OnEnable when left null — the scene has one " +
                 "shared sensing volume. Independent of the volume's filterMode: that switch " +
                 "governs point culling, this gate has its own toggle above.")]
        public BoundingVolume trackingVolume;

        [Tooltip("Outward slack (meters) added to every box face for the gate test only. " +
                 "Keeps a visitor straddling the boundary from flickering in and out; the " +
                 "bystander this gate exists for stood ~3 m outside, far beyond any margin.")]
        [Min(0f)] public float volumeGateMarginMeters = 0.25f;

        [Header("Debug")]
        [Tooltip("Also render each worker's raw skeleton (pre-merge) in a hue derived " +
                 "from the camera serial. Useful for comparing per-camera tracking quality " +
                 "vs the merged output, and for seeing the confidence-weighted average " +
                 "in action.")]
        public bool showPerWorkerSkeletons = false;

        [Tooltip("Joint radius multiplier applied to per-worker raw " +
                 "skeletons (so they sit visually smaller than the merged skeleton).")]
        [Range(0.1f, 1f)]
        public float perWorkerVisualScale = 0.5f;

        [Tooltip("Draw an OnGUI overlay listing per-worker bodies, per-cluster members, " +
                 "and merged persons. Useful during Editor Pause / Step for frame-by-frame " +
                 "evaluation against recorded playback.")]
        public bool showDebugHud = false;

        [Header("Crowd alert")]
        [Tooltip("Show on-screen warning when more than one merged person is detected. " +
                 "Per CLAUDE.md this installation is single-person only.")]
        public bool showCrowdAlert = true;

        [Tooltip("Seconds the >1 person condition must hold before the alert appears (debounce).")]
        [Min(0f)] public float alertOnDelaySeconds = 0.5f;

        [Tooltip("Seconds the ≤1 person condition must hold before the alert hides (debounce).")]
        [Min(0f)] public float alertOffDelaySeconds = 1.0f;

        [Tooltip("Alert message text shown on the GUI overlay.")]
        [TextArea(1, 4)]
        public string alertMessage =
            "複数人検出: 1人のみご利用ください\nMultiple people detected — please use solo";

        [Header("Diagnostics")]
        [Tooltip("Log per-second merge counters (clusters, persons, dropped stale, etc.).")]
        public bool diagnosticLogging = false;

        // --- runtime state ---

        // Per-serial latest snapshot pool. The host fires OnSkeletonsReady with a
        // host-owned reusable buffer, so the handler must copy out (id + 32 joints
        // each) into the pool's per-serial array. Phase 3 reads this pool every
        // Update to do the world-space transform + clustering.
        internal sealed class WorkerLatest
        {
            // Transform used for world-space conversion. For live frames this is the
            // PointCloudRenderer.transform; for recorded playback it is the
            // SensorRecorder's `_Playback_<serial>` GO transform. Both have the
            // same convention (localPosition / localRotation from extrinsics, plus
            // the per-mesh localScale.y = -1 that SkeletonWorldTransform ignores).
            public Transform SourceTransform;
            // Depth→color extrinsic for this camera, in the sensor's mm / OpenCV frame
            // (color_mm = R · depth_mm + T). k4abt joints are in the DEPTH frame; the point
            // cloud is reconstructed in the COLOR frame, so this maps the skeleton into the
            // same space (see SkeletonWorldTransform.ToWorld). Identity = no correction
            // (old recordings without extrinsics fall back to the previous behavior).
            public Matrix4x4 DepthToColorMm = Matrix4x4.identity;
            // Same rotation with the S=diag(1,-1,1) basis change baked in, for orientations.
            public Quaternion DepthToColorRotUnity = Quaternion.identity;
            public string Serial;
            public int BodyCount;
            public ulong CapturedTsNs; // raw frame ts from the worker
            public float CapturedAtRealtime; // host-side wall clock at receipt
            public BodySnapshot[] Bodies = new BodySnapshot[K4abtWorkerSharedLayout.MaxBodies];
            public float LastEnqueueRealtime = -1f; // btInferenceFps throttle bookkeeping
            public WorkerLatest()
            {
                for (int i = 0; i < Bodies.Length; i++) Bodies[i] = new BodySnapshot();
            }
        }

        private readonly Dictionary<string, WorkerLatest> _latestBySerial = new();
        private readonly HashSet<PointCloudRenderer> _boundRenderers = new();
        private bool _hostSubscribed;
        private bool _disabledByGuard;

        // Per-frame work objects for the merge pipeline. Reused across Updates
        // (pool grows as needed; we manage the active count separately so we
        // can avoid allocations even when the body count fluctuates).
        private sealed class Candidate
        {
            public WorkerLatest Slot;
            public int BodyIndex;
            public Vector3 PelvisWorld;
            public int MaxConfidence;
            public bool Consumed;
        }
        private sealed class Cluster
        {
            public uint Id;
            public bool IsCarryOver;
            public readonly List<int> MemberIndices = new();
            public Candidate Seed;
        }
        private readonly List<Candidate> _candidatePool = new();
        private int _candidateCount;
        private readonly List<Cluster> _clusterPool = new();
        private int _clusterCount;
        private readonly List<(uint id, int conf)> _priorSorted = new();

        // Persistent state across frames. Per-body visuals + GC live in
        // BodyVisualPool (Phase 5b); priorPelvis/priorMaxConf are MultiLive-specific
        // continuity hints cleaned up via the pool's onEvicted callback.
        private BodyVisualPool _pool;
        private readonly Dictionary<uint, Vector3> _priorPelvisById = new();
        private readonly Dictionary<uint, int> _priorMaxConfById = new();
        private uint _nextMergedId = 1;

        // BigJump-event diagnostics. When logBigJumps is on, every frame we compare
        // the new merged world position of each joint against the previous frame's
        // merged value; if the delta exceeds bigJumpLogThresholdMeters we emit one
        // [BIGJUMP] line per joint with each contributing camera's raw position +
        // confidence. Used to classify whether bone jumps come from k4abt NN noise
        // (both cameras agree on the new pos), merge swap (cameras disagree and the
        // merge weight shifts), or occlusion (one camera dropped to NONE).
        private readonly Dictionary<uint, Vector3[]> _prevMergedPosByCluster = new();

        // Continuity-gate history, per cluster id (ids are stable across frames via
        // the pelvis carry-over reclaim). Unlike _prevMergedPosByCluster this is
        // updated unconditionally (not just when logBigJumps is on) and only on
        // FRESH BT data (LastMaxCapturedTsNs guard) so the ~48 fps Update loop
        // re-merging the same ~19 Hz worker snapshot doesn't zero the velocity.
        private sealed class GateState
        {
            public readonly Vector3[] PrevPos = new Vector3[K4ABTConsts.K4ABT_JOINT_COUNT];
            public readonly Vector3[] Vel = new Vector3[K4ABTConsts.K4ABT_JOINT_COUNT];
            public readonly bool[] HasPrev = new bool[K4ABTConsts.K4ABT_JOINT_COUNT];
            public readonly int[] DivergeCount = new int[K4ABTConsts.K4ABT_JOINT_COUNT];
            public float LastFreshRealtime;
            public ulong LastMaxCapturedTsNs;
        }
        private readonly Dictionary<uint, GateState> _gateStateById = new();
        // Per-MergeJoint-call transients (single-threaded, reused across joints).
        // _gateState/_gateNow are set per cluster in BuildMergedSkeleton; the
        // scratch arrays carry Pass 1 decisions into Pass 2 (same effective weight
        // for position and orientation) and into UpdateGateHistory (divergence
        // flags counted only on fresh merges).
        private GateState _gateState;
        private float _gateNow;
        private float[] _gateWeightScratch = new float[8];
        private bool _gateUseGated;
        private readonly bool[] _gateDivergedScratch = new bool[K4ABTConsts.K4ABT_JOINT_COUNT];
        private int _gateFallbacksThisSecond;
        private int _gateResetsThisSecond;
        private float _gateDiagWindowStart;

        // Reusable synthetic skeleton handed to BodyVisual.UpdateFromSkeleton.
        // We encode merged world joint positions back into k4a camera-local mm
        // such that K4AmmToUnity (called inside BodyVisual) produces the desired
        // world position. With SkeletonMerger's own transform at world
        // identity, the per-joint K4AmmToUnity output IS the world position.
        private k4abt_skeleton_t _mergedSkel = new k4abt_skeleton_t
        {
            Joints = new k4abt_joint_t[K4ABTConsts.K4ABT_JOINT_COUNT],
        };

        // ---- merged-person read API (experience flow / PresenceDetector) ----
        // Filled once per Update from the final merged skeletons; consumers get
        // world-space positions with no k4a types leaking out.
        //
        // Coordinate contract: _mergedSkel.Joints[].Position holds SYNTHETIC
        // k4a-mm values — merged Unity WORLD positions encoded via UnityToK4Amm.
        // Recover them ONLY with BodyTrackingShared.K4AmmToUnity. Never run them
        // through SkeletonWorldTransform.ToWorld (that is for raw per-worker
        // snapshots and would double-transform).
        public struct PersonSample
        {
            public uint Id;
            public Vector3 PelvisWorld;
            public Vector3 HandLeftWorld, HandRightWorld;
            public bool HandLeftTracked, HandRightTracked; // confidence != NONE
        }

        private readonly List<PersonSample> _persons = new List<PersonSample>(4);

        /// <summary>Merged persons this frame (post-debounceless raw output —
        /// consumers add their own hysteresis).</summary>
        public IReadOnlyList<PersonSample> Persons => _persons;

        /// <summary>Merged person count this frame (raw, no debounce).</summary>
        public int PersonCount => _persons.Count;

        /// <summary>Crowd condition (&gt;1 merged person), with the existing
        /// alertOn/OffDelaySeconds debounce the on-screen warning uses.</summary>
        public bool CrowdActive => _alertActive;

        /// <summary>First merged person (single-person installation: the one
        /// visitor). False when nobody is tracked this frame.</summary>
        public bool TryGetPrimaryPerson(out PersonSample person)
        {
            if (_persons.Count > 0) { person = _persons[0]; return true; }
            person = default;
            return false;
        }

        private void RecordPersonSample(uint id, in k4abt_skeleton_t skel)
        {
            // k4abt's HAND joints are frequently NONE even while the WRIST is
            // MEDIUM (measured on the 12-50-09 take: hands NONE the whole time).
            // For touch purposes the wrist is ~8 cm from the palm — well inside
            // a dwell-sphere radius — so fall back to it.
            ResolveHand(in skel, k4abt_joint_id_t.K4ABT_JOINT_HAND_LEFT,
                        k4abt_joint_id_t.K4ABT_JOINT_WRIST_LEFT, out var handL, out bool trackedL);
            ResolveHand(in skel, k4abt_joint_id_t.K4ABT_JOINT_HAND_RIGHT,
                        k4abt_joint_id_t.K4ABT_JOINT_WRIST_RIGHT, out var handR, out bool trackedR);
            _persons.Add(new PersonSample
            {
                Id = id,
                PelvisWorld = BodyTrackingShared.K4AmmToUnity(skel.Joints[kPelvisIdx].Position),
                HandLeftWorld = handL,
                HandRightWorld = handR,
                HandLeftTracked = trackedL,
                HandRightTracked = trackedR,
            });
        }

        private static void ResolveHand(in k4abt_skeleton_t skel, k4abt_joint_id_t hand,
                                        k4abt_joint_id_t wrist, out Vector3 world, out bool tracked)
        {
            var j = skel.Joints[(int)hand];
            if (j.ConfidenceLevel == k4abt_joint_confidence_level_t.K4ABT_JOINT_CONFIDENCE_NONE)
                j = skel.Joints[(int)wrist];
            world = BodyTrackingShared.K4AmmToUnity(j.Position);
            tracked = j.ConfidenceLevel != k4abt_joint_confidence_level_t.K4ABT_JOINT_CONFIDENCE_NONE;
        }

        // Per-worker debug pools + scratch skeleton for showPerWorkerSkeletons.
        // Each pool draws one camera's raw bodies in a serial-derived hue.
        private readonly Dictionary<string, BodyVisualPool> _perWorkerPools = new Dictionary<string, BodyVisualPool>();
        private k4abt_skeleton_t _rawScratchSkel = new k4abt_skeleton_t
        {
            Joints = new k4abt_joint_t[K4ABTConsts.K4ABT_JOINT_COUNT],
        };

        // Diagnostic counters.
        private int _diagSnapshotsRecv;
        private int _diagDroppedStaleSnapshots;
        private int _diagDroppedOutsideVolume;
        private int _diagFreshIterations; // CollectCandidates: slot non-empty AND not stale (= candidate built)
        private int _diagClustersFormed;
        private int _diagPersonsOutput;
        private int _diagContinuityCarryOver;
        private float _diagMaxObservedAgeMs;
        private float _diagWindowStart;

        // Crowd-alert debounce state. Uses Time.realtimeSinceStartup so the
        // debounce works even when Time.timeScale is 0 (Editor Pause).
        private float _multiPersonSince = -1f;
        private float _singlePersonSince = -1f;
        private bool _alertActive;
        private GUIStyle _alertStyleCache;

        private SensorRecorder _subscribedRecorder;

        private void OnEnable()
        {
            if (!ResolveDependencies()) { _disabledByGuard = true; enabled = false; return; }

            if (_pool == null) _pool = new BodyVisualPool(transform);
            if (trackingVolume == null) trackingVolume = FindFirstObjectByType<BoundingVolume>();
            if (workerHost != null && !_hostSubscribed)
            {
                workerHost.OnSkeletonsReady += OnWorkerSkeletons;
                _hostSubscribed = true;
            }

            // Wire recorded-playback source if a SensorRecorder is present in the
            // scene. Live and playback can coexist: HandleRawFrame and the
            // OnPlaybackRawFrame handler both funnel into DispatchRawFrame.
            // OnPlaybackBodies feeds the per-serial slot directly when the recording
            // includes saved BT (bodies_main); the worker spawn is skipped in that
            // case so playback works on platforms without k4abt (Mac).
            _subscribedRecorder = FindFirstObjectByType<SensorRecorder>();
            if (_subscribedRecorder != null)
            {
                _subscribedRecorder.OnPlaybackRawFrame += OnPlaybackRawFrame;
                _subscribedRecorder.OnPlaybackBodies += OnPlaybackBodies;
                _subscribedRecorder.OnPlaybackLooped += HandlePlaybackLooped;
            }
        }

        // Playback wrapped to the start: the playhead — and every live k4abt worker's
        // tracked pose — jumps discontinuously backward. Drop all carried-over
        // continuity, stale visuals, and per-slot bodies so the next loop starts from a
        // clean slate instead of stranding the previous loop's last person as a ghost
        // cluster (workers are left running; they re-lock within a frame or two).
        private void HandlePlaybackLooped()
        {
            _playbackLoopedOnce = true;
            _priorPelvisById.Clear();
            _priorMaxConfById.Clear();
            _gateStateById.Clear();
            _pool.DestroyAll();
            foreach (var kv in _latestBySerial) kv.Value.BodyCount = 0;
        }

        private void OnPlaybackRawFrame(string serial, ObCameraParam? camParam, Transform sourceTransform, RawFrameData frame)
        {
            HandlePlaybackRawFrame(serial, camParam, sourceTransform, frame);
        }

        // Recorded-body playback path. Decodes the bodies_main payload into a per-
        // serial slot directly — no k4abt worker, so this works on platforms
        // without the BT SDK (Mac). Mirrors OnWorkerSkeletons but takes its
        // BodySnapshot inputs from disk instead of the worker MMF.
        private void OnPlaybackBodies(string serial, ulong tsNs, byte[] bytes, int byteCount,
                                      Transform sourceTransform, ObCameraParam? cameraParam)
        {
            // Forcing live k4abt on playback: drop the recorded bodies_main entirely so stale /
            // mismatched recorded skeletons don't fill the slot and fight the worker output.
            // External-source mode likewise owns the slots exclusively.
            if (IgnoreRecordedActive || useExternalBodies) return;
            IngestBodies(serial, tsNs, bytes, byteCount, sourceTransform, cameraParam);
        }

        /// <summary>External body-source entry (LiveFusedBodySource): same per-serial
        /// slot path as recorded bodies_main, accepted only while
        /// <see cref="useExternalBodies"/> is on so a stray feeder can never fight
        /// the worker/recorded sources.</summary>
        public void SubmitExternalBodies(string serial, ulong tsNs, byte[] bytes, int byteCount,
                                         Transform sourceTransform, ObCameraParam? cameraParam)
        {
            if (!useExternalBodies) return;
            IngestBodies(serial, tsNs, bytes, byteCount, sourceTransform, cameraParam);
        }

        private void IngestBodies(string serial, ulong tsNs, byte[] bytes, int byteCount,
                                  Transform sourceTransform, ObCameraParam? cameraParam)
        {
            // Data ingestion is independent of showBones now — that toggle only controls whether the
            // skeleton is DRAWN (bones + joints), so consumers like BonePoseHistory keep getting poses
            // while the skeleton is hidden.
            if (string.IsNullOrEmpty(serial)) return;
            if (!_latestBySerial.TryGetValue(serial, out var slot))
            {
                slot = new WorkerLatest { Serial = serial, SourceTransform = sourceTransform };
                _latestBySerial[serial] = slot;
            }
            else
            {
                // Recorder can rebuild the _Playback_<serial> GO between sessions;
                // refresh on every frame to track the latest transform.
                slot.SourceTransform = sourceTransform;
            }

            // k4abt joints in bodies_main are raw DEPTH-frame; map into the COLOR frame the
            // point cloud lives in (recorded bodies_main stays raw on disk — this is a
            // display-time correction only). See SkeletonWorldTransform.ToWorld.
            SetSlotExtrinsic(slot, cameraParam);

            int count = RecordedBodySerializer.Decode(bytes, byteCount, slot.Bodies);
            slot.BodyCount = count;
            slot.CapturedTsNs = tsNs;
            slot.CapturedAtRealtime = Time.realtimeSinceStartup;
            _diagSnapshotsRecv += count;
            _poseVersion++;
        }

        private void OnDisable()
        {
            if (workerHost != null && _hostSubscribed)
            {
                workerHost.OnSkeletonsReady -= OnWorkerSkeletons;
                _hostSubscribed = false;
            }

            if (_subscribedRecorder != null)
            {
                _subscribedRecorder.OnPlaybackRawFrame -= OnPlaybackRawFrame;
                _subscribedRecorder.OnPlaybackBodies -= OnPlaybackBodies;
                _subscribedRecorder.OnPlaybackLooped -= HandlePlaybackLooped;
                _subscribedRecorder = null;
            }

            // Stop every worker we started; unbind from every renderer we attached to.
            foreach (var serial in new List<string>(_latestBySerial.Keys))
            {
                if (workerHost != null) workerHost.StopWorker(serial);
            }
            _latestBySerial.Clear();
            foreach (var r in _boundRenderers)
            {
                if (r != null) r.OnRawFramesReady -= HandleRawFrame;
            }
            _pool?.DestroyAll();
            _persons.Clear();
            _priorPelvisById.Clear();
            _priorMaxConfById.Clear();
            _gateStateById.Clear();
            ClearPerWorkerSkeletons();
            _boundRenderers.Clear();
        }

        private void OnDestroy() => OnDisable();
        private void OnApplicationQuit() => OnDisable();

        /// <summary>Stop every k4abt worker and clear merge state WITHOUT
        /// unbinding renderer/recorder subscriptions — bound sources keep
        /// feeding frames, so fresh workers respawn immediately. The experience
        /// flow calls this at the attract→visitor moment: the recorded-clock →
        /// live-clock timestamp jump would otherwise trip the loop-seam guard.</summary>
        public void RestartWorkers()
        {
            foreach (var serial in new List<string>(_latestBySerial.Keys))
            {
                if (workerHost != null) workerHost.StopWorker(serial);
            }
            _latestBySerial.Clear();
            _persons.Clear();
            _priorPelvisById.Clear();
            _priorMaxConfById.Clear();
            _gateStateById.Clear();
            Debug.Log($"[{nameof(SkeletonMerger)}] workers restarted (attract→live handoff).", this);
        }

        /// <summary>Current merged WORLD position of one joint (single-person: first visual),
        /// read straight from the per-frame smoothed pose — NOT the fading-trail buffer. Returns
        /// false if no body or the joint isn't currently valid. Used by the TSDF trail capture so
        /// it doesn't depend on the trail visual being enabled.</summary>
        public bool TryGetJointWorld(k4abt_joint_id_t joint, out UnityEngine.Vector3 world)
        {
            world = UnityEngine.Vector3.zero;
            if (_pool == null) return false;
            foreach (var bv in _pool.Visuals.Values)
                return bv.TryGetJointWorld((int)joint, out world);
            return false;
        }

        private void Update()
        {
            bool nowPaused = _subscribedRecorder != null && _subscribedRecorder.IsPaused;

            // Late-binding for renderers spawned mid-Play by SensorManager.
            BindNewRenderers();

            // Always merge/update the skeleton so data consumers (BonePoseHistory / TSDF trail) keep
            // getting current poses. showBones no longer gates the compute — it flows into the visual
            // config (ShowAnatomicalBones) so BodyVisual hides the bones + joint spheres when it is off,
            // while positions keep updating underneath (IsActive stays true, data stays readable).
            CollectCandidates();
            BuildClusters();
            ApplyMergedSkeletons();
            GcStaleVisuals();
            StashPriorState();
            if (showBones && showPerWorkerSkeletons) ApplyPerWorkerSkeletons();
            else ClearPerWorkerSkeletons();

            // While paused, no new merged skeleton arrives → ApplyMergedSkeletons
            // skipped the per-visual Apply call that would normally push the latest
            // Inspector values into geometry. Push them now so jointRadius /
            // boneWidth / color tweaks reflect live.
            if (nowPaused && showBones && _pool != null)
            {
                var cfg = BuildVisualConfig();
                _pool.ReapplyConfigToAll(in cfg);
            }

            UpdateCrowdAlert();

            if (diagnosticLogging) PerSecondDiag();
        }

        private void UpdateCrowdAlert()
        {
            if (!showCrowdAlert) { _alertActive = false; return; }

            int merged = _pool != null ? _pool.Count : 0;
            float now = Time.realtimeSinceStartup;

            if (merged > 1)
            {
                if (_multiPersonSince < 0f) _multiPersonSince = now;
                _singlePersonSince = -1f;
                if (!_alertActive && now - _multiPersonSince >= alertOnDelaySeconds)
                    _alertActive = true;
            }
            else
            {
                _multiPersonSince = -1f;
                if (_alertActive)
                {
                    if (_singlePersonSince < 0f) _singlePersonSince = now;
                    if (now - _singlePersonSince >= alertOffDelaySeconds)
                        _alertActive = false;
                }
            }
        }

        private void OnGUI()
        {
            if (!_alertActive) return;
            if (_alertStyleCache == null)
            {
                _alertStyleCache = new GUIStyle(GUI.skin.label)
                {
                    fontSize = 32,
                    alignment = TextAnchor.MiddleCenter,
                    fontStyle = FontStyle.Bold,
                    wordWrap = true,
                };
                _alertStyleCache.normal.textColor = new Color(1f, 0.55f, 0.15f, 1f);
            }
            float w = Mathf.Min(Screen.width - 40, 900);
            float h = 140f;
            var rect = new Rect((Screen.width - w) * 0.5f, Screen.height * 0.08f, w, h);
            var prev = GUI.color;
            GUI.color = new Color(0f, 0f, 0f, 0.75f);
            GUI.DrawTexture(rect, Texture2D.whiteTexture);
            GUI.color = prev;
            GUI.Label(rect, alertMessage, _alertStyleCache);

            if (showDebugHud) DrawDebugHud();
        }

        private GUIStyle _hudStyleCache;
        private readonly System.Text.StringBuilder _hudSb = new System.Text.StringBuilder(2048);

        private void DrawDebugHud()
        {
            if (_hudStyleCache == null)
            {
                _hudStyleCache = new GUIStyle(GUI.skin.label)
                {
                    fontSize = 12,
                    alignment = TextAnchor.UpperLeft,
                    wordWrap = false,
                    richText = true,
                };
                _hudStyleCache.normal.textColor = new Color(0.95f, 0.95f, 0.95f, 1f);
            }

            _hudSb.Clear();
            // Time.timeScale == 0 is a reasonable proxy for "Editor Paused" without
            // pulling in UnityEditor.* (which would break player builds).
            bool paused = Time.timeScale <= 0.0001f;
            _hudSb.AppendLine($"<b>[SkeletonMerger Debug]</b> {(paused ? "PAUSED" : "running")}  frame={Time.frameCount}");
            _hudSb.AppendLine($"workers={_latestBySerial.Count} candidates={_candidateCount} clusters={_clusterCount} merged={_pool.Count}");

            // Per-worker bodies
            foreach (var kv in _latestBySerial)
            {
                var slot = kv.Value;
                _hudSb.AppendLine($"<color=#9cf>worker {slot.Serial}</color>  bodies={slot.BodyCount}  ts_ms={slot.CapturedTsNs / 1000000UL}");
                for (int b = 0; b < slot.BodyCount; b++)
                {
                    var body = slot.Bodies[b];
                    int conf = MaxConfidenceInBody(body);
                    var pelvis = body.Joints[kPelvisIdx].Position;
                    _hudSb.AppendLine($"  body[{b}] id={body.Id} maxConf={conf} pelvis_mm=({pelvis.X:F0},{pelvis.Y:F0},{pelvis.Z:F0})");
                }
            }

            // Per-cluster contents
            for (int c = 0; c < _clusterCount; c++)
            {
                var cl = _clusterPool[c];
                _hudSb.AppendLine($"<color=#fc9>cluster[{c}]</color> id={cl.Id} carryOver={cl.IsCarryOver} members={cl.MemberIndices.Count}");
                for (int m = 0; m < cl.MemberIndices.Count; m++)
                {
                    var cand = _candidatePool[cl.MemberIndices[m]];
                    _hudSb.AppendLine($"  {cand.Slot.Serial} body[{cand.BodyIndex}] pelvisW=({cand.PelvisWorld.x:F2},{cand.PelvisWorld.y:F2},{cand.PelvisWorld.z:F2}) maxConf={cand.MaxConfidence}");
                }
            }

            // Per-joint pelvis confidence summary across workers for the first cluster's members
            if (_clusterCount > 0)
            {
                var firstCluster = _clusterPool[0];
                _hudSb.AppendLine($"<color=#9f9>cluster[0] pelvis joint conf per worker</color>");
                for (int m = 0; m < firstCluster.MemberIndices.Count; m++)
                {
                    var cand = _candidatePool[firstCluster.MemberIndices[m]];
                    var pelvisJoint = cand.Slot.Bodies[cand.BodyIndex].Joints[kPelvisIdx];
                    _hudSb.Append($"  {cand.Slot.Serial}: pelvis conf={(int)pelvisJoint.ConfidenceLevel}");
                    _hudSb.AppendLine();
                }
            }

            var content = new GUIContent(_hudSb.ToString());
            var size = _hudStyleCache.CalcSize(content);
            float w = Mathf.Min(size.x + 16, Screen.width - 20);
            float h = Mathf.Min(size.y + 16, Screen.height - 20);
            var rect = new Rect(10, 10, w, h);
            var prev = GUI.color;
            GUI.color = new Color(0f, 0f, 0f, 0.75f);
            GUI.DrawTexture(rect, Texture2D.whiteTexture);
            GUI.color = prev;
            GUI.Label(new Rect(rect.x + 8, rect.y + 8, rect.width - 16, rect.height - 16), content, _hudStyleCache);
        }

        // --- guards ---

        private bool ResolveDependencies()
        {
            if (cameraManager == null) cameraManager = FindFirstObjectByType<SensorManager>();
            if (cameraManager == null)
            {
                Debug.LogError("[SkeletonMerger] SensorManager not found in scene; disabling.", this);
                return false;
            }
            if (workerHost == null) workerHost = FindFirstObjectByType<K4abtWorkerHost>();
            if (workerHost == null)
            {
                Debug.LogError("[SkeletonMerger] K4abtWorkerHost not found in scene; disabling. " +
                               "Add a K4abtWorkerHost MonoBehaviour and reference it here.", this);
                return false;
            }
            if (!workerHost.useWorker)
            {
                Debug.LogWarning("[SkeletonMerger] K4abtWorkerHost.useWorker is false; forcing it true.", this);
                workerHost.useWorker = true;
            }
            return true;
        }

        // applyExtrinsics is required only when more than one camera is active OR when
        // the user has asked for multi-worker agreement. Single-camera (1 renderer,
        // requireMinWorkerCount==1) tolerates identity transforms and runs without it.
        private bool RequiresApplyExtrinsics()
        {
            if (cameraManager == null) return false;
            int rendererCount = cameraManager.Renderers != null ? cameraManager.Renderers.Count : 0;
            return rendererCount > 1 || requireMinWorkerCount > 1;
        }

        // --- renderer + worker binding ---

        private void BindNewRenderers()
        {
            if (cameraManager == null || cameraManager.Renderers == null) return;
            for (int i = 0; i < cameraManager.Renderers.Count; i++)
            {
                var r = cameraManager.Renderers[i];
                if (r == null) continue;
                if (_boundRenderers.Contains(r)) continue;
                r.OnRawFramesReady += HandleRawFrame;
                _boundRenderers.Add(r);
            }
        }

        // Late-init worker on the first raw frame for a given source: that's when we
        // know depth/IR/color resolution and CameraParam is populated.
        private void HandleRawFrame(PointCloudRenderer src, RawFrameData frame)
        {
            // Live freeze: raw frames keep flowing (recording / health monitor)
            // but the skeleton + curves must hold the frozen moment.
            if (src.holdLiveFrame) return;
            string serial = string.IsNullOrEmpty(src.deviceSerial) ? src.gameObject.name : src.deviceSerial;
            // While a recording that contains this serial is being played back, the
            // playback stream owns the worker feed. Letting the live camera enqueue
            // too would interleave live-clock and recorded-clock timestamps in one
            // worker, and the loop-seam guard then drops every skeleton it produces.
            if (_subscribedRecorder != null
                && _subscribedRecorder.CurrentState == SensorRecorder.State.Playing
                && _subscribedRecorder.HasTrack(serial))
                return;
            DispatchRawFrame(serial, src.CameraParam, src.transform, frame);
        }

        /// <summary>
        /// Adapter for the offline / recorded playback path. SensorRecorder fires
        /// this with the same payload we'd build from a live PointCloudRenderer so
        /// the merge pipeline runs without caring about the source.
        /// </summary>
        public void HandlePlaybackRawFrame(string serial, ObCameraParam? cameraParam, Transform sourceTransform, RawFrameData frame)
        {
            if (string.IsNullOrEmpty(serial)) return;
            DispatchRawFrame(serial, cameraParam, sourceTransform, frame);
        }

        private void DispatchRawFrame(string serial, ObCameraParam? cameraParam, Transform sourceTransform, in RawFrameData frame)
        {
            // Data ingestion is independent of showBones now — that toggle only controls whether the
            // skeleton is DRAWN (bones + joints), so consumers like BonePoseHistory keep getting poses
            // while the skeleton is hidden.

            // External-source mode: skeletons arrive through SubmitExternalBodies;
            // never spawn or feed k4abt workers.
            if (useExternalBodies) return;

            // Recorded BT short-circuit: when the recorder is in Playing state AND
            // has a bodies_main track for this serial, skeletons flow in through
            // OnPlaybackBodies instead. Skip the k4abt spawn / enqueue path so
            // playback works on platforms without the BT SDK (Mac). Slot creation +
            // SourceTransform refresh happen in OnPlaybackBodies.
            //
            // Gated specifically on State.Playing because during State.Recording
            // the SAME serial's BodyFrames.Count grows from 0 → 1 → 2 … as
            // OnWorkerSkeletons writes — without this gate, the very first frame
            // we record would flip HasRecordedBodies to true and self-terminate
            // the live enqueue path, capping every recording at 1 body frame per
            // serial. State.Idle similarly must not skip (live capture w/o
            // recorder running is the default).
            if (!IgnoreRecordedActive
                && _subscribedRecorder != null
                && _subscribedRecorder.CurrentState == SensorRecorder.State.Playing
                && _subscribedRecorder.HasRecordedBodies(serial))
                return;

            if (workerHost == null) return;

            bool slotExisted = _latestBySerial.ContainsKey(serial);
            // Recovery path: worker may have crashed (Process.HasExited handled by
            // K4abtWorkerHost.Update, which removes the session). Our slot still
            // points at a dead serial — try StartWorker again so a new exe spawns.
            bool needsSpawn = !slotExisted || !workerHost.HasSession(serial);
            if (needsSpawn)
            {
                if (RequiresApplyExtrinsics() && cameraManager != null && !cameraManager.applyExtrinsics)
                {
                    Debug.LogError(
                        "[SkeletonMerger] applyExtrinsics is false but multi-camera mode " +
                        "needs world-aligned transforms. Enable SensorManager.applyExtrinsics " +
                        "and load extrinsics.yaml. Disabling.", this);
                    enabled = false;
                    _disabledByGuard = true;
                    return;
                }

                if (!cameraParam.HasValue) return; // wait until CameraParam is populated
                int irW = frame.IRWidth > 0 ? frame.IRWidth : frame.DepthWidth;
                int irH = frame.IRHeight > 0 ? frame.IRHeight : frame.DepthHeight;
                if (!workerHost.StartWorker(serial, cameraParam.Value,
                        frame.DepthWidth, frame.DepthHeight, irW, irH,
                        frame.ColorWidth, frame.ColorHeight))
                {
                    // StartWorker logs the underlying reason; don't spam this every frame.
                    if (!slotExisted) Debug.LogError($"[SkeletonMerger] StartWorker failed for serial='{serial}'", this);
                    return;
                }
                if (slotExisted)
                {
                    Debug.LogWarning($"[SkeletonMerger] re-spawned worker for serial='{serial}' (previous instance died)", this);
                    _latestBySerial[serial].SourceTransform = sourceTransform;
                    _latestBySerial[serial].BodyCount = 0;
                }
                else
                {
                    _latestBySerial[serial] = new WorkerLatest { SourceTransform = sourceTransform, Serial = serial };
                }
            }
            else
            {
                // Keep the SourceTransform pointer fresh in case the caller swaps it
                // (e.g. playback rebuilds the _Playback_<serial> GO between sessions).
                _latestBySerial[serial].SourceTransform = sourceTransform;
            }

            // Refresh the depth→color extrinsic each frame (cheap; camParam is stable).
            SetSlotExtrinsic(_latestBySerial[serial], cameraParam);

            if (!workerHost.IsReady(serial)) return;

            // Inference throttle: skip the enqueue if this camera got a frame too recently.
            // The worker keeps only the latest slot anyway, so the skipped frames were mostly
            // wasted GPU inference competing with rendering.
            var throttleSlot = _latestBySerial[serial];
            if (btInferenceFps > 0f
                && throttleSlot.LastEnqueueRealtime >= 0f
                && Time.realtimeSinceStartup - throttleSlot.LastEnqueueRealtime < 1f / btInferenceFps)
                return;
            throttleSlot.LastEnqueueRealtime = Time.realtimeSinceStartup;

            int depthBytes = frame.DepthByteCount;
            byte[] ir = frame.IRBytes;
            int irBytes = frame.IRByteCount;
            ulong tsNs = frame.TimestampUs * 1000UL;
            workerHost.EnqueueFrame(serial, frame.DepthBytes, depthBytes, ir, irBytes, tsNs);
        }

        // Store the per-camera depth→color extrinsic on the slot so world-space joint
        // conversion can map k4abt's depth-frame joints into the color frame the point
        // cloud lives in. No-op when the param is absent or lacks a transform.
        private static void SetSlotExtrinsic(WorkerLatest slot, ObCameraParam? cameraParam)
        {
            if (slot == null || !cameraParam.HasValue) return;
            var e = cameraParam.Value.Transform;
            if (e.Rot == null || e.Rot.Length < 9 || e.Trans == null || e.Trans.Length < 3) return;

            var m = e.ToMatrixMm();
            slot.DepthToColorMm = m;

            // Rotation part (unit scale) with the S=diag(1,-1,1) basis change baked in.
            Quaternion qR = m.rotation;
            slot.DepthToColorRotUnity = new Quaternion(qR.x, -qR.y, qR.z, qR.w).normalized;
        }

        // --- per-serial snapshot intake ---

        // Fired synchronously from K4abtWorkerHost.Update (DefaultExecutionOrder(-100))
        // before this script's Update runs. The host buffer is reused per frame, so
        // we copy the relevant state out into our per-serial slot. <c>tsNs</c> is the
        // depth-frame timestamp the bodies were tracked from (forwarded by the worker
        // from its input slot).
        private void OnWorkerSkeletons(string serial, ulong tsNs, BodySnapshot[] bodies, int count)
        {
            // Data ingestion is independent of showBones now — that toggle only controls whether the
            // skeleton is DRAWN (bones + joints), so consumers like BonePoseHistory keep getting poses
            // while the skeleton is hidden.

            // External-source mode: workers spawned BEFORE the mode flipped may still
            // deliver — their k4abt output must not fight the external fused bodies
            // in the same slots (DispatchRawFrame already stops feeding them).
            if (useExternalBodies) return;

            if (!_latestBySerial.TryGetValue(serial, out var slot)) return;

            // Loop-seam / backward-seek guard (live-k4abt-on-playback): BT inference lags
            // the enqueue by up to ~1s, so right after the playhead wraps backward the
            // workers still deliver skeletons tracked from PRE-seam frames. Legitimate
            // output can only lag the playhead — a result mapping AHEAD of it is a
            // previous-loop leftover; ingesting it would re-seed the previous loop's last
            // pose as a ghost person that the continuity pass then keeps alive. Drop it.
            if (_subscribedRecorder != null
                && _subscribedRecorder.CurrentState == SensorRecorder.State.Playing)
            {
                double snapSec = _subscribedRecorder.PlayheadSecondsOf(tsNs);
                if (snapSec > _subscribedRecorder.CurrentPlayheadSeconds + 0.25)
                {
                    _diagDroppedStaleSnapshots += count;
                    return;
                }
                // Seam blackout: right after a wrap the workers' trackers still hold the
                // previous loop's body and emit it as a PREDICTION against the new loop's
                // first frames (post-seam ts, so the leftover guard above passes it).
                // Suppress everything sourced from the first blackout window of the loop
                // so that burst can't re-seed a ghost.
                if (_playbackLoopedOnce && snapSec < loopSeamBlackoutSeconds)
                {
                    _diagDroppedStaleSnapshots += count;
                    return;
                }
            }

            int n = Mathf.Min(count, slot.Bodies.Length);
            for (int i = 0; i < n; i++)
                slot.Bodies[i].CopyFrom(bodies[i]);
            slot.BodyCount = n;
            slot.CapturedAtRealtime = Time.realtimeSinceStartup;
            slot.CapturedTsNs = tsNs;
            _diagSnapshotsRecv += n;
            _poseVersion++;

            // While recording, persist this worker's output to bodies_main so playback
            // can skip k4abt entirely (and run on Mac). Encode here so the byte buffer
            // crossing into PointCloud asmdef carries no BodyTracking-namespace types.
            if (_subscribedRecorder != null
                && _subscribedRecorder.CurrentState == SensorRecorder.State.Recording)
            {
                int needed = RecordedBodySerializer.FrameSize(n);
                if (_bodyEncodeScratch == null || _bodyEncodeScratch.Length < needed)
                    _bodyEncodeScratch = new byte[Mathf.Max(needed, 4096)];
                int byteCount = RecordedBodySerializer.Encode(slot.Bodies, n, _bodyEncodeScratch);
                _subscribedRecorder.RecordBodies(serial, tsNs, _bodyEncodeScratch, byteCount);
            }
        }

        // Reused per-frame encode scratch for the recording-side body sink. Grown
        // on demand to fit the largest body count seen this session; never shrinks.
        private byte[] _bodyEncodeScratch;

        private void PerSecondDiag()
        {
            float now = Time.realtimeSinceStartup;
            if (_diagWindowStart == 0f) _diagWindowStart = now;
            if (now - _diagWindowStart < 1f) return;
            int boundWorkers = _latestBySerial.Count;
            int totalBodiesNow = 0;
            foreach (var kv in _latestBySerial) totalBodiesNow += kv.Value.BodyCount;
            Debug.Log(
                $"[SkeletonMerger] workers={boundWorkers} " +
                $"snapshots/s={_diagSnapshotsRecv} dropped_stale/s={_diagDroppedStaleSnapshots} " +
                $"dropped_outside/s={_diagDroppedOutsideVolume} " +
                $"fresh_iter/s={_diagFreshIterations} max_age_ms={_diagMaxObservedAgeMs:F0} " +
                $"clusters/s={_diagClustersFormed} persons/s={_diagPersonsOutput} " +
                $"continuity_carry_over/s={_diagContinuityCarryOver} " +
                $"alive_visuals={_pool.Count} bodies_now={totalBodiesNow}",
                this);
            _diagSnapshotsRecv = 0;
            _diagDroppedStaleSnapshots = 0;
            _diagDroppedOutsideVolume = 0;
            _diagFreshIterations = 0;
            _diagMaxObservedAgeMs = 0f;
            _diagClustersFormed = 0;
            _diagPersonsOutput = 0;
            _diagContinuityCarryOver = 0;
            _diagWindowStart = now;
        }

        // --- merge pipeline ---

        private const int kPelvisIdx = (int)k4abt_joint_id_t.K4ABT_JOINT_PELVIS;

        private void CollectCandidates()
        {
            _candidateCount = 0;
            float now = Time.realtimeSinceStartup;
            float maxSkewSec = maxSkewMs * 0.001f;
            // While the recorder is paused (e.g. the TSDF bench freezes playback, or
            // the user scrubs to a frame), the frame is intentionally held still, so
            // wall-clock age is meaningless. Skipping the maxSkew staleness drop keeps
            // the held skeleton alive so the overlay stays on the frozen frame instead
            // of going stale within maxSkewMs and freezing at its last live pose.
            bool recorderPaused = _subscribedRecorder != null && _subscribedRecorder.IsPaused;

            foreach (var kv in _latestBySerial)
            {
                var slot = kv.Value;
                if (slot == null || slot.BodyCount == 0) continue;
                float ageMs = (now - slot.CapturedAtRealtime) * 1000f;
                if (ageMs > _diagMaxObservedAgeMs) _diagMaxObservedAgeMs = ageMs;
                if (!recorderPaused && now - slot.CapturedAtRealtime > maxSkewSec)
                {
                    _diagDroppedStaleSnapshots += slot.BodyCount;
                    continue;
                }
                _diagFreshIterations += slot.BodyCount;
                for (int i = 0; i < slot.BodyCount; i++)
                {
                    var body = slot.Bodies[i];
                    var pelvisJoint = body.Joints[kPelvisIdx];
                    // Earlier code skipped bodies whose pelvis was NONE, but that threw
                    // away whole bodies whose other joints were perfectly trackable
                    // (k4abt still emits a predicted pelvis position for occluded
                    // joints). Keep the body and let the per-joint merge weigh the
                    // pelvis at zero — clustering still works on the predicted pos.
                    Vector3 pelvisWorld = SkeletonWorldTransform.ToWorld(
                        pelvisJoint.Position,
                        slot.DepthToColorMm,
                        slot.SourceTransform);

                    // Volume gate: a body whose pelvis is outside the (margin-
                    // expanded) tracking volume never becomes a candidate, so it
                    // can't seed a cluster, be absorbed into one, spawn a
                    // BodyVisual, or count toward Persons / the crowd alert.
                    if (!PassesVolumeGate(pelvisWorld))
                    {
                        _diagDroppedOutsideVolume++;
                        continue;
                    }

                    var c = AcquireCandidate();
                    c.Slot = slot;
                    c.BodyIndex = i;
                    c.PelvisWorld = pelvisWorld;
                    c.MaxConfidence = MaxConfidenceInBody(body);
                    c.Consumed = false;
                }
            }
        }

        // True when the world position may enter the merge (gate off / no volume /
        // inside the margin-expanded OBB). Same normalized-box math as
        // PresenceDetector.IsInsideSensingVolume, minus the presence-specific XZ
        // inset and world-Y band; the margin expands OUTWARD instead so a visitor
        // straddling the boundary doesn't flicker in and out of tracking.
        private bool PassesVolumeGate(Vector3 world)
        {
            if (!enableVolumeGate) return true;
            if (trackingVolume == null)
            {
                if (!_volumeGateWarned)
                {
                    Debug.LogWarning(
                        $"[{nameof(SkeletonMerger)}] enableVolumeGate is on but no " +
                        "BoundingVolume was found in the scene; the gate is inert.", this);
                    _volumeGateWarned = true;
                }
                return true;
            }
            var t = trackingVolume.transform;
            Vector3 b = t.InverseTransformPoint(world);
            Vector3 s = t.lossyScale;
            float mx = 0.5f + (s.x != 0f ? volumeGateMarginMeters / Mathf.Abs(s.x) : 0f);
            float my = 0.5f + (s.y != 0f ? volumeGateMarginMeters / Mathf.Abs(s.y) : 0f);
            float mz = 0.5f + (s.z != 0f ? volumeGateMarginMeters / Mathf.Abs(s.z) : 0f);
            return Mathf.Abs(b.x) <= mx && Mathf.Abs(b.y) <= my && Mathf.Abs(b.z) <= mz;
        }

        private bool _volumeGateWarned;

        private void BuildClusters()
        {
            _clusterCount = 0;

            // Continuity pass: prior persons sorted by their last-frame max
            // confidence (descending) each claim the nearest unconsumed
            // candidate within continuityRadiusMeters as a seed, then absorb
            // other workers' bodies within mergeRadiusMeters.
            _priorSorted.Clear();
            foreach (var kv in _priorMaxConfById) _priorSorted.Add((kv.Key, kv.Value));
            _priorSorted.Sort((a, b) => b.conf.CompareTo(a.conf));

            for (int p = 0; p < _priorSorted.Count; p++)
            {
                uint priorId = _priorSorted[p].id;
                if (!_priorPelvisById.TryGetValue(priorId, out Vector3 priorPelvis)) continue;
                int seedIdx = NearestUnconsumed(priorPelvis, continuityRadiusMeters);
                if (seedIdx < 0) continue;
                var cluster = AcquireCluster();
                cluster.Id = priorId;
                cluster.IsCarryOver = true;
                cluster.Seed = _candidatePool[seedIdx];
                cluster.MemberIndices.Clear();
                cluster.MemberIndices.Add(seedIdx);
                _candidatePool[seedIdx].Consumed = true;
                AbsorbNeighbors(cluster, _candidatePool[seedIdx]);
                _diagContinuityCarryOver++;
            }

            // New-cluster pass: remaining unconsumed candidates; iterate by
            // descending MaxConfidence so the strongest detection seeds first.
            while (true)
            {
                int seedIdx = HighestConfidenceUnconsumed();
                if (seedIdx < 0) break;
                var cluster = AcquireCluster();
                cluster.Id = _nextMergedId++;
                if (_nextMergedId == 0) _nextMergedId = 1; // skip 0 on (extreme) wrap
                cluster.IsCarryOver = false;
                cluster.Seed = _candidatePool[seedIdx];
                cluster.MemberIndices.Clear();
                cluster.MemberIndices.Add(seedIdx);
                _candidatePool[seedIdx].Consumed = true;
                AbsorbNeighbors(cluster, _candidatePool[seedIdx]);
            }

            _diagClustersFormed += _clusterCount;
        }

        private void ApplyMergedSkeletons()
        {
            var cfg = BuildVisualConfig();
            _persons.Clear();
            for (int c = 0; c < _clusterCount; c++)
            {
                var cluster = _clusterPool[c];
                if (cluster.MemberIndices.Count < requireMinWorkerCount) continue;
                BuildMergedSkeleton(cluster, ref _mergedSkel);
                if (JointRefiner != null) RefineMergedJoints(cluster.Id, ref _mergedSkel);
                _pool.Apply(cluster.Id, in _mergedSkel, cfg, OnVisualEvicted);
                UpdateGateHistory(cluster, in _mergedSkel);
                RecordPersonSample(cluster.Id, in _mergedSkel);
                _diagPersonsOutput++;
            }
            LogGateDiagnosticsIfDue();
        }

        /// <summary>
        /// Optional post-merge refiner (e.g. TSDF surface snap). Set/cleared by
        /// the implementing component's OnEnable/OnDisable; not serialized.
        /// </summary>
        public IMergedJointRefiner JointRefiner { get; set; }

        // Joints the refiner sees. Ankles/feet are where the k4abt failure mode
        // the refiner corrects (occlusion-lagged limbs) was measured; knees are
        // included because an unsnapped knee leaves the shin bone axis ~0.18m off
        // the point cloud, outside the motion-curve seed classification radius
        // (the shin curve never appears). Order must match TSDFJointSnap's
        // per-index embed mapping.
        private static readonly k4abt_joint_id_t[] s_refineJoints =
        {
            k4abt_joint_id_t.K4ABT_JOINT_ANKLE_LEFT,
            k4abt_joint_id_t.K4ABT_JOINT_FOOT_LEFT,
            k4abt_joint_id_t.K4ABT_JOINT_ANKLE_RIGHT,
            k4abt_joint_id_t.K4ABT_JOINT_FOOT_RIGHT,
            k4abt_joint_id_t.K4ABT_JOINT_KNEE_LEFT,
            k4abt_joint_id_t.K4ABT_JOINT_KNEE_RIGHT,
        };
        private readonly Vector3[] _refineScratchPos = new Vector3[s_refineJoints.Length];
        private readonly bool[] _refineScratchValid = new bool[s_refineJoints.Length];

        private void RefineMergedJoints(uint clusterId, ref k4abt_skeleton_t skel)
        {
            for (int i = 0; i < s_refineJoints.Length; i++)
            {
                int j = (int)s_refineJoints[i];
                bool valid = skel.Joints[j].ConfidenceLevel != k4abt_joint_confidence_level_t.K4ABT_JOINT_CONFIDENCE_NONE;
                _refineScratchValid[i] = valid;
                _refineScratchPos[i] = valid ? BodyTrackingShared.K4AmmToUnity(skel.Joints[j].Position) : Vector3.zero;
            }

            JointRefiner.RefineJoints(clusterId, s_refineJoints, _refineScratchPos, _refineScratchValid);

            for (int i = 0; i < s_refineJoints.Length; i++)
            {
                if (!_refineScratchValid[i]) continue;
                int j = (int)s_refineJoints[i];
                // ConfidenceLevel stays as merged (>= LOW) so BodyVisual applies
                // the refined position instead of holding the previous one.
                skel.Joints[j].Position = BodyTrackingShared.UnityToK4Amm(_refineScratchPos[i]);
            }
        }

        private void GcStaleVisuals()
        {
            // Runs even while paused. Paused playback still re-applies the held
            // frame's skeletons every Update (CollectCandidates keeps the frozen
            // slots alive), so a body backed by the current frame keeps its unseen
            // counter at 0 and survives. Only genuinely dead visuals tick past
            // unseenFramesBeforeDestroy — e.g. a ghost left over from the pre-seek
            // warmup frame that no current skeleton matches (the bench seeks the
            // playhead, so the start-of-play body would otherwise float forever).
            // (Before the slots were kept alive while paused this GC had to be
            // skipped, or it would have destroyed the frozen skeleton mid-pause.)
            _pool.GcStale(unseenFramesBeforeDestroy, OnVisualEvicted);
        }

        private void OnVisualEvicted(uint id)
        {
            _priorPelvisById.Remove(id);
            _priorMaxConfById.Remove(id);
            _prevMergedPosByCluster.Remove(id);
            _gateStateById.Remove(id);
        }

        private void StashPriorState()
        {
            // Snapshot the current frame's merged persons for next-frame continuity.
            // We stash the seed candidate's world pelvis for each cluster.
            //
            // Critically, we do NOT clear when the current frame produced zero
            // clusters. Worker output (~16-30 fps) is slower than Unity Update
            // (60+ fps), so most Updates legitimately have stale snapshots and
            // form no clusters. Clearing on those Updates would wipe the carry-
            // over hint and force every fresh frame to assign a new id. Visual
            // pool eviction (OnVisualEvicted) is the source of truth for when a
            // prior should disappear long-term.
            if (_clusterCount == 0) return;

            // Only refresh the entries for clusters we actually emitted this frame.
            // Other prior entries (e.g. a body briefly missing while the visual is
            // still alive) stay around so the next fresh frame can reclaim them.
            for (int c = 0; c < _clusterCount; c++)
            {
                var cluster = _clusterPool[c];
                if (cluster.MemberIndices.Count < requireMinWorkerCount) continue;
                _priorPelvisById[cluster.Id] = cluster.Seed.PelvisWorld;
                _priorMaxConfById[cluster.Id] = cluster.Seed.MaxConfidence;
            }
        }

        /// <summary>
        /// Iterate every active body in the merged pool and write each valid bone
        /// (both endpoints valid) as a world-space capsule into
        /// <paramref name="filter"/>. Existing entries in the filter are cleared
        /// first. <paramref name="radius"/> is applied to every capsule. Stops
        /// once the filter's capacity is reached. Returns the number of capsules
        /// written.
        /// </summary>
        public int PublishBoneCapsulesWorld(PointCloud.PointCloudCapsuleFilter filter, float radius)
        {
            if (filter == null) return 0;
            filter.Clear();
            if (_pool == null) return 0;
            int written = 0;
            foreach (var kv in _pool.Visuals)
            {
                var bv = kv.Value;
                if (bv == null || !bv.IsActive) continue;
                var bones = BodyTrackingShared.Bones;
                for (int b = 0; b < bones.Length; b++)
                {
                    int ia = (int)bones[b].a;
                    int ic = (int)bones[b].b;
                    if (!bv.JointValid(ia) || !bv.JointValid(ic)) continue;
                    Vector3 wa = bv.WorldOf(bv.JointPosition(ia));
                    Vector3 wc = bv.WorldOf(bv.JointPosition(ic));
                    if (!filter.TryAdd(wa, wc, radius)) return written;
                    written++;
                }
            }
            return written;
        }

        /// <summary>World endpoints of one bone for the primary (single-person) body, with
        /// validity + freshness. Consumed by <see cref="BonePoseHistory"/> to build per-point
        /// curved motion trails. Unlike raw k4abt joint orientations (which the merge path does
        /// not preserve), endpoints are stable, so the curve's stable frame is rebuilt from
        /// these each frame.</summary>
        public struct BoneEndpoints
        {
            public Vector3 A;   // world position of bone.a (parent side)
            public Vector3 B;   // world position of bone.b (child side)
            public bool Valid;  // both endpoints currently valid
            public bool Fresh;  // both endpoints freshly observed (>=LOW confidence, recent)
        }

        /// <summary>Fill <paramref name="dst"/> (index = <see cref="BodyTrackingShared.Bones"/>
        /// index) with the primary body's per-bone world endpoints + valid/fresh flags. Mirrors
        /// <see cref="PublishBoneCapsulesWorld"/> but single-body and CPU-side. Freshness = both
        /// endpoints have &gt;=LOW confidence and were refreshed within <paramref name="freshWindowFrames"/>
        /// frames, so history can reset on tracking loss instead of dragging stale joints.
        /// Returns true if a primary body existed (all Bones.Length entries written).</summary>
        public bool TryReadBonePosesWorld(BoneEndpoints[] dst, int freshWindowFrames)
        {
            var bones = BodyTrackingShared.Bones;
            if (dst == null || dst.Length < bones.Length) return false;
            for (int b = 0; b < bones.Length; b++) dst[b] = default;
            if (_pool == null) return false;

            BodyVisual bv = null;
            foreach (var kv in _pool.Visuals)
            {
                if (kv.Value != null && kv.Value.IsActive) { bv = kv.Value; break; }
            }
            if (bv == null) return false;

            int curFrame = Time.frameCount;
            for (int b = 0; b < bones.Length; b++)
            {
                int ia = (int)bones[b].a, ic = (int)bones[b].b;
                bool valid = bv.JointValid(ia) && bv.JointValid(ic);
                var e = new BoneEndpoints { Valid = valid };
                if (valid)
                {
                    e.A = bv.WorldOf(bv.JointPosition(ia));
                    e.B = bv.WorldOf(bv.JointPosition(ic));
                    bool freshA = bv.LastConfidence(ia) >= k4abt_joint_confidence_level_t.K4ABT_JOINT_CONFIDENCE_LOW
                                  && (curFrame - bv.LastFreshFrame(ia)) <= freshWindowFrames;
                    bool freshC = bv.LastConfidence(ic) >= k4abt_joint_confidence_level_t.K4ABT_JOINT_CONFIDENCE_LOW
                                  && (curFrame - bv.LastFreshFrame(ic)) <= freshWindowFrames;
                    e.Fresh = freshA && freshC;
                }
                dst[b] = e;
            }
            return true;
        }

        // Per-body finite-difference state for PublishJointMotionsWorld. Holds
        // each joint's previous world position + smoothed velocity. To avoid
        // velocity spikes when k4abt loses sight of a joint for N publishes
        // and the position then jumps back to a real value, we key the
        // finite-difference dt off BodyVisual's per-joint LastFreshFrame:
        // velocity is recomputed only when that frame counter actually
        // advances (= a new BT sample arrived for that joint), using the
        // realtime delta since the previous fresh sample. Between fresh
        // arrivals the smoothed value is held (no decay) so visuals stay
        // stable while BT misses the joint briefly.
        private sealed class JointMotionState
        {
            public Vector3[] PrevWorldPos = new Vector3[K4ABTConsts.K4ABT_JOINT_COUNT];
            public Vector3[] SmoothedVel = new Vector3[K4ABTConsts.K4ABT_JOINT_COUNT];
            public int[] LastBtFreshFrame = new int[K4ABTConsts.K4ABT_JOINT_COUNT];
            public float[] LastBtFreshRealtime = new float[K4ABTConsts.K4ABT_JOINT_COUNT];
            public bool[] HasPrev = new bool[K4ABTConsts.K4ABT_JOINT_COUNT];
        }
        private readonly Dictionary<uint, JointMotionState> _jointMotionStateById = new();
        private readonly HashSet<uint> _jointMotionSeenIds = new HashSet<uint>();
        private readonly List<uint> _jointMotionGcScratch = new List<uint>();

        /// <summary>
        /// Iterate every active body in the merged pool and write each valid
        /// joint's world-space position + estimated velocity (m/s) into
        /// <paramref name="field"/>. Existing entries are cleared first.
        ///
        /// Velocity is keyed off BodyVisual.LastFreshFrame(j): we only
        /// recompute the finite difference when that counter advances (= k4abt
        /// produced a new sample for that joint since our previous publish),
        /// using the realtime delta between fresh samples. This avoids the
        /// classic stale-prev-with-tiny-dt spike when BT temporarily loses
        /// sight of a joint — the joint's reported position is held inside
        /// BodyVisual during the gap, so naively dividing the eventual jump
        /// by one publish dt would explode the velocity.
        /// <paramref name="velocitySmoothing"/> EMA-blends the new raw
        /// velocity into the held smoothed value (0 = raw, 0.95 = heavily
        /// smoothed). Stops writing once the field's capacity is reached but
        /// still finishes state bookkeeping + per-body GC, so the dictionary
        /// doesn't leak across id flapping. Returns the number of joints
        /// written.
        /// </summary>
        public int PublishJointMotionsWorld(PointCloud.PointCloudJointMotionField field, float velocitySmoothing)
        {
            if (field == null) return 0;
            field.Clear();
            if (_pool == null) return 0;

            float now = Time.realtimeSinceStartup;
            float alpha = 1f - Mathf.Clamp(velocitySmoothing, 0f, 0.95f);

            _jointMotionSeenIds.Clear();
            int written = 0;
            bool fieldFull = false;
            foreach (var kv in _pool.Visuals)
            {
                uint id = kv.Key;
                var bv = kv.Value;
                if (bv == null || !bv.IsActive) continue;
                _jointMotionSeenIds.Add(id);

                if (!_jointMotionStateById.TryGetValue(id, out var state))
                {
                    state = new JointMotionState();
                    _jointMotionStateById[id] = state;
                }

                for (int j = 0; j < K4ABTConsts.K4ABT_JOINT_COUNT; j++)
                {
                    if (!bv.JointValid(j)) continue;
                    Vector3 cur = bv.WorldOf(bv.JointPosition(j));
                    int btFresh = bv.LastFreshFrame(j);

                    if (!state.HasPrev[j])
                    {
                        // First time we see this joint for this body: seed
                        // history with zero velocity so we don't emit a fake
                        // jump on round one.
                        state.PrevWorldPos[j] = cur;
                        state.SmoothedVel[j] = Vector3.zero;
                        state.LastBtFreshFrame[j] = btFresh;
                        state.LastBtFreshRealtime[j] = now;
                        state.HasPrev[j] = true;
                    }
                    else if (btFresh != state.LastBtFreshFrame[j])
                    {
                        // A new BT sample arrived since our previous publish.
                        // Use the realtime delta between fresh samples — NOT
                        // the publish dt — so a multi-frame BT outage divides
                        // the position delta by the real elapsed time.
                        float dt = Mathf.Max(1e-4f, now - state.LastBtFreshRealtime[j]);
                        Vector3 rawVel = (cur - state.PrevWorldPos[j]) / dt;
                        state.SmoothedVel[j] = Vector3.Lerp(state.SmoothedVel[j], rawVel, alpha);
                        state.PrevWorldPos[j] = cur;
                        state.LastBtFreshFrame[j] = btFresh;
                        state.LastBtFreshRealtime[j] = now;
                    }
                    // else: joint position is stale (BT didn't pump a new
                    // sample for this joint since our last publish). Keep
                    // SmoothedVel unchanged — decaying it here would cancel
                    // out the EMA's smoothing benefit during normal BT pop
                    // cadences slower than the renderer.

                    if (!fieldFull)
                    {
                        if (field.TryAdd(cur, state.SmoothedVel[j])) written++;
                        else fieldFull = true;
                    }
                }
            }

            // GC per-id state for bodies that left the pool since last publish.
            // Runs unconditionally (even when fieldFull capped writes) so the
            // dictionary doesn't accumulate stale ids during id flapping.
            _jointMotionGcScratch.Clear();
            foreach (var k in _jointMotionStateById.Keys)
                if (!_jointMotionSeenIds.Contains(k)) _jointMotionGcScratch.Add(k);
            for (int i = 0; i < _jointMotionGcScratch.Count; i++)
                _jointMotionStateById.Remove(_jointMotionGcScratch[i]);
            return written;
        }

        private BodyVisualConfig BuildVisualConfig() => new BodyVisualConfig
        {
            JointRadius = jointRadius,
            SkeletonColor = skeletonColor,
            ShowAnatomicalBones = showBones,
            BoneWidth = boneWidth,
            MaxBodies = maxBodies,
            UseOneEuroFilter = useOneEuroFilter,
            OneEuroMinCutoff = oneEuroMinCutoff,
            OneEuroBeta = oneEuroBeta,
            OneEuroDerivCutoff = oneEuroDerivCutoff,
        };

        // --- per-cluster merge (joint-by-joint) ---

        private void BuildMergedSkeleton(Cluster cluster, ref k4abt_skeleton_t output)
        {
            // Gate context for this cluster: MergeJoint reads history only;
            // UpdateGateHistory (after _pool.Apply) is the sole writer.
            _gateState = enableContinuityGate ? GetOrCreateGateState(cluster.Id) : null;
            _gateNow = Time.realtimeSinceStartup;
            if (_gateWeightScratch.Length < cluster.MemberIndices.Count)
                _gateWeightScratch = new float[Mathf.NextPowerOfTwo(cluster.MemberIndices.Count)];

            for (int j = 0; j < K4ABTConsts.K4ABT_JOINT_COUNT; j++)
            {
                MergeJoint(cluster, j, ref output.Joints[j]);
            }
        }

        private GateState GetOrCreateGateState(uint clusterId)
        {
            if (!_gateStateById.TryGetValue(clusterId, out var s))
            {
                s = new GateState();
                _gateStateById[clusterId] = s;
            }
            return s;
        }

        // Sole writer of the gate history. Runs after _pool.Apply so the stored
        // positions match what BodyVisual received. Kept warm even while
        // enableContinuityGate is off so toggling it on mid-run starts from real
        // velocities instead of a cold start.
        private void UpdateGateHistory(Cluster cluster, in k4abt_skeleton_t merged)
        {
            var state = GetOrCreateGateState(cluster.Id);

            ulong maxTs = 0;
            for (int m = 0; m < cluster.MemberIndices.Count; m++)
            {
                var cand = _candidatePool[cluster.MemberIndices[m]];
                if (cand.Slot.CapturedTsNs > maxTs) maxTs = cand.Slot.CapturedTsNs;
            }
            // Update (~48 fps) re-merges the same ~19 Hz worker snapshot on most
            // frames; recomputing velocity from identical data would zero it out.
            if (maxTs == state.LastMaxCapturedTsNs) return;

            float now = Time.realtimeSinceStartup;
            float dt = state.LastFreshRealtime > 0f ? Mathf.Max(1e-4f, now - state.LastFreshRealtime) : 0f;

            for (int j = 0; j < K4ABTConsts.K4ABT_JOINT_COUNT; j++)
            {
                if (merged.Joints[j].ConfidenceLevel == k4abt_joint_confidence_level_t.K4ABT_JOINT_CONFIDENCE_NONE)
                    continue; // no sample this merge — keep history as-is
                Vector3 cur = BodyTrackingShared.K4AmmToUnity(merged.Joints[j].Position);

                if (!state.HasPrev[j] || dt <= 0f)
                {
                    // Cold start (first sighting, post-reset, or unknown dt):
                    // seed with zero velocity so no fake prediction is emitted.
                    state.PrevPos[j] = cur;
                    state.Vel[j] = Vector3.zero;
                    state.HasPrev[j] = true;
                    state.DivergeCount[j] = 0;
                    continue;
                }

                Vector3 rawVel = (cur - state.PrevPos[j]) / dt;
                state.Vel[j] = Vector3.Lerp(rawVel, state.Vel[j], gateVelocitySmoothing);
                state.PrevPos[j] = cur;

                // Lock-on escape: gated output persistently far from the ungated
                // consensus means the prediction is tracking a wrong branch —
                // reset the history and accept the consensus on the next merge.
                if (_gateDivergedScratch[j])
                {
                    if (++state.DivergeCount[j] >= Mathf.Max(1, gateDivergenceResetSamples))
                    {
                        state.HasPrev[j] = false;
                        state.DivergeCount[j] = 0;
                        _gateResetsThisSecond++;
                    }
                }
                else state.DivergeCount[j] = 0;
            }

            state.LastFreshRealtime = now;
            state.LastMaxCapturedTsNs = maxTs;
        }

        private void LogGateDiagnosticsIfDue()
        {
            if (!logGateDiagnostics)
            {
                _gateFallbacksThisSecond = 0;
                _gateResetsThisSecond = 0;
                return;
            }
            float now = Time.realtimeSinceStartup;
            if (_gateDiagWindowStart == 0f) _gateDiagWindowStart = now;
            if (now - _gateDiagWindowStart < 1f) return;
            if (_gateFallbacksThisSecond > 0 || _gateResetsThisSecond > 0)
                Debug.Log($"[SkeletonMerger] gate/sec fallbacks={_gateFallbacksThisSecond} divergenceResets={_gateResetsThisSecond}", this);
            _gateFallbacksThisSecond = 0;
            _gateResetsThisSecond = 0;
            _gateDiagWindowStart = now;
        }

        // BigJump diagnostic: compare new merged world position to previous frame's
        // and, if delta > bigJumpLogThresholdMeters, emit a single Debug.Log line
        // listing each contributing camera's raw world position + confidence so the
        // cause can be classified post-hoc (NN noise / merge swap / occlusion).
        private void LogBigJumpIfAny(Cluster cluster, int jointIndex, Vector3 mergedPosWorld)
        {
            if (!_prevMergedPosByCluster.TryGetValue(cluster.Id, out var prevArr))
            {
                prevArr = new Vector3[K4ABTConsts.K4ABT_JOINT_COUNT];
                // sentinel: NaN means "no prior sample yet"
                for (int i = 0; i < prevArr.Length; i++) prevArr[i] = new Vector3(float.NaN, 0f, 0f);
                _prevMergedPosByCluster[cluster.Id] = prevArr;
            }

            var prev = prevArr[jointIndex];
            if (!float.IsNaN(prev.x))
            {
                float delta = Vector3.Distance(prev, mergedPosWorld);
                if (delta > bigJumpLogThresholdMeters)
                {
                    var sb = new System.Text.StringBuilder();
                    sb.AppendFormat("[BIGJUMP] frame={0} body={1} joint={2} delta={3:F3}m prev=({4:F2},{5:F2},{6:F2}) new=({7:F2},{8:F2},{9:F2})",
                        Time.frameCount, cluster.Id, (k4abt_joint_id_t)jointIndex,
                        delta, prev.x, prev.y, prev.z, mergedPosWorld.x, mergedPosWorld.y, mergedPosWorld.z);
                    for (int m = 0; m < cluster.MemberIndices.Count; m++)
                    {
                        var cand = _candidatePool[cluster.MemberIndices[m]];
                        var jt = cand.Slot.Bodies[cand.BodyIndex].Joints[jointIndex];
                        if (jt.ConfidenceLevel >= k4abt_joint_confidence_level_t.K4ABT_JOINT_CONFIDENCE_LOW)
                        {
                            Vector3 worldPos = SkeletonWorldTransform.ToWorld(jt.Position, cand.Slot.DepthToColorMm, cand.Slot.SourceTransform);
                            sb.AppendFormat(" | cam={0} conf={1} pos=({2:F2},{3:F2},{4:F2})",
                                cand.Slot.Serial, jt.ConfidenceLevel, worldPos.x, worldPos.y, worldPos.z);
                        }
                        else
                        {
                            sb.AppendFormat(" | cam={0} conf={1}", cand.Slot.Serial, jt.ConfidenceLevel);
                        }
                    }
                    Debug.Log(sb.ToString(), this);
                }
            }
            prevArr[jointIndex] = mergedPosWorld;
        }

        private void MergeJoint(Cluster cluster, int jointIndex, ref k4abt_joint_t outJoint)
        {
            // NONE is always excluded from the merge; LOW+ contributes with a
            // confidence-proportional weight.
            const int minAcceptLevel = (int)k4abt_joint_confidence_level_t.K4ABT_JOINT_CONFIDENCE_LOW;

            // Pass 0: k4abt reports occluded joints as LOW with a *predicted*
            // position (biased toward the floor for lifted feet). If any camera
            // still tracks the joint at MEDIUM+, averaging those predictions in
            // drags the merged joint back down — so LOW only participates as a
            // fallback when no camera has MEDIUM or better.
            int maxLevelSeen = 0;
            for (int m = 0; m < cluster.MemberIndices.Count; m++)
            {
                var cand = _candidatePool[cluster.MemberIndices[m]];
                int level = (int)cand.Slot.Bodies[cand.BodyIndex].Joints[jointIndex].ConfidenceLevel;
                if (level > maxLevelSeen) maxLevelSeen = level;
            }
            int effectiveMinLevel = maxLevelSeen >= (int)k4abt_joint_confidence_level_t.K4ABT_JOINT_CONFIDENCE_MEDIUM
                ? (int)k4abt_joint_confidence_level_t.K4ABT_JOINT_CONFIDENCE_MEDIUM
                : minAcceptLevel;

            // Continuity gate: predict this joint from the merged history and
            // down-weight samples far from the prediction. Occluded-side cameras
            // report floor-biased positions at MEDIUM confidence, so confidence
            // alone cannot reject them — trajectory consistency can.
            bool gateActive = _gateState != null && _gateState.HasPrev[jointIndex];
            Vector3 predicted = default;
            float gateSigmaSqr = gateSigma * gateSigma;
            if (gateActive)
            {
                float predDt = Mathf.Clamp(_gateNow - _gateState.LastFreshRealtime, 0f, gateMaxPredictionDt);
                predicted = _gateState.PrevPos[jointIndex] + _gateState.Vel[jointIndex] * predDt;
            }

            // Pass 1: find the highest-confidence sample to use as the
            // hemisphere reference for the quaternion mean. Accumulate the gated
            // and ungated weighted positions side by side so the fallback needs
            // no second pass over the members.
            float refConf = -1f;
            Quaternion refRot = Quaternion.identity;

            float wSumRaw = 0f, wSumGated = 0f;
            Vector3 posSumRaw = Vector3.zero, posSumGated = Vector3.zero;
            int maxConf = 0;
            int sampleCount = 0;

            for (int m = 0; m < cluster.MemberIndices.Count; m++)
            {
                _gateWeightScratch[m] = 0f;
                var cand = _candidatePool[cluster.MemberIndices[m]];
                var jt = cand.Slot.Bodies[cand.BodyIndex].Joints[jointIndex];
                int level = (int)jt.ConfidenceLevel;
                if (level < effectiveMinLevel) continue; // NONE excluded; LOW excluded when MEDIUM+ exists

                float weight = level; // confidence-linear weight
                Vector3 worldPos = SkeletonWorldTransform.ToWorld(jt.Position, cand.Slot.DepthToColorMm, cand.Slot.SourceTransform);
                Quaternion worldRot = SkeletonWorldTransform.ToWorldRotation(jt.Orientation, cand.Slot.DepthToColorRotUnity, cand.Slot.SourceTransform);

                wSumRaw += weight;
                posSumRaw += worldPos * weight;
                if (gateActive)
                {
                    float g = 1f / (1f + (worldPos - predicted).sqrMagnitude / gateSigmaSqr);
                    float gw = weight * g;
                    _gateWeightScratch[m] = gw;
                    wSumGated += gw;
                    posSumGated += worldPos * gw;
                }
                if (level > maxConf) maxConf = level;
                if (weight > refConf)
                {
                    refConf = weight;
                    refRot = worldRot;
                }
                sampleCount++;
            }

            if (sampleCount == 0)
            {
                // No usable sample this frame for this joint — emit a NONE so
                // BodyVisual keeps the joint at its previous position.
                _gateDivergedScratch[jointIndex] = false;
                outJoint.Position = default;
                outJoint.Orientation = default;
                outJoint.ConfidenceLevel = k4abt_joint_confidence_level_t.K4ABT_JOINT_CONFIDENCE_NONE;
                return;
            }

            // Fallback: when the gated sum collapses relative to the ungated one,
            // every camera disagrees with the prediction — trust the consensus
            // (fast unpredicted motion) rather than starving the merge.
            _gateUseGated = gateActive
                && wSumGated > 1e-4f
                && wSumGated >= gateFallbackRatio * wSumRaw;
            if (gateActive && !_gateUseGated) _gateFallbacksThisSecond++;

            Vector3 mergedPosWorld = _gateUseGated ? posSumGated / wSumGated : posSumRaw / wSumRaw;

            // Lock-on detection input: gated output sitting far from the ungated
            // consensus. Counted per FRESH merge in UpdateGateHistory; after
            // gateDivergenceResetSamples in a row the joint history resets.
            _gateDivergedScratch[jointIndex] = _gateUseGated
                && (mergedPosWorld - posSumRaw / wSumRaw).sqrMagnitude > 4f * gateSigmaSqr;

            // BIGJUMP detection runs on the raw merged position — the diagnostic
            // measures the input to BodyVisual's One-Euro filter, not the output,
            // which is what tells us whether the underlying merge is unstable.
            if (logBigJumps) LogBigJumpIfAny(cluster, jointIndex, mergedPosWorld);

            // Position smoothing now lives in BodyVisual (1€ filter, per-joint). The
            // previous merge-stage One-Euro / mergeSmoothing EMAs were removed
            // because they layered redundant flat low-passes on top of the same
            // signal — see BodyVisual.UpdateFromSkeleton.

            // Pass 2: hemisphere-aligned weighted quaternion sum.
            Vector4 qSum = Vector4.zero;
            for (int m = 0; m < cluster.MemberIndices.Count; m++)
            {
                var cand = _candidatePool[cluster.MemberIndices[m]];
                var jt = cand.Slot.Bodies[cand.BodyIndex].Joints[jointIndex];
                int level = (int)jt.ConfidenceLevel;
                if (level < effectiveMinLevel) continue;

                // Same effective weight as the position pass: a camera whose
                // position the gate rejected must not steer the orientation.
                float weight = _gateUseGated ? _gateWeightScratch[m] : level;
                Quaternion qLocal = SkeletonWorldTransform.ToWorldRotation(jt.Orientation, cand.Slot.DepthToColorRotUnity, cand.Slot.SourceTransform);
                if (Quaternion.Dot(qLocal, refRot) < 0f)
                {
                    qLocal = new Quaternion(-qLocal.x, -qLocal.y, -qLocal.z, -qLocal.w);
                }
                qSum.x += qLocal.x * weight;
                qSum.y += qLocal.y * weight;
                qSum.z += qLocal.z * weight;
                qSum.w += qLocal.w * weight;
            }
            float qMag = Mathf.Sqrt(qSum.x * qSum.x + qSum.y * qSum.y + qSum.z * qSum.z + qSum.w * qSum.w);
            Quaternion mergedRot = qMag > 1e-6f
                ? new Quaternion(qSum.x / qMag, qSum.y / qMag, qSum.z / qMag, qSum.w / qMag)
                : refRot;

            // Encode merged world position as synthetic camera-local mm so the
            // BodyVisual.UpdateFromSkeleton path (which calls K4AmmToUnity) ends up
            // placing the joint at the desired world position — UnityToK4Amm is
            // the exact inverse.
            outJoint.Position = BodyTrackingShared.UnityToK4Amm(mergedPosWorld);
            // Orientation isn't read by BodyVisual today, but keep the merged
            // value populated so downstream consumers (motion line, future IK)
            // see something sensible.
            outJoint.Orientation = new k4a_quaternion_t
            {
                W = mergedRot.w,
                X = mergedRot.x,
                Y = mergedRot.y,
                Z = mergedRot.z,
            };
            outJoint.ConfidenceLevel = (k4abt_joint_confidence_level_t)maxConf;
        }

        // --- helpers ---

        private static int MaxConfidenceInBody(BodySnapshot body)
        {
            int max = 0;
            for (int j = 0; j < body.Joints.Length; j++)
            {
                int c = (int)body.Joints[j].ConfidenceLevel;
                if (c > max) max = c;
            }
            return max;
        }

        // Find the nearest unconsumed candidate within radius of a point. Returns
        // -1 if none. Callers pass continuityRadiusMeters or mergeRadiusMeters as
        // the cap; greedy nearest-neighbor at this scale (~10 candidates) is fine.
        private int NearestUnconsumed(Vector3 point, float radius)
        {
            int best = -1;
            float bestSqr = radius * radius;
            for (int i = 0; i < _candidateCount; i++)
            {
                var c = _candidatePool[i];
                if (c.Consumed) continue;
                float d = (c.PelvisWorld - point).sqrMagnitude;
                if (d <= bestSqr) { bestSqr = d; best = i; }
            }
            return best;
        }

        private int HighestConfidenceUnconsumed()
        {
            int best = -1;
            int bestConf = -1;
            for (int i = 0; i < _candidateCount; i++)
            {
                var c = _candidatePool[i];
                if (c.Consumed) continue;
                if (c.MaxConfidence > bestConf)
                {
                    bestConf = c.MaxConfidence;
                    best = i;
                }
            }
            return best;
        }

        // After picking a seed, pull in any other workers' bodies that lie within
        // mergeRadiusMeters of the seed pelvis. Skip same-worker bodies so a single
        // worker contributes at most one detection per cluster (k4abt assigns
        // distinct ids per worker so this is the right invariant).
        private void AbsorbNeighbors(Cluster cluster, Candidate seed)
        {
            float radiusSqr = mergeRadiusMeters * mergeRadiusMeters;
            for (int i = 0; i < _candidateCount; i++)
            {
                var cand = _candidatePool[i];
                if (cand.Consumed) continue;
                if (cand.Slot == seed.Slot) continue; // same worker
                if ((cand.PelvisWorld - seed.PelvisWorld).sqrMagnitude > radiusSqr) continue;
                cluster.MemberIndices.Add(i);
                cand.Consumed = true;
            }
        }

        private Candidate AcquireCandidate()
        {
            Candidate c;
            if (_candidateCount < _candidatePool.Count)
            {
                c = _candidatePool[_candidateCount];
            }
            else
            {
                c = new Candidate();
                _candidatePool.Add(c);
            }
            _candidateCount++;
            return c;
        }

        private Cluster AcquireCluster()
        {
            Cluster c;
            if (_clusterCount < _clusterPool.Count)
            {
                c = _clusterPool[_clusterCount];
            }
            else
            {
                c = new Cluster();
                _clusterPool.Add(c);
            }
            _clusterCount++;
            return c;
        }

        // --- per-worker raw skeleton debug (showPerWorkerSkeletons) ---

        private void ApplyPerWorkerSkeletons()
        {
            // Reuse the candidate list built by CollectCandidates so we get only the
            // bodies that survived the maxSkewMs gate and the pelvis-confidence filter.
            var baseCfg = BuildVisualConfig();
            for (int i = 0; i < _candidateCount; i++)
            {
                var cand = _candidatePool[i];
                if (cand.Slot == null) continue;
                string serial = cand.Slot.Serial;
                if (!_perWorkerPools.TryGetValue(serial, out var pool))
                {
                    pool = new BodyVisualPool(transform);
                    _perWorkerPools[serial] = pool;
                }
                BuildPerWorkerWorldSkeleton(cand, ref _rawScratchSkel);
                var cfg = baseCfg;
                cfg.SkeletonColor = ColorForSerial(serial);
                cfg.MaxBodies = K4abtWorkerSharedLayout.MaxBodies;
                float scale = Mathf.Clamp(perWorkerVisualScale, 0.1f, 1f);
                cfg.JointRadius = baseCfg.JointRadius * scale;
                pool.Apply((uint)cand.BodyIndex, in _rawScratchSkel, cfg);
            }
            // GC stale per-worker visuals on the same threshold as merged — also
            // while paused (held slots keep current bodies fresh; see GcStaleVisuals).
            foreach (var pool in _perWorkerPools.Values)
                pool.GcStale(unseenFramesBeforeDestroy);
        }

        private void ClearPerWorkerSkeletons()
        {
            if (_perWorkerPools.Count == 0) return;
            foreach (var pool in _perWorkerPools.Values) pool.DestroyAll();
            _perWorkerPools.Clear();
        }

        private void BuildPerWorkerWorldSkeleton(Candidate cand, ref k4abt_skeleton_t output)
        {
            var body = cand.Slot.Bodies[cand.BodyIndex];
            Transform rendererT = cand.Slot.SourceTransform;
            Matrix4x4 depthToColor = cand.Slot.DepthToColorMm;
            for (int j = 0; j < K4ABTConsts.K4ABT_JOINT_COUNT; j++)
            {
                var jt = body.Joints[j];
                if ((int)jt.ConfidenceLevel <= 0)
                {
                    // Keep BodyVisual on its previous position for this joint.
                    output.Joints[j] = new k4abt_joint_t { ConfidenceLevel = k4abt_joint_confidence_level_t.K4ABT_JOINT_CONFIDENCE_NONE };
                    continue;
                }
                Vector3 worldPos = SkeletonWorldTransform.ToWorld(jt.Position, depthToColor, rendererT);
                output.Joints[j] = new k4abt_joint_t
                {
                    // Synthetic camera-local mm encoding (inverse of K4AmmToUnity),
                    // same trick as MergeJoint above.
                    Position = BodyTrackingShared.UnityToK4Amm(worldPos),
                    Orientation = jt.Orientation,
                    ConfidenceLevel = jt.ConfidenceLevel,
                };
            }
        }

        // Hash-into-palette per camera serial. Femto Bolt serials like
        // CL8F253004Z and CL8F253004N differ in only the last byte; a continuous
        // hue mapping still lands them close on the colour wheel even with FNV-1a.
        // A discrete palette of well-spaced hues makes adjacent serials visually
        // distinct, which is what matters for debug viz.
        private static readonly Color[] s_workerPalette = new[]
        {
            new Color(0.95f, 0.30f, 0.20f, 1f), // red
            new Color(0.20f, 0.70f, 0.95f, 1f), // cyan
            new Color(0.95f, 0.80f, 0.20f, 1f), // yellow
            new Color(0.55f, 0.95f, 0.30f, 1f), // green
            new Color(0.80f, 0.30f, 0.95f, 1f), // purple
            new Color(0.95f, 0.55f, 0.20f, 1f), // orange
            new Color(0.30f, 0.95f, 0.80f, 1f), // teal
            new Color(0.95f, 0.40f, 0.70f, 1f), // pink
        };

        private static Color ColorForSerial(string serial)
        {
            if (string.IsNullOrEmpty(serial)) return new Color(0.7f, 0.7f, 0.7f, 1f);
            uint hash = 2166136261u;
            for (int i = 0; i < serial.Length; i++)
            {
                hash ^= serial[i];
                hash *= 16777619u;
            }
            return s_workerPalette[hash % (uint)s_workerPalette.Length];
        }
    }
}
