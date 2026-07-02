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
// Phase 5a moved BodyVisual to its own top-level file. Phase 5b will pull
// the _bodies dict / EvictIfFull / GC pass / ApplyBodySkeleton into a
// shared BodyVisualPool helper.

using System.Collections.Generic;
using BodyTracking.MultiCam;
using BodyTracking.Shared;
using Orbbec;
using PointCloud;
using UnityEngine;

namespace BodyTracking
{
    [DisallowMultipleComponent]
    public class SkeletonMerger : MonoBehaviour
    {
        [Header("Sources")]
        [Tooltip("PointCloudCameraManager whose Renderers we drive workers off. " +
                 "Spawned at Play time; this script binds when the list becomes non-empty.")]
        public PointCloudCameraManager cameraManager;

        [Tooltip("K4abtWorkerHost used to spawn one worker per camera. Auto-found at " +
                 "OnEnable if left null. Required.")]
        public K4abtWorkerHost workerHost;

        [Header("Display")]
        [Tooltip("Master switch for skeleton rendering: ON draws the bone lines between " +
                 "joints (and the joint spheres). OFF hides everything.")]
        [UnityEngine.Serialization.FormerlySerializedAs("showSkeleton")]
        public bool showBones = true;

        [Tooltip("Joint marker radius (m). Set to 0 to hide the joint spheres entirely " +
                 "(the bone lines stay visible while showBones is on).")]
        [Range(0f, 0.2f)] public float jointRadius = 0.05f;

        [Tooltip("Skeleton color. Bones inherit this; joints are slightly brighter.")]
        public Color skeletonColor = new Color(0.2f, 0.9f, 1f, 1f);

        [Header("Trails")]
        [Tooltip("Attach a TrailRenderer to each merged joint so motion leaves a line in real time.")]
        public bool showTrails = true;

        [Tooltip("PerJointHue assigns each joint a distinct hue. FlatColor uses trailFlatColor for every joint.")]
        public BodyTrackingShared.TrailColorMode trailColorMode = BodyTrackingShared.TrailColorMode.PerJointHue;

        [Tooltip("Trail color in FlatColor mode (also tints PerJointHue).")]
        public Color trailFlatColor = Color.white;

        [Header("Trail FrameHue mode")]
        [Tooltip("FrameHue mode: hue cycles around this center hue (0..1) as a function of Time.frameCount.")]
        [Range(0f, 1f)]
        public float frameHueCenter = 0.5f;

        [Tooltip("FrameHue mode: hue sweep width around the center (0..1). 0 = static hue, 1 = full color wheel.")]
        [Range(0f, 1f)]
        public float frameHueRange = 1f;

        [Tooltip("FrameHue mode: HSV Saturation (0..1).")]
        [Range(0f, 1f)]
        public float frameHueSaturation = 0.85f;

        [Tooltip("FrameHue mode: HSV Value / brightness (0..1).")]
        [Range(0f, 1f)]
        public float frameHueValue = 1f;

        [Tooltip("FrameHue mode: number of frames per full sine cycle of the hue oscillation. " +
                 "At 60 fps, 120 ≈ a 2-second wave.")]
        [Min(1f)]
        public float frameHueCyclePeriodFrames = 120f;

        [Min(0.05f)]
        [Tooltip("How long (s) each trail segment stays visible before fading out.")]
        public float trailDuration = 2.0f;

        [Range(0.001f, 0.05f)]
        [Tooltip("Trail width (m) at the head; tail tapers to ~0.")]
        public float trailWidth = 0.005f;

        [Range(0.001f, 0.05f)]
        [Tooltip("Radius (m) of the tube mesh drawn for each anatomical bone segment. " +
                 "Independent from trailWidth — joint sphere radius / trail width do not " +
                 "influence this. Has no effect when Show Anatomical Bones is off.")]
        public float boneWidth = 0.005f;

        [Range(0f, 0.5f)]
        [Tooltip("Parametric step for additional interpolation-point trails along each bone. " +
                 "0 disables them (only joint trails draw). 0.1 → 9 interp points per bone, " +
                 "0.05 → 19. Uniform spacing along the bone parameter so visual density is " +
                 "consistent regardless of bone length. Each interp point gets its own trail " +
                 "fed at the same cadence as joint trails.")]
        public float boneTrailStep = 0f;

        [Tooltip("Acceleration value (m/s^2) that maps to the cold/base trail color. " +
                 "Same semantics as MotionLineRenderer.accelMin on the playback side.")]
        public float accelMin = 0f;

        [Tooltip("Acceleration value (m/s^2) that maps to the hot color. Default 56 " +
                 "comes from the p95 of the reference recording.")]
        public float accelMax = 56f;

        [Tooltip("Hot end of the AccelHeatmap. The cold end is trailFlatColor.")]
        public Color accelHotColor = Color.red;

        [Tooltip("If true, accelMax is replaced every frame by a rolling p95 of |a| " +
                 "across all joints — same auto behavior as MotionLineRenderer.autoAccelMax. " +
                 "Avoids having to hand-tune accelMax for each scene/recording.")]
        public bool autoAccelMax = false;

        [Tooltip("If true, when at least one camera reports MED or HIGH confidence for a " +
                 "joint, all other cameras' LOW samples are dropped from the merge. Sounded " +
                 "principled but in practice MADE THE 'flying bone' problem WORSE (16x more " +
                 "bigJumps): MED↔LOW flapping on either camera abruptly switched the merged " +
                 "result between two stationary-but-different-by-0.5m positions. Kept as a " +
                 "toggle for A/B comparison; the One-Euro filter below is the actual fix.")]
        public bool dropLowWhenHigherAvailable = false;

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

        public enum WeightStrategy { Linear, Squared }
        [Tooltip("Confidence-to-weight mapping. Linear: weight = (int)level. Squared: " +
                 "weight = level * level. v1 implements Linear only; Squared placeholder.")]
        public WeightStrategy weightStrategy = WeightStrategy.Linear;

        [Header("Debug")]
        [Tooltip("Also render each worker's raw skeleton (pre-merge) in a hue derived " +
                 "from the camera serial. Useful for comparing per-camera tracking quality " +
                 "vs the merged output, and for seeing the confidence-weighted average " +
                 "in action.")]
        public bool showPerWorkerSkeletons = false;

        [Tooltip("Joint radius / trail width multiplier applied to per-worker raw " +
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
            // PointCloudRecorder's `_Playback_<serial>` GO transform. Both have the
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

        // Reusable synthetic skeleton handed to BodyVisual.UpdateFromSkeleton.
        // We encode merged world joint positions back into k4a camera-local mm
        // such that K4AmmToUnity (called inside BodyVisual) produces the desired
        // world position. With SkeletonMerger's own transform at world
        // identity, the per-joint K4AmmToUnity output IS the world position.
        private k4abt_skeleton_t _mergedSkel = new k4abt_skeleton_t
        {
            Joints = new k4abt_joint_t[K4ABTConsts.K4ABT_JOINT_COUNT],
        };

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

        private PointCloudRecorder _subscribedRecorder;

        // Pause-aware trail clock. JointTrailMesh sample timestamps + DropOldSamples
        // cutoff both flow off _trailNow, which is Time.timeAsDouble minus the time
        // we've spent inside PointCloudRecorder pauses. While the recorder is paused
        // _trailNow stays frozen at the moment of pause-entry, so no trail samples
        // expire while the user tweaks Inspector values to refine the visual.
        private bool _wasRecorderPaused;
        private double _pauseStartTime;
        private double _pausedAccumDuration;
        private double _trailNow;

        private void OnEnable()
        {
            if (!ResolveDependencies()) { _disabledByGuard = true; enabled = false; return; }

            if (_pool == null) _pool = new BodyVisualPool(transform);
            if (workerHost != null && !_hostSubscribed)
            {
                workerHost.OnSkeletonsReady += OnWorkerSkeletons;
                _hostSubscribed = true;
            }

            // Wire recorded-playback source if a PointCloudRecorder is present in the
            // scene. Live and playback can coexist: HandleRawFrame and the
            // OnPlaybackRawFrame handler both funnel into DispatchRawFrame.
            // OnPlaybackBodies feeds the per-serial slot directly when the recording
            // includes saved BT (bodies_main); the worker spawn is skipped in that
            // case so playback works on platforms without k4abt (Mac).
            _subscribedRecorder = FindFirstObjectByType<PointCloudRecorder>();
            if (_subscribedRecorder != null)
            {
                _subscribedRecorder.OnPlaybackRawFrame += OnPlaybackRawFrame;
                _subscribedRecorder.OnPlaybackBodies += OnPlaybackBodies;
            }
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
            if (!showBones) return;
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
            _priorPelvisById.Clear();
            _priorMaxConfById.Clear();
            ClearPerWorkerSkeletons();
            _boundRenderers.Clear();
        }

        private void OnDestroy() => OnDisable();
        private void OnApplicationQuit() => OnDisable();

        /// <summary>Append the current merged body's windowed trail centerline (world space,
        /// oldest→newest) for <paramref name="joint"/> to <paramref name="outWorld"/>. Single
        /// person assumed (first visual in the pool). Returns the number appended; 0 if no body
        /// or no trail. Exposes the same per-joint ribbon the JointTrailMesh draws so the SDF
        /// trail baker can bake it as capsules. Trails only accumulate while showBones is on.</summary>
        public int CopyTrailWorldPoints(k4abt_joint_id_t joint, List<UnityEngine.Vector3> outWorld)
        {
            if (_pool == null || outWorld == null) return 0;
            foreach (var bv in _pool.Visuals.Values)
                return bv.CopyTrailWorldPoints((int)joint, outWorld);
            return 0;
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
            // Pause-aware trail clock. Track entry/exit transitions so the
            // accumulated paused duration grows only when the recorder is paused.
            // While paused, _trailNow stays frozen so JointTrailMesh.DropOldSamples
            // doesn't drop anything (cutoff = frozenNow - duration, sample times
            // are all <= frozenNow, the in-window subset stays in-window forever).
            bool nowPaused = _subscribedRecorder != null && _subscribedRecorder.IsPaused;
            double rawNow = Time.timeAsDouble;
            if (nowPaused && !_wasRecorderPaused) _pauseStartTime = rawNow;
            else if (!nowPaused && _wasRecorderPaused) _pausedAccumDuration += rawNow - _pauseStartTime;
            _wasRecorderPaused = nowPaused;
            _trailNow = (nowPaused ? _pauseStartTime : rawNow) - _pausedAccumDuration;

            // Late-binding for renderers spawned mid-Play by PointCloudCameraManager.
            BindNewRenderers();

            if (showBones)
            {
                CollectCandidates();
                BuildClusters();
                ApplyMergedSkeletons();
                GcStaleVisuals();
                StashPriorState();
                if (showPerWorkerSkeletons) ApplyPerWorkerSkeletons();
                else ClearPerWorkerSkeletons();
            }
            else if (_perWorkerPools.Count > 0)
            {
                ClearPerWorkerSkeletons();
            }

            // While paused, no new merged skeleton arrives → ApplyMergedSkeletons
            // skipped the per-visual Apply call that would normally push the latest
            // Inspector values into geometry + trail Configure. Push them now so
            // jointRadius / boneWidth / boneTrailStep / color tweaks reflect live.
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
            if (cameraManager == null) cameraManager = FindFirstObjectByType<PointCloudCameraManager>();
            if (cameraManager == null)
            {
                Debug.LogError("[SkeletonMerger] PointCloudCameraManager not found in scene; disabling.", this);
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
            string serial = string.IsNullOrEmpty(src.deviceSerial) ? src.gameObject.name : src.deviceSerial;
            DispatchRawFrame(serial, src.CameraParam, src.transform, frame);
        }

        /// <summary>
        /// Adapter for the offline / recorded playback path. PointCloudRecorder fires
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
            if (!showBones) return;

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
            if (_subscribedRecorder != null
                && _subscribedRecorder.CurrentState == PointCloudRecorder.State.Playing
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
                        "needs world-aligned transforms. Enable PointCloudCameraManager.applyExtrinsics " +
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

            var m = Matrix4x4.identity;
            m.SetRow(0, new Vector4(e.Rot[0], e.Rot[1], e.Rot[2], e.Trans[0]));
            m.SetRow(1, new Vector4(e.Rot[3], e.Rot[4], e.Rot[5], e.Trans[1]));
            m.SetRow(2, new Vector4(e.Rot[6], e.Rot[7], e.Rot[8], e.Trans[2]));
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
            if (!showBones) return;
            if (!_latestBySerial.TryGetValue(serial, out var slot)) return;
            int n = Mathf.Min(count, slot.Bodies.Length);
            for (int i = 0; i < n; i++)
            {
                slot.Bodies[i].Id = bodies[i].Id;
                System.Array.Copy(bodies[i].Joints, slot.Bodies[i].Joints, slot.Bodies[i].Joints.Length);
            }
            slot.BodyCount = n;
            slot.CapturedAtRealtime = Time.realtimeSinceStartup;
            slot.CapturedTsNs = tsNs;
            _diagSnapshotsRecv += n;

            // While recording, persist this worker's output to bodies_main so playback
            // can skip k4abt entirely (and run on Mac). Encode here so the byte buffer
            // crossing into PointCloud asmdef carries no BodyTracking-namespace types.
            if (_subscribedRecorder != null
                && _subscribedRecorder.CurrentState == PointCloudRecorder.State.Recording)
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
                $"fresh_iter/s={_diagFreshIterations} max_age_ms={_diagMaxObservedAgeMs:F0} " +
                $"clusters/s={_diagClustersFormed} persons/s={_diagPersonsOutput} " +
                $"continuity_carry_over/s={_diagContinuityCarryOver} " +
                $"alive_visuals={_pool.Count} bodies_now={totalBodiesNow}",
                this);
            _diagSnapshotsRecv = 0;
            _diagDroppedStaleSnapshots = 0;
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

                    var c = AcquireCandidate();
                    c.Slot = slot;
                    c.BodyIndex = i;
                    c.PelvisWorld = pelvisWorld;
                    c.MaxConfidence = MaxConfidenceInBody(body);
                    c.Consumed = false;
                }
            }
        }

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
            for (int c = 0; c < _clusterCount; c++)
            {
                var cluster = _clusterPool[c];
                if (cluster.MemberIndices.Count < requireMinWorkerCount) continue;
                BuildMergedSkeleton(cluster, ref _mergedSkel);
                _pool.Apply(cluster.Id, in _mergedSkel, cfg, _trailNow, OnVisualEvicted);
                _diagPersonsOutput++;
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

        private void LateUpdate()
        {
            var cam = Camera.main;
            if (_pool != null) _pool.TickTrails(cam, autoAccelMax, _trailNow);
            if (_perWorkerPools != null)
                foreach (var p in _perWorkerPools.Values)
                    p.TickTrails(cam, autoAccelMax, _trailNow);
        }

        /// <summary>
        /// Per-joint detail dump: confidence + last-frame-fresh + last inter-pop jump
        /// (meters) + window-max jump. Reset window-max after reading by passing reset=true.
        /// Use to find which joints are flapping fast even while staying at LOW+ confidence.
        /// </summary>
        public string DumpJointJumps(bool reset, float minJumpToList)
        {
            if (_pool == null) return "(pool null)";
            int curFrame = Time.frameCount;
            var sb = new System.Text.StringBuilder();
            sb.AppendFormat("[JointJumps] frame={0} bodies={1} (listing joints with windowMaxJump > {2:F2}m)\n",
                curFrame, _pool.Count, minJumpToList);
            foreach (var kv in _pool.Visuals)
            {
                var bv = kv.Value;
                sb.AppendFormat("body {0}:\n", kv.Key);
                for (int i = 0; i < K4ABTConsts.K4ABT_JOINT_COUNT; i++)
                {
                    float wm = bv.MaxJumpInWindow(i);
                    if (wm < minJumpToList) continue;
                    int sinceFresh = curFrame - bv.LastFreshFrame(i);
                    sb.AppendFormat("  {0,-22} conf={1,-32} sinceFresh={2,3}f  lastJump={3:F3}m  windowMax={4:F3}m\n",
                        (k4abt_joint_id_t)i, bv.LastConfidence(i), sinceFresh,
                        bv.LastJumpMeters(i), wm);
                }
                if (reset) bv.ResetJumpWindow();
            }
            return sb.ToString();
        }

        /// <summary>Snapshot of the merge pipeline state for live debugging.</summary>
        public string DumpPipelineState()
        {
            var sb = new System.Text.StringBuilder();
            sb.AppendFormat("workers={0} candidates={1} clusters={2} merged={3} snapsRecv={4} dropStale={5} freshIter={6}\n",
                _latestBySerial.Count, _candidateCount, _clusterCount,
                _pool != null ? _pool.Count : 0,
                _diagSnapshotsRecv, _diagDroppedStaleSnapshots, _diagFreshIterations);
            foreach (var kv in _latestBySerial)
            {
                var w = kv.Value;
                float ageMs = (Time.realtimeSinceStartup - w.CapturedAtRealtime) * 1000f;
                sb.AppendFormat("  serial={0} bodyCount={1} ageMs={2:F0}\n", kv.Key, w.BodyCount, ageMs);
            }
            return sb.ToString();
        }

        /// <summary>
        /// Diagnostic for the "flying bones" investigation. Classifies each bone
        /// of every merged body by endpoint freshness (using BodyVisual's stashed
        /// per-joint confidence + last-fresh-frame). Returns a multi-line summary
        /// suitable for Debug.Log.
        /// </summary>
        public string DumpBoneFreshness()
        {
            if (_pool == null) return "(pool null)";
            int curFrame = Time.frameCount;
            var sb = new System.Text.StringBuilder();
            sb.AppendFormat("[BoneFreshness] frame={0} bodies={1}\n", curFrame, _pool.Count);
            foreach (var kv in _pool.Visuals)
            {
                var bv = kv.Value;
                int[] tally = new int[4];
                int stale10 = 0, stale30 = 0;
                for (int i = 0; i < K4ABTConsts.K4ABT_JOINT_COUNT; i++)
                {
                    var c = bv.LastConfidence(i);
                    int ci = Mathf.Clamp((int)c, 0, 3);
                    tally[ci]++;
                    int sinceFresh = curFrame - bv.LastFreshFrame(i);
                    if (bv.JointValid(i) && sinceFresh > 10) stale10++;
                    if (bv.JointValid(i) && sinceFresh > 30) stale30++;
                }
                sb.AppendFormat("body {0}: conf NONE/LOW/MED/HIGH = {1}/{2}/{3}/{4}  stale>10f={5} stale>30f={6}\n",
                    kv.Key, tally[0], tally[1], tally[2], tally[3], stale10, stale30);

                int both = 0, oneStale = 0, bothStale = 0, notDrawn = 0;
                var stretchHits = new System.Text.StringBuilder();
                for (int b = 0; b < BodyTrackingShared.Bones.Length; b++)
                {
                    var bone = BodyTrackingShared.Bones[b];
                    int ia = (int)bone.a, ic = (int)bone.b;
                    bool va = bv.JointValid(ia), vc = bv.JointValid(ic);
                    if (!va || !vc) { notDrawn++; continue; }
                    var ca = bv.LastConfidence(ia);
                    var cc = bv.LastConfidence(ic);
                    bool freshA = ca >= k4abt_joint_confidence_level_t.K4ABT_JOINT_CONFIDENCE_LOW;
                    bool freshC = cc >= k4abt_joint_confidence_level_t.K4ABT_JOINT_CONFIDENCE_LOW;
                    if (freshA && freshC) both++;
                    else if (!freshA && !freshC) bothStale++;
                    else
                    {
                        oneStale++;
                        int sA = curFrame - bv.LastFreshFrame(ia);
                        int sC = curFrame - bv.LastFreshFrame(ic);
                        float len = Vector3.Distance(bv.JointPosition(ia), bv.JointPosition(ic));
                        if (stretchHits.Length < 1500)
                            stretchHits.AppendFormat("  {0}-{1} len={2:F2}m staleA={3}f({4}) staleC={5}f({6})\n",
                                bone.a, bone.b, len, sA, ca, sC, cc);
                    }
                }
                sb.AppendFormat("  bones: bothFresh={0} oneStale={1} bothStale={2} notDrawn={3}\n{4}",
                    both, oneStale, bothStale, notDrawn, stretchHits);
            }
            return sb.ToString();
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
            ShowTrails = showTrails,
            TrailDuration = trailDuration,
            TrailWidth = trailWidth,
            BoneWidth = boneWidth,
            TrailColorMode = trailColorMode,
            TrailFlatColor = trailFlatColor,
            FrameHue = new BodyTrackingShared.FrameHueParams
            {
                CenterHue = frameHueCenter,
                HueRange = frameHueRange,
                Saturation = frameHueSaturation,
                Value = frameHueValue,
                CyclePeriodFrames = frameHueCyclePeriodFrames,
            },
            MaxBodies = maxBodies,
            UseOneEuroFilter = useOneEuroFilter,
            OneEuroMinCutoff = oneEuroMinCutoff,
            OneEuroBeta = oneEuroBeta,
            OneEuroDerivCutoff = oneEuroDerivCutoff,
            AccelMin = accelMin,
            AccelMax = accelMax,
            AccelHotColor = accelHotColor,
            BoneTrailStep = boneTrailStep,
        };

        // --- per-cluster merge (joint-by-joint) ---

        private void BuildMergedSkeleton(Cluster cluster, ref k4abt_skeleton_t output)
        {
            for (int j = 0; j < K4ABTConsts.K4ABT_JOINT_COUNT; j++)
            {
                MergeJoint(cluster, j, ref output.Joints[j]);
            }
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
            // Pre-pass: peak confidence across cameras for this joint. If any camera
            // is MED+ and dropLowWhenHigherAvailable is on, raise the floor so LOW
            // contributions (typically k4abt body-model inferences for an occluded
            // joint, off by ~0.5m from the visible camera) are excluded entirely.
            int peakConf = 0;
            for (int mp = 0; mp < cluster.MemberIndices.Count; mp++)
            {
                int lvl = (int)_candidatePool[cluster.MemberIndices[mp]]
                    .Slot.Bodies[_candidatePool[cluster.MemberIndices[mp]].BodyIndex]
                    .Joints[jointIndex].ConfidenceLevel;
                if (lvl > peakConf) peakConf = lvl;
            }
            int minAcceptLevel = (dropLowWhenHigherAvailable
                && peakConf >= (int)k4abt_joint_confidence_level_t.K4ABT_JOINT_CONFIDENCE_MEDIUM)
                ? (int)k4abt_joint_confidence_level_t.K4ABT_JOINT_CONFIDENCE_MEDIUM
                : (int)k4abt_joint_confidence_level_t.K4ABT_JOINT_CONFIDENCE_LOW;

            // Pass 1: find the highest-confidence sample to use as the
            // hemisphere reference for the quaternion mean. Also accumulate
            // weighted positions in this same pass so we touch each member once.
            float refConf = -1f;
            Quaternion refRot = Quaternion.identity;

            float wSum = 0f;
            Vector3 posSum = Vector3.zero;
            int maxConf = 0;
            int sampleCount = 0;

            for (int m = 0; m < cluster.MemberIndices.Count; m++)
            {
                var cand = _candidatePool[cluster.MemberIndices[m]];
                var jt = cand.Slot.Bodies[cand.BodyIndex].Joints[jointIndex];
                int level = (int)jt.ConfidenceLevel;
                if (level < minAcceptLevel) continue; // NONE always excluded; LOW conditionally excluded

                float weight = WeightFor(level);
                Vector3 worldPos = SkeletonWorldTransform.ToWorld(jt.Position, cand.Slot.DepthToColorMm, cand.Slot.SourceTransform);
                Quaternion worldRot = SkeletonWorldTransform.ToWorldRotation(jt.Orientation, cand.Slot.DepthToColorRotUnity, cand.Slot.SourceTransform);

                wSum += weight;
                posSum += worldPos * weight;
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
                outJoint.Position = default;
                outJoint.Orientation = default;
                outJoint.ConfidenceLevel = k4abt_joint_confidence_level_t.K4ABT_JOINT_CONFIDENCE_NONE;
                return;
            }

            Vector3 mergedPosWorld = posSum / wSum;

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
                if (level < minAcceptLevel) continue;

                float weight = WeightFor(level);
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
            // placing the joint at the desired world position. K4AmmToUnity is
            //   (x, y, z) mm → (x*0.001, -y*0.001, z*0.001) m
            // so we invert: jointMm = (worldX*1000, -worldY*1000, worldZ*1000).
            outJoint.Position = new k4a_float3_t
            {
                X = mergedPosWorld.x * 1000f,
                Y = -mergedPosWorld.y * 1000f,
                Z = mergedPosWorld.z * 1000f,
            };
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

        private float WeightFor(int level)
        {
            return weightStrategy == WeightStrategy.Squared ? level * level : level;
        }

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
                cfg.TrailWidth = baseCfg.TrailWidth * scale;
                pool.Apply((uint)cand.BodyIndex, in _rawScratchSkel, cfg, _trailNow);
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
                    Position = new k4a_float3_t
                    {
                        X = worldPos.x * 1000f,
                        Y = -worldPos.y * 1000f,
                        Z = worldPos.z * 1000f,
                    },
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
