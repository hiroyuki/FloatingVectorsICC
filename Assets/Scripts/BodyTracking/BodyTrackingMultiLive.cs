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
    public class BodyTrackingMultiLive : MonoBehaviour
    {
        [Header("Sources")]
        [Tooltip("PointCloudCameraManager whose Renderers we drive workers off. " +
                 "Spawned at Play time; this script binds when the list becomes non-empty.")]
        public PointCloudCameraManager cameraManager;

        [Tooltip("K4abtWorkerHost used to spawn one worker per camera. Auto-found at " +
                 "OnEnable if left null. Required.")]
        public K4abtWorkerHost workerHost;

        [Header("Display")]
        [Tooltip("Master switch for skeleton rendering. Mirror of BodyTrackingLive.showSkeleton " +
                 "so the Inspector workflow is consistent across the two paths.")]
        public bool showSkeleton = true;

        [Tooltip("Show the bone lines between joints. Same toggle as BodyTrackingLive.")]
        public bool showAnatomicalBones = true;

        [Tooltip("Joint marker radius (m).")]
        [Range(0.005f, 0.2f)] public float jointRadius = 0.05f;

        [Tooltip("Skeleton color. Bones inherit this; joints are slightly brighter.")]
        public Color skeletonColor = new Color(0.2f, 0.9f, 1f, 1f);

        [Header("Trails")]
        [Tooltip("Attach a TrailRenderer to each merged joint so motion leaves a line in real time.")]
        public bool showTrails = true;

        [Tooltip("PerJointHue / FlatColor — same semantics as BodyTrackingLive.")]
        public BodyTrackingLive.TrailColorMode trailColorMode = BodyTrackingLive.TrailColorMode.PerJointHue;

        [Tooltip("Trail color in FlatColor mode (also tints PerJointHue).")]
        public Color trailFlatColor = Color.white;

        [Min(0.05f)]
        [Tooltip("How long (s) each trail segment stays visible before fading out.")]
        public float trailDuration = 2.0f;

        [Range(0.001f, 0.05f)]
        [Tooltip("Trail width (m) at the head; tail tapers to ~0.")]
        public float trailWidth = 0.005f;

        [Header("Visual lifetime")]
        [Tooltip("Hard cap on cached BodyVisuals. Same role as BodyTrackingLive.maxBodies.")]
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
                 "merge. Snapshots older than this are dropped to avoid temporal ghosting.")]
        [Range(16, 200)]
        public int maxSkewMs = 50;

        [Tooltip("Minimum number of worker snapshots required for a cluster to emit a person. " +
                 "Set to 2 to require multi-camera agreement; 1 (default) accepts single-cam " +
                 "detections too — equivalent to BodyTrackingLive in 1-camera scenarios.")]
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

        // Reusable synthetic skeleton handed to BodyVisual.UpdateFromSkeleton.
        // We encode merged world joint positions back into k4a camera-local mm
        // such that K4AmmToUnity (called inside BodyVisual) produces the desired
        // world position. With BodyTrackingMultiLive's own transform at world
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
        private int _diagClustersFormed;
        private int _diagPersonsOutput;
        private int _diagContinuityCarryOver;
        private float _diagWindowStart;

        // Crowd-alert debounce state. Uses Time.realtimeSinceStartup so the
        // debounce works even when Time.timeScale is 0 (Editor Pause).
        private float _multiPersonSince = -1f;
        private float _singlePersonSince = -1f;
        private bool _alertActive;
        private GUIStyle _alertStyleCache;

        private PointCloudRecorder _subscribedRecorder;

        private void OnEnable()
        {
            if (!ResolveDependencies()) { _disabledByGuard = true; enabled = false; return; }
            if (!CheckMutualExclusion()) { _disabledByGuard = true; enabled = false; return; }

            if (_pool == null) _pool = new BodyVisualPool(transform);
            if (workerHost != null && !_hostSubscribed)
            {
                workerHost.OnSkeletonsReady += OnWorkerSkeletons;
                _hostSubscribed = true;
            }

            // Wire recorded-playback source if a PointCloudRecorder is present in the
            // scene. Live and playback can coexist: HandleRawFrame and the
            // OnPlaybackRawFrame handler both funnel into DispatchRawFrame.
            _subscribedRecorder = FindFirstObjectByType<PointCloudRecorder>();
            if (_subscribedRecorder != null)
                _subscribedRecorder.OnPlaybackRawFrame += OnPlaybackRawFrame;
        }

        private void OnPlaybackRawFrame(string serial, ObCameraParam? camParam, Transform sourceTransform, RawFrameData frame)
        {
            HandlePlaybackRawFrame(serial, camParam, sourceTransform, frame);
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

        private void Update()
        {
            // Late-binding for renderers spawned mid-Play by PointCloudCameraManager.
            BindNewRenderers();

            if (showSkeleton)
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
        }

        // --- guards ---

        private bool ResolveDependencies()
        {
            if (cameraManager == null) cameraManager = FindFirstObjectByType<PointCloudCameraManager>();
            if (cameraManager == null)
            {
                Debug.LogError("[BodyTrackingMultiLive] PointCloudCameraManager not found in scene; disabling.", this);
                return false;
            }
            if (workerHost == null) workerHost = FindFirstObjectByType<K4abtWorkerHost>();
            if (workerHost == null)
            {
                Debug.LogError("[BodyTrackingMultiLive] K4abtWorkerHost not found in scene; disabling. " +
                               "Add a K4abtWorkerHost MonoBehaviour and reference it here.", this);
                return false;
            }
            if (!workerHost.useWorker)
            {
                Debug.LogWarning("[BodyTrackingMultiLive] K4abtWorkerHost.useWorker is false; forcing it true.", this);
                workerHost.useWorker = true;
            }
            return true;
        }

        // BodyTrackingLive and BodyTrackingMultiLive cannot coexist: K4abtWorkerHost.StartWorker
        // returns false on duplicate serial, and BodyTrackingLive falls back to in-process
        // tracking on that path — leading to two BT pipelines on the same source.
        private bool CheckMutualExclusion()
        {
            var lives = FindObjectsByType<BodyTrackingLive>(FindObjectsInactive.Exclude, FindObjectsSortMode.None);
            int activeLives = 0;
            foreach (var l in lives) if (l != null && l.enabled) activeLives++;
            if (activeLives > 0)
            {
                Debug.LogError(
                    "[BodyTrackingMultiLive] BodyTrackingLive is also enabled in this scene. " +
                    "The two paths cannot coexist (worker session conflict). Disable one of them. " +
                    "MultiLive disabling itself.", this);
                return false;
            }
            return true;
        }

        // applyExtrinsics is required only when more than one camera is active OR when
        // the user has asked for multi-worker agreement. Single-camera (1 renderer,
        // requireMinWorkerCount==1) tolerates identity transforms and behaves like
        // BodyTrackingLive in that fallback case.
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
            if (!showSkeleton) return;
            if (workerHost == null) return;

            if (!_latestBySerial.ContainsKey(serial))
            {
                if (RequiresApplyExtrinsics() && cameraManager != null && !cameraManager.applyExtrinsics)
                {
                    Debug.LogError(
                        "[BodyTrackingMultiLive] applyExtrinsics is false but multi-camera mode " +
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
                    Debug.LogError($"[BodyTrackingMultiLive] StartWorker failed for serial='{serial}'", this);
                    return;
                }
                var slot = new WorkerLatest { SourceTransform = sourceTransform, Serial = serial };
                _latestBySerial[serial] = slot;
            }
            else
            {
                // Keep the SourceTransform pointer fresh in case the caller swaps it
                // (e.g. playback rebuilds the _Playback_<serial> GO between sessions).
                _latestBySerial[serial].SourceTransform = sourceTransform;
            }

            if (!workerHost.IsReady(serial)) return;

            int depthBytes = frame.DepthByteCount;
            byte[] ir = frame.IRBytes;
            int irBytes = frame.IRByteCount;
            ulong tsNs = frame.TimestampUs * 1000UL;
            workerHost.EnqueueFrame(serial, frame.DepthBytes, depthBytes, ir, irBytes, tsNs);
        }

        // --- per-serial snapshot intake ---

        // Fired synchronously from K4abtWorkerHost.Update (DefaultExecutionOrder(-100))
        // before this script's Update runs. The host buffer is reused per frame, so
        // we copy the relevant state out into our per-serial slot.
        private void OnWorkerSkeletons(string serial, BodySnapshot[] bodies, int count)
        {
            if (!showSkeleton) return;
            if (!_latestBySerial.TryGetValue(serial, out var slot)) return;
            int n = Mathf.Min(count, slot.Bodies.Length);
            for (int i = 0; i < n; i++)
            {
                slot.Bodies[i].Id = bodies[i].Id;
                System.Array.Copy(bodies[i].Joints, slot.Bodies[i].Joints, slot.Bodies[i].Joints.Length);
            }
            slot.BodyCount = n;
            slot.CapturedAtRealtime = Time.realtimeSinceStartup;
            // CapturedTsNs is filled when Phase 3 wires the per-snapshot tsNs through
            // the host's BodySnapshot DTO. v1 falls back to Time.realtimeSinceStartup
            // for staleness gating until BodySnapshot carries tsNs.
            slot.CapturedTsNs = (ulong)(Time.realtimeSinceStartupAsDouble * 1e9);
            _diagSnapshotsRecv += n;
        }

        private void PerSecondDiag()
        {
            float now = Time.realtimeSinceStartup;
            if (_diagWindowStart == 0f) _diagWindowStart = now;
            if (now - _diagWindowStart < 1f) return;
            int boundWorkers = _latestBySerial.Count;
            int totalBodiesNow = 0;
            foreach (var kv in _latestBySerial) totalBodiesNow += kv.Value.BodyCount;
            Debug.Log(
                $"[BodyTrackingMultiLive] workers={boundWorkers} " +
                $"snapshots/s={_diagSnapshotsRecv} dropped_stale/s={_diagDroppedStaleSnapshots} " +
                $"clusters/s={_diagClustersFormed} persons/s={_diagPersonsOutput} " +
                $"continuity_carry_over/s={_diagContinuityCarryOver} " +
                $"alive_visuals={_pool.Count} bodies_now={totalBodiesNow}",
                this);
            _diagSnapshotsRecv = 0;
            _diagDroppedStaleSnapshots = 0;
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

            foreach (var kv in _latestBySerial)
            {
                var slot = kv.Value;
                if (slot == null || slot.BodyCount == 0) continue;
                if (now - slot.CapturedAtRealtime > maxSkewSec)
                {
                    _diagDroppedStaleSnapshots += slot.BodyCount;
                    continue;
                }
                for (int i = 0; i < slot.BodyCount; i++)
                {
                    var body = slot.Bodies[i];
                    var pelvisJoint = body.Joints[kPelvisIdx];
                    if ((int)pelvisJoint.ConfidenceLevel <= 0) continue; // pelvis NONE -> skip body

                    Vector3 pelvisWorld = SkeletonWorldTransform.ToWorld(
                        pelvisJoint.Position,
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
                _pool.Apply(cluster.Id, in _mergedSkel, cfg, OnVisualEvicted);
                _diagPersonsOutput++;
            }
        }

        private void GcStaleVisuals()
        {
            _pool.GcStale(unseenFramesBeforeDestroy, OnVisualEvicted);
        }

        private void OnVisualEvicted(uint id)
        {
            _priorPelvisById.Remove(id);
            _priorMaxConfById.Remove(id);
        }

        private void StashPriorState()
        {
            // Snapshot the current frame's merged persons for next-frame continuity.
            // We stash the seed candidate's world pelvis for each cluster; the seed
            // is the highest-confidence detection in the cluster (or the carry-over
            // seed). Use the seed instead of an averaged pelvis because the seed is
            // the closest analogue to the last-frame "I saw this person here" datum.
            // Old entries fade through the GC pass above (when their visual gets
            // destroyed), so this dictionary is bounded by maxBodies.
            _priorPelvisById.Clear();
            _priorMaxConfById.Clear();
            for (int c = 0; c < _clusterCount; c++)
            {
                var cluster = _clusterPool[c];
                if (cluster.MemberIndices.Count < requireMinWorkerCount) continue;
                _priorPelvisById[cluster.Id] = cluster.Seed.PelvisWorld;
                _priorMaxConfById[cluster.Id] = cluster.Seed.MaxConfidence;
            }
        }

        private BodyVisualConfig BuildVisualConfig() => new BodyVisualConfig
        {
            JointRadius = jointRadius,
            SkeletonColor = skeletonColor,
            ShowAnatomicalBones = showAnatomicalBones,
            ShowTrails = showTrails,
            TrailDuration = trailDuration,
            TrailWidth = trailWidth,
            TrailColorMode = trailColorMode,
            TrailFlatColor = trailFlatColor,
            MaxBodies = maxBodies,
        };

        // --- per-cluster merge (joint-by-joint) ---

        private void BuildMergedSkeleton(Cluster cluster, ref k4abt_skeleton_t output)
        {
            for (int j = 0; j < K4ABTConsts.K4ABT_JOINT_COUNT; j++)
            {
                MergeJoint(cluster, j, ref output.Joints[j]);
            }
        }

        private void MergeJoint(Cluster cluster, int jointIndex, ref k4abt_joint_t outJoint)
        {
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
                if (level <= 0) continue; // NONE excluded from average

                float weight = WeightFor(level);
                Vector3 worldPos = SkeletonWorldTransform.ToWorld(jt.Position, cand.Slot.SourceTransform);
                Quaternion worldRot = SkeletonWorldTransform.ToWorldRotation(jt.Orientation, cand.Slot.SourceTransform);

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

            // Pass 2: hemisphere-aligned weighted quaternion sum.
            Vector4 qSum = Vector4.zero;
            for (int m = 0; m < cluster.MemberIndices.Count; m++)
            {
                var cand = _candidatePool[cluster.MemberIndices[m]];
                var jt = cand.Slot.Bodies[cand.BodyIndex].Joints[jointIndex];
                int level = (int)jt.ConfidenceLevel;
                if (level <= 0) continue;

                float weight = WeightFor(level);
                Quaternion qLocal = SkeletonWorldTransform.ToWorldRotation(jt.Orientation, cand.Slot.SourceTransform);
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
                pool.Apply((uint)cand.BodyIndex, in _rawScratchSkel, cfg);
            }
            // GC stale per-worker visuals on the same threshold as merged.
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
            for (int j = 0; j < K4ABTConsts.K4ABT_JOINT_COUNT; j++)
            {
                var jt = body.Joints[j];
                if ((int)jt.ConfidenceLevel <= 0)
                {
                    // Keep BodyVisual on its previous position for this joint.
                    output.Joints[j] = new k4abt_joint_t { ConfidenceLevel = k4abt_joint_confidence_level_t.K4ABT_JOINT_CONFIDENCE_NONE };
                    continue;
                }
                Vector3 worldPos = SkeletonWorldTransform.ToWorld(jt.Position, rendererT);
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
