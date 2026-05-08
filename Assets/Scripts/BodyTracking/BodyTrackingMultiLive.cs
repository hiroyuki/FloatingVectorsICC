// Multi-camera body tracking via the per-camera k4abt worker fleet from
// issue #10. Spawns one worker per PointCloudRenderer in the cameraManager,
// collects skeletons from each, transforms them to world space using the
// renderer's extrinsic-applied transform, then (in Phase 3) clusters and
// merges them into a single set of BodyVisuals.
//
// Phase 2 deliverable: lifecycle + mutual exclusion with BodyTrackingLive +
// per-serial snapshot pool. No visual update yet — Phase 3 adds clustering,
// joint merge, and the BodyVisual write path. Phase 5a/5b refactor BodyVisual
// out of BodyTrackingLive so MultiLive can share it without duplication.

using System.Collections.Generic;
using BodyTracking.MultiCam;
using BodyTracking.Shared;
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
            public PointCloudRenderer Renderer;
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

        // Diagnostic counters (Phase 3 expands these).
        private int _diagSnapshotsRecv;
        private int _diagDroppedStaleSnapshots;
        private float _diagWindowStart;

        private void OnEnable()
        {
            if (!ResolveDependencies()) { _disabledByGuard = true; enabled = false; return; }
            if (!CheckMutualExclusion()) { _disabledByGuard = true; enabled = false; return; }

            if (workerHost != null && !_hostSubscribed)
            {
                workerHost.OnSkeletonsReady += OnWorkerSkeletons;
                _hostSubscribed = true;
            }
        }

        private void OnDisable()
        {
            if (workerHost != null && _hostSubscribed)
            {
                workerHost.OnSkeletonsReady -= OnWorkerSkeletons;
                _hostSubscribed = false;
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
            _boundRenderers.Clear();
        }

        private void OnDestroy() => OnDisable();
        private void OnApplicationQuit() => OnDisable();

        private void Update()
        {
            // Late-binding for renderers spawned mid-Play by PointCloudCameraManager.
            BindNewRenderers();

            if (diagnosticLogging) PerSecondDiag();

            // Phase 3 will pull snapshots from _latestBySerial here, do world-space
            // clustering + joint merge, and update BodyVisuals.
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
            if (!showSkeleton) return;
            if (workerHost == null) return;

            string serial = string.IsNullOrEmpty(src.deviceSerial) ? src.gameObject.name : src.deviceSerial;
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

                if (!src.CameraParam.HasValue) return; // wait until CameraParam is populated
                int irW = frame.IRWidth > 0 ? frame.IRWidth : frame.DepthWidth;
                int irH = frame.IRHeight > 0 ? frame.IRHeight : frame.DepthHeight;
                if (!workerHost.StartWorker(serial, src.CameraParam.Value,
                        frame.DepthWidth, frame.DepthHeight, irW, irH,
                        frame.ColorWidth, frame.ColorHeight))
                {
                    Debug.LogError($"[BodyTrackingMultiLive] StartWorker failed for serial='{serial}'", this);
                    return;
                }
                var slot = new WorkerLatest { Renderer = src, Serial = serial };
                _latestBySerial[serial] = slot;
            }

            if (!workerHost.IsReady(serial)) return;

            // Forward depth + IR to the worker for inference. Reuse caller's byte arrays
            // (the worker does its own seq-lock copy into the MMF, and PointCloudRenderer
            // pools these buffers, so we don't need an extra copy here).
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
                $"bodies_now={totalBodiesNow}",
                this);
            _diagSnapshotsRecv = 0;
            _diagDroppedStaleSnapshots = 0;
            _diagWindowStart = now;
        }
    }
}
