// Full-joint LIVE skeleton source for the experience flow's pose detection
// (star-pose calibration, banzai trigger). SkeletonMerger.PersonSample exposes
// only pelvis + hands, and — more importantly — during visitor playback the
// merger neither feeds the k4abt workers with live frames (HandleRawFrame
// returns early while the recorder is Playing a take that contains the live
// serials) nor wants their output (the recorded bodies_main drives the merge).
// This component fills exactly that gap:
//
//   Reading: always subscribed to K4abtWorkerHost.OnSkeletonsReady. Whatever
//   feeds the workers (the merger during live states, us during visitor
//   playback), the freshest in-volume body per serial is converted to world
//   space and kept for TryGetBestSkeleton.
//
//   Feeding: subscribes to live PointCloudRenderer.OnRawFramesReady and
//   enqueues frames ONLY in the exact complement of the merger's ownership:
//   recorder Playing AND the take has this serial AND the merger consumes the
//   recorded bodies (ignoreRecordedBodies false). During live states the
//   merger feeds; during attract (ignoreRecordedBodies true) the merger feeds
//   playback frames and we must not interleave live-clock frames into the
//   same worker. One feeder per worker at any moment, by construction.
//
//   Ingest gating mirrors the same rule: while the attract ghost occupies the
//   workers, their output describes the RECORDING, not the live visitor, and
//   is skipped here.
//
// Degrades gracefully on machines without the BT SDK: StartWorker fails, we
// log once per serial and retry on a slow cadence; TryGetBestSkeleton simply
// reports nobody.

using System.Collections.Generic;
using BodyTracking;
using BodyTracking.MultiCam;
using Orbbec;
using PointCloud;
using UnityEngine;

namespace Experience
{
    [DisallowMultipleComponent]
    public class LiveSkeletonFeed : MonoBehaviour
    {
        [Tooltip("Worker host shared with SkeletonMerger. Auto-resolves when empty.")]
        public K4abtWorkerHost workerHost;

        [Tooltip("Merged-skeleton owner — read for worker-ownership state " +
                 "(ignoreRecordedBodies), never for joints. Auto-resolves when empty.")]
        public SkeletonMerger merger;

        [Tooltip("Live rig whose renderers we read frames from. Auto-resolves when empty.")]
        public SensorManager sensorManager;

        [Tooltip("Recorder whose playback state decides worker ownership. Auto-resolves when empty.")]
        public SensorRecorder recorder;

        [Tooltip("Bodies whose pelvis is outside this OBB (+margin) are ignored. " +
                 "Auto-resolves when empty; null = no gate.")]
        public BoundingVolume trackingVolume;

        [Min(0f)]
        [Tooltip("Outward slack (m) on the volume gate, matching SkeletonMerger's convention.")]
        public float volumeGateMarginMeters = 0.25f;

        [Range(1f, 30f)]
        [Tooltip("Cap on frames/second sent to each worker while WE feed (visitor playback). " +
                 "Pose detection doesn't need full sensor rate, and the GPU is busy rendering " +
                 "the playback sculpture.")]
        public float liveFeedFps = 15f;

        [Min(0.1f)]
        [Tooltip("A serial's skeleton older than this (s) no longer counts as a person.")]
        public float staleSeconds = 0.5f;

        [Min(1f)]
        [Tooltip("Seconds between StartWorker retries after a spawn failure (no BT SDK / crash).")]
        public float spawnRetrySeconds = 5f;

        // ---- read API ----

        /// <summary>Any serial delivered an in-volume skeleton within staleSeconds.</summary>
        public bool HasRecentPerson
        {
            get
            {
                float now = Time.realtimeSinceStartup;
                foreach (var kv in _bySerial)
                    if (kv.Value.ReceivedAtRealtime >= 0f
                        && now - kv.Value.ReceivedAtRealtime <= staleSeconds)
                        return true;
                return false;
            }
        }

        /// <summary>
        /// Best fresh live skeleton across cameras (highest joint-confidence sum).
        /// The returned arrays are internal buffers overwritten on the next worker
        /// event — read immediately, copy if you must retain. World-space, K4ABT
        /// joint order; <paramref name="jointValid"/> is confidence != NONE.
        /// </summary>
        public bool TryGetBestSkeleton(out Vector3[] jointsWorld, out bool[] jointValid)
        {
            float now = Time.realtimeSinceStartup;
            SerialState best = null;
            foreach (var kv in _bySerial)
            {
                var s = kv.Value;
                if (s.ReceivedAtRealtime < 0f || now - s.ReceivedAtRealtime > staleSeconds) continue;
                if (best == null || s.ConfidenceSum > best.ConfidenceSum) best = s;
            }
            if (best == null)
            {
                jointsWorld = null;
                jointValid = null;
                return false;
            }
            jointsWorld = best.JointsWorld;
            jointValid = best.JointValid;
            return true;
        }

        // ---- state ----

        private sealed class SerialState
        {
            public Transform SourceTransform;
            public Matrix4x4 DepthToColorMm = Matrix4x4.identity;
            public readonly Vector3[] JointsWorld = new Vector3[K4ABTConsts.K4ABT_JOINT_COUNT];
            public readonly bool[] JointValid = new bool[K4ABTConsts.K4ABT_JOINT_COUNT];
            public int ConfidenceSum;
            public float ReceivedAtRealtime = -1f;
            public float LastEnqueueRealtime = -1f;
            public float NextSpawnRetryRealtime;
            public bool SpawnFailureLogged;
        }

        private readonly Dictionary<string, SerialState> _bySerial = new();
        private readonly HashSet<PointCloudRenderer> _boundRenderers = new();
        private bool _hostSubscribed;

        // ---- lifecycle ----

        private void OnEnable()
        {
            if (workerHost == null) workerHost = FindFirstObjectByType<K4abtWorkerHost>();
            if (merger == null) merger = FindFirstObjectByType<SkeletonMerger>();
            if (sensorManager == null) sensorManager = FindFirstObjectByType<SensorManager>();
            if (recorder == null) recorder = FindFirstObjectByType<SensorRecorder>();
            if (trackingVolume == null) trackingVolume = FindFirstObjectByType<BoundingVolume>();

            if (workerHost != null && !_hostSubscribed)
            {
                workerHost.OnSkeletonsReady += OnWorkerSkeletons;
                _hostSubscribed = true;
            }
        }

        private void OnDisable()
        {
            if (workerHost != null && _hostSubscribed)
                workerHost.OnSkeletonsReady -= OnWorkerSkeletons;
            _hostSubscribed = false;

            foreach (var r in _boundRenderers)
                if (r != null) r.OnRawFramesReady -= HandleRawFrame;
            _boundRenderers.Clear();
            _bySerial.Clear();
            // Workers are intentionally NOT stopped here: they belong to the
            // merger/host lifecycle; we only borrow the feed during playback.
        }

        private void Update()
        {
            BindNewRenderers();
        }

        private void BindNewRenderers()
        {
            if (sensorManager == null || sensorManager.Renderers == null) return;
            for (int i = 0; i < sensorManager.Renderers.Count; i++)
            {
                var r = sensorManager.Renderers[i];
                if (r == null || _boundRenderers.Contains(r)) continue;
                r.OnRawFramesReady += HandleRawFrame;
                _boundRenderers.Add(r);
            }
        }

        // ---- worker-ownership predicates (must stay the exact complement of
        //      SkeletonMerger.HandleRawFrame's playback gate) ----

        private bool PlaybackHasSerial(string serial) =>
            recorder != null
            && recorder.CurrentState == SensorRecorder.State.Playing
            && recorder.HasTrack(serial);

        /// <summary>True when WE feed the worker for this serial: playback owns the
        /// merger's sculpture via recorded bodies, leaving the worker free for the
        /// live visitor.</summary>
        private bool WeFeedWorker(string serial) =>
            PlaybackHasSerial(serial)
            && merger != null
            && !merger.IgnoreRecordedBodiesActive
            && !merger.useExternalBodies;

        /// <summary>True when worker output currently describes a RECORDING (attract
        /// ghost re-analyzed by k4abt), not the live visitor — skip ingest.</summary>
        private bool WorkerOutputIsPlayback(string serial) =>
            PlaybackHasSerial(serial)
            && (merger == null || merger.IgnoreRecordedBodiesActive);

        // ---- feeding ----

        private void HandleRawFrame(PointCloudRenderer src, RawFrameData frame)
        {
            if (src.holdLiveFrame) return;
            string serial = string.IsNullOrEmpty(src.deviceSerial) ? src.gameObject.name : src.deviceSerial;

            if (!_bySerial.TryGetValue(serial, out var state))
            {
                state = new SerialState();
                _bySerial[serial] = state;
            }
            // Keep transform + depth→color extrinsic fresh on every live frame so
            // world conversion is correct no matter who feeds the worker.
            state.SourceTransform = src.transform;
            SetExtrinsic(state, src.CameraParam);

            if (!WeFeedWorker(serial)) return;
            if (workerHost == null) return;

            float now = Time.realtimeSinceStartup;
            if (!workerHost.HasSession(serial))
            {
                // Session usually survives from the live phase. If it's gone (crash,
                // no BT SDK on this machine), respawn on a slow cadence.
                if (now < state.NextSpawnRetryRealtime) return;
                state.NextSpawnRetryRealtime = now + spawnRetrySeconds;
                if (!src.CameraParam.HasValue) return;
                int irW = frame.IRWidth > 0 ? frame.IRWidth : frame.DepthWidth;
                int irH = frame.IRHeight > 0 ? frame.IRHeight : frame.DepthHeight;
                if (!workerHost.StartWorker(serial, src.CameraParam.Value,
                        frame.DepthWidth, frame.DepthHeight, irW, irH,
                        frame.ColorWidth, frame.ColorHeight))
                {
                    if (!state.SpawnFailureLogged)
                    {
                        Debug.LogWarning(
                            $"[{nameof(LiveSkeletonFeed)}] StartWorker failed for serial='{serial}' — " +
                            "live pose detection unavailable (no BT SDK on this machine?). " +
                            $"Retrying every {spawnRetrySeconds:0}s.", this);
                        state.SpawnFailureLogged = true;
                    }
                    return;
                }
                state.SpawnFailureLogged = false;
            }

            if (!workerHost.IsReady(serial)) return;

            if (liveFeedFps > 0f
                && state.LastEnqueueRealtime >= 0f
                && now - state.LastEnqueueRealtime < 1f / liveFeedFps)
                return;
            state.LastEnqueueRealtime = now;

            workerHost.EnqueueFrame(serial, frame.DepthBytes, frame.DepthByteCount,
                                    frame.IRBytes, frame.IRByteCount,
                                    frame.TimestampUs * 1000UL);
        }

        // Same recipe as SkeletonMerger.SetSlotExtrinsic (position part only —
        // pose classification never needs joint orientations).
        private static void SetExtrinsic(SerialState state, ObCameraParam? cameraParam)
        {
            if (!cameraParam.HasValue) return;
            var e = cameraParam.Value.Transform;
            if (e.Rot == null || e.Rot.Length < 9 || e.Trans == null || e.Trans.Length < 3) return;
            state.DepthToColorMm = e.ToMatrixMm();
        }

        // ---- ingest ----

        private void OnWorkerSkeletons(string serial, ulong tsNs, BodySnapshot[] bodies, int count)
        {
            if (!_bySerial.TryGetValue(serial, out var state)) return; // not a live rig serial
            if (WorkerOutputIsPlayback(serial)) return; // attract ghost, not the visitor

            // Pick the highest-confidence in-volume body of this event. The host
            // buffer is reused per frame, so convert/copy inside the handler.
            int bestIdx = -1, bestConf = -1;
            for (int i = 0; i < count; i++)
            {
                var joints = bodies[i].Joints;
                Vector3 pelvis = SkeletonWorldTransform.ToWorld(
                    joints[(int)k4abt_joint_id_t.K4ABT_JOINT_PELVIS].Position,
                    state.DepthToColorMm, state.SourceTransform);
                if (!InsideVolume(pelvis)) continue;
                int conf = 0;
                for (int j = 0; j < joints.Length; j++) conf += (int)joints[j].ConfidenceLevel;
                if (conf > bestConf) { bestConf = conf; bestIdx = i; }
            }
            if (bestIdx < 0) return; // nobody in the volume — freshness decays naturally

            var best = bodies[bestIdx].Joints;
            for (int j = 0; j < K4ABTConsts.K4ABT_JOINT_COUNT && j < best.Length; j++)
            {
                state.JointsWorld[j] = SkeletonWorldTransform.ToWorld(
                    best[j].Position, state.DepthToColorMm, state.SourceTransform);
                state.JointValid[j] = best[j].ConfidenceLevel
                    != k4abt_joint_confidence_level_t.K4ABT_JOINT_CONFIDENCE_NONE;
            }
            state.ConfidenceSum = bestConf;
            state.ReceivedAtRealtime = Time.realtimeSinceStartup;
        }

        // OBB-with-margin test, same convention as SkeletonMerger's volume gate.
        private bool InsideVolume(Vector3 world)
        {
            if (trackingVolume == null) return true;
            var t = trackingVolume.transform;
            Vector3 b = t.InverseTransformPoint(world);
            Vector3 s = t.lossyScale;
            float mx = 0.5f + (s.x != 0f ? volumeGateMarginMeters / Mathf.Abs(s.x) : 0f);
            float my = 0.5f + (s.y != 0f ? volumeGateMarginMeters / Mathf.Abs(s.y) : 0f);
            float mz = 0.5f + (s.z != 0f ? volumeGateMarginMeters / Mathf.Abs(s.z) : 0f);
            return Mathf.Abs(b.x) <= mx && Mathf.Abs(b.y) <= my && Mathf.Abs(b.z) <= mz;
        }
    }
}
