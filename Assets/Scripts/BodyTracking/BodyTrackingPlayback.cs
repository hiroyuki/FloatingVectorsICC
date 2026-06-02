// Builds per-joint trajectories from a PointCloudRecorder's loaded body track
// (bodies_main RCSV). MotionLineRenderer consumes these to draw the static line
// meshes after Read finishes.
//
// Spec change (issue: Mac-playback): BT results are now recorded alongside the
// raw sensor streams (bodies_main), so playback no longer needs k4abt at all —
// this script just decodes the saved BodySnapshot bytes and folds them into
// JointTrajectory lists. The previous k4abt-recompute path (Phase C/D offline
// pass) was Windows-only and broke the moment the user moved their dataset to a
// Mac for review.

using System;
using System.Collections;
using System.Collections.Generic;
using PointCloud;
using UnityEngine;

namespace BodyTracking
{
    public class BodyTrackingPlayback : MonoBehaviour
    {
        [Header("Source")]
        [Tooltip("Recorder providing recorded body tracks. Auto-found if left blank.")]
        public PointCloudRecorder recorder;

        [Tooltip("Limit trajectories to one device's recording. Empty = include every loaded track that has bodies.")]
        public string deviceSerialFilter = "";

        [Tooltip("Run the trajectory build immediately after Read fires OnTracksLoaded. " +
                 "Safe to leave on now that the build path no longer touches k4abt — it's " +
                 "a pure decode of bodies_main into managed lists, no native calls.")]
        public bool autoProcessOnRead = true;

        // --- output ---

        /// <summary>Time-ordered position samples for one (body id, joint id) pair, in
        /// the local Unity space of <see cref="BodyTrackingPlayback"/>'s GameObject (m).</summary>
        public sealed class JointTrajectory
        {
            public uint BodyId;
            public k4abt_joint_id_t JointId;
            public readonly List<TrajectorySample> Samples = new List<TrajectorySample>();
        }

        public readonly struct TrajectorySample
        {
            public readonly double TimeSec;
            public readonly Vector3 Position;
            public readonly k4abt_joint_confidence_level_t Confidence;
            public TrajectorySample(double t, Vector3 p, k4abt_joint_confidence_level_t c)
            { TimeSec = t; Position = p; Confidence = c; }
        }

        public IReadOnlyList<JointTrajectory> Trajectories => _trajectories;
        public string ProcessingStatus { get; private set; } = "";
        public bool IsProcessing => _processing != null;

        public event Action OnTrajectoriesReady;

        // --- state ---

        private readonly List<JointTrajectory> _trajectories = new List<JointTrajectory>();
        private readonly Dictionary<(uint, k4abt_joint_id_t), JointTrajectory> _byKey =
            new Dictionary<(uint, k4abt_joint_id_t), JointTrajectory>();
        private Coroutine _processing;

        // Reused decode buffer — MaxBodies snapshots, all joint arrays pre-allocated.
        // Sized once at first use so the decode loop never re-allocates.
        private BodySnapshot[] _decodeBuffer;

        private void OnEnable()
        {
            if (recorder == null) recorder = FindFirstObjectByType<PointCloudRecorder>();
            if (recorder != null) recorder.OnTracksLoaded += HandleTracksLoaded;
        }

        private void OnDisable()
        {
            if (recorder != null) recorder.OnTracksLoaded -= HandleTracksLoaded;
            if (_processing != null) { StopCoroutine(_processing); _processing = null; }
        }

        private void HandleTracksLoaded()
        {
            if (autoProcessOnRead) Process();
        }

        [ContextMenu("Build trajectories from recorded bodies")]
        public void Process()
        {
            if (_processing != null)
            {
                Debug.LogWarning("[BodyTrackingPlayback] processing already in progress.");
                return;
            }
            if (recorder == null)
            {
                Debug.LogWarning("[BodyTrackingPlayback] no PointCloudRecorder bound; nothing to process.");
                return;
            }
            _processing = StartCoroutine(ProcessCoroutine());
        }

        private IEnumerator ProcessCoroutine()
        {
            ClearTrajectories();

            if (_decodeBuffer == null)
            {
                _decodeBuffer = new BodySnapshot[Shared.K4abtWorkerSharedLayout.MaxBodies];
                for (int i = 0; i < _decodeBuffer.Length; i++) _decodeBuffer[i] = new BodySnapshot();
            }

            int tracksProcessed = 0;
            int framesProcessed = 0;
            int samplesAppended = 0;

            var tracks = recorder.GetRecordedDepthTracks();
            // Choose origin for the relative TimeSec axis: earliest body-frame ts
            // across the picked tracks. This keeps TimeSec close to 0 at the start
            // of the recording regardless of the absolute device timer.
            ulong t0 = ulong.MaxValue;
            foreach (var t in tracks)
            {
                if (t.BodyFrames == null || t.BodyFrames.Count == 0) continue;
                if (!string.IsNullOrEmpty(deviceSerialFilter) && t.Serial != deviceSerialFilter) continue;
                ulong first = TimestampNsAt(t.BodyFrames, 0);
                if (first < t0) t0 = first;
            }
            if (t0 == ulong.MaxValue)
            {
                ProcessingStatus = "no recorded body frames to process " +
                                   "(record while BodyTrackingMultiLive is feeding K4abtWorkerHost, then Read).";
                Debug.LogWarning("[BodyTrackingPlayback] " + ProcessingStatus, this);
                _processing = null;
                yield break;
            }

            foreach (var t in tracks)
            {
                if (t.BodyFrames == null || t.BodyFrames.Count == 0) continue;
                if (!string.IsNullOrEmpty(deviceSerialFilter) && t.Serial != deviceSerialFilter) continue;
                tracksProcessed++;

                int total = t.BodyFrames.Count;
                for (int i = 0; i < total; i++)
                {
                    var f = t.BodyFrames[i];
                    if (f == null || f.Bytes == null || f.ByteCount == 0) continue;

                    int bodyCount = RecordedBodySerializer.Decode(f.Bytes, f.ByteCount, _decodeBuffer);
                    double tSec = (f.TimestampNs - t0) * 1e-9;

                    for (int b = 0; b < bodyCount; b++)
                    {
                        var body = _decodeBuffer[b];
                        if (body.Id == K4ABTConsts.K4ABT_INVALID_BODY_ID) continue;
                        for (int j = 0; j < K4ABTConsts.K4ABT_JOINT_COUNT; j++)
                        {
                            var joint = body.Joints[j];
                            if (joint.ConfidenceLevel < k4abt_joint_confidence_level_t.K4ABT_JOINT_CONFIDENCE_LOW)
                                continue;
                            // K4AmmToUnity: (x, y, z) mm in k4a-camera-local → (x*0.001, -y*0.001, z*0.001) m.
                            Vector3 pos = BodyTrackingShared.K4AmmToUnity(joint.Position);
                            AppendSample(body.Id, (k4abt_joint_id_t)j, tSec, pos, joint.ConfidenceLevel);
                            samplesAppended++;
                        }
                    }
                    framesProcessed++;

                    // Yield occasionally so the Editor stays responsive on very long recordings.
                    if ((framesProcessed & 31) == 0)
                    {
                        ProcessingStatus = $"BT decode: {framesProcessed} frame(s), {_trajectories.Count} trajectories so far";
                        yield return null;
                    }
                }
            }

            ProcessingStatus = tracksProcessed == 0
                ? "no body tracks matched the device filter"
                : $"done: {_trajectories.Count} trajectories from {framesProcessed} frame(s) " +
                  $"across {tracksProcessed} device(s) ({samplesAppended} samples)";
            _processing = null;

            try { OnTrajectoriesReady?.Invoke(); }
            catch (Exception e) { Debug.LogException(e, this); }
        }

        private static ulong TimestampNsAt(IReadOnlyList<PointCloudRecording.Frame> frames, int idx)
        {
            if (frames is PointCloudRecording.RcsvFrameStream s) return s.TimestampNsAt(idx);
            return frames[idx].TimestampNs;
        }

        private void AppendSample(uint bodyId, k4abt_joint_id_t jointId, double tSec, Vector3 pos,
                                   k4abt_joint_confidence_level_t conf)
        {
            var key = (bodyId, jointId);
            if (!_byKey.TryGetValue(key, out var traj))
            {
                traj = new JointTrajectory { BodyId = bodyId, JointId = jointId };
                _byKey[key] = traj;
                _trajectories.Add(traj);
            }
            traj.Samples.Add(new TrajectorySample(tSec, pos, conf));
        }

        public void ClearTrajectories()
        {
            _trajectories.Clear();
            _byKey.Clear();
            ProcessingStatus = "";
        }
    }
}
