// Replays a recorded depth track through the body tracker and accumulates per-joint
// trajectories so MotionLineRenderer can draw them as static lines once Read finishes.
//
// Per the issue spec ("ボーンのレコードはせず、ポイントクラウド再生で再計算"), no bone
// data is stored on disk — Phase C/D recordings contain raw Y16 depth + RGB8 color +
// per-device calibration, and we recompute the skeleton offline by feeding those depth
// frames straight back into k4abt. The processing runs as a coroutine that yields each
// frame so the Editor stays responsive on multi-thousand-frame recordings.

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
        [Tooltip("Recorder providing raw depth tracks. Auto-found if left blank.")]
        public PointCloudRecorder recorder;

        [Tooltip("Limit BT to one device's recording. Empty = use first track found.")]
        public string deviceSerialFilter = "";

        [Header("Tracker")]
        public k4abt_tracker_processing_mode_t processingMode =
            k4abt_tracker_processing_mode_t.K4ABT_TRACKER_PROCESSING_MODE_GPU_DIRECTML;

        [Tooltip("Temporal smoothing applied during the offline pass.")]
        [Range(0f, 1f)] public float smoothing = 0.0f;

        [Tooltip("Auto-process depth tracks immediately after Read fires OnTracksLoaded.")]
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

        private void Awake()
        {
            BodyTrackingBootstrap.Initialize();
        }

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
            Debug.Log($"[BodyTrackingPlayback] OnTracksLoaded fired, autoProcessOnRead={autoProcessOnRead}", this);
            if (autoProcessOnRead) Process();
        }

        [ContextMenu("Process recorded depth")]
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
            var tracks = recorder.GetRecordedDepthTracks();
            PointCloudRecorder.RecordedDepthTrack pick = null;
            foreach (var t in tracks)
            {
                if (t.DepthFrames == null || t.DepthFrames.Count == 0) continue;
                if (string.IsNullOrEmpty(deviceSerialFilter) || t.Serial == deviceSerialFilter)
                {
                    pick = t; break;
                }
            }
            if (pick == null)
            {
                ProcessingStatus = "no recorded depth track to process";
                Debug.LogWarning("[BodyTrackingPlayback] " + ProcessingStatus, this);
                _processing = null;
                yield break;
            }
            if (!pick.CameraParam.HasValue)
            {
                ProcessingStatus = $"track {pick.Serial} has no calibration; cannot build k4a_calibration_t";
                Debug.LogWarning("[BodyTrackingPlayback] " + ProcessingStatus, this);
                _processing = null;
                yield break;
            }
            Debug.Log($"[BodyTrackingPlayback] processing serial={pick.Serial} depthFrames={pick.DepthFrames.Count} dW={pick.DepthWidth} dH={pick.DepthHeight}", this);

            int dW = pick.DepthWidth > 0 ? pick.DepthWidth : 640;
            int dH = pick.DepthHeight > 0 ? pick.DepthHeight : 576;
            int cW = pick.ColorWidth > 0 ? pick.ColorWidth : 1280;
            int cH = pick.ColorHeight > 0 ? pick.ColorHeight : 720;

            IntPtr calibration = K4ACalibration.Build(pick.CameraParam.Value, dW, dH, cW, cH);
            IntPtr tracker = IntPtr.Zero;
            try
            {
                var cfg = new k4abt_tracker_configuration_t
                {
                    SensorOrientation = k4abt_sensor_orientation_t.K4ABT_SENSOR_ORIENTATION_DEFAULT,
                    ProcessingMode = processingMode,
                    GpuDeviceId = 0,
                    ModelPath = null,
                };
                if (K4ABTNative.k4abt_tracker_create(calibration, cfg, out tracker)
                    != k4a_result_t.K4A_RESULT_SUCCEEDED)
                {
                    ProcessingStatus = "k4abt_tracker_create failed";
                    Debug.LogError("[BodyTrackingPlayback] " + ProcessingStatus, this);
                    _processing = null;
                    yield break;
                }
                K4ABTNative.k4abt_tracker_set_temporal_smoothing(tracker, smoothing);

                int total = pick.DepthFrames.Count;
                ulong t0 = total > 0 ? pick.DepthFrames[0].TimestampNs : 0UL;
                int processed = 0;

                for (int i = 0; i < total; i++)
                {
                    var f = pick.DepthFrames[i];
                    if (f == null || f.Bytes == null || f.Bytes.Length == 0) continue;

                    // Convert ns → us for k4a's device timestamp.
                    ulong tsUsec = f.TimestampNs / 1000UL;

                    IntPtr capture = K4ACaptureBridge.CreateCaptureFromDepthY16(
                        f.Bytes, f.Bytes.Length, dW, dH, tsUsec);
                    if (capture == IntPtr.Zero) continue;

                    // Offline path: block until enqueued, then block until popped. There's no
                    // live producer competing for the queue so this is just synchronous I/O.
                    K4ABTNative.k4abt_tracker_enqueue_capture(tracker, capture, K4ABTConsts.K4A_WAIT_INFINITE);
                    K4ANative.k4a_capture_release(capture);

                    if (K4ABTNative.k4abt_tracker_pop_result(tracker, out IntPtr bodyFrame,
                            K4ABTConsts.K4A_WAIT_INFINITE) == k4a_wait_result_t.K4A_WAIT_RESULT_SUCCEEDED)
                    {
                        try
                        {
                            double tSec = (f.TimestampNs - t0) * 1e-9;
                            uint nBodies = K4ABTNative.k4abt_frame_get_num_bodies(bodyFrame);
                            for (uint b = 0; b < nBodies; b++)
                            {
                                uint id = K4ABTNative.k4abt_frame_get_body_id(bodyFrame, b);
                                if (id == K4ABTConsts.K4ABT_INVALID_BODY_ID) continue;
                                if (K4ABTNative.k4abt_frame_get_body_skeleton(bodyFrame, b, out var skel)
                                    != k4a_result_t.K4A_RESULT_SUCCEEDED) continue;

                                for (int j = 0; j < K4ABTConsts.K4ABT_JOINT_COUNT; j++)
                                {
                                    var joint = skel.Joints[j];
                                    if (joint.ConfidenceLevel < k4abt_joint_confidence_level_t.K4ABT_JOINT_CONFIDENCE_LOW) continue;
                                    var pos = BodyTrackingLive.K4AmmToUnity(joint.Position);
                                    AppendSample(id, (k4abt_joint_id_t)j, tSec, pos, joint.ConfidenceLevel);
                                }
                            }
                        }
                        finally
                        {
                            K4ABTNative.k4abt_frame_release(bodyFrame);
                        }
                    }
                    processed++;

                    // Yield every few frames so the Editor stays responsive and progress
                    // text updates without dragging frame rate down to 0.
                    if ((processed & 7) == 0)
                    {
                        ProcessingStatus = $"BT recompute: {processed} / {total}";
                        yield return null;
                    }
                }

                ProcessingStatus = $"done: {_trajectories.Count} trajectories from {processed} frames";
                Debug.Log("[BodyTrackingPlayback] " + ProcessingStatus, this);
            }
            finally
            {
                if (tracker != IntPtr.Zero)
                {
                    K4ABTNative.k4abt_tracker_shutdown(tracker);
                    K4ABTNative.k4abt_tracker_destroy(tracker);
                }
                K4ACalibration.Free(calibration);
                _processing = null;
            }

            try { OnTrajectoriesReady?.Invoke(); }
            catch (Exception e) { Debug.LogException(e, this); }
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
