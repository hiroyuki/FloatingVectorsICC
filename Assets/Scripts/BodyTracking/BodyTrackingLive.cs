// Live skeleton overlay. Subscribes to the designated PointCloudRenderer's
// raw-depth stream, feeds each frame into k4abt, polls body frames, and
// updates a small joint+bone visualization on top of the point cloud.
//
// Single-device by design: per the issue spec ("multi device は skip"), one
// PointCloudRenderer (the "BT source") drives one tracker. Other devices
// keep rendering their point clouds but don't contribute to body tracking.

using System.Collections.Generic;
using PointCloud;
using UnityEngine;

namespace BodyTracking
{
    public class BodyTrackingLive : MonoBehaviour
    {
        [Header("Source")]
        [Tooltip("PointCloudCameraManager that spawns per-device PointCloudRenderers at runtime. " +
                 "Drag your existing 'PointClouds' GameObject here so this component can pick up " +
                 "the renderer once it's spawned (Edit-time D&D into the source field below isn't " +
                 "possible because renderers don't exist until Play).")]
        public PointCloudCameraManager cameraManager;

        [Tooltip("Specific PointCloudRenderer to drive body tracking from. If empty, the first " +
                 "renderer the cameraManager spawns is used. Set this only if you have multiple " +
                 "devices and want to pin BT to one of them.")]
        public PointCloudRenderer source;

        [Header("Display")]
        [Tooltip("Show the skeleton (joints + anatomical bones) on the point cloud. " +
                 "Bones can be hidden separately with showAnatomicalBones below.")]
        public bool showSkeleton = true;

        [Tooltip("Draw the connecting lines between joints (shoulder–elbow–wrist etc). " +
                 "Issue #7 explicitly exposes this as an opt-out toggle.")]
        public bool showAnatomicalBones = true;

        [Tooltip("Joint marker radius in meters.")]
        [Range(0.005f, 0.2f)] public float jointRadius = 0.05f;

        [Tooltip("Skeleton color. Bones inherit this; joints are shown slightly brighter.")]
        public Color skeletonColor = new Color(0.2f, 0.9f, 1f, 1f);

        [Header("Trails")]
        [Tooltip("Attach a TrailRenderer to each joint so its motion leaves a line in real time. " +
                 "Equivalent to the Motion Line view but live, frame by frame.")]
        public bool showTrails = true;

        public enum TrailColorMode { PerJointHue, FlatColor }

        [Tooltip("PerJointHue gives each K4ABT joint a distinct color so individual limbs are " +
                 "traceable. FlatColor uses trailFlatColor for every joint.")]
        public TrailColorMode trailColorMode = TrailColorMode.PerJointHue;

        [Tooltip("Trail color used in FlatColor mode (also tints PerJointHue).")]
        public Color trailFlatColor = Color.white;

        [Min(0.05f)]
        [Tooltip("How long (seconds) each segment of the trail stays visible before fading out.")]
        public float trailDuration = 2.0f;

        [Range(0.001f, 0.05f)]
        [Tooltip("Trail width in meters at the head (= the joint). Tail tapers to ~0.")]
        public float trailWidth = 0.005f;

        [Header("Tracker")]
        [Tooltip("BT processing mode. DirectML works on most Windows GPUs without CUDA.")]
        public k4abt_tracker_processing_mode_t processingMode =
            k4abt_tracker_processing_mode_t.K4ABT_TRACKER_PROCESSING_MODE_GPU_DIRECTML;

        [Tooltip("Temporal smoothing in [0,1]. 0 = none, 1 = full smoothing. " +
                 "Ignored when useWorker is true (the worker fixes smoothing to 0 in v1).")]
        [Range(0f, 1f)] public float smoothing = 0.0f;

        [Tooltip("Run k4abt in a separate process via K4abtWorkerHost instead of in-process. " +
                 "Required for future multi-camera scenarios where multiple trackers can't " +
                 "share one process. v1 still drives a single tracker but exercises the IPC " +
                 "path so issue #11 can extend it.")]
        public bool useWorker = false;

        [Tooltip("K4abtWorkerHost in the scene. Auto-found via FindFirstObjectByType when " +
                 "useWorker is true and this is left null.")]
        public K4abtWorkerHost workerHost;

        // --- runtime state ---

        private System.IntPtr _tracker;
        private System.IntPtr _calibration;
        private bool _trackerReady;
        private bool _workerStarted;
        private string _workerSerial;

        // Per-body visuals: id -> Transform of a parent GameObject that owns 32 joint
        // spheres + a Mesh holding bone line segments. We keep them around between
        // frames so movement is smooth and we don't allocate every Update.
        private readonly Dictionary<uint, BodyVisual> _bodies = new Dictionary<uint, BodyVisual>();
        private readonly Dictionary<uint, int> _lastSeenFrame = new Dictionary<uint, int>();
        private readonly HashSet<uint> _seenThisFrame = new HashSet<uint>();
        private readonly List<uint> _toDestroy = new List<uint>();

        [Header("Visual lifetime")]
        [Tooltip("Hard cap on cached BodyVisuals. K4abt body IDs can flap between frames " +
                 "when tracking is unstable; without a cap we'd allocate 32 GameObjects + " +
                 "Mesh + Material per id forever and freeze Unity. Single-person sessions " +
                 "rarely need more than 2.")]
        [Min(1)] public int maxBodies = 4;

        [Tooltip("Destroy a BodyVisual that hasn't been seen for this many Update ticks. " +
                 "At 30 fps, 60 frames = ~2s of grace before re-creating on a new ID.")]
        [Min(1)] public int unseenFramesBeforeDestroy = 60;

        [Tooltip("Keep a BodyVisual visible for this many Update ticks after it was last " +
                 "popped. Tracker runs at ~30 Hz but Update runs at ~60 Hz, so without grace " +
                 "every other frame would hide the skeleton (= the 'flicker' the user sees).")]
        [Min(0)] public int unseenFramesBeforeHide = 6;

        [Tooltip("Log diagnostic counters once per second (captures enqueued/dropped, body " +
                 "frames popped, bodies detected). Useful while tuning IR/processing mode.")]
        public bool diagnosticLogging = false;

        // Diagnostic counters, reset every ~1s of Update ticks.
        private int _diagEnqueueOk;
        private int _diagEnqueueDropped;
        private int _diagPoppedFrames;
        private int _diagBodiesSeen;
        private float _diagWindowStart;

        private byte[] _depthCopy;   // reused scratch so we don't realloc each callback
        private byte[] _irCopy;      // ditto, only used when IR is delivered alongside depth

        internal static readonly (k4abt_joint_id_t a, k4abt_joint_id_t b)[] s_bones = new[]
        {
            // spine
            (k4abt_joint_id_t.K4ABT_JOINT_PELVIS, k4abt_joint_id_t.K4ABT_JOINT_SPINE_NAVEL),
            (k4abt_joint_id_t.K4ABT_JOINT_SPINE_NAVEL, k4abt_joint_id_t.K4ABT_JOINT_SPINE_CHEST),
            (k4abt_joint_id_t.K4ABT_JOINT_SPINE_CHEST, k4abt_joint_id_t.K4ABT_JOINT_NECK),
            (k4abt_joint_id_t.K4ABT_JOINT_NECK, k4abt_joint_id_t.K4ABT_JOINT_HEAD),
            (k4abt_joint_id_t.K4ABT_JOINT_HEAD, k4abt_joint_id_t.K4ABT_JOINT_NOSE),
            (k4abt_joint_id_t.K4ABT_JOINT_NOSE, k4abt_joint_id_t.K4ABT_JOINT_EYE_LEFT),
            (k4abt_joint_id_t.K4ABT_JOINT_NOSE, k4abt_joint_id_t.K4ABT_JOINT_EYE_RIGHT),
            (k4abt_joint_id_t.K4ABT_JOINT_HEAD, k4abt_joint_id_t.K4ABT_JOINT_EAR_LEFT),
            (k4abt_joint_id_t.K4ABT_JOINT_HEAD, k4abt_joint_id_t.K4ABT_JOINT_EAR_RIGHT),

            // left arm
            (k4abt_joint_id_t.K4ABT_JOINT_SPINE_CHEST, k4abt_joint_id_t.K4ABT_JOINT_CLAVICLE_LEFT),
            (k4abt_joint_id_t.K4ABT_JOINT_CLAVICLE_LEFT, k4abt_joint_id_t.K4ABT_JOINT_SHOULDER_LEFT),
            (k4abt_joint_id_t.K4ABT_JOINT_SHOULDER_LEFT, k4abt_joint_id_t.K4ABT_JOINT_ELBOW_LEFT),
            (k4abt_joint_id_t.K4ABT_JOINT_ELBOW_LEFT, k4abt_joint_id_t.K4ABT_JOINT_WRIST_LEFT),
            (k4abt_joint_id_t.K4ABT_JOINT_WRIST_LEFT, k4abt_joint_id_t.K4ABT_JOINT_HAND_LEFT),
            (k4abt_joint_id_t.K4ABT_JOINT_HAND_LEFT, k4abt_joint_id_t.K4ABT_JOINT_HANDTIP_LEFT),
            (k4abt_joint_id_t.K4ABT_JOINT_WRIST_LEFT, k4abt_joint_id_t.K4ABT_JOINT_THUMB_LEFT),

            // right arm
            (k4abt_joint_id_t.K4ABT_JOINT_SPINE_CHEST, k4abt_joint_id_t.K4ABT_JOINT_CLAVICLE_RIGHT),
            (k4abt_joint_id_t.K4ABT_JOINT_CLAVICLE_RIGHT, k4abt_joint_id_t.K4ABT_JOINT_SHOULDER_RIGHT),
            (k4abt_joint_id_t.K4ABT_JOINT_SHOULDER_RIGHT, k4abt_joint_id_t.K4ABT_JOINT_ELBOW_RIGHT),
            (k4abt_joint_id_t.K4ABT_JOINT_ELBOW_RIGHT, k4abt_joint_id_t.K4ABT_JOINT_WRIST_RIGHT),
            (k4abt_joint_id_t.K4ABT_JOINT_WRIST_RIGHT, k4abt_joint_id_t.K4ABT_JOINT_HAND_RIGHT),
            (k4abt_joint_id_t.K4ABT_JOINT_HAND_RIGHT, k4abt_joint_id_t.K4ABT_JOINT_HANDTIP_RIGHT),
            (k4abt_joint_id_t.K4ABT_JOINT_WRIST_RIGHT, k4abt_joint_id_t.K4ABT_JOINT_THUMB_RIGHT),

            // left leg
            (k4abt_joint_id_t.K4ABT_JOINT_PELVIS, k4abt_joint_id_t.K4ABT_JOINT_HIP_LEFT),
            (k4abt_joint_id_t.K4ABT_JOINT_HIP_LEFT, k4abt_joint_id_t.K4ABT_JOINT_KNEE_LEFT),
            (k4abt_joint_id_t.K4ABT_JOINT_KNEE_LEFT, k4abt_joint_id_t.K4ABT_JOINT_ANKLE_LEFT),
            (k4abt_joint_id_t.K4ABT_JOINT_ANKLE_LEFT, k4abt_joint_id_t.K4ABT_JOINT_FOOT_LEFT),

            // right leg
            (k4abt_joint_id_t.K4ABT_JOINT_PELVIS, k4abt_joint_id_t.K4ABT_JOINT_HIP_RIGHT),
            (k4abt_joint_id_t.K4ABT_JOINT_HIP_RIGHT, k4abt_joint_id_t.K4ABT_JOINT_KNEE_RIGHT),
            (k4abt_joint_id_t.K4ABT_JOINT_KNEE_RIGHT, k4abt_joint_id_t.K4ABT_JOINT_ANKLE_RIGHT),
            (k4abt_joint_id_t.K4ABT_JOINT_ANKLE_RIGHT, k4abt_joint_id_t.K4ABT_JOINT_FOOT_RIGHT),
        };

        // Convert a K4A camera-frame point (mm, right-handed: +X right, +Y down, +Z forward)
        // into the Unity local frame attached to this GameObject (m, left-handed: +Y up).
        // The world placement of this transform is up to the user.
        public static Vector3 K4AmmToUnity(in k4a_float3_t p)
        {
            return new Vector3(p.X * 0.001f, -p.Y * 0.001f, p.Z * 0.001f);
        }

        private void Awake()
        {
            // Make sure the bootstrap has run so PATH is set before any P/Invoke happens.
            BodyTrackingBootstrap.Initialize();
        }

        private bool _bound;

        private void OnEnable()
        {
            if (cameraManager == null) cameraManager = FindFirstObjectByType<PointCloudCameraManager>();
            TryBindSource();
        }

        private void OnDisable()
        {
            UnbindSource();
            StopWorkerIfStarted();
            DestroyTracker();
            foreach (var v in _bodies.Values) v.Destroy();
            _bodies.Clear();
        }

        private void StopWorkerIfStarted()
        {
            if (!_workerStarted) return;
            if (workerHost != null)
            {
                workerHost.OnSkeletonsReady -= OnWorkerSkeletons;
                workerHost.StopWorker(_workerSerial);
            }
            _workerStarted = false;
            _workerSerial = null;
        }

        // Renderers are spawned at runtime by PointCloudCameraManager.Start, so we may
        // need a few frames before one is available. Re-check from Update until bound.
        private void TryBindSource()
        {
            if (_bound) return;

            if (source == null && cameraManager != null && cameraManager.Renderers.Count > 0)
            {
                source = cameraManager.Renderers[0];
            }
            if (source == null && cameraManager == null)
            {
                source = FindFirstObjectByType<PointCloudRenderer>();
            }
            if (source == null) return;

            source.OnRawFramesReady += HandleRawFrame;
            _bound = true;
        }

        private void UnbindSource()
        {
            if (_bound && source != null) source.OnRawFramesReady -= HandleRawFrame;
            _bound = false;
        }

        private bool TryEnsureWorker(in RawFrameData frame)
        {
            if (_workerStarted) return true;
            if (!source.CameraParam.HasValue) return false;
            if (workerHost == null) workerHost = FindFirstObjectByType<K4abtWorkerHost>();
            if (workerHost == null)
            {
                Debug.LogError("[BodyTrackingLive] useWorker=true but no K4abtWorkerHost in scene; disabling worker path");
                useWorker = false;
                return false;
            }

            _workerSerial = string.IsNullOrEmpty(source.deviceSerial)
                ? source.gameObject.name
                : source.deviceSerial;

            int irW = frame.IRWidth > 0 ? frame.IRWidth : frame.DepthWidth;
            int irH = frame.IRHeight > 0 ? frame.IRHeight : frame.DepthHeight;
            if (!workerHost.StartWorker(_workerSerial, source.CameraParam.Value,
                    frame.DepthWidth, frame.DepthHeight, irW, irH,
                    frame.ColorWidth, frame.ColorHeight))
            {
                Debug.LogError("[BodyTrackingLive] StartWorker failed; falling back to in-process tracker");
                useWorker = false;
                return false;
            }
            workerHost.OnSkeletonsReady += OnWorkerSkeletons;
            _workerStarted = true;
            return true;
        }

        private bool TryEnsureTracker(in RawFrameData frame)
        {
            if (_trackerReady) return true;
            if (!source.CameraParam.HasValue) return false;

            _calibration = K4ACalibration.Build(
                source.CameraParam.Value,
                frame.DepthWidth, frame.DepthHeight,
                frame.ColorWidth, frame.ColorHeight);

            var cfg = new k4abt_tracker_configuration_t
            {
                SensorOrientation = k4abt_sensor_orientation_t.K4ABT_SENSOR_ORIENTATION_DEFAULT,
                ProcessingMode = processingMode,
                GpuDeviceId = 0,
                ModelPath = null,
            };

            var rc = K4ABTNative.k4abt_tracker_create(_calibration, cfg, out _tracker);
            if (rc != k4a_result_t.K4A_RESULT_SUCCEEDED)
            {
                Debug.LogError("[BodyTrackingLive] k4abt_tracker_create failed; disabling.");
                K4ACalibration.Free(_calibration); _calibration = System.IntPtr.Zero;
                enabled = false;
                return false;
            }

            K4ABTNative.k4abt_tracker_set_temporal_smoothing(_tracker, smoothing);
            _trackerReady = true;
            return true;
        }

        private void DestroyTracker()
        {
            if (_tracker != System.IntPtr.Zero)
            {
                K4ABTNative.k4abt_tracker_shutdown(_tracker);
                K4ABTNative.k4abt_tracker_destroy(_tracker);
                _tracker = System.IntPtr.Zero;
            }
            if (_calibration != System.IntPtr.Zero)
            {
                K4ACalibration.Free(_calibration);
                _calibration = System.IntPtr.Zero;
            }
            _trackerReady = false;
        }

        private void HandleRawFrame(PointCloudRenderer src, RawFrameData frame)
        {
            if (!showSkeleton) return;

            int dNeeded = frame.DepthByteCount;
            if (_depthCopy == null || _depthCopy.Length < dNeeded) _depthCopy = new byte[dNeeded];
            System.Buffer.BlockCopy(frame.DepthBytes, 0, _depthCopy, 0, dNeeded);

            byte[] irBytes = null;
            int irBytesCount = 0, irW = 0, irH = 0;
            if (frame.IRByteCount > 0 && frame.IRBytes != null)
            {
                int iNeeded = frame.IRByteCount;
                if (_irCopy == null || _irCopy.Length < iNeeded) _irCopy = new byte[iNeeded];
                System.Buffer.BlockCopy(frame.IRBytes, 0, _irCopy, 0, iNeeded);
                irBytes = _irCopy;
                irBytesCount = iNeeded;
                irW = frame.IRWidth;
                irH = frame.IRHeight;
            }

            if (useWorker)
            {
                if (!TryEnsureWorker(frame)) return;
                ulong tsNs = frame.TimestampUs * 1000UL;
                if (workerHost.EnqueueFrame(_workerSerial, _depthCopy, dNeeded,
                                            irBytes, irBytesCount, tsNs))
                {
                    _diagEnqueueOk++;
                }
                else
                {
                    _diagEnqueueDropped++;
                }
                return;
            }

            if (!TryEnsureTracker(frame)) return;

            ulong tsUsec = frame.TimestampUs; // already microseconds
            var capture = K4ACaptureBridge.CreateCaptureFromDepthAndIR(
                _depthCopy, dNeeded, frame.DepthWidth, frame.DepthHeight,
                irBytes, irBytesCount, irW, irH,
                tsUsec);
            if (capture == System.IntPtr.Zero) return;

            // Non-blocking enqueue (returns TIMEOUT if the input queue is full — drop the frame).
            var enq = K4ABTNative.k4abt_tracker_enqueue_capture(_tracker, capture, 0);
            K4ANative.k4a_capture_release(capture);
            if (enq != k4a_wait_result_t.K4A_WAIT_RESULT_SUCCEEDED)
            {
                _diagEnqueueDropped++;
                return;
            }
            _diagEnqueueOk++;
        }

        // Synchronously fired from K4abtWorkerHost.Update via OnSkeletonsReady.
        // Because the host has DefaultExecutionOrder(-100) it runs before this
        // script's Update each frame, so Update only needs to do GC/diag.
        private void OnWorkerSkeletons(string serial, BodySnapshot[] bodies, int count)
        {
            if (!showSkeleton) return;
            if (serial != _workerSerial) return; // someone else's worker
            _diagPoppedFrames++;
            _diagBodiesSeen += count;

            // Reuse a scratch skel struct: BodyVisual.UpdateFromSkeleton just iterates
            // skel.Joints[i], so we can swap the joint array reference each iteration.
            var skel = new k4abt_skeleton_t();
            for (int i = 0; i < count; i++)
            {
                var snap = bodies[i];
                skel.Joints = snap.Joints;
                ApplyBodySkeleton(snap.Id, in skel);
            }
        }

        private void Update()
        {
            if (!_bound) TryBindSource();
            if (!showSkeleton) return;

            // Worker mode: skeletons were already applied via OnWorkerSkeletons (the host
            // has DefaultExecutionOrder(-100) so it ran before us this frame). _seenThisFrame
            // is populated by ApplyBodySkeleton inside that handler. We just need to GC.
            // In-process mode: clear and drain pop_result here.
            if (useWorker)
            {
                if (!_workerStarted) { /* will start on first HandleRawFrame */ }
            }
            else
            {
                if (!_trackerReady) return;
                _seenThisFrame.Clear();
                while (true)
                {
                    var rc = K4ABTNative.k4abt_tracker_pop_result(_tracker, out System.IntPtr bodyFrame, 0);
                    if (rc != k4a_wait_result_t.K4A_WAIT_RESULT_SUCCEEDED) break;
                    _diagPoppedFrames++;
                    try
                    {
                        uint nBodies = K4ABTNative.k4abt_frame_get_num_bodies(bodyFrame);
                        _diagBodiesSeen += (int)nBodies;
                        for (uint i = 0; i < nBodies; i++)
                        {
                            uint id = K4ABTNative.k4abt_frame_get_body_id(bodyFrame, i);
                            if (id == K4ABTConsts.K4ABT_INVALID_BODY_ID) continue;
                            if (K4ABTNative.k4abt_frame_get_body_skeleton(bodyFrame, i, out var skel)
                                != k4a_result_t.K4A_RESULT_SUCCEEDED) continue;
                            ApplyBodySkeleton(id, in skel);
                        }
                    }
                    finally
                    {
                        K4ABTNative.k4abt_frame_release(bodyFrame);
                    }
                }
            }

            // While a body is alive in the dict its visual stays visible — never call
            // SetActive in the per-tick loop. The only path that removes the visual is
            // Destroy() below, after unseenFramesBeforeDestroy ticks without a pop.
            // This eliminates the SetActive(true)/SetActive(false) toggle storm that
            // happened when the per-tick visibility check fluctuated between true/false.
            _toDestroy.Clear();
            foreach (var kv in _bodies)
            {
                int lastSeen = _lastSeenFrame.TryGetValue(kv.Key, out var f) ? f : -1;
                int sinceLastSeen = Time.frameCount - lastSeen;
                kv.Value.TickDiagAfterUpdate();
                if (sinceLastSeen > unseenFramesBeforeDestroy)
                {
                    _toDestroy.Add(kv.Key);
                }
            }
            foreach (var id in _toDestroy)
            {
                _bodies[id].Destroy();
                _bodies.Remove(id);
                _lastSeenFrame.Remove(id);
            }

            if (diagnosticLogging)
            {
                float now = Time.realtimeSinceStartup;
                if (_diagWindowStart == 0f) _diagWindowStart = now;
                float elapsed = now - _diagWindowStart;
                if (elapsed >= 1f)
                {
                    // Probe a representative joint (PELVIS) so we know whether the visual
                    // is actually being placed somewhere visible or off in nowhere-land.
                    string sample = "<no body>";
                    foreach (var kv in _bodies)
                    {
                        var v = kv.Value;
                        float meanJump = v.JumpSamples > 0 ? v.SumJumpThisWindow / v.JumpSamples : 0f;
                        sample = $"id={kv.Key} active={v.IsActive} " +
                                 $"toggles/s={v.VisibilityToggles} " +
                                 $"setActive(true/false)={v.SetActiveCallsTrue}/{v.SetActiveCallsFalse} " +
                                 $"pelvis_jump_max={v.MaxJumpThisWindow:F3}m " +
                                 $"pelvis_jump_avg={meanJump:F3}m";
                        v.ResetDiagWindow();
                        break;
                    }
                    Debug.Log(
                        $"[BodyTrackingLive] enq_ok={_diagEnqueueOk}/s " +
                        $"enq_dropped={_diagEnqueueDropped}/s " +
                        $"popped={_diagPoppedFrames}/s " +
                        $"bodies_seen={_diagBodiesSeen}/s " +
                        $"alive_visuals={_bodies.Count} | {sample}",
                        this);
                    _diagEnqueueOk = 0;
                    _diagEnqueueDropped = 0;
                    _diagPoppedFrames = 0;
                    _diagBodiesSeen = 0;
                    _diagWindowStart = now;
                }
            }
        }

        // Shared per-body update path used by both the in-process pop loop and the
        // worker-event handler. Creates the BodyVisual on first sight (with eviction
        // if we'd exceed maxBodies), then updates joints/bones/trails and stamps
        // _lastSeenFrame so the GC pass at the end of Update can see we touched it.
        private void ApplyBodySkeleton(uint id, in k4abt_skeleton_t skel)
        {
            if (!_bodies.TryGetValue(id, out var visual))
            {
                EvictIfFull();
                visual = new BodyVisual(transform, id, jointRadius, skeletonColor,
                    showTrails, trailDuration, trailWidth,
                    trailColorMode, trailFlatColor);
                _bodies[id] = visual;
            }
            visual.UpdateFromSkeleton(skel, jointRadius, showAnatomicalBones, skeletonColor);
            visual.ApplyTrailParams(showTrails, trailDuration, trailWidth, trailColorMode, trailFlatColor);
            _seenThisFrame.Add(id);
            _lastSeenFrame[id] = Time.frameCount;
        }

        private void EvictIfFull()
        {
            while (_bodies.Count >= maxBodies)
            {
                uint oldestId = 0;
                int oldestFrame = int.MaxValue;
                bool any = false;
                foreach (var kv in _bodies)
                {
                    int f = _lastSeenFrame.TryGetValue(kv.Key, out var v) ? v : -1;
                    if (f < oldestFrame)
                    {
                        oldestFrame = f;
                        oldestId = kv.Key;
                        any = true;
                    }
                }
                if (!any) break;
                _bodies[oldestId].Destroy();
                _bodies.Remove(oldestId);
                _lastSeenFrame.Remove(oldestId);
            }
        }

    }
}
