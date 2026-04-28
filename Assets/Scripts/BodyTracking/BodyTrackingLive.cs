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

        [Header("Tracker")]
        [Tooltip("BT processing mode. DirectML works on most Windows GPUs without CUDA.")]
        public k4abt_tracker_processing_mode_t processingMode =
            k4abt_tracker_processing_mode_t.K4ABT_TRACKER_PROCESSING_MODE_GPU_DIRECTML;

        [Tooltip("Temporal smoothing in [0,1]. 0 = none, 1 = full smoothing.")]
        [Range(0f, 1f)] public float smoothing = 0.0f;

        // --- runtime state ---

        private System.IntPtr _tracker;
        private System.IntPtr _calibration;
        private bool _trackerReady;

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
        public bool diagnosticLogging = true;

        // Diagnostic counters, reset every ~1s of Update ticks.
        private int _diagEnqueueOk;
        private int _diagEnqueueDropped;
        private int _diagPoppedFrames;
        private int _diagBodiesSeen;
        private float _diagWindowStart;

        private byte[] _depthCopy;   // reused scratch so we don't realloc each callback
        private byte[] _irCopy;      // ditto, only used when IR is delivered alongside depth

        private static readonly (k4abt_joint_id_t a, k4abt_joint_id_t b)[] s_bones = new[]
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
            DestroyTracker();
            foreach (var v in _bodies.Values) v.Destroy();
            _bodies.Clear();
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
            if (!TryEnsureTracker(frame)) return;

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

        private void Update()
        {
            if (!_bound) TryBindSource();
            if (!_trackerReady || !showSkeleton) return;

            // Drain whatever the tracker has produced this frame; non-blocking.
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

                        if (!_bodies.TryGetValue(id, out var visual))
                        {
                            // Hard cap: if the dict is full, evict the oldest unseen body
                            // before allocating a new visual. Without this we'd leak 32+ GOs
                            // every time k4abt assigns a new id (which can happen per frame
                            // when tracking is unstable, freezing Unity within seconds).
                            EvictIfFull();
                            visual = new BodyVisual(transform, id, jointRadius, skeletonColor);
                            _bodies[id] = visual;
                        }
                        visual.UpdateFromSkeleton(skel, jointRadius, showAnatomicalBones, skeletonColor);
                        _seenThisFrame.Add(id);
                        _lastSeenFrame[id] = Time.frameCount;
                    }
                }
                finally
                {
                    K4ABTNative.k4abt_frame_release(bodyFrame);
                }
            }

            // Tracker runs at ~30 Hz, Update at ~60 Hz, so most ticks have an empty
            // _seenThisFrame even when the body is being tracked perfectly. Keep the
            // visual visible for unseenFramesBeforeHide ticks of grace, and only destroy
            // the GO once unseenFramesBeforeDestroy ticks have passed without a pop.
            _toDestroy.Clear();
            foreach (var kv in _bodies)
            {
                int lastSeen = _lastSeenFrame.TryGetValue(kv.Key, out var f) ? f : -1;
                int sinceLastSeen = Time.frameCount - lastSeen;
                kv.Value.SetVisible(sinceLastSeen <= unseenFramesBeforeHide);
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

        // Per-body visuals: 32 joint markers + a single line-list mesh of bones.
        private sealed class BodyVisual
        {
            private readonly GameObject _root;
            private readonly Transform[] _joints = new Transform[K4ABTConsts.K4ABT_JOINT_COUNT];
            private readonly Vector3[] _jointPositions = new Vector3[K4ABTConsts.K4ABT_JOINT_COUNT];
            private readonly bool[] _jointValid = new bool[K4ABTConsts.K4ABT_JOINT_COUNT];

            private readonly GameObject _bonesGO;
            private readonly Mesh _bonesMesh;
            private readonly MeshRenderer _bonesRenderer;
            private readonly Material _bonesMat;
            private readonly Vector3[] _boneVerts;
            private readonly int[] _boneIndices;

            public BodyVisual(Transform parent, uint id, float jointRadius, Color color)
            {
                _root = new GameObject($"Body_{id}");
                _root.transform.SetParent(parent, false);

                // Pre-resolve a shader. `new Material(null)` throws ArgumentNullException
                // immediately, so the old shader-then-fallback pattern (`new Material(Find(a));
                // if (mat.shader == null) mat = new Material(Find(b))`) crashes before the
                // fallback runs. Resolve the chain first, then construct the Material once.
                var unlitShader = ResolveUnlitShader();

                var jointMat = new Material(unlitShader);
                SetMaterialColor(jointMat, new Color(color.r * 1.2f, color.g * 1.2f, color.b * 1.2f, 1f));

                for (int i = 0; i < _joints.Length; i++)
                {
                    var sphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                    Object.Destroy(sphere.GetComponent<Collider>());
                    sphere.transform.SetParent(_root.transform, false);
                    sphere.transform.localScale = Vector3.one * (jointRadius * 2f);
                    sphere.GetComponent<MeshRenderer>().sharedMaterial = jointMat;
                    sphere.name = ((k4abt_joint_id_t)i).ToString();
                    _joints[i] = sphere.transform;
                }

                _bonesGO = new GameObject("Bones");
                _bonesGO.transform.SetParent(_root.transform, false);
                var mf = _bonesGO.AddComponent<MeshFilter>();
                _bonesRenderer = _bonesGO.AddComponent<MeshRenderer>();
                _bonesMesh = new Mesh { name = "Bones", indexFormat = UnityEngine.Rendering.IndexFormat.UInt16 };
                mf.sharedMesh = _bonesMesh;

                _bonesMat = new Material(unlitShader);
                SetMaterialColor(_bonesMat, color);
                _bonesRenderer.sharedMaterial = _bonesMat;

                _boneVerts = new Vector3[s_bones.Length * 2];
                _boneIndices = new int[s_bones.Length * 2];
                for (int i = 0; i < _boneIndices.Length; i++) _boneIndices[i] = i;
            }

            public void UpdateFromSkeleton(in k4abt_skeleton_t skel, float jointRadius,
                                            bool showBones, Color color)
            {
                // Each pop refreshes the last-known position for every joint with at least
                // LOW confidence. Joints whose confidence flaps below LOW keep their previous
                // position (don't toggle SetActive — that's what produced per-joint flicker
                // when a body was being tracked at the edge of the depth model's range).
                for (int i = 0; i < _joints.Length; i++)
                {
                    var j = skel.Joints[i];
                    if (j.ConfidenceLevel >= k4abt_joint_confidence_level_t.K4ABT_JOINT_CONFIDENCE_LOW)
                    {
                        _jointPositions[i] = K4AmmToUnity(j.Position);
                        _jointValid[i] = true;
                    }
                    // else: leave _jointPositions[i] / _jointValid[i] from previous pop.
                    _joints[i].localPosition = _jointPositions[i];
                    _joints[i].localScale = Vector3.one * (jointRadius * 2f);
                    if (!_joints[i].gameObject.activeSelf && _jointValid[i])
                    {
                        _joints[i].gameObject.SetActive(true);
                    }
                }

                if (showBones)
                {
                    int v = 0;
                    for (int b = 0; b < s_bones.Length; b++)
                    {
                        var (a, c) = s_bones[b];
                        if (_jointValid[(int)a] && _jointValid[(int)c])
                        {
                            _boneVerts[v++] = _jointPositions[(int)a];
                            _boneVerts[v++] = _jointPositions[(int)c];
                        }
                    }
                    // Pad unused slots to last valid vertex so the line list stays well-formed.
                    if (v == 0)
                    {
                        _bonesMesh.Clear();
                    }
                    else
                    {
                        var verts = new Vector3[v];
                        var idx = new int[v];
                        System.Array.Copy(_boneVerts, verts, v);
                        for (int i = 0; i < v; i++) idx[i] = i;
                        _bonesMesh.Clear();
                        _bonesMesh.vertices = verts;
                        _bonesMesh.SetIndices(idx, MeshTopology.Lines, 0, true);
                    }
                    SetMaterialColor(_bonesMat, color);
                    if (!_bonesGO.activeSelf) _bonesGO.SetActive(true);
                }
                else
                {
                    if (_bonesGO.activeSelf) _bonesGO.SetActive(false);
                }
            }

            public void SetVisible(bool visible)
            {
                if (_root != null && _root.activeSelf != visible) _root.SetActive(visible);
            }

            public bool IsActive => _root != null && _root.activeInHierarchy;

            // Diagnostic accessors (used by the parent's per-second log).
            public Vector3 GetSamplePosition()
            {
                int idx = (int)k4abt_joint_id_t.K4ABT_JOINT_PELVIS;
                return idx < _jointPositions.Length ? _jointPositions[idx] : Vector3.zero;
            }

            public Vector3 WorldOf(Vector3 local)
            {
                return _root != null ? _root.transform.TransformPoint(local) : local;
            }

            // Diagnostic: how often did this body's visibility toggle and how big are
            // the per-pop position jumps? Reset by ResetDiagWindow.
            public int VisibilityToggles { get; private set; }
            public float MaxJumpThisWindow { get; private set; }
            public float SumJumpThisWindow { get; private set; }
            public int JumpSamples { get; private set; }
            private bool _wasActive;
            private Vector3 _prevPelvis;

            public void TickDiagAfterUpdate()
            {
                bool nowActive = _root != null && _root.activeSelf;
                if (nowActive != _wasActive) VisibilityToggles++;
                _wasActive = nowActive;

                int idx = (int)k4abt_joint_id_t.K4ABT_JOINT_PELVIS;
                if (idx < _jointPositions.Length)
                {
                    var cur = _jointPositions[idx];
                    if (_prevPelvis != Vector3.zero)
                    {
                        float d = (cur - _prevPelvis).magnitude;
                        if (d > MaxJumpThisWindow) MaxJumpThisWindow = d;
                        SumJumpThisWindow += d;
                        JumpSamples++;
                    }
                    _prevPelvis = cur;
                }
            }

            public void ResetDiagWindow()
            {
                VisibilityToggles = 0;
                MaxJumpThisWindow = 0f;
                SumJumpThisWindow = 0f;
                JumpSamples = 0;
            }

            public void Destroy()
            {
                if (_bonesMesh != null) Object.Destroy(_bonesMesh);
                if (_root != null) Object.Destroy(_root);
            }

            private static Shader ResolveUnlitShader()
            {
                Shader s = Shader.Find("Universal Render Pipeline/Unlit");
                if (s == null) s = Shader.Find("Unlit/Color");
                if (s == null) s = Shader.Find("Sprites/Default");
                return s;
            }

            // URP/Unlit binds the visible color to "_BaseColor"; legacy Material.color
            // writes "_Color" instead and gets ignored on URP shaders. Set both so the
            // material picks up the colour regardless of which pipeline is in use.
            private static void SetMaterialColor(Material m, Color c)
            {
                if (m == null) return;
                if (m.HasProperty("_BaseColor")) m.SetColor("_BaseColor", c);
                if (m.HasProperty("_Color")) m.SetColor("_Color", c);
                m.color = c;
            }
        }
    }
}
