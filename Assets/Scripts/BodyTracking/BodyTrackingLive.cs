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
        [Tooltip("PointCloudRenderer whose depth stream feeds the body tracker. " +
                 "Leave empty to auto-pick the first PointCloudRenderer found in the scene.")]
        public PointCloudRenderer source;

        [Header("Display")]
        [Tooltip("Show the skeleton (joints + anatomical bones) on the point cloud. " +
                 "Bones can be hidden separately with showAnatomicalBones below.")]
        public bool showSkeleton = true;

        [Tooltip("Draw the connecting lines between joints (shoulder–elbow–wrist etc). " +
                 "Issue #7 explicitly exposes this as an opt-out toggle.")]
        public bool showAnatomicalBones = true;

        [Tooltip("Joint marker radius in meters.")]
        [Range(0.005f, 0.05f)] public float jointRadius = 0.015f;

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
        private readonly HashSet<uint> _seenThisFrame = new HashSet<uint>();

        private byte[] _depthCopy;   // reused scratch so we don't realloc each callback

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

        private void OnEnable()
        {
            if (source == null) source = FindFirstObjectByType<PointCloudRenderer>();
            if (source == null)
            {
                Debug.LogWarning("[BodyTrackingLive] no PointCloudRenderer in scene; disabled.");
                enabled = false;
                return;
            }
            source.OnRawFramesReady += HandleRawFrame;
        }

        private void OnDisable()
        {
            if (source != null) source.OnRawFramesReady -= HandleRawFrame;
            DestroyTracker();
            foreach (var v in _bodies.Values) v.Destroy();
            _bodies.Clear();
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

            int needed = frame.DepthByteCount;
            if (_depthCopy == null || _depthCopy.Length < needed) _depthCopy = new byte[needed];
            System.Buffer.BlockCopy(frame.DepthBytes, 0, _depthCopy, 0, needed);

            ulong tsUsec = frame.TimestampUs; // already microseconds
            var capture = K4ACaptureBridge.CreateCaptureFromDepthY16(
                _depthCopy, needed, frame.DepthWidth, frame.DepthHeight, tsUsec);
            if (capture == System.IntPtr.Zero) return;

            // Non-blocking enqueue (returns TIMEOUT if the input queue is full — drop the frame).
            var enq = K4ABTNative.k4abt_tracker_enqueue_capture(_tracker, capture, 0);
            K4ANative.k4a_capture_release(capture);
            if (enq != k4a_wait_result_t.K4A_WAIT_RESULT_SUCCEEDED) return;
        }

        private void Update()
        {
            if (!_trackerReady || !showSkeleton) return;

            // Drain whatever the tracker has produced this frame; non-blocking.
            _seenThisFrame.Clear();
            while (true)
            {
                var rc = K4ABTNative.k4abt_tracker_pop_result(_tracker, out System.IntPtr bodyFrame, 0);
                if (rc != k4a_wait_result_t.K4A_WAIT_RESULT_SUCCEEDED) break;
                try
                {
                    uint nBodies = K4ABTNative.k4abt_frame_get_num_bodies(bodyFrame);
                    for (uint i = 0; i < nBodies; i++)
                    {
                        uint id = K4ABTNative.k4abt_frame_get_body_id(bodyFrame, i);
                        if (id == K4ABTConsts.K4ABT_INVALID_BODY_ID) continue;
                        if (K4ABTNative.k4abt_frame_get_body_skeleton(bodyFrame, i, out var skel)
                            != k4a_result_t.K4A_RESULT_SUCCEEDED) continue;

                        if (!_bodies.TryGetValue(id, out var visual))
                        {
                            visual = new BodyVisual(transform, id, jointRadius, skeletonColor);
                            _bodies[id] = visual;
                        }
                        visual.UpdateFromSkeleton(skel, jointRadius, showAnatomicalBones, skeletonColor);
                        _seenThisFrame.Add(id);
                    }
                }
                finally
                {
                    K4ABTNative.k4abt_frame_release(bodyFrame);
                }
            }

            // Bodies that disappeared this tick are hidden but kept around for a few
            // frames in case they re-appear with the same id (cheap to keep, expensive
            // to destroy and re-create the GameObjects).
            foreach (var kv in _bodies)
            {
                kv.Value.SetVisible(_seenThisFrame.Contains(kv.Key));
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

                var jointMat = new Material(Shader.Find("Universal Render Pipeline/Unlit"));
                if (jointMat.shader == null) jointMat = new Material(Shader.Find("Unlit/Color"));
                jointMat.color = new Color(color.r * 1.2f, color.g * 1.2f, color.b * 1.2f, 1f);

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

                _bonesMat = new Material(Shader.Find("Universal Render Pipeline/Unlit"));
                if (_bonesMat.shader == null) _bonesMat = new Material(Shader.Find("Unlit/Color"));
                _bonesMat.color = color;
                _bonesRenderer.sharedMaterial = _bonesMat;

                _boneVerts = new Vector3[s_bones.Length * 2];
                _boneIndices = new int[s_bones.Length * 2];
                for (int i = 0; i < _boneIndices.Length; i++) _boneIndices[i] = i;
            }

            public void UpdateFromSkeleton(in k4abt_skeleton_t skel, float jointRadius,
                                            bool showBones, Color color)
            {
                for (int i = 0; i < _joints.Length; i++)
                {
                    var j = skel.Joints[i];
                    bool valid = j.ConfidenceLevel >= k4abt_joint_confidence_level_t.K4ABT_JOINT_CONFIDENCE_LOW;
                    _jointValid[i] = valid;
                    if (valid)
                    {
                        var p = K4AmmToUnity(j.Position);
                        _jointPositions[i] = p;
                        _joints[i].localPosition = p;
                        _joints[i].localScale = Vector3.one * (jointRadius * 2f);
                        _joints[i].gameObject.SetActive(true);
                    }
                    else
                    {
                        _joints[i].gameObject.SetActive(false);
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
                    _bonesMat.color = color;
                    _bonesGO.SetActive(true);
                }
                else
                {
                    _bonesGO.SetActive(false);
                }
            }

            public void SetVisible(bool visible) => _root.SetActive(visible);

            public void Destroy()
            {
                if (_bonesMesh != null) Object.Destroy(_bonesMesh);
                if (_root != null) Object.Destroy(_root);
            }
        }
    }
}
