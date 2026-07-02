// Feeds per-joint world-space position + velocity samples into a
// PointCloud.PointCloudJointMotionField every LateUpdate. The field is then
// consumed by PointCloudRenderer (live shader nearest-joint sampling) and
// SensorRecorder (playback shader), mirroring the BodyTubeCapsuleFeeder
// path. This is what gives each point in the cloud its "nearest joint
// motion" attribute the way issue #24 asks for.
//
// LateUpdate matches BodyTubeCapsuleFeeder: SkeletonMerger updates
// skeletons in Update, so by LateUpdate the joint positions reflect the
// current frame. The point cloud shader picks up the values next frame —
// same one-frame lag the OBB / capsule paths already accept.

using UnityEngine;
using PointCloud;

namespace BodyTracking
{
    [DisallowMultipleComponent]
    public class BodyJointMotionFeeder : MonoBehaviour
    {
        [Tooltip("Body-tracking source. Leave empty to auto-resolve the first " +
                 "SkeletonMerger in the scene at OnEnable.")]
        public SkeletonMerger bodyTracking;

        [Tooltip("Joint motion field that receives the per-joint world-space pos + velocity. " +
                 "Leave empty to auto-resolve the first PointCloudJointMotionField in the scene at OnEnable.")]
        public PointCloudJointMotionField target;

        [Range(0f, 0.95f)]
        [Tooltip("Per-joint velocity EMA smoothing factor. 0 = raw finite difference " +
                 "(noisy at high fps), 0.8 = heavy smoothing (lagged). The One-Euro " +
                 "filter on the body side already smooths the positions; this damps the " +
                 "derivative noise that survives the difference.")]
        public float velocitySmoothing = 0.5f;

        private void OnEnable()
        {
            if (bodyTracking == null) bodyTracking = FindFirstObjectByType<SkeletonMerger>();
            if (target == null) target = FindFirstObjectByType<PointCloudJointMotionField>();
        }

        private void LateUpdate()
        {
            if (target == null) return;
            if (bodyTracking == null)
            {
                target.Clear();
                return;
            }
            bodyTracking.PublishJointMotionsWorld(target, velocitySmoothing);
        }

        private void OnDisable()
        {
            if (target != null) target.Clear();
        }
    }
}
