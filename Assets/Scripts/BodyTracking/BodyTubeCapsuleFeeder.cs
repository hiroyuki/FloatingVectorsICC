// Feeds the body's anatomical bones (as world-space capsules with a configurable
// radius) into a PointCloud.PointCloudCapsuleFilter every LateUpdate. The
// filter is then consumed by PointCloudRenderer (live shader cull),
// SensorRecorder (playback shader cull), and optionally PointCloudCumulative
// (CPU pre-filter for snapshot freezing). End-to-end this lets the renderer
// keep only the point cloud lying inside the BT tube volume, with the
// cumulative snapshot pattern accumulating those frozen points as a body
// movement trail (issue #23).
//
// LateUpdate is intentional: SkeletonMerger updates skeletons in Update,
// so this picks up the same-frame joint positions. PointCloudRenderer's Update
// reads the filter contents next frame — a one-frame lag matching the existing
// OBB transform read.

using UnityEngine;
using PointCloud;

namespace BodyTracking
{
    [DisallowMultipleComponent]
    public class BodyTubeCapsuleFeeder : MonoBehaviour
    {
        [Tooltip("Body-tracking source. Leave empty to auto-resolve the first " +
                 "SkeletonMerger in the scene at OnEnable.")]
        public SkeletonMerger bodyTracking;

        [Tooltip("Capsule filter that receives the per-bone world-space capsules. " +
                 "Leave empty to auto-resolve the first PointCloudCapsuleFilter in the scene at OnEnable.")]
        public PointCloudCapsuleFilter target;

        [Min(0f)]
        [Tooltip("Per-bone tube radius in meters used for the point cloud filter. " +
                 "Independent from SkeletonMerger.boneWidth (which controls the " +
                 "rendered bone tube width). Use a generous value (e.g. 0.1–0.25 m) to " +
                 "scoop a wide slab of points around the body for the movement trail.")]
        public float tubeRadius = 0.15f;

        private void OnEnable()
        {
            if (bodyTracking == null) bodyTracking = FindFirstObjectByType<SkeletonMerger>();
            if (target == null) target = FindFirstObjectByType<PointCloudCapsuleFilter>();
        }

        private void LateUpdate()
        {
            if (target == null) return;
            if (bodyTracking == null)
            {
                target.Clear();
                return;
            }
            bodyTracking.PublishBoneCapsulesWorld(target, tubeRadius);
        }

        private void OnDisable()
        {
            if (target != null) target.Clear();
        }
    }
}
