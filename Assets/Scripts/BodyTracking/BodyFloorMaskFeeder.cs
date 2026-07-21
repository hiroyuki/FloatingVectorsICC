// Pushes the tracked feet into a PointCloud.PointCloudFloorMask every
// LateUpdate, so the point-cloud shader can keep the floor only where someone is
// standing. Mirrors BodyJointMotionFeeder: SkeletonMerger refreshes skeletons in
// Update, so by LateUpdate the joints are this frame's, and the shader picks the
// values up next frame — the same one-frame lag the OBB / capsule / motion paths
// already accept.
//
// Both the ankle and the foot joint are published per side. k4abt's ankle sits a
// few cm above the floor and its foot joint sits at the toes; publishing both
// stretches the kept patch along the shoe instead of centring it on the ankle,
// which otherwise clips the toe end when the visitor stands with feet apart.

using System.Collections.Generic;
using UnityEngine;
using PointCloud;

namespace BodyTracking
{
    [DisallowMultipleComponent]
    public class BodyFloorMaskFeeder : MonoBehaviour
    {
        [Tooltip("Body-tracking source. Leave empty to auto-resolve the first " +
                 "SkeletonMerger in the scene at OnEnable.")]
        public SkeletonMerger bodyTracking;

        [Tooltip("Floor mask that receives the foot world positions. Leave empty to " +
                 "auto-resolve the first PointCloudFloorMask in the scene at OnEnable.")]
        public PointCloudFloorMask target;

        private static readonly k4abt_joint_id_t[] kFootJoints =
        {
            k4abt_joint_id_t.K4ABT_JOINT_ANKLE_LEFT,
            k4abt_joint_id_t.K4ABT_JOINT_FOOT_LEFT,
            k4abt_joint_id_t.K4ABT_JOINT_ANKLE_RIGHT,
            k4abt_joint_id_t.K4ABT_JOINT_FOOT_RIGHT,
        };

        private void OnEnable()
        {
            if (bodyTracking == null) bodyTracking = FindFirstObjectByType<SkeletonMerger>();
            if (target == null) target = FindFirstObjectByType<PointCloudFloorMask>();
        }

        [Tooltip("Keep publishing the last known feet for this long after tracking " +
                 "drops them (s). k4abt loses a joint for a frame or two routinely; " +
                 "without this the floor patch under a standing visitor blinks out " +
                 "and back on every dropout. Long enough to ride out the noise, short " +
                 "enough that the floor still clears promptly when they walk away.")]
        [Min(0f)]
        public float footHoldSeconds = 0.5f;

        private readonly List<Vector3> _lastFeet = new List<Vector3>(4);
        private float _lastFeetTime = float.NegativeInfinity;

        private void LateUpdate()
        {
            if (target == null) return;
            target.Clear();
            if (bodyTracking == null || !bodyTracking.isActiveAndEnabled)
            {
                _lastFeet.Clear();
                return;
            }

            bool any = false;
            for (int i = 0; i < kFootJoints.Length; i++)
            {
                if (!bodyTracking.TryGetJointWorld(kFootJoints[i], out Vector3 world)) continue;
                if (!any) { _lastFeet.Clear(); any = true; }
                _lastFeet.Add(world);
                target.TryAdd(world);
            }

            if (any) { _lastFeetTime = Time.time; return; }

            // Nothing tracked this frame. Within the hold window this is a dropout,
            // so republish the last set; past it the visitor has genuinely left and
            // the empty set hides the whole floor, which is the intended empty stage.
            if (Time.time - _lastFeetTime > footHoldSeconds) { _lastFeet.Clear(); return; }
            for (int i = 0; i < _lastFeet.Count; i++) target.TryAdd(_lastFeet[i]);
        }

        private void OnDisable()
        {
            if (target != null) target.Clear();
            _lastFeet.Clear();
            _lastFeetTime = float.NegativeInfinity;
        }
    }
}
