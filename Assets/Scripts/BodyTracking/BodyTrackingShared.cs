// Shared, stateless helpers used by every BodyTracking consumer
// (MultiLive, Playback, BodyVisual, SkeletonWorldTransform, tests).
// Previously these lived as statics on BodyTrackingLive; the class is gone
// and they belong somewhere neutral.

using UnityEngine;

namespace BodyTracking
{
    public static class BodyTrackingShared
    {
        // Convert a K4A camera-frame point (mm, right-handed: +X right, +Y down, +Z forward)
        // into Unity local coordinates (m, left-handed: +Y up).
        public static Vector3 K4AmmToUnity(in k4a_float3_t p)
        {
            return new Vector3(p.X * 0.001f, -p.Y * 0.001f, p.Z * 0.001f);
        }

        public static readonly (k4abt_joint_id_t a, k4abt_joint_id_t b)[] Bones = new[]
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
            // WRIST is the arm tip: HAND/HANDTIP/THUMB are dropped project-wide because
            // k4abt cannot resolve them reliably from these camera views (confidence
            // collapses to NONE across all 4 cams in every recording, so the joint is a
            // wild ~2 m prediction that stretches the bone). See IsDrawnJoint below.

            // right arm
            (k4abt_joint_id_t.K4ABT_JOINT_SPINE_CHEST, k4abt_joint_id_t.K4ABT_JOINT_CLAVICLE_RIGHT),
            (k4abt_joint_id_t.K4ABT_JOINT_CLAVICLE_RIGHT, k4abt_joint_id_t.K4ABT_JOINT_SHOULDER_RIGHT),
            (k4abt_joint_id_t.K4ABT_JOINT_SHOULDER_RIGHT, k4abt_joint_id_t.K4ABT_JOINT_ELBOW_RIGHT),
            (k4abt_joint_id_t.K4ABT_JOINT_ELBOW_RIGHT, k4abt_joint_id_t.K4ABT_JOINT_WRIST_RIGHT),
            // (right HAND/HANDTIP/THUMB bones dropped — see left arm note)

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

        // Project-wide policy: the hand joints past the wrist (HAND/HANDTIP/THUMB, both
        // sides) are NOT drawn or used. k4abt reports them at confidence NONE from every
        // camera view we have (verified across the jump / turn / walking recordings), so
        // their position is a wild prediction that only produces stretched bones and
        // floating blobs. WRIST is the reliable arm tip. Consumers (BodyVisual joint
        // spheres/trails, TSDFTrailBaker ribbons) gate on this so the exclusion lives in
        // one place. The Bones table above already omits the hand bones for the same reason.
        public static bool IsDrawnJoint(k4abt_joint_id_t j)
        {
            switch (j)
            {
                case k4abt_joint_id_t.K4ABT_JOINT_HAND_LEFT:
                case k4abt_joint_id_t.K4ABT_JOINT_HANDTIP_LEFT:
                case k4abt_joint_id_t.K4ABT_JOINT_THUMB_LEFT:
                case k4abt_joint_id_t.K4ABT_JOINT_HAND_RIGHT:
                case k4abt_joint_id_t.K4ABT_JOINT_HANDTIP_RIGHT:
                case k4abt_joint_id_t.K4ABT_JOINT_THUMB_RIGHT:
                    return false;
                default:
                    return true;
            }
        }
    }
}
