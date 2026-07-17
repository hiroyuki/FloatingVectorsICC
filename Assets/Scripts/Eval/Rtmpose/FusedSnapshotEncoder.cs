// Shared fused-skeleton → BodySnapshot conversion. The fused EvalSkeleton
// lives in the origin-camera (world) color frame in mm; bodies_main stores
// k4abt DEPTH-frame joints per camera. This maps world -> that camera's color
// frame (inverse GlobalTrColorCamera) -> depth frame (inverse D2C), writes the
// EvalSkeleton joints onto their k4abt ids, and synthesizes the joints the
// 15-joint eval skeleton lacks (spine navel/chest, clavicles from
// pelvis→neck, nose from head) at LOW confidence.
//
// Single source of truth for what used to be three hand-synced copies:
// FusedBodiesExport (editor export), LiveFusedBodySource (live path) and
// FusedTakeConverter (runtime visitor-take conversion) all call Build().

using Orbbec;
using UnityEngine;

namespace BodyTracking.Eval.Rtmpose
{
    public static class FusedSnapshotEncoder
    {
        /// <summary>
        /// Fill <paramref name="snap"/> (reused caller-owned instance) with the fused
        /// skeleton converted into one camera's depth frame. Joints not covered by the
        /// eval skeleton (or invalid this frame) are reset to confidence NONE.
        /// </summary>
        /// <param name="colorToWorld">The camera's GlobalTrColorCamera (color → world, mm).</param>
        /// <param name="depthToColor">The camera's D2C extrinsic (depth → color, mm).</param>
        public static void Build(BodySnapshot snap, EvalSkeleton p,
                                 in ObExtrinsic colorToWorld, in ObExtrinsic depthToColor)
        {
            for (int i = 0; i < K4ABTConsts.K4ABT_JOINT_COUNT; i++)
                snap.Joints[i] = new k4abt_joint_t
                {
                    Position = new k4a_float3_t(),
                    Orientation = new k4a_quaternion_t { W = 1f },
                    ConfidenceLevel = k4abt_joint_confidence_level_t.K4ABT_JOINT_CONFIDENCE_NONE,
                };

            for (int j = 0; j < EvalSkeleton.JointCount; j++)
            {
                if (!p.Joints[j].Valid) continue;
                Vector3 d = ToDepth(p.Joints[j].PositionMm, colorToWorld, depthToColor);
                SetJoint(snap, EvalSkeletonMap.K4abtSource[j], d,
                         k4abt_joint_confidence_level_t.K4ABT_JOINT_CONFIDENCE_MEDIUM);
            }

            bool hasPelvis = p.Joints[(int)EvalJointId.Pelvis].Valid;
            bool hasNeck = p.Joints[(int)EvalJointId.Neck].Valid;
            if (hasPelvis && hasNeck)
            {
                Vector3 pel = ToDepth(p.Joints[(int)EvalJointId.Pelvis].PositionMm, colorToWorld, depthToColor);
                Vector3 nk = ToDepth(p.Joints[(int)EvalJointId.Neck].PositionMm, colorToWorld, depthToColor);
                SetJoint(snap, k4abt_joint_id_t.K4ABT_JOINT_SPINE_NAVEL, Vector3.Lerp(pel, nk, 1f / 3f),
                         k4abt_joint_confidence_level_t.K4ABT_JOINT_CONFIDENCE_LOW);
                SetJoint(snap, k4abt_joint_id_t.K4ABT_JOINT_SPINE_CHEST, Vector3.Lerp(pel, nk, 2f / 3f),
                         k4abt_joint_confidence_level_t.K4ABT_JOINT_CONFIDENCE_LOW);
                SetJoint(snap, k4abt_joint_id_t.K4ABT_JOINT_CLAVICLE_LEFT, nk,
                         k4abt_joint_confidence_level_t.K4ABT_JOINT_CONFIDENCE_LOW);
                SetJoint(snap, k4abt_joint_id_t.K4ABT_JOINT_CLAVICLE_RIGHT, nk,
                         k4abt_joint_confidence_level_t.K4ABT_JOINT_CONFIDENCE_LOW);
            }
            if (p.Joints[(int)EvalJointId.Head].Valid)
            {
                Vector3 hd = ToDepth(p.Joints[(int)EvalJointId.Head].PositionMm, colorToWorld, depthToColor);
                SetJoint(snap, k4abt_joint_id_t.K4ABT_JOINT_NOSE, hd,
                         k4abt_joint_confidence_level_t.K4ABT_JOINT_CONFIDENCE_LOW);
            }
        }

        /// <summary>world mm → this camera's color frame → depth frame (both inverses).</summary>
        public static Vector3 ToDepth(Vector3 worldMm, in ObExtrinsic colorToWorld, in ObExtrinsic depthToColor)
            => InverseTransform(InverseTransform(worldMm, colorToWorld), depthToColor);

        static void SetJoint(BodySnapshot snap, k4abt_joint_id_t id, Vector3 posMm,
                             k4abt_joint_confidence_level_t conf)
        {
            snap.Joints[(int)id] = new k4abt_joint_t
            {
                Position = new k4a_float3_t { X = posMm.x, Y = posMm.y, Z = posMm.z },
                Orientation = new k4a_quaternion_t { W = 1f },
                ConfidenceLevel = conf,
            };
        }

        static Vector3 InverseTransform(Vector3 p, in ObExtrinsic e)
        {
            var R = e.Rot; var T = e.Trans;
            float x = p.x - T[0], y = p.y - T[1], z = p.z - T[2];
            return new Vector3(
                R[0] * x + R[3] * y + R[6] * z,
                R[1] * x + R[4] * y + R[7] * z,
                R[2] * x + R[5] * y + R[8] * z);
        }
    }
}
