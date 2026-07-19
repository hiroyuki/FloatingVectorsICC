// Common skeleton format for the body-tracking evaluation harness.
//
// Every tracker under evaluation (k4abt / Nuitrack / RTMPose) normalizes its
// native output into EvalSkeleton so metrics and visualization are tracker-
// agnostic. Positions are kept in the k4abt convention — millimeters, OpenCV
// camera frame (+X right, +Y down, +Z forward, right-handed) — so the k4abt
// baseline needs no conversion and Unity overlay reuses BodyTrackingShared.

using System.Collections.Generic;
using UnityEngine;

namespace BodyTracking.Eval
{
    /// <summary>
    /// Canonical 15-joint subset shared by k4abt (32), Nuitrack (~19) and
    /// RTMPose / COCO-17. Only joints all three can produce (directly or by a
    /// trivial derivation) are included, so cross-tracker comparison is fair.
    /// </summary>
    public enum EvalJointId
    {
        Pelvis = 0,
        Neck,
        Head,
        ShoulderL,
        ElbowL,
        WristL,
        ShoulderR,
        ElbowR,
        WristR,
        HipL,
        KneeL,
        AnkleL,
        HipR,
        KneeR,
        AnkleR,
        Count
    }

    public struct EvalJoint
    {
        /// <summary>Camera-space position, OpenCV frame, millimeters.</summary>
        public Vector3 PositionMm;
        /// <summary>Normalized 0..1 confidence (tracker-native scale mapped in the adapter).</summary>
        public float Confidence;
        /// <summary>False when the tracker did not produce this joint this frame.</summary>
        public bool Valid;
    }

    /// <summary>One person's pose for one frame in the common format.</summary>
    public sealed class EvalSkeleton
    {
        public const int JointCount = (int)EvalJointId.Count; // 15

        /// <summary>Tracker-assigned body id (for ID-swap analysis).</summary>
        public int PersonId;
        public ulong TimestampNs;
        public readonly EvalJoint[] Joints = new EvalJoint[JointCount];

        public void Reset(int personId, ulong tsNs)
        {
            PersonId = personId;
            TimestampNs = tsNs;
            for (int i = 0; i < Joints.Length; i++)
            {
                Joints[i].Valid = false;
                Joints[i].Confidence = 0f;
                Joints[i].PositionMm = Vector3.zero;
            }
        }

        public ref EvalJoint Joint(EvalJointId id) => ref Joints[(int)id];

        /// <summary>Count of joints flagged valid — used to pick the primary body.</summary>
        public int ValidCount()
        {
            int n = 0;
            for (int i = 0; i < Joints.Length; i++) if (Joints[i].Valid) n++;
            return n;
        }
    }

    /// <summary>All bodies a tracker produced for one source frame.</summary>
    public sealed class EvalSkeletonFrame
    {
        public string TrackerName;
        public string Serial;
        public ulong TimestampNs;
        public readonly List<EvalSkeleton> Bodies = new List<EvalSkeleton>();

        public void Reset(string tracker, string serial, ulong tsNs)
        {
            TrackerName = tracker;
            Serial = serial;
            TimestampNs = tsNs;
            Bodies.Clear();
        }

        /// <summary>
        /// Body with the most valid joints, or null when there is no body with at
        /// least one valid joint (an all-invalid body is not a tracked pose and
        /// must not count toward continuity).
        /// </summary>
        public EvalSkeleton Primary()
        {
            EvalSkeleton best = null;
            int bestN = 0; // require >=1 valid joint
            for (int i = 0; i < Bodies.Count; i++)
            {
                int n = Bodies[i].ValidCount();
                if (n > bestN) { bestN = n; best = Bodies[i]; }
            }
            return best;
        }
    }

    /// <summary>k4abt(32) → Eval(15) mapping and conversion helpers.</summary>
    public static class EvalSkeletonMap
    {
        /// <summary>Source k4abt joint per EvalJointId (index-aligned to the enum).</summary>
        public static readonly k4abt_joint_id_t[] K4abtSource = new k4abt_joint_id_t[(int)EvalJointId.Count]
        {
            k4abt_joint_id_t.K4ABT_JOINT_PELVIS,         // Pelvis
            k4abt_joint_id_t.K4ABT_JOINT_NECK,           // Neck
            k4abt_joint_id_t.K4ABT_JOINT_HEAD,           // Head
            k4abt_joint_id_t.K4ABT_JOINT_SHOULDER_LEFT,  // ShoulderL
            k4abt_joint_id_t.K4ABT_JOINT_ELBOW_LEFT,     // ElbowL
            k4abt_joint_id_t.K4ABT_JOINT_WRIST_LEFT,     // WristL
            k4abt_joint_id_t.K4ABT_JOINT_SHOULDER_RIGHT, // ShoulderR
            k4abt_joint_id_t.K4ABT_JOINT_ELBOW_RIGHT,    // ElbowR
            k4abt_joint_id_t.K4ABT_JOINT_WRIST_RIGHT,    // WristR
            k4abt_joint_id_t.K4ABT_JOINT_HIP_LEFT,       // HipL
            k4abt_joint_id_t.K4ABT_JOINT_KNEE_LEFT,      // KneeL
            k4abt_joint_id_t.K4ABT_JOINT_ANKLE_LEFT,     // AnkleL
            k4abt_joint_id_t.K4ABT_JOINT_HIP_RIGHT,      // HipR
            k4abt_joint_id_t.K4ABT_JOINT_KNEE_RIGHT,     // KneeR
            k4abt_joint_id_t.K4ABT_JOINT_ANKLE_RIGHT,    // AnkleR
        };

        // k4abt confidence levels are 0..3 (NONE/LOW/MEDIUM/HIGH); MEDIUM is the
        // current SDK ceiling. Normalize by 3 so 0..1 spans the defined range.
        private const float ConfDenom = 3f;

        /// <summary>Populate <paramref name="dst"/> from a k4abt body snapshot.</summary>
        public static void FromK4abt(BodySnapshot src, ulong tsNs, EvalSkeleton dst)
        {
            dst.Reset(unchecked((int)src.Id), tsNs);
            for (int e = 0; e < K4abtSource.Length; e++)
            {
                var jt = src.Joints[(int)K4abtSource[e]];
                ref var oj = ref dst.Joints[e];
                oj.PositionMm = new Vector3(jt.Position.X, jt.Position.Y, jt.Position.Z);
                oj.Confidence = Mathf.Clamp01((float)jt.ConfidenceLevel / ConfDenom);
                // k4abt confidence NONE(0) means out-of-range/unreliable; treat as
                // present-but-invalid for the strict eval (LOW counts as valid but low).
                oj.Valid = jt.ConfidenceLevel > k4abt_joint_confidence_level_t.K4ABT_JOINT_CONFIDENCE_NONE;
            }
        }

        /// <summary>Convert an Eval camera-space mm position to Unity meters (Y-up).</summary>
        public static Vector3 CameraMmToUnity(Vector3 mm) => new Vector3(mm.x, -mm.y, mm.z) * 0.001f;
    }
}
