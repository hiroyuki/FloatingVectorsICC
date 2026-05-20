// Shared, stateless helpers used by every BodyTracking consumer
// (MultiLive, Playback, BodyVisual, SkeletonWorldTransform, tests).
// Previously these lived as statics on BodyTrackingLive; the class is gone
// and they belong somewhere neutral.

using UnityEngine;

namespace BodyTracking
{
    public static class BodyTrackingShared
    {
        public enum TrailColorMode { PerJointHue, FlatColor, PerBody, AccelHeatmap, FrameHue }

        // Inputs for TrailColorMode.FrameHue: hue is driven by Time.frameCount,
        // oscillating sinusoidally around CenterHue with amplitude HueRange/2.
        // Saturation/Value are constants (HSV mode), CyclePeriodFrames sets the
        // wavelength of the oscillation. All values clamp to sane ranges in
        // FrameHueRGB so out-of-range Inspector values stay safe.
        public struct FrameHueParams
        {
            public float CenterHue;        // [0, 1)
            public float HueRange;         // [0, 1] full sweep width around CenterHue
            public float Saturation;       // [0, 1]
            public float Value;            // [0, 1]
            public float CyclePeriodFrames; // > 0; sin period in frames
        }

        /// <summary>
        /// Convert the FrameHue parameters at the given frame index into an RGB color.
        /// Hue cycles as: h = (CenterHue + HueRange/2 * sin(2π * frame / period)) mod 1.
        /// </summary>
        public static UnityEngine.Color FrameHueRGB(in FrameHueParams p, int frame)
        {
            float period = p.CyclePeriodFrames > 0f ? p.CyclePeriodFrames : 1f;
            float phase = 2f * UnityEngine.Mathf.PI * (frame / period);
            float halfRange = UnityEngine.Mathf.Clamp01(p.HueRange) * 0.5f;
            float rawHue = p.CenterHue + halfRange * UnityEngine.Mathf.Sin(phase);
            float h = UnityEngine.Mathf.Repeat(rawHue, 1f);
            float s = UnityEngine.Mathf.Clamp01(p.Saturation);
            float v = UnityEngine.Mathf.Clamp01(p.Value);
            return UnityEngine.Color.HSVToRGB(h, s, v);
        }

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
    }
}
