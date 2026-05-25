// Per-joint motion field consumed by the point cloud shader (issue #24).
// Each entry is a single BT joint sample: world-space position + velocity
// (m/s). The shader assigns every point its nearest joint and uses that
// joint's velocity to visualize motion + optionally cull points whose
// nearest joint is too far away (the "vertex filter" called out in the
// issue). Live and playback paths share the same field via
// PointCloudShaderFilters.Apply (same pattern as PointCloudCapsuleFilter).
//
// Producers (BodyTracking.BodyJointMotionFeeder) call Clear + TryAdd each
// LateUpdate. Consumers (PointCloudShaderFilters writing into the
// PointCloudRenderer / PointCloudRecorder material property block) read
// JointPos / JointVel / JointCount / Mode at render time.

using UnityEngine;

namespace PointCloud
{
    [DisallowMultipleComponent]
    public class PointCloudJointMotionField : MonoBehaviour
    {
        public enum FilterMode
        {
            Disabled = 0,
            Enabled = 1,
        }

        public enum ColorMode
        {
            // Pass-through: vertex color stays untouched; only displacement and
            // distance culling are honored.
            None = 0,
            // Tint each point toward HotColor as the nearest joint's speed
            // approaches SpeedMax.
            MagnitudeHeatmap = 1,
            // Map the unit velocity vector into RGB ((dir + 1) / 2). Stationary
            // joints render as neutral grey.
            Direction = 2,
        }

        // Shader array uniforms must have a fixed compile-time size. Bumping
        // requires updating PointCloudUnlit.shader's _MotionPos / _MotionVel
        // declarations in lockstep. 64 covers 32 joints * 2 bodies — enough
        // headroom for the brief two-body flapping mentioned in CLAUDE.md
        // while staying single-person by design.
        public const int MaxJoints = 64;

        [Tooltip("Disabled = pass-through (no motion visualization, no distance cull). " +
                 "Enabled = nearest-joint motion drives color modulation, optional " +
                 "displacement, and (when Max Assign Distance > 0) vertex culling.")]
        public FilterMode filterMode = FilterMode.Enabled;

        [Tooltip("How the nearest joint's motion is rendered. None = data attached " +
                 "to every point but not visualized (useful when combined with " +
                 "Displace Vertices). MagnitudeHeatmap = speed-driven tint. Direction = " +
                 "RGB encodes the unit velocity vector.")]
        public ColorMode colorMode = ColorMode.MagnitudeHeatmap;

        [Min(0f)]
        [Tooltip("Maximum distance (m) between a point and its nearest joint for " +
                 "the point to render. Points whose nearest joint is farther are " +
                 "culled in the vertex shader. 0 disables the cull (visualize all " +
                 "points). This is the per-vertex filter the issue calls out.")]
        public float maxAssignDistance = 0.5f;

        [Tooltip("If true, vertex position is offset by nearestJoint.velocity * " +
                 "displaceScale in world space. Combine with a small scale (~0.05) " +
                 "for a streak / motion-blur look.")]
        public bool displaceVertices = false;

        [Min(0f)]
        [Tooltip("Multiplier applied to velocity (m/s) when displaceVertices is on. " +
                 "Roughly: streak length (m) = speed * scale.")]
        public float displaceScale = 0f;

        [Min(0.01f)]
        [Tooltip("Speed (m/s) that maps to the hot end of the MagnitudeHeatmap palette. " +
                 "Speeds above this saturate at HotColor.")]
        public float speedMax = 2f;

        [Tooltip("Hot end of the MagnitudeHeatmap palette. The cold end is the " +
                 "point's original vertex color (untouched).")]
        public Color hotColor = new Color(1f, 0.2f, 0.05f, 1f);

        // xyz = world joint position, w = unused (reserved for future per-joint
        // influence radius).
        private readonly Vector4[] _jointPos = new Vector4[MaxJoints];
        // xyz = world joint velocity (m/s), w = |velocity| (precomputed so the
        // shader doesn't recompute length per inner-loop iteration).
        private readonly Vector4[] _jointVel = new Vector4[MaxJoints];
        private int _jointCount;

        public FilterMode Mode => filterMode;
        public ColorMode Coloring => colorMode;
        public int JointCount => _jointCount;
        public Vector4[] JointPos => _jointPos;
        public Vector4[] JointVel => _jointVel;

        public void Clear()
        {
            _jointCount = 0;
        }

        /// <summary>
        /// Append one joint sample. Returns false if the buffer is full (caller
        /// should stop submitting). The shader honors w = speed in _jointVel
        /// so we cache it here once per Add.
        /// </summary>
        public bool TryAdd(Vector3 worldPos, Vector3 worldVel)
        {
            if (_jointCount >= MaxJoints) return false;
            float speed = worldVel.magnitude;
            _jointPos[_jointCount] = new Vector4(worldPos.x, worldPos.y, worldPos.z, 0f);
            _jointVel[_jointCount] = new Vector4(worldVel.x, worldVel.y, worldVel.z, speed);
            _jointCount++;
            return true;
        }
    }
}
