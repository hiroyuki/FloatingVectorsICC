// Capsule-union filter for point clouds, used by PointCloudRenderer / Recorder
// playback / Cumulative. A "capsule" is a line segment (endpoint A → endpoint
// B) with a radius. KeepInside passes only points whose distance to ANY
// capsule's segment is ≤ that capsule's radius. KeepOutside passes only points
// outside every capsule. Disabled = pass-through.
//
// Capsule data is held in world space (single source of truth across multiple
// renderers / playback meshes). Producers (e.g. BodyTracking.BodyTubeCapsule-
// Feeder) call Clear + TryAdd each frame; consumers (the shader filter writer
// in PointCloudShaderFilters, and the CPU pre-filter in PointCloudCumulative)
// read CapsuleA / CapsuleB / CapsuleCount / Mode at render time.
//
// MaxCapsules is sized for the single-person installation use case (CLAUDE.md):
// 31 anatomical bones × 1 body = 31, with headroom to cover the brief two-body
// transitions when k4abt's body id flaps between pops.

using UnityEngine;

namespace PointCloud
{
    [DisallowMultipleComponent]
    public class PointCloudCapsuleFilter : MonoBehaviour
    {
        public enum FilterMode
        {
            Disabled = 0,
            KeepInside = 1,
            KeepOutside = 2,
        }

        // Shader array uniforms must have a fixed compile-time size. Bump only if
        // the shader's _CapsA / _CapsB array declarations are bumped in lockstep.
        public const int MaxCapsules = 64;

        [Tooltip("Disabled = pass all points through. KeepInside = keep points inside the union " +
                 "of capsules. KeepOutside = keep points outside every capsule.")]
        public FilterMode filterMode = FilterMode.KeepInside;

        // xyz = world-space endpoint, w = capsule radius (capsuleA only; capsuleB
        // shares the radius from its paired A entry).
        private readonly Vector4[] _capsuleA = new Vector4[MaxCapsules];
        private readonly Vector4[] _capsuleB = new Vector4[MaxCapsules];
        private int _capsuleCount;

        public FilterMode Mode => filterMode;
        public int CapsuleCount => _capsuleCount;
        public Vector4[] CapsuleA => _capsuleA;
        public Vector4[] CapsuleB => _capsuleB;

        public void Clear()
        {
            _capsuleCount = 0;
        }

        /// <summary>
        /// Append one capsule. Returns false if the buffer is full (caller should
        /// stop submitting). radius is clamped to ≥ 0.
        /// </summary>
        public bool TryAdd(Vector3 worldA, Vector3 worldB, float radius)
        {
            if (_capsuleCount >= MaxCapsules) return false;
            float r = Mathf.Max(0f, radius);
            _capsuleA[_capsuleCount] = new Vector4(worldA.x, worldA.y, worldA.z, r);
            _capsuleB[_capsuleCount] = new Vector4(worldB.x, worldB.y, worldB.z, 0f);
            _capsuleCount++;
            return true;
        }

        /// <summary>
        /// True iff <paramref name="worldPoint"/> falls inside at least one capsule
        /// of the union. Used by PointCloudCumulative's CPU pre-filter when
        /// snapshotting tube-interior points.
        /// </summary>
        public bool ContainsWorldPoint(Vector3 worldPoint)
        {
            for (int i = 0; i < _capsuleCount; i++)
            {
                Vector4 a4 = _capsuleA[i];
                Vector4 b4 = _capsuleB[i];
                float r = a4.w;
                Vector3 a = new Vector3(a4.x, a4.y, a4.z);
                Vector3 b = new Vector3(b4.x, b4.y, b4.z);
                Vector3 ab = b - a;
                float abLen2 = ab.sqrMagnitude;
                if (abLen2 < 1e-12f)
                {
                    if ((worldPoint - a).sqrMagnitude <= r * r) return true;
                    continue;
                }
                float t = Mathf.Clamp01(Vector3.Dot(worldPoint - a, ab) / abLen2);
                Vector3 closest = a + t * ab;
                if ((worldPoint - closest).sqrMagnitude <= r * r) return true;
            }
            return false;
        }
    }
}
