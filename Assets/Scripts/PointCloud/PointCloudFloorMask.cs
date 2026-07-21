// Hides the bare floor from the point cloud, keeping only the patch a visitor is
// standing on. Consumed by PointCloudUnlit.shader's PassFloor via
// PointCloudShaderFilters.
//
// Why: the floor returns a full-density sheet of points that shimmers with the
// sensors' depth noise (measured sigma ~8 mm on this rig). Across a 3 m square
// that shimmer is the most eye-catching thing on screen and it carries no
// meaning. The points under someone's feet do carry meaning — they read as
// contact, weight, presence — so those are kept and the rest of the floor is
// dropped.
//
// With the mask on and nobody tracked, the floor disappears completely. That is
// the specified behaviour, not a degraded fallback: an empty room should show an
// empty stage rather than a shimmering rectangle.
//
// This component only holds state. Feet are pushed in every frame by
// BodyTracking.BodyFloorMaskFeeder (same split as PointCloudJointMotionField /
// BodyJointMotionFeeder), which keeps the PointCloud assembly free of any
// body-tracking dependency.

using UnityEngine;

namespace PointCloud
{
    [DisallowMultipleComponent]
    public class PointCloudFloorMask : MonoBehaviour
    {
        // 4 people x 2 feet. The piece is designed for a single visitor, so this
        // is pure headroom; it must match the _FloorFoot[8] array length in
        // PointCloudUnlit.shader.
        public const int MaxFeet = 8;

        public enum FilterMode
        {
            Disabled = 0,   // whole cloud drawn, floor included
            Enabled = 1,    // floor hidden except near feet
        }

        [Tooltip("Disabled = draw the floor as usual. Enabled = hide points below " +
                 "Floor Height except within Keep Radius of a tracked foot.")]
        public FilterMode mode = FilterMode.Enabled;

        [Tooltip("Points below this world height are subject to the mask (m). The " +
                 "floor sits at y=0 after floor levelling and the depth noise is " +
                 "~8 mm, so this only needs to clear the noise — keep it under the " +
                 "ankle joint height (~0.10 m) or feet get masked too. 0 is a valid " +
                 "threshold (mask exactly what is below the levelled floor); use " +
                 "Mode = Disabled to switch the mask off.")]
        public float floorHeight = 0.01f;

        [Tooltip("Horizontal radius around each foot in which floor points survive (m).")]
        public float keepRadius = 0.5f;

        private readonly Vector4[] _feet = new Vector4[MaxFeet];
        private int _footCount;

        public FilterMode Mode => mode;

        /// <summary>True when this mask should actually be applied. Separate from
        /// FloorHeight so that a height of exactly 0 — a legitimate threshold once
        /// the floor is levelled to y=0 — still masks, instead of reading as "off".
        /// Mode is the only switch that disables the mask.</summary>
        public bool MaskActive => mode != FilterMode.Disabled;

        public float FloorHeight => Mathf.Max(0f, floorHeight);
        public float KeepRadius => Mathf.Max(0f, keepRadius);

        /// <summary>Foot world positions; only the first <see cref="FootCount"/>
        /// entries are meaningful. Backing array is reused — do not retain.</summary>
        public Vector4[] Feet => _feet;
        public int FootCount => _footCount;

        /// <summary>Drop all feet. Call before re-adding this frame's set; a frame
        /// with no bodies then correctly masks the whole floor.</summary>
        public void Clear() => _footCount = 0;

        /// <summary>Add one foot in world space. Returns false when full.</summary>
        public bool TryAdd(Vector3 worldPos)
        {
            if (_footCount >= MaxFeet) return false;
            _feet[_footCount++] = new Vector4(worldPos.x, worldPos.y, worldPos.z, 0f);
            return true;
        }

        private void OnDisable() => Clear();
    }
}
