using UnityEngine;

namespace Shared
{
    /// <summary>The displayed TSDF SDF volume, exposed so OTHER geometry (the motion
    /// curves) can cast the body's self-shadow onto itself: a fragment marches this
    /// volume toward the light and darkens when the body occludes it. Implemented by
    /// TSDFView; consumed by PointCloudMotionCurves. Lives in Shared for the same
    /// reason as <see cref="ITriangleSeedSource"/> — TSDF already references
    /// BodyTracking, so the consumer must reach the volume through an interface, not a
    /// TSDF type, to avoid a circular assembly reference.
    ///
    /// The march parameters (light direction, reach, bias, occluder threshold) come
    /// from here too, so the curves inherit EXACTLY the mesh's shadow so the two read
    /// as one lit scene. Only the strength is the consumer's own knob.</summary>
    public interface ISdfShadowVolume
    {
        /// <summary>True while the volume's front SDF buffer is valid and the mesh that
        /// defines the shadow is being drawn. False -> consumers skip the shadow (leave
        /// curves at full brightness) rather than sampling a stale/empty buffer.</summary>
        bool SdfShadowReady { get; }

        /// <summary>Front SDF buffer, one float2 (x = sdf metres, y = weight) per voxel,
        /// laid out x + Dim.x*(y + Dim.y*z). The same buffer the mesh shades from.</summary>
        ComputeBuffer SdfBuffer { get; }

        /// <summary>Maps a world point to voxel-centre coords (index + 0.5).</summary>
        Matrix4x4 SdfVoxelFromWorld { get; }

        /// <summary>Volume dimensions in voxels.</summary>
        Vector3Int SdfDim { get; }

        /// <summary>Minimum voxel weight to treat a sample as observed (the mesh's gate).</summary>
        float SdfMinWeight { get; }

        /// <summary>Voxel edge length (m) — the shadow march's step size.</summary>
        float SdfVoxelSize { get; }

        /// <summary>Truncation distance (m); the occluder threshold is a fraction of it.</summary>
        float SdfTau { get; }

        /// <summary>Direction the light comes FROM (need not be normalised). Shared with
        /// the mesh's diffuse + shadow so the cast direction matches.</summary>
        Vector3 ShadowLightDir { get; }

        /// <summary>March reach in voxel steps.</summary>
        float ShadowSteps { get; }

        /// <summary>March start bias in voxels (skip the fragment's own shell).</summary>
        float ShadowBiasVox { get; }

        /// <summary>Occluder threshold as a fraction of tau.</summary>
        float ShadowIsoFrac { get; }
    }
}
