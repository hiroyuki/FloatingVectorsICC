using UnityEngine;

namespace Shared
{
    /// <summary>A GPU triangle-soup surface other systems can seed points from: an
    /// appended buffer of world-space triangles, each 3 x (float3 pos + float3 colour)
    /// = 72B, plus the DrawProceduralIndirect args whose [0] holds the drawn vertex
    /// count (triangles * 3, capacity-clamped). Implemented by TSDFView; consumed by
    /// PointCloudMotionCurves. Lives in Shared because TSDF already references
    /// BodyTracking — the consumer must discover the surface through an interface,
    /// not a TSDF type, to avoid a circular assembly reference.</summary>
    public interface ITriangleSeedSource
    {
        /// <summary>Append buffer of triangles (72B stride), world space. Null until allocated.</summary>
        ComputeBuffer TriangleBuffer { get; }

        /// <summary>Indirect draw args; [0] = vertex count = triangle count * 3, capacity-clamped.</summary>
        ComputeBuffer TriangleArgsBuffer { get; }

        /// <summary>TriangleBuffer capacity in triangles (upper bound for dispatch sizing).</summary>
        int MaxTriangles { get; }

        /// <summary>True while the surface is being extracted and drawn — the buffers hold a
        /// current mesh. False when the producing view is hidden or in another mode (the
        /// buffer would be stale).</summary>
        bool TrianglesReady { get; }
    }
}
