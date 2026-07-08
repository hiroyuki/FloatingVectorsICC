// Shared capsule-bake plumbing for TSDFTrailBake.compute (3-7 dedup): the 48-byte
// segment struct, the volume/segment bind block, and the full-grid batched dispatch
// were duplicated between TSDFTrailBaker (BakeCore / StampNewSegments) and
// TSDFPrintExporter (BakeSegs, "Same dispatch pattern as TSDFTrailBaker.BakeCore").
// One definition here keeps the C# layout and the kernel contract in one place.

using UnityEngine;

namespace TSDF
{
    /// <summary>Shared helpers for baking capsule segments into a TSDF volume via
    /// Resources/TSDFTrailBake.compute (BakeTrail full-grid / BakeTrailBox kernels).</summary>
    internal static class TrailBakeOps
    {
        /// <summary>GPU-side capsule, must match struct Seg in TSDFTrailBake.compute
        /// (48 bytes = 12 floats: endpoints, taper radii, colour, pad).</summary>
        public struct TrailSeg
        {
            public Vector3 a;
            public Vector3 b;
            public float ra;
            public float rb;
            public Vector3 color;
            public float pad;
        }

        /// <summary>ComputeBuffer stride for <see cref="TrailSeg"/>.</summary>
        public const int SegStride = sizeof(float) * 12;

        /// <summary>
        /// Bind the per-bake uniforms + buffers shared by BOTH TSDFTrailBake kernels:
        /// grid mapping, segment source, volume write targets, and the write-set
        /// active-block marking (baked voxels must mark occupancy or active-block MC
        /// drops the sculpture — the marks ride the same Publish/swap).
        /// </summary>
        public static void BindBakeTargets(ComputeShader shader, int kernel,
                                           TSDFVolume volume, ComputeBuffer segs)
        {
            var dim = volume.Dim;
            shader.SetInts("_Dim", dim.x, dim.y, dim.z);
            shader.SetFloat("_Tau", volume.Tau);
            shader.SetMatrix("_WorldFromVoxel", volume.WorldFromVoxel);
            shader.SetBuffer(kernel, "_Segs", segs);
            shader.SetBuffer(kernel, "_VoxelsOut", volume.WriteBuffer);
            shader.SetBuffer(kernel, "_ColorsOut", volume.WriteColorBuffer);
            volume.BindBlockMarking(shader, kernel, volume.WriteBlockActive);
        }

        /// <summary>
        /// Full-grid capsule bake (BakeTrail kernel): min-union <paramref name="segCount"/>
        /// segments into the volume write buffer, batched over the segment list so a single
        /// dispatch never runs long enough to trip the GPU TDR watchdog (min-union is
        /// idempotent, so batching is safe). Returns the number of dispatches issued.
        /// </summary>
        public static int BakeSegments(ComputeShader shader, int kernel, TSDFVolume volume,
                                       ComputeBuffer segs, int segCount, int batchSize)
        {
            BindBakeTargets(shader, kernel, volume, segs);
            var dim = volume.Dim;
            int total = dim.x * dim.y * dim.z;
            int batches = 0;
            for (int off = 0; off < segCount; off += batchSize)
            {
                shader.SetInt("_SegOffset", off);
                shader.SetInt("_SegCount", Mathf.Min(batchSize, segCount - off));
                TSDFComputeUtil.DispatchLinear(shader, kernel, total);
                batches++;
            }
            return batches;
        }
    }
}
