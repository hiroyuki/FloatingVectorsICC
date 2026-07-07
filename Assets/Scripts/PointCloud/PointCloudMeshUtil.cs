// Shared factory for the project's point-cloud Mesh layout:
// position float3 + color float3 interleaved in stream 0 (24 bytes/vertex,
// matches OBColorPoint), IndexFormat.UInt32, identity index buffer,
// MeshTopology.Points submesh. Previously duplicated across
// PointCloudReconstructor.EnsureMesh, PointCloudRenderer.BuildMesh (CPU
// branch) and PointCloudCumulative.CaptureSnapshotGpu.

using Unity.Collections;
using UnityEngine;
using UnityEngine.Rendering;

namespace PointCloud
{
    internal static class PointCloudMeshUtil
    {
        /// <summary>
        /// Create a fully-initialised point mesh.
        ///
        /// The mesh is "complete" on return: vertex params + index params +
        /// identity index data + an initial SubMesh. This full initialisation
        /// order matters for compute-write consumers — Unity 6 leaves the
        /// underlying vertex GPU buffer in an uncommitted state if the
        /// index-side init is missing, and a compute kernel's _Dst.Store3
        /// writes via GetVertexBuffer(0) silently go nowhere (confirmed by
        /// GetData readback showing all-zeros despite a non-zero atomic
        /// counter). Do NOT reorder or drop steps here.
        /// </summary>
        /// <param name="name">Mesh name.</param>
        /// <param name="capacity">Vertex / index count to allocate.</param>
        /// <param name="rawVertexBufferTarget">Add GraphicsBuffer.Target.Raw to
        /// vertexBufferTarget so a compute kernel can write the vertex buffer
        /// as an RWByteAddressBuffer.</param>
        /// <param name="initialSubmeshCount">Index count of the initial
        /// SubMesh (0 = starts empty, sized later by the caller).</param>
        /// <param name="sharedIndices">Optional caller-owned identity index
        /// array (length >= capacity). When not created, a Temp identity array
        /// is built and disposed internally.</param>
        public static Mesh CreatePointMesh(string name, int capacity, bool rawVertexBufferTarget,
                                           int initialSubmeshCount = 0,
                                           NativeArray<uint> sharedIndices = default)
        {
            var mesh = new Mesh
            {
                name = name,
                indexFormat = IndexFormat.UInt32,
                // Wide static bounds: per-vertex positions may put invalid
                // pixels far offscreen (clip-culled), so a recalculate would be
                // wrong (GPU read-back) and unnecessary (positions stable enough).
                bounds = new Bounds(Vector3.zero, Vector3.one * 100f),
            };
            // RWByteAddressBuffer access from a compute kernel needs Raw target.
            if (rawVertexBufferTarget)
                mesh.vertexBufferTarget |= GraphicsBuffer.Target.Raw;
            // Vertex layout = OBColorPoint (position float3 + color float3, 24 bytes).
            var attrs = new[]
            {
                new VertexAttributeDescriptor(VertexAttribute.Position, VertexAttributeFormat.Float32, 3, stream: 0),
                new VertexAttributeDescriptor(VertexAttribute.Color,    VertexAttributeFormat.Float32, 3, stream: 0),
            };
            mesh.SetVertexBufferParams(capacity, attrs);
            mesh.SetIndexBufferParams(capacity, IndexFormat.UInt32);
            // Identity index buffer for MeshTopology.Points.
            if (sharedIndices.IsCreated)
            {
                mesh.SetIndexBufferData(sharedIndices, 0, 0, capacity,
                    MeshUpdateFlags.DontRecalculateBounds | MeshUpdateFlags.DontValidateIndices);
            }
            else
            {
                // (try/finally instead of `using` because `using` makes the variable
                //  readonly, which blocks NativeArray's indexer set on a struct.)
                var indices = new NativeArray<uint>(capacity, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
                try
                {
                    for (int i = 0; i < capacity; i++) indices[i] = (uint)i;
                    mesh.SetIndexBufferData(indices, 0, 0, capacity,
                        MeshUpdateFlags.DontRecalculateBounds | MeshUpdateFlags.DontValidateIndices);
                }
                finally { indices.Dispose(); }
            }
            mesh.SetSubMesh(0, new SubMeshDescriptor(0, initialSubmeshCount, MeshTopology.Points),
                MeshUpdateFlags.DontRecalculateBounds | MeshUpdateFlags.DontValidateIndices);
            return mesh;
        }
    }
}
