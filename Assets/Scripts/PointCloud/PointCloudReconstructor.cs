// GPU depth -> color point cloud reconstruction. One ComputeShader.Dispatch
// per call writes vertex positions + colors directly into the owned Mesh's
// vertex buffer. Used by both live capture (PointCloudRenderer) and recorded
// playback (PointCloudRecorder) — previously this logic was duplicated
// verbatim between those two callers.
//
// Lifecycle: caller `new`s one per renderer/track, plugs the Mesh property
// into its MeshFilter, calls Dispatch per frame, and Dispose()s on teardown.
// All GPU buffers / scratch arrays / shader load state are owned here.
//
// Mesh layout: position float3 + color float3 stream 0, identity index buffer,
// IndexFormat.UInt32, vertexBufferTarget includes GraphicsBuffer.Target.Raw
// so the compute kernel can write via RWByteAddressBuffer.
//
// Resize: capacity = depthWidth * depthHeight. If the caller passes new
// dimensions whose product exceeds the current mesh capacity, the mesh is
// torn down and rebuilt. GPU input buffers grow on the same trigger.

using System;
using Orbbec;
using Unity.Collections;
using UnityEngine;
using UnityEngine.Rendering;

namespace PointCloud
{
    public sealed class PointCloudReconstructor : IDisposable
    {
        // Shared compute shader cache. Same kernel + property IDs used by every
        // instance in the project — point cloud reconstruction is one fixed
        // algorithm, no per-instance customisation.
        private static ComputeShader s_shader;
        private static int s_kernel = -1;
        private static readonly int kId_Depth    = Shader.PropertyToID("_Depth");
        private static readonly int kId_Color    = Shader.PropertyToID("_Color");
        private static readonly int kId_Out      = Shader.PropertyToID("_Out");
        private static readonly int kId_DepthW   = Shader.PropertyToID("_DepthW");
        private static readonly int kId_DepthH   = Shader.PropertyToID("_DepthH");
        private static readonly int kId_ColorW   = Shader.PropertyToID("_ColorW");
        private static readonly int kId_ColorH   = Shader.PropertyToID("_ColorH");
        private static readonly int kId_HasColor = Shader.PropertyToID("_HasColor");
        private static readonly int kId_FxD = Shader.PropertyToID("_FxD");
        private static readonly int kId_FyD = Shader.PropertyToID("_FyD");
        private static readonly int kId_CxD = Shader.PropertyToID("_CxD");
        private static readonly int kId_CyD = Shader.PropertyToID("_CyD");
        private static readonly int kId_FxC = Shader.PropertyToID("_FxC");
        private static readonly int kId_FyC = Shader.PropertyToID("_FyC");
        private static readonly int kId_CxC = Shader.PropertyToID("_CxC");
        private static readonly int kId_CyC = Shader.PropertyToID("_CyC");
        private static readonly int kId_Rrow0 = Shader.PropertyToID("_Rrow0");
        private static readonly int kId_Rrow1 = Shader.PropertyToID("_Rrow1");
        private static readonly int kId_Rrow2 = Shader.PropertyToID("_Rrow2");
        private static readonly int kId_T     = Shader.PropertyToID("_T");

        private readonly string _name;
        private Mesh _mesh;
        private int _meshCapacity;
        private GraphicsBuffer _depthGpu;
        private GraphicsBuffer _colorGpu;
        private uint[] _depthScratchU32;
        private uint[] _colorScratchU32;

        /// <summary>Mesh receiving the reconstructed point cloud. Plug into MeshFilter.sharedMesh.
        /// Lazy-created on first <see cref="EnsureMesh"/> or <see cref="Dispatch"/>.</summary>
        public Mesh Mesh => _mesh;

        public PointCloudReconstructor(string name) { _name = name ?? "PointCloud"; }

        /// <summary>
        /// Pre-allocate the mesh at the given capacity so the caller can assign
        /// it to MeshFilter.sharedMesh before the first Dispatch. capacity =
        /// depthWidth * depthHeight (one vertex per depth pixel; invalid pixels
        /// are pushed offscreen by the compute kernel).
        /// </summary>
        public void EnsureMesh(int capacity)
        {
            if (_mesh != null && _meshCapacity >= capacity) return;

            if (_mesh != null)
            {
                if (Application.isPlaying) UnityEngine.Object.Destroy(_mesh);
                else UnityEngine.Object.DestroyImmediate(_mesh);
            }

            _mesh = new Mesh
            {
                name = $"PointCloudMesh_{_name}",
                indexFormat = IndexFormat.UInt32,
                // Wide static bounds: per-vertex positions get clip-culled invalid
                // pixels far offscreen, so a recalculate-after-Dispatch would be
                // wrong (read-back) and unnecessary (positions stable enough).
                bounds = new Bounds(Vector3.zero, Vector3.one * 100f),
            };
            // RWByteAddressBuffer access from the compute kernel needs Raw target.
            _mesh.vertexBufferTarget |= GraphicsBuffer.Target.Raw;
            var attrs = new[]
            {
                new VertexAttributeDescriptor(VertexAttribute.Position, VertexAttributeFormat.Float32, 3, stream: 0),
                new VertexAttributeDescriptor(VertexAttribute.Color,    VertexAttributeFormat.Float32, 3, stream: 0),
            };
            _mesh.SetVertexBufferParams(capacity, attrs);
            _mesh.SetIndexBufferParams(capacity, IndexFormat.UInt32);
            // Identity index buffer for MeshTopology.Points.
            var indices = new NativeArray<uint>(capacity, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
            try
            {
                for (int i = 0; i < capacity; i++) indices[i] = (uint)i;
                _mesh.SetIndexBufferData(indices, 0, 0, capacity,
                    MeshUpdateFlags.DontRecalculateBounds | MeshUpdateFlags.DontValidateIndices);
            }
            finally { indices.Dispose(); }
            _mesh.SetSubMesh(0, new SubMeshDescriptor(0, capacity, MeshTopology.Points),
                MeshUpdateFlags.DontRecalculateBounds | MeshUpdateFlags.DontValidateIndices);
            _meshCapacity = capacity;
        }

        /// <summary>
        /// Collapse the submesh to zero so nothing renders. Used when the caller
        /// has no intrinsics yet but the Mesh is still bound to a MeshFilter
        /// (don't leave stale points visible from the previous frame).
        /// </summary>
        public void BlankMesh()
        {
            if (_mesh == null) return;
            _mesh.SetSubMesh(0, new SubMeshDescriptor(0, 0, MeshTopology.Points),
                MeshUpdateFlags.DontRecalculateBounds | MeshUpdateFlags.DontValidateIndices);
        }

        /// <summary>
        /// Upload depth + color frames into GPU buffers and dispatch the
        /// reconstruction kernel. Returns false if the compute shader could not
        /// be loaded — caller logs / disables features as appropriate.
        /// </summary>
        public bool Dispatch(
            byte[] depthBytes, int depthByteCount, int depthW, int depthH,
            byte[] colorBytes, int colorByteCount, int colorW, int colorH,
            in ObCameraParam cam)
        {
            if (!TryLoadShader()) return false;

            int capacity = depthW * depthH;
            EnsureMesh(capacity);
            EnsureDepthBuffer(depthByteCount);
            // ColorGpu's stride is 4 (uint). Even when hasColor is false we
            // bind a tiny placeholder buffer so the compute kernel doesn't
            // sample a stale earlier-frame buffer.
            bool hasColor = colorByteCount > 0 && colorW > 0 && colorH > 0;
            EnsureColorBuffer(hasColor ? colorByteCount : 4);

            // Buffer stride is 4 (uint), so we BlockCopy raw bytes into the
            // scratch u32 array first.
            Buffer.BlockCopy(depthBytes, 0, _depthScratchU32, 0, depthByteCount);
            _depthGpu.SetData(_depthScratchU32, 0, 0, _depthScratchU32.Length);
            if (hasColor)
            {
                Buffer.BlockCopy(colorBytes, 0, _colorScratchU32, 0, colorByteCount);
                _colorGpu.SetData(_colorScratchU32, 0, 0, _colorScratchU32.Length);
            }

            var shader = s_shader;
            int k = s_kernel;
            // GetVertexBuffer returns a temporary that must be Dispose()'d.
            var vbuf = _mesh.GetVertexBuffer(0);
            try
            {
                shader.SetBuffer(k, kId_Depth, _depthGpu);
                shader.SetBuffer(k, kId_Color, _colorGpu);
                shader.SetBuffer(k, kId_Out,   vbuf);
                shader.SetInt(kId_DepthW, depthW);
                shader.SetInt(kId_DepthH, depthH);
                shader.SetInt(kId_ColorW, colorW);
                shader.SetInt(kId_ColorH, colorH);
                shader.SetInt(kId_HasColor, hasColor ? 1 : 0);
                shader.SetFloat(kId_FxD, cam.DepthIntrinsic.Fx);
                shader.SetFloat(kId_FyD, cam.DepthIntrinsic.Fy);
                shader.SetFloat(kId_CxD, cam.DepthIntrinsic.Cx);
                shader.SetFloat(kId_CyD, cam.DepthIntrinsic.Cy);
                shader.SetFloat(kId_FxC, cam.RgbIntrinsic.Fx);
                shader.SetFloat(kId_FyC, cam.RgbIntrinsic.Fy);
                shader.SetFloat(kId_CxC, cam.RgbIntrinsic.Cx);
                shader.SetFloat(kId_CyC, cam.RgbIntrinsic.Cy);
                var R = cam.Transform.Rot;
                shader.SetVector(kId_Rrow0, new Vector4(R[0], R[1], R[2], 0f));
                shader.SetVector(kId_Rrow1, new Vector4(R[3], R[4], R[5], 0f));
                shader.SetVector(kId_Rrow2, new Vector4(R[6], R[7], R[8], 0f));
                var T = cam.Transform.Trans;
                shader.SetVector(kId_T, new Vector4(T[0], T[1], T[2], 0f));

                int gx = (depthW + 7) / 8;
                int gy = (depthH + 7) / 8;
                shader.Dispatch(k, gx, gy, 1);
            }
            finally { vbuf.Dispose(); }

            // Re-assert full-capacity submesh after Dispatch — EnsureMesh sets it
            // initially but a caller may have BlankMesh()'d when intrinsics were
            // briefly missing. Idempotent if already at full capacity.
            _mesh.SetSubMesh(0, new SubMeshDescriptor(0, capacity, MeshTopology.Points),
                MeshUpdateFlags.DontRecalculateBounds | MeshUpdateFlags.DontValidateIndices);
            return true;
        }

        private static bool TryLoadShader()
        {
            if (s_shader != null && s_kernel >= 0) return true;
            s_shader = Resources.Load<ComputeShader>("PointCloudReconstruct");
            if (s_shader == null) return false;
            s_kernel = s_shader.FindKernel("CSMain");
            return s_kernel >= 0;
        }

        private void EnsureDepthBuffer(int byteCount)
        {
            int u32 = Math.Max(1, (byteCount + 3) / 4);
            if (_depthGpu != null && _depthScratchU32 != null && _depthScratchU32.Length >= u32) return;
            _depthGpu?.Dispose();
            _depthGpu = new GraphicsBuffer(GraphicsBuffer.Target.Raw, u32, sizeof(uint));
            _depthScratchU32 = new uint[u32];
        }

        private void EnsureColorBuffer(int byteCount)
        {
            int u32 = Math.Max(1, (byteCount + 3) / 4);
            if (_colorGpu != null && _colorScratchU32 != null && _colorScratchU32.Length >= u32) return;
            _colorGpu?.Dispose();
            _colorGpu = new GraphicsBuffer(GraphicsBuffer.Target.Raw, u32, sizeof(uint));
            _colorScratchU32 = new uint[u32];
        }

        public void Dispose()
        {
            _depthGpu?.Dispose(); _depthGpu = null;
            _colorGpu?.Dispose(); _colorGpu = null;
            _depthScratchU32 = null;
            _colorScratchU32 = null;
            if (_mesh != null)
            {
                if (Application.isPlaying) UnityEngine.Object.Destroy(_mesh);
                else UnityEngine.Object.DestroyImmediate(_mesh);
                _mesh = null;
            }
            _meshCapacity = 0;
        }
    }
}
