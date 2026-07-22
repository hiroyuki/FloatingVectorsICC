// GPU depth -> color point cloud reconstruction. One ComputeShader.Dispatch
// per call writes vertex positions + colors directly into the owned Mesh's
// vertex buffer. Used by both live capture (PointCloudRenderer) and recorded
// playback (SensorRecorder) — previously this logic was duplicated
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

        /// <summary>
        /// Debug A/B switch: when true the kernel ignores the undistortion LUT and
        /// back-projects with the bare pinhole model, so the lens-distortion
        /// correction can be toggled live (e.g. to compare point clouds with and
        /// without it). Affects every reconstructor instance. Default false.
        /// </summary>
        public static bool ForcePinhole = false;
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
        private static readonly int kId_RayLut    = Shader.PropertyToID("_RayLut");
        private static readonly int kId_HasRayLut = Shader.PropertyToID("_HasRayLut");
        private static readonly int kId_SpeckleRadius    = Shader.PropertyToID("_SpeckleRadius");
        private static readonly int kId_SpeckleMinNeigh  = Shader.PropertyToID("_SpeckleMinNeighbours");
        private static readonly int kId_SpeckleTolMm     = Shader.PropertyToID("_SpeckleTolMm");
        private static readonly int kId_SpeckleTolFrac   = Shader.PropertyToID("_SpeckleTolFrac");

        /// <summary>
        /// Local-density rejection applied in the kernel: a depth pixel is only
        /// emitted when enough of its neighbours sit at nearly the same depth.
        /// Kills the mid-air chatter (flying pixels at silhouette edges, small
        /// floating blobs) that no world-space filter can distinguish from real
        /// surface points. Radius 0 disables it, which is the struct's default —
        /// a caller that never assigns <see cref="Speckle"/> gets the old
        /// behaviour unchanged.
        /// </summary>
        [Serializable]
        public struct SpeckleSettings
        {
            [Tooltip("Neighbourhood half-width in depth pixels. 0 = off. 1 = 3x3 " +
                     "(isolated flying pixels only), 2 = 5x5, 3 = 7x7 (kills blobs " +
                     "up to roughly the window size). Cost is (2r+1)^2 depth loads " +
                     "per pixel — negligible at 320x288 even at r=3.")]
            [Range(0, 3)] public int radius;

            [Tooltip("How many of the (2r+1)^2-1 neighbours must agree for the " +
                     "point to survive. Higher = more aggressive; too high erodes " +
                     "genuine silhouette edges.")]
            [Min(0)] public int minNeighbours;

            [Tooltip("Depth agreement tolerance (mm) at close range.")]
            [Min(0f)] public float tolMm;

            [Tooltip("Depth agreement tolerance as a fraction of range — sensor " +
                     "depth noise grows with distance, so the tolerance has to as " +
                     "well. Effective tolerance = max(tolMm, tolFrac * z).")]
            [Min(0f)] public float tolFrac;

            /// <summary>Tuned on the 4-camera rig: 5x5 window, half the neighbours
            /// must agree. Removes the airborne chatter while leaving the body's
            /// own silhouette intact.</summary>
            public static SpeckleSettings Default => new SpeckleSettings
            {
                radius = 2, minNeighbours = 12, tolMm = 30f, tolFrac = 0.01f,
            };

            public bool Enabled => radius > 0 && minNeighbours > 0;
        }

        /// <summary>Speckle rejection for this reconstructor's next Dispatch.
        /// Owner may reassign every frame (Inspector-driven tuning).</summary>
        public SpeckleSettings Speckle;

        private readonly string _name;
        private Mesh _mesh;
        private int _meshCapacity;
        private GraphicsBuffer _depthGpu;
        private GraphicsBuffer _colorGpu;
        private uint[] _depthScratchU32;
        private uint[] _colorScratchU32;

        // Undistorted-ray LUT (StructuredBuffer<float2>). Null until a depth
        // stream with real distortion is seen; rebuilt only when the depth
        // intrinsics / distortion / resolution change. When null the kernel
        // falls back to the pinhole back-projection (identical to no distortion).
        private GraphicsBuffer _rayLutGpu;
        private bool _rayLutValid;
        private int _rayLutW, _rayLutH;
        private ObCameraIntrinsic _rayLutIntr;
        private ObCameraDistortion _rayLutDist;

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
                PointCloudUtil.DestroySafe(_mesh);

            // Raw vertex-buffer target: the compute kernel writes via
            // RWByteAddressBuffer. Full submesh [0, capacity): every Dispatch
            // fills the entire buffer (invalid pixels are pushed offscreen by
            // the kernel, not compacted).
            _mesh = PointCloudMeshUtil.CreatePointMesh($"PointCloudMesh_{_name}", capacity,
                rawVertexBufferTarget: true, initialSubmeshCount: capacity);
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
            EnsureRayLut(cam.DepthIntrinsic, cam.DepthDistortion, depthW, depthH);

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
                // Always bind a buffer (the placeholder when no LUT) so the
                // StructuredBuffer slot is never left dangling between frames.
                shader.SetBuffer(k, kId_RayLut, _rayLutGpu);
                shader.SetInt(kId_HasRayLut, (_rayLutValid && !ForcePinhole) ? 1 : 0);
                bool speckle = Speckle.Enabled;
                shader.SetInt(kId_SpeckleRadius, speckle ? Speckle.radius : 0);
                shader.SetInt(kId_SpeckleMinNeigh, speckle ? Speckle.minNeighbours : 0);
                shader.SetFloat(kId_SpeckleTolMm, Speckle.tolMm);
                shader.SetFloat(kId_SpeckleTolFrac, Speckle.tolFrac);
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

        // Rebuild the undistorted-ray LUT only when the depth intrinsics,
        // distortion, or resolution change (rare — effectively session-static).
        // A null LUT (no/unsupported distortion) leaves _rayLutValid false and
        // binds a 1-element placeholder so the kernel uses the pinhole path.
        private void EnsureRayLut(in ObCameraIntrinsic intr, in ObCameraDistortion dist, int depthW, int depthH)
        {
            bool unchanged = _rayLutGpu != null &&
                             _rayLutW == depthW && _rayLutH == depthH &&
                             IntrinsicsEqual(_rayLutIntr, intr) && DistortionEqual(_rayLutDist, dist);
            if (unchanged) return;

            _rayLutIntr = intr;
            _rayLutDist = dist;
            _rayLutW = depthW;
            _rayLutH = depthH;

            Vector2[] lut = DepthUndistortLut.Build(intr, dist, depthW, depthH);
            _rayLutGpu?.Dispose();
            if (lut != null)
            {
                _rayLutGpu = new GraphicsBuffer(GraphicsBuffer.Target.Structured, lut.Length, sizeof(float) * 2);
                _rayLutGpu.SetData(lut);
                _rayLutValid = true;
            }
            else
            {
                // Placeholder so the StructuredBuffer<float2> slot stays bound.
                _rayLutGpu = new GraphicsBuffer(GraphicsBuffer.Target.Structured, 1, sizeof(float) * 2);
                _rayLutValid = false;
            }
        }

        private static bool IntrinsicsEqual(in ObCameraIntrinsic a, in ObCameraIntrinsic b)
            => a.Fx == b.Fx && a.Fy == b.Fy && a.Cx == b.Cx && a.Cy == b.Cy;

        private static bool DistortionEqual(in ObCameraDistortion a, in ObCameraDistortion b)
            => a.K1 == b.K1 && a.K2 == b.K2 && a.K3 == b.K3 && a.K4 == b.K4 &&
               a.K5 == b.K5 && a.K6 == b.K6 && a.P1 == b.P1 && a.P2 == b.P2 && a.Model == b.Model;

        public void Dispose()
        {
            _depthGpu?.Dispose(); _depthGpu = null;
            _colorGpu?.Dispose(); _colorGpu = null;
            _rayLutGpu?.Dispose(); _rayLutGpu = null;
            _rayLutValid = false;
            _depthScratchU32 = null;
            _colorScratchU32 = null;
            if (_mesh != null)
            {
                PointCloudUtil.DestroySafe(_mesh);
                _mesh = null;
            }
            _meshCapacity = 0;
        }
    }
}
