// Unified visualisation of the TSDF volume. One component, three modes
// switchable from the Inspector (or via the View Mode field at runtime):
//
//   Voxel : sphere per voxel coloured by SDF (red = surface, white = +tau,
//           black = -tau, hidden when weight == 0 or sdf above the hide
//           threshold). Use for SDF correctness checks.
//   Cell  : cube per Marching-Cubes cell whose 8 corner SDFs straddle the
//           iso level AND all weights pass the gate. Use to verify "where
//           MC would emit a triangle" before drilling into the triTable.
//   Mesh  : the actual MC mesh output. Dispatches the MC compute kernel,
//           feeds an AppendStructuredBuffer of vertices, draws via
//           DrawProceduralIndirect.
//
// All three modes read the SAME ComputeBuffer owned by TSDFVolume — no CPU
// readback, no buffer duplication.

using UnityEngine;
using UnityEngine.Rendering;

namespace TSDF
{
    [AddComponentMenu("TSDF/View")]
    [DefaultExecutionOrder(10)]
    public sealed class TSDFView : MonoBehaviour
    {
        public enum ViewMode
        {
            Voxel,
            Cell,
            Mesh,
        }

        [Header("References")]
        [Tooltip("Volume to visualise. Auto-located on enable.")]
        public TSDFVolume volume;

        [Header("Mode")]
        [Tooltip("Which representation to draw this frame. Switchable at runtime.")]
        public ViewMode mode = ViewMode.Mesh;

        // ---------- Voxel mode ----------
        [Header("Voxel view")]
        [Tooltip("Material that uses the TSDF/Voxel shader. Auto-created if null.")]
        public Material voxelMaterial;
        [Range(0.05f, 1f)]
        [Tooltip("Sphere diameter as a fraction of voxelSize.")]
        public float voxelSphereScale = 0.15f;
        [Tooltip("Voxels with sdf above this threshold (in metres) are hidden. " +
                 "Set to 0.05 (= one voxelSize) to hide the free-space white shell; " +
                 "set to volume.Tau to show everything.")]
        public float voxelHideAboveSdf = 0.05f;

        // ---------- Cell mode ----------
        [Header("Cell view")]
        [Tooltip("Material that uses the TSDF/Cell shader. Auto-created if null.")]
        public Material cellMaterial;
        [Range(0.05f, 1f)]
        [Tooltip("Cube diameter as a fraction of voxelSize.")]
        public float cellCubeScale = 0.30f;
        [Tooltip("Boundary-cell colour. Cells whose corner SDFs straddle the " +
                 "iso level and pass the weight gate get drawn in this colour.")]
        public Color cellColor = new Color(0.2f, 1f, 0.5f, 1f);

        // ---------- Mesh mode ----------
        [Header("Mesh view")]
        [Tooltip("Material that uses the TSDF/Mesh shader. Auto-created if null.")]
        public Material meshMaterial;
        [Tooltip("Max triangles the MC output buffer can hold. 333k tris = 1 M " +
                 "vertices, ~12 MB at 36 bytes per triangle (= 3 × float3).")]
        [Min(1024)] public int meshMaxTriangles = 333_333;
        [Tooltip("Iso-level for MC. 0 = surface where SDF changes sign.")]
        public float meshIsoLevel = 0f;
        [Tooltip("Minimum corner weight for MC to consider a cell. Matches the " +
                 "Cell view's weight gate so the two views show the same set of cells.")]
        [Min(0f)] public float meshMinWeight = 0.5f;
        [Tooltip("Run MC every Nth frame. 1 = every frame. Draws keep using the " +
                 "last extracted vertex buffer on skip frames.")]
        [Min(1)] public int mcEveryNFrames = 1;


        [Header("Diagnostics")]
        public bool diagnosticLogging = false;

        // ---- shared state ----
        private Mesh _sphereMesh;
        private Mesh _cubeMesh;
        private ComputeShader _mcShader;
        private int _mcKernel;
        private int _scaleArgsKernel;
        private TSDFIntegrator _integrator;
        // Last batch index for which we dispatched MC. The mesh on screen
        // therefore corresponds to a COMPLETE multi-cam observation cycle,
        // not a half-finished one — which is what produced the flicker /
        // partial-body artifacts in the earlier interval-based clear scheme.
        private int _lastDispatchedBatch = -1;

        private ComputeBuffer _voxelArgsBuffer;
        private ComputeBuffer _cellArgsBuffer;
        private ComputeBuffer _meshTrianglesBuffer;  // AppendStructuredBuffer<Tri> (= 3 float3)
        private ComputeBuffer _meshArgsBuffer;       // IndirectArguments [vertCount, 1, 0, 0]
        private int _frameCounter;

        private bool _ownsVoxelMat;
        private bool _ownsCellMat;
        private bool _ownsMeshMat;

        private float _diagWindowStart;
        private int _diagMcDispatchesThisWindow;

        // ---------------------------------------------------------------
        private void OnEnable()
        {
            if (volume == null) volume = FindAnyObjectByType<TSDFVolume>(FindObjectsInactive.Include);
            if (volume == null)
            {
                Debug.LogError("[TSDFView] No TSDFVolume in scene; disabling.", this);
                enabled = false;
                return;
            }
            EnsurePrimitiveMeshes();
            EnsureMaterials();
            EnsureBuffers();
            EnsureMcShader();
            _integrator = FindAnyObjectByType<TSDFIntegrator>(FindObjectsInactive.Include);
        }

        private void OnDisable()
        {
            ReleaseBuffers();
            DestroyOwnedMaterials();
        }

        private void EnsurePrimitiveMeshes()
        {
            if (_sphereMesh == null) _sphereMesh = BorrowPrimitiveMesh(PrimitiveType.Sphere);
            if (_cubeMesh   == null) _cubeMesh   = BorrowPrimitiveMesh(PrimitiveType.Cube);
        }

        private static Mesh BorrowPrimitiveMesh(PrimitiveType type)
        {
            var temp = GameObject.CreatePrimitive(type);
            var mesh = temp.GetComponent<MeshFilter>().sharedMesh;
            if (Application.isPlaying) Destroy(temp); else DestroyImmediate(temp);
            return mesh;
        }

        private void EnsureMaterials()
        {
            if (voxelMaterial == null)
            {
                var sh = Shader.Find("TSDF/Voxel");
                if (sh == null) { Debug.LogError("[TSDFView] Shader \"TSDF/Voxel\" not found."); enabled = false; return; }
                voxelMaterial = new Material(sh) { name = "TSDF Voxel (auto)", hideFlags = HideFlags.DontSave };
                _ownsVoxelMat = true;
            }
            if (cellMaterial == null)
            {
                var sh = Shader.Find("TSDF/Cell");
                if (sh == null) { Debug.LogError("[TSDFView] Shader \"TSDF/Cell\" not found."); enabled = false; return; }
                cellMaterial = new Material(sh) { name = "TSDF Cell (auto)", hideFlags = HideFlags.DontSave };
                _ownsCellMat = true;
            }
            if (meshMaterial == null)
            {
                var sh = Shader.Find("TSDF/TSDFMesh");
                if (sh == null) { Debug.LogError("[TSDFView] Shader \"TSDF/TSDFMesh\" not found."); enabled = false; return; }
                meshMaterial = new Material(sh) { name = "TSDF Mesh (auto)", hideFlags = HideFlags.DontSave };
                _ownsMeshMat = true;
            }
        }

        private void DestroyOwnedMaterials()
        {
            if (_ownsVoxelMat && voxelMaterial != null) { DestroyMat(ref voxelMaterial); _ownsVoxelMat = false; }
            if (_ownsCellMat  && cellMaterial  != null) { DestroyMat(ref cellMaterial);  _ownsCellMat  = false; }
            if (_ownsMeshMat  && meshMaterial  != null) { DestroyMat(ref meshMaterial);  _ownsMeshMat  = false; }
        }

        private static void DestroyMat(ref Material m)
        {
            if (Application.isPlaying) Destroy(m); else DestroyImmediate(m);
            m = null;
        }

        private void EnsureBuffers()
        {
            // Indirect-args buffers for the two instanced views. Layout (5 uints):
            // [indexCount, instanceCount, startIndex, baseVertex, startInstance].
            if (_voxelArgsBuffer == null)
                _voxelArgsBuffer = new ComputeBuffer(5, sizeof(uint), ComputeBufferType.IndirectArguments);
            if (_cellArgsBuffer == null)
                _cellArgsBuffer = new ComputeBuffer(5, sizeof(uint), ComputeBufferType.IndirectArguments);

            // Mesh-view AppendBuffer of Tri = 3 × float3 = 36 bytes per slot.
            const int triStride = 36;
            if (_meshTrianglesBuffer == null || _meshTrianglesBuffer.count != meshMaxTriangles)
            {
                _meshTrianglesBuffer?.Release();
                _meshTrianglesBuffer = new ComputeBuffer(meshMaxTriangles, triStride, ComputeBufferType.Append);
            }
            // 4-uint indirect args buffer for DrawProceduralIndirect: [vertCount, 1, 0, 0]
            if (_meshArgsBuffer == null)
            {
                _meshArgsBuffer = new ComputeBuffer(4, sizeof(uint), ComputeBufferType.IndirectArguments);
                _meshArgsBuffer.SetData(new uint[] { 0, 1, 0, 0 });
            }
        }

        private void ReleaseBuffers()
        {
            _voxelArgsBuffer?.Release();     _voxelArgsBuffer = null;
            _cellArgsBuffer?.Release();      _cellArgsBuffer = null;
            _meshTrianglesBuffer?.Release(); _meshTrianglesBuffer = null;
            _meshArgsBuffer?.Release();      _meshArgsBuffer = null;
        }

        private void EnsureMcShader()
        {
            if (_mcShader != null) return;
            _mcShader = Resources.Load<ComputeShader>("TSDFMarchingCubes");
            if (_mcShader == null)
            {
                Debug.LogError("[TSDFView] Compute shader \"Resources/TSDFMarchingCubes.compute\" not found.", this);
                enabled = false;
                return;
            }
            _mcKernel = _mcShader.FindKernel("MarchCubes");
            _scaleArgsKernel = _mcShader.FindKernel("ScaleArgs");
        }

        // ---------------------------------------------------------------
        private void Update()
        {
            if (volume == null || volume.VoxelBuffer == null) return;
            EnsureBuffers();
            switch (mode)
            {
                case ViewMode.Voxel: DrawVoxels(); break;
                case ViewMode.Cell:  DrawCells();  break;
                case ViewMode.Mesh:  DrawMesh();   break;
            }
            if (diagnosticLogging) PerSecondDiag();
        }

        // ---------- Voxel ----------
        private void DrawVoxels()
        {
            int total = volume.Dim.x * volume.Dim.y * volume.Dim.z;
            var args = new uint[5]
            {
                _sphereMesh.GetIndexCount(0),
                (uint)total,
                _sphereMesh.GetIndexStart(0),
                _sphereMesh.GetBaseVertex(0),
                0u,
            };
            _voxelArgsBuffer.SetData(args);

            voxelMaterial.SetBuffer("_Voxels", volume.VoxelBuffer);
            voxelMaterial.SetMatrix("_WorldFromVoxel", volume.WorldFromVoxel);
            voxelMaterial.SetVector("_Dim", new Vector4(volume.Dim.x, volume.Dim.y, volume.Dim.z, 0));
            voxelMaterial.SetFloat("_VoxelSize", volume.voxelSize);
            voxelMaterial.SetFloat("_Tau", volume.Tau);
            voxelMaterial.SetFloat("_Scale", voxelSphereScale);
            voxelMaterial.SetFloat("_HideAboveSdf", voxelHideAboveSdf);

            Graphics.DrawMeshInstancedIndirect(
                _sphereMesh, 0, voxelMaterial, BuildBounds(voxelSphereScale),
                _voxelArgsBuffer, 0, null, ShadowCastingMode.Off, false, gameObject.layer);
        }

        // ---------- Cell ----------
        private void DrawCells()
        {
            int cdx = Mathf.Max(1, volume.Dim.x - 1);
            int cdy = Mathf.Max(1, volume.Dim.y - 1);
            int cdz = Mathf.Max(1, volume.Dim.z - 1);
            int total = cdx * cdy * cdz;
            var args = new uint[5]
            {
                _cubeMesh.GetIndexCount(0),
                (uint)total,
                _cubeMesh.GetIndexStart(0),
                _cubeMesh.GetBaseVertex(0),
                0u,
            };
            _cellArgsBuffer.SetData(args);

            cellMaterial.SetBuffer("_Voxels", volume.VoxelBuffer);
            cellMaterial.SetMatrix("_WorldFromVoxel", volume.WorldFromVoxel);
            cellMaterial.SetVector("_Dim", new Vector4(volume.Dim.x, volume.Dim.y, volume.Dim.z, 0));
            cellMaterial.SetFloat("_VoxelSize", volume.voxelSize);
            cellMaterial.SetFloat("_Scale", cellCubeScale);
            cellMaterial.SetFloat("_MinWeight", meshMinWeight);
            cellMaterial.SetFloat("_IsoLevel", meshIsoLevel);
            cellMaterial.SetColor("_Color", cellColor);

            Graphics.DrawMeshInstancedIndirect(
                _cubeMesh, 0, cellMaterial, BuildBounds(cellCubeScale),
                _cellArgsBuffer, 0, null, ShadowCastingMode.Off, false, gameObject.layer);
        }

        // ---------- Mesh ----------
        private void DrawMesh()
        {
            _frameCounter++;

            // Re-extract MC only when the integrator finished a new multi-cam
            // batch. Between batches the previous triangles buffer keeps
            // drawing — that gives a stable mesh that updates ~once per
            // recorder frame (30 fps) instead of every Unity tick over a
            // half-accumulated volume.
            int batch = _integrator != null ? _integrator.CompletedBatchCount : 0;
            if (batch != _lastDispatchedBatch && (_frameCounter % mcEveryNFrames == 0))
            {
                DispatchMarchCubes();
                _lastDispatchedBatch = batch;
            }

            meshMaterial.SetBuffer("_Triangles", _meshTrianglesBuffer);
            Graphics.DrawProceduralIndirect(meshMaterial, BuildBounds(1.0f),
                MeshTopology.Triangles, _meshArgsBuffer, 0,
                camera: null, properties: null,
                castShadows: ShadowCastingMode.Off, receiveShadows: false,
                layer: gameObject.layer);
        }

        private void DispatchMarchCubes()
        {
            _meshTrianglesBuffer.SetCounterValue(0);

            _mcShader.SetBuffer(_mcKernel, "_Voxels", volume.VoxelBuffer);
            _mcShader.SetInts("_Dim", volume.Dim.x, volume.Dim.y, volume.Dim.z);
            _mcShader.SetMatrix("_WorldFromVoxel", volume.WorldFromVoxel);
            _mcShader.SetFloat("_IsoLevel", meshIsoLevel);
            _mcShader.SetFloat("_MinWeight", meshMinWeight);
            _mcShader.SetBuffer(_mcKernel, "_Triangles", _meshTrianglesBuffer);

            int gx = Mathf.CeilToInt((volume.Dim.x - 1) / 4f);
            int gy = Mathf.CeilToInt((volume.Dim.y - 1) / 4f);
            int gz = Mathf.CeilToInt((volume.Dim.z - 1) / 4f);
            _mcShader.Dispatch(_mcKernel, Mathf.Max(1, gx), Mathf.Max(1, gy), Mathf.Max(1, gz));

            // Triangle count -> args[0]. Then the ScaleArgs kernel multiplies
            // args[0] by 3 so DrawProceduralIndirect sees a vertex count.
            ComputeBuffer.CopyCount(_meshTrianglesBuffer, _meshArgsBuffer, 0);
            _mcShader.SetBuffer(_scaleArgsKernel, "_Args", _meshArgsBuffer);
            _mcShader.Dispatch(_scaleArgsKernel, 1, 1, 1);

            _diagMcDispatchesThisWindow++;
        }

        // ---------- shared ----------
        private Bounds BuildBounds(float scaleMargin)
        {
            // Conservative AABB of the rotated voxel grid. Used only by the
            // Graphics layer for frustum culling — over-sizing is harmless.
            Vector3 min = Vector3.positiveInfinity, max = Vector3.negativeInfinity;
            for (int b = 0; b < 8; b++)
            {
                float i = (b & 1) != 0 ? volume.Dim.x : 0f;
                float j = (b & 2) != 0 ? volume.Dim.y : 0f;
                float k = (b & 4) != 0 ? volume.Dim.z : 0f;
                Vector3 w = volume.WorldFromVoxel.MultiplyPoint(new Vector3(i, j, k));
                min = Vector3.Min(min, w);
                max = Vector3.Max(max, w);
            }
            Vector3 size = (max - min) + Vector3.one * (volume.voxelSize * scaleMargin);
            return new Bounds((min + max) * 0.5f, size);
        }

        private void PerSecondDiag()
        {
            float now = Time.realtimeSinceStartup;
            if (_diagWindowStart == 0f) _diagWindowStart = now;
            if (now - _diagWindowStart < 1f) return;

            if (mode == ViewMode.Mesh)
            {
                var args = new uint[4];
                _meshArgsBuffer.GetData(args);
                int verts = (int)args[0];
                Debug.Log($"[TSDFView Mesh] mcDispatches={_diagMcDispatchesThisWindow}/s vertices={verts} tris={verts / 3}", this);
            }
            else
            {
                Debug.Log($"[TSDFView] mode={mode}", this);
            }
            _diagMcDispatchesThisWindow = 0;
            _diagWindowStart = now;
        }
    }
}
