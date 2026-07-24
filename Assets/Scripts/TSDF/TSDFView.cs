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

using Shared;
using Unity.Profiling;
using UnityEngine;
using UnityEngine.Rendering;

namespace TSDF
{
    [AddComponentMenu("TSDF/View")]
    [DefaultExecutionOrder(10)]
    public sealed class TSDFView : MonoBehaviour, Shared.IViewToggle, Shared.IPanelTunable,
                                   Shared.ITriangleSeedSource, Shared.ISdfShadowVolume
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

        [Tooltip("Hide / show the TSDF visualisation (whichever mode is active) without " +
                 "stopping integration or releasing GPU buffers — Update simply skips the " +
                 "draw call while off. Re-enabling resumes instantly (the mesh keeps being " +
                 "extracted off the latest published front buffer). Mirrors " +
                 "PointCloudView.showPointClouds; toggle at runtime (TSDFDebugSession's M key).")]
        public bool showMesh = true;

        [Tooltip("Hide the mesh WITHOUT gating TrianglesReady (runtime only). showMesh " +
                 "cannot be used to merely hide it: TrianglesReady requires showMesh, and " +
                 "PointCloudMotionCurves.CollectSeeds seeds the ribbons off this view's " +
                 "triangle buffer through that flag — dropping showMesh would silently " +
                 "change which curves get built, and therefore what the capture exports. " +
                 "This suppresses only the draw call; Marching Cubes still runs. " +
                 "Not serialized: a stray true in a scene would blank the sculpture.")]
        [System.NonSerialized] public bool suppressDraw;

        // ---- Shared.IViewToggle (unified Views panel) ----
        public string ViewLabel => "TSDF mesh";
        public bool Visible { get => showMesh; set => showMesh = value; }

        // ---- Shared.ITriangleSeedSource (motion curves seed from the displayed surface) ----
        // The extracted MC triangle soup, world space. Ready only while the mesh is actually
        // being drawn: Update early-outs on !showMesh / other modes, so the buffer would go
        // stale the moment the view is hidden — consumers must fall back to the clouds then.
        public ComputeBuffer TriangleBuffer => _meshTrianglesBuffer;
        public ComputeBuffer TriangleArgsBuffer => _meshArgsBuffer;
        public int MaxTriangles => meshMaxTriangles;
        public bool TrianglesReady =>
            isActiveAndEnabled && showMesh && mode == ViewMode.Mesh
            && _meshTrianglesBuffer != null && _meshArgsBuffer != null && _lastPublishVersion >= 0;

        // ---- Shared.ISdfShadowVolume (motion curves receive the body's self-shadow) ----
        // Exposes the same front SDF buffer + march parameters the mesh shades from, so
        // the curves cast the body's shadow onto themselves and read as one lit scene.
        // Ready whenever the SDF volume is live and current (showMesh keeps the front
        // buffer being produced — same gate as TrianglesReady). Deliberately NOT gated on
        // meshShadowStrength or suppressDraw: the experience draws only the CURVES as the
        // final model (the mesh is suppressed / white point cloud), so the curves must still
        // receive the body shadow even when the solid mesh — and its own shadow — is off.
        // The consumer's own strength knob decides whether it applies.
        public bool SdfShadowReady =>
            isActiveAndEnabled && showMesh && mode == ViewMode.Mesh
            && volume != null && volume.FrontBuffer != null;
        public ComputeBuffer SdfBuffer => volume != null ? volume.FrontBuffer : null;
        public Matrix4x4 SdfVoxelFromWorld => volume != null ? volume.WorldFromVoxel.inverse : Matrix4x4.identity;
        public Vector3Int SdfDim => volume != null ? volume.Dim : Vector3Int.zero;
        public float SdfMinWeight => meshMinWeight;
        public float SdfVoxelSize => volume != null ? volume.voxelSize : 0f;
        public float SdfTau => volume != null ? volume.Tau : 0f;
        public Vector3 ShadowLightDir => meshLightDir;
        public float ShadowSteps => meshShadowSteps;
        public float ShadowBiasVox => meshShadowBiasVox;
        public float ShadowIsoFrac => meshShadowIsoFrac;

        // ---- Shared.IPanelTunable (one-stop Control Panel) ----
        // Mesh look knobs so the surface can be graded at runtime, not just the Inspector.
        public string TuningLabel => "TSDF mesh";
        public int TunableCount => 8;
        public string TunableName(int i) =>
            i == 0 ? "Brightness" :
            i == 1 ? "Saturation" :
            i == 2 ? "Gamma" :
            i == 3 ? "Gradient normals" :
            i == 4 ? "Rim Boost" :
            i == 5 ? "Shadow Strength" :
            i == 6 ? "Shadow Reach" : "Shadow Bias";
        public float TunableValue(int i) =>
            i == 0 ? meshBrightness :
            i == 1 ? meshSaturation :
            i == 2 ? meshGamma :
            i == 3 ? meshGradNormals :
            i == 4 ? meshRimBoost :
            i == 5 ? meshShadowStrength :
            i == 6 ? meshShadowSteps : meshShadowBiasVox;
        public void SetTunableValue(int i, float value)
        {
            switch (i)
            {
                case 0: meshBrightness = Mathf.Clamp(value, 0f, 3f); break;
                case 1: meshSaturation = Mathf.Clamp(value, 0f, 3f); break;
                case 2: meshGamma = Mathf.Clamp(value, 0.2f, 3f); break;
                case 3: meshGradNormals = Mathf.Clamp01(value); break;
                case 4: meshRimBoost = Mathf.Clamp(value, 0f, 2f); break;
                case 5: meshShadowStrength = Mathf.Clamp01(value); break;
                case 6: meshShadowSteps = Mathf.Clamp(Mathf.RoundToInt(value), 1, 64); break;
                default: meshShadowBiasVox = Mathf.Clamp(value, 0f, 12f); break;
            }
        }
        public float TunableMin(int i) => i == 2 ? 0.2f : i == 6 ? 1f : 0f;
        public float TunableMax(int i) =>
            i == 0 ? 3f :
            i == 1 ? 3f :
            i == 2 ? 3f :
            i == 3 ? 1f :
            i == 4 ? 2f :
            i == 5 ? 1f :
            i == 6 ? 64f : 12f;
        public bool TunableIsInt(int i) => i == 6;

        // ---------- Voxel mode ----------
        [Header("Voxel view")]
        [Tooltip("Material that uses the TSDF/Voxel shader. Auto-created if null.")]
        public Material voxelMaterial;
        [Range(0.05f, 1f)]
        [Tooltip("Sphere diameter as a fraction of voxelSize.")]
        public float voxelSphereScale = 0.15f;
        [Range(0f, 1f)]
        [Tooltip("Hide voxels whose sdf exceeds this FRACTION of tau (the truncation " +
                 "distance). Free space in front of a surface is clamped to +tau, so this " +
                 "culls the white free-space fill. It scales WITH tau (= tauMultiplier × " +
                 "voxelSize), so the cull stays consistent as you change voxelSize — an " +
                 "absolute-metres threshold instead stops working once tau shrinks below " +
                 "it, which fills the whole volume with white spheres at fine resolutions. " +
                 "1 = hide only the saturated +tau free space (show the rest); lower = " +
                 "tighten onto the surface; 0 = hide everything from the surface outward.")]
        public float voxelHideAboveSdfFraction = 1f;

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

        [Header("Mesh colour grading")]
        [Range(0f, 3f)]
        [Tooltip("Saturation of the baked camera colour. 1 = as captured; >1 boosts " +
                 "(the camera RGB tends to read desaturated under the flat lighting).")]
        public float meshSaturation = 1.4f;
        [Range(0f, 3f)]
        [Tooltip("Overall brightness multiplier on the lit mesh colour.")]
        public float meshBrightness = 1f;
        [Range(0.2f, 3f)]
        [Tooltip("Gamma applied to the baked colour. <1 brightens / lifts shadows " +
                 "(use ~0.45 if colours look dark from sRGB-as-linear sampling).")]
        public float meshGamma = 1f;
        [Range(0f, 1f)]
        [Tooltip("Smooth (SDF-gradient) normals for the mesh. 0 = flat ddx/ddy " +
                 "facets, 1 = smooth. Samples the displayed front SDF buffer per " +
                 "fragment; falls back to flat where a neighbour voxel is unobserved.")]
        public float meshGradNormals = 1f;
        [Range(0f, 2f)]
        [Tooltip("Rim boost. Brightens the silhouette in the surface's own colour " +
                 "(no white wash) so shading stays saturated instead of greying out.")]
        public float meshRimBoost = 0.5f;

        [Header("Mesh self-shadow")]
        [Tooltip("Direction the light comes FROM, used for both the diffuse shading " +
                 "and the self-shadow march (so the cast direction always matches the " +
                 "shading). Need not be normalised. Default matches the shader's old " +
                 "hardcoded light (up-front-right).")]
        public Vector3 meshLightDir = new Vector3(0.35f, 0.85f, 0.40f);
        [Range(0f, 1f)]
        [Tooltip("Self-shadow strength. 0 = off (original flat-lit look); 1 = fully " +
                 "darken occluded fragments toward the ambient floor. Parts of the body " +
                 "hidden from the light by another part (arm over torso) go darker.")]
        public float meshShadowStrength = 0.7f;
        [Range(1, 64)]
        [Tooltip("Self-shadow reach in voxel steps: how far along the light ray to look " +
                 "for an occluding body part. Reach in metres = this × voxelSize. Higher " +
                 "catches more distant occluders but costs more per fragment.")]
        public int meshShadowSteps = 24;
        [Range(0f, 12f)]
        [Tooltip("Self-shadow start bias in voxels: how far along the light ray to skip " +
                 "before testing, so a surface doesn't shadow its OWN shell (the TSDF is a " +
                 "±tau band, not a hard surface). Raise if you see shadow acne on lit faces; " +
                 "lower to let near-touching parts (arm resting on torso) shadow sooner.")]
        public float meshShadowBiasVox = 3f;
        [Range(0f, 1f)]
        [Tooltip("Occluder threshold as a fraction of tau: the SDF value at which the march " +
                 "counts a sampled voxel as 'inside another surface'. ~0.3 catches the shell " +
                 "as the ray enters it; 0 requires reaching the exact iso surface (can miss " +
                 "thin shells between steps).")]
        public float meshShadowIsoFrac = 0.3f;
        [Tooltip("Max triangles the MC output buffer can hold. 333k tris = 1 M " +
                 "vertices, ~12 MB at 36 bytes per triangle (= 3 × float3).")]
        [Min(1024)] public int meshMaxTriangles = 333_333;
        [Tooltip("Iso-level for MC. 0 = surface where SDF changes sign. NOTE: the " +
                 "active-block path (useFullGridMC = false) assumes iso ~ 0 — a large " +
                 "positive iso can drop blocks (active-block marking is one-sided around " +
                 "0); use full-grid MC if you need a non-trivial nonzero iso.")]
        public float meshIsoLevel = 0f;
        [Tooltip("Minimum corner weight for MC to consider a cell. Matches the " +
                 "Cell view's weight gate so the two views show the same set of cells.")]
        [Min(0f)] public float meshMinWeight = 0.5f;
        [Tooltip("Run MC every Nth frame. 1 = every frame. Draws keep using the " +
                 "last extracted vertex buffer on skip frames.")]
        [Min(1)] public int mcEveryNFrames = 1;

        [Header("White point cloud")]
        [Tooltip("Draw the mesh as an edge net (interior discarded) instead of the " +
                 "shaded surface — the white-point-cloud look. Not serialized: a stray " +
                 "true in a scene would leave the sculpture as the point cloud. The " +
                 "experience flips this on for the ResultShow / practice replay and back " +
                 "off for the frozen model.")]
        [System.NonSerialized] public bool whitePointCloud;
        [Tooltip("White point cloud line colour.")]
        [UnityEngine.Serialization.FormerlySerializedAs("wireColor")]
        public Color whitePointCloudColor = Color.white;
        [Range(0.2f, 4f)]
        [Tooltip("White point cloud line width in pixels (screen-space, distance-independent).")]
        [UnityEngine.Serialization.FormerlySerializedAs("wireThickness")]
        public float whitePointCloudThickness = 1.2f;
        [Range(1, 32)]
        [Tooltip("Thin the net by drawing only every Nth marching-cubes triangle. The " +
                 "MC mesh is far denser than a pixel, so a 1:1 net reads as a solid " +
                 "silhouette; raise this until the edges read as a point cloud.")]
        [UnityEngine.Serialization.FormerlySerializedAs("wireStride")]
        public int whitePointCloudStride = 8;

        [Header("Perf / A-B (Phase 0)")]
        [Tooltip("Reference path: force the OLD full-grid Marching Cubes dispatch over " +
                 "every (dim-1)^3 cell. ON = full grid (always correct, the pre-0-1 " +
                 "behaviour); OFF = active-block MC (surface-proportional, added in task " +
                 "0-1). Flip between the two on the SAME published volume to check the " +
                 "active-block mesh matches the full-grid mesh (triangle count + visual " +
                 "parity). Until 0-1 lands this only ever runs the full-grid path.")]
        public bool useFullGridMC = true;

        [Header("Diagnostics")]
        [Tooltip("Log a per-second summary (voxelSize, dim, tris, VRAM MB, frame ms/fps, " +
                 "and — once 0-1 lands — active/total blocks). Drives the Phase 0 baseline " +
                 "table and exit gate. Also wraps MC passes in ProfilerMarkers so the Unity " +
                 "Profiler shows reliable per-pass GPU/CPU ms on any backend (incl. Metal).")]
        public bool diagnosticLogging = false;

        // ---- shared state ----
        private Mesh _sphereMesh;
        private Mesh _cubeMesh;
        private ComputeShader _mcShader;
        private int _mcKernel;
        private int _mcActiveKernel;
        private int _compactKernel;
        private int _buildArgsKernel;
        private int _scaleArgsKernel;
        // Last volume PublishVersion for which we dispatched MC. The mesh on
        // screen therefore corresponds to the last PUBLISHED front buffer — the
        // same complete-batch snapshot the Voxel/Cell views read this frame. All
        // three modes update in lockstep off the same publish, so they can be
        // compared apples-to-apples while debugging.
        private int _lastPublishVersion = -1;

        private ComputeBuffer _voxelArgsBuffer;
        private ComputeBuffer _cellArgsBuffer;
        private ComputeBuffer _meshTrianglesBuffer;  // AppendStructuredBuffer<Tri> (= 3 float3)
        private ComputeBuffer _meshArgsBuffer;       // IndirectArguments [vertCount, 1, 0, 0]
        private ComputeBuffer _activeBlocksBuffer;   // Append<uint>: compacted active block indices
        private ComputeBuffer _dispatchArgsBuffer;   // IndirectArguments [blockCount, 1, 1]
        private int _totalBlocksAtLastDispatch = -1;            // for per-second diag (active blocks)
        private int _frameCounter;

        private bool _ownsVoxelMat;
        private bool _ownsCellMat;
        private bool _ownsMeshMat;

        private float _diagWindowStart;
        private int _diagMcDispatchesThisWindow;

        // Per-pass CPU markers. Wrapping the Dispatch calls makes each TSDF pass show
        // up as a named sample in the Unity Profiler, which is the reliable way to read
        // per-pass GPU/CPU ms on Metal (CPU stopwatches around an async Dispatch only
        // measure enqueue cost). Named so they group under "TSDF." in the profiler.
        private static readonly ProfilerMarker s_markMarch = new ProfilerMarker("TSDF.MC.March");
        private static readonly ProfilerMarker s_markCompact = new ProfilerMarker("TSDF.MC.Compact");

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
                var sh = LoadShader("TSDFVoxel", "TSDF/Voxel");
                if (sh == null) { enabled = false; return; }
                voxelMaterial = new Material(sh) { name = "TSDF Voxel (auto)", hideFlags = HideFlags.DontSave };
                _ownsVoxelMat = true;
            }
            if (cellMaterial == null)
            {
                var sh = LoadShader("TSDFCell", "TSDF/Cell");
                if (sh == null) { enabled = false; return; }
                cellMaterial = new Material(sh) { name = "TSDF Cell (auto)", hideFlags = HideFlags.DontSave };
                _ownsCellMat = true;
            }
            if (meshMaterial == null)
            {
                var sh = LoadShader("TSDFMesh", "TSDF/TSDFMesh");
                if (sh == null) { enabled = false; return; }
                meshMaterial = new Material(sh) { name = "TSDF Mesh (auto)", hideFlags = HideFlags.DontSave };
                _ownsMeshMat = true;
            }
        }

        // Resources first so the shaders survive player-build stripping — nothing else
        // references them (the scene materials are auto-created), so a bare Shader.Find
        // returns null in a build and the whole view silently vanished. Same pattern as
        // PointCloudMotionCurves' MotionCurves shader.
        private static Shader LoadShader(string resourceName, string shaderName)
        {
            var sh = Resources.Load<Shader>(resourceName);
            if (sh == null) sh = Shader.Find(shaderName);
            if (sh == null) Debug.LogError($"[TSDFView] Shader \"{shaderName}\" not found (Resources/{resourceName}).");
            return sh;
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
            GpuBuf.Ensure(ref _voxelArgsBuffer, 5, sizeof(uint), ComputeBufferType.IndirectArguments);
            GpuBuf.Ensure(ref _cellArgsBuffer, 5, sizeof(uint), ComputeBufferType.IndirectArguments);

            // Mesh-view AppendBuffer of Tri = per vertex (float3 pos + float3
            // colour) × 3 = 6 float3 = 72 bytes per slot. Must match the Tri
            // struct in TSDFMarchingCubes.compute and TSDFMesh.shader.
            const int triStride = 72;
            GpuBuf.Ensure(ref _meshTrianglesBuffer, meshMaxTriangles, triStride, ComputeBufferType.Append);
            // 4-uint indirect args buffer for DrawProceduralIndirect: [vertCount, 1, 0, 0]
            // Seed only on (re)alloc — ScaleArgs owns [0] afterwards, so an unconditional
            // SetData here would zero the draw between publishes.
            if (_meshArgsBuffer == null)
            {
                GpuBuf.Ensure(ref _meshArgsBuffer, 4, sizeof(uint), ComputeBufferType.IndirectArguments);
                _meshArgsBuffer.SetData(new uint[] { 0, 1, 0, 0 });
            }

            // Active-block MC buffers (task 0-1). _activeBlocksBuffer holds the
            // compacted list of active block indices (capacity = worst-case all blocks);
            // reallocated when the block grid changes (voxelSize / bbox edit).
            // _dispatchArgsBuffer is the 3-uint DispatchIndirect args (distinct from the
            // 4-uint DRAW args above).
            int blockCount = volume != null ? Mathf.Max(1, volume.BlockDim.x * volume.BlockDim.y * volume.BlockDim.z) : 1;
            GpuBuf.Ensure(ref _activeBlocksBuffer, blockCount, sizeof(uint), ComputeBufferType.Append);
            if (_dispatchArgsBuffer == null)
            {
                GpuBuf.Ensure(ref _dispatchArgsBuffer, 3, sizeof(uint), ComputeBufferType.IndirectArguments);
                _dispatchArgsBuffer.SetData(new uint[] { 0, 1, 1 });
            }
        }

        private void ReleaseBuffers()
        {
            GpuBuf.Release(ref _voxelArgsBuffer);
            GpuBuf.Release(ref _cellArgsBuffer);
            GpuBuf.Release(ref _meshTrianglesBuffer);
            GpuBuf.Release(ref _meshArgsBuffer);
            GpuBuf.Release(ref _activeBlocksBuffer);
            GpuBuf.Release(ref _dispatchArgsBuffer);
        }

        private void EnsureMcShader()
        {
            if (_mcShader != null) return;
            if (!TSDFComputeUtil.TryLoad(ref _mcShader, "TSDFMarchingCubes", "TSDFView", this))
            {
                enabled = false;
                return;
            }
            _mcKernel = _mcShader.FindKernel("MarchCubes");
            _mcActiveKernel = _mcShader.FindKernel("MarchCubesActive");
            _compactKernel = _mcShader.FindKernel("CompactBlocks");
            _buildArgsKernel = _mcShader.FindKernel("BuildDispatchArgs");
            _scaleArgsKernel = _mcShader.FindKernel("ScaleArgs");
        }

        // ---------------------------------------------------------------
        private void Update()
        {
            if (volume == null || volume.FrontBuffer == null) return;
            if (!showMesh) return;
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

            voxelMaterial.SetBuffer("_Voxels", volume.FrontBuffer);
            voxelMaterial.SetMatrix("_WorldFromVoxel", volume.WorldFromVoxel);
            voxelMaterial.SetVector("_Dim", new Vector4(volume.Dim.x, volume.Dim.y, volume.Dim.z, 0));
            voxelMaterial.SetFloat("_VoxelSize", volume.voxelSize);
            voxelMaterial.SetFloat("_Tau", volume.Tau);
            voxelMaterial.SetFloat("_Scale", voxelSphereScale);
            // Scale the hide threshold by tau so the free-space cull tracks voxelSize.
            voxelMaterial.SetFloat("_HideAboveSdf", voxelHideAboveSdfFraction * volume.Tau);

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

            cellMaterial.SetBuffer("_Voxels", volume.FrontBuffer);
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

            // Re-extract MC only when the volume published a new front buffer.
            // Between publishes the previous triangles buffer keeps drawing — a
            // stable mesh that updates exactly when the Voxel/Cell views' front
            // buffer changes, never over a half-accumulated back buffer.
            int published = volume.PublishVersion;
            if (published != _lastPublishVersion && (_frameCounter % mcEveryNFrames == 0))
            {
                DispatchMarchCubes();
                _lastPublishVersion = published;
            }

            // Draw-only suppression, deliberately placed AFTER the extraction: the
            // ribbons seed from _meshTrianglesBuffer via TrianglesReady, so the
            // triangles must keep being produced even while the sculpture is hidden
            // (the experience hides it through Processing, where the capture still
            // reads those seeds). Returning before DispatchMarchCubes — or using
            // showMesh — would change what the capture exports.
            if (suppressDraw) return;

            meshMaterial.SetBuffer("_Triangles", _meshTrianglesBuffer);
            meshMaterial.SetFloat("_Saturation", meshSaturation);
            meshMaterial.SetFloat("_Brightness", meshBrightness);
            meshMaterial.SetFloat("_Gamma", meshGamma);
            // Gradient-normal inputs: sample the same displayed front SDF buffer the
            // mesh was extracted from. Cheap to keep bound even when the toggle is 0.
            meshMaterial.SetFloat("_GradNormals", meshGradNormals);
            meshMaterial.SetFloat("_RimBoost", meshRimBoost);
            meshMaterial.SetBuffer("_Voxels", volume.FrontBuffer);
            meshMaterial.SetMatrix("_WorldFromVoxel", volume.WorldFromVoxel);
            meshMaterial.SetMatrix("_VoxelFromWorld", volume.WorldFromVoxel.inverse);
            meshMaterial.SetVector("_VDim", new Vector4(volume.Dim.x, volume.Dim.y, volume.Dim.z, 0f));
            meshMaterial.SetFloat("_MinWeight", meshMinWeight);
            // Lighting + self-shadow: same L drives both, and the march steps in voxels
            // (metres reach = steps × voxelSize) sampling the front SDF bound above.
            meshMaterial.SetVector("_LightDir", meshLightDir == Vector3.zero ? Vector3.up : meshLightDir);
            meshMaterial.SetFloat("_VoxelSize", volume.voxelSize);
            meshMaterial.SetFloat("_Tau", volume.Tau);
            meshMaterial.SetFloat("_ShadowStrength", meshShadowStrength);
            meshMaterial.SetFloat("_ShadowSteps", meshShadowSteps);
            meshMaterial.SetFloat("_ShadowBiasVox", meshShadowBiasVox);
            meshMaterial.SetFloat("_ShadowIsoFrac", meshShadowIsoFrac);
            // White-point-cloud toggle — always set so it never leaks a stale 1 from a
            // previous replay into the shaded model reveal.
            meshMaterial.SetFloat("_WhitePointCloud", whitePointCloud ? 1f : 0f);
            meshMaterial.SetColor("_WhitePointCloudColor", whitePointCloudColor);
            meshMaterial.SetFloat("_WhitePointCloudThickness", whitePointCloudThickness);
            meshMaterial.SetFloat("_WhitePointCloudStride", Mathf.Max(1, whitePointCloudStride));
            Graphics.DrawProceduralIndirect(meshMaterial, BuildBounds(1.0f),
                MeshTopology.Triangles, _meshArgsBuffer, 0,
                camera: null, properties: null,
                castShadows: ShadowCastingMode.Off, receiveShadows: false,
                layer: gameObject.layer);
        }

        private void DispatchMarchCubes()
        {
            _meshTrianglesBuffer.SetCounterValue(0);

            // Iso / weight gates + geometry transform are global shader uniforms shared
            // by both the full-grid and active-block MC kernels.
            // _CellZBase persists on the shared ComputeShader asset — the print exporter
            // dispatches Z-slabs with non-zero bases, so normal rendering must reset it.
            _mcShader.SetInt("_CellZBase", 0);
            _mcShader.SetInts("_Dim", volume.Dim.x, volume.Dim.y, volume.Dim.z);
            _mcShader.SetMatrix("_WorldFromVoxel", volume.WorldFromVoxel);
            _mcShader.SetFloat("_IsoLevel", meshIsoLevel);
            _mcShader.SetFloat("_MinWeight", meshMinWeight);

            if (useFullGridMC)
                DispatchFullGridMC();
            else
                DispatchActiveBlockMC();

            // Triangle count -> args[0]. Then the ScaleArgs kernel multiplies
            // args[0] by 3 so DrawProceduralIndirect sees a vertex count.
            ComputeBuffer.CopyCount(_meshTrianglesBuffer, _meshArgsBuffer, 0);
            _mcShader.SetBuffer(_scaleArgsKernel, "_Args", _meshArgsBuffer);
            _mcShader.SetInt("_MaxTriangles", meshMaxTriangles);   // clamp overflow -> no out-of-bounds draw
            _mcShader.Dispatch(_scaleArgsKernel, 1, 1, 1);

            _diagMcDispatchesThisWindow++;
        }

        // Reference path: one thread per cell over the whole (dim-1)^3 grid.
        private void DispatchFullGridMC()
        {
            _mcShader.SetBuffer(_mcKernel, "_Voxels", volume.FrontBuffer);
            _mcShader.SetBuffer(_mcKernel, "_Colors", volume.FrontColorBuffer);
            _mcShader.SetBuffer(_mcKernel, "_Triangles", _meshTrianglesBuffer);

            int gx = Mathf.CeilToInt((volume.Dim.x - 1) / 4f);
            int gy = Mathf.CeilToInt((volume.Dim.y - 1) / 4f);
            int gz = Mathf.CeilToInt((volume.Dim.z - 1) / 4f);
            using (s_markMarch.Auto())
                _mcShader.Dispatch(_mcKernel, Mathf.Max(1, gx), Mathf.Max(1, gy), Mathf.Max(1, gz));
            _totalBlocksAtLastDispatch = -1;   // N/A on the full-grid path
        }

        // Active-block path (task 0-1): compact the front occupancy set into a block
        // list, then indirect-dispatch one thread-group per active block. Meshes the
        // SAME published front buffer as the full-grid path, so the two are comparable
        // via the useFullGridMC A/B toggle.
        private void DispatchActiveBlockMC()
        {
            // 1) Compact set-bit front blocks into the append list.
            _activeBlocksBuffer.SetCounterValue(0);
            _mcShader.SetInts("_BDim", volume.BlockDim.x, volume.BlockDim.y, volume.BlockDim.z);
            _mcShader.SetBuffer(_compactKernel, "_BlockActiveFront", volume.FrontBlockActive);
            _mcShader.SetBuffer(_compactKernel, "_ActiveBlocksAppend", _activeBlocksBuffer);
            int totalBlocks = volume.BlockDim.x * volume.BlockDim.y * volume.BlockDim.z;
            int cgx = Mathf.Max(1, Mathf.CeilToInt(totalBlocks / 64f));
            using (s_markCompact.Auto())
                _mcShader.Dispatch(_compactKernel, cgx, 1, 1);

            // 2) active-block count -> _dispatchArgs[0]; the kernel fills [1]=[2]=1.
            ComputeBuffer.CopyCount(_activeBlocksBuffer, _dispatchArgsBuffer, 0);
            _mcShader.SetBuffer(_buildArgsKernel, "_DispatchArgs", _dispatchArgsBuffer);
            _mcShader.Dispatch(_buildArgsKernel, 1, 1, 1);

            // 3) one group per active block over the front volume. Zero active blocks
            //    => [0,1,1] => a no-op dispatch (the tri counter stays 0 => nothing drawn).
            _mcShader.SetBuffer(_mcActiveKernel, "_Voxels", volume.FrontBuffer);
            _mcShader.SetBuffer(_mcActiveKernel, "_Colors", volume.FrontColorBuffer);
            _mcShader.SetBuffer(_mcActiveKernel, "_ActiveBlocks", _activeBlocksBuffer);
            _mcShader.SetBuffer(_mcActiveKernel, "_Triangles", _meshTrianglesBuffer);
            using (s_markMarch.Auto())
                _mcShader.DispatchIndirect(_mcActiveKernel, _dispatchArgsBuffer);

            _totalBlocksAtLastDispatch = totalBlocks;
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

            // VRAM = volume backing buffers + this view's own GPU buffers, from real
            // count×stride (Phase 0 task 0-0: measured, not hand-estimated).
            long viewBytes = 0;
            if (_meshTrianglesBuffer != null) viewBytes += (long)_meshTrianglesBuffer.count * _meshTrianglesBuffer.stride;
            long totalBytes = volume.GpuBufferBytes + viewBytes;
            float vramMb = totalBytes / (1024f * 1024f);
            float frameMs = Time.smoothDeltaTime * 1000f;
            float fps = frameMs > 0f ? 1000f / frameMs : 0f;
            Vector3Int d = volume.Dim;
            string path = useFullGridMC ? "full-grid" : "active-block";

            if (mode == ViewMode.Mesh)
            {
                var args = new uint[4];
                _meshArgsBuffer.GetData(args);
                int verts = (int)args[0];
                string blocks = "";
                if (!useFullGridMC && _dispatchArgsBuffer != null && _totalBlocksAtLastDispatch > 0)
                {
                    var da = new uint[3];
                    _dispatchArgsBuffer.GetData(da);
                    blocks = $" activeBlocks={da[0]}/{_totalBlocksAtLastDispatch}";
                }
                Debug.Log($"[TSDFView Mesh] path={path} voxelSize={volume.voxelSize:F3}m " +
                          $"dim={d.x}x{d.y}x{d.z} mcDispatches={_diagMcDispatchesThisWindow}/s " +
                          $"tris={verts / 3}{blocks} vram={vramMb:F1}MB frame={frameMs:F1}ms ({fps:F0}fps)", this);
            }
            else
            {
                Debug.Log($"[TSDFView] mode={mode} voxelSize={volume.voxelSize:F3}m " +
                          $"dim={d.x}x{d.y}x{d.z} vram={vramMb:F1}MB frame={frameMs:F1}ms ({fps:F0}fps)", this);
            }
            _diagMcDispatchesThisWindow = 0;
            _diagWindowStart = now;
        }
    }
}
