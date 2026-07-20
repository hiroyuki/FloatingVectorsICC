// Authoritative TSDF voxel grid for the v2 motion-to-mesh pipeline. Holds a
// single RWStructuredBuffer<float2> (x = signed distance, y = weight) covering
// the working volume defined by a BoundingVolume. The grid is axis-
// aligned in the bounding box's LOCAL frame so rotating the bbox rotates the
// grid with it.
//
// Step 1 of FloatingVectorsICC_v2_TSDF_spec.md §9 — this component owns the
// buffer + Clear kernel. TSDFIntegrator (step 1 commit 2) and TSDFMarching
// Cubes (commit 3) read this volume; they never allocate their own.

using PointCloud;
using Shared;
using UnityEngine;

namespace TSDF
{
    /// <summary>
    /// Voxel volume backed by a ComputeBuffer of float2 (tsdf, weight). One
    /// instance per scene; the integrator + marching cubes stages bind to this
    /// component's <see cref="FrontBuffer"/>/<see cref="WriteBuffer"/> instead of
    /// allocating their own.
    /// </summary>
    [DefaultExecutionOrder(-10)]
    public sealed class TSDFVolume : MonoBehaviour
    {
        public enum AccumulationMode
        {
            /// <summary>Weighted average across cameras — denoises one static instant.
            /// Smears motion away if used while accumulating over time.</summary>
            [InspectorName("Average (merge cameras)")]
            StandardWeightedAvg = 0,
            /// <summary>Spec §3.2 (A) — keeps the closest-to-surface observation, so
            /// a hand passing through a voxel leaves a permanent shell.</summary>
            [InspectorName("Retain ghost (keep motion)")]
            RetainGhost = 1,
        }

        [Header("Working volume")]
        [Tooltip("Defines the world-space extent and orientation of the TSDF grid. " +
                 "Local scale (x,y,z) becomes the box edge lengths; rotation is " +
                 "honored so the voxel axes follow the bbox. Mandatory.")]
        public BoundingVolume boundingBox;

        [Header("Grid resolution")]
        [Tooltip("Edge length of one (cubic) voxel in metres — drag to tune the mesh " +
                 "granularity live (the grid rebuilds next frame). Finer = more detail " +
                 "but voxel count grows as 1/size^3 (VRAM ×2 for double buffering) and " +
                 "more triangles — if the mesh starts clipping, raise TSDFView." +
                 "meshMaxTriangles. The Femto Bolt depth footprint (~few mm at 1-2 m) " +
                 "is the real floor, so ~0.01-0.02 m is the practical sweet spot; below " +
                 "that you mostly add holes/noise. Slider 1 mm - 10 cm — but voxel count " +
                 "grows as 1/size^3, so very small sizes are only viable on a small " +
                 "bounding box (a guard caps total voxels and logs an error otherwise).")]
        [Range(0.001f, 0.1f)]
        public float voxelSize = 0.05f;

        [Tooltip("Truncation distance = tauMultiplier * voxelSize. Spec recommends 4x.")]
        [Min(1f)]
        public float tauMultiplier = 4f;

        [Header("Camera fusion (multi-cam → one instant)")]
        [Tooltip("How cameras observing the SAME instant are merged into each voxel.\n" +
                 "Average = denoise across cameras (live default).\n" +
                 "Retain ghost = keep the closest-to-surface observation; needed to " +
                 "capture frame-to-frame motion (MeshCumulative forces it).\n" +
                 "NOTE: whether observations accumulate over TIME is governed by the " +
                 "integrator's live-follow clear, not here — this only sets the merge rule.")]
        public AccumulationMode accumulationMode = AccumulationMode.StandardWeightedAvg;

        // Time-union neck radius for the accumulate (motion-sweep) fold, in metres.
        // 0 = hard RetainGhost |sdf|-min (separate pose-shells, no bridging). >0 makes
        // FoldInstanceIntoAccumulation blend consecutive instants with a separation-gated
        // smooth-min so the sweep reads as one connected ribbon. Driven by
        // TSDFDebugSession (smoothUnion / smoothUnionK) while accumulating; hidden because
        // it only matters during the accumulate pipeline.
        [HideInInspector] public float accumulateSmoothK = 0f;

        // Ping-pong the voxel buffer so the views (Voxel/Cell/Mesh) only ever read a
        // COMPLETE batch while the integrator fills the next one into a hidden back
        // buffer. Eliminates the mid-clear / partial-fill flicker. Hidden from the
        // Inspector because TSDFIntegrator drives it every frame to match its mode:
        // ON for live-follow (clearVolumeOnNewBatch), OFF for accumulate
        // (read-modify-write needs a single persistent buffer). Costs 2x VRAM when on.
        [HideInInspector] public bool doubleBuffered = true;

        [Header("Diagnostics")]
        [Tooltip("Log allocation + clear info on enable. Off by default.")]
        public bool verboseLogging = false;

        // GPU state — exposed so the integrator/MC stages can bind without
        // re-allocating. Two physical buffers (_bufA/_bufB) ping-pong: views read
        // FrontBuffer (last published, complete batch), the integrator + Clear write
        // WriteBuffer (the hidden back buffer). Publish() swaps them. When
        // doubleBuffered is off, FrontBuffer == WriteBuffer == _bufA (single buffer).
        public ComputeBuffer FrontBuffer { get; private set; }
        public ComputeBuffer WriteBuffer { get; private set; }

        /// <summary>Per-voxel colour (float4 rgb + colour-weight), parallel to the sdf
        /// buffer and swapped together. The integrator writes the camera RGB of the
        /// retained observation; Marching Cubes interpolates it per vertex. Never
        /// cleared — MC only reads colours of voxels whose weight passed the gate, and
        /// the integrator overwrites colour on a voxel's first observation.</summary>
        public ComputeBuffer FrontColorBuffer { get; private set; }
        public ComputeBuffer WriteColorBuffer { get; private set; }

        /// <summary>Bumped every Publish() (and on full Clear / rebuild). TSDFView reads
        /// this to re-extract Marching Cubes only when new content was published, so all
        /// three view modes update in lockstep with the same cadence.</summary>
        public int PublishVersion { get; private set; }

        public Vector3Int Dim { get; private set; }
        public float Tau => tauMultiplier * voxelSize;

        // ---- Active-block occupancy (Phase 0 task 0-1) ----
        /// <summary>Marking rim as a multiple of voxelSize: a written voxel with
        /// sdf below this × voxelSize (ONE-SIDED — no lower bound) marks its owning
        /// blocks active. The negative side is unbounded (all sdf&lt;0 marks) so no
        /// straddling cell is ever dropped; this is just a small positive rim past
        /// iso. See TSDFBlockActive.hlsl for the completeness argument.</summary>
        public const float MarkBandVoxels = 1.0f;

        /// <summary>Block-grid dims = ceil((Dim-1)/8) per axis (8³ cells per block).
        /// One <see cref="FrontBlockActive"/>/<see cref="WriteBlockActive"/> uint per
        /// block records whether that block currently emits surface.</summary>
        public Vector3Int BlockDim { get; private set; }

        /// <summary>Block-grid dims for the depth-basis touched/clear set = ceil(Dim/8)
        /// (VOXEL blocks, indexed by voxel&gt;&gt;3). Distinct from <see cref="BlockDim"/>
        /// (MC cell blocks = ceil((Dim-1)/8)) — they differ when an axis is ≡1 mod 8.</summary>
        public Vector3Int VoxelBlockDim { get; private set; }

        /// <summary>Per-block occupancy set (uint per block, 1 = active) for the
        /// DISPLAYED front buffer — active-block Marching Cubes consumes ONLY this.
        /// Swapped with <see cref="FrontBuffer"/> in <see cref="Publish"/> so it always
        /// matches the front SDF. Aliases <see cref="WriteBlockActive"/> in single-buffer mode.</summary>
        public ComputeBuffer FrontBlockActive { get; private set; }
        /// <summary>Occupancy set for the hidden write buffer — kernels that write
        /// observable voxels (integrate/fold/trail-bake) mark this. Zeroed by
        /// <see cref="ClearWrite"/>, swapped to front in <see cref="Publish"/>.</summary>
        public ComputeBuffer WriteBlockActive { get; private set; }
        /// <summary>Occupancy scratch paired with the accumulate INSTANCE buffer. The
        /// integrate kernel always writes _BlockActive, so accumulate-mode integration
        /// (which fills the instance buffer) marks here; nothing reads it — the real
        /// accumulation set is marked by TSDFFold into <see cref="WriteBlockActive"/>.</summary>
        public ComputeBuffer InstanceBlockActive => _baInst;

        /// <summary>Total GPU memory (bytes) currently held by this volume's compute
        /// buffers — sdf + colour physical backing pair, plus the accumulate instance
        /// scratch when allocated. Reported by TSDFView's per-second diagnostics so the
        /// VRAM figure is measured from real <c>count × stride</c>, not hand-estimated.</summary>
        public long GpuBufferBytes
        {
            get
            {
                long b = 0;
                if (_bufA != null) b += (long)_bufA.count * sizeof(float) * 2;
                if (_bufB != null) b += (long)_bufB.count * sizeof(float) * 2;
                if (_colA != null) b += (long)_colA.count * sizeof(float) * 4;
                if (_colB != null) b += (long)_colB.count * sizeof(float) * 4;
                if (_instSdf != null) b += (long)_instSdf.count * sizeof(float) * 2;
                if (_instColor != null) b += (long)_instColor.count * sizeof(float) * 4;
                if (_baA != null) b += (long)_baA.count * sizeof(uint);
                if (_baB != null) b += (long)_baB.count * sizeof(uint);
                if (_baInst != null) b += (long)_baInst.count * sizeof(uint);
                if (_keyA != null) b += (long)_keyA.count * sizeof(uint);
                if (_keyB != null) b += (long)_keyB.count * sizeof(uint);
                if (_touchA != null) b += (long)_touchA.count * sizeof(uint);
                if (_touchB != null) b += (long)_touchB.count * sizeof(uint);
                return b;
            }
        }

        /// <summary>
        /// Maps voxel index space (i+0.5, j+0.5, k+0.5) → world. Use this in the
        /// integrate / MC kernels to recover each voxel's world position without
        /// touching <see cref="boundingBox"/>'s Transform directly.
        /// </summary>
        public Matrix4x4 WorldFromVoxel { get; private set; }

        /// <summary>Inverse of <see cref="WorldFromVoxel"/>. Use to test a world point
        /// against the grid extents (e.g. clamping floor points to in-bounds voxels).</summary>
        public Matrix4x4 VoxelFromWorld { get; private set; }

        private ComputeShader _clearShader;
        private int _clearKernel;
        private int _clearUintKernel;
        private bool _dimsDirty = true;

        // Physical active-block backing buffers, mirroring _bufA/_bufB. _baB is null in
        // single-buffer mode (WriteBlockActive aliases FrontBlockActive == _baA).
        private ComputeBuffer _baA;
        private ComputeBuffer _baB;
        // Active-block scratch for the accumulate instance buffer (never MC-consumed).
        private ComputeBuffer _baInst;

        [Header("Noise removal (connected-component filter — issue #29)")]
        [Tooltip("On RemoveSmallComponents(), drop solid components smaller than this " +
                 "many voxels (floating islands), keeping the thin main surface intact. " +
                 "Run on a frozen result (B-mode pause), not per frame.")]
        [Min(1)] public int minComponentVoxels = 200;
        [Tooltip("Max label-propagation iterations for connected-component labelling " +
                 "(stops early once labels stabilise). Raise if large bodies get holes.")]
        [Min(1)] public int ccMaxIterations = 256;
        [Tooltip("A voxel counts as solid for the component filter when sdf < 0 and " +
                 "weight >= this. Keep at/below the Marching Cubes weight gate so the " +
                 "filter sees everything that gets meshed.")]
        [Min(0f)] public float ccMinWeight = 0.5f;

        private ComputeShader _ccShader;
        private int _ccInit, _ccProp, _ccResetSize, _ccCount, _ccCull;
        private ComputeBuffer _ccLabel, _ccSize, _ccChanged;

        // Physical backing buffers. _bufB is null in single-buffer mode.
        private ComputeBuffer _bufA;
        private ComputeBuffer _bufB;
        // Parallel colour backing buffers (float4 rgb+w), swapped with the sdf pair.
        private ComputeBuffer _colA;
        private ComputeBuffer _colB;
        private bool _lastDoubleBuffered;

        // Per-instant "instance" buffers for the accumulate pipeline. One instant's
        // cameras average into these (denoised), then FoldInstanceIntoAccumulation()
        // unions the clean instant into the persistent accumulation buffer over time.
        // Allocated lazily on first use so the extra VRAM is only paid in accumulate.
        private ComputeBuffer _instSdf;
        private ComputeBuffer _instColor;
        private ComputeShader _foldShader;
        private int _foldKernel;

        // Depth-basis integration scratch (option ④): per-voxel packed |sdf|-min key for
        // the 2-pass InterlockedMin scatter, DOUBLE buffered so it swaps with the SDF
        // buffer in Publish. That lets ONE touched-block set (also swapped) identify each
        // physical buffer's stale region for the Phase-1b active-block clear. Allocated
        // lazily on first depth batch; both init to 0xFFFFFFFF. _keyB is null pre-alloc.
        private ComputeBuffer _keyA;
        private ComputeBuffer _keyB;
        // Per-block "written since last clear" set (Phase 1b), one uint per block, paired
        // and swapped with the SDF/key buffers. ScatterMin marks it; ClearBlocks clears
        // the marked blocks' voxels + keys and resets the flags.
        private ComputeBuffer _touchA;
        private ComputeBuffer _touchB;
        private int _clearBlocksKernel = -1;

        // Front->Write copy (sdf + colour) for TSDFTrailBaker's fuse-into-displayed-body.
        private ComputeShader _copyShader;
        private int _copyKernel;

        // One-shot beautify (3x3x3 sdf median) for a held front buffer — see
        // Resources/TSDFBeautify.compute and BeautifyFront().
        private ComputeShader _beautifyShader;
        private int _medianKernel;

        /// <summary>Per-instant sdf+weight scratch (camera-averaged). Bind the integrate
        /// kernel here, not <see cref="WriteBuffer"/>, when separating camera fusion from
        /// time accumulation. Null until <see cref="ClearInstance"/> first allocates it.</summary>
        public ComputeBuffer InstanceBuffer => _instSdf;
        /// <summary>Per-instant colour scratch, parallel to <see cref="InstanceBuffer"/>.</summary>
        public ComputeBuffer InstanceColorBuffer => _instColor;

        /// <summary>Depth-basis 2-pass scatter key buffer (packed |sdf|-min per voxel) for
        /// the HIDDEN write buffer. Bind the ScatterMin/ScatterWrite kernels here. Null
        /// until the first depth batch allocates it. Swaps with the SDF buffer in Publish.</summary>
        public ComputeBuffer WriteKeyBuffer { get; private set; }
        private ComputeBuffer FrontKeyBuffer { get; set; }

        /// <summary>Per-block touched set for the write buffer (Phase 1b active-block clear).
        /// ScatterMin marks it; <see cref="ClearWriteActiveBlocks"/> consumes + resets it.
        /// Swaps with the write buffer in Publish. Null until the first depth batch.</summary>
        public ComputeBuffer WriteTouched { get; private set; }
        private ComputeBuffer FrontTouched { get; set; }

        // Track the bbox transform values we last built the grid from so we
        // can rebuild lazily when the user edits the bbox or voxel size.
        private Vector3 _lastBboxPos;
        private Quaternion _lastBboxRot;
        private Vector3 _lastBboxScale;
        private float _lastVoxelSize;

        private void OnEnable()
        {
            EnsureClearShader();
            RebuildIfNeeded(forceClear: true);
        }

        private void OnDisable()
        {
            ReleaseBuffer();
        }

        private void Update()
        {
            // Rebuild on bbox / voxelSize changes. Editing in Inspector during
            // playmode therefore takes effect on the next frame.
            if (HasGridChanged())
                RebuildIfNeeded(forceClear: true);
        }

        private bool HasGridChanged()
        {
            if (boundingBox == null) return false;
            var t = boundingBox.transform;
            return _dimsDirty
                || _lastBboxPos != t.position
                || _lastBboxRot != t.rotation
                || _lastBboxScale != t.localScale
                || _lastDoubleBuffered != doubleBuffered
                || !Mathf.Approximately(_lastVoxelSize, voxelSize);
        }

        /// <summary>
        /// Clear ONLY the hidden write (back) buffer to (tsdf=+tau, weight=0).
        /// Used by the integrator at the start of a new batch and by debug replay —
        /// the displayed FrontBuffer is left untouched, so nothing flickers while the
        /// next batch is built up. In single-buffer mode this is the displayed buffer.
        /// </summary>
        public void ClearWrite()
        {
            ClearBuffer(WriteBuffer);
            // The write set travels with the write SDF buffer: a live-follow batch
            // clears and fully rebuilds it, so its occupancy must reset too. In
            // single-buffer mode this aliases the front set — but ClearWrite is only
            // called by the live-follow path (double-buffered), never in accumulate,
            // so the persistent accumulate set is not wrongly wiped here.
            ClearBlockActive(WriteBlockActive);
        }

        /// <summary>
        /// (Separated pipeline) Reset the per-instant instance buffer to (tsdf=+tau,
        /// weight=0) so the next instant's cameras average in from scratch. Allocates
        /// the instance buffers on first call (sized to the current grid).
        /// </summary>
        public void ClearInstance()
        {
            EnsureInstanceBuffers();
            ClearBuffer(_instSdf);
            // Zero the instance's own occupancy scratch each instant (integrate marks
            // it while filling the instance buffer). It is never MC-consumed, but must
            // be a valid, bound buffer for the integrate dispatch.
            ClearBlockActive(_baInst);
            // Colour scratch never needs clearing: the integrate Average path takes
            // rgb fresh on a voxel's first observation (sdf weight 0), matching the
            // main colour-buffer contract.
        }

        /// <summary>
        /// (Separated pipeline) Union the just-built instance volume into the
        /// accumulation buffer (<see cref="WriteBuffer"/>) with the RetainGhost rule
        /// (|sdf| min wins) — keeps the motion trail while replacing noisier priors
        /// with the clean camera-averaged instant. No-op until ClearInstance has
        /// allocated the instance buffers.
        /// </summary>
        public void FoldInstanceIntoAccumulation()
        {
            if (_instSdf == null || WriteBuffer == null) return;
            if (!EnsureFoldShader()) return;
            int total = Dim.x * Dim.y * Dim.z;
            if (total <= 0) return;

            _foldShader.SetBuffer(_foldKernel, "_Acc", WriteBuffer);
            _foldShader.SetBuffer(_foldKernel, "_AccColor", WriteColorBuffer);
            _foldShader.SetBuffer(_foldKernel, "_Inst", _instSdf);
            _foldShader.SetBuffer(_foldKernel, "_InstColor", _instColor);
            _foldShader.SetInts("_Dim", Dim.x, Dim.y, Dim.z);
            // 0 = hard RetainGhost |sdf|-min; >0 = separation-gated smooth-min so the
            // motion sweep's pose-shells join with organic necks (see TSDFFold.compute).
            _foldShader.SetFloat("_Tau", Tau);
            _foldShader.SetFloat("_SmoothK", Mathf.Max(0f, accumulateSmoothK));
            // Fold marks the accumulation set (= WriteBlockActive; == front in single-
            // buffer accumulate). Never cleared per instant, so preserved surfaces stay.
            BindBlockMarking(_foldShader, _foldKernel, WriteBlockActive);
            TSDFComputeUtil.DispatchLinear(_foldShader, _foldKernel, total);
        }

        /// <summary>
        /// Copy the displayed FrontBuffer (sdf AND colour) into the hidden
        /// WriteBuffer so a subsequent write-buffer bake (e.g. TSDFTrailBaker's fuse)
        /// min-unions into EXACTLY what is on screen, not the stale scratch left in the
        /// write buffer after the last Publish() swap. No-op in single-buffer mode
        /// (Front == Write), where the write buffer already holds the displayed content.
        /// Copies colour too — geometry-only would leave the body's colour stale.
        /// </summary>
        public void CopyFrontToWrite()
        {
            if (FrontBuffer == null || WriteBuffer == null) return;
            if (ReferenceEquals(FrontBuffer, WriteBuffer)) return;   // single buffer: nothing to do
            if (!CopyBuffers(FrontBuffer, FrontColorBuffer, WriteBuffer, WriteColorBuffer)) return;

            // Copy the occupancy set too, so a subsequent trail-bake into the write
            // buffer min-unions onto the DISPLAYED body's active blocks (not the stale
            // set left after the last swap). Block count is tiny and this is a rare
            // manual fuse path, so a CPU roundtrip is acceptable.
            if (FrontBlockActive != null && WriteBlockActive != null &&
                !ReferenceEquals(FrontBlockActive, WriteBlockActive) &&
                FrontBlockActive.count == WriteBlockActive.count)
            {
                var tmp = new uint[FrontBlockActive.count];
                FrontBlockActive.GetData(tmp);
                WriteBlockActive.SetData(tmp);
            }
        }

        private bool EnsureCopyShader()
        {
            if (_copyShader != null) return true;
            if (!TSDFComputeUtil.TryLoad(ref _copyShader, "TSDFCopy", "TSDFVolume", this)) return false;
            _copyKernel = _copyShader.FindKernel("Copy");
            return true;
        }

        /// <summary>
        /// GPU copy of one sdf+colour buffer pair into another over the current grid
        /// (one thread per voxel, TSDFCopy kernel). Shared by
        /// <see cref="CopyFrontToWrite"/> and external snapshot users (e.g.
        /// TSDFDebugSession's instant capture), so the copy dispatch exists once.
        /// Returns false when the shader is missing or the grid is empty.
        /// </summary>
        public bool CopyBuffers(ComputeBuffer srcSdf, ComputeBuffer srcColor,
                                ComputeBuffer dstSdf, ComputeBuffer dstColor)
        {
            if (srcSdf == null || srcColor == null || dstSdf == null || dstColor == null) return false;
            if (!EnsureCopyShader()) return false;
            int total = Dim.x * Dim.y * Dim.z;
            if (total <= 0) return false;

            _copyShader.SetInts("_Dim", Dim.x, Dim.y, Dim.z);
            _copyShader.SetBuffer(_copyKernel, "_SrcSdf", srcSdf);
            _copyShader.SetBuffer(_copyKernel, "_SrcColor", srcColor);
            _copyShader.SetBuffer(_copyKernel, "_DstSdf", dstSdf);
            _copyShader.SetBuffer(_copyKernel, "_DstColor", dstColor);
            TSDFComputeUtil.DispatchLinear(_copyShader, _copyKernel, total);
            return true;
        }

        /// <summary>
        /// One-shot repair of the DISPLAYED (front) buffer for a held/paused mesh:
        /// an even number of 3x3x3 sdf-median passes. Kills the RetainGhost multi-cam
        /// ribbing / double-shell fray on thin limbs while preserving true surfaces
        /// (median is edge-preserving); bridged gap voxels gain a fill weight so they
        /// pass the MC gate. Re-publishes so Marching Cubes re-extracts. NOT a per-
        /// frame operation — call when playback pauses / the mesh freezes (the next
        /// live publish overwrites the repaired front, which is the desired behaviour).
        /// Scratch: the hidden write buffer when double-buffered (then fully re-cleared
        /// to keep the depth-basis active-block-clear invariant), else the instance pair.
        /// </summary>
        public void BeautifyFront(int iterations = 2)
        {
            if (FrontBuffer == null || FrontColorBuffer == null || FrontBlockActive == null) return;
            int total = Dim.x * Dim.y * Dim.z;
            if (total <= 0) return;
            if (_beautifyShader == null)
            {
                if (!TSDFComputeUtil.TryLoad(ref _beautifyShader, "TSDFBeautify", "TSDFVolume", this)) return;
                _medianKernel = _beautifyShader.FindKernel("Median");
            }

            // Scratch pair for the ping-pong. Double-buffered: reuse the hidden write
            // buffers (clobbered — restored below). Single-buffer (frozen accumulate):
            // reuse the instance scratch pair.
            ComputeBuffer scratchSdf, scratchCol;
            bool scratchIsWrite = doubleBuffered && !ReferenceEquals(WriteBuffer, FrontBuffer)
                                  && WriteBuffer != null && WriteColorBuffer != null;
            if (scratchIsWrite) { scratchSdf = WriteBuffer; scratchCol = WriteColorBuffer; }
            else
            {
                EnsureInstanceBuffers();
                scratchSdf = _instSdf; scratchCol = _instColor;
                if (scratchSdf == null || scratchCol == null) return;
            }

            // Round up to an even pass count so the result lands back in the FRONT pair.
            int passes = Mathf.Max(2, (iterations + 1) & ~1);
            ComputeBuffer src = FrontBuffer, srcC = FrontColorBuffer;
            ComputeBuffer dst = scratchSdf, dstC = scratchCol;
            for (int p = 0; p < passes; p++)
            {
                _beautifyShader.SetBuffer(_medianKernel, "_MedSrc", src);
                _beautifyShader.SetBuffer(_medianKernel, "_MedSrcCol", srcC);
                _beautifyShader.SetBuffer(_medianKernel, "_MedDst", dst);
                _beautifyShader.SetBuffer(_medianKernel, "_MedDstCol", dstC);
                _beautifyShader.SetInts("_MedDim", Dim.x, Dim.y, Dim.z);
                _beautifyShader.SetFloat("_MedTau", Tau);
                _beautifyShader.SetInt("_MedMinObs", 14);
                // Gate + write-marking both use the FRONT active set (bits only added).
                BindBlockMarking(_beautifyShader, _medianKernel, FrontBlockActive);
                TSDFComputeUtil.DispatchLinear(_beautifyShader, _medianKernel, total);
                var t = src; src = dst; dst = t;
                var tc = srcC; srcC = dstC; dstC = tc;
            }

            // The write-buffer scratch was clobbered WITHOUT marking the depth-basis
            // touched set, so a surface-proportional clear would miss the residue and
            // ghost — restore the invariant with a full write clear (one-shot, cheap here).
            if (scratchIsWrite)
            {
                if (WriteKeyBuffer != null) ClearWriteFull();
                else ClearWrite();
            }

            MarkFrontDirty();   // re-extract Marching Cubes on the repaired front
        }

        private void EnsureInstanceBuffers()
        {
            int total = Dim.x * Dim.y * Dim.z;
            if (total <= 0) return;
            GpuBuf.Ensure(ref _instSdf, total, sizeof(float) * 2);
            GpuBuf.Ensure(ref _instColor, total, sizeof(float) * 4);
            int blocks = Mathf.Max(1, BlockDim.x * BlockDim.y * BlockDim.z);
            GpuBuf.Ensure(ref _baInst, blocks, sizeof(uint));
        }

        private bool EnsureFoldShader()
        {
            if (_foldShader != null) return true;
            if (!TSDFComputeUtil.TryLoad(ref _foldShader, "TSDFFold", "TSDFVolume", this)) return false;
            _foldKernel = _foldShader.FindKernel("Fold");
            return true;
        }

        /// <summary>
        /// Full wipe: clears BOTH buffers and publishes, so the displayed volume
        /// empties immediately. Called on enable, on grid change, and exposed for the
        /// capture-start trigger from §7 of the spec.
        /// </summary>
        [ContextMenu("Clear Volume")]
        public void Clear()
        {
            ClearBuffer(_bufA);
            if (_bufB != null) ClearBuffer(_bufB);
            // Both SDF buffers cleared → both occupancy sets must be zeroed too, so
            // the front active set still matches the cleared front SDF (no swap here).
            ClearBlockActive(_baA);
            if (_baB != null) ClearBlockActive(_baB);
            PublishVersion++;

            if (verboseLogging)
            {
                int total = Dim.x * Dim.y * Dim.z;
                Debug.Log($"[TSDFVolume] cleared {total} voxels (dim={Dim}, voxelSize={voxelSize} m, tau={Tau:F4} m)", this);
            }
        }

        private void ClearBuffer(ComputeBuffer buf)
        {
            if (buf == null) return;
            if (_clearShader == null) EnsureClearShader();
            if (_clearShader == null) return;

            _clearShader.SetBuffer(_clearKernel, "_Voxels", buf);
            _clearShader.SetInts("_Dim", Dim.x, Dim.y, Dim.z);
            _clearShader.SetFloat("_InitTsdf", Tau);

            // 64-thread groups, laid out as a 2D grid so the per-axis 65535
            // threadgroup limit is never exceeded (a 1x1m@1cm volume already
            // needs >100k groups).
            int total = Dim.x * Dim.y * Dim.z;
            TSDFComputeUtil.DispatchLinear(_clearShader, _clearKernel, total);
        }

        /// <summary>
        /// Publish the write (back) buffer to the front so views pick it up. In
        /// double-buffer mode this swaps Front/Write (the just-filled back becomes the
        /// displayed front; the old front becomes the next batch's scratch). In single-
        /// buffer mode it only bumps the version (content is already visible). Always
        /// bumps PublishVersion so TSDFView re-extracts Marching Cubes on new content.
        /// </summary>
        public void Publish()
        {
            if (doubleBuffered && _bufB != null)
            {
                var tmp = FrontBuffer;
                FrontBuffer = WriteBuffer;
                WriteBuffer = tmp;
                var tmpC = FrontColorBuffer;
                FrontColorBuffer = WriteColorBuffer;
                WriteColorBuffer = tmpC;
                // Swap the occupancy sets IN LOCKSTEP so the front set always
                // describes the front SDF (a partial swap desyncs mesh from volume).
                var tmpB = FrontBlockActive;
                FrontBlockActive = WriteBlockActive;
                WriteBlockActive = tmpB;
                // Depth-basis key + touched sets (Phase 1b) travel with their SDF buffer, so
                // the touched set keeps identifying the physical buffer's own stale region.
                if (WriteKeyBuffer != null)
                {
                    var tmpK = FrontKeyBuffer; FrontKeyBuffer = WriteKeyBuffer; WriteKeyBuffer = tmpK;
                    var tmpT = FrontTouched; FrontTouched = WriteTouched; WriteTouched = tmpT;
                }
            }
            PublishVersion++;
        }

        /// <summary>Bump PublishVersion WITHOUT swapping buffers — for in-place edits
        /// of the FRONT buffer (print exporter's snapshot restore) so views re-extract.
        /// Publish() would swap in double-buffer mode and display the stale write side.</summary>
        public void MarkFrontDirty() => PublishVersion++;

        private void EnsureCcShader()
        {
            if (_ccShader != null) return;
            if (!TSDFComputeUtil.TryLoad(ref _ccShader, "TSDFConnectedComponents", "TSDFVolume", this)) return;
            _ccInit = _ccShader.FindKernel("InitLabels");
            _ccProp = _ccShader.FindKernel("Propagate");
            _ccResetSize = _ccShader.FindKernel("ResetSizes");
            _ccCount = _ccShader.FindKernel("CountSizes");
            _ccCull = _ccShader.FindKernel("Cull");
        }

        /// <summary>
        /// Connected-component noise filter on the DISPLAYED (front) buffer: label
        /// solid voxels (sdf &lt; 0, weight ≥ minWeight) by 6-connectivity via
        /// iterative label propagation, then remove every component smaller than
        /// <see cref="minComponentVoxels"/> (set those voxels to +tau). Removes
        /// floating islands by total size without eroding the thin main surface.
        /// Re-publishes so Marching Cubes re-extracts. Run on a frozen result
        /// (B-mode pause) — it is a one-shot, not a per-frame, operation.
        /// </summary>
        public void RemoveSmallComponents() => RemoveSmallComponents(minComponentVoxels);

        public void RemoveSmallComponents(int minVoxels)
        {
            EnsureCcShader();
            if (_ccShader == null || FrontBuffer == null) return;
            int total = Dim.x * Dim.y * Dim.z;
            if (total <= 0) return;

            EnsureCcResources(total);

            void Bind(int k)
            {
                _ccShader.SetBuffer(k, "_Voxels", FrontBuffer);
                _ccShader.SetBuffer(k, "_Label", _ccLabel);
                _ccShader.SetBuffer(k, "_Size", _ccSize);
                _ccShader.SetBuffer(k, "_Changed", _ccChanged);
                _ccShader.SetInts("_Dim", Dim.x, Dim.y, Dim.z);
                _ccShader.SetFloat("_MinWeight", ccMinWeight);
                _ccShader.SetFloat("_Tau", Tau);
                _ccShader.SetInt("_MinComponent", Mathf.Max(1, minVoxels));
            }

            Bind(_ccInit);
            TSDFComputeUtil.DispatchLinear(_ccShader, _ccInit, total);

            int iters = Mathf.Max(1, ccMaxIterations);
            bool converged = false;
            var changed = new uint[1];
            for (int it = 0; it < iters; it++)
            {
                _ccChanged.SetData(new uint[] { 0 });
                Bind(_ccProp);
                TSDFComputeUtil.DispatchLinear(_ccShader, _ccProp, total);
                _ccChanged.GetData(changed);
                if (changed[0] == 0) { converged = true; break; }   // labels stabilised
            }

            if (!converged)
            {
                // Still changing at the cap: a large body may be split into partial
                // labels, so culling could delete sub-threshold pieces of it and
                // punch holes. Leave the volume untouched and ask for more iterations.
                Debug.LogWarning($"[TSDFVolume] connected-component labelling did not converge in " +
                                 $"{iters} iterations — skipping noise cull to avoid holes in the " +
                                 "surface. Raise ccMaxIterations.", this);
                return;
            }

            Bind(_ccResetSize);
            TSDFComputeUtil.DispatchLinear(_ccShader, _ccResetSize, total);
            Bind(_ccCount);
            TSDFComputeUtil.DispatchLinear(_ccShader, _ccCount, total);
            Bind(_ccCull);
            TSDFComputeUtil.DispatchLinear(_ccShader, _ccCull, total);

            PublishVersion++;   // re-extract Marching Cubes on the cleaned front buffer
        }

        private void EnsureCcResources(int total)
        {
            GpuBuf.Ensure(ref _ccLabel, total, sizeof(uint));
            GpuBuf.Ensure(ref _ccSize, total, sizeof(uint));
            if (_ccChanged == null)
                _ccChanged = new ComputeBuffer(1, sizeof(uint));
        }

        private void EnsureClearShader()
        {
            if (_clearShader != null) return;
            if (!TSDFComputeUtil.TryLoad(ref _clearShader, "TSDFClear", "TSDFVolume", this)) return;
            _clearKernel = _clearShader.FindKernel("Clear");
            _clearUintKernel = _clearShader.FindKernel("ClearUint");
            _clearBlocksKernel = _clearShader.FindKernel("ClearBlocks");
        }

        /// <summary>Zero a uint occupancy buffer (block-active set) via the ClearUint
        /// kernel, linearising the dispatch the same way as the voxel Clear.</summary>
        private void ClearBlockActive(ComputeBuffer buf)
        {
            if (buf == null) return;
            if (_clearShader == null) EnsureClearShader();
            if (_clearShader == null) return;

            int count = buf.count;
            _clearShader.SetBuffer(_clearUintKernel, "_UintBuf", buf);
            _clearShader.SetInt("_UintCount", count);
            _clearShader.SetInt("_UintClearValue", 0);
            TSDFComputeUtil.DispatchLinear(_clearShader, _clearUintKernel, count);
        }

        /// <summary>Depth-basis (option ④, Phase 1b): (re)allocate the double-buffered key +
        /// touched-block sets to the current grid, pairing them with the current front/write
        /// SDF buffers so they swap in lockstep. Keys init to 0xFFFFFFFF, touched to 0.</summary>
        private void EnsureDepthBuffers()
        {
            int total = Dim.x * Dim.y * Dim.z;
            int blocks = Mathf.Max(1, VoxelBlockDim.x * VoxelBlockDim.y * VoxelBlockDim.z);
            if (total <= 0) return;
            bool realloc = _keyA == null || _keyA.count != total
                           || _touchA == null || _touchA.count != blocks
                           || (doubleBuffered && (_keyB == null || _touchB == null));
            if (!realloc) return;

            _keyA?.Release(); _keyB?.Release(); _touchA?.Release(); _touchB?.Release();
            _keyA = new ComputeBuffer(total, sizeof(uint));
            _keyB = doubleBuffered ? new ComputeBuffer(total, sizeof(uint)) : null;
            _touchA = new ComputeBuffer(blocks, sizeof(uint));
            _touchB = doubleBuffered ? new ComputeBuffer(blocks, sizeof(uint)) : null;
            FullClearKey(_keyA); if (_keyB != null) FullClearKey(_keyB);
            ClearBlockActive(_touchA); if (_touchB != null) ClearBlockActive(_touchB);

            // Pair with the current front/write SDF pairing so lockstep swaps preserve
            // "each key/touched travels with its physical SDF buffer".
            bool frontIsA = ReferenceEquals(FrontBuffer, _bufA);
            FrontKeyBuffer = (doubleBuffered && !frontIsA) ? _keyB : _keyA;
            WriteKeyBuffer = doubleBuffered ? (frontIsA ? _keyB : _keyA) : _keyA;
            FrontTouched = (doubleBuffered && !frontIsA) ? _touchB : _touchA;
            WriteTouched = doubleBuffered ? (frontIsA ? _touchB : _touchA) : _touchA;
        }

        /// <summary>Full-clear a key buffer to 0xFFFFFFFF (empty / farthest |sdf|).</summary>
        private void FullClearKey(ComputeBuffer buf)
        {
            if (buf == null) return;
            if (_clearShader == null) EnsureClearShader();
            if (_clearShader == null) return;
            int count = buf.count;
            _clearShader.SetBuffer(_clearUintKernel, "_UintBuf", buf);
            _clearShader.SetInt("_UintCount", count);
            _clearShader.SetInt("_UintClearValue", unchecked((int)0xFFFFFFFF));
            TSDFComputeUtil.DispatchLinear(_clearShader, _clearUintKernel, count);
        }

        /// <summary>Depth-basis (option ④, Phase 1b): clear ONLY the blocks the scatter
        /// touched last time the write buffer was used — reset those voxels' SDF (→ +tau, 0)
        /// and key (→ 0xFFFFFFFF) and the touched flags — instead of the whole grid. The
        /// tiny MC occupancy set is still fully zeroed (cheap). Replaces the per-batch
        /// full-grid ClearWrite + key clear (~2 ms/batch → surface-proportional).</summary>
        public void ClearWriteActiveBlocks()
        {
            EnsureDepthBuffers();
            if (_clearShader == null) EnsureClearShader();
            if (_clearShader == null || WriteBuffer == null || WriteKeyBuffer == null || WriteTouched == null) return;

            int totalB = Mathf.Max(1, VoxelBlockDim.x * VoxelBlockDim.y * VoxelBlockDim.z);
            int gx = Mathf.Max(1, Mathf.Min(totalB, 65535));
            int gy = Mathf.Max(1, Mathf.CeilToInt(totalB / (float)gx));
            _clearShader.SetBuffer(_clearBlocksKernel, "_CBVoxels", WriteBuffer);
            _clearShader.SetBuffer(_clearBlocksKernel, "_CBKey", WriteKeyBuffer);
            _clearShader.SetBuffer(_clearBlocksKernel, "_CBTouched", WriteTouched);
            _clearShader.SetInts("_CBDim", Dim.x, Dim.y, Dim.z);
            _clearShader.SetInts("_CBBDim", VoxelBlockDim.x, VoxelBlockDim.y, VoxelBlockDim.z);
            _clearShader.SetFloat("_CBInitTsdf", Tau);
            _clearShader.SetInt("_CBGroupsX", gx);
            _clearShader.Dispatch(_clearBlocksKernel, gx, gy, 1);

            // MC occupancy set is marked one-sided during scatter and is tiny → full zero.
            ClearBlockActive(WriteBlockActive);
        }

        /// <summary>Depth-basis full-grid write clear (SDF + key + occupancy + touched). Used
        /// instead of <see cref="ClearWriteActiveBlocks"/> when a BeforePublish subscriber
        /// (e.g. TSDFTrailBaker) may stamp geometry into the write buffer WITHOUT marking the
        /// touched set — its residue would otherwise survive the surface-proportional clear
        /// and ghost. Correct but forfeits the Phase-1b speedup for that (niche) combo.</summary>
        public void ClearWriteFull()
        {
            EnsureDepthBuffers();
            ClearBuffer(WriteBuffer);
            FullClearKey(WriteKeyBuffer);
            ClearBlockActive(WriteBlockActive);
            if (WriteTouched != null) ClearBlockActive(WriteTouched);
        }

        /// <summary>Full reset to the clean base state the active-block-clear invariant needs
        /// (both SDF buffers → +tau/0, both keys → 0xFFFFFFFF, both touched + MC sets → 0).
        /// Call when switching INTO the depth-basis path — the voxel-basis kernel writes
        /// voxels without marking the touched set, so its residue would not be cleared by the
        /// surface-proportional clear and would ghost. Full-grid, but only on the toggle.</summary>
        public void ResetForDepthBasis()
        {
            EnsureDepthBuffers();
            ClearBuffer(_bufA); if (_bufB != null) ClearBuffer(_bufB);
            FullClearKey(_keyA); if (_keyB != null) FullClearKey(_keyB);
            ClearBlockActive(_touchA); if (_touchB != null) ClearBlockActive(_touchB);
            ClearBlockActive(_baA); if (_baB != null) ClearBlockActive(_baB);
            PublishVersion++;
        }

        /// <summary>Zero ONLY the write buffer's occupancy set, without touching the
        /// write SDF. For full-volume recompose kernels (e.g. TSDFSmoothUnion) that
        /// overwrite every voxel and re-mark from scratch: call this before dispatch so
        /// the active set reflects only the composed result, not stale bits from a
        /// prior compose. (ClearWrite also wipes the SDF, which those kernels don't want.)</summary>
        public void ClearWriteBlockActive() => ClearBlockActive(WriteBlockActive);

        /// <summary>Bind the shared active-block marking uniforms
        /// (_BlockActive/_BDim/_MarkBand from TSDFBlockActive.hlsl) so a write-time
        /// kernel marks <paramref name="blockActive"/>. Every dispatch of a kernel that
        /// includes TSDFBlockActive.hlsl (integrate/fold/trail-bake) MUST call this, or
        /// the unbound RWStructuredBuffer trips a dispatch error on some backends.</summary>
        public void BindBlockMarking(ComputeShader shader, int kernel, ComputeBuffer blockActive)
        {
            if (shader == null || blockActive == null) return;
            shader.SetBuffer(kernel, "_BlockActive", blockActive);
            shader.SetInts("_BDim", BlockDim.x, BlockDim.y, BlockDim.z);
            shader.SetFloat("_MarkBand", MarkBandVoxels * voxelSize);
        }

        /// <summary>Force a (re)build of the backing buffers now — used after toggling
        /// <see cref="doubleBuffered"/> at runtime so the change takes effect immediately
        /// instead of on the next Update. Clears the volume.</summary>
        public void ForceRebuild() => RebuildIfNeeded(true);

        private void RebuildIfNeeded(bool forceClear)
        {
            if (boundingBox == null)
            {
                if (verboseLogging) Debug.LogWarning("[TSDFVolume] boundingBox is null; skipping rebuild.", this);
                return;
            }

            var t = boundingBox.transform;
            Vector3 size = t.localScale;

            // Guard against an absurd voxel count (voxelSize grows it as 1/size^3,
            // which would exceed VRAM and overflow ComputeBuffer's int element
            // count). Rather than reject the rebuild — which would leave the
            // public doubleBuffered flag out of sync with the physical buffers —
            // CLAMP voxelSize UP until the ACTUAL ceiled dimensions fit the cap,
            // then rebuild normally. Flag and topology therefore stay consistent.
            // Bounded by the 2 GiB-per-ComputeBuffer D3D limit on the largest-stride
            // buffer (colour float4 = 16 B/voxel → 134,217,728), NOT by total VRAM:
            // 150M voxels passed the old cap but the colour alloc threw every frame.
            const long MaxVoxels = (2L * 1024 * 1024 * 1024) / 16;
            float boxVolume = Mathf.Max(1e-9f, size.x * size.y * size.z);
            // Seed from the isotropic estimate, then verify against the real
            // per-axis ceil (an anisotropic/thin box can still overflow the
            // volume estimate), raising voxelSize until dx*dy*dz <= cap.
            float v = Mathf.Max(voxelSize, Mathf.Pow(boxVolume / MaxVoxels, 1f / 3f));
            int dx, dy, dz;
            long totalL;
            for (int guard = 0; ; guard++)
            {
                dx = Mathf.Max(1, Mathf.CeilToInt(size.x / v));
                dy = Mathf.Max(1, Mathf.CeilToInt(size.y / v));
                dz = Mathf.Max(1, Mathf.CeilToInt(size.z / v));
                totalL = (long)dx * dy * dz;
                // Strictly less: at exactly MaxVoxels the colour buffer is 2 GiB,
                // which is already one byte past the D3D per-resource ceiling.
                if (totalL < MaxVoxels || guard >= 64) break;
                v *= 1.26f; // ~cbrt(2): roughly halves the voxel count per step
            }
            if (v > voxelSize)
            {
                Debug.LogWarning($"[TSDFVolume] voxelSize {voxelSize} m on bbox {size} exceeds the " +
                                 $"{MaxVoxels:N0}-voxel cap; clamping to {v:F4} m ({totalL:N0} voxels). " +
                                 "Shrink the bounding box to go finer.", this);
                voxelSize = v;
            }
            Dim = new Vector3Int(dx, dy, dz);
            // MC block grid = ceil(cells/8) where cells = dim-1 per axis (8³ cells/block).
            BlockDim = new Vector3Int(
                Mathf.Max(1, Mathf.CeilToInt((dx - 1) / 8f)),
                Mathf.Max(1, Mathf.CeilToInt((dy - 1) / 8f)),
                Mathf.Max(1, Mathf.CeilToInt((dz - 1) / 8f)));
            // Depth-basis touched/clear block grid = ceil(VOXELS/8): the touched set is
            // indexed by voxel>>3, and the max voxel (dim-1) maps to block (dim-1)/8, which
            // exceeds the MC cell-grid ceil((dim-1)/8) when dim%8==1. Must be its own dim.
            VoxelBlockDim = new Vector3Int(
                Mathf.Max(1, Mathf.CeilToInt(dx / 8f)),
                Mathf.Max(1, Mathf.CeilToInt(dy / 8f)),
                Mathf.Max(1, Mathf.CeilToInt(dz / 8f)));

            // World-from-voxel: place voxel index (i+0.5, j+0.5, k+0.5) at the
            // world position of voxel center i,j,k. Anchor is the bbox's
            // negative-local-corner, then rotate by bbox rotation, then translate
            // by bbox position. Uniform scale = voxelSize.
            Vector3 halfGrid = new Vector3(dx, dy, dz) * voxelSize * 0.5f;
            Quaternion rot = t.rotation;
            Vector3 anchor = t.position - rot * halfGrid;
            WorldFromVoxel = Matrix4x4.TRS(anchor, rot, Vector3.one * voxelSize);
            VoxelFromWorld = WorldFromVoxel.inverse;

            int total = dx * dy * dz;
            int blocks = Mathf.Max(1, BlockDim.x * BlockDim.y * BlockDim.z);
            // Reallocate on voxel-count OR block-count change: a bbox reshape can keep
            // dx*dy*dz constant while BlockDim (and _BDim used by the kernels) changes,
            // which would leave the active-block buffers sized for the old grid and let
            // CompactBlocks / MarkVoxelActive index out of bounds.
            bool realloc = _bufA == null || _bufA.count != total || _lastDoubleBuffered != doubleBuffered
                           || _baA == null || _baA.count != blocks;
            if (realloc)
            {
                ReleaseBuffer();
                // float2 = 8 bytes per voxel. At 0.01 m on a 3x2x4 m volume this
                // is ~192 MB per buffer — fine on GPUs with >=2 GB VRAM. Step 1
                // smoke test uses 0.05 m which is ~1.5 MB. doubleBuffered doubles it.
                _bufA = new ComputeBuffer(total, sizeof(float) * 2);
                _bufB = doubleBuffered ? new ComputeBuffer(total, sizeof(float) * 2) : null;
                FrontBuffer = _bufA;
                WriteBuffer = doubleBuffered ? _bufB : _bufA;
                // Parallel colour buffers (float4 rgb + colour-weight = 16 bytes).
                _colA = new ComputeBuffer(total, sizeof(float) * 4);
                _colB = doubleBuffered ? new ComputeBuffer(total, sizeof(float) * 4) : null;
                FrontColorBuffer = _colA;
                WriteColorBuffer = doubleBuffered ? _colB : _colA;
                // Active-block occupancy pair, mirroring the SDF front/write pairing
                // (single-buffer mode → one aliased set, no swap in Publish).
                _baA = new ComputeBuffer(blocks, sizeof(uint));
                _baB = doubleBuffered ? new ComputeBuffer(blocks, sizeof(uint)) : null;
                FrontBlockActive = _baA;
                WriteBlockActive = doubleBuffered ? _baB : _baA;
                if (verboseLogging)
                    Debug.Log($"[TSDFVolume] allocated {(doubleBuffered ? 2 : 1)}x {total} voxels = " +
                              $"{total * 8 / (1024 * 1024) * (doubleBuffered ? 2 : 1)} MB " +
                              $"(dim={dx}x{dy}x{dz}, voxelSize={voxelSize} m, doubleBuffered={doubleBuffered})", this);
            }

            _lastBboxPos = t.position;
            _lastBboxRot = t.rotation;
            _lastBboxScale = t.localScale;
            _lastVoxelSize = voxelSize;
            _lastDoubleBuffered = doubleBuffered;
            _dimsDirty = false;

            if (forceClear) Clear();
        }

        private void ReleaseBuffer()
        {
            GpuBuf.Release(ref _bufA);
            GpuBuf.Release(ref _bufB);
            GpuBuf.Release(ref _colA);
            GpuBuf.Release(ref _colB);
            GpuBuf.Release(ref _baA);
            GpuBuf.Release(ref _baB);
            GpuBuf.Release(ref _baInst);
            FrontBlockActive = null;
            WriteBlockActive = null;
            GpuBuf.Release(ref _instSdf);
            GpuBuf.Release(ref _instColor);
            GpuBuf.Release(ref _keyA);
            GpuBuf.Release(ref _keyB);
            GpuBuf.Release(ref _touchA);
            GpuBuf.Release(ref _touchB);
            WriteKeyBuffer = null;
            FrontKeyBuffer = null;
            WriteTouched = null;
            FrontTouched = null;
            GpuBuf.Release(ref _ccLabel);
            GpuBuf.Release(ref _ccSize);
            GpuBuf.Release(ref _ccChanged);
            FrontBuffer = null;
            WriteBuffer = null;
            FrontColorBuffer = null;
            WriteColorBuffer = null;
        }

        private void OnValidate()
        {
            // Inspector edits invalidate the grid; rebuild happens on next Update.
            _dimsDirty = true;
        }

#if UNITY_EDITOR
        // Visualise the grid extent so the user can confirm bbox + voxelSize
        // line up with the working area before they hit Play.
        private void OnDrawGizmosSelected()
        {
            if (boundingBox == null) return;
            var t = boundingBox.transform;
            Gizmos.matrix = Matrix4x4.TRS(t.position, t.rotation, Vector3.one);

            // Box outline matches the bbox extent.
            Gizmos.color = new Color(0.2f, 1f, 0.4f, 0.8f);
            Gizmos.DrawWireCube(Vector3.zero, t.localScale);

            // Hint label position only — actual dims are read from Dim once the
            // component has run OnEnable, so this is best-effort while not playing.
            Gizmos.matrix = Matrix4x4.identity;
        }
#endif
    }
}
