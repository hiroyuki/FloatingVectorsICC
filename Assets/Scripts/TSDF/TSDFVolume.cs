// Authoritative TSDF voxel grid for the v2 motion-to-mesh pipeline. Holds a
// single RWStructuredBuffer<float2> (x = signed distance, y = weight) covering
// the working volume defined by a PointCloudBoundingBox. The grid is axis-
// aligned in the bounding box's LOCAL frame so rotating the bbox rotates the
// grid with it.
//
// Step 1 of FloatingVectorsICC_v2_TSDF_spec.md §9 — this component owns the
// buffer + Clear kernel. TSDFIntegrator (step 1 commit 2) and TSDFMarching
// Cubes (commit 3) read this volume; they never allocate their own.

using PointCloud;
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
            /// <summary>Weighted average — animate-debug only. Smears motion away.</summary>
            StandardWeightedAvg = 0,
            /// <summary>Spec §3.2 (A) — keeps the closest-to-surface observation, so
            /// a hand passing through a voxel leaves a permanent shell.</summary>
            RetainGhost = 1,
        }

        [Header("Working volume")]
        [Tooltip("Defines the world-space extent and orientation of the TSDF grid. " +
                 "Local scale (x,y,z) becomes the box edge lengths; rotation is " +
                 "honored so the voxel axes follow the bbox. Mandatory.")]
        public PointCloudBoundingBox boundingBox;

        [Header("Grid resolution")]
        [Tooltip("Edge length of one (cubic) voxel in metres. Spec target is 0.01 m; " +
                 "start at 0.05 m while smoke-testing the pipeline — at 0.01 m a 3x2x4 m " +
                 "volume is ~24M voxels (~192 MB) which is fine on a 4090 but slow to " +
                 "validate on a Mac MPS dev box. Lower this AFTER the integrator + MC " +
                 "round-trip is visibly working.")]
        [Min(0.001f)]
        public float voxelSize = 0.05f;

        [Tooltip("Truncation distance = tauMultiplier * voxelSize. Spec recommends 4x.")]
        [Min(1f)]
        public float tauMultiplier = 4f;

        [Header("Accumulation")]
        [Tooltip("StandardWeightedAvg for step 1 smoke test (static targets). " +
                 "Switch to RetainGhost once the static pipeline is verified (spec §9.2).")]
        public AccumulationMode accumulationMode = AccumulationMode.StandardWeightedAvg;

        [Header("Double buffering")]
        [Tooltip("Ping-pong the voxel buffer so the views (Voxel/Cell/Mesh) only ever " +
                 "read a COMPLETE batch while the integrator fills the next one into a " +
                 "hidden back buffer. Eliminates the mid-clear / partial-fill flicker. " +
                 "Driven by TSDFIntegrator to match its mode: ON for live-follow " +
                 "(clearVolumeOnNewBatch), OFF for accumulate (read-modify-write needs " +
                 "a single persistent buffer). Costs 2x VRAM when on.")]
        public bool doubleBuffered = true;

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

        /// <summary>Bumped every Publish() (and on full Clear / rebuild). TSDFView reads
        /// this to re-extract Marching Cubes only when new content was published, so all
        /// three view modes update in lockstep with the same cadence.</summary>
        public int PublishVersion { get; private set; }

        public Vector3Int Dim { get; private set; }
        public float Tau => tauMultiplier * voxelSize;

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
        private bool _dimsDirty = true;

        // Physical backing buffers. _bufB is null in single-buffer mode.
        private ComputeBuffer _bufA;
        private ComputeBuffer _bufB;
        private bool _lastDoubleBuffered;

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

            // 64-thread groups; voxel count rounded up.
            int total = Dim.x * Dim.y * Dim.z;
            int groups = Mathf.CeilToInt(total / 64f);
            _clearShader.Dispatch(_clearKernel, Mathf.Max(1, groups), 1, 1);
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
            }
            PublishVersion++;
        }

        private void EnsureClearShader()
        {
            if (_clearShader != null) return;
            _clearShader = Resources.Load<ComputeShader>("TSDFClear");
            if (_clearShader == null)
            {
                Debug.LogError("[TSDFVolume] Compute shader \"Resources/TSDFClear.compute\" not found. " +
                               "Did the Resources/ folder import correctly?", this);
                return;
            }
            _clearKernel = _clearShader.FindKernel("Clear");
        }

        private void RebuildIfNeeded(bool forceClear)
        {
            if (boundingBox == null)
            {
                if (verboseLogging) Debug.LogWarning("[TSDFVolume] boundingBox is null; skipping rebuild.", this);
                return;
            }

            var t = boundingBox.transform;
            Vector3 size = t.localScale;
            int dx = Mathf.Max(1, Mathf.CeilToInt(size.x / voxelSize));
            int dy = Mathf.Max(1, Mathf.CeilToInt(size.y / voxelSize));
            int dz = Mathf.Max(1, Mathf.CeilToInt(size.z / voxelSize));
            Dim = new Vector3Int(dx, dy, dz);

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
            bool realloc = _bufA == null || _bufA.count != total || _lastDoubleBuffered != doubleBuffered;
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
            _bufA?.Release();
            _bufB?.Release();
            _bufA = null;
            _bufB = null;
            FrontBuffer = null;
            WriteBuffer = null;
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
