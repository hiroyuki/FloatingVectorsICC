// Drives the per-camera TSDF integration kernel. Subscribes to both the live
// (PointCloudRenderer.OnRawFramesReady) and playback (SensorRecorder
// .OnPlaybackRawFrame) raw-frame events so the same component works in both
// modes without scene re-wiring.
//
// Step 1b of FloatingVectorsICC_v2_TSDF_spec.md §9. The TSDF volume is owned
// by TSDFVolume; we only push observations into it.

using System.Collections.Generic;
using Orbbec;
using PointCloud;
using Shared;
using UnityEngine;

namespace TSDF
{
    /// <summary>
    /// Subscribes to the raw-frame pipeline, uploads per-camera depth bytes
    /// to GPU, and dispatches the Integrate kernel once per cam per frame.
    /// </summary>
    [DefaultExecutionOrder(0)]
    public sealed class TSDFIntegrator : MonoBehaviour, global::Shared.IPanelTunable
    {
        [Tooltip("Debug A/B switch: when on, skip the forward depth-lens distortion " +
                 "and project voxels with the bare pinhole model, so a TSDF mesh can " +
                 "be rebuilt with and without the correction for comparison. " +
                 "Mirrors PointCloudReconstructor.ForcePinhole.")]
        public bool forcePinhole = false;

        [Tooltip("Volume to integrate into. Auto-located on enable if left null.")]
        public TSDFVolume volume;

        [Tooltip("Base observation weight per frame per camera. Spec §3.1: " +
                 "scaled by view-angle / range later — start at 1.0.")]
        [Min(0f)] public float observationWeight = 1f;

        [Tooltip("Silhouette-edge rejection radius in depth pixels (0 = off). " +
                 "Depth pixels whose (2r+1)^2 neighbourhood contains an invalid " +
                 "pixel or a depth jump larger than edgeRejectDepthMm are skipped, " +
                 "so D2C misalignment at the person's rim can't bake background " +
                 "(white wall) colour into the TSDF.")]
        [Range(0, 4)] public int edgeRejectRadius = 2;

        [Tooltip("Depth discontinuity threshold (mm) for the edge rejection. " +
                 "Neighbour depths differing more than this mark the pixel as a " +
                 "silhouette edge. Keep well above the per-pixel gradient of " +
                 "steeply slanted real surfaces (~100 mm is a safe start).")]
        [Range(10f, 500f)] public float edgeRejectDepthMm = 100f;

        [Tooltip("Depth-basis only: instead of REJECTING silhouette-edge pixels " +
                 "(which visibly thins limbs — their whole perimeter is 'edge'), keep " +
                 "their GEOMETRY and only swap the colour for one borrowed from a " +
                 "nearby interior pixel. Full limb thickness + no background-colour " +
                 "rim bleed. edgeRejectRadius then just defines what counts as edge. " +
                 "Off = legacy reject (and the voxel-basis path always rejects).")]
        public bool edgeColorBorrow = true;

        [Tooltip("Dev colour override: when alpha > 0, bake this flat colour into " +
                 "integrated voxels instead of the camera RGB (debug tooling uses it " +
                 "to tag frames/instants — e.g. red vs blue). Alpha == 0 (default) = " +
                 "use camera colour. Callers must reset it after use.")]
        public Color colorOverride = new Color(0f, 0f, 0f, 0f);

        [Tooltip("Subscribe to live PointCloudRenderer.OnRawFramesReady too. " +
                 "Leave on so live capture also feeds the TSDF when a Femto Bolt " +
                 "is attached.")]
        public bool subscribeLive = true;

        [Tooltip("Subscribe to every SensorRecorder.OnPlaybackRawFrame in " +
                 "the scene. On Mac dev without live cameras this is the only " +
                 "feed.")]
        public bool subscribePlayback = true;

        [Tooltip("Live-follow mode: wipe the TSDF volume once every batch of " +
                 "<expectedCamCount> unique cameras has integrated, then start " +
                 "accumulating the next batch. The mesh therefore reflects only " +
                 "the most recent complete multi-cam observation and tracks the " +
                 "current point cloud over time.")]
        public bool clearVolumeOnNewBatch = true;

        [Tooltip("Live-follow normally double-buffers the volume so views never see a " +
                 "mid-clear / partial-fill front. Forcing single-buffer halves the volume's " +
                 "VRAM (~3.4 GB at 5.2 mm voxels) at the cost of that flicker — trade-off " +
                 "for fitting 4x full-model k4abt workers on a 12 GB GPU. No effect in " +
                 "accumulate mode, which is single-buffered anyway.")]
        public bool forceSingleBuffer = false;

        [Tooltip("Number of unique serials per batch. Default 4 = one Femto Bolt " +
                 "set; lower it if you are debugging with fewer cams attached.")]
        [Min(1)] public int expectedCamCount = 4;

        [Header("Depth-basis integration (option ④ — perf refactor)")]
        [Tooltip("Use the DEPTH-BASIS scatter kernel (1 thread per depth pixel, " +
                 "~369K/frame) instead of the voxel-basis kernel (1 thread per voxel, " +
                 "~515M/frame). Off = original path (A/B and rollback). Live-follow " +
                 "(clearVolumeOnNewBatch) only for now; accumulate mode still uses the " +
                 "voxel-basis kernel. See Plans/tsdf-depth-basis-integration-design.md.")]
        public bool useDepthBasis = false;

        [Tooltip("Depth-basis: axis-neighbour splat radius (0 or 1) to close lateral " +
                 "screen-door holes where voxelSize <= depth-pixel footprint. Start at " +
                 "1; drop to 0 if the mesh is hole-free without it (cheaper).")]
        [Range(0, 1)] public int latRadius = 1;

        [Tooltip("Depth-basis: capsule radius around each pixel's view ray, in VOXELS. " +
                 "Wide enough to bridge inter-pixel gaps (no holes), narrow enough to " +
                 "avoid grazing-angle fattening. ~1.0-1.4 is the sweet spot; raise if " +
                 "holes reappear, lower if the body looks bloated.")]
        [Range(0.5f, 3f)] public float tubeRadiusVoxels = 1.2f;

        // Runtime gate for the live/playback integration path. When false, incoming
        // frames are ignored so the volume (and thus the mesh) freezes on its last
        // state. MeshCumulative flips this to hold the final mesh after a run ends
        // (spec §7). Debug replay via IntegrateRawFrame bypasses this gate.
        // NonSerialized so a frozen state (e.g. left by the fixed-frame bench) can
        // never get saved into the scene — it always starts true on load/play.
        [System.NonSerialized] public bool integrationEnabled = true;

        // Set of serials that have integrated since the last clear. Once it
        // reaches expectedCamCount, the batch is published to the front buffer
        // and the next integrate clears the back buffer to start a fresh batch.
        private readonly System.Collections.Generic.HashSet<string> _batchSerials
            = new System.Collections.Generic.HashSet<string>();

        /// <summary>
        /// Monotonically increasing per-batch counter. Bumped each time a batch
        /// reaches <see cref="expectedCamCount"/> unique serials (= one Publish).
        /// Kept as a diagnostic; view refresh is driven by TSDFVolume.PublishVersion.
        /// </summary>
        public int CompletedBatchCount { get; private set; }

        /// <summary>
        /// Fired ONCE per completed multi-cam batch, immediately BEFORE the batch is
        /// published to the front buffer. Subscribers (e.g. TSDFTrailBaker.LiveFuse)
        /// may min-union extra geometry into <see cref="TSDFVolume.WriteBuffer"/> so it
        /// rides along in the same Publish — they must NOT publish themselves. The
        /// integrator always publishes right after, even if a subscriber throws
        /// (each is invoked in its own try/catch), so a failing trail can never stall
        /// the body's publish. Fires for both the live-follow (clear-per-batch) and
        /// accumulate (fold) paths, just before their respective Publish().
        /// </summary>
        public event System.Action<TSDFIntegrator, TSDFVolume> BeforePublishCompleteBatch;

        // Invoke each subscriber in isolation so one that throws neither stops the
        // others nor blocks the Publish that follows (Codex: a trail-bake exception
        // must never freeze the body mesh).
        private void InvokeBeforePublishCompleteBatch()
        {
            var ev = BeforePublishCompleteBatch;
            if (ev == null) return;
            foreach (var d in ev.GetInvocationList())
            {
                try { ((System.Action<TSDFIntegrator, TSDFVolume>)d)(this, volume); }
                catch (System.Exception e) { Debug.LogException(e, this); }
            }
        }

        [Tooltip("Log a per-second summary of integrated frames per serial.")]
        public bool diagnosticLogging = false;

        // ---- Shared.IPanelTunable (one-stop Control Panel) ----
        // Silhouette-edge rejection knobs: radius 0 turns the test off live, so
        // the white-rim bleed can be A/B'd from the panel without touching the
        // Inspector. Values are read per dispatch — effective next frame.
        public string TuningLabel => "TSDF edge reject";
        public int TunableCount => 2;
        // Labels must match the Inspector field names (Edge Reject Radius /
        // Edge Reject Depth Mm) so the two UIs are recognisably the same knob.
        public string TunableName(int i) => i == 0 ? "Edge Reject Radius (0=off)" : "Edge Reject Depth Mm";
        public float TunableValue(int i) => i == 0 ? edgeRejectRadius : edgeRejectDepthMm;
        public void SetTunableValue(int i, float value)
        {
            if (i == 0) edgeRejectRadius = Mathf.Clamp(Mathf.RoundToInt(value), 0, 4);
            else edgeRejectDepthMm = Mathf.Clamp(value, 10f, 500f);
        }
        public float TunableMin(int i) => i == 0 ? 0f : 10f;
        public float TunableMax(int i) => i == 0 ? 4f : 500f;
        public bool TunableIsInt(int i) => i == 0;

        private ComputeShader _shader;
        private int _kernel;

        // Depth-basis scatter kernel (option ④). Loaded lazily when useDepthBasis is on.
        private ComputeShader _depthShader;
        private int _kScatterMin;
        private int _kScatterWrite;
        // Tracks whether the depth-basis path was active last dispatch, so a mid-run
        // toggle (Inspector or accumulationMode change) resets the shared batch state
        // and clears the write buffer instead of letting one path inherit the other's
        // in-flight batch. -1 = uninitialised.
        private int _depthPathActive = -1;
        private bool _warnedDepthFallback;

        // Per-serial GPU + CPU state. Keyed by device serial so the same
        // entry survives the live <-> playback handoff.
        private sealed class CamState
        {
            public string Serial;
            public Transform SourceTransform;
            public ObCameraParam CamParam;
            public bool HasCamParam;
            public ComputeBuffer DepthBuf;      // ComputeBufferType.Raw, byte/4 uints
            public uint[] DepthScratchU32;      // BlockCopy target for the SetData upload
            public int DepthByteCount;
            public int DepthWidth;
            public int DepthHeight;
            public ComputeBuffer ColorBuf;      // ComputeBufferType.Raw, RGB8 bytes / 4 uints
            public uint[] ColorScratchU32;
            public int ColorByteCount;
            public int ColorWidth;
            public int ColorHeight;
            public int FramesIntegrated;        // diag counter, reset per second

            // Depth-basis: undistorted-ray LUT (shared with the point cloud). Built once
            // per (intrinsics, distortion, resolution); rebuilt only when those change.
            public ComputeBuffer RayLutBuf;     // StructuredBuffer<float2>, or 1-elem dummy
            public bool HasRayLut;
            public int RayLutW, RayLutH;
            public ObCameraDistortion RayLutDist;   // full cache key: coeffs + model...
            public ObCameraIntrinsic RayLutIntr;    // ...intrinsics...
            public bool RayLutForcePinhole;         // ...and the forcePinhole debug toggle.
            public bool LastHadColor;           // did the last uploaded frame carry colour
        }
        private readonly Dictionary<string, CamState> _states = new Dictionary<string, CamState>();

        // Tracked subscriptions so we can detach cleanly. Keyed by component
        // so the same renderer reappearing after a Stop+Play binds once.
        private readonly Dictionary<PointCloudRenderer,
                                   System.Action<PointCloudRenderer, RawFrameData>> _liveHandlers
            = new Dictionary<PointCloudRenderer,
                              System.Action<PointCloudRenderer, RawFrameData>>();
        private readonly Dictionary<SensorRecorder,
                                   System.Action<string, ObCameraParam?, Transform, RawFrameData>> _playbackHandlers
            = new Dictionary<SensorRecorder,
                              System.Action<string, ObCameraParam?, Transform, RawFrameData>>();

        private float _diagWindowStart;

        private void OnEnable()
        {
            if (volume == null) volume = FindAnyObjectByType<TSDFVolume>(FindObjectsInactive.Include);
            if (volume == null)
            {
                Debug.LogError("[TSDFIntegrator] No TSDFVolume in scene. Component disabled.", this);
                enabled = false;
                return;
            }
            EnsureShader();
            // Live-follow (clear-per-batch) wants double buffering so views never
            // see a mid-clear / partial-fill front. Accumulate (no clear) is a
            // read-modify-write that needs ONE persistent buffer. Drive the volume
            // to match our mode so the two stay consistent. forceSingleBuffer
            // trades the flicker back for half the volume VRAM.
            volume.doubleBuffered = clearVolumeOnNewBatch && !forceSingleBuffer;
            BindAllSources();
        }

        private void OnDisable()
        {
            UnbindAll();
            foreach (var kv in _states)
            {
                GpuBuf.Release(ref kv.Value.DepthBuf);
                GpuBuf.Release(ref kv.Value.ColorBuf);
                GpuBuf.Release(ref kv.Value.RayLutBuf);
            }
            _states.Clear();
        }

        private void EnsureShader()
        {
            if (_shader != null) return;
            if (!TSDFComputeUtil.TryLoad(ref _shader, "TSDFIntegrate", "TSDFIntegrator", this))
            {
                enabled = false;
                return;
            }
            _kernel = _shader.FindKernel("Integrate");
        }

        private bool EnsureDepthShader()
        {
            if (_depthShader != null) return true;
            if (!TSDFComputeUtil.TryLoad(ref _depthShader, "TSDFIntegrateDepth", "TSDFIntegrator", this)) return false;
            _kScatterMin = _depthShader.FindKernel("ScatterMin");
            _kScatterWrite = _depthShader.FindKernel("ScatterWrite");
            return true;
        }

        private void BindAllSources()
        {
            if (subscribeLive)
            {
                foreach (var r in FindObjectsByType<PointCloudRenderer>(FindObjectsInactive.Exclude, FindObjectsSortMode.None))
                    BindLive(r);
            }
            if (subscribePlayback)
            {
                foreach (var rec in FindObjectsByType<SensorRecorder>(FindObjectsInactive.Exclude, FindObjectsSortMode.None))
                    BindPlayback(rec);
            }
        }

        private void BindLive(PointCloudRenderer r)
        {
            if (r == null || _liveHandlers.ContainsKey(r)) return;
            System.Action<PointCloudRenderer, RawFrameData> h = HandleLiveRawFrame;
            r.OnRawFramesReady += h;
            _liveHandlers[r] = h;
        }

        private void BindPlayback(SensorRecorder rec)
        {
            if (rec == null || _playbackHandlers.ContainsKey(rec)) return;
            System.Action<string, ObCameraParam?, Transform, RawFrameData> h = HandlePlaybackRawFrame;
            rec.OnPlaybackRawFrame += h;
            _playbackHandlers[rec] = h;
        }

        private void UnbindAll()
        {
            foreach (var kv in _liveHandlers)
                if (kv.Key != null) kv.Key.OnRawFramesReady -= kv.Value;
            _liveHandlers.Clear();
            foreach (var kv in _playbackHandlers)
                if (kv.Key != null) kv.Key.OnPlaybackRawFrame -= kv.Value;
            _playbackHandlers.Clear();
        }

        private void HandleLiveRawFrame(PointCloudRenderer r, RawFrameData raw)
        {
            if (r == null) return;
            // Live renderer's own GameObject transform IS the source — its
            // localToWorldMatrix encodes the extrinsics + Y-flip baked in by
            // ExtrinsicsApply.ToUnityLocal.
            DispatchIntegrate(r.deviceSerial, r.CameraParam, r.transform, raw);
        }

        private void HandlePlaybackRawFrame(string serial, ObCameraParam? camParam, Transform sourceTransform, RawFrameData raw)
        {
            DispatchIntegrate(serial, camParam, sourceTransform, raw);
        }

        /// <summary>
        /// Integrate ONE depth frame from one camera into <see cref="volume"/>.
        /// Exposed so debug / sub-frame interpolation paths can feed synthetic or
        /// snapshotted frames without going through the OnRawFramesReady /
        /// OnPlaybackRawFrame events.
        /// </summary>
        public void IntegrateRawFrame(string serial, ObCameraParam? camParam,
                                      Transform sourceTransform, RawFrameData raw)
            => IntegrateOne(serial, camParam, sourceTransform, raw,
                            volume != null ? volume.WriteBuffer : null,
                            volume != null ? volume.WriteColorBuffer : null,
                            -1);

        /// <summary>
        /// Reset live-follow batch tracking and clear the write buffer so the NEXT
        /// live/playback batch starts clean. Call after external code (e.g. debug
        /// B mode) has driven volume.ClearWrite/Publish + IntegrateRawFrame directly
        /// while <see cref="integrationEnabled"/> was off — otherwise the resumed
        /// batch state machine could think it is mid-batch and publish a partial or
        /// debug-contaminated buffer for the first batch.
        /// </summary>
        public void BeginFreshBatch()
        {
            _batchSerials.Clear();
            if (volume != null) volume.ClearWrite();
        }

        /// <summary>
        /// Drop the in-flight multi-cam batch WITHOUT touching any buffer. For hold-time
        /// interventions (TSDFHoldBeautify) that reuse the write/instance scratch while a
        /// batch may be partially collected: clearing the serial set forces every camera
        /// to re-arrive after resume before a batch can complete, so a batch can never mix
        /// pre-hold and post-hold frames. Buffers are deliberately left alone — in single-
        /// buffer (frozen accumulate) mode a ClearWrite here would erase the held mesh, and
        /// each path re-establishes its own scratch at batch/instant start anyway
        /// (live-follow: the write buffer was already fully re-cleared by the intervention;
        /// accumulate: the empty serial set triggers ClearInstance on the next arrival).
        /// </summary>
        public void DropInFlightBatch() => _batchSerials.Clear();

        /// <summary>
        /// Force the next depth-basis batch to wipe the WHOLE write buffer instead of
        /// the touched-blocks-only fast clear. Needed after anything writes voxels
        /// outside the touched-set bookkeeping — e.g. TSDFHoldBeautify's median repair
        /// on a SINGLE-buffered volume (front == write there, and its smeared voxels
        /// land in blocks the partial clear never resets, ghosting the held frame
        /// into live forever).
        /// </summary>
        public void RequestFullClearNextBatch() => _fullClearPending = true;
        private bool _fullClearPending;

        /// <summary>
        /// Event-path entry: integrate one cam frame AND drive the live-follow
        /// batch state machine (clear the back buffer at batch start, publish it
        /// to the front once <see cref="expectedCamCount"/> unique cams have
        /// landed). The raw integrate itself is in <see cref="IntegrateOne"/> so
        /// debug replay can drive clear/publish on its own schedule.
        /// </summary>
        private void DispatchIntegrate(string serial, ObCameraParam? camParam,
                                       Transform sourceTransform, RawFrameData raw)
        {
            if (volume == null) return;
            if (!integrationEnabled) return; // frozen — hold the last accumulated state

            // Depth-basis scatter (option ④) is only valid for live-follow + RetainGhost
            // camera fusion (the |sdf|-min the 2-pass InterlockedMin implements). Average
            // fusion and the accumulate/Fold path are NOT ported yet, so fall back to the
            // voxel-basis kernel for those — enabling the perf flag must never silently
            // change fusion semantics.
            bool depthActive = useDepthBasis && clearVolumeOnNewBatch
                && volume.accumulationMode == TSDFVolume.AccumulationMode.RetainGhost;

            if (useDepthBasis && !depthActive && !_warnedDepthFallback)
            {
                Debug.LogWarning("[TSDFIntegrator] useDepthBasis is on but the depth-basis path " +
                    "only supports live-follow + RetainGhost fusion; using the voxel-basis kernel " +
                    "for this configuration (clearVolumeOnNewBatch=" + clearVolumeOnNewBatch +
                    ", accumulationMode=" + volume.accumulationMode + ").", this);
                _warnedDepthFallback = true;
            }

            // On any transition between the two paths, drop the shared in-flight batch so
            // neither path inherits the other's stale batch state. Entering depth-basis also
            // needs a FULL reset (both SDF buffers + keys + touched): the voxel path writes
            // voxels without marking the touched set, so its residue would survive the
            // surface-proportional active-block clear and ghost. Leaving depth-basis just
            // needs the voxel path's usual write-buffer wipe.
            int depthFlag = depthActive ? 1 : 0;
            if (_depthPathActive != depthFlag)
            {
                _batchSerials.Clear();
                if (depthActive) volume.ResetForDepthBasis();
                else volume.ClearWrite();
                _depthPathActive = depthFlag;
                if (depthActive) _warnedDepthFallback = false; // re-warn if it falls back again later
            }

            if (depthActive)
            {
                DispatchIntegrateDepth(serial, camParam, sourceTransform, raw);
                return;
            }

            if (clearVolumeOnNewBatch)
            {
                // Live-follow: when a complete batch finished last time, wipe the
                // hidden BACK buffer here at the START of the next batch's first
                // integration. The displayed front keeps showing the last complete
                // batch until this one finishes and publishes — no flicker.
                // A pending full clear (hold-beautify residue) must fire here too:
                // RunNow dropped the in-flight serials, so the count-based clear
                // would skip and the first resumed batch would integrate on top of
                // the beautified front (single buffer: front == write). ClearWrite
                // (not ClearWriteFull) — the voxel path needs the full-grid SDF wipe
                // only; ClearWriteFull is depth-basis-specific and would allocate
                // the key/touched buffers just to clear them.
                if (_fullClearPending)
                {
                    volume.ClearWrite();
                    _fullClearPending = false;
                    _batchSerials.Clear();
                }
                else if (_batchSerials.Count >= expectedCamCount)
                {
                    volume.ClearWrite();
                    _batchSerials.Clear();
                }

                if (!IntegrateOne(serial, camParam, sourceTransform, raw,
                                  volume.WriteBuffer, volume.WriteColorBuffer, -1)) return;

                // Tally this serial into the in-flight batch. Once full, bump the
                // batch counter and publish the back buffer to the front so all
                // three views pick up the same complete snapshot this frame.
                _batchSerials.Add(serial);
                if (_batchSerials.Count == expectedCamCount)
                {
                    CompletedBatchCount++;
                    InvokeBeforePublishCompleteBatch();   // baker may min-union a trail into WriteBuffer
                    volume.Publish();
                }
            }
            else
            {
                // Accumulate: separate camera fusion from time accumulation. Average
                // this instant's cameras into the scratch INSTANCE buffer (denoise →
                // one clean surface), then union the finished clean instant into the
                // persistent accumulation buffer over time (the motion trail). This
                // stops thin limbs fraying into double-shell strands. The instance is
                // cleared at the start of each instant (batch empty); also
                // (re)allocate if a mid-accumulate grid rebuild released it.
                if (_batchSerials.Count == 0 || volume.InstanceBuffer == null) volume.ClearInstance();

                if (!IntegrateOne(serial, camParam, sourceTransform, raw,
                                  volume.InstanceBuffer, volume.InstanceColorBuffer, 0)) return;

                _batchSerials.Add(serial);
                if (_batchSerials.Count >= expectedCamCount)
                {
                    volume.FoldInstanceIntoAccumulation();   // time union → mesh re-extracts
                    _batchSerials.Clear();
                    CompletedBatchCount++;
                    InvokeBeforePublishCompleteBatch();   // baker may min-union a trail into WriteBuffer
                    volume.Publish();
                }
            }
        }

        /// <summary>
        /// Validate one raw frame and upload its depth (+ colour) bytes into the
        /// per-serial <see cref="CamState"/>. Shared by the voxel-basis IntegrateOne
        /// and the depth-basis batch path. Returns null if the frame was rejected
        /// (missing intrinsics / no depth / no transform); <paramref name="hasColor"/>
        /// reports whether a colour frame was uploaded this call.
        /// </summary>
        private CamState PrepareCamState(string serial, ObCameraParam? camParam,
                                         Transform sourceTransform, RawFrameData raw,
                                         out bool hasColor)
        {
            hasColor = false;
            if (string.IsNullOrEmpty(serial)) return null;
            if (raw.DepthBytes == null || raw.DepthByteCount <= 0) return null;
            if (!camParam.HasValue) return null; // need intrinsics
            if (sourceTransform == null) return null;

            if (!_states.TryGetValue(serial, out var st))
            {
                st = new CamState { Serial = serial };
                _states[serial] = st;
            }
            st.SourceTransform = sourceTransform;
            st.CamParam = camParam.Value;
            st.HasCamParam = true;
            st.DepthByteCount = raw.DepthByteCount;
            st.DepthWidth = raw.DepthWidth;
            st.DepthHeight = raw.DepthHeight;

            // Upload depth bytes to a raw ComputeBuffer (4-byte uints). The
            // backing buffer stride is 4 (uint), so SetData's `count` argument
            // is the number of UINT ELEMENTS — not bytes. Match the working
            // pattern from PointCloudReconstructor: BlockCopy the raw bytes
            // into a scratch uint[] first, then upload by element count.
            int uintCount = (raw.DepthByteCount + 3) / 4;
            GpuBuf.Ensure(ref st.DepthBuf, uintCount, 4, ComputeBufferType.Raw);
            if (st.DepthScratchU32 == null || st.DepthScratchU32.Length != uintCount)
                st.DepthScratchU32 = new uint[uintCount];
            System.Buffer.BlockCopy(raw.DepthBytes, 0, st.DepthScratchU32, 0, raw.DepthByteCount);
            st.DepthBuf.SetData(st.DepthScratchU32, 0, 0, uintCount);

            // Upload the colour frame (RGB8) the same way, so the integrate kernel
            // can sample the camera colour per voxel. When the frame has no colour
            // (depth-only), keep a 1-uint dummy bound and flag _HasColor=0 — the
            // kernel's binding must still exist even though it short-circuits.
            hasColor = raw.ColorBytes != null && raw.ColorByteCount > 0
                       && raw.ColorWidth > 0 && raw.ColorHeight > 0;
            if (hasColor)
            {
                st.ColorByteCount = raw.ColorByteCount;
                st.ColorWidth = raw.ColorWidth;
                st.ColorHeight = raw.ColorHeight;
                int cUintCount = (raw.ColorByteCount + 3) / 4;
                GpuBuf.Ensure(ref st.ColorBuf, cUintCount, 4, ComputeBufferType.Raw);
                if (st.ColorScratchU32 == null || st.ColorScratchU32.Length != cUintCount)
                    st.ColorScratchU32 = new uint[cUintCount];
                System.Buffer.BlockCopy(raw.ColorBytes, 0, st.ColorScratchU32, 0, raw.ColorByteCount);
                st.ColorBuf.SetData(st.ColorScratchU32, 0, 0, cUintCount);
            }
            else if (st.ColorBuf == null)
            {
                st.ColorBuf = new ComputeBuffer(1, 4, ComputeBufferType.Raw);
            }
            return st;
        }

        /// <summary>
        /// Integrate ONE depth frame into <see cref="TSDFVolume.WriteBuffer"/>. No
        /// batch / clear / publish side effects — pure GPU integrate + counters.
        /// Returns false if the frame was rejected (missing intrinsics, no depth…).
        /// </summary>
        private bool IntegrateOne(string serial, ObCameraParam? camParam,
                                         Transform sourceTransform, RawFrameData raw,
                                         ComputeBuffer targetSdf, ComputeBuffer targetColor,
                                         int modeOverride)
        {
            if (volume == null || targetSdf == null) return false;
            var st = PrepareCamState(serial, camParam, sourceTransform, raw, out bool hasColor);
            if (st == null) return false;

            // Build per-camera world->depth-mm matrix.
            Matrix4x4 depthFromWorld = ComputeDepthFromWorld(sourceTransform, st.CamParam);
            var intr = st.CamParam.DepthIntrinsic;

            _shader.SetBuffer(_kernel, "_Voxels", targetSdf);
            _shader.SetInts("_Dim", volume.Dim.x, volume.Dim.y, volume.Dim.z);
            _shader.SetMatrix("_WorldFromVoxel", volume.WorldFromVoxel);
            _shader.SetFloat("_Tau", volume.Tau);
            _shader.SetInt("_AccumulationMode", modeOverride >= 0 ? modeOverride : (int)volume.accumulationMode);

            // Active-block marking (task 0-1): mark the occupancy set of whichever
            // buffer we're filling. Live-follow writes the write buffer (→ its set, MC-
            // consumed after Publish); accumulate writes the instance scratch (→ its
            // unread set — the real accumulation set is marked later by TSDFFold).
            volume.BindBlockMarking(_shader, _kernel,
                ReferenceEquals(targetSdf, volume.WriteBuffer)
                    ? volume.WriteBlockActive
                    : volume.InstanceBlockActive);

            _shader.SetBuffer(_kernel, "_Depth", st.DepthBuf);
            _shader.SetInt("_DepthW", raw.DepthWidth);
            _shader.SetInt("_DepthH", raw.DepthHeight);
            _shader.SetFloat("_FxD", intr.Fx);
            _shader.SetFloat("_FyD", intr.Fy);
            _shader.SetFloat("_CxD", intr.Cx);
            _shader.SetFloat("_CyD", intr.Cy);
            // Forward depth-lens distortion (shares the point cloud's gate so the
            // two paths agree on when distortion applies). The voxel ray is
            // distorted before projecting, matching the real wide lens.
            var ddist = st.CamParam.DepthDistortion;
            bool hasDepthDist = !forcePinhole && PointCloud.DepthUndistortLut.IsApplicable(ddist);
            _shader.SetInt("_HasDepthDist", hasDepthDist ? 1 : 0);
            _shader.SetFloat("_DK1", ddist.K1);
            _shader.SetFloat("_DK2", ddist.K2);
            _shader.SetFloat("_DK3", ddist.K3);
            _shader.SetFloat("_DK4", ddist.K4);
            _shader.SetFloat("_DK5", ddist.K5);
            _shader.SetFloat("_DK6", ddist.K6);
            _shader.SetFloat("_DP1", ddist.P1);
            _shader.SetFloat("_DP2", ddist.P2);
            _shader.SetMatrix("_DepthFromWorld", depthFromWorld);
            _shader.SetFloat("_WObs", observationWeight);
            _shader.SetInt("_EdgeRejectRadius", edgeRejectRadius);
            _shader.SetFloat("_EdgeRejectDepthMm", edgeRejectDepthMm);

            // Colour buffer + colour-camera projection. world->colour-mm is the
            // renderer-transform half of depthFromWorld (the source GO transform
            // is already in colour-camera metres), scaled to mm — without the
            // colour->depth extrinsic inverse that depthFromWorld applies.
            Matrix4x4 colorFromWorld = Units.MToMmScale
                                       * sourceTransform.worldToLocalMatrix;
            var cintr = st.CamParam.RgbIntrinsic;
            _shader.SetBuffer(_kernel, "_Colors", targetColor);
            _shader.SetBuffer(_kernel, "_ColorImg", st.ColorBuf);
            _shader.SetInt("_ColorW", st.ColorWidth);
            _shader.SetInt("_ColorH", st.ColorHeight);
            _shader.SetFloat("_FxC", cintr.Fx);
            _shader.SetFloat("_FyC", cintr.Fy);
            _shader.SetFloat("_CxC", cintr.Cx);
            _shader.SetFloat("_CyC", cintr.Cy);
            _shader.SetMatrix("_ColorFromWorld", colorFromWorld);
            _shader.SetInt("_HasColor", hasColor ? 1 : 0);
            _shader.SetInt("_UseColorOverride", colorOverride.a > 0f ? 1 : 0);
            _shader.SetVector("_OverrideColor", new Vector4(colorOverride.r, colorOverride.g, colorOverride.b, 0f));

            // 2D group grid so fine volumes (voxelSize 0.01 → >100k groups) stay
            // under the 65535 threadgroup-per-axis limit. The kernel linearises
            // via _DispatchWidth = groupsX * 64.
            int total = volume.Dim.x * volume.Dim.y * volume.Dim.z;
            TSDFComputeUtil.DispatchLinear(_shader, _kernel, total);

            st.FramesIntegrated++;
            TotalIntegrationCount++;
            return true;
        }

        // ===== Depth-basis integration (option ④) ===============================

        /// <summary>
        /// Depth-basis live-follow entry: upload each camera's frame as it arrives,
        /// and once a full batch (<see cref="expectedCamCount"/> unique serials) has
        /// landed, run the 2-pass scatter over all cameras and publish. Deferring the
        /// integrate to batch-complete lets the |sdf|-min <c>_VoxelKey</c> be cleared
        /// exactly once per batch (design §8).
        /// </summary>
        private void DispatchIntegrateDepth(string serial, ObCameraParam? camParam,
                                            Transform sourceTransform, RawFrameData raw)
        {
            if (!EnsureDepthShader()) return;
            var st = PrepareCamState(serial, camParam, sourceTransform, raw, out bool hasColor);
            if (st == null) return;
            st.LastHadColor = hasColor;
            EnsureRayLut(st);
            _batchSerials.Add(serial);
            if (_batchSerials.Count < expectedCamCount) return;

            RunDepthBatch();
            _batchSerials.Clear();
        }

        // Wipe the back buffer + key set, scatter every batch camera (all ScatterMin,
        // then all ScatterWrite so the global |sdf|-min is settled before materialise),
        // then publish the completed snapshot.
        private void RunDepthBatch()
        {
            // Phase 1b: surface-proportional clear — reset only the blocks the previous use
            // of this write buffer touched (SDF → +tau/0, key → 0xFFFFFFFF), not the whole
            // grid. Replaces the ~2 ms/batch full-grid ClearWrite + key clear (measured
            // −1.9 ms/frame, +10 fps). EXCEPTION: a BeforePublish subscriber (e.g.
            // TSDFTrailBaker) can stamp geometry into the write buffer WITHOUT marking the
            // touched set, so its residue would ghost through the ping-pong — fall back to a
            // full write clear when any subscriber is attached (correct, forgoes the speedup).
            if (BeforePublishCompleteBatch != null || _fullClearPending)
            {
                volume.ClearWriteFull();
                _fullClearPending = false;
            }
            else volume.ClearWriteActiveBlocks();

            // Volume-level uniforms (global scalars, shared by both kernels).
            _depthShader.SetInts("_Dim", volume.Dim.x, volume.Dim.y, volume.Dim.z);
            _depthShader.SetInts("_TBDim", volume.VoxelBlockDim.x, volume.VoxelBlockDim.y, volume.VoxelBlockDim.z);
            _depthShader.SetMatrix("_WorldFromVoxel", volume.WorldFromVoxel);
            _depthShader.SetMatrix("_VoxelFromWorld", volume.VoxelFromWorld);
            _depthShader.SetFloat("_Tau", volume.Tau);
            _depthShader.SetFloat("_WObs", observationWeight);
            _depthShader.SetInt("_LatRadius", latRadius);
            _depthShader.SetFloat("_TubeRadius", Mathf.Max(0.01f, tubeRadiusVoxels) * volume.voxelSize);
            _depthShader.SetInt("_EdgeRejectRadius", edgeRejectRadius);
            _depthShader.SetFloat("_EdgeRejectDepthMm", edgeRejectDepthMm);
            _depthShader.SetInt("_EdgeColorBorrow", edgeColorBorrow ? 1 : 0);

            // Pass A: settle the per-voxel |sdf|-min across every camera of the batch.
            foreach (var s in _batchSerials)
                if (_states.TryGetValue(s, out var st) && st.HasCamParam && st.DepthBuf != null)
                    { EnsureRayLut(st); BindDepthCam(_kScatterMin, st); DispatchDepth(_kScatterMin, st); }

            // Pass B: materialise the winning observation (sdf + colour) into _Voxels.
            foreach (var s in _batchSerials)
                if (_states.TryGetValue(s, out var st) && st.HasCamParam && st.DepthBuf != null)
                    { BindDepthCam(_kScatterWrite, st); DispatchDepth(_kScatterWrite, st); }

            CompletedBatchCount++;
            InvokeBeforePublishCompleteBatch();
            volume.Publish();
            TotalIntegrationCount++;
        }

        // Bind one camera's per-dispatch buffers + matrices for the given kernel.
        private void BindDepthCam(int kernel, CamState st)
        {
            Matrix4x4 depthFromWorld = ComputeDepthFromWorld(st.SourceTransform, st.CamParam);
            var intr = st.CamParam.DepthIntrinsic;
            var cintr = st.CamParam.RgbIntrinsic;
            Matrix4x4 colorFromWorld = Units.MToMmScale
                                       * st.SourceTransform.worldToLocalMatrix;

            _depthShader.SetBuffer(kernel, "_Voxels", volume.WriteBuffer);
            _depthShader.SetBuffer(kernel, "_Colors", volume.WriteColorBuffer);
            _depthShader.SetBuffer(kernel, "_VoxelKey", volume.WriteKeyBuffer);
            _depthShader.SetBuffer(kernel, "_Touched", volume.WriteTouched);
            volume.BindBlockMarking(_depthShader, kernel, volume.WriteBlockActive);

            _depthShader.SetBuffer(kernel, "_Depth", st.DepthBuf);
            _depthShader.SetInt("_DepthW", st.DepthWidth);
            _depthShader.SetInt("_DepthH", st.DepthHeight);
            _depthShader.SetFloat("_FxD", intr.Fx);
            _depthShader.SetFloat("_FyD", intr.Fy);
            _depthShader.SetFloat("_CxD", intr.Cx);
            _depthShader.SetFloat("_CyD", intr.Cy);
            _depthShader.SetMatrix("_DepthFromWorld", depthFromWorld);
            _depthShader.SetMatrix("_WorldFromDepthMm", depthFromWorld.inverse);
            _depthShader.SetBuffer(kernel, "_RayLut", st.RayLutBuf);
            _depthShader.SetInt("_HasRayLut", st.HasRayLut ? 1 : 0);

            // Forward distortion for the projection gate — same gate as the point cloud
            // undistort LUT so unproject/project round-trips to the same pixel.
            var ddist = st.CamParam.DepthDistortion;
            bool hasDepthDist = !forcePinhole && PointCloud.DepthUndistortLut.IsApplicable(ddist);
            _depthShader.SetInt("_HasDepthDist", hasDepthDist ? 1 : 0);
            _depthShader.SetFloat("_DK1", ddist.K1); _depthShader.SetFloat("_DK2", ddist.K2);
            _depthShader.SetFloat("_DK3", ddist.K3); _depthShader.SetFloat("_DK4", ddist.K4);
            _depthShader.SetFloat("_DK5", ddist.K5); _depthShader.SetFloat("_DK6", ddist.K6);
            _depthShader.SetFloat("_DP1", ddist.P1); _depthShader.SetFloat("_DP2", ddist.P2);

            _depthShader.SetBuffer(kernel, "_ColorImg", st.ColorBuf);
            _depthShader.SetInt("_ColorW", st.ColorWidth);
            _depthShader.SetInt("_ColorH", st.ColorHeight);
            _depthShader.SetFloat("_FxC", cintr.Fx);
            _depthShader.SetFloat("_FyC", cintr.Fy);
            _depthShader.SetFloat("_CxC", cintr.Cx);
            _depthShader.SetFloat("_CyC", cintr.Cy);
            _depthShader.SetMatrix("_ColorFromWorld", colorFromWorld);
            _depthShader.SetInt("_HasColor", st.LastHadColor ? 1 : 0);
            _depthShader.SetInt("_UseColorOverride", colorOverride.a > 0f ? 1 : 0);
            _depthShader.SetVector("_OverrideColor", new Vector4(colorOverride.r, colorOverride.g, colorOverride.b, 0f));
        }

        private void DispatchDepth(int kernel, CamState st)
        {
            int gx = Mathf.Max(1, Mathf.CeilToInt(st.DepthWidth / 8f));
            int gy = Mathf.Max(1, Mathf.CeilToInt(st.DepthHeight / 8f));
            _depthShader.Dispatch(kernel, gx, gy, 1);
        }

        // Build / refresh the undistorted-ray LUT for one camera. Rebuilt only when the
        // intrinsics-derived distortion or resolution changes (effectively never). When
        // no correction applies (pinhole / forcePinhole / unsupported model) HasRayLut
        // is false and a 1-element dummy stays bound so the kernel binding is valid.
        private void EnsureRayLut(CamState st)
        {
            var intr = st.CamParam.DepthIntrinsic;
            var dist = st.CamParam.DepthDistortion;
            // Cache key MUST cover everything the LUT and the projection gate depend on:
            // resolution, distortion (coeffs + model), intrinsics, AND forcePinhole. Missing
            // forcePinhole/intrinsics would let a toggle return a stale LUT while BindDepthCam
            // flips _HasDepthDist, so unprojection and the forward-projection gate would use
            // different camera models (the gate then misses valid voxels / stamps a wrong band).
            bool same = st.RayLutBuf != null && st.RayLutW == st.DepthWidth && st.RayLutH == st.DepthHeight
                        && st.RayLutForcePinhole == forcePinhole
                        && st.RayLutDist.Model == dist.Model
                        && DistApproxEqual(st.RayLutDist, dist)
                        && IntrExactEqual(st.RayLutIntr, intr);
            if (same) return;

            st.RayLutW = st.DepthWidth;
            st.RayLutH = st.DepthHeight;
            st.RayLutDist = dist;
            st.RayLutIntr = intr;
            st.RayLutForcePinhole = forcePinhole;

            Vector2[] lut = forcePinhole ? null
                          : PointCloud.DepthUndistortLut.Build(intr, dist, st.DepthWidth, st.DepthHeight);
            if (lut != null && lut.Length > 0)
            {
                GpuBuf.Ensure(ref st.RayLutBuf, lut.Length, sizeof(float) * 2);
                st.RayLutBuf.SetData(lut);
                st.HasRayLut = true;
            }
            else
            {
                if (st.RayLutBuf == null)
                    st.RayLutBuf = new ComputeBuffer(1, sizeof(float) * 2);
                st.HasRayLut = false;
            }
        }

        private static bool DistApproxEqual(in ObCameraDistortion a, in ObCameraDistortion b)
        {
            return a.K1 == b.K1 && a.K2 == b.K2 && a.K3 == b.K3 && a.K4 == b.K4
                && a.K5 == b.K5 && a.K6 == b.K6 && a.P1 == b.P1 && a.P2 == b.P2;
        }

        private static bool IntrExactEqual(in ObCameraIntrinsic a, in ObCameraIntrinsic b)
        {
            return a.Fx == b.Fx && a.Fy == b.Fy && a.Cx == b.Cx && a.Cy == b.Cy;
        }

        /// <summary>
        /// Monotonically increasing counter, +1 per successful integrate. Kept as a
        /// diagnostic / external progress signal; flicker-free view updates are now
        /// driven by <see cref="TSDFVolume.PublishVersion"/>, not this.
        /// </summary>
        public int TotalIntegrationCount { get; private set; }

        /// <summary>
        /// Composes the matrix world(metres) -> depth-camera(millimetres) used
        /// in the Integrate kernel. Coordinate convention notes (see
        /// PointCloudReconstruct.compute + BodyTrackingShared.K4AmmToUnity):
        ///   - Source transform's worldToLocalMatrix maps Unity world metres
        ///     to color-camera metres in the sensor's native Y-down convention,
        ///     because the renderer GO carries localScale = (1, -1, 1).
        ///   - ObCameraParam.Transform is "color = R * depth + T" in mm.
        ///     Inverting it gives depth-mm-from-color-mm.
        /// </summary>
        private static Matrix4x4 ComputeDepthFromWorld(Transform src, in ObCameraParam camParam)
        {
            Matrix4x4 colorMFromWorldM = src.worldToLocalMatrix;
            Matrix4x4 scaleMmFromM = Units.MToMmScale;

            var ext = camParam.Transform;
            Matrix4x4 colorMmFromDepthMm = ext.ToMatrixMm();

            Matrix4x4 depthMmFromColorMm = colorMmFromDepthMm.inverse;
            return depthMmFromColorMm * scaleMmFromM * colorMFromWorldM;
        }

        private void Update()
        {
            // Late-binding: SensorManager may spawn renderers / the
            // recorder may rebuild its _Playback_ GOs mid-Play, so re-scan and
            // bind anything new. Existing entries are short-circuited by the
            // Dictionary.ContainsKey guards.
            BindAllSources();

            // Keep the volume's buffering mode tracking ours if the user toggles
            // clearVolumeOnNewBatch / forceSingleBuffer at runtime (volume rebuilds
            // on the change).
            if (volume != null) volume.doubleBuffered = clearVolumeOnNewBatch && !forceSingleBuffer;

            if (!diagnosticLogging) return;
            float now = Time.realtimeSinceStartup;
            if (_diagWindowStart == 0f) _diagWindowStart = now;
            if (now - _diagWindowStart < 1f) return;
            var sb = new System.Text.StringBuilder("[TSDFIntegrator] /sec ");
            foreach (var kv in _states)
            {
                sb.Append(TruncSerial(kv.Key)).Append('=').Append(kv.Value.FramesIntegrated).Append(' ');
                kv.Value.FramesIntegrated = 0;
            }
            Debug.Log(sb.ToString(), this);
            _diagWindowStart = now;
        }

        private static string TruncSerial(string s)
        {
            if (string.IsNullOrEmpty(s) || s.Length <= 6) return s;
            return s.Substring(s.Length - 6);
        }
    }
}
