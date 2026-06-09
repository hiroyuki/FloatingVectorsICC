// Drives the per-camera TSDF integration kernel. Subscribes to both the live
// (PointCloudRenderer.OnRawFramesReady) and playback (PointCloudRecorder
// .OnPlaybackRawFrame) raw-frame events so the same component works in both
// modes without scene re-wiring.
//
// Step 1b of FloatingVectorsICC_v2_TSDF_spec.md §9. The TSDF volume is owned
// by TSDFVolume; we only push observations into it.

using System.Collections.Generic;
using System.Runtime.InteropServices;
using Orbbec;
using PointCloud;
using UnityEngine;

namespace TSDF
{
    /// <summary>
    /// Subscribes to the raw-frame pipeline, uploads per-camera depth bytes
    /// to GPU, and dispatches the Integrate kernel once per cam per frame.
    /// </summary>
    [DefaultExecutionOrder(0)]
    public sealed class TSDFIntegrator : MonoBehaviour
    {
        [Tooltip("Volume to integrate into. Auto-located on enable if left null.")]
        public TSDFVolume volume;

        [Tooltip("Base observation weight per frame per camera. Spec §3.1: " +
                 "scaled by view-angle / range later — start at 1.0.")]
        [Min(0f)] public float observationWeight = 1f;

        [Tooltip("Dev colour override: when alpha > 0, bake this flat colour into " +
                 "integrated voxels instead of the camera RGB (debug tooling uses it " +
                 "to tag frames/instants — e.g. red vs blue). Alpha == 0 (default) = " +
                 "use camera colour. Callers must reset it after use.")]
        public Color colorOverride = new Color(0f, 0f, 0f, 0f);

        [Tooltip("Subscribe to live PointCloudRenderer.OnRawFramesReady too. " +
                 "Leave on so live capture also feeds the TSDF when a Femto Bolt " +
                 "is attached.")]
        public bool subscribeLive = true;

        [Tooltip("Subscribe to every PointCloudRecorder.OnPlaybackRawFrame in " +
                 "the scene. On Mac dev without live cameras this is the only " +
                 "feed.")]
        public bool subscribePlayback = true;

        [Tooltip("Live-follow mode: wipe the TSDF volume once every batch of " +
                 "<expectedCamCount> unique cameras has integrated, then start " +
                 "accumulating the next batch. The mesh therefore reflects only " +
                 "the most recent complete multi-cam observation and tracks the " +
                 "current point cloud over time.")]
        public bool clearVolumeOnNewBatch = true;

        [Tooltip("Number of unique serials per batch. Default 4 = one Femto Bolt " +
                 "set; lower it if you are debugging with fewer cams attached.")]
        [Min(1)] public int expectedCamCount = 4;

        [Tooltip("Gate for the live/playback integration path. When false, incoming " +
                 "frames are ignored so the volume (and thus the mesh) freezes on its " +
                 "last state. TSDFCaptureSession flips this to hold the final mesh after " +
                 "a capture window ends (spec §7). Debug replay via IntegrateRawFrame " +
                 "bypasses this gate.")]
        public bool integrationEnabled = true;

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

        [Tooltip("Log a per-second summary of integrated frames per serial.")]
        public bool diagnosticLogging = false;

        private ComputeShader _shader;
        private int _kernel;

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
        }
        private readonly Dictionary<string, CamState> _states = new Dictionary<string, CamState>();

        // Tracked subscriptions so we can detach cleanly. Keyed by component
        // so the same renderer reappearing after a Stop+Play binds once.
        private readonly Dictionary<PointCloudRenderer,
                                   System.Action<PointCloudRenderer, RawFrameData>> _liveHandlers
            = new Dictionary<PointCloudRenderer,
                              System.Action<PointCloudRenderer, RawFrameData>>();
        private readonly Dictionary<PointCloudRecorder,
                                   System.Action<string, ObCameraParam?, Transform, RawFrameData>> _playbackHandlers
            = new Dictionary<PointCloudRecorder,
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
            // to match our mode so the two stay consistent.
            volume.doubleBuffered = clearVolumeOnNewBatch;
            BindAllSources();
        }

        private void OnDisable()
        {
            UnbindAll();
            foreach (var kv in _states)
            {
                kv.Value.DepthBuf?.Release();
                kv.Value.ColorBuf?.Release();
            }
            _states.Clear();
        }

        private void EnsureShader()
        {
            if (_shader != null) return;
            _shader = Resources.Load<ComputeShader>("TSDFIntegrate");
            if (_shader == null)
            {
                Debug.LogError("[TSDFIntegrator] Compute shader \"Resources/TSDFIntegrate.compute\" not found.", this);
                enabled = false;
                return;
            }
            _kernel = _shader.FindKernel("Integrate");
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
                foreach (var rec in FindObjectsByType<PointCloudRecorder>(FindObjectsInactive.Exclude, FindObjectsSortMode.None))
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

        private void BindPlayback(PointCloudRecorder rec)
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
            => IntegrateOne(serial, camParam, sourceTransform, raw);

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

            // Live-follow: when a complete batch finished last time, wipe the
            // hidden BACK buffer here at the START of the next batch's first
            // integration. The displayed front keeps showing the last complete
            // batch until this one finishes and publishes — no flicker.
            if (clearVolumeOnNewBatch && _batchSerials.Count >= expectedCamCount)
            {
                volume.ClearWrite();
                _batchSerials.Clear();
            }

            if (!IntegrateOne(serial, camParam, sourceTransform, raw)) return;

            if (clearVolumeOnNewBatch)
            {
                // Tally this serial into the in-flight batch. Once full, bump the
                // batch counter and publish the back buffer to the front so all
                // three views pick up the same complete snapshot this frame.
                _batchSerials.Add(serial);
                if (_batchSerials.Count == expectedCamCount)
                {
                    CompletedBatchCount++;
                    volume.Publish();
                }
            }
            else
            {
                // Accumulate (single buffer): content is already in the visible
                // buffer; just bump the publish version so the mesh re-extracts.
                volume.Publish();
            }
        }

        /// <summary>
        /// Integrate ONE depth frame into <see cref="TSDFVolume.WriteBuffer"/>. No
        /// batch / clear / publish side effects — pure GPU integrate + counters.
        /// Returns false if the frame was rejected (missing intrinsics, no depth…).
        /// </summary>
        private unsafe bool IntegrateOne(string serial, ObCameraParam? camParam,
                                         Transform sourceTransform, RawFrameData raw)
        {
            if (string.IsNullOrEmpty(serial)) return false;
            if (volume == null || volume.WriteBuffer == null) return false;
            if (raw.DepthBytes == null || raw.DepthByteCount <= 0) return false;
            if (!camParam.HasValue) return false; // need intrinsics
            if (sourceTransform == null) return false;

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
            if (st.DepthBuf == null || st.DepthBuf.count != uintCount)
            {
                st.DepthBuf?.Release();
                st.DepthBuf = new ComputeBuffer(uintCount, 4, ComputeBufferType.Raw);
            }
            if (st.DepthScratchU32 == null || st.DepthScratchU32.Length != uintCount)
                st.DepthScratchU32 = new uint[uintCount];
            System.Buffer.BlockCopy(raw.DepthBytes, 0, st.DepthScratchU32, 0, raw.DepthByteCount);
            st.DepthBuf.SetData(st.DepthScratchU32, 0, 0, uintCount);

            // Upload the colour frame (RGB8) the same way, so the integrate kernel
            // can sample the camera colour per voxel. When the frame has no colour
            // (depth-only), keep a 1-uint dummy bound and flag _HasColor=0 — the
            // kernel's binding must still exist even though it short-circuits.
            bool hasColor = raw.ColorBytes != null && raw.ColorByteCount > 0
                            && raw.ColorWidth > 0 && raw.ColorHeight > 0;
            if (hasColor)
            {
                st.ColorByteCount = raw.ColorByteCount;
                st.ColorWidth = raw.ColorWidth;
                st.ColorHeight = raw.ColorHeight;
                int cUintCount = (raw.ColorByteCount + 3) / 4;
                if (st.ColorBuf == null || st.ColorBuf.count != cUintCount)
                {
                    st.ColorBuf?.Release();
                    st.ColorBuf = new ComputeBuffer(cUintCount, 4, ComputeBufferType.Raw);
                }
                if (st.ColorScratchU32 == null || st.ColorScratchU32.Length != cUintCount)
                    st.ColorScratchU32 = new uint[cUintCount];
                System.Buffer.BlockCopy(raw.ColorBytes, 0, st.ColorScratchU32, 0, raw.ColorByteCount);
                st.ColorBuf.SetData(st.ColorScratchU32, 0, 0, cUintCount);
            }
            else if (st.ColorBuf == null)
            {
                st.ColorBuf = new ComputeBuffer(1, 4, ComputeBufferType.Raw);
            }

            // Build per-camera world->depth-mm matrix.
            Matrix4x4 depthFromWorld = ComputeDepthFromWorld(sourceTransform, st.CamParam);
            var intr = st.CamParam.DepthIntrinsic;

            _shader.SetBuffer(_kernel, "_Voxels", volume.WriteBuffer);
            _shader.SetInts("_Dim", volume.Dim.x, volume.Dim.y, volume.Dim.z);
            _shader.SetMatrix("_WorldFromVoxel", volume.WorldFromVoxel);
            _shader.SetFloat("_Tau", volume.Tau);
            _shader.SetInt("_AccumulationMode", (int)volume.accumulationMode);

            _shader.SetBuffer(_kernel, "_Depth", st.DepthBuf);
            _shader.SetInt("_DepthW", raw.DepthWidth);
            _shader.SetInt("_DepthH", raw.DepthHeight);
            _shader.SetFloat("_FxD", intr.Fx);
            _shader.SetFloat("_FyD", intr.Fy);
            _shader.SetFloat("_CxD", intr.Cx);
            _shader.SetFloat("_CyD", intr.Cy);
            _shader.SetMatrix("_DepthFromWorld", depthFromWorld);
            _shader.SetFloat("_WObs", observationWeight);

            // Colour buffer + colour-camera projection. world->colour-mm is the
            // renderer-transform half of depthFromWorld (the source GO transform
            // is already in colour-camera metres), scaled to mm — without the
            // colour->depth extrinsic inverse that depthFromWorld applies.
            Matrix4x4 colorFromWorld = Matrix4x4.Scale(new Vector3(1000f, 1000f, 1000f))
                                       * sourceTransform.worldToLocalMatrix;
            var cintr = st.CamParam.RgbIntrinsic;
            _shader.SetBuffer(_kernel, "_Colors", volume.WriteColorBuffer);
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

            int total = volume.Dim.x * volume.Dim.y * volume.Dim.z;
            int groups = Mathf.CeilToInt(total / 64f);
            // 2D group grid so fine volumes (voxelSize 0.01 → >100k groups) stay
            // under the 65535 threadgroup-per-axis limit. The kernel linearises
            // via _DispatchWidth = groupsX * 64.
            int gx = Mathf.Min(groups, 65535);
            int gy = Mathf.CeilToInt(groups / (float)gx);
            _shader.SetInt("_DispatchWidth", gx * 64);
            _shader.Dispatch(_kernel, Mathf.Max(1, gx), Mathf.Max(1, gy), 1);

            st.FramesIntegrated++;
            TotalIntegrationCount++;
            return true;
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
            Matrix4x4 scaleMmFromM = Matrix4x4.Scale(new Vector3(1000f, 1000f, 1000f));

            var ext = camParam.Transform;
            Matrix4x4 colorMmFromDepthMm = Matrix4x4.identity;
            colorMmFromDepthMm.m00 = ext.Rot[0]; colorMmFromDepthMm.m01 = ext.Rot[1]; colorMmFromDepthMm.m02 = ext.Rot[2]; colorMmFromDepthMm.m03 = ext.Trans[0];
            colorMmFromDepthMm.m10 = ext.Rot[3]; colorMmFromDepthMm.m11 = ext.Rot[4]; colorMmFromDepthMm.m12 = ext.Rot[5]; colorMmFromDepthMm.m13 = ext.Trans[1];
            colorMmFromDepthMm.m20 = ext.Rot[6]; colorMmFromDepthMm.m21 = ext.Rot[7]; colorMmFromDepthMm.m22 = ext.Rot[8]; colorMmFromDepthMm.m23 = ext.Trans[2];

            Matrix4x4 depthMmFromColorMm = colorMmFromDepthMm.inverse;
            return depthMmFromColorMm * scaleMmFromM * colorMFromWorldM;
        }

        private void Update()
        {
            // Late-binding: PointCloudCameraManager may spawn renderers / the
            // recorder may rebuild its _Playback_ GOs mid-Play, so re-scan and
            // bind anything new. Existing entries are short-circuited by the
            // Dictionary.ContainsKey guards.
            BindAllSources();

            // Keep the volume's buffering mode tracking ours if the user toggles
            // clearVolumeOnNewBatch at runtime (volume rebuilds on the change).
            if (volume != null) volume.doubleBuffered = clearVolumeOnNewBatch;

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
