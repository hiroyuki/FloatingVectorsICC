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

        // Set of serials that have integrated since the last clear. Once it
        // reaches expectedCamCount, the next IntegrateRawFrame call wipes the
        // volume and resets the set — i.e. the just-finished batch is the one
        // MC sees (TSDFView only re-dispatches when CompletedBatchCount moves).
        private readonly System.Collections.Generic.HashSet<string> _batchSerials
            = new System.Collections.Generic.HashSet<string>();

        /// <summary>
        /// Monotonically increasing per-batch counter. Bumped each time a batch
        /// reaches <see cref="expectedCamCount"/> unique serials. TSDFView reads
        /// this to decide when to re-run Marching Cubes — the mesh therefore
        /// only updates on batch completion, never on partial batches.
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
            BindAllSources();
        }

        private void OnDisable()
        {
            UnbindAll();
            foreach (var kv in _states)
                kv.Value.DepthBuf?.Release();
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
            => DispatchIntegrate(serial, camParam, sourceTransform, raw);

        private unsafe void DispatchIntegrate(string serial, ObCameraParam? camParam,
                                              Transform sourceTransform, RawFrameData raw)
        {
            if (string.IsNullOrEmpty(serial)) return;
            if (volume == null || volume.VoxelBuffer == null) return;
            if (raw.DepthBytes == null || raw.DepthByteCount <= 0) return;
            if (!camParam.HasValue) return; // need intrinsics
            if (sourceTransform == null) return;

            // Live-follow: when a complete batch finished last time, wipe the
            // volume here at the START of the next batch's first integration.
            // TSDFView only re-runs MC on CompletedBatchCount changes, so the
            // mesh on screen stays bound to the LAST complete batch all the way
            // until the new batch finishes — no flicker on partial batches.
            if (clearVolumeOnNewBatch && _batchSerials.Count >= expectedCamCount)
            {
                volume.Clear();
                _batchSerials.Clear();
            }

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

            // Build per-camera world->depth-mm matrix.
            Matrix4x4 depthFromWorld = ComputeDepthFromWorld(sourceTransform, st.CamParam);
            var intr = st.CamParam.DepthIntrinsic;

            _shader.SetBuffer(_kernel, "_Voxels", volume.VoxelBuffer);
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

            int total = volume.Dim.x * volume.Dim.y * volume.Dim.z;
            int groups = Mathf.CeilToInt(total / 64f);
            _shader.Dispatch(_kernel, groups, 1, 1);

            st.FramesIntegrated++;
            TotalIntegrationCount++;

            // Tally this serial into the in-flight batch. The check is below
            // (not above) the dispatch so this frame's contribution lands in
            // the volume BEFORE we declare the batch complete.
            if (clearVolumeOnNewBatch)
            {
                _batchSerials.Add(serial);
                if (_batchSerials.Count == expectedCamCount)
                    CompletedBatchCount++;
            }
        }

        /// <summary>
        /// Monotonically increasing counter, +1 per successful IntegrateRawFrame.
        /// TSDFView reads this to decide "did the integrator do work this Unity
        /// frame?" — if yes, re-run MC + clear in LateUpdate; if no, hold the
        /// previous mesh on screen so it does not flicker on Unity frames where
        /// the recorder did not emit (Unity 60+ fps vs recorder 30 fps).
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
