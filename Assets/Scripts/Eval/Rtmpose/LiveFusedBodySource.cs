// Live RTMPose fusion source: runs the FusedRtmposeAdapter (YOLOX detect ->
// RTMPose 2D -> depth lift -> 3-stage cluster fusion, v11) on a BACKGROUND
// thread over live camera frames and feeds the fused skeleton into the
// production SkeletonMerger through SubmitExternalBodies — the k4abt-free
// body-tracking path.
//
// Threading: Unity main thread only copies each arriving frame into a
// per-serial pending slot (latest wins) and drains fused results back into
// the merger; ALL inference and fusion runs on one worker thread. The whole
// adapter chain (ORT + pure C# math) has no Unity-API dependency, verified —
// and single-threaded use inside the worker keeps the adapter's state safe.
// The GPU sets the achievable fusion rate (~10 Hz on DirectML with 4 cams;
// CUDA/TensorRT EP is the path to 30 Hz); the adapter's temporal hold +
// heartbeat keep the output cadence steady in between.
//
// Frame sources:
//  - LIVE: every PointCloudRenderer.OnRawFramesReady in the scene.
//  - usePlaybackFrames: SensorRecorder.OnPlaybackRawFrame too — the dev
//    "live-sim" mode: recorded takes drive the exact live code path on
//    machines without cameras.
//
// The fused skeleton (origin-camera world frame) is converted into each
// camera's DEPTH frame and injected per serial — the merger then sees four
// coincident candidates and merges them into the identical pose, the same
// session shape as the exported v11s A/B roots (see FusedBodiesExport).

using System;
using System.Collections.Generic;
using System.IO;
using System.Threading;
using Orbbec;
using PointCloud;
using UnityEngine;

namespace BodyTracking.Eval.Rtmpose
{
    [DisallowMultipleComponent]
    public class LiveFusedBodySource : MonoBehaviour
    {
        [Tooltip("Merger that receives the fused skeleton. Auto-resolves when empty. " +
                 "While this component is enabled the merger runs in useExternalBodies " +
                 "mode (no k4abt workers, recorded bodies_main ignored).")]
        public SkeletonMerger merger;

        [Tooltip("Also consume SensorRecorder playback frames (dev live-sim on " +
                 "machines without cameras). Live renderer frames are always consumed.")]
        public bool usePlaybackFrames = true;

        [Tooltip("Folder with yolox-m/ and rtmpose-m/ ONNX models, relative to the " +
                 "project root.")]
        public string modelsDir = "eval/models";

        [Tooltip("Bone-length profile JSON (BodyProfileBuilder output), relative to " +
                 "the project root. Empty = fusion runs without bone-length priors.")]
        public string bodyProfilePath = "eval/body_profile.json";

        [Tooltip("RTMPose keypoint confidence threshold (person selection & joints).")]
        public float confThreshold = 0.3f;

        [Tooltip("Capture volume center (mm, origin-camera world frame) handed to " +
                 "the person selector — same convention as FusedBodiesExport.")]
        public Vector3 captureVolumeCenterMm = new Vector3(0, 200, 3000);
        [Tooltip("Capture volume half extents (mm).")]
        public Vector3 captureVolumeHalfMm = new Vector3(1100, 1500, 1100);

        [Tooltip("Emit a heartbeat (temporal-hold frame) when no camera frame arrived " +
                 "for this many ms — covers sensor hiccups so curves never bridge a " +
                 "hole with a straight kink.")]
        public int heartbeatMs = 40;

        /// <summary>Fused frames emitted per second (diagnostic, worker-thread rate).</summary>
        public float FusedHz { get; private set; }

        // ---- per-serial frame slot (main thread writes, worker consumes) ----
        // Double-buffered: Ingest writes the PENDING arrays under _slotLock; the
        // worker SWAPS them with its processing arrays (still under the lock)
        // and reads the swapped-out set outside the lock — the worker always
        // owns immutable frame bytes, a later Ingest writes the other set.
        private sealed class Slot
        {
            public string Serial;
            // pending (main-thread writes)
            public byte[] Depth = Array.Empty<byte>(), Color = Array.Empty<byte>(), IR = Array.Empty<byte>();
            public int DepthCount, ColorCount, IRCount;
            public int DW, DH, CW, CH, IW, IH;
            public ulong TsNs;
            public bool Fresh;
            // processing (worker-owned after the swap)
            public byte[] ProcDepth = Array.Empty<byte>(), ProcColor = Array.Empty<byte>(), ProcIR = Array.Empty<byte>();
            public int ProcDepthCount, ProcColorCount, ProcIRCount;
            public int PDW, PDH, PCW, PCH, PIW, PIH;
            public ulong ProcTsNs;
            public ObCameraParam? CamParam;      // from calibration (world-injection side)
            public Transform SourceTransform;    // renderer / playback GO
            public bool Configured;              // adapter.Configure done (worker thread)
        }

        private readonly Dictionary<string, Slot> _slots = new Dictionary<string, Slot>();
        private readonly object _slotLock = new object();
        private readonly AutoResetEvent _wake = new AutoResetEvent(false);

        // ---- fused output queue (worker produces, main thread injects) ----
        private sealed class FusedResult
        {
            public ulong TsNs;
            public readonly Dictionary<string, byte[]> BytesBySerial = new Dictionary<string, byte[]>();
            public int ByteCount;
        }
        private readonly Queue<FusedResult> _results = new Queue<FusedResult>();
        private readonly object _resultLock = new object();

        private OrtRtmposeBackend _backend;
        private FusedRtmposeAdapter _fused;
        private Thread _worker;
        private volatile bool _stop;
        private bool _mergerFlagOwned;

        // world -> per-camera depth conversion (worker thread reads only)
        private sealed class CamXform { public ObExtrinsic D2C; public ObExtrinsic G; }
        private readonly Dictionary<string, CamXform> _xform = new Dictionary<string, CamXform>();

        private readonly List<PointCloudRenderer> _subscribedRenderers = new List<PointCloudRenderer>();
        private SensorRecorder _recorder;
        private BodySnapshot _snap;
        private byte[] _encodeScratch;

        // Playback loop = timestamps jump BACKWARD, and the adapter is strictly
        // monotonic (TryFuse rejects nowTs <= last) — without a reset, fusion is
        // dead for good after the first loop. Live capture never loops; this is
        // the dev live-sim plumbing. Rebuild happens on the worker thread.
        private volatile bool _resetPending;
        private int _fusedEmitted; // OnFusedSkeletons count (worker thread, Interlocked)

        private void OnEnable()
        {
            if (merger == null) merger = FindFirstObjectByType<SkeletonMerger>();
            if (merger != null && !merger.useExternalBodies)
            {
                merger.useExternalBodies = true;
                _mergerFlagOwned = true;
            }

            try
            {
                string root = Directory.GetParent(Application.dataPath).FullName;
                string models = Path.Combine(root, modelsDir);
                string yolox = FirstOnnx(Path.Combine(models, "yolox-m"));
                string rtm = FirstOnnx(Path.Combine(models, "rtmpose-m"));
                if (yolox == null || rtm == null)
                {
                    Debug.LogError($"[{nameof(LiveFusedBodySource)}] ONNX models not found under {models} — disabling.", this);
                    enabled = false;
                    return;
                }
                _backend = new OrtRtmposeBackend(yolox, rtm);
                _fused = new FusedRtmposeAdapter(_backend) { ConfThreshold = confThreshold };
                string profile = Path.Combine(root, bodyProfilePath);
                if (!string.IsNullOrEmpty(bodyProfilePath) && File.Exists(profile))
                    _fused.Profile = BodyProfile.Load(profile);
                _fused.SetCaptureVolume(captureVolumeCenterMm, captureVolumeHalfMm);
                _fused.OnSkeletons += OnFusedSkeletons; // worker thread!
            }
            catch (Exception e)
            {
                Debug.LogException(e, this);
                enabled = false;
                return;
            }

            _snap = new BodySnapshot { Id = 1 };
            _encodeScratch = new byte[RecordedBodySerializer.FrameSize(1)];

            if (!LoadCalibration())
            {
                // no rig geometry -> no world transforms, no injection targets;
                // tear the half-built state down and stay disabled
                _fused.OnSkeletons -= OnFusedSkeletons;
                _fused = null;
                _backend?.Dispose();
                _backend = null;
                enabled = false;
                return;
            }
            SubscribeSources();

            _stop = false;
            _worker = new Thread(WorkerLoop) { IsBackground = true, Name = "LiveFusedBodySource" };
            _worker.Start();
        }

        private void OnDisable()
        {
            _stop = true;
            _wake.Set();
            bool workerExited = _worker == null || _worker.Join(5000);
            _worker = null;

            foreach (var r in _subscribedRenderers)
                if (r != null) r.OnRawFramesReady -= OnLiveFrame;
            _subscribedRenderers.Clear();
            if (_recorder != null)
            {
                _recorder.OnPlaybackRawFrame -= OnPlaybackFrame;
                _recorder.OnPlaybackLooped -= HandlePlaybackLooped;
                _recorder = null;
            }

            if (workerExited)
            {
                if (_fused != null) { _fused.OnSkeletons -= OnFusedSkeletons; _fused = null; }
                _backend?.Dispose();
                _backend = null;
            }
            else
            {
                // The worker may still be inside ORT inference — disposing the
                // native sessions under it would crash the editor. Leak the
                // backend deliberately (the thread is background; process
                // teardown reclaims it) and make the failure loud.
                Debug.LogError($"[{nameof(LiveFusedBodySource)}] worker did not stop within 5s — leaking the ORT backend instead of disposing it under a running inference", this);
                _fused = null;
                _backend = null;
            }

            if (merger != null && _mergerFlagOwned) merger.useExternalBodies = false;
            _mergerFlagOwned = false;

            lock (_slotLock) _slots.Clear();
            lock (_resultLock) _results.Clear();
            _xform.Clear();
        }

        private void Update()
        {
            // late-joining renderers (live cameras open asynchronously)
            SubscribeSources();

            // drain fused results into the merger (main thread — merger touches
            // Unity transforms)
            while (true)
            {
                FusedResult res;
                lock (_resultLock)
                {
                    if (_results.Count == 0) break;
                    res = _results.Dequeue();
                }
                if (merger == null) continue;
                foreach (var kv in res.BytesBySerial)
                {
                    Slot slot;
                    lock (_slotLock) _slots.TryGetValue(kv.Key, out slot);
                    if (slot == null) continue;
                    merger.SubmitExternalBodies(kv.Key, res.TsNs, kv.Value, res.ByteCount,
                        slot.SourceTransform, slot.CamParam);
                }
            }
        }

        // ---------------- frame intake (main thread) ----------------

        private void SubscribeSources()
        {
            foreach (var r in UnityEngine.Object.FindObjectsByType<PointCloudRenderer>(FindObjectsSortMode.None))
            {
                if (_subscribedRenderers.Contains(r)) continue;
                r.OnRawFramesReady += OnLiveFrame;
                _subscribedRenderers.Add(r);
            }
            if (usePlaybackFrames && _recorder == null)
            {
                _recorder = FindFirstObjectByType<SensorRecorder>();
                if (_recorder != null)
                {
                    _recorder.OnPlaybackRawFrame += OnPlaybackFrame;
                    _recorder.OnPlaybackLooped += HandlePlaybackLooped;
                }
            }
        }

        private void HandlePlaybackLooped()
        {
            _resetPending = true;
            _wake.Set();
        }

        private void OnLiveFrame(PointCloudRenderer r, RawFrameData frame)
            => Ingest(r.deviceSerial, r.CameraParam, r.transform, frame);

        private void OnPlaybackFrame(string serial, ObCameraParam? camParam, Transform tr, RawFrameData frame)
            => Ingest(serial, camParam, tr, frame);

        private void Ingest(string serial, ObCameraParam? camParam, Transform tr, in RawFrameData f)
        {
            if (string.IsNullOrEmpty(serial) || f.DepthBytes == null || f.ColorBytes == null) return;
            lock (_slotLock)
            {
                if (!_slots.TryGetValue(serial, out var s))
                {
                    s = new Slot { Serial = serial };
                    _slots[serial] = s;
                }
                s.CamParam = camParam ?? s.CamParam;
                s.SourceTransform = tr != null ? tr : s.SourceTransform;
                CopyInto(ref s.Depth, f.DepthBytes, f.DepthByteCount); s.DepthCount = f.DepthByteCount;
                CopyInto(ref s.Color, f.ColorBytes, f.ColorByteCount); s.ColorCount = f.ColorByteCount;
                CopyInto(ref s.IR, f.IRBytes, f.IRByteCount); s.IRCount = f.IRByteCount;
                s.DW = f.DepthWidth; s.DH = f.DepthHeight;
                s.CW = f.ColorWidth; s.CH = f.ColorHeight;
                s.IW = f.IRWidth; s.IH = f.IRHeight;
                s.TsNs = f.TimestampUs * 1000UL;
                s.Fresh = true;
            }
            _wake.Set();
        }

        private static void CopyInto(ref byte[] dst, byte[] src, int count)
        {
            if (src == null || count <= 0) { return; }
            if (dst.Length < count) dst = new byte[count];
            Buffer.BlockCopy(src, 0, dst, 0, count);
        }

        // ---------------- worker thread ----------------

        private void WorkerLoop()
        {
            var burst = new List<Slot>(8);
            long lastFrameWallMs = Environment.TickCount;
            ulong lastTsNs = 0;
            long rateWindowStart = Environment.TickCount;

            while (!_stop)
            {
                _wake.WaitOne(10);
                if (_stop) break;

                if (_resetPending)
                {
                    _resetPending = false;
                    RebuildAdapter();
                    lastTsNs = 0;
                }

                burst.Clear();
                lock (_slotLock)
                {
                    foreach (var kv in _slots)
                    {
                        var s = kv.Value;
                        if (!s.Fresh) continue;
                        s.Fresh = false;
                        // swap pending <-> processing so the worker owns the bytes
                        (s.Depth, s.ProcDepth) = (s.ProcDepth, s.Depth);
                        (s.Color, s.ProcColor) = (s.ProcColor, s.Color);
                        (s.IR, s.ProcIR) = (s.ProcIR, s.IR);
                        s.ProcDepthCount = s.DepthCount; s.ProcColorCount = s.ColorCount; s.ProcIRCount = s.IRCount;
                        s.PDW = s.DW; s.PDH = s.DH; s.PCW = s.CW; s.PCH = s.CH; s.PIW = s.IW; s.PIH = s.IH;
                        s.ProcTsNs = s.TsNs;
                        burst.Add(s);
                    }
                }

                if (burst.Count == 0)
                {
                    // sensor hiccup / GPU starved: keep the output cadence steady
                    long idle = Environment.TickCount - lastFrameWallMs;
                    if (lastTsNs != 0 && idle > heartbeatMs)
                    {
                        lastFrameWallMs = Environment.TickCount;
                        try { _fused.Heartbeat(lastTsNs + (ulong)idle * 1_000_000UL); }
                        catch (Exception e) { Debug.LogException(e, this); }
                    }
                    continue;
                }

                foreach (var s in burst)
                {
                    if (_stop) break;
                    try
                    {
                        EnsureConfigured(s);
                        var raw = new RawFrameData(
                            s.ProcDepth, s.ProcDepthCount, s.PDW, s.PDH,
                            s.ProcColor, s.ProcColorCount, s.PCW, s.PCH,
                            s.ProcIR, s.ProcIRCount, s.PIW, s.PIH,
                            s.ProcTsNs / 1000UL);
                        _fused.SubmitFrame(s.Serial, raw, s.ProcTsNs);
                        lastTsNs = s.ProcTsNs > lastTsNs ? s.ProcTsNs : lastTsNs;
                        lastFrameWallMs = Environment.TickCount;
                    }
                    catch (Exception e) { Debug.LogException(e, this); }
                }

                long win = Environment.TickCount - rateWindowStart;
                if (win >= 2000)
                {
                    // actual fused OUTPUT rate (OnFusedSkeletons emissions)
                    FusedHz = Interlocked.Exchange(ref _fusedEmitted, 0) * 1000f / win;
                    rateWindowStart = Environment.TickCount;
                }
            }
            try { _fused?.FlushLag(); } catch { }
        }

        // Worker thread. Fresh adapter after a playback loop — the shared
        // backend (ORT sessions) survives, only the fusion state is rebuilt.
        private void RebuildAdapter()
        {
            try
            {
                if (_fused != null) _fused.OnSkeletons -= OnFusedSkeletons;
                var next = new FusedRtmposeAdapter(_backend) { ConfThreshold = confThreshold };
                if (_fused != null) next.Profile = _fused.Profile;
                next.SetCaptureVolume(captureVolumeCenterMm, captureVolumeHalfMm);
                foreach (var kv in _xform) next.SetWorldTransform(kv.Key, kv.Value.G);
                next.OnSkeletons += OnFusedSkeletons;
                _fused = next;
                lock (_slotLock)
                    foreach (var kv in _slots) { kv.Value.Configured = false; kv.Value.Fresh = false; }
            }
            catch (Exception e) { Debug.LogException(e, this); }
        }

        // adapter.Configure needs stream dims — known on first frame. Worker
        // thread only, before the serial's first SubmitFrame.
        private void EnsureConfigured(Slot s)
        {
            if (s.Configured) return;
            if (!s.CamParam.HasValue)
            {
                // playback path always has calibration-built params; live fills in
                // once the renderer publishes them — until then skip the frame.
                throw new InvalidOperationException($"no CameraParam yet for {s.Serial}");
            }
            var ctx = new EvalCameraContext(s.Serial, s.PDW, s.PDH, s.PCW, s.PCH, s.CamParam);
            _fused.Configure(ctx);
            if (_xform.TryGetValue(s.Serial, out var x))
                _fused.SetWorldTransform(s.Serial, x.G);
            s.Configured = true;
        }

        // ---------------- fused output (worker thread) ----------------

        private void OnFusedSkeletons(EvalSkeletonFrame f)
        {
            Interlocked.Increment(ref _fusedEmitted);
            var p = f.Primary();
            if (p == null) return;

            var res = new FusedResult { TsNs = f.TimestampNs };
            foreach (var kv in _xform)
            {
                BuildSnapshot(p, kv.Value);
                int bytes = RecordedBodySerializer.Encode(new[] { _snap }, 1, _encodeScratch);
                var copy = new byte[bytes];
                Buffer.BlockCopy(_encodeScratch, 0, copy, 0, bytes);
                res.BytesBySerial[kv.Key] = copy;
                res.ByteCount = bytes;
            }
            if (res.BytesBySerial.Count == 0) return;
            lock (_resultLock)
            {
                _results.Enqueue(res);
                // backpressure: never let the queue grow unbounded if the main
                // thread stalls — drop the oldest
                while (_results.Count > 8) _results.Dequeue();
            }
        }

        // Same skeleton conversion as FusedBodiesExport (kept in sync by hand —
        // the export lives in an Editor assembly this runtime component can't
        // reference): world -> camera color -> camera depth, EvalSkeleton joints
        // mapped to k4abt ids + derived spine/clavicle/nose fills.
        private void BuildSnapshot(EvalSkeleton p, CamXform x)
        {
            for (int i = 0; i < K4ABTConsts.K4ABT_JOINT_COUNT; i++)
                _snap.Joints[i] = new k4abt_joint_t
                {
                    Position = new k4a_float3_t(),
                    Orientation = new k4a_quaternion_t { W = 1f },
                    ConfidenceLevel = k4abt_joint_confidence_level_t.K4ABT_JOINT_CONFIDENCE_NONE,
                };

            for (int j = 0; j < EvalSkeleton.JointCount; j++)
            {
                if (!p.Joints[j].Valid) continue;
                Vector3 d = ToDepth(p.Joints[j].PositionMm, x);
                SetJoint(EvalSkeletonMap.K4abtSource[j], d, k4abt_joint_confidence_level_t.K4ABT_JOINT_CONFIDENCE_MEDIUM);
            }

            bool hasPelvis = p.Joints[(int)EvalJointId.Pelvis].Valid;
            bool hasNeck = p.Joints[(int)EvalJointId.Neck].Valid;
            if (hasPelvis && hasNeck)
            {
                Vector3 pel = ToDepth(p.Joints[(int)EvalJointId.Pelvis].PositionMm, x);
                Vector3 nk = ToDepth(p.Joints[(int)EvalJointId.Neck].PositionMm, x);
                SetJoint(k4abt_joint_id_t.K4ABT_JOINT_SPINE_NAVEL, Vector3.Lerp(pel, nk, 1f / 3f), k4abt_joint_confidence_level_t.K4ABT_JOINT_CONFIDENCE_LOW);
                SetJoint(k4abt_joint_id_t.K4ABT_JOINT_SPINE_CHEST, Vector3.Lerp(pel, nk, 2f / 3f), k4abt_joint_confidence_level_t.K4ABT_JOINT_CONFIDENCE_LOW);
                SetJoint(k4abt_joint_id_t.K4ABT_JOINT_CLAVICLE_LEFT, nk, k4abt_joint_confidence_level_t.K4ABT_JOINT_CONFIDENCE_LOW);
                SetJoint(k4abt_joint_id_t.K4ABT_JOINT_CLAVICLE_RIGHT, nk, k4abt_joint_confidence_level_t.K4ABT_JOINT_CONFIDENCE_LOW);
            }
            if (p.Joints[(int)EvalJointId.Head].Valid)
            {
                Vector3 hd = ToDepth(p.Joints[(int)EvalJointId.Head].PositionMm, x);
                SetJoint(k4abt_joint_id_t.K4ABT_JOINT_NOSE, hd, k4abt_joint_confidence_level_t.K4ABT_JOINT_CONFIDENCE_LOW);
            }
        }

        private void SetJoint(k4abt_joint_id_t id, Vector3 posMm, k4abt_joint_confidence_level_t conf)
        {
            _snap.Joints[(int)id] = new k4abt_joint_t
            {
                Position = new k4a_float3_t { X = posMm.x, Y = posMm.y, Z = posMm.z },
                Orientation = new k4a_quaternion_t { W = 1f },
                ConfidenceLevel = conf,
            };
        }

        private static Vector3 ToDepth(Vector3 world, CamXform x)
            => InverseTransform(InverseTransform(world, x.G), x.D2C);

        private static Vector3 InverseTransform(Vector3 p, ObExtrinsic e)
        {
            var R = e.Rot; var T = e.Trans;
            float x = p.x - T[0], y = p.y - T[1], z = p.z - T[2];
            return new Vector3(
                R[0] * x + R[3] * y + R[6] * z,
                R[1] * x + R[4] * y + R[7] * z,
                R[2] * x + R[5] * y + R[8] * z);
        }

        // ---------------- calibration ----------------

        private bool LoadCalibration()
        {
            string root = ResolveCalibrationRoot();
            if (root == null)
            {
                Debug.LogError($"[{nameof(LiveFusedBodySource)}] no extrinsics.yaml found (live manager root or playback root) — disabling.", this);
                return false;
            }
            var calib = PointCloudRecording.ReadExtrinsicsYaml(root);
            var rig = new List<ProjectorMask.CameraPose>();
            foreach (var c in calib)
            {
                if (!c.GlobalTrColorCamera.HasValue) continue;
                _xform[c.Serial] = new CamXform { D2C = c.DepthToColor, G = c.GlobalTrColorCamera.Value };
                _fused.SetWorldTransform(c.Serial, c.GlobalTrColorCamera.Value);
                rig.Add(new ProjectorMask.CameraPose
                {
                    Serial = c.Serial,
                    Param = new ObCameraParam { DepthIntrinsic = c.DepthIntrinsic, RgbIntrinsic = c.ColorIntrinsic, Transform = c.DepthToColor },
                    World = c.GlobalTrColorCamera.Value,
                });
            }
            if (_xform.Count == 0)
            {
                Debug.LogError($"[{nameof(LiveFusedBodySource)}] extrinsics.yaml at {root} has no camera with a world transform — disabling.", this);
                return false;
            }
            // live projector-flare mask uses the same rig geometry
            ProjectorMask.Configure(rig);
            Debug.Log($"[{nameof(LiveFusedBodySource)}] calibration loaded from {root}: {_xform.Count} camera(s)", this);
            return true;
        }

        private string ResolveCalibrationRoot()
        {
            // live rig calibration first; playback root as the dev-sim fallback
            var mgr = FindFirstObjectByType<SensorManager>();
            if (mgr != null)
            {
                try
                {
                    string r = mgr.ResolveExtrinsicsRoot();
                    if (r != null && File.Exists(Path.Combine(PointCloudRecording.CalibrationDir(r), "extrinsics.yaml")))
                        return r;
                }
                catch { }
            }
            if (usePlaybackFrames)
            {
                var rec = FindFirstObjectByType<SensorRecorder>();
                if (rec != null && !string.IsNullOrEmpty(rec.playbackFolderPath)
                    && File.Exists(Path.Combine(PointCloudRecording.CalibrationDir(rec.playbackFolderPath), "extrinsics.yaml")))
                    return rec.playbackFolderPath;
            }
            return null;
        }

        private static string FirstOnnx(string dir)
        {
            if (!Directory.Exists(dir)) return null;
            var files = Directory.GetFiles(dir, "*.onnx", SearchOption.AllDirectories);
            return files.Length > 0 ? files[0] : null;
        }
    }
}
