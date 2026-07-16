// Live RTMPose fusion source: runs the FusedRtmposeAdapter (YOLOX detect ->
// RTMPose 2D -> depth lift -> 3-stage cluster fusion, v11) on a BACKGROUND
// thread over live camera frames and feeds the fused skeleton into the
// production SkeletonMerger through SubmitExternalBodies — the k4abt-free
// body-tracking path.
//
// Threading: Unity main thread only copies each arriving frame into a
// per-serial pending slot (latest wins) and drains fused results back into
// the merger; ALL inference and fusion runs on one worker thread. The whole
// adapter chain (ORT + pure C# math) has no Unity-API dependency (verified),
// and single-threaded use inside the worker keeps the adapter's state safe.
// The GPU sets the achievable fusion rate (~10-25 Hz on DirectML with 4
// cams; CUDA/TensorRT EP is the path to a locked 30 Hz); the adapter's
// temporal hold + heartbeat keep the output cadence steady in between.
//
// Ownership: everything the worker thread touches lives in one Session
// object. Disable joins the worker; if the join TIMES OUT (worker stuck in
// a native inference call) the whole session — backend, adapter, slots,
// queues, scratch — is abandoned as a unit and deliberately leaked, so the
// stuck thread can never race against a re-enabled component's fresh
// session. (Codex-reviewed shutdown contract.)
//
// Frame sources:
//  - LIVE: every PointCloudRenderer.OnRawFramesReady in the scene.
//  - usePlaybackFrames: SensorRecorder.OnPlaybackRawFrame too — the dev
//    "live-sim" mode: recorded takes drive the exact live code path on
//    machines without cameras. Playback loops rewind timestamps, which the
//    strictly-monotonic adapter would reject forever, so OnPlaybackLooped
//    schedules an adapter rebuild on the worker thread.
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
        public float FusedHz => _session != null ? _session.FusedHz : 0f;

        // ---- per-serial frame slot (main thread writes, worker consumes) ----
        // Double-buffered: Ingest writes the PENDING arrays under the session's
        // slot lock; the worker SWAPS them with its processing arrays (still
        // under the lock) and reads the swapped-out set outside the lock — the
        // worker always owns immutable frame bytes, a later Ingest writes the
        // other set.
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

        private sealed class CamXform { public ObExtrinsic D2C; public ObExtrinsic G; }

        private sealed class FusedResult
        {
            public ulong TsNs;
            public readonly Dictionary<string, byte[]> BytesBySerial = new Dictionary<string, byte[]>();
            public int ByteCount;
        }

        /// <summary>Everything the worker thread can touch, owned as a unit. On a
        /// clean shutdown the session is disposed; on a stuck shutdown it is
        /// abandoned whole (leaked), so the stuck thread can never observe a
        /// later session's state.</summary>
        private sealed class Session
        {
            public LiveFusedBodySource Owner; // config + Debug context only
            public OrtRtmposeBackend Backend;
            public FusedRtmposeAdapter Fused;
            public Thread Worker;
            public volatile bool Stop;
            public volatile bool ResetPending;
            public readonly AutoResetEvent Wake = new AutoResetEvent(false);
            public readonly object SlotLock = new object();
            public readonly Dictionary<string, Slot> Slots = new Dictionary<string, Slot>();
            public readonly object ResultLock = new object();
            public readonly Queue<FusedResult> Results = new Queue<FusedResult>();
            public readonly Dictionary<string, CamXform> Xform = new Dictionary<string, CamXform>();
            public BodySnapshot Snap;
            public byte[] EncodeScratch;
            public int FusedEmitted;
            public float FusedHz;
        }

        private Session _session;
        private readonly List<PointCloudRenderer> _subscribedRenderers = new List<PointCloudRenderer>();
        private SensorRecorder _recorder;
        private bool _mergerFlagOwned;

        private void OnEnable()
        {
            if (merger == null) merger = FindFirstObjectByType<SkeletonMerger>();

            var s = new Session
            {
                Owner = this,
                Snap = new BodySnapshot { Id = 1 },
                EncodeScratch = new byte[RecordedBodySerializer.FrameSize(1)],
            };

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
                s.Backend = new OrtRtmposeBackend(yolox, rtm);
                s.Fused = new FusedRtmposeAdapter(s.Backend) { ConfThreshold = confThreshold };
                string profile = Path.Combine(root, bodyProfilePath);
                if (!string.IsNullOrEmpty(bodyProfilePath) && File.Exists(profile))
                    s.Fused.Profile = BodyProfile.Load(profile);
                s.Fused.SetCaptureVolume(captureVolumeCenterMm, captureVolumeHalfMm);
                s.Fused.OnSkeletons += f => OnFusedSkeletons(s, f); // worker thread!
            }
            catch (Exception e)
            {
                Debug.LogException(e, this);
                s.Backend?.Dispose();
                enabled = false;
                return;
            }

            if (!LoadCalibration(s))
            {
                s.Backend?.Dispose();
                enabled = false;
                return;
            }

            if (merger != null && !merger.useExternalBodies)
            {
                merger.useExternalBodies = true;
                _mergerFlagOwned = true;
            }

            _session = s;
            SubscribeSources();

            s.Worker = new Thread(() => WorkerLoop(s)) { IsBackground = true, Name = "LiveFusedBodySource" };
            s.Worker.Start();
        }

        private void OnDisable()
        {
            foreach (var r in _subscribedRenderers)
                if (r != null) r.OnRawFramesReady -= OnLiveFrame;
            _subscribedRenderers.Clear();
            if (_recorder != null)
            {
                _recorder.OnPlaybackRawFrame -= OnPlaybackFrame;
                _recorder.OnPlaybackLooped -= HandlePlaybackLooped;
                _recorder = null;
            }

            if (merger != null && _mergerFlagOwned) merger.useExternalBodies = false;
            _mergerFlagOwned = false;

            var s = _session;
            _session = null;
            if (s == null) return;

            s.Stop = true;
            s.Wake.Set();
            if (s.Worker == null || s.Worker.Join(5000))
            {
                // clean exit: the worker is gone, the session is ours to dispose
                s.Backend?.Dispose();
            }
            else
            {
                // The worker is stuck inside a native inference call. Abandon the
                // ENTIRE session — backend, adapter, slots, queues, scratch — as a
                // unit: nothing of it is cleared or reused, so the stuck thread
                // can only ever touch its own leaked state, never a fresh
                // session created by a later OnEnable. Deliberate leak; process
                // teardown reclaims it.
                Debug.LogError($"[{nameof(LiveFusedBodySource)}] worker did not stop within 5s — abandoning the whole session (deliberate leak) instead of racing a running inference", this);
            }
        }

        private void Update()
        {
            var s = _session;
            if (s == null) return;

            // late-joining renderers (live cameras open asynchronously)
            SubscribeSources();

            // drain fused results into the merger (main thread — merger touches
            // Unity transforms)
            while (true)
            {
                FusedResult res;
                lock (s.ResultLock)
                {
                    if (s.Results.Count == 0) break;
                    res = s.Results.Dequeue();
                }
                if (merger == null) continue;
                foreach (var kv in res.BytesBySerial)
                {
                    Slot slot;
                    lock (s.SlotLock) s.Slots.TryGetValue(kv.Key, out slot);
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
            var s = _session;
            if (s == null) return;
            s.ResetPending = true;
            s.Wake.Set();
        }

        private void OnLiveFrame(PointCloudRenderer r, RawFrameData frame)
            => Ingest(r.deviceSerial, r.CameraParam, r.transform, frame);

        private void OnPlaybackFrame(string serial, ObCameraParam? camParam, Transform tr, RawFrameData frame)
            => Ingest(serial, camParam, tr, frame);

        private void Ingest(string serial, ObCameraParam? camParam, Transform tr, in RawFrameData f)
        {
            var s = _session;
            if (s == null) return;
            if (string.IsNullOrEmpty(serial) || f.DepthBytes == null || f.ColorBytes == null) return;
            lock (s.SlotLock)
            {
                if (!s.Slots.TryGetValue(serial, out var slot))
                {
                    slot = new Slot { Serial = serial };
                    s.Slots[serial] = slot;
                }
                slot.CamParam = camParam ?? slot.CamParam;
                slot.SourceTransform = tr != null ? tr : slot.SourceTransform;
                CopyInto(ref slot.Depth, f.DepthBytes, f.DepthByteCount); slot.DepthCount = f.DepthByteCount;
                CopyInto(ref slot.Color, f.ColorBytes, f.ColorByteCount); slot.ColorCount = f.ColorByteCount;
                CopyInto(ref slot.IR, f.IRBytes, f.IRByteCount); slot.IRCount = f.IRByteCount;
                slot.DW = f.DepthWidth; slot.DH = f.DepthHeight;
                slot.CW = f.ColorWidth; slot.CH = f.ColorHeight;
                slot.IW = f.IRWidth; slot.IH = f.IRHeight;
                slot.TsNs = f.TimestampUs * 1000UL;
                slot.Fresh = true;
            }
            s.Wake.Set();
        }

        private static void CopyInto(ref byte[] dst, byte[] src, int count)
        {
            if (src == null || count <= 0) return;
            if (dst.Length < count) dst = new byte[count];
            Buffer.BlockCopy(src, 0, dst, 0, count);
        }

        // ---------------- worker thread (session-owned) ----------------

        private void WorkerLoop(Session s)
        {
            var burst = new List<Slot>(8);
            long lastFrameWallMs = Environment.TickCount;
            ulong lastTsNs = 0;
            long rateWindowStart = Environment.TickCount;

            while (!s.Stop)
            {
                s.Wake.WaitOne(10);
                if (s.Stop) break;

                if (s.ResetPending)
                {
                    s.ResetPending = false;
                    RebuildAdapter(s);
                    lastTsNs = 0;
                }

                burst.Clear();
                lock (s.SlotLock)
                {
                    foreach (var kv in s.Slots)
                    {
                        var slot = kv.Value;
                        if (!slot.Fresh) continue;
                        slot.Fresh = false;
                        // swap pending <-> processing so the worker owns the bytes
                        (slot.Depth, slot.ProcDepth) = (slot.ProcDepth, slot.Depth);
                        (slot.Color, slot.ProcColor) = (slot.ProcColor, slot.Color);
                        (slot.IR, slot.ProcIR) = (slot.ProcIR, slot.IR);
                        slot.ProcDepthCount = slot.DepthCount; slot.ProcColorCount = slot.ColorCount; slot.ProcIRCount = slot.IRCount;
                        slot.PDW = slot.DW; slot.PDH = slot.DH; slot.PCW = slot.CW; slot.PCH = slot.CH; slot.PIW = slot.IW; slot.PIH = slot.IH;
                        slot.ProcTsNs = slot.TsNs;
                        burst.Add(slot);
                    }
                }

                if (burst.Count == 0)
                {
                    // sensor hiccup / GPU starved: keep the output cadence steady
                    long idle = Environment.TickCount - lastFrameWallMs;
                    if (lastTsNs != 0 && idle > heartbeatMs)
                    {
                        lastFrameWallMs = Environment.TickCount;
                        try { s.Fused.Heartbeat(lastTsNs + (ulong)idle * 1_000_000UL); }
                        catch (Exception e) { Debug.LogException(e, this); }
                    }
                    continue;
                }

                foreach (var slot in burst)
                {
                    if (s.Stop) break;
                    try
                    {
                        EnsureConfigured(s, slot);
                        var raw = new RawFrameData(
                            slot.ProcDepth, slot.ProcDepthCount, slot.PDW, slot.PDH,
                            slot.ProcColor, slot.ProcColorCount, slot.PCW, slot.PCH,
                            slot.ProcIR, slot.ProcIRCount, slot.PIW, slot.PIH,
                            slot.ProcTsNs / 1000UL);
                        s.Fused.SubmitFrame(slot.Serial, raw, slot.ProcTsNs);
                        lastTsNs = slot.ProcTsNs > lastTsNs ? slot.ProcTsNs : lastTsNs;
                        lastFrameWallMs = Environment.TickCount;
                    }
                    catch (Exception e) { Debug.LogException(e, this); }
                }

                long win = Environment.TickCount - rateWindowStart;
                if (win >= 2000)
                {
                    // actual fused OUTPUT rate (OnFusedSkeletons emissions)
                    s.FusedHz = Interlocked.Exchange(ref s.FusedEmitted, 0) * 1000f / win;
                    rateWindowStart = Environment.TickCount;
                }
            }
            try { s.Fused?.FlushLag(); } catch { }
        }

        // Worker thread. Fresh adapter after a playback loop — timestamps rewind
        // and the adapter is strictly monotonic. The shared backend (ORT
        // sessions) survives, only the fusion state is rebuilt.
        private void RebuildAdapter(Session s)
        {
            try
            {
                var next = new FusedRtmposeAdapter(s.Backend) { ConfThreshold = confThreshold };
                next.Profile = s.Fused?.Profile;
                next.SetCaptureVolume(captureVolumeCenterMm, captureVolumeHalfMm);
                foreach (var kv in s.Xform) next.SetWorldTransform(kv.Key, kv.Value.G);
                next.OnSkeletons += f => OnFusedSkeletons(s, f);
                s.Fused = next; // old adapter (and its subscription) drops with it
                lock (s.SlotLock)
                    foreach (var kv in s.Slots) { kv.Value.Configured = false; kv.Value.Fresh = false; }
            }
            catch (Exception e) { Debug.LogException(e, this); }
        }

        // adapter.Configure needs stream dims — known on first frame. Worker
        // thread only, before the serial's first SubmitFrame.
        private void EnsureConfigured(Session s, Slot slot)
        {
            if (slot.Configured) return;
            if (!slot.CamParam.HasValue)
            {
                // playback path always has calibration-built params; live fills in
                // once the renderer publishes them — until then skip the frame.
                throw new InvalidOperationException($"no CameraParam yet for {slot.Serial}");
            }
            var ctx = new EvalCameraContext(slot.Serial, slot.PDW, slot.PDH, slot.PCW, slot.PCH, slot.CamParam);
            s.Fused.Configure(ctx);
            if (s.Xform.TryGetValue(slot.Serial, out var x))
                s.Fused.SetWorldTransform(slot.Serial, x.G);
            slot.Configured = true;
        }

        // ---------------- fused output (worker thread, session-owned) ----------------

        private void OnFusedSkeletons(Session s, EvalSkeletonFrame f)
        {
            Interlocked.Increment(ref s.FusedEmitted);
            var p = f.Primary();
            if (p == null) return;

            var res = new FusedResult { TsNs = f.TimestampNs };
            foreach (var kv in s.Xform)
            {
                BuildSnapshot(s, p, kv.Value);
                int bytes = RecordedBodySerializer.Encode(new[] { s.Snap }, 1, s.EncodeScratch);
                var copy = new byte[bytes];
                Buffer.BlockCopy(s.EncodeScratch, 0, copy, 0, bytes);
                res.BytesBySerial[kv.Key] = copy;
                res.ByteCount = bytes;
            }
            if (res.BytesBySerial.Count == 0) return;
            lock (s.ResultLock)
            {
                s.Results.Enqueue(res);
                // backpressure: never let the queue grow unbounded if the main
                // thread stalls — drop the oldest
                while (s.Results.Count > 8) s.Results.Dequeue();
            }
        }

        // Same skeleton conversion as FusedBodiesExport (kept in sync by hand —
        // the export lives in an Editor assembly this runtime component can't
        // reference): world -> camera color -> camera depth, EvalSkeleton joints
        // mapped to k4abt ids + derived spine/clavicle/nose fills.
        private void BuildSnapshot(Session s, EvalSkeleton p, CamXform x)
        {
            for (int i = 0; i < K4ABTConsts.K4ABT_JOINT_COUNT; i++)
                s.Snap.Joints[i] = new k4abt_joint_t
                {
                    Position = new k4a_float3_t(),
                    Orientation = new k4a_quaternion_t { W = 1f },
                    ConfidenceLevel = k4abt_joint_confidence_level_t.K4ABT_JOINT_CONFIDENCE_NONE,
                };

            for (int j = 0; j < EvalSkeleton.JointCount; j++)
            {
                if (!p.Joints[j].Valid) continue;
                Vector3 d = ToDepth(p.Joints[j].PositionMm, x);
                SetJoint(s, EvalSkeletonMap.K4abtSource[j], d, k4abt_joint_confidence_level_t.K4ABT_JOINT_CONFIDENCE_MEDIUM);
            }

            bool hasPelvis = p.Joints[(int)EvalJointId.Pelvis].Valid;
            bool hasNeck = p.Joints[(int)EvalJointId.Neck].Valid;
            if (hasPelvis && hasNeck)
            {
                Vector3 pel = ToDepth(p.Joints[(int)EvalJointId.Pelvis].PositionMm, x);
                Vector3 nk = ToDepth(p.Joints[(int)EvalJointId.Neck].PositionMm, x);
                SetJoint(s, k4abt_joint_id_t.K4ABT_JOINT_SPINE_NAVEL, Vector3.Lerp(pel, nk, 1f / 3f), k4abt_joint_confidence_level_t.K4ABT_JOINT_CONFIDENCE_LOW);
                SetJoint(s, k4abt_joint_id_t.K4ABT_JOINT_SPINE_CHEST, Vector3.Lerp(pel, nk, 2f / 3f), k4abt_joint_confidence_level_t.K4ABT_JOINT_CONFIDENCE_LOW);
                SetJoint(s, k4abt_joint_id_t.K4ABT_JOINT_CLAVICLE_LEFT, nk, k4abt_joint_confidence_level_t.K4ABT_JOINT_CONFIDENCE_LOW);
                SetJoint(s, k4abt_joint_id_t.K4ABT_JOINT_CLAVICLE_RIGHT, nk, k4abt_joint_confidence_level_t.K4ABT_JOINT_CONFIDENCE_LOW);
            }
            if (p.Joints[(int)EvalJointId.Head].Valid)
            {
                Vector3 hd = ToDepth(p.Joints[(int)EvalJointId.Head].PositionMm, x);
                SetJoint(s, k4abt_joint_id_t.K4ABT_JOINT_NOSE, hd, k4abt_joint_confidence_level_t.K4ABT_JOINT_CONFIDENCE_LOW);
            }
        }

        private static void SetJoint(Session s, k4abt_joint_id_t id, Vector3 posMm, k4abt_joint_confidence_level_t conf)
        {
            s.Snap.Joints[(int)id] = new k4abt_joint_t
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

        private bool LoadCalibration(Session s)
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
                s.Xform[c.Serial] = new CamXform { D2C = c.DepthToColor, G = c.GlobalTrColorCamera.Value };
                s.Fused.SetWorldTransform(c.Serial, c.GlobalTrColorCamera.Value);
                rig.Add(new ProjectorMask.CameraPose
                {
                    Serial = c.Serial,
                    Param = new ObCameraParam { DepthIntrinsic = c.DepthIntrinsic, RgbIntrinsic = c.ColorIntrinsic, Transform = c.DepthToColor },
                    World = c.GlobalTrColorCamera.Value,
                });
            }
            if (s.Xform.Count == 0)
            {
                Debug.LogError($"[{nameof(LiveFusedBodySource)}] extrinsics.yaml at {root} has no camera with a world transform — disabling.", this);
                return false;
            }
            // live projector-flare mask uses the same rig geometry
            ProjectorMask.Configure(rig);
            Debug.Log($"[{nameof(LiveFusedBodySource)}] calibration loaded from {root}: {s.Xform.Count} camera(s)", this);
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
