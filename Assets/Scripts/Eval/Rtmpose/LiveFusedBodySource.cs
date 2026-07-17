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

        [Tooltip("ONNX Runtime execution provider. Falls back downward only " +
                 "(Cuda→DirectML→Cpu); a mismatch between requested and actual " +
                 "EP is logged as an error. 30Hz fusion needs Cuda.")]
        public OrtProvider provider = OrtProvider.Cuda;

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

        [Tooltip("Median-5 fixed-lag smoothing on the fused output (~2 frames of " +
                 "latency, 66ms at 30Hz). Designed for 30Hz fusion; at low rates the " +
                 "lag grows proportionally. Applied on (re)build of the session adapter.")]
        public bool medianLagFilter = true;

        /// <summary>Fused frames emitted per second (diagnostic, worker-thread rate).
        /// Includes Heartbeat temporal-hold frames — use FreshFusedHz for benchmarks.</summary>
        public float FusedHz => _session != null ? _session.FusedHz : 0f;

        /// <summary>New fusion results per second, EXCLUDING temporal-hold (held)
        /// frames. This is the 30Hz benchmark gate (eval/PLAN_live_gpu.md Phase 3).</summary>
        public float FreshFusedHz => _session != null ? _session.FreshFusedHz : 0f;

        /// <summary>Heartbeat temporal-hold frames per second. ~0 when inference
        /// keeps up with the camera rate.</summary>
        public float HeldEmitHz => _session != null ? _session.HeldEmitHz : 0f;

        /// <summary>Bench probe access to the live fusion adapter (stage timings,
        /// fresh/held counters). Null when no session is running.</summary>
        public FusedRtmposeAdapter CurrentAdapter => _session?.Fused;

        // ---- per-serial frame queue (main thread writes, worker consumes) ----
        // A small ring absorbs delivery bursts: playback catch-up (the recorder's
        // deliverAllPlaybackFrames) and a momentarily busy worker both push
        // several frames per serial between wakes. The previous single pending
        // slot kept only the newest — which bound live fusion to the EDITOR
        // frame rate (measured 20.9Hz live vs 29.7Hz offline on the same take,
        // eval/results/live_v11s_*.md). Frames are handed to the worker as
        // PendingFrame instances and recycled through a per-slot free stack, so
        // the worker always owns the bytes it is reading and steady state
        // allocates nothing.
        private sealed class PendingFrame
        {
            public byte[] Depth = Array.Empty<byte>(), Color = Array.Empty<byte>(), IR = Array.Empty<byte>();
            public int DepthCount, ColorCount, IRCount;
            public int DW, DH, CW, CH, IW, IH;
            public ulong TsNs;
        }

        private sealed class Slot
        {
            // 16 ≈ 530ms of 30fps frames: an UNFOCUSED throttled editor ticks at
            // ~2fps and delivers ~13 frames per serial per Update — the ring must
            // hold a whole tick's catch-up batch or frames drop before the worker
            // sees them. Entries allocate lazily, so live capture (1 frame per
            // callback) never pays for the depth.
            public const int QueueDepth = 16;
            public string Serial;
            public readonly PendingFrame[] Ring = new PendingFrame[QueueDepth]; // SlotLock
            public int Tail, Count;                                             // SlotLock
            public readonly Stack<PendingFrame> Free = new Stack<PendingFrame>(QueueDepth); // SlotLock
            public int DroppedOldest;            // SlotLock (diagnostic: ring overflowed)
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
            public float FreshFusedHz;
            public float HeldEmitHz;
            /// <summary>Main thread refreshes this every Update. While playback
            /// drives the frames, the wall-clock heartbeat must stay OFF: a
            /// throttled editor delivers frames in batches, the heartbeat's
            /// synthetic timestamp overruns the recorded stream, and the fusion
            /// monotonic guard then drops every real burst until the recording
            /// catches up (measured as multi-second output holes + resume spikes).
            /// Recorded gaps are real gaps — the offline path sees them too.</summary>
            public volatile bool PlaybackActive;

            // ---- playback tap (reader thread) ----
            // The recorder's event path reads frame bytes on the MAIN thread, so
            // full-rate delivery there competes with rendering (and snowballs on
            // slow ticks — measured 2.5fps on a 62GB take). Instead a dedicated
            // reader streams the RCSV files itself, paced by CurrentPlayheadNs.
            public Thread Reader;
            public volatile string TapRoot;   // playback folder while playing, else null
            public long PlayheadNs;           // Interlocked; absolute recorded-ts ns
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
                    s.Wake.Dispose();
                    enabled = false;
                    return;
                }
                s.Backend = new OrtRtmposeBackend(yolox, rtm, provider);
                s.Fused = new FusedRtmposeAdapter(s.Backend) { ConfThreshold = confThreshold, medianLagFilter = medianLagFilter, AsyncDetect = true };
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
                s.Wake.Dispose();
                enabled = false;
                return;
            }

            if (!LoadCalibration(s))
            {
                s.Backend?.Dispose();
                s.Wake.Dispose();
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
            s.Reader = new Thread(() => ReaderLoop(s)) { IsBackground = true, Name = "LiveFusedTap" };
            s.Reader.Start();
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
            // reader sleeps ≤50ms between polls; if it is stuck in a long file read
            // it closes its own streams in its finally block — abandoning is safe
            s.Reader?.Join(1000);
            if (s.Worker == null || s.Worker.Join(5000))
            {
                // clean exit: the worker is gone, the session is ours to dispose
                s.Backend?.Dispose();
                s.Wake.Dispose();
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
            bool playing = _recorder != null && _recorder.IsPlaying;
            s.PlaybackActive = playing;
            if (playing)
            {
                s.TapRoot = _recorder.playbackFolderPath;
                Interlocked.Exchange(ref s.PlayheadNs, (long)_recorder.CurrentPlayheadNs);
            }
            else s.TapRoot = null;

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
                    // events supply slot METADATA (CamParam, source transform) at
                    // editor rate; the frame BYTES come from the reader-thread tap
                    // (see ReaderLoop) so full-rate delivery never costs main-thread
                    // time — the recorder's own event path only carries the frames
                    // the viewer renders.
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
                // while the playback tap streams the frame bytes itself, the event
                // path only carries metadata — enqueueing here too would duplicate
                // frames (the tap covers every recorded frame, events only the
                // viewer-rendered subset)
                if (s.TapRoot != null) return;

                PendingFrame pf;
                if (slot.Count == Slot.QueueDepth)
                {
                    // ring full: sacrifice the OLDEST queued frame, reuse its buffers
                    pf = slot.Ring[slot.Tail];
                    slot.Ring[slot.Tail] = null;
                    slot.Tail = (slot.Tail + 1) % Slot.QueueDepth;
                    slot.Count--;
                    slot.DroppedOldest++;
                }
                else pf = slot.Free.Count > 0 ? slot.Free.Pop() : new PendingFrame();

                CopyInto(ref pf.Depth, f.DepthBytes, f.DepthByteCount); pf.DepthCount = f.DepthByteCount;
                CopyInto(ref pf.Color, f.ColorBytes, f.ColorByteCount); pf.ColorCount = f.ColorByteCount;
                CopyInto(ref pf.IR, f.IRBytes, f.IRByteCount); pf.IRCount = f.IRByteCount;
                pf.DW = f.DepthWidth; pf.DH = f.DepthHeight;
                pf.CW = f.ColorWidth; pf.CH = f.ColorHeight;
                pf.IW = f.IRWidth; pf.IH = f.IRHeight;
                pf.TsNs = f.TimestampUs * 1000UL;
                slot.Ring[(slot.Tail + slot.Count) % Slot.QueueDepth] = pf;
                slot.Count++;
            }
            s.Wake.Set();
        }

        private static void CopyInto(ref byte[] dst, byte[] src, int count)
        {
            if (src == null || count <= 0) return;
            if (dst.Length < count) dst = new byte[count];
            Buffer.BlockCopy(src, 0, dst, 0, count);
        }

        // ---------------- playback tap (reader thread, session-owned) ----------------
        // Streams EVERY recorded frame straight from the RCSV files with its own
        // stream instances, paced by the recorder's playhead. IR is not read (the
        // pose pipeline uses depth+color only) which saves ~1/5 of the bandwidth.
        // No ProjectorMask is applied — the offline v11s conversion path does not
        // mask either, so tap output stays comparable.

        private sealed class TapTrack
        {
            public string Serial;
            public PointCloudRecording.RcsvFrameStream Depth, Color;
            public int DW, DH, CW, CH;
            public int Cursor;
            public int Skipped;                // diagnostic: disk fell >500ms behind
            public ObCameraParam? Cam;
        }

        private void ReaderLoop(Session s)
        {
            var tracks = new List<TapTrack>();
            string root = null;
            long lastPlayhead = 0;
            var staging = new PendingFrame(); // reader-owned; swapped into the ring
            try
            {
                while (!s.Stop)
                {
                    string want = s.TapRoot;
                    if (string.IsNullOrEmpty(want))
                    {
                        if (root != null) { CloseTap(tracks); root = null; }
                        Thread.Sleep(50);
                        continue;
                    }
                    if (root != want)
                    {
                        CloseTap(tracks);
                        root = want; // set even on failure so a broken root logs once, not per poll
                        try { OpenTap(want, tracks); lastPlayhead = 0; }
                        catch (Exception e) { Debug.LogException(e, this); }
                    }
                    long playhead = Interlocked.Read(ref s.PlayheadNs);
                    if (tracks.Count == 0 || playhead <= 0) { Thread.Sleep(10); continue; }
                    if (playhead + 1_000_000_000L < lastPlayhead)
                        foreach (var t in tracks) t.Cursor = 0; // playback rewound (loop/restart)
                    lastPlayhead = playhead;

                    bool any = false;
                    foreach (var t in tracks)
                    {
                        int guard = 0; // bound one poll's work per track so no track starves
                        while (!s.Stop && t.Cursor < t.Depth.Count && guard++ < 64 &&
                               (long)t.Depth.TimestampNsAt(t.Cursor) <= playhead)
                        {
                            // disk cannot keep up: drop the tail, keep the newest 500ms
                            if (playhead - (long)t.Depth.TimestampNsAt(t.Cursor) > 500_000_000L)
                            {
                                t.Cursor++; t.Skipped++;
                                continue;
                            }
                            var df = t.Depth[t.Cursor];
                            var cf = t.Color != null && t.Color.Count > 0
                                ? t.Color[Math.Min(t.Cursor, t.Color.Count - 1)] : null;
                            if (df?.Bytes != null && cf?.Bytes != null)
                                staging = EnqueueTapFrame(s, t, df, cf, staging);
                            t.Cursor++;
                            any = true;
                        }
                    }
                    if (!any) Thread.Sleep(3);
                }
            }
            catch (Exception e) { Debug.LogException(e, this); }
            finally { CloseTap(tracks); }
        }

        private void OpenTap(string rootDir, List<TapTrack> tracks)
        {
            var calib = PointCloudRecording.ReadExtrinsicsYaml(rootDir);
            foreach (var (serial, deviceDir) in PointCloudRecording.EnumerateDevices(rootDir))
            {
                string depthPath = Path.Combine(deviceDir, PointCloudRecording.DepthSensorName);
                string colorPath = Path.Combine(deviceDir, PointCloudRecording.ColorSensorName);
                if (!File.Exists(depthPath) || !File.Exists(colorPath)) continue;
                var t = new TapTrack { Serial = serial };
                t.Depth = new PointCloudRecording.RcsvFrameStream(depthPath);
                t.Color = new PointCloudRecording.RcsvFrameStream(colorPath);
                (t.DW, t.DH) = PointCloudRecording.ReadRcsvHeaderDimensions(depthPath);
                (t.CW, t.CH) = PointCloudRecording.ReadRcsvHeaderDimensions(colorPath);
                if (calib != null)
                {
                    foreach (var c in calib)
                    {
                        if (!string.Equals(c.Serial, serial, StringComparison.OrdinalIgnoreCase)) continue;
                        t.Cam = new ObCameraParam
                        {
                            DepthIntrinsic = c.DepthIntrinsic,
                            RgbIntrinsic = c.ColorIntrinsic,
                            DepthDistortion = c.DepthDistortion,
                            RgbDistortion = c.ColorDistortion,
                            Transform = c.DepthToColor,
                            IsMirrored = false,
                        };
                        break;
                    }
                }
                tracks.Add(t);
            }
            Debug.Log($"[{nameof(LiveFusedBodySource)}] playback tap open: {tracks.Count} device(s) under {rootDir}");
        }

        private static void CloseTap(List<TapTrack> tracks)
        {
            foreach (var t in tracks)
            {
                try { t.Depth?.Dispose(); } catch { }
                try { t.Color?.Dispose(); } catch { }
            }
            tracks.Clear();
        }

        /// <summary>Copy one tapped frame into the serial's ring. The bytes are
        /// copied into <paramref name="staging"/> OUTSIDE the slot lock, then the
        /// filled instance is swapped into the ring; returns the instance to use
        /// as the next staging buffer.</summary>
        private PendingFrame EnqueueTapFrame(Session s, TapTrack t,
            PointCloudRecording.Frame df, PointCloudRecording.Frame cf, PendingFrame staging)
        {
            CopyInto(ref staging.Depth, df.Bytes, df.ByteCount); staging.DepthCount = df.ByteCount;
            CopyInto(ref staging.Color, cf.Bytes, cf.ByteCount); staging.ColorCount = cf.ByteCount;
            staging.IRCount = 0;
            staging.DW = t.DW; staging.DH = t.DH;
            staging.CW = t.CW; staging.CH = t.CH;
            staging.IW = 0; staging.IH = 0;
            staging.TsNs = df.TimestampNs;

            PendingFrame next;
            lock (s.SlotLock)
            {
                if (!s.Slots.TryGetValue(t.Serial, out var slot))
                {
                    slot = new Slot { Serial = t.Serial };
                    s.Slots[t.Serial] = slot;
                }
                if (!slot.CamParam.HasValue && t.Cam.HasValue) slot.CamParam = t.Cam;
                if (slot.Count == Slot.QueueDepth)
                {
                    next = slot.Ring[slot.Tail]; // steal the OLDEST queued frame
                    slot.Ring[slot.Tail] = null;
                    slot.Tail = (slot.Tail + 1) % Slot.QueueDepth;
                    slot.Count--;
                    slot.DroppedOldest++;
                }
                else next = slot.Free.Count > 0 ? slot.Free.Pop() : new PendingFrame();
                slot.Ring[(slot.Tail + slot.Count) % Slot.QueueDepth] = staging;
                slot.Count++;
            }
            s.Wake.Set();
            return next;
        }

        // ---------------- worker thread (session-owned) ----------------

        private void WorkerLoop(Session s)
        {
            var drained = new List<(Slot Slot, PendingFrame Pf)>(16);
            var items = new List<FusedRtmposeAdapter.BurstItem>(16);
            long lastFrameWallMs = Environment.TickCount;
            ulong lastTsNs = 0;
            long rateWindowStart = Environment.TickCount;
            int lastFreshEmits = 0, lastHeldEmits = 0;

            while (!s.Stop)
            {
                s.Wake.WaitOne(10);
                if (s.Stop) break;

                if (s.ResetPending)
                {
                    s.ResetPending = false;
                    RebuildAdapter(s);
                    lastTsNs = 0;
                    lastFreshEmits = 0; lastHeldEmits = 0; // fresh adapter, counters restart
                }

                drained.Clear();
                lock (s.SlotLock)
                {
                    foreach (var kv in s.Slots)
                    {
                        var slot = kv.Value;
                        while (slot.Count > 0)
                        {
                            var pf = slot.Ring[slot.Tail];
                            slot.Ring[slot.Tail] = null;
                            slot.Tail = (slot.Tail + 1) % Slot.QueueDepth;
                            slot.Count--;
                            drained.Add((slot, pf));
                        }
                    }
                }

                if (drained.Count == 0)
                {
                    // sensor hiccup / GPU starved: keep the output cadence steady.
                    // Playback-driven sessions skip this — see Session.PlaybackActive.
                    long idle = Environment.TickCount - lastFrameWallMs;
                    if (!s.PlaybackActive && lastTsNs != 0 && idle > heartbeatMs)
                    {
                        lastFrameWallMs = Environment.TickCount;
                        try { s.Fused.Heartbeat(lastTsNs + (ulong)idle * 1_000_000UL); }
                        catch (Exception e) { Debug.LogException(e, this); }
                    }
                    continue;
                }

                items.Clear();
                ulong prevTsNs = lastTsNs; // stream position BEFORE this drain (gap-bridge baseline)
                foreach (var (slot, pf) in drained)
                {
                    if (s.Stop) break;
                    try
                    {
                        EnsureConfigured(s, slot, pf); // must precede SubmitBurst (adapter creation is not parallel-safe by design)
                        var raw = new RawFrameData(
                            pf.Depth, pf.DepthCount, pf.DW, pf.DH,
                            pf.Color, pf.ColorCount, pf.CW, pf.CH,
                            pf.IR, pf.IRCount, pf.IW, pf.IH,
                            pf.TsNs / 1000UL);
                        items.Add(new FusedRtmposeAdapter.BurstItem { Serial = slot.Serial, Frame = raw, TsNs = pf.TsNs });
                        lastTsNs = pf.TsNs > lastTsNs ? pf.TsNs : lastTsNs;
                        lastFrameWallMs = Environment.TickCount;
                    }
                    catch (Exception e) { Debug.LogException(e, this); }
                }
                if (items.Count > 0 && !s.Stop)
                {
                    // playback: bridge RECORDED rig-wide gaps (e.g. 15-50-24 has a
                    // 300ms drop at t=33s) with held frames at camera cadence, the
                    // same way FusedBodiesExport does offline. Bounded by the next
                    // REAL frame's timestamp, so unlike a wall-clock heartbeat it
                    // can never overrun the stream.
                    if (s.PlaybackActive && prevTsNs != 0)
                    {
                        ulong nextTs = ulong.MaxValue;
                        foreach (var it in items) if (it.TsNs < nextTs) nextTs = it.TsNs;
                        if (nextTs != ulong.MaxValue && nextTs > prevTsNs + 40_000_000UL)
                            for (ulong t = prevTsNs + 33_000_000UL; t + 5_000_000UL < nextTs; t += 33_000_000UL)
                            {
                                try { s.Fused.Heartbeat(t); }
                                catch (Exception e) { Debug.LogException(e, this); }
                            }
                    }
                    // per-camera inference runs concurrently; a catch-up drain can
                    // hold several frame-sets — SubmitBurst splits them into burst
                    // groups by timestamp and fuses each in order
                    try { s.Fused.SubmitBurst(items); }
                    catch (Exception e) { Debug.LogException(e, this); }
                }
                // SubmitBurst returns only after all reads of the frame bytes are
                // done, so the instances can be recycled for the next Ingest
                lock (s.SlotLock)
                    foreach (var (slot, pf) in drained)
                        if (slot.Free.Count < Slot.QueueDepth) slot.Free.Push(pf);

                long win = Environment.TickCount - rateWindowStart;
                if (win >= 2000)
                {
                    // actual fused OUTPUT rate (OnFusedSkeletons emissions)
                    s.FusedHz = Interlocked.Exchange(ref s.FusedEmitted, 0) * 1000f / win;
                    // fresh vs held split from the adapter (counters live on this
                    // worker thread). FreshFusedHz is the 30Hz bench gate: held
                    // frames keep the cadence and would fake the rate on a slow EP.
                    if (s.Fused != null)
                    {
                        s.FreshFusedHz = (s.Fused.StatFreshEmits - lastFreshEmits) * 1000f / win;
                        s.HeldEmitHz = (s.Fused.StatHeldEmits - lastHeldEmits) * 1000f / win;
                        lastFreshEmits = s.Fused.StatFreshEmits;
                        lastHeldEmits = s.Fused.StatHeldEmits;
                    }
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
                var next = new FusedRtmposeAdapter(s.Backend) { ConfThreshold = confThreshold, medianLagFilter = medianLagFilter, AsyncDetect = true };
                next.Profile = s.Fused?.Profile;
                next.SetCaptureVolume(captureVolumeCenterMm, captureVolumeHalfMm);
                foreach (var kv in s.Xform) next.SetWorldTransform(kv.Key, kv.Value.G);
                next.OnSkeletons += f => OnFusedSkeletons(s, f);
                s.Fused = next; // old adapter (and its subscription) drops with it
                lock (s.SlotLock)
                    foreach (var kv in s.Slots)
                    {
                        var slot = kv.Value;
                        slot.Configured = false;
                        // drop pre-loop queued frames — their timestamps precede the rewind
                        while (slot.Count > 0)
                        {
                            var pf = slot.Ring[slot.Tail];
                            slot.Ring[slot.Tail] = null;
                            slot.Tail = (slot.Tail + 1) % Slot.QueueDepth;
                            slot.Count--;
                            if (slot.Free.Count < Slot.QueueDepth) slot.Free.Push(pf);
                        }
                    }
            }
            catch (Exception e) { Debug.LogException(e, this); }
        }

        // adapter.Configure needs stream dims — known on first frame. Worker
        // thread only, before the serial's first SubmitFrame.
        private void EnsureConfigured(Session s, Slot slot, PendingFrame pf)
        {
            if (slot.Configured) return;
            if (!slot.CamParam.HasValue)
            {
                // playback path always has calibration-built params; live fills in
                // once the renderer publishes them — until then skip the frame.
                throw new InvalidOperationException($"no CameraParam yet for {slot.Serial}");
            }
            var ctx = new EvalCameraContext(slot.Serial, pf.DW, pf.DH, pf.CW, pf.CH, slot.CamParam);
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

        // Skeleton conversion shared with FusedBodiesExport / FusedTakeConverter
        // (world -> camera color -> camera depth, k4abt id mapping + derived fills).
        private static void BuildSnapshot(Session s, EvalSkeleton p, CamXform x)
            => FusedSnapshotEncoder.Build(s.Snap, p, x.G, x.D2C);

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
