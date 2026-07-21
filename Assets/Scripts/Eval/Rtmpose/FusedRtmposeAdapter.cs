// Multi-camera RTMPose fusion (the "3-stage fusion" from the eval report,
// 2026-07-16). Wraps one per-camera RtmPoseAdapter and fuses the per-serial
// skeletons into a single world-space skeleton (serial "fused"):
//
//   Stage 1  per-camera bone-length sanity — with a calibrated body profile
//            (eval/body_profile.json), a bone whose length is way off blames
//            its DISTAL joint and drops it from fusion (catches depth-lift
//            accidents a confidence threshold cannot: measured f788, a 592 mm
//            pelvis error carried conf 0.64).
//   Stage 2  per-joint cross-camera consensus — component-wise median across
//            cameras, outliers beyond a per-joint gate rejected (gates are
//            per-joint because surface parallax puts a legitimate 150-250 mm
//            floor on torso joints between opposing cameras), survivors
//            averaged with confidence weights.
//   Stage 3  fallbacks for no-consensus joints, in order:
//            (a) ray x bone-length re-lift — depth-lift errors are Z-only, so
//                the observed camera RAY is still right; intersect it with a
//                sphere of the calibrated bone length around the fused parent
//                to recover the depth analytically (fixes the ~800 mm blurred
//                wrist where all cameras disagree);
//            (b) temporal hold — previous fused position + capped velocity
//                extrapolation, decaying validity after holdMaxSeconds.
//
// World frame: the OpenCV-convention origin-camera frame in millimeters (the
// same frame RtmPoseAdapter.SetWorldTransform / SetCaptureVolume use), so no
// Unity-axis conversion happens inside the fusion math.

using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Text.RegularExpressions;
using System.Threading;
using Orbbec;
using PointCloud;
using UnityEngine;

namespace BodyTracking.Eval.Rtmpose
{
    /// <summary>Calibrated per-bone lengths (mm). See BodyProfileBuilder.</summary>
    public sealed class BodyProfile
    {
        /// <summary>Length (mm) per FusedRtmposeAdapter.Bones index; 0 = unknown.</summary>
        public readonly float[] LengthMm = new float[FusedRtmposeAdapter.Bones.Length];

        public static BodyProfile Load(string jsonPath)
        {
            var p = new BodyProfile();
            string text = File.ReadAllText(jsonPath);
            var rx = new Regex("\"(\\w+-\\w+)\"\\s*:\\s*\\{\\s*\"medianMm\"\\s*:\\s*([\\d.]+)");
            var byName = new Dictionary<string, float>();
            foreach (Match m in rx.Matches(text))
                byName[m.Groups[1].Value] = float.Parse(m.Groups[2].Value, CultureInfo.InvariantCulture);
            for (int b = 0; b < FusedRtmposeAdapter.Bones.Length; b++)
            {
                var (a, c) = FusedRtmposeAdapter.Bones[b];
                if (byName.TryGetValue($"{a}-{c}", out float len)) p.LengthMm[b] = len;
            }
            return p;
        }
    }

    public sealed class FusedRtmposeAdapter : ITrackerAdapter
    {
        public string Name => "rtmfused";

        /// <summary>Parent→child bone list; enum order doubles as the kinematic
        /// solve order (parents always fuse before their children).</summary>
        public static readonly (EvalJointId a, EvalJointId b)[] Bones =
        {
            (EvalJointId.Pelvis, EvalJointId.Neck), (EvalJointId.Neck, EvalJointId.Head),
            (EvalJointId.Neck, EvalJointId.ShoulderL), (EvalJointId.ShoulderL, EvalJointId.ElbowL), (EvalJointId.ElbowL, EvalJointId.WristL),
            (EvalJointId.Neck, EvalJointId.ShoulderR), (EvalJointId.ShoulderR, EvalJointId.ElbowR), (EvalJointId.ElbowR, EvalJointId.WristR),
            (EvalJointId.Pelvis, EvalJointId.HipL), (EvalJointId.HipL, EvalJointId.KneeL), (EvalJointId.KneeL, EvalJointId.AnkleL),
            (EvalJointId.Pelvis, EvalJointId.HipR), (EvalJointId.HipR, EvalJointId.KneeR), (EvalJointId.KneeR, EvalJointId.AnkleR),
        };

        // boneOfChild[joint] = index into Bones whose .b == joint, or -1 (Pelvis).
        static readonly int[] BoneOfChild = BuildBoneOfChild();
        static int[] BuildBoneOfChild()
        {
            var map = new int[(int)EvalJointId.Count];
            for (int i = 0; i < map.Length; i++) map[i] = -1;
            for (int b = 0; b < Bones.Length; b++) map[(int)Bones[b].b] = b;
            return map;
        }

        // ---- tuning ----
        /// <summary>Consensus gate (mm) per joint: deviation from the others-median
        /// beyond this rejects the sample. Torso joints get wider gates than
        /// extremities (surface-parallax floor between opposing cameras).</summary>
        public readonly float[] gateMm = BuildDefaultGates();
        static float[] BuildDefaultGates()
        {
            var g = new float[(int)EvalJointId.Count];
            for (int i = 0; i < g.Length; i++) g[i] = 300f;
            g[(int)EvalJointId.Pelvis] = 350f; g[(int)EvalJointId.HipL] = 350f; g[(int)EvalJointId.HipR] = 350f;
            g[(int)EvalJointId.ElbowL] = 280f; g[(int)EvalJointId.ElbowR] = 280f;
            g[(int)EvalJointId.KneeL] = 280f; g[(int)EvalJointId.KneeR] = 280f;
            g[(int)EvalJointId.WristL] = 250f; g[(int)EvalJointId.WristR] = 250f;
            g[(int)EvalJointId.AnkleL] = 250f; g[(int)EvalJointId.AnkleR] = 250f;
            return g;
        }

        /// <summary>Bone-length tolerance: |len − L| ≤ max(relTol·L, absTolMm) passes.</summary>
        public float lenRelTol = 0.35f;
        public float lenAbsTolMm = 90f;
        /// <summary>Max cross-camera timestamp skew for samples entering one fusion (ms).</summary>
        public float skewMs = 60f;
        /// <summary>Minimum interval between fused emissions (ms) — caps the fused rate near the camera rate.</summary>
        public float minFuseIntervalMs = 25f;
        /// <summary>Temporal hold: keep predicting a lost joint for this long (s).</summary>
        public float holdMaxSeconds = 0.3f;
        /// <summary>Velocity extrapolation cap (s) for the temporal prediction.</summary>
        public float maxPredictSeconds = 0.2f;
        /// <summary>Velocity EMA retention (0 = raw frame-to-frame velocity).</summary>
        public float velSmoothing = 0.5f;
        /// <summary>Frame-to-frame velocities above this (m/s) never enter the
        /// velocity history — a spike-derived velocity otherwise turns the
        /// temporal hold into a runaway straight streak (seen as a ~1.2 m line
        /// off the wrist in the motion curves).</summary>
        public float velMaxMps = 5f;
        /// <summary>Cap (mm) on the total extrapolation displacement of a held joint.</summary>
        public float holdMaxStepMm = 200f;
        /// <summary>Two-strike jump gate (mm): a FRESH fused position this far from
        /// the recent history is held back once; the second consecutive frame is
        /// accepted when it lands near the pending position OR continues in the
        /// same direction (genuine fast motion keeps moving; an out-and-back
        /// spike reverses). 220 mm/frame ≈ 6.6 m/s at 30 fps.</summary>
        public float jumpGateMm = 220f;
        /// <summary>Confirmations a WEAK-evidence jump (no cross-camera cluster
        /// behind it — single camera or ray re-lift) needs before acceptance. A
        /// grossly wrong camera repeats the same wrong position for several
        /// frames, so proximity-to-pending alone is not proof for it; strong
        /// (clustered) jumps still accept on the first confirmation. Late
        /// acceptance leaves at most a 1-frame excursion, which the median-5
        /// lag filter then removes structurally.</summary>
        public int weakJumpConfirms = 3;

        public BodyProfile Profile;

        // One RtmPoseAdapter PER CAMERA so SubmitBurst can run the four cameras'
        // inference concurrently: each adapter's scratch buffers (input tensor,
        // simcc outputs, depth lift) are then thread-confined. Behavior-identical
        // to the old shared adapter — its per-camera state (track box, intrinsics)
        // was already keyed by serial.
        readonly OrtRtmposeBackend _backend;
        readonly Dictionary<string, RtmPoseAdapter> _perSerial = new();
        readonly object _adapterLock = new();  // guards _perSerial mutation
        readonly object _ingestLock = new();   // guards _latest ingestion from parallel workers
        float _confThreshold = 0.3f;
        bool _asyncDetect;
        Vector3 _volCenterCfg, _volHalfCfg; bool _hasVolumeCfg;

        /// <summary>Forwarded to every per-camera adapter — see RtmPoseAdapter.asyncDetect.</summary>
        public bool AsyncDetect
        {
            get => _asyncDetect;
            set { _asyncDetect = value; lock (_adapterLock) foreach (var a in _perSerial.Values) a.asyncDetect = value; }
        }

        /// <summary>Per-camera adapters (bench probes aggregate their timings).</summary>
        public IEnumerable<RtmPoseAdapter> InnerAdapters { get { lock (_adapterLock) return new List<RtmPoseAdapter>(_perSerial.Values); } }

        /// <summary>Per-camera (pre-fusion) skeleton stream — the old Inner.OnSkeletons.</summary>
        public event Action<EvalSkeletonFrame> OnPerCameraSkeletons;

        RtmPoseAdapter PerSerial(string serial)
        {
            lock (_adapterLock)
            {
                if (_perSerial.TryGetValue(serial, out var a)) return a;
                a = new RtmPoseAdapter(_backend) { confThreshold = _confThreshold, asyncDetect = _asyncDetect };
                if (_hasVolumeCfg) a.SetCaptureVolume(_volCenterCfg, _volHalfCfg);
                a.OnSkeletons += OnInnerSkeletons;
                _perSerial[serial] = a;
                return a;
            }
        }

        // per-serial world transform (OpenCV frame, mm): world = R·cam + T
        sealed class WorldTr { public float[] R; public float[] T; }
        readonly Dictionary<string, WorldTr> _world = new();

        // ---- per-camera depth reference for the visibility (z-buffer) test ----
        // A camera's joint estimate is only trustworthy if the camera can SEE the
        // joint. The depth map answers that geometrically: project a competing
        // hypothesis into this camera — if the camera's surface at that pixel is
        // clearly CLOSER than the hypothesis, the hypothesis is occluded from
        // this camera, i.e. this camera was guessing (e.g. a hand fully hidden
        // behind the torso: the model hallucinates it at the body silhouette).
        sealed class DepthRef
        {
            public ushort[] D; public int W, H;      // stride-2 downsample
            public float CalFx, CalFy, CalCx, CalCy; // as calibrated
            public int CalW, CalH;                   // calibration resolution
            public float Fx, Fy, Cx, Cy;             // effective for the buffer
                                                     // (scaled to frame res / 2)
            public float[] Rd2c, Td2c;               // depth -> color extrinsic
            public bool HasCal;
        }
        readonly Dictionary<string, DepthRef> _depth = new();

        /// <summary>A competing joint hypothesis must be at least this much (mm)
        /// BEHIND a camera's measured surface before that camera counts as
        /// occluded (body thickness scale; keeps normal surface noise out).</summary>
        public float occlusionMarginMm = 200f;

        sealed class CamSample
        {
            public ulong Ts;
            public readonly Vector3[] World = new Vector3[(int)EvalJointId.Count];
            public readonly float[] Conf = new float[(int)EvalJointId.Count];
            public readonly bool[] Valid = new bool[(int)EvalJointId.Count];
        }
        readonly Dictionary<string, CamSample> _latest = new();

        // per-joint temporal history (fresh fused results only)
        readonly Vector3[] _histPos = new Vector3[(int)EvalJointId.Count];
        readonly Vector3[] _histVel = new Vector3[(int)EvalJointId.Count];
        readonly ulong[] _histTs = new ulong[(int)EvalJointId.Count];
        readonly bool[] _histHas = new bool[(int)EvalJointId.Count];
        // two-strike jump gate: pending suspicious position per joint
        readonly Vector3[] _jumpPending = new Vector3[(int)EvalJointId.Count];
        readonly bool[] _jumpHasPending = new bool[(int)EvalJointId.Count];
        readonly int[] _jumpConfirms = new int[(int)EvalJointId.Count];

        ulong _lastFusedTs;
        readonly EvalSkeletonFrame _frame = new();
        readonly EvalSkeleton _fused = new();

        /// <summary>Fixed-lag median output filter: emission is delayed by TWO
        /// fused frames (~66 ms) so each emitted joint is the component-wise
        /// median of a 5-frame window centered on the emitted frame. Excursions
        /// lasting up to TWO frames are structurally removed (they can never be
        /// the median of 5), while sustained motion passes undistorted (median of
        /// a monotone run is its middle sample). Live-viable — the only cost is
        /// the two-frame latency, negligible for trail-style visuals.</summary>
        public bool medianLagFilter = true;

        // lag ring, oldest.._lag[4]=newest (post length-projection)
        readonly EvalSkeleton[] _lag = { new EvalSkeleton(), new EvalSkeleton(), new EvalSkeleton(), new EvalSkeleton(), new EvalSkeleton() };
        readonly ulong[] _lagTs = new ulong[5];
        int _lagCount;
        readonly EvalSkeleton _emit = new();
        readonly List<float> _medScratch = new(5);

        public event Action<EvalSkeletonFrame> OnSkeletons;

        // diagnostics (per session, read for reporting)
        public int StatConsensus, StatSingleAccepted, StatRelifted, StatHeld, StatDroppedLen, StatOutliers, StatJumpHeld, StatLenProjected, StatOccDropped;
        // frame-level emit split: fresh = new fusion result, held = Heartbeat's
        // temporal-hold frame. FreshFusedHz (bench gate) counts only the former —
        // held frames keep the cadence and would fake a 30Hz FusedHz on a slow EP.
        public int StatFreshEmits, StatHeldEmits;

        // Debug probe: when dbgJoint >= 0, every fusion of that joint inside
        // [dbgFromNs, dbgToNs] appends its per-camera candidates (with stage-1
        // validity) and the decision (path, strong, jump-gate action) to DbgSb.
        // Zero cost when disabled; read/clear DbgSb from an editor script.
        public int dbgJoint = -1;
        public ulong dbgFromNs, dbgToNs;
        public readonly System.Text.StringBuilder DbgSb = new();
        bool _dbgInFuse;

        public FusedRtmposeAdapter(OrtRtmposeBackend backend)
        {
            _backend = backend;
        }

        public float ConfThreshold
        {
            get => _confThreshold;
            set { _confThreshold = value; lock (_adapterLock) foreach (var a in _perSerial.Values) a.confThreshold = value; }
        }

        public void SetWorldTransform(string serial, ObExtrinsic ext)
        {
            PerSerial(serial).SetWorldTransform(serial, ext);
            _world[serial] = new WorldTr { R = (float[])ext.Rot.Clone(), T = (float[])ext.Trans.Clone() };
        }

        public void SetCaptureVolume(Vector3 centerMm, Vector3 halfMm)
        {
            _volCenterCfg = centerMm; _volHalfCfg = halfMm; _hasVolumeCfg = true;
            lock (_adapterLock) foreach (var a in _perSerial.Values) a.SetCaptureVolume(centerMm, halfMm);
        }

        public void Configure(in EvalCameraContext ctx)
        {
            PerSerial(ctx.Serial).Configure(ctx);
            if (ctx.CameraParam.HasValue)
            {
                var cp = ctx.CameraParam.Value;
                var dr = new DepthRef
                {
                    CalFx = cp.DepthIntrinsic.Fx, CalFy = cp.DepthIntrinsic.Fy,
                    CalCx = cp.DepthIntrinsic.Cx, CalCy = cp.DepthIntrinsic.Cy,
                    CalW = cp.DepthIntrinsic.Width, CalH = cp.DepthIntrinsic.Height,
                    HasCal = cp.Transform.Rot != null && cp.Transform.Rot.Length >= 9
                             && cp.Transform.Trans != null && cp.Transform.Trans.Length >= 3,
                };
                if (dr.HasCal)
                {
                    dr.Rd2c = (float[])cp.Transform.Rot.Clone();
                    dr.Td2c = (float[])cp.Transform.Trans.Clone();
                }
                _depth[ctx.Serial] = dr;
            }
        }

        // Burst-close fusion: the HW-synced rig delivers all four cameras within
        // ~1 ms of each other (a "burst"). Fusing on the burst's FIRST frame
        // would combine one fresh sample with three previous-burst samples, so
        // fusion is deferred until the burst is complete — detected when a frame
        // arrives more than BurstGapNs later. Costs one burst (~33 ms) of
        // latency, consistent with the fixed-lag output filter.
        const ulong BurstGapNs = 5_000_000UL;
        ulong _pendingBurstTs;

        public void SubmitFrame(string serial, in RawFrameData frame, ulong tsNs)
        {
            // Close the previous burst BEFORE ingesting a frame that starts a new
            // one — otherwise this first next-burst sample (and its depth buffer)
            // would sit inside the skew window and contaminate the previous
            // burst's fusion/occlusion tests with future data.
            if (_pendingBurstTs != 0 && tsNs > _pendingBurstTs + BurstGapNs)
                FusePendingBurst();

            StashDepth(serial, frame);
            PerSerial(serial).SubmitFrame(serial, frame, tsNs); // synchronous → OnInnerSkeletons fires inside

            if (_pendingBurstTs == 0) _pendingBurstTs = tsNs;
            else if (tsNs > _pendingBurstTs) _pendingBurstTs = tsNs; // same burst, later stamp
        }

        public struct BurstItem { public string Serial; public RawFrameData Frame; public ulong TsNs; }

        /// <summary>Submit one drained wake's frames (at most one per serial),
        /// running the per-camera inference CONCURRENTLY. Semantics match calling
        /// SubmitFrame sequentially in timestamp order: items are sorted, split
        /// into burst groups (consecutive gap ≤ BurstGapNs), the pending burst is
        /// closed before a group that starts a new one, and fusion state is only
        /// touched on this (the caller's) thread. Requires Configure to have run
        /// for every serial (LiveFusedBodySource.EnsureConfigured does).</summary>
        public void SubmitBurst(List<BurstItem> items)
        {
            if (items == null || items.Count == 0) return;
            if (items.Count == 1) { SubmitFrame(items[0].Serial, items[0].Frame, items[0].TsNs); return; }
            items.Sort((a, b) => a.TsNs.CompareTo(b.TsNs));
            int start = 0;
            while (start < items.Count)
            {
                int end = start + 1;
                while (end < items.Count && items[end].TsNs - items[end - 1].TsNs <= BurstGapNs) end++;

                // sequential-submit equivalence: a close can only trigger before a
                // group's first frame — within a group each later ts is within
                // BurstGapNs of the (already advanced) pending stamp
                if (_pendingBurstTs != 0 && items[start].TsNs > _pendingBurstTs + BurstGapNs)
                    FusePendingBurst();

                RunGroupOnDedicatedThreads(items, start, end);

                for (int i = start; i < end; i++)
                {
                    StashDepth(items[i].Serial, items[i].Frame);
                    if (_pendingBurstTs == 0 || items[i].TsNs > _pendingBurstTs) _pendingBurstTs = items[i].TsNs;
                }
                start = end;
            }
        }

        // ---- dedicated inference threads ----
        // These replace a Parallel.For over the burst. The parallelism is the same
        // (one camera per thread, barrier at the end); what changes is that the
        // threads are LONG-LIVED and reused, instead of whatever thread-pool threads
        // happen to be free.
        //
        // That is a memory requirement. ORT's CUDA EP keeps a per-thread context
        // (cuBLAS/cuDNN handles + workspace) for every thread that ever calls Run and
        // never releases it, so cycling inference across the thread pool grows the
        // device arena without bound. Measured offline (4 cameras, 360 bursts, no
        // Play, no TSDF, no cameras): Parallel.For + Task.Run climbed 2.4 GB -> 11.4 GB
        // and was still rising; the same work on a fixed thread per camera sat FLAT at
        // 3.2 GB. On the 16 GB card that difference is the whole paging stall (27 fps
        // -> 1.7 fps). See Plans/handoff-rtmpose-vram-paging.md.
        sealed class BurstWorker
        {
            public Thread Thread;
            public readonly AutoResetEvent Go = new AutoResetEvent(false);
            public readonly ManualResetEventSlim Done = new ManualResetEventSlim(true);
            public volatile bool Stop;
            public FusedRtmposeAdapter Owner;
            public string Serial;
            public RawFrameData Frame;
            public ulong TsNs;
        }
        readonly List<BurstWorker> _burstWorkers = new();

        void RunGroupOnDedicatedThreads(List<BurstItem> items, int start, int end)
        {
            int n = end - start;
            while (_burstWorkers.Count < n)
            {
                var w = new BurstWorker { Owner = this };
                w.Thread = new Thread(() => BurstLoop(w)) { IsBackground = true, Name = "rtmpose-pose" };
                _burstWorkers.Add(w);
                w.Thread.Start();
            }
            for (int i = 0; i < n; i++)
            {
                var w = _burstWorkers[i];
                var it = items[start + i];
                w.Serial = it.Serial; w.Frame = it.Frame; w.TsNs = it.TsNs;
                w.Done.Reset();
                w.Go.Set();
            }
            for (int i = 0; i < n; i++) _burstWorkers[i].Done.Wait();
        }

        static void BurstLoop(BurstWorker w)
        {
            while (true)
            {
                w.Go.WaitOne();
                if (w.Stop) return;
                try { w.Owner.PerSerial(w.Serial).SubmitFrame(w.Serial, w.Frame, w.TsNs); }
                catch (Exception e) { Debug.LogException(e); }
                finally { w.Done.Set(); }
            }
        }

        /// <summary>Returns false when any worker had to be abandoned — see
        /// InferenceThreadsAbandoned.</summary>
        bool StopBurstWorkers()
        {
            bool allStopped = true;
            foreach (var w in _burstWorkers)
            {
                w.Stop = true;
                w.Go.Set();
                // bounded work (one camera's inference); abandon rather than stall
                if (w.Thread != null && !w.Thread.Join(2000)) allStopped = false;
            }
            _burstWorkers.Clear();
            return allStopped;
        }

        void FusePendingBurst()
        {
            if (_pendingBurstTs == 0) return;
            TryFuse(_pendingBurstTs);
            _pendingBurstTs = 0;
        }

        // Keep a stride-2 copy of this camera's latest depth map for visibility tests.
        void StashDepth(string serial, in RawFrameData frame)
        {
            if (!_depth.TryGetValue(serial, out var dr) || !dr.HasCal) return;
            int dw = frame.DepthWidth, dh = frame.DepthHeight;
            if (dw <= 0 || dh <= 0 || frame.DepthBytes == null || frame.DepthByteCount < dw * dh * 2) return;
            int w = dw / 2, h = dh / 2;
            if (dr.D == null || dr.W != w || dr.H != h)
            {
                dr.D = new ushort[w * h]; dr.W = w; dr.H = h;
                // calibration intrinsics are for CalW x CalH; scale to the actual
                // frame resolution, then halve for the stride-2 buffer (same
                // scaling DepthLift applies for the lift itself)
                float sx = dr.CalW > 0 ? dw / (float)dr.CalW : 1f;
                float sy = dr.CalH > 0 ? dh / (float)dr.CalH : 1f;
                dr.Fx = dr.CalFx * sx * 0.5f; dr.Cx = dr.CalCx * sx * 0.5f;
                dr.Fy = dr.CalFy * sy * 0.5f; dr.Cy = dr.CalCy * sy * 0.5f;
            }
            var src = frame.DepthBytes;
            for (int y = 0; y < h; y++)
            {
                int srcRow = (y * 2) * dw;
                int dstRow = y * w;
                for (int x = 0; x < w; x++)
                {
                    int si = (srcRow + x * 2) * 2;
                    dr.D[dstRow + x] = (ushort)(src[si] | (src[si + 1] << 8));
                }
            }
        }

        /// <summary>Can this camera see the world-space point?
        /// 1 = visible, -1 = provably occluded / out of view, 0 = unknown.</summary>
        int VisibilityOf(string serial, Vector3 world)
        {
            if (!_depth.TryGetValue(serial, out var dr) || !dr.HasCal || dr.D == null) return 0;
            if (!_world.TryGetValue(serial, out var tr)) return 0;
            // world -> color frame (inverse of color->world), then color -> depth
            var pc = InverseTransform(world, tr.R, tr.T);
            var pd = InverseTransform(pc, dr.Rd2c, dr.Td2c);
            if (pd.z <= 100f) return 0;
            int u = (int)(dr.Fx * pd.x / pd.z + dr.Cx);
            int v = (int)(dr.Fy * pd.y / pd.z + dr.Cy);
            if (u < 1 || v < 1 || u >= dr.W - 1 || v >= dr.H - 1) return -1; // out of view = cannot see
            // closest valid surface in a 3x3 neighborhood
            float minD = float.MaxValue;
            for (int dy = -1; dy <= 1; dy++)
                for (int dx = -1; dx <= 1; dx++)
                {
                    ushort d = dr.D[(v + dy) * dr.W + (u + dx)];
                    if (d > 0 && d < minD) minD = d;
                }
            if (minD == float.MaxValue) return 0; // no valid depth here
            return minD < pd.z - occlusionMarginMm ? -1 : 1;
        }

        static Vector3 InverseTransform(Vector3 p, float[] R, float[] T)
        {
            float x = p.x - T[0], y = p.y - T[1], z = p.z - T[2];
            return new Vector3(
                R[0] * x + R[3] * y + R[6] * z,
                R[1] * x + R[4] * y + R[7] * z,
                R[2] * x + R[5] * y + R[8] * z);
        }

        public void SubmitRecordedBodies(string serial, byte[] payload, int byteCount, ulong tsNs) { }
        public void Pump() { lock (_adapterLock) foreach (var a in _perSerial.Values) a.Pump(); }

        // The inner adapter runs on the shared cached backend (see
        // BtFrameInspectorWindow.SharedBackend) — disposing it here would kill
        // the backend for every other user and crash later inference natively.
        // The dedicated inference threads ARE ours, though, and a retired adapter
        // (playback-loop rebuild) must not leave them parked forever: each one
        // holds an ORT per-thread CUDA context worth hundreds of MB.
        public void Dispose()
        {
            bool stopped = StopBurstWorkers();
            lock (_adapterLock)
                foreach (var a in _perSerial.Values)
                    if (!a.StopDetectThread()) stopped = false;
            // LATCHES: Dispose can run twice (retired by RebuildAdapter, then again
            // from OnDisable). The second pass sees no threads left and would
            // otherwise clear the flag, telling the caller the backend is safe to
            // dispose when a thread is still inside ORT.
            InferenceThreadsAbandoned |= !stopped;
            if (!stopped)
                Debug.LogError($"[{nameof(FusedRtmposeAdapter)}] an inference thread did not stop " +
                               "within 2s — the caller must NOT dispose the shared backend now " +
                               "(a thread is still inside ONNX Runtime)");
        }

        /// <summary>True when Dispose() had to abandon a thread that is still inside
        /// an ORT Run. The backend MUST then be leaked rather than disposed:
        /// OrtRtmposeBackend.Dispose takes the same _detectLock a running Detect
        /// holds, so disposing would stall the calling (main) thread indefinitely —
        /// exactly the unbounded wait the abandonment exists to avoid.</summary>
        public bool InferenceThreadsAbandoned { get; private set; }

        void OnInnerSkeletons(EvalSkeletonFrame f)
        {
            var p = f.Primary();
            if (p == null) return;
            if (!_world.TryGetValue(f.Serial, out var tr)) return;
            // SubmitBurst fires this concurrently from per-camera workers; the
            // lock guards the _latest dictionary (different serials → different
            // CamSample instances, but dictionary insertion itself races).
            lock (_ingestLock)
            {
                if (!_latest.TryGetValue(f.Serial, out var s)) { s = new CamSample(); _latest[f.Serial] = s; }
                s.Ts = f.TimestampNs;
                for (int j = 0; j < (int)EvalJointId.Count; j++)
                {
                    s.Valid[j] = p.Joints[j].Valid;
                    s.Conf[j] = p.Joints[j].Confidence;
                    if (s.Valid[j]) s.World[j] = ToWorld(tr, p.Joints[j].PositionMm);
                }
            }
            OnPerCameraSkeletons?.Invoke(f);
        }

        static Vector3 ToWorld(WorldTr tr, Vector3 camMm) => new Vector3(
            tr.R[0] * camMm.x + tr.R[1] * camMm.y + tr.R[2] * camMm.z + tr.T[0],
            tr.R[3] * camMm.x + tr.R[4] * camMm.y + tr.R[5] * camMm.z + tr.T[1],
            tr.R[6] * camMm.x + tr.R[7] * camMm.y + tr.R[8] * camMm.z + tr.T[2]);

        // ---- fusion ----

        readonly List<CamSample> _work = new();
        readonly List<string> _workSerial = new();
        // per-sample effective validity after Stage 1 (parallel to _work)
        readonly List<bool[]> _workValid = new();

        void TryFuse(ulong nowTs)
        {
            if (_lastFusedTs != 0)
            {
                // drop out-of-order/duplicate triggers so _lastFusedTs (and the
                // emitted stream) is strictly monotonic
                if (nowTs <= _lastFusedTs) return;
                if (nowTs - _lastFusedTs < (ulong)(minFuseIntervalMs * 1e6f)) return;
            }

            _work.Clear(); _workSerial.Clear(); _workValid.Clear();
            ulong skewNs = (ulong)(skewMs * 1e6f);
            foreach (var kv in _latest)
            {
                ulong d = kv.Value.Ts > nowTs ? kv.Value.Ts - nowTs : nowTs - kv.Value.Ts;
                if (d > skewNs) continue;
                _work.Add(kv.Value); _workSerial.Add(kv.Key);
            }
            if (_work.Count == 0)
            {
                // No fresh camera sample this beat (person selection momentarily
                // failed on every camera) — emit the temporal hold instead of
                // skipping a frame.
                EmitHeldFrame(nowTs);
                return;
            }
            _lastFusedTs = nowTs;

            // Stage 1: per-camera bone-length sanity (blame the distal joint).
            foreach (var s in _work)
            {
                var v = new bool[(int)EvalJointId.Count];
                Array.Copy(s.Valid, v, v.Length);
                if (Profile != null)
                {
                    for (int b = 0; b < Bones.Length; b++)
                    {
                        float L = Profile.LengthMm[b];
                        if (L <= 0f) continue;
                        int ia = (int)Bones[b].a, ib = (int)Bones[b].b;
                        if (!v[ia] || !v[ib]) continue;
                        float len = Vector3.Distance(s.World[ia], s.World[ib]);
                        if (Mathf.Abs(len - L) > Mathf.Max(lenRelTol * L, lenAbsTolMm))
                        {
                            v[ib] = false; // distal blame
                            StatDroppedLen++;
                        }
                    }
                }
                _workValid.Add(v);
            }

            _fused.Reset(1, nowTs);
            var fusedPos = new Vector3[(int)EvalJointId.Count];
            var fusedOk = new bool[(int)EvalJointId.Count];
            var fusedFresh = new bool[(int)EvalJointId.Count];

            // enum order = kinematic order (parents first)
            for (int j = 0; j < (int)EvalJointId.Count; j++)
            {
                // gather candidates
                var pts = new List<Vector3>(); var wts = new List<float>(); var srcSample = new List<int>();
                for (int i = 0; i < _work.Count; i++)
                {
                    if (!_workValid[i][j]) continue;
                    pts.Add(_work[i].World[j]); wts.Add(Mathf.Max(0.05f, _work[i].Conf[j])); srcSample.Add(i);
                }

                bool dbg = dbgJoint == j && nowTs >= dbgFromNs && nowTs <= dbgToNs;
                if (dbg)
                {
                    DbgSb.Append(nowTs).Append(' ');
                    for (int i = 0; i < _work.Count; i++)
                    {
                        var w = _work[i].World[j];
                        DbgSb.Append(_workSerial[i].Substring(_workSerial[i].Length - 2))
                             .Append(_work[i].Valid[j] ? (_workValid[i][j] ? "" : "!len") : "!inv")
                             .Append('(').Append(w.x.ToString("F0")).Append(',')
                             .Append(w.y.ToString("F0")).Append(',').Append(w.z.ToString("F0"))
                             .Append(")c").Append(_work[i].Conf[j].ToString("F2")).Append(' ');
                    }
                }

                float conf; Vector3 pos; bool fresh, strong;
                _dbgInFuse = dbg;
                if (FuseJoint(j, pts, wts, srcSample, out pos, out conf, out fresh, out strong))
                {
                    // Jump gate: a fresh position far from recent history is
                    // suspicious (spike). Hold it back; acceptance depends on the
                    // evidence tier — a cross-camera cluster (strong) accepts on
                    // the second consecutive frame near the pending position or
                    // on direction continuation, while weak evidence (single
                    // camera / re-lift) needs weakJumpConfirms agreeing frames,
                    // because one wrong camera repeats the same wrong position
                    // frame after frame and would otherwise self-confirm.
                    if (fresh && _histHas[j])
                    {
                        float age = AgeSec(nowTs, _histTs[j]);
                        if (age <= holdMaxSeconds
                            && Vector3.Distance(pos, _histPos[j]) > jumpGateMm)
                        {
                            bool accept = false;
                            if (_jumpHasPending[j])
                            {
                                if (Vector3.Distance(pos, _jumpPending[j]) <= jumpGateMm)
                                {
                                    _jumpConfirms[j]++;
                                    accept = _jumpConfirms[j] >= (strong ? 1 : weakJumpConfirms);
                                }
                                else if (strong)
                                {
                                    // Direction continuation — genuine fast motion
                                    // keeps moving the same way each frame (the
                                    // pending point itself has moved on), while an
                                    // out-and-back spike reverses direction (dot < 0).
                                    // Trusted only with cross-camera agreement.
                                    var stepA = _jumpPending[j] - _histPos[j];
                                    var stepB = pos - _jumpPending[j];
                                    if (stepA.sqrMagnitude > 1f && stepB.sqrMagnitude > 1f
                                        && Vector3.Dot(stepA.normalized, stepB.normalized) > 0.3f)
                                        accept = true;
                                    if (!accept) _jumpConfirms[j] = 0;
                                }
                                else _jumpConfirms[j] = 0;
                            }
                            if (accept)
                            {
                                _jumpHasPending[j] = false; _jumpConfirms[j] = 0;
                            }
                            else
                            {
                                _jumpPending[j] = pos; _jumpHasPending[j] = true;
                                pos = HeldPrediction(j, age);
                                conf = 0.25f; fresh = false;
                                StatJumpHeld++;
                            }
                        }
                        else { _jumpHasPending[j] = false; _jumpConfirms[j] = 0; }
                    }
                    else { _jumpHasPending[j] = false; _jumpConfirms[j] = 0; }

                    if (dbg)
                        DbgSb.Append("=> (").Append(pos.x.ToString("F0")).Append(',')
                             .Append(pos.y.ToString("F0")).Append(',').Append(pos.z.ToString("F0"))
                             .Append(") fresh=").Append(fresh).Append(" strong=").Append(strong)
                             .Append(" pend=").Append(_jumpHasPending[j]).Append('\n');

                    fusedPos[j] = pos; fusedOk[j] = true; fusedFresh[j] = fresh;
                    ref var oj = ref _fused.Joints[j];
                    oj.PositionMm = pos; oj.Confidence = conf; oj.Valid = true;
                }
                else if (dbg) DbgSb.Append("=> none\n");
            }

            // Final pass: calibrated bone lengths are HARD output constraints.
            // A violation beyond tolerance projects the CHILD onto the sphere of
            // the calibrated length around its (already corrected) parent —
            // direction is kept, length is enforced. This is the only guard that
            // catches an ALL-cameras-agree-but-wrong systematic bias: e.g. the
            // hair-flip pose (face invisible from every camera) collapses the
            // head into the neck on every view, so consensus happily passes it,
            // and the collapse onsets gradually enough to ride inside the
            // per-frame gates. Bones is parent-first, so corrections chain.
            if (Profile != null)
            {
                for (int b = 0; b < Bones.Length; b++)
                {
                    float L = Profile.LengthMm[b];
                    if (L <= 0f) continue;
                    int ia = (int)Bones[b].a, ib = (int)Bones[b].b;
                    if (!fusedOk[ia] || !fusedOk[ib]) continue;
                    var dir = fusedPos[ib] - fusedPos[ia];
                    float len = dir.magnitude;
                    if (Mathf.Abs(len - L) <= Mathf.Max(lenRelTol * L, lenAbsTolMm)) continue;
                    if (len < 1f) { _fused.Joints[ib].Valid = false; fusedOk[ib] = false; continue; }
                    var np = fusedPos[ia] + dir * (L / len);
                    fusedPos[ib] = np;
                    _fused.Joints[ib].PositionMm = np;
                    _fused.Joints[ib].Confidence *= 0.8f;
                    StatLenProjected++;
                }
            }

            // temporal history update (fresh joints only — predicted joints must
            // not feed back into their own velocity estimate)
            for (int j = 0; j < (int)EvalJointId.Count; j++)
            {
                if (!fusedOk[j] || !fusedFresh[j]) continue;
                if (_histHas[j] && nowTs > _histTs[j])
                {
                    float dt = (nowTs - _histTs[j]) / 1e9f;
                    if (dt > 1e-4f)
                    {
                        var rawVel = (fusedPos[j] - _histPos[j]) / dt;
                        // spike-derived velocities must never seed the hold
                        // extrapolation (runaway streak); keep the previous
                        // estimate when the implied speed is superhuman.
                        if (rawVel.magnitude <= velMaxMps * 1000f)
                            _histVel[j] = Vector3.Lerp(rawVel, _histVel[j], velSmoothing);
                    }
                }
                else _histVel[j] = Vector3.zero;
                _histPos[j] = fusedPos[j]; _histTs[j] = nowTs; _histHas[j] = true;
            }

            StatFreshEmits++;
            PushLagAndEmit(nowTs);
        }

        /// <summary>Heartbeat: emit a temporal-hold frame when the cameras went
        /// silent (recorded frame drop, live sensor hiccup). Keeps the output
        /// cadence steady so downstream motion curves never bridge a hole with a
        /// straight segment (visible kink). Safe to call any time — no-op inside
        /// the normal fuse interval or when nothing recent enough to hold.</summary>
        public void Heartbeat(ulong nowTs)
        {
            // close a burst the heartbeat is jumping past, so its samples fuse
            // before the held frame lands after them
            if (_pendingBurstTs != 0 && nowTs > _pendingBurstTs + BurstGapNs) FusePendingBurst();
            if (_lastFusedTs != 0)
            {
                if (nowTs <= _lastFusedTs) return; // keep the stream monotonic
                if (nowTs - _lastFusedTs < (ulong)(minFuseIntervalMs * 1e6f)) return;
            }
            EmitHeldFrame(nowTs);
        }

        // Emit the per-joint temporal-hold prediction as a full frame. Held
        // joints do NOT update the history (no feedback into velocity).
        void EmitHeldFrame(ulong nowTs)
        {
            _lastFusedTs = nowTs;
            _fused.Reset(1, nowTs);
            int heldJoints = 0;
            for (int j = 0; j < (int)EvalJointId.Count; j++)
            {
                if (!_histHas[j]) continue;
                float age = AgeSec(nowTs, _histTs[j]);
                if (age > holdMaxSeconds) continue;
                ref var oj = ref _fused.Joints[j];
                oj.PositionMm = HeldPrediction(j, age);
                oj.Confidence = 0.2f;
                oj.Valid = true;
                heldJoints++;
            }
            if (heldJoints == 0) return;
            StatHeld += heldJoints;
            StatHeldEmits++;
            ProjectBoneLengths(_fused);
            PushLagAndEmit(nowTs);
        }

        void PushLagAndEmit(ulong nowTs)
        {
            if (!medianLagFilter)
            {
                _frame.Reset(Name, "fused", nowTs);
                _frame.Bodies.Add(_fused);
                OnSkeletons?.Invoke(_frame);
                return;
            }

            // push into the lag ring (oldest.._lag[4]=newest)
            if (_lagCount >= 5)
            {
                var tmp = _lag[0];
                for (int i = 0; i < 4; i++) { _lag[i] = _lag[i + 1]; _lagTs[i] = _lagTs[i + 1]; }
                _lag[4] = tmp;
            }
            int newest = Mathf.Min(_lagCount, 4);
            CopySkeleton(_fused, _lag[newest]);
            _lagTs[newest] = nowTs;
            if (_lagCount < 5) _lagCount++;

            // emit index len-3 (fixed lag of two frames once warm); window = the
            // whole current ring (3..5 samples)
            if (_lagCount < 3) return;
            int emitIdx = _lagCount - 3;
            CopySkeleton(_lag[emitIdx], _emit);
            for (int j = 0; j < (int)EvalJointId.Count; j++)
            {
                bool all = true;
                for (int i = 0; i < _lagCount && all; i++) all = _lag[i].Joints[j].Valid;
                if (!all) continue;
                _emit.Joints[j].PositionMm = new Vector3(
                    MedN(j, 0), MedN(j, 1), MedN(j, 2));
            }
            ProjectBoneLengths(_emit); // median mixing can bend a length back out of tolerance
            EmitSkeleton(_emit, _lagTs[emitIdx]);
        }

        // component-wise median over the current lag window (axis: 0=x 1=y 2=z)
        float MedN(int j, int axis)
        {
            _medScratch.Clear();
            for (int i = 0; i < _lagCount; i++)
            {
                var p = _lag[i].Joints[j].PositionMm;
                _medScratch.Add(axis == 0 ? p.x : axis == 1 ? p.y : p.z);
            }
            _medScratch.Sort();
            int n = _medScratch.Count;
            return (n & 1) == 1 ? _medScratch[n / 2]
                                : 0.5f * (_medScratch[n / 2 - 1] + _medScratch[n / 2]);
        }

        /// <summary>Emit the frames still waiting in the fixed-lag ring (raw — the
        /// median window is incomplete for them). For offline export end-of-stream;
        /// a live session never ends so it never calls this.</summary>
        public void FlushLag()
        {
            FusePendingBurst(); // the final burst never sees a successor
            if (!medianLagFilter) return;
            for (int i = Mathf.Max(0, _lagCount - 2); i < _lagCount; i++)
                EmitSkeleton(_lag[i], _lagTs[i]);
            _lagCount = 0;
        }

        void EmitSkeleton(EvalSkeleton s, ulong ts)
        {
            _frame.Reset(Name, "fused", ts);
            _frame.Bodies.Add(s);
            OnSkeletons?.Invoke(_frame);
        }

        static void CopySkeleton(EvalSkeleton src, EvalSkeleton dst)
        {
            dst.Reset(src.PersonId, src.TimestampNs);
            for (int j = 0; j < (int)EvalJointId.Count; j++) dst.Joints[j] = src.Joints[j];
        }

        static float Med3(float a, float b, float c) =>
            Mathf.Max(Mathf.Min(a, b), Mathf.Min(Mathf.Max(a, b), c));

        void ProjectBoneLengths(EvalSkeleton s)
        {
            if (Profile == null) return;
            for (int b = 0; b < Bones.Length; b++)
            {
                float L = Profile.LengthMm[b];
                if (L <= 0f) continue;
                int ia = (int)Bones[b].a, ib = (int)Bones[b].b;
                if (!s.Joints[ia].Valid || !s.Joints[ib].Valid) continue;
                var dir = s.Joints[ib].PositionMm - s.Joints[ia].PositionMm;
                float len = dir.magnitude;
                if (Mathf.Abs(len - L) <= Mathf.Max(lenRelTol * L, lenAbsTolMm)) continue;
                if (len < 1f) { s.Joints[ib].Valid = false; continue; }
                s.Joints[ib].PositionMm = s.Joints[ia].PositionMm + dir * (L / len);
                StatLenProjected++;
            }
        }

        bool FuseJoint(int j, List<Vector3> pts, List<float> wts, List<int> srcSample,
                       out Vector3 pos, out float conf, out bool fresh, out bool strong)
        {
            pos = default; conf = 0f; fresh = false; strong = false;

            // NOTE: the occlusion z-test used to run HERE, before clustering,
            // and that dropped the CORRECT camera in the mirror-pair failure:
            // when the person faces away, two cameras flip the right wrist onto
            // the left hand — a real, visible surface — while a correct camera
            // is genuinely blind to that spot (it is across the body), and
            // silhouette-edge depth makes the flipped pair look "sighted" at
            // the true position. Blindness to a rival's location is not
            // evidence against one's own claim, so the z-test must never veto
            // clustering; it now arbitrates only among isolated candidates in
            // the weak path below (its original 1-vs-1 hidden-joint use).

            // Cluster consensus: the per-axis component median is not robust
            // when half the cameras are wrong — two bad samples drag the median
            // between the clusters, so the cameras that actually AGREE get
            // rejected as outliers of the poisoned median (seen live as a head
            // side-flip while standing still: one camera lifted the head onto
            // the background wall, another was meters off, and the two correct
            // cameras got gated out). With <=4 cameras the exact answer is
            // cheap: enumerate subsets and take the largest mutually-consistent
            // cluster (all pairwise distances within the joint gate). A cluster
            // of >=2 is STRONG evidence; everything below is weak.
            int n = pts.Count;
            if (n >= 2)
            {
                // Equal-count cluster ties break on TEMPORAL CONTINUITY, not
                // confidence: when the person faces away, two cameras can
                // mirror-flip the same limb the same way (both put the right
                // wrist on the left hand) and form a perfectly valid-looking
                // 2-camera cluster AGAINST the 2 correct cameras — and
                // confidence is blind to that (it never sees 3D error). The
                // joint history disambiguates decisively: the true cluster
                // continues the track, the flipped one is a bone-length away.
                bool hasPred = false; Vector3 pred = Vector3.zero;
                {
                    float pAge = AgeSec(_lastFusedTs, _histTs[j]);
                    if (_histHas[j] && pAge <= holdMaxSeconds) { pred = HeldPrediction(j, pAge); hasPred = true; }
                }
                int bestMask = 0, bestCount = 0; float bestWsum = 0f, bestPredD = float.MaxValue;
                for (int mask = 3; mask < (1 << n); mask++)
                {
                    int cnt = 0; float msum = 0f; bool ok = true;
                    Vector3 macc = Vector3.zero;
                    for (int a = 0; a < n && ok; a++)
                    {
                        if ((mask & (1 << a)) == 0) continue;
                        cnt++; msum += wts[a]; macc += pts[a] * wts[a];
                        for (int b = a + 1; b < n && ok; b++)
                            if ((mask & (1 << b)) != 0 && Vector3.Distance(pts[a], pts[b]) > gateMm[j])
                                ok = false;
                    }
                    if (!ok || cnt < 2) continue;
                    float predD = hasPred ? Vector3.Distance(macc / msum, pred) : 0f;
                    bool better;
                    if (cnt != bestCount) better = cnt > bestCount;
                    else if (hasPred) better = predD < bestPredD;
                    else better = msum > bestWsum;
                    if (better)
                    { bestMask = mask; bestCount = cnt; bestWsum = msum; bestPredD = predD; }
                }
                if (_dbgInFuse)
                    DbgSb.Append("[cluster n=").Append(bestCount).Append(" mask=").Append(bestMask)
                         .Append(" predD=").Append(hasPred ? bestPredD.ToString("F0") : "nopred").Append("] ");
                if (bestCount >= 2)
                {
                    // Loyalty demotion: a PAIR that jumps a bone-length off the
                    // predicted track while some other candidate stays on it is
                    // the mirror-pair signature (two cameras flipping the same
                    // limb the same way still out-count one correct camera when
                    // its partner dropped out for a frame). Trust the track:
                    // send the far pair down the weak path, whose prediction-
                    // ordered vetting picks the loyal candidate instead. A
                    // cluster of >=3 is never demoted — three cameras agreeing
                    // on a jump is genuine fast motion.
                    bool demoted = false;
                    if (hasPred && bestCount <= 2 && bestPredD > jumpGateMm)
                    {
                        for (int i = 0; i < n && !demoted; i++)
                        {
                            if ((bestMask & (1 << i)) != 0) continue;
                            if (Vector3.Distance(pts[i], pred) <= jumpGateMm && PassesParentLength(j, pts[i]))
                                demoted = true;
                        }
                        if (_dbgInFuse && demoted) DbgSb.Append("[demote] ");
                    }
                    if (!demoted)
                    {
                        Vector3 acc = Vector3.zero; float wsum = 0f, cmax = 0f;
                        for (int i = 0; i < n; i++)
                        {
                            if ((bestMask & (1 << i)) == 0) { StatOutliers++; continue; }
                            acc += pts[i] * wts[i]; wsum += wts[i]; cmax = Mathf.Max(cmax, wts[i]);
                        }
                        pos = acc / wsum; conf = cmax; fresh = true; strong = true; StatConsensus++;
                        return true;
                    }
                }
            }

            // No trusted cluster — weak-evidence path.
            {
                float predAge = AgeSec(_lastFusedTs, _histTs[j]);
                bool predFresh = _histHas[j] && predAge <= holdMaxSeconds;

                // Occlusion arbitration between DISAGREEING isolated samples:
                // if camera C provably cannot see the place camera A puts the
                // joint (A's position is behind C's measured surface, or
                // outside C's view), while A CAN see C's claimed location and
                // still disagrees, then C was guessing an occluded joint
                // (e.g. a hand fully hidden behind the torso) — drop C's
                // sample so the sighted camera wins. Runs only WITHOUT fresh
                // history, after clustering: silhouette-edge depth can fake
                // the "sighted" verdict (and did — it vetoed the correct
                // camera during a mirror-pair flip), so whenever the track can
                // arbitrate, the track wins; the z-test only breaks cold-start
                // disagreements.
                if (!predFresh && pts.Count >= 2)
                {
                    for (int i = pts.Count - 1; i >= 0 && pts.Count > 1; i--)
                    {
                        bool dropI = false;
                        for (int k = 0; k < pts.Count && !dropI; k++)
                        {
                            if (k == i) continue;
                            if (Vector3.Distance(pts[i], pts[k]) <= gateMm[j]) continue;
                            string si = _workSerial[srcSample[i]];
                            string sk = _workSerial[srcSample[k]];
                            if (VisibilityOf(si, pts[k]) == -1 && VisibilityOf(sk, pts[i]) == 1
                                && wts[k] >= wts[i] * 0.7f)
                                dropI = true;
                        }
                        if (dropI)
                        {
                            if (_dbgInFuse) DbgSb.Append("[occdrop ").Append(_workSerial[srcSample[i]].Substring(_workSerial[srcSample[i]].Length - 2)).Append("] ");
                            pts.RemoveAt(i); wts.RemoveAt(i); srcSample.RemoveAt(i);
                            StatOccDropped++;
                        }
                    }
                }

                // Order the candidates by trust (proximity to the temporal
                // prediction when history is fresh, else confidence) so a
                // grossly wrong camera can never win by iteration order, then
                // vet each: direct accept on a sane bone length to the fused
                // parent, else ray x bone-length re-lift.
                n = pts.Count;
                if (n > 0)
                {
                    var order = new List<int>(n);
                    for (int i = 0; i < n; i++) order.Add(i);
                    if (predFresh)
                    {
                        var pred = HeldPrediction(j, predAge);
                        order.Sort((a, b) => Vector3.Distance(pts[a], pred).CompareTo(Vector3.Distance(pts[b], pred)));
                    }
                    else
                    {
                        order.Sort((a, b) => wts[b].CompareTo(wts[a]));
                    }

                    foreach (int i in order)
                    {
                        if (!PassesParentLength(j, pts[i])) continue;
                        pos = pts[i]; conf = wts[i] * 0.9f; fresh = true; StatSingleAccepted++;
                        return true;
                    }
                    // Stage 3a: ray x bone-length re-lift (needs fused parent + profile)
                    foreach (int i in order)
                    {
                        if (TryRayRelift(j, pts[i], srcSample[i], out var relift))
                        {
                            pos = relift; conf = wts[i] * 0.7f; fresh = true; StatRelifted++;
                            return true;
                        }
                    }
                }
            }

            // Stage 3b: temporal hold
            if (_histHas[j])
            {
                float age = AgeSec(_lastFusedTs, _histTs[j]);
                if (age <= holdMaxSeconds)
                {
                    pos = HeldPrediction(j, age);
                    conf = 0.2f; fresh = false; StatHeld++;
                    return true;
                }
            }
            return false;
        }

        // Age in seconds with ulong-underflow safety: an out-of-order pair (then
        // in the future) reads as "expired" (MaxValue), never as a huge valid age.
        static float AgeSec(ulong now, ulong then) =>
            now >= then ? (now - then) / 1e9f : float.MaxValue;

        // Clamped extrapolation from the joint history: velocity applies for at
        // most maxPredictSeconds AND at most holdMaxStepMm of displacement, so a
        // bad velocity can never draw a long straight streak.
        Vector3 HeldPrediction(int j, float age)
        {
            float dt = Mathf.Clamp(age, 0f, maxPredictSeconds);
            var disp = _histVel[j] * dt;
            if (disp.magnitude > holdMaxStepMm) disp = disp.normalized * holdMaxStepMm;
            return _histPos[j] + disp;
        }

        bool PassesParentLength(int j, Vector3 world)
        {
            if (Profile == null) return true;
            int b = BoneOfChild[j];
            if (b < 0) return true;
            float L = Profile.LengthMm[b];
            if (L <= 0f) return true;
            int parent = (int)Bones[b].a;
            if (!_fused.Joints[parent].Valid) return true; // nothing to check against
            float len = Vector3.Distance(world, _fused.Joints[parent].PositionMm);
            return Mathf.Abs(len - L) <= Mathf.Max(lenRelTol * L, lenAbsTolMm);
        }

        bool TryRayRelift(int j, Vector3 observedWorld, int sampleIdx, out Vector3 result)
        {
            result = default;
            if (Profile == null) return false;
            int b = BoneOfChild[j];
            if (b < 0) return false;
            float L = Profile.LengthMm[b];
            if (L <= 0f) return false;
            int parent = (int)Bones[b].a;
            if (!_fused.Joints[parent].Valid) return false;
            if (!_world.TryGetValue(_workSerial[sampleIdx], out var tr)) return false;

            // camera origin in world = T; ray through the observed sample
            var o = new Vector3(tr.T[0], tr.T[1], tr.T[2]);
            var d = observedWorld - o;
            float dist = d.magnitude;
            if (dist < 1f) return false;
            d /= dist;

            var P = _fused.Joints[parent].PositionMm;
            // |o + t·d − P|² = L² → t² − 2t·m + (|oP|² − L²) = 0, m = dot(P−o, d)
            var oP = P - o;
            float m = Vector3.Dot(oP, d);
            float c = oP.sqrMagnitude - L * L;
            float disc = m * m - c;
            Vector3 candidate;
            if (disc >= 0f)
            {
                float sq = Mathf.Sqrt(disc);
                float t1 = m - sq, t2 = m + sq;
                // prefer the root closest to the OBSERVED depth (the ray is
                // trusted; the depth is what broke)
                float tBest = Mathf.Abs(t1 - dist) <= Mathf.Abs(t2 - dist) ? t1 : t2;
                if (tBest <= 0f) tBest = Mathf.Max(t1, t2);
                if (tBest <= 0f) return false;
                candidate = o + d * tBest;
            }
            else
            {
                // ray misses the sphere: closest ray point, pulled onto the sphere
                var closest = o + d * Mathf.Max(0f, m);
                var dir = closest - P;
                if (dir.sqrMagnitude < 1f) return false;
                candidate = P + dir.normalized * L;
            }

            // sanity: don't teleport across the scene relative to prediction
            if (_histHas[j])
            {
                float age = AgeSec(_lastFusedTs, _histTs[j]);
                if (age <= holdMaxSeconds)
                {
                    var pred = _histPos[j] + _histVel[j] * Mathf.Min(age, maxPredictSeconds);
                    if (Vector3.Distance(candidate, pred) > 2f * gateMm[j]) return false;
                }
            }
            result = candidate;
            return true;
        }

    }
}
