// Track B adapter: RTMPose (top-down) + depth back-projection, normalized into
// the common EvalSkeleton. Inference is synchronous, so OnSkeletons is raised
// inside SubmitFrame and measured latency includes detect+pose+lift.
//
// Person selection uses a WORLD capture volume (single-person, known space):
// detections are filtered to the person whose box-center 3D world position is
// inside the volume, rejecting bystanders.
//
// Latency: YOLOX detection dominates (~126ms vs ~20ms RTMPose). So we DETECT
// only to (re)acquire the person, then TRACK by deriving the next frame's box
// from the current keypoints — re-detecting every redetectEveryN frames, when
// pose confidence drops, or when the tracked person leaves the volume. Most
// frames skip YOLOX entirely (~6x faster).
//
// Requires a ready IRtmposeBackend and per-camera ObCameraParam for the 3D lift.

using System;
using System.Collections.Generic;
using Orbbec;
using PointCloud;
using UnityEngine;

namespace BodyTracking.Eval.Rtmpose
{
    public sealed class RtmPoseAdapter : ITrackerAdapter
    {
        public string Name => "rtmpose";
        public event Action<EvalSkeletonFrame> OnSkeletons;

        [Range(0f, 1f)] public float confThreshold = 0.3f;
        public int depthWindowHalf = 3;      // 7x7 median
        public int redetectEveryN = 30;      // safety re-detection interval
        public float minTrackConf = 0.35f;   // below this mean keypoint score, re-detect
        public float trackPad = 0.15f;       // keypoint-bbox padding fraction for the next box

        /// <summary>Run detection on a background thread instead of blocking the
        /// submit call. LIVE ONLY: a 40-56ms YOLOX pass inside the burst barrier
        /// stalls all four cameras exactly at hard-motion moments (frequent track
        /// loss), overflowing the frame ring — the visible "shape collapse"
        /// (eval/results/live_v11s_*). While a detect is pending the camera skips
        /// its frames (fusion continues on the others) and adopts the box on
        /// completion. Offline exports keep this OFF: skipping frames there would
        /// change deterministic output for no gain (they have no time budget).</summary>
        public bool asyncDetect = false;

        /// <summary>asyncDetect only: after losing the track, keep using the last
        /// box for up to this many frames while the background detect runs.
        /// DEFAULT 0 (off): measured on 15-50-24, riding a stale box across a
        /// fast turn produced garbage poses with passable confidence (2.5m
        /// shoulder spikes, pelvis accel x1.8) — worse than sitting the frames
        /// out. Kept for experimentation.</summary>
        public int trackGraceFrames = 0;

        private readonly IRtmposeBackend _backend;
        private readonly DepthLift _lift = new DepthLift();
        private readonly Dictionary<string, ObCameraParam?> _cam = new Dictionary<string, ObCameraParam?>();
        private readonly Dictionary<string, ObExtrinsic> _colorToWorld = new Dictionary<string, ObExtrinsic>();

        private Vector3 _volCenter, _volHalf;
        private bool _hasVolume;

        private struct Track { public float X1, Y1, X2, Y2; public int Age; public bool Valid; public int Grace; }
        private readonly Dictionary<string, Track> _track = new Dictionary<string, Track>();

        private const int MaxDet = 8;
        private readonly DetBox[] _boxes = new DetBox[MaxDet];
        private float[] _input, _simccX, _simccY;
        private readonly Vector2[] _coco;
        private readonly float[] _cocoScore;
        private readonly Vector2[] _xy = new Vector2[EvalSkeleton.JointCount];
        private readonly float[] _score = new float[EvalSkeleton.JointCount];
        private readonly bool[] _valid = new bool[EvalSkeleton.JointCount];
        private readonly EvalSkeleton _skel = new EvalSkeleton();
        private readonly EvalSkeletonFrame _frame = new EvalSkeletonFrame();

        // Per-stage timing (worker thread; bench probe reads snapshots — a torn
        // read only skews one sample). Ring of the last TimingCap per-call ms.
        public const int TimingCap = 4096;
        public readonly float[] LiftMs = new float[TimingCap];
        public readonly float[] DetectMs = new float[TimingCap];
        public readonly float[] PreMs = new float[TimingCap];
        public readonly float[] PoseMs = new float[TimingCap];
        public int LiftN, DetectN, PreN, PoseN; // total counts (ring index = N % cap)
        public int StatTrackCalls, StatDetectCalls, StatNoDetect;
        public int StatAsyncKicks, StatAsyncSkippedFrames; // async-detect diagnostics

        // ---- async detect state ----
        // Owner thread = the adapter's single submit thread; the detect job runs
        // on the thread pool and touches ONLY its own copies. Handoff via
        // _asyncState (volatile): 0=idle (worker owns), 1=running (pool owns),
        // 2=done (worker adopts, then resets to 0).
        private byte[] _detColorCopy = System.Array.Empty<byte>();
        private readonly DetBox[] _asyncBoxes = new DetBox[MaxDet];
        private volatile int _asyncState;
        private int _asyncCount, _asyncW, _asyncH;
        private volatile bool _detectFailLogged;

        private void KickAsyncDetect(byte[] rgb, int w, int h)
        {
            if (_asyncState != 0) return; // one in flight per camera
            int bytes = w * h * 3;
            if (_detColorCopy.Length < bytes) _detColorCopy = new byte[bytes];
            Buffer.BlockCopy(rgb, 0, _detColorCopy, 0, bytes);
            _asyncW = w; _asyncH = h;
            StatAsyncKicks++;
            _asyncState = 1;
            System.Threading.Tasks.Task.Run(() =>
            {
                long t0 = System.Diagnostics.Stopwatch.GetTimestamp();
                try { _asyncCount = _backend.Detect(_detColorCopy, _asyncW, _asyncH, _asyncBoxes); }
                catch (Exception e)
                {
                    _asyncCount = 0;
                    // A swallowed detect failure is indistinguishable from "nobody is
                    // in frame": both just bump StatNoDetect, and fusion silently
                    // emits nothing forever. That cost a whole debugging session
                    // once (a CUDA arena cap made EVERY Detect throw on the input
                    // tensor while the counters read like an empty room), so failures
                    // are now visible. Logged ONCE per adapter: this runs per frame
                    // per camera, and a repeating native error would flood the
                    // console faster than it could be read.
                    if (!_detectFailLogged)
                    {
                        _detectFailLogged = true;
                        Debug.LogError($"[rtmpose] async detect failed (further failures on this " +
                                       $"camera are silent; StatNoDetect keeps counting them): {e}");
                    }
                }
                finally
                {
                    DetectMs[DetectN++ % TimingCap] = MsSince(t0);
                    _asyncState = 2;
                }
            });
        }

        static float MsSince(long t0) => (System.Diagnostics.Stopwatch.GetTimestamp() - t0) * 1000f / System.Diagnostics.Stopwatch.Frequency;

        public RtmPoseAdapter(IRtmposeBackend backend = null)
        {
            _backend = backend ?? new NullRtmposeBackend();
            int k = _backend.NumKeypoints;
            _coco = new Vector2[k];
            _cocoScore = new float[k];
            var spec = _backend.Spec;
            _input = new float[3 * spec.InH * spec.InW];
            _simccX = new float[k * Mathf.RoundToInt(spec.InW * spec.SplitRatio)];
            _simccY = new float[k * Mathf.RoundToInt(spec.InH * spec.SplitRatio)];
        }

        public void Configure(in EvalCameraContext ctx) => _cam[ctx.Serial] = ctx.CameraParam;
        public void SetCaptureVolume(Vector3 centerMm, Vector3 halfExtentsMm) { _volCenter = centerMm; _volHalf = halfExtentsMm; _hasVolume = true; }
        public void SetWorldTransform(string serial, ObExtrinsic colorToWorld) => _colorToWorld[serial] = colorToWorld;
        public void SubmitRecordedBodies(string serial, byte[] payload, int byteCount, ulong tsNs) { }

        public void SubmitFrame(string serial, in RawFrameData frame, ulong tsNs)
        {
            if (!_backend.Ready) return;
            int cw = frame.ColorWidth, ch = frame.ColorHeight;
            // Require a FULL RGB payload: on a truncated/skipped color record the
            // driver reuses its buffer with a short/zero ColorByteCount, and running
            // inference on stale tail data would corrupt the metrics.
            if (frame.ColorBytes == null || cw <= 0 || ch <= 0 ||
                frame.ColorByteCount < cw * ch * 3) return;

            ObCameraParam? cam = _cam.TryGetValue(serial, out var c) ? c : null;
            // Guard on the ACTUAL byte count: the replay driver reuses its depth
            // buffer, so a skipped/truncated record (DepthByteCount==0) would
            // otherwise lift against the PREVIOUS frame's stale depth.
            bool depthValid = frame.DepthBytes != null &&
                              frame.DepthByteCount >= frame.DepthWidth * frame.DepthHeight * 2;
            long t0 = System.Diagnostics.Stopwatch.GetTimestamp();
            bool have3d = cam.HasValue && depthValid &&
                          _lift.BuildAligned(frame.DepthBytes, frame.DepthWidth, frame.DepthHeight, cw, ch, cam.Value);
            LiftMs[LiftN++ % TimingCap] = MsSince(t0);

            // detect (to acquire) vs track (reuse previous keypoint box)
            DetBox box = default; bool tracking = false;
            bool haveTrack = _track.TryGetValue(serial, out var tr) && tr.Valid;

            if (asyncDetect)
            {
                bool adopted = false;
                // adopt a finished background detect; person selection runs HERE,
                // against the CURRENT frame's depth lift (fresher than the detect's
                // own source frame — the padded pose crop absorbs the small drift)
                if (_asyncState == 2)
                {
                    int nDet = Mathf.Min(_asyncCount, MaxDet);
                    if (nDet > 0) Array.Copy(_asyncBoxes, _boxes, nDet);
                    int sel = nDet > 0 ? SelectPerson(nDet, serial, cam, have3d, cw, ch) : -1;
                    _asyncState = 0;
                    if (nDet <= 0) StatNoDetect++;
                    if (sel >= 0) { box = _boxes[sel]; adopted = true; StatDetectCalls++; }
                }
                if (!adopted)
                {
                    if (haveTrack)
                    {
                        box = new DetBox { X1 = tr.X1, Y1 = tr.Y1, X2 = tr.X2, Y2 = tr.Y2, Score = 1f };
                        tracking = true;
                        StatTrackCalls++;
                        // safety refresh: keep tracking while the detect runs
                        if (tr.Age >= redetectEveryN) KickAsyncDetect(frame.ColorBytes, cw, ch);
                    }
                    else if (_track.TryGetValue(serial, out var stale) && stale.Grace > 0)
                    {
                        // grace bridge: ride the last box while the detect runs
                        KickAsyncDetect(frame.ColorBytes, cw, ch);
                        box = new DetBox { X1 = stale.X1, Y1 = stale.Y1, X2 = stale.X2, Y2 = stale.Y2, Score = 1f };
                        tracking = true;
                        stale.Grace--;
                        _track[serial] = stale;
                    }
                    else
                    {
                        // track lost and grace spent: this camera sits the frame out
                        // (fusion keeps running on the others) instead of stalling
                        // the burst
                        KickAsyncDetect(frame.ColorBytes, cw, ch);
                        StatAsyncSkippedFrames++;
                        return;
                    }
                }
            }
            else if (haveTrack && tr.Age < redetectEveryN)
            {
                box = new DetBox { X1 = tr.X1, Y1 = tr.Y1, X2 = tr.X2, Y2 = tr.Y2, Score = 1f };
                tracking = true;
                StatTrackCalls++;
            }
            else
            {
                StatDetectCalls++;
                t0 = System.Diagnostics.Stopwatch.GetTimestamp();
                int nDet = _backend.Detect(frame.ColorBytes, cw, ch, _boxes);
                DetectMs[DetectN++ % TimingCap] = MsSince(t0);
                if (nDet <= 0) { _track.Remove(serial); StatNoDetect++; return; }
                if (nDet > MaxDet) nDet = MaxDet;
                int sel = SelectPerson(nDet, serial, cam, have3d, cw, ch);
                if (sel < 0) { _track.Remove(serial); return; }
                box = _boxes[sel];
            }

            var spec = _backend.Spec;
            var roi = RtmposePreprocess.RoiFromBox(box.X1, box.Y1, box.X2, box.Y2, spec);
            t0 = System.Diagnostics.Stopwatch.GetTimestamp();
            RtmposePreprocess.BuildInput(frame.ColorBytes, cw, ch, roi, spec, _input);
            PreMs[PreN++ % TimingCap] = MsSince(t0);
            t0 = System.Diagnostics.Stopwatch.GetTimestamp();
            bool poseOk = _backend.Pose(_input, _simccX, _simccY);
            PoseMs[PoseN++ % TimingCap] = MsSince(t0);
            if (!poseOk) { _track.Remove(serial); return; }

            int k = _backend.NumKeypoints;
            SimccDecoder.Decode(_simccX, _simccY, k, roi, spec, _coco, _cocoScore);
            CocoToEval.Map(_coco, _cocoScore, confThreshold, _xy, _score, _valid);

            _skel.Reset(0, tsNs);
            float x1 = float.MaxValue, y1 = float.MaxValue, x2 = float.MinValue, y2 = float.MinValue;
            float confSum = 0; int confN = 0;
            for (int j = 0; j < EvalSkeleton.JointCount; j++)
            {
                ref var oj = ref _skel.Joints[j];
                oj.Confidence = _score[j];
                if (_valid[j])
                {
                    confSum += _score[j]; confN++;
                    x1 = Mathf.Min(x1, _xy[j].x); y1 = Mathf.Min(y1, _xy[j].y);
                    x2 = Mathf.Max(x2, _xy[j].x); y2 = Mathf.Max(y2, _xy[j].y);
                }
                if (!_valid[j] || !have3d) { oj.Valid = false; continue; }
                float d = _lift.SampleMm(Mathf.RoundToInt(_xy[j].x), Mathf.RoundToInt(_xy[j].y), depthWindowHalf);
                if (d <= 0f) { oj.Valid = false; continue; }
                oj.PositionMm = DepthLift.Backproject(_xy[j].x, _xy[j].y, d, cw, ch, cam.Value);
                oj.Valid = true;
            }

            // volume re-check while tracking: if the tracked person left the volume, re-detect next frame
            if (tracking && _hasVolume && have3d && _colorToWorld.TryGetValue(serial, out var e))
            {
                ref var pelvis = ref _skel.Joints[(int)EvalJointId.Pelvis];
                if (pelvis.Valid && !InVolume(ToWorld(pelvis.PositionMm, e))) { InvalidateTrack(serial); return; }
            }

            // update the tracked box from the keypoint bounds (padded) for the next frame
            float meanConf = confN > 0 ? confSum / confN : 0f;
            if (confN >= 3 && x2 > x1 && y2 > y1)
            {
                float pw = (x2 - x1) * trackPad, ph = (y2 - y1) * trackPad;
                bool validNow = meanConf >= minTrackConf;
                _track[serial] = new Track
                {
                    X1 = x1 - pw, Y1 = y1 - ph, X2 = x2 + pw, Y2 = y2 + ph,
                    Age = tracking ? tr.Age + 1 : 0,
                    Valid = validNow,
                    // a valid track re-arms the grace budget; an invalid one keeps
                    // whatever the bridge has left (granted at the transition)
                    Grace = validNow ? trackGraceFrames
                        : (_track.TryGetValue(serial, out var cur) ? (cur.Valid ? trackGraceFrames : cur.Grace) : 0),
                };
            }
            else InvalidateTrack(serial);

            _frame.Reset(Name, serial, tsNs);
            _frame.Bodies.Add(_skel);
            OnSkeletons?.Invoke(_frame);
        }

        /// <summary>Drop the track — but under asyncDetect keep the last box as an
        /// INVALID entry with a grace budget, so the bridge can ride it while the
        /// background detect re-acquires. Sync mode removes outright (original
        /// behavior; the next frame blocks on a fresh detect anyway).</summary>
        private void InvalidateTrack(string serial)
        {
            if (asyncDetect && _track.TryGetValue(serial, out var prev))
                _track[serial] = new Track
                {
                    X1 = prev.X1, Y1 = prev.Y1, X2 = prev.X2, Y2 = prev.Y2,
                    Age = prev.Age, Valid = false,
                    Grace = prev.Valid ? trackGraceFrames : prev.Grace,
                };
            else _track.Remove(serial);
        }

        private int SelectPerson(int nDet, string serial, ObCameraParam? cam, bool have3d, int cw, int ch)
        {
            bool useVolume = _hasVolume && have3d && cam.HasValue && _colorToWorld.ContainsKey(serial);
            if (!useVolume)
            {
                int best = 0;
                for (int i = 1; i < nDet; i++) if (_boxes[i].Score > _boxes[best].Score) best = i;
                return best;
            }
            var e = _colorToWorld[serial];
            int sel = -1; float selScore = -1f;
            for (int i = 0; i < nDet; i++)
            {
                // A single box-center sample can fall through a body gap and hit
                // the BACKGROUND (e.g. legs apart -> center samples the far wall,
                // person gets rejected). Sample a coarse grid over the inner box
                // and take the median of valid depths instead.
                float d = MedianBoxDepthMm(_boxes[i]);
                if (d <= 0f) continue;
                float bx = 0.5f * (_boxes[i].X1 + _boxes[i].X2), by = 0.5f * (_boxes[i].Y1 + _boxes[i].Y2);
                Vector3 world = ToWorld(DepthLift.Backproject(bx, by, d, cw, ch, cam.Value), e);
                if (!InVolume(world)) continue;
                if (_boxes[i].Score > selScore) { selScore = _boxes[i].Score; sel = i; }
            }
            return sel;
        }

        /// <summary>Median of valid depths over a 5x5 grid spanning the inner 60% of the box.</summary>
        private float MedianBoxDepthMm(in DetBox b)
        {
            float cx = 0.5f * (b.X1 + b.X2), cy = 0.5f * (b.Y1 + b.Y2);
            float hw = 0.3f * (b.X2 - b.X1), hh = 0.3f * (b.Y2 - b.Y1);
            Span<float> vals = stackalloc float[25];
            int n = 0;
            for (int gy = -2; gy <= 2; gy++)
                for (int gx = -2; gx <= 2; gx++)
                {
                    float d = _lift.SampleMm(
                        Mathf.RoundToInt(cx + gx * 0.5f * hw),
                        Mathf.RoundToInt(cy + gy * 0.5f * hh), 2);
                    if (d > 0f) vals[n++] = d;
                }
            if (n == 0) return 0f;
            // insertion sort + median (n <= 25)
            for (int i = 1; i < n; i++) { float k = vals[i]; int j = i - 1; while (j >= 0 && vals[j] > k) { vals[j + 1] = vals[j]; j--; } vals[j + 1] = k; }
            return vals[n / 2];
        }

        private bool InVolume(Vector3 pMm) =>
            Mathf.Abs(pMm.x - _volCenter.x) <= _volHalf.x &&
            Mathf.Abs(pMm.y - _volCenter.y) <= _volHalf.y &&
            Mathf.Abs(pMm.z - _volCenter.z) <= _volHalf.z;

        private static Vector3 ToWorld(Vector3 camMm, in ObExtrinsic e)
        {
            var R = e.Rot; var T = e.Trans;
            return new Vector3(
                R[0] * camMm.x + R[1] * camMm.y + R[2] * camMm.z + T[0],
                R[3] * camMm.x + R[4] * camMm.y + R[5] * camMm.z + T[1],
                R[6] * camMm.x + R[7] * camMm.y + R[8] * camMm.z + T[2]);
        }

        public void Pump() { }
        public void Dispose() => _backend.Dispose();
    }
}
