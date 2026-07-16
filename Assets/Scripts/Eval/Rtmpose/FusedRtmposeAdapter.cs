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

        public BodyProfile Profile;

        readonly RtmPoseAdapter _inner;
        public RtmPoseAdapter Inner => _inner;

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
            public float Fx, Fy, Cx, Cy;             // depth intrinsics (halved)
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

        public FusedRtmposeAdapter(OrtRtmposeBackend backend)
        {
            _inner = new RtmPoseAdapter(backend);
            _inner.OnSkeletons += OnInnerSkeletons;
        }

        public float ConfThreshold { get => _inner.confThreshold; set => _inner.confThreshold = value; }

        public void SetWorldTransform(string serial, ObExtrinsic ext)
        {
            _inner.SetWorldTransform(serial, ext);
            _world[serial] = new WorldTr { R = (float[])ext.Rot.Clone(), T = (float[])ext.Trans.Clone() };
        }

        public void SetCaptureVolume(Vector3 centerMm, Vector3 halfMm) => _inner.SetCaptureVolume(centerMm, halfMm);

        public void Configure(in EvalCameraContext ctx)
        {
            _inner.Configure(ctx);
            if (ctx.CameraParam.HasValue)
            {
                var cp = ctx.CameraParam.Value;
                var dr = new DepthRef
                {
                    Fx = cp.DepthIntrinsic.Fx * 0.5f, Fy = cp.DepthIntrinsic.Fy * 0.5f,
                    Cx = cp.DepthIntrinsic.Cx * 0.5f, Cy = cp.DepthIntrinsic.Cy * 0.5f,
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

        public void SubmitFrame(string serial, in RawFrameData frame, ulong tsNs)
        {
            StashDepth(serial, frame);
            _inner.SubmitFrame(serial, frame, tsNs); // synchronous → OnInnerSkeletons fires inside
            TryFuse(tsNs);
        }

        // Keep a stride-2 copy of this camera's latest depth map for visibility tests.
        void StashDepth(string serial, in RawFrameData frame)
        {
            if (!_depth.TryGetValue(serial, out var dr) || !dr.HasCal) return;
            int dw = frame.DepthWidth, dh = frame.DepthHeight;
            if (dw <= 0 || dh <= 0 || frame.DepthBytes == null || frame.DepthByteCount < dw * dh * 2) return;
            int w = dw / 2, h = dh / 2;
            if (dr.D == null || dr.W != w || dr.H != h) { dr.D = new ushort[w * h]; dr.W = w; dr.H = h; }
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
        public void Pump() => _inner.Pump();

        // The inner adapter runs on the shared cached backend (see
        // BtFrameInspectorWindow.SharedBackend) — disposing it here would kill
        // the backend for every other user and crash later inference natively.
        public void Dispose() { }

        void OnInnerSkeletons(EvalSkeletonFrame f)
        {
            var p = f.Primary();
            if (p == null) return;
            if (!_world.TryGetValue(f.Serial, out var tr)) return;
            if (!_latest.TryGetValue(f.Serial, out var s)) { s = new CamSample(); _latest[f.Serial] = s; }
            s.Ts = f.TimestampNs;
            for (int j = 0; j < (int)EvalJointId.Count; j++)
            {
                s.Valid[j] = p.Joints[j].Valid;
                s.Conf[j] = p.Joints[j].Confidence;
                if (s.Valid[j]) s.World[j] = ToWorld(tr, p.Joints[j].PositionMm);
            }
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
            if (_lastFusedTs != 0 && nowTs > _lastFusedTs
                && (nowTs - _lastFusedTs) < (ulong)(minFuseIntervalMs * 1e6f)) return;

            _work.Clear(); _workSerial.Clear(); _workValid.Clear();
            ulong skewNs = (ulong)(skewMs * 1e6f);
            foreach (var kv in _latest)
            {
                ulong d = kv.Value.Ts > nowTs ? kv.Value.Ts - nowTs : nowTs - kv.Value.Ts;
                if (d > skewNs) continue;
                _work.Add(kv.Value); _workSerial.Add(kv.Key);
            }
            if (_work.Count == 0) return;
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

                float conf; Vector3 pos; bool fresh;
                if (FuseJoint(j, pts, wts, srcSample, out pos, out conf, out fresh))
                {
                    // Two-strike jump gate: a fresh position far from recent history
                    // is suspicious (single-frame spike). Hold it back once; accept
                    // only when a second consecutive frame lands near the pending
                    // position (genuine fast motion keeps moving the same way).
                    if (fresh && _histHas[j])
                    {
                        float age = (nowTs - _histTs[j]) / 1e9f;
                        if (age >= 0f && age <= holdMaxSeconds
                            && Vector3.Distance(pos, _histPos[j]) > jumpGateMm)
                        {
                            // Second strike accepts on proximity to the pending
                            // position OR on direction continuation — genuine fast
                            // motion keeps moving the same way each frame (the
                            // pending point itself has moved on), while an
                            // out-and-back spike reverses direction (dot < 0).
                            bool accept = false;
                            if (_jumpHasPending[j])
                            {
                                if (Vector3.Distance(pos, _jumpPending[j]) <= jumpGateMm) accept = true;
                                else
                                {
                                    var stepA = _jumpPending[j] - _histPos[j];
                                    var stepB = pos - _jumpPending[j];
                                    if (stepA.sqrMagnitude > 1f && stepB.sqrMagnitude > 1f
                                        && Vector3.Dot(stepA.normalized, stepB.normalized) > 0.3f)
                                        accept = true;
                                }
                            }
                            if (accept)
                            {
                                _jumpHasPending[j] = false;
                            }
                            else
                            {
                                _jumpPending[j] = pos; _jumpHasPending[j] = true;
                                pos = HeldPrediction(j, age);
                                conf = 0.25f; fresh = false;
                                StatJumpHeld++;
                            }
                        }
                        else _jumpHasPending[j] = false;
                    }
                    else _jumpHasPending[j] = false;

                    fusedPos[j] = pos; fusedOk[j] = true; fusedFresh[j] = fresh;
                    ref var oj = ref _fused.Joints[j];
                    oj.PositionMm = pos; oj.Confidence = conf; oj.Valid = true;
                }
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
                       out Vector3 pos, out float conf, out bool fresh)
        {
            pos = default; conf = 0f; fresh = false;

            // Occlusion arbitration between DISAGREEING samples: if camera C
            // provably cannot see the place camera A puts the joint (A's position
            // is behind C's measured surface, or outside C's view), while A CAN
            // see C's claimed location and still disagrees, then C was guessing an
            // occluded joint (e.g. a hand fully hidden behind the torso) — drop
            // C's sample so the sighted camera wins.
            if (pts.Count >= 2)
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
                        pts.RemoveAt(i); wts.RemoveAt(i); srcSample.RemoveAt(i);
                        StatOccDropped++;
                    }
                }
            }

            if (pts.Count >= 2)
            {
                var med = ComponentMedian(pts);
                // survivors within the per-joint gate of the median
                Vector3 acc = Vector3.zero; float wsum = 0f, cmax = 0f; int nOk = 0;
                for (int i = 0; i < pts.Count; i++)
                {
                    if (Vector3.Distance(pts[i], med) > gateMm[j]) { StatOutliers++; continue; }
                    acc += pts[i] * wts[i]; wsum += wts[i]; cmax = Mathf.Max(cmax, wts[i]); nOk++;
                }
                if (nOk >= 2 || (nOk == 1 && pts.Count == 2))
                {
                    pos = acc / wsum; conf = cmax; fresh = true; StatConsensus++;
                    return true;
                }
                // fall through: candidates exist but no consensus
            }
            else if (pts.Count == 1)
            {
                // single camera: accept when the bone length to the fused parent is sane
                if (PassesParentLength(j, pts[0]))
                {
                    pos = pts[0]; conf = wts[0] * 0.9f; fresh = true; StatSingleAccepted++;
                    return true;
                }
            }

            // Stage 3a: ray x bone-length re-lift (needs fused parent + profile)
            for (int i = 0; i < pts.Count; i++)
            {
                if (TryRayRelift(j, pts[i], srcSample[i], out var relift))
                {
                    pos = relift; conf = wts[i] * 0.7f; fresh = true; StatRelifted++;
                    return true;
                }
            }

            // Stage 3b: temporal hold
            if (_histHas[j])
            {
                float age = (_lastFusedTs - _histTs[j]) / 1e9f;
                if (age >= 0f && age <= holdMaxSeconds)
                {
                    pos = HeldPrediction(j, age);
                    conf = 0.2f; fresh = false; StatHeld++;
                    return true;
                }
            }
            return false;
        }

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
                float age = (_lastFusedTs - _histTs[j]) / 1e9f;
                if (age >= 0f && age <= holdMaxSeconds)
                {
                    var pred = _histPos[j] + _histVel[j] * Mathf.Min(age, maxPredictSeconds);
                    if (Vector3.Distance(candidate, pred) > 2f * gateMm[j]) return false;
                }
            }
            result = candidate;
            return true;
        }

        static Vector3 ComponentMedian(List<Vector3> pts)
        {
            var xs = new List<float>(pts.Count); var ys = new List<float>(pts.Count); var zs = new List<float>(pts.Count);
            foreach (var p in pts) { xs.Add(p.x); ys.Add(p.y); zs.Add(p.z); }
            xs.Sort(); ys.Sort(); zs.Sort();
            int m = pts.Count / 2;
            return new Vector3(xs[m], ys[m], zs[m]);
        }
    }
}
