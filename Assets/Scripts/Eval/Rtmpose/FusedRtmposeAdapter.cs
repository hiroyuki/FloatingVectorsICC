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

        public BodyProfile Profile;

        readonly RtmPoseAdapter _inner;
        public RtmPoseAdapter Inner => _inner;

        // per-serial world transform (OpenCV frame, mm): world = R·cam + T
        sealed class WorldTr { public float[] R; public float[] T; }
        readonly Dictionary<string, WorldTr> _world = new();

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

        ulong _lastFusedTs;
        readonly EvalSkeletonFrame _frame = new();
        readonly EvalSkeleton _fused = new();

        public event Action<EvalSkeletonFrame> OnSkeletons;

        // diagnostics (per session, read for reporting)
        public int StatConsensus, StatSingleAccepted, StatRelifted, StatHeld, StatDroppedLen, StatOutliers;

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

        public void Configure(in EvalCameraContext ctx) => _inner.Configure(ctx);

        public void SubmitFrame(string serial, in RawFrameData frame, ulong tsNs)
        {
            _inner.SubmitFrame(serial, frame, tsNs); // synchronous → OnInnerSkeletons fires inside
            TryFuse(tsNs);
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
                    fusedPos[j] = pos; fusedOk[j] = true; fusedFresh[j] = fresh;
                    ref var oj = ref _fused.Joints[j];
                    oj.PositionMm = pos; oj.Confidence = conf; oj.Valid = true;
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
                        _histVel[j] = _histHas[j] ? Vector3.Lerp(rawVel, _histVel[j], velSmoothing) : rawVel;
                    }
                }
                else _histVel[j] = Vector3.zero;
                _histPos[j] = fusedPos[j]; _histTs[j] = nowTs; _histHas[j] = true;
            }

            _frame.Reset(Name, "fused", nowTs);
            _frame.Bodies.Add(_fused);
            OnSkeletons?.Invoke(_frame);
        }

        bool FuseJoint(int j, List<Vector3> pts, List<float> wts, List<int> srcSample,
                       out Vector3 pos, out float conf, out bool fresh)
        {
            pos = default; conf = 0f; fresh = false;

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
                    float dt = Mathf.Min(age, maxPredictSeconds);
                    pos = _histPos[j] + _histVel[j] * dt;
                    conf = 0.2f; fresh = false; StatHeld++;
                    return true;
                }
            }
            return false;
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
