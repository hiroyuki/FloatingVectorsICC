// Quantitative live-vs-v11s verification on the SAME take (eval/PLAN_live_gpu
// Phase 3 follow-up: "live looks dirtier than recorded v11s" — measure it).
//
// Reference (A): the take's v11s bodies_main (offline FusedBodiesExport output),
//   decoded from ONE device dir and transformed depth -> color -> world with the
//   exact inverse of FusedBodiesExport.ToDepth.
// Live (C): FusedRtmposeAdapter.OnSkeletons captured during live playback
//   (world frame, post median-lag filter — the same stream the export recorded).
//
// Usage (execute_code):
//   LiveV11sVerify.StartCapture();          // while live playback is running
//   LiveV11sVerify.Status();                // sample count
//   LiveV11sVerify.StopAndReport(takeDir);  // writes eval/results/live_v11s_*.md
//
// Metrics: emit cadence (dt p50/p95, gaps), per-joint validity, dt-aware
// acceleration RMS (high-frequency jitter), and nearest-timestamp RMSE live↔ref.

using System;
using System.Collections.Generic;
using System.IO;
using System.Text;
using Orbbec;
using PointCloud;
using UnityEngine;

namespace BodyTracking.Eval.Rtmpose
{
    public static class LiveV11sVerify
    {
        sealed class Sample
        {
            public ulong Ts;
            public Vector3[] Pos = new Vector3[EvalSkeleton.JointCount];
            public bool[] Valid = new bool[EvalSkeleton.JointCount];
        }

        static readonly object _lock = new object();
        static List<Sample> _live;
        static FusedRtmposeAdapter _hooked;
        static Action<EvalSkeletonFrame> _handler;

        public static string StartCapture()
        {
            var lf = UnityEngine.Object.FindFirstObjectByType<LiveFusedBodySource>();
            var ad = lf != null ? lf.CurrentAdapter : null;
            if (ad == null) return "no live adapter — enable LiveFusedBodySource in play mode first";
            Unhook();
            _live = new List<Sample>(1 << 14);
            _handler = f =>
            {
                var p = f.Primary();
                if (p == null) return;
                var s = new Sample { Ts = f.TimestampNs };
                for (int j = 0; j < EvalSkeleton.JointCount; j++)
                {
                    s.Pos[j] = p.Joints[j].PositionMm;
                    s.Valid[j] = p.Joints[j].Valid;
                }
                lock (_lock) _live.Add(s);
            };
            ad.OnSkeletons += _handler; // worker thread — handler stays allocation-light
            _hooked = ad;
            return "capturing (adapter hooked)";
        }

        public static string Status()
        {
            lock (_lock) return _live == null ? "idle" : $"captured {_live.Count} live emits";
        }

        public static string StopAndReport(string takeDir)
        {
            Unhook();
            List<Sample> live;
            lock (_lock) { live = _live; _live = null; }
            if (live == null || live.Count < 30) return $"too few live samples ({live?.Count ?? 0})";
            live.Sort((a, b) => a.Ts.CompareTo(b.Ts));

            var reference = LoadV11sWorld(takeDir, out string refInfo);
            if (reference == null) return refInfo;

            // clip reference to the captured window (+/- 50ms)
            ulong t0 = live[0].Ts - 50_000_000UL, t1 = live[live.Count - 1].Ts + 50_000_000UL;
            reference = reference.FindAll(s => s.Ts >= t0 && s.Ts <= t1);
            if (reference.Count < 30) return $"reference has too few samples in the captured window ({reference.Count})";

            var sb = new StringBuilder();
            sb.AppendLine("# live vs v11s — " + Path.GetFileName(takeDir.TrimEnd('/', '\\')));
            sb.AppendLine();
            sb.AppendLine(refInfo);
            sb.AppendLine($"captured window: {(live[live.Count - 1].Ts - live[0].Ts) / 1e9:F1}s, live emits={live.Count}, ref emits={reference.Count}");
            sb.AppendLine();
            Describe(sb, "v11s (recorded, offline)", reference);
            Describe(sb, "live (captured)", live);
            CrossRmse(sb, live, reference);

            string outDir = Path.Combine(Directory.GetParent(Application.dataPath).FullName, "eval", "results");
            Directory.CreateDirectory(outDir);
            string path = Path.Combine(outDir, $"live_v11s_{DateTime.Now:yyyyMMdd_HHmmss}.md");
            File.WriteAllText(path, sb.ToString());
            Debug.Log($"[LiveV11sVerify] report -> {path}\n{sb}");
            return path;
        }

        static void Unhook()
        {
            if (_hooked != null && _handler != null) _hooked.OnSkeletons -= _handler;
            _hooked = null; _handler = null;
        }

        /// <summary>Offline A/B: compare two exported bodies_main takes (e.g. the
        /// stored v11s vs a re-export with a different inference backend). Both
        /// sides go through the same loader, so ONLY the export content differs —
        /// no live-delivery variance.</summary>
        public static string CompareTakes(string refDir, string testDir)
        {
            var reference = LoadV11sWorld(refDir, out string refInfo);
            if (reference == null) return refInfo;
            var test = LoadV11sWorld(testDir, out string testInfo);
            if (test == null) return testInfo;

            // clip to the overlapping window
            ulong t0 = Math.Max(reference[0].Ts, test[0].Ts);
            ulong t1 = Math.Min(reference[reference.Count - 1].Ts, test[test.Count - 1].Ts);
            reference = reference.FindAll(s => s.Ts >= t0 && s.Ts <= t1);
            test = test.FindAll(s => s.Ts >= t0 && s.Ts <= t1);
            if (reference.Count < 30 || test.Count < 30) return $"overlap too small (ref={reference.Count}, test={test.Count})";

            var sb = new StringBuilder();
            sb.AppendLine("# offline A/B — " + Path.GetFileName(refDir.TrimEnd('/', '\\')) + " vs " + Path.GetFileName(testDir.TrimEnd('/', '\\')));
            sb.AppendLine();
            sb.AppendLine("ref:  " + refInfo);
            sb.AppendLine("test: " + testInfo);
            sb.AppendLine();
            Describe(sb, "ref (" + Path.GetFileName(refDir.TrimEnd('/', '\\')) + ")", reference);
            Describe(sb, "test (" + Path.GetFileName(testDir.TrimEnd('/', '\\')) + ")", test);
            CrossRmse(sb, test, reference);

            string outDir = Path.Combine(Directory.GetParent(Application.dataPath).FullName, "eval", "results");
            Directory.CreateDirectory(outDir);
            string path = Path.Combine(outDir, $"offline_ab_{DateTime.Now:yyyyMMdd_HHmmss}.md");
            File.WriteAllText(path, sb.ToString());
            Debug.Log($"[LiveV11sVerify] offline A/B -> {path}\n{sb}");
            return path;
        }

        // ---- reference loading ------------------------------------------------

        static List<Sample> LoadV11sWorld(string takeDir, out string info)
        {
            info = "";
            string serial = null, deviceDir = null;
            foreach (var (s, dir) in PointCloudRecording.EnumerateDevices(takeDir))
            {
                if (File.Exists(Path.Combine(dir, PointCloudRecording.BodiesSensorName))) { serial = s; deviceDir = dir; break; }
            }
            if (serial == null) { info = "no device dir with bodies_main under " + takeDir; return null; }

            var calib = PointCloudRecording.ReadExtrinsicsYaml(takeDir);
            if (calib == null) { info = "no extrinsics.yaml under " + takeDir; return null; }
            ObExtrinsic? g = null, e = null;
            foreach (var dc in calib)
            {
                if (!string.Equals(dc.Serial, serial, StringComparison.OrdinalIgnoreCase)) continue;
                if (dc.GlobalTrColorCamera.HasValue) g = dc.GlobalTrColorCamera.Value;
                e = dc.DepthToColor;
            }
            if (!g.HasValue || !e.HasValue) { info = $"no calibration for serial {serial}"; return null; }

            var outList = new List<Sample>(1 << 14);
            var bodies = new BodySnapshot[2];
            for (int i = 0; i < bodies.Length; i++) bodies[i] = new BodySnapshot();
            using (var fs = new PointCloudRecording.RcsvFrameStream(Path.Combine(deviceDir, PointCloudRecording.BodiesSensorName)))
            {
                for (int i = 0; i < fs.Count; i++)
                {
                    var fr = fs[i];
                    int n = RecordedBodySerializer.Decode(fr.Bytes, fr.ByteCount, bodies);
                    if (n <= 0) continue;
                    var snap = bodies[0];
                    var s = new Sample { Ts = fr.TimestampNs };
                    for (int j = 0; j < EvalSkeleton.JointCount; j++)
                    {
                        var jt = snap.Joints[(int)EvalSkeletonMap.K4abtSource[j]];
                        if (jt.ConfidenceLevel == k4abt_joint_confidence_level_t.K4ABT_JOINT_CONFIDENCE_NONE) continue;
                        var d = new Vector3(jt.Position.X, jt.Position.Y, jt.Position.Z);
                        s.Pos[j] = Fwd(Fwd(d, e.Value), g.Value); // depth -> color -> world
                        s.Valid[j] = true;
                    }
                    outList.Add(s);
                }
            }
            info = $"reference: serial={serial} ({outList.Count} body frames total)";
            return outList;
        }

        // forward of FusedBodiesExport.InverseTransform (row-major R, mm T)
        static Vector3 Fwd(Vector3 p, in ObExtrinsic ext)
        {
            var R = ext.Rot; var T = ext.Trans;
            return new Vector3(
                R[0] * p.x + R[1] * p.y + R[2] * p.z + T[0],
                R[3] * p.x + R[4] * p.y + R[5] * p.z + T[1],
                R[6] * p.x + R[7] * p.y + R[8] * p.z + T[2]);
        }

        // ---- metrics ----------------------------------------------------------

        static readonly (string name, int[] joints)[] Groups =
        {
            ("pelvis/neck/head", new[] { (int)EvalJointId.Pelvis, (int)EvalJointId.Neck, (int)EvalJointId.Head }),
            ("shoulders/elbows", new[] { (int)EvalJointId.ShoulderL, (int)EvalJointId.ShoulderR, (int)EvalJointId.ElbowL, (int)EvalJointId.ElbowR }),
            ("wrists",           new[] { (int)EvalJointId.WristL, (int)EvalJointId.WristR }),
            ("hips/knees",       new[] { (int)EvalJointId.HipL, (int)EvalJointId.HipR, (int)EvalJointId.KneeL, (int)EvalJointId.KneeR }),
            ("ankles",           new[] { (int)EvalJointId.AnkleL, (int)EvalJointId.AnkleR }),
        };

        static void Describe(StringBuilder sb, string label, List<Sample> s)
        {
            sb.AppendLine($"## {label}");
            var dts = new List<double>(s.Count);
            int gaps = 0;
            for (int i = 1; i < s.Count; i++)
            {
                double dt = (s[i].Ts - s[i - 1].Ts) / 1e6; // ms
                dts.Add(dt);
                if (dt > 60) gaps++;
            }
            dts.Sort();
            double durS = (s[s.Count - 1].Ts - s[0].Ts) / 1e9;
            sb.AppendLine($"- emits: {s.Count} over {durS:F1}s = {(s.Count - 1) / durS:F1} Hz; dt p50={P(dts, 0.5):F1}ms p95={P(dts, 0.95):F1}ms max={P(dts, 1.0):F1}ms; gaps>60ms: {gaps}");
            sb.AppendLine("- per group: validity% | accel RMS mm/s^2 | speed p95 mm/s");
            foreach (var (name, joints) in Groups)
            {
                long validN = 0, totalN = 0;
                var accels = new List<double>();
                var speeds = new List<double>();
                foreach (int j in joints)
                {
                    for (int i = 0; i < s.Count; i++) { totalN++; if (s[i].Valid[j]) validN++; }
                    for (int i = 1; i < s.Count - 1; i++)
                    {
                        if (!s[i - 1].Valid[j] || !s[i].Valid[j] || !s[i + 1].Valid[j]) continue;
                        double dt0 = (s[i].Ts - s[i - 1].Ts) / 1e9, dt1 = (s[i + 1].Ts - s[i].Ts) / 1e9;
                        if (dt0 <= 1e-4 || dt1 <= 1e-4 || dt0 > 0.1 || dt1 > 0.1) continue;
                        Vector3 v0 = (s[i].Pos[j] - s[i - 1].Pos[j]) / (float)dt0;
                        Vector3 v1 = (s[i + 1].Pos[j] - s[i].Pos[j]) / (float)dt1;
                        accels.Add(((v1 - v0) / (float)(0.5 * (dt0 + dt1))).magnitude);
                        speeds.Add(v1.magnitude);
                    }
                }
                double rms = 0; foreach (var a in accels) rms += a * a;
                rms = accels.Count > 0 ? Math.Sqrt(rms / accels.Count) : 0;
                speeds.Sort();
                sb.AppendLine($"  - {name}: {100.0 * validN / Math.Max(1, totalN):F1}% | {rms:F0} | {P(speeds, 0.95):F0}");
            }
            sb.AppendLine();
        }

        static void CrossRmse(StringBuilder sb, List<Sample> live, List<Sample> reference)
        {
            sb.AppendLine("## live vs v11s nearest-timestamp distance (mm, pairs within 30ms)");
            int cursor = 0;
            var dists = new List<double>[Groups.Length];
            for (int gi = 0; gi < Groups.Length; gi++) dists[gi] = new List<double>();
            int paired = 0;
            foreach (var ls in live)
            {
                while (cursor + 1 < reference.Count &&
                       AbsDiff(reference[cursor + 1].Ts, ls.Ts) <= AbsDiff(reference[cursor].Ts, ls.Ts)) cursor++;
                var rs = reference[cursor];
                if (AbsDiff(rs.Ts, ls.Ts) > 30_000_000UL) continue;
                paired++;
                for (int gi = 0; gi < Groups.Length; gi++)
                    foreach (int j in Groups[gi].joints)
                        if (ls.Valid[j] && rs.Valid[j])
                            dists[gi].Add((ls.Pos[j] - rs.Pos[j]).magnitude);
            }
            sb.AppendLine($"- paired emits: {paired}/{live.Count}");
            for (int gi = 0; gi < Groups.Length; gi++)
            {
                var d = dists[gi]; d.Sort();
                sb.AppendLine($"  - {Groups[gi].name}: p50={P(d, 0.5):F1} p95={P(d, 0.95):F1} max={P(d, 1.0):F1}");
            }
        }

        static ulong AbsDiff(ulong a, ulong b) => a > b ? a - b : b - a;

        static double P(List<double> sorted, double q)
        {
            if (sorted == null || sorted.Count == 0) return 0;
            int idx = Math.Min(sorted.Count - 1, (int)(q * sorted.Count));
            return sorted[idx];
        }
    }
}
