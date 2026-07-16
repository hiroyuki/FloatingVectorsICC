// Chunked comparison of the FUSED RTMPose stream against the k4abt recorded
// baseline and the raw per-camera RTMPose stream — the "does 3-stage fusion
// fix occlusion?" rematch. Same chunked Start/Step/Finish shape as
// RtmposeCompareChunked (static state persists between execute_code calls; do
// NOT recompile mid-run).
//
//   FusedCompareChunked.Start("...15-50-24", 0, 0.3f)   // maxFramesPerDevice (0 = all), conf
//   FusedCompareChunked.Step(300)                        // repeat until DONE
//   FusedCompareChunked.Finish("")                       // CSV + summary ("" = eval/results)
//   FusedCompareChunked.SpotCheck(1784011855172824000)   // fused joints nearest a ts (f788)
//
// Metrics keys: "k4abt" (per camera), "rtmpose" (per camera), "rtmfused"
// (single stream, serial "fused").

using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Text;
using PointCloud;
using UnityEngine;

namespace BodyTracking.Eval.Rtmpose
{
    public static class FusedCompareChunked
    {
        static GameObject _go;
        static EvalReplayDriver _driver;
        static K4abtBaselineAdapter _baseline;
        static FusedRtmposeAdapter _fused;
        static EvalMetrics _metrics;
        static List<(int dev, int idx)> _order;
        static int _cursor;
        static Stopwatch _sw;
        static readonly List<(ulong ts, EvalSkeleton skel)> _fusedLog = new();

        public static float volCx = 0, volCy = 200, volCz = 3000, volHx = 1000, volHy = 1400, volHz = 900;

        public static string Start(string sessionRoot, int maxFramesPerDevice, float conf)
        {
            Abort();
            _go = new GameObject("FusedCompareChunked");
            _driver = _go.AddComponent<EvalReplayDriver>();
            _driver.loadColor = true; _driver.loadIR = false;
            _metrics = new EvalMetrics();
            _metrics.Configure(EvalMetrics.Config.Default);
            _baseline = new K4abtBaselineAdapter();
            _fused = new FusedRtmposeAdapter(BtFrameInspectorWindow.SharedBackend()) { ConfThreshold = conf };

            string profilePath = Path.GetFullPath(Path.Combine(Application.dataPath, "..", "eval", "body_profile.json"));
            if (File.Exists(profilePath)) _fused.Profile = BodyProfile.Load(profilePath);

            _baseline.OnSkeletons += f => _metrics.AddFrame("k4abt", f);
            _fused.Inner.OnSkeletons += f => _metrics.AddFrame("rtmpose", f);
            _fused.OnSkeletons += f =>
            {
                _metrics.AddFrame("rtmfused", f);
                var p = f.Primary();
                if (p != null)
                {
                    var copy = new EvalSkeleton();
                    copy.Reset(p.PersonId, p.TimestampNs);
                    for (int j = 0; j < EvalSkeleton.JointCount; j++) copy.Joints[j] = p.Joints[j];
                    _fusedLog.Add((f.TimestampNs, copy));
                }
            };
            _driver.OnFrame += (serial, frame, cam, ts) =>
            {
                _metrics.AddSubmitted("k4abt", serial);
                _metrics.AddSubmitted("rtmpose", serial);
                _metrics.AddSubmitted("rtmfused", "fused");
                _fused.SubmitFrame(serial, frame, ts);
            };
            _driver.OnRecordedBodies += (serial, bytes, bc, ts) => _baseline.SubmitRecordedBodies(serial, bytes, bc, ts);

            if (!_driver.Load(sessionRoot)) { Abort(); return "driver.Load failed"; }
            foreach (var dev in _driver.Devices)
            {
                var ctx = new EvalCameraContext(dev.Serial, dev.DepthW, dev.DepthH, dev.ColorW, dev.ColorH, dev.CameraParam);
                _baseline.Configure(ctx);
                _fused.Configure(ctx);
            }

            var calib = PointCloudRecording.ReadExtrinsicsYaml(sessionRoot);
            if (calib == null) { Abort(); return "no extrinsics.yaml"; }
            foreach (var dc in calib)
                if (dc.GlobalTrColorCamera.HasValue)
                    _fused.SetWorldTransform(dc.Serial, dc.GlobalTrColorCamera.Value);
            _fused.SetCaptureVolume(new Vector3(volCx, volCy, volCz), new Vector3(volHx, volHy, volHz));

            int cap = maxFramesPerDevice > 0 ? maxFramesPerDevice : int.MaxValue;
            _order = new List<(int, int)>();
            var withTs = new List<(ulong ts, int dev, int idx)>();
            for (int d = 0; d < _driver.Devices.Count; d++)
            {
                var mt = _driver.Devices[d].MasterTs;
                int m = Math.Min(mt.Length, cap);
                for (int i = 0; i < m; i++) withTs.Add((mt[i], d, i));
            }
            withTs.Sort((a, b) => a.ts.CompareTo(b.ts));
            foreach (var w in withTs) _order.Add((w.dev, w.idx));
            _cursor = 0;
            _sw = Stopwatch.StartNew();
            return $"ready: {_order.Count} frames, {_driver.Devices.Count} devices, profile={( _fused.Profile != null ? "loaded" : "MISSING")}";
        }

        public static string Step(int n)
        {
            if (_driver == null || _order == null) return "not started";
            int end = Math.Min(_cursor + Math.Max(1, n), _order.Count);
            for (; _cursor < end; _cursor++) { var o = _order[_cursor]; _driver.EmitAt(o.dev, o.idx); }
            bool done = _cursor >= _order.Count;
            return $"{_cursor}/{_order.Count}{(done ? " DONE" : "")} elapsed={_sw.Elapsed.TotalSeconds:F1}s";
        }

        public static string Finish(string resultsDir)
        {
            if (_metrics == null) return "not started";
            if (string.IsNullOrEmpty(resultsDir))
                resultsDir = Path.GetFullPath(Path.Combine(Application.dataPath, "..", "eval", "results", "fused"));
            Directory.CreateDirectory(resultsDir);
            _metrics.WriteCsv(resultsDir);
            var sb = new StringBuilder();
            sb.AppendLine(_metrics.BuildSummary());
            sb.AppendLine($"fusion stats: consensus={_fused.StatConsensus} single={_fused.StatSingleAccepted} " +
                          $"relift={_fused.StatRelifted} held={_fused.StatHeld} jumpHeld={_fused.StatJumpHeld} " +
                          $"droppedLen={_fused.StatDroppedLen} outliers={_fused.StatOutliers}");
            sb.AppendLine("csv dir: " + resultsDir);
            // NOTE: no Abort here — _fusedLog stays available for SpotCheck().
            return sb.ToString();
        }

        /// <summary>Fused joints nearest a timestamp (world mm, OpenCV origin-camera frame).</summary>
        public static string SpotCheck(ulong ts)
        {
            if (_fusedLog.Count == 0) return "no fused frames logged";
            int best = -1; ulong bd = ulong.MaxValue;
            for (int i = 0; i < _fusedLog.Count; i++)
            {
                ulong d = _fusedLog[i].ts > ts ? _fusedLog[i].ts - ts : ts - _fusedLog[i].ts;
                if (d < bd) { bd = d; best = i; }
            }
            var (fts, skel) = _fusedLog[best];
            var sb = new StringBuilder();
            sb.AppendLine($"fused @ ts={fts} (Δ{bd / 1e6:F0}ms), validJoints={skel.ValidCount()}");
            for (int j = 0; j < EvalSkeleton.JointCount; j++)
            {
                var jt = skel.Joints[j];
                sb.AppendLine($"  {(EvalJointId)j,-10} {(jt.Valid ? jt.PositionMm.ToString("F0") : "-"),-24} conf={jt.Confidence:F2}");
            }
            return sb.ToString();
        }

        public static void Abort()
        {
            try { _baseline?.Dispose(); } catch { }
            // _fused.Dispose() is a deliberate no-op (shared backend) — safe either way.
            if (_go != null) UnityEngine.Object.DestroyImmediate(_go);
            _go = null; _driver = null; _baseline = null; _fused = null;
            _metrics = null; _order = null; _cursor = 0;
            _fusedLog.Clear();
        }
    }
}
