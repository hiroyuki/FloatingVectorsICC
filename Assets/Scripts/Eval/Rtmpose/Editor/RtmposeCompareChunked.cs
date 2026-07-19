// Chunked Stage-2 comparison: same as RtmposeCompare but processes frames in
// bounded steps so each MCP execute_code call stays under the response timeout.
// Static state persists between calls (do NOT recompile mid-run).
//
//   Start("...15-50-24", 200, 0.3f, 15)   // maxFramesPerDevice, conf, warmupFrames
//   Step(150)   // repeat until it reports done
//   Finish("")  // writes CSV, returns summary

using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using PointCloud;
using UnityEngine;

namespace BodyTracking.Eval.Rtmpose
{
    public static class RtmposeCompareChunked
    {
        static GameObject _go;
        static EvalReplayDriver _driver;
        static K4abtBaselineAdapter _baseline;
        static OrtRtmposeBackend _backend;
        static RtmPoseAdapter _rtmpose;
        static EvalMetrics _metrics;
        static List<(int dev, int idx)> _order;
        static int _cursor, _warmup, _emitted;
        static bool _measureLat;
        static long _curTick, _freq;
        static Stopwatch _sw;

        // World capture volume (mm, world = origin camera frame). 0 half => disabled.
        public static float volCx = 0, volCy = 200, volCz = 3000, volHx = 1000, volHy = 1400, volHz = 900;

        public static string Start(string sessionRoot, int maxFramesPerDevice, float conf, int warmup)
        {
            Abort();
            string modelsDir = Path.GetFullPath(Path.Combine(Application.dataPath, "..", "eval", "models"));
            string yolox = FirstOnnx(Path.Combine(modelsDir, "yolox-m"));
            string rtm = FirstOnnx(Path.Combine(modelsDir, "rtmpose-m"));
            if (yolox == null || rtm == null) return "models not found";

            _go = new GameObject("RtmposeCompareChunked");
            _driver = _go.AddComponent<EvalReplayDriver>();
            _driver.loadColor = true; _driver.loadIR = false;
            _metrics = new EvalMetrics();
            _metrics.Configure(EvalMetrics.Config.Default);
            _baseline = new K4abtBaselineAdapter();
            _backend = new OrtRtmposeBackend(yolox, rtm) { detScoreThreshold = 0.3f };
            _rtmpose = new RtmPoseAdapter(_backend) { confThreshold = conf };
            _warmup = Mathf.Max(0, warmup); _emitted = 0; _cursor = 0;
            _freq = Stopwatch.Frequency;

            _baseline.OnSkeletons += f => _metrics.AddFrame("k4abt", f);
            _rtmpose.OnSkeletons += f =>
            {
                if (_measureLat) _metrics.AddLatency("rtmpose", f.Serial, (Stopwatch.GetTimestamp() - _curTick) * 1000.0 / _freq);
                _metrics.AddFrame("rtmpose", f);
            };
            _driver.OnFrame += (serial, frame, cam, ts) =>
            {
                _metrics.AddSubmitted("k4abt", serial);
                _metrics.AddSubmitted("rtmpose", serial);
                _measureLat = _emitted >= _warmup;
                _curTick = Stopwatch.GetTimestamp();
                _rtmpose.SubmitFrame(serial, frame, ts);
            };
            _driver.OnRecordedBodies += (serial, bytes, bc, ts) => _baseline.SubmitRecordedBodies(serial, bytes, bc, ts);

            if (!_driver.Load(sessionRoot)) { Abort(); return "driver.Load failed"; }
            foreach (var dev in _driver.Devices)
            {
                var ctx = new EvalCameraContext(dev.Serial, dev.DepthW, dev.DepthH, dev.ColorW, dev.ColorH, dev.CameraParam);
                _baseline.Configure(ctx);
                _rtmpose.Configure(ctx);
            }

            // world capture volume + per-camera color->world (rejects bystanders / person-switching)
            if (volHx > 0 && volHy > 0 && volHz > 0)
            {
                try
                {
                    var calib = PointCloudRecording.ReadExtrinsicsYaml(sessionRoot);
                    if (calib != null)
                        foreach (var dc in calib)
                            if (dc.GlobalTrColorCamera.HasValue)
                                _rtmpose.SetWorldTransform(dc.Serial, dc.GlobalTrColorCamera.Value);
                }
                catch { }
                _rtmpose.SetCaptureVolume(new Vector3(volCx, volCy, volCz), new Vector3(volHx, volHy, volHz));
            }

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

            _sw = Stopwatch.StartNew();
            return $"ready: {_order.Count} frames, {_driver.Devices.Count} devices, warmup={_warmup}";
        }

        public static string Step(int n)
        {
            if (_driver == null || _order == null) return "not started";
            int end = Math.Min(_cursor + Math.Max(1, n), _order.Count);
            for (; _cursor < end; _cursor++) { var o = _order[_cursor]; _driver.EmitAt(o.dev, o.idx); _emitted++; }
            bool done = _cursor >= _order.Count;
            return $"{_cursor}/{_order.Count}{(done ? " DONE" : "")} elapsed={_sw.Elapsed.TotalSeconds:F1}s";
        }

        public static string Finish(string resultsDir)
        {
            if (_metrics == null) return "not started";
            if (string.IsNullOrWhiteSpace(resultsDir))
                resultsDir = Path.Combine(Directory.GetParent(Application.dataPath).FullName, "eval", "results", "compare");
            _metrics.WriteCsv(resultsDir);
            string sum = $"frames={_cursor}/{(_order != null ? _order.Count : 0)} wall={(_sw != null ? _sw.Elapsed.TotalSeconds : 0):F1}s\nCSV: {resultsDir}\n" + _metrics.BuildSummary();
            Abort();
            return sum;
        }

        public static string Status() =>
            _order == null ? "idle" : $"{_cursor}/{_order.Count} emitted={_emitted}";

        public static void Abort()
        {
            try { _rtmpose?.Dispose(); } catch { }
            try { _baseline?.Dispose(); } catch { }
            if (_go != null) UnityEngine.Object.DestroyImmediate(_go);
            _go = null; _driver = null; _baseline = null; _backend = null; _rtmpose = null;
            _metrics = null; _order = null; _cursor = 0; _emitted = 0;
        }

        static string FirstOnnx(string dir)
        {
            if (!Directory.Exists(dir)) return null;
            var f = Directory.GetFiles(dir, "end2end.onnx", SearchOption.AllDirectories);
            return f.Length > 0 ? f[0] : null;
        }
    }
}
