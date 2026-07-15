// Stage-2 comparison runner: feeds the recorded k4abt baseline AND RTMPose the
// same replayed frames and writes per-(tracker,serial) metrics to CSV. Runs in
// the interactive Editor (RTMPose needs a GPU/DirectML device), callable from
// MCP execute_code:
//
//   return BodyTracking.Eval.Rtmpose.RtmposeCompare.Run(
//       "D:/Dropbox/projects/ICC/Recordings/RecordingBase/2026-07-14_15-50-24",
//       60, 0.3f, "");   // maxFramesPerDevice (0=all), confThreshold, resultsDir

using System;
using System.Diagnostics;
using System.IO;
using PointCloud;
using UnityEngine;

namespace BodyTracking.Eval.Rtmpose
{
    public static class RtmposeCompare
    {
        public static string Run(string sessionRoot, int maxFramesPerDevice, float conf, string resultsDir)
        {
            string modelsDir = Path.GetFullPath(Path.Combine(Application.dataPath, "..", "eval", "models"));
            string yolox = FirstOnnx(Path.Combine(modelsDir, "yolox-m"));
            string rtm = FirstOnnx(Path.Combine(modelsDir, "rtmpose-m"));
            if (yolox == null || rtm == null) return $"models not found under {modelsDir}";

            var go = new GameObject("RtmposeCompare");
            var driver = go.AddComponent<EvalReplayDriver>();
            driver.loadColor = true; driver.loadIR = false;

            var metrics = new EvalMetrics();
            metrics.Configure(EvalMetrics.Config.Default);
            var baseline = new K4abtBaselineAdapter();
            var backend = new OrtRtmposeBackend(yolox, rtm) { detScoreThreshold = 0.3f };
            var rtmpose = new RtmPoseAdapter(backend) { confThreshold = conf };

            long freq = Stopwatch.Frequency, curTick = 0;
            baseline.OnSkeletons += f => metrics.AddFrame("k4abt", f);
            rtmpose.OnSkeletons += f =>
            {
                metrics.AddLatency("rtmpose", f.Serial, (Stopwatch.GetTimestamp() - curTick) * 1000.0 / freq);
                metrics.AddFrame("rtmpose", f);
            };
            driver.OnFrame += (serial, frame, cam, ts) =>
            {
                metrics.AddSubmitted("k4abt", serial);
                metrics.AddSubmitted("rtmpose", serial);
                curTick = Stopwatch.GetTimestamp();
                rtmpose.SubmitFrame(serial, frame, ts); // synchronous: detect+pose+lift+emit
            };
            driver.OnRecordedBodies += (serial, bytes, bc, ts) => baseline.SubmitRecordedBodies(serial, bytes, bc, ts);

            var sw = Stopwatch.StartNew();
            try
            {
                if (!driver.Load(sessionRoot)) { Cleanup(go, rtmpose, baseline); return "driver.Load failed"; }
                foreach (var dev in driver.Devices)
                {
                    var ctx = new EvalCameraContext(dev.Serial, dev.DepthW, dev.DepthH, dev.ColorW, dev.ColorH, dev.CameraParam);
                    baseline.Configure(ctx);
                    rtmpose.Configure(ctx);
                }
                driver.RunToEndSync(maxFramesPerDevice);
                sw.Stop();

                if (string.IsNullOrWhiteSpace(resultsDir))
                    resultsDir = Path.Combine(Directory.GetParent(Application.dataPath).FullName, "eval", "results", "compare");
                metrics.WriteCsv(resultsDir);
                string sum = metrics.BuildSummary();
                Cleanup(go, rtmpose, baseline);
                return $"devices={CountDevices(sessionRoot)} cap={maxFramesPerDevice} conf={conf} wall={sw.Elapsed.TotalSeconds:F1}s\nCSV: {resultsDir}\n{sum}";
            }
            catch (Exception e) { Debug.LogError("[RtmposeCompare] " + e); Cleanup(go, rtmpose, baseline); return "EXCEPTION: " + e; }
        }

        static int CountDevices(string root)
        {
            int n = 0; foreach (var _ in PointCloudRecording.EnumerateDevices(root)) n++; return n;
        }

        static string FirstOnnx(string dir)
        {
            if (!Directory.Exists(dir)) return null;
            var f = Directory.GetFiles(dir, "end2end.onnx", SearchOption.AllDirectories);
            return f.Length > 0 ? f[0] : null;
        }

        static void Cleanup(GameObject go, RtmPoseAdapter rtmpose, K4abtBaselineAdapter baseline)
        {
            try { rtmpose?.Dispose(); } catch { }
            try { baseline?.Dispose(); } catch { }
            if (go != null) UnityEngine.Object.DestroyImmediate(go);
        }
    }
}
