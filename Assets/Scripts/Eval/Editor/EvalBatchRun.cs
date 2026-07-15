// Headless batch entry for a metric run — reproducible, no interactive Play.
//
//   Unity.exe -batchmode -nographics -quit -projectPath <worktree> \
//     -executeMethod BodyTracking.Eval.Editor.EvalBatchRun.RunFromArgs \
//     -evalRoot "D:\...\RecordingBase\2026-07-14_11-29-59" \
//     -evalResults "F:\...\eval\results\11-29-59"
//
// Wires the k4abt baseline (recorded bodies) through the shared harness and
// writes summary.csv + jitter_<tracker>.csv. Track branches add their own
// adapter registration alongside the baseline.

using System;
using UnityEngine;

namespace BodyTracking.Eval.Editor
{
    public static class EvalBatchRun
    {
        /// <summary>Invoked via -executeMethod; reads -evalRoot / -evalResults from the command line.</summary>
        public static void RunFromArgs()
        {
            string root = ArgValue("-evalRoot");
            string results = ArgValue("-evalResults");
            if (string.IsNullOrEmpty(root))
            {
                Debug.LogError("[EvalBatch] missing -evalRoot");
                return;
            }
            Run(root, results);
        }

        public static string Run(string sessionRoot, string resultsDir)
        {
            var go = new GameObject("EvalBatch");
            try
            {
                var driver = go.AddComponent<EvalReplayDriver>();
                var metrics = new EvalMetrics();
                metrics.Configure(EvalMetrics.Config.Default);

                var baseline = new K4abtBaselineAdapter();
                baseline.OnSkeletons += f => metrics.AddFrame(baseline.Name, f);

                driver.OnFrame += (serial, frame, cam, ts) =>
                {
                    metrics.AddSubmitted(baseline.Name, serial);
                    baseline.SubmitFrame(serial, in frame, ts);
                };
                driver.OnRecordedBodies += (serial, bytes, bc, ts) =>
                    baseline.SubmitRecordedBodies(serial, bytes, bc, ts);

                if (!driver.Load(sessionRoot))
                    return "[EvalBatch] load failed";

                foreach (var dev in driver.Devices)
                {
                    var ctx = new EvalCameraContext(dev.Serial, dev.DepthW, dev.DepthH, dev.ColorW, dev.ColorH, dev.CameraParam);
                    baseline.Configure(in ctx);
                }

                driver.RunToEndSync();

                if (string.IsNullOrWhiteSpace(resultsDir))
                    resultsDir = System.IO.Path.Combine(
                        System.IO.Directory.GetParent(Application.dataPath)?.FullName ?? Application.dataPath,
                        "eval", "results", "batch");
                metrics.WriteCsv(resultsDir);

                string summary = metrics.BuildSummary();
                Debug.Log($"[EvalBatch] wrote CSV to {resultsDir}\n{summary}");
                baseline.Dispose();
                return summary;
            }
            finally
            {
                UnityEngine.Object.DestroyImmediate(go);
            }
        }

        private static string ArgValue(string flag)
        {
            var args = Environment.GetCommandLineArgs();
            for (int i = 0; i < args.Length - 1; i++)
                if (string.Equals(args[i], flag, StringComparison.OrdinalIgnoreCase))
                    return args[i + 1];
            return null;
        }
    }
}
