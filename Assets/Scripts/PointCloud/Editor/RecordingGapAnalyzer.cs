// Diagnostic: read an existing recording's depth_main RCSV per device and
// summarize timestamp gaps. Used to decide whether intermittent playback
// stuttering is rooted at capture time (gaps already baked into the RCSV
// timestamps = device or USB dropped that frame) or at playback time
// (handled separately by the per-Update skip log in SensorRecorder).
//
// Menu: Window > Diag > Analyze Recording Gaps
//
// Picks a recording root folder (defaulting to the active SensorRecorder's
// folderPath, then Application.persistentDataPath/Recordings) and walks
// EnumerateDevices to find every FemtoBolt_<serial>. For each device's
// depth_main RCSV it prints frame count, duration, mean fps, an interval
// histogram bucketed around the expected per-frame period, and the top-N
// largest gaps with absolute timestamps so the user can correlate against
// the recording's wall clock.

using System.Collections.Generic;
using System.IO;
using System.Linq;
using UnityEditor;
using UnityEngine;

namespace PointCloud.EditorTools
{
    public static class RecordingGapAnalyzer
    {
        [MenuItem("Window/Diag/Analyze Recording Gaps")]
        private static void Run()
        {
            string initialDir = ResolveInitialDir();
            string picked = EditorUtility.OpenFolderPanel("Analyze recording at...", initialDir, "");
            if (string.IsNullOrEmpty(picked)) return;

            AnalyzeFolder(picked);
        }

        private static string ResolveInitialDir()
        {
            // Prefer the active recorder's currently-loaded folder so the operator
            // usually just hits Enter on the picker. Resolves via the same helper
            // SensorRecorder itself uses (default folder name + mac override
            // included — this copy used to lack both).
            var rec = Object.FindFirstObjectByType<SensorRecorder>();
            if (rec != null)
            {
                string p = PointCloudRecording.ResolveRecordingRoot(rec.folderPath, rec.folderPathMacOverride);
                if (Directory.Exists(p)) return p;
            }
            string fallback = Path.Combine(Application.persistentDataPath, "Recordings");
            return Directory.Exists(fallback) ? fallback : Application.persistentDataPath;
        }

        private static void AnalyzeFolder(string root)
        {
            var devices = PointCloudRecording.EnumerateDevices(root).ToList();
            if (devices.Count == 0)
            {
                Debug.LogWarning($"[RecordingGapAnalyzer] No FemtoBolt_<serial> directories under {root}/dataset/<host>/");
                return;
            }
            Debug.Log($"[RecordingGapAnalyzer] root={root} devices={devices.Count}");
            foreach (var (serial, deviceDir) in devices)
            {
                string depthPath = Path.Combine(deviceDir, PointCloudRecording.DepthSensorName);
                if (!File.Exists(depthPath))
                {
                    Debug.LogWarning($"[RecordingGapAnalyzer] {serial}: depth_main not found at {depthPath}");
                    continue;
                }
                try
                {
                    var frames = PointCloudRecording.ReadRcsv(depthPath);
                    AnalyzeTrack(serial, frames);
                }
                catch (System.Exception e)
                {
                    Debug.LogError($"[RecordingGapAnalyzer] {serial}: read failed — {e.Message}");
                }
            }
        }

        private static void AnalyzeTrack(string serial, IReadOnlyList<PointCloudRecording.Frame> frames)
        {
            if (frames.Count < 2)
            {
                Debug.LogWarning($"[RecordingGapAnalyzer] {serial}: only {frames.Count} frame(s); nothing to compare.");
                return;
            }

            // Per-frame interval in milliseconds.
            int n = frames.Count;
            var deltaMs = new double[n - 1];
            for (int i = 1; i < n; i++)
                deltaMs[i - 1] = (frames[i].TimestampNs - frames[i - 1].TimestampNs) / 1e6;

            double durationSec = (frames[n - 1].TimestampNs - frames[0].TimestampNs) / 1e9;
            double meanMs = deltaMs.Average();
            double medianMs = Median(deltaMs);
            double fps = durationSec > 0 ? (n - 1) / durationSec : 0;

            // Bucket against the expected interval. Auto-pick expected from
            // median rounded to the nearest common-rate bucket so this works for
            // both 30fps (33.3 ms) and 15fps captures without hardcoding.
            double expectedMs = SnapExpectedInterval(medianMs);
            int[] buckets = new int[6];
            // <0.75x | 0.75-1.25x | 1.25-1.75x | 1.75-2.5x | 2.5-4x | >4x
            foreach (var d in deltaMs)
            {
                double r = d / expectedMs;
                int b = r < 0.75 ? 0
                      : r < 1.25 ? 1
                      : r < 1.75 ? 2
                      : r < 2.5  ? 3
                      : r < 4.0  ? 4
                      : 5;
                buckets[b]++;
            }

            int onTime = buckets[1];
            int suspect = n - 1 - onTime;  // anything not in the "normal" bucket

            // Top 10 worst gaps with frame index + absolute time-from-start.
            var topGaps = deltaMs
                .Select((d, i) => (i, d))
                .OrderByDescending(t => t.d)
                .Take(10)
                .ToList();

            var sb = new System.Text.StringBuilder();
            sb.AppendLine($"--- {serial} ---");
            sb.AppendLine($"  frames={n} duration={durationSec:F2}s fps={fps:F2}");
            sb.AppendLine($"  interval median={medianMs:F2}ms mean={meanMs:F2}ms expected={expectedMs:F2}ms (== {1000.0 / expectedMs:F1}fps target)");
            sb.AppendLine($"  on-time intervals (0.75-1.25x): {onTime}/{n - 1}    suspect: {suspect}");
            sb.AppendLine($"  histogram:");
            sb.AppendLine($"    <0.75x ({expectedMs * 0.75:F1}ms)  : {buckets[0]}");
            sb.AppendLine($"    0.75-1.25x (~normal)               : {buckets[1]}");
            sb.AppendLine($"    1.25-1.75x (~1 frame skipped)      : {buckets[2]}");
            sb.AppendLine($"    1.75-2.5x  (~2 frames skipped)     : {buckets[3]}");
            sb.AppendLine($"    2.5-4.0x   (~3 frames skipped)     : {buckets[4]}");
            sb.AppendLine($"    >4.0x      (severe stall)          : {buckets[5]}");
            sb.AppendLine($"  top 10 largest gaps:");
            ulong tBaseNs = frames[0].TimestampNs;
            foreach (var (i, d) in topGaps)
            {
                double atSec = (frames[i + 1].TimestampNs - tBaseNs) / 1e9;
                double mult = d / expectedMs;
                sb.AppendLine($"    [{i + 1}/{n}] dt={d,7:F2}ms ({mult:F2}x) at t={atSec:F2}s");
            }
            Debug.Log(sb.ToString());
        }

        // Pick the most likely "intended" inter-frame interval from a median
        // observation. Snaps to the nearest common camera rate so noise around
        // 33.4ms (live observed) still maps to the 30fps target (33.33ms).
        private static double SnapExpectedInterval(double medianMs)
        {
            double[] candidates = { 1000.0 / 60.0, 1000.0 / 30.0, 1000.0 / 15.0, 1000.0 / 10.0, 1000.0 / 5.0 };
            double best = candidates[0];
            double bestDist = System.Math.Abs(medianMs - candidates[0]);
            for (int i = 1; i < candidates.Length; i++)
            {
                double d = System.Math.Abs(medianMs - candidates[i]);
                if (d < bestDist) { bestDist = d; best = candidates[i]; }
            }
            return best;
        }

        private static double Median(double[] values)
        {
            var sorted = (double[])values.Clone();
            System.Array.Sort(sorted);
            int n = sorted.Length;
            return n % 2 == 1 ? sorted[n / 2] : (sorted[n / 2 - 1] + sorted[n / 2]) * 0.5;
        }
    }
}
