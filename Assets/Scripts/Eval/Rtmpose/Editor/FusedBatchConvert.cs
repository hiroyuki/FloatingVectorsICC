// Self-driving batch conversion: every take under a base folder -> fused v11s
// bodies_main in <outBase>/<take>/ (same dataset layout; streams are NOT
// copied — hardlink them in afterwards). Runs on EditorApplication.update so
// the multi-hour GPU run needs no external stepping; progress is polled with
// Status(). Resumable: a take whose output contains a v11s.done marker is
// skipped, so a crashed/aborted run just continues where it left off.
//
//   FusedBatchConvert.Start(@"D:\...\RecordingBase", @"D:\...\RecordingBaseV11s")
//   FusedBatchConvert.Status()   // poll
//   FusedBatchConvert.Stop()     // abort (current take's partial output removed)
//
// Skip rules: no calibration/extrinsics.yaml, no device with BOTH >=30 depth
// frames and a color stream (RTMPose reads color), or already converted.

using System;
using System.Collections.Generic;
using System.IO;
using PointCloud;
using UnityEditor;
using UnityEngine;

namespace BodyTracking.Eval.Rtmpose
{
    public static class FusedBatchConvert
    {
        private sealed class Item { public string Session, Out, Name; }

        private static readonly List<Item> _queue = new List<Item>();
        private static readonly System.Text.StringBuilder _report = new System.Text.StringBuilder();
        private static int _index = -1;
        private static bool _active;    // a take's export is mid-flight
        private static bool _running;
        private static int _done, _failed;
        private static string _lastStep = "";

        /// <summary>Frames fed to FusedBodiesExport per editor tick. ~40 frames
        /// ≈ 2 s of GPU work — long enough to be efficient, short enough that
        /// the editor stays minimally responsive.</summary>
        public static int FramesPerTick = 40;

        public static string Start(string baseDir, string outBase)
        {
            Stop();
            _queue.Clear(); _report.Clear();
            _index = -1; _active = false; _done = 0; _failed = 0;

            if (!Directory.Exists(baseDir)) return "base dir not found: " + baseDir;
            Directory.CreateDirectory(outBase);

            foreach (var dir in Directory.GetDirectories(baseDir))
            {
                string name = Path.GetFileName(dir);
                string outDir = Path.Combine(outBase, name);
                string skip = SkipReason(dir, outDir);
                if (skip != null) { _report.AppendLine($"SKIP {name}: {skip}"); continue; }
                _queue.Add(new Item { Session = dir, Out = outDir, Name = name });
            }
            if (_queue.Count == 0) return "nothing to convert\n" + _report;

            _running = true;
            EditorApplication.update += Tick;
            return $"queued {_queue.Count} take(s)\n" + _report;
        }

        public static string Status()
        {
            string cur = _active && _index >= 0 && _index < _queue.Count
                ? $" current={_queue[_index].Name} [{_lastStep}]" : "";
            return $"running={_running} take {Math.Min(_index + 1, _queue.Count)}/{_queue.Count} done={_done} failed={_failed}{cur}\n"
                   + TailOfReport(25);
        }

        public static string Stop()
        {
            if (!_running && !_active) return "idle";
            EditorApplication.update -= Tick;
            _running = false;
            if (_active)
            {
                FusedBodiesExport.Abort();
                // a half-written take must not look convertible-complete
                if (_index >= 0 && _index < _queue.Count)
                {
                    try { Directory.Delete(_queue[_index].Out, true); } catch { }
                    _report.AppendLine($"ABORTED {_queue[_index].Name} (partial output removed)");
                }
                _active = false;
            }
            return "stopped\n" + TailOfReport(10);
        }

        private static string SkipReason(string dir, string outDir)
        {
            if (File.Exists(Path.Combine(outDir, "v11s.done"))) return "already converted";
            if (!File.Exists(Path.Combine(dir, "calibration", "extrinsics.yaml"))) return "no extrinsics.yaml";
            bool usable = false;
            foreach (var (serial, deviceDir) in PointCloudRecording.EnumerateDevices(dir))
            {
                string depth = Path.Combine(deviceDir, PointCloudRecording.DepthSensorName);
                string color = Path.Combine(deviceDir, PointCloudRecording.ColorSensorName);
                if (!File.Exists(depth) || !File.Exists(color)) continue;
                if (new FileInfo(depth).Length < 30L * 184_324) continue; // < ~1 s of depth
                usable = true; break;
            }
            return usable ? null : "no device with >=30 depth frames + color";
        }

        private static void Tick()
        {
            try
            {
                if (!_running) { EditorApplication.update -= Tick; return; }

                if (!_active)
                {
                    _index++;
                    if (_index >= _queue.Count)
                    {
                        _running = false;
                        EditorApplication.update -= Tick;
                        _report.AppendLine($"ALL DONE: converted={_done} failed={_failed}");
                        Debug.Log($"[FusedBatchConvert] all done: converted={_done} failed={_failed}");
                        return;
                    }
                    var it = _queue[_index];
                    string r = FusedBodiesExport.Start(it.Session, it.Out, 0);
                    if (!r.StartsWith("ready"))
                    {
                        _report.AppendLine($"FAIL {it.Name}: {r}");
                        _failed++;
                        return; // next Tick advances to the next take
                    }
                    _report.AppendLine($"START {it.Name} ({r.Replace("\n", " ")})");
                    _active = true;
                    return;
                }

                _lastStep = FusedBodiesExport.Step(FramesPerTick);
                if (_lastStep.Contains("DONE"))
                {
                    var it = _queue[_index];
                    FusedBodiesExport.Finish();
                    string smooth = FusedCatchupSmooth.Run(it.Out);
                    File.WriteAllText(Path.Combine(it.Out, "v11s.done"),
                        DateTime.UtcNow.ToString("o") + "\n" + smooth);
                    _report.AppendLine($"DONE {it.Name}");
                    Debug.Log($"[FusedBatchConvert] done {it.Name} ({_done + 1}/{_queue.Count})");
                    _done++;
                    _active = false;
                }
            }
            catch (Exception e)
            {
                string name = _index >= 0 && _index < _queue.Count ? _queue[_index].Name : "?";
                _report.AppendLine($"ERROR {name}: {e.Message}");
                Debug.LogException(e);
                try { FusedBodiesExport.Abort(); } catch { }
                _failed++;
                _active = false; // move on to the next take
            }
        }

        private static string TailOfReport(int lines)
        {
            var all = _report.ToString().TrimEnd().Split('\n');
            int from = Math.Max(0, all.Length - lines);
            return string.Join("\n", all, from, all.Length - from);
        }
    }
}
