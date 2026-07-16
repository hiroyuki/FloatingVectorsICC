// Body-profile calibration, step 1: scan the whole session for frames where
// the 4 cameras' RTMPose skeletons AGREE — i.e. every joint has >= minCams
// valid samples and every camera sits within gateMm of the median of the
// others. Those "good frames" are the calibration set the user visually
// confirms (Frame Inspector) before bone lengths are extracted into a body
// profile. Chunked like RtmposeCompareChunked so each MCP execute_code call
// stays under the response timeout (static state persists — do NOT recompile
// mid-run).
//
//   GoodFrameScan.Start("D:/.../2026-07-14_15-50-24", 0.3f, 3)  // conf, stride
//   GoodFrameScan.Step(300)     // repeat until DONE
//   GoodFrameScan.Finish(120f, 3, "")  // gateMm, minCams, resultsDir("" = eval/results)
//
// Output: CSV of every evaluated reference frame (frame index on the
// reference timeline = first device's depth stream, same as the Frame
// Inspector) with per-frame worst-joint deviation, plus a ranked list of the
// best candidates in the return string.

using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Text;
using PointCloud;
using UnityEngine;

namespace BodyTracking.Eval.Rtmpose
{
    public static class GoodFrameScan
    {
        sealed class Sample
        {
            public ulong Ts;
            public readonly Vector3[] World = new Vector3[EvalSkeleton.JointCount];
            public readonly float[] Conf = new float[EvalSkeleton.JointCount];
            public readonly bool[] Valid = new bool[EvalSkeleton.JointCount];
        }

        static GameObject _go;
        static EvalReplayDriver _driver;
        static OrtRtmposeBackend _backend;
        static RtmPoseAdapter _rtmpose;
        static List<(int dev, int idx)> _order;
        static int _cursor;
        static Stopwatch _sw;
        static Dictionary<string, List<Sample>> _samples;
        static Dictionary<string, Matrix4x4> _toWorld;
        static List<int> _refIndices; // sampled indices on device 0 (reference timeline)

        // Same validated world capture volume as RtmposeCompareChunked.
        public static float volCx = 0, volCy = 200, volCz = 3000, volHx = 1000, volHy = 1400, volHz = 900;

        public static string Start(string sessionRoot, float conf, int stride)
        {
            Abort();
            string modelsDir = Path.GetFullPath(Path.Combine(Application.dataPath, "..", "eval", "models"));
            string yolox = FirstOnnx(Path.Combine(modelsDir, "yolox-m"));
            string rtm = FirstOnnx(Path.Combine(modelsDir, "rtmpose-m"));
            if (yolox == null || rtm == null) return "models not found";
            stride = Mathf.Max(1, stride);

            _go = new GameObject("GoodFrameScan");
            _driver = _go.AddComponent<EvalReplayDriver>();
            _driver.loadColor = true; _driver.loadIR = false;
            _backend = new OrtRtmposeBackend(yolox, rtm) { detScoreThreshold = 0.3f };
            _rtmpose = new RtmPoseAdapter(_backend) { confThreshold = conf };
            _samples = new Dictionary<string, List<Sample>>();
            _toWorld = new Dictionary<string, Matrix4x4>();

            _rtmpose.OnSkeletons += f =>
            {
                var p = f.Primary();
                if (p == null) return;
                Matrix4x4 m;
                if (!_toWorld.TryGetValue(f.Serial, out m)) return;
                var s = new Sample { Ts = p.TimestampNs };
                for (int j = 0; j < EvalSkeleton.JointCount; j++)
                {
                    s.Valid[j] = p.Joints[j].Valid;
                    s.Conf[j] = p.Joints[j].Confidence;
                    if (s.Valid[j]) s.World[j] = m.MultiplyPoint3x4(p.Joints[j].PositionMm * 0.001f);
                }
                List<Sample> list;
                if (!_samples.TryGetValue(f.Serial, out list)) { list = new List<Sample>(); _samples[f.Serial] = list; }
                list.Add(s);
            };
            _driver.OnFrame += (serial, frame, cam, ts) => _rtmpose.SubmitFrame(serial, frame, ts);

            if (!_driver.Load(sessionRoot)) { Abort(); return "driver.Load failed"; }
            foreach (var dev in _driver.Devices)
            {
                var ctx = new EvalCameraContext(dev.Serial, dev.DepthW, dev.DepthH, dev.ColorW, dev.ColorH, dev.CameraParam);
                _rtmpose.Configure(ctx);
            }

            var calib = PointCloudRecording.ReadExtrinsicsYaml(sessionRoot);
            if (calib == null) { Abort(); return "no extrinsics.yaml"; }
            foreach (var dc in calib)
            {
                if (!dc.GlobalTrColorCamera.HasValue) continue;
                _rtmpose.SetWorldTransform(dc.Serial, dc.GlobalTrColorCamera.Value);
                var ext = dc.GlobalTrColorCamera.Value;
                Vector3 pos; Quaternion rot;
                Calibration.ExtrinsicsApply.ToUnityLocal(ext, out pos, out rot);
                _toWorld[dc.Serial] = Matrix4x4.TRS(pos, rot, new Vector3(1f, -1f, 1f));
            }
            _rtmpose.SetCaptureVolume(new Vector3(volCx, volCy, volCz), new Vector3(volHx, volHy, volHz));

            // Emit in global ts order (keeps per-camera tracking warm), but only
            // every stride-th index per device. Device 0's sampled indices are the
            // reference frames reported to the user.
            _order = new List<(int, int)>();
            _refIndices = new List<int>();
            var withTs = new List<(ulong ts, int dev, int idx)>();
            for (int d = 0; d < _driver.Devices.Count; d++)
            {
                var mt = _driver.Devices[d].MasterTs;
                for (int i = 0; i < mt.Length; i++)
                {
                    if (i % stride != 0) continue;
                    withTs.Add((mt[i], d, i));
                    if (d == 0) _refIndices.Add(i);
                }
            }
            withTs.Sort((a, b) => a.ts.CompareTo(b.ts));
            foreach (var w in withTs) _order.Add((w.dev, w.idx));
            _cursor = 0;
            _sw = Stopwatch.StartNew();
            return $"ready: {_order.Count} emissions, {_driver.Devices.Count} devices, {_refIndices.Count} reference frames, stride={stride}";
        }

        public static string Step(int n)
        {
            if (_driver == null || _order == null) return "not started";
            int end = Math.Min(_cursor + Math.Max(1, n), _order.Count);
            for (; _cursor < end; _cursor++) { var o = _order[_cursor]; _driver.EmitAt(o.dev, o.idx); }
            bool done = _cursor >= _order.Count;
            return $"{_cursor}/{_order.Count}{(done ? " DONE" : "")} elapsed={_sw.Elapsed.TotalSeconds:F1}s";
        }

        public static string Finish(float gateMm, int minCams, string resultsDir)
        {
            if (_driver == null || _samples == null) return "not started";
            if (string.IsNullOrEmpty(resultsDir))
                resultsDir = Path.GetFullPath(Path.Combine(Application.dataPath, "..", "eval", "results"));
            Directory.CreateDirectory(resultsDir);

            var refDev = _driver.Devices[0];
            var refTs = refDev.MasterTs;
            var serials = new List<string>(_samples.Keys);
            serials.Sort();

            // ts-indexed lookup per serial (samples were appended in ts order)
            const double tolMs = 20.0;
            var csv = new StringBuilder();
            csv.AppendLine("refFrame,tsNs,cams,worstJoint,worstDevMm,meanDevMm,jointsOk");
            // (refFrame, worstDev, meanDev) of frames where ALL joints pass
            var good = new List<(int frame, float worst, float mean)>();

            foreach (int fi in _refIndices)
            {
                ulong ts = refTs[fi];
                // nearest sample per serial
                var frameSamples = new List<Sample>();
                foreach (var s in serials)
                {
                    var list = _samples[s];
                    Sample bestS = null; double bd = double.MaxValue;
                    foreach (var smp in list)
                    {
                        double d = Math.Abs((double)(long)(smp.Ts - ts)) / 1e6;
                        if (d < bd) { bd = d; bestS = smp; }
                    }
                    if (bestS != null && bd <= tolMs) frameSamples.Add(bestS);
                }
                if (frameSamples.Count < minCams) continue;

                float worst = 0f; float devSum = 0f; int devN = 0; int jointsOk = 0;
                string worstName = "";
                bool allOk = true;
                for (int j = 0; j < EvalSkeleton.JointCount; j++)
                {
                    var pts = new List<Vector3>();
                    foreach (var smp in frameSamples) if (smp.Valid[j]) pts.Add(smp.World[j]);
                    if (pts.Count < minCams) { allOk = false; continue; }
                    float jointWorst = 0f;
                    for (int i = 0; i < pts.Count; i++)
                    {
                        var ox = new List<float>(); var oy = new List<float>(); var oz = new List<float>();
                        for (int o = 0; o < pts.Count; o++) { if (o == i) continue; ox.Add(pts[o].x); oy.Add(pts[o].y); oz.Add(pts[o].z); }
                        ox.Sort(); oy.Sort(); oz.Sort();
                        var med = new Vector3(ox[ox.Count / 2], oy[oy.Count / 2], oz[oz.Count / 2]);
                        float dev = Vector3.Distance(pts[i], med) * 1000f;
                        if (dev > jointWorst) jointWorst = dev;
                        devSum += dev; devN++;
                    }
                    if (jointWorst > worst) { worst = jointWorst; worstName = ((EvalJointId)j).ToString(); }
                    if (jointWorst <= gateMm) jointsOk++;
                    else allOk = false;
                }
                float mean = devN > 0 ? devSum / devN : 0f;
                csv.AppendLine($"{fi},{ts},{frameSamples.Count},{worstName},{worst:F0},{mean:F0},{jointsOk}/15");
                if (allOk) good.Add((fi, worst, mean));
            }

            string csvPath = Path.Combine(resultsDir, "goodframe_scan.csv");
            File.WriteAllText(csvPath, csv.ToString());

            good.Sort((a, b) => a.worst.CompareTo(b.worst));
            var sb = new StringBuilder();
            sb.AppendLine($"evaluated={_refIndices.Count} refFrames, ALL-joints-pass (gate={gateMm}mm, minCams={minCams}): {good.Count}");
            sb.AppendLine("top candidates (refFrame: worstDevMm / meanDevMm):");
            for (int i = 0; i < Math.Min(20, good.Count); i++)
                sb.AppendLine($"  {good[i].frame}: {good[i].worst:F0} / {good[i].mean:F0}");
            sb.AppendLine("csv: " + csvPath);
            return sb.ToString();
        }

        public static string Abort()
        {
            try { _rtmpose?.Dispose(); } catch { }
            if (_go != null) UnityEngine.Object.DestroyImmediate(_go);
            _go = null; _driver = null; _order = null; _samples = null; _toWorld = null; _refIndices = null;
            _rtmpose = null; _backend = null;
            return "aborted";
        }

        static string FirstOnnx(string dir)
        {
            if (!Directory.Exists(dir)) return null;
            var files = Directory.GetFiles(dir, "*.onnx", SearchOption.AllDirectories);
            return files.Length > 0 ? files[0] : null;
        }
    }
}
