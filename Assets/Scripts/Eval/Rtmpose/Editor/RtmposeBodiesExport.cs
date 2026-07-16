// Export RTMPose skeletons as bodies_main RCSV files so the PRODUCTION scene can
// replay them through its existing recorded-bodies path (SkeletonMerger.
// ignoreRecordedBodies=false) — a k4abt-vs-RTMPose A/B of the actual final
// visuals with zero production-code changes.
//
// Frame conventions: bodies_main stores k4abt-layout joints in the DEPTH camera
// frame (mm). RTMPose 3D is in the COLOR frame, so positions are converted with
// the inverse D2C (p_d = R^T (p_c - T)). The 15 estimated joints are written with
// MEDIUM confidence; spine/clavicle/face/hand joints are synthesized from the
// pelvis-neck-head chain with LOW confidence so bone-driven production effects
// don't silently vanish.
//
// Chunked (same pattern as RtmposeCompareChunked):
//   Start(sessionRoot, outDir, maxFramesPerDevice)  -> Step(150) ... -> Finish()
//
// Output mirrors the session-root layout the replay path actually reads
// (SensorRecorder.Load / PointCloudRecording.EnumerateDevices):
//   <outDir>/dataset/<host>/FemtoBolt_<serial>/bodies_main
// plus copies of the small session yamls + calibration, so <outDir> becomes a
// playable root once the big depth/color/ir streams are hardlinked/copied in
// (see eval docs — they are deliberately NOT duplicated here).

using System;
using System.Collections.Generic;
using System.IO;
using Orbbec;
using PointCloud;
using UnityEngine;

namespace BodyTracking.Eval.Rtmpose
{
    public static class RtmposeBodiesExport
    {
        static GameObject _go;
        static EvalReplayDriver _driver;
        static OrtRtmposeBackend _backend;
        static RtmPoseAdapter _adapter;
        static Dictionary<string, PointCloudRecording.RcsvStreamWriter> _writers;
        static Dictionary<string, ObExtrinsic> _d2c;
        static List<(int dev, int idx)> _order;
        static int _cursor;
        static string _outDir;
        static BodySnapshot _snap;
        static byte[] _scratch;
        static int _written;

        public static string Start(string sessionRoot, string outDir, int maxFramesPerDevice)
        {
            Abort();
            string modelsDir = Path.GetFullPath(Path.Combine(Application.dataPath, "..", "eval", "models"));
            string yolox = First(Path.Combine(modelsDir, "yolox-m"));
            string rtm = First(Path.Combine(modelsDir, "rtmpose-m"));
            if (yolox == null || rtm == null) return "models not found";

            _outDir = string.IsNullOrWhiteSpace(outDir)
                ? Path.Combine(Directory.GetParent(Application.dataPath).FullName, "eval", "results", "rtbodies")
                : outDir;
            Directory.CreateDirectory(_outDir);

            // Map serial -> device dir RELATIVE to the session root, so the output
            // mirrors the dataset/<host>/FemtoBolt_<serial>/ layout the replay reads.
            var relDeviceDir = new Dictionary<string, string>();
            string rootFull = Path.GetFullPath(sessionRoot).TrimEnd(Path.DirectorySeparatorChar, Path.AltDirectorySeparatorChar);
            foreach (var (serial, deviceDir) in PointCloudRecording.EnumerateDevices(sessionRoot))
            {
                string devFull = Path.GetFullPath(deviceDir);
                if (devFull.StartsWith(rootFull, StringComparison.OrdinalIgnoreCase))
                    relDeviceDir[serial] = devFull.Substring(rootFull.Length).TrimStart(Path.DirectorySeparatorChar, Path.AltDirectorySeparatorChar);
            }

            // Copy the small session metadata so <outDir> is a near-ready replay root.
            foreach (var yaml in new[] { "configuration.yaml", "dataset.yaml", "sensor_node_config.yaml" })
            {
                string src = Path.Combine(sessionRoot, yaml);
                if (File.Exists(src)) File.Copy(src, Path.Combine(_outDir, yaml), true);
            }
            string calSrc = Path.Combine(PointCloudRecording.CalibrationDir(sessionRoot), "extrinsics.yaml");
            if (File.Exists(calSrc))
            {
                Directory.CreateDirectory(PointCloudRecording.CalibrationDir(_outDir));
                File.Copy(calSrc, Path.Combine(PointCloudRecording.CalibrationDir(_outDir), "extrinsics.yaml"), true);
            }

            _go = new GameObject("RtmposeBodiesExport");
            _driver = _go.AddComponent<EvalReplayDriver>();
            _driver.loadColor = true; _driver.loadIR = false;
            _backend = new OrtRtmposeBackend(yolox, rtm) { detScoreThreshold = 0.3f };
            _adapter = new RtmPoseAdapter(_backend) { confThreshold = 0.3f, redetectEveryN = 30 };
            _writers = new Dictionary<string, PointCloudRecording.RcsvStreamWriter>();
            _d2c = new Dictionary<string, ObExtrinsic>();
            _snap = new BodySnapshot { Id = 1 };
            _scratch = new byte[RecordedBodySerializer.FrameSize(1)];
            _written = 0;

            _adapter.OnSkeletons += OnSkeletons;
            _driver.OnFrame += (serial, frame, cam, ts) => _adapter.SubmitFrame(serial, frame, ts);

            if (!_driver.Load(sessionRoot)) { Abort(); return "load failed"; }
            foreach (var dev in _driver.Devices)
            {
                var ctx = new EvalCameraContext(dev.Serial, dev.DepthW, dev.DepthH, dev.ColorW, dev.ColorH, dev.CameraParam);
                _adapter.Configure(ctx);
                if (dev.CameraParam.HasValue) _d2c[dev.Serial] = dev.CameraParam.Value.Transform;
                // Write into the mirrored dataset layout (fall back to a flat file
                // only if the device dir couldn't be resolved).
                string devOut = relDeviceDir.TryGetValue(dev.Serial, out var rel)
                    ? Path.Combine(_outDir, rel)
                    : _outDir;
                Directory.CreateDirectory(devOut);
                _writers[dev.Serial] = new PointCloudRecording.RcsvStreamWriter(
                    Path.Combine(devOut, PointCloudRecording.BodiesSensorName),
                    PointCloudRecording.BuildBodiesHeaderYaml(dev.Serial));
            }
            try
            {
                var calib = PointCloudRecording.ReadExtrinsicsYaml(sessionRoot);
                if (calib != null) foreach (var dc in calib) if (dc.GlobalTrColorCamera.HasValue) _adapter.SetWorldTransform(dc.Serial, dc.GlobalTrColorCamera.Value);
            }
            catch { }
            _adapter.SetCaptureVolume(new Vector3(0, 200, 3000), new Vector3(1100, 1500, 1100));

            int cap = maxFramesPerDevice > 0 ? maxFramesPerDevice : int.MaxValue;
            var withTs = new List<(ulong ts, int dev, int idx)>();
            for (int d = 0; d < _driver.Devices.Count; d++)
            {
                var mt = _driver.Devices[d].MasterTs;
                int m = Math.Min(mt.Length, cap);
                for (int i = 0; i < m; i++) withTs.Add((mt[i], d, i));
            }
            withTs.Sort((a, b) => a.ts.CompareTo(b.ts));
            _order = new List<(int, int)>();
            foreach (var w in withTs) _order.Add((w.dev, w.idx));
            _cursor = 0;
            return $"ready: {_order.Count} frames -> {_outDir}";
        }

        static void OnSkeletons(EvalSkeletonFrame f)
        {
            var p = f.Primary();
            if (p == null || !_writers.TryGetValue(f.Serial, out var w) || !_d2c.TryGetValue(f.Serial, out var e)) return;

            // reset all 32 joints to NONE
            for (int i = 0; i < K4ABTConsts.K4ABT_JOINT_COUNT; i++)
                _snap.Joints[i] = new k4abt_joint_t
                {
                    Position = new k4a_float3_t(),
                    Orientation = new k4a_quaternion_t { W = 1f },
                    ConfidenceLevel = k4abt_joint_confidence_level_t.K4ABT_JOINT_CONFIDENCE_NONE,
                };

            // measured 15 joints (color -> depth frame), MEDIUM confidence
            for (int j = 0; j < EvalSkeleton.JointCount; j++)
            {
                if (!p.Joints[j].Valid) continue;
                Vector3 d = ColorToDepth(p.Joints[j].PositionMm, e);
                SetJoint(EvalSkeletonMap.K4abtSource[j], d, k4abt_joint_confidence_level_t.K4ABT_JOINT_CONFIDENCE_MEDIUM);
            }

            // synthesized fill (LOW confidence) so bone-driven visuals don't vanish
            bool hasPelvis = p.Joints[(int)EvalJointId.Pelvis].Valid;
            bool hasNeck = p.Joints[(int)EvalJointId.Neck].Valid;
            bool hasHead = p.Joints[(int)EvalJointId.Head].Valid;
            if (hasPelvis && hasNeck)
            {
                Vector3 pel = ColorToDepth(p.Joints[(int)EvalJointId.Pelvis].PositionMm, e);
                Vector3 nk = ColorToDepth(p.Joints[(int)EvalJointId.Neck].PositionMm, e);
                SetJoint(k4abt_joint_id_t.K4ABT_JOINT_SPINE_NAVEL, Vector3.Lerp(pel, nk, 1f / 3f), k4abt_joint_confidence_level_t.K4ABT_JOINT_CONFIDENCE_LOW);
                SetJoint(k4abt_joint_id_t.K4ABT_JOINT_SPINE_CHEST, Vector3.Lerp(pel, nk, 2f / 3f), k4abt_joint_confidence_level_t.K4ABT_JOINT_CONFIDENCE_LOW);
                SetJoint(k4abt_joint_id_t.K4ABT_JOINT_CLAVICLE_LEFT, nk, k4abt_joint_confidence_level_t.K4ABT_JOINT_CONFIDENCE_LOW);
                SetJoint(k4abt_joint_id_t.K4ABT_JOINT_CLAVICLE_RIGHT, nk, k4abt_joint_confidence_level_t.K4ABT_JOINT_CONFIDENCE_LOW);
            }
            if (hasHead)
            {
                Vector3 hd = ColorToDepth(p.Joints[(int)EvalJointId.Head].PositionMm, e);
                SetJoint(k4abt_joint_id_t.K4ABT_JOINT_NOSE, hd, k4abt_joint_confidence_level_t.K4ABT_JOINT_CONFIDENCE_LOW);
            }

            int bytes = RecordedBodySerializer.Encode(new[] { _snap }, 1, _scratch);
            w.WriteFrame(f.TimestampNs, _scratch, bytes);
            _written++;
        }

        static void SetJoint(k4abt_joint_id_t id, Vector3 posMm, k4abt_joint_confidence_level_t conf)
        {
            _snap.Joints[(int)id] = new k4abt_joint_t
            {
                Position = new k4a_float3_t { X = posMm.x, Y = posMm.y, Z = posMm.z },
                Orientation = new k4a_quaternion_t { W = 1f },
                ConfidenceLevel = conf,
            };
        }

        static Vector3 ColorToDepth(Vector3 c, in ObExtrinsic e)
        {
            // inverse of depth->color: p_d = R^T (p_c - T)
            var R = e.Rot; var T = e.Trans;
            float x = c.x - T[0], y = c.y - T[1], z = c.z - T[2];
            return new Vector3(
                R[0] * x + R[3] * y + R[6] * z,
                R[1] * x + R[4] * y + R[7] * z,
                R[2] * x + R[5] * y + R[8] * z);
        }

        public static string Step(int n)
        {
            if (_driver == null || _order == null) return "not started";
            int end = Math.Min(_cursor + Math.Max(1, n), _order.Count);
            for (; _cursor < end; _cursor++) { var o = _order[_cursor]; _driver.EmitAt(o.dev, o.idx); }
            return $"{_cursor}/{_order.Count}{(_cursor >= _order.Count ? " DONE" : "")} written={_written}";
        }

        public static string Finish()
        {
            if (_writers == null) return "not started";
            var sb = new System.Text.StringBuilder();
            foreach (var kv in _writers) { sb.AppendLine($"{kv.Key}: {kv.Value.FrameCount} body frames -> {kv.Value.FilePath}"); kv.Value.Dispose(); }
            _writers = null;
            Abort();
            return sb.ToString();
        }

        public static void Abort()
        {
            if (_writers != null) foreach (var kv in _writers) { try { kv.Value.Dispose(); } catch { } }
            _writers = null;
            try { _adapter?.Dispose(); } catch { }
            if (_go != null) UnityEngine.Object.DestroyImmediate(_go);
            _go = null; _driver = null; _adapter = null; _backend = null; _order = null; _cursor = 0; _written = 0;
        }

        static string First(string dir) { if (!Directory.Exists(dir)) return null; var f = Directory.GetFiles(dir, "end2end.onnx", SearchOption.AllDirectories); return f.Length > 0 ? f[0] : null; }
    }
}
