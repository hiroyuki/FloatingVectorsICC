// Export the FUSED multi-camera RTMPose skeleton (FusedRtmposeAdapter — the
// 3-stage occlusion-aware fusion) as bodies_main RCSV files, so the production
// scene replays it through the recorded-bodies path exactly like the k4abt /
// per-camera-RTMPose A/B roots (SkeletonMerger.ignoreRecordedBodies=false).
//
// The fused skeleton lives in the origin-camera (world) color frame; every
// camera's track receives the SAME skeleton converted into that camera's depth
// frame (world -> color via inverse GlobalTrColorCamera, color -> depth via
// inverse D2C). The merger then sees four coincident candidates and merges
// them into the identical pose — single-track output would also work, but
// all-track output keeps the session shape identical to the other A/B roots.
//
// Chunked (same pattern as RtmposeBodiesExport):
//   Start(sessionRoot, outDir, maxFramesPerDevice)  -> Step(300) ... -> Finish()
// Streams (depth/color/ir) are NOT duplicated — hardlink them in afterwards.

using System;
using System.Collections.Generic;
using System.IO;
using Orbbec;
using PointCloud;
using UnityEngine;

namespace BodyTracking.Eval.Rtmpose
{
    public static class FusedBodiesExport
    {
        static GameObject _go;
        static EvalReplayDriver _driver;
        static FusedRtmposeAdapter _fused;
        static Dictionary<string, PointCloudRecording.RcsvStreamWriter> _writers;
        static Dictionary<string, ObExtrinsic> _d2c;
        static Dictionary<string, ObExtrinsic> _gTr; // color->world per serial
        static List<(int dev, int idx)> _order;
        static List<ulong> _orderTs;
        static ulong _prevTs;
        static int _cursor;
        static string _outDir;
        static BodySnapshot _snap;
        static byte[] _scratch;
        static int _written;

        /// <summary>The adapter of the current chunked run — for setting debug-probe fields between Start and Step.</summary>
        public static FusedRtmposeAdapter ActiveAdapter => _fused;

        public static string Start(string sessionRoot, string outDir, int maxFramesPerDevice)
        {
            Abort();
            _outDir = string.IsNullOrWhiteSpace(outDir)
                ? Path.Combine(Directory.GetParent(Application.dataPath).FullName, "eval", "results", "fusedbodies")
                : outDir;
            Directory.CreateDirectory(_outDir);

            var relDeviceDir = new Dictionary<string, string>();
            string rootFull = Path.GetFullPath(sessionRoot).TrimEnd(Path.DirectorySeparatorChar, Path.AltDirectorySeparatorChar);
            foreach (var (serial, deviceDir) in PointCloudRecording.EnumerateDevices(sessionRoot))
            {
                string devFull = Path.GetFullPath(deviceDir);
                if (devFull.StartsWith(rootFull, StringComparison.OrdinalIgnoreCase))
                    relDeviceDir[serial] = devFull.Substring(rootFull.Length).TrimStart(Path.DirectorySeparatorChar, Path.AltDirectorySeparatorChar);
            }
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

            _go = new GameObject("FusedBodiesExport");
            _driver = _go.AddComponent<EvalReplayDriver>();
            _driver.loadColor = true; _driver.loadIR = false;
            _fused = new FusedRtmposeAdapter(BtFrameInspectorWindow.SharedBackend()) { ConfThreshold = 0.3f };
            string profilePath = Path.GetFullPath(Path.Combine(Application.dataPath, "..", "eval", "body_profile.json"));
            if (File.Exists(profilePath)) _fused.Profile = BodyProfile.Load(profilePath);
            _writers = new Dictionary<string, PointCloudRecording.RcsvStreamWriter>();
            _d2c = new Dictionary<string, ObExtrinsic>();
            _gTr = new Dictionary<string, ObExtrinsic>();
            _snap = new BodySnapshot { Id = 1 };
            _scratch = new byte[RecordedBodySerializer.FrameSize(1)];
            _written = 0;

            _fused.OnSkeletons += OnFusedSkeletons;
            _driver.OnFrame += (serial, frame, cam, ts) => _fused.SubmitFrame(serial, frame, ts);

            if (!_driver.Load(sessionRoot)) { Abort(); return "load failed"; }
            foreach (var dev in _driver.Devices)
            {
                var ctx = new EvalCameraContext(dev.Serial, dev.DepthW, dev.DepthH, dev.ColorW, dev.ColorH, dev.CameraParam);
                _fused.Configure(ctx);
                if (dev.CameraParam.HasValue) _d2c[dev.Serial] = dev.CameraParam.Value.Transform;
                string devOut = relDeviceDir.TryGetValue(dev.Serial, out var rel)
                    ? Path.Combine(_outDir, rel)
                    : _outDir;
                Directory.CreateDirectory(devOut);
                _writers[dev.Serial] = new PointCloudRecording.RcsvStreamWriter(
                    Path.Combine(devOut, PointCloudRecording.BodiesSensorName),
                    PointCloudRecording.BuildBodiesHeaderYaml(dev.Serial));
            }
            var calib = PointCloudRecording.ReadExtrinsicsYaml(sessionRoot);
            if (calib == null) { Abort(); return "no extrinsics.yaml"; }
            foreach (var dc in calib)
            {
                if (!dc.GlobalTrColorCamera.HasValue) continue;
                _fused.SetWorldTransform(dc.Serial, dc.GlobalTrColorCamera.Value);
                _gTr[dc.Serial] = dc.GlobalTrColorCamera.Value;
            }
            _fused.SetCaptureVolume(new Vector3(0, 200, 3000), new Vector3(1100, 1500, 1100));

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
            _orderTs = new List<ulong>();
            foreach (var w in withTs) { _order.Add((w.dev, w.idx)); _orderTs.Add(w.ts); }
            _cursor = 0;
            _prevTs = 0;
            return $"ready: {_order.Count} frames -> {_outDir} profile={(_fused.Profile != null ? "loaded" : "MISSING")}";
        }

        static void OnFusedSkeletons(EvalSkeletonFrame f)
        {
            var p = f.Primary();
            if (p == null) return;
            foreach (var kv in _writers)
            {
                string serial = kv.Key;
                if (!_d2c.TryGetValue(serial, out var e) || !_gTr.TryGetValue(serial, out var g)) continue;

                for (int i = 0; i < K4ABTConsts.K4ABT_JOINT_COUNT; i++)
                    _snap.Joints[i] = new k4abt_joint_t
                    {
                        Position = new k4a_float3_t(),
                        Orientation = new k4a_quaternion_t { W = 1f },
                        ConfidenceLevel = k4abt_joint_confidence_level_t.K4ABT_JOINT_CONFIDENCE_NONE,
                    };

                for (int j = 0; j < EvalSkeleton.JointCount; j++)
                {
                    if (!p.Joints[j].Valid) continue;
                    Vector3 d = ToDepth(p.Joints[j].PositionMm, g, e);
                    SetJoint(EvalSkeletonMap.K4abtSource[j], d, k4abt_joint_confidence_level_t.K4ABT_JOINT_CONFIDENCE_MEDIUM);
                }

                bool hasPelvis = p.Joints[(int)EvalJointId.Pelvis].Valid;
                bool hasNeck = p.Joints[(int)EvalJointId.Neck].Valid;
                bool hasHead = p.Joints[(int)EvalJointId.Head].Valid;
                if (hasPelvis && hasNeck)
                {
                    Vector3 pel = ToDepth(p.Joints[(int)EvalJointId.Pelvis].PositionMm, g, e);
                    Vector3 nk = ToDepth(p.Joints[(int)EvalJointId.Neck].PositionMm, g, e);
                    SetJoint(k4abt_joint_id_t.K4ABT_JOINT_SPINE_NAVEL, Vector3.Lerp(pel, nk, 1f / 3f), k4abt_joint_confidence_level_t.K4ABT_JOINT_CONFIDENCE_LOW);
                    SetJoint(k4abt_joint_id_t.K4ABT_JOINT_SPINE_CHEST, Vector3.Lerp(pel, nk, 2f / 3f), k4abt_joint_confidence_level_t.K4ABT_JOINT_CONFIDENCE_LOW);
                    SetJoint(k4abt_joint_id_t.K4ABT_JOINT_CLAVICLE_LEFT, nk, k4abt_joint_confidence_level_t.K4ABT_JOINT_CONFIDENCE_LOW);
                    SetJoint(k4abt_joint_id_t.K4ABT_JOINT_CLAVICLE_RIGHT, nk, k4abt_joint_confidence_level_t.K4ABT_JOINT_CONFIDENCE_LOW);
                }
                if (hasHead)
                {
                    Vector3 hd = ToDepth(p.Joints[(int)EvalJointId.Head].PositionMm, g, e);
                    SetJoint(k4abt_joint_id_t.K4ABT_JOINT_NOSE, hd, k4abt_joint_confidence_level_t.K4ABT_JOINT_CONFIDENCE_LOW);
                }

                int bytes = RecordedBodySerializer.Encode(new[] { _snap }, 1, _scratch);
                kv.Value.WriteFrame(f.TimestampNs, _scratch, bytes);
            }
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

        // world -> this camera's color frame -> depth frame (both inverses)
        static Vector3 ToDepth(Vector3 world, in ObExtrinsic colorToWorld, in ObExtrinsic depthToColor)
            => InverseTransform(InverseTransform(world, colorToWorld), depthToColor);

        static Vector3 InverseTransform(Vector3 p, in ObExtrinsic e)
        {
            var R = e.Rot; var T = e.Trans;
            float x = p.x - T[0], y = p.y - T[1], z = p.z - T[2];
            return new Vector3(
                R[0] * x + R[3] * y + R[6] * z,
                R[1] * x + R[4] * y + R[7] * z,
                R[2] * x + R[5] * y + R[8] * z);
        }

        public static string Step(int n)
        {
            if (_driver == null || _order == null) return "not started";
            int end = Math.Min(_cursor + Math.Max(1, n), _order.Count);
            for (; _cursor < end; _cursor++)
            {
                var o = _order[_cursor];
                ulong ts = _orderTs[_cursor];
                // Recorded frame drops leave holes the fusion never gets called
                // for (all cameras drop together on this HW-synced rig) — inject
                // heartbeat beats so the output stream stays at cadence.
                if (_prevTs != 0 && ts > _prevTs && ts - _prevTs > 40_000_000UL)
                {
                    for (ulong t = _prevTs + 33_000_000UL; t + 5_000_000UL < ts; t += 33_000_000UL)
                        _fused.Heartbeat(t);
                }
                _driver.EmitAt(o.dev, o.idx);
                _prevTs = ts;
            }
            return $"{_cursor}/{_order.Count}{(_cursor >= _order.Count ? " DONE" : "")} fusedFrames={_written}";
        }

        public static string Finish()
        {
            if (_writers == null) return "not started";
            _fused?.FlushLag(); // emit the last two lag-ring frames before closing
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
            // FusedRtmposeAdapter.Dispose is a deliberate no-op (shared backend).
            if (_go != null) UnityEngine.Object.DestroyImmediate(_go);
            _go = null; _driver = null; _fused = null; _order = null; _cursor = 0; _written = 0;
        }
    }
}
