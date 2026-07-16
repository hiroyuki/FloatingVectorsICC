// 4-camera INTEGRATED (world-space) visualization + agreement metric. For one
// synced frame, runs RTMPose on all cameras, places each camera's point cloud +
// RTMPose skeleton + recorded k4abt skeleton into world (via global_tr_colorCamera,
// the same path the live renderer uses), draws a confidence-weighted FUSED RTMPose
// skeleton (white), and reports per-joint cross-camera agreement (how close the
// cameras' world estimates are — small = good calibration/fusion).
//
//   Run("D:/.../2026-07-14_15-50-24","PAN-SHI", 100, true)

using System;
using System.Collections.Generic;
using System.IO;
using System.Text;
using BodyTracking;
using Calibration;
using Orbbec;
using PointCloud;
using UnityEngine;

namespace BodyTracking.Eval.Rtmpose
{
    public static class RtmposeFused
    {
        static readonly List<IDisposable> _keep = new List<IDisposable>();
        static readonly Color[] CamCols = { new Color(1f, 0.5f, 0.1f), new Color(1f, 0.2f, 0.8f), new Color(1f, 0.9f, 0.1f), new Color(0.2f, 0.6f, 1f) };

        public static string Run(string root, string host, int frameIndex, bool clouds = true)
        {
            var old = GameObject.Find("FusedViz"); if (old != null) UnityEngine.Object.DestroyImmediate(old);
            foreach (var d in _keep) { try { d.Dispose(); } catch { } } _keep.Clear();

            string modelsDir = Path.GetFullPath(Path.Combine(Application.dataPath, "..", "eval", "models"));
            string yolox = First(Path.Combine(modelsDir, "yolox-m"));
            string rtm = First(Path.Combine(modelsDir, "rtmpose-m"));
            if (yolox == null || rtm == null) return "models not found";

            var calib = PointCloudRecording.ReadExtrinsicsYaml(root);
            if (calib == null) return "no extrinsics";

            var backend = new OrtRtmposeBackend(yolox, rtm) { detScoreThreshold = 0.3f };
            var adapter = new RtmPoseAdapter(backend) { confThreshold = 0.3f, redetectEveryN = 1 };

            // capture the adapter's per-frame skeleton
            EvalSkeleton captured = null;
            adapter.OnSkeletons += f => { var p = f.Primary(); captured = p; };

            var sb = new StringBuilder();
            var root2 = new GameObject("FusedViz");
            int nJ = EvalSkeleton.JointCount;
            var accum = new Vector3[nJ]; var accW = new float[nJ]; var perCam = new List<Vector3>[nJ];
            for (int j = 0; j < nJ; j++) perCam[j] = new List<Vector3>();

            // configure adapter for all cameras first (volume selection)
            int ci = 0;
            var serials = new List<string>();
            foreach (var dc in calib)
            {
                var cam = BuildCam(dc);
                var ctx = new EvalCameraContext(dc.Serial, dc.DepthIntrinsic.Width, dc.DepthIntrinsic.Height, dc.ColorIntrinsic.Width, dc.ColorIntrinsic.Height, cam);
                adapter.Configure(ctx);
                if (dc.GlobalTrColorCamera.HasValue) adapter.SetWorldTransform(dc.Serial, dc.GlobalTrColorCamera.Value);
                serials.Add(dc.Serial);
            }
            adapter.SetCaptureVolume(new Vector3(0, 200, 3000), new Vector3(1100, 1500, 1100));

            foreach (var dc in calib)
            {
                if (!dc.GlobalTrColorCamera.HasValue) continue;
                string serial = dc.Serial;
                var cam = BuildCam(dc);
                string colorPath = PointCloudRecording.SensorFilePath(root, host, serial, PointCloudRecording.ColorSensorName);
                string depthPath = PointCloudRecording.SensorFilePath(root, host, serial, PointCloudRecording.DepthSensorName);
                string bodiesPath = PointCloudRecording.SensorFilePath(root, host, serial, PointCloudRecording.BodiesSensorName);
                if (!File.Exists(colorPath) || !File.Exists(depthPath)) continue;

                byte[] color, depth; int cw, ch, dw, dh; ulong ts;
                using (var cs = new PointCloudRecording.RcsvFrameStream(colorPath)) { if (frameIndex >= cs.Count) continue; var f = cs[frameIndex]; color = Copy(f); ts = f.TimestampNs; }
                (cw, ch) = PointCloudRecording.ReadRcsvHeaderDimensions(colorPath);
                using (var ds = new PointCloudRecording.RcsvFrameStream(depthPath)) { depth = Copy(ds[Nearest(ds, ts)]); }
                (dw, dh) = PointCloudRecording.ReadRcsvHeaderDimensions(depthPath);

                // per-camera parent placed in world (extrinsic) + flipY
                var parent = new GameObject("cam_" + serial);
                parent.transform.SetParent(root2.transform, false);
                ExtrinsicsApply.ApplyToTransform(parent.transform, dc.GlobalTrColorCamera.Value);
                parent.transform.localScale = new Vector3(1f, -1f, 1f);
                Color col = CamCols[ci % CamCols.Length];

                if (clouds)
                {
                    var recon = new PointCloudReconstructor("fused_" + serial);
                    if (recon.Dispatch(depth, depth.Length, dw, dh, color, color.Length, cw, ch, cam)) { _keep.Add(recon); var cg = new GameObject("cloud"); cg.transform.SetParent(parent.transform, false); cg.AddComponent<MeshFilter>().sharedMesh = recon.Mesh; var mr = cg.AddComponent<MeshRenderer>(); var sh = Shader.Find("Orbbec/PointCloudUnlit"); if (sh != null) mr.sharedMaterial = new Material(sh); }
                    else recon.Dispose();
                }

                // RTMPose (world via parent) — localPos = colorMm * 0.001
                captured = null;
                var raw = new RawFrameData(depth, depth.Length, dw, dh, color, color.Length, cw, ch, Array.Empty<byte>(), 0, 0, 0, ts / 1000UL);
                adapter.SubmitFrame(serial, raw, ts);
                int rtValid = 0;
                if (captured != null)
                    for (int j = 0; j < nJ; j++)
                    {
                        if (!captured.Joints[j].Valid) continue;
                        rtValid++;
                        Vector3 local = captured.Joints[j].PositionMm * 0.001f;
                        SpawnSphere(parent.transform, $"rt_{(EvalJointId)j}", local, 0.05f, col);
                        Vector3 world = parent.transform.TransformPoint(local);
                        accum[j] += world; accW[j] += 1f; perCam[j].Add(world);
                    }

                // k4abt (recorded) -> color via D2C -> world via parent
                int k4Valid = SpawnK4abt(parent.transform, bodiesPath, ts, cam);
                sb.AppendLine($"{serial}: rtmpose valid={rtValid}/15, k4abt joints={k4Valid}");
                ci++;
            }

            // fused RTMPose (confidence-weighted avg == simple mean here) + agreement
            float spreadSum = 0; int spreadN = 0;
            for (int j = 0; j < nJ; j++)
            {
                if (accW[j] < 1f) continue;
                Vector3 mean = accum[j] / accW[j];
                SpawnSphere(root2.transform, $"fused_{(EvalJointId)j}", mean, 0.07f, Color.white);
                if (perCam[j].Count >= 2)
                {
                    float s = 0; foreach (var p in perCam[j]) s += (p - mean).sqrMagnitude; s = Mathf.Sqrt(s / perCam[j].Count) * 1000f; // mm
                    spreadSum += s; spreadN++;
                }
            }
            adapter.Dispose();
            sb.AppendLine($"FUSED cross-camera agreement (mean per-joint spread) = {(spreadN > 0 ? spreadSum / spreadN : 0):F1} mm over {spreadN} joints");
            sb.AppendLine("colors: cam0=orange cam1=magenta cam2=yellow cam3=blue, k4abt=green, FUSED=white");
            return sb.ToString();
        }

        static int SpawnK4abt(Transform parent, string bodiesPath, ulong ts, ObCameraParam cam)
        {
            if (!File.Exists(bodiesPath)) return 0;
            using var bs = new PointCloudRecording.RcsvFrameStream(bodiesPath);
            int bi = Nearest(bs, ts); if (bi < 0) return 0;
            var f = bs[bi];
            var buf = new BodySnapshot[6]; for (int i = 0; i < buf.Length; i++) buf[i] = new BodySnapshot();
            int n = RecordedBodySerializer.Decode(f.Bytes, f.ByteCount, buf); if (n <= 0) return 0;
            var body = buf[0]; var R = cam.Transform.Rot; var T = cam.Transform.Trans; int cnt = 0;
            for (int ji = 0; ji < K4ABTConsts.K4ABT_JOINT_COUNT; ji++)
            {
                if (!BodyTrackingShared.IsDrawnJoint((k4abt_joint_id_t)ji)) continue;
                var jt = body.Joints[ji];
                if (jt.ConfidenceLevel <= k4abt_joint_confidence_level_t.K4ABT_JOINT_CONFIDENCE_NONE) continue;
                var p = jt.Position;
                Vector3 colorMm = new Vector3(R[0] * p.X + R[1] * p.Y + R[2] * p.Z + T[0], R[3] * p.X + R[4] * p.Y + R[5] * p.Z + T[1], R[6] * p.X + R[7] * p.Y + R[8] * p.Z + T[2]);
                SpawnSphere(parent, $"k4_{(k4abt_joint_id_t)ji}", colorMm * 0.001f, 0.035f, Color.green);
                cnt++;
            }
            return cnt;
        }

        static void SpawnSphere(Transform parent, string name, Vector3 localPos, float scale, Color col)
        {
            var s = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            s.name = name; s.transform.SetParent(parent, false); s.transform.localPosition = localPos; s.transform.localScale = Vector3.one * scale;
            var r = s.GetComponent<Renderer>(); if (r != null) r.material.color = col;
        }

        static ObCameraParam BuildCam(PointCloudRecording.DeviceCalibration dc) => new ObCameraParam { DepthIntrinsic = dc.DepthIntrinsic, RgbIntrinsic = dc.ColorIntrinsic, DepthDistortion = dc.DepthDistortion, RgbDistortion = dc.ColorDistortion, Transform = dc.DepthToColor, IsMirrored = false };
        static string First(string dir) { if (!Directory.Exists(dir)) return null; var f = Directory.GetFiles(dir, "end2end.onnx", SearchOption.AllDirectories); return f.Length > 0 ? f[0] : null; }
        static byte[] Copy(PointCloudRecording.Frame f) { var b = new byte[f.ByteCount]; Buffer.BlockCopy(f.Bytes, 0, b, 0, f.ByteCount); return b; }
        static int Nearest(PointCloudRecording.RcsvFrameStream s, ulong ts) { int best = -1; ulong bd = ulong.MaxValue; for (int i = 0; i < s.Count; i++) { ulong t = s.TimestampNsAt(i), d = t > ts ? t - ts : ts - t; if (d < bd) { bd = d; best = i; } } return best; }
    }
}
