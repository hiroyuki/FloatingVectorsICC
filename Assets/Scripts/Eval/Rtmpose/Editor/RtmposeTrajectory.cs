// Per-frame joint-trajectory export for one camera, to visualize how the
// person-selection improvement changed RTMPose over time. Records the pelvis 3D
// position per frame for the k4abt baseline and RTMPose, to CSV.
//
//   Run(root,"PAN-SHI","CL8F253004L",150,false, 1, "...naive.csv")   // before
//   Run(root,"PAN-SHI","CL8F253004L",150,true, 30, "...fixed.csv")   // after

using System;
using System.IO;
using System.Text;
using PointCloud;
using UnityEngine;

namespace BodyTracking.Eval.Rtmpose
{
    public static class RtmposeTrajectory
    {
        public static string Run(string sessionRoot, string host, string serial, int frames,
                                 bool useVolume, int redetectN, string csvPath)
        {
            string modelsDir = Path.GetFullPath(Path.Combine(Application.dataPath, "..", "eval", "models"));
            string yolox = First(Path.Combine(modelsDir, "yolox-m"));
            string rtm = First(Path.Combine(modelsDir, "rtmpose-m"));
            if (yolox == null || rtm == null) return "models not found";

            var go = new GameObject("RtmposeTrajectory");
            var driver = go.AddComponent<EvalReplayDriver>();
            driver.loadColor = true; driver.loadIR = false;
            var baseline = new K4abtBaselineAdapter();
            var backend = new OrtRtmposeBackend(yolox, rtm) { detScoreThreshold = 0.3f };
            var rtmpose = new RtmPoseAdapter(backend) { confThreshold = 0.3f, redetectEveryN = redetectN };

            Vector3? k4 = null, rt = null;
            baseline.OnSkeletons += f => { if (f.Serial == serial) { var p = f.Primary(); if (p != null) { var j = p.Joints[(int)EvalJointId.Pelvis]; if (j.Valid) k4 = j.PositionMm; } } };
            rtmpose.OnSkeletons += f => { if (f.Serial == serial) { var p = f.Primary(); if (p != null) { var j = p.Joints[(int)EvalJointId.Pelvis]; if (j.Valid) rt = j.PositionMm; } } };
            driver.OnFrame += (s, frame, cam, ts) => rtmpose.SubmitFrame(s, frame, ts);
            driver.OnRecordedBodies += (s, bytes, bc, ts) => baseline.SubmitRecordedBodies(s, bytes, bc, ts);

            try
            {
                if (!driver.Load(sessionRoot)) { Cleanup(go, rtmpose, baseline); return "load failed"; }
                int tgt = -1;
                for (int d = 0; d < driver.Devices.Count; d++)
                {
                    var dev = driver.Devices[d];
                    var ctx = new EvalCameraContext(dev.Serial, dev.DepthW, dev.DepthH, dev.ColorW, dev.ColorH, dev.CameraParam);
                    baseline.Configure(ctx); rtmpose.Configure(ctx);
                    if (dev.Serial == serial) tgt = d;
                }
                if (tgt < 0) { Cleanup(go, rtmpose, baseline); return "serial not found"; }

                if (useVolume)
                {
                    try
                    {
                        var calib = PointCloudRecording.ReadExtrinsicsYaml(sessionRoot);
                        if (calib != null) foreach (var dc in calib) if (dc.GlobalTrColorCamera.HasValue) rtmpose.SetWorldTransform(dc.Serial, dc.GlobalTrColorCamera.Value);
                    }
                    catch { }
                    rtmpose.SetCaptureVolume(new Vector3(0, 200, 3000), new Vector3(1100, 1500, 1100));
                }

                var mt = driver.Devices[tgt].MasterTs;
                int n = Mathf.Min(frames, mt.Length);
                var sb = new StringBuilder();
                sb.AppendLine("frame,ts,k4x,k4y,k4z,rtx,rty,rtz");
                for (int i = 0; i < n; i++)
                {
                    k4 = null; rt = null;
                    driver.EmitAt(tgt, i);
                    sb.Append(i).Append(',').Append(mt[i]).Append(',');
                    sb.Append(k4.HasValue ? $"{k4.Value.x:F1},{k4.Value.y:F1},{k4.Value.z:F1}" : ",,").Append(',');
                    sb.Append(rt.HasValue ? $"{rt.Value.x:F1},{rt.Value.y:F1},{rt.Value.z:F1}" : ",,").AppendLine();
                }
                Directory.CreateDirectory(Path.GetDirectoryName(csvPath));
                File.WriteAllText(csvPath, sb.ToString());
                Cleanup(go, rtmpose, baseline);
                return $"wrote {csvPath} ({n} frames)";
            }
            catch (Exception e) { Cleanup(go, rtmpose, baseline); return "EXCEPTION: " + e; }
        }

        static string First(string dir)
        {
            if (!Directory.Exists(dir)) return null;
            var f = Directory.GetFiles(dir, "end2end.onnx", SearchOption.AllDirectories);
            return f.Length > 0 ? f[0] : null;
        }

        static void Cleanup(GameObject go, RtmPoseAdapter r, K4abtBaselineAdapter b)
        {
            try { r?.Dispose(); } catch { }
            try { b?.Dispose(); } catch { }
            if (go != null) UnityEngine.Object.DestroyImmediate(go);
        }
    }
}
