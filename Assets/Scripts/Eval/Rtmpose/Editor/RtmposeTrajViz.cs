// Visual trajectory comparison: draws the pelvis path over frames for one camera
// as 3D polylines in the scene — k4abt (green), RTMPose "before" (naive best-box,
// red), RTMPose "after" (capture volume + track, cyan) — so the person-switching
// fix is visible as a jagged red line vs a smooth cyan line following green.
//
// Chunk-friendly: Collect one pass per call (naive pass with redetect=1 is slow),
// then Draw().
//
//   Collect(root,"PAN-SHI","CL8F253004L",120,false,1,"before")
//   Collect(root,"PAN-SHI","CL8F253004L",120,true,30,"after")
//   Draw()

using System;
using System.Collections.Generic;
using System.IO;
using Orbbec;
using PointCloud;
using UnityEngine;

namespace BodyTracking.Eval.Rtmpose
{
    public static class RtmposeTrajViz
    {
        static readonly Dictionary<string, List<Vector3>> _traj = new Dictionary<string, List<Vector3>>();

        public static string Reset() { _traj.Clear(); return "cleared"; }

        public static string Collect(string root, string host, string serial, int frames, bool useVolume, int redetectN, string label)
        {
            string modelsDir = Path.GetFullPath(Path.Combine(Application.dataPath, "..", "eval", "models"));
            string yolox = First(Path.Combine(modelsDir, "yolox-m"));
            string rtm = First(Path.Combine(modelsDir, "rtmpose-m"));
            if (yolox == null || rtm == null) return "models not found";

            var go = new GameObject("TrajCollect");
            var driver = go.AddComponent<EvalReplayDriver>();
            driver.loadColor = true; driver.loadIR = false;
            var baseline = new K4abtBaselineAdapter();
            var backend = new OrtRtmposeBackend(yolox, rtm) { detScoreThreshold = 0.3f };
            var rtmpose = new RtmPoseAdapter(backend) { confThreshold = 0.3f, redetectEveryN = redetectN };

            var rtList = new List<Vector3>();
            var k4List = new List<Vector3>();
            _traj[label] = rtList;
            _traj["k4abt"] = k4List;
            ObExtrinsic d2c = default; bool hasCam = false;

            Vector3? k4 = null, rt = null;
            baseline.OnSkeletons += f => { if (f.Serial == serial) { var p = f.Primary(); if (p != null) { var j = p.Joints[(int)EvalJointId.Pelvis]; if (j.Valid) k4 = j.PositionMm; } } };
            rtmpose.OnSkeletons += f => { if (f.Serial == serial) { var p = f.Primary(); if (p != null) { var j = p.Joints[(int)EvalJointId.Pelvis]; if (j.Valid) rt = j.PositionMm; } } };
            driver.OnFrame += (s, frame, cam, ts) => rtmpose.SubmitFrame(s, frame, ts);
            driver.OnRecordedBodies += (s, bytes, bc, ts) => baseline.SubmitRecordedBodies(s, bytes, bc, ts);

            try
            {
                if (!driver.Load(root)) { Clean(go, rtmpose, baseline); return "load failed"; }
                int tgt = -1;
                foreach (var dev in driver.Devices)
                {
                    var ctx = new EvalCameraContext(dev.Serial, dev.DepthW, dev.DepthH, dev.ColorW, dev.ColorH, dev.CameraParam);
                    baseline.Configure(ctx); rtmpose.Configure(ctx);
                }
                for (int d = 0; d < driver.Devices.Count; d++)
                    if (driver.Devices[d].Serial == serial) { tgt = d; if (driver.Devices[d].CameraParam.HasValue) { d2c = driver.Devices[d].CameraParam.Value.Transform; hasCam = true; } }
                if (tgt < 0) { Clean(go, rtmpose, baseline); return "serial not found"; }

                if (useVolume)
                {
                    try { var calib = PointCloudRecording.ReadExtrinsicsYaml(root); if (calib != null) foreach (var dc in calib) if (dc.GlobalTrColorCamera.HasValue) rtmpose.SetWorldTransform(dc.Serial, dc.GlobalTrColorCamera.Value); } catch { }
                    rtmpose.SetCaptureVolume(new Vector3(0, 200, 3000), new Vector3(1100, 1500, 1100));
                }

                var mt = driver.Devices[tgt].MasterTs;
                int n = Mathf.Min(frames, mt.Length);
                for (int i = 0; i < n; i++)
                {
                    k4 = null; rt = null;
                    driver.EmitAt(tgt, i);
                    if (rt.HasValue) rtList.Add(EvalSkeletonMap.CameraMmToUnity(rt.Value));
                    if (k4.HasValue) { var w = hasCam ? ToColor(k4.Value, d2c) : k4.Value; k4List.Add(EvalSkeletonMap.CameraMmToUnity(w)); }
                }
                Clean(go, rtmpose, baseline);
                return $"collected '{label}': rtmpose={rtList.Count}, k4abt={k4List.Count} pts";
            }
            catch (Exception e) { Clean(go, rtmpose, baseline); return "EXCEPTION: " + e; }
        }

        public static string Draw()
        {
            var parent = GameObject.Find("TrajViz");
            if (parent != null) UnityEngine.Object.DestroyImmediate(parent);
            parent = new GameObject("TrajViz");
            var colors = new Dictionary<string, Color> { { "k4abt", Color.green }, { "before", new Color(1f, 0.2f, 0.2f) }, { "after", Color.cyan } };
            int drawn = 0;
            foreach (var kv in _traj)
            {
                if (kv.Value.Count < 2) continue;
                var go = new GameObject("line_" + kv.Key);
                go.transform.SetParent(parent.transform, false);
                var lr = go.AddComponent<LineRenderer>();
                lr.useWorldSpace = false;
                lr.widthMultiplier = 0.015f;
                lr.positionCount = kv.Value.Count;
                lr.SetPositions(kv.Value.ToArray());
                lr.material = new Material(Shader.Find("Sprites/Default"));
                var col = colors.TryGetValue(kv.Key, out var cc) ? cc : Color.white;
                lr.startColor = lr.endColor = col;
                drawn++;
            }
            return $"drew {drawn} lines (k4abt=green, before=red, after=cyan)";
        }

        static Vector3 ToColor(Vector3 depthMm, in ObExtrinsic e)
        {
            var R = e.Rot; var T = e.Trans;
            return new Vector3(
                R[0] * depthMm.x + R[1] * depthMm.y + R[2] * depthMm.z + T[0],
                R[3] * depthMm.x + R[4] * depthMm.y + R[5] * depthMm.z + T[1],
                R[6] * depthMm.x + R[7] * depthMm.y + R[8] * depthMm.z + T[2]);
        }

        static string First(string dir) { if (!Directory.Exists(dir)) return null; var f = Directory.GetFiles(dir, "end2end.onnx", SearchOption.AllDirectories); return f.Length > 0 ? f[0] : null; }
        static void Clean(GameObject go, RtmPoseAdapter r, K4abtBaselineAdapter b) { try { r?.Dispose(); } catch { } try { b?.Dispose(); } catch { } if (go != null) UnityEngine.Object.DestroyImmediate(go); }
    }
}
