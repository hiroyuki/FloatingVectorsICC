// Single-frame, single-camera RTMPose verification driver, callable from the
// MCP execute_code console. Runs the pipeline stages directly (detect -> pose ->
// SimCC decode -> COCO map -> depth lift) so it can report detection, 2D
// keypoints, scores, depths and 3D positions, and optionally spawn spheres in the
// scene for both RTMPose and the recorded k4abt baseline at the same frame.
//
// Usage (MCP execute_code):
//   return BodyTracking.Eval.Rtmpose.RtmposeVerify.Run(
//       "D:/Dropbox/projects/ICC/Recordings/RecordingBase/2026-07-14_15-50-24",
//       "PAN-SHI", "CL8F253004N", 800, /*yoloxBgr*/ true, /*conf*/ 0.3f, /*spawn*/ true);

using System;
using System.IO;
using System.Text;
using BodyTracking;
using Orbbec;
using PointCloud;
using UnityEngine;

namespace BodyTracking.Eval.Rtmpose
{
    public static class RtmposeVerify
    {
        // Keeps GPU-backed reconstructor meshes alive (their vertex buffers are
        // freed when the reconstructor is disposed/GC'd).
        static readonly System.Collections.Generic.List<IDisposable> _keep = new System.Collections.Generic.List<IDisposable>();

        /// <summary>Per-stage latency breakdown (YOLOX detect / RTMPose pose / depth align+lift).</summary>
        public static string Measure(string sessionRoot, string host, string serial, int frameIndex, int iters)
        {
            string modelsDir = Path.GetFullPath(Path.Combine(Application.dataPath, "..", "eval", "models"));
            string yoloxPath = FirstOnnx(Path.Combine(modelsDir, "yolox-m"));
            string rtmPath = FirstOnnx(Path.Combine(modelsDir, "rtmpose-m"));
            if (yoloxPath == null || rtmPath == null) return "models not found";
            ObCameraParam? cam = LoadCameraParam(sessionRoot, serial);

            string colorPath = PointCloudRecording.SensorFilePath(sessionRoot, host, serial, PointCloudRecording.ColorSensorName);
            string depthPath = PointCloudRecording.SensorFilePath(sessionRoot, host, serial, PointCloudRecording.DepthSensorName);
            byte[] color, depth; int cw, ch, dw, dh; ulong ts;
            using (var cs = new PointCloudRecording.RcsvFrameStream(colorPath)) { var f = cs[frameIndex]; color = Copy(f); ts = f.TimestampNs; }
            (cw, ch) = PointCloudRecording.ReadRcsvHeaderDimensions(colorPath);
            using (var ds = new PointCloudRecording.RcsvFrameStream(depthPath)) { depth = Copy(ds[NearestByTs(ds, ts)]); }
            (dw, dh) = PointCloudRecording.ReadRcsvHeaderDimensions(depthPath);

            var backend = new OrtRtmposeBackend(yoloxPath, rtmPath, true);
            var spec = backend.Spec;
            var boxes = new DetBox[8];
            var lift = new DepthLift();
            var input = new float[3 * spec.InH * spec.InW];
            var sx = new float[backend.NumKeypoints * Mathf.RoundToInt(spec.InW * spec.SplitRatio)];
            var sy = new float[backend.NumKeypoints * Mathf.RoundToInt(spec.InH * spec.SplitRatio)];

            // warmup
            for (int i = 0; i < 5; i++) { backend.Detect(color, cw, ch, boxes); if (cam.HasValue) lift.BuildAligned(depth, dw, dh, cw, ch, cam.Value); backend.Pose(input, sx, sy); }

            var sw = new System.Diagnostics.Stopwatch();
            double tDet = 0, tPose = 0, tAlign = 0;
            for (int i = 0; i < iters; i++) { sw.Restart(); backend.Detect(color, cw, ch, boxes); tDet += sw.Elapsed.TotalMilliseconds; }
            int n = backend.Detect(color, cw, ch, boxes);
            var box = boxes[0];
            var roi = RtmposePreprocess.RoiFromBox(box.X1, box.Y1, box.X2, box.Y2, spec);
            RtmposePreprocess.BuildInput(color, cw, ch, roi, spec, input);
            for (int i = 0; i < iters; i++) { sw.Restart(); backend.Pose(input, sx, sy); tPose += sw.Elapsed.TotalMilliseconds; }
            if (cam.HasValue) for (int i = 0; i < iters; i++) { sw.Restart(); lift.BuildAligned(depth, dw, dh, cw, ch, cam.Value); tAlign += sw.Elapsed.TotalMilliseconds; }
            backend.Dispose();

            return $"per-frame breakdown (iters={iters}, cam {serial}):\n" +
                   $"  YOLOX detect  = {tDet / iters:F1} ms\n" +
                   $"  RTMPose pose  = {tPose / iters:F1} ms\n" +
                   $"  depth align   = {(cam.HasValue ? (tAlign / iters).ToString("F1") : "n/a")} ms (C# {dw}x{dh}->{cw}x{ch})\n" +
                   $"  SUM           = {(tDet + tPose + tAlign) / iters:F1} ms";
        }

        public static string Run(string sessionRoot, string host, string serial, int frameIndex,
                                 bool yoloxBgr = true, float conf = 0.3f, bool spawnViz = true, bool spawnCloud = true)
        {
            var sb = new StringBuilder();
            try
            {
                // ---- models ----
                string modelsDir = Path.GetFullPath(Path.Combine(Application.dataPath, "..", "eval", "models"));
                string yoloxPath = FirstOnnx(Path.Combine(modelsDir, "yolox-m"));
                string rtmPath = FirstOnnx(Path.Combine(modelsDir, "rtmpose-m"));
                if (yoloxPath == null || rtmPath == null) return $"model onnx not found under {modelsDir}";

                // ---- calibration ----
                ObCameraParam? cam = LoadCameraParam(sessionRoot, serial);
                if (cam == null) sb.AppendLine("WARN: no ObCameraParam for serial (3D lift disabled)");

                // ---- frames ----
                string colorPath = PointCloudRecording.SensorFilePath(sessionRoot, host, serial, PointCloudRecording.ColorSensorName);
                string depthPath = PointCloudRecording.SensorFilePath(sessionRoot, host, serial, PointCloudRecording.DepthSensorName);
                string bodiesPath = PointCloudRecording.SensorFilePath(sessionRoot, host, serial, PointCloudRecording.BodiesSensorName);
                if (!File.Exists(colorPath)) return $"color_main not found: {colorPath}";

                byte[] color; int cw, ch; ulong colorTs;
                using (var cs = new PointCloudRecording.RcsvFrameStream(colorPath))
                {
                    if (frameIndex < 0 || frameIndex >= cs.Count) return $"frameIndex {frameIndex} out of range (0..{cs.Count - 1})";
                    var f = cs[frameIndex];
                    color = Copy(f); colorTs = f.TimestampNs;
                }
                (cw, ch) = PointCloudRecording.ReadRcsvHeaderDimensions(colorPath);

                byte[] depth = null; int dw = 0, dh = 0;
                if (File.Exists(depthPath))
                {
                    using var ds = new PointCloudRecording.RcsvFrameStream(depthPath);
                    int di = NearestByTs(ds, colorTs);
                    if (di >= 0) depth = Copy(ds[di]);
                    (dw, dh) = PointCloudRecording.ReadRcsvHeaderDimensions(depthPath);
                }

                sb.AppendLine($"session={Path.GetFileName(sessionRoot)} serial={serial} frame={frameIndex} colorTs={colorTs}");
                sb.AppendLine($"color {cw}x{ch} depth {dw}x{dh} yoloxBgr={yoloxBgr} conf={conf}");

                // ---- backend ----
                var backend = new OrtRtmposeBackend(yoloxPath, rtmPath, useDirectML: true) { yoloxBgr = yoloxBgr, detScoreThreshold = 0.3f };
                var spec = backend.Spec;

                // ---- detect ----
                var boxes = new DetBox[8];
                int nDet = backend.Detect(color, cw, ch, boxes);
                sb.AppendLine($"YOLOX detections={nDet}");
                if (nDet <= 0) { backend.Dispose(); return sb.ToString(); }
                int best = 0; for (int i = 1; i < nDet; i++) if (boxes[i].Score > boxes[best].Score) best = i;
                var box = boxes[best];
                sb.AppendLine($"best box score={box.Score:F3} x1={box.X1:F0} y1={box.Y1:F0} x2={box.X2:F0} y2={box.Y2:F0}");

                // ---- pose ----
                int k = backend.NumKeypoints;
                var roi = RtmposePreprocess.RoiFromBox(box.X1, box.Y1, box.X2, box.Y2, spec);
                var input = RtmposePreprocess.BuildInput(color, cw, ch, roi, spec, null);
                var simccX = new float[k * Mathf.RoundToInt(spec.InW * spec.SplitRatio)];
                var simccY = new float[k * Mathf.RoundToInt(spec.InH * spec.SplitRatio)];
                bool posed = backend.Pose(input, simccX, simccY);
                sb.AppendLine($"RTMPose ran={posed}");
                if (!posed) { backend.Dispose(); return sb.ToString(); }

                var coco = new Vector2[k]; var cocoScore = new float[k];
                SimccDecoder.Decode(simccX, simccY, k, roi, spec, coco, cocoScore);

                // score distribution
                float smin = float.MaxValue, smax = float.MinValue, ssum = 0;
                for (int i = 0; i < k; i++) { smin = Mathf.Min(smin, cocoScore[i]); smax = Mathf.Max(smax, cocoScore[i]); ssum += cocoScore[i]; }
                sb.AppendLine($"COCO score min={smin:F3} max={smax:F3} mean={ssum / k:F3}");
                string[] cocoNames = { "nose", "Leye", "Reye", "Lear", "Rear", "Lsho", "Rsho", "Lelb", "Relb", "Lwri", "Rwri", "Lhip", "Rhip", "Lkne", "Rkne", "Lank", "Rank" };
                for (int i = 0; i < k; i++)
                    sb.AppendLine($"  {cocoNames[i],-5} uv=({coco[i].x:F0},{coco[i].y:F0}) s={cocoScore[i]:F3} inImg={(coco[i].x >= 0 && coco[i].x < cw && coco[i].y >= 0 && coco[i].y < ch)}");

                // ---- map to 15 + depth lift ----
                var xy = new Vector2[EvalSkeleton.JointCount]; var esc = new float[EvalSkeleton.JointCount]; var valid = new bool[EvalSkeleton.JointCount];
                CocoToEval.Map(coco, cocoScore, conf, xy, esc, valid);

                var lift = new DepthLift();
                bool have3d = cam.HasValue && depth != null && lift.BuildAligned(depth, dw, dh, cw, ch, cam.Value);
                sb.AppendLine($"have3d={have3d}");
                var pos3d = new Vector3[EvalSkeleton.JointCount]; var has3d = new bool[EvalSkeleton.JointCount];
                int validCount = 0, lifted = 0;
                for (int j = 0; j < EvalSkeleton.JointCount; j++)
                {
                    if (valid[j]) validCount++;
                    if (!valid[j] || !have3d) continue;
                    float d = lift.SampleMm(Mathf.RoundToInt(xy[j].x), Mathf.RoundToInt(xy[j].y), 3);
                    if (d <= 0f) continue;
                    pos3d[j] = DepthLift.Backproject(xy[j].x, xy[j].y, d, cw, ch, cam.Value);
                    has3d[j] = true; lifted++;
                }
                sb.AppendLine($"eval joints valid={validCount}/15 lifted3d={lifted}/15");
                for (int j = 0; j < EvalSkeleton.JointCount; j++)
                    sb.AppendLine($"  {((EvalJointId)j),-10} valid={valid[j]} s={esc[j]:F3} d={(has3d[j] ? pos3d[j].z.ToString("F0") : "-")}mm 3d={(has3d[j] ? $"({pos3d[j].x:F0},{pos3d[j].y:F0},{pos3d[j].z:F0})" : "-")}");

                // anatomical sanity (image v grows downward): head above hips above ankles
                if (valid[(int)EvalJointId.Head] && valid[(int)EvalJointId.HipL] && valid[(int)EvalJointId.AnkleL])
                    sb.AppendLine($"anatomy(headV<hipV<ankleV)={xy[(int)EvalJointId.Head].y < xy[(int)EvalJointId.HipL].y && xy[(int)EvalJointId.HipL].y < xy[(int)EvalJointId.AnkleL].y}");

                // ---- viz ----
                if (spawnViz)
                {
                    var parent = new GameObject($"RtmposeVerify_{serial}_{frameIndex}");
                    if (spawnCloud && cam.HasValue && depth != null)
                        SpawnCloud(parent.transform, depth, dw, dh, color, cw, ch, cam.Value);
                    SpawnJoints(parent.transform, "rtmpose", pos3d, has3d, EvalSkeletonMap.CameraMmToUnity, new Color(1f, 0.4f, 0.1f));
                    SpawnK4abt(parent.transform, bodiesPath, colorTs, cam);
                    sb.AppendLine($"spawned viz under '{parent.name}' (point cloud + rtmpose=orange + k4abt=cyan, all color-cam space)");
                }

                backend.Dispose();
            }
            catch (Exception e) { sb.AppendLine("EXCEPTION: " + e); }
            return sb.ToString();
        }

        static void SpawnJoints(Transform parent, string tag, Vector3[] posMm, bool[] has, Func<Vector3, Vector3> toUnity, Color col)
        {
            for (int j = 0; j < posMm.Length; j++)
            {
                if (!has[j]) continue;
                var s = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                s.name = $"{tag}_{(EvalJointId)j}";
                s.transform.SetParent(parent, false);
                s.transform.localPosition = toUnity(posMm[j]);
                s.transform.localScale = Vector3.one * 0.05f;
                var r = s.GetComponent<Renderer>(); if (r != null) r.material.color = col;
            }
        }

        // Point cloud reconstructed for this frame in color-camera space (meters),
        // rendered with the project's point shader. localScale flips Y to match the
        // joints' CameraMmToUnity convention.
        static void SpawnCloud(Transform parent, byte[] depth, int dw, int dh, byte[] color, int cw, int ch, ObCameraParam cam)
        {
            var recon = new PointCloudReconstructor("verify");
            if (!recon.Dispatch(depth, depth.Length, dw, dh, color, color.Length, cw, ch, cam)) { recon.Dispose(); return; }
            _keep.Add(recon);
            var go = new GameObject("PointCloud");
            go.transform.SetParent(parent, false);
            go.transform.localScale = new Vector3(1f, -1f, 1f);
            go.AddComponent<MeshFilter>().sharedMesh = recon.Mesh;
            var mr = go.AddComponent<MeshRenderer>();
            var sh = Shader.Find("Orbbec/PointCloudUnlit");
            if (sh != null) mr.sharedMaterial = new Material(sh);
        }

        // k4abt joints are in the DEPTH camera frame; transform to the COLOR frame
        // (via the D2C extrinsic) so they share the point cloud / RTMPose space.
        static void SpawnK4abt(Transform parent, string bodiesPath, ulong colorTs, ObCameraParam? cam)
        {
            if (!File.Exists(bodiesPath)) return;
            using var bs = new PointCloudRecording.RcsvFrameStream(bodiesPath);
            int bi = NearestByTs(bs, colorTs);
            if (bi < 0) return;
            var f = bs[bi];
            var buf = new BodySnapshot[6]; for (int i = 0; i < buf.Length; i++) buf[i] = new BodySnapshot();
            int n = RecordedBodySerializer.Decode(f.Bytes, f.ByteCount, buf);
            if (n <= 0) return;
            var body = buf[0];
            for (int ji = 0; ji < K4ABTConsts.K4ABT_JOINT_COUNT; ji++)
            {
                if (!BodyTrackingShared.IsDrawnJoint((k4abt_joint_id_t)ji)) continue;
                var jt = body.Joints[ji];
                if (jt.ConfidenceLevel <= k4abt_joint_confidence_level_t.K4ABT_JOINT_CONFIDENCE_NONE) continue;
                Vector3 posMm;
                if (cam.HasValue)
                {
                    var R = cam.Value.Transform.Rot; var T = cam.Value.Transform.Trans;
                    var p = jt.Position;
                    posMm = new Vector3(
                        R[0] * p.X + R[1] * p.Y + R[2] * p.Z + T[0],
                        R[3] * p.X + R[4] * p.Y + R[5] * p.Z + T[1],
                        R[6] * p.X + R[7] * p.Y + R[8] * p.Z + T[2]);
                }
                else posMm = new Vector3(jt.Position.X, jt.Position.Y, jt.Position.Z);
                var s = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                s.name = $"k4abt_{(k4abt_joint_id_t)ji}";
                s.transform.SetParent(parent, false);
                s.transform.localPosition = EvalSkeletonMap.CameraMmToUnity(posMm);
                s.transform.localScale = Vector3.one * 0.04f;
                var r = s.GetComponent<Renderer>(); if (r != null) r.material.color = Color.cyan;
            }
        }

        static string FirstOnnx(string dir)
        {
            if (!Directory.Exists(dir)) return null;
            var f = Directory.GetFiles(dir, "end2end.onnx", SearchOption.AllDirectories);
            return f.Length > 0 ? f[0] : null;
        }

        static byte[] Copy(PointCloudRecording.Frame f) { var b = new byte[f.ByteCount]; Buffer.BlockCopy(f.Bytes, 0, b, 0, f.ByteCount); return b; }

        static int NearestByTs(PointCloudRecording.RcsvFrameStream s, ulong ts)
        {
            int best = -1; ulong bd = ulong.MaxValue;
            for (int i = 0; i < s.Count; i++)
            {
                ulong t = s.TimestampNsAt(i); ulong d = t > ts ? t - ts : ts - t;
                if (d < bd) { bd = d; best = i; }
            }
            return best;
        }

        static ObCameraParam? LoadCameraParam(string root, string serial)
        {
            try
            {
                var calib = PointCloudRecording.ReadExtrinsicsYaml(root);
                if (calib == null) return null;
                foreach (var c in calib)
                {
                    if (!string.Equals(c.Serial, serial, StringComparison.OrdinalIgnoreCase)) continue;
                    return new ObCameraParam
                    {
                        DepthIntrinsic = c.DepthIntrinsic,
                        RgbIntrinsic = c.ColorIntrinsic,
                        DepthDistortion = c.DepthDistortion,
                        RgbDistortion = c.ColorDistortion,
                        Transform = c.DepthToColor,
                        IsMirrored = false,
                    };
                }
            }
            catch { }
            return null;
        }
    }
}
