// Renders a per-camera color-image overlay of the RTMPose skeleton (orange,
// 3D joints reprojected into the same camera) for one reference timestamp —
// used to visually verify body-profile calibration candidate frames from
// GoodFrameScan. Runs the adapter in detect-once-then-track mode with a
// configurable warm-up run of preceding frames so the tracking state matches
// the scan / production behavior (a cold single-frame detect can fail person
// selection where a warm track succeeds).
//
//   CandOverlay.Render(1056, 1784011865456744000UL, 10, "C:/out/dir")
//
// Reuses BtFrameInspectorWindow's cached ONNX backend (never disposes it:
// RtmPoseAdapter.Dispose() disposes the backend it was handed — that
// hard-crashed the editor twice when a later batch reused the shared cache).

using System;
using System.IO;
using System.Text;
using PointCloud;
using UnityEngine;

namespace BodyTracking.Eval.Rtmpose
{
    public static class CandOverlay
    {
        public static string SessionRoot = "D:/Dropbox/projects/ICC/Recordings/RecordingBase/2026-07-14_15-50-24";

        static readonly int[,] EvalBones = new int[,]
        {
            {0,1},{1,2},{1,3},{3,4},{4,5},{1,6},{6,7},{7,8},{0,9},{9,10},{10,11},{0,12},{12,13},{13,14}
        };

        public static string Render(int frameNo, ulong targetTs, int warmupFrames, string outDir)
        {
            var sb = new StringBuilder();
            var backend = BtFrameInspectorWindow.SharedBackend();
            var calib = PointCloudRecording.ReadExtrinsicsYaml(SessionRoot);
            if (calib == null) return "no extrinsics.yaml";
            var adapter = new RtmPoseAdapter(backend) { confThreshold = 0.3f };
            foreach (var dc0 in calib)
                if (dc0.GlobalTrColorCamera.HasValue) adapter.SetWorldTransform(dc0.Serial, dc0.GlobalTrColorCamera.Value);
            // Derive the person-selection volume from THIS take's rig. The old
            // constant (0,200,3000)+-(1000,1400,900) meant "3 m in front of the
            // origin camera" and only held while extrinsics put a camera at the
            // origin; on a room-centred solve it points outside the room and every
            // frame reports NO SKEL. See RigCaptureVolume.
            if (RigCaptureVolume.TryDerive(calib, out var volC, out var volH))
                adapter.SetCaptureVolume(volC, volH);
            else
                adapter.SetCaptureVolume(new Vector3(0, 200, 3000), new Vector3(1000, 1400, 900));
            EvalSkeleton lastSkel = null;
            adapter.OnSkeletons += f => lastSkel = f.Primary();

            foreach (var (serial, dir) in PointCloudRecording.EnumerateDevices(SessionRoot))
            {
                string suffix = serial.Substring(Mathf.Max(0, serial.Length - 2));
                PointCloudRecording.DeviceCalibration dc = null;
                foreach (var c in calib) if (c.Serial == serial) dc = c;
                if (dc == null) continue;
                string colorPath = Path.Combine(dir, PointCloudRecording.ColorSensorName);
                string depthPath = Path.Combine(dir, PointCloudRecording.DepthSensorName);
                var (cw, ch) = PointCloudRecording.ReadRcsvHeaderDimensions(colorPath);
                var (dw, dh) = PointCloudRecording.ReadRcsvHeaderDimensions(depthPath);
                var cam = new Orbbec.ObCameraParam
                {
                    DepthIntrinsic = dc.DepthIntrinsic, RgbIntrinsic = dc.ColorIntrinsic,
                    DepthDistortion = dc.DepthDistortion, RgbDistortion = dc.ColorDistortion,
                    Transform = dc.DepthToColor, IsMirrored = false,
                };
                var ctx = new EvalCameraContext(serial, dw, dh, cw, ch, cam);
                adapter.Configure(ctx);

                using var cs = new PointCloudRecording.RcsvFrameStream(colorPath);
                using var ds = new PointCloudRecording.RcsvFrameStream(depthPath);
                int tgt = Nearest(cs, targetTs);
                if (tgt < 0) { sb.AppendLine($"f{frameNo} {suffix} -> no color frame"); continue; }

                byte[] lastColor = null;
                for (int k = warmupFrames; k >= 0; k--)
                {
                    int ci = tgt - k * 3;
                    if (ci < 0 || ci >= cs.Count) continue;
                    var fr = cs[ci];
                    var color = Copy(fr);
                    ulong ts = fr.TimestampNs;
                    var dfr = ds[Nearest(ds, ts)];
                    var depth = Copy(dfr);
                    lastSkel = null;
                    var raw = new RawFrameData(depth, depth.Length, dw, dh, color, color.Length, cw, ch,
                                               Array.Empty<byte>(), 0, 0, 0, ts / 1000UL);
                    adapter.SubmitFrame(serial, raw, ts);
                    lastColor = color;
                }

                var skel = lastSkel;
                DrawAndSave(lastColor, cw, ch, skel, dc, Path.Combine(outDir, $"cand{frameNo}_{suffix}.png"));
                sb.AppendLine($"f{frameNo} {suffix} -> " + (skel != null ? "ok" : "NO SKEL"));
                GC.Collect();
            }
            // adapter deliberately not disposed (shared backend, see header).
            return sb.ToString();
        }

        static void DrawAndSave(byte[] img, int cw, int ch, EvalSkeleton skel,
                                PointCloudRecording.DeviceCalibration dc, string outPath)
        {
            void Put(int x, int y, byte r, byte g, byte b)
            {
                if (x < 0 || y < 0 || x >= cw || y >= ch) return;
                int idx = (y * cw + x) * 3; img[idx] = r; img[idx + 1] = g; img[idx + 2] = b;
            }
            void Dot(int x, int y, int rad, byte r, byte g, byte b)
            {
                for (int yy = -rad; yy <= rad; yy++)
                    for (int xx = -rad; xx <= rad; xx++)
                        if (xx * xx + yy * yy <= rad * rad) Put(x + xx, y + yy, r, g, b);
            }
            void Line(float x0, float y0, float x1, float y1, int th, byte r, byte g, byte b)
            {
                float dx = x1 - x0, dy = y1 - y0;
                int n = Mathf.Max(1, (int)Mathf.Sqrt(dx * dx + dy * dy));
                for (int i = 0; i <= n; i++) { float t = (float)i / n; Dot((int)(x0 + dx * t), (int)(y0 + dy * t), th, r, g, b); }
            }

            if (skel != null)
            {
                float fx = dc.ColorIntrinsic.Fx, fy = dc.ColorIntrinsic.Fy;
                float cx = dc.ColorIntrinsic.Cx, cy = dc.ColorIntrinsic.Cy;
                var px = new Vector2[EvalSkeleton.JointCount];
                var ok = new bool[EvalSkeleton.JointCount];
                for (int j = 0; j < EvalSkeleton.JointCount; j++)
                {
                    if (!skel.Joints[j].Valid) continue;
                    var p = skel.Joints[j].PositionMm;
                    if (p.z <= 1f) continue;
                    px[j] = new Vector2(fx * p.x / p.z + cx, fy * p.y / p.z + cy);
                    ok[j] = true;
                }
                for (int b = 0; b < EvalBones.GetLength(0); b++)
                {
                    int ia = EvalBones[b, 0], ib = EvalBones[b, 1];
                    if (ok[ia] && ok[ib]) Line(px[ia].x, px[ia].y, px[ib].x, px[ib].y, 2, 255, 120, 10);
                }
                for (int j = 0; j < EvalSkeleton.JointCount; j++)
                    if (ok[j]) Dot((int)px[j].x, (int)px[j].y, 5, 255, 120, 10);
            }

            var tex = new Texture2D(cw, ch, TextureFormat.RGB24, false);
            var flipped = new byte[img.Length];
            int stride = cw * 3;
            for (int y = 0; y < ch; y++) Array.Copy(img, y * stride, flipped, (ch - 1 - y) * stride, stride);
            tex.LoadRawTextureData(flipped);
            tex.Apply();
            Directory.CreateDirectory(Path.GetDirectoryName(outPath));
            File.WriteAllBytes(outPath, tex.EncodeToPNG());
            UnityEngine.Object.DestroyImmediate(tex);
        }

        static byte[] Copy(PointCloudRecording.Frame f)
        {
            var b = new byte[f.ByteCount];
            Array.Copy(f.Bytes, b, f.ByteCount);
            return b;
        }

        static int Nearest(PointCloudRecording.RcsvFrameStream s, ulong ts)
        {
            int best = -1; ulong bd = ulong.MaxValue;
            for (int i = 0; i < s.Count; i++)
            {
                ulong t = s.TimestampNsAt(i);
                ulong d = t > ts ? t - ts : ts - t;
                if (d < bd) { bd = d; best = i; }
            }
            return best;
        }
    }
}
