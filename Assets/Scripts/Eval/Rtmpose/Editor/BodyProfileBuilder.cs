// Body-profile calibration, step 2: extract the dancer's bone lengths from
// user-approved "good frames" (frames where the 4 cameras' RTMPose skeletons
// agree AND the user visually confirmed the overlays — see GoodFrameScan /
// CandOverlay). For every (frame, camera) the adapter runs with a tracking
// warm-up (production mode), and each bone contributes one length sample when
// BOTH endpoint joints are valid in that SAME camera — same-camera pairs so
// the cross-camera surface-parallax offset cannot inflate a length. The
// per-bone median over the whole pool becomes the profile.
//
//   BodyProfileBuilder.Build(new[]{45,84,369,993,1056},
//                            new ulong[]{...}, 10, "")   // warmup, outPath("" = eval/body_profile.json)
//
// Output JSON is consumed by the (planned) fused-RTMPose adapter: length
// sanity gate, ray x bone-length depth re-lift, and the leg L/R swap test.

using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Text;
using PointCloud;
using UnityEngine;

namespace BodyTracking.Eval.Rtmpose
{
    public static class BodyProfileBuilder
    {
        public static string SessionRoot = "D:/Dropbox/projects/ICC/Recordings/RecordingBase/2026-07-14_15-50-24";

        static readonly (EvalJointId a, EvalJointId b)[] Bones =
        {
            (EvalJointId.Pelvis, EvalJointId.Neck), (EvalJointId.Neck, EvalJointId.Head),
            (EvalJointId.Neck, EvalJointId.ShoulderL), (EvalJointId.ShoulderL, EvalJointId.ElbowL), (EvalJointId.ElbowL, EvalJointId.WristL),
            (EvalJointId.Neck, EvalJointId.ShoulderR), (EvalJointId.ShoulderR, EvalJointId.ElbowR), (EvalJointId.ElbowR, EvalJointId.WristR),
            (EvalJointId.Pelvis, EvalJointId.HipL), (EvalJointId.HipL, EvalJointId.KneeL), (EvalJointId.KneeL, EvalJointId.AnkleL),
            (EvalJointId.Pelvis, EvalJointId.HipR), (EvalJointId.HipR, EvalJointId.KneeR), (EvalJointId.KneeR, EvalJointId.AnkleR),
        };

        public static string Build(int[] frames, ulong[] timestamps, int warmupFrames, string outPath)
        {
            if (frames == null || timestamps == null || frames.Length != timestamps.Length)
                return "frames/timestamps mismatch";
            if (string.IsNullOrEmpty(outPath))
                outPath = Path.GetFullPath(Path.Combine(Application.dataPath, "..", "eval", "body_profile.json"));

            var backend = BtFrameInspectorWindow.SharedBackend();
            var calib = PointCloudRecording.ReadExtrinsicsYaml(SessionRoot);
            if (calib == null) return "no extrinsics.yaml";
            var adapter = new RtmPoseAdapter(backend) { confThreshold = 0.3f };
            foreach (var dc0 in calib)
                if (dc0.GlobalTrColorCamera.HasValue) adapter.SetWorldTransform(dc0.Serial, dc0.GlobalTrColorCamera.Value);
            adapter.SetCaptureVolume(new Vector3(0, 200, 3000), new Vector3(1000, 1400, 900));
            EvalSkeleton lastSkel = null;
            adapter.OnSkeletons += f => lastSkel = f.Primary();

            // samples[boneIndex] = list of lengths (mm), one per (frame, camera)
            var samples = new List<float>[Bones.Length];
            for (int i = 0; i < samples.Length; i++) samples[i] = new List<float>();

            foreach (var (serial, dir) in PointCloudRecording.EnumerateDevices(SessionRoot))
            {
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
                for (int fi = 0; fi < frames.Length; fi++)
                {
                    int tgt = Nearest(cs, timestamps[fi]);
                    if (tgt < 0) continue;
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
                    }
                    var skel = lastSkel;
                    if (skel == null) continue;
                    for (int b = 0; b < Bones.Length; b++)
                    {
                        int ia = (int)Bones[b].a, ib = (int)Bones[b].b;
                        if (!skel.Joints[ia].Valid || !skel.Joints[ib].Valid) continue;
                        // camera-space mm; same camera for both endpoints, so no
                        // cross-camera surface parallax enters the length.
                        samples[b].Add(Vector3.Distance(skel.Joints[ia].PositionMm, skel.Joints[ib].PositionMm));
                    }
                }
                GC.Collect();
            }
            // adapter deliberately not disposed (shared backend).

            var sb = new StringBuilder();
            var json = new StringBuilder();
            json.AppendLine("{");
            json.AppendLine($"  \"session\": \"{SessionRoot}\",");
            json.AppendLine($"  \"frames\": [{string.Join(",", frames)}],");
            json.AppendLine("  \"lengthsMm\": {");
            sb.AppendLine("bone                     medianMm  n   IQRmm");
            for (int b = 0; b < Bones.Length; b++)
            {
                string name = $"{Bones[b].a}-{Bones[b].b}";
                var list = samples[b];
                float med = 0f, iqr = 0f;
                if (list.Count > 0)
                {
                    list.Sort();
                    med = list[list.Count / 2];
                    iqr = list[(int)(list.Count * 0.75f)] - list[(int)(list.Count * 0.25f)];
                }
                sb.AppendLine($"{name,-24} {med,7:F0} {list.Count,3} {iqr,6:F0}");
                json.Append($"    \"{name}\": {{ \"medianMm\": {med.ToString("F1", CultureInfo.InvariantCulture)}, \"samples\": {list.Count}, \"iqrMm\": {iqr.ToString("F1", CultureInfo.InvariantCulture)} }}");
                json.AppendLine(b < Bones.Length - 1 ? "," : "");
            }
            json.AppendLine("  }");
            json.AppendLine("}");
            File.WriteAllText(outPath, json.ToString());
            sb.AppendLine("written: " + outPath);
            return sb.ToString();
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
