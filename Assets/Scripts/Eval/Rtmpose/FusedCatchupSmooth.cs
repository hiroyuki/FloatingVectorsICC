// Offline post-pass over exported fused bodies_main: redistributes tracker
// CATCH-UP jumps so motion curves stop kinking in the motion-blur band.
//
// Signature (measured at 54.8-55.3s of the 15-50-24 session): during a fast
// hand swing the blurred image makes the 2D pose STICK for a few frames
// (steps drop to ~40-60mm while the real hand moves ~100-150mm/frame), then
// the tracker catches up with a single ~400mm jump and continues smoothly.
// The curve renderer (Catmull-Rom through control points) draws that as a
// straight segment ending in a sharp corner.
//
// The hand genuinely moved through the stuck span, so redistributing the
// jump across it — anchor before the stuck run, landing at the jump target,
// time-proportional interpolation — reconstructs plausible smooth motion.
// Out-and-back spikes are NOT redistributed (the jump must CONTINUE in the
// same direction afterwards to qualify).
//
// Applies the same per-joint pass to every device track; positions are the
// same skeleton in different rigid frames, so distances (and therefore
// detection) agree across tracks and the tracks stay consistent.
//
// Usage (editor):
//   FusedCatchupSmooth.Run(@"D:/FVICC_eval/tmp-v11-bodies")  -> stats string
// Writes bodies_main in place (original saved as bodies_main.prekink once).

using System;
using System.Collections.Generic;
using System.IO;
using PointCloud;
using UnityEngine;

namespace BodyTracking.Eval.Rtmpose
{
    public static class FusedCatchupSmooth
    {
        /// <summary>A frame-to-frame step this large (mm) is a candidate catch-up jump.</summary>
        public static float JumpMm = 250f;
        /// <summary>Steps below this fraction of the jump count as "stuck".</summary>
        public static float StuckFrac = 0.45f;
        /// <summary>Max stuck frames redistributed before a jump.</summary>
        public static int MaxStuck = 6;

        public static string Run(string sessionRoot)
        {
            var sb = new System.Text.StringBuilder();
            foreach (var (serial, deviceDir) in PointCloudRecording.EnumerateDevices(sessionRoot))
            {
                string path = Path.Combine(deviceDir, PointCloudRecording.BodiesSensorName);
                if (!File.Exists(path)) continue;
                string backup = path + ".prekink";
                if (!File.Exists(backup)) File.Copy(path, backup);

                // Load every frame (timestamps + snapshots).
                var timestamps = new List<ulong>();
                var frames = new List<BodySnapshot>();
                var counts = new List<int>();
                var decode = new BodySnapshot[8];
                for (int i = 0; i < decode.Length; i++) decode[i] = new BodySnapshot();
                using (var s = new PointCloudRecording.RcsvFrameStream(backup))
                {
                    for (int i = 0; i < s.Count; i++)
                    {
                        var fr = s[i];
                        int n = RecordedBodySerializer.Decode(fr.Bytes, fr.ByteCount, decode);
                        timestamps.Add(fr.TimestampNs);
                        counts.Add(n);
                        var copy = new BodySnapshot();
                        if (n >= 1) copy.CopyFrom(decode[0]);
                        frames.Add(copy);
                    }
                }

                int fixedJumps = 0;
                for (int j = 0; j < K4ABTConsts.K4ABT_JOINT_COUNT; j++)
                    fixedJumps += RedistributeJoint(timestamps, frames, counts, j);

                // Write back (single body per frame, matching the export layout).
                var scratch = new byte[RecordedBodySerializer.FrameSize(1)];
                var one = new BodySnapshot[1];
                using (var wtr = new PointCloudRecording.RcsvStreamWriter(
                           path, PointCloudRecording.BuildBodiesHeaderYaml(serial)))
                {
                    for (int i = 0; i < frames.Count; i++)
                    {
                        if (counts[i] < 1) continue;
                        one[0] = frames[i];
                        int bytes = RecordedBodySerializer.Encode(one, 1, scratch);
                        wtr.WriteFrame(timestamps[i], scratch, bytes);
                    }
                }
                sb.AppendLine($"{serial}: {fixedJumps} catch-up jumps redistributed -> {path}");
            }
            return sb.Length > 0 ? sb.ToString() : "no devices found";
        }

        private static int RedistributeJoint(List<ulong> ts, List<BodySnapshot> frames, List<int> counts, int joint)
        {
            int n = frames.Count, fixedJumps = 0;
            Func<int, bool> valid = i => counts[i] >= 1
                && frames[i].Joints[joint].ConfidenceLevel != k4abt_joint_confidence_level_t.K4ABT_JOINT_CONFIDENCE_NONE;
            Func<int, Vector3> pos = i =>
            {
                var p = frames[i].Joints[joint].Position;
                return new Vector3(p.X, p.Y, p.Z);
            };
            Action<int, Vector3> set = (i, p) =>
            {
                var jt = frames[i].Joints[joint];
                jt.Position = new k4a_float3_t { X = p.x, Y = p.y, Z = p.z };
                frames[i].Joints[joint] = jt;
            };

            for (int t = 2; t < n - 1; t++)
            {
                if (!valid(t) || !valid(t - 1)) continue;
                var jump = pos(t) - pos(t - 1);
                float step = jump.magnitude;
                if (step < JumpMm) continue;

                // must CONTINUE after the jump (catch-up), not reverse (spike)
                if (!valid(t + 1)) continue;
                var next = pos(t + 1) - pos(t);
                if (next.magnitude > 1f && Vector3.Dot(next.normalized, jump.normalized) < -0.2f) continue;

                // walk back over the stuck run (small steps)
                float stuckLimit = StuckFrac * step;
                int k = 0;
                while (k < MaxStuck && t - 2 - k >= 0
                       && valid(t - 1 - k) && valid(t - 2 - k)
                       && (pos(t - 1 - k) - pos(t - 2 - k)).magnitude < stuckLimit)
                    k++;
                if (k == 0) continue;

                // redistribute: anchor a = t-1-k, landing = t; stuck frames
                // [t-k, t-1] re-timed onto the straight anchor->landing path.
                int a = t - 1 - k;
                var pa = pos(a); var pt = pos(t);
                double ta = ts[a], tt = ts[t];
                if (tt <= ta) continue;
                for (int i = t - k; i <= t - 1; i++)
                {
                    float f = (float)((ts[i] - ta) / (tt - ta));
                    set(i, Vector3.LerpUnclamped(pa, pt, f));
                }
                fixedJumps++;
                t += 1; // don't re-detect inside the same event
            }
            return fixedJumps;
        }
    }
}
