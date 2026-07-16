// Masks the depth pixels where a RIVAL camera's IR projector appears.
//
// Facing ToF pairs saturate each other's sensor around the rival projector's
// image position, and the flare pixels return FALSE depth — measured on the
// 15-50-24 rig: ~3.0 m where the true geometry is 6.5 m — which floats
// phantom points mid-air on the pair's sightline (visible as flickering
// speckle at stage center; both rig pairs N↔EG and L↔Z face each other with
// the projector landing near image center).
//
// The disc is masked ONLY while the projector is actually visible in the IR
// frame (brightness gate): a facing projector reads ~29000 (saturated) on the
// 5x5 mean, an occluded or idle one ~50, so when the performer's body covers
// the projector the flare is physically gone, the body pixels are genuine,
// and the mask lifts automatically — no body pixels are ever lost.
//
// Configure once from the rig calibration (every camera's depth intrinsics +
// D2C + color→world extrinsic); Apply per frame on the raw Y16 depth buffer
// BEFORE any consumer (mesh reconstruction, TSDF integration, BT, event
// subscribers) sees it.

using System.Collections.Generic;
using Orbbec;
using UnityEngine;

namespace PointCloud
{
    public static class ProjectorMask
    {
        public struct CameraPose
        {
            public string Serial;
            /// <summary>Depth intrinsics + depth→color extrinsic of this camera.</summary>
            public ObCameraParam Param;
            /// <summary>Color camera → world (rig calibration).</summary>
            public ObExtrinsic World;
        }

        private sealed class Disc
        {
            public float U, V;      // blob center at the depth-intrinsic calibration resolution
            public int CalW, CalH;  // that calibration resolution (scaled to the stream at Apply time)
        }

        private static readonly Dictionary<string, List<Disc>> _discs = new Dictionary<string, List<Disc>>();

        /// <summary>Mask radius in pixels at a 320-wide depth image; scales with stream width.
        /// The measured flare strip sits up to ~37 px from the blob center.</summary>
        public static float MaskRadius320 = 45f;

        /// <summary>IR gate: the 5x5 mean (Y16) at the blob center must exceed this for the
        /// mask to engage. Visible projector ≈ 29000, occluded ≈ 50 (measured) — the gate
        /// is insensitive to the exact value.</summary>
        public static int IrGateThreshold = 2000;

        public static void Clear() => _discs.Clear();

        public static int DiscCount(string serial)
            => _discs.TryGetValue(serial, out var l) ? l.Count : 0;

        /// <summary>Register the rig: for every camera, project every OTHER camera's
        /// world position into its depth image; in-view rivals become mask discs.</summary>
        public static void Configure(IReadOnlyList<CameraPose> rig)
        {
            _discs.Clear();
            if (rig == null) return;
            foreach (var cam in rig)
            {
                var intr = cam.Param.DepthIntrinsic;
                if (intr.Fx <= 0f || intr.Width <= 0 || intr.Height <= 0) continue;
                if (cam.World.Rot == null || cam.World.Rot.Length < 9) continue;
                if (cam.Param.Transform.Rot == null || cam.Param.Transform.Rot.Length < 9) continue;

                var list = new List<Disc>();
                foreach (var other in rig)
                {
                    if (other.Serial == cam.Serial) continue;
                    if (other.World.Trans == null || other.World.Trans.Length < 3) continue;
                    var world = new Vector3(other.World.Trans[0], other.World.Trans[1], other.World.Trans[2]);
                    var pc = InverseTransform(world, cam.World);          // world -> cam color frame
                    var pd = InverseTransform(pc, cam.Param.Transform);   // color -> depth frame
                    if (pd.z <= 100f) continue;                           // behind or degenerate
                    float u = intr.Fx * pd.x / pd.z + intr.Cx;
                    float v = intr.Fy * pd.y / pd.z + intr.Cy;
                    // keep slightly-out-of-frame blobs: the flare bleeds inward
                    if (u < -40f || u > intr.Width + 40f || v < -40f || v > intr.Height + 40f) continue;
                    list.Add(new Disc { U = u, V = v, CalW = intr.Width, CalH = intr.Height });
                }
                if (list.Count > 0) _discs[cam.Serial] = list;
            }
        }

        /// <summary>Zero the depth inside each registered disc of <paramref name="serial"/>,
        /// gated per frame on the IR brightness at the disc center. A missing IR buffer
        /// masks unconditionally — the disc only ever contains the rival camera and the
        /// wall behind it, both outside the capture volume.</summary>
        public static void Apply(string serial, byte[] depth, int depthByteCount, int w, int h,
                                 byte[] ir, int irByteCount, int irW, int irH)
        {
            if (depth == null || w <= 0 || h <= 0 || depthByteCount < w * h * 2) return;
            if (!_discs.TryGetValue(serial, out var list)) return;
            foreach (var d in list)
            {
                float su = d.CalW > 0 ? (float)w / d.CalW : 1f;
                float sv = d.CalH > 0 ? (float)h / d.CalH : 1f;
                float cu = d.U * su, cv = d.V * sv;
                if (!ProjectorVisible(ir, irByteCount, irW, irH,
                        cu * (irW > 0 ? (float)irW / w : 1f), cv * (irH > 0 ? (float)irH / h : 1f)))
                    continue;
                float r = MaskRadius320 * (w / 320f);
                int u0 = Mathf.Max(0, (int)(cu - r)), u1 = Mathf.Min(w - 1, (int)(cu + r));
                int v0 = Mathf.Max(0, (int)(cv - r)), v1 = Mathf.Min(h - 1, (int)(cv + r));
                float r2 = r * r;
                for (int v = v0; v <= v1; v++)
                {
                    float dv = v - cv;
                    for (int u = u0; u <= u1; u++)
                    {
                        float du = u - cu;
                        if (du * du + dv * dv > r2) continue;
                        int i = (v * w + u) * 2;
                        depth[i] = 0; depth[i + 1] = 0;
                    }
                }
            }
        }

        private static bool ProjectorVisible(byte[] ir, int irByteCount, int irW, int irH, float cu, float cv)
        {
            if (ir == null || irW <= 0 || irH <= 0 || irByteCount < irW * irH * 2) return true;
            int uc = Mathf.Clamp((int)cu, 2, irW - 3), vc = Mathf.Clamp((int)cv, 2, irH - 3);
            long sum = 0;
            for (int dv = -2; dv <= 2; dv++)
                for (int du = -2; du <= 2; du++)
                {
                    int i = ((vc + dv) * irW + (uc + du)) * 2;
                    sum += ir[i] | (ir[i + 1] << 8);
                }
            return sum / 25 >= IrGateThreshold;
        }

        private static Vector3 InverseTransform(Vector3 p, ObExtrinsic e)
        {
            var R = e.Rot; var T = e.Trans;
            float x = p.x - T[0], y = p.y - T[1], z = p.z - T[2];
            return new Vector3(
                R[0] * x + R[3] * y + R[6] * z,
                R[1] * x + R[4] * y + R[7] * z,
                R[2] * x + R[5] * y + R[8] * z);
        }
    }
}
