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

        /// <summary>Two depth pixels belong to the same surface when they differ by at
        /// most this (mm) — the component-growing link tolerance.</summary>
        public static int DepthLinkMm = 300;

        /// <summary>A connected component inside the disc counts as REAL geometry when it
        /// reaches this many pixels (at 320-wide; scales quadratically) — the measured
        /// flare patch is &lt; ~200 px, a body or wall region is thousands.</summary>
        public static int MinRealComponentPx320 = 500;

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

        /// <summary>Clean the flare artifacts inside each registered disc of
        /// <paramref name="serial"/>, gated per frame on the IR brightness at the disc
        /// center. Inside the disc, only ISOLATED small depth components are zeroed:
        /// the flare patch is a free-floating island of self-consistent false depth
        /// (&lt; ~200 px, enclosed by saturated/invalid pixels), while real geometry —
        /// a body crossing the disc, the wall behind the rival camera — always connects
        /// outward into a large surface. Blanket-zeroing the disc is NOT safe: the body
        /// regularly overlaps the disc while the projector still peeks past its edge.</summary>
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
                FilterDisc(depth, w, h, cu, cv, MaskRadius320 * (w / 320f));
            }
        }

        // Scratch for the component sweep (grown on demand, box-local indices).
        // _queue doubles as the component pixel list: indices [0, qt) after a
        // traversal are exactly the pixels of that component.
        private static byte[] _visited = System.Array.Empty<byte>();
        private static int[] _queue = System.Array.Empty<int>();

        /// <summary>Flood-fill every valid pixel inside the disc into its depth-linked
        /// component (4-connectivity, |Δdepth| ≤ DepthLinkMm per edge) within a working
        /// box (disc + margin). A component is REAL — and kept — as soon as it touches
        /// the box boundary (it connects to the world outside), reaches
        /// MinRealComponentPx, or touches a pixel already proven real; anything that
        /// exhausts while still small and enclosed is the flare island and gets zeroed.
        ///
        /// Perf contract: this runs per camera per frame on the CPU (Mono in-editor),
        /// so the big surface (body/wall) must never be walked exhaustively. The
        /// early exits cap it at ~MinRealComponentPx dequeues once; every later seed
        /// of the same surface resolves in O(1-ish) by touching a REAL-marked pixel —
        /// the naive exhaustive version cost ~45 ms/frame across four cameras.
        /// Visited states: 0 = unvisited, 1 = REAL (kept), 2 = consumed (invalid or
        /// zeroed ghost).</summary>
        private static void FilterDisc(byte[] depth, int w, int h, float cu, float cv, float r)
        {
            int margin = 24; // ghost patch reaches ~40px from center; r+24 keeps it enclosed
            int bx0 = Mathf.Max(0, (int)(cu - r) - margin);
            int bx1 = Mathf.Min(w - 1, (int)(cu + r) + margin);
            int by0 = Mathf.Max(0, (int)(cv - r) - margin);
            int by1 = Mathf.Min(h - 1, (int)(cv + r) + margin);
            int bw = bx1 - bx0 + 1, bh = by1 - by0 + 1;
            int boxArea = bw * bh;
            if (boxArea <= 0) return;
            if (_visited.Length < boxArea)
            {
                _visited = new byte[boxArea];
                _queue = new int[boxArea];
            }
            System.Array.Clear(_visited, 0, boxArea);

            float r2 = r * r;
            float scale = w / 320f;
            int minReal = Mathf.Max(1, (int)(MinRealComponentPx320 * scale * scale));
            int link = DepthLinkMm;

            int u0 = Mathf.Max(bx0, (int)(cu - r)), u1 = Mathf.Min(bx1, (int)(cu + r));
            int v0 = Mathf.Max(by0, (int)(cv - r)), v1 = Mathf.Min(by1, (int)(cv + r));
            for (int sv2 = v0; sv2 <= v1; sv2++)
            {
                float dvv = sv2 - cv;
                for (int su2 = u0; su2 <= u1; su2++)
                {
                    float duu = su2 - cu;
                    if (duu * duu + dvv * dvv > r2) continue;
                    int seedLocal = (sv2 - by0) * bw + (su2 - bx0);
                    if (_visited[seedLocal] != 0) continue;
                    int seedIdx = (sv2 * w + su2) * 2;
                    int seedDepth = depth[seedIdx] | (depth[seedIdx + 1] << 8);
                    if (seedDepth == 0) { _visited[seedLocal] = 2; continue; }

                    int qh = 0, qt = 0;
                    bool real = false;
                    _visited[seedLocal] = 2;
                    _queue[qt++] = seedLocal;
                    while (qh < qt)
                    {
                        int cur = _queue[qh++];
                        int cy = cur / bw + by0, cx = cur % bw + bx0;
                        if (cx == bx0 || cx == bx1 || cy == by0 || cy == by1) { real = true; break; }
                        if (qt >= minReal) { real = true; break; }
                        int curDepth = depth[(cy * w + cx) * 2] | (depth[(cy * w + cx) * 2 + 1] << 8);
                        for (int n = 0; n < 4 && !real; n++)
                        {
                            int nx = cx + (n == 0 ? 1 : n == 1 ? -1 : 0);
                            int ny = cy + (n == 2 ? 1 : n == 3 ? -1 : 0);
                            if (nx < bx0 || nx > bx1 || ny < by0 || ny > by1) continue;
                            int nl = (ny - by0) * bw + (nx - bx0);
                            byte vs = _visited[nl];
                            int nd = depth[(ny * w + nx) * 2] | (depth[(ny * w + nx) * 2 + 1] << 8);
                            if (nd == 0) { if (vs == 0) _visited[nl] = 2; continue; }
                            // same-surface link required for BOTH growth and the
                            // touch-real shortcut (a 3 m flare pixel bordering a 6 m
                            // wall pixel is NOT connected to it)
                            if (System.Math.Abs(nd - curDepth) > link) continue;
                            if (vs == 1) { real = true; break; }  // touches a proven-real surface
                            if (vs != 0) continue;                 // already part of this traversal
                            _visited[nl] = 2;
                            _queue[qt++] = nl;
                        }
                        if (real) break;
                    }
                    if (real)
                    {
                        // promote everything this traversal touched to REAL so later
                        // seeds of the same surface resolve on contact
                        for (int i = 0; i < qt; i++) _visited[_queue[i]] = 1;
                        continue;
                    }
                    // enclosed small island -> flare: zero it
                    for (int i = 0; i < qt; i++)
                    {
                        int l = _queue[i];
                        int y = l / bw + by0, x = l % bw + bx0;
                        int idx = (y * w + x) * 2;
                        depth[idx] = 0; depth[idx + 1] = 0;
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
