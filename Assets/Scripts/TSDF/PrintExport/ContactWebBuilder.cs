// Local-patch smin fillet webs ("方法2"): for every GRAZING line pair, a small
// high-resolution Marching Cubes patch of the two lines' smooth-min SDF —
// a membrane that hugs both bars ("接合部のみMCでつくる"). The crisp direct
// tube mesh is never touched; patches overlap into the bars and the slicer
// unions them per layer.
//
// MC lookup tables are PARSED from Resources/MarchingCubesTables.hlsl at first
// use (same tables the GPU path uses) — no hand-copied table to get wrong.
//
// Triangle winding is set per face from the SDF gradient (outward), then the
// caller's OrientOutward pass may flip the whole block — patches flip with the
// tubes, so relative orientation stays consistent.

using System.Collections.Generic;
using System.IO;
using UnityEngine;

namespace TSDF
{
    internal static class ContactWebBuilder
    {
        private static int[][] _triTable; // 256 x (<=16, -1 terminated)

        // Cube corner offsets — must match MarchingCubesTables.hlsl header.
        private static readonly Vector3Int[] kCorner =
        {
            new Vector3Int(0, 0, 0), new Vector3Int(1, 0, 0),
            new Vector3Int(1, 1, 0), new Vector3Int(0, 1, 0),
            new Vector3Int(0, 0, 1), new Vector3Int(1, 0, 1),
            new Vector3Int(1, 1, 1), new Vector3Int(0, 1, 1),
        };
        private static readonly int[] kEdgeA = { 0, 1, 2, 3, 4, 5, 6, 7, 0, 1, 2, 3 };
        private static readonly int[] kEdgeB = { 1, 2, 3, 0, 5, 6, 7, 4, 4, 5, 6, 7 };

        private static bool LoadTables()
        {
            if (_triTable != null) return true;
            string path = Path.Combine(Application.dataPath,
                "Scripts/TSDF/Resources/MarchingCubesTables.hlsl");
            if (!File.Exists(path))
            {
                Debug.LogError("[ContactWebBuilder] MarchingCubesTables.hlsl not found: " + path);
                return false;
            }
            string text = File.ReadAllText(path);
            // find the DECLARATION ("triTable[256]"), not the header comment's
            // mention of the name (which sits before edgeTable's hex values)
            int at = text.IndexOf("triTable[256]", System.StringComparison.Ordinal);
            if (at < 0) { Debug.LogError("[ContactWebBuilder] triTable[256] not found in hlsl."); return false; }
            var nums = new List<int>(4096);
            bool neg = false; int cur = 0; bool inNum = false;
            for (int i = text.IndexOf('{', at) + 1; i < text.Length; i++)
            {
                char ch = text[i];
                if (ch == ';') break;
                if (ch == '-') { neg = true; continue; }
                if (ch >= '0' && ch <= '9') { cur = cur * 10 + (ch - '0'); inNum = true; continue; }
                if (inNum) { nums.Add(neg ? -cur : cur); cur = 0; neg = false; inNum = false; }
            }
            if (nums.Count != 256 * 16)
            {
                Debug.LogError($"[ContactWebBuilder] triTable parse got {nums.Count} ints, want 4096.");
                return false;
            }
            _triTable = new int[256][];
            for (int c = 0; c < 256; c++)
            {
                _triTable[c] = new int[16];
                for (int k = 0; k < 16; k++) _triTable[c][k] = nums[c * 16 + k];
            }
            return true;
        }

        private struct Seg { public Vector3 A, B; public float Ra, Rb; }

        private static float SegDist(List<Seg> segs, Vector3 x)
        {
            float best = float.MaxValue;
            for (int i = 0; i < segs.Count; i++)
            {
                var s = segs[i];
                Vector3 ab = s.B - s.A;
                float len2 = ab.sqrMagnitude;
                float t = len2 > 1e-12f ? Mathf.Clamp01(Vector3.Dot(x - s.A, ab) / len2) : 0f;
                Vector3 cp = s.A + ab * t;
                float d = (x - cp).magnitude - Mathf.Lerp(s.Ra, s.Rb, t);
                if (d < best) best = d;
            }
            return best;
        }

        private static float Smin(float a, float b, float k)
        {
            float h = Mathf.Clamp01(0.5f + 0.5f * (b - a) / k);
            return Mathf.Lerp(b, a, h) - k * h * (1f - h);
        }

        /// <summary>Fillet webs for grazing pairs. pts/ranges = the chain point
        /// cloud with local radii (linkPts data). Returns the number of webs
        /// appended. Pairs whose closest approach is inside
        /// [0.7, 1.4] × (r1+r2) get a patch (solid overlaps weld themselves,
        /// farther pairs aren't bridgeable).</summary>
        public static int AppendWebs(List<(Vector3 p, float r, Vector3 tan)> pts,
                                     List<(int start, int count)> ranges,
                                     float voxel, float blendK, int maxWebs,
                                     List<Vector3> tp, List<Vector3> tn, List<Vector3> tc, List<int> ti,
                                     out int skippedPairs)
        {
            skippedPairs = 0;
            if (!LoadTables() || pts == null || ranges == null || ranges.Count < 2) return 0;

            int n = pts.Count;
            var chainOf = new int[n];
            for (int c = 0; c < ranges.Count; c++)
                for (int i = 0; i < ranges[c].count; i++)
                    chainOf[ranges[c].start + i] = c;

            // spatial hash + per-pair closest approach (bead detection reuse)
            float cell = 0.05f;
            var grid = new Dictionary<long, List<int>>(n / 2 + 1);
            long Key(int kx, int ky, int kz) => (kx * 73856093L) ^ (ky * 19349663L) ^ (kz * 83492791L);
            int C(float v) => Mathf.FloorToInt(v / cell);
            for (int i = 0; i < n; i++)
            {
                long k = Key(C(pts[i].p.x), C(pts[i].p.y), C(pts[i].p.z));
                if (!grid.TryGetValue(k, out var l)) grid[k] = l = new List<int>(8);
                l.Add(i);
            }
            var best = new Dictionary<long, (float dSq, int i, int qi)>();
            for (int i = 0; i < n; i++)
            {
                var (p, r, _) = pts[i];
                int cx = C(p.x), cy = C(p.y), cz = C(p.z);
                for (int gx = cx - 1; gx <= cx + 1; gx++)
                    for (int gy = cy - 1; gy <= cy + 1; gy++)
                        for (int gz = cz - 1; gz <= cz + 1; gz++)
                        {
                            if (!grid.TryGetValue(Key(gx, gy, gz), out var l)) continue;
                            foreach (int qi in l)
                            {
                                if (chainOf[qi] <= chainOf[i]) continue;
                                var (q, qr, _) = pts[qi];
                                float lim = (r + qr) * 1.4f;
                                float dSq = (q - p).sqrMagnitude;
                                if (dSq > lim * lim) continue;
                                long pk = ((long)chainOf[i] << 32) | (uint)chainOf[qi];
                                if (!best.TryGetValue(pk, out var curBest) || dSq < curBest.dSq)
                                    best[pk] = (dSq, i, qi);
                            }
                        }
            }

            int webs = 0;
            var segsA = new List<Seg>(16); var segsB = new List<Seg>(16);
            foreach (var kvp in best)
            {
                var (dSq, ia, ib) = kvp.Value;
                float rsum = pts[ia].r + pts[ib].r;
                float d = Mathf.Sqrt(dSq);
                if (d < rsum * 0.7f) continue;  // solid overlap — welds itself
                if (webs >= maxWebs) { skippedPairs++; continue; }

                Vector3 a = pts[ia].p, b = pts[ib].p;
                Vector3 center = (a + b) * 0.5f;
                float half = Mathf.Max(pts[ia].r, pts[ib].r) * 2.2f + d * 0.5f;

                // segments of both chains inside the box (+margin)
                GatherSegs(pts, ranges, chainOf[ia], center, half + voxel * 2f, segsA);
                GatherSegs(pts, ranges, chainOf[ib], center, half + voxel * 2f, segsB);
                if (segsA.Count == 0 || segsB.Count == 0) continue;

                webs += EmitPatch(center, half, voxel, blendK, segsA, segsB, tp, tn, tc, ti) ? 1 : 0;
            }
            return webs;
        }

        private static void GatherSegs(List<(Vector3 p, float r, Vector3 tan)> pts,
                                       List<(int start, int count)> ranges, int chain,
                                       Vector3 center, float half, List<Seg> outSegs)
        {
            outSegs.Clear();
            var (start, count) = ranges[chain];
            float reach = half * 1.7321f; // box half-diagonal
            for (int i = 0; i < count - 1; i++)
            {
                Vector3 pa = pts[start + i].p, pb = pts[start + i + 1].p;
                if ((pa - center).sqrMagnitude > reach * reach &&
                    (pb - center).sqrMagnitude > reach * reach) continue;
                outSegs.Add(new Seg { A = pa, B = pb, Ra = pts[start + i].r, Rb = pts[start + i + 1].r });
            }
        }

        private static bool EmitPatch(Vector3 center, float half, float voxel, float k,
                                      List<Seg> segsA, List<Seg> segsB,
                                      List<Vector3> tp, List<Vector3> tn, List<Vector3> tc, List<int> ti)
        {
            int nx = Mathf.Clamp(Mathf.CeilToInt(2f * half / voxel), 4, 48);
            int nv = nx + 1;
            var field = new float[nv * nv * nv];
            Vector3 org = center - new Vector3(half, half, half);
            float step = 2f * half / nx;
            for (int z = 0; z < nv; z++)
                for (int y = 0; y < nv; y++)
                    for (int x = 0; x < nv; x++)
                    {
                        Vector3 pos = org + new Vector3(x, y, z) * step;
                        field[(z * nv + y) * nv + x] =
                            Smin(SegDist(segsA, pos), SegDist(segsB, pos), k);
                    }

            int before = ti.Count;
            var edgeP = new Vector3[12];
            for (int z = 0; z < nx; z++)
                for (int y = 0; y < nx; y++)
                    for (int x = 0; x < nx; x++)
                    {
                        int caseIdx = 0;
                        for (int c = 0; c < 8; c++)
                        {
                            var o = kCorner[c];
                            if (field[((z + o.z) * nv + (y + o.y)) * nv + (x + o.x)] < 0f)
                                caseIdx |= 1 << c;
                        }
                        if (caseIdx == 0 || caseIdx == 255) continue;
                        var tri = _triTable[caseIdx];
                        for (int t = 0; tri[t] != -1; t += 3)
                        {
                            for (int e = 0; e < 3; e++)
                            {
                                int edge = tri[t + e];
                                var ca = kCorner[kEdgeA[edge]]; var cb = kCorner[kEdgeB[edge]];
                                float fa = field[((z + ca.z) * nv + (y + ca.y)) * nv + (x + ca.x)];
                                float fb = field[((z + cb.z) * nv + (y + cb.y)) * nv + (x + cb.x)];
                                float tt = Mathf.Abs(fa - fb) > 1e-9f ? fa / (fa - fb) : 0.5f;
                                edgeP[e] = org + new Vector3(
                                    Mathf.Lerp(x + ca.x, x + cb.x, tt),
                                    Mathf.Lerp(y + ca.y, y + cb.y, tt),
                                    Mathf.Lerp(z + ca.z, z + cb.z, tt)) * step;
                            }
                            Vector3 va = edgeP[0], vb = edgeP[1], vc = edgeP[2];
                            Vector3 cen = (va + vb + vc) / 3f;
                            // keep only the MEMBRANE: skip triangles lying on (or
                            // inside) either bar's own surface, so patches never
                            // shadow the crisp tube mesh
                            float plain = Mathf.Min(SegDist(segsA, cen), SegDist(segsB, cen));
                            if (plain < voxel * 0.1f) continue;
                            // outward = +gradient of the blended field
                            Vector3 grad = new Vector3(
                                FieldAt(segsA, segsB, k, cen + Vector3.right * voxel) -
                                FieldAt(segsA, segsB, k, cen - Vector3.right * voxel),
                                FieldAt(segsA, segsB, k, cen + Vector3.up * voxel) -
                                FieldAt(segsA, segsB, k, cen - Vector3.up * voxel),
                                FieldAt(segsA, segsB, k, cen + Vector3.forward * voxel) -
                                FieldAt(segsA, segsB, k, cen - Vector3.forward * voxel));
                            Vector3 fn = Vector3.Cross(vb - va, vc - va);
                            if (Vector3.Dot(fn, grad) < 0f) { var tmp = vb; vb = vc; vc = tmp; fn = -fn; }
                            Vector3 nrm = fn.sqrMagnitude > 1e-16f ? fn.normalized : Vector3.up;
                            int vb0 = tp.Count;
                            tp.Add(va); tp.Add(vb); tp.Add(vc);
                            tn.Add(nrm); tn.Add(nrm); tn.Add(nrm);
                            tc.Add(Vector3.one); tc.Add(Vector3.one); tc.Add(Vector3.one);
                            ti.Add(vb0); ti.Add(vb0 + 1); ti.Add(vb0 + 2);
                        }
                    }
            return ti.Count > before;
        }

        private static float FieldAt(List<Seg> a, List<Seg> b, float k, Vector3 x)
            => Smin(SegDist(a, x), SegDist(b, x), k);
    }
}
