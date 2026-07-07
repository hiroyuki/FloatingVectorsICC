// Quadric edge-collapse decimation (Garland–Heckbert) for the web export: the
// print STL keeps the full Marching-Cubes resolution, but a web/AR file wants
// ~150k triangles, not 1-2M. Simplified variant tuned for the welded, Taubin-
// smoothed MC surface:
//   - collapse position is the best of {a, b, midpoint} (no 4x4 solve),
//   - no incidence lists: stale heap entries are detected by per-vertex version
//     stamps and re-pushed after a Find() remap,
//   - removed-face counting is approximate (2 per collapse), so the driver
//     re-runs passes until the rebuilt count lands within ~5% of the target,
//   - boundary edges (the floor-crop rim) get a heavy perpendicular constraint
//     quadric so the rim doesn't erode inward,
//   - vertex colours merge count-weighted across collapses.

using System;
using System.Collections.Generic;
using UnityEngine;

namespace TSDF
{
    internal static class MeshDecimator
    {
        /// <summary>Simplify in place to approximately targetTris triangles.</summary>
        public static void Simplify(ref Vector3[] pos, ref Vector3[] col, ref int[] tri, int targetTris)
        {
            for (int pass = 0; pass < 4 && tri.Length / 3 > (long)targetTris * 21 / 20; pass++)
                if (!CollapsePass(ref pos, ref col, ref tri, targetTris))
                    break;
        }

        private static bool CollapsePass(ref Vector3[] pos, ref Vector3[] col, ref int[] tri, int targetTris)
        {
            int nv = pos.Length, nt = tri.Length / 3;
            var q = new double[nv * 10]; // symmetric 4x4 per vertex

            // Face quadrics, area-weighted.
            int aliveTris = 0;
            for (int t = 0; t < nt; t++)
            {
                int i0 = tri[t * 3], i1 = tri[t * 3 + 1], i2 = tri[t * 3 + 2];
                if (i0 == i1 || i1 == i2 || i0 == i2) continue;
                aliveTris++;
                Vector3 a = pos[i0], b = pos[i1], c = pos[i2];
                Vector3 n = Vector3.Cross(b - a, c - a);
                float len = n.magnitude;
                if (len < 1e-16f) continue;
                n /= len;
                double w = len * 0.5; // triangle area
                double d = -Vector3.Dot(n, a);
                AddPlane(q, i0, n, d, w); AddPlane(q, i1, n, d, w); AddPlane(q, i2, n, d, w);
            }
            if (aliveTris <= targetTris) return false;

            // Edge -> face-count via the sort-and-scan pattern (see TaubinSmooth:
            // no HashSet at these sizes). faceOf rides along for boundary planes.
            var keys = new long[nt * 3];
            var faceOf = new int[nt * 3];
            int ec = 0;
            for (int t = 0; t < nt; t++)
            {
                int i0 = tri[t * 3], i1 = tri[t * 3 + 1], i2 = tri[t * 3 + 2];
                if (i0 == i1 || i1 == i2 || i0 == i2) continue;
                keys[ec] = EdgeKey(i0, i1); faceOf[ec++] = t;
                keys[ec] = EdgeKey(i1, i2); faceOf[ec++] = t;
                keys[ec] = EdgeKey(i2, i0); faceOf[ec++] = t;
            }
            Array.Sort(keys, faceOf, 0, ec);

            var ver = new int[nv];
            var heap = new List<Entry>(ec / 2 + 16);
            for (int i = 0; i < ec;)
            {
                int j = i + 1;
                while (j < ec && keys[j] == keys[i]) j++;
                int a = (int)(keys[i] >> 32), b = (int)(keys[i] & 0xffffffffL);
                if (j - i == 1)
                {
                    // Boundary edge: constraint plane through the edge, perpendicular
                    // to the adjacent face, weighted by edge length so long rim edges
                    // resist hardest.
                    int t = faceOf[i];
                    Vector3 fa = pos[tri[t * 3]], fb = pos[tri[t * 3 + 1]], fc = pos[tri[t * 3 + 2]];
                    Vector3 fn = Vector3.Cross(fb - fa, fc - fa);
                    Vector3 ed = pos[b] - pos[a];
                    Vector3 cn = Vector3.Cross(ed, fn);
                    float cl = cn.magnitude;
                    if (cl > 1e-16f)
                    {
                        cn /= cl;
                        double bw = 100.0 * ed.sqrMagnitude;
                        double bd = -Vector3.Dot(cn, pos[a]);
                        AddPlane(q, a, cn, bd, bw); AddPlane(q, b, cn, bd, bw);
                    }
                }
                i = j;
            }
            for (int i = 0; i < ec;)
            {
                int j = i + 1;
                while (j < ec && keys[j] == keys[i]) j++;
                int a = (int)(keys[i] >> 32), b = (int)(keys[i] & 0xffffffffL);
                double cost = EdgeCost(q, pos, a, b, out Vector3 p);
                HeapPush(heap, new Entry { cost = cost, a = a, b = b, va = 0, vb = 0, p = p });
                i = j;
            }

            var parent = new int[nv];
            for (int i = 0; i < nv; i++) parent[i] = i;
            var colW = new int[nv];
            for (int i = 0; i < nv; i++) colW[i] = 1;

            int alive = aliveTris;
            while (alive > targetTris && heap.Count > 0)
            {
                Entry e = HeapPop(heap);
                int a = Find(parent, e.a), b = Find(parent, e.b);
                if (a == b) continue;
                if (ver[a] != e.va || ver[b] != e.vb || a != e.a || b != e.b)
                {
                    // Stale (an endpoint collapsed since): re-rank under the merged
                    // quadrics and try again later.
                    double cost = EdgeCost(q, pos, a, b, out Vector3 np);
                    HeapPush(heap, new Entry { cost = cost, a = a, b = b, va = ver[a], vb = ver[b], p = np });
                    continue;
                }
                pos[a] = e.p;
                for (int k = 0; k < 10; k++) q[a * 10 + k] += q[b * 10 + k];
                col[a] = (col[a] * colW[a] + col[b] * colW[b]) / (colW[a] + colW[b]);
                colW[a] += colW[b];
                parent[b] = a;
                ver[a]++;
                alive -= 2; // approximate (boundary collapses remove 1) — see driver
            }

            // Rebuild compact arrays.
            var remap = new int[nv];
            for (int i = 0; i < nv; i++) remap[i] = -1;
            var newTri = new List<int>(Mathf.Max(16, alive * 3));
            for (int t = 0; t < nt; t++)
            {
                int i0 = Find(parent, tri[t * 3]), i1 = Find(parent, tri[t * 3 + 1]), i2 = Find(parent, tri[t * 3 + 2]);
                if (i0 == i1 || i1 == i2 || i0 == i2) continue;
                newTri.Add(i0); newTri.Add(i1); newTri.Add(i2);
            }
            int nv2 = 0;
            for (int i = 0; i < newTri.Count; i++)
                if (remap[newTri[i]] < 0) remap[newTri[i]] = nv2++;
            var newPos = new Vector3[nv2];
            var newCol = new Vector3[nv2];
            for (int i = 0; i < nv; i++)
                if (remap[i] >= 0) { newPos[remap[i]] = pos[i]; newCol[remap[i]] = col[i]; }
            var nt2 = new int[newTri.Count];
            for (int i = 0; i < newTri.Count; i++) nt2[i] = remap[newTri[i]];

            bool progressed = nt2.Length < aliveTris * 3;
            pos = newPos; col = newCol; tri = nt2;
            return progressed;
        }

        private struct Entry
        {
            public double cost;
            public int a, b, va, vb;
            public Vector3 p;
        }

        private static long EdgeKey(int a, int b)
        {
            long lo = Math.Min(a, b), hi = Math.Max(a, b);
            return (lo << 32) | hi;
        }

        private static int Find(int[] parent, int i)
        {
            while (parent[i] != i) { parent[i] = parent[parent[i]]; i = parent[i]; }
            return i;
        }

        private static void AddPlane(double[] q, int vi, Vector3 n, double d, double w)
        {
            int o = vi * 10;
            double a = n.x, b = n.y, c = n.z;
            q[o] += w * a * a; q[o + 1] += w * a * b; q[o + 2] += w * a * c; q[o + 3] += w * a * d;
            q[o + 4] += w * b * b; q[o + 5] += w * b * c; q[o + 6] += w * b * d;
            q[o + 7] += w * c * c; q[o + 8] += w * c * d;
            q[o + 9] += w * d * d;
        }

        private static double QuadricEval(double[] q, int oa, int ob, Vector3 v)
        {
            double x = v.x, y = v.y, z = v.z;
            double q11 = q[oa] + q[ob], q12 = q[oa + 1] + q[ob + 1], q13 = q[oa + 2] + q[ob + 2],
                   q14 = q[oa + 3] + q[ob + 3], q22 = q[oa + 4] + q[ob + 4], q23 = q[oa + 5] + q[ob + 5],
                   q24 = q[oa + 6] + q[ob + 6], q33 = q[oa + 7] + q[ob + 7], q34 = q[oa + 8] + q[ob + 8],
                   q44 = q[oa + 9] + q[ob + 9];
            return q11 * x * x + 2 * q12 * x * y + 2 * q13 * x * z + 2 * q14 * x
                 + q22 * y * y + 2 * q23 * y * z + 2 * q24 * y
                 + q33 * z * z + 2 * q34 * z + q44;
        }

        private static double EdgeCost(double[] q, Vector3[] pos, int a, int b, out Vector3 best)
        {
            int oa = a * 10, ob = b * 10;
            Vector3 pa = pos[a], pb = pos[b], pm = (pa + pb) * 0.5f;
            double ca = QuadricEval(q, oa, ob, pa);
            double cb = QuadricEval(q, oa, ob, pb);
            double cm = QuadricEval(q, oa, ob, pm);
            if (cm <= ca && cm <= cb) { best = pm; return cm; }
            if (ca <= cb) { best = pa; return ca; }
            best = pb; return cb;
        }

        // Array-backed binary min-heap on Entry.cost.
        private static void HeapPush(List<Entry> h, Entry e)
        {
            h.Add(e);
            int i = h.Count - 1;
            while (i > 0)
            {
                int p = (i - 1) >> 1;
                if (h[p].cost <= h[i].cost) break;
                (h[p], h[i]) = (h[i], h[p]);
                i = p;
            }
        }

        private static Entry HeapPop(List<Entry> h)
        {
            Entry top = h[0];
            int last = h.Count - 1;
            h[0] = h[last];
            h.RemoveAt(last);
            int i = 0;
            while (true)
            {
                int l = i * 2 + 1, r = l + 1, m = i;
                if (l < h.Count && h[l].cost < h[m].cost) m = l;
                if (r < h.Count && h[r].cost < h[m].cost) m = r;
                if (m == i) break;
                (h[m], h[i]) = (h[i], h[m]);
                i = m;
            }
            return top;
        }
    }
}
