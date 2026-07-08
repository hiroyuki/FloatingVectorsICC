// Pure-geometry mesh operations shared by the print/web exports (3-2: mechanical
// extraction from TSDFPrintExporter — the exporter keeps orchestration + GPU work).
// All functions are allocation-transparent CPU passes over welded mesh arrays.

using System;
using System.Collections.Generic;
using System.Threading.Tasks;
using UnityEngine;

namespace TSDF
{
    internal static class MeshOps
    {
        // Weld the per-slab triangle soup (Tri = p0 c0 p1 c1 p2 c2, 3 floats each)
        // into unique vertices + index list. Vertex colours are averaged over all
        // soup occurrences. Keys are positions quantised to 0.1 mm packed into a
        // long (21 bits/axis = ±104 m range).
        public static void WeldSlabs(List<(float[] data, int tris)> slabs,
                                     out Vector3[] positions, out Vector3[] colors, out int[] triangles)
        {
            var map = new Dictionary<long, int>();
            var pos = new List<Vector3>();
            var colSum = new List<Vector3>();
            var colCnt = new List<int>();
            var tri = new List<int>();

            foreach (var (data, tris) in slabs)
                for (int t = 0; t < tris; t++)
                {
                    int o = t * 18;
                    for (int v = 0; v < 3; v++)
                    {
                        int p = o + v * 6;
                        var w = new Vector3(data[p], data[p + 1], data[p + 2]);
                        long kx = (long)Math.Round(w.x * 10000.0) + 1048576;
                        long ky = (long)Math.Round(w.y * 10000.0) + 1048576;
                        long kz = (long)Math.Round(w.z * 10000.0) + 1048576;
                        long key = (kx << 42) | (ky << 21) | kz;
                        if (!map.TryGetValue(key, out int idx))
                        {
                            idx = pos.Count;
                            map.Add(key, idx);
                            pos.Add(w);
                            colSum.Add(Vector3.zero);
                            colCnt.Add(0);
                        }
                        colSum[idx] += new Vector3(data[p + 3], data[p + 4], data[p + 5]);
                        colCnt[idx]++;
                        tri.Add(idx);
                    }
                }

            positions = pos.ToArray();
            colors = new Vector3[positions.Length];
            for (int i = 0; i < colors.Length; i++)
                colors[i] = colCnt[i] > 0 ? colSum[i] / colCnt[i] : Vector3.one * 0.5f;
            triangles = tri.ToArray();
        }

        // Taubin λ|μ smoothing (non-shrinking): alternating positive/negative
        // uniform-Laplacian steps over the unique-edge adjacency. In-place on pos.
        //
        // The unique-edge set is built by sorting a primitive long[] of packed edge
        // keys and deduping linearly. Do NOT switch this to HashSet<long>: at
        // millions of entries it degrades to minutes inside the editor Mono
        // (measured 397 s where the sort path takes ~1 s).
        public static void TaubinSmooth(Vector3[] pos, int[] tri, int iterations)
        {
            int n = pos.Length;
            var phase = System.Diagnostics.Stopwatch.StartNew();

            // All triangle edges as packed keys (min<<32|max); -1 = degenerate.
            var keys = new long[tri.Length];
            int kc = 0;
            for (int t = 0; t < tri.Length; t += 3)
            {
                keys[kc++] = EdgeKey(tri[t], tri[t + 1]);
                keys[kc++] = EdgeKey(tri[t + 1], tri[t + 2]);
                keys[kc++] = EdgeKey(tri[t + 2], tri[t]);
            }
            Array.Sort(keys);
            int e0 = 0;
            while (e0 < keys.Length && keys[e0] < 0) e0++; // skip degenerates
            int uniq = 0;
            for (int i = e0; i < keys.Length; i++)
                if (uniq == 0 || keys[i] != keys[uniq - 1]) keys[uniq++] = keys[i];

            var degree = new int[n];
            for (int i = 0; i < uniq; i++)
            { degree[(int)(keys[i] >> 32)]++; degree[(int)(keys[i] & 0xffffffffL)]++; }
            var offset = new int[n + 1];
            for (int i = 0; i < n; i++) offset[i + 1] = offset[i] + degree[i];
            var adj = new int[offset[n]];
            var cursor = (int[])offset.Clone();
            for (int i = 0; i < uniq; i++)
            {
                int a = (int)(keys[i] >> 32), b = (int)(keys[i] & 0xffffffffL);
                adj[cursor[a]++] = b;
                adj[cursor[b]++] = a;
            }
            Debug.Log($"[TSDFPrintExporter] adjacency: {uniq} edges in {phase.ElapsedMilliseconds} ms");
            phase.Restart();

            const float lambda = 0.5f, mu = -0.53f;
            var tmp = new Vector3[n];
            var src = pos;
            for (int it = 0; it < iterations; it++)
            {
                LaplacianPass(src, tmp, offset, adj, lambda);
                LaplacianPass(tmp, src, offset, adj, mu);
                if ((it + 1) % 5 == 0 || it + 1 == iterations)
                    Debug.Log($"[TSDFPrintExporter] smooth {it + 1}/{iterations} ({phase.ElapsedMilliseconds} ms elapsed)");
            }
        }

        private static long EdgeKey(int a, int b)
        {
            if (a == b) return -1;
            long lo = Math.Min(a, b), hi = Math.Max(a, b);
            return (lo << 32) | hi;
        }

        // Reads src, writes dst — no cross-vertex write dependency, so the vertex
        // range parallelises cleanly across cores.
        private static void LaplacianPass(Vector3[] src, Vector3[] dst, int[] offset, int[] adj, float factor)
        {
            Parallel.For(0, src.Length, v =>
            {
                int a = offset[v], b = offset[v + 1];
                if (b == a) { dst[v] = src[v]; return; }
                Vector3 avg = Vector3.zero;
                for (int i = a; i < b; i++) avg += src[adj[i]];
                avg /= (b - a);
                dst[v] = src[v] + factor * (avg - src[v]);
            });
        }

        // Douglas-Peucker in place: keeps endpoints, drops points closer to the
        // local chord than the tolerance. The Catmull-Rom polylines are heavily
        // oversampled where the motion is slow, so 1-2 mm typically halves the
        // point count without visible change.
        public static void SimplifyPolyline(List<Vector3> p, float tol)
        {
            int n = p.Count;
            var keep = new bool[n];
            keep[0] = keep[n - 1] = true;
            var stack = new Stack<(int lo, int hi)>();
            stack.Push((0, n - 1));
            float tol2 = tol * tol;
            while (stack.Count > 0)
            {
                var (lo, hi) = stack.Pop();
                if (hi - lo < 2) continue;
                Vector3 a = p[lo], ab = p[hi] - a;
                float abLen2 = ab.sqrMagnitude;
                float worst = -1f;
                int wi = -1;
                for (int i = lo + 1; i < hi; i++)
                {
                    Vector3 ap = p[i] - a;
                    float t = abLen2 > 1e-16f ? Mathf.Clamp01(Vector3.Dot(ap, ab) / abLen2) : 0f;
                    float d2 = (ap - t * ab).sqrMagnitude;
                    if (d2 > worst) { worst = d2; wi = i; }
                }
                if (worst > tol2)
                {
                    keep[wi] = true;
                    stack.Push((lo, wi));
                    stack.Push((wi, hi));
                }
            }
            int w = 0;
            for (int i = 0; i < n; i++)
                if (keep[i]) p[w++] = p[i];
            p.RemoveRange(w, n - w);
        }

        // Area-weighted smooth vertex normals (the accumulated cross product is
        // proportional to triangle area, so big faces dominate — the right bias
        // for an MC surface).
        public static Vector3[] ComputeVertexNormals(Vector3[] pos, int[] tri)
        {
            var n = new Vector3[pos.Length];
            for (int t = 0; t < tri.Length; t += 3)
            {
                int i0 = tri[t], i1 = tri[t + 1], i2 = tri[t + 2];
                Vector3 fn = Vector3.Cross(pos[i1] - pos[i0], pos[i2] - pos[i0]);
                n[i0] += fn; n[i1] += fn; n[i2] += fn;
            }
            for (int i = 0; i < n.Length; i++)
            {
                float len = n[i].magnitude;
                n[i] = len > 1e-12f ? n[i] / len : Vector3.up;
            }
            return n;
        }
    }
}
