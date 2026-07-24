// Per-vertex ambient-occlusion bake for the web/AR export (GLB + USDZ).
// Pure array math over the final export-space geometry (surface mesh + curve
// tubes): a median/mid-split BVH over all triangles, cosine-weighted
// hemisphere rays per vertex (deterministic golden-ratio spiral — no RNG),
// then two neighbour-smoothing passes to hide per-vertex noise. The result
// multiplies into the sRGB vertex colours in LINEAR space, so both writers
// inherit the bake unchanged (the USDZ atlas averages the darkened corners).
//
// Design notes:
// - Any-hit binary occlusion within maxDistance: softness comes from the
//   cosine weighting + smoothing, not distance falloff — any-hit is ~2x
//   cheaper than closest-hit and the difference disappears after smoothing.
// - AO is directionless, so it composes with the viewer's PBR lighting
//   without double-lighting (unlike baking a directional light would).
// - The curve tubes always OCCLUDE (they sit against the body and should
//   ground it); whether they also DARKEN is the caller's choice
//   (Options.affectCurves) — the tubes read as light strokes, and shading
//   them like solid geometry fights that.

using System;
using System.Threading.Tasks;
using UnityEngine;

namespace TSDF
{
    public static class MeshAO
    {
        /// <summary>Web-export AO bake settings. strength 0 disables the bake
        /// entirely (ExportFiles skips the computation).</summary>
        [Serializable]
        public struct Options
        {
            public float strength;     // 0..1 colour multiply weight; 0 = off
            public int samples;        // hemisphere rays per vertex
            public float maxDistance;  // occlusion search radius (m)
            public bool affectCurves;  // darken tube vertices too (they always occlude)
        }

        /// <summary>Per-vertex openness in [0,1]: 1 = unoccluded, 0 = fully
        /// occluded. Rays test against ALL triangles in (pos, tri); vertices
        /// with degenerate normals stay fully open. Thread-safe pure math —
        /// internally parallel, no Unity API touched after the allocations.</summary>
        public static float[] Compute(Vector3[] pos, Vector3[] nrm, int[] tri, int samples, float maxDistance)
        {
            int vc = pos.Length;
            var open = new float[vc];
            if (tri.Length == 0)
            {
                for (int i = 0; i < vc; i++) open[i] = 1f;
                return open;
            }

            var bvh = new Bvh(pos, tri);
            var dirs = CosineHemisphere(Mathf.Max(1, samples));

            Parallel.For(0, vc, () => new int[128], (i, _, stack) =>
            {
                Vector3 n = nrm[i];
                float len = n.magnitude;
                if (len < 1e-6f) { open[i] = 1f; return stack; }
                n /= len;
                Vector3 up = Mathf.Abs(n.y) < 0.99f ? Vector3.up : Vector3.right;
                Vector3 t = Vector3.Normalize(Vector3.Cross(up, n));
                Vector3 b = Vector3.Cross(n, t);
                Vector3 o = pos[i] + n * 0.002f; // lift off the surface: no self-hit

                int hits = 0;
                for (int s = 0; s < dirs.Length; s++)
                {
                    Vector3 d = t * dirs[s].x + b * dirs[s].y + n * dirs[s].z;
                    if (bvh.Occluded(o, d, maxDistance, stack)) hits++;
                }
                open[i] = 1f - hits / (float)dirs.Length;
                return stack;
            }, _ => { });

            Smooth(open, tri, 2);
            return open;
        }

        /// <summary>Darken the first <paramref name="count"/> vertex colours by
        /// their openness: linear-space multiply by lerp(1, open, strength).
        /// Colours are sRGB 0-1 in and out (the writers' input contract).</summary>
        public static void ApplyToColors(Vector3[] col, float[] open, float strength, int count)
        {
            strength = Mathf.Clamp01(strength);
            for (int i = 0; i < count; i++)
            {
                float f = 1f - strength * (1f - open[i]);
                col[i] = new Vector3(
                    LinearToSrgb(SrgbToLinear(col[i].x) * f),
                    LinearToSrgb(SrgbToLinear(col[i].y) * f),
                    LinearToSrgb(SrgbToLinear(col[i].z) * f));
            }
        }

        // Deterministic cosine-weighted hemisphere (+Z up): golden-ratio spiral
        // in phi, sqrt stratification in radius. pdf ∝ cosθ, so a plain hit
        // average IS the cosine-weighted visibility integral.
        private static Vector3[] CosineHemisphere(int samples)
        {
            var dirs = new Vector3[samples];
            for (int i = 0; i < samples; i++)
            {
                float u = (i + 0.5f) / samples;
                float phi = 2f * Mathf.PI * ((i * 0.6180339887f) % 1f);
                float r = Mathf.Sqrt(u), z = Mathf.Sqrt(1f - u);
                dirs[i] = new Vector3(r * Mathf.Cos(phi), r * Mathf.Sin(phi), z);
            }
            return dirs;
        }

        // Two Jacobi passes of 50/50 self/neighbour averaging along mesh edges.
        // Shared edges count twice — harmless for smoothing weights.
        private static void Smooth(float[] ao, int[] tri, int passes)
        {
            var sum = new float[ao.Length];
            var cnt = new int[ao.Length];
            for (int p = 0; p < passes; p++)
            {
                Array.Clear(sum, 0, sum.Length);
                Array.Clear(cnt, 0, cnt.Length);
                for (int t = 0; t < tri.Length; t += 3)
                {
                    int a = tri[t], b = tri[t + 1], c = tri[t + 2];
                    sum[a] += ao[b] + ao[c]; cnt[a] += 2;
                    sum[b] += ao[a] + ao[c]; cnt[b] += 2;
                    sum[c] += ao[a] + ao[b]; cnt[c] += 2;
                }
                for (int i = 0; i < ao.Length; i++)
                    if (cnt[i] > 0) ao[i] = 0.5f * ao[i] + 0.5f * (sum[i] / cnt[i]);
            }
        }

        private static float SrgbToLinear(float s)
        {
            s = Mathf.Clamp01(s);
            return s <= 0.04045f ? s / 12.92f : Mathf.Pow((s + 0.055f) / 1.055f, 2.4f);
        }

        private static float LinearToSrgb(float l)
        {
            l = Mathf.Clamp01(l);
            return l <= 0.0031308f ? l * 12.92f : 1.055f * Mathf.Pow(l, 1f / 2.4f) - 0.055f;
        }

        // ---- BVH: preorder nodes (left child = index+1, right stored), ----
        // ---- triangles baked into leaf order for cache-friendly any-hit ----
        private sealed class Bvh
        {
            private struct Node
            {
                public Vector3 bmin, bmax;
                public int leftFirst; // leaf: first tri; internal: right-child node index
                public int count;     // leaf: tri count; internal: 0
            }

            private readonly Node[] _nodes;
            private readonly Vector3[] _v0, _e1, _e2; // Möller precompute, leaf order
            private int _nodeCount;

            public Bvh(Vector3[] pos, int[] tri)
            {
                int tc = tri.Length / 3;
                var cent = new Vector3[tc];
                var tmin = new Vector3[tc];
                var tmax = new Vector3[tc];
                var order = new int[tc];
                var a0 = new Vector3[tc]; var a1 = new Vector3[tc]; var a2 = new Vector3[tc];
                for (int t = 0; t < tc; t++)
                {
                    Vector3 a = pos[tri[t * 3]], b = pos[tri[t * 3 + 1]], c = pos[tri[t * 3 + 2]];
                    a0[t] = a; a1[t] = b; a2[t] = c;
                    tmin[t] = Vector3.Min(a, Vector3.Min(b, c));
                    tmax[t] = Vector3.Max(a, Vector3.Max(b, c));
                    cent[t] = (a + b + c) / 3f;
                    order[t] = t;
                }
                _nodes = new Node[Mathf.Max(2, tc * 2)];
                Build(0, tc, cent, tmin, tmax, order, 0);
                _v0 = new Vector3[tc]; _e1 = new Vector3[tc]; _e2 = new Vector3[tc];
                for (int t = 0; t < tc; t++)
                {
                    int src = order[t];
                    _v0[t] = a0[src];
                    _e1[t] = a1[src] - a0[src];
                    _e2[t] = a2[src] - a0[src];
                }
            }

            // Mid-split on the longest centroid axis; halve the range when the
            // split degenerates (identical centroids). Depth cap 60 keeps the
            // traversal stack bounded regardless of input.
            private int Build(int first, int count, Vector3[] cent, Vector3[] tmin, Vector3[] tmax,
                              int[] order, int depth)
            {
                int ni = _nodeCount++;
                Vector3 bmin = Vector3.positiveInfinity, bmax = Vector3.negativeInfinity;
                Vector3 cmin = Vector3.positiveInfinity, cmax = Vector3.negativeInfinity;
                for (int i = first; i < first + count; i++)
                {
                    int t = order[i];
                    bmin = Vector3.Min(bmin, tmin[t]); bmax = Vector3.Max(bmax, tmax[t]);
                    cmin = Vector3.Min(cmin, cent[t]); cmax = Vector3.Max(cmax, cent[t]);
                }
                if (count <= 4 || depth >= 60)
                {
                    _nodes[ni] = new Node { bmin = bmin, bmax = bmax, leftFirst = first, count = count };
                    return ni;
                }
                Vector3 ext = cmax - cmin;
                int axis = ext.x >= ext.y ? (ext.x >= ext.z ? 0 : 2) : (ext.y >= ext.z ? 1 : 2);
                float split = (Axis(cmin, axis) + Axis(cmax, axis)) * 0.5f;
                int mid = Partition(order, cent, first, count, axis, split);
                if (mid == first || mid == first + count) mid = first + count / 2;
                Build(first, mid - first, cent, tmin, tmax, order, depth + 1); // == ni+1
                int ri = Build(mid, first + count - mid, cent, tmin, tmax, order, depth + 1);
                _nodes[ni] = new Node { bmin = bmin, bmax = bmax, leftFirst = ri, count = 0 };
                return ni;
            }

            private static int Partition(int[] order, Vector3[] cent, int first, int count,
                                         int axis, float split)
            {
                int i = first, j = first + count - 1;
                while (i <= j)
                {
                    if (Axis(cent[order[i]], axis) < split) i++;
                    else { (order[i], order[j]) = (order[j], order[i]); j--; }
                }
                return i;
            }

            private static float Axis(Vector3 v, int a) => a == 0 ? v.x : a == 1 ? v.y : v.z;

            /// <summary>Any-hit within (1e-4, tMax]. stack must hold ≥128 ints
            /// (depth-capped tree needs ≤ ~64).</summary>
            public bool Occluded(Vector3 o, Vector3 d, float tMax, int[] stack)
            {
                float ix = 1f / d.x, iy = 1f / d.y, iz = 1f / d.z; // ±inf on zero components is fine
                int sp = 0;
                stack[sp++] = 0;
                while (sp > 0)
                {
                    int idx = stack[--sp];
                    ref Node n = ref _nodes[idx];
                    float t1 = (n.bmin.x - o.x) * ix, t2 = (n.bmax.x - o.x) * ix;
                    float tn = Mathf.Min(t1, t2), tf = Mathf.Max(t1, t2);
                    t1 = (n.bmin.y - o.y) * iy; t2 = (n.bmax.y - o.y) * iy;
                    tn = Mathf.Max(tn, Mathf.Min(t1, t2)); tf = Mathf.Min(tf, Mathf.Max(t1, t2));
                    t1 = (n.bmin.z - o.z) * iz; t2 = (n.bmax.z - o.z) * iz;
                    tn = Mathf.Max(tn, Mathf.Min(t1, t2)); tf = Mathf.Min(tf, Mathf.Max(t1, t2));
                    if (tf < Mathf.Max(tn, 0f) || tn > tMax) continue;

                    if (n.count > 0)
                    {
                        for (int t = n.leftFirst; t < n.leftFirst + n.count; t++)
                            if (HitTri(t, o, d, tMax)) return true;
                    }
                    else
                    {
                        stack[sp++] = idx + 1;
                        stack[sp++] = n.leftFirst;
                    }
                }
                return false;
            }

            private bool HitTri(int t, Vector3 o, Vector3 d, float tMax)
            {
                Vector3 e1 = _e1[t], e2 = _e2[t];
                Vector3 p = Vector3.Cross(d, e2);
                float det = Vector3.Dot(e1, p);
                if (det > -1e-9f && det < 1e-9f) return false;
                float f = 1f / det;
                Vector3 s = o - _v0[t];
                float u = f * Vector3.Dot(s, p);
                if (u < 0f || u > 1f) return false;
                Vector3 q = Vector3.Cross(s, e1);
                float v = f * Vector3.Dot(d, q);
                if (v < 0f || u + v > 1f) return false;
                float th = f * Vector3.Dot(e2, q);
                return th > 1e-4f && th < tMax;
            }
        }
    }
}
