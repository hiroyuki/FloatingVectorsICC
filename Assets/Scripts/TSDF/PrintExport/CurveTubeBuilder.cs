// Curve-polyline → tube-mesh builder for the web/AR export (3-2: mechanical
// extraction from TSDFPrintExporter; polyline simplification lives in MeshOps).

using System.Collections.Generic;
using UnityEngine;

namespace TSDF
{
    internal static class CurveTubeBuilder
    {
        // Sweep each polyline into a closed tube: parallel-transport frames along
        // the curve, an N-sided ring per point (radial normals), quad strips
        // between rings and centre-fan end caps. Geometry is emitted directly in
        // export space (mirrored right-handed, recentred) with CCW-outward
        // winding, so it appends to the already-oriented mesh arrays untouched.
        // Returns the number of tubes actually emitted.
        // tipTaper: radius scale at the OLD end of the polyline (index 0 — curve
        // history runs oldest -> newest), lerped up to the full radius at the new
        // end, so trails read as strokes that fade in from the past. 1 = constant.
        // exportSpace false = keep the polylines' world space (no mirror, no
        // recentre; center/minY unused) for in-scene display meshes. Winding is
        // then mirrored relative to Unity's front faces — render with Cull Off.
        // raindrop: shape the radius ALONG the tube like a falling raindrop —
        // motion visualization, so the ROUND head sits at the NEWEST end
        // (index n-1, where the bridge anchors) and the chopstick tail thins
        // into the past. The head is a TRUE hemisphere lathed past the last
        // point (radius headR = min(radius, 40% of chain length)) — built
        // geometrically, so sparse centreline sampling can never stretch it.
        // The body swells quadratically from headR*tipTaper at the tail to
        // headR at the head, meeting the hemisphere tangent-continuously.
        // Cross-section stays circular. Overrides the plain tipTaper ramp.
        public static int AppendCurveTubes(List<Vector3[]> lines, List<Vector3> lineCols, float brightness,
                                           float radius, int sides, float tolerance, Vector3 center, float minY,
                                           List<Vector3> pos, List<Vector3> nrm, List<Vector3> col,
                                           List<int> idx, float tipTaper = 1f, bool exportSpace = true,
                                           bool raindrop = false)
        {
            int tubes = 0;
            var p = new List<Vector3>(256);
            for (int li = 0; li < lines.Count; li++)
            {
                // To export space; drop near-duplicate points (a paused pose makes
                // the oldest history frames identical -> zero-length segments).
                p.Clear();
                foreach (var w in lines[li])
                {
                    var e = exportSpace ? new Vector3(-(w.x - center.x), w.y - minY, w.z - center.z) : w;
                    if (p.Count == 0 || (e - p[p.Count - 1]).sqrMagnitude > 1e-10f) p.Add(e);
                }
                if (tolerance > 0f && p.Count > 2) MeshOps.SimplifyPolyline(p, tolerance);
                int n = p.Count;
                if (n < 2) continue;
                tubes++;

                Vector3 c = lineCols[li] * brightness;
                c = new Vector3(Mathf.Clamp01(c.x), Mathf.Clamp01(c.y), Mathf.Clamp01(c.z));

                // raindrop silhouette needs arc length from the head (newest end)
                float[] cum = null; float len = 0f, headR = 0f, tailR = 0f;
                if (raindrop)
                {
                    cum = new float[n];
                    for (int i = 1; i < n; i++) cum[i] = cum[i - 1] + (p[i] - p[i - 1]).magnitude;
                    len = cum[n - 1];
                    headR = Mathf.Min(radius, Mathf.Max(len * 0.4f, 1e-4f));
                    tailR = headR * Mathf.Clamp01(tipTaper);
                }

                int ringBase = pos.Count;
                Vector3 startT = (p[1] - p[0]).normalized;
                Vector3 u = Vector3.Cross(startT, Mathf.Abs(startT.y) < 0.9f ? Vector3.up : Vector3.right).normalized;
                Vector3 prevT = startT;
                for (int i = 0; i < n; i++)
                {
                    Vector3 t = p[Mathf.Min(i + 1, n - 1)] - p[Mathf.Max(i - 1, 0)];
                    t = t.sqrMagnitude > 1e-12f ? t.normalized : prevT;
                    u = (Quaternion.FromToRotation(prevT, t) * u).normalized;
                    prevT = t;
                    Vector3 v = Vector3.Cross(t, u);
                    float r;
                    bool drop = raindrop && len > 1e-6f;
                    if (drop)
                    {
                        // smooth quadratic swell: thin chopstick tail for most of
                        // the length, blending tangent-continuously into the
                        // hemisphere head cap appended after the loop
                        float u01 = cum[i] / len; // 0 = tail, 1 = head
                        r = tailR + (headR - tailR) * u01 * u01;
                    }
                    else
                        r = radius * Mathf.Lerp(Mathf.Clamp01(tipTaper), 1f, n > 1 ? (float)i / (n - 1) : 1f);
                    for (int k = 0; k < sides; k++)
                    {
                        float ang = 2f * Mathf.PI * k / sides;
                        Vector3 rd = Mathf.Cos(ang) * u + Mathf.Sin(ang) * v;
                        pos.Add(p[i] + rd * r);
                        nrm.Add(rd);
                        col.Add(c);
                    }
                }
                Vector3 endT = prevT;

                // Raindrop head: a TRUE hemisphere lathed past the newest point —
                // built from headR alone, so centreline sampling can never
                // stretch it into a spike. Rings continue the tube's index grid.
                int rings = n;
                bool headCap = raindrop && len > 1e-6f;
                if (headCap)
                {
                    Vector3 vEnd = Vector3.Cross(endT, u);
                    const int capRings = 5;
                    for (int j = 1; j <= capRings; j++)
                    {
                        float a = 0.5f * Mathf.PI * j / (capRings + 1);
                        Vector3 cc = p[n - 1] + endT * (headR * Mathf.Sin(a));
                        float cr = headR * Mathf.Cos(a);
                        for (int k = 0; k < sides; k++)
                        {
                            float ang = 2f * Mathf.PI * k / sides;
                            Vector3 rd = Mathf.Cos(ang) * u + Mathf.Sin(ang) * vEnd;
                            pos.Add(cc + rd * cr);
                            nrm.Add((rd * Mathf.Cos(a) + endT * Mathf.Sin(a)).normalized);
                            col.Add(c);
                        }
                    }
                    rings = n + capRings;
                }

                // Side quads: ring k -> k+1 x segment i -> i+1, CCW-outward.
                for (int i = 0; i < rings - 1; i++)
                    for (int k = 0; k < sides; k++)
                    {
                        int k1 = (k + 1) % sides;
                        int a = ringBase + i * sides + k;
                        int b = ringBase + i * sides + k1;
                        int c2 = ringBase + (i + 1) * sides + k1;
                        int d = ringBase + (i + 1) * sides + k;
                        idx.Add(a); idx.Add(b); idx.Add(c2);
                        idx.Add(a); idx.Add(c2); idx.Add(d);
                    }

                // End caps: ring verts duplicated with the axial normal + centre fan.
                int capBase = pos.Count;
                for (int k = 0; k < sides; k++) { pos.Add(pos[ringBase + k]); nrm.Add(-startT); col.Add(c); }
                pos.Add(p[0]); nrm.Add(-startT); col.Add(c);
                for (int k = 0; k < sides; k++)
                { idx.Add(capBase + sides); idx.Add(capBase + (k + 1) % sides); idx.Add(capBase + k); }

                capBase = pos.Count;
                int lastRing = ringBase + (rings - 1) * sides;
                Vector3 pole = headCap ? p[n - 1] + endT * headR : p[n - 1];
                for (int k = 0; k < sides; k++) { pos.Add(pos[lastRing + k]); nrm.Add(endT); col.Add(c); }
                pos.Add(pole); nrm.Add(endT); col.Add(c);
                for (int k = 0; k < sides; k++)
                { idx.Add(capBase + sides); idx.Add(capBase + k); idx.Add(capBase + (k + 1) % sides); }
            }
            return tubes;
        }
    }
}
