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
        public static int AppendCurveTubes(List<Vector3[]> lines, List<Vector3> lineCols, float brightness,
                                           float radius, int sides, float tolerance, Vector3 center, float minY,
                                           List<Vector3> pos, List<Vector3> nrm, List<Vector3> col,
                                           List<int> idx)
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
                    var e = new Vector3(-(w.x - center.x), w.y - minY, w.z - center.z);
                    if (p.Count == 0 || (e - p[p.Count - 1]).sqrMagnitude > 1e-10f) p.Add(e);
                }
                if (tolerance > 0f && p.Count > 2) MeshOps.SimplifyPolyline(p, tolerance);
                int n = p.Count;
                if (n < 2) continue;
                tubes++;

                Vector3 c = lineCols[li] * brightness;
                c = new Vector3(Mathf.Clamp01(c.x), Mathf.Clamp01(c.y), Mathf.Clamp01(c.z));

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
                    for (int k = 0; k < sides; k++)
                    {
                        float ang = 2f * Mathf.PI * k / sides;
                        Vector3 rd = Mathf.Cos(ang) * u + Mathf.Sin(ang) * v;
                        pos.Add(p[i] + rd * radius);
                        nrm.Add(rd);
                        col.Add(c);
                    }
                }
                Vector3 endT = prevT;

                // Side quads: ring k -> k+1 x segment i -> i+1, CCW-outward.
                for (int i = 0; i < n - 1; i++)
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
                int lastRing = ringBase + (n - 1) * sides;
                for (int k = 0; k < sides; k++) { pos.Add(pos[lastRing + k]); nrm.Add(endT); col.Add(c); }
                pos.Add(p[n - 1]); nrm.Add(endT); col.Add(c);
                for (int k = 0; k < sides; k++)
                { idx.Add(capBase + sides); idx.Add(capBase + k); idx.Add(capBase + (k + 1) % sides); }
            }
            return tubes;
        }
    }
}
