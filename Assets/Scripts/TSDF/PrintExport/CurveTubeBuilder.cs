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
        // teardrop: droplet cross-section instead of a circle — lower 3/4 arc of
        // the circle plus a 90-degree apex (height sqrt(2)*r, tangent to the
        // circle at +-45 degrees) pointing WORLD UP on every ring, so overhangs
        // self-support in FDM and the tubes read as falling drops. Needs
        // sides >= 4; the ring frame comes from projecting world up into the
        // ring plane (near-vertical segments reuse the previous ring's frame).
        public static int AppendCurveTubes(List<Vector3[]> lines, List<Vector3> lineCols, float brightness,
                                           float radius, int sides, float tolerance, Vector3 center, float minY,
                                           List<Vector3> pos, List<Vector3> nrm, List<Vector3> col,
                                           List<int> idx, float tipTaper = 1f, bool exportSpace = true,
                                           bool teardrop = false)
        {
            int tubes = 0;
            var p = new List<Vector3>(256);
            Vector2[] prof = null;
            if (teardrop && sides >= 4)
            {
                prof = new Vector2[sides];
                prof[0] = new Vector2(0f, 1.41421356f); // apex
                for (int k = 1; k < sides; k++)
                {
                    // CCW in the (u,v) ring plane: apex -> left tangent point
                    // (135 deg) -> bottom -> right tangent point (45 deg)
                    float phi = Mathf.Deg2Rad * (45f + 270f * (k - 1) / (sides - 2));
                    prof[k] = new Vector2(-Mathf.Sin(phi), Mathf.Cos(phi));
                }
            }
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

                int ringBase = pos.Count;
                Vector3 startT = (p[1] - p[0]).normalized;
                Vector3 u = Vector3.Cross(startT, Mathf.Abs(startT.y) < 0.9f ? Vector3.up : Vector3.right).normalized;
                Vector3 prevT = startT;
                Vector3 dPrev = u; // droplet-up fallback for near-vertical segments
                for (int i = 0; i < n; i++)
                {
                    Vector3 t = p[Mathf.Min(i + 1, n - 1)] - p[Mathf.Max(i - 1, 0)];
                    t = t.sqrMagnitude > 1e-12f ? t.normalized : prevT;
                    u = (Quaternion.FromToRotation(prevT, t) * u).normalized;
                    prevT = t;
                    float r = radius * Mathf.Lerp(Mathf.Clamp01(tipTaper), 1f, n > 1 ? (float)i / (n - 1) : 1f);
                    if (prof != null)
                    {
                        Vector3 d = Vector3.up - t * t.y; // world up in the ring plane
                        d = d.sqrMagnitude > 1e-6f ? d.normalized : dPrev;
                        dPrev = d;
                        Vector3 uA = Vector3.Cross(d, t); // so that d == cross(t, uA)
                        for (int k = 0; k < sides; k++)
                        {
                            Vector3 rd = prof[k].x * uA + prof[k].y * d;
                            pos.Add(p[i] + rd * r);
                            nrm.Add(rd.normalized);
                            col.Add(c);
                        }
                    }
                    else
                    {
                        Vector3 v = Vector3.Cross(t, u);
                        for (int k = 0; k < sides; k++)
                        {
                            float ang = 2f * Mathf.PI * k / sides;
                            Vector3 rd = Mathf.Cos(ang) * u + Mathf.Sin(ang) * v;
                            pos.Add(p[i] + rd * r);
                            nrm.Add(rd);
                            col.Add(c);
                        }
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
