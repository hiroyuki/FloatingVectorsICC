// OBJ + MTL writer with per-triangle-range MATERIALS (usemtl groups) — the
// portable way to carry the two-colour print split (body/floor vs curves)
// into previewers (MeshLab, Blender, Windows 3D Viewer) and into Bambu
// Studio's colored-OBJ import, which maps materials to AMS filaments.
// Vertex colours were rejected: the xyzrgb OBJ extension is a dialect with
// spotty support; materials are first-class OBJ.
//
// Coordinates follow StlWriter's convention exactly: (pos - center) * scale,
// millimetres, Unity axes as-is, CCW-outward winding — same orientation and
// size as the sibling STL/3MF.

using System.Collections.Generic;
using System.Globalization;
using System.IO;
using UnityEngine;

namespace TSDF
{
    internal static class ObjMaterialWriter
    {
        /// <summary>Writes path (.obj) plus a sibling .mtl with the same stem.
        /// Spans reuse ThreeMfWriter.Span (triangle ranges → material index).</summary>
        public static void Write(string path, IReadOnlyList<Vector3> pos, IReadOnlyList<int> tri,
                                 IReadOnlyList<ThreeMfWriter.Span> spans,
                                 IReadOnlyList<(string name, Color color)> materials,
                                 Vector3 center, float scale)
        {
            var inv = CultureInfo.InvariantCulture;
            string mtlPath = Path.ChangeExtension(path, ".mtl");
            string mtlName = Path.GetFileName(mtlPath);

            using (var w = new StreamWriter(mtlPath, false, new System.Text.UTF8Encoding(false)))
            {
                foreach (var (name, color) in materials)
                {
                    w.Write("newmtl ");
                    w.Write(name);
                    w.Write("\nKd ");
                    w.Write(color.r.ToString("0.###", inv));
                    w.Write(' ');
                    w.Write(color.g.ToString("0.###", inv));
                    w.Write(' ');
                    w.Write(color.b.ToString("0.###", inv));
                    // mild specular so curvature reads in viewers (flat Ks 0
                    // made the smoothed body look like unshaded flat colour)
                    w.Write("\nKa 0 0 0\nKs 0.25 0.25 0.25\nNs 32\nd 1\nillum 2\n\n");
                }
            }

            // Smooth vertex normals (area-weighted face-normal accumulation).
            // Snapshot/tube normals are dropped upstream and viewers left to
            // guess produced flat, unshaded bodies — write vn explicitly.
            var nrm = new Vector3[pos.Count];
            for (int t = 0; t + 2 < tri.Count; t += 3)
            {
                Vector3 a = pos[tri[t]], b = pos[tri[t + 1]], c = pos[tri[t + 2]];
                Vector3 fn = Vector3.Cross(b - a, c - a); // length = 2*area (weighting)
                nrm[tri[t]] += fn; nrm[tri[t + 1]] += fn; nrm[tri[t + 2]] += fn;
            }
            for (int i = 0; i < nrm.Length; i++)
                nrm[i] = nrm[i].sqrMagnitude > 1e-20f ? nrm[i].normalized : Vector3.up;

            using (var w = new StreamWriter(path, false, new System.Text.UTF8Encoding(false), 1 << 16))
            {
                w.Write("# FloatingVectorsICC print export\nmtllib ");
                w.Write(mtlName);
                w.Write('\n');
                for (int i = 0; i < pos.Count; i++)
                {
                    Vector3 v = (pos[i] - center) * scale;
                    w.Write("v ");
                    w.Write(v.x.ToString("0.###", inv));
                    w.Write(' ');
                    w.Write(v.y.ToString("0.###", inv));
                    w.Write(' ');
                    w.Write(v.z.ToString("0.###", inv));
                    w.Write('\n');
                }
                for (int i = 0; i < nrm.Length; i++)
                {
                    w.Write("vn ");
                    w.Write(nrm[i].x.ToString("0.###", inv));
                    w.Write(' ');
                    w.Write(nrm[i].y.ToString("0.###", inv));
                    w.Write(' ');
                    w.Write(nrm[i].z.ToString("0.###", inv));
                    w.Write('\n');
                }
                foreach (var span in spans)
                {
                    w.Write("usemtl ");
                    w.Write(materials[span.Material].name);
                    w.Write('\n');
                    for (int t = span.StartTri; t < span.StartTri + span.TriCount; t++)
                    {
                        int b = t * 3;
                        string i0 = (tri[b] + 1).ToString(inv);
                        string i1 = (tri[b + 1] + 1).ToString(inv);
                        string i2 = (tri[b + 2] + 1).ToString(inv);
                        w.Write("f ");
                        w.Write(i0); w.Write("//"); w.Write(i0);
                        w.Write(' ');
                        w.Write(i1); w.Write("//"); w.Write(i1);
                        w.Write(' ');
                        w.Write(i2); w.Write("//"); w.Write(i2);
                        w.Write('\n');
                    }
                }
            }
        }
    }
}
