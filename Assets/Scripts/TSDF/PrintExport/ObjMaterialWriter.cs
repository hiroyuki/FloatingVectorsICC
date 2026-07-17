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
                    w.Write("\nKa 0 0 0\nKs 0 0 0\nd 1\nillum 1\n\n");
                }
            }

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
                foreach (var span in spans)
                {
                    w.Write("usemtl ");
                    w.Write(materials[span.Material].name);
                    w.Write('\n');
                    for (int t = span.StartTri; t < span.StartTri + span.TriCount; t++)
                    {
                        int b = t * 3;
                        w.Write("f ");
                        w.Write((tri[b] + 1).ToString(inv));
                        w.Write(' ');
                        w.Write((tri[b + 1] + 1).ToString(inv));
                        w.Write(' ');
                        w.Write((tri[b + 2] + 1).ToString(inv));
                        w.Write('\n');
                    }
                }
            }
        }
    }
}
