// Minimal Standard-3MF writer for multi-colour printing: ONE mesh object with
// per-triangle basematerials (face colouring) — the encoding Bambu Studio's
// "Standard 3MF Color Parsing" dialog reads and maps to AMS filaments
// (wiki.bambulab.com/en/bambu-studio/Standard-3MF-File-Color-Parsing).
//
// Coordinates follow StlWriter's convention exactly: (pos - center) * scale,
// millimetres, Unity axes as-is (no swap), CCW-outward winding — so the 3MF
// imports with the same orientation and size as the sibling STL.
//
// A .3mf is an OPC ZIP: [Content_Types].xml + _rels/.rels + 3D/3dmodel.model.
// Standard DEFLATE entries are fine here (unlike .usdz, which needs aligned
// STORED entries and is hand-rolled in UsdzWriter).

using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.IO.Compression;
using UnityEngine;

namespace TSDF
{
    internal static class ThreeMfWriter
    {
        /// <summary>A run of triangles sharing one material index (into the
        /// materials list). Spans must cover [0, triCount) in ascending order.</summary>
        public struct Span { public int StartTri, TriCount, Material; }

        private const string ContentTypes =
            "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n" +
            "<Types xmlns=\"http://schemas.openxmlformats.org/package/2006/content-types\">\n" +
            " <Default Extension=\"rels\" ContentType=\"application/vnd.openxmlformats-package.relationships+xml\"/>\n" +
            " <Default Extension=\"model\" ContentType=\"application/vnd.ms-package.3dmanufacturing-3dmodel+xml\"/>\n" +
            "</Types>\n";

        private const string Rels =
            "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n" +
            "<Relationships xmlns=\"http://schemas.openxmlformats.org/package/2006/relationships\">\n" +
            " <Relationship Target=\"/3D/3dmodel.model\" Id=\"rel-1\" Type=\"http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel\"/>\n" +
            "</Relationships>\n";

        public static void Write(string path, IReadOnlyList<Vector3> pos, IReadOnlyList<int> tri,
                                 IReadOnlyList<Span> spans, IReadOnlyList<(string name, Color color)> materials,
                                 Vector3 center, float scale)
        {
            var inv = CultureInfo.InvariantCulture;
            using var fs = File.Create(path);
            using var zip = new ZipArchive(fs, ZipArchiveMode.Create);

            WriteText(zip, "[Content_Types].xml", ContentTypes);
            WriteText(zip, "_rels/.rels", Rels);

            var entry = zip.CreateEntry("3D/3dmodel.model", System.IO.Compression.CompressionLevel.Optimal);
            using var w = new StreamWriter(entry.Open(), new System.Text.UTF8Encoding(false), 1 << 16);
            w.Write("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
            w.Write("<model unit=\"millimeter\" xml:lang=\"en-US\" xmlns=\"http://schemas.microsoft.com/3dmanufacturing/core/2015/02\">\n");
            w.Write(" <resources>\n");
            w.Write("  <basematerials id=\"1\">\n");
            foreach (var (name, color) in materials)
                w.Write($"   <base name=\"{name}\" displaycolor=\"#{Hex(color)}\"/>\n");
            w.Write("  </basematerials>\n");
            w.Write("  <object id=\"2\" type=\"model\" pid=\"1\" pindex=\"0\">\n   <mesh>\n    <vertices>\n");
            for (int i = 0; i < pos.Count; i++)
            {
                Vector3 v = (pos[i] - center) * scale;
                // 3 decimals of a millimetre = micron precision, compact XML
                w.Write("     <vertex x=\"");
                w.Write(v.x.ToString("0.###", inv));
                w.Write("\" y=\"");
                w.Write(v.y.ToString("0.###", inv));
                w.Write("\" z=\"");
                w.Write(v.z.ToString("0.###", inv));
                w.Write("\"/>\n");
            }
            w.Write("    </vertices>\n    <triangles>\n");
            foreach (var span in spans)
            {
                string p1 = span.Material.ToString(inv);
                for (int t = span.StartTri; t < span.StartTri + span.TriCount; t++)
                {
                    int b = t * 3;
                    w.Write("     <triangle v1=\"");
                    w.Write(tri[b].ToString(inv));
                    w.Write("\" v2=\"");
                    w.Write(tri[b + 1].ToString(inv));
                    w.Write("\" v3=\"");
                    w.Write(tri[b + 2].ToString(inv));
                    w.Write("\" p1=\"");
                    w.Write(p1);
                    w.Write("\"/>\n");
                }
            }
            w.Write("    </triangles>\n   </mesh>\n  </object>\n </resources>\n");
            w.Write(" <build>\n  <item objectid=\"2\"/>\n </build>\n</model>\n");
        }

        private static void WriteText(ZipArchive zip, string name, string content)
        {
            var entry = zip.CreateEntry(name, System.IO.Compression.CompressionLevel.Optimal);
            using var w = new StreamWriter(entry.Open(), new System.Text.UTF8Encoding(false));
            w.Write(content);
        }

        private static string Hex(Color c)
        {
            Color32 b = c;
            return $"{b.r:X2}{b.g:X2}{b.b:X2}FF";
        }
    }
}
