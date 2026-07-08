// Binary STL writer for the print export (3-2: mechanical extraction from
// TSDFPrintExporter.WriteMeshFiles — the exporter keeps bounds/winding/scale
// decisions and error handling; this only serialises).

using System;
using System.IO;
using UnityEngine;

namespace TSDF
{
    internal static class StlWriter
    {
        /// <summary>
        /// Write an indexed mesh as binary STL, recentred by <paramref name="center"/>
        /// and scaled to printed millimetres. Degenerate triangles (repeated indices —
        /// the 0.1 mm weld can collapse slivers) are skipped; <paramref name="written"/>
        /// must be their pre-counted number (it sizes the header count and the buffer).
        /// <paramref name="flip"/> reverses winding (caller decides via signed volume).
        /// Builds the whole STL in memory and writes it in ONE call: ~/Documents is
        /// TCC/sync-managed on macOS and millions of small BinaryWriter writes go
        /// through it catastrophically slowly.
        /// </summary>
        public static void Write(string path, Vector3[] pos, int[] tri, int written,
                                 Vector3 center, float scale, bool flip)
        {
            int triCount = tri.Length / 3;
            long stlBytes = 84L + 50L * written;
            var ms = new MemoryStream((int)stlBytes);
            using (var bw = new BinaryWriter(ms))
            {
                var header = new byte[80];
                var tag = System.Text.Encoding.ASCII.GetBytes("FloatingVectorsICC print export");
                Array.Copy(tag, header, Math.Min(tag.Length, 80));
                bw.Write(header);
                bw.Write((uint)written);
                for (int t = 0; t < triCount; t++)
                {
                    int i0 = tri[t * 3], i1 = tri[t * 3 + 1], i2 = tri[t * 3 + 2];
                    if (i0 == i1 || i1 == i2 || i0 == i2) continue;
                    if (flip) (i1, i2) = (i2, i1);
                    Vector3 a = (pos[i0] - center) * scale;
                    Vector3 b = (pos[i1] - center) * scale;
                    Vector3 c = (pos[i2] - center) * scale;
                    Vector3 nrm = Vector3.Cross(b - a, c - a);
                    float len = nrm.magnitude;
                    nrm = len > 1e-12f ? nrm / len : Vector3.up;
                    WriteV(bw, nrm); WriteV(bw, a); WriteV(bw, b); WriteV(bw, c);
                    bw.Write((ushort)0);
                }
                bw.Flush();
                using (var fs = File.Create(path))
                    fs.Write(ms.GetBuffer(), 0, (int)ms.Length);
            }
        }

        private static void WriteV(BinaryWriter bw, Vector3 v)
        {
            bw.Write(v.x); bw.Write(v.y); bw.Write(v.z);
        }
    }
}
