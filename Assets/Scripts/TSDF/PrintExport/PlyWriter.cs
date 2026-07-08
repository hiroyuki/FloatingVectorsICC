// Binary little-endian PLY writer for the colour print export (3-2: mechanical
// extraction from TSDFPrintExporter — welded vertices with averaged colours +
// indexed faces, same recentre/scale/flip contract as the STL).

using System.IO;
using UnityEngine;

namespace TSDF
{
    internal static class PlyWriter
    {
        public static void Write(string path, Vector3[] pos, Vector3[] col, int[] tri,
                                 int faceCount, Vector3 center, float scale, bool flip)
        {
            // Same single-write strategy as the STL (sync-managed ~/Documents).
            var fs = new MemoryStream(pos.Length * 15 + faceCount * 13 + 512);
            string header = "ply\nformat binary_little_endian 1.0\n" +
                            $"element vertex {pos.Length}\n" +
                            "property float x\nproperty float y\nproperty float z\n" +
                            "property uchar red\nproperty uchar green\nproperty uchar blue\n" +
                            $"element face {faceCount}\n" +
                            "property list uchar int vertex_indices\nend_header\n";
            var hb = System.Text.Encoding.ASCII.GetBytes(header);
            fs.Write(hb, 0, hb.Length);
            using var bw = new BinaryWriter(fs);
            for (int i = 0; i < pos.Length; i++)
            {
                Vector3 w = (pos[i] - center) * scale;
                bw.Write(w.x); bw.Write(w.y); bw.Write(w.z);
                bw.Write((byte)Mathf.Clamp(Mathf.RoundToInt(col[i].x * 255f), 0, 255));
                bw.Write((byte)Mathf.Clamp(Mathf.RoundToInt(col[i].y * 255f), 0, 255));
                bw.Write((byte)Mathf.Clamp(Mathf.RoundToInt(col[i].z * 255f), 0, 255));
            }
            for (int t = 0; t < tri.Length; t += 3)
            {
                int i0 = tri[t], i1 = tri[t + 1], i2 = tri[t + 2];
                if (i0 == i1 || i1 == i2 || i0 == i2) continue;
                if (flip) (i1, i2) = (i2, i1);
                bw.Write((byte)3);
                bw.Write(i0); bw.Write(i1); bw.Write(i2);
            }
            bw.Flush();
            using var file = File.Create(path);
            file.Write(fs.GetBuffer(), 0, (int)fs.Length);
        }
    }
}
