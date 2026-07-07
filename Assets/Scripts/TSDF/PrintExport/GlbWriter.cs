// Minimal binary glTF 2.0 (.glb) writer for the web export: one indexed
// primitive with POSITION / NORMAL / COLOR_0 and a rough non-metallic material
// that the vertex colours multiply into. No package dependency — the mesh is
// simple enough (one buffer, four accessors) that hand-writing the container
// stays smaller than pulling in glTFast just for export.
//
// Input contract (prepared by TSDFPrintExporter.WriteWebFiles): positions in
// metres, already right-handed Y-up with CCW-outward winding; colours are the
// welded per-vertex averages in sRGB 0-1 (same values the PLY writes).
// glTF's COLOR_0 is defined as LINEAR, so colours are linearised here before
// being packed as normalised RGBA8.
//
// Verified targets: three.js / <model-viewer> / Babylon.js / macOS Quick Look.

using System;
using System.Globalization;
using System.IO;
using System.Text;
using UnityEngine;

namespace TSDF
{
    internal static class GlbWriter
    {
        /// <summary>Writes the .glb and returns its size in bytes.</summary>
        public static long Write(string path, Vector3[] pos, Vector3[] nrm, Vector3[] col, int[] tri)
        {
            int vc = pos.Length;
            int posBytes = vc * 12, nrmBytes = vc * 12, colBytes = vc * 4, idxBytes = tri.Length * 4;
            int binLen = Pad4(posBytes) + Pad4(nrmBytes) + Pad4(colBytes) + Pad4(idxBytes);

            Vector3 min = Vector3.positiveInfinity, max = Vector3.negativeInfinity;
            foreach (var p in pos) { min = Vector3.Min(min, p); max = Vector3.Max(max, p); }

            // ---- BIN chunk: positions | normals | colours | indices ----
            var bin = new MemoryStream(binLen);
            var bw = new BinaryWriter(bin);
            foreach (var p in pos) { bw.Write(p.x); bw.Write(p.y); bw.Write(p.z); }
            PadTo4(bw);
            foreach (var n in nrm) { bw.Write(n.x); bw.Write(n.y); bw.Write(n.z); }
            PadTo4(bw);
            foreach (var c in col)
            {
                bw.Write(LinearByte(c.x)); bw.Write(LinearByte(c.y)); bw.Write(LinearByte(c.z));
                bw.Write((byte)255);
            }
            PadTo4(bw);
            foreach (var i in tri) bw.Write((uint)i);
            PadTo4(bw);
            bw.Flush();

            // ---- JSON chunk ----
            var inv = CultureInfo.InvariantCulture;
            int o0 = 0, o1 = Pad4(posBytes), o2 = o1 + Pad4(nrmBytes), o3 = o2 + Pad4(colBytes);
            string json =
                "{\"asset\":{\"version\":\"2.0\",\"generator\":\"FloatingVectorsICC print export\"}," +
                "\"scene\":0,\"scenes\":[{\"nodes\":[0]}]," +
                "\"nodes\":[{\"mesh\":0,\"name\":\"Sculpture\"}]," +
                "\"meshes\":[{\"primitives\":[{\"attributes\":{\"POSITION\":0,\"NORMAL\":1,\"COLOR_0\":2}," +
                "\"indices\":3,\"material\":0,\"mode\":4}]}]," +
                "\"materials\":[{\"name\":\"VertexColour\",\"pbrMetallicRoughness\":" +
                "{\"baseColorFactor\":[1,1,1,1],\"metallicFactor\":0,\"roughnessFactor\":0.9}}]," +
                "\"accessors\":[" +
                $"{{\"bufferView\":0,\"componentType\":5126,\"count\":{vc},\"type\":\"VEC3\"," +
                $"\"min\":[{F(min.x, inv)},{F(min.y, inv)},{F(min.z, inv)}]," +
                $"\"max\":[{F(max.x, inv)},{F(max.y, inv)},{F(max.z, inv)}]}}," +
                $"{{\"bufferView\":1,\"componentType\":5126,\"count\":{vc},\"type\":\"VEC3\"}}," +
                $"{{\"bufferView\":2,\"componentType\":5121,\"count\":{vc},\"type\":\"VEC4\",\"normalized\":true}}," +
                $"{{\"bufferView\":3,\"componentType\":5125,\"count\":{tri.Length},\"type\":\"SCALAR\"}}]," +
                "\"bufferViews\":[" +
                $"{{\"buffer\":0,\"byteOffset\":{o0},\"byteLength\":{posBytes},\"target\":34962}}," +
                $"{{\"buffer\":0,\"byteOffset\":{o1},\"byteLength\":{nrmBytes},\"target\":34962}}," +
                $"{{\"buffer\":0,\"byteOffset\":{o2},\"byteLength\":{colBytes},\"target\":34962}}," +
                $"{{\"buffer\":0,\"byteOffset\":{o3},\"byteLength\":{idxBytes},\"target\":34963}}]," +
                $"\"buffers\":[{{\"byteLength\":{binLen}}}]}}";
            var jsonBytes = Encoding.UTF8.GetBytes(json);
            int jsonPadded = Pad4(jsonBytes.Length); // JSON chunk pads with spaces

            // ---- GLB container (single write: ~/Documents is sync-managed) ----
            long total = 12 + 8 + jsonPadded + 8 + binLen;
            var outMs = new MemoryStream((int)total);
            var ow = new BinaryWriter(outMs);
            ow.Write(0x46546C67u);            // 'glTF'
            ow.Write(2u);
            ow.Write((uint)total);
            ow.Write((uint)jsonPadded);
            ow.Write(0x4E4F534Au);            // 'JSON'
            ow.Write(jsonBytes);
            for (int i = jsonBytes.Length; i < jsonPadded; i++) ow.Write((byte)' ');
            ow.Write((uint)binLen);
            ow.Write(0x004E4942u);            // 'BIN\0'
            ow.Write(bin.GetBuffer(), 0, (int)bin.Length);
            ow.Flush();
            using (var fs = File.Create(path))
                fs.Write(outMs.GetBuffer(), 0, (int)outMs.Length);
            return total;
        }

        // POSITION min/max must bound the stored floats EXACTLY (validators promote
        // the accessor floats to double and compare) — print the float's exact
        // double value, not a rounded decimal.
        private static string F(float v, CultureInfo inv) => ((double)v).ToString("R", inv);

        private static int Pad4(int n) => (n + 3) & ~3;

        private static void PadTo4(BinaryWriter bw)
        {
            while ((bw.BaseStream.Position & 3) != 0) bw.Write((byte)0);
        }

        // sRGB 0-1 -> linear, packed to a normalised byte (glTF COLOR_0 is linear).
        private static byte LinearByte(float srgb)
        {
            float s = Mathf.Clamp01(srgb);
            float lin = s <= 0.04045f ? s / 12.92f : Mathf.Pow((s + 0.055f) / 1.055f, 2.4f);
            return (byte)Mathf.Clamp(Mathf.RoundToInt(lin * 255f), 0, 255);
        }
    }
}
