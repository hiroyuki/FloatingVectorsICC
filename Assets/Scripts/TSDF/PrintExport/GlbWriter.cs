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
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Text;
using UnityEngine;

namespace TSDF
{
    internal static class GlbWriter
    {
        /// <summary>Writes the .glb and returns its size in bytes. When
        /// <paramref name="pcPos"/> is non-empty, a second primitive of GL_POINTS
        /// (mode 0, POSITION + COLOR_0, no indices) is emitted in the same mesh —
        /// the decimated point cloud rendered as points alongside the curve/mesh
        /// triangles. Point colours follow the same sRGB→linear packing.</summary>
        public static long Write(string path, Vector3[] pos, Vector3[] nrm, Vector3[] col, int[] tri,
                                 Vector3[] pcPos = null, Vector3[] pcCol = null)
        {
            int vc = pos.Length;
            int pcc = pcPos != null ? pcPos.Length : 0;
            bool hasPc = pcc > 0;
            int posBytes = vc * 12, nrmBytes = vc * 12, colBytes = vc * 4, idxBytes = tri.Length * 4;
            int pcPosBytes = pcc * 12, pcColBytes = pcc * 4;
            int binLen = Pad4(posBytes) + Pad4(nrmBytes) + Pad4(colBytes) + Pad4(idxBytes);
            if (hasPc) binLen += Pad4(pcPosBytes) + Pad4(pcColBytes);

            Vector3 min = Vector3.positiveInfinity, max = Vector3.negativeInfinity;
            foreach (var p in pos) { min = Vector3.Min(min, p); max = Vector3.Max(max, p); }
            Vector3 pcMin = Vector3.positiveInfinity, pcMax = Vector3.negativeInfinity;
            if (hasPc) foreach (var p in pcPos) { pcMin = Vector3.Min(pcMin, p); pcMax = Vector3.Max(pcMax, p); }

            // ---- BIN chunk: positions | normals | colours | indices [| pc pos | pc col] ----
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
            if (hasPc)
            {
                foreach (var p in pcPos) { bw.Write(p.x); bw.Write(p.y); bw.Write(p.z); }
                PadTo4(bw);
                foreach (var c in pcCol)
                {
                    bw.Write(LinearByte(c.x)); bw.Write(LinearByte(c.y)); bw.Write(LinearByte(c.z));
                    bw.Write((byte)255);
                }
                PadTo4(bw);
            }
            bw.Flush();

            // ---- JSON chunk ----
            // Built dynamically so a points-only GLB (no mesh/curve triangles, vc==0)
            // stays valid: the triangle primitive + its accessors/bufferViews are
            // emitted only when there is mesh geometry, otherwise the point cloud is
            // the sole primitive. An empty-mesh triangle accessor would carry the
            // Vector3.positiveInfinity min/max and fail validators.
            var inv = CultureInfo.InvariantCulture;
            bool hasMesh = vc > 0;
            int o0 = 0, o1 = Pad4(posBytes), o2 = o1 + Pad4(nrmBytes), o3 = o2 + Pad4(colBytes);
            int o4 = o3 + Pad4(idxBytes), o5 = o4 + Pad4(pcPosBytes);
            var accessors = new List<string>();
            var bufferViews = new List<string>();
            var primitives = new List<string>();
            if (hasMesh)
            {
                accessors.Add($"{{\"bufferView\":{bufferViews.Count},\"componentType\":5126,\"count\":{vc}," +
                    $"\"type\":\"VEC3\",\"min\":[{F(min.x, inv)},{F(min.y, inv)},{F(min.z, inv)}]," +
                    $"\"max\":[{F(max.x, inv)},{F(max.y, inv)},{F(max.z, inv)}]}}");
                bufferViews.Add($"{{\"buffer\":0,\"byteOffset\":{o0},\"byteLength\":{posBytes},\"target\":34962}}");
                int posA = accessors.Count - 1;
                accessors.Add($"{{\"bufferView\":{bufferViews.Count},\"componentType\":5126,\"count\":{vc},\"type\":\"VEC3\"}}");
                bufferViews.Add($"{{\"buffer\":0,\"byteOffset\":{o1},\"byteLength\":{nrmBytes},\"target\":34962}}");
                int nrmA = accessors.Count - 1;
                accessors.Add($"{{\"bufferView\":{bufferViews.Count},\"componentType\":5121,\"count\":{vc},\"type\":\"VEC4\",\"normalized\":true}}");
                bufferViews.Add($"{{\"buffer\":0,\"byteOffset\":{o2},\"byteLength\":{colBytes},\"target\":34962}}");
                int colA = accessors.Count - 1;
                accessors.Add($"{{\"bufferView\":{bufferViews.Count},\"componentType\":5125,\"count\":{tri.Length},\"type\":\"SCALAR\"}}");
                bufferViews.Add($"{{\"buffer\":0,\"byteOffset\":{o3},\"byteLength\":{idxBytes},\"target\":34963}}");
                int idxA = accessors.Count - 1;
                primitives.Add($"{{\"attributes\":{{\"POSITION\":{posA},\"NORMAL\":{nrmA},\"COLOR_0\":{colA}}}," +
                    $"\"indices\":{idxA},\"material\":0,\"mode\":4}}");
            }
            if (hasPc)
            {
                accessors.Add($"{{\"bufferView\":{bufferViews.Count},\"componentType\":5126,\"count\":{pcc}," +
                    $"\"type\":\"VEC3\",\"min\":[{F(pcMin.x, inv)},{F(pcMin.y, inv)},{F(pcMin.z, inv)}]," +
                    $"\"max\":[{F(pcMax.x, inv)},{F(pcMax.y, inv)},{F(pcMax.z, inv)}]}}");
                bufferViews.Add($"{{\"buffer\":0,\"byteOffset\":{o4},\"byteLength\":{pcPosBytes},\"target\":34962}}");
                int pcPosA = accessors.Count - 1;
                accessors.Add($"{{\"bufferView\":{bufferViews.Count},\"componentType\":5121,\"count\":{pcc},\"type\":\"VEC4\",\"normalized\":true}}");
                bufferViews.Add($"{{\"buffer\":0,\"byteOffset\":{o5},\"byteLength\":{pcColBytes},\"target\":34962}}");
                int pcColA = accessors.Count - 1;
                primitives.Add($"{{\"attributes\":{{\"POSITION\":{pcPosA},\"COLOR_0\":{pcColA}}},\"material\":0,\"mode\":0}}");
            }
            string json =
                "{\"asset\":{\"version\":\"2.0\",\"generator\":\"FloatingVectorsICC print export\"}," +
                "\"scene\":0,\"scenes\":[{\"nodes\":[0]}]," +
                "\"nodes\":[{\"mesh\":0,\"name\":\"Sculpture\"}]," +
                "\"meshes\":[{\"primitives\":[" + string.Join(",", primitives) + "]}]," +
                "\"materials\":[{\"name\":\"VertexColour\",\"pbrMetallicRoughness\":" +
                "{\"baseColorFactor\":[1,1,1,1],\"metallicFactor\":0,\"roughnessFactor\":0.9}}]," +
                "\"accessors\":[" + string.Join(",", accessors) + "]," +
                "\"bufferViews\":[" + string.Join(",", bufferViews) + "]," +
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
