// .usdz writer for iPhone AR Quick Look. No package dependency: a .usdz is a
// ZIP archive with STORED (uncompressed) entries whose file data starts on
// 64-byte boundaries, the root .usda layer first — same approach as the
// three.js USDZExporter, hand-rolled here because System.IO.Compression can't
// control entry alignment.
//
// Colour: AR Quick Look does NOT render vertex colours / the displayColor
// primvar (unsupported in RealityKit, appears pink/white —
// developer.apple.com/forums/thread/691674). So the welded per-vertex colours
// are baked into a texture atlas: each triangle owns a 2x2-texel block filled
// with its average corner colour, and all three faceVarying st corners point
// at the block centre. Flat colour per triangle — invisible at the export's
// triangle density, and immune to bilinear bleed (0.5 texel of margin).
//
// Input contract (TSDFPrintExporter.WriteWebFiles): positions in metres,
// right-handed Y-up, base at y=0, CCW-outward winding; colours sRGB 0-1.
//
// The USD payload is authored as ASCII usda, then converted to binary usdc
// (3-4x smaller) through an external python with Pixar's usd-core when one is
// available — see TryConvertToUsdc. Fallback: the usda goes into the zip
// directly (usdz permits either as the root layer).

using System;
using System.Globalization;
using System.IO;
using System.Text;
using UnityEngine;

namespace TSDF
{
    internal static class UsdzWriter
    {
        public struct Result
        {
            public long bytes;       // final .usdz size
            public int texSize;      // baked colour-atlas resolution (px)
            public bool binaryUsdc;  // payload converted to binary usdc (vs ASCII usda fallback)
            public long convertMs;   // usda -> usdc conversion time (0 when fallback)
        }

        /// <summary>Writes the .usdz. The USD payload is converted to binary usdc
        /// (3-4x smaller: crate compresses the index arrays) when a python with
        /// the pxr module (pip package usd-core) is found — see PythonCandidates;
        /// otherwise the ASCII usda goes in as-is.</summary>
        public static Result Write(string path, Vector3[] pos, Vector3[] nrm, Vector3[] col, int[] tri,
                                   string usdPythonOverride)
        {
            int triCount = tri.Length / 3;
            int texSize = 64;
            while (texSize < 4096 && (long)(texSize / 2) * (texSize / 2) < triCount) texSize *= 2;
            if ((long)(texSize / 2) * (texSize / 2) < triCount)
                throw new InvalidOperationException(
                    $"{triCount} triangles exceed the 4096px colour atlas (max {2048L * 2048L}) — " +
                    "raise Web Curve Stride or shrink the volume");

            byte[] png = BakeAtlasPng(pos, col, tri, triCount, texSize);
            byte[] usda = BuildUsda(pos, nrm, col, tri, triCount, texSize);

            var r = new Result { texSize = texSize };
            var sw = System.Diagnostics.Stopwatch.StartNew();
            byte[] usdc = TryConvertToUsdc(usda, usdPythonOverride);
            byte[] payload = usda;
            string payloadName = "model.usda";
            if (usdc != null)
            {
                payload = usdc;
                payloadName = "model.usdc";
                r.binaryUsdc = true;
                r.convertMs = sw.ElapsedMilliseconds;
            }

            var zip = new MemoryStream(payload.Length + png.Length + 4096);
            WriteStoredZip(zip, new[] { (payloadName, payload), ("texture0.png", png) });
            using (var fs = File.Create(path))
                fs.Write(zip.GetBuffer(), 0, (int)zip.Length);
            r.bytes = zip.Length;
            return r;
        }

        // ---- usda -> usdc via an external python with usd-core ----
        // Hand-writing the crate format is not realistic, and the Unity USD
        // package is a heavyweight native dependency — so the conversion shells
        // out to Pixar's own code. Soft dependency: no python/pxr found means the
        // usdz simply ships the ASCII usda (bigger, still valid).
        //
        // Setup (one-time, both OSes):
        //   macOS:   python3 -m venv ~/.venvs/usd && ~/.venvs/usd/bin/pip install usd-core
        //   Windows: py -3 -m venv %USERPROFILE%\.venvs\usd
        //            %USERPROFILE%\.venvs\usd\Scripts\pip install usd-core

        private static string s_python; // cached working python; "" = probed, none found

        private static byte[] TryConvertToUsdc(byte[] usda, string overridePath)
        {
            string tmp = Path.Combine(Path.GetTempPath(), "fv_usdz_" + Guid.NewGuid().ToString("N"));
            try
            {
                Directory.CreateDirectory(tmp);
                string inPath = Path.Combine(tmp, "model.usda");
                string outPath = Path.Combine(tmp, "model.usdc");
                string script = Path.Combine(tmp, "convert.py");
                File.WriteAllBytes(inPath, usda);
                File.WriteAllText(script,
                    "import sys\n" +
                    "from pxr import Sdf\n" +
                    "ok = Sdf.Layer.FindOrOpen(sys.argv[1]).Export(sys.argv[2])\n" +
                    "sys.exit(0 if ok else 1)\n");
                foreach (string py in PythonCandidates(overridePath))
                {
                    if (s_python != null && s_python.Length > 0 && s_python != py) continue;
                    if (s_python != null && s_python.Length == 0 && string.IsNullOrWhiteSpace(overridePath))
                        break; // auto-detect already failed this session
                    if (RunPython(py, script, inPath, outPath) && File.Exists(outPath))
                    {
                        s_python = py;
                        return File.ReadAllBytes(outPath);
                    }
                }
                if (string.IsNullOrWhiteSpace(overridePath)) s_python = "";
                return null;
            }
            catch
            {
                return null;
            }
            finally
            {
                try { Directory.Delete(tmp, true); } catch { }
            }
        }

        private static System.Collections.Generic.IEnumerable<string> PythonCandidates(string overridePath)
        {
            if (!string.IsNullOrWhiteSpace(overridePath))
            {
                yield return overridePath.Trim();
                yield break;
            }
            string home = Environment.GetFolderPath(Environment.SpecialFolder.UserProfile);
            bool windows = Application.platform == RuntimePlatform.WindowsEditor ||
                           Application.platform == RuntimePlatform.WindowsPlayer;
            if (windows)
            {
                yield return Path.Combine(home, ".venvs", "usd", "Scripts", "python.exe");
                yield return "python";
                yield return "py";
            }
            else
            {
                yield return Path.Combine(home, ".venvs", "usd", "bin", "python3");
                yield return "/opt/homebrew/bin/python3";
                yield return "/usr/local/bin/python3";
                yield return "/usr/bin/python3";
            }
        }

        private static bool RunPython(string exe, string script, string inPath, string outPath)
        {
            try
            {
                var psi = new System.Diagnostics.ProcessStartInfo
                {
                    FileName = exe,
                    Arguments = $"\"{script}\" \"{inPath}\" \"{outPath}\"",
                    UseShellExecute = false,
                    CreateNoWindow = true,
                    RedirectStandardOutput = true,
                    RedirectStandardError = true,
                };
                using var proc = System.Diagnostics.Process.Start(psi);
                if (proc == null) return false;
                if (!proc.WaitForExit(120_000))
                {
                    try { proc.Kill(); } catch { }
                    return false;
                }
                return proc.ExitCode == 0;
            }
            catch
            {
                return false;
            }
        }

        // ---- colour atlas: one 2x2 block per triangle, average corner colour ----
        private static byte[] BakeAtlasPng(Vector3[] pos, Vector3[] col, int[] tri, int triCount, int texSize)
        {
            int blocksPerRow = texSize / 2;
            var pixels = new Color32[texSize * texSize];
            var grey = new Color32(128, 128, 128, 255);
            for (int i = 0; i < pixels.Length; i++) pixels[i] = grey;
            for (int t = 0; t < triCount; t++)
            {
                Vector3 c = (col[tri[t * 3]] + col[tri[t * 3 + 1]] + col[tri[t * 3 + 2]]) / 3f;
                var c32 = new Color32(
                    (byte)Mathf.Clamp(Mathf.RoundToInt(c.x * 255f), 0, 255),
                    (byte)Mathf.Clamp(Mathf.RoundToInt(c.y * 255f), 0, 255),
                    (byte)Mathf.Clamp(Mathf.RoundToInt(c.z * 255f), 0, 255), 255);
                int bx = (t % blocksPerRow) * 2, by = (t / blocksPerRow) * 2;
                int row0 = by * texSize + bx;
                pixels[row0] = c32; pixels[row0 + 1] = c32;
                pixels[row0 + texSize] = c32; pixels[row0 + texSize + 1] = c32;
            }
            var tex = new Texture2D(texSize, texSize, TextureFormat.RGBA32, false);
            try
            {
                tex.SetPixels32(pixels);
                return tex.EncodeToPNG();
            }
            finally
            {
                UnityEngine.Object.DestroyImmediate(tex);
            }
        }

        // ---- root layer (ASCII usda; usdz allows usda as the default layer) ----
        private static byte[] BuildUsda(Vector3[] pos, Vector3[] nrm, Vector3[] col, int[] tri,
                                        int triCount, int texSize)
        {
            var inv = CultureInfo.InvariantCulture;
            int blocksPerRow = texSize / 2;
            Vector3 min = Vector3.positiveInfinity, max = Vector3.negativeInfinity;
            foreach (var p in pos) { min = Vector3.Min(min, p); max = Vector3.Max(max, p); }

            // Streamed, not StringBuilder: the arrays run to tens of MB of text.
            var ms = new MemoryStream(pos.Length * 60 + tri.Length * 30 + 4096);
            var w = new StreamWriter(ms, new UTF8Encoding(false), 1 << 16);

            w.Write("#usda 1.0\n(\n    defaultPrim = \"Sculpture\"\n" +
                    "    metersPerUnit = 1\n    upAxis = \"Y\"\n)\n\n" +
                    "def Xform \"Sculpture\" (\n    kind = \"component\"\n)\n{\n" +
                    "    def Mesh \"Geom\" (\n        prepend apiSchemas = [\"MaterialBindingAPI\"]\n    )\n    {\n" +
                    "        uniform bool doubleSided = 0\n");

            w.Write($"        float3[] extent = [({F4(min.x, inv)}, {F4(min.y, inv)}, {F4(min.z, inv)}), " +
                    $"({F4(max.x, inv)}, {F4(max.y, inv)}, {F4(max.z, inv)})]\n");

            w.Write("        int[] faceVertexCounts = [");
            for (int t = 0; t < triCount; t++) { if (t > 0) w.Write(", "); w.Write('3'); }
            w.Write("]\n");

            w.Write("        int[] faceVertexIndices = [");
            for (int i = 0; i < tri.Length; i++)
            {
                if (i > 0) w.Write(", ");
                w.Write(tri[i].ToString(inv));
            }
            w.Write("]\n");

            w.Write("        point3f[] points = [");
            for (int i = 0; i < pos.Length; i++)
            {
                if (i > 0) w.Write(", ");
                w.Write($"({F4(pos[i].x, inv)}, {F4(pos[i].y, inv)}, {F4(pos[i].z, inv)})");
            }
            w.Write("]\n");

            w.Write("        normal3f[] normals = [");
            for (int i = 0; i < nrm.Length; i++)
            {
                if (i > 0) w.Write(", ");
                w.Write($"({F3(nrm[i].x, inv)}, {F3(nrm[i].y, inv)}, {F3(nrm[i].z, inv)})");
            }
            w.Write("] (\n            interpolation = \"vertex\"\n        )\n");

            // faceVarying st: three identical corners per triangle, pointing at
            // that triangle's 2x2 block centre.
            w.Write("        texCoord2f[] primvars:st = [");
            for (int t = 0; t < triCount; t++)
            {
                float u = ((t % blocksPerRow) * 2 + 1) / (float)texSize;
                float v = ((t / blocksPerRow) * 2 + 1) / (float)texSize;
                string st = $"({F5(u, inv)}, {F5(v, inv)})";
                if (t > 0) w.Write(", ");
                w.Write(st); w.Write(", "); w.Write(st); w.Write(", "); w.Write(st);
            }
            w.Write("] (\n            interpolation = \"faceVarying\"\n        )\n");

            w.Write("        uniform token subdivisionScheme = \"none\"\n" +
                    "        rel material:binding = </Sculpture/Materials/Mat>\n    }\n\n" +
                    "    def Scope \"Materials\"\n    {\n        def Material \"Mat\"\n        {\n" +
                    "            token outputs:surface.connect = </Sculpture/Materials/Mat/PBRShader.outputs:surface>\n\n" +
                    "            def Shader \"PBRShader\"\n            {\n" +
                    "                uniform token info:id = \"UsdPreviewSurface\"\n" +
                    "                color3f inputs:diffuseColor.connect = </Sculpture/Materials/Mat/DiffuseTex.outputs:rgb>\n" +
                    "                float inputs:metallic = 0\n" +
                    "                float inputs:roughness = 0.9\n" +
                    "                token outputs:surface\n            }\n\n" +
                    "            def Shader \"StReader\"\n            {\n" +
                    "                uniform token info:id = \"UsdPrimvarReader_float2\"\n" +
                    "                string inputs:varname = \"st\"\n" +
                    "                float2 outputs:result\n            }\n\n" +
                    "            def Shader \"DiffuseTex\"\n            {\n" +
                    "                uniform token info:id = \"UsdUVTexture\"\n" +
                    "                asset inputs:file = @texture0.png@\n" +
                    "                float2 inputs:st.connect = </Sculpture/Materials/Mat/StReader.outputs:result>\n" +
                    "                token inputs:sourceColorSpace = \"sRGB\"\n" +
                    "                token inputs:wrapS = \"clamp\"\n" +
                    "                token inputs:wrapT = \"clamp\"\n" +
                    "                float3 outputs:rgb\n            }\n        }\n    }\n}\n");

            w.Flush();
            return ms.ToArray();
        }

        private static string F3(float v, CultureInfo inv) => v.ToString("0.###", inv);
        private static string F4(float v, CultureInfo inv) => v.ToString("0.####", inv);
        private static string F5(float v, CultureInfo inv) => v.ToString("0.#####", inv);

        // ---- STORED zip with 64-byte-aligned entry data (usdz packaging rule) ----
        private static void WriteStoredZip(MemoryStream outMs, (string name, byte[] data)[] entries)
        {
            var bw = new BinaryWriter(outMs);
            var now = DateTime.Now;
            ushort dosTime = (ushort)((now.Hour << 11) | (now.Minute << 5) | (now.Second / 2));
            ushort dosDate = (ushort)(((now.Year - 1980) << 9) | (now.Month << 5) | now.Day);

            var offsets = new long[entries.Length];
            var crcs = new uint[entries.Length];
            for (int e = 0; e < entries.Length; e++)
            {
                var (name, data) = entries[e];
                var nameBytes = Encoding.ASCII.GetBytes(name);
                offsets[e] = outMs.Position;
                crcs[e] = Crc32(data);

                // Pad the local header's extra field so the data starts on a
                // 64-byte boundary (extra field = id + size + zero fill, min 4B).
                long dataStart = outMs.Position + 30 + nameBytes.Length;
                int pad = (int)((64 - dataStart % 64) % 64);
                if (pad > 0 && pad < 4) pad += 64;

                bw.Write(0x04034b50u);             // local file header
                bw.Write((ushort)20);              // version needed
                bw.Write((ushort)0);               // flags
                bw.Write((ushort)0);               // method: stored
                bw.Write(dosTime); bw.Write(dosDate);
                bw.Write(crcs[e]);
                bw.Write((uint)data.Length);       // compressed
                bw.Write((uint)data.Length);       // uncompressed
                bw.Write((ushort)nameBytes.Length);
                bw.Write((ushort)pad);
                bw.Write(nameBytes);
                if (pad > 0)
                {
                    bw.Write((ushort)0x1986);      // arbitrary private extra-field id
                    bw.Write((ushort)(pad - 4));
                    for (int i = 4; i < pad; i++) bw.Write((byte)0);
                }
                bw.Write(data);
            }

            long cdStart = outMs.Position;
            for (int e = 0; e < entries.Length; e++)
            {
                var (name, data) = entries[e];
                var nameBytes = Encoding.ASCII.GetBytes(name);
                bw.Write(0x02014b50u);             // central directory header
                bw.Write((ushort)20);              // version made by
                bw.Write((ushort)20);              // version needed
                bw.Write((ushort)0);               // flags
                bw.Write((ushort)0);               // method
                bw.Write(dosTime); bw.Write(dosDate);
                bw.Write(crcs[e]);
                bw.Write((uint)data.Length);
                bw.Write((uint)data.Length);
                bw.Write((ushort)nameBytes.Length);
                bw.Write((ushort)0);               // extra len
                bw.Write((ushort)0);               // comment len
                bw.Write((ushort)0);               // disk number
                bw.Write((ushort)0);               // internal attrs
                bw.Write(0u);                      // external attrs
                bw.Write((uint)offsets[e]);
                bw.Write(nameBytes);
            }
            long cdSize = outMs.Position - cdStart;

            bw.Write(0x06054b50u);                 // end of central directory
            bw.Write((ushort)0); bw.Write((ushort)0);
            bw.Write((ushort)entries.Length); bw.Write((ushort)entries.Length);
            bw.Write((uint)cdSize);
            bw.Write((uint)cdStart);
            bw.Write((ushort)0);
            bw.Flush();
        }

        private static uint[] _crcTable;

        private static uint Crc32(byte[] data)
        {
            if (_crcTable == null)
            {
                _crcTable = new uint[256];
                for (uint i = 0; i < 256; i++)
                {
                    uint c = i;
                    for (int k = 0; k < 8; k++)
                        c = (c & 1) != 0 ? 0xEDB88320u ^ (c >> 1) : c >> 1;
                    _crcTable[i] = c;
                }
            }
            uint crc = 0xFFFFFFFFu;
            foreach (byte b in data)
                crc = _crcTable[(crc ^ b) & 0xFF] ^ (crc >> 8);
            return crc ^ 0xFFFFFFFFu;
        }
    }
}
