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
            // RuntimeInformation, not Application.platform: this runs on the
            // export worker thread, where Unity's Application API is off-limits.
            bool windows = System.Runtime.InteropServices.RuntimeInformation.IsOSPlatform(
                System.Runtime.InteropServices.OSPlatform.Windows);
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
            return EncodePng(pixels, texSize);
        }

        // Pure-C# PNG encode (RGBA8) so the whole usdz write can run on a worker
        // thread — Texture2D.EncodeToPNG is main-thread-only and was the last
        // Unity-API dependency in this path. Matches EncodeToPNG's orientation:
        // pixels[] is bottom-up (SetPixels32 layout), PNG scanlines are top-down.
        private static byte[] EncodePng(Color32[] pixels, int size)
        {
            var raw = new byte[size * (size * 4 + 1)];
            int o = 0;
            for (int y = size - 1; y >= 0; y--)
            {
                raw[o++] = 0; // per-scanline filter: None
                int row = y * size;
                for (int x = 0; x < size; x++)
                {
                    var p = pixels[row + x];
                    raw[o++] = p.r; raw[o++] = p.g; raw[o++] = p.b; raw[o++] = p.a;
                }
            }

            byte[] deflated;
            using (var ms = new MemoryStream())
            {
                using (var ds = new System.IO.Compression.DeflateStream(
                           ms, System.IO.Compression.CompressionLevel.Fastest, leaveOpen: true))
                    ds.Write(raw, 0, raw.Length);
                deflated = ms.ToArray();
            }

            // zlib wrapper: CMF/FLG 0x78 0x01 ((0x7801 % 31) == 0), then adler32.
            var idat = new byte[2 + deflated.Length + 4];
            idat[0] = 0x78; idat[1] = 0x01;
            System.Buffer.BlockCopy(deflated, 0, idat, 2, deflated.Length);
            uint adler = Adler32(raw);
            idat[idat.Length - 4] = (byte)(adler >> 24);
            idat[idat.Length - 3] = (byte)(adler >> 16);
            idat[idat.Length - 2] = (byte)(adler >> 8);
            idat[idat.Length - 1] = (byte)adler;

            var ihdr = new byte[13];
            WriteBE(ihdr, 0, (uint)size);
            WriteBE(ihdr, 4, (uint)size);
            ihdr[8] = 8;  // bit depth
            ihdr[9] = 6;  // colour type: RGBA
            // compression 0 / filter 0 / interlace 0 already zero

            var outMs = new MemoryStream(idat.Length + 128);
            outMs.Write(new byte[] { 0x89, 0x50, 0x4E, 0x47, 0x0D, 0x0A, 0x1A, 0x0A }, 0, 8);
            WritePngChunk(outMs, "IHDR", ihdr);
            WritePngChunk(outMs, "IDAT", idat);
            WritePngChunk(outMs, "IEND", System.Array.Empty<byte>());
            return outMs.ToArray();
        }

        private static void WritePngChunk(MemoryStream ms, string type, byte[] data)
        {
            var len = new byte[4];
            WriteBE(len, 0, (uint)data.Length);
            ms.Write(len, 0, 4);
            var typeBytes = Encoding.ASCII.GetBytes(type);
            ms.Write(typeBytes, 0, 4);
            ms.Write(data, 0, data.Length);
            uint crc = Crc32(typeBytes, Crc32Seed);
            crc = Crc32(data, crc);
            var crcBytes = new byte[4];
            WriteBE(crcBytes, 0, ~crc);
            ms.Write(crcBytes, 0, 4);
        }

        private static void WriteBE(byte[] b, int at, uint v)
        {
            b[at] = (byte)(v >> 24); b[at + 1] = (byte)(v >> 16);
            b[at + 2] = (byte)(v >> 8); b[at + 3] = (byte)v;
        }

        private static uint Adler32(byte[] data)
        {
            const uint mod = 65521;
            uint a = 1, b = 0;
            for (int i = 0; i < data.Length;)
            {
                int end = System.Math.Min(i + 5552, data.Length); // overflow-safe stride
                for (; i < end; i++) { a += data[i]; b += a; }
                a %= mod; b %= mod;
            }
            return (b << 16) | a;
        }

        private const uint Crc32Seed = 0xFFFFFFFFu;
        private static uint[] s_crcTable;

        private static uint Crc32(byte[] data, uint crc)
        {
            if (s_crcTable == null)
            {
                var table = new uint[256];
                for (uint n = 0; n < 256; n++)
                {
                    uint c = n;
                    for (int k = 0; k < 8; k++)
                        c = (c & 1) != 0 ? 0xEDB88320u ^ (c >> 1) : c >> 1;
                    table[n] = c;
                }
                s_crcTable = table;
            }
            foreach (byte t in data) crc = s_crcTable[(crc ^ t) & 0xFF] ^ (crc >> 8);
            return crc;
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
            // that triangle's 2x2 block centre. The st array is 3 values per
            // triangle and dominates the usdc size, so it is written as
            // texCoord2h (half) — ~5.5 MB smaller at show sizes. Only up to a
            // 2048px atlas though: a half's spacing in [0.5, 1) is 2^-11, which
            // at 2048 equals the 1-texel safe window around the block centre
            // (a representable half always exists inside it — snapped below),
            // while at 4096 the window is narrower than the spacing and pure
            // block colour can no longer be guaranteed — that size keeps float.
            bool halfSt = texSize <= 2048;
            w.Write(halfSt ? "        texCoord2h[] primvars:st = ["
                           : "        texCoord2f[] primvars:st = [");
            for (int t = 0; t < triCount; t++)
            {
                float u, v;
                if (halfSt)
                {
                    u = SafeHalfBlockUv((t % blocksPerRow) * 2, texSize);
                    v = SafeHalfBlockUv((t / blocksPerRow) * 2, texSize);
                }
                else
                {
                    u = ((t % blocksPerRow) * 2 + 1) / (float)texSize;
                    v = ((t / blocksPerRow) * 2 + 1) / (float)texSize;
                }
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

        // ---- half-precision st ----
        // The UV for a 2x2 atlas block must, AFTER rounding to half, still sample
        // pure block colour: at least 0.5 texel inside the block so the bilinear
        // kernel never reaches the neighbour. Returns the representable half
        // nearest the block centre, nudged one ULP when the rounded centre falls
        // outside the safe window. Callers guarantee texSize <= 2048, where the
        // window (1 texel) is never narrower than the half spacing, so a valid
        // half always exists.
        private static float SafeHalfBlockUv(int blockTexel, int texSize)
        {
            float lo = (blockTexel + 0.5f) / texSize;
            float hi = (blockTexel + 1.5f) / texSize;
            float h = HalfRound((blockTexel + 1) / (float)texSize);
            if (h < lo) h = HalfToFloat((ushort)(FloatToHalf(h) + 1));
            else if (h > hi) h = HalfToFloat((ushort)(FloatToHalf(h) - 1));
            return h;
        }

        private static float HalfRound(float f) => HalfToFloat(FloatToHalf(f));

        // IEEE 754 binary16 conversions (round-to-nearest-even), managed so the
        // export path stays free of main-thread-only Unity APIs. Inputs here are
        // always finite UVs in (0, 1]; the clamping below is just safety.
        private static ushort FloatToHalf(float f)
        {
            uint bits = (uint)BitConverter.SingleToInt32Bits(f);
            uint sign = (bits >> 16) & 0x8000u;
            int exp = (int)((bits >> 23) & 0xFF) - 127 + 15;
            uint man = bits & 0x007FFFFFu;
            if (exp >= 31) return (ushort)(sign | 0x7C00u); // overflow -> inf
            if (exp <= 0)
            {
                if (exp < -10) return (ushort)sign; // underflow -> 0
                man |= 0x00800000u;                 // subnormal half
                int shift = 14 - exp;
                uint sub = man >> shift;
                uint rem = man & ((1u << shift) - 1u);
                uint half2 = 1u << (shift - 1);
                if (rem > half2 || (rem == half2 && (sub & 1u) != 0)) sub++;
                return (ushort)(sign | sub);
            }
            uint rounded = man >> 13;
            uint rest = man & 0x1FFFu;
            if (rest > 0x1000u || (rest == 0x1000u && (rounded & 1u) != 0)) rounded++;
            uint result = sign | ((uint)exp << 10) + rounded; // mantissa carry may bump exp
            return (ushort)result;
        }

        private static float HalfToFloat(ushort h)
        {
            uint sign = (uint)(h & 0x8000) << 16;
            int exp = (h >> 10) & 0x1F;
            uint man = (uint)(h & 0x03FF);
            uint bits;
            if (exp == 0)
            {
                if (man == 0) bits = sign; // zero
                else
                {
                    // subnormal half -> normalised float
                    int e = -1;
                    do { man <<= 1; e++; } while ((man & 0x0400u) == 0);
                    bits = sign | (uint)(127 - 15 - e) << 23 | ((man & 0x03FFu) << 13);
                }
            }
            else if (exp == 31) bits = sign | 0x7F800000u | (man << 13); // inf/nan
            else bits = sign | (uint)(exp - 15 + 127) << 23 | (man << 13);
            return BitConverter.Int32BitsToSingle((int)bits);
        }

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
