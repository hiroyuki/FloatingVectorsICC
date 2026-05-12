// ScannedReality-Studio-adjacent recording layout. For each recorded device we write
// multiple per-sensor RCSV files under
//   <root>/dataset/<hostname>/FemtoBolt_<serial>/<sensor>
// along with the top-level YAML metadata files and a calibration/extrinsics.yaml that
// describes the camera intrinsics and depth-to-color extrinsic.
//
// The three sensor files we emit are:
//   - depth_main    : raw Y16 depth frames (2 bytes per pixel, width*height*2 B/record)
//   - color_main    : raw RGB8 color frames (3 bytes per pixel, width*height*3 B/record)
//   - pointcloud_main (legacy) : ObColorPoint buffer (24 B per point) — kept for back-compat
//                     when raw sensor capture is unavailable; not part of the SR layout.
//
// Every RCSV file uses the variable-size record layout: `u64 timestamp_ns + u32 size + bytes`
// followed by a trailing index chunk (u64 count, then u64 offsets[count + 1]). Even though
// depth records are strictly constant-size, we keep the uniform layout to share the reader.
//
// Caveat: SR Studio expects MJPG in color_main (not raw RGB) and a "constant-size variant"
// header in depth_main, so files written here are not directly consumable by SR Studio. The
// format is intentionally close to SR's directory layout so a MJPG/constant-size migration
// later can drop in without reshuffling the whole tree.

using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Text;
using Orbbec;
using YamlDotNet.Core;
using YamlDotNet.RepresentationModel;

namespace PointCloud
{
    public static class PointCloudRecording
    {
        public const string Magic = "RCSV";

        public const string DepthSensorName      = "depth_main";
        public const string ColorSensorName      = "color_main";
        public const string IRSensorName         = "ir_main";
        public const string PointcloudSensorName = "pointcloud_main"; // legacy

        public const int PointCloudVertexStride = 24; // sizeof(ObColorPoint)

        /// <summary>One recorded frame for a single sensor. <see cref="Bytes"/> is the raw payload.</summary>
        public sealed class Frame
        {
            public ulong TimestampNs;
            public byte[] Bytes;

            /// <summary>Convenience: legacy pointcloud_main byte count expressed as point count.</summary>
            public int PointCount => Bytes == null ? 0 : Bytes.Length / PointCloudVertexStride;
        }

        // ------------------------------------------------------------------
        // RCSV reader/writer (shared across all sensor files)
        // ------------------------------------------------------------------

        /// <summary>Write an RCSV file with the provided header YAML and variable-size records.</summary>
        public static void WriteRcsv(string filePath, string headerYaml, IReadOnlyList<Frame> frames)
        {
            if (filePath == null) throw new ArgumentNullException(nameof(filePath));
            if (frames == null) throw new ArgumentNullException(nameof(frames));

            Directory.CreateDirectory(Path.GetDirectoryName(filePath));
            byte[] headerBytes = Encoding.UTF8.GetBytes(headerYaml ?? string.Empty);

            using var fs = new FileStream(filePath, FileMode.Create, FileAccess.Write, FileShare.None);
            using var bw = new BinaryWriter(fs, Encoding.UTF8, leaveOpen: false);

            bw.Write(Encoding.ASCII.GetBytes(Magic));
            long indexChunkOffsetPos = fs.Position;
            bw.Write((ulong)0);                  // placeholder: index chunk offset
            bw.Write((ulong)0);                  // reserved
            bw.Write((uint)headerBytes.Length);
            bw.Write(headerBytes);

            var offsets = new List<ulong>(frames.Count + 1);
            for (int i = 0; i < frames.Count; i++)
            {
                var f = frames[i];
                if (f == null || f.Bytes == null) continue;
                offsets.Add((ulong)fs.Position);
                bw.Write(f.TimestampNs);
                bw.Write((uint)f.Bytes.Length);
                bw.Write(f.Bytes);
            }
            offsets.Add((ulong)fs.Position);

            ulong indexChunkOffset = (ulong)fs.Position;
            ulong validRecordCount = (ulong)(offsets.Count - 1);
            bw.Write(validRecordCount);
            for (int i = 0; i < offsets.Count; i++) bw.Write(offsets[i]);

            bw.Flush();
            fs.Position = indexChunkOffsetPos;
            bw.Write(indexChunkOffset);
        }

        /// <summary>Read an RCSV file, returning its records in order.</summary>
        public static List<Frame> ReadRcsv(string filePath)
        {
            if (filePath == null) throw new ArgumentNullException(nameof(filePath));

            using var fs = new FileStream(filePath, FileMode.Open, FileAccess.Read, FileShare.Read);
            using var br = new BinaryReader(fs, Encoding.UTF8, leaveOpen: false);

            var magic = Encoding.ASCII.GetString(br.ReadBytes(4));
            if (magic != Magic)
                throw new InvalidDataException($"Not an RCSV file: magic '{magic}' at {filePath}");
            ulong indexChunkOffset = br.ReadUInt64();
            br.ReadUInt64(); // reserved
            uint headerTextSize = br.ReadUInt32();
            fs.Seek(headerTextSize, SeekOrigin.Current); // skip header text

            if (indexChunkOffset == 0 || indexChunkOffset >= (ulong)fs.Length)
                throw new InvalidDataException($"Invalid index chunk offset in {filePath}");
            fs.Seek((long)indexChunkOffset, SeekOrigin.Begin);
            ulong validRecordCount = br.ReadUInt64();
            var offsets = new ulong[validRecordCount + 1];
            for (ulong i = 0; i <= validRecordCount; i++) offsets[i] = br.ReadUInt64();

            var frames = new List<Frame>((int)validRecordCount);
            for (ulong i = 0; i < validRecordCount; i++)
            {
                fs.Seek((long)offsets[i], SeekOrigin.Begin);
                ulong ts = br.ReadUInt64();
                uint size = br.ReadUInt32();
                byte[] bytes = br.ReadBytes((int)size);
                if (bytes.Length != (int)size)
                    throw new EndOfStreamException($"Truncated record {i} in {filePath}");
                frames.Add(new Frame { TimestampNs = ts, Bytes = bytes });
            }
            return frames;
        }

        /// <summary>
        /// Read just the (width, height) recorded in an RCSV file's header. Used by
        /// the playback path to recover image dimensions when no live PointCloudRenderer
        /// is available (e.g. offline frame-by-frame eval with cameras disconnected).
        /// Returns (0, 0) if the header lacks the keys or the file is malformed.
        /// </summary>
        public static (int width, int height) ReadRcsvHeaderDimensions(string filePath)
        {
            if (string.IsNullOrEmpty(filePath) || !File.Exists(filePath)) return (0, 0);
            using var fs = new FileStream(filePath, FileMode.Open, FileAccess.Read, FileShare.Read);
            using var br = new BinaryReader(fs, Encoding.UTF8, leaveOpen: false);

            var magic = Encoding.ASCII.GetString(br.ReadBytes(4));
            if (magic != Magic) return (0, 0);
            br.ReadUInt64(); // index chunk offset
            br.ReadUInt64(); // reserved
            uint headerTextSize = br.ReadUInt32();
            if (headerTextSize == 0 || headerTextSize > 64 * 1024) return (0, 0);
            byte[] headerBytes = br.ReadBytes((int)headerTextSize);
            string headerText = Encoding.UTF8.GetString(headerBytes);
            int w = ParseFirstYamlInt(headerText, "width");
            int h = ParseFirstYamlInt(headerText, "height");
            return (w, h);
        }

        // Tiny YAML scanner that returns the first value for `<key>:` in the header
        // text. We don't want a full parser dependency for two ints; this looks for
        // a line that starts (after trim) with the key + ':' and parses the integer.
        private static int ParseFirstYamlInt(string yaml, string key)
        {
            if (string.IsNullOrEmpty(yaml)) return 0;
            foreach (var rawLine in yaml.Split('\n'))
            {
                string line = rawLine.TrimStart();
                if (!line.StartsWith(key, StringComparison.Ordinal)) continue;
                int colon = line.IndexOf(':');
                if (colon != key.Length) continue; // matched a longer key like "width_real"
                string v = line.Substring(colon + 1).Trim();
                int comment = v.IndexOf('#');
                if (comment >= 0) v = v.Substring(0, comment).Trim();
                if (int.TryParse(v, System.Globalization.NumberStyles.Integer,
                                 System.Globalization.CultureInfo.InvariantCulture, out int n))
                {
                    return n;
                }
            }
            return 0;
        }

        // ------------------------------------------------------------------
        // Directory / file-path helpers (Scanned Reality-style layout)
        // ------------------------------------------------------------------

        public static string DatasetRoot(string rootDir) =>
            Path.Combine(rootDir, "dataset");

        public static string HostDir(string rootDir, string hostname) =>
            Path.Combine(DatasetRoot(rootDir), SanitizeForPath(hostname));

        public static string DeviceDir(string rootDir, string hostname, string serial) =>
            Path.Combine(HostDir(rootDir, hostname), $"FemtoBolt_{SanitizeForPath(serial)}");

        public static string SensorFilePath(string rootDir, string hostname, string serial, string sensorName) =>
            Path.Combine(DeviceDir(rootDir, hostname, serial), sensorName);

        public static string CalibrationDir(string rootDir) =>
            Path.Combine(rootDir, "calibration");

        /// <summary>
        /// Enumerates recorded devices under <paramref name="rootDir"/>. Returns (serial, deviceDir)
        /// pairs so callers can pick the sensor files they care about (depth_main, color_main, etc.).
        /// </summary>
        public static IEnumerable<(string serial, string deviceDir)> EnumerateDevices(string rootDir)
        {
            string datasetDir = DatasetRoot(rootDir);
            if (!Directory.Exists(datasetDir)) yield break;

            foreach (var hostDir in Directory.EnumerateDirectories(datasetDir))
            {
                foreach (var deviceDir in Directory.EnumerateDirectories(hostDir))
                {
                    string devName = Path.GetFileName(deviceDir);
                    string serial = devName;
                    int underscore = devName.IndexOf('_');
                    if (underscore >= 0 && underscore + 1 < devName.Length)
                        serial = devName.Substring(underscore + 1);
                    yield return (serial, deviceDir);
                }
            }
        }

        // ------------------------------------------------------------------
        // Sensor header YAML builders
        // ------------------------------------------------------------------

        public static string BuildDepthHeaderYaml(string serial, int width, int height)
        {
            var sb = new StringBuilder();
            sb.Append("record_format:\n");
            sb.Append("  - name: timestamp_ns\n");
            sb.Append("    type: u64\n");
            sb.Append("    comment: device frame timestamp; comparable across devices after ob_device_timer_sync_with_host\n");
            sb.Append("  - name: image\n");
            sb.Append("    type: u16\n");
            sb.Append($"    count: {width * height}\n");
            sb.Append("    comment: Y16 depth (2 bytes per pixel, row-major, values in millimeters)\n");
            sb.Append("custom:\n");
            sb.Append("  sensor:\n");
            sb.Append("    sensor_type: depth\n");
            sb.Append("    format: y16\n");
            sb.Append($"    width: {width}\n");
            sb.Append($"    height: {height}\n");
            sb.Append($"  device_serial: \"{EscapeYaml(serial)}\"\n");
            sb.Append("  timestamp_basis: device_synced_unix_like_ns\n");
            sb.Append("  producer: FloatingVectorsICC\n");
            return sb.ToString();
        }

        public static string BuildColorHeaderYaml(string serial, int width, int height)
        {
            var sb = new StringBuilder();
            sb.Append("record_format:\n");
            sb.Append("  - name: timestamp_ns\n");
            sb.Append("    type: u64\n");
            sb.Append("  - name: image\n");
            sb.Append("    type: u8\n");
            sb.Append($"    count: {width * height * 3}\n");
            sb.Append("    comment: RGB8 (3 bytes per pixel, row-major)\n");
            sb.Append("custom:\n");
            sb.Append("  sensor:\n");
            sb.Append("    sensor_type: color\n");
            sb.Append("    format: rgb8\n");
            sb.Append($"    width: {width}\n");
            sb.Append($"    height: {height}\n");
            sb.Append($"  device_serial: \"{EscapeYaml(serial)}\"\n");
            sb.Append("  timestamp_basis: device_synced_unix_like_ns\n");
            sb.Append("  producer: FloatingVectorsICC\n");
            return sb.ToString();
        }

        public static string BuildIRHeaderYaml(string serial, int width, int height)
        {
            var sb = new StringBuilder();
            sb.Append("record_format:\n");
            sb.Append("  - name: timestamp_ns\n");
            sb.Append("    type: u64\n");
            sb.Append("  - name: image\n");
            sb.Append("    type: u16\n");
            sb.Append($"    count: {width * height}\n");
            sb.Append("    comment: Y16 passive IR (2 bytes per pixel, row-major). Required by k4abt for offline body recompute.\n");
            sb.Append("custom:\n");
            sb.Append("  sensor:\n");
            sb.Append("    sensor_type: ir\n");
            sb.Append("    format: y16\n");
            sb.Append($"    width: {width}\n");
            sb.Append($"    height: {height}\n");
            sb.Append($"  device_serial: \"{EscapeYaml(serial)}\"\n");
            sb.Append("  timestamp_basis: device_synced_unix_like_ns\n");
            sb.Append("  producer: FloatingVectorsICC\n");
            return sb.ToString();
        }

        public static string BuildPointcloudHeaderYaml(string serial)
        {
            var sb = new StringBuilder();
            sb.Append("record_format:\n");
            sb.Append("  - name: timestamp_ns\n");
            sb.Append("    type: u64\n");
            sb.Append("  - name: image\n");
            sb.Append("    type: u32\n");
            sb.Append("    count: 0\n");
            sb.Append("    comment: ObColorPoint[] (legacy, 6 floats per point, 24 B stride)\n");
            sb.Append("custom:\n");
            sb.Append("  sensor:\n");
            sb.Append("    sensor_type: pointcloud_legacy\n");
            sb.Append($"  device_serial: \"{EscapeYaml(serial)}\"\n");
            sb.Append("  producer: FloatingVectorsICC\n");
            return sb.ToString();
        }

        // ------------------------------------------------------------------
        // Top-level metadata (configuration / dataset / sensor_node_config / hostinfo / extrinsics)
        // ------------------------------------------------------------------

        public static void WriteDatasetMetadata(
            string rootDir,
            string hostname,
            string datasetName,
            IReadOnlyList<string> deviceSerials)
        {
            Directory.CreateDirectory(rootDir);

            File.WriteAllText(
                Path.Combine(rootDir, "configuration.yaml"),
                BuildConfigurationYaml(hostname),
                new UTF8Encoding(false));

            File.WriteAllText(
                Path.Combine(rootDir, "dataset.yaml"),
                BuildDatasetYaml(datasetName, hostname, deviceSerials),
                new UTF8Encoding(false));

            File.WriteAllText(
                Path.Combine(rootDir, "sensor_node_config.yaml"),
                BuildSensorNodeConfigYaml(hostname, deviceSerials),
                new UTF8Encoding(false));

            Directory.CreateDirectory(HostDir(rootDir, hostname));
            File.WriteAllText(
                Path.Combine(HostDir(rootDir, hostname), "hostinfo.yaml"),
                BuildHostinfoYaml(hostname),
                new UTF8Encoding(false));
        }

        /// <summary>
        /// Writes <c>calibration/extrinsics.yaml</c>. Each device entry contains the color/depth
        /// intrinsics, depth-to-color extrinsic, and a <c>global_tr_colorCamera</c> transform
        /// taken from <paramref name="globalExtrinsics"/>. When <paramref name="globalExtrinsics"/>
        /// is null for a device, identity is written (single-camera shortcut).
        /// </summary>
        public static void WriteExtrinsicsYaml(
            string rootDir,
            IReadOnlyList<DeviceCalibration> calibrations)
        {
            if (calibrations == null || calibrations.Count == 0) return;
            Directory.CreateDirectory(CalibrationDir(rootDir));
            string path = Path.Combine(CalibrationDir(rootDir), "extrinsics.yaml");

            var sb = new StringBuilder();
            sb.Append("# Generated by FloatingVectorsICC. Calibration values are expressed in the\n");
            sb.Append("# Orbbec/OpenCV camera convention: +x right, +y down, +z forward (right-handed),\n");
            sb.Append("# translations in meters. Unity-side basis conversion happens at apply time.\n");
            sb.Append("cameras:\n");
            foreach (var c in calibrations)
            {
                sb.Append($"  - device_serial: \"{EscapeYaml(c.Serial)}\"\n");

                sb.Append("    color_intrinsic:\n");
                AppendIntrinsicYaml(sb, c.ColorIntrinsic, indent: "      ");
                sb.Append("    depth_intrinsic:\n");
                AppendIntrinsicYaml(sb, c.DepthIntrinsic, indent: "      ");

                sb.Append("    depth_distortion:\n");
                AppendDistortionYaml(sb, c.DepthDistortion, indent: "      ");
                sb.Append("    color_distortion:\n");
                AppendDistortionYaml(sb, c.ColorDistortion, indent: "      ");

                // SDK transform is depth -> color in millimeters. Express as
                //   colorCamera_tr_depthCamera (SR convention) in meters.
                sb.Append("    colorCamera_tr_depthCamera:\n");
                AppendExtrinsicYaml(sb, c.DepthToColor, indent: "      ");

                var global = c.GlobalTrColorCamera ?? Identity;
                sb.Append("    global_tr_colorCamera:\n");
                AppendExtrinsicYaml(sb, global, indent: "      ");
            }
            File.WriteAllText(path, sb.ToString(), new UTF8Encoding(false));
        }

        /// <summary>
        /// Reads <c>calibration/extrinsics.yaml</c> back into <see cref="DeviceCalibration"/>
        /// records. Throws when the file is missing, malformed, or has an unsafe
        /// <c>device_serial</c> (path traversal). Translations come back as millimeters
        /// in <see cref="ObExtrinsic.Trans"/> to keep the in-memory struct convention
        /// consistent with the Orbbec SDK (YAML stores meters; we convert at this boundary).
        /// </summary>
        public static IReadOnlyList<DeviceCalibration> ReadExtrinsicsYaml(string rootDir)
        {
            if (string.IsNullOrEmpty(rootDir)) throw new ArgumentException("rootDir is empty", nameof(rootDir));
            string path = Path.Combine(CalibrationDir(rootDir), "extrinsics.yaml");
            if (!File.Exists(path))
                throw new FileNotFoundException("extrinsics.yaml not found under calibration/", path);

            var stream = new YamlStream();
            try
            {
                using var reader = new StringReader(File.ReadAllText(path));
                stream.Load(reader);
            }
            catch (YamlException e)
            {
                throw new InvalidDataException($"extrinsics.yaml parse error: {e.Message}", e);
            }
            if (stream.Documents.Count == 0)
                throw new InvalidDataException("extrinsics.yaml is empty");

            if (!(stream.Documents[0].RootNode is YamlMappingNode root))
                throw new InvalidDataException("extrinsics.yaml: root must be a mapping");
            if (!root.Children.TryGetValue(new YamlScalarNode("cameras"), out var camerasObj)
                || !(camerasObj is YamlSequenceNode camerasSeq))
                throw new InvalidDataException("extrinsics.yaml: missing 'cameras:' sequence");

            var result = new List<DeviceCalibration>(camerasSeq.Children.Count);
            int idx = 0;
            foreach (var camNode in camerasSeq.Children)
            {
                if (!(camNode is YamlMappingNode cam))
                    throw new InvalidDataException($"cameras[{idx}]: entry must be a mapping");

                string serial = ReadScalarString(cam, "device_serial", $"cameras[{idx}]");
                if (serial.Contains("..") || serial.Contains('/') || serial.Contains('\\'))
                    throw new InvalidDataException(
                        $"cameras[{idx}]: unsafe device_serial '{serial}' (path traversal)");

                var cal = new DeviceCalibration
                {
                    Serial = serial,
                    ColorIntrinsic = ReadIntrinsicNode(cam, "color_intrinsic", $"cameras[{idx}]"),
                    DepthIntrinsic = ReadIntrinsicNode(cam, "depth_intrinsic", $"cameras[{idx}]"),
                    ColorDistortion = ReadDistortionNode(cam, "color_distortion", $"cameras[{idx}]"),
                    DepthDistortion = ReadDistortionNode(cam, "depth_distortion", $"cameras[{idx}]"),
                    DepthToColor = ReadExtrinsicNode(cam, "colorCamera_tr_depthCamera", $"cameras[{idx}]"),
                    GlobalTrColorCamera = ReadExtrinsicNode(cam, "global_tr_colorCamera", $"cameras[{idx}]"),
                };
                result.Add(cal);
                idx++;
            }
            return result;
        }

        private static YamlMappingNode RequireMapping(YamlMappingNode parent, string key, string ctx)
        {
            if (!parent.Children.TryGetValue(new YamlScalarNode(key), out var v))
                throw new InvalidDataException($"{ctx}: missing '{key}'");
            if (!(v is YamlMappingNode m))
                throw new InvalidDataException($"{ctx}.{key}: must be a mapping");
            return m;
        }

        private static YamlSequenceNode RequireSequence(YamlMappingNode parent, string key, string ctx)
        {
            if (!parent.Children.TryGetValue(new YamlScalarNode(key), out var v))
                throw new InvalidDataException($"{ctx}: missing '{key}'");
            if (!(v is YamlSequenceNode s))
                throw new InvalidDataException($"{ctx}.{key}: must be a sequence");
            return s;
        }

        private static string ReadScalarString(YamlMappingNode m, string key, string ctx)
        {
            if (!m.Children.TryGetValue(new YamlScalarNode(key), out var v))
                throw new InvalidDataException($"{ctx}: missing '{key}'");
            if (!(v is YamlScalarNode s))
                throw new InvalidDataException($"{ctx}.{key}: expected scalar");
            return s.Value ?? string.Empty;
        }

        private static float ReadScalarFloat(YamlMappingNode m, string key, string ctx)
        {
            string s = ReadScalarString(m, key, ctx);
            if (!float.TryParse(s, NumberStyles.Float, CultureInfo.InvariantCulture, out float v))
                throw new InvalidDataException($"{ctx}.{key}: '{s}' is not a float");
            return v;
        }

        private static int ReadScalarInt(YamlMappingNode m, string key, string ctx)
        {
            string s = ReadScalarString(m, key, ctx);
            if (!int.TryParse(s, NumberStyles.Integer, CultureInfo.InvariantCulture, out int v))
                throw new InvalidDataException($"{ctx}.{key}: '{s}' is not an int");
            return v;
        }

        private static float[] ReadFloatSequence(YamlMappingNode m, string key, int expectedLen, string ctx)
        {
            var seq = RequireSequence(m, key, ctx);
            if (seq.Children.Count != expectedLen)
                throw new InvalidDataException(
                    $"{ctx}.{key}: expected {expectedLen} entries, got {seq.Children.Count}");
            var arr = new float[expectedLen];
            for (int i = 0; i < expectedLen; i++)
            {
                if (!(seq.Children[i] is YamlScalarNode s))
                    throw new InvalidDataException($"{ctx}.{key}[{i}]: expected scalar");
                if (!float.TryParse(s.Value, NumberStyles.Float, CultureInfo.InvariantCulture, out arr[i]))
                    throw new InvalidDataException($"{ctx}.{key}[{i}]: '{s.Value}' is not a float");
            }
            return arr;
        }

        private static ObCameraIntrinsic ReadIntrinsicNode(YamlMappingNode parent, string key, string ctx)
        {
            var n = RequireMapping(parent, key, ctx);
            string sub = $"{ctx}.{key}";
            int w = ReadScalarInt(n, "width", sub);
            int h = ReadScalarInt(n, "height", sub);
            if (w < 0 || w > short.MaxValue || h < 0 || h > short.MaxValue)
                throw new InvalidDataException($"{sub}: width/height out of int16 range");
            return new ObCameraIntrinsic
            {
                Fx = ReadScalarFloat(n, "fx", sub),
                Fy = ReadScalarFloat(n, "fy", sub),
                Cx = ReadScalarFloat(n, "cx", sub),
                Cy = ReadScalarFloat(n, "cy", sub),
                Width = (short)w,
                Height = (short)h,
            };
        }

        private static ObCameraDistortion ReadDistortionNode(YamlMappingNode parent, string key, string ctx)
        {
            var n = RequireMapping(parent, key, ctx);
            string sub = $"{ctx}.{key}";
            string modelStr = ReadScalarString(n, "model", sub);
            if (!Enum.TryParse(modelStr, ignoreCase: true, out ObCameraDistortionModel model))
                throw new InvalidDataException($"{sub}.model: unknown distortion model '{modelStr}'");
            return new ObCameraDistortion
            {
                K1 = ReadScalarFloat(n, "k1", sub),
                K2 = ReadScalarFloat(n, "k2", sub),
                K3 = ReadScalarFloat(n, "k3", sub),
                K4 = ReadScalarFloat(n, "k4", sub),
                K5 = ReadScalarFloat(n, "k5", sub),
                K6 = ReadScalarFloat(n, "k6", sub),
                P1 = ReadScalarFloat(n, "p1", sub),
                P2 = ReadScalarFloat(n, "p2", sub),
                Model = model,
            };
        }

        private static ObExtrinsic ReadExtrinsicNode(YamlMappingNode parent, string key, string ctx)
        {
            var n = RequireMapping(parent, key, ctx);
            string sub = $"{ctx}.{key}";
            float[] rot = ReadFloatSequence(n, "rot", 9, sub);
            float[] transM = ReadFloatSequence(n, "trans_m", 3, sub);
            // Convert YAML-meters back into the struct's mm convention to stay in lockstep
            // with values produced by the Orbbec SDK (ob_extrinsic.trans is mm).
            return new ObExtrinsic
            {
                Rot = rot,
                Trans = new[] { transM[0] * 1000f, transM[1] * 1000f, transM[2] * 1000f },
            };
        }

        private static string BuildConfigurationYaml(string hostname)
        {
            var sb = new StringBuilder();
            sb.Append("# configuration.yaml — dataset-level configuration (FloatingVectorsICC)\n");
            sb.Append("version: 1\n");
            sb.Append("producer: FloatingVectorsICC\n");
            sb.Append($"created_at: \"{DateTime.UtcNow.ToString("o", CultureInfo.InvariantCulture)}\"\n");
            sb.Append($"primary_host: \"{EscapeYaml(hostname)}\"\n");
            return sb.ToString();
        }

        private static string BuildDatasetYaml(string datasetName, string hostname, IReadOnlyList<string> serials)
        {
            var sb = new StringBuilder();
            sb.Append("# dataset.yaml — lists the hosts and devices that participate in this dataset\n");
            sb.Append($"name: \"{EscapeYaml(datasetName)}\"\n");
            sb.Append("hosts:\n");
            sb.Append($"  - name: \"{EscapeYaml(hostname)}\"\n");
            sb.Append("    devices:\n");
            foreach (var serial in serials)
            {
                sb.Append($"      - device_type: FemtoBolt\n");
                sb.Append($"        serial: \"{EscapeYaml(serial)}\"\n");
                sb.Append($"        sensors: [depth_main, color_main]\n");
            }
            return sb.ToString();
        }

        private static string BuildSensorNodeConfigYaml(string hostname, IReadOnlyList<string> serials)
        {
            var sb = new StringBuilder();
            sb.Append("# sensor_node_config.yaml — per-sensor configuration written at record time\n");
            sb.Append($"host: \"{EscapeYaml(hostname)}\"\n");
            sb.Append("sensors:\n");
            foreach (var serial in serials)
            {
                sb.Append($"  - device_serial: \"{EscapeYaml(serial)}\"\n");
                sb.Append("    sensor_name: depth_main\n");
                sb.Append("    format: y16\n");
                sb.Append($"  - device_serial: \"{EscapeYaml(serial)}\"\n");
                sb.Append("    sensor_name: color_main\n");
                sb.Append("    format: rgb8\n");
            }
            return sb.ToString();
        }

        private static string BuildHostinfoYaml(string hostname)
        {
            var sb = new StringBuilder();
            sb.Append("# hostinfo.yaml — metadata about the recording host\n");
            sb.Append($"name: \"{EscapeYaml(hostname)}\"\n");
            sb.Append($"recorded_at: \"{DateTime.UtcNow.ToString("o", CultureInfo.InvariantCulture)}\"\n");
            sb.Append("producer: FloatingVectorsICC\n");
            return sb.ToString();
        }

        private static void AppendIntrinsicYaml(StringBuilder sb, ObCameraIntrinsic k, string indent)
        {
            sb.Append(indent).Append("fx: ").Append(F(k.Fx)).Append('\n');
            sb.Append(indent).Append("fy: ").Append(F(k.Fy)).Append('\n');
            sb.Append(indent).Append("cx: ").Append(F(k.Cx)).Append('\n');
            sb.Append(indent).Append("cy: ").Append(F(k.Cy)).Append('\n');
            sb.Append(indent).Append("width: ").Append(k.Width).Append('\n');
            sb.Append(indent).Append("height: ").Append(k.Height).Append('\n');
        }

        private static void AppendDistortionYaml(StringBuilder sb, ObCameraDistortion d, string indent)
        {
            sb.Append(indent).Append($"model: {d.Model}\n");
            sb.Append(indent).Append($"k1: {F(d.K1)}\n");
            sb.Append(indent).Append($"k2: {F(d.K2)}\n");
            sb.Append(indent).Append($"k3: {F(d.K3)}\n");
            sb.Append(indent).Append($"k4: {F(d.K4)}\n");
            sb.Append(indent).Append($"k5: {F(d.K5)}\n");
            sb.Append(indent).Append($"k6: {F(d.K6)}\n");
            sb.Append(indent).Append($"p1: {F(d.P1)}\n");
            sb.Append(indent).Append($"p2: {F(d.P2)}\n");
        }

        private static void AppendExtrinsicYaml(StringBuilder sb, ObExtrinsic e, string indent)
        {
            sb.Append(indent).Append("rot: [");
            for (int i = 0; i < 9; i++)
            {
                sb.Append(F(e.Rot[i]));
                if (i < 8) sb.Append(", ");
            }
            sb.Append("]\n");
            // SR convention uses meters; SDK returns millimeters. Convert here for translation only.
            sb.Append(indent).Append("trans_m: [");
            for (int i = 0; i < 3; i++)
            {
                sb.Append(F(e.Trans[i] * 0.001f));
                if (i < 2) sb.Append(", ");
            }
            sb.Append("]\n");
        }

        // ------------------------------------------------------------------
        // Utilities
        // ------------------------------------------------------------------

        /// <summary>Identity extrinsic: zero rotation matrix diagonal of 1s, zero translation.</summary>
        public static ObExtrinsic Identity => new ObExtrinsic
        {
            Rot = new float[] { 1, 0, 0,  0, 1, 0,  0, 0, 1 },
            Trans = new float[] { 0, 0, 0 },
        };

        public sealed class DeviceCalibration
        {
            public string Serial;
            public ObCameraIntrinsic ColorIntrinsic;
            public ObCameraIntrinsic DepthIntrinsic;
            public ObCameraDistortion ColorDistortion;
            public ObCameraDistortion DepthDistortion;
            public ObExtrinsic DepthToColor;      // colorCamera_tr_depthCamera (mm in .Trans)
            public ObExtrinsic? GlobalTrColorCamera; // null -> identity
        }

        private static string F(float v) => v.ToString("R", CultureInfo.InvariantCulture);

        private static string EscapeYaml(string s)
        {
            if (string.IsNullOrEmpty(s)) return string.Empty;
            return s.Replace("\\", "\\\\").Replace("\"", "\\\"");
        }

        private static string SanitizeForPath(string s)
        {
            if (string.IsNullOrEmpty(s)) return "unknown";
            var invalid = Path.GetInvalidFileNameChars();
            var sb = new StringBuilder(s.Length);
            foreach (char c in s)
            {
                sb.Append(Array.IndexOf(invalid, c) >= 0 ? '_' : c);
            }
            return sb.ToString();
        }
    }
}
