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

namespace PointCloud
{
    public static class PointCloudRecording
    {
        public const string Magic = "RCSV";

        public const string DepthSensorName      = "depth_main";
        public const string ColorSensorName      = "color_main";
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
            sb.Append("# Generated by FloatingVectorsICC. Coordinates follow Scanned Reality convention:\n");
            sb.Append("#   +x left, +y up, +z forward. Translations in meters.\n");
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
