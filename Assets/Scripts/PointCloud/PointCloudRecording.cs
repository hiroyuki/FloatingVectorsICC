// ScannedReality-Studio-compatible recording file I/O for Femto Bolt point clouds.
// Writes one RCSV (variable-size record) binary file per device under
//   <root>/dataset/<hostname>/<devicetype>_<serial>/pointcloud_main
// matching the directory layout documented at
//   https://scanned-reality.com/documentation/data_access.html
//
// Each record is one frame of OBColorPoint data:
//   u64 timestamp_ns   (wall-clock Unix time in nanoseconds)
//   u32 image_size     (size of the point buffer in bytes)
//   byte[image_size]   (raw OBColorPoint array: 6 floats per point, 24 bytes per vertex)
//
// The YAML header describes record_format and camera_sensor so that the file is
// self-describing; a trailing index chunk enables O(1) random access to each record.

using System;
using System.Collections.Generic;
using System.IO;
using System.Text;

namespace PointCloud
{
    internal static class PointCloudRecording
    {
        public const string Magic = "RCSV";
        public const string SensorFileName = "pointcloud_main";
        public const int VertexStride = 24; // sizeof(ObColorPoint)

        /// <summary>
        /// One recorded frame for a single device. Bytes are a packed OBColorPoint array.
        /// </summary>
        public sealed class Frame
        {
            public ulong TimestampNs;
            public byte[] Bytes;
            public int PointCount => Bytes == null ? 0 : Bytes.Length / VertexStride;
        }

        /// <summary>
        /// Writes the recording for a single device to the given sensor file path.
        /// </summary>
        public static void WriteDeviceFile(string filePath, string serial, IReadOnlyList<Frame> frames)
        {
            if (filePath == null) throw new ArgumentNullException(nameof(filePath));
            if (frames == null) throw new ArgumentNullException(nameof(frames));

            Directory.CreateDirectory(Path.GetDirectoryName(filePath));

            string headerText = BuildHeaderYaml(serial);
            byte[] headerBytes = Encoding.UTF8.GetBytes(headerText);

            using var fs = new FileStream(filePath, FileMode.Create, FileAccess.Write, FileShare.None);
            using var bw = new BinaryWriter(fs, Encoding.UTF8, leaveOpen: false);

            // --- Header ---
            bw.Write(Encoding.ASCII.GetBytes(Magic));       // "RCSV"
            long indexChunkOffsetPos = fs.Position;
            bw.Write((ulong)0);                             // placeholder: index chunk offset
            bw.Write((ulong)0);                             // reserved (per format spec)
            bw.Write((uint)headerBytes.Length);             // header text size (u32 LE)
            bw.Write(headerBytes);                          // header text (UTF-8 YAML)

            // --- Records ---
            var offsets = new List<ulong>(frames.Count + 1);
            for (int i = 0; i < frames.Count; i++)
            {
                var f = frames[i];
                if (f == null || f.Bytes == null) continue;
                offsets.Add((ulong)fs.Position);
                bw.Write(f.TimestampNs);                    // u64 timestamp_ns
                bw.Write((uint)f.Bytes.Length);             // u32 image size prefix
                bw.Write(f.Bytes);                          // raw ObColorPoint array
            }
            offsets.Add((ulong)fs.Position); // end-of-last-record offset

            // --- Index chunk ---
            ulong indexChunkOffset = (ulong)fs.Position;
            ulong validRecordCount = (ulong)(offsets.Count - 1);
            bw.Write(validRecordCount);                     // u64
            for (int i = 0; i < offsets.Count; i++)
                bw.Write(offsets[i]);                       // u64 offsets (validRecordCount + 1 entries)

            // Back-patch index chunk offset in header.
            bw.Flush();
            fs.Position = indexChunkOffsetPos;
            bw.Write(indexChunkOffset);
        }

        /// <summary>
        /// Reads a single device's RCSV file, returning the frames in record order.
        /// </summary>
        public static List<Frame> ReadDeviceFile(string filePath)
        {
            if (filePath == null) throw new ArgumentNullException(nameof(filePath));

            using var fs = new FileStream(filePath, FileMode.Open, FileAccess.Read, FileShare.Read);
            using var br = new BinaryReader(fs, Encoding.UTF8, leaveOpen: false);

            // --- Header ---
            var magic = Encoding.ASCII.GetString(br.ReadBytes(4));
            if (magic != Magic)
                throw new InvalidDataException($"Not an RCSV file: magic '{magic}' at {filePath}");
            ulong indexChunkOffset = br.ReadUInt64();
            br.ReadUInt64();                                 // reserved
            uint headerTextSize = br.ReadUInt32();
            fs.Seek(headerTextSize, SeekOrigin.Current);     // skip header text (we only round-trip our own writes)

            // --- Index chunk ---
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
        /// Returns the per-device sensor file path for the given recording root and serial.
        /// </summary>
        public static string DeviceFilePath(string rootDir, string hostname, string serial)
        {
            string safeSerial = SanitizeForPath(serial);
            return Path.Combine(rootDir, "dataset", SanitizeForPath(hostname), $"FemtoBolt_{safeSerial}", SensorFileName);
        }

        /// <summary>
        /// Enumerates (serial, sensorFilePath) pairs under the recording root.
        /// </summary>
        public static IEnumerable<(string serial, string filePath)> EnumerateDevices(string rootDir)
        {
            string datasetDir = Path.Combine(rootDir, "dataset");
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

                    string sensor = Path.Combine(deviceDir, SensorFileName);
                    if (File.Exists(sensor))
                        yield return (serial, sensor);
                }
            }
        }

        private static string BuildHeaderYaml(string serial)
        {
            var sb = new StringBuilder();
            sb.Append("record_format:\n");
            sb.Append("  - name: timestamp_ns\n");
            sb.Append("    type: u64\n");
            sb.Append("    comment: Unix time in nanoseconds (wall clock on the recording host)\n");
            sb.Append("  - name: image\n");
            sb.Append("    type: u32\n");
            sb.Append("    count: 0\n");
            sb.Append("    comment: ObColorPoint[] (position float3 + color float3 per point, 24 B stride)\n");
            sb.Append("custom:\n");
            sb.Append("  camera_sensor:\n");
            sb.Append("    sensor_type: pointcloud\n");
            sb.Append("    vertex_stride: 24\n");
            sb.Append("    component_format: f32\n");
            sb.Append("    components: [x, y, z, r, g, b]\n");
            sb.Append("  device_serial: \"").Append(EscapeYamlScalar(serial)).Append("\"\n");
            sb.Append("  unity_scale: 1.0\n");
            sb.Append("  producer: FloatingVectorsICC\n");
            return sb.ToString();
        }

        private static string EscapeYamlScalar(string s)
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
