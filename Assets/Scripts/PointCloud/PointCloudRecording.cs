// ScannedReality-Studio-adjacent recording layout. For each recorded device we write
// multiple per-sensor RCSV files under
//   <root>/dataset/<hostname>/FemtoBolt_<serial>/<sensor>
// along with the top-level YAML metadata files and a calibration/extrinsics.yaml that
// describes the camera intrinsics and depth-to-color extrinsic.
//
// The sensor files we emit are:
//   - depth_main    : raw Y16 depth frames (2 bytes per pixel, width*height*2 B/record)
//   - color_main    : raw RGB8 color frames (3 bytes per pixel, width*height*3 B/record)
//   - ir_main       : raw Y16 IR frames (debug-only; not read on playback)
//   - bodies_main   : k4abt skeleton records (see RecordedBodySerializer)
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
        public const string BodiesSensorName     = "bodies_main";
        // Optional sibling of bodies_main holding offline v11s-fused bodies (same RCSV
        // payload layout). SensorRecorder.useV11sBodies selects it at Read time.
        public const string BodiesV11sSensorName = "bodies_v11s";

        /// <summary>One recorded frame for a single sensor. <see cref="Bytes"/> is the raw payload.</summary>
        public sealed class Frame
        {
            public ulong TimestampNs;
            public byte[] Bytes;

            /// <summary>
            /// Number of valid bytes in <see cref="Bytes"/>. Always prefer this over
            /// <c>Bytes.Length</c>: Frames returned by <see cref="RcsvFrameStream"/>'s
            /// indexer share a reusable scratch buffer whose <c>Length</c> grows to fit
            /// the largest record seen so far, so <c>Bytes.Length</c> ≥ <see cref="ByteCount"/>
            /// in general. For Frames produced by <see cref="ReadRcsv"/> (legacy
            /// fully-materialized path) and recording-time timestamp sentinels,
            /// <see cref="ByteCount"/> equals <c>Bytes?.Length ?? 0</c>.
            /// </summary>
            public int ByteCount;
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
                if (f == null || f.Bytes == null || f.ByteCount <= 0) continue;
                offsets.Add((ulong)fs.Position);
                bw.Write(f.TimestampNs);
                bw.Write((uint)f.ByteCount);
                bw.Write(f.Bytes, 0, f.ByteCount);
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

        /// <summary>
        /// Streaming RCSV writer. Opens the file at construction (writes magic +
        /// placeholder index offset + reserved + header), then appends one record
        /// per <see cref="WriteFrame"/> call. <see cref="Dispose"/> writes the
        /// trailing index chunk and patches the header's index offset.
        ///
        /// Use this from the live capture path so per-frame byte[] allocations
        /// can be skipped entirely: the SDK's raw buffer is handed straight to
        /// WriteFrame which copies it into the FileStream. Compared to the
        /// "buffer everything, WriteRcsv at Save" path, this drops the Gen2
        /// retained heap to near-zero and lets the OS page cache absorb the
        /// disk traffic instead of fighting GC.
        /// </summary>
        public sealed class RcsvStreamWriter : IDisposable
        {
            private readonly string _filePath;
            private FileStream _fs;
            private BinaryWriter _bw;
            private long _indexChunkOffsetPos;
            // Per-frame file offsets, finalized into the trailing index chunk
            // on Dispose. ulong is 8 B/entry — 1 hour at 30 fps = ~860 KB total,
            // a rounding error next to the ~10 GB body data.
            private readonly List<ulong> _offsets = new List<ulong>();
            private bool _disposed;

            public string FilePath => _filePath;
            public int FrameCount => _offsets.Count;

            public RcsvStreamWriter(string filePath, string headerYaml)
            {
                if (string.IsNullOrEmpty(filePath)) throw new ArgumentNullException(nameof(filePath));
                _filePath = filePath;

                Directory.CreateDirectory(Path.GetDirectoryName(filePath));
                byte[] headerBytes = Encoding.UTF8.GetBytes(headerYaml ?? string.Empty);

                // Larger write buffer (1 MB) lets several small ts/size headers
                // batch into one syscall and gives the OS more freedom to
                // coalesce writes. SequentialScan tells Windows we're appending
                // only — keeps page-cache eviction policy sensible for the
                // ~200 MB/s/stream throughput a 4-cam capture pushes.
                _fs = new FileStream(filePath, FileMode.Create, FileAccess.Write, FileShare.Read,
                                     bufferSize: 1024 * 1024,
                                     options: FileOptions.SequentialScan);
                _bw = new BinaryWriter(_fs, Encoding.UTF8, leaveOpen: false);

                _bw.Write(Encoding.ASCII.GetBytes(Magic));
                _indexChunkOffsetPos = _fs.Position;
                _bw.Write((ulong)0);                // placeholder: index chunk offset (patched on Dispose)
                _bw.Write((ulong)0);                // reserved
                _bw.Write((uint)headerBytes.Length);
                _bw.Write(headerBytes);
            }

            /// <summary>
            /// Append one record. <paramref name="bytes"/> + <paramref name="byteCount"/>
            /// follow the byte[]/length pair convention so callers can pass the SDK's
            /// pre-allocated raw buffer without slicing it. No copy of <paramref name="bytes"/>
            /// is retained after the call returns; the FileStream takes ownership of the data.
            /// </summary>
            public void WriteFrame(ulong timestampNs, byte[] bytes, int byteCount)
            {
                if (_disposed) throw new ObjectDisposedException(nameof(RcsvStreamWriter));
                if (bytes == null || byteCount <= 0) return;
                _offsets.Add((ulong)_fs.Position);
                _bw.Write(timestampNs);
                _bw.Write((uint)byteCount);
                _bw.Write(bytes, 0, byteCount);
            }

            public void Dispose()
            {
                if (_disposed) return;
                _disposed = true;
                if (_bw == null) return;
                try
                {
                    // Trailing index chunk: count + per-record offsets + sentinel
                    // end-of-data offset. The sentinel equals the position right
                    // after the last record's bytes (== indexChunkOffset, since
                    // the index chunk starts immediately after data). This
                    // matches WriteRcsv's offsets[count] convention so the
                    // reader's (validRecordCount + 1) -slot offsets array fills
                    // exactly the same way.
                    ulong endOfDataPos = (ulong)_fs.Position;
                    _bw.Write((ulong)_offsets.Count);
                    for (int i = 0; i < _offsets.Count; i++) _bw.Write(_offsets[i]);
                    _bw.Write(endOfDataPos);
                    _bw.Flush();

                    _fs.Position = _indexChunkOffsetPos;
                    _bw.Write(endOfDataPos);
                }
                finally
                {
                    _bw.Dispose();
                    _bw = null;
                    _fs = null;
                }
            }
        }

        /// <summary>
        /// Lazy, file-backed view over an RCSV's records. Open the file once via
        /// <see cref="RcsvFrameStream"/>, then index into it as if it were a
        /// <c>List&lt;Frame&gt;</c>: only the requested frame's bytes are read from
        /// disk (timestamps + offsets are loaded eagerly into small arrays so
        /// <see cref="Count"/>, timestamp scans, and seek planning are O(1)).
        ///
        /// Compared to <see cref="ReadRcsv"/> which fully materializes every
        /// frame's payload into managed byte[] upfront, this keeps multi-GB
        /// recordings out of the managed heap. Each indexer call allocates a
        /// fresh byte[] for the record — callers should not assume payload
        /// identity across repeated accesses.
        ///
        /// Thread-safety: the indexer mutates the underlying FileStream's
        /// position, so concurrent reads are unsafe. Single-threaded sequential
        /// or random access is fine.
        /// </summary>
        public sealed class RcsvFrameStream : IReadOnlyList<Frame>, IDisposable
        {
            private readonly string _filePath;
            // _fs / _br: 1 MB-buffered handle used by the indexer to pull full
            // record payloads (the playback path reads records contiguously, so
            // a large buffer + SequentialScan amortizes the disk hit).
            private FileStream _fs;
            private BinaryReader _br;
            // _tsFs / _tsBr: 1 B-buffered handle dedicated to scattered 8-B
            // timestamp reads. Sharing the main handle's 1 MB buffer would
            // force a 1 MB refill per ts peek (records are typically 700 KB+
            // apart, so consecutive ts offsets land in different buffer pages),
            // turning a few-MB ts walk into a few-GB read.
            private FileStream _tsFs;
            private BinaryReader _tsBr;
            // offsets[i] is the absolute file position of record i's first byte
            // (the u64 timestamp). offsets[Count] is the end-of-data sentinel
            // (== index chunk start). Loaded once on construction.
            private readonly ulong[] _offsets;
            // Lazy timestamp cache. Populated on first TimestampNsAt(i) call.
            // Eager pre-load is tempting (only 8 B × N total) but the N
            // scattered syscalls add up to seconds on multi-cam recordings.
            // The playback Update loop reads ts monotonically (cursor+1 each
            // tick) so every ts is fetched exactly once across the session
            // anyway — pre-loading just front-loads that cost into Play startup.
            private readonly ulong[] _timestamps;
            private readonly bool[] _tsLoaded;
            // Reusable payload buffer shared by every indexer access. Grows
            // on demand (next-pow-2) to fit the largest record so far, never
            // shrinks. The indexer hands this buffer back wrapped in a Frame
            // whose ByteCount marks the valid prefix; callers MUST treat the
            // tail bytes (ByteCount..Length) as undefined and MUST consume
            // the Frame before the next indexer call on this stream.
            // Playback paths (SensorRecorder.Update, BodyTrackingPlayback.
            // ProcessCoroutine) read each Frame synchronously within one tick,
            // so the reuse is safe; the contract is documented on Frame.ByteCount.
            private byte[] _scratch = Array.Empty<byte>();
            private bool _disposed;

            public string FilePath => _filePath;
            public int Count => _timestamps.Length;

            public RcsvFrameStream(string filePath)
            {
                if (filePath == null) throw new ArgumentNullException(nameof(filePath));
                _filePath = filePath;

                // SequentialScan is a slight lie (we may seek backwards on
                // random access from the Editor), but the common cases
                // (BodyTrackingPlayback's i++ loop, recorder Update's
                // PlaybackCursor advance) ARE sequential, and the hint nudges
                // Windows to read ahead which is what we want.
                _fs = new FileStream(filePath, FileMode.Open, FileAccess.Read, FileShare.Read,
                                     bufferSize: 1024 * 1024,
                                     options: FileOptions.SequentialScan);
                _br = new BinaryReader(_fs, Encoding.UTF8, leaveOpen: false);

                var magic = Encoding.ASCII.GetString(_br.ReadBytes(4));
                if (magic != Magic)
                {
                    Dispose();
                    throw new InvalidDataException($"Not an RCSV file: magic '{magic}' at {filePath}");
                }
                ulong indexChunkOffset = _br.ReadUInt64();
                _br.ReadUInt64(); // reserved
                uint headerTextSize = _br.ReadUInt32();
                _fs.Seek(headerTextSize, SeekOrigin.Current); // skip header text

                if (indexChunkOffset == 0 || indexChunkOffset >= (ulong)_fs.Length)
                {
                    Dispose();
                    throw new InvalidDataException($"Invalid index chunk offset in {filePath}");
                }
                _fs.Seek((long)indexChunkOffset, SeekOrigin.Begin);
                ulong validRecordCount = _br.ReadUInt64();
                _offsets = new ulong[validRecordCount + 1];
                for (ulong i = 0; i <= validRecordCount; i++) _offsets[i] = _br.ReadUInt64();

                _timestamps = new ulong[validRecordCount];
                _tsLoaded = new bool[validRecordCount];

                // Long-lived small-buffer + RandomAccess handle for ts peeks.
                // bufferSize:1 disables read-ahead so each ReadUInt64 transfers
                // ~8 B (rounded to one disk sector by the OS page cache layer)
                // instead of refilling the main stream's 1 MB buffer.
                _tsFs = new FileStream(filePath, FileMode.Open, FileAccess.Read, FileShare.Read,
                                       bufferSize: 1,
                                       options: FileOptions.RandomAccess);
                _tsBr = new BinaryReader(_tsFs, Encoding.UTF8, leaveOpen: false);
            }

            public Frame this[int index]
            {
                get
                {
                    if (_disposed) throw new ObjectDisposedException(nameof(RcsvFrameStream));
                    if ((uint)index >= (uint)_timestamps.Length)
                        throw new ArgumentOutOfRangeException(nameof(index));
                    _fs.Seek((long)_offsets[index], SeekOrigin.Begin);
                    ulong ts = _br.ReadUInt64();
                    uint size = _br.ReadUInt32();
                    // Grow scratch on the first record that exceeds current
                    // capacity. Round up to the next power of two so a noisy
                    // mix of slightly-larger records doesn't trigger a fresh
                    // alloc on every other access.
                    if (_scratch.Length < (int)size)
                    {
                        int cap = _scratch.Length == 0 ? 1024 : _scratch.Length;
                        while (cap < (int)size) cap <<= 1;
                        _scratch = new byte[cap];
                    }
                    // Loop until the requested count is fully read — FileStream.Read
                    // is allowed to return fewer bytes than asked (especially on
                    // platforms where the underlying handle is non-blocking or the
                    // OS chooses to chunk a large read), and BinaryReader.ReadBytes
                    // does this loop internally. Replicating it here keeps the
                    // short-read path well-defined (throw on real EOF, retry otherwise).
                    int total = 0;
                    while (total < (int)size)
                    {
                        int n = _fs.Read(_scratch, total, (int)size - total);
                        if (n == 0) break;
                        total += n;
                    }
                    if (total != (int)size)
                        throw new EndOfStreamException($"Truncated record {index} in {_filePath}");
                    // Backfill the ts cache as a free side effect — a subsequent
                    // TimestampNsAt(index) call returns without a disk hit.
                    _timestamps[index] = ts;
                    _tsLoaded[index] = true;
                    return new Frame { TimestampNs = ts, Bytes = _scratch, ByteCount = (int)size };
                }
            }

            /// <summary>
            /// Cheap timestamp lookup. First call per index reads 8 B from the
            /// dedicated small-buffer FileStream; subsequent calls return the
            /// cached value. The playback Update loop and BodyTracking pass
            /// both walk timestamps monotonically, so the total disk traffic
            /// across a playback session equals the eager-preload cost but
            /// is amortized over playback time instead of blocking Play startup.
            /// </summary>
            public ulong TimestampNsAt(int index)
            {
                if (_disposed) throw new ObjectDisposedException(nameof(RcsvFrameStream));
                if ((uint)index >= (uint)_timestamps.Length)
                    throw new ArgumentOutOfRangeException(nameof(index));
                if (!_tsLoaded[index])
                {
                    _tsFs.Seek((long)_offsets[index], SeekOrigin.Begin);
                    _timestamps[index] = _tsBr.ReadUInt64();
                    _tsLoaded[index] = true;
                }
                return _timestamps[index];
            }

            public IEnumerator<Frame> GetEnumerator()
            {
                for (int i = 0; i < _timestamps.Length; i++) yield return this[i];
            }

            System.Collections.IEnumerator System.Collections.IEnumerable.GetEnumerator() => GetEnumerator();

            public void Dispose()
            {
                if (_disposed) return;
                _disposed = true;
                _br?.Dispose();
                _br = null;
                _fs = null;
                _tsBr?.Dispose();
                _tsBr = null;
                _tsFs = null;
            }
        }

        /// <summary>
        /// Read an RCSV file, returning its records in order. Loads the entire
        /// payload into memory; for large recordings prefer <see cref="RcsvFrameStream"/>
        /// which keeps the bytes on disk until indexed.
        /// </summary>
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
                frames.Add(new Frame { TimestampNs = ts, Bytes = bytes, ByteCount = (int)size });
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
        /// Resolves a configured recording root to an absolute path:
        /// empty/whitespace → <c>persistentDataPath/Recordings/recording</c>,
        /// relative → under <c>persistentDataPath</c>, absolute → as-is.
        /// On macOS (editor or standalone) a non-blank <paramref name="macOverride"/>
        /// replaces <paramref name="configured"/> before resolution, so a Mac
        /// checkout can point at e.g. a local Dropbox mount without touching
        /// the canonical configured path.
        /// </summary>
        public static string ResolveRecordingRoot(string configured, string macOverride = null)
        {
            string p = configured;
#if UNITY_EDITOR_OSX || UNITY_STANDALONE_OSX
            if (!string.IsNullOrWhiteSpace(macOverride))
                p = macOverride;
#endif
            if (string.IsNullOrWhiteSpace(p))
                p = Path.Combine(UnityEngine.Application.persistentDataPath, "Recordings", "recording");
            else if (!Path.IsPathRooted(p))
                p = Path.Combine(UnityEngine.Application.persistentDataPath, p);
            return p;
        }

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

        public static string BuildBodiesHeaderYaml(string serial)
        {
            // bodies_main keeps the same k4abt body record layout the worker MMF uses, so
            // the in-memory BodySnapshot[] can be blit-encoded record-by-record without
            // touching managed structs per joint. Layout is documented here for human
            // readers; decoders rely on BodyTracking.RecordedBodySerializer.
            var sb = new StringBuilder();
            sb.Append("record_format:\n");
            sb.Append("  - name: timestamp_ns\n");
            sb.Append("    type: u64\n");
            sb.Append("    comment: depth-frame timestamp the bodies were tracked from\n");
            sb.Append("  - name: payload\n");
            sb.Append("    type: bytes\n");
            sb.Append("    comment: u32 body_count + body_count * (u32 id + u32 reserved + 32 joints × {f32 posX, f32 posY, f32 posZ (mm), f32 quatW, f32 quatX, f32 quatY, f32 quatZ, u32 confidence})\n");
            sb.Append("custom:\n");
            sb.Append("  sensor:\n");
            sb.Append("    sensor_type: bodies\n");
            sb.Append("    format: k4abt_bodies_v1\n");
            sb.Append("    joint_count: 32\n");
            sb.Append("    joint_record_bytes: 32\n");
            sb.Append("    body_record_bytes: 1032\n");
            sb.Append("    max_bodies_per_frame: 6\n");
            sb.Append($"  device_serial: \"{EscapeYaml(serial)}\"\n");
            sb.Append("  timestamp_basis: device_synced_unix_like_ns\n");
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

        // ------------------------------------------------------------------
        // Camera-id map (calibration/cameras.yaml)
        // ------------------------------------------------------------------

        /// <summary>
        /// Writes <c>calibration/cameras.yaml</c> — the stable camera-id ↔ device-serial
        /// map. The list order IS the id (entry 0 = id 0). <c>origin_serial</c> designates
        /// the camera pinned to the world origin (cam0) during extrinsic calibration and is
        /// independent of id ordering. Devices that are temporarily disconnected are kept in
        /// the file so their id survives a replug.
        /// </summary>
        public static void WriteCamerasYaml(string rootDir, CameraIdMap map)
        {
            if (map == null) return;
            Directory.CreateDirectory(CalibrationDir(rootDir));
            string path = Path.Combine(CalibrationDir(rootDir), "cameras.yaml");

            var sb = new StringBuilder();
            sb.Append("# Generated by FloatingVectorsICC. Stable camera-id <-> device-serial map.\n");
            sb.Append("# List order is the id (first entry = id 0). 'origin_serial' designates the\n");
            sb.Append("# camera pinned to the world origin (cam0) during extrinsic calibration; it is\n");
            sb.Append("# independent of id ordering.\n");
            if (!string.IsNullOrEmpty(map.OriginSerial))
                sb.Append($"origin_serial: \"{EscapeYaml(map.OriginSerial)}\"\n");
            sb.Append("cameras:\n");
            if (map.Cameras != null)
            {
                for (int i = 0; i < map.Cameras.Count; i++)
                {
                    var c = map.Cameras[i];
                    sb.Append($"  - id: {i}\n");
                    sb.Append($"    serial: \"{EscapeYaml(c.Serial)}\"\n");
                    if (!string.IsNullOrEmpty(c.Label))
                        sb.Append($"    label: \"{EscapeYaml(c.Label)}\"\n");
                }
            }
            File.WriteAllText(path, sb.ToString(), new UTF8Encoding(false));
        }

        /// <summary>
        /// Reads <c>calibration/cameras.yaml</c>. Returns <c>null</c> when the file does not
        /// exist (the id map is optional). Entries come back ordered by their stored <c>id</c>.
        /// Throws on parse errors or an unsafe <c>serial</c> (path traversal).
        /// </summary>
        public static CameraIdMap ReadCamerasYaml(string rootDir)
        {
            if (string.IsNullOrEmpty(rootDir)) throw new ArgumentException("rootDir is empty", nameof(rootDir));
            string path = Path.Combine(CalibrationDir(rootDir), "cameras.yaml");
            if (!File.Exists(path)) return null;

            var stream = new YamlStream();
            try
            {
                using var reader = new StringReader(File.ReadAllText(path));
                stream.Load(reader);
            }
            catch (YamlException e)
            {
                throw new InvalidDataException($"cameras.yaml parse error: {e.Message}", e);
            }
            var map = new CameraIdMap();
            if (stream.Documents.Count == 0) return map;
            if (!(stream.Documents[0].RootNode is YamlMappingNode root))
                throw new InvalidDataException("cameras.yaml: root must be a mapping");

            if (root.Children.TryGetValue(new YamlScalarNode("origin_serial"), out var originNode)
                && originNode is YamlScalarNode originScalar)
            {
                map.OriginSerial = originScalar.Value ?? string.Empty;
            }

            if (root.Children.TryGetValue(new YamlScalarNode("cameras"), out var camerasObj)
                && camerasObj is YamlSequenceNode camerasSeq)
            {
                var withId = new List<(int id, CameraIdEntry entry)>(camerasSeq.Children.Count);
                int idx = 0;
                foreach (var camNode in camerasSeq.Children)
                {
                    if (!(camNode is YamlMappingNode cam))
                        throw new InvalidDataException($"cameras[{idx}]: entry must be a mapping");
                    string serial = ReadScalarString(cam, "serial", $"cameras[{idx}]");
                    if (serial.Contains("..") || serial.Contains('/') || serial.Contains('\\'))
                        throw new InvalidDataException(
                            $"cameras[{idx}]: unsafe serial '{serial}' (path traversal)");
                    int id = ReadScalarInt(cam, "id", $"cameras[{idx}]");
                    string label = cam.Children.ContainsKey(new YamlScalarNode("label"))
                        ? ReadScalarString(cam, "label", $"cameras[{idx}]")
                        : string.Empty;
                    withId.Add((id, new CameraIdEntry { Serial = serial, Label = label }));
                    idx++;
                }
                withId.Sort((a, b) => a.id.CompareTo(b.id));
                foreach (var w in withId) map.Cameras.Add(w.entry);
            }
            return map;
        }

        /// <summary>
        /// Rig serial order for world rebase / sensing area. <c>calibration/cameras.yaml</c>
        /// under <paramref name="rootDir"/> wins (list order = camera id 0..3) because it is
        /// machine- or recording-local; the scene-serialized fallback gets git-synced between
        /// the two rigs ("4070"/"5080") and therefore lists the wrong set's serials on one of
        /// them. Falls back when the yaml is absent, unreadable, or does not hold exactly
        /// 4 distinct serials. <paramref name="source"/> names the winner for log messages.
        /// </summary>
        public static string[] ResolveRigSerialOrder(string rootDir, string[] sceneFallback, out string source)
        {
            try
            {
                var map = ReadCamerasYaml(rootDir);
                if (map?.Cameras != null)
                {
                    var serials = new List<string>(4);
                    foreach (var c in map.Cameras)
                        if (c != null && !string.IsNullOrEmpty(c.Serial) && !serials.Contains(c.Serial))
                            serials.Add(c.Serial);
                    if (serials.Count == 4)
                    {
                        source = "cameras.yaml";
                        return serials.ToArray();
                    }
                    source = $"scene rigSerialOrder (cameras.yaml has {serials.Count} serial(s), need 4)";
                    return sceneFallback;
                }
            }
            catch (Exception e)
            {
                source = $"scene rigSerialOrder (cameras.yaml unreadable: {e.Message})";
                return sceneFallback;
            }
            source = "scene rigSerialOrder (no cameras.yaml)";
            return sceneFallback;
        }

        /// <summary>
        /// Writes <c>calibration/floor.yaml</c> — the operator-tuned floor height
        /// (SensorManager.rebaseFloorY). Kept out of the scene on purpose: it is a
        /// per-room, per-rig measurement, so it must not travel between the two
        /// camera sets via git like a serialized Inspector value would.
        /// </summary>
        public static void WriteFloorY(string rootDir, float floorY)
        {
            Directory.CreateDirectory(CalibrationDir(rootDir));
            string path = Path.Combine(CalibrationDir(rootDir), "floor.yaml");
            var sb = new StringBuilder();
            sb.Append("# Generated by FloatingVectorsICC. Operator-tuned floor height for this\n");
            sb.Append("# room/rig: the calibration-frame Y that becomes world y=0.\n");
            sb.Append($"rebase_floor_y: {F(floorY)}\n");
            File.WriteAllText(path, sb.ToString(), new UTF8Encoding(false));
        }

        /// <summary>
        /// Reads <c>calibration/floor.yaml</c>. Returns false (leaving
        /// <paramref name="floorY"/> at 0) when the file is missing or unparsable —
        /// callers keep their configured value.
        /// </summary>
        public static bool TryReadFloorY(string rootDir, out float floorY)
        {
            floorY = 0f;
            if (string.IsNullOrEmpty(rootDir)) return false;
            string path = Path.Combine(CalibrationDir(rootDir), "floor.yaml");
            if (!File.Exists(path)) return false;
            try
            {
                var stream = new YamlStream();
                using var reader = new StringReader(File.ReadAllText(path));
                stream.Load(reader);
                if (stream.Documents.Count == 0) return false;
                if (!(stream.Documents[0].RootNode is YamlMappingNode root)) return false;
                if (!root.Children.TryGetValue(new YamlScalarNode("rebase_floor_y"), out var v)) return false;
                if (!(v is YamlScalarNode s)) return false;
                if (!float.TryParse(s.Value, NumberStyles.Float, CultureInfo.InvariantCulture, out floorY))
                    return false;
                return float.IsFinite(floorY) && Math.Abs(floorY) < 100f;
            }
            catch (Exception)
            {
                return false;
            }
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
            // Legacy fix-up: recordings made before the OrbbecNative enum was
            // corrected wrote "KannalaBrandt4" for native value 4, which is
            // actually BrownConradyK6 (the enum was missing that entry, so every
            // model >= 4 was off by one). The Femto Bolt depth is never a true
            // fisheye, so map the legacy label to its real model.
            if (string.Equals(modelStr, "KannalaBrandt4", StringComparison.OrdinalIgnoreCase))
                modelStr = nameof(ObCameraDistortionModel.BrownConradyK6);
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

        /// <summary>Stable camera-id ↔ serial map (calibration/cameras.yaml).</summary>
        public sealed class CameraIdMap
        {
            /// <summary>Ordered camera entries; the list index IS the id (entry 0 = id 0).</summary>
            public List<CameraIdEntry> Cameras = new List<CameraIdEntry>();
            /// <summary>Serial of the camera pinned to the world origin (cam0). Empty = none chosen.</summary>
            public string OriginSerial = string.Empty;
        }

        public sealed class CameraIdEntry
        {
            public string Serial;
            public string Label = string.Empty;
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
