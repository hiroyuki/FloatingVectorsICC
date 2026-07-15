// Deterministic RCSV replay for the evaluation harness.
//
// Reads a recorded session (the project's Scanned-Reality-style RCSV layout)
// directly via PointCloudRecording.RcsvFrameStream — independent of
// SensorRecorder's editor Play/Stop state — and feeds depth+color+IR frames
// (plus the recorded bodies_main payload) to subscribers. Depth is the master
// timeline per device; color / IR / bodies are matched to the nearest depth
// frame by timestamp.
//
// Two play modes:
//   Realtime — paced by wall clock * speed against recorded timestamps (for viz).
//   Batch    — one frame per device per Update, as fast as possible, giving each
//              async adapter one Update to Pump between frames (for metric runs).

using System;
using System.Collections.Generic;
using System.IO;
using Orbbec;
using PointCloud;
using UnityEngine;

namespace BodyTracking.Eval
{
    public sealed class EvalReplayDriver : MonoBehaviour
    {
        public enum Mode { Realtime, Batch }

        [Tooltip("Session directory containing dataset/ and calibration/ (the RCSV root).")]
        public string sessionRoot = "";
        public Mode mode = Mode.Batch;
        [Range(0.05f, 4f)] public float speed = 1f;
        public bool loop = true;
        public bool autoPlayOnStart = false;

        [Tooltip("Max time gap (ms) for matching a recorded bodies frame to a depth frame. " +
                 "Beyond this the depth frame is treated as having no body, so tracking gaps " +
                 "register in the continuity/occlusion metric. Ignored when bodies is the master timeline.")]
        public float bodyMatchSkewMs = 40f;

        public event Action<string, RawFrameData, ObCameraParam?, ulong> OnFrame;
        public event Action<string, byte[], int, ulong> OnRecordedBodies;
        public event Action OnLoaded;
        public event Action OnLoopComplete;

        public bool IsLoaded => _devices.Count > 0;
        public bool IsPlaying { get; private set; }
        public IReadOnlyList<Device> Devices => _devices;

        public sealed class Device : IDisposable
        {
            public string Serial;
            public PointCloudRecording.RcsvFrameStream Depth, Color, IR, Bodies;
            public int DepthW, DepthH, ColorW, ColorH, IRW, IRH;
            public ObCameraParam? CameraParam;

            public ulong[] MasterTs;     // depth timeline, or bodies timeline for baseline-only recordings
            public int[] NearestColor;   // per master index
            public int[] NearestIR;
            public int[] NearestBodies;

            public byte[] DepthBuf, ColorBuf, IRBuf, BodiesBuf;
            public int Cursor;

            public void Dispose()
            {
                Depth?.Dispose(); Color?.Dispose(); IR?.Dispose(); Bodies?.Dispose();
            }
        }

        private readonly List<Device> _devices = new List<Device>();
        private double _elapsedNs;
        private ulong _t0Ns;

        private void Start()
        {
            if (autoPlayOnStart && !string.IsNullOrEmpty(sessionRoot))
            {
                if (Load(sessionRoot)) Play();
            }
        }

        private void OnDestroy() => Unload();

        // ------------------------------------------------------------------

        public bool Load(string root)
        {
            Unload();
            sessionRoot = root;
            if (string.IsNullOrWhiteSpace(root) || !Directory.Exists(PointCloudRecording.DatasetRoot(root)))
            {
                Debug.LogError($"[EvalReplay] no dataset under '{root}'");
                return false;
            }

            IReadOnlyList<PointCloudRecording.DeviceCalibration> calib = null;
            try { calib = PointCloudRecording.ReadExtrinsicsYaml(root); }
            catch (Exception e) { Debug.LogWarning($"[EvalReplay] no/invalid extrinsics.yaml: {e.Message}"); }

            foreach (var (serial, deviceDir) in PointCloudRecording.EnumerateDevices(root))
            {
                string depthPath = Path.Combine(deviceDir, PointCloudRecording.DepthSensorName);

                var dev = new Device { Serial = serial };
                dev.Depth = OpenIfExists(depthPath);   // may be null (baseline-only recordings have no depth)
                dev.Color = OpenIfExists(Path.Combine(deviceDir, PointCloudRecording.ColorSensorName));
                dev.IR = OpenIfExists(Path.Combine(deviceDir, PointCloudRecording.IRSensorName));
                dev.Bodies = OpenIfExists(Path.Combine(deviceDir, PointCloudRecording.BodiesSensorName));

                // Master timeline: depth when present, else bodies (a bodies-only
                // recording still yields a valid k4abt baseline stream).
                var master = dev.Depth ?? dev.Bodies;
                if (master == null || master.Count == 0) { dev.Dispose(); continue; }

                if (dev.Depth != null) (dev.DepthW, dev.DepthH) = ReadDims(depthPath);
                if (dev.Color != null) (dev.ColorW, dev.ColorH) = ReadDims(Path.Combine(deviceDir, PointCloudRecording.ColorSensorName));
                if (dev.IR != null) (dev.IRW, dev.IRH) = ReadDims(Path.Combine(deviceDir, PointCloudRecording.IRSensorName));

                dev.MasterTs = TimestampArray(master);
                // Color / IR are per-frame streams (~depth rate): nearest match, no skew limit.
                dev.NearestColor = NearestMap(dev.MasterTs, dev.Color, ulong.MaxValue);
                dev.NearestIR = NearestMap(dev.MasterTs, dev.IR, ulong.MaxValue);
                // Bodies: bound the match so a depth frame with no body near it in time
                // registers as a tracking gap. When bodies IS the master, index maps to itself.
                ulong bodySkewNs = (ulong)Math.Max(0f, bodyMatchSkewMs) * 1_000_000UL;
                dev.NearestBodies = (dev.Depth != null)
                    ? NearestMap(dev.MasterTs, dev.Bodies, bodySkewNs)
                    : IdentityMap(dev.MasterTs.Length, dev.Bodies);

                // Buffers grow lazily in CopyFrame — avoids reading a (possibly
                // truncated) payload at load time just to size them.
                dev.DepthBuf = Array.Empty<byte>();
                dev.ColorBuf = Array.Empty<byte>();
                dev.IRBuf = Array.Empty<byte>();
                dev.BodiesBuf = Array.Empty<byte>();

                dev.CameraParam = BuildCameraParam(calib, serial);
                _devices.Add(dev);
            }

            if (_devices.Count == 0)
            {
                Debug.LogError($"[EvalReplay] no playable devices under '{root}'");
                return false;
            }

            _t0Ns = ulong.MaxValue;
            foreach (var d in _devices) if (d.MasterTs.Length > 0 && d.MasterTs[0] < _t0Ns) _t0Ns = d.MasterTs[0];
            ResetPlayhead();
            Debug.Log($"[EvalReplay] loaded {_devices.Count} device(s) from '{root}'");
            OnLoaded?.Invoke();
            return true;
        }

        public void Unload()
        {
            foreach (var d in _devices) d.Dispose();
            _devices.Clear();
            IsPlaying = false;
        }

        public void Play() { if (IsLoaded) IsPlaying = true; }
        public void Pause() => IsPlaying = false;

        public void ResetPlayhead()
        {
            _elapsedNs = 0;
            foreach (var d in _devices) d.Cursor = 0;
        }

        /// <summary>
        /// Emit every frame across all devices in global timestamp order, synchronously,
        /// ignoring the Update clock. For headless / batch metric runs where each adapter
        /// processes frames synchronously. Raises OnLoopComplete once at the end.
        /// </summary>
        public void RunToEndSync()
        {
            if (_devices.Count == 0) return;
            var order = new List<(ulong ts, int dev, int idx)>();
            for (int d = 0; d < _devices.Count; d++)
            {
                var dev = _devices[d];
                for (int i = 0; i < dev.MasterTs.Length; i++) order.Add((dev.MasterTs[i], d, i));
            }
            order.Sort((a, b) => a.ts.CompareTo(b.ts));
            foreach (var o in order) EmitFrame(_devices[o.dev], o.idx);
            OnLoopComplete?.Invoke();
        }

        private void Update()
        {
            if (!IsPlaying || _devices.Count == 0) return;

            if (mode == Mode.Batch)
            {
                bool anyEmitted = false;
                foreach (var d in _devices)
                {
                    if (d.Cursor < d.MasterTs.Length) { EmitFrame(d, d.Cursor); d.Cursor++; anyEmitted = true; }
                }
                if (!anyEmitted) HandleEnd();
                return;
            }

            // Realtime
            _elapsedNs += Time.deltaTime * speed * 1e9;
            bool progressed = false;
            foreach (var d in _devices)
            {
                while (d.Cursor < d.MasterTs.Length && (d.MasterTs[d.Cursor] - _t0Ns) <= (ulong)Math.Max(0, _elapsedNs))
                {
                    EmitFrame(d, d.Cursor);
                    d.Cursor++;
                    progressed = true;
                }
            }
            if (!progressed && AllAtEnd()) HandleEnd();
        }

        private bool AllAtEnd()
        {
            foreach (var d in _devices) if (d.Cursor < d.MasterTs.Length) return false;
            return true;
        }

        private void HandleEnd()
        {
            OnLoopComplete?.Invoke();
            if (loop) ResetPlayhead();
            else IsPlaying = false;
        }

        private void EmitFrame(Device d, int i)
        {
            ulong tsNs = d.MasterTs[i];

            int depthCount = d.Depth != null ? CopyFrame(d.Depth, i, ref d.DepthBuf) : 0;
            int colorCount = 0, irCount = 0;
            if (d.Color != null && d.NearestColor[i] >= 0) colorCount = CopyFrame(d.Color, d.NearestColor[i], ref d.ColorBuf);
            if (d.IR != null && d.NearestIR[i] >= 0) irCount = CopyFrame(d.IR, d.NearestIR[i], ref d.IRBuf);

            var raw = new RawFrameData(
                d.DepthBuf, depthCount, d.DepthW, d.DepthH,
                d.ColorBuf, colorCount, d.ColorW, d.ColorH,
                d.IRBuf, irCount, d.IRW, d.IRH,
                tsNs / 1000UL);
            OnFrame?.Invoke(d.Serial, raw, d.CameraParam, tsNs);

            if (d.Bodies != null && d.NearestBodies[i] >= 0)
            {
                int bc = CopyFrame(d.Bodies, d.NearestBodies[i], ref d.BodiesBuf);
                OnRecordedBodies?.Invoke(d.Serial, d.BodiesBuf, bc, tsNs);
            }
        }

        // ------------------------------------------------------------------
        // helpers

        private static PointCloudRecording.RcsvFrameStream OpenIfExists(string path)
        {
            if (!File.Exists(path)) return null;
            try
            {
                var s = new PointCloudRecording.RcsvFrameStream(path);
                return s.Count > 0 ? s : DisposeAndNull(s);
            }
            catch { return null; }
        }

        private static PointCloudRecording.RcsvFrameStream DisposeAndNull(PointCloudRecording.RcsvFrameStream s)
        {
            s.Dispose();
            return null;
        }

        /// <summary>Identity map (index i -> i) when a stream is itself the master timeline.</summary>
        private static int[] IdentityMap(int length, PointCloudRecording.RcsvFrameStream stream)
        {
            var map = new int[length];
            for (int i = 0; i < length; i++) map[i] = (stream != null && i < stream.Count) ? i : -1;
            return map;
        }

        private static (int, int) ReadDims(string path)
        {
            try { return PointCloudRecording.ReadRcsvHeaderDimensions(path); }
            catch { return (0, 0); }
        }

        private static ulong[] TimestampArray(PointCloudRecording.RcsvFrameStream s)
        {
            var a = new ulong[s.Count];
            for (int i = 0; i < a.Length; i++) a[i] = s.TimestampNsAt(i);
            return a;
        }

        /// <summary>
        /// Copy stream[index] payload into buf (growing it if needed); returns byte count.
        /// Returns 0 (and warns once) if the record is unreadable/truncated — real
        /// recordings can be cut off mid-frame, and one bad frame must not kill the run.
        /// </summary>
        private static int CopyFrame(PointCloudRecording.RcsvFrameStream s, int index, ref byte[] buf)
        {
            try
            {
                var f = s[index];
                if (buf == null || buf.Length < f.ByteCount) buf = new byte[Math.Max(1, f.ByteCount)];
                Buffer.BlockCopy(f.Bytes, 0, buf, 0, f.ByteCount);
                return f.ByteCount;
            }
            catch (Exception e)
            {
                Debug.LogWarning($"[EvalReplay] skipping unreadable frame {index} in {Path.GetFileName(s.FilePath)}: {e.Message}");
                return 0;
            }
        }

        /// <summary>
        /// For each master ts, the nearest index in the (sorted) other stream, or -1
        /// if none exists within <paramref name="maxSkewNs"/> (use ulong.MaxValue for no limit).
        /// </summary>
        private static int[] NearestMap(ulong[] masterTs, PointCloudRecording.RcsvFrameStream other, ulong maxSkewNs)
        {
            var map = new int[masterTs.Length];
            if (other == null || other.Count == 0)
            {
                for (int i = 0; i < map.Length; i++) map[i] = -1;
                return map;
            }
            int n = other.Count;
            int j = 0;
            for (int i = 0; i < masterTs.Length; i++)
            {
                ulong t = masterTs[i];
                while (j + 1 < n && AbsDiff(other.TimestampNsAt(j + 1), t) <= AbsDiff(other.TimestampNsAt(j), t)) j++;
                map[i] = AbsDiff(other.TimestampNsAt(j), t) <= maxSkewNs ? j : -1;
            }
            return map;
        }

        private static ulong AbsDiff(ulong a, ulong b) => a > b ? a - b : b - a;

        private static ObCameraParam? BuildCameraParam(
            IReadOnlyList<PointCloudRecording.DeviceCalibration> calib, string serial)
        {
            if (calib == null) return null;
            foreach (var c in calib)
            {
                if (!string.Equals(c.Serial, serial, StringComparison.OrdinalIgnoreCase)) continue;
                return new ObCameraParam
                {
                    DepthIntrinsic = c.DepthIntrinsic,
                    RgbIntrinsic = c.ColorIntrinsic,
                    DepthDistortion = c.DepthDistortion,
                    RgbDistortion = c.ColorDistortion,
                    Transform = c.DepthToColor,
                    IsMirrored = false,
                };
            }
            return null;
        }
    }
}
