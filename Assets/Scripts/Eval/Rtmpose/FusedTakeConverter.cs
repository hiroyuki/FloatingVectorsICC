// Runtime v11s conversion of one recorded take: run the RTMPose fused
// pipeline (FusedRtmposeAdapter — the same chain FusedBodiesExport drives in
// the editor) over the take's depth+color streams on a BACKGROUND thread and
// replace its bodies_main with the fused skeleton, finishing with the
// FusedCatchupSmooth post-pass ("v11s" = v11 fusion + catch-up smoothing).
//
// Built for the visitor experience flow: Explore records ~10s, Processing
// runs this converter (director polls Progress for the progress bar), Watch
// plays the take back through the recorded-bodies path
// (SkeletonMerger.ignoreRecordedBodies = false).
//
// Output contract — IN PLACE with a rename dance, so the playback root stays
// the recording root and failure always leaves a playable take:
//   1. Fused bodies stream into bodies_main.v11s next to each device's
//      bodies_main (the open k4abt file is never touched).
//   2. On success: bodies_main -> bodies_main.k4abt (backup),
//      bodies_main.v11s -> bodies_main, then FusedCatchupSmooth.Run
//      (which makes its own bodies_main.prekink backup).
//   3. On failure / abort: the .v11s temps are deleted; the original k4abt
//      bodies_main is untouched -> caller falls back to k4abt playback.
//
// Threading: everything (including OrtRtmposeBackend construction — seconds
// of ONNX session build) runs on one worker thread; the main thread only
// polls Status/Progress. The adapter chain is Unity-API-free (see
// LiveFusedBodySource header); this class only uses Debug.Log* (thread-safe)
// and Vector3 math off the main thread.

using System;
using System.Collections.Generic;
using System.IO;
using System.Threading;
using Orbbec;
using PointCloud;
using UnityEngine;

namespace BodyTracking.Eval.Rtmpose
{
    public sealed class FusedTakeConverter
    {
        public enum ConvertStatus { Idle, Running, Done, Failed, Aborted }

        public sealed class Options
        {
            /// <summary>Folder with yolox-m/ and rtmpose-m/ ONNX models, project-root relative
            /// (or absolute).</summary>
            public string ModelsDir = "eval/models";
            /// <summary>Bone-length profile JSON, project-root relative (or absolute).
            /// Empty/missing = fusion runs without bone-length priors.</summary>
            public string BodyProfilePath = "eval/body_profile.json";
            /// <summary>Per-visitor profile measured during the calibration pose.
            /// Non-null overrides BodyProfilePath.</summary>
            public BodyProfile ProfileOverride;
            public OrtProvider Provider = OrtProvider.Cuda;
            public float ConfThreshold = 0.3f;
            /// <summary>Run the catch-up smoothing post-pass (the "s" in v11s).</summary>
            public bool RunCatchupSmooth = true;
            /// <summary>Same person-selection volume convention as FusedBodiesExport
            /// (mm, origin-camera world frame).</summary>
            public Vector3 CaptureVolumeCenterMm = new Vector3(0, 200, 3000);
            public Vector3 CaptureVolumeHalfMm = new Vector3(1100, 1500, 1100);
            /// <summary>Optional pre-built backend to reuse (NOT disposed by the
            /// converter). Null = build own from ModelsDir and dispose at the end.</summary>
            public OrtRtmposeBackend ExternalBackend;
        }

        const string V11sSuffix = ".v11s";
        const string K4abtBackupSuffix = ".k4abt";

        volatile ConvertStatus _status = ConvertStatus.Idle;
        volatile float _progress;
        volatile bool _stop;
        volatile string _error = "";
        volatile string _report = "";
        Thread _worker;
        // Finalize gate: 0 = open, 1 = worker is finalizing (rename + smooth),
        // 2 = abort locked finalize out. Whoever wins the CompareExchange
        // decides — an aborted run can NEVER touch bodies_main afterwards, and
        // a run that already entered finalize completes it (bounded file I/O),
        // which callers can await via IsFinalizing.
        int _finalizeGate;

        public ConvertStatus Status => _status;
        /// <summary>0..1. Frame processing spans 0.02..0.95; smooth+rename fills the rest.</summary>
        public float Progress => _progress;
        public string Error => _error;
        /// <summary>Per-device frame counts etc. after Done.</summary>
        public string Report => _report;

        /// <summary>Kick off the conversion. Returns false (with Error set) when the
        /// converter is already running or the arguments are unusable.</summary>
        public bool Start(string sessionRoot, Options options)
        {
            if (_status == ConvertStatus.Running) { _error = "already running"; return false; }
            if (string.IsNullOrWhiteSpace(sessionRoot) || !Directory.Exists(sessionRoot))
            {
                _error = $"session root not found: {sessionRoot}";
                _status = ConvertStatus.Failed;
                return false;
            }
            options ??= new Options();

            // Resolve project-root-relative paths on the MAIN thread
            // (Application.dataPath is main-thread-only).
            string projectRoot = Directory.GetParent(Application.dataPath).FullName;
            string modelsDir = Path.IsPathRooted(options.ModelsDir)
                ? options.ModelsDir : Path.Combine(projectRoot, options.ModelsDir);
            string profilePath = string.IsNullOrEmpty(options.BodyProfilePath) ? null
                : (Path.IsPathRooted(options.BodyProfilePath)
                    ? options.BodyProfilePath : Path.Combine(projectRoot, options.BodyProfilePath));

            _stop = false;
            _error = "";
            _report = "";
            _progress = 0f;
            // A reused instance still carries the previous run's gate value (1 or
            // 2) — the new worker would always lose the finalize CompareExchange
            // and report Aborted despite succeeding. Reopen it.
            Interlocked.Exchange(ref _finalizeGate, 0);
            _status = ConvertStatus.Running;
            _worker = new Thread(() => WorkerMain(sessionRoot, options, modelsDir, profilePath))
            {
                IsBackground = true,
                Name = "FusedTakeConverter",
            };
            _worker.Start();
            return true;
        }

        /// <summary>Request cancellation. The worker deletes its temps and flips
        /// Status to Aborted; the original bodies_main is never touched. If the
        /// worker has NOT yet entered its finalize section this atomically locks
        /// it out of ever renaming/smoothing; if it already has, the finalize
        /// completes — poll <see cref="IsFinalizing"/> before reading the take.</summary>
        public void Abort()
        {
            _stop = true;
            Interlocked.CompareExchange(ref _finalizeGate, 2, 0);
        }

        /// <summary>True while the worker is inside the finalize section (rename
        /// dance + catch-up smoothing) — bounded file I/O; callers that abandoned
        /// a stuck conversion must wait this out before touching the take.</summary>
        public bool IsFinalizing =>
            _status == ConvertStatus.Running && Volatile.Read(ref _finalizeGate) == 1;

        // ---------------- worker thread ----------------

        sealed class Track
        {
            public string Serial;
            public string DeviceDir;
            public PointCloudRecording.RcsvFrameStream Depth, Color;
            public int DW, DH, CW, CH;
            public ulong[] DepthTs;
            public int[] NearestColor;
            public byte[] DepthBuf = Array.Empty<byte>(), ColorBuf = Array.Empty<byte>();
            public ObCameraParam? Cam;
            public ObExtrinsic D2C;
            public ObExtrinsic ColorToWorld;
            public PointCloudRecording.RcsvStreamWriter Writer;
            public bool Configured;
        }

        void WorkerMain(string sessionRoot, Options options, string modelsDir, string profilePath)
        {
            var tracks = new List<Track>();
            OrtRtmposeBackend ownBackend = null;
            bool renamed = false;
            try
            {
                // ---- open the take ----
                IReadOnlyList<PointCloudRecording.DeviceCalibration> calib;
                try { calib = PointCloudRecording.ReadExtrinsicsYaml(sessionRoot); }
                catch (Exception e) { Fail($"no extrinsics.yaml: {e.Message}"); return; }

                foreach (var (serial, deviceDir) in PointCloudRecording.EnumerateDevices(sessionRoot))
                {
                    string depthPath = Path.Combine(deviceDir, PointCloudRecording.DepthSensorName);
                    string colorPath = Path.Combine(deviceDir, PointCloudRecording.ColorSensorName);
                    if (!File.Exists(depthPath)) continue; // no depth = not a playable camera

                    // A PLAYABLE camera that cannot be converted must fail the
                    // WHOLE take: promoting fused bodies for the other cameras
                    // would make playback merge fused and k4abt streams — the
                    // exact mixed state the all-or-nothing rename exists to
                    // prevent. Failing keeps the take on consistent k4abt bodies.
                    PointCloudRecording.DeviceCalibration dc = null;
                    foreach (var c in calib)
                        if (string.Equals(c.Serial, serial, StringComparison.OrdinalIgnoreCase)) { dc = c; break; }
                    if (!File.Exists(colorPath))
                    { Fail($"device {serial} has depth but no color stream — whole take stays k4abt"); return; }
                    if (dc == null || !dc.GlobalTrColorCamera.HasValue)
                    { Fail($"device {serial} has no world transform in extrinsics.yaml — whole take stays k4abt"); return; }

                    var t = new Track { Serial = serial, DeviceDir = deviceDir };
                    t.Depth = new PointCloudRecording.RcsvFrameStream(depthPath);
                    t.Color = new PointCloudRecording.RcsvFrameStream(colorPath);
                    if (t.Depth.Count == 0)
                    {
                        // zero frames contribute nothing to playback — safe to skip
                        t.Depth.Dispose(); t.Color.Dispose();
                        continue;
                    }
                    if (t.Color.Count == 0)
                    {
                        t.Depth.Dispose(); t.Color.Dispose();
                        Fail($"device {serial} has depth frames but an empty color stream — whole take stays k4abt");
                        return;
                    }
                    (t.DW, t.DH) = PointCloudRecording.ReadRcsvHeaderDimensions(depthPath);
                    (t.CW, t.CH) = PointCloudRecording.ReadRcsvHeaderDimensions(colorPath);
                    t.DepthTs = new ulong[t.Depth.Count];
                    for (int i = 0; i < t.DepthTs.Length; i++) t.DepthTs[i] = t.Depth.TimestampNsAt(i);
                    t.NearestColor = NearestMap(t.DepthTs, t.Color);
                    t.D2C = dc.DepthToColor;
                    t.ColorToWorld = dc.GlobalTrColorCamera.Value;
                    t.Cam = new ObCameraParam
                    {
                        DepthIntrinsic = dc.DepthIntrinsic,
                        RgbIntrinsic = dc.ColorIntrinsic,
                        DepthDistortion = dc.DepthDistortion,
                        RgbDistortion = dc.ColorDistortion,
                        Transform = dc.DepthToColor,
                        IsMirrored = false,
                    };
                    tracks.Add(t);
                }
                if (tracks.Count == 0) { Fail("no convertible device (needs depth+color+world transform)"); return; }

                // ---- build the fusion chain ----
                OrtRtmposeBackend backend = options.ExternalBackend;
                if (backend == null)
                {
                    string yolox = FirstOnnx(Path.Combine(modelsDir, "yolox-m"));
                    string rtm = FirstOnnx(Path.Combine(modelsDir, "rtmpose-m"));
                    if (yolox == null || rtm == null) { Fail($"ONNX models not found under {modelsDir}"); return; }
                    ownBackend = backend = new OrtRtmposeBackend(yolox, rtm, options.Provider);
                }
                // Same construction as FusedBodiesExport: defaults for
                // medianLagFilter (true) / AsyncDetect (false) reproduce the
                // offline v11s output exactly.
                var fused = new FusedRtmposeAdapter(backend) { ConfThreshold = options.ConfThreshold };
                if (options.ProfileOverride != null)
                    fused.Profile = options.ProfileOverride;
                else if (profilePath != null && File.Exists(profilePath))
                    fused.Profile = BodyProfile.Load(profilePath);
                fused.SetCaptureVolume(options.CaptureVolumeCenterMm, options.CaptureVolumeHalfMm);
                foreach (var t in tracks)
                {
                    fused.Configure(new EvalCameraContext(t.Serial, t.DW, t.DH, t.CW, t.CH, t.Cam));
                    fused.SetWorldTransform(t.Serial, t.ColorToWorld);
                    t.Configured = true;
                }

                // ---- writers + skeleton sink ----
                var snap = new BodySnapshot { Id = 1 };
                var scratch = new byte[RecordedBodySerializer.FrameSize(1)];
                int written = 0;
                foreach (var t in tracks)
                    t.Writer = new PointCloudRecording.RcsvStreamWriter(
                        Path.Combine(t.DeviceDir, PointCloudRecording.BodiesSensorName + V11sSuffix),
                        PointCloudRecording.BuildBodiesHeaderYaml(t.Serial));
                fused.OnSkeletons += f =>
                {
                    var p = f.Primary();
                    if (p == null) return;
                    foreach (var t in tracks)
                    {
                        FusedSnapshotEncoder.Build(snap, p, t.ColorToWorld, t.D2C);
                        int bytes = RecordedBodySerializer.Encode(new[] { snap }, 1, scratch);
                        t.Writer.WriteFrame(f.TimestampNs, scratch, bytes);
                    }
                    written++;
                };

                // ---- global timestamp order + frame loop ----
                var order = new List<(ulong ts, int trk, int idx)>();
                for (int k = 0; k < tracks.Count; k++)
                    for (int i = 0; i < tracks[k].DepthTs.Length; i++)
                        order.Add((tracks[k].DepthTs[i], k, i));
                order.Sort((a, b) => a.ts.CompareTo(b.ts));

                _progress = 0.02f;
                ulong prevTs = 0;
                for (int c = 0; c < order.Count; c++)
                {
                    if (_stop) { CleanupTemps(tracks); _status = ConvertStatus.Aborted; return; }
                    var (ts, k, i) = order[c];
                    var t = tracks[k];

                    // Recorded rig-wide drops leave holes — inject heartbeats so the
                    // output stream stays at cadence (same as FusedBodiesExport.Step).
                    if (prevTs != 0 && ts > prevTs && ts - prevTs > 40_000_000UL)
                        for (ulong hb = prevTs + 33_000_000UL; hb + 5_000_000UL < ts; hb += 33_000_000UL)
                            fused.Heartbeat(hb);
                    prevTs = ts;

                    int depthCount = CopyFrame(t.Depth, i, ref t.DepthBuf);
                    int colorCount = t.NearestColor[i] >= 0
                        ? CopyFrame(t.Color, t.NearestColor[i], ref t.ColorBuf) : 0;
                    if (depthCount > 0 && colorCount > 0)
                    {
                        var raw = new RawFrameData(
                            t.DepthBuf, depthCount, t.DW, t.DH,
                            t.ColorBuf, colorCount, t.CW, t.CH,
                            Array.Empty<byte>(), 0, 0, 0,
                            ts / 1000UL);
                        fused.SubmitFrame(t.Serial, raw, ts);
                    }
                    _progress = 0.02f + 0.93f * (c + 1) / order.Count;
                }
                fused.FlushLag();

                foreach (var t in tracks) { t.Writer.Dispose(); t.Writer = null; }

                if (written == 0)
                {
                    CleanupTemps(tracks);
                    Fail("fusion produced no skeleton frames (nobody in the capture volume?)");
                    return;
                }

                // Claim the finalize gate. Losing means Abort() got there first —
                // possibly while we were stuck inside the last native inference
                // call and the director already fell back to k4abt playback of
                // this very take. bodies_main must not be touched then.
                if (Interlocked.CompareExchange(ref _finalizeGate, 1, 0) != 0)
                {
                    CleanupTemps(tracks);
                    _status = ConvertStatus.Aborted;
                    return;
                }

                // ---- rename dance (all devices or none) ----
                // A partial rename would leave the take half-converted: some
                // cameras' bodies_main fused, others k4abt — the merger would mix
                // the two skeleton streams. Stage per phase and roll back EVERY
                // device on any failure so the failure contract ("the original
                // k4abt take stays playable") holds for multi-camera takes.
                PromoteAllOrRollback(tracks);
                renamed = true;
                _progress = 0.96f;

                // FusedCatchupSmooth reads from bodies_main.prekink when one
                // exists (it backs up only once). A rerun on a previously
                // converted take would therefore smooth the PREVIOUS run's
                // skeletons over the fresh promote — drop stale backups so the
                // smoother snapshots the bodies_main we just wrote.
                foreach (var t in tracks)
                {
                    try
                    {
                        string prekink = Path.Combine(t.DeviceDir,
                            PointCloudRecording.BodiesSensorName) + ".prekink";
                        if (File.Exists(prekink)) File.Delete(prekink);
                    }
                    catch { /* smoother failure is non-fatal below anyway */ }
                }

                // ---- catch-up smoothing (non-fatal: the fused output already stands) ----
                string smoothReport = "";
                if (options.RunCatchupSmooth && !_stop)
                {
                    try { smoothReport = FusedCatchupSmooth.Run(sessionRoot); }
                    catch (Exception e)
                    {
                        Debug.LogWarning($"[{nameof(FusedTakeConverter)}] catch-up smoothing failed (fused output kept): {e.Message}");
                    }
                }

                var sb = new System.Text.StringBuilder();
                sb.AppendLine($"fused {written} frames across {tracks.Count} device(s) under {sessionRoot}");
                if (smoothReport.Length > 0) sb.Append(smoothReport);
                _report = sb.ToString();
                _progress = 1f;
                _status = ConvertStatus.Done;
                Debug.Log($"[{nameof(FusedTakeConverter)}] {_report}");
            }
            catch (Exception e)
            {
                if (!renamed) CleanupTemps(tracks);
                Fail(e.Message);
                Debug.LogException(e);
            }
            finally
            {
                foreach (var t in tracks)
                {
                    try { t.Writer?.Dispose(); } catch { }
                    try { t.Depth?.Dispose(); } catch { }
                    try { t.Color?.Dispose(); } catch { }
                }
                try { ownBackend?.Dispose(); } catch { }
            }
        }

        void Fail(string message)
        {
            _error = message;
            _status = ConvertStatus.Failed;
            Debug.LogError($"[{nameof(FusedTakeConverter)}] {message}");
        }

        /// <summary>Promote every device's bodies_main.v11s to bodies_main
        /// (originals backed up as .k4abt), atomically across devices: any
        /// failure rolls back all completed moves and rethrows, leaving the
        /// take exactly as it was.</summary>
        static void PromoteAllOrRollback(List<Track> tracks)
        {
            // Phase 0: every temp must exist before anything is touched.
            foreach (var t in tracks)
            {
                string temp = Path.Combine(t.DeviceDir, PointCloudRecording.BodiesSensorName + V11sSuffix);
                if (!File.Exists(temp))
                    throw new FileNotFoundException("fused temp missing before rename", temp);
            }

            var backedUp = new List<Track>();  // main moved -> backup
            var promoted = new List<Track>();  // temp moved -> main
            try
            {
                // Phase 1: park the originals.
                foreach (var t in tracks)
                {
                    string main = Path.Combine(t.DeviceDir, PointCloudRecording.BodiesSensorName);
                    string backup = main + K4abtBackupSuffix;
                    if (!File.Exists(main)) continue; // take had no k4abt bodies for this cam
                    if (File.Exists(backup)) File.Delete(backup);
                    File.Move(main, backup);
                    backedUp.Add(t);
                }
                // Phase 2: promote the fused temps.
                foreach (var t in tracks)
                {
                    string main = Path.Combine(t.DeviceDir, PointCloudRecording.BodiesSensorName);
                    File.Move(main + V11sSuffix, main);
                    promoted.Add(t);
                }
            }
            catch
            {
                // Roll back in reverse: fused mains -> temps, then backups -> mains.
                foreach (var t in promoted)
                {
                    string main = Path.Combine(t.DeviceDir, PointCloudRecording.BodiesSensorName);
                    try { if (File.Exists(main)) File.Move(main, main + V11sSuffix); }
                    catch { /* best effort — the backup restore below still runs */ }
                }
                foreach (var t in backedUp)
                {
                    string main = Path.Combine(t.DeviceDir, PointCloudRecording.BodiesSensorName);
                    string backup = main + K4abtBackupSuffix;
                    try { if (!File.Exists(main) && File.Exists(backup)) File.Move(backup, main); }
                    catch { /* best effort */ }
                }
                throw;
            }
        }

        static void CleanupTemps(List<Track> tracks)
        {
            foreach (var t in tracks)
            {
                try { t.Writer?.Dispose(); } catch { }
                t.Writer = null;
                try
                {
                    string temp = Path.Combine(t.DeviceDir, PointCloudRecording.BodiesSensorName + V11sSuffix);
                    if (File.Exists(temp)) File.Delete(temp);
                }
                catch { }
            }
        }

        static int CopyFrame(PointCloudRecording.RcsvFrameStream s, int index, ref byte[] buf)
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
                Debug.LogWarning($"[{nameof(FusedTakeConverter)}] skipping unreadable frame {index} in {Path.GetFileName(s.FilePath)}: {e.Message}");
                return 0;
            }
        }

        // Nearest color index per depth timestamp (no skew limit — color runs at
        // the depth rate). Same walk as EvalReplayDriver.NearestMap.
        static int[] NearestMap(ulong[] masterTs, PointCloudRecording.RcsvFrameStream other)
        {
            var map = new int[masterTs.Length];
            int n = other.Count, j = 0;
            for (int i = 0; i < masterTs.Length; i++)
            {
                ulong t = masterTs[i];
                while (j + 1 < n && AbsDiff(other.TimestampNsAt(j + 1), t) <= AbsDiff(other.TimestampNsAt(j), t)) j++;
                map[i] = j;
            }
            return map;
        }

        static ulong AbsDiff(ulong a, ulong b) => a > b ? a - b : b - a;

        static string FirstOnnx(string dir)
        {
            if (!Directory.Exists(dir)) return null;
            var files = Directory.GetFiles(dir, "*.onnx", SearchOption.AllDirectories);
            return files.Length > 0 ? files[0] : null;
        }
    }
}
