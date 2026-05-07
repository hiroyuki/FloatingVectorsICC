// Unity-side host that owns one k4abt_worker.exe per camera, brokers depth+IR
// frames into the worker via a per-camera memory-mapped file, and surfaces
// returned skeletons through OnSkeletonsReady. The IPC contract is defined in
// BodyTracking.Shared.K4abtWorkerSharedLayout. v1 only handles a single
// camera at a time (the dictionary makes future multi-cam expansion trivial).
//
// Lifecycle: StartWorker creates MMF + 4 events, writes the global header
// (including the calibration blob built from ObCameraParam), spawns
// k4abt_worker.exe, and returns immediately. A background watcher waits on
// readyEvt and flips IsReady when the worker has loaded the BT model.
// EnqueueFrame writes a slot under the seq-lock protocol; the host never
// blocks the caller. Update polls outputEvt with WaitOne(0) and fires
// OnSkeletonsReady on the main thread.
//
// Teardown sets shutdownEvt, waits up to TeardownWaitMs for the process to
// exit, then Process.Kill, then disposes everything.

using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.IO.MemoryMappedFiles;
using System.Runtime.InteropServices;
using System.Threading;
using BodyTracking.Shared;
using Orbbec;
using UnityEngine;
using Debug = UnityEngine.Debug;

namespace BodyTracking
{
    // Execute Update before any subscriber to OnSkeletonsReady (e.g. BodyTrackingLive)
    // so the populated state from this frame's worker output is visible when those
    // scripts run their own Update GC / housekeeping pass.
    [DefaultExecutionOrder(-100)]
    public sealed class K4abtWorkerHost : MonoBehaviour
    {
        [Header("Worker")]
        [Tooltip("Relative or absolute path to k4abt_worker.exe. Default points at the published " +
                 "single-file build under Workers/K4abtWorker. Edit if you publish elsewhere.")]
        public string workerExePath =
            "Workers/K4abtWorker/bin/Release/net8.0-windows/win-x64/publish/k4abt_worker.exe";

        [Tooltip("Optional override for the K4A Wrapper bin path. Empty = use WorkerBootstrap default.")]
        public string k4aWrapperBinOverride = "";

        [Tooltip("Optional override for the BT SDK bin path. Empty = use WorkerBootstrap default.")]
        public string btSdkBinOverride = "";

        [Tooltip("Master enable. BodyTrackingLive checks this when it decides between in-process " +
                 "and worker-process tracking.")]
        public bool useWorker = false;

        [Tooltip("Log per-second host-side counters (produced/dropped/consumed) to the Unity console.")]
        public bool diagnosticLogging = false;

        /// <summary>
        /// Fires synchronously on the main thread inside Update when a worker has produced
        /// new skeletons. The (snapshots, count) pair points at a host-owned reusable
        /// buffer — copy anything you need to persist; do not retain references past the
        /// handler return.
        /// </summary>
        public event Action<string, BodySnapshot[], int> OnSkeletonsReady;

        // --- per-camera session state ---

        private sealed unsafe class WorkerSession : IDisposable
        {
            public string Serial;
            public Guid SessionGuid;
            public Process Process;
            public MemoryMappedFile Mmf;
            public MemoryMappedViewAccessor View;
            public byte* BasePtr;
            public bool PointerAcquired;
            public EventWaitHandle InputEvt, OutputEvt, ReadyEvt, ShutdownEvt;
            public Thread ReadyWatcher;
            public CancellationTokenSource ReadyCts;
            public int DepthW, DepthH, IrW, IrH;
            public int ColorW, ColorH; // used only when building the calibration blob
            public volatile bool Ready;
            public ulong NextFrameId = 1; // start at 1 so 0 means "never written"
            public ulong[] InputSeq = new ulong[K4abtWorkerSharedLayout.InputSlotCount];
            public int NextInputSlot;
            public ulong LastSeenOutputFrameId;
            public BodySnapshot[] BodyBuffer = new BodySnapshot[K4abtWorkerSharedLayout.MaxBodies];
            // Diagnostic
            public int DiagEnqueued, DiagDroppedNotReady, DiagDroppedOverwrite, DiagSkeletonsRecv;
            public float DiagWindowStart;

            public WorkerSession()
            {
                for (int i = 0; i < BodyBuffer.Length; i++) BodyBuffer[i] = new BodySnapshot();
            }

            public void Dispose()
            {
                ReadyCts?.Cancel();
                try { ReadyWatcher?.Join(500); } catch { }
                ReadyCts?.Dispose();

                if (PointerAcquired && View != null)
                {
                    try { View.SafeMemoryMappedViewHandle.ReleasePointer(); } catch { }
                    PointerAcquired = false;
                    BasePtr = null;
                }
                View?.Dispose();
                Mmf?.Dispose();
                try { InputEvt?.Dispose(); } catch { }
                try { OutputEvt?.Dispose(); } catch { }
                try { ReadyEvt?.Dispose(); } catch { }
                try { ShutdownEvt?.Dispose(); } catch { }

                if (Process != null)
                {
                    try { if (!Process.HasExited) Process.Kill(); } catch { }
                    try { Process.Dispose(); } catch { }
                    Process = null;
                }
            }
        }

        private readonly Dictionary<string, WorkerSession> _sessions = new Dictionary<string, WorkerSession>();

        public bool IsReady(string serial) =>
            _sessions.TryGetValue(serial, out var s) && s.Ready;

        public bool HasSession(string serial) => _sessions.ContainsKey(serial);

        // --- Public API ---

        public unsafe bool StartWorker(string serial, in ObCameraParam calib,
                                        int depthW, int depthH, int irW, int irH,
                                        int colorW, int colorH)
        {
            if (string.IsNullOrEmpty(serial)) { Debug.LogError("[K4abtWorkerHost] serial is required"); return false; }
            if (_sessions.ContainsKey(serial))
            {
                Debug.LogWarning($"[K4abtWorkerHost] worker already running for serial '{serial}'");
                return false;
            }

            string exePath = ResolveExePath(workerExePath);
            if (!File.Exists(exePath))
            {
                Debug.LogError(
                    $"[K4abtWorkerHost] worker exe not found at {exePath}. " +
                    "Run `dotnet publish -c Release -r win-x64` in Workers/K4abtWorker first.");
                return false;
            }

            var session = new WorkerSession
            {
                Serial = serial,
                SessionGuid = Guid.NewGuid(),
                DepthW = depthW,
                DepthH = depthH,
                IrW = irW,
                IrH = irH,
                ColorW = colorW,
                ColorH = colorH,
            };

            try
            {
                long mmfSize = K4abtWorkerSharedLayout.TotalMmfSize(depthW, depthH, irW, irH);
                string mmfName = K4abtWorkerSharedLayout.MmfName(session.SessionGuid, serial);
                session.Mmf = MemoryMappedFile.CreateNew(mmfName, mmfSize, MemoryMappedFileAccess.ReadWrite);
                session.View = session.Mmf.CreateViewAccessor(0, mmfSize, MemoryMappedFileAccess.ReadWrite);

                byte* p = null;
                session.View.SafeMemoryMappedViewHandle.AcquirePointer(ref p);
                if (p == null)
                {
                    Debug.LogError("[K4abtWorkerHost] AcquirePointer returned null");
                    session.Dispose();
                    return false;
                }
                session.BasePtr = p;
                session.PointerAcquired = true;

                // Zero the global header region (slot regions are written on each frame
                // so they don't need pre-zeroing; the seq-lock keeps readers honest).
                int headerBytes = K4abtWorkerSharedLayout.GlobalHeaderBytes;
                for (int i = 0; i < headerBytes; i++) session.BasePtr[i] = 0;

                WriteHeader(session, calib);

                bool createdNew;
                session.InputEvt = new EventWaitHandle(false, EventResetMode.AutoReset,
                    K4abtWorkerSharedLayout.InputEventName(session.SessionGuid, serial), out createdNew);
                if (!createdNew) { Debug.LogError("[K4abtWorkerHost] input event already exists"); session.Dispose(); return false; }
                session.OutputEvt = new EventWaitHandle(false, EventResetMode.AutoReset,
                    K4abtWorkerSharedLayout.OutputEventName(session.SessionGuid, serial), out createdNew);
                if (!createdNew) { Debug.LogError("[K4abtWorkerHost] output event already exists"); session.Dispose(); return false; }
                session.ReadyEvt = new EventWaitHandle(false, EventResetMode.AutoReset,
                    K4abtWorkerSharedLayout.ReadyEventName(session.SessionGuid, serial), out createdNew);
                if (!createdNew) { Debug.LogError("[K4abtWorkerHost] ready event already exists"); session.Dispose(); return false; }
                session.ShutdownEvt = new EventWaitHandle(false, EventResetMode.AutoReset,
                    K4abtWorkerSharedLayout.ShutdownEventName(session.SessionGuid, serial), out createdNew);
                if (!createdNew) { Debug.LogError("[K4abtWorkerHost] shutdown event already exists"); session.Dispose(); return false; }

                session.Process = SpawnWorker(session, exePath);
                if (session.Process == null) { session.Dispose(); return false; }

                session.ReadyCts = new CancellationTokenSource();
                session.ReadyWatcher = new Thread(() => WaitForReady(session))
                {
                    Name = $"K4abtWorkerHost.Ready({serial})",
                    IsBackground = true,
                };
                session.ReadyWatcher.Start();

                _sessions[serial] = session;
                Debug.Log($"[K4abtWorkerHost] spawned worker for serial='{serial}' pid={session.Process.Id}");
                return true;
            }
            catch (Exception ex)
            {
                Debug.LogError($"[K4abtWorkerHost] StartWorker failed: {ex}");
                session.Dispose();
                return false;
            }
        }

        public unsafe bool EnqueueFrame(string serial, byte[] depth, int depthBytes,
                                         byte[] ir, int irBytes, ulong tsNs)
        {
            if (!_sessions.TryGetValue(serial, out var s)) return false;
            if (!s.Ready)
            {
                s.DiagDroppedNotReady++;
                return false;
            }

            int expectedDepth = s.DepthW * s.DepthH * 2;
            int expectedIr = s.IrW * s.IrH * 2;
            if (depth == null || depthBytes < expectedDepth)
            {
                Debug.LogError($"[K4abtWorkerHost] depth buffer too small: {depthBytes}/{expectedDepth}");
                return false;
            }
            bool irValid = ir != null && irBytes >= expectedIr;

            int slotIdx = s.NextInputSlot;
            s.NextInputSlot = (s.NextInputSlot + 1) % K4abtWorkerSharedLayout.InputSlotCount;

            long slotOffset = K4abtWorkerSharedLayout.InputSlotOffset(slotIdx, s.DepthW, s.DepthH, s.IrW, s.IrH);
            byte* slot = s.BasePtr + slotOffset;

            // Detect overwrite-without-consume: if previous slot was committed (even seq) and
            // produced > consumed counter, increment dropped.
            ulong prevSeq = s.InputSeq[slotIdx];
            ulong writingSeq = prevSeq + 1; // odd = writing
            *(ulong*)(slot + K4abtWorkerSharedLayout.SlotOffsetSeq) = writingSeq;
            Thread.MemoryBarrier();

            ulong frameId = s.NextFrameId++;
            *(ulong*)(slot + K4abtWorkerSharedLayout.SlotOffsetFrameId) = frameId;
            *(ulong*)(slot + K4abtWorkerSharedLayout.SlotOffsetTsNs) = tsNs;
            uint flags = K4abtWorkerSharedLayout.InputFlagDepthValid;
            if (irValid) flags |= K4abtWorkerSharedLayout.InputFlagIrValid;
            *(uint*)(slot + K4abtWorkerSharedLayout.SlotOffsetFlags) = flags;
            *(uint*)(slot + K4abtWorkerSharedLayout.SlotOffsetReserved) = 0;

            byte* depthDst = slot + K4abtWorkerSharedLayout.InputDepthPayloadOffset();
            byte* irDst = slot + K4abtWorkerSharedLayout.InputIrPayloadOffset(s.DepthW, s.DepthH);
            fixed (byte* depthSrc = depth)
                Buffer.MemoryCopy(depthSrc, depthDst, expectedDepth, expectedDepth);
            if (irValid)
            {
                fixed (byte* irSrc = ir)
                    Buffer.MemoryCopy(irSrc, irDst, expectedIr, expectedIr);
            }

            Thread.MemoryBarrier();
            ulong committed = writingSeq + 1;
            *(ulong*)(slot + K4abtWorkerSharedLayout.SlotOffsetSeq) = committed;
            s.InputSeq[slotIdx] = committed;

            // Bump counters and signal.
            byte* bp = s.BasePtr;
            ulong* producedPtr = (ulong*)(bp + K4abtWorkerSharedLayout.OffsetProduced);
            ulong producedNew = (ulong)Interlocked.Increment(ref *(long*)producedPtr);
            ulong consumed = (ulong)Volatile.Read(ref *(long*)(bp + K4abtWorkerSharedLayout.OffsetConsumed));
            if (producedNew - consumed > 1)
            {
                Interlocked.Increment(ref *(long*)(bp + K4abtWorkerSharedLayout.OffsetDropped));
                s.DiagDroppedOverwrite++;
            }
            Volatile.Write(ref *(uint*)(bp + K4abtWorkerSharedLayout.OffsetLatestInputSlot), (uint)slotIdx);

            try { s.InputEvt.Set(); } catch { return false; }
            s.DiagEnqueued++;
            return true;
        }

        public void StopWorker(string serial)
        {
            if (!_sessions.TryGetValue(serial, out var s)) return;
            _sessions.Remove(serial);
            TeardownSession(s);
        }

        // --- Unity lifecycle ---

        private void Update()
        {
            // Local snapshot; StopWorker mutates the dictionary.
            if (_sessions.Count == 0) return;
            // Avoid LINQ; iterate a fresh array each frame.
            var keys = new string[_sessions.Count];
            int ki = 0;
            foreach (var k in _sessions.Keys) keys[ki++] = k;

            for (int i = 0; i < keys.Length; i++)
            {
                if (!_sessions.TryGetValue(keys[i], out var s)) continue;
                if (!s.Ready) continue;
                if (s.Process != null && s.Process.HasExited)
                {
                    Debug.LogWarning($"[K4abtWorkerHost] worker for serial='{s.Serial}' exited (code={s.Process.ExitCode}); tearing down session");
                    StopWorker(s.Serial);
                    continue;
                }
                if (!s.OutputEvt.WaitOne(0)) continue;
                ProcessOutputSlot(s);
            }

            if (diagnosticLogging) PerSecondDiag();
        }

        private void OnDisable() => StopAllWorkers();

        private void OnDestroy() => StopAllWorkers();

        private void OnApplicationQuit() => StopAllWorkers();

        private void StopAllWorkers()
        {
            if (_sessions.Count == 0) return;
            var keys = new string[_sessions.Count];
            int ki = 0;
            foreach (var k in _sessions.Keys) keys[ki++] = k;
            for (int i = 0; i < keys.Length; i++) StopWorker(keys[i]);
        }

        // --- internals ---

        private static unsafe void WriteHeader(WorkerSession s, in ObCameraParam calib)
        {
            byte* bp = s.BasePtr;
            *(uint*)(bp + K4abtWorkerSharedLayout.OffsetMagic) = K4abtWorkerSharedLayout.Magic;
            *(uint*)(bp + K4abtWorkerSharedLayout.OffsetLayoutVersion) = K4abtWorkerSharedLayout.LayoutVersion;
            *(uint*)(bp + K4abtWorkerSharedLayout.OffsetDepthW) = (uint)s.DepthW;
            *(uint*)(bp + K4abtWorkerSharedLayout.OffsetDepthH) = (uint)s.DepthH;
            *(uint*)(bp + K4abtWorkerSharedLayout.OffsetIrW) = (uint)s.IrW;
            *(uint*)(bp + K4abtWorkerSharedLayout.OffsetIrH) = (uint)s.IrH;
            *(uint*)(bp + K4abtWorkerSharedLayout.OffsetDepthBpp) = 2;
            *(uint*)(bp + K4abtWorkerSharedLayout.OffsetIrBpp) = 2;
            *(uint*)(bp + K4abtWorkerSharedLayout.OffsetInputSlotCount) = K4abtWorkerSharedLayout.InputSlotCount;
            *(uint*)(bp + K4abtWorkerSharedLayout.OffsetOutputSlotCount) = K4abtWorkerSharedLayout.OutputSlotCount;

            // Build calibration via existing K4ACalibration helper, copy raw bytes into the
            // header, then free the unmanaged buffer immediately. The MMF retains the bytes.
            // The blob layout is the same one k4abt_tracker_create expects, by contract.
            IntPtr calibBuf = K4ACalibration.Build(calib, s.DepthW, s.DepthH, s.ColorW, s.ColorH);
            try
            {
                byte* src = (byte*)calibBuf.ToPointer();
                byte* dst = bp + K4abtWorkerSharedLayout.OffsetCalibration;
                Buffer.MemoryCopy(src, dst,
                    K4abtWorkerSharedLayout.CalibrationBlobBytes,
                    K4abtWorkerSharedLayout.CalibrationBlobBytes);
            }
            finally
            {
                K4ACalibration.Free(calibBuf);
            }
        }

        private Process SpawnWorker(WorkerSession session, string exePath)
        {
            var psi = new ProcessStartInfo
            {
                FileName = exePath,
                UseShellExecute = false,
                CreateNoWindow = true,
                RedirectStandardOutput = true,
                RedirectStandardError = true,
                WorkingDirectory = Path.GetDirectoryName(Path.GetFullPath(exePath)) ?? Environment.CurrentDirectory,
            };
            AppendArg(psi, $"--session={session.SessionGuid:N}");
            AppendArg(psi, $"--serial={session.Serial}");
            AppendArg(psi, $"--parent-pid={Process.GetCurrentProcess().Id}");
            AppendArg(psi, $"--depth-w={session.DepthW}");
            AppendArg(psi, $"--depth-h={session.DepthH}");
            AppendArg(psi, $"--ir-w={session.IrW}");
            AppendArg(psi, $"--ir-h={session.IrH}");
            if (!string.IsNullOrEmpty(k4aWrapperBinOverride)) AppendArg(psi, $"--k4a-wrapper-bin={k4aWrapperBinOverride}");
            if (!string.IsNullOrEmpty(btSdkBinOverride)) AppendArg(psi, $"--bt-sdk-bin={btSdkBinOverride}");

            try
            {
                var p = new Process { StartInfo = psi, EnableRaisingEvents = false };
                string serial = session.Serial;
                p.OutputDataReceived += (sender, args) =>
                {
                    if (string.IsNullOrEmpty(args.Data)) return;
                    Debug.Log($"[k4abt_worker {serial}] {args.Data}");
                };
                p.ErrorDataReceived += (sender, args) =>
                {
                    if (string.IsNullOrEmpty(args.Data)) return;
                    Debug.LogError($"[k4abt_worker {serial}] {args.Data}");
                };
                if (!p.Start()) return null;
                p.BeginOutputReadLine();
                p.BeginErrorReadLine();
                return p;
            }
            catch (Exception ex)
            {
                Debug.LogError($"[K4abtWorkerHost] failed to spawn worker: {ex}");
                return null;
            }
        }

        private static void AppendArg(ProcessStartInfo psi, string arg)
        {
#if UNITY_2021_2_OR_NEWER
            psi.ArgumentList.Add(arg);
#else
            // Older Unity Mono runtimes don't expose ArgumentList; fall back to Arguments.
            // Worker args never contain spaces (GUIDs, ints, paths quoted by caller) so a
            // simple space concat is sufficient for v1.
            psi.Arguments = string.IsNullOrEmpty(psi.Arguments) ? arg : psi.Arguments + " " + arg;
#endif
        }

        private void WaitForReady(WorkerSession s)
        {
            try
            {
                bool ok = s.ReadyEvt.WaitOne(K4abtWorkerSharedLayout.ReadyWaitTimeoutMs);
                if (s.ReadyCts.IsCancellationRequested) return;
                if (!ok)
                {
                    Debug.LogError($"[K4abtWorkerHost] worker for serial='{s.Serial}' did not signal ready in {K4abtWorkerSharedLayout.ReadyWaitTimeoutMs}ms; killing");
                    try { if (s.Process != null && !s.Process.HasExited) s.Process.Kill(); } catch { }
                    return;
                }
                s.Ready = true;
                Debug.Log($"[K4abtWorkerHost] worker ready for serial='{s.Serial}'");
            }
            catch (Exception ex)
            {
                Debug.LogError($"[K4abtWorkerHost] WaitForReady error: {ex}");
            }
        }

        private unsafe void ProcessOutputSlot(WorkerSession s)
        {
            byte* bp = s.BasePtr;
            uint slotIdx = Volatile.Read(ref *(uint*)(bp + K4abtWorkerSharedLayout.OffsetLatestOutputSlot));
            if (slotIdx >= K4abtWorkerSharedLayout.OutputSlotCount) return;

            long slotOffset = K4abtWorkerSharedLayout.OutputSlotOffset(
                (int)slotIdx, s.DepthW, s.DepthH, s.IrW, s.IrH);
            byte* slot = bp + slotOffset;

            ulong seqBefore = Volatile.Read(ref *(ulong*)(slot + K4abtWorkerSharedLayout.SlotOffsetSeq));
            if ((seqBefore & 1) != 0) return; // worker mid-write

            ulong frameId = *(ulong*)(slot + K4abtWorkerSharedLayout.SlotOffsetFrameId);
            ulong tsNs = *(ulong*)(slot + K4abtWorkerSharedLayout.SlotOffsetTsNs);
            uint bodyCount = *(uint*)(slot + K4abtWorkerSharedLayout.OutSlotOffsetBodyCount);
            if (bodyCount > K4abtWorkerSharedLayout.MaxBodies) return;

            // Drain bodies into the pooled BodySnapshot[].
            byte* bodies = slot + K4abtWorkerSharedLayout.OutSlotOffsetBodies;
            for (uint i = 0; i < bodyCount; i++)
            {
                byte* rec = bodies + (int)i * K4abtWorkerSharedLayout.BodyRecordBytes;
                var snap = s.BodyBuffer[(int)i];
                snap.Id = *(uint*)(rec + 0);
                byte* jp = rec + 8;
                for (int j = 0; j < K4abtWorkerSharedLayout.K4abtJointCount; j++)
                {
                    var jt = new k4abt_joint_t
                    {
                        Position = new k4a_float3_t
                        {
                            X = *(float*)(jp + 0),
                            Y = *(float*)(jp + 4),
                            Z = *(float*)(jp + 8),
                        },
                        Orientation = new k4a_quaternion_t
                        {
                            W = *(float*)(jp + 12),
                            X = *(float*)(jp + 16),
                            Y = *(float*)(jp + 20),
                            Z = *(float*)(jp + 24),
                        },
                        ConfidenceLevel = (k4abt_joint_confidence_level_t)(*(uint*)(jp + 28)),
                    };
                    snap.Joints[j] = jt;
                    jp += K4abtWorkerSharedLayout.JointRecordBytes;
                }
            }

            Thread.MemoryBarrier();
            ulong seqAfter = Volatile.Read(ref *(ulong*)(slot + K4abtWorkerSharedLayout.SlotOffsetSeq));
            if (seqAfter != seqBefore || (seqAfter & 1) != 0) return; // torn — drop this read

            if (frameId <= s.LastSeenOutputFrameId) return; // stale
            s.LastSeenOutputFrameId = frameId;
            s.DiagSkeletonsRecv += (int)bodyCount;

            try { OnSkeletonsReady?.Invoke(s.Serial, s.BodyBuffer, (int)bodyCount); }
            catch (Exception ex) { Debug.LogError($"[K4abtWorkerHost] OnSkeletonsReady handler threw: {ex}"); }
        }

        private void TeardownSession(WorkerSession s)
        {
            try { s.ShutdownEvt?.Set(); } catch { }
            try
            {
                if (s.Process != null && !s.Process.HasExited)
                {
                    if (!s.Process.WaitForExit(K4abtWorkerSharedLayout.TeardownWaitMs))
                    {
                        Debug.LogWarning($"[K4abtWorkerHost] worker for serial='{s.Serial}' did not exit in {K4abtWorkerSharedLayout.TeardownWaitMs}ms; killing");
                        try { s.Process.Kill(); } catch { }
                    }
                }
            }
            catch (Exception ex)
            {
                Debug.LogWarning($"[K4abtWorkerHost] teardown wait error: {ex}");
            }
            s.Dispose();
        }

        private static string ResolveExePath(string raw)
        {
            if (Path.IsPathRooted(raw)) return raw;
            // Project-relative when running in Editor; runtime CWD when in a player build.
            string projectRoot = Path.GetFullPath(Path.Combine(Application.dataPath, ".."));
            string candidate = Path.GetFullPath(Path.Combine(projectRoot, raw));
            return candidate;
        }

        private unsafe void PerSecondDiag()
        {
            float now = Time.realtimeSinceStartup;
            foreach (var s in _sessions.Values)
            {
                if (s.DiagWindowStart == 0f) s.DiagWindowStart = now;
                if (now - s.DiagWindowStart < 1f) continue;
                ulong produced = (ulong)Volatile.Read(ref *(long*)(s.BasePtr + K4abtWorkerSharedLayout.OffsetProduced));
                ulong consumed = (ulong)Volatile.Read(ref *(long*)(s.BasePtr + K4abtWorkerSharedLayout.OffsetConsumed));
                ulong dropped = (ulong)Volatile.Read(ref *(long*)(s.BasePtr + K4abtWorkerSharedLayout.OffsetDropped));
                Debug.Log(
                    $"[K4abtWorkerHost {s.Serial}] enqueued/s={s.DiagEnqueued} " +
                    $"dropped_not_ready/s={s.DiagDroppedNotReady} " +
                    $"dropped_overwrite/s={s.DiagDroppedOverwrite} " +
                    $"skeletons/s={s.DiagSkeletonsRecv} " +
                    $"produced={produced} consumed={consumed} dropped={dropped}");
                s.DiagEnqueued = s.DiagDroppedNotReady = s.DiagDroppedOverwrite = s.DiagSkeletonsRecv = 0;
                s.DiagWindowStart = now;
            }
        }
    }
}
