// Standalone k4abt worker process.
//
// Receives depth+IR frames from Unity through a per-camera memory-mapped file
// (seq-lock double-buffered slots), runs the Microsoft Azure Kinect Body
// Tracking SDK on each capture, and writes skeleton results back through the
// same MMF. See Plans/issue-10-k4abt-worker-process.md for the full IPC
// contract and lifecycle (parent watchdog, shutdown event, ready event).

using System;
using System.Runtime.InteropServices;
using System.Threading;
using BodyTracking;
using BodyTracking.Shared;

namespace K4abtWorker
{
    internal static class Program
    {
        private static int Main(string[] args)
        {
            WorkerLog.SetLogger(new ConsoleWorkerLogger());

            if (!WorkerArgs.TryParse(args, out var parsed, out var argError))
            {
                WorkerLog.Error($"[Args] {argError}\n{WorkerArgs.Usage}");
                return K4abtWorkerSharedLayout.ExitArgError;
            }

            WorkerLog.Info(
                $"k4abt_worker starting: serial={parsed.Serial} session={parsed.SessionGuid:N} " +
                $"parent={parsed.ParentPid} depth={parsed.DepthW}x{parsed.DepthH} ir={parsed.IrW}x{parsed.IrH}");

            // PATH prepend so k4a / k4abt P/Invoke resolves their DLLs. Mirrors what
            // BodyTrackingBootstrap does in Unity. Missing paths warn but don't abort
            // — failure surfaces as DllNotFoundException on the first P/Invoke instead.
            WorkerBootstrap.PrependDllSearchPaths(parsed.K4aWrapperBin, parsed.BtSdkBin);

            EventWaitHandle inputEvt = null, outputEvt = null, readyEvt = null, shutdownEvt = null;
            MmfRegion mmf = null;
            ParentWatchdog watchdog = null;
            IntPtr calibration = IntPtr.Zero;
            IntPtr tracker = IntPtr.Zero;

            try
            {
                // 1. IPC handles (host already created them; OpenExisting must succeed).
                inputEvt = EventWaitHandle.OpenExisting(
                    K4abtWorkerSharedLayout.InputEventName(parsed.SessionGuid, parsed.Serial));
                outputEvt = EventWaitHandle.OpenExisting(
                    K4abtWorkerSharedLayout.OutputEventName(parsed.SessionGuid, parsed.Serial));
                readyEvt = EventWaitHandle.OpenExisting(
                    K4abtWorkerSharedLayout.ReadyEventName(parsed.SessionGuid, parsed.Serial));
                shutdownEvt = EventWaitHandle.OpenExisting(
                    K4abtWorkerSharedLayout.ShutdownEventName(parsed.SessionGuid, parsed.Serial));

                mmf = new MmfRegion(
                    K4abtWorkerSharedLayout.MmfName(parsed.SessionGuid, parsed.Serial),
                    parsed.DepthW, parsed.DepthH, parsed.IrW, parsed.IrH);

                // 2. Verify header (rejects mismatched layout / resolution before we touch BT).
                string headerError = mmf.VerifyHeader();
                if (headerError != null)
                {
                    WorkerLog.Error($"[Mmf] header verify failed: {headerError}");
                    return K4abtWorkerSharedLayout.ExitLayoutMismatch;
                }

                // 3. Pull calibration blob (raw 1032 bytes, never bound to a managed struct).
                calibration = mmf.AllocAndCopyCalibration();

                // 4. Create the BT tracker (5–10 s for ONNX model load on first run).
                var trackerCfg = new k4abt_tracker_configuration_t
                {
                    SensorOrientation = k4abt_sensor_orientation_t.K4ABT_SENSOR_ORIENTATION_DEFAULT,
                    ProcessingMode = k4abt_tracker_processing_mode_t.K4ABT_TRACKER_PROCESSING_MODE_GPU_DIRECTML,
                    GpuDeviceId = 0,
                    ModelPath = null,
                };
                var rc = K4ABTNative.k4abt_tracker_create(calibration, trackerCfg, out tracker);
                if (rc != k4a_result_t.K4A_RESULT_SUCCEEDED)
                {
                    WorkerLog.Error($"[k4abt] tracker_create failed (rc={rc})");
                    return K4abtWorkerSharedLayout.ExitTrackerCreateFailed;
                }
                K4ABTNative.k4abt_tracker_set_temporal_smoothing(tracker, 0f);
                WorkerLog.Info("[k4abt] tracker_create succeeded");

                // 5. Tell the host we're ready (blocks no longer; host can EnqueueFrame).
                readyEvt.Set();

                // 6. Parent watchdog (1 Hz). Self-shuts-down if Unity exits abruptly.
                watchdog = new ParentWatchdog(parsed.ParentPid, shutdownEvt);
                watchdog.Start();

                // 7. Main loop.
                int exitCode = RunLoop(parsed, mmf, tracker, inputEvt, outputEvt, shutdownEvt);
                if (watchdog.ParentExited && exitCode == K4abtWorkerSharedLayout.ExitOk)
                {
                    exitCode = K4abtWorkerSharedLayout.ExitParentDied;
                }
                return exitCode;
            }
            catch (Exception ex)
            {
                WorkerLog.Error($"[Main] unhandled exception: {ex}");
                return K4abtWorkerSharedLayout.ExitTrackerCreateFailed;
            }
            finally
            {
                // Order matters: stop the tracker (so it stops calling back into our
                // delegates) before freeing the calibration buffer or the MMF region.
                if (tracker != IntPtr.Zero)
                {
                    try { K4ABTNative.k4abt_tracker_shutdown(tracker); } catch { }
                    try { K4ABTNative.k4abt_tracker_destroy(tracker); } catch { }
                }
                if (calibration != IntPtr.Zero)
                {
                    try { Marshal.FreeHGlobal(calibration); } catch { }
                }
                watchdog?.Dispose();
                mmf?.Dispose();
                try { inputEvt?.Dispose(); } catch { }
                try { outputEvt?.Dispose(); } catch { }
                try { readyEvt?.Dispose(); } catch { }
                try { shutdownEvt?.Dispose(); } catch { }
            }
        }

        private static int RunLoop(
            WorkerArgs parsed,
            MmfRegion mmf,
            IntPtr tracker,
            EventWaitHandle inputEvt,
            EventWaitHandle outputEvt,
            EventWaitHandle shutdownEvt)
        {
            int depthBytes = parsed.DepthW * parsed.DepthH * 2;
            int irBytes = parsed.IrW * parsed.IrH * 2;
            byte[] depthScratch = new byte[depthBytes];
            byte[] irScratch = new byte[irBytes];

            ulong lastSeenInputFrameId = 0;
            // Per-output-slot writer state. Even values mean committed; we always
            // begin from "lastWrittenSeq+1" to keep the seq strictly monotonic.
            ulong[] outputSeq = new ulong[K4abtWorkerSharedLayout.OutputSlotCount];
            int nextOutputSlot = 0;

            // Per-second diagnostic counters (worker-local).
            int diagEnqOk = 0, diagEnqDropped = 0, diagPopped = 0, diagBodies = 0, diagTorn = 0;
            DateTime diagStart = DateTime.UtcNow;

            WaitHandle[] waits = new WaitHandle[] { shutdownEvt, inputEvt };

            while (true)
            {
                int idx = WaitHandle.WaitAny(waits, K4abtWorkerSharedLayout.InputWaitTimeoutMs);
                if (idx == 0)
                {
                    WorkerLog.Info("[Loop] shutdown signalled");
                    break;
                }
                if (idx == WaitHandle.WaitTimeout) continue;
                // idx == 1: input_event was set.

                uint slotIdx = mmf.ReadLatestInputSlot();
                if (slotIdx >= K4abtWorkerSharedLayout.InputSlotCount)
                {
                    diagTorn++;
                    continue;
                }
                if (!mmf.TryReadInputSlot((int)slotIdx, depthScratch, irScratch, out var snap))
                {
                    diagTorn++;
                    continue;
                }
                if (snap.FrameId <= lastSeenInputFrameId)
                {
                    // Either the host signalled twice for the same slot, or we beat the
                    // writer to a slot it'd already overwritten. Skip silently.
                    continue;
                }
                lastSeenInputFrameId = snap.FrameId;
                mmf.IncrementConsumed();

                bool depthValid = (snap.Flags & K4abtWorkerSharedLayout.InputFlagDepthValid) != 0;
                bool irValid = (snap.Flags & K4abtWorkerSharedLayout.InputFlagIrValid) != 0;
                if (!depthValid)
                {
                    // Without depth there is nothing for the tracker to chew on.
                    continue;
                }

                ulong tsUsec = snap.TsNs / 1000UL;
                IntPtr capture = K4ACaptureBridge.CreateCaptureFromDepthAndIR(
                    depthScratch, depthBytes, parsed.DepthW, parsed.DepthH,
                    irValid ? irScratch : null,
                    irValid ? irBytes : 0,
                    irValid ? parsed.IrW : 0,
                    irValid ? parsed.IrH : 0,
                    tsUsec);
                if (capture == IntPtr.Zero) continue;

                var enq = K4ABTNative.k4abt_tracker_enqueue_capture(
                    tracker, capture, K4abtWorkerSharedLayout.EnqueueTimeoutMs);
                K4ANative.k4a_capture_release(capture);
                if (enq != k4a_wait_result_t.K4A_WAIT_RESULT_SUCCEEDED)
                {
                    diagEnqDropped++;
                    continue;
                }
                diagEnqOk++;

                var pop = K4ABTNative.k4abt_tracker_pop_result(
                    tracker, out IntPtr bodyFrame, K4abtWorkerSharedLayout.PopTimeoutMs);
                if (pop != k4a_wait_result_t.K4A_WAIT_RESULT_SUCCEEDED)
                {
                    continue;
                }

                try
                {
                    diagPopped++;
                    int outSlot = nextOutputSlot;
                    nextOutputSlot = (nextOutputSlot + 1) % K4abtWorkerSharedLayout.OutputSlotCount;

                    ulong writingSeq = mmf.BeginWriteOutputSlot(outSlot, outputSeq[outSlot]);
                    uint nWritten = SkeletonOutputWriter.Write(mmf, outSlot, bodyFrame, snap.FrameId, snap.TsNs);
                    diagBodies += (int)nWritten;
                    outputSeq[outSlot] = mmf.CommitOutputSlot(outSlot, writingSeq);
                    mmf.WriteLatestOutputSlot((uint)outSlot);
                    outputEvt.Set();
                }
                finally
                {
                    K4ABTNative.k4abt_frame_release(bodyFrame);
                }

                // Per-second diagnostic blob → stdout (host pipes it into Unity's console).
                var elapsed = DateTime.UtcNow - diagStart;
                if (elapsed.TotalSeconds >= 1.0)
                {
                    WorkerLog.Info(
                        $"[Diag] enq_ok={diagEnqOk}/s enq_dropped={diagEnqDropped}/s " +
                        $"popped={diagPopped}/s bodies={diagBodies}/s torn={diagTorn}/s " +
                        $"produced={mmf.ReadProduced()} consumed={mmf.ReadConsumed()} " +
                        $"dropped={mmf.ReadDropped()}");
                    diagEnqOk = diagEnqDropped = diagPopped = diagBodies = diagTorn = 0;
                    diagStart = DateTime.UtcNow;
                }
            }

            return K4abtWorkerSharedLayout.ExitOk;
        }
    }
}
