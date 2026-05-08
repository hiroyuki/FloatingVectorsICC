// Background thread that polls the parent (Unity) process at a fixed cadence
// and signals shutdown if the parent disappears. Without this, Unity force-
// quitting (or a process crash that bypasses the shutdown event) would leave
// the worker .exe alive.
//
// Per the plan we don't try to be clever about the polling — Process.GetProcessById
// throws ArgumentException once the parent exits, which is the cheap, portable
// signal we need. 1 Hz is plenty (worker startup is ~5 s, Unity's Editor lifetime
// is minutes/hours, an extra second of tail latency before self-shutdown is fine).

using System;
using System.Diagnostics;
using System.Threading;
using BodyTracking.Shared;

namespace K4abtWorker
{
    internal sealed class ParentWatchdog : IDisposable
    {
        private readonly int _parentPid;
        private readonly EventWaitHandle _shutdownEvent;
        private readonly Thread _thread;
        private readonly CancellationTokenSource _cts = new CancellationTokenSource();
        public bool ParentExited { get; private set; }

        public ParentWatchdog(int parentPid, EventWaitHandle shutdownEvent)
        {
            _parentPid = parentPid;
            _shutdownEvent = shutdownEvent;
            _thread = new Thread(Run)
            {
                Name = "K4abtWorker.ParentWatchdog",
                IsBackground = true,
            };
        }

        public void Start() => _thread.Start();

        private void Run()
        {
            var token = _cts.Token;
            while (!token.IsCancellationRequested)
            {
                if (!IsParentAlive())
                {
                    ParentExited = true;
                    WorkerLog.Warning(
                        $"[ParentWatchdog] parent PID {_parentPid} no longer running — signalling shutdown");
                    try { _shutdownEvent.Set(); } catch { /* worker already tearing down */ }
                    return;
                }

                try { Thread.Sleep(K4abtWorkerSharedLayout.ParentWatchdogIntervalMs); }
                catch (ThreadInterruptedException) { /* fall through to re-check token */ }
            }
        }

        private bool IsParentAlive()
        {
            try
            {
                using var p = Process.GetProcessById(_parentPid);
                // HasExited is the authoritative signal; GetProcessById alone returns a stale
                // handle that may belong to an already-exited process for a few seconds.
                return !p.HasExited;
            }
            catch (ArgumentException)
            {
                // No process with that PID exists.
                return false;
            }
            catch (InvalidOperationException)
            {
                return false;
            }
        }

        public void Dispose()
        {
            _cts.Cancel();
            try { _thread.Interrupt(); } catch { }
            try { _thread.Join(2000); } catch { }
            _cts.Dispose();
        }
    }
}
