// Standalone k4abt worker process. Receives depth+IR frames from Unity through
// a per-camera memory-mapped file, runs the Microsoft Azure Kinect Body
// Tracking SDK on each capture, and writes skeleton results back through the
// same MMF. See Plans/issue-10-k4abt-worker-process.md for the full IPC
// contract; this Phase 2 stub only proves the project builds and links the
// shared interop layer. Phase 3 fills in the actual main loop.

using System;
using BodyTracking.Shared;

namespace K4abtWorker
{
    internal static class Program
    {
        private static int Main(string[] args)
        {
            WorkerLog.SetLogger(new ConsoleWorkerLogger());

            // Phase 2: argument parsing not wired up yet — this entrypoint exists so
            // the .csproj produces a runnable single-file exe that can be smoke-tested
            // before the IPC loop lands in Phase 3.
            WorkerLog.Info(
                $"k4abt_worker starting (layout v{K4abtWorkerSharedLayout.LayoutVersion}, " +
                $"args={args.Length}). Phase 3 will replace this with the real main loop.");

            return K4abtWorkerSharedLayout.ExitOk;
        }
    }
}
