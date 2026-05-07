// Static facade so shared code (K4ACaptureBridge, etc.) can log without taking
// an IWorkerLogger as a parameter. The Unity bootstrap installs a Unity-aware
// implementation at startup; the worker installs a Console-based one in main().

using System;

namespace BodyTracking.Shared
{
    public static class WorkerLog
    {
        private static IWorkerLogger s_logger = new ConsoleWorkerLogger();

        public static void SetLogger(IWorkerLogger logger)
        {
            s_logger = logger ?? new ConsoleWorkerLogger();
        }

        public static void Info(string message) => s_logger.Log(WorkerLogLevel.Info, message);
        public static void Warning(string message) => s_logger.Log(WorkerLogLevel.Warning, message);
        public static void Error(string message) => s_logger.Log(WorkerLogLevel.Error, message);
    }

    public sealed class ConsoleWorkerLogger : IWorkerLogger
    {
        public void Log(WorkerLogLevel level, string message)
        {
            string prefix = level switch
            {
                WorkerLogLevel.Error => "[ERROR] ",
                WorkerLogLevel.Warning => "[WARN] ",
                _ => string.Empty,
            };
            var writer = level == WorkerLogLevel.Error ? Console.Error : Console.Out;
            writer.WriteLine(prefix + message);
        }
    }
}
