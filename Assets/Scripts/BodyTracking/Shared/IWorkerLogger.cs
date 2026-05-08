// Logger contract used by code shared between Unity and the standalone worker
// .exe. Unity binds a UnityEngine.Debug-backed implementation; the worker uses
// a Console-backed default. Keeping this surface tiny avoids dragging Unity
// types into the shared assembly.

namespace BodyTracking.Shared
{
    public enum WorkerLogLevel
    {
        Info,
        Warning,
        Error,
    }

    public interface IWorkerLogger
    {
        void Log(WorkerLogLevel level, string message);
    }
}
