// Routes WorkerLog calls (used inside shared code that also runs in the
// standalone worker .exe) into Unity's Debug.Log channels so warnings/errors
// from K4ACaptureBridge, WorkerBootstrap, etc. show up in the Editor console.
// Bound explicitly by BodyTrackingBootstrap.Initialize so we don't depend on
// RuntimeInitializeOnLoad ordering relative to the bootstrap.

using BodyTracking.Shared;
using UnityEngine;

namespace BodyTracking
{
    public sealed class UnityWorkerLogger : IWorkerLogger
    {
        public void Log(WorkerLogLevel level, string message)
        {
            switch (level)
            {
                case WorkerLogLevel.Error:
                    Debug.LogError(message);
                    break;
                case WorkerLogLevel.Warning:
                    Debug.LogWarning(message);
                    break;
                default:
                    Debug.Log(message);
                    break;
            }
        }
    }
}
