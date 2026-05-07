// Adds the K4A Wrapper bin directory and the Azure Kinect Body Tracking SDK
// runtime bin directory to the process DLL search path before any P/Invoke
// into k4a / k4abt happens. The DLLs are NOT copied into Assets/Plugins/
// (they total ~70MB+ and the SDKs are installed system-wide), so we instead
// extend PATH at process start so Windows resolves them on demand.
//
// The actual PATH manipulation is delegated to the shared WorkerBootstrap so
// the standalone k4abt_worker.exe (Workers/K4abtWorker) reuses the same code
// without dragging UnityEngine into the worker build.

using BodyTracking.Shared;
using UnityEngine;

#if UNITY_EDITOR
using UnityEditor;
#endif

namespace BodyTracking
{
    public static class BodyTrackingBootstrap
    {
        // Re-exported for any caller that still wants a direct reference to the path.
        public const string K4AWrapperBin = WorkerBootstrap.K4AWrapperBinDefault;
        public const string BodyTrackingBin = WorkerBootstrap.BodyTrackingBinDefault;

        private static bool _initialized;

#if UNITY_EDITOR
        [InitializeOnLoadMethod]
        private static void EditorInit() => Initialize();
#endif

        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.SubsystemRegistration)]
        private static void RuntimeInit() => Initialize();

        // Diagnostic kill switch — leave false in normal operation. Setting to true skips
        // the PATH prepend, which makes k4abt P/Invoke fail with DllNotFoundException but
        // can be useful when ruling out PATH-related interference.
        public static bool DisableForDiagnostics = false;

        public static void Initialize()
        {
            if (_initialized) return;
            _initialized = true;

            // Bind the Unity-aware logger eagerly so any warnings emitted during
            // PrependDllSearchPaths surface in the Editor console rather than the
            // process stdout (which Unity does not capture for in-Editor static ctors).
            WorkerLog.SetLogger(new UnityWorkerLogger());

            if (DisableForDiagnostics)
            {
                Debug.LogWarning(
                    "[BodyTrackingBootstrap] DisableForDiagnostics=true — skipping PATH " +
                    "prepend. Body tracking will fail with DllNotFoundException; flip the " +
                    "flag back to false once point cloud rendering is verified.");
                return;
            }

            WorkerBootstrap.PrependDllSearchPaths();
        }
    }
}
