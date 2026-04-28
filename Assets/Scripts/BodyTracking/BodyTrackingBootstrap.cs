// Adds the K4A Wrapper bin directory and the Azure Kinect Body Tracking SDK
// runtime bin directory to the process DLL search path before any P/Invoke
// into k4a / k4abt happens. The DLLs are NOT copied into Assets/Plugins/
// (they total ~70MB+ and the SDKs are installed system-wide), so we instead
// extend PATH at process start so Windows resolves them on demand.
//
// Order matters. SubsystemRegistration runs before scene load and before any
// MonoBehaviour.Start, which is the earliest hook Unity exposes for runtime
// player code; InitializeOnLoad covers the Editor.

using System;
using System.IO;
using UnityEngine;

#if UNITY_EDITOR
using UnityEditor;
#endif

namespace BodyTracking
{
    public static class BodyTrackingBootstrap
    {
        // Canonical install paths (also documented in CLAUDE.md). If you move the
        // K4A Wrapper or install BT SDK to a non-default location, edit here.
        public const string K4AWrapperBin =
            @"D:\OrbbecSDK_K4A_Wrapper_v2.0.11_windows_202510221441\bin";

        public const string BodyTrackingBin =
            @"C:\Program Files\Azure Kinect Body Tracking SDK\sdk\windows-desktop\amd64\release\bin";

        private static bool _initialized;

#if UNITY_EDITOR
        [InitializeOnLoadMethod]
        private static void EditorInit() => Initialize();
#endif

        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.SubsystemRegistration)]
        private static void RuntimeInit() => Initialize();

        public static void Initialize()
        {
            if (_initialized) return;
            _initialized = true;

            string current = Environment.GetEnvironmentVariable("PATH") ?? string.Empty;
            string prepend = string.Empty;

            if (Directory.Exists(K4AWrapperBin))
            {
                prepend += K4AWrapperBin + ";";
            }
            else
            {
                Debug.LogWarning(
                    $"[BodyTrackingBootstrap] K4A Wrapper bin not found at {K4AWrapperBin}. " +
                    "Body tracking P/Invoke will fail until the path is corrected (see CLAUDE.md).");
            }

            if (Directory.Exists(BodyTrackingBin))
            {
                prepend += BodyTrackingBin + ";";
            }
            else
            {
                Debug.LogWarning(
                    $"[BodyTrackingBootstrap] BT SDK bin not found at {BodyTrackingBin}. " +
                    "Install Azure Kinect Body Tracking SDK 1.1.2 or update the path.");
            }

            if (prepend.Length > 0)
            {
                Environment.SetEnvironmentVariable("PATH", prepend + current);
            }
        }
    }
}
