// Pure-.NET helper for prepending the K4A Wrapper bin and Azure Kinect Body
// Tracking SDK bin to the process PATH so subsequent k4a / k4abt P/Invoke
// calls can resolve their DLLs. Used both by Unity (via BodyTrackingBootstrap)
// and by the standalone worker .exe.

using System;
using System.IO;

namespace BodyTracking.Shared
{
    public static class WorkerBootstrap
    {
        // Canonical install paths (also documented in CLAUDE.md). Override via
        // arguments if SDKs are installed elsewhere.
        public const string K4AWrapperBinDefault =
            @"D:\OrbbecSDK_K4A_Wrapper_v2.0.11_windows_202510221441\bin";
        public const string BodyTrackingBinDefault =
            @"C:\Program Files\Azure Kinect Body Tracking SDK\sdk\windows-desktop\amd64\release\bin";

        /// <summary>
        /// Prepend K4A wrapper bin and BT SDK bin to PATH. Returns true if at least
        /// one path was prepended. Missing paths are reported via WorkerLog.Warning
        /// but are not fatal here — the caller decides how strict to be.
        /// </summary>
        public static bool PrependDllSearchPaths(string k4aWrapperBin = null, string btSdkBin = null)
        {
            if (string.IsNullOrEmpty(k4aWrapperBin)) k4aWrapperBin = K4AWrapperBinDefault;
            if (string.IsNullOrEmpty(btSdkBin)) btSdkBin = BodyTrackingBinDefault;

            string current = Environment.GetEnvironmentVariable("PATH") ?? string.Empty;
            string prepend = string.Empty;

            if (Directory.Exists(k4aWrapperBin))
            {
                prepend += k4aWrapperBin + ";";
            }
            else
            {
                WorkerLog.Warning(
                    $"[WorkerBootstrap] K4A Wrapper bin not found at {k4aWrapperBin}. " +
                    "Body tracking P/Invoke will fail until the path is corrected.");
            }

            if (Directory.Exists(btSdkBin))
            {
                prepend += btSdkBin + ";";
            }
            else
            {
                WorkerLog.Warning(
                    $"[WorkerBootstrap] BT SDK bin not found at {btSdkBin}. " +
                    "Install Azure Kinect Body Tracking SDK 1.1.2 or pass --bt-sdk-bin.");
            }

            if (prepend.Length == 0) return false;
            Environment.SetEnvironmentVariable("PATH", prepend + current);
            return true;
        }
    }
}
