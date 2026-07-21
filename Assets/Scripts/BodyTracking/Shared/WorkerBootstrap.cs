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
        // The BT SDK installs to a fixed, versionless location, so it can stay a
        // constant. The Orbbec K4A Wrapper is a zip whose folder name carries both
        // the version and the build timestamp (…_v2.0.11_windows_202510221441),
        // so it differs per machine: the 4070 set has v2.0.12, the 5080 set
        // v2.0.11. Hard-coding either — or pinning it in the scene, which the two
        // sets share — breaks the other machine, so it is discovered instead.
        public const string BodyTrackingBinDefault =
            @"C:\Program Files\Azure Kinect Body Tracking SDK\sdk\windows-desktop\amd64\release\bin";

        /// <summary>Environment variable that pins the wrapper bin, checked before
        /// any disk search. The per-machine escape hatch that needs no rebuild.</summary>
        public const string K4AWrapperBinEnvVar = "FVICC_K4A_WRAPPER_BIN";

        private const string kWrapperFolderPrefix = "OrbbecSDK_K4A_Wrapper";
        private const string kWrapperProbeDll = "k4a.dll";

        // Drive roots scanned for the wrapper folder, in order. Kept to the drive
        // roots on purpose: that is where the zip is unpacked on both sets, and a
        // deep recursive scan on a spinning disk would stall worker startup.
        private static readonly string[] kWrapperSearchRoots = { @"D:\", @"C:\", @"E:\", @"F:\" };

        /// <summary>
        /// Locate the Orbbec K4A Wrapper bin directory, or null when nothing
        /// plausible exists. Resolution order: the FVICC_K4A_WRAPPER_BIN env var,
        /// then <c>&lt;root&gt;\OrbbecSDK_K4A_Wrapper*\bin</c> over the drive roots.
        /// A candidate only counts if it actually contains k4a.dll, so an empty or
        /// half-extracted folder does not shadow a good one. When several match,
        /// the highest-sorting folder name wins — the names are version- and
        /// timestamp-ordered, so that is the newest build.
        /// </summary>
        public static string ResolveK4AWrapperBin()
        {
            string pinned = Environment.GetEnvironmentVariable(K4AWrapperBinEnvVar);
            if (!string.IsNullOrEmpty(pinned) && HasProbeDll(pinned)) return pinned;

            string best = null;
            for (int i = 0; i < kWrapperSearchRoots.Length; i++)
            {
                string root = kWrapperSearchRoots[i];
                string[] dirs;
                try
                {
                    if (!Directory.Exists(root)) continue;
                    dirs = Directory.GetDirectories(root, kWrapperFolderPrefix + "*");
                }
                catch (Exception)
                {
                    continue;   // unreadable / disconnected drive — just skip it
                }
                for (int d = 0; d < dirs.Length; d++)
                {
                    string bin = Path.Combine(dirs[d], "bin");
                    if (!HasProbeDll(bin)) continue;
                    if (best == null || string.CompareOrdinal(dirs[d], best) > 0) best = dirs[d];
                }
                if (best != null) return Path.Combine(best, "bin");  // earlier roots win
            }
            return null;
        }

        private static bool HasProbeDll(string binDir)
        {
            try { return !string.IsNullOrEmpty(binDir) && File.Exists(Path.Combine(binDir, kWrapperProbeDll)); }
            catch (Exception) { return false; }
        }

        /// <summary>
        /// Prepend K4A wrapper bin and BT SDK bin to PATH. Returns true if at least
        /// one path was prepended. Missing paths are reported via WorkerLog.Warning
        /// but are not fatal here — the caller decides how strict to be.
        /// </summary>
        public static bool PrependDllSearchPaths(string k4aWrapperBin = null, string btSdkBin = null)
        {
            bool discovered = false;
            if (string.IsNullOrEmpty(k4aWrapperBin))
            {
                k4aWrapperBin = ResolveK4AWrapperBin();
                discovered = k4aWrapperBin != null;
            }
            if (string.IsNullOrEmpty(btSdkBin)) btSdkBin = BodyTrackingBinDefault;

            string current = Environment.GetEnvironmentVariable("PATH") ?? string.Empty;
            string prepend = string.Empty;

            if (!string.IsNullOrEmpty(k4aWrapperBin) && Directory.Exists(k4aWrapperBin))
            {
                prepend += k4aWrapperBin + ";";
                if (discovered)
                    WorkerLog.Info($"[WorkerBootstrap] K4A Wrapper found at {k4aWrapperBin}.");
            }
            else
            {
                WorkerLog.Warning(
                    "[WorkerBootstrap] K4A Wrapper bin not found" +
                    (string.IsNullOrEmpty(k4aWrapperBin)
                        ? $" — no {kWrapperFolderPrefix}*\\bin containing {kWrapperProbeDll} under " +
                          string.Join(", ", kWrapperSearchRoots) + ". Unpack the wrapper at a drive " +
                          $"root, set {K4AWrapperBinEnvVar}, or pass --k4a-wrapper-bin."
                        : $" at {k4aWrapperBin}.") +
                    " Body tracking P/Invoke will fail until the path is corrected.");
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
