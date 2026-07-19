// Pins the CUDA 13 / cuDNN 9 (cuda13 build) DLLs that live next to the ORT
// plugin BEFORE ONNX Runtime resolves them by bare name.
//
// Why this exists: the cuDNN shim (cudnn64_9.dll) loads its sub-libraries
// (cudnn_graph64_9.dll etc.) by NAME, and the Windows loader searches PATH
// after the app dir. A machine with a CUDA 12 toolkit on PATH can carry a
// cu12 build of cuDNN under the IDENTICAL file names and file version
// (e.g. v12.6\bin\cudnn64_9.dll, also 9.24.0.43) — mixing those into a cu13
// ORT silently changes the inference build flavor, which measurably shifts
// numerics (see eval/PLAN_live_gpu.md, Phase 3 追補 2). Preloading every
// library from the plugin directory by absolute path wins all later
// name-based lookups, because LoadLibrary returns an already-loaded module
// regardless of the requested path.

using System;
using System.IO;
using System.Runtime.InteropServices;
using UnityEngine;

namespace BodyTracking.Eval.Rtmpose
{
    internal static class CudaDllPreload
    {
        private static bool s_done;

        // Prefix order only matters for logging clarity; the loader resolves
        // each DLL's static imports from the same directory via
        // LOAD_WITH_ALTERED_SEARCH_PATH, so any order links correctly.
        private static readonly string[] Prefixes =
        {
            "cudart64_", "nvrtc-builtins", "nvrtc64_", "cublasLt64_",
            "cublas64_", "cufft64_", "cudnn",
        };

        [DllImport("kernel32", CharSet = CharSet.Unicode, SetLastError = true)]
        private static extern IntPtr LoadLibraryExW(string path, IntPtr reserved, uint flags);

        private const uint LoadWithAlteredSearchPath = 0x00000008;

        public static void EnsureLoaded()
        {
            if (s_done) return;
            s_done = true;
            if (Application.platform != RuntimePlatform.WindowsEditor &&
                Application.platform != RuntimePlatform.WindowsPlayer) return;

            string dir = FindPluginDir();
            if (dir == null) return; // machine relies on PATH (e.g. toolkit install) — nothing to pin

            int ok = 0, fail = 0;
            foreach (string prefix in Prefixes)
            {
                foreach (string path in Directory.GetFiles(dir, prefix + "*.dll"))
                {
                    if (LoadLibraryExW(path, IntPtr.Zero, LoadWithAlteredSearchPath) != IntPtr.Zero) ok++;
                    else
                    {
                        fail++;
                        Debug.LogWarning($"[rtmpose] CUDA preload failed (err={Marshal.GetLastWin32Error()}): {path}");
                    }
                }
            }
            if (ok > 0 || fail > 0)
                Debug.Log($"[rtmpose] CUDA preload from {dir}: {ok} loaded, {fail} failed");
        }

        private static string FindPluginDir()
        {
            // cudnn64_9.dll marks a plugin dir that carries its own CUDA stack.
            string[] candidates =
            {
                // editor: embedded package
                Path.GetFullPath(Path.Combine(Application.dataPath,
                    "../Packages/com.github.asus4.onnxruntime/Plugins/Windows/x64")),
                // standalone player: flattened plugin dir
                Path.Combine(Application.dataPath, "Plugins/x86_64"),
            };
            foreach (string c in candidates)
                if (File.Exists(Path.Combine(c, "cudnn64_9.dll"))) return c;
            return null;
        }
    }
}
