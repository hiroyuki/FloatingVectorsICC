// Pins ONNX Runtime and the CUDA 13 / cuDNN 9 (cuda13 build) DLLs that live next
// to the ORT plugin BEFORE anything resolves them by bare name.
//
// onnxruntime.dll itself is the one that MUST be pinned. Windows 11 ships its own
// inbox copy in System32 (Windows ML; fileVersion 1.17.x "os-germanium"), and the
// player's plugin directory is not on the OS search path, so a bare-name lookup
// finds the System32 one first. The managed binding here is built against 1.26 and
// calls exports the 1.17 inbox build does not have (OrtGetCompileApi), which is not
// a graceful failure — it is an access violation that takes the process down at
// OrtRtmposeBackend's constructor. The Editor happens to resolve the package copy
// and is therefore immune, so this only ever bites a standalone build. The System32
// copy is an OS component and cannot be removed.
//
// The CUDA stack needs the same treatment for a different reason: the cuDNN shim
// (cudnn64_9.dll) loads its sub-libraries
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

        // onnxruntime goes first because it is the one competing with a copy that is
        // already reachable by name (System32); the rest only compete with whatever a
        // machine happens to have on PATH. Beyond that the order is cosmetic — the
        // loader resolves each DLL's static imports from the same directory via
        // LOAD_WITH_ALTERED_SEARCH_PATH, so any order links correctly.
        private static readonly string[] Prefixes =
        {
            "onnxruntime",
            "cudart64_", "nvrtc-builtins", "nvrtc64_", "cublasLt64_",
            "cublas64_", "cufft64_", "cudnn",
        };

        [DllImport("kernel32", CharSet = CharSet.Unicode, SetLastError = true)]
        private static extern IntPtr LoadLibraryExW(string path, IntPtr reserved, uint flags);

        private const uint LoadWithAlteredSearchPath = 0x00000008;

        // Pin at player startup, NOT merely at the top of OrtRtmposeBackend's constructor.
        // That call site looks early enough but is not: OrtRtmposeBackend has an instance
        // field initializer (`new RunOptions()`), and C# runs instance field initializers
        // BEFORE the constructor body — so the ORT native library was already resolved,
        // by bare name, before the pin ever ran. Anything that touches an ONNX Runtime
        // managed type before this hook re-opens the same hole, so keep the pin here where
        // no call ordering can outrun it.
        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterAssembliesLoaded)]
        private static void PinAtStartup() => EnsureLoaded();

        public static void EnsureLoaded()
        {
            if (s_done) return;
            s_done = true;
            if (Application.platform != RuntimePlatform.WindowsEditor &&
                Application.platform != RuntimePlatform.WindowsPlayer) return;

            string dir = FindPluginDir();
            if (dir == null) return; // no plugin dir found — nothing of ours to pin

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
                Debug.Log($"[rtmpose] ORT/CUDA preload from {dir}: {ok} loaded, {fail} failed");
        }

        private static string FindPluginDir()
        {
            // onnxruntime.dll marks the plugin dir. Keyed on it rather than on
            // cudnn64_9.dll so that a machine without the bundled CUDA stack still
            // gets ORT pinned — that pin is what keeps the System32 inbox build from
            // being picked up, and it matters whether or not cuDNN is present.
            string[] candidates =
            {
                // editor: embedded package
                Path.GetFullPath(Path.Combine(Application.dataPath,
                    "../Packages/com.github.asus4.onnxruntime/Plugins/Windows/x64")),
                // standalone player: flattened plugin dir
                Path.Combine(Application.dataPath, "Plugins/x86_64"),
            };
            foreach (string c in candidates)
                if (File.Exists(Path.Combine(c, "onnxruntime.dll"))) return c;
            return null;
        }
    }
}
