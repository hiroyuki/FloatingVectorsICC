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

        // The CUDA stack goes first and onnxruntime LAST: loading onnxruntime.dll before
        // its CUDA dependencies are resident fails its init routine outright
        // (ERROR_DLL_INIT_FAILED), and a failed pin means the bare-name lookup later finds
        // the System32 inbox copy — the exact outcome this is meant to prevent. Order
        // within the CUDA group is cosmetic; the loader resolves each DLL's static imports
        // from the same directory via LOAD_WITH_ALTERED_SEARCH_PATH.
        //
        // TensorRT is deliberately absent: its provider DLL needs nvinfer*, which is not
        // shipped, so preloading it only ever logs a spurious ERROR_MOD_NOT_FOUND that
        // buries the failures worth reading.
        private static readonly string[] CudaPrefixes =
        {
            "cudart64_", "nvrtc-builtins", "nvrtc64_", "cublasLt64_",
            "cublas64_", "cufft64_", "cudnn",
        };

        private static readonly string[] SkipContains = { "tensorrt" };

        /// <summary>Whether the ONNX Runtime now resident in this process is the one from
        /// our plugin directory. False means a foreign copy won (the Windows inbox build in
        /// System32 is the one that does this), and calling into the managed binding would
        /// take the process down rather than fail — so callers must skip ORT entirely.</summary>
        public static bool OrtPinned { get; private set; }

        /// <summary>Full path of the loaded onnxruntime.dll, or null if none is resident.
        /// Worth logging verbatim: it is the single fact that distinguishes "our 1.26" from
        /// "the OS's 1.17".</summary>
        public static string OrtPath { get; private set; }

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
            LoadGroup(dir, CudaPrefixes, LoadWithAlteredSearchPath, ref ok, ref fail);

            // Only onnxruntime.dll is pinned, and only it needs to be: it is the sole name
            // that collides with a copy already reachable by name (System32 has no
            // onnxruntime_providers_*). ORT loads its own provider DLLs out of its own
            // directory when an execution provider is appended, which is both correct and
            // the only way they link — preloading them here just produced failures that
            // buried the one line worth reading.
            string ortDll = Path.Combine(dir, "onnxruntime.dll");
            if (LoadLibraryExW(ortDll, IntPtr.Zero, 0) != IntPtr.Zero) ok++;
            else
            {
                fail++;
                Debug.LogWarning($"[rtmpose] ORT preload failed (err={Marshal.GetLastWin32Error()}): {ortDll}");
            }

            // Verify rather than trust the return code: a successful LoadLibrary only means
            // SOME module of that name is resident, and if a foreign copy got there first
            // the loader hands back that one. The resident path is the only proof.
            // Compare canonicalised paths — dir comes from Application.dataPath and carries
            // forward slashes, while the OS reports the module with backslashes, so a raw
            // StartsWith reports a foreign DLL even when our own is the one loaded.
            OrtPath = FindLoadedModule("onnxruntime.dll");
            string residentDir = OrtPath != null ? Path.GetDirectoryName(Path.GetFullPath(OrtPath)) : null;
            OrtPinned = residentDir != null && string.Equals(
                residentDir.TrimEnd('\\', '/'),
                Path.GetFullPath(dir).TrimEnd('\\', '/'),
                StringComparison.OrdinalIgnoreCase);

            Debug.Log($"[rtmpose] ORT/CUDA preload from {dir}: {ok} loaded, {fail} failed; " +
                      $"onnxruntime resident at {OrtPath ?? "(none)"} -> pinned={OrtPinned}");
            if (!OrtPinned && OrtPath != null)
                Debug.LogError("[rtmpose] a foreign onnxruntime.dll won the load — the ORT managed " +
                               "binding would crash the process against it, so RTMPose will be skipped.");
        }

        // Absolute paths throughout, so `flags` only ever changes how each DLL's own imports
        // are resolved — never which file we load.
        private static void LoadGroup(string dir, string[] prefixes, uint flags, ref int ok, ref int fail)
        {
            foreach (string prefix in prefixes)
            {
                foreach (string path in Directory.GetFiles(dir, prefix + "*.dll"))
                {
                    bool skip = false;
                    foreach (string s in SkipContains)
                        if (path.IndexOf(s, StringComparison.OrdinalIgnoreCase) >= 0) { skip = true; break; }
                    if (skip) continue;

                    if (LoadLibraryExW(path, IntPtr.Zero, flags) != IntPtr.Zero) ok++;
                    else
                    {
                        fail++;
                        Debug.LogWarning($"[rtmpose] preload failed (err={Marshal.GetLastWin32Error()}): {path}");
                    }
                }
            }
        }

        private static string FindLoadedModule(string moduleName)
        {
            try
            {
                foreach (System.Diagnostics.ProcessModule m in
                         System.Diagnostics.Process.GetCurrentProcess().Modules)
                    if (string.Equals(m.ModuleName, moduleName, StringComparison.OrdinalIgnoreCase))
                        return m.FileName;
            }
            catch (Exception e)
            {
                // Module enumeration is a diagnostic, never a reason to fail startup.
                Debug.LogWarning($"[rtmpose] could not enumerate modules: {e.Message}");
            }
            return null;
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
