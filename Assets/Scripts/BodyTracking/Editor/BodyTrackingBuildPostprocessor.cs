// Copies the published k4abt_worker.exe into the built player after a Standalone
// Windows build.
//
// Why this is needed: K4abtWorkerHost.ResolveExePath resolves the relative
// workerExePath ("Workers/K4abtWorker/bin/Release/net8.0-windows/win-x64/
// publish/k4abt_worker.exe") against `Application.dataPath/..`. In the Editor
// that is the project root, where `dotnet publish` put the exe. In a player
// build it is the build root, which has no Workers/ tree — so every StartWorker
// fails with "worker exe not found" and body tracking never runs.
//
// The worker is a framework-dependent single-file publish (~200 KB); its native
// deps (k4abt / onnxruntime / DirectML / k4a wrapper / depthengine / ONNX model)
// are resolved at runtime from PATH by WorkerBootstrap, which points at the
// machine's installed SDKs — so only the exe needs to travel with the build.
// (.NET 8 desktop runtime must be installed on the target, same as the dev box.)
//
// This copies the published exe (+ .pdb if present) to
// `<build>/Workers/K4abtWorker/bin/Release/net8.0-windows/win-x64/publish/`,
// matching the default workerExePath. If the publish is missing the step warns
// and does nothing (run `dotnet publish -c Release -r win-x64` in
// Workers/K4abtWorker first).

using System;
using System.IO;
using UnityEditor;
using UnityEditor.Build;
using UnityEditor.Build.Reporting;
using UnityEngine;
using Shared.EditorTools;

namespace BodyTracking.EditorTools
{
    public sealed class BodyTrackingBuildPostprocessor : IPostprocessBuildWithReport
    {
        public int callbackOrder => 0;

        // Relative to the project root in the Editor / the build root in a player —
        // matches K4abtWorkerHost.workerExePath's default and its ResolveExePath.
        private const string WorkerRelDir =
            "Workers/K4abtWorker/bin/Release/net8.0-windows/win-x64/publish";
        private const string WorkerExe = "k4abt_worker.exe";

        public void OnPostprocessBuild(BuildReport report)
        {
            var target = report.summary.platform;
            if (!WindowsBuildCopy.IsWindowsTarget(target))
                return;

            string exePath = report.summary.outputPath;
            // Build root = folder containing the player exe. ResolveExePath uses
            // Application.dataPath/.. which is this same folder in a player.
            if (!WindowsBuildCopy.TryGetBuildDirectory(exePath, out string buildRoot))
            {
                Debug.LogWarning("[BodyTrackingBuildPostprocessor] no build output path; skipping worker copy.");
                return;
            }
            // Editor: Application.dataPath == <project>/Assets, so project root is its parent.
            string projectRoot = Path.GetFullPath(Path.Combine(Application.dataPath, ".."));

            string srcDir = Path.Combine(projectRoot, WorkerRelDir);
            string srcExe = Path.Combine(srcDir, WorkerExe);
            if (!File.Exists(srcExe))
            {
                Debug.LogWarning(
                    $"[BodyTrackingBuildPostprocessor] published worker not found at {srcExe}; " +
                    "body tracking will not run in the build. Run " +
                    "`dotnet publish -c Release -r win-x64` in Workers/K4abtWorker first.");
                return;
            }

            string dstDir = Path.Combine(buildRoot, WorkerRelDir);
            try
            {
                Directory.CreateDirectory(dstDir);
                File.Copy(srcExe, Path.Combine(dstDir, WorkerExe), overwrite: true);
                string srcPdb = Path.Combine(srcDir, "k4abt_worker.pdb");
                if (File.Exists(srcPdb))
                    File.Copy(srcPdb, Path.Combine(dstDir, "k4abt_worker.pdb"), overwrite: true);
                Debug.Log($"[BodyTrackingBuildPostprocessor] copied k4abt_worker.exe to {dstDir}");
            }
            catch (Exception e)
            {
                Debug.LogError($"[BodyTrackingBuildPostprocessor] failed to copy worker to {dstDir}: {e}");
            }
        }
    }
}
