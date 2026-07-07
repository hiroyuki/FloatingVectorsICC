// Copies the OrbbecSDK `extensions/` tree into the built player after a
// Standalone Windows build.
//
// Why this is needed: OrbbecRuntime points the native SDK at
// `Application.dataPath/Plugins/OrbbecSDK/extensions` (depthengine / filters /
// frameprocessor / firmwareupdater). In the Editor that resolves to
// `Assets/Plugins/OrbbecSDK/extensions`, which exists. But Unity treats those
// .dll files as native plugins and RELOCATES them into
// `<build>_Data/Plugins/x86_64/` (flattening the folder structure), so the
// `extensions/` directory the SDK expects is absent from the build. Without it
// `ob_pipeline_start_with_config` fails with "DeviceComponentPtr is nullptr"
// and live capture never starts (only the floor grid shows).
//
// This post-build step copies the extensions tree verbatim to
// `<build>_Data/Plugins/OrbbecSDK/extensions`, replicating the working Editor
// layout. Editor play is unaffected (it already reads from Assets/Plugins).

using System;
using System.IO;
using UnityEditor;
using UnityEditor.Build;
using UnityEditor.Build.Reporting;
using UnityEngine;
using Shared.EditorTools;

namespace Orbbec.EditorTools
{
    public sealed class OrbbecBuildPostprocessor : IPostprocessBuildWithReport
    {
        public int callbackOrder => 0;

        // Source tree, relative to the project root (Application.dataPath == <project>/Assets).
        private const string SourceRel = "Plugins/OrbbecSDK/extensions";
        // Destination inside the player's data folder, matching OrbbecRuntime's lookup:
        // Application.dataPath/Plugins/OrbbecSDK/extensions.
        private const string DestRel = "Plugins/OrbbecSDK/extensions";

        public void OnPostprocessBuild(BuildReport report)
        {
            var target = report.summary.platform;
            if (!WindowsBuildCopy.IsWindowsTarget(target))
                return;

            string exePath = report.summary.outputPath;
            if (!WindowsBuildCopy.TryGetBuildDirectory(exePath, out string buildDir))
            {
                Debug.LogWarning("[OrbbecBuildPostprocessor] no build output path; skipping extensions copy.");
                return;
            }

            string productDataDir =
                Path.Combine(buildDir, Path.GetFileNameWithoutExtension(exePath) + "_Data");
            if (!Directory.Exists(productDataDir))
            {
                Debug.LogWarning($"[OrbbecBuildPostprocessor] data dir not found: {productDataDir}; " +
                                 "skipping extensions copy.");
                return;
            }

            // Application.dataPath at build time == <project>/Assets.
            string src = Path.Combine(Application.dataPath, SourceRel);
            string dst = Path.Combine(productDataDir, DestRel);

            if (!Directory.Exists(src))
            {
                Debug.LogWarning($"[OrbbecBuildPostprocessor] source extensions not found: {src}; " +
                                 "live capture will fail in the build.");
                return;
            }

            try
            {
                int files = WindowsBuildCopy.CopyTreeSkippingMeta(src, dst);
                Debug.Log($"[OrbbecBuildPostprocessor] copied {files} OrbbecSDK extension file(s) to {dst}");
            }
            catch (Exception e)
            {
                Debug.LogError($"[OrbbecBuildPostprocessor] failed to copy extensions to {dst}: {e}");
            }
        }
    }
}
