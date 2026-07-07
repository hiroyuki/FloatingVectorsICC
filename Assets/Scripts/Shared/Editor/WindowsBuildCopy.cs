// Shared scaffolding for Windows-standalone build postprocessors
// (OrbbecBuildPostprocessor / BodyTrackingBuildPostprocessor): the platform
// gate, build-directory derivation from the player exe path, and the
// .meta-skipping recursive copy. Each postprocessor keeps its own paths,
// warnings and copy shape.

using System;
using System.IO;
using UnityEditor;

namespace Shared.EditorTools
{
    public static class WindowsBuildCopy
    {
        /// <summary>True for the Standalone Windows player targets.</summary>
        public static bool IsWindowsTarget(BuildTarget target) =>
            target == BuildTarget.StandaloneWindows64 || target == BuildTarget.StandaloneWindows;

        /// <summary>
        /// Derive the build directory (folder containing the player exe) from the
        /// build report's outputPath. Returns false when outputPath is empty;
        /// the caller logs its own warning and skips.
        /// </summary>
        public static bool TryGetBuildDirectory(string outputPath, out string buildDir)
        {
            if (string.IsNullOrEmpty(outputPath))
            {
                buildDir = null;
                return false;
            }
            buildDir = Path.GetDirectoryName(outputPath);
            return true;
        }

        // Recursively copies src -> dst, overwriting, and skipping Unity .meta
        // sidecars. Returns the number of files copied.
        public static int CopyTreeSkippingMeta(string src, string dst)
        {
            Directory.CreateDirectory(dst);
            int count = 0;
            foreach (string file in Directory.GetFiles(src))
            {
                if (file.EndsWith(".meta", StringComparison.OrdinalIgnoreCase)) continue;
                File.Copy(file, Path.Combine(dst, Path.GetFileName(file)), overwrite: true);
                count++;
            }
            foreach (string dir in Directory.GetDirectories(src))
                count += CopyTreeSkippingMeta(dir, Path.Combine(dst, Path.GetFileName(dir)));
            return count;
        }
    }
}
