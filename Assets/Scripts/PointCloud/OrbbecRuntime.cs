// Process-wide OrbbecContext singleton with explicit shutdown.
// PointCloudCameraManager owns the lifecycle for a scene; renderers
// reference the shared context.

using System.IO;
using Orbbec;
using UnityEngine;

namespace PointCloud
{
    public static class OrbbecRuntime
    {
        private static OrbbecContext s_context;
        private static readonly object s_lock = new object();
        private static bool s_shutdownHooked;

        public static OrbbecContext Context
        {
            get
            {
                lock (s_lock)
                {
                    if (s_context == null)
                    {
                        // Point the SDK at the bundled extensions/ tree before creating the context.
                        var extDir = Path.Combine(Application.dataPath, "Plugins", "OrbbecSDK", "extensions");
                        if (Directory.Exists(extDir))
                        {
                            OrbbecContext.SetExtensionsDirectory(extDir);
                        }
                        else
                        {
                            Debug.LogWarning($"[OrbbecRuntime] extensions dir not found: {extDir}");
                        }
                        s_context = new OrbbecContext();
                        Application.quitting += OnApplicationQuitting;
                        s_shutdownHooked = true;
                    }
                    return s_context;
                }
            }
        }

        /// <summary>Tear down on the next frame boundary (after MonoBehaviour OnDestroy).</summary>
        public static void RequestShutdown()
        {
            // Defer to Application.quitting; if we're already past that, dispose now.
            if (!s_shutdownHooked) Shutdown();
        }

        private static void OnApplicationQuitting()
        {
            Shutdown();
        }

        private static void Shutdown()
        {
            lock (s_lock)
            {
                if (s_context != null)
                {
                    s_context.Dispose();
                    s_context = null;
                }
            }
        }
    }
}
