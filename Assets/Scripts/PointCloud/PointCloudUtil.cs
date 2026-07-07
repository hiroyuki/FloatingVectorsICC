// Small shared helpers for the PointCloud assembly. Behavior-neutral
// extractions of idioms that were previously duplicated across
// PointCloudRenderer / SensorRecorder / CameraHealthMonitor /
// BoundingVolume / CameraPoseMarker / FloorOrigin / PointCloudCumulative.

using UnityEngine;

namespace PointCloud
{
    internal static class PointCloudUtil
    {
        /// <summary>
        /// Last <paramref name="n"/> characters of <paramref name="s"/>.
        /// Null / empty / already-short strings pass through unchanged.
        /// Serials only differ in the tail (CL8F253004N → 4N), so a short
        /// suffix is enough to identify a camera in logs and alert banners.
        /// </summary>
        public static string TailSerial(string s, int n) =>
            string.IsNullOrEmpty(s) || s.Length <= n ? s : s.Substring(s.Length - n);

        /// <summary>
        /// Destroy in play mode, DestroyImmediate in edit mode. No-op on null.
        /// </summary>
        public static void DestroySafe(Object o)
        {
            if (o == null) return;
            if (Application.isPlaying) Object.Destroy(o);
            else Object.DestroyImmediate(o);
        }

        /// <summary>
        /// Standard config for the project's unlit point-cloud / viz renderers:
        /// no shadow casting or receiving, no light / reflection probes.
        /// </summary>
        public static void ConfigureUnlitRenderer(MeshRenderer mr)
        {
            mr.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
            mr.receiveShadows = false;
            mr.lightProbeUsage = UnityEngine.Rendering.LightProbeUsage.Off;
            mr.reflectionProbeUsage = UnityEngine.Rendering.ReflectionProbeUsage.Off;
        }
    }
}
