// Multi-device coordinator. Enumerates Femto Bolts on Start and spawns
// one PointCloudRenderer GameObject per device. Owns the OrbbecRuntime
// lifecycle for the scene.

using System.Collections.Generic;
using Orbbec;
using UnityEngine;

namespace PointCloud
{
    public class PointCloudCameraManager : MonoBehaviour
    {
        [Tooltip("Material assigned to spawned renderers.")]
        public Material defaultPointMaterial;

        [Header("Per-device defaults")]
        public uint depthWidth = 640;
        public uint depthHeight = 576;
        public uint depthFps = 30;
        public uint colorWidth = 1280;
        public uint colorHeight = 720;
        public uint colorFps = 30;
        [Tooltip("Stream that depth gets aligned TO via the Align filter (D2C target).")]
        public ObStreamType alignTargetStream = ObStreamType.Color;
        public int maxPointsPerDevice = 1280 * 720;

        [Tooltip("Optional shared bounding box applied to every spawned renderer.")]
        public PointCloudBoundingBox defaultBoundingBox;

        [Tooltip("Optional shared decimater applied to every spawned renderer.")]
        public PointCloudDecimater defaultDecimater;

        [Tooltip("Optional shared cumulative snapshotter applied to every spawned renderer.")]
        public PointCloudCumulative defaultCumulative;

        [Header("Multi-device sync")]
        [Tooltip("Enable hardware multi-device sync via Sync Hub Pro. Index 0 is configured as " +
                 "PRIMARY, the rest as SECONDARY_SYNCED. When off, every device runs standalone.")]
        public bool enableHardwareSync = true;
        [Tooltip("Call ob_device_timer_sync_with_host on each device at startup so their frame " +
                 "timestamps share a host-time reference.")]
        public bool enableTimerSyncWithHost = true;

        [Header("Diagnostics")]
        public bool verboseLogging = true;

        public IReadOnlyList<PointCloudRenderer> Renderers => _renderers;

        private readonly List<PointCloudRenderer> _renderers = new List<PointCloudRenderer>();

        private void Start()
        {
            var ctx = OrbbecRuntime.Context;
            var devices = ctx.QueryDevices();
            if (verboseLogging)
                Debug.Log($"[{nameof(PointCloudCameraManager)}] Found {devices.Count} device(s).");

            for (int i = 0; i < devices.Count; i++)
            {
                var d = devices[i];
                if (verboseLogging)
                    Debug.Log($"  [{i}] {d}");
                _renderers.Add(SpawnRenderer(d, i));
            }
        }

        private void OnDestroy()
        {
            // Renderers Dispose themselves on OnDestroy; nothing to do here for them.
            // Tear down the shared context after children are gone.
            // Unity destroys children before parents in default scene teardown,
            // but to be safe we defer context shutdown to next frame via Application.quitting in OrbbecRuntime.
            OrbbecRuntime.RequestShutdown();
        }

        private PointCloudRenderer SpawnRenderer(OrbbecDeviceDescriptor desc, int index)
        {
            var go = new GameObject($"PointCloud[{index}] {desc.Name} ({desc.Serial})");
            go.transform.SetParent(transform, worldPositionStays: false);
            go.AddComponent<MeshFilter>();
            go.AddComponent<MeshRenderer>();
            var pcr = go.AddComponent<PointCloudRenderer>();

            pcr.deviceSerial = desc.Serial;
            pcr.depthWidth = depthWidth;
            pcr.depthHeight = depthHeight;
            pcr.depthFps = depthFps;
            pcr.colorWidth = colorWidth;
            pcr.colorHeight = colorHeight;
            pcr.colorFps = colorFps;
            pcr.alignTargetStream = alignTargetStream;
            pcr.maxPoints = maxPointsPerDevice;
            pcr.pointMaterial = defaultPointMaterial;
            pcr.boundingBox = defaultBoundingBox;
            pcr.decimater = defaultDecimater;
            pcr.cumulative = defaultCumulative;
            pcr.syncMode = ResolveSyncMode(index);
            pcr.timerSyncWithHost = enableTimerSyncWithHost;

            return pcr;
        }

        private ObMultiDeviceSyncMode ResolveSyncMode(int index)
        {
            if (!enableHardwareSync) return ObMultiDeviceSyncMode.Standalone;
            // Sync Hub Pro topology: first device is the clock source, rest follow.
            return index == 0 ? ObMultiDeviceSyncMode.Primary : ObMultiDeviceSyncMode.SecondarySynced;
        }
    }
}
