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
        [Header("Spawn")]
        [Tooltip("Prefab containing PointCloudRenderer + MeshFilter + MeshRenderer. " +
                 "If null, a bare GameObject with those components is created instead.")]
        public GameObject rendererPrefab;

        [Tooltip("Material assigned to spawned renderers when no prefab is set.")]
        public Material defaultPointMaterial;

        [Header("Per-device defaults (used when rendererPrefab is null)")]
        public uint depthWidth = 640;
        public uint depthHeight = 576;
        public uint depthFps = 30;
        public uint colorWidth = 1280;
        public uint colorHeight = 720;
        public uint colorFps = 30;
        [Tooltip("Stream that depth gets aligned TO via the Align filter (D2C target).")]
        public ObStreamType alignTargetStream = ObStreamType.Color;
        public int maxPointsPerDevice = 1280 * 720;

        [Tooltip("Optional shared bounding box applied to every spawned renderer. " +
                 "Only used when rendererPrefab is null (bare GameObject path).")]
        public PointCloudBoundingBox defaultBoundingBox;

        [Tooltip("Optional shared decimater applied to every spawned renderer. " +
                 "Only used when rendererPrefab is null (bare GameObject path).")]
        public PointCloudDecimater defaultDecimater;

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
            GameObject go;
            if (rendererPrefab != null)
            {
                go = Instantiate(rendererPrefab, transform);
            }
            else
            {
                go = new GameObject("PointCloudRenderer");
                go.transform.SetParent(transform, worldPositionStays: false);
                go.AddComponent<MeshFilter>();
                go.AddComponent<MeshRenderer>();
                go.AddComponent<PointCloudRenderer>();
            }

            go.name = $"PointCloud[{index}] {desc.Name} ({desc.Serial})";

            var pcr = go.GetComponent<PointCloudRenderer>();
            if (pcr == null)
            {
                Debug.LogError($"[{nameof(PointCloudCameraManager)}] " +
                               $"Spawned object has no PointCloudRenderer component.", go);
                return null;
            }

            pcr.deviceSerial = desc.Serial;

            // Apply manager defaults only when a bare GameObject was created
            // (so an authored prefab keeps its own values).
            if (rendererPrefab == null)
            {
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
            }

            return pcr;
        }
    }
}
