// Multi-device coordinator. Enumerates Femto Bolts on Start and spawns
// one PointCloudRenderer GameObject per device. Owns the OrbbecRuntime
// lifecycle for the scene.

using System;
using System.Collections.Generic;
using System.IO;
using Calibration;
using Orbbec;
using UnityEngine;

namespace PointCloud
{
    public class PointCloudCameraManager : MonoBehaviour
    {
        [Header("Mode")]
        [Tooltip("When ON, Start() does NOT enumerate Femto Bolt devices or spawn live " +
                 "PointCloudRenderers — Play simply drives PointCloudRecorder playback of " +
                 "whatever recording PointCloudRecorder.folderPath points at. Use to test " +
                 "scenes without the camera rig plugged in. When OFF the original live " +
                 "capture path runs.")]
        public bool playbackOnly = false;

        [Tooltip("Material assigned to spawned renderers.")]
        public Material defaultPointMaterial;

        [Header("Per-device defaults")]
        public uint depthWidth = 640;
        public uint depthHeight = 576;
        public uint depthFps = 30;
        public uint colorWidth = 1280;
        public uint colorHeight = 720;
        public uint colorFps = 30;
        [Tooltip("Color stream pixel format applied to every spawned renderer. RGB = uncompressed " +
                 "(saturates USB3 with 4 cameras at 1080p). MJPG = compressed (~10x lighter, " +
                 "matches OrbbecViewer's default).")]
        public ObFormat colorFormat = ObFormat.RGB;
        [Tooltip("Stream that depth gets aligned TO via the Align filter (D2C target).")]
        public ObStreamType alignTargetStream = ObStreamType.Color;

        [Tooltip("Optional shared bounding box applied to every spawned renderer.")]
        public PointCloudBoundingBox defaultBoundingBox;

        [Tooltip("Optional shared decimater applied to every spawned renderer.")]
        public PointCloudDecimater defaultDecimater;

        [Tooltip("Optional shared cumulative snapshotter applied to every spawned renderer.")]
        public PointCloudCumulative defaultCumulative;

        [Header("Multi-device sync")]
        [Tooltip("Sync mode applied to every spawned renderer. SyncHubPro: all devices set to " +
                 "Secondary (the hub generates trigger pulses on VSYNC, every camera receives " +
                 "via VSYNC_IN). DaisyChain: index 0 = Primary (generates the pulse), rest = " +
                 "Secondary (forwards to the next camera). Note: Femto Bolt firmware exposes " +
                 "Secondary, not SecondarySynced — observed bitmap 0x000F = FreeRun|Standalone|" +
                 "Primary|Secondary.")]
        public SyncTopology syncTopology = SyncTopology.SyncHubPro;
        [Tooltip("When true, every spawned renderer rewrites the device's sync config on pipeline " +
                 "start using syncTopology above. Set false to leave the device's existing sync " +
                 "mode alone (e.g. when configured externally via OrbbecViewer / Sync Hub Pro " +
                 "tooling). syncTopology is ignored in that case.")]
        public bool applySyncConfig = true;
        [Tooltip("Stagger trigger2ImageDelayUs across devices to reduce iToF NIR pulse interference. " +
                 "Each device gets index * step microseconds. 0 disables the stagger.")]
        [Min(0)]
        public int trigger2ImageDelayStepUs = 160;
        [Tooltip("Call ob_device_timer_sync_with_host on each device at startup so their frame " +
                 "timestamps share a host-time reference.")]
        public bool enableTimerSyncWithHost = true;
        [Tooltip("Call ob_device_enable_global_timestamp(true) so frame.GlobalTimestampUs becomes " +
                 "host-clock-aligned. The recorder uses this as the recording timestamp.")]
        public bool enableGlobalTimestamp = true;

        public enum SyncTopology
        {
            /// <summary>Sync Hub Pro: hub fans trigger out, cam0 = Primary (seeds the chain), rest = Secondary.</summary>
            SyncHubPro = 0,
            /// <summary>Camera-to-camera daisy chain: cam0 = Primary, rest = Secondary.</summary>
            DaisyChain = 1,
            /// <summary>No hardware sync; each device runs Standalone (use only for single-camera or testing).</summary>
            Standalone = 2,
        }

        [Header("Depth work mode")]
        [Tooltip("Depth work mode name applied to every spawned renderer before its pipeline starts. " +
                 "Leave empty to keep each device's current mode. Run the renderer's 'Log Depth Work " +
                 "Modes' context menu to list the available names for your firmware.")]
        public string depthWorkMode = string.Empty;

        [Header("Display")]
        [Tooltip("Hide / show all point cloud renderers. Toggling at runtime disables the " +
                 "MeshRenderer on each spawned PointCloudRenderer GO without stopping capture, " +
                 "so re-enabling resumes immediately with no pipeline restart.")]
        public bool showPointClouds = true;

        [Header("Extrinsics (issue #9)")]
        [Tooltip("Apply per-device global_tr_colorCamera read from <extrinsicsRoot>/calibration/" +
                 "extrinsics.yaml to each spawned renderer GO. Off → renderers stay at the manager's " +
                 "local origin (legacy behavior).")]
        public bool applyExtrinsics = false;
        [Tooltip("Root directory for extrinsics.yaml lookup. Same root convention as " +
                 "PointCloudRecorder.folderPath. Empty → Application.persistentDataPath/Recordings/recording.")]
        public string extrinsicsRoot = string.Empty;

        [Header("Diagnostics")]
        public bool verboseLogging = true;

        public IReadOnlyList<PointCloudRenderer> Renderers => _renderers;

        private readonly List<PointCloudRenderer> _renderers = new List<PointCloudRenderer>();
        private bool _lastAppliedShowPointClouds = true;

        private void Start()
        {
            if (playbackOnly)
            {
                if (verboseLogging)
                    Debug.Log($"[{nameof(PointCloudCameraManager)}] playbackOnly=true; skipping device enumeration. " +
                              "PointCloudRecorder will drive playback from its folderPath.");
                return;
            }
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

            if (applyExtrinsics) ApplyExtrinsicsToLive();
        }

        private void Update()
        {
            // Live-toggle the MeshRenderer on each spawned point cloud GO without stopping
            // capture. Capture thread keeps producing frames so re-enabling is instant.
            if (showPointClouds == _lastAppliedShowPointClouds) return;
            for (int i = 0; i < _renderers.Count; i++)
            {
                var r = _renderers[i];
                if (r == null) continue;
                var mr = r.GetComponent<MeshRenderer>();
                if (mr != null && mr.enabled != showPointClouds) mr.enabled = showPointClouds;
            }
            _lastAppliedShowPointClouds = showPointClouds;
        }

        private void OnDestroy()
        {
            // Renderers Dispose themselves on OnDestroy; nothing to do here for them.
            // Tear down the shared context after children are gone.
            // Unity destroys children before parents in default scene teardown,
            // but to be safe we defer context shutdown to next frame via Application.quitting in OrbbecRuntime.
            OrbbecRuntime.RequestShutdown();
        }

        /// <summary>
        /// Destroy every spawned PointCloudRenderer GameObject so its capture
        /// thread / OrbbecSDK pipeline tears down and releases the USB device.
        /// Called by PointCloudRecorder.Read so playback runs without the live
        /// cameras competing for USB bandwidth / GPU. Manager itself stays alive
        /// but won't re-spawn (Start only ran once). Re-connecting requires
        /// reloading the scene (or re-entering Play mode).
        /// </summary>
        public void DestroyAllRenderers()
        {
            for (int i = 0; i < _renderers.Count; i++)
            {
                var r = _renderers[i];
                if (r == null) continue;
                // GameObject destruction triggers PointCloudRenderer.OnDestroy
                // which joins the capture thread and disposes the pipeline.
                UnityEngine.Object.Destroy(r.gameObject);
            }
            _renderers.Clear();
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
            pcr.colorFormat = colorFormat;
            pcr.alignTargetStream = alignTargetStream;
            pcr.maxPoints = ResolveMaxPointsPerDevice();
            pcr.pointMaterial = defaultPointMaterial;
            pcr.boundingBox = defaultBoundingBox;
            pcr.decimater = defaultDecimater;
            pcr.cumulative = defaultCumulative;
            pcr.syncMode = ResolveSyncMode(index);
            pcr.applySyncConfig = applySyncConfig;
            pcr.trigger2ImageDelayUs = trigger2ImageDelayStepUs * index;
            pcr.timerSyncWithHost = enableTimerSyncWithHost;
            pcr.enableGlobalTimestamp = enableGlobalTimestamp;
            pcr.depthWorkMode = depthWorkMode;

            return pcr;
        }

        /// <summary>
        /// Read <c><extrinsicsRoot>/calibration/extrinsics.yaml</c> and apply each
        /// device's <c>global_tr_colorCamera</c> to the matching spawned renderer GO
        /// (sets <c>localPosition</c> + <c>localRotation</c>; leaves <c>localScale</c>
        /// alone so the existing per-renderer Y flip stays). Devices without a
        /// matching yaml entry keep their default transform.
        /// </summary>
        public void ApplyExtrinsicsToLive()
        {
            string root = ResolveExtrinsicsRoot();
            string path = Path.Combine(PointCloudRecording.CalibrationDir(root), "extrinsics.yaml");
            if (!File.Exists(path))
            {
                if (verboseLogging)
                    Debug.Log($"[{nameof(PointCloudCameraManager)}] applyExtrinsics: no file at {path}, skipping.");
                return;
            }

            IReadOnlyList<PointCloudRecording.DeviceCalibration> calibrations;
            try
            {
                calibrations = PointCloudRecording.ReadExtrinsicsYaml(root);
            }
            catch (Exception e)
            {
                Debug.LogWarning(
                    $"[{nameof(PointCloudCameraManager)}] applyExtrinsics: parse failed for {path}: {e.Message}",
                    this);
                return;
            }

            int applied = 0;
            foreach (var c in calibrations)
            {
                var renderer = FindRendererBySerial(c.Serial);
                if (renderer == null) continue;
                if (!c.GlobalTrColorCamera.HasValue) continue;
                ExtrinsicsApply.ApplyToTransform(renderer.transform, c.GlobalTrColorCamera.Value);
                applied++;
            }
            if (verboseLogging)
                Debug.Log(
                    $"[{nameof(PointCloudCameraManager)}] applyExtrinsics: applied to {applied}/{_renderers.Count} renderer(s) from {path}.");
        }

        /// <summary>
        /// Resolves <see cref="extrinsicsRoot"/> to an absolute path: empty string
        /// defaults to <c>&lt;persistentDataPath&gt;/Recordings/recording</c>;
        /// non-empty relative paths are taken under <c>persistentDataPath</c>.
        /// Exposed so PointCloudRecorder.Save/Read can inherit the same calibration
        /// the live renderers used — recordings inherit the rig's current setup
        /// rather than starting from identity in a fresh folder.
        /// </summary>
        public string ResolveExtrinsicsRoot()
        {
            string p = extrinsicsRoot;
            if (string.IsNullOrWhiteSpace(p))
                p = Path.Combine(Application.persistentDataPath, "Recordings", "recording");
            else if (!Path.IsPathRooted(p))
                p = Path.Combine(Application.persistentDataPath, p);
            return p;
        }

        private PointCloudRenderer FindRendererBySerial(string serial)
        {
            for (int i = 0; i < _renderers.Count; i++)
            {
                var r = _renderers[i];
                if (r != null && r.deviceSerial == serial) return r;
            }
            return null;
        }

        /// <summary>
        /// Derive the per-device point buffer size from the active align target
        /// stream — D2C aligns depth to color so the aligned cloud has
        /// colorWidth*colorHeight points; C2D would produce depthWidth*depthHeight.
        /// </summary>
        private int ResolveMaxPointsPerDevice()
        {
            if (alignTargetStream == ObStreamType.Depth)
                return checked((int)(depthWidth * depthHeight));
            return checked((int)(colorWidth * colorHeight));
        }

        private ObMultiDeviceSyncMode ResolveSyncMode(int index)
        {
            switch (syncTopology)
            {
                case SyncTopology.SyncHubPro:
                case SyncTopology.DaisyChain:
                    // Femto Bolt sync requires exactly one Primary even when a Sync Hub Pro
                    // is fanning the trigger pulses. cam0 is Primary so its VSYNC OUT seeds
                    // the chain / hub; the remaining cameras are Secondary receivers.
                    return index == 0 ? ObMultiDeviceSyncMode.Primary : ObMultiDeviceSyncMode.Secondary;
                default:
                    return ObMultiDeviceSyncMode.Standalone;
            }
        }
    }
}
