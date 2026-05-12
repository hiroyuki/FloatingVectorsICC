// Rec / Play / Save / Read for raw Femto Bolt data. Records the raw Y16 depth + RGB8
// color frames emitted by one or more PointCloudRenderers and stores them in memory.
// On Save, each device's streams are written to the Scanned-Reality-style layout
//   <root>/dataset/<host>/FemtoBolt_<serial>/{depth_main,color_main}
// along with configuration.yaml / dataset.yaml / sensor_node_config.yaml,
// <host>/hostinfo.yaml and calibration/extrinsics.yaml (intrinsics + D2C extrinsic
// per device; global_tr_colorCamera defaults to identity until Phase E adds ChArUco
// calibration).
//
// On Play, points are reconstructed per-frame from raw depth + color using the stored
// intrinsics and depth-to-color extrinsic: each depth pixel is unprojected into the
// depth camera's frame, transformed into the color camera's frame, and the color
// sampled by reprojecting onto the color image. Output is emitted in color-camera
// space (meters), which aligns with how global_tr_colorCamera is defined.

using System;
using System.Collections.Generic;
using System.IO;
using Orbbec;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using UnityEngine;
using UnityEngine.Rendering;

namespace PointCloud
{
    [DisallowMultipleComponent]
    public class PointCloudRecorder : MonoBehaviour
    {
        public enum State { Idle, Recording, Playing }

        [Header("Source")]
        [Tooltip("Camera manager whose spawned PointCloudRenderers will be recorded. " +
                 "If null, all PointCloudRenderers in the scene are used.")]
        public PointCloudCameraManager cameraManager;

        [Tooltip("Material used for playback meshes. If null, falls back to each source renderer's pointMaterial.")]
        public Material playbackMaterial;

        [Header("Files")]
        [Tooltip("Root folder for recordings. Relative paths resolve under Application.persistentDataPath. " +
                 "Leave empty to use '<persistentDataPath>/Recordings/recording'.")]
        public string folderPath = "";

        [Tooltip("Dataset name written into dataset.yaml. Defaults to the recording folder name.")]
        public string datasetName = "";

        [Header("Playback")]
        public bool loop = true;

        [Min(0.01f)]
        [Tooltip("Playback rate multiplier (1.0 = real time).")]
        public float playbackRate = 1.0f;

        /// <summary>
        /// Fires once per advanced playback frame per device, with the same payload
        /// shape a live PointCloudRenderer would produce in <c>OnRawFramesReady</c>.
        /// Subscribers (e.g. BodyTrackingMultiLive) use this to run merge / inference
        /// against recorded footage; frame-by-frame stepping then works via Unity's
        /// built-in Pause + Step buttons because cursor advancement is driven by
        /// <see cref="Time.timeAsDouble"/>.
        /// </summary>
        public event System.Action<string, ObCameraParam?, Transform, RawFrameData> OnPlaybackRawFrame;

        // --- Runtime state ---
        public State CurrentState { get; private set; } = State.Idle;
        public int RecordedFrameCount
        {
            get
            {
                int n = 0;
                foreach (var kv in _tracks) n += kv.Value.DepthFrames.Count;
                return n;
            }
        }

        /// <summary>Duration in seconds of the longest per-device track.</summary>
        public float RecordedDuration
        {
            get
            {
                ulong maxSpan = 0;
                foreach (var kv in _tracks)
                {
                    var track = kv.Value;
                    if (track.DepthFrames.Count < 2) continue;
                    ulong first = track.DepthFrames[0].TimestampNs;
                    ulong last = track.DepthFrames[track.DepthFrames.Count - 1].TimestampNs;
                    ulong span = last - first;
                    if (span > maxSpan) maxSpan = span;
                }
                return maxSpan / 1_000_000_000f;
            }
        }

        public int DeviceCount => _tracks.Count;
        public string StatusMessage { get; private set; } = "";

        /// <summary>Raised after Read() finishes loading raw recordings into memory. Subscribers
        /// (e.g. BodyTrackingPlayback) can use it to kick off post-Read processing of the
        /// freshly populated tracks. Not raised for Rec/Save.</summary>
        public event Action OnTracksLoaded;

        /// <summary>Read-only view of one recorded device's raw depth track + the calibration
        /// captured at record (or load) time. Returned by <see cref="GetRecordedDepthTracks"/>.</summary>
        public sealed class RecordedDepthTrack
        {
            public string Serial;
            public int DepthWidth;
            public int DepthHeight;
            public int ColorWidth;
            public int ColorHeight;
            public int IRWidth;
            public int IRHeight;
            public ObCameraParam? CameraParam;
            // World ← color camera transform (issue #9 / Phase E). Loaded from
            // calibration/extrinsics.yaml on Read; null when the file is absent or
            // the device entry has identity (single-camera shortcut).
            public ObExtrinsic? GlobalTrColorCamera;
            public IReadOnlyList<PointCloudRecording.Frame> DepthFrames;
            // IR frames recorded alongside depth (Y16, depth-aligned dimensions). May be empty
            // when IR streaming was off at record time, in which case Playback BT falls back
            // to using depth bytes as a stand-in IR (and the BT model rejects all detections).
            public IReadOnlyList<PointCloudRecording.Frame> IRFrames;
        }

        /// <summary>Snapshot of all loaded depth tracks (post-Read or post-Rec). Frames list is
        /// the recorder's live list; do not mutate from outside.</summary>
        public IReadOnlyList<RecordedDepthTrack> GetRecordedDepthTracks()
        {
            var list = new List<RecordedDepthTrack>(_tracks.Count);
            foreach (var kv in _tracks)
            {
                var t = kv.Value;
                list.Add(new RecordedDepthTrack
                {
                    Serial = t.Serial,
                    DepthWidth = t.DepthWidth,
                    DepthHeight = t.DepthHeight,
                    ColorWidth = t.ColorWidth,
                    ColorHeight = t.ColorHeight,
                    IRWidth = t.IRWidth,
                    IRHeight = t.IRHeight,
                    CameraParam = t.CameraParam,
                    GlobalTrColorCamera = t.GlobalTrColorCamera,
                    DepthFrames = t.DepthFrames,
                    IRFrames = t.IRFrames,
                });
            }
            return list;
        }

        // --- Internals ---

        private sealed class DeviceTrack
        {
            public string Serial;

            // Raw sensor data.
            public readonly List<PointCloudRecording.Frame> DepthFrames = new List<PointCloudRecording.Frame>();
            public readonly List<PointCloudRecording.Frame> ColorFrames = new List<PointCloudRecording.Frame>();
            public readonly List<PointCloudRecording.Frame> IRFrames = new List<PointCloudRecording.Frame>();
            public int DepthWidth, DepthHeight;
            public int ColorWidth, ColorHeight;
            public int IRWidth, IRHeight;

            // Calibration captured at record time (or loaded from YAML on Read).
            public ObCameraParam? CameraParam;
            // World ← color camera transform (issue #9 / Phase E). Loaded from
            // calibration/extrinsics.yaml on Read.
            public ObExtrinsic? GlobalTrColorCamera;

            // Playback plumbing.
            public GameObject PlaybackObject;
            public MeshFilter PlaybackFilter;
            public MeshRenderer PlaybackRenderer;
            public Mesh PlaybackMesh;
            public int PlaybackMeshCapacity;
            public int PlaybackCursor;
            public NativeArray<ObColorPoint> ReconstructBuffer; // scratch for Reconstruct (sized once)
        }

        private readonly Dictionary<string, DeviceTrack> _tracks = new Dictionary<string, DeviceTrack>();
        private readonly List<PointCloudRenderer> _subscribed = new List<PointCloudRenderer>();
        private readonly Dictionary<PointCloudRenderer, Action<PointCloudRenderer, RawFrameData>>
            _subscribedHandlers = new Dictionary<PointCloudRenderer, Action<PointCloudRenderer, RawFrameData>>();
        private double _playbackWallStart;
        private ulong _playbackTrackStartNs;

        // --- Public API (invoked by Inspector buttons or runtime UI) ---

        [ContextMenu("Toggle Rec")]
        public void ToggleRecord()
        {
            if (CurrentState == State.Recording) StopRecording();
            else StartRecording();
        }

        [ContextMenu("Toggle Play")]
        public void TogglePlay()
        {
            if (CurrentState == State.Playing) StopPlayback();
            else StartPlayback();
        }

        [ContextMenu("Save")]
        public void Save()
        {
            if (_tracks.Count == 0)
            {
                SetStatus("Save: nothing to save (no recorded frames).", warn: true);
                return;
            }
            try
            {
                string root = ResolveRoot();
                string host = SafeMachineName();
                var serials = new List<string>();
                var calibrations = new List<PointCloudRecording.DeviceCalibration>();
                int totalDepthFrames = 0;
                int totalColorFrames = 0;
                int totalIRFrames = 0;

                // Preserve existing global_tr_colorCamera values written by the
                // CalibrationWindow (issue #9). Save used to overwrite extrinsics.yaml
                // with identity transforms, wiping the calibration; now we read the
                // file first and pass the prior per-serial value through.
                var existingGlobalBySerial = new Dictionary<string, ObExtrinsic>();
                try
                {
                    string extPath = Path.Combine(PointCloudRecording.CalibrationDir(root), "extrinsics.yaml");
                    if (File.Exists(extPath))
                    {
                        foreach (var cal in PointCloudRecording.ReadExtrinsicsYaml(root))
                        {
                            if (cal.GlobalTrColorCamera.HasValue)
                                existingGlobalBySerial[cal.Serial] = cal.GlobalTrColorCamera.Value;
                        }
                    }
                }
                catch (Exception preserveEx)
                {
                    Debug.LogWarning(
                        $"[{nameof(PointCloudRecorder)}] could not read existing extrinsics.yaml " +
                        $"to preserve global_tr_colorCamera: {preserveEx.Message}", this);
                }

                // Compute centroid of camera positions so extrinsics.yaml puts world origin at
                // the centroid for multi-device setups (identity for a single device).
                var centroid = Vector3.zero;
                int withParam = 0;
                foreach (var kv in _tracks)
                {
                    if (kv.Value.CameraParam.HasValue)
                    {
                        // For single-host single-rig setups we don't have a marker yet, so each
                        // camera's own color-camera frame is its own origin; centroid across
                        // cameras stays at zero. Kept as a scaffold for Phase E.
                        withParam++;
                    }
                }
                _ = centroid; _ = withParam;

                foreach (var kv in _tracks)
                {
                    var track = kv.Value;
                    serials.Add(track.Serial);

                    if (track.DepthFrames.Count > 0)
                    {
                        string depthPath = PointCloudRecording.SensorFilePath(
                            root, host, track.Serial, PointCloudRecording.DepthSensorName);
                        PointCloudRecording.WriteRcsv(
                            depthPath,
                            PointCloudRecording.BuildDepthHeaderYaml(track.Serial, track.DepthWidth, track.DepthHeight),
                            track.DepthFrames);
                        totalDepthFrames += track.DepthFrames.Count;
                    }
                    if (track.ColorFrames.Count > 0)
                    {
                        string colorPath = PointCloudRecording.SensorFilePath(
                            root, host, track.Serial, PointCloudRecording.ColorSensorName);
                        PointCloudRecording.WriteRcsv(
                            colorPath,
                            PointCloudRecording.BuildColorHeaderYaml(track.Serial, track.ColorWidth, track.ColorHeight),
                            track.ColorFrames);
                        totalColorFrames += track.ColorFrames.Count;
                    }
                    if (track.IRFrames.Count > 0)
                    {
                        string irPath = PointCloudRecording.SensorFilePath(
                            root, host, track.Serial, PointCloudRecording.IRSensorName);
                        PointCloudRecording.WriteRcsv(
                            irPath,
                            PointCloudRecording.BuildIRHeaderYaml(track.Serial, track.IRWidth, track.IRHeight),
                            track.IRFrames);
                        totalIRFrames += track.IRFrames.Count;
                    }

                    if (track.CameraParam.HasValue)
                    {
                        var p = track.CameraParam.Value;
                        ObExtrinsic? preservedGlobal = null;
                        if (existingGlobalBySerial.TryGetValue(track.Serial, out var preserved))
                            preservedGlobal = preserved;
                        calibrations.Add(new PointCloudRecording.DeviceCalibration
                        {
                            Serial           = track.Serial,
                            ColorIntrinsic   = p.RgbIntrinsic,
                            DepthIntrinsic   = p.DepthIntrinsic,
                            ColorDistortion  = p.RgbDistortion,
                            DepthDistortion  = p.DepthDistortion,
                            DepthToColor     = p.Transform,
                            // Pass through whatever the CalibrationWindow wrote earlier;
                            // null means identity (single-cam or pre-calibration recordings).
                            GlobalTrColorCamera = preservedGlobal,
                        });
                    }
                }

                string ds = string.IsNullOrWhiteSpace(datasetName) ? Path.GetFileName(root.TrimEnd(Path.DirectorySeparatorChar, Path.AltDirectorySeparatorChar)) : datasetName;
                PointCloudRecording.WriteDatasetMetadata(root, host, ds, serials);
                if (calibrations.Count > 0)
                    PointCloudRecording.WriteExtrinsicsYaml(root, calibrations);

                SetStatus(
                    $"Saved {totalDepthFrames} depth / {totalColorFrames} color / {totalIRFrames} IR frame(s) across {_tracks.Count} device(s) to {root}");
            }
            catch (Exception e)
            {
                SetStatus($"Save failed: {e.Message}", warn: true);
                Debug.LogException(e, this);
            }
        }

        [ContextMenu("Read")]
        public void Load()
        {
            if (CurrentState != State.Idle)
            {
                SetStatus("Read: stop recording/playback first.", warn: true);
                return;
            }
            try
            {
                string root = ResolveRoot();
                if (!Directory.Exists(PointCloudRecording.DatasetRoot(root)))
                {
                    SetStatus($"Read: no dataset under {root}", warn: true);
                    return;
                }

                ClearTracks();
                int totalDepth = 0, totalColor = 0, totalIR = 0;
                foreach (var (serial, deviceDir) in PointCloudRecording.EnumerateDevices(root))
                {
                    string depthPath = Path.Combine(deviceDir, PointCloudRecording.DepthSensorName);
                    string colorPath = Path.Combine(deviceDir, PointCloudRecording.ColorSensorName);
                    string irPath = Path.Combine(deviceDir, PointCloudRecording.IRSensorName);
                    bool hasDepth = File.Exists(depthPath);
                    bool hasColor = File.Exists(colorPath);
                    bool hasIR = File.Exists(irPath);
                    if (!hasDepth && !hasColor && !hasIR) continue;

                    var track = GetOrCreateTrack(serial);
                    if (hasDepth)
                    {
                        track.DepthFrames.AddRange(PointCloudRecording.ReadRcsv(depthPath));
                        totalDepth += track.DepthFrames.Count;
                        var (dw, dh) = PointCloudRecording.ReadRcsvHeaderDimensions(depthPath);
                        if (dw > 0 && dh > 0) { track.DepthWidth = dw; track.DepthHeight = dh; }
                    }
                    if (hasColor)
                    {
                        track.ColorFrames.AddRange(PointCloudRecording.ReadRcsv(colorPath));
                        totalColor += track.ColorFrames.Count;
                        var (cw, ch) = PointCloudRecording.ReadRcsvHeaderDimensions(colorPath);
                        if (cw > 0 && ch > 0) { track.ColorWidth = cw; track.ColorHeight = ch; }
                    }
                    if (hasIR)
                    {
                        track.IRFrames.AddRange(PointCloudRecording.ReadRcsv(irPath));
                        totalIR += track.IRFrames.Count;
                        var (iw, ih) = PointCloudRecording.ReadRcsvHeaderDimensions(irPath);
                        if (iw > 0 && ih > 0) { track.IRWidth = iw; track.IRHeight = ih; }
                    }

                    // Fall back to live renderer dimensions if the RCSV header was
                    // missing values (older recordings). Harmless when the header
                    // already populated everything.
                    FillDimensionsFromRenderer(track);
                }

                if (totalDepth == 0 && totalColor == 0)
                {
                    SetStatus($"Read: no raw sensor frames found under {root}", warn: true);
                    return;
                }

                // Pull intrinsics + extrinsics from calibration/extrinsics.yaml so playback
                // can reconstruct point clouds without depending on a live renderer
                // (per Plans/issue-9-multicam-extrinsic-calibration.md → Phase 4).
                int extrinsicsApplied = LoadExtrinsicsFromYaml(root);

                SetStatus(
                    $"Loaded {totalDepth} depth / {totalColor} color / {totalIR} IR frame(s) across {_tracks.Count} device(s)"
                    + (extrinsicsApplied > 0 ? $", extrinsics applied to {extrinsicsApplied} device(s)" : "")
                    + $" from {root}");

                try { OnTracksLoaded?.Invoke(); }
                catch (Exception subscriberEx) { Debug.LogException(subscriberEx, this); }
            }
            catch (Exception e)
            {
                SetStatus($"Read failed: {e.Message}", warn: true);
                Debug.LogException(e, this);
            }
        }

        /// <summary>
        /// Read <c>calibration/extrinsics.yaml</c> if present and populate each
        /// matching track's <see cref="DeviceTrack.CameraParam"/> and
        /// <see cref="DeviceTrack.GlobalTrColorCamera"/>. Returns the number of
        /// devices that received a non-identity <c>global_tr_colorCamera</c>.
        /// </summary>
        private int LoadExtrinsicsFromYaml(string root)
        {
            string path = Path.Combine(PointCloudRecording.CalibrationDir(root), "extrinsics.yaml");
            if (!File.Exists(path)) return 0;

            IReadOnlyList<PointCloudRecording.DeviceCalibration> calibrations;
            try
            {
                calibrations = PointCloudRecording.ReadExtrinsicsYaml(root);
            }
            catch (Exception e)
            {
                Debug.LogWarning($"[{nameof(PointCloudRecorder)}] extrinsics.yaml present but failed to parse: {e.Message}", this);
                return 0;
            }

            int withGlobal = 0;
            foreach (var c in calibrations)
            {
                if (!_tracks.TryGetValue(c.Serial, out var track)) continue;
                // Calibration metadata: keep RCSV-derived dimensions as the source of truth
                // (per plan: yaml intrinsic.width/height is calibration-time and may drift
                // from the actual recorded stream shape if the mode changes).
                track.CameraParam = new ObCameraParam
                {
                    DepthIntrinsic = c.DepthIntrinsic,
                    RgbIntrinsic = c.ColorIntrinsic,
                    DepthDistortion = c.DepthDistortion,
                    RgbDistortion = c.ColorDistortion,
                    Transform = c.DepthToColor,
                    IsMirrored = false,
                };
                track.GlobalTrColorCamera = c.GlobalTrColorCamera;
                if (c.GlobalTrColorCamera.HasValue && !IsIdentityExtrinsic(c.GlobalTrColorCamera.Value))
                    withGlobal++;
            }
            return withGlobal;
        }

        private static bool IsIdentityExtrinsic(ObExtrinsic e)
        {
            const float eps = 1e-5f;
            if (Math.Abs(e.Trans[0]) > eps || Math.Abs(e.Trans[1]) > eps || Math.Abs(e.Trans[2]) > eps) return false;
            for (int r = 0; r < 3; r++)
                for (int col = 0; col < 3; col++)
                {
                    float expected = (r == col) ? 1f : 0f;
                    if (Math.Abs(e.Rot[r * 3 + col] - expected) > eps) return false;
                }
            return true;
        }

        // --- Recording ---

        private void StartRecording()
        {
            if (CurrentState == State.Playing) StopPlayback();
            ClearTracks();

            var renderers = CollectSourceRenderers();
            if (renderers.Count == 0)
            {
                SetStatus("Rec: no PointCloudRenderer found in scene.", warn: true);
                return;
            }

            foreach (var r in renderers)
            {
                if (r == null) continue;
                Action<PointCloudRenderer, RawFrameData> h = HandleRawFrame;
                r.OnRawFramesReady += h;
                _subscribed.Add(r);
                _subscribedHandlers[r] = h;

                // Snapshot calibration at start — CameraParam is populated once pipeline starts.
                var track = GetOrCreateTrack(string.IsNullOrEmpty(r.deviceSerial) ? r.name : r.deviceSerial);
                track.CameraParam = r.CameraParam;
            }
            CurrentState = State.Recording;
            SetStatus($"Recording ({_subscribed.Count} device(s))…");
        }

        private void StopRecording()
        {
            UnsubscribeAll();
            CurrentState = State.Idle;
            SetStatus($"Recorded {RecordedFrameCount} frame(s) across {_tracks.Count} device(s).");
        }

        private void HandleRawFrame(PointCloudRenderer src, RawFrameData raw)
        {
            if (CurrentState != State.Recording) return;
            if (src == null) return;

            string serial = string.IsNullOrEmpty(src.deviceSerial) ? src.name : src.deviceSerial;
            var track = GetOrCreateTrack(serial);
            if (!track.CameraParam.HasValue) track.CameraParam = src.CameraParam;

            ulong timestampNs = raw.TimestampUs * 1000UL;

            if (raw.DepthByteCount > 0)
            {
                var depthBuf = new byte[raw.DepthByteCount];
                Buffer.BlockCopy(raw.DepthBytes, 0, depthBuf, 0, raw.DepthByteCount);
                track.DepthFrames.Add(new PointCloudRecording.Frame { TimestampNs = timestampNs, Bytes = depthBuf });
                track.DepthWidth = raw.DepthWidth;
                track.DepthHeight = raw.DepthHeight;
            }
            if (raw.ColorByteCount > 0)
            {
                var colorBuf = new byte[raw.ColorByteCount];
                Buffer.BlockCopy(raw.ColorBytes, 0, colorBuf, 0, raw.ColorByteCount);
                track.ColorFrames.Add(new PointCloudRecording.Frame { TimestampNs = timestampNs, Bytes = colorBuf });
                track.ColorWidth = raw.ColorWidth;
                track.ColorHeight = raw.ColorHeight;
            }
            if (raw.IRByteCount > 0 && raw.IRBytes != null)
            {
                var irBuf = new byte[raw.IRByteCount];
                Buffer.BlockCopy(raw.IRBytes, 0, irBuf, 0, raw.IRByteCount);
                track.IRFrames.Add(new PointCloudRecording.Frame { TimestampNs = timestampNs, Bytes = irBuf });
                track.IRWidth = raw.IRWidth;
                track.IRHeight = raw.IRHeight;
            }
        }

        // --- Playback ---

        private void StartPlayback()
        {
            if (CurrentState == State.Recording) StopRecording();
            if (_tracks.Count == 0)
            {
                SetStatus("Play: nothing to play. Record or Read first.", warn: true);
                return;
            }

            ulong firstTs = ulong.MaxValue;
            bool anyFrames = false;
            foreach (var kv in _tracks)
            {
                var track = kv.Value;
                if (track.DepthFrames.Count == 0) continue;
                anyFrames = true;
                if (track.DepthFrames[0].TimestampNs < firstTs) firstTs = track.DepthFrames[0].TimestampNs;
                track.PlaybackCursor = -1;
                EnsurePlaybackObject(track);
            }
            if (!anyFrames)
            {
                SetStatus("Play: nothing to play (no depth frames).", warn: true);
                return;
            }
            _playbackTrackStartNs = firstTs == ulong.MaxValue ? 0 : firstTs;
            _playbackWallStart = Time.timeAsDouble;
            CurrentState = State.Playing;
            SetStatus($"Playing {_tracks.Count} device(s)…");
        }

        private void StopPlayback()
        {
            CurrentState = State.Idle;
            SetStatus("Playback stopped.");
        }

        private void Update()
        {
            if (CurrentState != State.Playing) return;
            double elapsedSec = (Time.timeAsDouble - _playbackWallStart) * Mathf.Max(0.01f, playbackRate);
            ulong elapsedNs = (ulong)Math.Max(0.0, elapsedSec * 1_000_000_000.0);
            ulong playheadNs = _playbackTrackStartNs + elapsedNs;

            bool anyRemaining = false;
            foreach (var kv in _tracks)
            {
                var track = kv.Value;
                if (track.DepthFrames.Count == 0) continue;

                int cursor = Mathf.Max(track.PlaybackCursor, 0);
                while (cursor + 1 < track.DepthFrames.Count && track.DepthFrames[cursor + 1].TimestampNs <= playheadNs)
                    cursor++;

                if (cursor < track.DepthFrames.Count - 1) anyRemaining = true;
                if (cursor != track.PlaybackCursor)
                {
                    track.PlaybackCursor = cursor;
                    var depthFrame = track.DepthFrames[cursor];
                    PointCloudRecording.Frame colorFrame = null;
                    if (track.ColorFrames.Count > 0)
                    {
                        // Match the color frame with the closest timestamp (frames should be 1:1 with depth
                        // when frame-sync is enabled, but tolerate slight drift).
                        int colorIdx = Mathf.Min(cursor, track.ColorFrames.Count - 1);
                        colorFrame = track.ColorFrames[colorIdx];
                    }
                    PointCloudRecording.Frame irFrame = null;
                    if (track.IRFrames.Count > 0)
                    {
                        int irIdx = Mathf.Min(cursor, track.IRFrames.Count - 1);
                        irFrame = track.IRFrames[irIdx];
                    }
                    ReconstructAndUpload(track, depthFrame, colorFrame);
                    FirePlaybackEvent(track, depthFrame, colorFrame, irFrame);
                }
            }

            if (!anyRemaining)
            {
                if (loop)
                {
                    _playbackWallStart = Time.timeAsDouble;
                    foreach (var kv in _tracks) kv.Value.PlaybackCursor = -1;
                }
                else
                {
                    StopPlayback();
                }
            }
        }

        private void EnsurePlaybackObject(DeviceTrack track)
        {
            if (track.PlaybackObject != null) return;

            var go = new GameObject($"_Playback_{track.Serial}");
            go.transform.SetParent(transform, worldPositionStays: false);
            // Mirror the live renderer's image-Y-down -> Unity-Y-up flip.
            go.transform.localScale = new Vector3(1f, -1f, 1f);
            // Apply the per-track world ← color camera transform if extrinsics were
            // loaded from extrinsics.yaml (issue #9 / Phase 5). Sets localPosition
            // and localRotation; localScale is the Y-flip above.
            if (track.GlobalTrColorCamera.HasValue)
                Calibration.ExtrinsicsApply.ApplyToTransform(go.transform, track.GlobalTrColorCamera.Value);
            var mf = go.AddComponent<MeshFilter>();
            var mr = go.AddComponent<MeshRenderer>();
            mr.shadowCastingMode = ShadowCastingMode.Off;
            mr.receiveShadows = false;
            mr.lightProbeUsage = LightProbeUsage.Off;
            mr.reflectionProbeUsage = ReflectionProbeUsage.Off;

            Material mat = playbackMaterial;
            if (mat == null)
            {
                foreach (var r in _subscribed)
                {
                    if (r != null && r.deviceSerial == track.Serial && r.pointMaterial != null)
                    { mat = r.pointMaterial; break; }
                }
            }
            if (mat == null)
            {
                var shader = Shader.Find("Orbbec/PointCloudUnlit");
                if (shader != null) mat = new Material(shader) { name = "PointCloudUnlit (playback)" };
            }
            if (mat != null) mr.sharedMaterial = mat;

            track.PlaybackObject = go;
            track.PlaybackFilter = mf;
            track.PlaybackRenderer = mr;
            track.PlaybackMesh = null;
            track.PlaybackMeshCapacity = 0;
        }

        // Wraps the per-frame playback data in a RawFrameData and fires
        // OnPlaybackRawFrame so downstream subscribers (e.g. BodyTrackingMultiLive)
        // can run the same merge pipeline they use on live OnRawFramesReady.
        private void FirePlaybackEvent(
            DeviceTrack track,
            PointCloudRecording.Frame depthFrame,
            PointCloudRecording.Frame colorFrame,
            PointCloudRecording.Frame irFrame)
        {
            if (OnPlaybackRawFrame == null) return;
            if (depthFrame?.Bytes == null) return;
            ulong tsUs = depthFrame.TimestampNs / 1000UL;
            byte[] colorBytes = colorFrame?.Bytes;
            int colorCount = colorBytes?.Length ?? 0;
            byte[] irBytes = irFrame?.Bytes;
            int irCount = irBytes?.Length ?? 0;
            var raw = new RawFrameData(
                depthBytes: depthFrame.Bytes, depthByteCount: depthFrame.Bytes.Length,
                depthWidth: track.DepthWidth, depthHeight: track.DepthHeight,
                colorBytes: colorBytes, colorByteCount: colorCount,
                colorWidth: track.ColorWidth, colorHeight: track.ColorHeight,
                irBytes: irBytes, irByteCount: irCount,
                irWidth: track.IRWidth, irHeight: track.IRHeight,
                timestampUs: tsUs);
            Transform t = track.PlaybackObject != null ? track.PlaybackObject.transform : null;
            OnPlaybackRawFrame.Invoke(track.Serial, track.CameraParam, t, raw);
        }

        private void ReconstructAndUpload(
            DeviceTrack track,
            PointCloudRecording.Frame depthFrame,
            PointCloudRecording.Frame colorFrame)
        {
            if (!track.CameraParam.HasValue)
            {
                // No intrinsics — nothing to reconstruct. Blank the mesh.
                if (track.PlaybackMesh != null)
                {
                    track.PlaybackMesh.SetSubMesh(0, new SubMeshDescriptor(0, 0, MeshTopology.Points),
                        MeshUpdateFlags.DontRecalculateBounds | MeshUpdateFlags.DontValidateIndices);
                }
                return;
            }
            int dw = track.DepthWidth, dh = track.DepthHeight;
            if (dw <= 0 || dh <= 0) return;

            int capacity = dw * dh;
            if (!track.ReconstructBuffer.IsCreated || track.ReconstructBuffer.Length < capacity)
            {
                if (track.ReconstructBuffer.IsCreated) track.ReconstructBuffer.Dispose();
                track.ReconstructBuffer = new NativeArray<ObColorPoint>(
                    capacity, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
            }

            int pointCount = PointReconstruction.Reconstruct(
                depthFrame.Bytes, dw, dh,
                colorFrame != null ? colorFrame.Bytes : null,
                track.ColorWidth, track.ColorHeight,
                track.CameraParam.Value,
                track.ReconstructBuffer);

            EnsurePlaybackObject(track);
            if (track.PlaybackMesh == null || track.PlaybackMeshCapacity < capacity)
            {
                if (track.PlaybackMesh != null) Destroy(track.PlaybackMesh);
                var mesh = new Mesh
                {
                    name = $"PlaybackMesh_{track.Serial}",
                    indexFormat = IndexFormat.UInt32,
                    bounds = new Bounds(Vector3.zero, Vector3.one * 100f),
                };
                var attrs = new[]
                {
                    new VertexAttributeDescriptor(VertexAttribute.Position, VertexAttributeFormat.Float32, 3, stream: 0),
                    new VertexAttributeDescriptor(VertexAttribute.Color,    VertexAttributeFormat.Float32, 3, stream: 0),
                };
                mesh.SetVertexBufferParams(capacity, attrs);
                mesh.SetIndexBufferParams(capacity, IndexFormat.UInt32);
                var indices = new NativeArray<uint>(capacity, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
                try
                {
                    for (int i = 0; i < capacity; i++) indices[i] = (uint)i;
                    mesh.SetIndexBufferData(indices, 0, 0, capacity,
                        MeshUpdateFlags.DontRecalculateBounds | MeshUpdateFlags.DontValidateIndices);
                }
                finally { indices.Dispose(); }
                track.PlaybackMesh = mesh;
                track.PlaybackMeshCapacity = capacity;
                track.PlaybackFilter.sharedMesh = mesh;
            }

            track.PlaybackMesh.SetVertexBufferData(track.ReconstructBuffer, 0, 0, pointCount,
                flags: MeshUpdateFlags.DontRecalculateBounds | MeshUpdateFlags.DontValidateIndices);
            track.PlaybackMesh.SetSubMesh(0, new SubMeshDescriptor(0, pointCount, MeshTopology.Points),
                MeshUpdateFlags.DontRecalculateBounds | MeshUpdateFlags.DontValidateIndices);
        }

        // --- Lifecycle / helpers ---

        private void OnDisable()
        {
            UnsubscribeAll();
            if (CurrentState != State.Idle) CurrentState = State.Idle;
        }

        private void OnDestroy()
        {
            UnsubscribeAll();
            ClearTracks();
        }

        private void UnsubscribeAll()
        {
            for (int i = 0; i < _subscribed.Count; i++)
            {
                var r = _subscribed[i];
                if (r != null && _subscribedHandlers.TryGetValue(r, out var h))
                    r.OnRawFramesReady -= h;
            }
            _subscribed.Clear();
            _subscribedHandlers.Clear();
        }

        private void ClearTracks()
        {
            foreach (var kv in _tracks)
            {
                var track = kv.Value;
                if (track.ReconstructBuffer.IsCreated) track.ReconstructBuffer.Dispose();
                if (track.PlaybackMesh != null)
                {
                    if (Application.isPlaying) Destroy(track.PlaybackMesh);
                    else DestroyImmediate(track.PlaybackMesh);
                }
                if (track.PlaybackObject != null)
                {
                    if (Application.isPlaying) Destroy(track.PlaybackObject);
                    else DestroyImmediate(track.PlaybackObject);
                }
            }
            _tracks.Clear();
        }

        private DeviceTrack GetOrCreateTrack(string serial)
        {
            if (!_tracks.TryGetValue(serial, out var track))
            {
                track = new DeviceTrack { Serial = serial };
                _tracks[serial] = track;
            }
            return track;
        }

        private void FillDimensionsFromRenderer(DeviceTrack track)
        {
            if (track.DepthWidth > 0 && track.ColorWidth > 0) return;
            foreach (var r in CollectSourceRenderers())
            {
                if (r == null) continue;
                string serial = string.IsNullOrEmpty(r.deviceSerial) ? r.name : r.deviceSerial;
                if (serial != track.Serial) continue;
                if (track.DepthWidth == 0)  { track.DepthWidth  = (int)r.depthWidth;  track.DepthHeight  = (int)r.depthHeight; }
                if (track.ColorWidth == 0)  { track.ColorWidth  = (int)r.colorWidth;  track.ColorHeight  = (int)r.colorHeight; }
                if (track.IRWidth == 0)     { track.IRWidth     = (int)r.depthWidth;  track.IRHeight     = (int)r.depthHeight; }
                if (!track.CameraParam.HasValue && r.CameraParam.HasValue) track.CameraParam = r.CameraParam;
                break;
            }
        }

        private List<PointCloudRenderer> CollectSourceRenderers()
        {
            var list = new List<PointCloudRenderer>();
            if (cameraManager != null && cameraManager.Renderers != null)
            {
                foreach (var r in cameraManager.Renderers)
                    if (r != null) list.Add(r);
            }
            if (list.Count == 0)
            {
                var found = UnityEngine.Object.FindObjectsByType<PointCloudRenderer>(FindObjectsSortMode.None);
                list.AddRange(found);
            }
            return list;
        }

        private string ResolveRoot()
        {
            string p = folderPath;
            if (string.IsNullOrWhiteSpace(p))
                p = Path.Combine(Application.persistentDataPath, "Recordings", "recording");
            else if (!Path.IsPathRooted(p))
                p = Path.Combine(Application.persistentDataPath, p);
            return p;
        }

        private static string SafeMachineName()
        {
            try { return Environment.MachineName; }
            catch { return "unknown"; }
        }

        private void SetStatus(string msg, bool warn = false)
        {
            StatusMessage = msg;
            if (warn) Debug.LogWarning($"[{nameof(PointCloudRecorder)}] {msg}", this);
            else Debug.Log($"[{nameof(PointCloudRecorder)}] {msg}", this);
        }
    }

    /// <summary>
    /// CPU-side Phase D reconstruction: raw depth Y16 + raw color RGB + intrinsics/distortion
    /// + depth-to-color extrinsic (all in millimeters for translation) -> ObColorPoint array in
    /// color-camera space (meters), packed densely (invalid-depth pixels skipped).
    /// </summary>
    internal static class PointReconstruction
    {
        public static unsafe int Reconstruct(
            byte[] depthY16, int depthW, int depthH,
            byte[] colorRgb8, int colorW, int colorH,
            ObCameraParam cam,
            NativeArray<ObColorPoint> output)
        {
            if (depthY16 == null || depthW <= 0 || depthH <= 0) return 0;
            int expected = depthW * depthH * 2;
            if (depthY16.Length < expected) return 0;

            float fxD = cam.DepthIntrinsic.Fx, fyD = cam.DepthIntrinsic.Fy;
            float cxD = cam.DepthIntrinsic.Cx, cyD = cam.DepthIntrinsic.Cy;
            float fxC = cam.RgbIntrinsic.Fx,   fyC = cam.RgbIntrinsic.Fy;
            float cxC = cam.RgbIntrinsic.Cx,   cyC = cam.RgbIntrinsic.Cy;

            // SDK transform: rotation (depth->color) + translation in millimeters.
            float r0 = cam.Transform.Rot[0], r1 = cam.Transform.Rot[1], r2 = cam.Transform.Rot[2];
            float r3 = cam.Transform.Rot[3], r4 = cam.Transform.Rot[4], r5 = cam.Transform.Rot[5];
            float r6 = cam.Transform.Rot[6], r7 = cam.Transform.Rot[7], r8 = cam.Transform.Rot[8];
            float tx = cam.Transform.Trans[0], ty = cam.Transform.Trans[1], tz = cam.Transform.Trans[2];

            bool hasColor = colorRgb8 != null && colorW > 0 && colorH > 0
                            && colorRgb8.Length >= colorW * colorH * 3;
            int outCount = 0;
            int max = output.Length;

            // Use a stub non-null empty array in the else-branch so `fixed` doesn't receive
            // a null candidate (some C# versions reject the mixed ternary).
            byte[] colorSrc = hasColor ? colorRgb8 : Array.Empty<byte>();

            fixed (byte* depthP = depthY16)
            fixed (byte* colorP = colorSrc)
            {
                ushort* depthPtr = (ushort*)depthP;
                for (int v = 0; v < depthH; v++)
                {
                    for (int u = 0; u < depthW; u++)
                    {
                        if (outCount >= max) return outCount;
                        ushort zRaw = depthPtr[v * depthW + u];
                        if (zRaw == 0) continue;

                        float z = zRaw; // mm
                        float xD = (u - cxD) * z / fxD;
                        float yD = (v - cyD) * z / fyD;

                        // Transform to color camera space.
                        float xC = r0 * xD + r1 * yD + r2 * z + tx;
                        float yC = r3 * xD + r4 * yD + r5 * z + ty;
                        float zC = r6 * xD + r7 * yD + r8 * z + tz;

                        // Default color (grey) when sampling fails.
                        float cr = 0.5f, cg = 0.5f, cb = 0.5f;
                        if (hasColor && zC > 0f)
                        {
                            float uC = fxC * xC / zC + cxC;
                            float vC = fyC * yC / zC + cyC;
                            int iu = (int)uC;
                            int iv = (int)vC;
                            if (iu >= 0 && iu < colorW && iv >= 0 && iv < colorH)
                            {
                                byte* px = colorP + (iv * colorW + iu) * 3;
                                cr = px[0] * (1.0f / 255f);
                                cg = px[1] * (1.0f / 255f);
                                cb = px[2] * (1.0f / 255f);
                            }
                        }

                        output[outCount] = new ObColorPoint
                        {
                            X = xC * 0.001f, // mm -> m, in color camera space
                            Y = yC * 0.001f,
                            Z = zC * 0.001f,
                            R = cr, G = cg, B = cb,
                        };
                        outCount++;
                    }
                }
            }
            return outCount;
        }
    }
}
