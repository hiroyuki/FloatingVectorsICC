// Rec / Play / Save / Read component. Records point clouds emitted by one or more
// PointCloudRenderer(s), stores them in memory, saves/loads them as ScannedReality-
// Studio-compatible RCSV files (one per device under <root>/dataset/<host>/.../pointcloud_main),
// and plays them back onto child meshes.
//
// The four button actions are exposed as public methods so they can be triggered from
// PointCloudRecorderEditor (Inspector) or from runtime UI.

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

        [Header("Playback")]
        public bool loop = true;

        [Min(0.01f)]
        [Tooltip("Playback rate multiplier (1.0 = real time).")]
        public float playbackRate = 1.0f;

        // --- Runtime state ---
        public State CurrentState { get; private set; } = State.Idle;
        public int RecordedFrameCount
        {
            get
            {
                int n = 0;
                foreach (var kv in _tracks) n += kv.Value.Frames.Count;
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
                    if (track.Frames.Count < 2) continue;
                    ulong span = track.Frames[track.Frames.Count - 1].TimestampNs - track.Frames[0].TimestampNs;
                    if (span > maxSpan) maxSpan = span;
                }
                return maxSpan / 1_000_000_000f;
            }
        }

        public int DeviceCount => _tracks.Count;
        public string StatusMessage { get; private set; } = "";

        // --- Internals ---

        private sealed class DeviceTrack
        {
            public string Serial;
            public readonly List<PointCloudRecording.Frame> Frames = new List<PointCloudRecording.Frame>();
            public GameObject PlaybackObject;
            public MeshFilter PlaybackFilter;
            public MeshRenderer PlaybackRenderer;
            public Mesh PlaybackMesh;
            public int PlaybackMeshCapacity;
            public int PlaybackCursor;
        }

        private readonly Dictionary<string, DeviceTrack> _tracks = new Dictionary<string, DeviceTrack>();
        private readonly List<PointCloudRenderer> _subscribed = new List<PointCloudRenderer>();
        private readonly Dictionary<PointCloudRenderer, Action<PointCloudRenderer, NativeArray<ObColorPoint>, int, ulong>>
            _subscribedHandlers = new Dictionary<PointCloudRenderer, Action<PointCloudRenderer, NativeArray<ObColorPoint>, int, ulong>>();
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
                int totalFrames = 0;
                foreach (var kv in _tracks)
                {
                    var track = kv.Value;
                    if (track.Frames.Count == 0) continue;
                    string filePath = PointCloudRecording.DeviceFilePath(root, host, track.Serial);
                    PointCloudRecording.WriteDeviceFile(filePath, track.Serial, track.Frames);
                    totalFrames += track.Frames.Count;
                }
                SetStatus($"Saved {totalFrames} frame(s) across {_tracks.Count} device(s) to {root}");
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
                if (!Directory.Exists(Path.Combine(root, "dataset")))
                {
                    SetStatus($"Read: no dataset under {root}", warn: true);
                    return;
                }

                ClearTracks();
                int totalFrames = 0;
                foreach (var (serial, filePath) in PointCloudRecording.EnumerateDevices(root))
                {
                    var frames = PointCloudRecording.ReadDeviceFile(filePath);
                    if (frames.Count == 0) continue;
                    var track = GetOrCreateTrack(serial);
                    track.Frames.AddRange(frames);
                    totalFrames += frames.Count;
                }
                if (totalFrames == 0)
                {
                    SetStatus($"Read: no frames found under {root}", warn: true);
                    return;
                }
                SetStatus($"Loaded {totalFrames} frame(s) across {_tracks.Count} device(s) from {root}");
            }
            catch (Exception e)
            {
                SetStatus($"Read failed: {e.Message}", warn: true);
                Debug.LogException(e, this);
            }
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
                Action<PointCloudRenderer, NativeArray<ObColorPoint>, int, ulong> h = HandleFrame;
                r.OnFrameUploaded += h;
                _subscribed.Add(r);
                _subscribedHandlers[r] = h;
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

        private void HandleFrame(PointCloudRenderer src, NativeArray<ObColorPoint> buffer, int count, ulong timestampUs)
        {
            if (CurrentState != State.Recording) return;
            if (count <= 0 || src == null) return;

            string serial = string.IsNullOrEmpty(src.deviceSerial) ? src.name : src.deviceSerial;
            var track = GetOrCreateTrack(serial);

            int bytes = count * PointCloudRecording.VertexStride;
            var buf = new byte[bytes];
            unsafe
            {
                fixed (byte* dst = buf)
                {
                    UnsafeUtility.MemCpy(dst,
                        NativeArrayUnsafeUtility.GetUnsafeReadOnlyPtr(buffer),
                        bytes);
                }
            }
            track.Frames.Add(new PointCloudRecording.Frame
            {
                TimestampNs = UtcNowUnixNs(),
                Bytes = buf,
            });
        }

        // --- Playback ---

        private void StartPlayback()
        {
            if (CurrentState == State.Recording) StopRecording();
            if (_tracks.Count == 0 || RecordedFrameCount == 0)
            {
                SetStatus("Play: nothing to play. Record or Read first.", warn: true);
                return;
            }

            ulong firstTs = ulong.MaxValue;
            foreach (var kv in _tracks)
            {
                var track = kv.Value;
                if (track.Frames.Count == 0) continue;
                if (track.Frames[0].TimestampNs < firstTs) firstTs = track.Frames[0].TimestampNs;
                track.PlaybackCursor = 0;
                EnsurePlaybackObject(track);
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
                if (track.Frames.Count == 0) continue;

                int cursor = track.PlaybackCursor;
                while (cursor + 1 < track.Frames.Count && track.Frames[cursor + 1].TimestampNs <= playheadNs)
                    cursor++;

                if (cursor < track.Frames.Count - 1) anyRemaining = true;
                if (cursor != track.PlaybackCursor || track.PlaybackMesh == null)
                {
                    track.PlaybackCursor = cursor;
                    UploadFrameToMesh(track, track.Frames[cursor]);
                }
            }

            if (!anyRemaining)
            {
                if (loop)
                {
                    _playbackWallStart = Time.timeAsDouble;
                    foreach (var kv in _tracks) kv.Value.PlaybackCursor = 0;
                }
                else
                {
                    StopPlayback();
                }
            }
        }

        private void EnsurePlaybackObject(DeviceTrack track)
        {
            // Only gate on the GameObject existing: the mesh is lazily (re)created in
            // UploadFrameToMesh based on point count, so a null mesh with an existing
            // GO is a valid intermediate state.
            if (track.PlaybackObject != null) return;

            var go = new GameObject($"_Playback_{track.Serial}");
            go.transform.SetParent(transform, worldPositionStays: false);
            // Source PointCloudRenderers flip localScale.y to map image Y-down to Unity Y-up;
            // mirror that here so playback geometry lines up with live points.
            go.transform.localScale = new Vector3(1f, -1f, 1f);
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

        private void UploadFrameToMesh(DeviceTrack track, PointCloudRecording.Frame frame)
        {
            if (frame == null || frame.Bytes == null) return;
            int count = frame.PointCount;
            if (count <= 0) return;
            EnsurePlaybackObject(track);

            if (track.PlaybackMesh == null || track.PlaybackMeshCapacity < count)
            {
                if (track.PlaybackMesh != null) Destroy(track.PlaybackMesh);
                int capacity = Mathf.Max(count, 1);
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

            var tmp = new NativeArray<ObColorPoint>(count, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
            try
            {
                unsafe
                {
                    fixed (byte* src = frame.Bytes)
                    {
                        UnsafeUtility.MemCpy(
                            NativeArrayUnsafeUtility.GetUnsafePtr(tmp),
                            src,
                            (long)count * PointCloudRecording.VertexStride);
                    }
                }
                track.PlaybackMesh.SetVertexBufferData(tmp, 0, 0, count,
                    flags: MeshUpdateFlags.DontRecalculateBounds | MeshUpdateFlags.DontValidateIndices);
            }
            finally { tmp.Dispose(); }

            track.PlaybackMesh.SetSubMesh(0, new SubMeshDescriptor(0, count, MeshTopology.Points),
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
                    r.OnFrameUploaded -= h;
            }
            _subscribed.Clear();
            _subscribedHandlers.Clear();
        }

        private void ClearTracks()
        {
            foreach (var kv in _tracks)
            {
                var track = kv.Value;
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

        private static ulong UtcNowUnixNs()
        {
            // DateTime ticks are 100ns units; subtract Unix epoch then multiply by 100.
            long unixTicks = DateTime.UtcNow.Ticks - new DateTime(1970, 1, 1, 0, 0, 0, DateTimeKind.Utc).Ticks;
            return (ulong)unixTicks * 100UL;
        }

        private void SetStatus(string msg, bool warn = false)
        {
            StatusMessage = msg;
            if (warn) Debug.LogWarning($"[{nameof(PointCloudRecorder)}] {msg}", this);
            else Debug.Log($"[{nameof(PointCloudRecorder)}] {msg}", this);
        }
    }
}
