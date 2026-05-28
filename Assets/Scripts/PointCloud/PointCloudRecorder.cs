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
using System.Threading.Tasks;
using Orbbec;
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

        [Tooltip("Optional bounding box used to cull playback points the same way live PointCloudRenderers do. " +
                 "If null, the first PointCloudBoundingBox referenced by any live PointCloudRenderer in the " +
                 "scene is used. Set filterMode=Disabled to disable culling without unassigning the reference.")]
        public PointCloudBoundingBox boundingBox;

        [Tooltip("Optional random decimater applied to playback points (mirrors the live PointCloudRenderer.decimater path). " +
                 "If null, the first PointCloudDecimater in the scene is used.")]
        public PointCloudDecimater decimater;

        [Tooltip("Optional capsule filter applied to playback points (mirrors the live PointCloudRenderer.capsuleFilter path). " +
                 "If null, the first PointCloudCapsuleFilter referenced by any live PointCloudRenderer is used, then " +
                 "the first one found in the scene. Set Mode=Disabled to disable culling without unassigning.")]
        public PointCloudCapsuleFilter capsuleFilter;

        [Tooltip("Optional joint motion field applied to playback points (mirrors the live PointCloudRenderer.jointMotionField path). " +
                 "If null, the first PointCloudJointMotionField referenced by any live PointCloudRenderer is used, then " +
                 "the first one found in the scene. Set Mode=Disabled to leave playback colors untouched. Issue #24.")]
        public PointCloudJointMotionField jointMotionField;

        [Tooltip("Optional cumulative snapshotter for playback. When assigned and its No Erase toggle is on, the playback " +
                 "mesh's vertex buffer is read back from GPU and snapshotted every 'interval' advanced playback frames. " +
                 "If null, the first PointCloudCumulative in the scene is used. Snapshots include all reconstructed " +
                 "vertices (invalid depth pixels are written far off-screen by the reconstruct shader and clip-culled " +
                 "at draw time, same as the live playback mesh).")]
        public PointCloudCumulative cumulative;

        [Header("Files")]
        [Tooltip("Root folder for recordings. Relative paths resolve under Application.persistentDataPath. " +
                 "Leave empty to use '<persistentDataPath>/Recordings/recording'.")]
        public string folderPath = "";

        [Tooltip("Dataset name written into dataset.yaml. Defaults to the recording folder name.")]
        public string datasetName = "";

        [Header("Playback")]
        public bool loop = true;

        [Tooltip("Where to register playback meshes for visibility toggling. " +
                 "Auto-found via FindFirstObjectByType if left null.")]
        public PointCloudView view;

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

        /// <summary>True while playback is paused (CurrentState stays at Playing).
        /// Cursor advancement and OnPlaybackRawFrame events are halted; resuming
        /// shifts the wall-clock origin so the playhead picks up where it left off.
        /// </summary>
        public bool IsPaused { get; private set; }
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
                    ulong first = TimestampNsAt(track.DepthFrames, 0);
                    ulong last = TimestampNsAt(track.DepthFrames, track.DepthFrames.Count - 1);
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

        private sealed class DeviceTrack : IDisposable
        {
            public string Serial;

            // Raw sensor data exposed as read-only views so the same fields
            // can hold either:
            //   * Rec mode: List<Frame> filled by HandleRawFrame as Frame
            //               sentinels (Bytes = null; the actual bytes were
            //               streamed straight to disk via RcsvStreamWriter).
            //   * Read mode: RcsvFrameStream that lazy-reads bytes on demand.
            // Consumers (PlaybackCursor advance, BodyTrackingPlayback, gap
            // diagnostics) only need Count, indexer, and TimestampNs, so they
            // are mode-agnostic. Internal _xxxList / _xxxStream pointers exist
            // for the mode-specific mutate (Add) and dispose paths.
            public IReadOnlyList<PointCloudRecording.Frame> DepthFrames { get; private set; }
                = Array.Empty<PointCloudRecording.Frame>();
            public IReadOnlyList<PointCloudRecording.Frame> ColorFrames { get; private set; }
                = Array.Empty<PointCloudRecording.Frame>();
            public IReadOnlyList<PointCloudRecording.Frame> IRFrames { get; private set; }
                = Array.Empty<PointCloudRecording.Frame>();

            private List<PointCloudRecording.Frame> _depthList;
            private List<PointCloudRecording.Frame> _colorList;
            private List<PointCloudRecording.Frame> _irList;
            private PointCloudRecording.RcsvFrameStream _depthStream;
            private PointCloudRecording.RcsvFrameStream _colorStream;
            private PointCloudRecording.RcsvFrameStream _irStream;

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
            public int PlaybackCursor;

            // GPU reconstruction (Mesh + GraphicsBuffers + ComputeShader dispatch).
            // Shared implementation with live capture path; see PointCloudReconstructor.
            public PointCloudReconstructor Reconstructor;
            public Mesh PlaybackMesh => Reconstructor?.Mesh;

            // Switch into Rec mode lazily on the first frame add. Safe to call
            // repeatedly. If the track is currently in Read mode (streams
            // open), they are disposed first — the user kicked off a fresh
            // recording over a loaded track.
            public void AddRecordedDepthFrame(PointCloudRecording.Frame f)
            { EnsureRecordingMode(); _depthList.Add(f); }
            public void AddRecordedColorFrame(PointCloudRecording.Frame f)
            { EnsureRecordingMode(); _colorList.Add(f); }
            public void AddRecordedIRFrame(PointCloudRecording.Frame f)
            { EnsureRecordingMode(); _irList.Add(f); }

            private void EnsureRecordingMode()
            {
                if (_depthStream != null || _colorStream != null || _irStream != null)
                    Dispose();
                if (_depthList == null) _depthList = new List<PointCloudRecording.Frame>();
                if (_colorList == null) _colorList = new List<PointCloudRecording.Frame>();
                if (_irList == null) _irList = new List<PointCloudRecording.Frame>();
                DepthFrames = _depthList;
                ColorFrames = _colorList;
                IRFrames = _irList;
            }

            /// <summary>
            /// Adopt freshly opened RCSV streams as the backing storage for
            /// this track's depth/color/IR frames. Any previously held
            /// streams or recording lists are released. Pass <c>null</c> for
            /// a sensor that has no recording on disk.
            /// </summary>
            public void AdoptStreams(
                PointCloudRecording.RcsvFrameStream depth,
                PointCloudRecording.RcsvFrameStream color,
                PointCloudRecording.RcsvFrameStream ir)
            {
                Dispose();
                _depthStream = depth;
                _colorStream = color;
                _irStream = ir;
                DepthFrames = (IReadOnlyList<PointCloudRecording.Frame>)depth
                              ?? Array.Empty<PointCloudRecording.Frame>();
                ColorFrames = (IReadOnlyList<PointCloudRecording.Frame>)color
                              ?? Array.Empty<PointCloudRecording.Frame>();
                IRFrames = (IReadOnlyList<PointCloudRecording.Frame>)ir
                           ?? Array.Empty<PointCloudRecording.Frame>();
            }

            public void Dispose()
            {
                _depthStream?.Dispose(); _depthStream = null;
                _colorStream?.Dispose(); _colorStream = null;
                _irStream?.Dispose(); _irStream = null;
                _depthList = null;
                _colorList = null;
                _irList = null;
                DepthFrames = Array.Empty<PointCloudRecording.Frame>();
                ColorFrames = Array.Empty<PointCloudRecording.Frame>();
                IRFrames = Array.Empty<PointCloudRecording.Frame>();
            }
        }

        private readonly Dictionary<string, DeviceTrack> _tracks = new Dictionary<string, DeviceTrack>();
        private readonly List<PointCloudRenderer> _subscribed = new List<PointCloudRenderer>();
        private readonly Dictionary<PointCloudRenderer, Action<PointCloudRenderer, RawFrameData>>
            _subscribedHandlers = new Dictionary<PointCloudRenderer, Action<PointCloudRenderer, RawFrameData>>();
        private double _playbackWallStart;
        private ulong _playbackTrackStartNs;
        private double _pauseWallStart;

        [Header("Diagnostics")]
        [Tooltip("Log per-second playback fire counts per serial (FirePlaybackEvent rate). " +
                 "Useful to compare against K4abtWorkerHost enqueued/s when investigating " +
                 "frame-rate drops between recording and worker enqueue.")]
        public bool diagnosticLogging = true;
        private readonly Dictionary<string, int> _diagFiresPerSerial = new Dictionary<string, int>();

        // Per-second recording diagnostics, populated in HandleRawFrame and emitted
        // by PerSecondDiag when diagnosticLogging is on. Lets us spot capture-side
        // drops AT RECORDING TIME — by the time the RCSV is on disk the gap is
        // baked in, so we have to catch it live.
        private sealed class RecordWindowStats
        {
            public int Frames;       // count this second
            public long Bytes;       // depth+color+IR bytes enqueued this second
            public double MaxGapMs;  // largest inter-frame gap this second
            public ulong LastTsNs;   // previous frame's timestamp for gap calc
        }
        private readonly Dictionary<string, RecordWindowStats> _diagRecordPerSerial = new Dictionary<string, RecordWindowStats>();
        private float _diagWindowStart;

        // --- Stream-on-record state ---
        //
        // One StreamWriters per device; each holds up to 3 open RCSV files
        // (depth/color/IR). Opened lazily on the first frame per sensor so we
        // get the actual on-the-wire dimensions in the file header, and closed
        // in StopRecording where Dispose finalizes the trailing index chunk.
        // Resolved at StartRecording time so HandleRawFrame doesn't have to
        // hit folderPath / hostname per frame.
        private string _recordRoot;
        private string _recordHost;
        private sealed class StreamWriters
        {
            public PointCloudRecording.RcsvStreamWriter Depth;
            public PointCloudRecording.RcsvStreamWriter Color;
            public PointCloudRecording.RcsvStreamWriter IR;
        }
        private readonly Dictionary<string, StreamWriters> _streamWriters = new Dictionary<string, StreamWriters>();

        // Recorder-level (process-wide) capture diagnostics. The 4-cam gap
        // analyzer showed all cameras dropping the same timestamps, so the
        // suspect is host-side (GC pause / main-thread stall / disk I/O blip),
        // not per-camera USB. Track those signals alongside the per-serial
        // stats so a noisy second can be correlated with a GC spike or a
        // stalled Update tick.
        private int _diagGc0Start, _diagGc1Start, _diagGc2Start;
        private long _diagMonoMemStart;
        private float _diagMaxUpdateDtMs;      // longest single Update tick in this window
        private float _diagMaxUpdateDtAtSec;   // realtimeSinceStartup of the offending tick

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

        [ContextMenu("Toggle Pause")]
        public void TogglePause()
        {
            if (CurrentState != State.Playing) return;
            if (IsPaused) ResumePlayback();
            else PausePlayback();
        }

        public void PausePlayback()
        {
            if (CurrentState != State.Playing || IsPaused) return;
            IsPaused = true;
            _pauseWallStart = Time.timeAsDouble;
            SetStatus("Playback paused.");
        }

        public void ResumePlayback()
        {
            if (CurrentState != State.Playing || !IsPaused) return;
            // Shift the wall-clock origin so the playhead resumes from where it
            // was paused instead of jumping forward by the paused duration.
            _playbackWallStart += Time.timeAsDouble - _pauseWallStart;
            IsPaused = false;
            SetStatus($"Playing {_tracks.Count} device(s)…");
        }

        /// <summary>
        /// Advance each device's playback cursor by exactly one depth frame and
        /// re-emit the frame so the reconstructed mesh + downstream BT pipeline
        /// updates. No-op outside playback. Auto-pauses if currently playing.
        /// </summary>
        [ContextMenu("Step Forward")]
        public void StepForward()
        {
            if (CurrentState != State.Playing) return;
            if (!IsPaused) PausePlayback();
            StepCursor(+1);
        }

        /// <summary>
        /// Move each device's playback cursor back by one depth frame and
        /// re-emit the frame. No-op outside playback. Auto-pauses if currently
        /// playing.
        /// </summary>
        [ContextMenu("Step Backward")]
        public void StepBackward()
        {
            if (CurrentState != State.Playing) return;
            if (!IsPaused) PausePlayback();
            StepCursor(-1);
        }

        /// <summary>
        /// All-or-nothing cursor shift by <paramref name="delta"/> (±1 for
        /// arrow-key stepping). If EVERY non-empty track can shift its cursor
        /// by delta (within [0, Count-1]), every track moves and re-fires its
        /// playback frame. If any track is at its boundary, no track moves —
        /// this keeps multi-device inspection at a coherent frame instead of
        /// leaving the scene at mixed timestamps. The wall-clock origin is
        /// then shifted so a subsequent Resume continues from the stepped
        /// playhead. Playhead is clamped to be strictly less than every
        /// track's next-frame timestamp so the Update() auto-advance does not
        /// fire extra frames on resume.
        /// </summary>
        private void StepCursor(int delta)
        {
            if (_tracks.Count == 0) return;

            // All-or-nothing precheck: every non-empty track must be able to
            // move its cursor by delta without leaving [0, Count-1].
            bool anyNonEmpty = false;
            foreach (var kv in _tracks)
            {
                var track = kv.Value;
                if (track.DepthFrames.Count == 0) continue;
                anyNonEmpty = true;
                int newCursor = track.PlaybackCursor + delta;
                if (newCursor < 0 || newCursor >= track.DepthFrames.Count)
                {
                    SetStatus(delta > 0
                        ? $"Step forward: track {track.Serial} at last frame ({track.PlaybackCursor + 1}/{track.DepthFrames.Count})."
                        : $"Step backward: track {track.Serial} at first frame.");
                    return;
                }
            }
            if (!anyNonEmpty) return;

            ulong maxSteppedTs = 0;
            ulong minNextTs = ulong.MaxValue;
            foreach (var kv in _tracks)
            {
                var track = kv.Value;
                if (track.DepthFrames.Count == 0) continue;
                int newCursor = track.PlaybackCursor + delta;
                SetCursorAndEmit(track, newCursor);
                ulong ts = TimestampNsAt(track.DepthFrames, newCursor);
                if (ts > maxSteppedTs) maxSteppedTs = ts;
                int next = newCursor + 1;
                if (next < track.DepthFrames.Count)
                {
                    ulong nextTs = TimestampNsAt(track.DepthFrames, next);
                    if (nextTs < minNextTs) minNextTs = nextTs;
                }
            }

            // Valid playhead range for keeping every cursor at its stepped
            // frame on resume is [maxSteppedTs, minNextTs). If the range is
            // non-empty pick maxSteppedTs (sits exactly on the latest stepped
            // frame). If empty — pathological mixed framerate where one
            // track's next frame falls before another's stepped frame —
            // prefer minNextTs-1 to suppress Update()'s auto-advance on
            // resume; this is the lesser of two evils vs. the original
            // max-only anchor that allowed silent frame skips. Floor at
            // _playbackTrackStartNs so SyncWallClockTo's ulong subtraction
            // can never underflow.
            ulong newPlayhead = maxSteppedTs;
            if (minNextTs != ulong.MaxValue && minNextTs <= newPlayhead)
                newPlayhead = minNextTs > 0 ? minNextTs - 1 : 0;
            if (newPlayhead < _playbackTrackStartNs)
                newPlayhead = _playbackTrackStartNs;

            SyncWallClockTo(newPlayhead);
            double seconds = (newPlayhead - _playbackTrackStartNs) / 1_000_000_000.0;
            SetStatus($"Stepped {(delta > 0 ? "→" : "←")} to {seconds:0.000}s (paused)");
        }

        /// <summary>
        /// Apply <paramref name="cursor"/> to <paramref name="track"/> and run
        /// the same reconstruct + fire-event + bbox-filter chain Update() would
        /// run on a natural cursor advance, so stepping is visually identical
        /// to the live playhead crossing this frame.
        /// </summary>
        private void SetCursorAndEmit(DeviceTrack track, int cursor)
        {
            track.PlaybackCursor = cursor;
            var depthFrame = track.DepthFrames[cursor];
            PointCloudRecording.Frame colorFrame = null;
            if (track.ColorFrames.Count > 0)
            {
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
            FeedCumulative(track);
            ApplyBoundingBoxFilter(track);
        }

        // Forward the just-reconstructed playback mesh into the cumulative
        // snapshotter (if any). Shared between Update's natural-playback path
        // and SetCursorAndEmit's frame-stepping path so both honor the same
        // interval-based snapshot pacing.
        //
        // The bbox is resolved from this recorder's own filter chain (same one
        // that ApplyBoundingBoxFilter pushes to the shader) and passed to the
        // cumulative as a parameter — cumulative does not hold its own bbox
        // reference, so a single scene-wide bbox flows through both the live
        // shader filter and the cumulative pre-filter.
        private void FeedCumulative(DeviceTrack track)
        {
            var cum = ResolveCumulative();
            if (cum == null || track.PlaybackMesh == null || track.PlaybackObject == null) return;
            Material mat = track.PlaybackRenderer != null
                ? track.PlaybackRenderer.sharedMaterial
                : playbackMaterial;
            int capacity = track.DepthWidth * track.DepthHeight;
            cum.OnPlaybackFrame(track.PlaybackMesh, capacity,
                track.PlaybackObject.transform, mat, ResolveBoundingBox(), ResolveDecimater());
        }

        /// <summary>
        /// Shift _playbackWallStart so the running playhead expression
        /// (Time.timeAsDouble - _playbackWallStart) * playbackRate evaluates
        /// to (playheadNs - _playbackTrackStartNs)/1e9 right now. Also resets
        /// _pauseWallStart so ResumePlayback's pause-delta correction adds
        /// zero — i.e. resume continues from the stepped playhead.
        /// </summary>
        private void SyncWallClockTo(ulong playheadNs)
        {
            float rate = Mathf.Max(0.01f, playbackRate);
            // Defensive: avoid ulong underflow if a future caller passes a
            // playhead below the track origin. Callers in this file already
            // floor at _playbackTrackStartNs, but guard for robustness.
            ulong fromOrigin = playheadNs > _playbackTrackStartNs
                ? playheadNs - _playbackTrackStartNs : 0UL;
            double elapsedSec = fromOrigin / 1_000_000_000.0 / rate;
            _playbackWallStart = Time.timeAsDouble - elapsedSec;
            if (IsPaused) _pauseWallStart = Time.timeAsDouble;
        }

        [ContextMenu("Save")]
        public void Save()
        {
            // Body data (depth / color / IR RCSV) is now written incrementally
            // during Rec via RcsvStreamWriter, so Save is metadata-only:
            // extrinsics.yaml, hostinfo.yaml, dataset_meta.yaml. Call this
            // again after a Read + re-calibrate cycle to refresh the
            // extrinsics yaml without re-recording.
            if (_tracks.Count == 0)
            {
                SetStatus("Save: nothing to save. Use Read to load a recording first, or Rec to capture a new one.", warn: true);
                return;
            }
            try
            {
                string root = ResolveRoot();
                string host = SafeMachineName();
                WriteRecordingMetadata(root, host);
                SetStatus($"Saved metadata for {_tracks.Count} device(s) to {root}");
            }
            catch (Exception e)
            {
                SetStatus($"Save failed: {e.Message}", warn: true);
                Debug.LogException(e, this);
            }
        }

        // Write the metadata sidecars (extrinsics + hostinfo + dataset_meta)
        // for whatever is currently in _tracks. Body data is assumed already
        // present on disk via RcsvStreamWriter (Rec) or pre-existing (Read).
        // Called from StopRecording immediately after closing writers AND from
        // Save() so the user can refresh metadata after a Read+recalibrate.
        private void WriteRecordingMetadata(string root, string host)
        {
            var serials = new List<string>();
            var calibrations = new List<PointCloudRecording.DeviceCalibration>();

            // Build the per-serial extrinsics map used for the calibration
            // entries below. Priority:
            //   1. Existing recording-local yaml that already has non-identity
            //      values (= prior calibration session for THIS recording).
            //   2. The live PointCloudCameraManager's yaml (= the rig
            //      calibration Live was using when this recording was made).
            //   3. Identity (nothing else available).
            var existingGlobalBySerial = new Dictionary<string, ObExtrinsic>();
            try
            {
                string extPath = Path.Combine(PointCloudRecording.CalibrationDir(root), "extrinsics.yaml");
                if (File.Exists(extPath))
                {
                    foreach (var cal in PointCloudRecording.ReadExtrinsicsYaml(root))
                    {
                        if (cal.GlobalTrColorCamera.HasValue
                            && !IsIdentityExtrinsic(cal.GlobalTrColorCamera.Value))
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
            if (cameraManager != null)
            {
                try
                {
                    string liveRoot = cameraManager.ResolveExtrinsicsRoot();
                    string livePath = Path.Combine(PointCloudRecording.CalibrationDir(liveRoot), "extrinsics.yaml");
                    if (File.Exists(livePath))
                    {
                        foreach (var cal in PointCloudRecording.ReadExtrinsicsYaml(liveRoot))
                        {
                            if (existingGlobalBySerial.ContainsKey(cal.Serial)) continue;
                            if (cal.GlobalTrColorCamera.HasValue
                                && !IsIdentityExtrinsic(cal.GlobalTrColorCamera.Value))
                                existingGlobalBySerial[cal.Serial] = cal.GlobalTrColorCamera.Value;
                        }
                    }
                }
                catch (Exception liveEx)
                {
                    Debug.LogWarning(
                        $"[{nameof(PointCloudRecorder)}] could not read live manager's extrinsics.yaml: {liveEx.Message}", this);
                }
            }

            foreach (var kv in _tracks)
            {
                var track = kv.Value;
                serials.Add(track.Serial);

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
                        GlobalTrColorCamera = preservedGlobal,
                    });
                }
            }

            string ds = string.IsNullOrWhiteSpace(datasetName)
                ? Path.GetFileName(root.TrimEnd(Path.DirectorySeparatorChar, Path.AltDirectorySeparatorChar))
                : datasetName;
            PointCloudRecording.WriteDatasetMetadata(root, host, ds, serials);
            if (calibrations.Count > 0)
                PointCloudRecording.WriteExtrinsicsYaml(root, calibrations);
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

                    // Open RCSV files as lazy streams instead of loading every
                    // record's bytes into managed memory. Each stream eagerly
                    // reads the trailing index chunk + per-frame timestamps
                    // (~16 B per frame total) so .Count and timestamp scans are
                    // free; record bytes are pulled on demand by indexer access
                    // in playback / BodyTracking. Failure on one sensor disposes
                    // any partial streams to avoid leaking file handles before
                    // the exception propagates.
                    PointCloudRecording.RcsvFrameStream depthStream = null;
                    PointCloudRecording.RcsvFrameStream colorStream = null;
                    PointCloudRecording.RcsvFrameStream irStream = null;
                    try
                    {
                        if (hasDepth) depthStream = new PointCloudRecording.RcsvFrameStream(depthPath);
                        if (hasColor) colorStream = new PointCloudRecording.RcsvFrameStream(colorPath);
                        if (hasIR) irStream = new PointCloudRecording.RcsvFrameStream(irPath);
                    }
                    catch
                    {
                        depthStream?.Dispose();
                        colorStream?.Dispose();
                        irStream?.Dispose();
                        throw;
                    }

                    var track = GetOrCreateTrack(serial);
                    track.AdoptStreams(depthStream, colorStream, irStream);

                    if (depthStream != null)
                    {
                        totalDepth += depthStream.Count;
                        var (dw, dh) = PointCloudRecording.ReadRcsvHeaderDimensions(depthPath);
                        if (dw > 0 && dh > 0) { track.DepthWidth = dw; track.DepthHeight = dh; }
                    }
                    if (colorStream != null)
                    {
                        totalColor += colorStream.Count;
                        var (cw, ch) = PointCloudRecording.ReadRcsvHeaderDimensions(colorPath);
                        if (cw > 0 && ch > 0) { track.ColorWidth = cw; track.ColorHeight = ch; }
                    }
                    if (irStream != null)
                    {
                        totalIR += irStream.Count;
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

                // Free the live Femto Bolt pipelines now that the recording is on
                // disk in memory — the user pressed Read to play back, so the live
                // cameras compete for USB bandwidth and double-render the same
                // subjects. Live capture cannot be resumed without reloading the
                // scene; this is intentional (Read = commit to playback mode).
                int liveDestroyed = 0;
                if (cameraManager != null)
                {
                    liveDestroyed = cameraManager.Renderers.Count;
                    cameraManager.DestroyAllRenderers();
                }

                SetStatus(
                    $"Loaded {totalDepth} depth / {totalColor} color / {totalIR} IR frame(s) across {_tracks.Count} device(s)"
                    + (extrinsicsApplied > 0 ? $", extrinsics applied to {extrinsicsApplied} device(s)" : "")
                    + (liveDestroyed > 0 ? $", disconnected {liveDestroyed} live camera(s)" : "")
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
        /// Re-read extrinsics.yaml and push the latest <c>global_tr_colorCamera</c>
        /// to any existing <c>_Playback_*</c> GameObject's transform. Called from
        /// StartPlayback so the user doesn't have to press Read after a fresh
        /// CalibrationWindow Solve & Write.
        /// </summary>
        private void RefreshExtrinsicsAndReapply()
        {
            string root = ResolveRoot();
            int applied = LoadExtrinsicsFromYaml(root);
            if (applied == 0) return;
            foreach (var kv in _tracks)
            {
                var t = kv.Value;
                if (t.PlaybackObject == null) continue;
                if (!t.GlobalTrColorCamera.HasValue) continue;
                // ApplyToTransform sets localPosition/Rotation; localScale stays at
                // the (1, -1, 1) Y-flip from EnsurePlaybackObject.
                Calibration.ExtrinsicsApply.ApplyToTransform(
                    t.PlaybackObject.transform, t.GlobalTrColorCamera.Value);
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
            IReadOnlyList<PointCloudRecording.DeviceCalibration> calibrations = null;
            if (File.Exists(path))
            {
                try { calibrations = PointCloudRecording.ReadExtrinsicsYaml(root); }
                catch (Exception e)
                {
                    Debug.LogWarning($"[{nameof(PointCloudRecorder)}] extrinsics.yaml present but failed to parse: {e.Message}", this);
                }
            }

            int withGlobal = 0;
            if (calibrations != null)
            {
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
            }

            // Fallback for old recordings whose yaml is missing entirely or has
            // only identity values: pull the missing per-serial transform from the
            // live PointCloudCameraManager's yaml. Same "Live and Playback share
            // one rig calibration" principle as the Save side.
            if (cameraManager != null)
            {
                try
                {
                    string liveRoot = cameraManager.ResolveExtrinsicsRoot();
                    string livePath = Path.Combine(PointCloudRecording.CalibrationDir(liveRoot), "extrinsics.yaml");
                    if (File.Exists(livePath) && liveRoot != root)
                    {
                        foreach (var c in PointCloudRecording.ReadExtrinsicsYaml(liveRoot))
                        {
                            if (!_tracks.TryGetValue(c.Serial, out var track)) continue;
                            bool needFallback = !track.GlobalTrColorCamera.HasValue
                                || IsIdentityExtrinsic(track.GlobalTrColorCamera.Value);
                            if (!needFallback) continue;
                            if (c.GlobalTrColorCamera.HasValue
                                && !IsIdentityExtrinsic(c.GlobalTrColorCamera.Value))
                            {
                                track.GlobalTrColorCamera = c.GlobalTrColorCamera;
                                withGlobal++;
                            }
                        }
                    }
                }
                catch (Exception liveEx)
                {
                    Debug.LogWarning($"[{nameof(PointCloudRecorder)}] live manager extrinsics fallback failed: {liveEx.Message}", this);
                }
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

            // Stream-on-record requires the destination folder to be known at
            // Rec time so the per-device RcsvStreamWriter can open files. The
            // legacy "Save later picks the folder" UX is gone — folderPath
            // must be set in the Inspector before pressing Rec.
            try
            {
                _recordRoot = ResolveRoot();
                _recordHost = SafeMachineName();
                Directory.CreateDirectory(_recordRoot);
            }
            catch (Exception e)
            {
                SetStatus($"Rec: invalid folderPath '{folderPath}' — {e.Message}", warn: true);
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
            // Reset per-second diag state so a new recording's first frame doesn't
            // compute a huge gap against a stale timestamp from the previous run.
            _diagRecordPerSerial.Clear();
            _diagWindowStart = 0f;
            // If a prior recording left writers open (e.g. domain reload or a
            // bug short-circuited StopRecording), finalize them before opening
            // new files so we don't leak handles or strand half-written RCSVs.
            FinalizeAnyOpenStreamWriters();
            SetStatus($"Recording ({_subscribed.Count} device(s)) → {_recordRoot}");
        }

        private void StopRecording()
        {
            UnsubscribeAll();
            CurrentState = State.Idle;

            // Close every open RcsvStreamWriter — Dispose finalizes the index
            // chunk and patches the header. After this the files on disk are
            // self-describing and Read can load them. Snapshot frame counts
            // BEFORE dispose since the helper nulls writer refs as it goes,
            // and call the guarded helper so one stream's IO failure does not
            // strand the remaining streams (or skip _streamWriters.Clear()).
            int totalDepth = 0, totalColor = 0, totalIR = 0;
            foreach (var sw in _streamWriters.Values)
            {
                if (sw.Depth != null) totalDepth += sw.Depth.FrameCount;
                if (sw.Color != null) totalColor += sw.Color.FrameCount;
                if (sw.IR    != null) totalIR    += sw.IR.FrameCount;
            }
            FinalizeAnyOpenStreamWriters();

            // Write the metadata sidecars (extrinsics.yaml, hostinfo.yaml,
            // dataset metadata, per-device calibration). Body data is already
            // on disk via the writers above, so this is the "Save" step's
            // remaining work — wrap it here so the user doesn't have to click
            // Save separately.
            try { WriteRecordingMetadata(_recordRoot, _recordHost); }
            catch (Exception e)
            {
                Debug.LogWarning($"[{nameof(PointCloudRecorder)}] metadata write after StopRecording failed: {e}", this);
            }

            // _tracks still has per-track DepthFrames entries with TimestampNs
            // but Bytes=null (live capture path doesn't copy the buffer into
            // memory anymore). Playback indexes Bytes, so clear the in-memory
            // tracks here — the user clicks Read to reload the just-written
            // files into a Play-able form.
            int devCount = _tracks.Count;
            ClearTracks();
            SetStatus($"Recorded {totalDepth} depth / {totalColor} color / {totalIR} IR frame(s) across {devCount} device(s) to {_recordRoot}. Click Read to load for playback.");
        }

        private void HandleRawFrame(PointCloudRenderer src, RawFrameData raw)
        {
            if (CurrentState != State.Recording) return;
            if (src == null) return;

            string serial = string.IsNullOrEmpty(src.deviceSerial) ? src.name : src.deviceSerial;
            var track = GetOrCreateTrack(serial);
            if (!track.CameraParam.HasValue) track.CameraParam = src.CameraParam;

            ulong timestampNs = raw.TimestampUs * 1000UL;

            if (diagnosticLogging)
            {
                if (!_diagRecordPerSerial.TryGetValue(serial, out var rs))
                {
                    rs = new RecordWindowStats();
                    _diagRecordPerSerial[serial] = rs;
                }
                rs.Frames++;
                rs.Bytes += raw.DepthByteCount + raw.ColorByteCount + raw.IRByteCount;
                if (rs.LastTsNs != 0)
                {
                    double gapMs = (timestampNs - rs.LastTsNs) / 1e6;
                    if (gapMs > rs.MaxGapMs) rs.MaxGapMs = gapMs;
                }
                rs.LastTsNs = timestampNs;
            }

            // Stream-on-record: hand the SDK's raw buffer straight to the file
            // writer. No new byte[] is allocated and no reference is retained
            // after the call, so the Gen2 heap doesn't grow with recording
            // length and GC pauses don't blow up the device callback timing.
            // The track lists only get TimestampNs entries (Bytes = null) so
            // diagnostics / inspector counts remain accurate; the Bytes are
            // re-populated when the user Loads the just-written files.
            var sw = GetOrCreateStreamWriters(serial);
            if (raw.DepthByteCount > 0)
            {
                if (sw.Depth == null)
                {
                    string path = PointCloudRecording.SensorFilePath(_recordRoot, _recordHost, serial, PointCloudRecording.DepthSensorName);
                    string header = PointCloudRecording.BuildDepthHeaderYaml(serial, raw.DepthWidth, raw.DepthHeight);
                    sw.Depth = new PointCloudRecording.RcsvStreamWriter(path, header);
                }
                sw.Depth.WriteFrame(timestampNs, raw.DepthBytes, raw.DepthByteCount);
                track.AddRecordedDepthFrame(new PointCloudRecording.Frame { TimestampNs = timestampNs });
                track.DepthWidth = raw.DepthWidth;
                track.DepthHeight = raw.DepthHeight;
            }
            if (raw.ColorByteCount > 0)
            {
                if (sw.Color == null)
                {
                    string path = PointCloudRecording.SensorFilePath(_recordRoot, _recordHost, serial, PointCloudRecording.ColorSensorName);
                    string header = PointCloudRecording.BuildColorHeaderYaml(serial, raw.ColorWidth, raw.ColorHeight);
                    sw.Color = new PointCloudRecording.RcsvStreamWriter(path, header);
                }
                sw.Color.WriteFrame(timestampNs, raw.ColorBytes, raw.ColorByteCount);
                track.AddRecordedColorFrame(new PointCloudRecording.Frame { TimestampNs = timestampNs });
                track.ColorWidth = raw.ColorWidth;
                track.ColorHeight = raw.ColorHeight;
            }
            if (raw.IRByteCount > 0 && raw.IRBytes != null)
            {
                if (sw.IR == null)
                {
                    string path = PointCloudRecording.SensorFilePath(_recordRoot, _recordHost, serial, PointCloudRecording.IRSensorName);
                    string header = PointCloudRecording.BuildIRHeaderYaml(serial, raw.IRWidth, raw.IRHeight);
                    sw.IR = new PointCloudRecording.RcsvStreamWriter(path, header);
                }
                sw.IR.WriteFrame(timestampNs, raw.IRBytes, raw.IRByteCount);
                track.AddRecordedIRFrame(new PointCloudRecording.Frame { TimestampNs = timestampNs });
                track.IRWidth = raw.IRWidth;
                track.IRHeight = raw.IRHeight;
            }
        }

        private StreamWriters GetOrCreateStreamWriters(string serial)
        {
            if (!_streamWriters.TryGetValue(serial, out var sw))
            {
                sw = new StreamWriters();
                _streamWriters[serial] = sw;
            }
            return sw;
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

            // Refresh extrinsics from yaml so a fresh calibration written between
            // Read and Play is picked up without forcing the user to press Read
            // again. Also re-applies the new global_tr_colorCamera to any existing
            // _Playback_* GameObject (their transforms were set on first creation).
            RefreshExtrinsicsAndReapply();

            ulong firstTs = ulong.MaxValue;
            bool anyFrames = false;
            foreach (var kv in _tracks)
            {
                var track = kv.Value;
                if (track.DepthFrames.Count == 0) continue;
                anyFrames = true;
                ulong firstFrameTs = TimestampNsAt(track.DepthFrames, 0);
                if (firstFrameTs < firstTs) firstTs = firstFrameTs;
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
            IsPaused = false;
            CurrentState = State.Playing;
            SetStatus($"Playing {_tracks.Count} device(s)…");
        }

        private void StopPlayback()
        {
            CurrentState = State.Idle;
            IsPaused = false;
            SetStatus("Playback stopped.");
        }

        private void Awake()
        {
            if (view == null) view = FindFirstObjectByType<PointCloudView>();
        }

        private void Start()
        {
            // PointCloudCameraManager.playbackOnly means "don't connect to live devices,
            // play this folder instead". When set, auto-Load the configured folderPath
            // and immediately enter playback so pressing Editor Play "just works".
            if (cameraManager != null && cameraManager.playbackOnly)
            {
                Load();
                if (CurrentState == State.Idle && RecordedFrameCount > 0)
                    TogglePlay();
            }
        }

        private void Update()
        {
            // Recording has no per-tick work, but we still want the per-second
            // diag heartbeat to log capture-side fps / drops while it's happening.
            if (CurrentState == State.Recording)
            {
                if (diagnosticLogging) PerSecondDiag();
                return;
            }
            if (CurrentState != State.Playing) return;

            // Spacebar toggles pause/resume during playback (issue #17).
            if (Input.GetKeyDown(KeyCode.Space)) TogglePause();
            // Left / Right arrow step the cursor by one frame for close
            // inspection of generated mesh + BT output (issue #19).
            // Stepping auto-pauses if playback is still running.
            if (Input.GetKeyDown(KeyCode.RightArrow)) StepForward();
            if (Input.GetKeyDown(KeyCode.LeftArrow))  StepBackward();

            if (IsPaused)
            {
                if (diagnosticLogging) PerSecondDiag();
                return;
            }
            double elapsedSec = (Time.timeAsDouble - _playbackWallStart) * Mathf.Max(0.01f, playbackRate);
            ulong elapsedNs = (ulong)Math.Max(0.0, elapsedSec * 1_000_000_000.0);
            ulong playheadNs = _playbackTrackStartNs + elapsedNs;

            bool anyRemaining = false;
            foreach (var kv in _tracks)
            {
                var track = kv.Value;
                if (track.DepthFrames.Count == 0) continue;

                int cursor = Mathf.Max(track.PlaybackCursor, 0);
                int cursorBeforeAdvance = track.PlaybackCursor;
                while (cursor + 1 < track.DepthFrames.Count
                       && TimestampNsAt(track.DepthFrames, cursor + 1) <= playheadNs)
                    cursor++;

                // Playback drop diagnostic: when one Update consumes multiple
                // frames the operator never sees the intermediate ones — that's
                // a viewer-side skip (GC stall, mesh upload stall, foreground
                // window scrolling, etc.). Capture-time drops by contrast are
                // baked into the RCSV timestamps and surface as gaps in the
                // analyzer (Window > Diag > Analyze Recording Gaps). Gating on
                // diagnosticLogging so this stays opt-in.
                if (diagnosticLogging && cursorBeforeAdvance >= 0)
                {
                    int advanced = cursor - cursorBeforeAdvance;
                    if (advanced > 1)
                    {
                        double playheadSec = (playheadNs - _playbackTrackStartNs) / 1e9;
                        Debug.LogWarning($"[Playback {track.Serial}] viewer skipped {advanced - 1} frame(s) " +
                                         $"at cursor {cursor + 1}/{track.DepthFrames.Count} " +
                                         $"(playhead={playheadSec:F2}s, dt={Time.deltaTime * 1000f:F1}ms)");
                    }
                }

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
                    // Only emit on real cursor advances so a paused playhead doesn't
                    // re-snapshot the same frame every Update tick.
                    FeedCumulative(track);
                }
                ApplyBoundingBoxFilter(track);
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

            if (diagnosticLogging) PerSecondDiag();
        }

        private void PerSecondDiag()
        {
            float now = Time.realtimeSinceStartup;
            // Track the longest Update tick this window — a single 200 ms tick
            // is enough to drop 6 frames at 30 fps and would explain the gap
            // patterns we see in the RCSV analyzer. Use unscaledDeltaTime so
            // Time.timeScale pauses don't masquerade as host stalls.
            float dtMs = Time.unscaledDeltaTime * 1000f;
            if (dtMs > _diagMaxUpdateDtMs)
            {
                _diagMaxUpdateDtMs = dtMs;
                _diagMaxUpdateDtAtSec = now;
            }

            if (_diagWindowStart == 0f)
            {
                _diagWindowStart = now;
                SnapshotProcessDiagBaseline();
            }
            float elapsed = now - _diagWindowStart;
            if (elapsed < 1f) return;

            if (CurrentState == State.Recording && _diagRecordPerSerial.Count > 0)
            {
                var parts = new List<string>(_diagRecordPerSerial.Count);
                foreach (var kv in _diagRecordPerSerial)
                {
                    double fps = kv.Value.Frames / elapsed;
                    double mbps = kv.Value.Bytes / 1048576.0 / elapsed;
                    parts.Add($"{TruncSerial(kv.Key)}={fps:F1}fps,{mbps:F1}MB/s,gap={kv.Value.MaxGapMs:F0}ms");
                }

                int gc0 = System.GC.CollectionCount(0) - _diagGc0Start;
                int gc1 = System.GC.CollectionCount(1) - _diagGc1Start;
                int gc2 = System.GC.CollectionCount(2) - _diagGc2Start;
                long monoNow = UnityEngine.Profiling.Profiler.GetMonoUsedSizeLong();
                long monoDeltaKB = (monoNow - _diagMonoMemStart) / 1024;

                Debug.Log($"[PointCloudRecorder REC] {string.Join(" | ", parts)} || " +
                          $"gc=g0:{gc0},g1:{gc1},g2:{gc2} mono+={monoDeltaKB}KB " +
                          $"maxTick={_diagMaxUpdateDtMs:F0}ms@{_diagMaxUpdateDtAtSec - _diagWindowStart:F2}s", this);

                foreach (var key in new List<string>(_diagRecordPerSerial.Keys))
                {
                    var rs = _diagRecordPerSerial[key];
                    rs.Frames = 0; rs.Bytes = 0; rs.MaxGapMs = 0;
                }
            }

            if (CurrentState == State.Playing && _diagFiresPerSerial.Count > 0)
            {
                var parts = new List<string>(_diagFiresPerSerial.Count);
                foreach (var kv in _diagFiresPerSerial) parts.Add($"{kv.Key}={kv.Value}/s");
                Debug.Log($"[PointCloudRecorder PLAY] playback_fires {string.Join(" ", parts)}", this);
                foreach (var key in new List<string>(_diagFiresPerSerial.Keys)) _diagFiresPerSerial[key] = 0;
            }

            _diagWindowStart = now;
            // Always rebaseline GC/mem at window roll-over so a Play -> Record
            // transition doesn't report stale-window deltas on the first emit.
            SnapshotProcessDiagBaseline();
            _diagMaxUpdateDtMs = 0f;
        }

        // Snapshot GC counters + mono heap size at the start of every per-second
        // diag window so the next emit can report the delta.
        private void SnapshotProcessDiagBaseline()
        {
            _diagGc0Start    = System.GC.CollectionCount(0);
            _diagGc1Start    = System.GC.CollectionCount(1);
            _diagGc2Start    = System.GC.CollectionCount(2);
            _diagMonoMemStart = UnityEngine.Profiling.Profiler.GetMonoUsedSizeLong();
        }

        private static string TruncSerial(string s)
        {
            if (string.IsNullOrEmpty(s) || s.Length <= 6) return s;
            return s.Substring(s.Length - 6);
        }

        // Per-frame MPB scratch handed to PointCloudShaderFilters.Apply so the
        // playback path stays bit-for-bit identical with the live PointCloudRenderer.
        private MaterialPropertyBlock _filterMpb;

        private PointCloudBoundingBox ResolveBoundingBox()
        {
            if (boundingBox != null) return boundingBox;
            // Late-bind from any live PointCloudRenderer that already references one
            // (shared scene-level box pattern). When no live renderers exist
            // (playback-only sessions), fall back to a scene-wide search so we still
            // pick up the same box that authoring puts on the scene.
            foreach (var r in CollectSourceRenderers())
            {
                if (r != null && r.boundingBox != null)
                {
                    boundingBox = r.boundingBox;
                    return boundingBox;
                }
            }
            boundingBox = FindFirstObjectByType<PointCloudBoundingBox>();
            return boundingBox;
        }

        private PointCloudDecimater ResolveDecimater()
        {
            if (decimater != null) return decimater;
            foreach (var r in CollectSourceRenderers())
            {
                if (r != null && r.decimater != null) { decimater = r.decimater; return decimater; }
            }
            decimater = FindFirstObjectByType<PointCloudDecimater>();
            return decimater;
        }

        private PointCloudCapsuleFilter ResolveCapsuleFilter()
        {
            if (capsuleFilter != null) return capsuleFilter;
            foreach (var r in CollectSourceRenderers())
            {
                if (r != null && r.capsuleFilter != null)
                {
                    capsuleFilter = r.capsuleFilter;
                    return capsuleFilter;
                }
            }
            capsuleFilter = FindFirstObjectByType<PointCloudCapsuleFilter>();
            return capsuleFilter;
        }

        private PointCloudJointMotionField ResolveJointMotionField()
        {
            if (jointMotionField != null) return jointMotionField;
            foreach (var r in CollectSourceRenderers())
            {
                if (r != null && r.jointMotionField != null)
                {
                    jointMotionField = r.jointMotionField;
                    return jointMotionField;
                }
            }
            jointMotionField = FindFirstObjectByType<PointCloudJointMotionField>();
            return jointMotionField;
        }

        private PointCloudCumulative ResolveCumulative()
        {
            if (cumulative != null) return cumulative;
            foreach (var r in CollectSourceRenderers())
            {
                if (r != null && r.cumulative != null)
                {
                    cumulative = r.cumulative;
                    return cumulative;
                }
            }
            cumulative = FindFirstObjectByType<PointCloudCumulative>();
            return cumulative;
        }

        private void ApplyBoundingBoxFilter(DeviceTrack track)
        {
            if (track.PlaybackRenderer == null) return;
            // Visibility is owned by PointCloudView (registered at spawn time).
            if (_filterMpb == null) _filterMpb = new MaterialPropertyBlock();
            PointCloudShaderFilters.Apply(track.PlaybackRenderer, _filterMpb,
                track.PlaybackObject.transform, ResolveBoundingBox(), ResolveDecimater(),
                ResolveCapsuleFilter(), ResolveJointMotionField());
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
            track.Reconstructor = new PointCloudReconstructor(track.Serial);
            if (view != null) view.Register(mr);
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
            if (diagnosticLogging)
            {
                _diagFiresPerSerial.TryGetValue(track.Serial, out var n);
                _diagFiresPerSerial[track.Serial] = n + 1;
            }
        }

        // Per-track playback reconstruction. Delegates the entire GPU pipeline
        // (mesh / buffers / shader dispatch) to PointCloudReconstructor, which
        // is shared with the live capture path (PointCloudRenderer).
        private void ReconstructAndUpload(
            DeviceTrack track,
            PointCloudRecording.Frame depthFrame,
            PointCloudRecording.Frame colorFrame)
        {
            int dw = track.DepthWidth, dh = track.DepthHeight;
            if (dw <= 0 || dh <= 0) return;

            EnsurePlaybackObject(track);

            if (!track.CameraParam.HasValue)
            {
                // No intrinsics — nothing to reconstruct. Blank the mesh so the
                // last frame's points don't linger on the playback GO.
                track.Reconstructor.BlankMesh();
                return;
            }

            int cw = track.ColorWidth, ch = track.ColorHeight;
            bool hasColor = colorFrame != null && colorFrame.Bytes != null && cw > 0 && ch > 0;
            byte[] colorBytes = hasColor ? colorFrame.Bytes : null;
            int colorByteCount = hasColor ? colorFrame.Bytes.Length : 0;

            if (!track.Reconstructor.Dispatch(
                    depthFrame.Bytes, depthFrame.Bytes.Length, dw, dh,
                    colorBytes, colorByteCount, cw, ch,
                    track.CameraParam.Value))
            {
                Debug.LogError(
                    $"[{nameof(PointCloudRecorder)}] PointCloudReconstruct compute shader not found in Resources/", this);
                return;
            }

            // Mesh may have been (re-)allocated by EnsureMesh inside Dispatch on
            // the very first frame, so keep MeshFilter.sharedMesh in sync. Cheap
            // no-op after the first frame.
            if (track.PlaybackFilter.sharedMesh != track.Reconstructor.Mesh)
                track.PlaybackFilter.sharedMesh = track.Reconstructor.Mesh;
        }

        // --- Lifecycle / helpers ---

        private void OnDisable()
        {
            // If we're mid-recording the lifecycle cuts the OnRawFramesReady
            // pipeline immediately, but the RcsvStreamWriter dispose path is
            // what flushes the trailing index chunk + patches the header. Skip
            // it and the just-written files become unreadable. Reuse the
            // normal StopRecording path so metadata writes also fire.
            if (CurrentState == State.Recording) StopRecording();
            else FinalizeAnyOpenStreamWriters();
            UnsubscribeAll();
            if (CurrentState != State.Idle) CurrentState = State.Idle;
        }

        private void OnDestroy()
        {
            if (CurrentState == State.Recording) StopRecording();
            else FinalizeAnyOpenStreamWriters();
            UnsubscribeAll();
            ClearTracks();
        }

        // Defensive cleanup for any lingering RcsvStreamWriter (e.g. if
        // StartRecording opened streams but StopRecording was never called).
        // No-op when _streamWriters is already empty. Swallows IO exceptions
        // so we don't mask the original lifecycle event with a write error.
        private void FinalizeAnyOpenStreamWriters()
        {
            if (_streamWriters.Count == 0) return;
            foreach (var sw in _streamWriters.Values)
            {
                try { sw.Depth?.Dispose(); } catch (Exception e) { Debug.LogWarning($"[{nameof(PointCloudRecorder)}] depth writer dispose failed: {e.Message}", this); }
                try { sw.Color?.Dispose(); } catch (Exception e) { Debug.LogWarning($"[{nameof(PointCloudRecorder)}] color writer dispose failed: {e.Message}", this); }
                try { sw.IR?.Dispose();    } catch (Exception e) { Debug.LogWarning($"[{nameof(PointCloudRecorder)}] IR writer dispose failed: {e.Message}",    this); }
                sw.Depth = sw.Color = sw.IR = null;
            }
            _streamWriters.Clear();
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
                // Reconstructor disposes its own GraphicsBuffers, scratch arrays,
                // and (importantly) the Mesh it owns — caller MUST NOT manually
                // Destroy track.PlaybackMesh.
                if (view != null && track.PlaybackRenderer != null)
                    view.Unregister(track.PlaybackRenderer);
                track.Reconstructor?.Dispose();
                track.Reconstructor = null;
                if (track.PlaybackObject != null)
                {
                    if (Application.isPlaying) Destroy(track.PlaybackObject);
                    else DestroyImmediate(track.PlaybackObject);
                }
                // Close any open RcsvFrameStream handles. Skipping this would
                // leak FileStreams (12 handles per Read in a 4-cam setup) and
                // keep Windows from reopening the file for append on a
                // subsequent recording session.
                track.Dispose();
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

        /// <summary>
        /// Cheap timestamp lookup that avoids materializing a frame's bytes
        /// when the backing is an <see cref="PointCloudRecording.RcsvFrameStream"/>.
        /// The playback Update loop and gap diagnostics peek at next-frame
        /// timestamps every tick; routing those through the indexer would
        /// allocate a fresh byte[] per peek (~700 KB for depth) and read the
        /// full record off disk just to discard everything except 8 bytes.
        /// </summary>
        private static ulong TimestampNsAt(IReadOnlyList<PointCloudRecording.Frame> frames, int idx)
        {
            if (frames is PointCloudRecording.RcsvFrameStream s) return s.TimestampNsAt(idx);
            return frames[idx].TimestampNs;
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

}
