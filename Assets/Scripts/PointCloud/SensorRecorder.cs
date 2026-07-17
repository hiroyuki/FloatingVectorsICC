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
    public class SensorRecorder : MonoBehaviour, Shared.IRecorderTransport, Shared.IPanelTunable
    {
        public enum State { Idle, Recording, Playing }

        [Header("Source")]
        [Tooltip("Camera manager whose spawned PointCloudRenderers will be recorded. " +
                 "If null, all PointCloudRenderers in the scene are used.")]
        public SensorManager cameraManager;

        [Tooltip("Material used for playback meshes. If null, falls back to each source renderer's pointMaterial.")]
        public Material playbackMaterial;

        [Tooltip("Optional bounding box used to cull playback points the same way live PointCloudRenderers do. " +
                 "If null, the first BoundingVolume referenced by any live PointCloudRenderer in the " +
                 "scene is used. Set filterMode=Disabled to disable culling without unassigning the reference.")]
        public BoundingVolume boundingBox;

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
                 "mesh's vertex buffer is read back from GPU and snapshotted every 'intervalSeconds' seconds. " +
                 "If null, the first PointCloudCumulative in the scene is used. Snapshots include all reconstructed " +
                 "vertices (invalid depth pixels are written far off-screen by the reconstruct shader and clip-culled " +
                 "at draw time, same as the live playback mesh).")]
        public PointCloudCumulative cumulative;

        [Header("Files")]
        [Tooltip("RECORDING root — where Rec writes new takes. Each Rec creates a " +
                 "timestamped subfolder under here (see autoTimestampFolder). Relative " +
                 "paths resolve under Application.persistentDataPath. Leave empty to use " +
                 "'<persistentDataPath>/Recordings/recording'. Playback reads from " +
                 "playbackFolderPath instead, so recording and playback never collide.")]
        public string folderPath = "";

        [Tooltip("Optional override used only when running on macOS — Windows absolute paths like " +
                 "'D:/Dropbox/...' obviously do not exist here, so a dev opening the project on a Mac " +
                 "can point at the local Dropbox mount without touching the canonical folderPath.")]
        public string folderPathMacOverride = "";

        [Tooltip("PLAYBACK root — the specific recording Read/Play loads. Point this at a " +
                 "single take folder (e.g. <recordingRoot>/2026-07-08_12-50-09). Kept " +
                 "separate from folderPath so playing an old take never overwrites where " +
                 "new recordings are written. Leave empty to fall back to folderPath. " +
                 "Changing it while playing (or idle without live cameras) re-Reads and " +
                 "restarts playback automatically.")]
        public string playbackFolderPath = "";

        [Tooltip("macOS-only override for playbackFolderPath (same rationale as folderPathMacOverride).")]
        public string playbackFolderPathMacOverride = "";

        [Tooltip("Re-base the calibration world for playback GOs: origin = floor projection " +
                 "of the 4-camera centroid, +X = camera1→2 (yaw + XZ only). Same contract as " +
                 "SensorManager.applyWorldRebase so live and playback share one world. " +
                 "Needs rigSerialOrder. Off → Dev mode unchanged.")]
        public bool applyWorldRebase = false;

        [Tooltip("The rig's 4 camera serials in order 1..4 (must match SensorManager.rigSerialOrder). " +
                 "Rebase is skipped with a warning when these don't resolve against extrinsics.yaml.")]
        public string[] rigSerialOrder = new string[0];

        [Tooltip("Physical floor height in the CALIBRATION frame (m) — must match " +
                 "SensorManager.rebaseFloorY. The rebase shifts Y so this becomes y=0.")]
        public float rebaseFloorY = 0f;

        [Tooltip("Skip destroying the live renderers when a recording is loaded. The " +
                 "experience flow's attract playback coexists with the live rig (device " +
                 "re-enumeration costs ~15 s, unacceptable at the attract→visitor moment). " +
                 "Off = legacy behavior (playback frees the live pipelines).")]
        public bool keepLiveRenderersOnLoad = false;

        [Tooltip("Playback: fire OnPlaybackRawFrame for EVERY frame the playhead crossed " +
                 "this Update, not only the newest one (mesh upload stays newest-only). " +
                 "Live-sim consumers (LiveFusedBodySource) need every frame so fusion " +
                 "quality is not bound to the editor frame rate; the k4abt worker path " +
                 "leaves this off — workers only want the freshest frame.")]
        public bool deliverAllPlaybackFrames = false;

        [Tooltip("Playback: render the point cloud from this many frames BEHIND the " +
                 "playhead (events/fusion still get the newest frame). Aligns the " +
                 "displayed cloud with the live-fused skeleton, whose output lags " +
                 "~100-130ms (burst close + median-lag filter + inference). ~4 at " +
                 "30fps. 0 = off. Note: recorded-bodies playback matches bodies to " +
                 "the playhead, so leave this 0 when viewing recorded bodies. Only " +
                 "applies to natural playback — manual seek/step always renders the " +
                 "exact cursor frame.")]
        [Range(0, 15)]
        public int playbackRenderDelayFrames = 0;

        // ---- Shared.IPanelTunable (one-stop Control Panel) ----
        public string TuningLabel => "Playback";
        public int TunableCount => 1;
        public string TunableName(int i) => "Render delay (frames)";
        public float TunableValue(int i) => playbackRenderDelayFrames;
        public void SetTunableValue(int i, float value)
            => playbackRenderDelayFrames = Mathf.Clamp(Mathf.RoundToInt(value), 0, 15);
        public float TunableMin(int i) => 0f;
        public float TunableMax(int i) => 15f;
        public bool TunableIsInt(int i) => true;

        [Tooltip("Dataset name written into dataset.yaml. Defaults to the recording folder name.")]
        public string datasetName = "";

        [Tooltip("When on, each Rec creates a fresh timestamped subfolder " +
                 "<folderPath>/<yyyy-MM-dd_HH-mm-ss> so successive takes never overwrite " +
                 "each other. The created path is reported in the status line; to play a take " +
                 "back, point folderPath at that subfolder before pressing Read.")]
        public bool autoTimestampFolder = true;

        [Header("Hotkeys")]
        [Tooltip("Key to toggle Rec (start/stop) during live operation. Records depth/color/IR " +
                 "AND bodies_main (captured automatically by SkeletonMerger while recording) at " +
                 "once. Ignored during playback so it can't clobber a playback session. " +
                 "Set to None to disable the hotkey.")]
        public KeyCode recordKey = KeyCode.F9;

        [Min(0f)]
        [Tooltip("Seconds between Space and the live freeze actually engaging, with a big " +
                 "on-screen countdown — gives a dancer/operator time to hit the pose. " +
                 "0 = freeze immediately (old behaviour). Unfreeze is always immediate; " +
                 "Space during the countdown cancels it. Playback pause is unaffected.")]
        public float liveFreezeCountdownSeconds = 5f;

        // Wall-clock (Time.timeAsDouble) when the pending live freeze fires; < 0 = none.
        private double _freezeCountdownEnd = -1.0;

        [Header("Playback")]
        public bool loop = true;

        [Tooltip("Mask the depth pixels where a rival camera's IR projector appears. Facing " +
                 "ToF pairs saturate each other's sensor there and the flare returns FALSE " +
                 "mid-air depth (flickering speckle on the pair's sightline). Gated per frame " +
                 "on IR brightness at the projector pixel, so a body occluding the projector " +
                 "is never masked out. Applied to the raw depth before every consumer " +
                 "(mesh, TSDF, BT, event subscribers).")]
        public bool maskRivalProjectors = true;

        [Tooltip("Where to register playback meshes for visibility toggling. " +
                 "Auto-found via FindFirstObjectByType if left null.")]
        public PointCloudView view;

        [Min(0.01f)]
        [Tooltip("Playback rate multiplier (1.0 = real time).")]
        public float playbackRate = 1.0f;

        [Range(-10, 10)]
        [Tooltip("Lead the body (skeleton) stream by this many frames relative to depth, to " +
                 "compensate the k4abt skeleton lag baked into bodies_main (+1/+2 typical). " +
                 "Positive = show the body frame this many ahead of the depth frame (skeleton " +
                 "catches up to the point cloud); negative = lag it. Frame-exact because the " +
                 "body and depth tracks are 1:1. Playback only.")]
        public int bodyLeadFrames = 0;
        private int _lastBodyLeadFrames;

        // Playback-path hot-swap poll (see Update): last resolved playback root the
        // poll acted on. Polled rather than OnValidate so path edits from the Control
        // Panel / code are caught too, not just the Inspector. Not serialized — it
        // re-seeds on the first poll after entering Play mode, so simply starting
        // the app never counts as a "change".
        private string _watchedPlaybackRoot;
        private float _nextPlaybackPathCheckTime;

        /// <summary>
        /// Fires once per advanced playback frame per device, with the same payload
        /// shape a live PointCloudRenderer would produce in <c>OnRawFramesReady</c>.
        /// Subscribers (e.g. SkeletonMerger) use this to run merge / inference
        /// against recorded footage; frame-by-frame stepping then works via Unity's
        /// built-in Pause + Step buttons because cursor advancement is driven by
        /// <see cref="Time.timeAsDouble"/>.
        /// </summary>
        public event System.Action<string, ObCameraParam?, Transform, RawFrameData> OnPlaybackRawFrame;

        /// <summary>
        /// Fires once per advanced playback body frame per device. <c>tsNs</c> is the
        /// recorded body-frame timestamp (matches the depth-frame ts the bodies were
        /// tracked from). <c>bytes</c> + <c>byteCount</c> are the bodies_main RCSV
        /// payload — subscribers decode with <c>BodyTracking.RecordedBodySerializer</c>.
        /// The byte buffer is owned by an RcsvFrameStream's reusable scratch; do not
        /// retain a reference past the synchronous handler. <c>sourceTransform</c> is
        /// the per-device <c>_Playback_&lt;serial&gt;</c> transform so subscribers can
        /// re-project skeletons into world space the same way live frames do.
        /// </summary>
        public event System.Action<string, ulong, byte[], int, Transform, ObCameraParam?> OnPlaybackBodies;

        /// <summary>
        /// Fires once each time looping playback wraps from the end back to the start
        /// (every track's cursor reset to -1). The playhead — and therefore any
        /// stateful downstream consumer that tracks pose over time (live k4abt workers'
        /// trackers, SkeletonMerger continuity, BonePoseHistory ring) — jumps
        /// discontinuously backward here, so subscribers should flush their carried-over
        /// state to avoid stranding the previous loop's last body as a ghost.
        /// </summary>
        public event System.Action OnPlaybackLooped;

        // --- Runtime state ---
        public State CurrentState { get; private set; } = State.Idle;

        /// <summary>True while playback is paused (CurrentState stays at Playing).
        /// Cursor advancement and OnPlaybackRawFrame events are halted; resuming
        /// shifts the wall-clock origin so the playhead picks up where it left off.
        /// </summary>
        public bool IsPaused { get; private set; }

        // ---- Shared.IRecorderTransport (Control Panel) ----
        // ToggleRecord / TogglePlay / TogglePause below already satisfy the interface;
        // these readouts complete it so the panel can drive the transport centrally.
        public bool IsRecording => CurrentState == State.Recording;
        public bool IsPlaying => CurrentState == State.Playing;
        public string RecordingFolder
        {
            get => folderPath;
            set => folderPath = value ?? string.Empty;
        }
        public string ResolvedRecordingFolder => ResolveRoot();
        public string TransportStatus =>
            CurrentState + (IsPaused ? " (paused)" : "")
            + (CurrentState == State.Playing ? $" @ {CurrentPlayheadSeconds:0.00}s" : "")
            + (string.IsNullOrEmpty(StatusMessage) ? "" : " — " + StatusMessage);

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
            // when IR streaming was off at record time.
            public IReadOnlyList<PointCloudRecording.Frame> IRFrames;
            // Body-tracker results recorded alongside the sensor frames (one frame per
            // worker output). Empty when bodies_main was absent at record time (older
            // recordings) or when no SensorRecorder.bodyWorkerHost was bound. Each
            // Frame's Bytes payload is the bodies_main RCSV payload — decode with
            // BodyTracking.RecordedBodySerializer.
            public IReadOnlyList<PointCloudRecording.Frame> BodyFrames;
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
                    BodyFrames = t.BodyFrames,
                });
            }
            return list;
        }

        /// <summary>
        /// True when a body track is loaded on disk for <paramref name="serial"/>
        /// (recorded BT data exists). Used by SkeletonMerger to skip the
        /// k4abt worker spawn during playback — the live merge pipeline picks up
        /// bodies via <see cref="OnPlaybackBodies"/> instead, which also works on
        /// platforms without k4abt (Mac).
        /// </summary>
        public bool HasRecordedBodies(string serial)
        {
            if (string.IsNullOrEmpty(serial)) return false;
            return _tracks.TryGetValue(serial, out var t)
                   && t.BodyFrames != null
                   && t.BodyFrames.Count > 0;
        }

        /// <summary>
        /// True when the loaded recording has a track for <paramref name="serial"/>.
        /// Used by SkeletonMerger to keep live camera frames out of the k4abt worker
        /// feed while that serial is being played back — mixing live and recorded
        /// timestamps in one worker makes the loop-seam guard drop every skeleton.
        /// </summary>
        public bool HasTrack(string serial)
        {
            return !string.IsNullOrEmpty(serial) && _tracks.ContainsKey(serial);
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
            public IReadOnlyList<PointCloudRecording.Frame> BodyFrames { get; private set; }
                = Array.Empty<PointCloudRecording.Frame>();

            private List<PointCloudRecording.Frame> _depthList;
            private List<PointCloudRecording.Frame> _colorList;
            private List<PointCloudRecording.Frame> _irList;
            private List<PointCloudRecording.Frame> _bodyList;
            private PointCloudRecording.RcsvFrameStream _depthStream;
            private PointCloudRecording.RcsvFrameStream _colorStream;
            private PointCloudRecording.RcsvFrameStream _irStream;
            private PointCloudRecording.RcsvFrameStream _bodyStream;

            // Cursor into BodyFrames (independent of depth cursor: worker output rate
            // can lag the depth stream and may skip frames). Playback Update advances
            // it monotonically and fires OnPlaybackBodies for each crossed frame.
            public int BodyPlaybackCursor;

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
            public void AddRecordedBodyFrame(PointCloudRecording.Frame f)
            { EnsureRecordingMode(); _bodyList.Add(f); }

            private void EnsureRecordingMode()
            {
                if (_depthStream != null || _colorStream != null
                    || _irStream != null || _bodyStream != null)
                    Dispose();
                if (_depthList == null) _depthList = new List<PointCloudRecording.Frame>();
                if (_colorList == null) _colorList = new List<PointCloudRecording.Frame>();
                if (_irList == null) _irList = new List<PointCloudRecording.Frame>();
                if (_bodyList == null) _bodyList = new List<PointCloudRecording.Frame>();
                DepthFrames = _depthList;
                ColorFrames = _colorList;
                IRFrames = _irList;
                BodyFrames = _bodyList;
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
                PointCloudRecording.RcsvFrameStream ir,
                PointCloudRecording.RcsvFrameStream bodies)
            {
                Dispose();
                _depthStream = depth;
                _colorStream = color;
                _irStream = ir;
                _bodyStream = bodies;
                DepthFrames = (IReadOnlyList<PointCloudRecording.Frame>)depth
                              ?? Array.Empty<PointCloudRecording.Frame>();
                ColorFrames = (IReadOnlyList<PointCloudRecording.Frame>)color
                              ?? Array.Empty<PointCloudRecording.Frame>();
                IRFrames = (IReadOnlyList<PointCloudRecording.Frame>)ir
                           ?? Array.Empty<PointCloudRecording.Frame>();
                BodyFrames = (IReadOnlyList<PointCloudRecording.Frame>)bodies
                             ?? Array.Empty<PointCloudRecording.Frame>();
            }

            public void Dispose()
            {
                _depthStream?.Dispose(); _depthStream = null;
                _colorStream?.Dispose(); _colorStream = null;
                _irStream?.Dispose(); _irStream = null;
                _bodyStream?.Dispose(); _bodyStream = null;
                _depthList = null;
                _colorList = null;
                _irList = null;
                _bodyList = null;
                DepthFrames = Array.Empty<PointCloudRecording.Frame>();
                ColorFrames = Array.Empty<PointCloudRecording.Frame>();
                IRFrames = Array.Empty<PointCloudRecording.Frame>();
                BodyFrames = Array.Empty<PointCloudRecording.Frame>();
            }
        }

        private readonly Dictionary<string, DeviceTrack> _tracks = new Dictionary<string, DeviceTrack>();
        private readonly List<PointCloudRenderer> _subscribed = new List<PointCloudRenderer>();
        private readonly Dictionary<PointCloudRenderer, Action<PointCloudRenderer, RawFrameData>>
            _subscribedHandlers = new Dictionary<PointCloudRenderer, Action<PointCloudRenderer, RawFrameData>>();
        private double _playbackWallStart;
        private ulong _playbackTrackStartNs;
        private double _pauseWallStart;
        private ulong _lastPlayheadNs;   // most recent playhead (for readout / time-seek)

        /// <summary>Absolute playhead in recorded-timestamp nanoseconds. Consumers
        /// that stream the RCSV files themselves (LiveFusedBodySource's playback
        /// tap) pace their own cursors off this.</summary>
        public ulong CurrentPlayheadNs => _lastPlayheadNs;

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

            // --- per-stream WriteFrame() timing (Stopwatch ticks) ---
            // Sum + max + count for depth / color / IR writes during this window.
            // Avg = ticks / count, max = single worst call. Lets us tell whether
            // disk I/O is the source of host-side stalls or whether something
            // else on the main-thread callback (BT enqueue, allocations,
            // diagnostics) is the culprit.
            public long DepthWriteTicks, ColorWriteTicks, IRWriteTicks;
            public long DepthWriteMaxTicks, ColorWriteMaxTicks, IRWriteMaxTicks;
            public int DepthWriteCount, ColorWriteCount, IRWriteCount;

            // Total callback wall-clock (ticks) and mono heap delta over the
            // window — the mono delta isolates allocation activity attributable
            // to the record callback itself (separate from BT / renderer paths).
            public long CallbackTicks;
            public long CallbackMaxTicks;
            public long CallbackMonoDeltaBytes;
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
            public PointCloudRecording.RcsvStreamWriter Bodies;
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

        /// <summary>
        /// Root folder of the most recent recording (the timestamped take folder when
        /// autoTimestampFolder is on). Set by StartRecording and left intact by
        /// StopRecording, so a caller that drives Rec programmatically (experience
        /// flow) can find the take it just captured. Null before the first Rec.
        /// </summary>
        public string LastRecordingRoot => _recordRoot;

        [ContextMenu("Toggle Play")]
        public void TogglePlay()
        {
            if (CurrentState == State.Playing) StopPlayback();
            else StartPlayback();
        }

        /// <summary>
        /// True while live PointCloudRenderers exist. Loading a recording
        /// destroys them (Load → DestroyAllRenderers), so this doubles as the
        /// live/playback mode indicator for the Inspector's Switch Mode button.
        /// </summary>
        public bool IsLiveMode => ResolveManager() != null && ResolveManager().Renderers.Count > 0;

        /// <summary>
        /// Startup mode selector, backed by SensorManager.playbackOnly. Set this
        /// BEFORE entering Play mode: true → the scene auto-starts in PLAYBACK
        /// (Start() auto-plays the recording folder), false → LIVE capture. The
        /// Control Panel exposes it as a checkbox so the operator confirms the
        /// mode before pressing Play, without switching mid-session.
        /// </summary>
        public bool StartInPlaybackMode
        {
            get { var m = ResolveManager(); return m != null && m.playbackOnly; }
            set
            {
                var m = ResolveManager();
                if (m == null)
                {
                    SetStatus("Startup mode: no SensorManager in scene.", warn: true);
                    return;
                }
                if (m.playbackOnly == value) return;
                m.playbackOnly = value;
#if UNITY_EDITOR
                UnityEditor.EditorUtility.SetDirty(m);
                if (!Application.isPlaying)
                    UnityEditor.SceneManagement.EditorSceneManager.MarkSceneDirty(m.gameObject.scene);
#endif
                SetStatus($"Startup mode = {(value ? "PLAYBACK" : "LIVE")} (applies on next Play).");
            }
        }

        /// <summary>
        /// One-button live ⇄ playback switch. Live → playback: TogglePlay
        /// (auto-Loads folderPath; Load frees the live cameras). Playback →
        /// live: stop playback, drop the loaded tracks + _Playback_* GOs, and
        /// respawn the live renderers via SensorManager.StartLive.
        /// </summary>
        [ContextMenu("Switch Mode (Live/Playback)")]
        public void SwitchMode()
        {
            if (CurrentState == State.Recording)
            {
                SetStatus("Switch mode: stop recording first.", warn: true);
                return;
            }
            if (IsLiveMode) TogglePlay();
            else SwitchToLive();
        }

        /// <summary>
        /// Leave playback and reconnect the live cameras: stops playback,
        /// releases every loaded track (closes RCSV streams, destroys the
        /// _Playback_* GameObjects) and re-enumerates devices.
        /// </summary>
        public void SwitchToLive()
        {
            if (CurrentState == State.Recording)
            {
                SetStatus("Switch to live: stop recording first.", warn: true);
                return;
            }
            if (CurrentState == State.Playing) StopPlayback();
            ClearTracks();

            var mgr = ResolveManager();
            if (mgr == null)
            {
                SetStatus("Switch to live: no SensorManager in scene.", warn: true);
                return;
            }
            mgr.StartLive();
            SetStatus($"Live mode: {mgr.Renderers.Count} camera(s) connected.");
        }

        private SensorManager ResolveManager()
        {
            if (cameraManager == null)
                cameraManager = FindFirstObjectByType<SensorManager>();
            return cameraManager;
        }

        [ContextMenu("Toggle Pause")]
        public void TogglePause()
        {
            if (CurrentState == State.Playing)
            {
                if (IsPaused) ResumePlayback();
                else PausePlayback();
            }
            else ToggleLiveFreeze();
        }

        /// <summary>
        /// Live-mode counterpart of the playback pause: freezes every live renderer's frame
        /// intake so the whole visual — point cloud, TSDF mesh, BT skeleton and motion
        /// curves — holds the current moment. Reuses <see cref="IsPaused"/> as the signal so
        /// every downstream pause consumer (BonePoseHistory ring hold, PointCloudMotionCurves
        /// auto-hold, SkeletonMerger staleness skip, WorkerGapMonitor suppression,
        /// TSDFHoldBeautify) behaves exactly as in a playback pause. Works while idle OR
        /// recording in live mode (the renderers keep tapping raw frames to the recorder
        /// during a hold, so REC continues across a freeze). Starting Rec or Play
        /// unfreezes first.
        /// </summary>
        public void ToggleLiveFreeze()
        {
            if (CurrentState == State.Playing || !IsLiveMode) return;
            bool freeze = !IsPaused;
            var mgr = ResolveManager();
            foreach (var r in mgr.Renderers)
                if (r != null) r.holdLiveFrame = freeze;
            IsPaused = freeze;
            SetStatus(freeze ? (CurrentState == State.Recording
                                   ? "Live frozen (REC continues) — Space resumes."
                                   : "Live frozen — Space resumes.")
                             : $"Live mode: {mgr.Renderers.Count} camera(s) connected.");
        }

        /// <summary>Seconds until a pending Space-initiated live freeze fires; 0 = none.</summary>
        public float FreezeCountdownRemaining =>
            _freezeCountdownEnd < 0.0 ? 0f : Mathf.Max(0f, (float)(_freezeCountdownEnd - Time.timeAsDouble));

        private GUIStyle _countdownStyle;

        // Big centre-screen countdown so the dancer can see the freeze coming.
        private void OnGUI()
        {
            float remain = FreezeCountdownRemaining;
            if (remain <= 0f) return;
            if (_countdownStyle == null)
                _countdownStyle = new GUIStyle
                {
                    alignment = TextAnchor.MiddleCenter,
                    fontStyle = FontStyle.Bold,
                };
            _countdownStyle.fontSize = Screen.height / 3;
            string text = Mathf.CeilToInt(remain).ToString();
            var rect = new Rect(0, 0, Screen.width, Screen.height);
            _countdownStyle.normal.textColor = new Color(0f, 0f, 0f, 0.8f);
            GUI.Label(new Rect(rect.x + 6, rect.y + 6, rect.width, rect.height), text, _countdownStyle);
            _countdownStyle.normal.textColor = Color.white;
            GUI.Label(rect, text, _countdownStyle);
        }

        // Rec/Play must consume fresh frames; drop a live freeze before either starts.
        private void UnfreezeLiveIfFrozen()
        {
            if (CurrentState != State.Idle || !IsPaused) return;
            var mgr = ResolveManager();
            if (mgr != null)
                foreach (var r in mgr.Renderers)
                    if (r != null) r.holdLiveFrame = false;
            IsPaused = false;
        }

        /// <summary>
        /// Resume frame flow on whichever transport is holding it: a playback pause
        /// resumes playback, a live freeze unfreezes. For callers that only know
        /// "IsPaused is true and frames must advance" (e.g. TSDFDebugSession) —
        /// calling ResumePlayback directly no-ops on a live freeze and would leave
        /// every renderer's holdLiveFrame stuck. No-op when nothing is held.
        /// </summary>
        public void ResumeFrames()
        {
            if (CurrentState == State.Playing) ResumePlayback();
            else UnfreezeLiveIfFrozen();
        }

        /// <summary>
        /// Hold frame flow on whichever transport is active: playing pauses playback,
        /// idle live freezes intake. No-op when already held or recording.
        /// </summary>
        public void HoldFrames()
        {
            if (CurrentState == State.Playing) PausePlayback();
            else if (!IsPaused) ToggleLiveFreeze();
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
        /// Read this serial's current depth playback cursor (0-based). Returns
        /// -1 if the serial is unknown. Used by debug tooling to show which
        /// frame of the recording each camera is currently emitting.
        /// </summary>
        public int GetPlaybackCursor(string serial)
        {
            if (_tracks == null || string.IsNullOrEmpty(serial)) return -1;
            return _tracks.TryGetValue(serial, out var t) ? t.PlaybackCursor : -1;
        }

        /// <summary>
        /// Total recorded depth frames on this serial's track (or -1 if the
        /// serial is unknown). Pair with <see cref="GetPlaybackCursor"/> to
        /// display "frame N / total" in debug HUDs.
        /// </summary>
        public int GetTrackFrameCount(string serial)
        {
            if (_tracks == null || string.IsNullOrEmpty(serial)) return -1;
            if (!_tracks.TryGetValue(serial, out var t)) return -1;
            return t.DepthFrames != null ? t.DepthFrames.Count : 0;
        }

        /// <summary>
        /// Playhead position, in SECONDS from the recording start, of a given
        /// depth frame index on one camera's track. Returns -1 if the serial or
        /// cursor is out of range. Lets time-based tooling turn a frame OFFSET
        /// (e.g. "the frame 1 step later") into the playhead time of that frame.
        /// </summary>
        public double GetFramePlayheadSeconds(string serial, int cursor)
        {
            if (_tracks == null || string.IsNullOrEmpty(serial)) return -1.0;
            if (!_tracks.TryGetValue(serial, out var t)) return -1.0;
            if (t.DepthFrames == null || cursor < 0 || cursor >= t.DepthFrames.Count) return -1.0;
            ulong ts = TimestampNsAt(t.DepthFrames, cursor);
            return (ts > _playbackTrackStartNs ? ts - _playbackTrackStartNs : 0UL) / 1_000_000_000.0;
        }

        /// <summary>
        /// Seek every track to <paramref name="targetCursor"/> (clamped to each
        /// track's [0, Count-1]) and re-emit the frame so the rest of the
        /// pipeline catches up. Auto-pauses playback like the Step* APIs so the
        /// natural auto-advance doesn't immediately stomp on the manual seek.
        /// Returns true if any track moved.
        /// </summary>
        [ContextMenu("Seek All Tracks (debug)")]
        public bool SeekAllTracksTo(int targetCursor)
        {
            if (CurrentState != State.Playing) return false;
            if (_tracks.Count == 0) return false;
            if (!IsPaused) PausePlayback();
            bool moved = false;
            ulong maxSteppedTs = 0;
            foreach (var kv in _tracks)
            {
                var track = kv.Value;
                if (track.DepthFrames.Count == 0) continue;
                int clamped = Mathf.Clamp(targetCursor, 0, track.DepthFrames.Count - 1);
                if (clamped == track.PlaybackCursor) continue;
                SetCursorAndEmit(track, clamped);
                moved = true;
                ulong ts = TimestampNsAt(track.DepthFrames, clamped);
                if (ts > maxSteppedTs) maxSteppedTs = ts;
            }
            if (moved && maxSteppedTs > 0)
            {
                // Mirror the StepCursor logic so a subsequent Resume continues
                // from the stepped playhead instead of jumping back.
                SyncWallClockTo(maxSteppedTs);
            }
            return moved;
        }

        /// <summary>
        /// Current playhead position, in SECONDS from the recording start (0 at
        /// the first frame). This is the shared master-timeline value that
        /// Update() advances every track against. Valid while Playing (frozen
        /// while paused); 0 otherwise.
        /// </summary>
        public double CurrentPlayheadSeconds =>
            CurrentState != State.Playing ? 0.0
            : (_lastPlayheadNs > _playbackTrackStartNs ? _lastPlayheadNs - _playbackTrackStartNs : 0UL)
              / 1_000_000_000.0;

        /// <summary>
        /// Playhead position (seconds from the recording start) that a recorded
        /// absolute timestamp corresponds to. Lets consumers of async results
        /// (e.g. live k4abt worker output during playback) compare a result's
        /// source-frame time against <see cref="CurrentPlayheadSeconds"/> — a
        /// result mapping AHEAD of the playhead can only be a leftover from
        /// before a loop wrap / backward seek and should be dropped.
        /// </summary>
        public double PlayheadSecondsOf(ulong tsNs) =>
            tsNs > _playbackTrackStartNs ? (tsNs - _playbackTrackStartNs) / 1e9 : 0.0;

        /// <summary>
        /// Seek to a playhead given in SECONDS from the recording start and land
        /// EVERY track on its own timestamp-matched frame (the latest frame whose
        /// recorded timestamp is &lt;= the playhead) — i.e. resolve each camera
        /// independently exactly like Update() does, instead of forcing a shared
        /// cursor index. Auto-pauses like the Step* / SeekAllTracksTo APIs.
        /// Returns true if any track moved.
        /// </summary>
        [ContextMenu("Seek To Playhead (debug)")]
        public bool SeekToPlayheadSeconds(double secondsFromStart)
        {
            if (CurrentState != State.Playing) return false;
            if (_tracks.Count == 0) return false;
            if (!IsPaused) PausePlayback();

            ulong offsetNs = secondsFromStart > 0.0
                ? (ulong)(secondsFromStart * 1_000_000_000.0) : 0UL;
            ulong playheadNs = _playbackTrackStartNs + offsetNs;

            bool moved = false;
            foreach (var kv in _tracks)
            {
                var track = kv.Value;
                if (track.DepthFrames.Count == 0) continue;
                // Latest frame whose timestamp <= playhead (clamped to frame 0
                // when the whole track starts after the playhead).
                int target = Mathf.Max(0, AdvanceCursorTo(track.DepthFrames, -1, playheadNs));
                if (target == track.PlaybackCursor) continue;
                SetCursorAndEmit(track, target);
                moved = true;
            }
            SyncWallClockTo(playheadNs);   // also stamps _lastPlayheadNs
            return moved;
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
        // Emit the frame at `cursor` for one track: advance the cursor, pick the
        // index-matched colour/IR frames, reconstruct + upload the mesh, fire the
        // playback event and feed the cumulative snapshotter. Shared by Update's
        // natural-playback advance and SetCursorAndEmit's frame stepping (3-1 dedup),
        // so the two paths are identical by construction — not by the parallel
        // maintenance the old "visually identical" comment had to promise.
        // applyRenderDelay: only natural playback delays the rendered cloud —
        // manual seek/step must show EXACTLY the cursor frame (frame inspection
        // relies on cursor == shown geometry).
        private void EmitFrameAt(DeviceTrack track, int cursor, bool applyRenderDelay = false)
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
            // Rival-projector mask: zero the flare disc in the raw depth BEFORE any
            // consumer (mesh reconstruction, TSDF, BT, event subscribers) sees it.
            if (maskRivalProjectors && depthFrame?.Bytes != null)
                ProjectorMask.Apply(track.Serial, depthFrame.Bytes, depthFrame.ByteCount,
                    track.DepthWidth, track.DepthHeight,
                    irFrame?.Bytes, irFrame?.ByteCount ?? 0, track.IRWidth, track.IRHeight);

            int delay = applyRenderDelay ? Mathf.Max(0, playbackRenderDelayFrames) : 0;
            if (delay == 0)
            {
                ReconstructAndUpload(track, depthFrame, colorFrame);
                FirePlaybackEvent(track, depthFrame, colorFrame, irFrame);
            }
            else
            {
                // event first (subscribers copy synchronously), THEN re-index for the
                // delayed render frame — the streams reuse one scratch buffer per
                // track, so the second indexer call invalidates depthFrame's bytes
                FirePlaybackEvent(track, depthFrame, colorFrame, irFrame);
                int rIdx = Mathf.Max(0, cursor - delay);
                var dDelayed = track.DepthFrames[rIdx];
                PointCloudRecording.Frame cDelayed = track.ColorFrames.Count > 0
                    ? track.ColorFrames[Mathf.Min(rIdx, track.ColorFrames.Count - 1)] : null;
                PointCloudRecording.Frame iDelayed = track.IRFrames.Count > 0
                    ? track.IRFrames[Mathf.Min(rIdx, track.IRFrames.Count - 1)] : null;
                if (maskRivalProjectors && dDelayed?.Bytes != null)
                    ProjectorMask.Apply(track.Serial, dDelayed.Bytes, dDelayed.ByteCount,
                        track.DepthWidth, track.DepthHeight,
                        iDelayed?.Bytes, iDelayed?.ByteCount ?? 0, track.IRWidth, track.IRHeight);
                ReconstructAndUpload(track, dDelayed, cDelayed);
            }
            FeedCumulative(track);
        }

        // deliver-all support: fire ONLY the playback event for a crossed
        // intermediate frame — no cursor move, no mesh reconstruction, no
        // cumulative feed. Same masking as EmitFrameAt so every consumer sees
        // the same depth bytes regardless of which path delivered the frame.
        private void FirePlaybackEventOnlyAt(DeviceTrack track, int cursor)
        {
            var depthFrame = track.DepthFrames[cursor];
            PointCloudRecording.Frame colorFrame = track.ColorFrames.Count > 0
                ? track.ColorFrames[Mathf.Min(cursor, track.ColorFrames.Count - 1)] : null;
            PointCloudRecording.Frame irFrame = track.IRFrames.Count > 0
                ? track.IRFrames[Mathf.Min(cursor, track.IRFrames.Count - 1)] : null;
            if (maskRivalProjectors && depthFrame?.Bytes != null)
                ProjectorMask.Apply(track.Serial, depthFrame.Bytes, depthFrame.ByteCount,
                    track.DepthWidth, track.DepthHeight,
                    irFrame?.Bytes, irFrame?.ByteCount ?? 0, track.IRWidth, track.IRHeight);
            FirePlaybackEvent(track, depthFrame, colorFrame, irFrame);
        }

        private void SetCursorAndEmit(DeviceTrack track, int cursor)
        {
            EmitFrameAt(track, cursor);
            // Step forward / backward should also land on the BT frame whose
            // timestamp matches this depth frame (or the closest preceding one),
            // so the skeleton overlay stays in sync with the rendered mesh while
            // the user inspects frame-by-frame.
            SyncBodyCursorToDepth(track);
            ApplyBoundingBoxFilter(track);
        }

        private void SyncBodyCursorToDepth(DeviceTrack track)
        {
            int bodyCount = track.BodyFrames.Count;
            if (bodyCount == 0) { track.BodyPlaybackCursor = -1; return; }
            ulong depthTs = TimestampNsAt(track.DepthFrames, track.PlaybackCursor);

            // Pick the latest body frame whose ts ≤ depthTs. Falls back to -1 if
            // every body frame is in the future (e.g. user stepped to the very
            // first depth frame and bodies started a few frames later).
            int target = AdvanceCursorTo(track.BodyFrames, -1, depthTs);
            if (target == track.BodyPlaybackCursor) return;
            track.BodyPlaybackCursor = target;
            if (target >= 0) FireBodyEvent(track, target);
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
            _lastPlayheadNs = playheadNs;
        }

        [ContextMenu("Rewrite metadata (yaml)")]
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
                string root = ResolvePlaybackRoot();
                string host = SafeMachineName();
                // Save = refresh sidecars for an already-loaded recording; keep its
                // own calibration rather than clobbering it with the live rig.
                WriteRecordingMetadata(root, host, liveCalibrationWins: false);
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
        //
        // liveCalibrationWins controls whose global_tr_colorCamera is baked in
        // when both a recording-local yaml AND the live manager have a value:
        //   - true  (StopRecording): the LIVE calibration wins. A fresh recording
        //     must faithfully capture the calibration that was active at record
        //     time, even when the destination folder still holds a stale
        //     extrinsics.yaml from a previous take into the same root. Without
        //     this, the first calibration ever written to a reused folder sticks
        //     forever and playback uses the wrong extrinsics.
        //   - false (Save): the EXISTING recording-local yaml wins. Re-saving
        //     metadata after a Read must not clobber the recording's own
        //     calibration with whatever the live rig currently happens to be.
        private void WriteRecordingMetadata(string root, string host, bool liveCalibrationWins)
        {
            var serials = new List<string>();
            var calibrations = new List<PointCloudRecording.DeviceCalibration>();

            // Build the per-serial extrinsics map used for the calibration
            // entries below. Sources, with precedence decided by liveCalibrationWins:
            //   - Existing recording-local yaml with non-identity values
            //     (= prior calibration session for THIS recording).
            //   - The live SensorManager's yaml (= the rig calibration
            //     Live is using right now; at record time this IS the record-time
            //     calibration).
            //   - Identity (nothing else available).
            // When liveCalibrationWins the live yaml overrides the recording-local
            // one; otherwise the recording-local one is preserved and live only
            // fills serials that have no recording-local value.
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
                    $"[{nameof(SensorRecorder)}] could not read existing extrinsics.yaml " +
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
                            bool liveIsUsable = cal.GlobalTrColorCamera.HasValue
                                && !IsIdentityExtrinsic(cal.GlobalTrColorCamera.Value);
                            if (liveCalibrationWins)
                            {
                                // Live is authoritative for any serial it reports. A
                                // usable (non-identity) value replaces a stale
                                // recording-local one; an identity value means "no
                                // transform" and must DROP any stale non-identity value
                                // so it isn't written back (otherwise a reset/uncalibrated
                                // camera would silently keep the folder's old transform).
                                if (liveIsUsable) existingGlobalBySerial[cal.Serial] = cal.GlobalTrColorCamera.Value;
                                else existingGlobalBySerial.Remove(cal.Serial);
                            }
                            else
                            {
                                // Preserve the recording-local value; live only fills
                                // serials that have no recording-local value.
                                if (existingGlobalBySerial.ContainsKey(cal.Serial)) continue;
                                if (liveIsUsable) existingGlobalBySerial[cal.Serial] = cal.GlobalTrColorCamera.Value;
                            }
                        }
                    }
                }
                catch (Exception liveEx)
                {
                    Debug.LogWarning(
                        $"[{nameof(SensorRecorder)}] could not read live manager's extrinsics.yaml: {liveEx.Message}", this);
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
                string root = ResolvePlaybackRoot();
                if (!Directory.Exists(PointCloudRecording.DatasetRoot(root)))
                {
                    SetStatus($"Read: no dataset under {root}", warn: true);
                    return;
                }

                ClearTracks();
                int totalDepth = 0, totalColor = 0, totalIR = 0, totalBodies = 0;
                foreach (var (serial, deviceDir) in PointCloudRecording.EnumerateDevices(root))
                {
                    string depthPath = Path.Combine(deviceDir, PointCloudRecording.DepthSensorName);
                    string colorPath = Path.Combine(deviceDir, PointCloudRecording.ColorSensorName);
                    string irPath = Path.Combine(deviceDir, PointCloudRecording.IRSensorName);
                    string bodiesPath = Path.Combine(deviceDir, PointCloudRecording.BodiesSensorName);
                    bool hasDepth = File.Exists(depthPath);
                    bool hasColor = File.Exists(colorPath);
                    bool hasIR = File.Exists(irPath);
                    bool hasBodies = File.Exists(bodiesPath);
                    if (!hasDepth && !hasColor && !hasIR && !hasBodies) continue;

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
                    PointCloudRecording.RcsvFrameStream bodiesStream = null;
                    try
                    {
                        if (hasDepth) depthStream = new PointCloudRecording.RcsvFrameStream(depthPath);
                        if (hasColor) colorStream = new PointCloudRecording.RcsvFrameStream(colorPath);
                        if (hasIR) irStream = new PointCloudRecording.RcsvFrameStream(irPath);
                        if (hasBodies) bodiesStream = new PointCloudRecording.RcsvFrameStream(bodiesPath);
                    }
                    catch
                    {
                        depthStream?.Dispose();
                        colorStream?.Dispose();
                        irStream?.Dispose();
                        bodiesStream?.Dispose();
                        throw;
                    }

                    var track = GetOrCreateTrack(serial);
                    track.AdoptStreams(depthStream, colorStream, irStream, bodiesStream);

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
                    if (bodiesStream != null) totalBodies += bodiesStream.Count;

                    // Fall back to live renderer dimensions if the RCSV header was
                    // missing values (older recordings). Harmless when the header
                    // already populated everything.
                    FillDimensionsFromRenderer(track);
                }

                if (totalDepth == 0 && totalColor == 0 && totalBodies == 0)
                {
                    SetStatus($"Read: no raw sensor frames found under {root}", warn: true);
                    return;
                }

                // Pull intrinsics + extrinsics from calibration/extrinsics.yaml so playback
                // can reconstruct point clouds without depending on a live renderer
                // (per Plans/issue-9-multicam-extrinsic-calibration.md → Phase 4).
                int extrinsicsApplied = LoadExtrinsicsFromYaml(root);
                RecomputeWorldRebase();

                // Free the live Femto Bolt pipelines now that the recording is on
                // disk in memory — the user pressed Read to play back, so the live
                // cameras compete for USB bandwidth and double-render the same
                // subjects. Live capture resumes via SwitchToLive (Switch Mode
                // button), which re-enumerates the devices.
                // keepLiveRenderersOnLoad: the experience flow keeps the live rig
                // running through attract playback instead.
                int liveDestroyed = 0;
                if (cameraManager != null && !keepLiveRenderersOnLoad)
                {
                    liveDestroyed = cameraManager.Renderers.Count;
                    cameraManager.DestroyAllRenderers();
                }

                SetStatus(
                    $"Loaded {totalDepth} depth / {totalColor} color / {totalIR} IR / {totalBodies} bodies frame(s) across {_tracks.Count} device(s)"
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

        /// <summary>Public entry for the experience director: re-read extrinsics
        /// and re-place playback GOs (used to apply/undo the world rebase).</summary>
        public void ReapplyExtrinsics() => RefreshExtrinsicsAndReapply();

        /// <summary>
        /// Re-read extrinsics.yaml and push the latest <c>global_tr_colorCamera</c>
        /// to any existing <c>_Playback_*</c> GameObject's transform. Called from
        /// StartPlayback so the user doesn't have to press Read after a fresh
        /// CalibrationWindow Solve & Write.
        /// </summary>
        private void RefreshExtrinsicsAndReapply()
        {
            string root = ResolvePlaybackRoot();
            int applied = LoadExtrinsicsFromYaml(root);
            if (applied == 0) return;
            RecomputeWorldRebase();
            foreach (var kv in _tracks)
            {
                var t = kv.Value;
                if (t.PlaybackObject == null) continue;
                if (!t.GlobalTrColorCamera.HasValue) continue;
                // ApplyToTransform sets localPosition/Rotation; localScale stays at
                // the (1, -1, 1) Y-flip from EnsurePlaybackObject.
                if (_worldRebaseValid)
                    Calibration.ExtrinsicsApply.ApplyToTransform(
                        t.PlaybackObject.transform, t.GlobalTrColorCamera.Value, _worldRebase);
                else
                    Calibration.ExtrinsicsApply.ApplyToTransform(
                        t.PlaybackObject.transform, t.GlobalTrColorCamera.Value);
            }
        }

        // ---- world rebase (experience flow; see Calibration.WorldFrameRebase) ----
        // Group-level decision cached per extrinsics load: either every playback GO
        // composes the same rebase Pose or none does — never a mixed rig.
        private Pose _worldRebase = Pose.identity;
        private bool _worldRebaseValid;

        private void RecomputeWorldRebase()
        {
            _worldRebase = Pose.identity;
            _worldRebaseValid = false;
            if (!applyWorldRebase) return;
            // localPosition/localRotation composition equals a world rebase only
            // under an identity parent (playback GOs sit directly under us).
            if (!Calibration.WorldFrameRebase.ParentIsIdentity(transform))
            {
                Debug.LogWarning($"[{nameof(SensorRecorder)}] world rebase skipped: this GameObject's " +
                                 "transform is not identity, so local-pose composition would not be a " +
                                 "world rebase. Reset the SensorRecorder transform.", this);
                return;
            }
            var cams = new List<(string serial, Vector3 posUnity)>(_tracks.Count);
            foreach (var kv in _tracks)
            {
                if (!kv.Value.GlobalTrColorCamera.HasValue) continue;
                Calibration.ExtrinsicsApply.ToUnityLocal(kv.Value.GlobalTrColorCamera.Value, out var pos, out _);
                cams.Add((kv.Key, pos));
            }
            if (!Calibration.WorldFrameRebase.TryComputeFromCalibrations(
                    cams, rigSerialOrder, out _worldRebase, out string reason, rebaseFloorY))
            {
                Debug.LogWarning($"[{nameof(SensorRecorder)}] world rebase skipped: {reason}", this);
                _worldRebase = Pose.identity;
                return;
            }
            _worldRebaseValid = true;
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
                    Debug.LogWarning($"[{nameof(SensorRecorder)}] extrinsics.yaml present but failed to parse: {e.Message}", this);
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
            // live SensorManager's yaml. Same "Live and Playback share
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
                    Debug.LogWarning($"[{nameof(SensorRecorder)}] live manager extrinsics fallback failed: {liveEx.Message}", this);
                }
            }

            // Register the rig with the projector mask: every camera that has both
            // intrinsics and a world extrinsic can compute where the OTHER cameras'
            // IR projectors land in its depth image.
            var rig = new List<ProjectorMask.CameraPose>();
            foreach (var kv in _tracks)
            {
                var tr = kv.Value;
                if (!tr.CameraParam.HasValue || !tr.GlobalTrColorCamera.HasValue) continue;
                rig.Add(new ProjectorMask.CameraPose
                {
                    Serial = kv.Key,
                    Param = tr.CameraParam.Value,
                    World = tr.GlobalTrColorCamera.Value,
                });
            }
            ProjectorMask.Configure(rig);

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

        // Public since the experience flow drives Rec programmatically (Explore
        // state records the visitor's take); Inspector buttons keep using ToggleRecord.
        public void StartRecording()
        {
            if (CurrentState == State.Playing) StopPlayback();
            UnfreezeLiveIfFrozen();
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
                // Each take gets its own timestamped folder so successive recordings
                // never overwrite one another. DateTime format is filesystem-safe
                // (no ':'); local time is what the operator reads on the clock.
                // The timestamp is second-precision, so a stop+start within the same
                // second would land on the SAME folder — and RcsvStreamWriter opens
                // with FileMode.Create, silently overwriting the previous take.
                // Append a numeric suffix until the candidate is unused.
                if (autoTimestampFolder)
                {
                    string stamp = DateTime.Now.ToString("yyyy-MM-dd_HH-mm-ss");
                    string candidate = Path.Combine(_recordRoot, stamp);
                    for (int i = 2; Directory.Exists(candidate); i++)
                        candidate = Path.Combine(_recordRoot, stamp + "_" + i);
                    _recordRoot = candidate;
                }
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

        public void StopRecording()
        {
            UnsubscribeAll();
            CurrentState = State.Idle;

            // Close every open RcsvStreamWriter — Dispose finalizes the index
            // chunk and patches the header. After this the files on disk are
            // self-describing and Read can load them. Snapshot frame counts
            // BEFORE dispose since the helper nulls writer refs as it goes,
            // and call the guarded helper so one stream's IO failure does not
            // strand the remaining streams (or skip _streamWriters.Clear()).
            int totalDepth = 0, totalColor = 0, totalIR = 0, totalBodies = 0;
            foreach (var sw in _streamWriters.Values)
            {
                if (sw.Depth != null)  totalDepth  += sw.Depth.FrameCount;
                if (sw.Color != null)  totalColor  += sw.Color.FrameCount;
                if (sw.IR    != null)  totalIR     += sw.IR.FrameCount;
                if (sw.Bodies != null) totalBodies += sw.Bodies.FrameCount;
            }
            FinalizeAnyOpenStreamWriters();

            // Write the metadata sidecars (extrinsics.yaml, hostinfo.yaml,
            // dataset metadata, per-device calibration). Body data is already
            // on disk via the writers above, so this is the "Save" step's
            // remaining work — wrap it here so the user doesn't have to click
            // Save separately.
            // Fresh recording: snapshot the live (= record-time) calibration so
            // playback uses the extrinsics that were active when this was captured,
            // even if the destination folder still holds a stale yaml.
            try { WriteRecordingMetadata(_recordRoot, _recordHost, liveCalibrationWins: true); }
            catch (Exception e)
            {
                Debug.LogWarning($"[{nameof(SensorRecorder)}] metadata write after StopRecording failed: {e}", this);
            }

            // _tracks still has per-track DepthFrames entries with TimestampNs
            // but Bytes=null (live capture path doesn't copy the buffer into
            // memory anymore). Playback indexes Bytes, so clear the in-memory
            // tracks here — the user clicks Read to reload the just-written
            // files into a Play-able form.
            int devCount = _tracks.Count;
            ClearTracks();
            SetStatus($"Recorded {totalDepth} depth / {totalColor} color / {totalIR} IR / {totalBodies} bodies frame(s) across {devCount} device(s) to {_recordRoot}. Click Read to load for playback.");
        }

        private void HandleRawFrame(PointCloudRenderer src, RawFrameData raw)
        {
            if (CurrentState != State.Recording) return;
            if (src == null) return;

            string serial = string.IsNullOrEmpty(src.deviceSerial) ? src.name : src.deviceSerial;
            var track = GetOrCreateTrack(serial);
            if (!track.CameraParam.HasValue) track.CameraParam = src.CameraParam;

            ulong timestampNs = raw.TimestampUs * 1000UL;

            // Diag scaffolding: callback wall-clock + mono heap delta, plus
            // per-WriteFrame timing below. Stopwatch.GetTimestamp() avoids the
            // ~40 B alloc that `new Stopwatch()` causes; mono delta tells us
            // whether allocations are happening in this callback specifically.
            RecordWindowStats rs = null;
            long callbackStartTicks = 0;
            long monoStartBytes = 0;
            if (diagnosticLogging)
            {
                if (!_diagRecordPerSerial.TryGetValue(serial, out rs))
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
                callbackStartTicks = System.Diagnostics.Stopwatch.GetTimestamp();
                monoStartBytes = UnityEngine.Profiling.Profiler.GetMonoUsedSizeLong();
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
                long wStart = rs != null ? System.Diagnostics.Stopwatch.GetTimestamp() : 0;
                sw.Depth.WriteFrame(timestampNs, raw.DepthBytes, raw.DepthByteCount);
                if (rs != null)
                {
                    long d = System.Diagnostics.Stopwatch.GetTimestamp() - wStart;
                    rs.DepthWriteTicks += d;
                    rs.DepthWriteCount++;
                    if (d > rs.DepthWriteMaxTicks) rs.DepthWriteMaxTicks = d;
                }
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
                long wStart = rs != null ? System.Diagnostics.Stopwatch.GetTimestamp() : 0;
                sw.Color.WriteFrame(timestampNs, raw.ColorBytes, raw.ColorByteCount);
                if (rs != null)
                {
                    long d = System.Diagnostics.Stopwatch.GetTimestamp() - wStart;
                    rs.ColorWriteTicks += d;
                    rs.ColorWriteCount++;
                    if (d > rs.ColorWriteMaxTicks) rs.ColorWriteMaxTicks = d;
                }
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
                long wStart = rs != null ? System.Diagnostics.Stopwatch.GetTimestamp() : 0;
                sw.IR.WriteFrame(timestampNs, raw.IRBytes, raw.IRByteCount);
                if (rs != null)
                {
                    long d = System.Diagnostics.Stopwatch.GetTimestamp() - wStart;
                    rs.IRWriteTicks += d;
                    rs.IRWriteCount++;
                    if (d > rs.IRWriteMaxTicks) rs.IRWriteMaxTicks = d;
                }
                track.AddRecordedIRFrame(new PointCloudRecording.Frame { TimestampNs = timestampNs });
                track.IRWidth = raw.IRWidth;
                track.IRHeight = raw.IRHeight;
            }

            if (rs != null)
            {
                long cbTotal = System.Diagnostics.Stopwatch.GetTimestamp() - callbackStartTicks;
                rs.CallbackTicks += cbTotal;
                if (cbTotal > rs.CallbackMaxTicks) rs.CallbackMaxTicks = cbTotal;
                long monoEndBytes = UnityEngine.Profiling.Profiler.GetMonoUsedSizeLong();
                // GC during the callback can make the delta negative; clamp to
                // 0 so the window total still reflects "alloc attributable to
                // this callback" without subtracting freed bytes.
                long delta = monoEndBytes - monoStartBytes;
                if (delta > 0) rs.CallbackMonoDeltaBytes += delta;
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

        /// <summary>
        /// Append one body-tracker frame to <c>bodies_main</c> for the given
        /// device. Called by SkeletonMerger on every K4abtWorkerHost output
        /// while the recorder is in <see cref="State.Recording"/>; no-op otherwise.
        /// <paramref name="bytes"/> + <paramref name="byteCount"/> follow the
        /// usual SDK-buffer convention — no reference is retained after this
        /// call. The bodies_main RCSV file is opened lazily on the first frame
        /// per serial so its header records the actual k4abt body record format
        /// (consistent with depth_main / color_main / ir_main lazy opening).
        /// </summary>
        public void RecordBodies(string serial, ulong tsNs, byte[] bytes, int byteCount)
        {
            if (CurrentState != State.Recording) return;
            if (string.IsNullOrEmpty(serial)) return;
            if (bytes == null || byteCount <= 0) return;

            var track = GetOrCreateTrack(serial);
            var sw = GetOrCreateStreamWriters(serial);
            if (sw.Bodies == null)
            {
                string path = PointCloudRecording.SensorFilePath(
                    _recordRoot, _recordHost, serial, PointCloudRecording.BodiesSensorName);
                string header = PointCloudRecording.BuildBodiesHeaderYaml(serial);
                sw.Bodies = new PointCloudRecording.RcsvStreamWriter(path, header);
            }
            sw.Bodies.WriteFrame(tsNs, bytes, byteCount);
            track.AddRecordedBodyFrame(new PointCloudRecording.Frame { TimestampNs = tsNs });
        }

        // --- Playback ---

        private void StartPlayback()
        {
            if (CurrentState == State.Recording) StopRecording();
            UnfreezeLiveIfFrozen();
            if (_tracks.Count == 0)
            {
                // Auto-Read: pressing Play with nothing loaded loads the configured
                // recording first, so Read is not a separate manual step.
                Load();
                if (_tracks.Count == 0)
                {
                    SetStatus("Play: nothing to play (Read found no dataset — check folderPath).", warn: true);
                    return;
                }
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
                track.BodyPlaybackCursor = -1;
                EnsurePlaybackObject(track);
            }
            if (!anyFrames)
            {
                SetStatus("Play: nothing to play (no depth frames).", warn: true);
                return;
            }
            _playbackTrackStartNs = firstTs == ulong.MaxValue ? 0 : firstTs;
            _playbackWallStart = Time.timeAsDouble;
            _lastPlayheadNs = _playbackTrackStartNs;
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

        /// <summary>Stop playback (if running) and tear the whole session down:
        /// playback GOs, reconstructors, file handles. The experience flow calls
        /// this at the attract→visitor moment; the live rig (kept alive via
        /// keepLiveRenderersOnLoad) takes over as the only source.</summary>
        public void StopAndUnload()
        {
            if (CurrentState == State.Playing) StopPlayback();
            ClearTracks();
        }

        private void Awake()
        {
            // Editor がフォーカス外でも playback の wall-clock を回し続けたい。
            // false のままだと Update が ~3 fps (dt≈333ms) に落ち、playhead
            // だけ進んで一度に 4〜5 frame の cursor advance = drop になる。
            Application.runInBackground = true;
            if (view == null) view = FindFirstObjectByType<PointCloudView>();
        }

        private void Start()
        {
            // Auto-play on entering Play mode when the camera manager is in
            // playbackOnly mode ("don't connect to live devices, play this folder
            // instead"). StartPlayback auto-Reads, so pressing Editor Play "just
            // works". Otherwise the scene starts LIVE and the operator switches to
            // playback from the Control Panel when they want it.
            var mgr = ResolveManager();
            if (mgr != null && mgr.playbackOnly)
                TogglePlay();
        }

        private void Update()
        {
            // Hot-swap: when the playback path changes while running (Inspector edit,
            // Control Panel, code), re-Read and restart playback automatically —
            // otherwise the old take keeps playing (or nothing shows) and the change
            // silently does nothing until a manual Read. Guards: while Recording, or
            // while Idle with live cameras attached, the transport is left alone so a
            // path edit can never yank a live session into playback; the next Play
            // picks the new path up via its auto-Read.
            if (Time.unscaledTime >= _nextPlaybackPathCheckTime)
            {
                _nextPlaybackPathCheckTime = Time.unscaledTime + 0.5f;
                string playbackRoot = ResolvePlaybackRoot();
                if (_watchedPlaybackRoot == null)
                {
                    _watchedPlaybackRoot = playbackRoot;
                }
                else if (!string.Equals(playbackRoot, _watchedPlaybackRoot, StringComparison.Ordinal))
                {
                    _watchedPlaybackRoot = playbackRoot;
                    bool wasPlaying = CurrentState == State.Playing;
                    if (wasPlaying || (CurrentState == State.Idle && !IsLiveMode))
                    {
                        SetStatus($"Playback path changed → reloading {playbackRoot}");
                        if (wasPlaying) StopPlayback();
                        Load();
                        if (_tracks.Count > 0) StartPlayback();
                    }
                }
            }

            // Rec toggle hotkey — works from Idle (start) or Recording (stop). Suppressed
            // during playback so it can't clobber a playback session. bodies_main is captured
            // automatically by SkeletonMerger for the duration of State.Recording.
            if (recordKey != KeyCode.None && CurrentState != State.Playing
                && Input.GetKeyDown(recordKey))
                ToggleRecord();

            // Spacebar in LIVE mode freezes/unfreezes the whole visual — the live
            // counterpart of the playback pause toggle below. Works from Idle AND
            // Recording (REC keeps writing through a freeze). Freezing goes through
            // the countdown (liveFreezeCountdownSeconds) so a dancer can hit the
            // pose; unfreezing and cancelling a pending countdown are immediate.
            // Always log the keypress with the guard state: "Space did nothing" has
            // two very different causes — a state guard rejecting it (logged here)
            // or the app not having keyboard focus (no log at all; runInBackground
            // keeps the visuals moving, so focus loss is invisible otherwise).
            if (Input.GetKeyDown(KeyCode.Space))
                Debug.Log($"[SensorRecorder] Space: state={CurrentState} live={IsLiveMode} " +
                          $"paused={IsPaused} countdown={_freezeCountdownEnd >= 0.0}");
            if (CurrentState != State.Playing && Input.GetKeyDown(KeyCode.Space))
            {
                if (_freezeCountdownEnd >= 0.0)
                {
                    _freezeCountdownEnd = -1.0;
                    SetStatus("Freeze countdown cancelled.");
                }
                else if (IsPaused || liveFreezeCountdownSeconds <= 0f)
                {
                    ToggleLiveFreeze();
                }
                else if (IsLiveMode)
                {
                    _freezeCountdownEnd = Time.timeAsDouble + liveFreezeCountdownSeconds;
                    SetStatus($"Freezing in {liveFreezeCountdownSeconds:0}s… (Space cancels)");
                }
            }
            if (_freezeCountdownEnd >= 0.0 && Time.timeAsDouble >= _freezeCountdownEnd)
            {
                _freezeCountdownEnd = -1.0;
                if (!IsPaused) ToggleLiveFreeze();
            }

            // Recording has no per-tick work, but we still want the per-second
            // diag heartbeat to log capture-side fps / drops while it's happening.
            if (CurrentState == State.Recording)
            {
                if (diagnosticLogging) PerSecondDiag();
                return;
            }
            if (CurrentState != State.Playing) return;

            // Re-emit each track's current body frame when the lead offset is tuned in
            // the Inspector — runs even while paused so the skeleton shifts live during
            // frame-by-frame inspection (BodyPlaybackCursor stays the depth-matched
            // frame; FireBodyEvent re-applies the new lead).
            if (bodyLeadFrames != _lastBodyLeadFrames)
            {
                _lastBodyLeadFrames = bodyLeadFrames;
                foreach (var kv in _tracks)
                {
                    var track = kv.Value;
                    if (track.BodyPlaybackCursor >= 0) FireBodyEvent(track, track.BodyPlaybackCursor);
                }
            }

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
            _lastPlayheadNs = playheadNs;

            bool anyRemaining = false;
            foreach (var kv in _tracks)
            {
                var track = kv.Value;
                if (track.DepthFrames.Count == 0) continue;

                int cursorBeforeAdvance = track.PlaybackCursor;
                int cursor = AdvanceCursorTo(track.DepthFrames,
                                             Mathf.Max(track.PlaybackCursor, 0), playheadNs);

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
                // Only emit on real cursor advances so a paused playhead doesn't
                // re-reconstruct / re-snapshot the same frame every Update tick.
                if (cursor != track.PlaybackCursor)
                {
                    // deliver-all: the crossed intermediates exist only as the raw
                    // event (no mesh upload) — consumers copy synchronously, the
                    // frame streams reuse one scratch buffer per track.
                    // The catch-up is CAPPED: each delivered frame costs main-thread
                    // time (disk read + subscriber copy), so an unbounded catch-up
                    // snowballs after any hitch (slower tick -> more crossed frames
                    // -> slower tick) and pins the editor at ~2fps. Beyond the cap
                    // the OLDEST crossed frames are skipped, keeping the delivered
                    // run contiguous with the rendered frame.
                    if (deliverAllPlaybackFrames && cursorBeforeAdvance >= 0)
                    {
                        const int MaxCatchUpPerTick = 8;
                        int first = cursorBeforeAdvance + 1;
                        if (cursor - first > MaxCatchUpPerTick) first = cursor - MaxCatchUpPerTick;
                        for (int i = first; i < cursor; i++)
                            FirePlaybackEventOnlyAt(track, i);
                    }
                    EmitFrameAt(track, cursor, applyRenderDelay: true); // natural playback only
                }
                AdvanceBodyCursor(track, playheadNs);
                ApplyBoundingBoxFilter(track);
            }

            if (!anyRemaining)
            {
                if (loop)
                {
                    _playbackWallStart = Time.timeAsDouble;
                    foreach (var kv in _tracks)
                    {
                        kv.Value.PlaybackCursor = -1;
                        kv.Value.BodyPlaybackCursor = -1;
                    }
                    // Snap the readable playhead to the start NOW, not at the next
                    // Update. K4abtWorkerHost pumps at execution order -100 — before
                    // our next Update recomputes _lastPlayheadNs — so for that one
                    // frame CurrentPlayheadSeconds would still read the pre-wrap end
                    // value, and SkeletonMerger's ahead-of-playhead leftover guard
                    // would wave a previous-loop worker result straight through
                    // (re-seeding the ghost this event exists to prevent).
                    _lastPlayheadNs = _playbackTrackStartNs;
                    // Notify stateful consumers (live k4abt path, merger continuity,
                    // pose-history ring) that the playhead jumped back to the start so
                    // they can flush loop-1 residue instead of leaving a ghost body.
                    try { OnPlaybackLooped?.Invoke(); }
                    catch (System.Exception e) { Debug.LogException(e, this); }
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
                double tickToMs = 1000.0 / System.Diagnostics.Stopwatch.Frequency;
                var parts = new List<string>(_diagRecordPerSerial.Count);
                var writeParts = new List<string>(_diagRecordPerSerial.Count);
                foreach (var kv in _diagRecordPerSerial)
                {
                    var rs = kv.Value;
                    double fps = rs.Frames / elapsed;
                    double mbps = rs.Bytes / 1048576.0 / elapsed;
                    parts.Add($"{PointCloudUtil.TailSerial(kv.Key, 6)}={fps:F1}fps,{mbps:F1}MB/s,gap={rs.MaxGapMs:F0}ms");

                    // Per-stream WriteFrame timing (avg + max ms). Skip a
                    // stream's segment if it didn't fire this window so the
                    // line stays scannable for the common case (e.g. IR off).
                    var seg = new System.Text.StringBuilder();
                    seg.Append(PointCloudUtil.TailSerial(kv.Key, 6)).Append(' ');
                    if (rs.DepthWriteCount > 0)
                        seg.Append($"D[avg={rs.DepthWriteTicks * tickToMs / rs.DepthWriteCount:F2}ms,max={rs.DepthWriteMaxTicks * tickToMs:F1}ms] ");
                    if (rs.ColorWriteCount > 0)
                        seg.Append($"C[avg={rs.ColorWriteTicks * tickToMs / rs.ColorWriteCount:F2}ms,max={rs.ColorWriteMaxTicks * tickToMs:F1}ms] ");
                    if (rs.IRWriteCount > 0)
                        seg.Append($"I[avg={rs.IRWriteTicks * tickToMs / rs.IRWriteCount:F2}ms,max={rs.IRWriteMaxTicks * tickToMs:F1}ms] ");
                    int cbCount = Mathf.Max(rs.DepthWriteCount, Mathf.Max(rs.ColorWriteCount, rs.IRWriteCount));
                    if (cbCount > 0)
                    {
                        seg.Append($"cb[avg={rs.CallbackTicks * tickToMs / cbCount:F2}ms,max={rs.CallbackMaxTicks * tickToMs:F1}ms,alloc={rs.CallbackMonoDeltaBytes / 1024}KB]");
                    }
                    writeParts.Add(seg.ToString().TrimEnd());
                }

                int gc0 = System.GC.CollectionCount(0) - _diagGc0Start;
                int gc1 = System.GC.CollectionCount(1) - _diagGc1Start;
                int gc2 = System.GC.CollectionCount(2) - _diagGc2Start;
                long monoNow = UnityEngine.Profiling.Profiler.GetMonoUsedSizeLong();
                long monoDeltaKB = (monoNow - _diagMonoMemStart) / 1024;

                Debug.Log($"[SensorRecorder REC] {string.Join(" | ", parts)} || " +
                          $"gc=g0:{gc0},g1:{gc1},g2:{gc2} mono+={monoDeltaKB}KB " +
                          $"maxTick={_diagMaxUpdateDtMs:F0}ms@{_diagMaxUpdateDtAtSec - _diagWindowStart:F2}s", this);
                Debug.Log($"[SensorRecorder REC writes] {string.Join(" || ", writeParts)}", this);

                foreach (var key in new List<string>(_diagRecordPerSerial.Keys))
                {
                    var rs = _diagRecordPerSerial[key];
                    rs.Frames = 0; rs.Bytes = 0; rs.MaxGapMs = 0;
                    rs.DepthWriteTicks = rs.ColorWriteTicks = rs.IRWriteTicks = 0;
                    rs.DepthWriteMaxTicks = rs.ColorWriteMaxTicks = rs.IRWriteMaxTicks = 0;
                    rs.DepthWriteCount = rs.ColorWriteCount = rs.IRWriteCount = 0;
                    rs.CallbackTicks = 0;
                    rs.CallbackMaxTicks = 0;
                    rs.CallbackMonoDeltaBytes = 0;
                }
            }

            if (CurrentState == State.Playing && _diagFiresPerSerial.Count > 0)
            {
                var parts = new List<string>(_diagFiresPerSerial.Count);
                foreach (var kv in _diagFiresPerSerial) parts.Add($"{kv.Key}={kv.Value}/s");
                int gc0 = System.GC.CollectionCount(0) - _diagGc0Start;
                int gc1 = System.GC.CollectionCount(1) - _diagGc1Start;
                int gc2 = System.GC.CollectionCount(2) - _diagGc2Start;
                long monoNow = UnityEngine.Profiling.Profiler.GetMonoUsedSizeLong();
                long monoDeltaKB = (monoNow - _diagMonoMemStart) / 1024;
                Debug.Log($"[SensorRecorder PLAY] playback_fires {string.Join(" ", parts)} || " +
                          $"gc=g0:{gc0},g1:{gc1},g2:{gc2} mono+={monoDeltaKB}KB " +
                          $"maxTick={_diagMaxUpdateDtMs:F0}ms@{_diagMaxUpdateDtAtSec - _diagWindowStart:F2}s", this);
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

        // Per-frame MPB scratch handed to PointCloudShaderFilters.Apply so the
        // playback path stays bit-for-bit identical with the live PointCloudRenderer.
        private MaterialPropertyBlock _filterMpb;

        // Shared late-bind for the scene-level filter components (3-1 dedup — this was
        // five copy-pasted Resolve* methods): cached reference -> any live
        // PointCloudRenderer that already references one (shared scene-level component
        // pattern) -> scene-wide search, so playback-only sessions still pick up the
        // same component authoring puts on the scene. The selector lambdas below are
        // non-capturing, so the compiler caches them — no per-call allocation on the
        // per-frame filter paths.
        private T ResolveShared<T>(ref T cached, System.Func<PointCloudRenderer, T> fromRenderer)
            where T : UnityEngine.Object
        {
            if (cached != null) return cached;
            foreach (var r in CollectSourceRenderers())
            {
                if (r == null) continue;
                var v = fromRenderer(r);
                if (v != null) { cached = v; return cached; }
            }
            cached = FindFirstObjectByType<T>();
            return cached;
        }

        private BoundingVolume ResolveBoundingBox() => ResolveShared(ref boundingBox, r => r.boundingBox);
        private PointCloudDecimater ResolveDecimater() => ResolveShared(ref decimater, r => r.decimater);
        private PointCloudCapsuleFilter ResolveCapsuleFilter() => ResolveShared(ref capsuleFilter, r => r.capsuleFilter);
        private PointCloudJointMotionField ResolveJointMotionField() => ResolveShared(ref jointMotionField, r => r.jointMotionField);
        private PointCloudCumulative ResolveCumulative() => ResolveShared(ref cumulative, r => r.cumulative);

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
            {
                if (_worldRebaseValid)
                    Calibration.ExtrinsicsApply.ApplyToTransform(
                        go.transform, track.GlobalTrColorCamera.Value, _worldRebase);
                else
                    Calibration.ExtrinsicsApply.ApplyToTransform(go.transform, track.GlobalTrColorCamera.Value);
            }
            var mf = go.AddComponent<MeshFilter>();
            var mr = go.AddComponent<MeshRenderer>();
            PointCloudUtil.ConfigureUnlitRenderer(mr);

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

        // Advance the body cursor to the latest body frame whose timestamp does
        // not exceed playheadNs, firing OnPlaybackBodies for each crossed frame.
        // The body cursor is independent of the depth cursor because worker
        // output can lag (and skip) depth frames; matching by ts keeps bodies in
        // step with the rendered point cloud even when the two streams are not 1:1.
        private void AdvanceBodyCursor(DeviceTrack track, ulong playheadNs)
        {
            int count = track.BodyFrames.Count;
            if (count == 0) return;

            int cursor = AdvanceCursorTo(track.BodyFrames,
                                         Mathf.Max(track.BodyPlaybackCursor, -1), playheadNs);

            if (cursor == track.BodyPlaybackCursor) return;
            // Emit only the latest frame in the crossed run — listeners (BT merge
            // pipeline) care about the freshest skeleton, not stale ones.
            // Intermediate frames are skipped on purpose; same logic as the depth
            // cursor advance above which uploads only the most recent depth+color.
            track.BodyPlaybackCursor = cursor;
            if (cursor < 0) return;
            FireBodyEvent(track, cursor);
        }

        private void FireBodyEvent(DeviceTrack track, int cursor)
        {
            if (OnPlaybackBodies == null) return;
            // Lead/lag the emitted body frame relative to the depth-matched cursor to
            // compensate the k4abt skeleton latency baked into bodies_main. The cursor
            // tracking (BodyPlaybackCursor) stays on the depth-matched frame; only the
            // frame we hand to subscribers shifts. Tracks are 1:1 so this is exact.
            int count = track.BodyFrames.Count;
            int emit = count > 0 ? Mathf.Clamp(cursor + bodyLeadFrames, 0, count - 1) : cursor;
            // Indexer materializes the bytes into the stream's reusable scratch.
            // The handler must consume the buffer synchronously — callers downstream
            // either decode into their own buffers or fire-and-forget per-frame.
            var bodyFrame = track.BodyFrames[emit];
            if (bodyFrame == null || bodyFrame.Bytes == null || bodyFrame.ByteCount <= 0) return;
            Transform t = track.PlaybackObject != null ? track.PlaybackObject.transform : null;
            // Forward the depth→color extrinsic so SkeletonMerger can map raw depth-frame
            // k4abt joints into the color frame the point cloud is reconstructed in.
            OnPlaybackBodies.Invoke(track.Serial, bodyFrame.TimestampNs,
                bodyFrame.Bytes, bodyFrame.ByteCount, t, track.CameraParam);
        }

        // Wraps the per-frame playback data in a RawFrameData and fires
        // OnPlaybackRawFrame so downstream subscribers (e.g. SkeletonMerger)
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
            int colorCount = colorFrame?.ByteCount ?? 0;
            byte[] irBytes = irFrame?.Bytes;
            int irCount = irFrame?.ByteCount ?? 0;
            var raw = new RawFrameData(
                depthBytes: depthFrame.Bytes, depthByteCount: depthFrame.ByteCount,
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
            int colorByteCount = hasColor ? colorFrame.ByteCount : 0;

            if (!track.Reconstructor.Dispatch(
                    depthFrame.Bytes, depthFrame.ByteCount, dw, dh,
                    colorBytes, colorByteCount, cw, ch,
                    track.CameraParam.Value))
            {
                Debug.LogError(
                    $"[{nameof(SensorRecorder)}] PointCloudReconstruct compute shader not found in Resources/", this);
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
            // Destroy the _Playback_<serial> GameObjects (+ dispose reconstructors
            // and file handles) here too, not only in OnDestroy. OnDisable fires on
            // play-mode exit AND on the disable half of a domain reload, so this is
            // what keeps stale playback point-cloud objects from lingering in the
            // scene between sessions.
            ClearTracks();
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
                try { sw.Depth?.Dispose();  } catch (Exception e) { Debug.LogWarning($"[{nameof(SensorRecorder)}] depth writer dispose failed: {e.Message}",  this); }
                try { sw.Color?.Dispose();  } catch (Exception e) { Debug.LogWarning($"[{nameof(SensorRecorder)}] color writer dispose failed: {e.Message}",  this); }
                try { sw.IR?.Dispose();     } catch (Exception e) { Debug.LogWarning($"[{nameof(SensorRecorder)}] IR writer dispose failed: {e.Message}",     this); }
                try { sw.Bodies?.Dispose(); } catch (Exception e) { Debug.LogWarning($"[{nameof(SensorRecorder)}] bodies writer dispose failed: {e.Message}", this); }
                sw.Depth = sw.Color = sw.IR = sw.Bodies = null;
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
                    PointCloudUtil.DestroySafe(track.PlaybackObject);
                // Close any open RcsvFrameStream handles. Skipping this would
                // leak FileStreams (12 handles per Read in a 4-cam setup) and
                // keep Windows from reopening the file for append on a
                // subsequent recording session.
                track.Dispose();
            }
            _tracks.Clear();

            // Belt-and-suspenders: a script recompile (domain reload) while in
            // play mode resets the runtime _tracks dictionary but leaves the
            // _Playback_<serial> child GameObjects alive. Those become orphans the
            // loop above can no longer reach, so a subsequent Read spawns a fresh
            // set on top of them — they accumulate and, being ordinary children of
            // this transform, dirty the scene. Sweep any lingering _Playback_*
            // children by name so nothing is ever left behind.
            for (int i = transform.childCount - 1; i >= 0; i--)
            {
                var child = transform.GetChild(i);
                if (child == null || !child.name.StartsWith("_Playback_")) continue;
                PointCloudUtil.DestroySafe(child.gameObject);
            }
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

        // Latest index whose timestamp is <= limitNs, scanning forward from `cursor`
        // (3-1 dedup — this "advance to the newest frame at or before the playhead"
        // walk existed as four inline copies: the depth advance in Update, the body
        // advance, the playhead seek, and the body-to-depth sync). Returns `cursor`
        // unchanged when no later frame qualifies, so callers get their own fallback
        // semantics by seeding -1 (may stay -1), 0, or the current cursor.
        private static int AdvanceCursorTo(IReadOnlyList<PointCloudRecording.Frame> frames,
                                           int cursor, ulong limitNs)
        {
            while (cursor + 1 < frames.Count && TimestampNsAt(frames, cursor + 1) <= limitNs)
                cursor++;
            return cursor;
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

        private string ResolveRoot() =>
            PointCloudRecording.ResolveRecordingRoot(folderPath, folderPathMacOverride);

        // Playback (Read / Play / Save-after-read) resolves from the dedicated
        // playbackFolderPath so replaying an old take never touches the recording
        // destination. Falls back to the recording root when playbackFolderPath is
        // blank, preserving the pre-split single-folder behaviour.
        private string ResolvePlaybackRoot()
        {
            if (string.IsNullOrWhiteSpace(playbackFolderPath)) return ResolveRoot();
            string macOverride = playbackFolderPathMacOverride;
#if UNITY_EDITOR_OSX || UNITY_STANDALONE_OSX
            // The playback/record path split left existing scenes with only the
            // RECORDING mac override filled, while playbackFolderPath holds a Windows
            // take path that can never exist on a Mac — playback then silently loaded
            // nothing. Fall back to the recording mac override: Macs never record
            // (no live cameras), so the "playback must not touch the recording
            // destination" concern behind the split does not apply on this platform.
            if (string.IsNullOrWhiteSpace(macOverride)) macOverride = folderPathMacOverride;
#endif
            return PointCloudRecording.ResolveRecordingRoot(playbackFolderPath, macOverride);
        }

        private static string SafeMachineName()
        {
            try { return Environment.MachineName; }
            catch { return "unknown"; }
        }

        private void SetStatus(string msg, bool warn = false)
        {
            StatusMessage = msg;
            if (warn) Debug.LogWarning($"[{nameof(SensorRecorder)}] {msg}", this);
            else Debug.Log($"[{nameof(SensorRecorder)}] {msg}", this);
        }
    }

}
