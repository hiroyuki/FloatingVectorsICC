// Real-time per-device point cloud capture + Mesh rendering for one Femto Bolt.
// One GameObject per device. PointCloudCameraManager spawns these on enumeration.

using System;
using System.Runtime.InteropServices;
using System.Threading;
using Orbbec;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using UnityEngine;
using UnityEngine.Rendering;
using Stopwatch = System.Diagnostics.Stopwatch;

namespace PointCloud
{
    /// <summary>
    /// One frame of raw pre-D2C sensor data. Byte arrays are pooled — they live only for the
    /// duration of the OnRawFramesReady callback, so consumers must copy before returning.
    ///
    /// IR fields are populated when <see cref="PointCloudRenderer.enableIRStream"/> is true
    /// (Femto Bolt delivers passive IR at the same resolution as depth, Y16). They are zero
    /// otherwise; consumers that need IR should check <see cref="IRByteCount"/> &gt; 0.
    /// </summary>
    public readonly struct RawFrameData
    {
        public readonly byte[] DepthBytes;   // Y16 (2 bytes per pixel), row-major
        public readonly int DepthByteCount;  // actual bytes of interest (DepthBytes.Length may be larger due to pooling)
        public readonly int DepthWidth;
        public readonly int DepthHeight;
        public readonly byte[] ColorBytes;   // RGB8 (3 bytes per pixel), row-major
        public readonly int ColorByteCount;
        public readonly int ColorWidth;
        public readonly int ColorHeight;
        public readonly byte[] IRBytes;      // Y16 (2 bytes per pixel), row-major. May be empty.
        public readonly int IRByteCount;
        public readonly int IRWidth;
        public readonly int IRHeight;
        public readonly ulong TimestampUs;

        public RawFrameData(byte[] depthBytes, int depthByteCount, int depthWidth, int depthHeight,
                            byte[] colorBytes, int colorByteCount, int colorWidth, int colorHeight,
                            byte[] irBytes, int irByteCount, int irWidth, int irHeight,
                            ulong timestampUs)
        {
            DepthBytes = depthBytes;
            DepthByteCount = depthByteCount;
            DepthWidth = depthWidth;
            DepthHeight = depthHeight;
            ColorBytes = colorBytes;
            ColorByteCount = colorByteCount;
            ColorWidth = colorWidth;
            ColorHeight = colorHeight;
            IRBytes = irBytes;
            IRByteCount = irByteCount;
            IRWidth = irWidth;
            IRHeight = irHeight;
            TimestampUs = timestampUs;
        }
    }

    [RequireComponent(typeof(MeshFilter))]
    [RequireComponent(typeof(MeshRenderer))]
    public class PointCloudRenderer : MonoBehaviour
    {
        [Header("Device")]
        [Tooltip("Serial number of the Femto Bolt to open. Set by PointCloudCameraManager.")]
        public string deviceSerial = string.Empty;

        [Header("Streams")]
        public uint depthWidth = 640;
        public uint depthHeight = 576;
        public uint depthFps = 30;
        public uint colorWidth = 1280;
        public uint colorHeight = 720;
        public uint colorFps = 30;

        [Tooltip("Enable the Femto Bolt's passive IR stream. k4abt body tracking uses it as the " +
                 "real IR cue instead of the depth-as-IR fallback (which makes detections stop " +
                 "after the first frame). Requires PointCloudCameraManager.syncTopology = " +
                 "Standalone — multi-device sync modes drop framesets when IR is added to the " +
                 "aggregate. Turn OFF if you only need depth + color.")]
        public bool enableIRStream = true;

        [Header("Pipeline")]
        [Tooltip("Stream that depth gets aligned TO via the Align filter. " +
                 "OB_STREAM_COLOR = depth-to-color (D2C) per CLAUDE.md. " +
                 "Note: ALIGN_D2C_SW_MODE on the pipeline config is NOT supported by the SDK; " +
                 "we always use the Align filter instead.")]
        public ObStreamType alignTargetStream = ObStreamType.Color;
        public bool enableFrameSync = true;

        [Header("Multi-device sync")]
        [Tooltip("Hardware-level sync role for this device. Set by PointCloudCameraManager based " +
                 "on its SyncTopology field (SyncHubPro -> Secondary, DaisyChain -> Primary or " +
                 "Secondary by index). Femto Bolt firmware exposes Secondary, not SecondarySynced. " +
                 "Run the 'Log Supported Sync Modes' context menu after the device is open to see " +
                 "the actual bitmap reported by your unit.")]
        public ObMultiDeviceSyncMode syncMode = ObMultiDeviceSyncMode.Secondary;
        [Tooltip("Per-trigger image-capture delay in microseconds. Stagger across devices to reduce " +
                 "iToF NIR pulse interference. The camera manager fills this with 160µs * deviceIndex.")]
        public int trigger2ImageDelayUs = 0;
        [Tooltip("Calls ob_device_enable_global_timestamp(true) and uses depth.GlobalTimestampUs " +
                 "as the recording timestamp (host-clock domain in microseconds). Falls back to " +
                 "SystemTimestampUs (host wall clock) when the device doesn't support it.")]
        public bool enableGlobalTimestamp = true;
        [Tooltip("Calls ob_device_timer_sync_with_host before starting the pipeline so this device's " +
                 "frame timestamps share a host-time reference.")]
        public bool timerSyncWithHost = true;
        [Tooltip("If > 0, re-runs ob_device_timer_sync_with_host every N seconds during capture " +
                 "to limit drift. The Orbbec docs recommend 60 minutes; we default to 60 seconds " +
                 "matching the official multi-device sample's enableDeviceClockSync(60000).")]
        [Min(0)]
        public float timerSyncIntervalSec = 60f;

        [Header("Depth work mode")]
        [Tooltip("Name of the depth work mode to switch to before starting the pipeline. " +
                 "Leave empty to keep the device's current mode. Must match a name reported by " +
                 "ob_device_get_depth_work_mode_list for this device/firmware. Use the " +
                 "'Log Depth Work Modes' context menu at runtime to list the available names.")]
        public string depthWorkMode = string.Empty;

        [Header("Point cloud")]
        [Tooltip("Upper bound used to size the GPU vertex buffer. Frames with more points are clipped. " +
                 "Default = depth resolution (640x576 = 368640 pts) to keep the per-frame upload " +
                 "under D3D12's 16 MiB staging buffer (24 B/vertex × 920K verts at color resolution " +
                 "exceeds it and stalls the upload, especially when DirectML BT is also active). " +
                 "Set to color resolution (1280×720) only if you have headroom and need denser points.")]
        public int maxPoints = 640 * 576;
        [Tooltip("Material used for the points. Use Orbbec/PointCloudUnlit or compatible.")]
        public Material pointMaterial;
        [Tooltip("Negate Y on the GameObject's transform. The SDK emits points in image coordinates " +
                 "(Y points down); flipping Y maps them to Unity's Y-up convention.")]
        public bool flipY = true;

        [Header("Bounding box filter")]
        [Tooltip("Optional oriented bounding box. When assigned and its filterMode is not Disabled, " +
                 "points falling outside/inside the box (per mode) are culled in the vertex shader.")]
        public PointCloudBoundingBox boundingBox;

        [Header("Decimater")]
        [Tooltip("Optional random decimater. When assigned and its reductionPercent is > 0, " +
                 "each point is independently dropped in the vertex shader via a per-vertex hash.")]
        public PointCloudDecimater decimater;

        [Header("Cumulative")]
        [Tooltip("Optional cumulative snapshotter. When assigned and its No Erase toggle is on, " +
                 "snapshots are captured every 'interval' frames and kept visible until cleared.")]
        public PointCloudCumulative cumulative;

        [Header("Diagnostics")]
        [Tooltip("When on, logs captured / consumed / dropped fps once per second to the Unity console. " +
                 "captured = SDK frames published by the capture thread, consumed = frames uploaded to the " +
                 "Mesh on the main thread, dropped = publish-but-unread frames (capture outpacing Update).")]
        public bool logFps = false;

        /// <summary>
        /// Fires each frame after the point cloud mesh has been updated. Subscribers receive the
        /// raw SDK point buffer in renderer-local space (OBB / decimation filters run in the
        /// vertex shader, not on this buffer), the valid point count, and the SDK frame timestamp
        /// (microseconds). The buffer is reused next frame, so consumers must copy any data they
        /// need to retain before returning from the handler.
        /// </summary>
        public event Action<PointCloudRenderer, NativeArray<ObColorPoint>, int, ulong> OnFrameUploaded;

        /// <summary>
        /// Fires each frame alongside <see cref="OnFrameUploaded"/> with the raw pre-D2C sensor
        /// data: Y16 depth + RGB color bytes. The byte arrays live inside the slot pool and are
        /// reused next frame — consumers must copy any bytes they want to keep before returning.
        /// Depth and Color refer to the same capture moment on Femto Bolt when frame sync is
        /// enabled.
        /// </summary>
        public event Action<PointCloudRenderer, RawFrameData> OnRawFramesReady;

        /// <summary>
        /// Camera parameters fetched from the pipeline after it starts: depth/color intrinsics +
        /// distortion + depth-to-color extrinsic (millimeters). Null until pipeline has started.
        /// Values reflect the state at the moment the pipeline was started; changing depth work
        /// mode or stream profiles during capture will not update this snapshot.
        /// </summary>
        public ObCameraParam? CameraParam { get; private set; }

        // --- Native ---
        private OrbbecDevice _device;
        private OrbbecPipeline _pipeline;
        private OrbbecConfig _config;
        private OrbbecFilter _alignFilter;
        private OrbbecFilter _pointCloudFilter;

        // --- Threading ---
        private Thread _captureThread;
        private CancellationTokenSource _cts;
        private Exception _captureError;

        // --- Slot pool (capture thread fills, main thread consumes) ---
        private SlotPool _slots;

        // --- Rendering ---
        private MeshFilter _meshFilter;
        private MeshRenderer _meshRenderer;
        private Mesh _mesh;

        // --- Public state ---
        public bool IsCapturing => _captureThread != null && _captureThread.IsAlive;
        public int LastPointCount { get; private set; }
        public ulong LastTimestampUs { get; private set; }
        public ulong FramesDropped { get; private set; }
        /// <summary>
        /// True when the device confirmed global-timestamp support and EnableGlobalTimestamp(true)
        /// succeeded; capture loop reads frame.GlobalTimestampUs in that case.
        /// </summary>
        public bool GlobalTimestampActive { get; private set; }

        // Periodic timer sync.
        private Coroutine _timerSyncCoro;

        // --- FPS counters ---
        private long _capturedCount;          // capture thread: Interlocked.Increment on Publish
        private long _lastCapturedSnapshot;   // main thread only
        private ulong _lastDroppedSnapshot;   // main thread only
        private int _consumedCount;           // main thread only
        private float _fpsWindowStart;        // Time.realtimeSinceStartup, 0 = not started

        // --- Per-stage timers (main thread only, accumulated over fps window) ---
        private readonly Stopwatch _stageSw = new Stopwatch();
        private long _shaderTicks;
        private long _cumulTicks;
        private long _meshTicks;

        private void Awake()
        {
            _meshFilter = GetComponent<MeshFilter>();
            _meshRenderer = GetComponent<MeshRenderer>();
        }

        private void Start()
        {
            if (string.IsNullOrEmpty(deviceSerial))
            {
                Debug.LogError($"[{nameof(PointCloudRenderer)}] deviceSerial is empty.", this);
                return;
            }

            try
            {
                OpenDevice();
                BuildMesh();
                ApplyAxisConvention();
                StartCaptureThread();
                if (timerSyncWithHost && timerSyncIntervalSec > 0f)
                    _timerSyncCoro = StartCoroutine(PeriodicTimerSync());
            }
            catch (Exception e)
            {
                Debug.LogException(e, this);
                ShutdownCapture();
            }
        }

        private void Update()
        {
            // Surface any background error to the Unity console once.
            var err = Interlocked.Exchange(ref _captureError, null);
            if (err != null)
            {
                Debug.LogException(err, this);
            }

            // Filter parameters can change every frame (user dragging the bbox, tweaking decim).
            // Push them via MaterialPropertyBlock so the shared material stays untouched —
            // Cumulative snapshot GOs therefore render with the shader defaults (filters off).
            _stageSw.Restart();
            UpdateShaderFilterProperties();
            _stageSw.Stop();
            if (logFps) _shaderTicks += _stageSw.ElapsedTicks;

            var slot = _slots?.TryAcquireRead();
            if (slot != null)
            {
                try
                {
                    int n = Math.Min(slot.PointCount, maxPoints);

                    _stageSw.Restart();
                    cumulative?.OnFrame(slot.Buffer, n, transform, pointMaterial);
                    _stageSw.Stop();
                    if (logFps) _cumulTicks += _stageSw.ElapsedTicks;

                    _stageSw.Restart();
                    _mesh.SetVertexBufferData(slot.Buffer, 0, 0, n,
                        flags: MeshUpdateFlags.DontRecalculateBounds | MeshUpdateFlags.DontValidateIndices);
                    _mesh.SetSubMesh(0, new SubMeshDescriptor(0, n, MeshTopology.Points),
                        MeshUpdateFlags.DontRecalculateBounds | MeshUpdateFlags.DontValidateIndices);
                    _stageSw.Stop();
                    if (logFps) _meshTicks += _stageSw.ElapsedTicks;

                    LastPointCount = n;
                    LastTimestampUs = slot.TimestampUs;
                    _consumedCount++;
                    OnFrameUploaded?.Invoke(this, slot.Buffer, n, slot.TimestampUs);

                    if (OnRawFramesReady != null && slot.DepthByteCount > 0 && slot.ColorByteCount > 0)
                    {
                        OnRawFramesReady.Invoke(this, new RawFrameData(
                            depthBytes: slot.DepthBytes,
                            depthByteCount: slot.DepthByteCount,
                            depthWidth: slot.DepthWidth,
                            depthHeight: slot.DepthHeight,
                            colorBytes: slot.ColorBytes,
                            colorByteCount: slot.ColorByteCount,
                            colorWidth: slot.ColorWidth,
                            colorHeight: slot.ColorHeight,
                            irBytes: slot.IRBytes ?? Array.Empty<byte>(),
                            irByteCount: slot.IRByteCount,
                            irWidth: slot.IRWidth,
                            irHeight: slot.IRHeight,
                            timestampUs: slot.TimestampUs));
                    }
                }
                finally
                {
                    _slots.ReturnRead(slot);
                }
            }

            UpdateFpsDiagnostics();
        }

        private static readonly int _ObbObjToBoxId = Shader.PropertyToID("_ObbObjToBox");
        private static readonly int _ObbModeId     = Shader.PropertyToID("_ObbMode");
        private static readonly int _DecimKeepId   = Shader.PropertyToID("_DecimKeep");
        private static readonly int _DecimFrameId  = Shader.PropertyToID("_DecimFrame");
        private MaterialPropertyBlock _mpb;

        private void UpdateShaderFilterProperties()
        {
            if (_mpb == null) _mpb = new MaterialPropertyBlock();
            _meshRenderer.GetPropertyBlock(_mpb);

            // OBB: shader expects renderer-object-space -> box-local matrix.
            float obbMode = 0f;
            if (boundingBox != null)
            {
                var mode = boundingBox.Mode;
                if (mode != PointCloudBoundingBox.FilterMode.Disabled)
                {
                    obbMode = mode == PointCloudBoundingBox.FilterMode.KeepInside ? 1f : 2f;
                    var m = boundingBox.transform.worldToLocalMatrix * transform.localToWorldMatrix;
                    _mpb.SetMatrix(_ObbObjToBoxId, m);
                }
            }
            _mpb.SetFloat(_ObbModeId, obbMode);

            // Decimation: reuse the same KeepRatio the CPU path used; Enabled=false -> keep all.
            float decimKeep = 1f;
            if (decimater != null && decimater.Enabled) decimKeep = Mathf.Clamp01(decimater.KeepRatio);
            _mpb.SetFloat(_DecimKeepId, decimKeep);
            _mpb.SetFloat(_DecimFrameId, Time.frameCount);

            _meshRenderer.SetPropertyBlock(_mpb);
        }

        private void UpdateFpsDiagnostics()
        {
            if (!logFps)
            {
                // Reset window so the first sample after re-enabling isn't inflated by accumulated counts.
                if (_fpsWindowStart != 0f)
                {
                    _fpsWindowStart = 0f;
                    _consumedCount = 0;
                    _shaderTicks = _cumulTicks = _meshTicks = 0;
                    _lastCapturedSnapshot = Interlocked.Read(ref _capturedCount);
                    _lastDroppedSnapshot = FramesDropped;
                }
                return;
            }

            float now = Time.realtimeSinceStartup;
            if (_fpsWindowStart == 0f)
            {
                _fpsWindowStart = now;
                _lastCapturedSnapshot = Interlocked.Read(ref _capturedCount);
                _lastDroppedSnapshot = FramesDropped;
                _consumedCount = 0;
                _shaderTicks = _cumulTicks = _meshTicks = 0;
                return;
            }

            float elapsed = now - _fpsWindowStart;
            if (elapsed < 1f) return;

            long captured = Interlocked.Read(ref _capturedCount);
            ulong dropped = FramesDropped;
            float capturedFps = (captured - _lastCapturedSnapshot) / elapsed;
            float consumedFps = _consumedCount / elapsed;
            float droppedFps = (dropped - _lastDroppedSnapshot) / elapsed;

            // Average per-consumed-frame stage time in ms. Ticks -> ms: 1000 * ticks / Stopwatch.Frequency.
            int framesForAvg = _consumedCount > 0 ? _consumedCount : 1;
            double tickToMs = 1000.0 / Stopwatch.Frequency;
            double shaderMs = _shaderTicks * tickToMs / framesForAvg;
            double cumulMs  = _cumulTicks  * tickToMs / framesForAvg;
            double meshMs   = _meshTicks   * tickToMs / framesForAvg;

            Debug.Log(
                $"[PCR {Truncate(deviceSerial, 6)}] captured={capturedFps:F1} " +
                $"consumed={consumedFps:F1} dropped={droppedFps:F1} (/s) | " +
                $"per-frame avg ms: shader={shaderMs:F2} cumul={cumulMs:F2} " +
                $"mesh={meshMs:F2} pts={LastPointCount}",
                this);

            _lastCapturedSnapshot = captured;
            _lastDroppedSnapshot = dropped;
            _consumedCount = 0;
            _shaderTicks = _cumulTicks = _meshTicks = 0;
            _fpsWindowStart = now;
        }

        private void OnDestroy()
        {
            ShutdownCapture();
            if (_mesh != null)
            {
                Destroy(_mesh);
                _mesh = null;
            }
            _slots?.Dispose();
            _slots = null;
        }

        // --- Setup ---

        private void OpenDevice()
        {
            var ctx = OrbbecRuntime.Context;
            _device = ctx.OpenDeviceBySerial(deviceSerial);

            // Depth work mode must be switched while streams are stopped — always safe here
            // because the pipeline isn't started yet. Doing it before ApplySyncConfig so the
            // sync settings apply to the mode we'll actually stream in.
            ApplyDepthWorkMode();

            // Multi-device sync and timer sync must happen after the device is open but
            // before the pipeline starts, otherwise the sync-mode switch can race the stream.
            ApplySyncConfig();
            if (timerSyncWithHost)
            {
                try { _device.TimerSyncWithHost(); }
                catch (Exception e)
                {
                    Debug.LogWarning(
                        $"[{nameof(PointCloudRenderer)}] TimerSyncWithHost failed on {deviceSerial}: {e.Message}",
                        this);
                }
            }

            // Global timestamp must be enabled before stream start so the first frame already
            // carries a host-domain GlobalTimestampUs value.
            GlobalTimestampActive = false;
            if (enableGlobalTimestamp)
            {
                try
                {
                    if (_device.IsGlobalTimestampSupported())
                    {
                        _device.EnableGlobalTimestamp(true);
                        GlobalTimestampActive = true;
                    }
                    else
                    {
                        Debug.LogWarning(
                            $"[{nameof(PointCloudRenderer)}] {deviceSerial}: device reports global " +
                            "timestamp NOT supported; falling back to frame.SystemTimestampUs " +
                            "(host wall clock at frame arrival).", this);
                    }
                }
                catch (Exception e)
                {
                    Debug.LogWarning(
                        $"[{nameof(PointCloudRenderer)}] EnableGlobalTimestamp failed on {deviceSerial}: " +
                        $"{e.Message}. Falling back to SystemTimestampUs.", this);
                }
            }

            _config = new OrbbecConfig();
            _config.EnableVideoStream(ObStreamType.Depth, depthWidth, depthHeight, depthFps, ObFormat.Y16);
            _config.EnableVideoStream(ObStreamType.Color, colorWidth, colorHeight, colorFps, ObFormat.RGB);
            if (enableIRStream)
            {
                // Femto Bolt's passive IR stream matches depth (same dimensions, Y16, same fps).
                _config.EnableVideoStream(ObStreamType.IR, depthWidth, depthHeight, depthFps, ObFormat.Y16);
            }
            // Pipeline-level align supports only ALIGN_D2C_HW_MODE; we use the Align filter for SW D2C.
            _config.SetAlignMode(ObAlignMode.Disable);
            // When IR is enabled the aggregate mode has to drop to ColorFrameRequire,
            // otherwise the pipeline stalls waiting for IR + depth + color to all
            // arrive in the same frameset and never publishes anything (point cloud
            // disappears). With ColorFrameRequire the frameset still fires per color
            // frame and depth/IR are delivered when available.
            _config.SetFrameAggregateOutputMode(enableIRStream
                ? ObFrameAggregateOutputMode.ColorFrameRequire
                : ObFrameAggregateOutputMode.AllTypeFrameRequire);

            _pipeline = _device.CreatePipeline();
            if (enableFrameSync) _pipeline.EnableFrameSync();
            _pipeline.Start(_config);

            // Snapshot camera parameters now that the streams are configured. Recorder writes
            // these into extrinsics.yaml / sensor intrinsics so playback can reconstruct points.
            try { CameraParam = _pipeline.GetCameraParam(); }
            catch (Exception e)
            {
                Debug.LogWarning(
                    $"[{nameof(PointCloudRenderer)}] GetCameraParam failed on {deviceSerial}: {e.Message}. " +
                    "Raw-data recordings from this device will be missing intrinsics.", this);
            }

            // Align: depth gets aligned to the target stream (default = Color, i.e. D2C).
            _alignFilter = new OrbbecFilter("Align");
            _alignFilter.SetConfigValue("AlignType", (double)alignTargetStream);

            _pointCloudFilter = new OrbbecFilter("PointCloudFilter");
            _pointCloudFilter.SetConfigValue("pointFormat", (double)ObFormat.RGBPoint);
            _pointCloudFilter.SetConfigValue("coordinateDataScale", 0.001); // mm -> m
            _pointCloudFilter.SetConfigValue("colorDataNormalization", 1.0); // 0-255 -> 0-1
        }

        private void BuildMesh()
        {
            _mesh = new Mesh
            {
                name = $"PointCloud_{deviceSerial}",
                indexFormat = IndexFormat.UInt32,
                bounds = new Bounds(Vector3.zero, Vector3.one * 100f),
            };

            // Vertex layout = OBColorPoint (position float3 + color float3, 24 bytes).
            var attrs = new[]
            {
                new VertexAttributeDescriptor(VertexAttribute.Position, VertexAttributeFormat.Float32, 3, stream: 0),
                new VertexAttributeDescriptor(VertexAttribute.Color,    VertexAttributeFormat.Float32, 3, stream: 0),
            };
            _mesh.SetVertexBufferParams(maxPoints, attrs);

            // Identity index buffer for MeshTopology.Points.
            // (try/finally instead of `using` because `using` makes the variable
            //  readonly, which blocks NativeArray's indexer set on a struct.)
            _mesh.SetIndexBufferParams(maxPoints, IndexFormat.UInt32);
            var indices = new NativeArray<uint>(maxPoints, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
            try
            {
                for (int i = 0; i < maxPoints; i++) indices[i] = (uint)i;
                _mesh.SetIndexBufferData(indices, 0, 0, maxPoints,
                    MeshUpdateFlags.DontRecalculateBounds | MeshUpdateFlags.DontValidateIndices);
            }
            finally
            {
                indices.Dispose();
            }
            _mesh.SetSubMesh(0, new SubMeshDescriptor(0, 0, MeshTopology.Points),
                MeshUpdateFlags.DontRecalculateBounds | MeshUpdateFlags.DontValidateIndices);

            _meshFilter.sharedMesh = _mesh;
            if (pointMaterial == null)
            {
                var shader = Shader.Find("Orbbec/PointCloudUnlit");
                if (shader != null)
                {
                    pointMaterial = new Material(shader) { name = "PointCloudUnlit (auto)" };
                }
                else
                {
                    Debug.LogWarning(
                        $"[{nameof(PointCloudRenderer)}] Orbbec/PointCloudUnlit shader not found. " +
                        "Assign a vertex-color material to pointMaterial.", this);
                }
            }
            if (pointMaterial != null) _meshRenderer.sharedMaterial = pointMaterial;

            // Oversize raw buffers to the configured stream resolution. Y16 = 2 B/pixel,
            // RGB = 3 B/pixel. If the SDK actually delivers smaller frames we record only
            // the filled portion; if it delivers larger we log and skip the raw path.
            int depthRaw = (int)(depthWidth * depthHeight) * 2;
            int colorRaw = (int)(colorWidth * colorHeight) * 3;
            int irRaw = enableIRStream ? (int)(depthWidth * depthHeight) * 2 : 0; // IR matches depth dims
            _slots = new SlotPool(slotCount: 3, capacity: maxPoints,
                                  depthRawByteCapacity: depthRaw,
                                  colorRawByteCapacity: colorRaw,
                                  irRawByteCapacity: irRaw);
        }

        private void ApplyAxisConvention()
        {
            if (!flipY) return;
            var s = transform.localScale;
            s.y = -Mathf.Abs(s.y);
            transform.localScale = s;
        }

        private void ApplyDepthWorkMode()
        {
            if (string.IsNullOrWhiteSpace(depthWorkMode)) return;

            string current;
            try { current = _device.GetCurrentDepthWorkModeName(); }
            catch (Exception e)
            {
                Debug.LogWarning(
                    $"[{nameof(PointCloudRenderer)}] GetCurrentDepthWorkModeName failed on {deviceSerial}: {e.Message}. " +
                    $"Skipping depth mode switch.", this);
                return;
            }

            if (string.Equals(current, depthWorkMode, StringComparison.Ordinal)) return;

            try
            {
                _device.SwitchDepthWorkModeByName(depthWorkMode);
                Debug.Log(
                    $"[{nameof(PointCloudRenderer)}] {deviceSerial}: depth work mode \"{current}\" -> \"{depthWorkMode}\".",
                    this);
            }
            catch (Exception e)
            {
                Debug.LogWarning(
                    $"[{nameof(PointCloudRenderer)}] SwitchDepthWorkModeByName(\"{depthWorkMode}\") failed on " +
                    $"{deviceSerial}: {e.Message}. Keeping current mode \"{current}\".", this);
            }
        }

        [ContextMenu("Log Supported Sync Modes")]
        private void LogSupportedSyncModes()
        {
            if (_device == null)
            {
                Debug.LogWarning($"[{nameof(PointCloudRenderer)}] device not open yet.", this);
                return;
            }
            try
            {
                ushort bitmap = _device.GetSupportedSyncModeBitmap();
                var sb = new System.Text.StringBuilder();
                foreach (ObMultiDeviceSyncMode mode in Enum.GetValues(typeof(ObMultiDeviceSyncMode)))
                {
                    if ((bitmap & (ushort)mode) != 0)
                    {
                        if (sb.Length > 0) sb.Append(", ");
                        sb.Append(mode);
                    }
                }
                Debug.Log(
                    $"[{nameof(PointCloudRenderer)}] {deviceSerial} supported sync modes " +
                    $"(bitmap=0x{bitmap:X4}): {sb}",
                    this);
            }
            catch (Exception e)
            {
                Debug.LogException(e, this);
            }
        }

        [ContextMenu("Log Depth Work Modes")]
        private void LogDepthWorkModes()
        {
            if (_device == null)
            {
                Debug.LogWarning($"[{nameof(PointCloudRenderer)}] device not open yet.", this);
                return;
            }
            try
            {
                var names = _device.GetDepthWorkModeNames();
                string current = _device.GetCurrentDepthWorkModeName();
                Debug.Log(
                    $"[{nameof(PointCloudRenderer)}] {deviceSerial} depth modes (current=\"{current}\"): " +
                    string.Join(", ", names),
                    this);
            }
            catch (Exception e)
            {
                Debug.LogException(e, this);
            }
        }

        private void ApplySyncConfig()
        {
            ushort supported = _device.GetSupportedSyncModeBitmap();
            if ((supported & (ushort)syncMode) == 0)
            {
                // Silent fallback hides synchronisation failures (you'll think the cameras are
                // synced but every device is actually free-running). Throw so the caller has to
                // pick a supported mode explicitly via the Inspector.
                throw new InvalidOperationException(
                    $"Device {deviceSerial} does not support sync mode {syncMode} " +
                    $"(supported bitmap = 0x{supported:X4}). " +
                    "Choose a supported mode in the PointCloudCameraManager / PointCloudRenderer Inspector.");
            }

            // PRIMARY emits the pulse, SECONDARY forwards it down a daisy chain. For Sync Hub Pro
            // fan-out the secondary's VSYNC_OUT is unused, but enabling it is harmless. Only
            // disable when the device has nothing to broadcast (HardwareTriggering / Standalone /
            // FreeRun / SoftwareTriggering modes that don't normally drive a downstream camera).
            bool triggerOut = syncMode == ObMultiDeviceSyncMode.Primary
                           || syncMode == ObMultiDeviceSyncMode.Secondary
                           || syncMode == ObMultiDeviceSyncMode.SecondarySynced;

            var config = new ObMultiDeviceSyncConfig
            {
                SyncMode             = syncMode,
                DepthDelayUs         = 0,
                ColorDelayUs         = 0,
                Trigger2ImageDelayUs = trigger2ImageDelayUs,
                TriggerOutEnable     = triggerOut,
                TriggerOutDelayUs    = 0,
                FramesPerTrigger     = 1,
            };
            _device.SetSyncConfig(config);
        }

        // --- Capture thread ---

        private void StartCaptureThread()
        {
            _cts = new CancellationTokenSource();
            var token = _cts.Token;
            _captureThread = new Thread(() => CaptureLoop(token))
            {
                IsBackground = true,
                Name = $"OrbbecCapture-{Truncate(deviceSerial, 6)}",
            };
            _captureThread.Start();
        }

        private void CaptureLoop(CancellationToken token)
        {
            try
            {
                while (!token.IsCancellationRequested)
                {
                    using var frameset = _pipeline.WaitForFrameset(100);
                    if (frameset == null) continue;

                    // Pull the recording timestamp from the raw depth frame, not from the
                    // PointCloudFilter output. PointCloudFilter is a synthetic frame whose
                    // timestamp semantics aren't documented; depth is the real sensor frame.
                    // Prefer the global (host-domain) timestamp; fall back to the system
                    // timestamp (host wall clock at frame arrival) if global isn't active.
                    ulong frameTimestampUs;
                    using (var depthFrame = frameset.GetDepthFrame())
                    {
                        if (depthFrame == null) continue;
                        frameTimestampUs = GlobalTimestampActive
                            ? depthFrame.GlobalTimestampUs
                            : depthFrame.SystemTimestampUs;
                    }

                    using var aligned = _alignFilter.Process(frameset);
                    if (aligned == null) continue;

                    using var points = _pointCloudFilter.Process(aligned);
                    if (points == null) continue;

                    if (points.Format != ObFormat.RGBPoint) continue;

                    int pointCount = (int)(points.DataSize / (uint)UnsafeUtility.SizeOf<ObColorPoint>());
                    if (pointCount <= 0) continue;
                    if (pointCount > maxPoints) pointCount = maxPoints;

                    var slot = _slots.AcquireWrite();
                    if (slot == null) continue;

                    unsafe
                    {
                        UnsafeUtility.MemCpy(
                            NativeArrayUnsafeUtility.GetUnsafePtr(slot.Buffer),
                            (void*)points.DataPointer,
                            (long)pointCount * UnsafeUtility.SizeOf<ObColorPoint>());
                    }
                    slot.PointCount = pointCount;
                    slot.TimestampUs = frameTimestampUs;

                    // Raw pre-D2C depth + color — pulled off the original frameset (the align
                    // filter produces a new frameset, it doesn't mutate this one).
                    CopyRawStream(frameset.GetDepthFrame, slot.DepthBytes,
                                  out slot.DepthByteCount, out slot.DepthWidth, out slot.DepthHeight);
                    CopyRawStream(frameset.GetColorFrame, slot.ColorBytes,
                                  out slot.ColorByteCount, out slot.ColorWidth, out slot.ColorHeight);
                    if (enableIRStream)
                    {
                        CopyRawStream(frameset.GetIRFrame, slot.IRBytes,
                                      out slot.IRByteCount, out slot.IRWidth, out slot.IRHeight);
                    }
                    else
                    {
                        slot.IRByteCount = 0; slot.IRWidth = 0; slot.IRHeight = 0;
                    }

                    if (_slots.Publish(slot)) FramesDropped++;
                    Interlocked.Increment(ref _capturedCount);
                }
            }
            catch (Exception e)
            {
                Interlocked.Exchange(ref _captureError, e);
            }
        }

        private void CopyRawStream(
            Func<OrbbecFrame> getFrame,
            byte[] dest,
            out int byteCount,
            out int width,
            out int height)
        {
            byteCount = 0;
            width = 0;
            height = 0;
            if (dest == null || dest.Length == 0) return;

            OrbbecFrame frame = null;
            try
            {
                frame = getFrame();
                if (frame == null) return;

                int size = (int)frame.DataSize;
                if (size <= 0 || size > dest.Length) return;

                Marshal.Copy(frame.DataPointer, dest, 0, size);
                byteCount = size;
                width = (int)frame.VideoWidth;
                height = (int)frame.VideoHeight;
            }
            catch (Exception)
            {
                byteCount = 0;
                width = 0;
                height = 0;
            }
            finally
            {
                frame?.Dispose();
            }
        }

        private void ShutdownCapture()
        {
            if (_timerSyncCoro != null)
            {
                StopCoroutine(_timerSyncCoro);
                _timerSyncCoro = null;
            }
            _cts?.Cancel();
            if (_captureThread != null)
            {
                if (!_captureThread.Join(2000))
                    Debug.LogWarning($"[{nameof(PointCloudRenderer)}] capture thread did not exit cleanly.", this);
                _captureThread = null;
            }
            _cts?.Dispose();
            _cts = null;

            _pointCloudFilter?.Dispose(); _pointCloudFilter = null;
            _alignFilter?.Dispose(); _alignFilter = null;
            _pipeline?.Dispose(); _pipeline = null;
            _config?.Dispose(); _config = null;
            _device?.Dispose(); _device = null;
        }

        private System.Collections.IEnumerator PeriodicTimerSync()
        {
            // Note: TimerSyncWithHost can cause a one-frame timestamp jump per the SDK doc, but
            // long-running captures drift several ms across devices without periodic re-sync.
            var wait = new WaitForSecondsRealtime(timerSyncIntervalSec);
            while (true)
            {
                yield return wait;
                if (_device == null) yield break;
                try { _device.TimerSyncWithHost(); }
                catch (Exception e)
                {
                    Debug.LogWarning(
                        $"[{nameof(PointCloudRenderer)}] periodic TimerSyncWithHost failed on " +
                        $"{deviceSerial}: {e.Message}", this);
                }
            }
        }

        private static string Truncate(string s, int n) =>
            string.IsNullOrEmpty(s) || s.Length <= n ? s : s.Substring(s.Length - n);
    }

    /// <summary>
    /// Triple-slot frame pool. Capture thread writes the latest frame into a free slot
    /// and publishes it; the main thread atomically swaps it out and uploads to the Mesh.
    /// </summary>
    internal sealed class SlotPool : IDisposable
    {
        public sealed class Slot
        {
            public NativeArray<ObColorPoint> Buffer;
            public int PointCount;
            public ulong TimestampUs;

            // Raw pre-D2C frames captured alongside the point cloud. Byte arrays are
            // oversized (allocated once for the configured resolution) and reused.
            public byte[] DepthBytes;
            public int DepthByteCount;
            public int DepthWidth;
            public int DepthHeight;
            public byte[] ColorBytes;
            public int ColorByteCount;
            public int ColorWidth;
            public int ColorHeight;
            public byte[] IRBytes;       // empty when IR stream is disabled
            public int IRByteCount;
            public int IRWidth;
            public int IRHeight;
        }

        private readonly object _lock = new object();
        private readonly Slot[] _free;
        private int _freeCount;
        private Slot _ready;

        public SlotPool(int slotCount, int capacity, int depthRawByteCapacity, int colorRawByteCapacity,
                         int irRawByteCapacity)
        {
            if (slotCount < 2) throw new ArgumentOutOfRangeException(nameof(slotCount));
            _free = new Slot[slotCount];
            for (int i = 0; i < slotCount; i++)
            {
                _free[i] = new Slot
                {
                    Buffer = new NativeArray<ObColorPoint>(capacity, Allocator.Persistent,
                        NativeArrayOptions.UninitializedMemory),
                    DepthBytes = depthRawByteCapacity > 0 ? new byte[depthRawByteCapacity] : Array.Empty<byte>(),
                    ColorBytes = colorRawByteCapacity > 0 ? new byte[colorRawByteCapacity] : Array.Empty<byte>(),
                    IRBytes = irRawByteCapacity > 0 ? new byte[irRawByteCapacity] : Array.Empty<byte>(),
                };
            }
            _freeCount = slotCount;
        }

        /// <summary>Capture thread: get a writable slot. Falls back to recycling the unread ready slot.</summary>
        public Slot AcquireWrite()
        {
            lock (_lock)
            {
                if (_freeCount > 0) return _free[--_freeCount];
                // Pool exhausted; recycle the previously published frame (Unity hasn't picked it up).
                var s = _ready;
                _ready = null;
                return s;
            }
        }

        /// <summary>
        /// Capture thread: publish a filled slot as the latest. Returns true if it
        /// displaced a previously published-but-unread slot (a dropped frame).
        /// </summary>
        public bool Publish(Slot s)
        {
            lock (_lock)
            {
                bool dropped = false;
                if (_ready != null)
                {
                    _free[_freeCount++] = _ready;
                    dropped = true;
                }
                _ready = s;
                return dropped;
            }
        }

        /// <summary>Main thread: take the latest slot if any. Caller must call <see cref="ReturnRead"/>.</summary>
        public Slot TryAcquireRead()
        {
            lock (_lock)
            {
                var s = _ready;
                _ready = null;
                return s;
            }
        }

        public void ReturnRead(Slot s)
        {
            if (s == null) return;
            lock (_lock)
            {
                _free[_freeCount++] = s;
            }
        }

        public void Dispose()
        {
            lock (_lock)
            {
                for (int i = 0; i < _free.Length; i++)
                {
                    if (_free[i] != null && _free[i].Buffer.IsCreated)
                        _free[i].Buffer.Dispose();
                    _free[i] = null;
                }
                if (_ready != null && _ready.Buffer.IsCreated)
                    _ready.Buffer.Dispose();
                _ready = null;
                _freeCount = 0;
            }
        }
    }
}
