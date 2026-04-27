// Real-time per-device point cloud capture + Mesh rendering for one Femto Bolt.
// One GameObject per device. PointCloudCameraManager spawns these on enumeration.

using System;
using System.Threading;
using Orbbec;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using UnityEngine;
using UnityEngine.Rendering;

namespace PointCloud
{
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

        [Header("Pipeline")]
        [Tooltip("Stream that depth gets aligned TO via the Align filter. " +
                 "OB_STREAM_COLOR = depth-to-color (D2C) per CLAUDE.md. " +
                 "Note: ALIGN_D2C_SW_MODE on the pipeline config is NOT supported by the SDK; " +
                 "we always use the Align filter instead.")]
        public ObStreamType alignTargetStream = ObStreamType.Color;
        public bool enableFrameSync = true;

        [Header("Multi-device sync")]
        [Tooltip("Hardware-level sync role for this device. Default HardwareTriggering matches the " +
                 "Sync Hub Pro topology where the hub fans out trigger pulses to every camera's " +
                 "VSYNC_IN. Use Primary/SecondarySynced only for camera-to-camera daisy chain " +
                 "(no hub). Standalone disables hardware sync on this device.")]
        public ObMultiDeviceSyncMode syncMode = ObMultiDeviceSyncMode.HardwareTriggering;
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
                 "After D2C alignment, point count == color resolution (width*height).")]
        public int maxPoints = 1280 * 720;
        [Tooltip("Material used for the points. Use Orbbec/PointCloudUnlit or compatible.")]
        public Material pointMaterial;
        [Tooltip("Negate Y on the GameObject's transform. The SDK emits points in image coordinates " +
                 "(Y points down); flipping Y maps them to Unity's Y-up convention.")]
        public bool flipY = true;

        [Header("Bounding box filter")]
        [Tooltip("Optional oriented bounding box. When assigned and its filterMode is not Disabled, " +
                 "points falling outside/inside the box (per mode) are culled before upload.")]
        public PointCloudBoundingBox boundingBox;

        [Header("Decimater")]
        [Tooltip("Optional random decimater. When assigned and its reductionPercent is > 0, " +
                 "each surviving point is randomly dropped with the configured probability.")]
        public PointCloudDecimater decimater;

        [Header("Cumulative")]
        [Tooltip("Optional cumulative snapshotter. When assigned and its No Erase toggle is on, " +
                 "snapshots are captured every 'interval' frames and kept visible until cleared.")]
        public PointCloudCumulative cumulative;

        /// <summary>
        /// Fires each frame after the point cloud mesh has been updated. Subscribers receive the
        /// post-filter point buffer (renderer-local space), the valid point count, and the SDK
        /// frame timestamp (microseconds). The buffer is reused next frame, so consumers must
        /// copy any data they need to retain before returning from the handler.
        /// </summary>
        public event Action<PointCloudRenderer, NativeArray<ObColorPoint>, int, ulong> OnFrameUploaded;

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

            var slot = _slots?.TryAcquireRead();
            if (slot == null) return;

            try
            {
                int n = Math.Min(slot.PointCount, maxPoints);
                n = ApplyBoundingBoxFilter(slot.Buffer, n);
                n = ApplyDecimationFilter(slot.Buffer, n);
                cumulative?.OnFrame(slot.Buffer, n, transform, pointMaterial);
                _mesh.SetVertexBufferData(slot.Buffer, 0, 0, n,
                    flags: MeshUpdateFlags.DontRecalculateBounds | MeshUpdateFlags.DontValidateIndices);
                _mesh.SetSubMesh(0, new SubMeshDescriptor(0, n, MeshTopology.Points),
                    MeshUpdateFlags.DontRecalculateBounds | MeshUpdateFlags.DontValidateIndices);
                LastPointCount = n;
                LastTimestampUs = slot.TimestampUs;
                OnFrameUploaded?.Invoke(this, slot.Buffer, n, slot.TimestampUs);
            }
            finally
            {
                _slots.ReturnRead(slot);
            }
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
            // Pipeline-level align supports only ALIGN_D2C_HW_MODE; we use the Align filter for SW D2C.
            _config.SetAlignMode(ObAlignMode.Disable);
            _config.SetFrameAggregateOutputMode(ObFrameAggregateOutputMode.AllTypeFrameRequire);

            _pipeline = _device.CreatePipeline();
            if (enableFrameSync) _pipeline.EnableFrameSync();
            _pipeline.Start(_config);

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

            _slots = new SlotPool(slotCount: 3, capacity: maxPoints);
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

            // HARDWARE_TRIGGERING: Sync Hub Pro fans trigger pulses out to every camera's VSYNC_IN
            // and there's no daisy-chained trigger to forward — leave triggerOutEnable=false.
            // PRIMARY/SECONDARY_SYNCED need triggerOutEnable=true to forward the pulse to the next
            // camera in a chain.
            bool triggerOut = syncMode != ObMultiDeviceSyncMode.HardwareTriggering;

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

        // --- Bounding box filter ---

        // Compacts slot.Buffer in place, keeping only points that pass the bounding box test.
        // Points are transformed to box-local space (unit cube [-0.5, +0.5]^3) via the composed
        // matrix worldToBox * localToWorld, then tested per axis.
        private int ApplyBoundingBoxFilter(NativeArray<ObColorPoint> buffer, int count)
        {
            if (boundingBox == null || count <= 0) return count;
            var mode = boundingBox.Mode;
            if (mode == PointCloudBoundingBox.FilterMode.Disabled) return count;

            var m = boundingBox.transform.worldToLocalMatrix * transform.localToWorldMatrix;
            float m00 = m.m00, m01 = m.m01, m02 = m.m02, m03 = m.m03;
            float m10 = m.m10, m11 = m.m11, m12 = m.m12, m13 = m.m13;
            float m20 = m.m20, m21 = m.m21, m22 = m.m22, m23 = m.m23;
            bool keepInside = mode == PointCloudBoundingBox.FilterMode.KeepInside;

            unsafe
            {
                var ptr = (ObColorPoint*)NativeArrayUnsafeUtility.GetUnsafePtr(buffer);
                int w = 0;
                for (int r = 0; r < count; r++)
                {
                    var p = ptr[r];
                    float bx = m00 * p.X + m01 * p.Y + m02 * p.Z + m03;
                    float by = m10 * p.X + m11 * p.Y + m12 * p.Z + m13;
                    float bz = m20 * p.X + m21 * p.Y + m22 * p.Z + m23;
                    bool inside = bx >= -0.5f && bx <= 0.5f
                               && by >= -0.5f && by <= 0.5f
                               && bz >= -0.5f && bz <= 0.5f;
                    if (inside == keepInside)
                    {
                        if (w != r) ptr[w] = p;
                        w++;
                    }
                }
                return w;
            }
        }

        // --- Decimation filter ---

        // xorshift32 state for decimation. Seeded lazily on first use; rolls forward
        // across frames so successive frames don't reuse the same random sequence.
        private uint _decimateRngState;

        // Compacts slot.Buffer in place, keeping each point independently with
        // probability decimater.KeepRatio (Bernoulli sampling).
        private int ApplyDecimationFilter(NativeArray<ObColorPoint> buffer, int count)
        {
            if (decimater == null || count <= 0 || !decimater.Enabled) return count;
            float keep = decimater.KeepRatio;
            if (keep >= 1f) return count;
            if (keep <= 0f) return 0;

            uint state = _decimateRngState;
            if (state == 0u) state = (uint)System.Environment.TickCount | 1u;

            const float Inv = 1f / 16777216f; // 1 / 2^24
            unsafe
            {
                var ptr = (ObColorPoint*)NativeArrayUnsafeUtility.GetUnsafePtr(buffer);
                int w = 0;
                for (int r = 0; r < count; r++)
                {
                    state ^= state << 13;
                    state ^= state >> 17;
                    state ^= state << 5;
                    float u = (state & 0x00FFFFFFu) * Inv;
                    if (u < keep)
                    {
                        if (w != r) ptr[w] = ptr[r];
                        w++;
                    }
                }
                _decimateRngState = state;
                return w;
            }
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

                    if (_slots.Publish(slot)) FramesDropped++;
                }
            }
            catch (Exception e)
            {
                Interlocked.Exchange(ref _captureError, e);
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
        }

        private readonly object _lock = new object();
        private readonly Slot[] _free;
        private int _freeCount;
        private Slot _ready;

        public SlotPool(int slotCount, int capacity)
        {
            if (slotCount < 2) throw new ArgumentOutOfRangeException(nameof(slotCount));
            _free = new Slot[slotCount];
            for (int i = 0; i < slotCount; i++)
            {
                _free[i] = new Slot
                {
                    Buffer = new NativeArray<ObColorPoint>(capacity, Allocator.Persistent,
                        NativeArrayOptions.UninitializedMemory),
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
