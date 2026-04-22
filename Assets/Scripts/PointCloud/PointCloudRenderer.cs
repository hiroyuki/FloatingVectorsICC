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
                    slot.TimestampUs = points.TimestampUs;

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
