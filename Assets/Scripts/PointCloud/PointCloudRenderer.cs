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
        public uint colorWidth = 640;
        public uint colorHeight = 480;
        public uint colorFps = 30;

        [Header("Pipeline")]
        public ObAlignMode alignMode = ObAlignMode.D2CSwMode;
        public bool enableFrameSync = true;

        [Header("Point cloud")]
        [Tooltip("Upper bound used to size the GPU vertex buffer. Frames with more points are clipped.")]
        public int maxPoints = 640 * 576;
        [Tooltip("Material used for the points. Use Orbbec/PointCloudUnlit or compatible.")]
        public Material pointMaterial;

        // --- Native ---
        private OrbbecDevice _device;
        private OrbbecPipeline _pipeline;
        private OrbbecConfig _config;
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
                _mesh.SetVertexBufferData(slot.Buffer, 0, 0, n,
                    flags: MeshUpdateFlags.DontRecalculateBounds | MeshUpdateFlags.DontValidateIndices);
                _mesh.SetSubMesh(0, new SubMeshDescriptor(0, n, MeshTopology.Points),
                    MeshUpdateFlags.DontRecalculateBounds | MeshUpdateFlags.DontValidateIndices);
                LastPointCount = n;
                LastTimestampUs = slot.TimestampUs;
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
            _config.SetAlignMode(alignMode);
            _config.SetFrameAggregateOutputMode(ObFrameAggregateOutputMode.AllTypeFrameRequire);

            _pipeline = _device.CreatePipeline();
            if (enableFrameSync) _pipeline.EnableFrameSync();
            _pipeline.Start(_config);

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

                    using var points = _pointCloudFilter.Process(frameset);
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
