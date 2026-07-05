// Cumulative point cloud snapshotter. When "No Erase" is on, every
// `intervalSeconds` seconds (0 = every frame) the current point cloud is copied
// into its own Mesh and kept visible, so snapshots accumulate over time. Vertices are stored untransformed
// (renderer-local); the snapshot GameObject's world transform is frozen to the
// renderer's world transform at capture time, then reparented to this
// GameObject so future renderer motion is ignored but cumulative motion moves
// the whole accumulation together. The Clear button (Inspector) destroys all
// accumulated snapshots.
//
// Two entry points feed snapshots in:
//   - OnFrame(NativeArray, ...)   — live CPU path. PointCloudRenderer calls this
//     with the SDK's CPU-side point buffer (renderer.useGpuReconstruction must be
//     false; the GPU-reconstruct live path warns and skips, per the Renderer's
//     _warnedCumulative gate). CPU FilterForCapture compacts on the main thread.
//   - OnPlaybackFrame(Mesh, ...)  — playback path. SensorRecorder calls this
//     after ReconstructAndUpload writes the playback mesh on GPU. We never read
//     the vertex buffer back: a compute shader (PointCloudCumulativeFilter)
//     reads it directly, runs the same filter chain on GPU, atomic-appends
//     survivors to the snapshot mesh's vertex buffer, and the host reads back
//     only the 4-byte survivor count to size the SubMesh. Removed the previous
//     8.7 MB sync readback per snapshot which dominated the 45 ms/capture cost.

using System.Collections.Generic;
using Orbbec;
using Unity.Collections;
using UnityEngine;
using UnityEngine.Rendering;

namespace PointCloud
{
    [DisallowMultipleComponent]
    public class PointCloudCumulative : MonoBehaviour, Shared.IAccumulationController
    {
        [Tooltip("When enabled, every 'intervalSeconds' seconds the current point cloud is " +
                 "snapshotted and kept visible, accumulating over time. When disabled, " +
                 "no new snapshots are captured (existing snapshots remain until cleared).")]
        public bool noErase = false;

        [Min(0f)]
        [Tooltip("Seconds between snapshots (wall clock, per renderer). 0 = capture every frame. " +
                 "Replaces the old frame-count interval — seconds stay consistent whether the " +
                 "editor runs at 30 or 120 FPS.")]
        public float intervalSeconds = 1f;

        [Tooltip("Material used for snapshot meshes. If null, falls back to the renderer's pointMaterial.")]
        public Material snapshotMaterial;

        [Min(0)]
        [Tooltip("Maximum number of snapshots to keep. Oldest are dropped when exceeded. 0 = unlimited.")]
        public int maxSnapshots = 0;

        [Tooltip("Optional capsule filter. When assigned and its Mode is KeepInside/KeepOutside, each " +
                 "snapshot is pre-filtered on CPU so only the points passing the capsule test are " +
                 "frozen in space. Pair with PointCloudRenderer.capsuleFilter (same instance) to " +
                 "fixate the BT-tube-interior points as a body movement trail.")]
        public PointCloudCapsuleFilter capsuleFilter;

        [Header("Diagnostics")]
        [Tooltip("Per-second log: attempts/sec (cursor advances that hit OnFrame/OnPlaybackFrame), " +
                 "actual captures/sec (after interval gate), per-stage ms/capture (GPU readback, " +
                 "filter, mesh alloc+upload), total wallclock %, and current snapshot count. " +
                 "Lets you see if capture cost or accumulated draw cost is the FPS bottleneck.")]
        public bool diagnosticLogging = false;

        public int SnapshotCount => _snapshots.Count;

        // ---- Shared.IAccumulationController (unified accumulation UI) ----
        // Start/Stop decompose the noErase toggle; Clear deletes all snapshots
        // (destroys child GOs — the shared editor uses FullHierarchy undo).
        public bool IsAccumulating => noErase;
        public float IntervalSeconds
        {
            get => intervalSeconds;
            set => intervalSeconds = Mathf.Max(0f, value);
        }
        public string StatusText => SnapshotCount + " snapshot(s)" + (noErase ? " — accumulating" : "");
        public bool CanStart => !noErase;
        public string StartLabel => "Start (no erase)";
        public void StartAccumulate() => noErase = true;
        public bool CanStop => noErase;
        public void StopAccumulate() => noErase = false;
        public bool CanClear => SnapshotCount > 0;
        public string ClearLabel => "Clear snapshots";
        public void ClearAccumulated() => Clear();

        // Whether accumulated snapshots are currently rendered. Toggling this
        // off disables the MeshRenderer on every snapshot GO without destroying
        // them, so flipping back on restores the accumulation instantly. New
        // captures inherit this state. SensorRecorder pushes its
        // showPointClouds toggle into here so the single "point cloud visual
        // on/off" lever covers both the live playback mesh and the snapshots.
        public bool SnapshotsVisible
        {
            get => _snapshotsVisible;
            set
            {
                if (_snapshotsVisible == value) return;
                _snapshotsVisible = value;
                for (int i = 0; i < _snapshots.Count; i++)
                {
                    var go = _snapshots[i];
                    if (go == null) continue;
                    var mr = go.GetComponent<MeshRenderer>();
                    if (mr != null && mr.enabled != value) mr.enabled = value;
                }
            }
        }
        private bool _snapshotsVisible = true;

        // --- Diagnostics state ---
        // Per-second window counters / timers. _diagSw is shared because the three
        // bracketed regions (readback, filter, mesh) never nest.
        private int _diagAttempts;
        private int _diagCaptures;
        private long _diagReadbackTicks;
        private long _diagFilterTicks;
        private long _diagMeshTicks;
        private int _diagSurvivorsTotal;
        private float _diagWindowStart;
        private readonly System.Diagnostics.Stopwatch _diagSw = new System.Diagnostics.Stopwatch();

        private readonly List<GameObject> _snapshots = new List<GameObject>();

        // Per-renderer last-capture times, keyed by the renderer Transform's
        // InstanceID. A single cumulative component can legitimately be referenced
        // by multiple renderers; each must be paced independently so
        // `intervalSeconds` means "every N seconds per renderer", not "every N
        // seconds across all OnFrame calls combined".
        private readonly Dictionary<int, double> _lastCaptureTimes = new Dictionary<int, double>();

        // Time gate shared by the live and playback entry points. Returns true (and
        // stamps the clock) when this renderer is due for a capture. Playback pause
        // stops the OnPlaybackFrame calls themselves, so wall-clock time is safe
        // here — a paused recording can't burn through the interval.
        private bool IntervalElapsed(int rendererId)
        {
            if (intervalSeconds <= 0f) return true;   // every frame
            double now = Time.timeAsDouble;
            if (_lastCaptureTimes.TryGetValue(rendererId, out double last)
                && now - last < intervalSeconds) return false;
            _lastCaptureTimes[rendererId] = now;
            return true;
        }

        // Shared identity index buffer. Lazily grown so every snapshot mesh can
        // memcpy the first `count` entries instead of re-filling `0..N-1` each time.
        private NativeArray<uint> _sharedIndices;

        // Called by PointCloudRenderer each frame. `buffer` is the raw SDK point cloud
        // in renderer-local space. `boundingBox` / `decimater` are the caller's already-
        // resolved filter refs (so the same instances configured on the renderer are
        // reused — cumulative does not own its own copies). Both are applied as a CPU
        // pre-filter at capture so survivors are frozen with smaller meshes; the
        // decimater is intentionally captured at capture-time (not shader-time) so
        // each snapshot has a stable, non-flickering random subset.
        //
        // Motion-field is shader-level only and intentionally NOT applied here (it
        // would drift the captured trail as the body moved past).
        // The snapshot GO's world transform is frozen to rendererTransform's current
        // world transform so snapshots stay put if the renderer later moves.
        public void OnFrame(NativeArray<ObColorPoint> buffer, int count, Transform rendererTransform,
                            Material fallbackMaterial, BoundingVolume boundingBox,
                            PointCloudDecimater decimater)
        {
            if (!noErase || count <= 0 || rendererTransform == null) return;
            if (diagnosticLogging) _diagAttempts++;
            if (!IntervalElapsed(rendererTransform.GetInstanceID())) return;
            if (diagnosticLogging) _diagCaptures++;
            CaptureSnapshot(buffer, count, rendererTransform, fallbackMaterial, boundingBox, decimater);
        }

        // Playback variant: source data lives in `playbackMesh`'s GPU vertex buffer
        // (filled by PointCloudReconstruct.compute in SensorRecorder, which
        // sets Raw target so we can re-read it as ByteAddressBuffer here). We
        // dispatch PointCloudCumulativeFilter.compute to apply sanity / decim /
        // bbox / capsule filters directly on GPU and atomic-append survivors to
        // the snapshot mesh's vertex buffer — no full vertex-buffer readback,
        // only the 4-byte survivor counter comes back to CPU.
        //
        // The mesh layout must match ObColorPoint (6 floats = 24 bytes); the
        // playback reconstruct shader and the snapshot mesh both use this layout.
        public void OnPlaybackFrame(Mesh playbackMesh, int count, Transform playbackTransform,
                                    Material fallbackMaterial, BoundingVolume boundingBox,
                                    PointCloudDecimater decimater)
        {
            if (!noErase || count <= 0 || playbackMesh == null || playbackTransform == null) return;
            if (diagnosticLogging) _diagAttempts++;
            if (!IntervalElapsed(playbackTransform.GetInstanceID())) return;
            if (diagnosticLogging) _diagCaptures++;

            // KeepInside + empty-capsule: mirror live shader behavior and skip
            // the snapshot entirely (live view culls every point in that state,
            // so freezing it would paint a "lost the body" cloud into history).
            if (capsuleFilter != null
                && capsuleFilter.Mode == PointCloudCapsuleFilter.FilterMode.KeepInside
                && capsuleFilter.CapsuleCount == 0)
                return;

            var srcBuf = playbackMesh.GetVertexBuffer(0);
            if (srcBuf == null) return;
            try
            {
                CaptureSnapshotGpu(srcBuf, count, playbackTransform, fallbackMaterial,
                                   boundingBox, decimater);
            }
            finally
            {
                srcBuf.Dispose();
            }
        }

        [ContextMenu("Clear")]
        public void Clear()
        {
            for (int i = _snapshots.Count - 1; i >= 0; i--)
            {
                DestroySnapshotAt(i);
            }
            _lastCaptureTimes.Clear();
        }

        private void OnDestroy()
        {
            Clear();
            if (_sharedIndices.IsCreated) _sharedIndices.Dispose();
            if (_filterScratch.IsCreated) _filterScratch.Dispose();
            _counterBuf?.Dispose(); _counterBuf = null;
        }

        // --- GPU filter (playback path) ---
        //
        // Static across all PointCloudCumulative instances — same pattern the
        // recorder uses for PointCloudReconstruct.compute. Shader is in
        // Assets/Scripts/PointCloud/Resources/ so Resources.Load picks it up.
        private static ComputeShader s_filterShader;
        private static int s_filterKernel = -1;
        private static readonly int kId_Src          = Shader.PropertyToID("_Src");
        private static readonly int kId_Dst          = Shader.PropertyToID("_Dst");
        private static readonly int kId_Counter      = Shader.PropertyToID("_Counter");
        private static readonly int kId_Count        = Shader.PropertyToID("_Count");
        private static readonly int kId_SanityRange  = Shader.PropertyToID("_SanityRange");
        private static readonly int kId_KeepRatio    = Shader.PropertyToID("_KeepRatio");
        private static readonly int kId_DecimEnabled = Shader.PropertyToID("_DecimEnabled");
        private static readonly int kId_FrameSeed    = Shader.PropertyToID("_FrameSeed");
        private static readonly int kId_ObjToWorld   = Shader.PropertyToID("_ObjToWorld");
        private static readonly int kId_BboxMode     = Shader.PropertyToID("_BboxMode");
        private static readonly int kId_WorldToBbox  = Shader.PropertyToID("_WorldToBbox");
        private static readonly int kId_CapsMode     = Shader.PropertyToID("_CapsMode");
        private static readonly int kId_CapsCount    = Shader.PropertyToID("_CapsCount");
        private static readonly int kId_CapsA        = Shader.PropertyToID("_CapsA");
        private static readonly int kId_CapsB        = Shader.PropertyToID("_CapsB");

        // Per-instance: 1-uint counter buffer, single-element reset / readback
        // arrays. Persistent across captures — SetData/GetData reuse the same
        // GraphicsBuffer to avoid per-capture allocation churn.
        private GraphicsBuffer _counterBuf;
        private static readonly uint[] s_counterReset = { 0u };
        private readonly uint[] _counterReadback = new uint[1];

        // Padding for capsule arrays — the compute shader's _CapsA[64] /
        // _CapsB[64] uniforms have fixed size, so even when CapsuleCount==0
        // we must push a full 64-length array. _CapsCount gates visibility.
        private static readonly Vector4[] s_emptyCaps = new Vector4[PointCloudCapsuleFilter.MaxCapsules];

        private static bool TryLoadFilterShader()
        {
            if (s_filterShader != null && s_filterKernel >= 0) return true;
            s_filterShader = Resources.Load<ComputeShader>("PointCloudCumulativeFilter");
            if (s_filterShader == null) return false;
            s_filterKernel = s_filterShader.FindKernel("CSMain");
            return s_filterKernel >= 0;
        }

        private void CaptureSnapshotGpu(GraphicsBuffer srcBuf, int count,
                                        Transform rendererTransform, Material fallbackMaterial,
                                        BoundingVolume boundingBox,
                                        PointCloudDecimater decimater)
        {
            if (!TryLoadFilterShader())
            {
                Debug.LogError(
                    $"[{nameof(PointCloudCumulative)}] PointCloudCumulativeFilter compute shader " +
                    $"not found in Resources/", this);
                return;
            }

            if (_counterBuf == null)
                _counterBuf = new GraphicsBuffer(GraphicsBuffer.Target.Structured, 1, sizeof(uint));

            // Allocate snapshot mesh with FULL input capacity. The compute writes
            // only the survivors to a compact prefix [0, survivorCount); SubMesh
            // is sized to that count after the counter readback so the tail of
            // the vertex buffer is never indexed (left as initial-state bytes).
            // Memory cost per snapshot is count * 24 bytes regardless of decim
            // / bbox aggressiveness — for dw*dh=370k that's 8.85 MB per snap.
            // With maxSnapshots capping this is bounded; if unbounded growth
            // becomes a memory problem we'd switch to a temp-buffer + copy
            // scheme to size meshes to the actual survivor count.
            //
            // The mesh has to be FULLY initialised (vertex params + index params
            // + index data + an initial SubMesh) BEFORE GetVertexBuffer(0) is
            // bound for compute write. Without the index-side init, Unity 6
            // leaves the underlying vertex GPU buffer in an uncommitted state
            // and the compute's _Dst.Store3 writes go nowhere — confirmed by
            // GetData readback showing all-zeros despite a non-zero atomic
            // counter. This mirrors SensorRecorder.EnsurePlaybackMesh which
            // sets index params + index data at mesh-creation time and works.
            if (diagnosticLogging) _diagSw.Restart();
            var mesh = new Mesh
            {
                name = $"PointCloudSnapshot_{Time.frameCount}",
                indexFormat = IndexFormat.UInt32,
                bounds = new Bounds(Vector3.zero, Vector3.one * 100f),
            };
            mesh.vertexBufferTarget |= GraphicsBuffer.Target.Raw;
            var attrs = new[]
            {
                new VertexAttributeDescriptor(VertexAttribute.Position, VertexAttributeFormat.Float32, 3, stream: 0),
                new VertexAttributeDescriptor(VertexAttribute.Color,    VertexAttributeFormat.Float32, 3, stream: 0),
            };
            mesh.SetVertexBufferParams(count, attrs);
            mesh.SetIndexBufferParams(count, IndexFormat.UInt32);
            EnsureSharedIndices(count);
            mesh.SetIndexBufferData(_sharedIndices, 0, 0, count,
                MeshUpdateFlags.DontRecalculateBounds | MeshUpdateFlags.DontValidateIndices);
            // Initial SubMesh with zero indices so the mesh is "complete" before
            // the dispatch. SubMesh count is updated post-readback to expose the
            // compact survivor prefix to the renderer.
            mesh.SetSubMesh(0, new SubMeshDescriptor(0, 0, MeshTopology.Points),
                MeshUpdateFlags.DontRecalculateBounds | MeshUpdateFlags.DontValidateIndices);
            var dstBuf = mesh.GetVertexBuffer(0);
            if (diagnosticLogging) { _diagSw.Stop(); _diagMeshTicks += _diagSw.ElapsedTicks; }

            // --- Bind uniforms ---
            if (diagnosticLogging) _diagSw.Restart();
            var shader = s_filterShader;
            int k = s_filterKernel;

            shader.SetBuffer(k, kId_Src, srcBuf);
            shader.SetBuffer(k, kId_Dst, dstBuf);
            shader.SetBuffer(k, kId_Counter, _counterBuf);

            // Reset counter to 0. SetData with a static 1-element array avoids
            // per-frame allocation; it's a 4-byte upload.
            _counterBuf.SetData(s_counterReset);

            shader.SetInt(kId_Count, count);
            shader.SetFloat(kId_SanityRange, 100f);

            bool decimActive = decimater != null && decimater.Enabled;
            shader.SetFloat(kId_KeepRatio, decimActive ? decimater.KeepRatio : 1f);
            shader.SetInt(kId_DecimEnabled, decimActive ? 1 : 0);
            shader.SetInt(kId_FrameSeed, Time.frameCount);

            shader.SetMatrix(kId_ObjToWorld, rendererTransform.localToWorldMatrix);

            bool bboxActive = boundingBox != null
                && boundingBox.Mode != BoundingVolume.FilterMode.Disabled;
            int bboxMode = 0;
            if (bboxActive)
            {
                bboxMode = boundingBox.Mode == BoundingVolume.FilterMode.KeepInside ? 1 : 2;
                shader.SetMatrix(kId_WorldToBbox, boundingBox.transform.worldToLocalMatrix);
            }
            shader.SetInt(kId_BboxMode, bboxMode);

            int capsMode = 0;
            int capsCount = 0;
            bool capsuleActive = capsuleFilter != null
                && capsuleFilter.Mode != PointCloudCapsuleFilter.FilterMode.Disabled
                && capsuleFilter.CapsuleCount > 0;
            if (capsuleActive)
            {
                capsMode = capsuleFilter.Mode == PointCloudCapsuleFilter.FilterMode.KeepInside ? 1 : 2;
                capsCount = capsuleFilter.CapsuleCount;
                shader.SetVectorArray(kId_CapsA, capsuleFilter.CapsuleA);
                shader.SetVectorArray(kId_CapsB, capsuleFilter.CapsuleB);
            }
            else
            {
                shader.SetVectorArray(kId_CapsA, s_emptyCaps);
                shader.SetVectorArray(kId_CapsB, s_emptyCaps);
            }
            shader.SetInt(kId_CapsMode, capsMode);
            shader.SetInt(kId_CapsCount, capsCount);

            // --- Dispatch ---
            int groups = (count + 63) / 64;
            shader.Dispatch(k, groups, 1, 1);
            dstBuf.Dispose();
            if (diagnosticLogging) { _diagSw.Stop(); _diagFilterTicks += _diagSw.ElapsedTicks; }

            // --- Counter readback (4 bytes, blocking) ---
            // Sync stall ~0.2-0.5 ms while the GPU completes the dispatch. This
            // replaces the 3-9 ms full-vertex readback the old CPU path did, so
            // it's still a big win net.
            if (diagnosticLogging) _diagSw.Restart();
            _counterBuf.GetData(_counterReadback);
            int survivors = (int)_counterReadback[0];
            if (diagnosticLogging) { _diagSw.Stop(); _diagReadbackTicks += _diagSw.ElapsedTicks; }
            if (diagnosticLogging) _diagSurvivorsTotal += survivors;

            if (survivors == 0)
            {
                if (Application.isPlaying) Destroy(mesh); else DestroyImmediate(mesh);
                return;
            }

            // --- SubMesh sized to survivor count ---
            // Index buffer is already full-size with 0..count-1 from the pre-
            // dispatch init; only the SubMesh's indexCount needs updating to
            // expose [0, survivors) to the renderer. The tail indices reference
            // uninitialised vertex slots but the SubMesh range excludes them.
            if (diagnosticLogging) _diagSw.Restart();
            mesh.SetSubMesh(0, new SubMeshDescriptor(0, survivors, MeshTopology.Points),
                MeshUpdateFlags.DontRecalculateBounds | MeshUpdateFlags.DontValidateIndices);

            // --- GO setup + parenting (same as CPU CaptureSnapshot) ---
            var go = new GameObject($"_Snapshot_{_snapshots.Count}");
            var mf = go.AddComponent<MeshFilter>();
            var mr = go.AddComponent<MeshRenderer>();
            mr.shadowCastingMode = ShadowCastingMode.Off;
            mr.receiveShadows = false;
            mr.lightProbeUsage = LightProbeUsage.Off;
            mr.reflectionProbeUsage = ReflectionProbeUsage.Off;
            mr.enabled = _snapshotsVisible;
            mf.sharedMesh = mesh;

            var mat = snapshotMaterial != null ? snapshotMaterial : fallbackMaterial;
            if (mat != null) mr.sharedMaterial = mat;

            go.transform.SetParent(rendererTransform, worldPositionStays: false);
            go.transform.SetParent(transform, worldPositionStays: true);

            _snapshots.Add(go);
            if (maxSnapshots > 0)
            {
                while (_snapshots.Count > maxSnapshots) DestroySnapshotAt(0);
            }
            if (diagnosticLogging) { _diagSw.Stop(); _diagMeshTicks += _diagSw.ElapsedTicks; }
        }

        // Per-second diagnostic emit. Cheap when diagnosticLogging is off — early
        // return on the very first check. No per-frame work in cumulative
        // otherwise; drawing of accumulated snapshots is implicit (each snapshot
        // GO has its own MeshRenderer that the Unity renderer iterates).
        private void Update()
        {
            if (!diagnosticLogging) return;
            float now = Time.realtimeSinceStartup;
            if (_diagWindowStart == 0f) { _diagWindowStart = now; return; }
            float elapsed = now - _diagWindowStart;
            if (elapsed < 1f) return;

            double freq = System.Diagnostics.Stopwatch.Frequency;
            double rbMs = _diagReadbackTicks / freq * 1000.0;
            double flMs = _diagFilterTicks   / freq * 1000.0;
            double meMs = _diagMeshTicks     / freq * 1000.0;
            double totalMs = rbMs + flMs + meMs;
            int caps = Mathf.Max(1, _diagCaptures);
            int avgSurvivors = _diagCaptures > 0 ? _diagSurvivorsTotal / _diagCaptures : 0;

            Debug.Log(
                $"[Cumulative] attempts={_diagAttempts}/s captures={_diagCaptures}/s " +
                $"interval={intervalSeconds:0.##}s " +
                $"avg/cap: rb={rbMs / caps:F2}ms fl={flMs / caps:F2}ms me={meMs / caps:F2}ms " +
                $"(={(rbMs + flMs + meMs) / caps:F2}ms total) " +
                $"wallclock={totalMs / (elapsed * 1000.0) * 100.0:F1}% " +
                $"survivors~{avgSurvivors} snaps={_snapshots.Count}", this);

            _diagAttempts = 0;
            _diagCaptures = 0;
            _diagReadbackTicks = _diagFilterTicks = _diagMeshTicks = 0;
            _diagSurvivorsTotal = 0;
            _diagWindowStart = now;
        }

        // Reusable destination for capsule-filtered snapshots (allocated lazily,
        // grown as needed). Holds the subset of `buffer` that passed the capsule
        // test in world space; copied straight into the snapshot mesh below.
        private NativeArray<ObColorPoint> _filterScratch;

        private void CaptureSnapshot(NativeArray<ObColorPoint> buffer, int count, Transform rendererTransform,
                                     Material fallbackMaterial, BoundingVolume boundingBox,
                                     PointCloudDecimater decimater)
        {
            // Points stay in renderer-local space; the snapshot GO's world transform is
            // frozen to match the renderer's current world transform, so no per-point
            // CPU matrix multiply is needed. This is the hot path for interval=1.
            //
            // When a bbox or capsule filter is wired and active, we transform each
            // point to world space, test it, and keep only the survivors. The
            // snapshot then renders only those frozen points — both filters are
            // baked at capture (matches the existing capsule "freeze the trail"
            // semantic). The bbox reference is passed in from the caller
            // (PointCloudRenderer / SensorRecorder) — cumulative does not
            // duplicate the field, so a single scene-wide bbox flows through.
            NativeArray<ObColorPoint> srcBuffer = buffer;
            int srcCount = count;

            bool bboxActive = boundingBox != null
                && boundingBox.Mode != BoundingVolume.FilterMode.Disabled;
            bool capsuleAssigned = capsuleFilter != null
                && capsuleFilter.Mode != PointCloudCapsuleFilter.FilterMode.Disabled;
            bool decimActive = decimater != null && decimater.Enabled;
            // Playback's reconstruct shader emits dw*dh vertices, writing invalid-
            // depth pixels at (1e10, 1e10, 1e10) so the rasterizer clip-culls them.
            // Cumulative snapshots would otherwise freeze those dummies into the
            // mesh forever — pointless GPU memory. Drop them on capture with a
            // simple sanity range check. Real points are within Femto Bolt's ~5 m
            // reach, so 100 m is a huge margin. The live CPU path has no dummies
            // (count == valid count) so the check is a fast no-op there.
            const float kSanityRangeM = 100f;
            // Run the filter loop whenever ANY filter is active, OR (for safety)
            // always for now — the invalid-point early-out is cheap and useful.
            // Skipping the loop entirely when no filter is active is an
            // optimization for the live CPU path; in that case the input is
            // already compact and we'd save a 0-cost pass. Gate accordingly.
            bool needFilterPass = bboxActive || capsuleAssigned || decimActive;

            // KeepInside with an empty capsule list culls every point on the live
            // shader path; mirror that here so a body-momentarily-lost frame
            // doesn't accidentally freeze the full cloud. KeepOutside with empty
            // list keeps every point (no exclusion zone), so the capsule pass
            // can simply be skipped in the per-point loop.
            if (capsuleAssigned
                && capsuleFilter.CapsuleCount == 0
                && capsuleFilter.Mode == PointCloudCapsuleFilter.FilterMode.KeepInside)
                return;
            bool capsuleActive = capsuleAssigned && capsuleFilter.CapsuleCount > 0;

            if (needFilterPass)
            {
                EnsureFilterScratch(count);
                if (diagnosticLogging) _diagSw.Restart();
                srcCount = FilterForCapture(buffer, count, _filterScratch, rendererTransform,
                                            bboxActive ? boundingBox : null,
                                            capsuleActive,
                                            decimActive ? decimater.KeepRatio : 1f,
                                            kSanityRangeM);
                if (diagnosticLogging) { _diagSw.Stop(); _diagFilterTicks += _diagSw.ElapsedTicks; }
                if (srcCount == 0) return;
                srcBuffer = _filterScratch;
            }
            if (diagnosticLogging) _diagSurvivorsTotal += srcCount;

            if (diagnosticLogging) _diagSw.Restart();
            var mesh = new Mesh
            {
                name = $"PointCloudSnapshot_{Time.frameCount}",
                indexFormat = IndexFormat.UInt32,
                bounds = new Bounds(Vector3.zero, Vector3.one * 100f),
            };

            var attrs = new[]
            {
                new VertexAttributeDescriptor(VertexAttribute.Position, VertexAttributeFormat.Float32, 3, stream: 0),
                new VertexAttributeDescriptor(VertexAttribute.Color,    VertexAttributeFormat.Float32, 3, stream: 0),
            };
            mesh.SetVertexBufferParams(srcCount, attrs);
            // SetVertexBufferData copies into the mesh's own storage, so reusing `buffer`
            // / `_filterScratch` next frame is safe.
            mesh.SetVertexBufferData(srcBuffer, 0, 0, srcCount,
                flags: MeshUpdateFlags.DontRecalculateBounds | MeshUpdateFlags.DontValidateIndices);

            mesh.SetIndexBufferParams(srcCount, IndexFormat.UInt32);
            EnsureSharedIndices(srcCount);
            mesh.SetIndexBufferData(_sharedIndices, 0, 0, srcCount,
                MeshUpdateFlags.DontRecalculateBounds | MeshUpdateFlags.DontValidateIndices);
            mesh.SetSubMesh(0, new SubMeshDescriptor(0, srcCount, MeshTopology.Points),
                MeshUpdateFlags.DontRecalculateBounds | MeshUpdateFlags.DontValidateIndices);

            var go = new GameObject($"_Snapshot_{_snapshots.Count}");
            var mf = go.AddComponent<MeshFilter>();
            var mr = go.AddComponent<MeshRenderer>();
            mr.shadowCastingMode = ShadowCastingMode.Off;
            mr.receiveShadows = false;
            mr.lightProbeUsage = LightProbeUsage.Off;
            mr.reflectionProbeUsage = ReflectionProbeUsage.Off;
            mr.enabled = _snapshotsVisible;
            mf.sharedMesh = mesh;

            var mat = snapshotMaterial != null ? snapshotMaterial : fallbackMaterial;
            if (mat != null) mr.sharedMaterial = mat;

            // Freeze the snapshot's world transform to the renderer's current world transform,
            // then reparent to this cumulative GO preserving that world transform. Future
            // renderer motion no longer affects this snapshot; cumulative motion does.
            go.transform.SetParent(rendererTransform, worldPositionStays: false);
            go.transform.SetParent(transform, worldPositionStays: true);

            _snapshots.Add(go);

            if (maxSnapshots > 0)
            {
                while (_snapshots.Count > maxSnapshots)
                {
                    DestroySnapshotAt(0);
                }
            }
            if (diagnosticLogging) { _diagSw.Stop(); _diagMeshTicks += _diagSw.ElapsedTicks; }
        }

        private void EnsureSharedIndices(int count)
        {
            if (_sharedIndices.IsCreated && _sharedIndices.Length >= count) return;
            if (_sharedIndices.IsCreated) _sharedIndices.Dispose();
            _sharedIndices = new NativeArray<uint>(count, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
            for (int i = 0; i < count; i++) _sharedIndices[i] = (uint)i;
        }

        private void EnsureFilterScratch(int capacity)
        {
            if (_filterScratch.IsCreated && _filterScratch.Length >= capacity) return;
            if (_filterScratch.IsCreated) _filterScratch.Dispose();
            _filterScratch = new NativeArray<ObColorPoint>(capacity, Allocator.Persistent,
                NativeArrayOptions.UninitializedMemory);
        }

        // Copy the subset of `src[0..count)` that passes the active filters
        // (sanity range, bbox AND/OR capsule, decimater) into `dst`, preserving
        // order. Returns the survivor count.
        //
        // Filter order is chosen to discard the most points the fastest:
        //   1. Sanity range check — drops the (1e10,1e10,1e10) playback dummies
        //      with three abs+compare in renderer-local space (no matrix multiply).
        //   2. Decimater Bernoulli drop — single Random.value compare, drops
        //      `(1 - KeepRatio)` of survivors. Done BEFORE bbox/capsule so the
        //      expensive matrix multiplies aren't paid for points we'll drop
        //      anyway.
        //   3. World-space transform + bbox test (matrix multiply + 3 abs/compare,
        //      matches the shader's _ObbObjToBox / PassObb logic).
        //   4. Capsule test — delegates to PointCloudCapsuleFilter.ContainsWorldPoint
        //      (matches the shader's _Caps* / PassCaps logic). Slower per-point
        //      than bbox so it goes last.
        private int FilterForCapture(NativeArray<ObColorPoint> src, int count,
                                     NativeArray<ObColorPoint> dst, Transform rendererTransform,
                                     BoundingVolume boundingBox, bool useCapsule,
                                     float keepRatio, float sanityRangeM)
        {
            bool useBbox = boundingBox != null;
            bool useDecim = keepRatio < 0.999f; // tolerate float noise; effectively "<1"
            var l2w = rendererTransform.localToWorldMatrix;
            Matrix4x4 bboxW2L = useBbox ? boundingBox.transform.worldToLocalMatrix : Matrix4x4.identity;
            bool bboxKeepInside = useBbox
                && boundingBox.Mode == BoundingVolume.FilterMode.KeepInside;
            bool capsKeepInside = useCapsule
                && capsuleFilter.Mode == PointCloudCapsuleFilter.FilterMode.KeepInside;

            int written = 0;
            for (int i = 0; i < count; i++)
            {
                var p = src[i];
                if (Mathf.Abs(p.X) > sanityRangeM
                    || Mathf.Abs(p.Y) > sanityRangeM
                    || Mathf.Abs(p.Z) > sanityRangeM) continue;
                // Use >= so Random.value == 0 still drops at keepRatio == 0, matching
                // the GPU compute path's `u < _KeepRatio` semantics (Codex review feedback).
                if (useDecim && Random.value >= keepRatio) continue;
                if (useBbox || useCapsule)
                {
                    Vector3 wp = l2w.MultiplyPoint3x4(new Vector3(p.X, p.Y, p.Z));
                    if (useBbox)
                    {
                        Vector3 lp = bboxW2L.MultiplyPoint3x4(wp);
                        bool inside = Mathf.Abs(lp.x) <= 0.5f
                            && Mathf.Abs(lp.y) <= 0.5f
                            && Mathf.Abs(lp.z) <= 0.5f;
                        if (bboxKeepInside ? !inside : inside) continue;
                    }
                    if (useCapsule)
                    {
                        bool inside = capsuleFilter.ContainsWorldPoint(wp);
                        if (capsKeepInside ? !inside : inside) continue;
                    }
                }
                dst[written++] = p;
            }
            return written;
        }

        private void DestroySnapshotAt(int index)
        {
            var go = _snapshots[index];
            _snapshots.RemoveAt(index);
            if (go == null) return;

            Mesh mesh = null;
            var mf = go.GetComponent<MeshFilter>();
            if (mf != null) mesh = mf.sharedMesh;

            if (Application.isPlaying) Destroy(go); else DestroyImmediate(go);
            if (mesh != null)
            {
                if (Application.isPlaying) Destroy(mesh); else DestroyImmediate(mesh);
            }
        }
    }
}
