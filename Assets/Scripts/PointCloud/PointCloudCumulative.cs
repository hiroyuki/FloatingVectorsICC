// Cumulative point cloud snapshotter. When "No Erase" is on, every `interval`
// frames the current point cloud is copied into its own Mesh and kept visible,
// so snapshots accumulate over time. Vertices are stored untransformed
// (renderer-local); the snapshot GameObject's world transform is frozen to the
// renderer's world transform at capture time, then reparented to this
// GameObject so future renderer motion is ignored but cumulative motion moves
// the whole accumulation together. The Clear button (Inspector) destroys all
// accumulated snapshots.

using System.Collections.Generic;
using Orbbec;
using Unity.Collections;
using UnityEngine;
using UnityEngine.Rendering;

namespace PointCloud
{
    [DisallowMultipleComponent]
    public class PointCloudCumulative : MonoBehaviour
    {
        [Tooltip("When enabled, every 'interval' frames the current point cloud is " +
                 "snapshotted and kept visible, accumulating over time. When disabled, " +
                 "no new snapshots are captured (existing snapshots remain until cleared).")]
        public bool noErase = false;

        [Min(1)]
        [Tooltip("Frame interval between snapshots. 1 = every frame; 30 = roughly once per second at 30 FPS.")]
        public int interval = 30;

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

        public int SnapshotCount => _snapshots.Count;

        private readonly List<GameObject> _snapshots = new List<GameObject>();

        // Per-renderer frame counters, keyed by the renderer Transform's InstanceID.
        // A single cumulative component can legitimately be referenced by multiple
        // renderers; each must be paced independently so `interval` means
        // "every N frames per renderer", not "every N total OnFrame calls".
        private readonly Dictionary<int, int> _frameCounters = new Dictionary<int, int>();

        // Shared identity index buffer. Lazily grown so every snapshot mesh can
        // memcpy the first `count` entries instead of re-filling `0..N-1` each time.
        private NativeArray<uint> _sharedIndices;

        // Called by PointCloudRenderer each frame. `buffer` is the raw SDK point cloud
        // in renderer-local space; OBB / decimation filters are applied in the vertex
        // shader, not here, so snapshots include points that the live view culls.
        // The snapshot GO's world transform is frozen to rendererTransform's current
        // world transform so snapshots stay put if the renderer later moves.
        public void OnFrame(NativeArray<ObColorPoint> buffer, int count, Transform rendererTransform, Material fallbackMaterial)
        {
            if (!noErase || count <= 0 || rendererTransform == null) return;
            int iv = Mathf.Max(1, interval);

            int id = rendererTransform.GetInstanceID();
            _frameCounters.TryGetValue(id, out int c);
            c++;
            if (c < iv)
            {
                _frameCounters[id] = c;
                return;
            }
            _frameCounters[id] = 0;
            CaptureSnapshot(buffer, count, rendererTransform, fallbackMaterial);
        }

        [ContextMenu("Clear")]
        public void Clear()
        {
            for (int i = _snapshots.Count - 1; i >= 0; i--)
            {
                DestroySnapshotAt(i);
            }
            _frameCounters.Clear();
        }

        private void OnDestroy()
        {
            Clear();
            if (_sharedIndices.IsCreated) _sharedIndices.Dispose();
            if (_filterScratch.IsCreated) _filterScratch.Dispose();
        }

        // Reusable destination for capsule-filtered snapshots (allocated lazily,
        // grown as needed). Holds the subset of `buffer` that passed the capsule
        // test in world space; copied straight into the snapshot mesh below.
        private NativeArray<ObColorPoint> _filterScratch;

        private void CaptureSnapshot(NativeArray<ObColorPoint> buffer, int count, Transform rendererTransform, Material fallbackMaterial)
        {
            // Points stay in renderer-local space; the snapshot GO's world transform is
            // frozen to match the renderer's current world transform, so no per-point
            // CPU matrix multiply is needed. This is the hot path for interval=1.
            //
            // When a capsule filter is wired and active, we instead transform each
            // point to world space, test it against the capsule union, and keep
            // only the survivors. The snapshot then renders only those frozen
            // points — accumulating across frames forms a body-motion trail.
            NativeArray<ObColorPoint> srcBuffer = buffer;
            int srcCount = count;
            if (capsuleFilter != null
                && capsuleFilter.Mode != PointCloudCapsuleFilter.FilterMode.Disabled)
            {
                // KeepInside with an empty capsule list culls every point on the
                // live shader path; mirror that here so cumulative snapshots
                // don't accidentally freeze the full cloud while the body is
                // momentarily lost. KeepOutside with empty list keeps every
                // point (no exclusion zone), again mirroring the shader.
                if (capsuleFilter.CapsuleCount == 0)
                {
                    if (capsuleFilter.Mode == PointCloudCapsuleFilter.FilterMode.KeepInside) return;
                    // KeepOutside + empty -> pass-through (srcBuffer/srcCount unchanged).
                }
                else
                {
                    EnsureFilterScratch(count);
                    srcCount = FilterByCapsules(buffer, count, _filterScratch, rendererTransform);
                    if (srcCount == 0) return;
                    srcBuffer = _filterScratch;
                }
            }

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

        // Copy the subset of `src[0..count)` that passes the capsule test (in
        // world space) into `dst`, preserving order. Returns the survivor count.
        // Points are transformed local -> world via rendererTransform's TRS
        // because the capsule list is held in world space (single source of
        // truth shared between live shader filter and this CPU pre-filter).
        private int FilterByCapsules(NativeArray<ObColorPoint> src, int count,
                                      NativeArray<ObColorPoint> dst, Transform rendererTransform)
        {
            var l2w = rendererTransform.localToWorldMatrix;
            int written = 0;
            for (int i = 0; i < count; i++)
            {
                var p = src[i];
                Vector3 wp = l2w.MultiplyPoint3x4(new Vector3(p.X, p.Y, p.Z));
                bool inside = capsuleFilter.ContainsWorldPoint(wp);
                bool keep = capsuleFilter.Mode == PointCloudCapsuleFilter.FilterMode.KeepInside
                    ? inside
                    : !inside;
                if (!keep) continue;
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
