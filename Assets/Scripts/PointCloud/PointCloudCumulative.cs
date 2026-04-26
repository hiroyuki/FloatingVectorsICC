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
        }

        private void CaptureSnapshot(NativeArray<ObColorPoint> buffer, int count, Transform rendererTransform, Material fallbackMaterial)
        {
            // Points stay in renderer-local space; the snapshot GO's world transform is
            // frozen to match the renderer's current world transform, so no per-point
            // CPU matrix multiply is needed. This is the hot path for interval=1.

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
            mesh.SetVertexBufferParams(count, attrs);
            // SetVertexBufferData copies into the mesh's own storage, so reusing `buffer`
            // next frame is safe.
            mesh.SetVertexBufferData(buffer, 0, 0, count,
                flags: MeshUpdateFlags.DontRecalculateBounds | MeshUpdateFlags.DontValidateIndices);

            mesh.SetIndexBufferParams(count, IndexFormat.UInt32);
            EnsureSharedIndices(count);
            mesh.SetIndexBufferData(_sharedIndices, 0, 0, count,
                MeshUpdateFlags.DontRecalculateBounds | MeshUpdateFlags.DontValidateIndices);
            mesh.SetSubMesh(0, new SubMeshDescriptor(0, count, MeshTopology.Points),
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
