// Cumulative point cloud snapshotter. When "No Erase" is on, every `interval`
// frames the current (post-filter) point cloud is copied into its own Mesh
// and parented to this GameObject, so snapshots accumulate over time. The
// Clear button (Inspector) destroys all accumulated snapshots.

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

        // Called by PointCloudRenderer each frame after filtering. Passes the filtered
        // buffer in renderer-local space plus the renderer's transform. Points are
        // baked into this component's local space so snapshots stay put if the
        // renderer later moves.
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
        }

        private void CaptureSnapshot(NativeArray<ObColorPoint> buffer, int count, Transform rendererTransform, Material fallbackMaterial)
        {
            // renderer-local -> cumulative-local. Expanded inline for the tight copy loop.
            Matrix4x4 m = transform.worldToLocalMatrix * rendererTransform.localToWorldMatrix;
            float m00 = m.m00, m01 = m.m01, m02 = m.m02, m03 = m.m03;
            float m10 = m.m10, m11 = m.m11, m12 = m.m12, m13 = m.m13;
            float m20 = m.m20, m21 = m.m21, m22 = m.m22, m23 = m.m23;

            var data = new NativeArray<ObColorPoint>(count, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
            try
            {
                for (int i = 0; i < count; i++)
                {
                    var p = buffer[i];
                    data[i] = new ObColorPoint
                    {
                        X = m00 * p.X + m01 * p.Y + m02 * p.Z + m03,
                        Y = m10 * p.X + m11 * p.Y + m12 * p.Z + m13,
                        Z = m20 * p.X + m21 * p.Y + m22 * p.Z + m23,
                        R = p.R,
                        G = p.G,
                        B = p.B,
                    };
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
                mesh.SetVertexBufferParams(count, attrs);
                mesh.SetVertexBufferData(data, 0, 0, count,
                    flags: MeshUpdateFlags.DontRecalculateBounds | MeshUpdateFlags.DontValidateIndices);

                mesh.SetIndexBufferParams(count, IndexFormat.UInt32);
                var indices = new NativeArray<uint>(count, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
                try
                {
                    for (int i = 0; i < count; i++) indices[i] = (uint)i;
                    mesh.SetIndexBufferData(indices, 0, 0, count,
                        MeshUpdateFlags.DontRecalculateBounds | MeshUpdateFlags.DontValidateIndices);
                }
                finally
                {
                    indices.Dispose();
                }
                mesh.SetSubMesh(0, new SubMeshDescriptor(0, count, MeshTopology.Points),
                    MeshUpdateFlags.DontRecalculateBounds | MeshUpdateFlags.DontValidateIndices);

                var go = new GameObject($"_Snapshot_{_snapshots.Count}");
                go.transform.SetParent(transform, worldPositionStays: false);
                var mf = go.AddComponent<MeshFilter>();
                var mr = go.AddComponent<MeshRenderer>();
                mr.shadowCastingMode = ShadowCastingMode.Off;
                mr.receiveShadows = false;
                mr.lightProbeUsage = LightProbeUsage.Off;
                mr.reflectionProbeUsage = ReflectionProbeUsage.Off;
                mf.sharedMesh = mesh;

                var mat = snapshotMaterial != null ? snapshotMaterial : fallbackMaterial;
                if (mat != null) mr.sharedMaterial = mat;

                _snapshots.Add(go);

                if (maxSnapshots > 0)
                {
                    while (_snapshots.Count > maxSnapshots)
                    {
                        DestroySnapshotAt(0);
                    }
                }
            }
            finally
            {
                data.Dispose();
            }
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
