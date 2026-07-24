// Reads the displayed point cloud into plain CPU arrays for the web/AR export.
// The live/playback point clouds live in GPU meshes as ObColorPoint (pos12 +
// col12, 24-byte Raw ByteAddressBuffer) that the render shader interprets — so
// mesh.vertices/mesh.colors read garbage. We pull the raw buffer and decode it
// (same layout PointCloudMotionCurves seeds from), then apply the region +
// decimate filters on the CPU.
//
// NOTE (2026-07-24): only the BoundingVolume region + decimater KeepRatio are
// applied here — the floor mask and skeleton/capsule person-mask that the
// display shader also runs are NOT yet replicated, so the exported cloud is
// wider than the on-screen one (floor/background points survive). Wiring those
// is deferred until the look is validated.

using System.Collections.Generic;
using UnityEngine;

namespace PointCloud
{
    public static class PointCloudSnapshot
    {
        private const int kObColorPointStride = 24; // 3 floats pos + 3 floats colour

        /// <summary>World-space positions + sRGB colours of every displayed
        /// point-cloud point, after the BoundingVolume region filter and the
        /// decimater's KeepRatio. Deterministic for a given <paramref name="seed"/>.
        /// Runs a synchronous GPU readback — main thread only.</summary>
        public static void Gather(BoundingVolume bounds, float keepRatio, int seed,
                                  out Vector3[] pos, out Vector3[] col)
        {
            var pl = new List<Vector3>(1 << 16);
            var cl = new List<Vector3>(1 << 16);
            var rng = new System.Random(seed);
            keepRatio = Mathf.Clamp01(keepRatio);

            bool useBounds = bounds != null && bounds.Mode != BoundingVolume.FilterMode.Disabled;
            Matrix4x4 worldToBox = useBounds ? bounds.transform.worldToLocalMatrix : Matrix4x4.identity;
            bool keepInside = useBounds && bounds.Mode == BoundingVolume.FilterMode.KeepInside;

            foreach (var mf in FindPointCloudMeshes())
            {
                var m = mf.sharedMesh;
                int cnt = m.vertexCount;
                var raw = new float[cnt * 6];
                var vb = m.GetVertexBuffer(0); // GraphicsBuffer — must be disposed even if GetData throws
                try { vb.GetData(raw); }
                finally { vb.Dispose(); }
                var l2w = mf.transform.localToWorldMatrix;

                for (int i = 0; i < cnt; i++)
                {
                    float px = raw[i * 6], py = raw[i * 6 + 1], pz = raw[i * 6 + 2];
                    // Hole dummies sit at ~1e10 / NaN; real points never do.
                    if (float.IsNaN(px) || float.IsInfinity(px) ||
                        Mathf.Abs(px) > 1e4f || Mathf.Abs(py) > 1e4f || Mathf.Abs(pz) > 1e4f) continue;

                    Vector3 w = l2w.MultiplyPoint3x4(new Vector3(px, py, pz));
                    if (useBounds)
                    {
                        Vector3 b = worldToBox.MultiplyPoint3x4(w); // unit cube = [-0.5, 0.5]
                        bool inside = Mathf.Abs(b.x) <= 0.5f && Mathf.Abs(b.y) <= 0.5f && Mathf.Abs(b.z) <= 0.5f;
                        if (inside != keepInside) continue;
                    }
                    if (keepRatio < 1f && rng.NextDouble() > keepRatio) continue;

                    pl.Add(w);
                    cl.Add(new Vector3(Mathf.Clamp01(raw[i * 6 + 3]),
                                       Mathf.Clamp01(raw[i * 6 + 4]),
                                       Mathf.Clamp01(raw[i * 6 + 5])));
                }
            }
            pos = pl.ToArray();
            col = cl.ToArray();
        }

        // Every live PointCloudRenderer / _Playback_* mesh exposes its vertex
        // buffer 0 as a Raw ObColorPoint buffer — the same test IsUsableSource
        // uses to seed the motion curves. Anything else (CPU meshes, unrelated
        // geometry) is rejected so the raw read stays valid.
        private static IEnumerable<MeshFilter> FindPointCloudMeshes()
        {
            foreach (var mf in Object.FindObjectsByType<MeshFilter>(FindObjectsSortMode.None))
            {
                var m = mf.sharedMesh;
                if (m == null || m.vertexCount == 0) continue;
                if ((m.vertexBufferTarget & GraphicsBuffer.Target.Raw) == 0) continue;
                if (m.GetVertexBufferStride(0) != kObColorPointStride) continue;
                yield return mf;
            }
        }
    }
}
