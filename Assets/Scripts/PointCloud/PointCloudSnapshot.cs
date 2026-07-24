// Reads the displayed point cloud into plain CPU arrays for the web/AR export.
// The live/playback point clouds live in GPU meshes as ObColorPoint (pos12 +
// col12, 24-byte Raw ByteAddressBuffer) that the render shader interprets — so
// mesh.vertices/mesh.colors read garbage. We pull the raw buffer and decode it
// (same layout PointCloudMotionCurves seeds from), then apply the display
// filters on the CPU.
//
// Filter parity with PointCloudUnlit.shader (PassObb/PassCapsules/PassFloor):
// BoundingVolume region, decimater KeepRatio, capsule person-mask and the
// floor mask are all replicated here so the exported cloud is the on-screen
// cloud. The joint-motion displacement and reveal/fade clips are experience
// presentation effects, not content filters — deliberately not replicated.
//
// Source parity: renderers flagged suppressAsSource are skipped, same as
// TSDFIntegrator and PointCloudMotionCurves. During the visitor play-through
// the live rig is suppressed (StartVisitorPlayback), so without this skip the
// person WATCHING the replay was exported alongside their own take — the
// "second self" ghost in every glb/usdz.

using System.Collections.Generic;
using UnityEngine;

namespace PointCloud
{
    public static class PointCloudSnapshot
    {
        private const int kObColorPointStride = 24; // 3 floats pos + 3 floats colour

        /// <summary>World-space positions + sRGB colours of every displayed
        /// point-cloud point, after the BoundingVolume region filter, the
        /// decimater's KeepRatio, the capsule person-mask and the floor mask —
        /// the same content filters the display shader runs. Deterministic for
        /// a given <paramref name="seed"/>. Runs a synchronous GPU readback —
        /// main thread only.</summary>
        public static void Gather(BoundingVolume bounds, float keepRatio, int seed,
                                  out Vector3[] pos, out Vector3[] col,
                                  PointCloudCapsuleFilter capsules = null,
                                  PointCloudFloorMask floorMask = null)
        {
            var pl = new List<Vector3>(1 << 16);
            var cl = new List<Vector3>(1 << 16);
            var rng = new System.Random(seed);
            keepRatio = Mathf.Clamp01(keepRatio);

            bool useBounds = bounds != null && bounds.Mode != BoundingVolume.FilterMode.Disabled;
            Matrix4x4 worldToBox = useBounds ? bounds.transform.worldToLocalMatrix : Matrix4x4.identity;
            bool keepInside = useBounds && bounds.Mode == BoundingVolume.FilterMode.KeepInside;

            // Capsule person-mask (PassCapsules). Mode 1 keeps inside the
            // capsule union, mode 2 keeps outside; radius rides in CapsuleA.w.
            // Deliberately active even with ZERO capsules — the shader is too:
            // KeepInside with an empty union (tracking gap, feeder cleared)
            // hides the whole cloud on screen, so the export must go empty as
            // well, not fall back to the unmasked cloud.
            bool useCaps = capsules != null
                && capsules.Mode != PointCloudCapsuleFilter.FilterMode.Disabled;
            bool capsKeepInside = useCaps
                && capsules.Mode == PointCloudCapsuleFilter.FilterMode.KeepInside;
            int capsCount = useCaps ? capsules.CapsuleCount : 0;
            Vector4[] capsA = useCaps ? capsules.CapsuleA : null;
            Vector4[] capsB = useCaps ? capsules.CapsuleB : null;

            // Floor mask (PassFloor): below FloorHeight only points within
            // KeepRadius (XZ) of a tracked foot survive. A live mask with zero
            // feet hides the whole floor — intended, same as the display.
            bool useFloor = floorMask != null && floorMask.isActiveAndEnabled
                && floorMask.MaskActive;
            float floorY = useFloor ? floorMask.FloorHeight : float.NegativeInfinity;
            float footR2 = useFloor ? floorMask.KeepRadius * floorMask.KeepRadius : 0f;
            int footCount = useFloor ? Mathf.Min(floorMask.FootCount, PointCloudFloorMask.MaxFeet) : 0;
            Vector4[] feet = useFloor ? floorMask.Feet : null;

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
                    if (useCaps && InCapsuleUnion(w, capsA, capsB, capsCount) != capsKeepInside) continue;
                    if (useFloor && w.y < floorY && !NearAnyFoot(w, feet, footCount, footR2)) continue;
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

        private static bool InCapsuleUnion(Vector3 w, Vector4[] a, Vector4[] b, int n)
        {
            for (int i = 0; i < n; i++)
            {
                Vector3 ca = a[i];
                Vector3 cb = b[i];
                float r = a[i].w;
                Vector3 ab = cb - ca;
                float ab2 = Vector3.Dot(ab, ab);
                Vector3 closest = ab2 < 1e-12f
                    ? ca
                    : ca + Mathf.Clamp01(Vector3.Dot(w - ca, ab) / ab2) * ab;
                if ((w - closest).sqrMagnitude <= r * r) return true;
            }
            return false;
        }

        private static bool NearAnyFoot(Vector3 w, Vector4[] feet, int n, float r2)
        {
            for (int i = 0; i < n; i++)
            {
                float dx = w.x - feet[i].x, dz = w.z - feet[i].z;
                if (dx * dx + dz * dz <= r2) return true;
            }
            return false;
        }

        // Every live PointCloudRenderer / _Playback_* mesh exposes its vertex
        // buffer 0 as a Raw ObColorPoint buffer — the same test IsUsableSource
        // uses to seed the motion curves. Anything else (CPU meshes, unrelated
        // geometry) is rejected so the raw read stays valid. Renderers flagged
        // suppressAsSource (the live rig during a visitor play-through) are
        // skipped, matching TSDFIntegrator / PointCloudMotionCurves.
        private static IEnumerable<MeshFilter> FindPointCloudMeshes()
        {
            foreach (var mf in Object.FindObjectsByType<MeshFilter>(FindObjectsSortMode.None))
            {
                var m = mf.sharedMesh;
                if (m == null || m.vertexCount == 0) continue;
                if ((m.vertexBufferTarget & GraphicsBuffer.Target.Raw) == 0) continue;
                if (m.GetVertexBufferStride(0) != kObColorPointStride) continue;
                if (mf.TryGetComponent(out PointCloudRenderer r) && r.suppressAsSource) continue;
                yield return mf;
            }
        }
    }
}
