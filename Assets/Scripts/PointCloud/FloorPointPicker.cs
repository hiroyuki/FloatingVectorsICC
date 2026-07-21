// Runtime picking of a world-space point under the mouse cursor from the live
// point cloud, for the floor-levelling tune (CalibrationRuntimeUI). Live-only on
// purpose — see EnumerateClouds.
//
// The point cloud meshes live entirely on the GPU: PointCloudRenderer fills them
// with SetVertexBufferData (CPU path) or a compute kernel (GPU reconstruction
// path), and neither keeps a CPU vertex copy. So a click has to read the vertex
// buffer back via AsyncGPUReadback. This is expensive, but it only runs on a
// mouse-down during a deliberate calibration step (never per frame), so a
// blocking WaitForCompletion is acceptable and keeps the call synchronous.
//
// Vertex layout is fixed by PointCloudMeshUtil.CreatePointMesh: position float3 +
// color float3 interleaved, 24 bytes/vertex, position at byte offset 0. We read
// the whole active range, project each vertex to screen, and keep the one nearest
// the click that is also frontmost (smallest camera depth) — i.e. the visible
// surface the operator actually clicked on.

using System.Collections.Generic;
using Unity.Collections;
using UnityEngine;
using UnityEngine.Rendering;

namespace PointCloud
{
    public static class FloorPointPicker
    {
        private const int kVertexStride = 24;   // bytes: position float3 + color float3
        private const int kFloatsPerVertex = 6;
        // Reject implausibly-far vertices (e.g. the GPU reconstruction invalid-depth
        // sentinel ~1e10) before they can be picked. A real floor point is metres
        // from the world origin; 100 m matches WorldFrameRebase's sanity guard.
        private const float kMaxWorldMagnitudeSqr = 100f * 100f;

        /// <summary>
        /// Resolve the camera the operator is looking at (display 0 / main). Falls
        /// back through Camera.main → first active camera on display 0 → any camera.
        /// </summary>
        public static Camera ResolvePickCamera()
        {
            if (Camera.main != null && Camera.main.isActiveAndEnabled) return Camera.main;
            var all = Camera.allCameras;
            for (int i = 0; i < all.Length; i++)
                if (all[i] != null && all[i].isActiveAndEnabled && all[i].targetDisplay == 0)
                    return all[i];
            return all.Length > 0 ? all[0] : Camera.main;
        }

        /// <summary>
        /// Pick the world-space point-cloud vertex whose screen projection is
        /// within <paramref name="radiusPx"/> of <paramref name="screenPos"/> and
        /// is frontmost. Returns false when no cloud vertex falls under the cursor
        /// (e.g. the operator clicked empty space) or no readable mesh is active.
        /// </summary>
        public static bool TryPick(Camera cam, Vector2 screenPos, float radiusPx, out Vector3 world)
        {
            world = Vector3.zero;
            if (cam == null) return false;

            float bestScreenDist2 = radiusPx * radiusPx;
            float bestDepth = float.PositiveInfinity;
            bool found = false;

            foreach (var (mesh, l2w, count) in EnumerateClouds())
            {
                if (mesh == null || count <= 0) continue;
                GraphicsBuffer vbuf = null;
                try
                {
                    vbuf = mesh.GetVertexBuffer(0);
                    if (vbuf == null || vbuf.stride != kVertexStride) continue;
                    int n = Mathf.Min(count, vbuf.count);
                    if (n <= 0) continue;

                    var req = AsyncGPUReadback.Request(vbuf, n * kVertexStride, 0);
                    req.WaitForCompletion();
                    if (req.hasError) continue;

                    NativeArray<float> data = req.GetData<float>();
                    int verts = data.Length / kFloatsPerVertex;
                    for (int v = 0; v < verts; v++)
                    {
                        int b = v * kFloatsPerVertex;
                        Vector3 local = new Vector3(data[b], data[b + 1], data[b + 2]);
                        // Reconstruction can leave zeroed / NaN slots in the tail —
                        // skip the origin and any non-finite position.
                        if (!IsFinite(local)) continue;
                        Vector3 wp = l2w.MultiplyPoint3x4(local);
                        // Drop invalid-depth sentinels / garbage before projecting.
                        if (!IsFinite(wp) || wp.sqrMagnitude > kMaxWorldMagnitudeSqr) continue;

                        Vector3 sp = cam.WorldToScreenPoint(wp);
                        if (sp.z <= 0f) continue; // behind the camera
                        float dx = sp.x - screenPos.x;
                        float dy = sp.y - screenPos.y;
                        float d2 = dx * dx + dy * dy;
                        if (d2 > bestScreenDist2) continue;
                        // Among vertices under the cursor, keep the nearest one to
                        // the camera: that is the surface actually clicked.
                        if (sp.z < bestDepth)
                        {
                            bestDepth = sp.z;
                            world = wp;
                            found = true;
                        }
                    }
                }
                finally
                {
                    vbuf?.Dispose();
                }
            }
            return found;
        }

        // Yields (mesh, localToWorld, activePointCount) for every active LIVE
        // PointCloudRenderer. Deliberately live-only: the floor levelling this
        // picker feeds is applied through SensorManager (the live path). Playback
        // (`_Playback_<serial>`) rebases via SensorRecorder, which does not compose
        // the levelling — so picking on playback would save a pose that never gets
        // applied. Matching the picker's scope to the apply scope avoids that false
        // affordance (playback floor levelling is out of scope, exactly as playback
        // already ignores rebaseFloorY / floor.yaml).
        private static IEnumerable<(Mesh mesh, Matrix4x4 l2w, int count)> EnumerateClouds()
        {
            var live = Object.FindObjectsByType<PointCloudRenderer>(FindObjectsSortMode.None);
            for (int i = 0; i < live.Length; i++)
            {
                var pcr = live[i];
                if (pcr == null || !pcr.isActiveAndEnabled) continue;
                var mf = pcr.GetComponent<MeshFilter>();
                var mesh = mf != null ? mf.sharedMesh : null;
                if (mesh == null) continue;
                int count = pcr.LastPointCount > 0 ? pcr.LastPointCount : mesh.vertexCount;
                yield return (mesh, pcr.transform.localToWorldMatrix, count);
            }
        }

        private static bool IsFinite(Vector3 v) =>
            float.IsFinite(v.x) && float.IsFinite(v.y) && float.IsFinite(v.z);
    }
}
