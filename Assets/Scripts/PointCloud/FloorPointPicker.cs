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
        /// Camera the pick must project through, when the caller owns one. Floor
        /// tune sets this to the temporary top-down camera it puts on Display 1 —
        /// the view the operator is actually clicking in. Cleared on tune exit.
        /// </summary>
        public static Camera PickCameraOverride;

        /// <summary>
        /// Resolve the camera the operator is looking at. The override wins; then
        /// Camera.main → first active camera on Display 1 → any camera.
        /// </summary>
        public static Camera ResolvePickCamera()
        {
            if (PickCameraOverride != null && PickCameraOverride.isActiveAndEnabled)
                return PickCameraOverride;
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
        ///
        /// <paramref name="restrictTo"/> confines candidates to that volume's OBB.
        /// Pass the volume the clouds are being clipped to on screen: the meshes
        /// keep their full vertex buffer regardless of the filter, so without this
        /// the pick can land on a wall or the ceiling that the operator cannot see
        /// — and a "floor" fitted to a wall stands the world on its side.
        /// </summary>
        public static bool TryPick(Camera cam, Vector2 screenPos, float radiusPx, out Vector3 world,
                                   BoundingVolume restrictTo = null)
        {
            world = Vector3.zero;
            if (cam == null) return false;

            // Cache the OBB test as a world→unit-cube matrix; inside is |axis| <= 0.5.
            bool clip = restrictTo != null;
            Matrix4x4 toBox = clip ? restrictTo.transform.worldToLocalMatrix : Matrix4x4.identity;

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
                        // Only what the operator can actually see is pickable.
                        if (clip)
                        {
                            Vector3 bp = toBox.MultiplyPoint3x4(wp);
                            if (Mathf.Abs(bp.x) > 0.5f || Mathf.Abs(bp.y) > 0.5f || Mathf.Abs(bp.z) > 0.5f)
                                continue;
                        }

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
        // PointCloudRenderer. Deliberately live-only: the floor is MEASURED against
        // the live rig, which is the one actually pointed at the physical room.
        // SensorRecorder now consumes the same floor.yaml (loadFloorFromCalibration)
        // and composes the levelling into its own rebase, so a pick taken here does
        // reach playback — but picking ON playback would measure a recorded floor
        // and write it back as if it were the room's.
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
