// Whole-cloud floor fitting for the floor-levelling tune (CalibrationRuntimeUI).
//
// Replaces the earlier 3-point click picker. Clicking sampled three vertices and
// fitted a plane through them, which made the result hostage to where the
// operator happened to aim: a miss returned no point at all, and three picks that
// landed on a wall stood the world on its side. Fitting every visible floor point
// instead uses ~35k samples, so the normal is averaged over the whole floor and a
// stray surface cannot dominate it.
//
// Operating assumption: NOBODY IS STANDING IN THE SENSING AREA while this runs.
// The band selection below cannot tell a shoe from the floor, so a person adds
// their feet and shins to the fit. With an empty room that is a non-issue and no
// outlier rejection is needed; with a person in frame the tilt is polluted. The
// on-site procedure is "fit the floor before anyone steps in".
//
// The point clouds live entirely on the GPU (PointCloudRenderer fills them with
// SetVertexBufferData or a compute kernel, neither of which keeps a CPU copy), so
// this reads the vertex buffers back with AsyncGPUReadback. Blocking on the
// readback is fine: this runs on a deliberate calibration keypress, never per
// frame. Vertex layout is fixed by PointCloudMeshUtil.CreatePointMesh: position
// float3 + color float3 interleaved, 24 bytes/vertex, position at offset 0.
//
// Live-only, exactly like the levelling it feeds: SensorManager applies the
// levelling Pose to the live rig, and playback deliberately ignores
// rebaseFloorY / floor.yaml. Fitting on playback would save a pose nothing
// applies.

using System.Collections.Generic;
using Unity.Collections;
using UnityEngine;
using UnityEngine.Rendering;

namespace PointCloud
{
    public static class FloorCloudFitter
    {
        private const int kVertexStride = 24;     // bytes: position float3 + color float3
        private const int kFloatsPerVertex = 6;
        // Reject implausibly-far vertices (e.g. the GPU reconstruction invalid-depth
        // sentinel ~1e10) before they reach the histogram. Matches WorldFrameRebase.
        private const float kMaxWorldMagnitudeSqr = 100f * 100f;
        private const float kCoarseBinMeters = 0.05f;
        // Enough floor to trust a plane through it. A 3 m square at Femto Bolt
        // density yields tens of thousands; this only rules out "almost nothing".
        private const int kMinSamples = 500;

        /// <summary>
        /// Result of a floor fit: the plane (a point on it + upward unit normal) and
        /// the residual statistics that let the operator judge the fit.
        /// </summary>
        public struct Fit
        {
            public Vector3 Point;
            public Vector3 Normal;    // re-oriented to point up
            public int SampleCount;
            public float TiltDeg;     // angle between Normal and +Y
            public float MeanY;       // mean height of the band, metres
            public float SdY;         // std-dev of the band about that mean, metres
            public float CoarseY;     // height the coarse pass located the floor at
        }

        /// <summary>
        /// Locate the floor in the live clouds and fit a plane to it.
        ///
        /// Coarse pass: histogram every point's height and take the densest bin. The
        /// floor is the largest horizontal surface in the room by a wide margin, so
        /// this finds it no matter how far the current calibration has drifted (a
        /// floor sitting at y = +1.57 is exactly the case a fixed band around y = 0
        /// cannot recover from).
        ///
        /// Fine passes: fit the points within <paramref name="bandMeters"/> of that
        /// height, then refit twice against the plane itself, keeping points within
        /// ±2σ of it. This is NOT about people (the procedure says the area is
        /// empty) — it corrects a structural bias. The band is asymmetric by nature:
        /// skirting boards, cable runs, stand feet and depth-noise skirts all sit
        /// ABOVE the floor and nothing sits below it, so a plain mean is pulled up
        /// and the corners — with the longest lever on the normal — tilt the fit.
        /// Trimming against the plane settles it onto the floor proper.
        ///
        /// <paramref name="restrictTo"/> confines samples to the sensing area in XZ,
        /// shrunk by <paramref name="insetMeters"/> so the noisy box edges stay out
        /// of the fit. The box's own rotation is ignored — the region is taken
        /// axis-aligned about the box centre, because a tilted box would carve an
        /// asymmetric footprint out of a floor we are trying to measure. Y is
        /// deliberately NOT clipped by the box: the floor may currently sit far
        /// outside it, which is the situation we are fixing.
        /// </summary>
        public static bool TryFit(BoundingVolume restrictTo, float bandMeters, float insetMeters, out Fit fit)
        {
            fit = default;
            var pts = new List<Vector3>(1 << 16);
            Gather(restrictTo, insetMeters, pts);
            if (pts.Count < kMinSamples) return false;

            if (!TryFindFloorHeight(pts, out float coarseY)) return false;

            var band = new List<Vector3>(pts.Count);
            for (int i = 0; i < pts.Count; i++)
                if (Mathf.Abs(pts[i].y - coarseY) <= bandMeters) band.Add(pts[i]);
            if (band.Count < kMinSamples) return false;

            if (!Calibration.FloorPlaneMath.TryFitPlane(band, out var p, out var n)) return false;
            if (Vector3.Dot(n, Vector3.up) < 0f) n = -n;

            // Trim against the plane and refit. Two rounds is enough: the first
            // sheds the gross above-floor structure, the second the tail.
            double sd = ResidualSd(band, p, n);
            for (int round = 0; round < 2 && sd > 0d; round++)
            {
                float keep = (float)(2d * sd);
                var trimmed = new List<Vector3>(band.Count);
                for (int i = 0; i < band.Count; i++)
                    if (Mathf.Abs(Vector3.Dot(band[i] - p, n)) <= keep) trimmed.Add(band[i]);
                if (trimmed.Count < kMinSamples) break;
                if (!Calibration.FloorPlaneMath.TryFitPlane(trimmed, out var p2, out var n2)) break;
                if (Vector3.Dot(n2, Vector3.up) < 0f) n2 = -n2;
                band = trimmed; p = p2; n = n2;
                sd = ResidualSd(band, p, n);
            }

            double sum = 0d;
            for (int i = 0; i < band.Count; i++) sum += band[i].y;
            double mean = sum / band.Count;

            fit = new Fit
            {
                Point = p,
                Normal = n,
                SampleCount = band.Count,
                TiltDeg = Vector3.Angle(n, Vector3.up),
                MeanY = (float)mean,
                SdY = (float)sd,
                CoarseY = coarseY,
            };
            return true;
        }

        // Std-dev of the perpendicular distances to the plane (not of raw Y): on a
        // tilted floor those differ, and the trim threshold wants the former.
        private static double ResidualSd(List<Vector3> pts, Vector3 p, Vector3 n)
        {
            if (pts.Count == 0) return 0d;
            double s = 0d;
            for (int i = 0; i < pts.Count; i++) { double d = Vector3.Dot(pts[i] - p, n); s += d * d; }
            return System.Math.Sqrt(s / pts.Count);
        }

        // Densest height bin wins, then the estimate is refined to the mean of the
        // points actually in that bin — the bin centre alone carries up to half a
        // bin (2.5 cm) of quantisation error into the band selection.
        private static bool TryFindFloorHeight(List<Vector3> pts, out float floorY)
        {
            floorY = 0f;
            var hist = new Dictionary<int, int>();
            int bestBin = 0, bestCount = 0;
            for (int i = 0; i < pts.Count; i++)
            {
                int bin = Mathf.FloorToInt(pts[i].y / kCoarseBinMeters);
                hist.TryGetValue(bin, out int c);
                c++;
                hist[bin] = c;
                if (c > bestCount) { bestCount = c; bestBin = bin; }
            }
            if (bestCount < kMinSamples) return false;

            double sum = 0d;
            int n = 0;
            for (int i = 0; i < pts.Count; i++)
                if (Mathf.FloorToInt(pts[i].y / kCoarseBinMeters) == bestBin) { sum += pts[i].y; n++; }
            floorY = n > 0 ? (float)(sum / n) : (bestBin + 0.5f) * kCoarseBinMeters;
            return true;
        }

        // Read every active live cloud back from the GPU, transform to world, and
        // keep what falls inside the (inset, axis-aligned) sensing footprint in XZ.
        private static void Gather(BoundingVolume restrictTo, float insetMeters, List<Vector3> into)
        {
            bool clip = restrictTo != null;
            Vector3 centre = Vector3.zero, half = Vector3.zero;
            if (clip)
            {
                var t = restrictTo.transform;
                centre = t.position;
                // Axis-aligned half-extents about the centre, from the box's own
                // size only — see the doc comment on TryFit for why the rotation is
                // intentionally dropped here.
                var s = t.lossyScale;
                half = new Vector3(Mathf.Abs(s.x) * 0.5f - insetMeters, 0f,
                                   Mathf.Abs(s.z) * 0.5f - insetMeters);
                if (half.x <= 0.05f || half.z <= 0.05f) clip = false;  // inset ate the area
            }

            var live = Object.FindObjectsByType<PointCloudRenderer>(FindObjectsSortMode.None);
            for (int i = 0; i < live.Length; i++)
            {
                var pcr = live[i];
                if (pcr == null || !pcr.isActiveAndEnabled) continue;
                var mf = pcr.GetComponent<MeshFilter>();
                var mesh = mf != null ? mf.sharedMesh : null;
                if (mesh == null) continue;
                int count = pcr.LastPointCount > 0 ? pcr.LastPointCount : mesh.vertexCount;
                if (count <= 0) continue;

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
                    Matrix4x4 l2w = pcr.transform.localToWorldMatrix;
                    for (int v = 0; v < verts; v++)
                    {
                        int b = v * kFloatsPerVertex;
                        var local = new Vector3(data[b], data[b + 1], data[b + 2]);
                        // Reconstruction leaves zeroed / NaN slots in the tail.
                        if (!IsFinite(local)) continue;
                        Vector3 wp = l2w.MultiplyPoint3x4(local);
                        if (!IsFinite(wp) || wp.sqrMagnitude > kMaxWorldMagnitudeSqr) continue;
                        if (clip)
                        {
                            // XZ only — see the doc comment on TryFit.
                            if (Mathf.Abs(wp.x - centre.x) > half.x ||
                                Mathf.Abs(wp.z - centre.z) > half.z) continue;
                        }
                        into.Add(wp);
                    }
                }
                finally
                {
                    vbuf?.Dispose();
                }
            }
        }

        private static bool IsFinite(Vector3 v) =>
            float.IsFinite(v.x) && float.IsFinite(v.y) && float.IsFinite(v.z);
    }
}
