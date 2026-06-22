// Per-pixel undistortion lookup table for depth back-projection.
//
// The GPU reconstruction path (PointCloudReconstruct.compute) and the SDK
// PointCloudFilter differ in one important way: the SDK filter back-projects
// each depth pixel using the full lens model (intrinsics + Brown-Conrady
// distortion), while the GPU shader historically used only the pinhole terms
// (fx, fy, cx, cy) and dropped the distortion coefficients. On the Femto Bolt
// the radial/tangential distortion is non-negligible toward the frame edges,
// so the GPU point cloud drifted from the (correct) SDK point cloud there.
//
// This builds a lookup table mapping each integer depth pixel (u, v) to the
// *undistorted* normalized camera ray (x/z, y/z), exactly the quantity the
// pinhole formula `((u - cx) / fx, (v - cy) / fy)` approximates when there is
// no distortion. The GPU then recovers the 3D point as `ray * z` — same shape
// as before, one StructuredBuffer fetch instead of two divides.
//
// This mirrors the LUT approach used in the sibling ShiibaNFTUnity project
// (OpenCVUndistortHelper) but is implemented in pure C# (the iterative inverse
// of the rational radial + tangential model, identical to what OpenCV's
// cvUndistortPoints does internally) so the runtime point-cloud path keeps no
// dependency on OpenCV for Unity.
//
// Cost: built once per (intrinsics, distortion, resolution) and cached by the
// caller — only rebuilt when those change, which is effectively never during a
// session. Layout: flattened row-major, index = v * width + u.

using Orbbec;
using UnityEngine;

namespace PointCloud
{
    public static class DepthUndistortLut
    {
        // The Femto Bolt depth lens is wide (fisheye-ish) with strong rational
        // distortion (k1/k4 ~20), so the fixed-point inverse converges slowly:
        // measured round-trip residual is ~0.3px at 8 iters, 3e-4px at 20, and
        // ~1e-11px at 50. Use a generous cap with an early-out on convergence.
        // Cost is paid once per (intrinsics, distortion, resolution).
        private const int MaxIterations = 50;
        private const double ConvergenceEpsSq = 1e-14; // |delta|^2 in normalized coords
        private const float NegligibleCoeff = 1e-9f;

        /// <summary>
        /// Build the undistorted-ray LUT for a depth stream, or return null when
        /// no correction is needed (distortion disabled / all coefficients ~0) so
        /// the caller can keep the cheaper pinhole path with identical results.
        /// Returns null and logs once for distortion models this builder does not
        /// handle (e.g. Kannala-Brandt fisheye) rather than producing wrong geometry.
        /// </summary>
        public static Vector2[] Build(in ObCameraIntrinsic intr, in ObCameraDistortion dist, int width, int height)
        {
            if (width <= 0 || height <= 0) return null;
            if (intr.Fx == 0f || intr.Fy == 0f) return null;

            if (!IsRationalTangential(dist.Model))
            {
                if (HasDistortion(dist))
                {
                    Debug.LogWarning(
                        $"[DepthUndistortLut] Unsupported distortion model {dist.Model} with non-zero " +
                        "coefficients; falling back to pinhole (no undistortion). Point cloud may be " +
                        "distorted toward the frame edges.");
                }
                return null;
            }

            if (!HasDistortion(dist)) return null; // pinhole LUT would be identity — skip the buffer.

            double fx = intr.Fx, fy = intr.Fy, cx = intr.Cx, cy = intr.Cy;
            double k1 = dist.K1, k2 = dist.K2, k3 = dist.K3, k4 = dist.K4, k5 = dist.K5, k6 = dist.K6;
            double p1 = dist.P1, p2 = dist.P2;

            var lut = new Vector2[width * height];
            for (int v = 0; v < height; v++)
            {
                for (int u = 0; u < width; u++)
                {
                    // Distorted normalized coordinates straight off the sensor.
                    double x0 = (u - cx) / fx;
                    double y0 = (v - cy) / fy;

                    // Iteratively invert the forward model to recover the
                    // undistorted normalized ray (matches cvUndistortPoints).
                    double x = x0, y = y0;
                    for (int i = 0; i < MaxIterations; i++)
                    {
                        double r2 = x * x + y * y;
                        double radial = (1.0 + ((k3 * r2 + k2) * r2 + k1) * r2) /
                                        (1.0 + ((k6 * r2 + k5) * r2 + k4) * r2);
                        double deltaX = 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x);
                        double deltaY = p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y;
                        // icdist = 1/radial; (x0 - delta) * icdist
                        double icdist = 1.0 / radial;
                        double nx = (x0 - deltaX) * icdist;
                        double ny = (y0 - deltaY) * icdist;
                        double dx = nx - x, dy = ny - y;
                        x = nx; y = ny;
                        if (dx * dx + dy * dy < ConvergenceEpsSq) break;
                    }

                    lut[v * width + u] = new Vector2((float)x, (float)y);
                }
            }
            return lut;
        }

        // Models whose 8 coefficients are the rational radial (k1..k6) + tangential
        // (p1, p2) layout this inverse expects. The Femto Bolt depth lens reports
        // BrownConradyK6 (the rational, k6-enabled Brown-Conrady model). True
        // KannalaBrandt4 (fisheye, k1..k4 only, no tangential) is NOT one of these
        // — it would need a different (angle-based) inverse, so it is rejected.
        private static bool IsRationalTangential(ObCameraDistortionModel model)
        {
            switch (model)
            {
                case ObCameraDistortionModel.None:                 // treated as pinhole below
                case ObCameraDistortionModel.ModifiedBrownConrady:
                case ObCameraDistortionModel.InverseBrownConrady:
                case ObCameraDistortionModel.BrownConrady:
                case ObCameraDistortionModel.BrownConradyK6:
                    return true;
                default:
                    return false;
            }
        }

        private static bool HasDistortion(in ObCameraDistortion d)
        {
            return Mathf.Abs(d.K1) > NegligibleCoeff || Mathf.Abs(d.K2) > NegligibleCoeff ||
                   Mathf.Abs(d.K3) > NegligibleCoeff || Mathf.Abs(d.K4) > NegligibleCoeff ||
                   Mathf.Abs(d.K5) > NegligibleCoeff || Mathf.Abs(d.K6) > NegligibleCoeff ||
                   Mathf.Abs(d.P1) > NegligibleCoeff || Mathf.Abs(d.P2) > NegligibleCoeff;
        }
    }
}
