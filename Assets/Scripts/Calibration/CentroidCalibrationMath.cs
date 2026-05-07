using System;
using System.Collections.Generic;

namespace Calibration
{
    /// <summary>
    /// Pure math for the centroid-based world frame used in multi-camera extrinsic
    /// calibration (issue #6 / #9 confirmed spec).
    ///
    /// Given each camera's <c>cam_tr_marker</c> pose:
    /// - World **origin** = centroid of camera positions expressed in the marker frame
    /// - World **rotation** = camera 0's rotation in the marker frame
    ///
    /// Consequences (per issue #6 spec):
    /// - For a single camera, the world frame coincides with the camera frame (identity)
    /// - Camera 0 always has zero rotation in the world frame; its position is the
    ///   offset from the centroid expressed in cam-0-aligned axes
    /// - Other cameras' rotations are expressed relative to cam 0's frame
    /// </summary>
    public static class CentroidCalibrationMath
    {
        /// <summary>
        /// Compute per-camera <c>global_tr_colorCamera</c> from each camera's
        /// <c>cam_tr_marker</c> via centroid-origin world frame.
        /// </summary>
        public static Rigid3d[] SolveGlobalTrColorCamera(IReadOnlyList<Rigid3d> camTrMarker)
        {
            if (camTrMarker == null) throw new ArgumentNullException(nameof(camTrMarker));
            int n = camTrMarker.Count;
            if (n == 0) throw new ArgumentException("camTrMarker is empty", nameof(camTrMarker));

            // marker_tr_camN = inverse(camN_tr_marker). Its translation is the camera
            // origin in marker frame; its rotation is the camera's orientation in marker frame.
            var markerTrCam = new Rigid3d[n];
            double cx = 0, cy = 0, cz = 0;
            for (int i = 0; i < n; i++)
            {
                markerTrCam[i] = camTrMarker[i].Inverse();
                cx += markerTrCam[i].Translation[0];
                cy += markerTrCam[i].Translation[1];
                cz += markerTrCam[i].Translation[2];
            }
            cx /= n; cy /= n; cz /= n;

            // World axes are aligned with camera 0's color-camera frame, so:
            //   world_tr_marker.rotation = (markerTrCam[0].rotation)^T
            //   world_tr_marker.translation = -world_tr_marker.rotation * centroid_marker
            // (so that world's origin lands on the centroid expressed in marker frame).
            double[] worldRot = Transpose(markerTrCam[0].Rotation);
            double tx = -(worldRot[0] * cx + worldRot[1] * cy + worldRot[2] * cz);
            double ty = -(worldRot[3] * cx + worldRot[4] * cy + worldRot[5] * cz);
            double tz = -(worldRot[6] * cx + worldRot[7] * cy + worldRot[8] * cz);
            var worldTrMarker = new Rigid3d(worldRot, new[] { tx, ty, tz });

            var globalTrColorCamera = new Rigid3d[n];
            for (int i = 0; i < n; i++)
                globalTrColorCamera[i] = Rigid3d.Compose(worldTrMarker, markerTrCam[i]);

            return globalTrColorCamera;
        }

        private static double[] Transpose(double[] r)
        {
            return new[]
            {
                r[0], r[3], r[6],
                r[1], r[4], r[7],
                r[2], r[5], r[8],
            };
        }
    }
}
