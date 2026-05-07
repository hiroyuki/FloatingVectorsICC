using System;
using System.Collections.Generic;
using OpenCVForUnity.ArucoModule;
using OpenCVForUnity.Calib3dModule;
using OpenCVForUnity.CoreModule;
using OpenCVForUnity.ImgprocModule;
using OpenCVForUnity.ObjdetectModule;

namespace Calibration
{
    /// <summary>
    /// Detects a ChArUco board in an RGB8 image and returns the marker pose in the
    /// OpenCV camera frame (+x right, +y down, +z forward, right-handed, meters).
    /// Result transforms are <c>cam_tr_marker</c> per the project transform convention
    /// (Plans/issue-9-multicam-extrinsic-calibration.md → Transform 規約).
    /// </summary>
    public sealed class MarkerPoseEstimator : IDisposable
    {
        private readonly CharucoBoardSpec _spec;
        private readonly Dictionary _dict;
        private readonly CharucoBoard _board;
        private readonly DetectorParameters _detectorParams;
        private bool _disposed;

        public MarkerPoseEstimator(CharucoBoardSpec spec)
        {
            if (spec == null) throw new ArgumentNullException(nameof(spec));
            if (spec.squaresX < 2 || spec.squaresY < 2)
                throw new ArgumentException("squaresX/Y must be >= 2", nameof(spec));
            if (spec.markerLengthMeters >= spec.squareLengthMeters)
                throw new ArgumentException("markerLength must be strictly less than squareLength", nameof(spec));

            _spec = spec;
            _dict = Objdetect.getPredefinedDictionary(spec.OpenCVDictionaryId);
            _board = new CharucoBoard(
                new Size(spec.squaresX, spec.squaresY),
                spec.squareLengthMeters,
                spec.markerLengthMeters,
                _dict);
            _detectorParams = new DetectorParameters();
        }

        /// <summary>
        /// Pose result. <see cref="Success"/> is false when the board could not be
        /// detected with enough corners to estimate a pose. Translation is in meters,
        /// rotation is row-major 3x3, both in the OpenCV camera frame.
        /// </summary>
        public struct Result
        {
            public bool Success;
            public int DetectedMarkerCount;
            public int InterpolatedCornerCount;
            /// <summary>Row-major 3x3 (R[0..8] = R00, R01, R02, R10, ...). OpenCV camera frame.</summary>
            public double[] Rotation;
            /// <summary>(tx, ty, tz) in meters, OpenCV camera frame.</summary>
            public double[] Translation;

            public static Result Failed(int markers, int corners) => new Result
            {
                Success = false,
                DetectedMarkerCount = markers,
                InterpolatedCornerCount = corners,
                Rotation = null,
                Translation = null,
            };
        }

        /// <summary>
        /// Estimate the marker pose. <paramref name="rgb8"/> is row-major
        /// width*height*3 bytes (R, G, B order). Camera matrix is the standard
        /// 3x3 pinhole intrinsic; <paramref name="distortionBrownConrady"/> is
        /// (k1, k2, p1, p2, k3).
        /// </summary>
        public Result Estimate(
            byte[] rgb8,
            int width,
            int height,
            double fx, double fy, double cx, double cy,
            double[] distortionBrownConrady)
        {
            if (_disposed) throw new ObjectDisposedException(nameof(MarkerPoseEstimator));
            if (rgb8 == null) throw new ArgumentNullException(nameof(rgb8));
            if (rgb8.Length != width * height * 3)
                throw new ArgumentException(
                    $"rgb8 length {rgb8.Length} doesn't match width*height*3 = {width * height * 3}");
            if (distortionBrownConrady == null || distortionBrownConrady.Length < 4)
                throw new ArgumentException("distortionBrownConrady must have at least 4 entries (k1, k2, p1, p2)");

            using var rgb = new Mat(height, width, CvType.CV_8UC3);
            rgb.put(0, 0, rgb8);
            using var gray = new Mat();
            Imgproc.cvtColor(rgb, gray, Imgproc.COLOR_RGB2GRAY);

            using var cameraMatrix = new Mat(3, 3, CvType.CV_64FC1);
            cameraMatrix.put(0, 0, new double[]
            {
                fx, 0,  cx,
                0,  fy, cy,
                0,  0,  1
            });

            // Pad to 5 entries (k1, k2, p1, p2, k3); zero-pad if caller supplied fewer.
            double[] dist5 = new double[5];
            for (int i = 0; i < dist5.Length; i++)
                dist5[i] = i < distortionBrownConrady.Length ? distortionBrownConrady[i] : 0.0;
            using var distCoeffs = new Mat(1, 5, CvType.CV_64FC1);
            distCoeffs.put(0, 0, dist5);

            var corners = new List<Mat>();
            using var ids = new Mat();
            try
            {
                Aruco.detectMarkers(gray, _dict, corners, ids, _detectorParams);
                int detectedCount = ids.empty() ? 0 : (int)ids.total();
                if (detectedCount == 0) return Result.Failed(0, 0);

                using var charucoCorners = new Mat();
                using var charucoIds = new Mat();
                int interpolated = Aruco.interpolateCornersCharuco(
                    corners, ids, gray, _board, charucoCorners, charucoIds, cameraMatrix, distCoeffs);
                if (interpolated < 4)
                    return Result.Failed(detectedCount, interpolated);

                using var rvec = new Mat();
                using var tvec = new Mat();
                bool ok = Aruco.estimatePoseCharucoBoard(
                    charucoCorners, charucoIds, _board, cameraMatrix, distCoeffs, rvec, tvec);
                if (!ok) return Result.Failed(detectedCount, interpolated);

                using var rotMat = new Mat();
                Calib3d.Rodrigues(rvec, rotMat);

                double[] rotation = new double[9];
                double[] translation = new double[3];
                rotMat.get(0, 0, rotation);
                tvec.get(0, 0, translation);

                return new Result
                {
                    Success = true,
                    DetectedMarkerCount = detectedCount,
                    InterpolatedCornerCount = interpolated,
                    Rotation = rotation,
                    Translation = translation,
                };
            }
            finally
            {
                foreach (var c in corners) c.Dispose();
            }
        }

        public void Dispose()
        {
            if (_disposed) return;
            _disposed = true;
            _board?.Dispose();
            _dict?.Dispose();
            _detectorParams?.Dispose();
        }
    }
}
