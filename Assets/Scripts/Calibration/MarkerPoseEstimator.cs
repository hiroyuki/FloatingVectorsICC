using System;
using System.Collections.Generic;
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
    ///
    /// Implementation note: uses the OpenCV 4.7+ <c>CharucoDetector</c> +
    /// <c>Calib3d.solvePnP</c> pipeline. The legacy <c>Aruco.detectMarkers</c> /
    /// <c>Aruco.interpolateCornersCharuco</c> static methods both SEGV'd inside
    /// the OpenCV-for-Unity native binding when given AprilTag dictionaries
    /// (Editor crashes observed 2026-05-08), so we route everything through the
    /// modern class-based API instead.
    /// </summary>
    public sealed class MarkerPoseEstimator : IDisposable
    {
        private readonly CharucoBoardSpec _spec;
        private readonly Dictionary _dict;
        private readonly CharucoBoard _board;
        private readonly DetectorParameters _detectorParams;
        private readonly RefineParameters _refineParams;
        private readonly CharucoParameters _charucoParams;
        private readonly CharucoDetector _charucoDetector;
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
            // Default minMarkerPerimeterRate (3%) silently drops markers smaller than
            // ~3% of the image perimeter; at 1280x720 with 38 mm AprilTags at 1.5-2 m
            // the markers fall below that cutoff. Lower to 0.005 (≈5 px markers).
            // Empirically observed (Python sweep on dumped frames 2026-05-08): the
            // other AprilTag-specific knobs (aprilTagMinClusterPixels, MaxLineFitMse,
            // QuadSigma) had no effect on detection counts, and switching the corner
            // refinement to CORNER_REFINE_APRILTAG actually hurt — it dropped one
            // camera from 6 markers to 2. So we leave those at their defaults.
            _detectorParams.set_minMarkerPerimeterRate(0.005);
            _refineParams = new RefineParameters();
            _charucoParams = new CharucoParameters();
            _charucoDetector = new CharucoDetector(_board, _charucoParams, _detectorParams, _refineParams);
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
            using var grayRaw = new Mat();
            Imgproc.cvtColor(rgb, grayRaw, Imgproc.COLOR_RGB2GRAY);
            // Global histogram equalization noticeably improves AprilTag detection on
            // frames where the board has uneven brightness (the common indoor case)
            // — empirically lifted N camera from 0 → 2 markers and Z from 6 → 8 markers
            // on the dumped 2026-05-08 frames without harming any other config.
            using var gray = new Mat();
            Imgproc.equalizeHist(grayRaw, gray);

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
            using var distCoeffs = new MatOfDouble(dist5);

            using var charucoCorners = new Mat();   // Nx2 float, image-space chessboard corners
            using var charucoIds = new Mat();       // Nx1 int, indices into board chessboard corner grid
            var markerCornersList = new List<Mat>();
            using var markerIds = new Mat();
            try
            {
                // CharucoDetector runs both ArUco marker detection AND chessboard-corner
                // interpolation in one call. We use the chessboard corners when there are
                // enough of them (sub-pixel accurate), and fall back to the marker corners
                // when the chessboard interpolation didn't get traction (typical for small
                // / oblique boards where AprilTag still decodes 1-2 markers but the dense
                // chessboard pass can't lock).
                _charucoDetector.detectBoard(gray, charucoCorners, charucoIds, markerCornersList, markerIds);

                int detectedMarkers = markerIds.empty() ? 0 : (int)markerIds.total();
                int interpolated = charucoCorners.empty() ? 0 : (int)charucoCorners.total();

                MatOfPoint3f objPoints;
                MatOfPoint2f imgPoints;

                if (interpolated >= 4)
                {
                    // Preferred path: use chessboard corners (sub-pixel accurate).
                    using var allObjPoints = _board.getChessboardCorners(); // Mx3 float, in meters
                    int[] idsArr = new int[interpolated];
                    charucoIds.get(0, 0, idsArr);
                    float[] allObj = new float[(int)allObjPoints.total() * 3];
                    allObjPoints.get(0, 0, allObj);
                    float[] charucoCornersArr = new float[interpolated * 2];
                    charucoCorners.get(0, 0, charucoCornersArr);

                    var objList = new List<OpenCVForUnity.CoreModule.Point3>(interpolated);
                    var imgList = new List<OpenCVForUnity.CoreModule.Point>(interpolated);
                    for (int i = 0; i < interpolated; i++)
                    {
                        int id = idsArr[i];
                        objList.Add(new OpenCVForUnity.CoreModule.Point3(
                            allObj[id * 3 + 0], allObj[id * 3 + 1], allObj[id * 3 + 2]));
                        imgList.Add(new OpenCVForUnity.CoreModule.Point(
                            charucoCornersArr[i * 2 + 0], charucoCornersArr[i * 2 + 1]));
                    }
                    objPoints = new MatOfPoint3f();
                    objPoints.fromList(objList);
                    imgPoints = new MatOfPoint2f();
                    imgPoints.fromList(imgList);
                }
                else if (detectedMarkers >= 1)
                {
                    // Fallback path: marker corners only. Each ArUco marker contributes 4
                    // (2D ↔ 3D) correspondences. solvePnP needs 4 points → one marker is
                    // enough when its 4 corners are well-separated.
                    using var op = new Mat();
                    using var ip = new Mat();
                    _board.matchImagePoints(markerCornersList, markerIds, op, ip);
                    int n = (int)op.total();
                    if (n < 4) return Result.Failed(detectedMarkers, interpolated);
                    objPoints = new MatOfPoint3f(op);
                    imgPoints = new MatOfPoint2f(ip);
                }
                else
                {
                    return Result.Failed(detectedMarkers, interpolated);
                }

                using var _objPoints = objPoints;
                using var _imgPoints = imgPoints;
                using var rvec = new Mat();
                using var tvec = new Mat();
                bool ok = Calib3d.solvePnP(objPoints, imgPoints, cameraMatrix, distCoeffs, rvec, tvec);
                if (!ok) return Result.Failed(detectedMarkers, interpolated);

                using var rotMat = new Mat();
                Calib3d.Rodrigues(rvec, rotMat);

                double[] rotation = new double[9];
                double[] translation = new double[3];
                rotMat.get(0, 0, rotation);
                tvec.get(0, 0, translation);

                return new Result
                {
                    Success = true,
                    DetectedMarkerCount = detectedMarkers,
                    InterpolatedCornerCount = interpolated,
                    Rotation = rotation,
                    Translation = translation,
                };
            }
            finally
            {
                foreach (var m in markerCornersList) m.Dispose();
            }
        }

        public void Dispose()
        {
            if (_disposed) return;
            _disposed = true;
            _charucoDetector?.Dispose();
            _charucoParams?.Dispose();
            _refineParams?.Dispose();
            _detectorParams?.Dispose();
            _board?.Dispose();
            _dict?.Dispose();
        }
    }
}
