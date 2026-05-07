using System;
using Calibration;
using NUnit.Framework;

namespace Calibration.Tests
{
    public class CalibrationMathTests
    {
        private const double Epsilon = 1e-9;

        // ========= Rigid3d basic algebra =========

        [Test]
        public void Identity_Inverse_IsIdentity()
        {
            var inv = Rigid3d.Identity.Inverse();
            AssertEqual(inv, Rigid3d.Identity);
        }

        [Test]
        public void Translate_Inverse_NegatesTranslation()
        {
            var t = Rigid3d.Translate(1, 2, 3);
            var inv = t.Inverse();
            Assert.AreEqual(-1, inv.Translation[0], Epsilon);
            Assert.AreEqual(-2, inv.Translation[1], Epsilon);
            Assert.AreEqual(-3, inv.Translation[2], Epsilon);
        }

        [Test]
        public void Compose_TranslateThenRotate_AppliesInOrder()
        {
            // a = rotate 90° about Z; b = translate (1, 0, 0).
            var a = Rigid3d.FromAxisAngleRadians(0, 0, Math.PI / 2);
            var b = Rigid3d.Translate(1, 0, 0);
            // a * b applied to origin = a(b(origin)) = a((1,0,0)) = (0, 1, 0).
            var c = Rigid3d.Compose(a, b);
            c.TransformPoint(0, 0, 0, out var px, out var py, out var pz);
            Assert.AreEqual(0, px, 1e-12);
            Assert.AreEqual(1, py, 1e-12);
            Assert.AreEqual(0, pz, 1e-12);
        }

        [Test]
        public void Inverse_Compose_RecoversIdentity()
        {
            var t = MakeTransform(0.4, -0.3, 0.7, new[] { 0.5, -0.8, 1.2 });
            var inv = t.Inverse();
            var roundTrip = Rigid3d.Compose(t, inv);
            AssertEqual(roundTrip, Rigid3d.Identity);
        }

        // ========= Centroid extrinsic solve =========

        [Test]
        public void Centroid_SingleCamera_ReturnsIdentity()
        {
            // Single camera: world origin = camera position; result must be identity.
            var camTrMarker = new[] { MakeTransform(0.1, 0.2, 0.3, new[] { 4.0, -1.0, 2.5 }) };
            var result = CentroidCalibrationMath.SolveGlobalTrColorCamera(camTrMarker);
            Assert.AreEqual(1, result.Length);
            AssertEqual(result[0], Rigid3d.Identity);
        }

        [Test]
        public void Centroid_Cam0_AlwaysHasIdentityRotation()
        {
            // World axes are aligned with camera 0 → result[0].rotation must be identity.
            var camTrMarker = new[]
            {
                MakeTransform(0.4, -0.2, 0.7, new[] { 4.0, -1.0, 2.5 }),
                MakeTransform(-0.1, 0.5, 0.3, new[] { 1.5, 3.0, -0.7 }),
                MakeTransform(0.2, 0.6, -0.4, new[] { -2.0, 1.5, 4.2 }),
            };
            var result = CentroidCalibrationMath.SolveGlobalTrColorCamera(camTrMarker);
            for (int i = 0; i < 9; i++)
                Assert.AreEqual(i % 4 == 0 ? 1.0 : 0.0, result[0].Rotation[i], 1e-12,
                    $"result[0].rotation[{i}] should be identity");
        }

        [Test]
        public void Centroid_OutputCentroid_IsZero()
        {
            // Centroid of result translations must be zero by construction.
            var camTrMarker = new[]
            {
                MakeTransform(0.0, 0.0, 0.0, new[] { 5.0, 7.0, -2.0 }),
                MakeTransform(0.3, 0.0, 0.0, new[] { 1.0, -3.0, 4.0 }),
                MakeTransform(0.0, -0.4, 0.5, new[] { -8.0, 2.0, 0.5 }),
            };
            var result = CentroidCalibrationMath.SolveGlobalTrColorCamera(camTrMarker);
            double cx = 0, cy = 0, cz = 0;
            foreach (var r in result)
            {
                cx += r.Translation[0];
                cy += r.Translation[1];
                cz += r.Translation[2];
            }
            Assert.AreEqual(0, cx, 1e-9, "output centroid x");
            Assert.AreEqual(0, cy, 1e-9, "output centroid y");
            Assert.AreEqual(0, cz, 1e-9, "output centroid z");
        }

        [Test]
        public void Centroid_PairwiseDistance_IsPreserved()
        {
            // Algorithm is a rigid transform — pairwise distances between cameras
            // must equal those in the marker frame.
            var markerTrCam = new[]
            {
                MakeTransform(0.1, 0.0, 0.0, new[] { 1.0, 2.0, 3.0 }),
                MakeTransform(0.0, 0.2, 0.0, new[] { -2.0, 0.5, -1.0 }),
                MakeTransform(0.0, 0.0, 0.3, new[] { 4.0, -2.5, 2.0 }),
            };
            var camTrMarker = new Rigid3d[markerTrCam.Length];
            for (int i = 0; i < markerTrCam.Length; i++)
                camTrMarker[i] = markerTrCam[i].Inverse();

            var result = CentroidCalibrationMath.SolveGlobalTrColorCamera(camTrMarker);

            for (int i = 0; i < markerTrCam.Length; i++)
            {
                for (int j = i + 1; j < markerTrCam.Length; j++)
                {
                    double dMarker = Distance(markerTrCam[i].Translation, markerTrCam[j].Translation);
                    double dWorld  = Distance(result[i].Translation, result[j].Translation);
                    Assert.AreEqual(dMarker, dWorld, 1e-12,
                        $"pairwise distance ({i},{j}) must be preserved");
                }
            }
        }

        [Test]
        public void Centroid_TranslateAllCameras_IsInvariant()
        {
            // Shifting all cameras uniformly must shift the world origin by the same vector,
            // so the cameras' poses in the world frame are invariant.
            var basePoses = new[]
            {
                MakeTransform(0.0, 0.0, 0.0, new[] { 1.0, 0.0, 0.0 }),
                MakeTransform(0.0, 0.0, 0.0, new[] { -1.0, 0.0, 0.0 }),
            };
            var shifted = new Rigid3d[basePoses.Length];
            for (int i = 0; i < basePoses.Length; i++)
            {
                shifted[i] = new Rigid3d(
                    (double[])basePoses[i].Rotation.Clone(),
                    new[] {
                        basePoses[i].Translation[0] + 5.0,
                        basePoses[i].Translation[1] - 3.0,
                        basePoses[i].Translation[2] + 7.0,
                    });
            }

            var camFromBase = new Rigid3d[basePoses.Length];
            var camFromShifted = new Rigid3d[shifted.Length];
            for (int i = 0; i < basePoses.Length; i++)
            {
                camFromBase[i] = basePoses[i].Inverse();
                camFromShifted[i] = shifted[i].Inverse();
            }
            var resBase = CentroidCalibrationMath.SolveGlobalTrColorCamera(camFromBase);
            var resShift = CentroidCalibrationMath.SolveGlobalTrColorCamera(camFromShifted);

            for (int i = 0; i < basePoses.Length; i++)
                AssertEqual(resBase[i], resShift[i]);
        }

        [Test]
        public void Centroid_NonZeroCentroid_SubtractsCentroidFromCameraPositions()
        {
            // Three cameras at marker-frame positions (10, 0, 0), (0, 10, 0), (0, 0, 10).
            // Centroid = (10/3, 10/3, 10/3). After centroid subtraction, camera positions
            // in world frame should be (10 - 10/3, -10/3, -10/3), etc.
            var markerTrCam = new[]
            {
                MakeTransform(0, 0, 0, new[] { 10.0, 0, 0 }),
                MakeTransform(0, 0, 0, new[] { 0.0, 10, 0 }),
                MakeTransform(0, 0, 0, new[] { 0.0, 0, 10 }),
            };
            var camTrMarker = new Rigid3d[markerTrCam.Length];
            for (int i = 0; i < markerTrCam.Length; i++)
                camTrMarker[i] = markerTrCam[i].Inverse();

            var result = CentroidCalibrationMath.SolveGlobalTrColorCamera(camTrMarker);
            double third = 10.0 / 3.0;
            AssertVec(result[0].Translation, 10 - third, -third, -third);
            AssertVec(result[1].Translation, -third, 10 - third, -third);
            AssertVec(result[2].Translation, -third, -third, 10 - third);
        }

        // ========= helpers =========

        private static Rigid3d MakeTransform(double rx, double ry, double rz, double[] t)
        {
            var rot = Rigid3d.FromAxisAngleRadians(rx, ry, rz);
            return new Rigid3d(rot.Rotation, t);
        }

        private static void AssertEqual(Rigid3d actual, Rigid3d expected, double eps = Epsilon)
        {
            for (int i = 0; i < 9; i++)
                Assert.AreEqual(expected.Rotation[i], actual.Rotation[i], eps,
                    $"rotation[{i}] mismatch");
            for (int i = 0; i < 3; i++)
                Assert.AreEqual(expected.Translation[i], actual.Translation[i], eps,
                    $"translation[{i}] mismatch");
        }

        private static void AssertVec(double[] v, double x, double y, double z, double eps = 1e-12)
        {
            Assert.AreEqual(x, v[0], eps, "vec.x");
            Assert.AreEqual(y, v[1], eps, "vec.y");
            Assert.AreEqual(z, v[2], eps, "vec.z");
        }

        private static double Distance(double[] a, double[] b)
        {
            double dx = a[0] - b[0], dy = a[1] - b[1], dz = a[2] - b[2];
            return Math.Sqrt(dx * dx + dy * dy + dz * dz);
        }

        private static void AssertCentroidIsZero(Rigid3d[] poses)
        {
            double cx = 0, cy = 0, cz = 0;
            foreach (var p in poses)
            {
                cx += p.Translation[0];
                cy += p.Translation[1];
                cz += p.Translation[2];
            }
            Assert.AreEqual(0, cx, 1e-12, "test setup: centroid x must be 0");
            Assert.AreEqual(0, cy, 1e-12, "test setup: centroid y must be 0");
            Assert.AreEqual(0, cz, 1e-12, "test setup: centroid z must be 0");
        }
    }
}
