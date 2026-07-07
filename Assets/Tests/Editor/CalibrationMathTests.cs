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

    }
}
