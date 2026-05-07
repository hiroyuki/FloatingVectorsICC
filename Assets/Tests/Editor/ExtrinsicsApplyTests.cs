using NUnit.Framework;
using Orbbec;
using UnityEngine;

namespace Calibration.Tests
{
    public class ExtrinsicsApplyTests
    {
        [Test]
        public void Identity_ProducesZeroPositionAndIdentityRotation()
        {
            var ocv = new ObExtrinsic
            {
                Rot = new float[] { 1, 0, 0, 0, 1, 0, 0, 0, 1 },
                Trans = new float[] { 0, 0, 0 },
            };
            ExtrinsicsApply.ToUnityLocal(in ocv, out var pos, out var rot);
            Assert.AreEqual(Vector3.zero, pos);
            AssertQuaternionsClose(Quaternion.identity, rot);
        }

        [Test]
        public void Translation_ConvertsMmToMetersAndFlipsY()
        {
            // OpenCV translation (1000, 2000, 3000) mm → Unity (1, -2, 3) m.
            var ocv = new ObExtrinsic
            {
                Rot = new float[] { 1, 0, 0, 0, 1, 0, 0, 0, 1 },
                Trans = new float[] { 1000f, 2000f, 3000f },
            };
            ExtrinsicsApply.ToUnityLocal(in ocv, out var pos, out _);
            Assert.AreEqual(1f, pos.x, 1e-6);
            Assert.AreEqual(-2f, pos.y, 1e-6);
            Assert.AreEqual(3f, pos.z, 1e-6);
        }

        [Test]
        public void XAxisRotation_BasisChange_NegatesAngle()
        {
            // 90° rotation about OCV +x axis. Y/Z swap with sign.
            //   R_ocv (about x by 90°) = | 1  0  0 | | 0 |   | 0  |
            //                            | 0  0 -1 | | 1 | = | 0  |
            //                            | 0  1  0 | | 0 |   | 1  |
            // After basis change S=diag(1,-1,1):
            //   R_unity = S * R * S
            //   = diag(1,-1,1) * R_ocv * diag(1,-1,1)
            // The X axis rotation in Unity (left-handed +y up) appears flipped.
            // We test by rotating a known vector and checking the result.
            float c = Mathf.Cos(Mathf.PI / 2f);
            float s = Mathf.Sin(Mathf.PI / 2f);
            var ocv = new ObExtrinsic
            {
                Rot = new float[]
                {
                    1, 0, 0,
                    0, c, -s,
                    0, s,  c,
                },
                Trans = new float[] { 0, 0, 0 },
            };
            ExtrinsicsApply.ToUnityLocal(in ocv, out _, out var unityRot);

            // OCV: rotating (0, 1, 0) by R gives (0, 0, 1).
            // After S basis change to Unity: (0, 1, 0)_ocv corresponds to (0, -1, 0)_unity (Y flip).
            // The rotation moves it to (0, 0, 1)_ocv → (0, 0, 1)_unity (Z unchanged by S).
            // So in Unity space, applying unityRot to (0, -1, 0) should produce (0, 0, 1).
            Vector3 input = new Vector3(0, -1, 0);
            Vector3 actual = unityRot * input;
            Assert.AreEqual(0f, actual.x, 1e-5, "x");
            Assert.AreEqual(0f, actual.y, 1e-5, "y");
            Assert.AreEqual(1f, actual.z, 1e-5, "z");
        }

        [Test]
        public void NoMirror_RotationStaysProperRotation()
        {
            // Sanity: any proper rotation (det=+1) in OCV should stay proper after the
            // similarity transform (det(S R S^T) = det(R) since det(S) = -1 squared = +1
            // doesn't help, but actually det(S)^2 = 1 and S^T = S^-1 here so det = det(R)).
            // Build a rotation about a random axis; verify the resulting Quaternion is
            // unit and applying it preserves vector length (rigid).
            var ocv = new ObExtrinsic
            {
                Rot = AxisAngleRotation(0.5f, -0.3f, 0.7f, 0.6f),
                Trans = new float[] { 0, 0, 0 },
            };
            ExtrinsicsApply.ToUnityLocal(in ocv, out _, out var unityRot);

            // Quaternion is unit.
            float lenSq = unityRot.x * unityRot.x + unityRot.y * unityRot.y
                        + unityRot.z * unityRot.z + unityRot.w * unityRot.w;
            Assert.AreEqual(1f, lenSq, 1e-5, "rotation must be unit quaternion");

            // Apply to a unit vector: result should still be unit length.
            Vector3 v = new Vector3(0.6f, -0.8f, 0f); // unit
            Vector3 r = unityRot * v;
            Assert.AreEqual(1f, r.magnitude, 1e-5, "rotation must preserve length");
        }

        // ========= helpers =========

        private static float[] AxisAngleRotation(float ax, float ay, float az, float angle)
        {
            float n = Mathf.Sqrt(ax * ax + ay * ay + az * az);
            ax /= n; ay /= n; az /= n;
            float c = Mathf.Cos(angle), s = Mathf.Sin(angle), C = 1f - c;
            return new float[]
            {
                ax * ax * C + c,       ax * ay * C - az * s,  ax * az * C + ay * s,
                ay * ax * C + az * s,  ay * ay * C + c,       ay * az * C - ax * s,
                az * ax * C - ay * s,  az * ay * C + ax * s,  az * az * C + c,
            };
        }

        private static void AssertQuaternionsClose(Quaternion expected, Quaternion actual, float eps = 1e-5f)
        {
            // Quaternions q and -q represent the same rotation; compare both signs.
            float dot = expected.x * actual.x + expected.y * actual.y
                      + expected.z * actual.z + expected.w * actual.w;
            Assert.AreEqual(1f, Mathf.Abs(dot), eps, "quaternions differ");
        }
    }
}
