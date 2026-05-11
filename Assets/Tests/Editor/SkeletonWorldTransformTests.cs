// EditMode tests for SkeletonWorldTransform — the issue #11 helper that
// converts k4abt joint positions/orientations from camera-local mm/OpenCV
// to world-space Unity coordinates.
//
// Critical regression: rendererTransform.localScale.y = -1 (the per-mesh Y
// flip used by PointCloudRenderer) MUST NOT mirror the X axis of skeleton
// world positions, because K4AmmToUnity has already done the OpenCV→Unity
// Y flip. SkeletonWorldTransform.ToWorld must skip the renderer's localScale.

using BodyTracking;
using BodyTracking.MultiCam;
using NUnit.Framework;
using UnityEngine;

namespace Calibration.Tests
{
    public class SkeletonWorldTransformTests
    {
        private static k4a_float3_t Mm(float x, float y, float z) =>
            new k4a_float3_t { X = x, Y = y, Z = z };

        // Comparing Vector3 with Mathf.Approximately is too strict for compose math;
        // a small absolute tolerance is the standard NUnit pattern.
        private const float Eps = 1e-4f;

        private static void AssertApprox(Vector3 expected, Vector3 actual, string ctx)
        {
            Assert.AreEqual(expected.x, actual.x, Eps, ctx + " (x)");
            Assert.AreEqual(expected.y, actual.y, Eps, ctx + " (y)");
            Assert.AreEqual(expected.z, actual.z, Eps, ctx + " (z)");
        }

        [Test]
        public void NullTransform_FallsBackToK4AmmToUnity()
        {
            // jointMm = (1000, -500, 2000) (mm, OpenCV: +x right, +y down, +z fwd)
            // K4AmmToUnity → (1.0, 0.5, 2.0) — Y flipped, mm→m.
            var joint = Mm(1000f, -500f, 2000f);
            Vector3 expected = BodyTrackingLive.K4AmmToUnity(joint);
            Vector3 actual = SkeletonWorldTransform.ToWorld(joint, null);
            AssertApprox(expected, actual, "null transform");
        }

        [Test]
        public void IdentityTransform_FallsBackToK4AmmToUnity()
        {
            var go = new GameObject("identityRenderer");
            try
            {
                var t = go.transform;
                t.localPosition = Vector3.zero;
                t.localRotation = Quaternion.identity;
                t.localScale = Vector3.one;

                var joint = Mm(1000f, -500f, 2000f);
                AssertApprox(BodyTrackingLive.K4AmmToUnity(joint),
                             SkeletonWorldTransform.ToWorld(joint, t),
                             "identity transform");
            }
            finally { Object.DestroyImmediate(go); }
        }

        [Test]
        public void TranslationExtrinsic_ShiftsJointWorldPosition()
        {
            // 1m extrinsic translation along +x must shift the joint world position by +1m on x.
            var go = new GameObject("translatedRenderer");
            try
            {
                var t = go.transform;
                t.localPosition = new Vector3(1f, 0f, 0f);
                t.localRotation = Quaternion.identity;
                t.localScale = Vector3.one;

                var joint = Mm(0f, 0f, 1000f); // 1m straight ahead in camera frame
                Vector3 actual = SkeletonWorldTransform.ToWorld(joint, t);
                AssertApprox(new Vector3(1f, 0f, 1f), actual, "+1m x extrinsic");
            }
            finally { Object.DestroyImmediate(go); }
        }

        [Test]
        public void NegativeYScale_DoesNotMirrorX()
        {
            // The codex review BLOCKING regression: PointCloudRenderer sets
            // localScale.y = -1 on the renderer. SkeletonWorldTransform must NOT
            // apply that scale. We confirm by:
            //   (a) ToWorld produces the K4AmmToUnity result directly when
            //       extrinsic pos/rot are identity, even with localScale.y = -1
            //   (b) TransformPoint on the same renderer would Y-double-flip and
            //       therefore differ — that's the trap we're avoiding.
            var go = new GameObject("flipRenderer");
            try
            {
                var t = go.transform;
                t.localPosition = Vector3.zero;
                t.localRotation = Quaternion.identity;
                t.localScale = new Vector3(1f, -1f, 1f);

                var joint = Mm(500f, -800f, 1500f);
                Vector3 expected = BodyTrackingLive.K4AmmToUnity(joint); // (0.5, 0.8, 1.5)
                Vector3 actual = SkeletonWorldTransform.ToWorld(joint, t);
                AssertApprox(expected, actual, "scale.y=-1 must not change skeleton (a)");

                // Reference 'wrong' path — TransformPoint would Y-flip again. We
                // assert the helper's output differs from the wrong path so a
                // future refactor that accidentally uses TransformPoint fails this test.
                Vector3 wrong = t.TransformPoint(BodyTrackingLive.K4AmmToUnity(joint));
                Assert.AreNotEqual(wrong.y, actual.y,
                    "regression guard: TransformPoint result must differ from helper output");
            }
            finally { Object.DestroyImmediate(go); }
        }

        [Test]
        public void ParentTransform_ChainsThroughLocalToWorld()
        {
            // Parent translated by (10, 0, 0). Renderer localPosition (1, 0, 0).
            // Joint at (0, 0, 1000mm) → unityLocal (0, 0, 1) → renderer-local (1, 0, 1)
            // → world (11, 0, 1).
            var parent = new GameObject("parent");
            var child = new GameObject("child");
            try
            {
                parent.transform.localPosition = new Vector3(10f, 0f, 0f);
                child.transform.SetParent(parent.transform, false);
                child.transform.localPosition = new Vector3(1f, 0f, 0f);
                child.transform.localRotation = Quaternion.identity;
                child.transform.localScale = Vector3.one;

                var joint = Mm(0f, 0f, 1000f);
                Vector3 actual = SkeletonWorldTransform.ToWorld(joint, child.transform);
                AssertApprox(new Vector3(11f, 0f, 1f), actual, "parent chain");
            }
            finally
            {
                Object.DestroyImmediate(child);
                Object.DestroyImmediate(parent);
            }
        }

        [Test]
        public void Rotation90AroundY_RotatesJointInWorld()
        {
            // Camera rotated 90° around Y in world. A joint 1m forward in camera
            // frame should appear 1m to the right in world after rotation.
            var go = new GameObject("rotatedRenderer");
            try
            {
                var t = go.transform;
                t.localPosition = Vector3.zero;
                t.localRotation = Quaternion.Euler(0f, 90f, 0f);
                t.localScale = Vector3.one;

                var joint = Mm(0f, 0f, 1000f); // 1m fwd in camera frame
                Vector3 actual = SkeletonWorldTransform.ToWorld(joint, t);
                AssertApprox(new Vector3(1f, 0f, 0f), actual, "+90deg yaw");
            }
            finally { Object.DestroyImmediate(go); }
        }
    }
}
