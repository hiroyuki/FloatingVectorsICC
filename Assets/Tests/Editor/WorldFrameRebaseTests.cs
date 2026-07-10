// EditMode tests for Calibration.WorldFrameRebase (Phase 2,
// Plans/phase2-worldrebase-plan.md): pins the basis sign with a concrete
// four-camera layout, checks invariance under rig motion, rejects degenerate /
// corrupt input, and guards the localScale Y-flip non-interference contract.

using System.Collections.Generic;
using Calibration;
using NUnit.Framework;
using Orbbec;
using UnityEngine;

namespace Calibration.Tests
{
    public class WorldFrameRebaseTests
    {
        // Canonical rig: a 4x4 m square at 1.8 m height, serial order running
        // counter-clockwise seen from above so camera1→2 = +X, camera2→3 = +Z.
        private static readonly Vector3[] kCanonical =
        {
            new Vector3(-2f, 1.8f, -2f), // camera 1
            new Vector3( 2f, 1.8f, -2f), // camera 2
            new Vector3( 2f, 1.8f,  2f), // camera 3
            new Vector3(-2f, 1.8f,  2f), // camera 4
        };

        private static Vector3[] TransformRig(Quaternion rot, Vector3 offset)
        {
            var moved = new Vector3[4];
            for (int i = 0; i < 4; i++) moved[i] = rot * kCanonical[i] + offset;
            return moved;
        }

        [Test]
        public void CanonicalRig_YieldsIdentity()
        {
            Assert.IsTrue(WorldFrameRebase.TryCompute(kCanonical, out var rebase, out string reason), reason);
            Assert.Less(rebase.position.magnitude, 1e-4f);
            Assert.Less(Quaternion.Angle(rebase.rotation, Quaternion.identity), 0.01f);
        }

        [Test]
        public void MovedRig_RebasesBackToCanonical()
        {
            // Concrete expected positions (Codex round-1 requirement): a rig
            // yawed 30° and shifted (5, 0.3, 3) must land exactly back on the
            // canonical layout — floor height (Y) untouched by the rebase.
            var yaw = Quaternion.Euler(0f, 30f, 0f);
            var offset = new Vector3(5f, 0.3f, 3f);
            var moved = TransformRig(yaw, offset);

            Assert.IsTrue(WorldFrameRebase.TryCompute(moved, out var rebase, out string reason), reason);
            for (int i = 0; i < 4; i++)
            {
                Vector3 back = rebase.rotation * moved[i] + rebase.position;
                Vector3 expected = kCanonical[i] + new Vector3(0f, offset.y, 0f); // Y stays as calibrated
                Assert.Less((back - expected).magnitude, 1e-4f,
                    $"camera {i + 1}: got {back}, expected {expected}");
            }
        }

        [Test]
        public void RebaseIsYawAndXZOnly()
        {
            var moved = TransformRig(Quaternion.Euler(0f, -70f, 0f), new Vector3(-3f, 1.2f, 8f));
            Assert.IsTrue(WorldFrameRebase.TryCompute(moved, out var rebase, out _));
            Assert.AreEqual(0f, rebase.position.y, 1e-5f, "floorY 0 must not move the floor height");
            var e = rebase.rotation.eulerAngles;
            Assert.Less(Mathf.DeltaAngle(0f, e.x), 0.01f);
            Assert.Less(Mathf.DeltaAngle(0f, e.z), 0.01f);
        }

        [Test]
        public void FloorY_ShiftsCalibFloorToZero()
        {
            // Cameras 1.8 m above a floor that sits at -0.9 in the calib frame:
            // with floorY = -0.9 the rebased cameras must land at y = 2.7 and a
            // point ON the physical floor must land at y = 0.
            Assert.IsTrue(WorldFrameRebase.TryCompute(kCanonical, out var rebase, out string reason, -0.9f), reason);
            foreach (var cam in kCanonical)
            {
                Vector3 moved = rebase.rotation * cam + rebase.position;
                Assert.AreEqual(1.8f + 0.9f, moved.y, 1e-4f, "camera height above the floor");
            }
            Vector3 floorPoint = rebase.rotation * new Vector3(0.4f, -0.9f, 0.7f) + rebase.position;
            Assert.AreEqual(0f, floorPoint.y, 1e-4f, "calib floor maps to y=0");

            Assert.IsFalse(WorldFrameRebase.TryCompute(kCanonical, out _, out _, float.NaN),
                "non-finite floorY must fail");
        }

        [Test]
        public void SwappedSerialOrder_FailsInsteadOfMirroring()
        {
            // Clockwise numbering (3 and 4 swapped with each other's side):
            // camera2→3 then points -Z while X̂ implies +Z → must FAIL, not warn.
            var clockwise = new[] { kCanonical[3], kCanonical[2], kCanonical[1], kCanonical[0] };
            Assert.IsFalse(WorldFrameRebase.TryCompute(clockwise, out _, out string reason));
            StringAssert.Contains("serial order", reason);
        }

        [Test]
        public void DegenerateAndCorruptInputs_Fail()
        {
            Assert.IsFalse(WorldFrameRebase.TryCompute(null, out _, out _));
            Assert.IsFalse(WorldFrameRebase.TryCompute(new List<Vector3>(kCanonical).GetRange(0, 3), out _, out _));

            var dupe = (Vector3[])kCanonical.Clone();
            dupe[1] = dupe[0] + new Vector3(0.001f, 0f, 0.001f); // p1≈p2 in XZ
            Assert.IsFalse(WorldFrameRebase.TryCompute(dupe, out _, out _));

            var zDegenerate = (Vector3[])kCanonical.Clone();
            zDegenerate[2] = zDegenerate[1] + new Vector3(0f, 0.5f, 0f); // p2≈p3 in XZ
            Assert.IsFalse(WorldFrameRebase.TryCompute(zDegenerate, out _, out _));

            var nan = (Vector3[])kCanonical.Clone();
            nan[2] = new Vector3(float.NaN, 0f, 0f);
            Assert.IsFalse(WorldFrameRebase.TryCompute(nan, out _, out _));

            var absurd = (Vector3[])kCanonical.Clone();
            absurd[3] = new Vector3(0f, 0f, 5000f); // corrupt yaml guard
            Assert.IsFalse(WorldFrameRebase.TryCompute(absurd, out _, out _));
        }

        [Test]
        public void FromCalibrations_ResolvesSerialsAndRejectsBadOrders()
        {
            var cams = new List<(string serial, Vector3 posUnity)>
            {
                ("D", kCanonical[3]), ("B", kCanonical[1]), ("A", kCanonical[0]), ("C", kCanonical[2]),
            };
            var order = new[] { "A", "B", "C", "D" };
            Assert.IsTrue(WorldFrameRebase.TryComputeFromCalibrations(cams, order, out var rebase, out string reason), reason);
            Assert.Less(rebase.position.magnitude, 1e-4f);

            Assert.IsFalse(WorldFrameRebase.TryComputeFromCalibrations(cams, new[] { "A", "B", "C" }, out _, out _));
            Assert.IsFalse(WorldFrameRebase.TryComputeFromCalibrations(cams, new[] { "A", "A", "C", "D" }, out _, out _));
            Assert.IsFalse(WorldFrameRebase.TryComputeFromCalibrations(cams, new[] { "A", "B", "C", "X" }, out _, out _));

            // Duplicate serial in the EXTRINSICS list (corrupt yaml): the rebase
            // would use one entry while appliers may match the other — must fail
            // even when the duplicate isn't first, and regardless of position.
            var dupeCams = new List<(string serial, Vector3 posUnity)>(cams)
            {
                ("B", kCanonical[1] + new Vector3(0.5f, 0f, 0.5f)),
            };
            Assert.IsFalse(WorldFrameRebase.TryComputeFromCalibrations(dupeCams, order, out _, out string dupeReason));
            StringAssert.Contains("more than once", dupeReason);
        }

        // ---- localScale / Y-flip non-interference (Codex round-1 item 4) ----

        private static ObExtrinsic MakeExtrinsic(Vector3 unityPos, Quaternion unityRot)
        {
            // Build the OpenCV-side matrix whose ToUnityLocal image is exactly
            // (unityPos, unityRot): apply the S = diag(1,-1,1) conjugation in
            // reverse and mm scaling.
            var m = Matrix4x4.Rotate(unityRot);
            var ex = new ObExtrinsic
            {
                Rot = new float[9],
                Trans = new float[3]
            };
            // R_ocv[i,j] = s_i * R_unity[i,j] * s_j with s = (1,-1,1)
            float[] s = { 1f, -1f, 1f };
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                    ex.Rot[i * 3 + j] = s[i] * m[i, j] * s[j];
            ex.Trans[0] = unityPos.x * 1000f;
            ex.Trans[1] = -unityPos.y * 1000f;
            ex.Trans[2] = unityPos.z * 1000f;
            return ex;
        }

        [Test]
        public void RebaseApply_LeavesLocalScaleAlone_AndCommutesWithYFlip()
        {
            var unityPos = new Vector3(1.5f, 1.8f, -0.7f);
            var unityRot = Quaternion.Euler(10f, 200f, 0f);
            var ex = MakeExtrinsic(unityPos, unityRot);

            // Sanity: ToUnityLocal round-trips the constructed extrinsic.
            ExtrinsicsApply.ToUnityLocal(ex, out var rtPos, out var rtRot);
            Assert.Less((rtPos - unityPos).magnitude, 1e-4f);
            Assert.Less(Quaternion.Angle(rtRot, unityRot), 0.01f);

            var rebase = new Pose(new Vector3(-2f, 0f, 1f), Quaternion.Euler(0f, 45f, 0f));

            var goFlip = new GameObject("flip");
            var goPlain = new GameObject("plain");
            try
            {
                goFlip.transform.localScale = new Vector3(1f, -1f, 1f); // playback Y-flip
                ExtrinsicsApply.ApplyToTransform(goFlip.transform, ex, rebase);
                ExtrinsicsApply.ApplyToTransform(goPlain.transform, ex, rebase);

                // Scale untouched by the rebase-aware overload.
                Assert.AreEqual(new Vector3(1f, -1f, 1f), goFlip.transform.localScale);
                Assert.AreEqual(Vector3.one, goPlain.transform.localScale);

                // An OpenCV-camera-local mesh point and its pre-flipped twin
                // must land on the same world position through the flipped and
                // unflipped GOs — the rebase must not disturb the Y-flip.
                var camPointOcv = new Vector3(0.2f, 0.35f, 1.1f); // as stored in playback meshes
                var camPointUnity = new Vector3(0.2f, -0.35f, 1.1f); // pre-flipped twin
                Vector3 viaFlip = goFlip.transform.TransformPoint(camPointOcv);
                Vector3 viaPlain = goPlain.transform.TransformPoint(camPointUnity);
                Assert.Less((viaFlip - viaPlain).magnitude, 1e-4f,
                    $"flip path {viaFlip} vs plain path {viaPlain}");

                // And the composed pose matches the hand-computed left-compose.
                Vector3 expectedPos = rebase.rotation * unityPos + rebase.position;
                Assert.Less((goPlain.transform.localPosition - expectedPos).magnitude, 1e-4f);
            }
            finally
            {
                Object.DestroyImmediate(goFlip);
                Object.DestroyImmediate(goPlain);
            }
        }

        [Test]
        public void ParentIsIdentity_Gate()
        {
            var parent = new GameObject("parent");
            try
            {
                Assert.IsTrue(WorldFrameRebase.ParentIsIdentity(null));
                Assert.IsTrue(WorldFrameRebase.ParentIsIdentity(parent.transform));
                parent.transform.position = new Vector3(0f, 0.5f, 0f);
                Assert.IsFalse(WorldFrameRebase.ParentIsIdentity(parent.transform));
                parent.transform.position = Vector3.zero;
                parent.transform.rotation = Quaternion.Euler(0f, 10f, 0f);
                Assert.IsFalse(WorldFrameRebase.ParentIsIdentity(parent.transform));
            }
            finally
            {
                Object.DestroyImmediate(parent);
            }
        }
    }
}
