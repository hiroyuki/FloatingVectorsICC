// EditMode tests for the pose classifiers (Phase A of the new visitor
// sequence): star-pose / banzai predicates on synthetic skeletons, metric
// measurement plausibility, personal-adaptation margin scaling, and the
// PoseHoldDetector debounce timing (pure clock, no Unity time).

using BodyTracking;
using Experience;
using NUnit.Framework;
using UnityEngine;

namespace Calibration.Tests
{
    public class PoseClassifiersTests
    {
        const int N = K4ABTConsts.K4ABT_JOINT_COUNT;

        static int J(k4abt_joint_id_t id) => (int)id;

        // Adult-ish skeleton standing at the origin, arms hanging down, feet
        // together. Tests mutate from this base.
        static void MakeBase(out Vector3[] joints, out bool[] valid)
        {
            joints = new Vector3[N];
            valid = new bool[N];
            for (int i = 0; i < N; i++) valid[i] = true;

            joints[J(k4abt_joint_id_t.K4ABT_JOINT_PELVIS)] = new Vector3(0f, 0.95f, 0f);
            joints[J(k4abt_joint_id_t.K4ABT_JOINT_HEAD)] = new Vector3(0f, 1.65f, 0f);
            joints[J(k4abt_joint_id_t.K4ABT_JOINT_SHOULDER_LEFT)] = new Vector3(-0.18f, 1.40f, 0f);
            joints[J(k4abt_joint_id_t.K4ABT_JOINT_SHOULDER_RIGHT)] = new Vector3(0.18f, 1.40f, 0f);
            // Arms hanging: elbow / wrist below the shoulder.
            joints[J(k4abt_joint_id_t.K4ABT_JOINT_ELBOW_LEFT)] = new Vector3(-0.20f, 1.12f, 0f);
            joints[J(k4abt_joint_id_t.K4ABT_JOINT_ELBOW_RIGHT)] = new Vector3(0.20f, 1.12f, 0f);
            joints[J(k4abt_joint_id_t.K4ABT_JOINT_WRIST_LEFT)] = new Vector3(-0.21f, 0.86f, 0f);
            joints[J(k4abt_joint_id_t.K4ABT_JOINT_WRIST_RIGHT)] = new Vector3(0.21f, 0.86f, 0f);
            // Feet together.
            joints[J(k4abt_joint_id_t.K4ABT_JOINT_ANKLE_LEFT)] = new Vector3(-0.08f, 0.08f, 0f);
            joints[J(k4abt_joint_id_t.K4ABT_JOINT_ANKLE_RIGHT)] = new Vector3(0.08f, 0.08f, 0f);
        }

        // Star pose: arms straight out along the shoulder axis, feet spread.
        static void MakeStar(out Vector3[] joints, out bool[] valid)
        {
            MakeBase(out joints, out valid);
            joints[J(k4abt_joint_id_t.K4ABT_JOINT_ELBOW_LEFT)] = new Vector3(-0.46f, 1.42f, 0f);
            joints[J(k4abt_joint_id_t.K4ABT_JOINT_ELBOW_RIGHT)] = new Vector3(0.46f, 1.42f, 0f);
            joints[J(k4abt_joint_id_t.K4ABT_JOINT_WRIST_LEFT)] = new Vector3(-0.72f, 1.44f, 0f);
            joints[J(k4abt_joint_id_t.K4ABT_JOINT_WRIST_RIGHT)] = new Vector3(0.72f, 1.44f, 0f);
            joints[J(k4abt_joint_id_t.K4ABT_JOINT_ANKLE_LEFT)] = new Vector3(-0.30f, 0.08f, 0f);
            joints[J(k4abt_joint_id_t.K4ABT_JOINT_ANKLE_RIGHT)] = new Vector3(0.30f, 0.08f, 0f);
        }

        // ---- star pose ----

        [Test]
        public void StarPose_Detected()
        {
            MakeStar(out var joints, out var valid);
            Assert.IsTrue(PoseClassifiers.IsStarPose(joints, valid));
        }

        [Test]
        public void StarPose_ArmsDown_Rejected()
        {
            MakeBase(out var joints, out var valid); // arms hanging, feet together
            Assert.IsFalse(PoseClassifiers.IsStarPose(joints, valid));
        }

        [Test]
        public void StarPose_FeetTogether_Rejected()
        {
            MakeStar(out var joints, out var valid);
            joints[J(k4abt_joint_id_t.K4ABT_JOINT_ANKLE_LEFT)] = new Vector3(-0.08f, 0.08f, 0f);
            joints[J(k4abt_joint_id_t.K4ABT_JOINT_ANKLE_RIGHT)] = new Vector3(0.08f, 0.08f, 0f);
            Assert.IsFalse(PoseClassifiers.IsStarPose(joints, valid));
        }

        [Test]
        public void StarPose_ArmsForward_Rejected()
        {
            MakeStar(out var joints, out var valid);
            // Both arms pointing forward (+z), straight and level — must fail
            // the along-shoulder-axis requirement.
            joints[J(k4abt_joint_id_t.K4ABT_JOINT_ELBOW_LEFT)] = new Vector3(-0.18f, 1.42f, 0.28f);
            joints[J(k4abt_joint_id_t.K4ABT_JOINT_ELBOW_RIGHT)] = new Vector3(0.18f, 1.42f, 0.28f);
            joints[J(k4abt_joint_id_t.K4ABT_JOINT_WRIST_LEFT)] = new Vector3(-0.18f, 1.44f, 0.54f);
            joints[J(k4abt_joint_id_t.K4ABT_JOINT_WRIST_RIGHT)] = new Vector3(0.18f, 1.44f, 0.54f);
            Assert.IsFalse(PoseClassifiers.IsStarPose(joints, valid));
        }

        [Test]
        public void StarPose_RotatedBody_StillDetected()
        {
            MakeStar(out var joints, out var valid);
            // Rotate the whole skeleton 60° around Y — predicates must be
            // orientation-independent.
            var rot = Quaternion.AngleAxis(60f, Vector3.up);
            for (int i = 0; i < N; i++) joints[i] = rot * joints[i];
            Assert.IsTrue(PoseClassifiers.IsStarPose(joints, valid));
        }

        [Test]
        public void StarPose_MissingWrist_Rejected()
        {
            MakeStar(out var joints, out var valid);
            valid[J(k4abt_joint_id_t.K4ABT_JOINT_WRIST_LEFT)] = false;
            Assert.IsFalse(PoseClassifiers.IsStarPose(joints, valid));
        }

        // ---- banzai ----

        [Test]
        public void Banzai_Detected()
        {
            MakeBase(out var joints, out var valid);
            joints[J(k4abt_joint_id_t.K4ABT_JOINT_WRIST_LEFT)] = new Vector3(-0.20f, 1.95f, 0f);
            joints[J(k4abt_joint_id_t.K4ABT_JOINT_WRIST_RIGHT)] = new Vector3(0.20f, 1.95f, 0f);
            Assert.IsTrue(PoseClassifiers.IsBanzai(joints, valid, VisitorBodyMetrics.Default));
        }

        [Test]
        public void Banzai_OneHandDown_Rejected()
        {
            MakeBase(out var joints, out var valid);
            joints[J(k4abt_joint_id_t.K4ABT_JOINT_WRIST_LEFT)] = new Vector3(-0.20f, 1.95f, 0f);
            // right wrist stays at hip height
            Assert.IsFalse(PoseClassifiers.IsBanzai(joints, valid, VisitorBodyMetrics.Default));
        }

        [Test]
        public void Banzai_MarginScalesWithArmLength()
        {
            MakeBase(out var joints, out var valid);
            // Wrists barely above the head: clears a short-armed margin but not a
            // long-armed one.
            float headY = joints[J(k4abt_joint_id_t.K4ABT_JOINT_HEAD)].y;
            joints[J(k4abt_joint_id_t.K4ABT_JOINT_WRIST_LEFT)] = new Vector3(-0.20f, headY + 0.08f, 0f);
            joints[J(k4abt_joint_id_t.K4ABT_JOINT_WRIST_RIGHT)] = new Vector3(0.20f, headY + 0.08f, 0f);

            var shortArms = new VisitorBodyMetrics { ArmLengthMeters = 0.4f };  // margin 0.06
            var longArms = new VisitorBodyMetrics { ArmLengthMeters = 0.8f };   // margin 0.12
            Assert.IsTrue(PoseClassifiers.IsBanzai(joints, valid, shortArms));
            Assert.IsFalse(PoseClassifiers.IsBanzai(joints, valid, longArms));
        }

        [Test]
        public void Banzai_MissingHead_Rejected()
        {
            MakeBase(out var joints, out var valid);
            joints[J(k4abt_joint_id_t.K4ABT_JOINT_WRIST_LEFT)] = new Vector3(-0.20f, 1.95f, 0f);
            joints[J(k4abt_joint_id_t.K4ABT_JOINT_WRIST_RIGHT)] = new Vector3(0.20f, 1.95f, 0f);
            valid[J(k4abt_joint_id_t.K4ABT_JOINT_HEAD)] = false;
            Assert.IsFalse(PoseClassifiers.IsBanzai(joints, valid, VisitorBodyMetrics.Default));
        }

        // ---- measurement ----

        [Test]
        public void Measure_StarPose_PlausibleValues()
        {
            MakeStar(out var joints, out var valid);
            Assert.IsTrue(PoseClassifiers.TryMeasureMetrics(joints, valid, out var m));
            // Arm path: shoulder(±0.18,1.40)→elbow(±0.46,1.42)→wrist(±0.72,1.44) ≈ 0.54.
            Assert.AreEqual(0.54f, m.ArmLengthMeters, 0.03f);
            Assert.AreEqual(1.44f, m.ArmSpanMeters, 0.03f);
            Assert.AreEqual(0.36f, m.ShoulderWidthMeters, 0.01f);
            Assert.AreEqual(1.57f, m.StandingHeightMeters, 0.03f);
        }

        [Test]
        public void Measure_CollapsedSkeleton_Rejected()
        {
            var joints = new Vector3[N];
            var valid = new bool[N];
            for (int i = 0; i < N; i++) { joints[i] = Vector3.one; valid[i] = true; }
            Assert.IsFalse(PoseClassifiers.TryMeasureMetrics(joints, valid, out _));
        }

        [Test]
        public void Measure_MissingJoint_Rejected()
        {
            MakeStar(out var joints, out var valid);
            valid[J(k4abt_joint_id_t.K4ABT_JOINT_ANKLE_RIGHT)] = false;
            Assert.IsFalse(PoseClassifiers.TryMeasureMetrics(joints, valid, out _));
        }

        // ---- hold detector ----

        [Test]
        public void Hold_TriggersAfterHoldSeconds()
        {
            var h = new PoseHoldDetector(0.5f, 0.2f);
            Assert.IsFalse(h.Update(true, 10.0f));
            Assert.IsFalse(h.Update(true, 10.3f));
            Assert.IsTrue(h.Update(true, 10.5f));
            Assert.IsTrue(h.Update(true, 11.0f)); // stays true while held
        }

        [Test]
        public void Hold_ShortDropout_Forgiven()
        {
            var h = new PoseHoldDetector(0.5f, 0.2f);
            Assert.IsFalse(h.Update(true, 10.0f));
            Assert.IsFalse(h.Update(false, 10.2f)); // dropout 0.1s < tolerance
            Assert.IsFalse(h.Update(true, 10.3f));
            Assert.IsTrue(h.Update(true, 10.5f)); // window survived the dropout
        }

        [Test]
        public void Hold_LongDropout_Resets()
        {
            var h = new PoseHoldDetector(0.5f, 0.2f);
            Assert.IsFalse(h.Update(true, 10.0f));
            Assert.IsFalse(h.Update(false, 10.45f)); // last active 10.0 → 0.45s > 0.2 tolerance → reset
            Assert.IsFalse(h.Update(true, 10.5f));   // window restarts here
            Assert.IsFalse(h.Update(true, 10.9f));
            Assert.IsTrue(h.Update(true, 11.0f));
        }

        [Test]
        public void Hold_ProgressReported()
        {
            var h = new PoseHoldDetector(1.0f, 0.2f);
            h.Update(true, 10.0f);
            h.Update(true, 10.5f);
            Assert.AreEqual(0.5f, h.Progress01, 0.01f);
            h.Reset();
            Assert.AreEqual(0f, h.Progress01, 0.001f);
        }
    }
}
