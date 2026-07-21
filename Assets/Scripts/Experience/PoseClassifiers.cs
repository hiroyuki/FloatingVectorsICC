// Pure pose classification for the visitor experience flow: the calibration
// star pose (大の字 — arms and legs spread) and the banzai pose (both hands
// raised). Operates on world-space joint arrays produced by LiveSkeletonFeed
// (K4ABT joint order, K4ABTConsts.K4ABT_JOINT_COUNT entries) so everything in
// this file is EditMode-testable with synthetic skeletons.
//
// Personal adaptation: the star pose is *self-scaling* (all thresholds derive
// from the same frame's own bone lengths, so it works before any measurement
// exists). While the visitor holds it, TryMeasureMetrics samples the body —
// the resulting VisitorBodyMetrics then personalizes the banzai margin so a
// child's short reach and an adult's long reach trigger equally.

using BodyTracking;
using UnityEngine;

namespace Experience
{
    /// <summary>
    /// Per-visitor body measurements taken during the calibration pose. Only
    /// used to scale pose-detection thresholds (NOT the fusion body profile).
    /// </summary>
    [System.Serializable]
    public struct VisitorBodyMetrics
    {
        /// <summary>Shoulder→elbow→wrist path length in meters, L/R average.</summary>
        public float ArmLengthMeters;
        /// <summary>Wrist-to-wrist distance in meters (arms spread).</summary>
        public float ArmSpanMeters;
        /// <summary>Shoulder-to-shoulder distance in meters.</summary>
        public float ShoulderWidthMeters;
        /// <summary>Head-to-lowest-ankle vertical distance in meters.</summary>
        public float StandingHeightMeters;

        /// <summary>Fallback when the calibration pose was never matched.
        /// Sized between a child and a small adult so neither is badly served.</summary>
        public static VisitorBodyMetrics Default => new VisitorBodyMetrics
        {
            ArmLengthMeters = 0.55f,
            ArmSpanMeters = 1.3f,
            ShoulderWidthMeters = 0.32f,
            StandingHeightMeters = 1.45f,
        };
    }

    public static class PoseClassifiers
    {
        // Joint index shorthands (K4ABT order).
        const int ShL = (int)k4abt_joint_id_t.K4ABT_JOINT_SHOULDER_LEFT;
        const int ShR = (int)k4abt_joint_id_t.K4ABT_JOINT_SHOULDER_RIGHT;
        const int ElL = (int)k4abt_joint_id_t.K4ABT_JOINT_ELBOW_LEFT;
        const int ElR = (int)k4abt_joint_id_t.K4ABT_JOINT_ELBOW_RIGHT;
        const int WrL = (int)k4abt_joint_id_t.K4ABT_JOINT_WRIST_LEFT;
        const int WrR = (int)k4abt_joint_id_t.K4ABT_JOINT_WRIST_RIGHT;
        const int AnL = (int)k4abt_joint_id_t.K4ABT_JOINT_ANKLE_LEFT;
        const int AnR = (int)k4abt_joint_id_t.K4ABT_JOINT_ANKLE_RIGHT;
        const int Head = (int)k4abt_joint_id_t.K4ABT_JOINT_HEAD;
        const int Pelvis = (int)k4abt_joint_id_t.K4ABT_JOINT_PELVIS;

        /// <summary>
        /// Star pose, as FOUR INDEPENDENT LIMB ANGLES: each arm's shoulder→wrist
        /// line against +Y, each leg's pelvis→ankle line against −Y. Nothing is
        /// measured against the shoulder axis or in absolute metres, so the test is
        /// free of body orientation and of body size alike.
        ///
        /// What this pose is FOR is measuring the visitor's limb lengths, so the
        /// only thing it has to guarantee is that every limb is held CLEAR OF THE
        /// TORSO and of the other limbs — nothing about the direction it points.
        /// Hence a single upper bound on the arm angle: banzai (0°), arms forward
        /// or out to the side (90°) and arms low-and-out all measure fine; only an
        /// arm hanging at the side (≈180°), where it overlaps the torso, does not.
        /// A bent elbow is fine for the same reason.
        /// </summary>
        /// <param name="jointsWorld">World-space joints, K4ABT order, length ≥ 32.</param>
        /// <param name="jointValid">Per-joint validity (confidence != NONE).</param>
        /// <param name="armAngleFromUpMaxDeg">Max angle of shoulder→wrist from +Y. 0 = straight up, 90 = horizontal, 180 = hanging.</param>
        /// <param name="legAngleFromDownMinDeg">Min angle of pelvis→ankle from −Y (0 = standing straight).</param>
        public static bool IsStarPose(Vector3[] jointsWorld, bool[] jointValid,
                                      float armAngleFromUpMaxDeg = 135f,
                                      float legAngleFromDownMinDeg = 12f)
        {
            // Elbows and knees are not part of the predicate but ARE part of what
            // the pose exists to measure — an unseen elbow means no arm length.
            if (!AllValid(jointValid, Pelvis, ShL, ShR, ElL, ElR, WrL, WrR, AnL, AnR)) return false;

            if (!ArmAngleOk(jointsWorld, ShL, WrL, armAngleFromUpMaxDeg)) return false;
            if (!ArmAngleOk(jointsWorld, ShR, WrR, armAngleFromUpMaxDeg)) return false;
            if (!LegAngleOk(jointsWorld, AnL, legAngleFromDownMinDeg)) return false;
            if (!LegAngleOk(jointsWorld, AnR, legAngleFromDownMinDeg)) return false;
            return true;
        }

        // Angle of the limb line against an axis, or NaN when the limb is too
        // short to have a meaningful direction (collapsed / mis-tracked joints).
        static float LimbAngle(Vector3 from, Vector3 to, Vector3 axis)
        {
            Vector3 v = to - from;
            return v.magnitude < 0.1f ? float.NaN : Vector3.Angle(v, axis);
        }

        static bool ArmAngleOk(Vector3[] j, int sh, int wr, float maxDeg)
        {
            float a = LimbAngle(j[sh], j[wr], Vector3.up);
            return !float.IsNaN(a) && a <= maxDeg;
        }

        static bool LegAngleOk(Vector3[] j, int ankle, float minDeg)
        {
            float a = LimbAngle(j[Pelvis], j[ankle], Vector3.down);
            return !float.IsNaN(a) && a >= minDeg;
        }

        /// <summary>
        /// Human-readable breakdown of every star-pose condition and the measured
        /// value behind it, for tuning the thresholds against a real visitor. Same
        /// maths as <see cref="IsStarPose"/> — kept adjacent so they stay in step.
        /// </summary>
        public static string DescribeStarPose(Vector3[] jointsWorld, bool[] jointValid,
                                              float armAngleFromUpMaxDeg = 135f,
                                              float legAngleFromDownMinDeg = 12f)
        {
            if (!AllValid(jointValid, Pelvis, ShL, ShR, ElL, ElR, WrL, WrR, AnL, AnR))
            {
                var missing = new System.Text.StringBuilder("invalid joints:");
                void Chk(int idx, string name) { if (!jointValid[idx]) missing.Append(' ').Append(name); }
                Chk(Pelvis, "pelvis");
                Chk(ShL, "shL"); Chk(ShR, "shR"); Chk(ElL, "elL"); Chk(ElR, "elR");
                Chk(WrL, "wrL"); Chk(WrR, "wrR"); Chk(AnL, "anL"); Chk(AnR, "anR");
                return missing.ToString();
            }

            return $"armL {Fmt(LimbAngle(jointsWorld[ShL], jointsWorld[WrL], Vector3.up), 0f, armAngleFromUpMaxDeg)} " +
                   $"armR {Fmt(LimbAngle(jointsWorld[ShR], jointsWorld[WrR], Vector3.up), 0f, armAngleFromUpMaxDeg)} " +
                   $"legL {Fmt(LimbAngle(jointsWorld[Pelvis], jointsWorld[AnL], Vector3.down), legAngleFromDownMinDeg, 180f)} " +
                   $"legR {Fmt(LimbAngle(jointsWorld[Pelvis], jointsWorld[AnR], Vector3.down), legAngleFromDownMinDeg, 180f)} " +
                   $"(arm ≤{armAngleFromUpMaxDeg:0}°, leg ≥{legAngleFromDownMinDeg:0}°)";
        }

        static string Fmt(float deg, float lo, float hi) =>
            float.IsNaN(deg) ? "short!" : $"{deg:0}°{(deg >= lo && deg <= hi ? "" : "!")}";

        /// <summary>
        /// Banzai: both wrists above the head by a margin. The margin scales with the
        /// measured arm length so the trigger effort is comparable across body sizes.
        /// </summary>
        public static bool IsBanzai(Vector3[] jointsWorld, bool[] jointValid,
                                    in VisitorBodyMetrics metrics,
                                    float marginMeters = 0.05f,
                                    float marginArmFraction = 0.15f)
        {
            if (!AllValid(jointValid, Head, WrL, WrR)) return false;
            float margin = Mathf.Max(marginMeters, marginArmFraction * metrics.ArmLengthMeters);
            float headY = jointsWorld[Head].y;
            return jointsWorld[WrL].y > headY + margin
                && jointsWorld[WrR].y > headY + margin;
        }

        /// <summary>
        /// One-frame body measurement. Meant to be sampled repeatedly while the star
        /// pose is held and averaged by the caller. False when a needed joint is
        /// invalid or the values are anatomically implausible.
        /// </summary>
        public static bool TryMeasureMetrics(Vector3[] jointsWorld, bool[] jointValid,
                                             out VisitorBodyMetrics metrics)
        {
            metrics = default;
            if (!AllValid(jointValid, ShL, ShR, ElL, ElR, WrL, WrR, AnL, AnR, Head)) return false;

            float armL = Vector3.Distance(jointsWorld[ShL], jointsWorld[ElL])
                       + Vector3.Distance(jointsWorld[ElL], jointsWorld[WrL]);
            float armR = Vector3.Distance(jointsWorld[ShR], jointsWorld[ElR])
                       + Vector3.Distance(jointsWorld[ElR], jointsWorld[WrR]);
            float lowestAnkleY = Mathf.Min(jointsWorld[AnL].y, jointsWorld[AnR].y);

            metrics = new VisitorBodyMetrics
            {
                ArmLengthMeters = 0.5f * (armL + armR),
                ArmSpanMeters = Vector3.Distance(jointsWorld[WrL], jointsWorld[WrR]),
                ShoulderWidthMeters = Vector3.Distance(jointsWorld[ShL], jointsWorld[ShR]),
                StandingHeightMeters = jointsWorld[Head].y - lowestAnkleY,
            };

            // Plausibility: reject collapsed/exploded skeletons so one bad frame
            // can't poison the averaged session metrics.
            return metrics.ArmLengthMeters is > 0.25f and < 1.2f
                && metrics.ShoulderWidthMeters is > 0.1f and < 0.7f
                && metrics.StandingHeightMeters is > 0.6f and < 2.3f;
        }

        static bool AllValid(bool[] valid, params int[] indices)
        {
            if (valid == null) return false;
            foreach (int i in indices)
                if (i >= valid.Length || !valid[i]) return false;
            return true;
        }
    }

    /// <summary>
    /// Debounced pose hold: the pose must stay active for <c>holdSeconds</c>, with
    /// dropouts up to <c>dropoutToleranceSeconds</c> forgiven (BT confidence flickers).
    /// Pure (caller supplies the clock) so it's EditMode-testable.
    /// </summary>
    public sealed class PoseHoldDetector
    {
        readonly float _holdSeconds;
        readonly float _dropoutTolerance;
        float _activeSince = -1f;
        float _lastActiveAt = -1f;

        public PoseHoldDetector(float holdSeconds, float dropoutToleranceSeconds = 0.2f)
        {
            _holdSeconds = Mathf.Max(0f, holdSeconds);
            _dropoutTolerance = Mathf.Max(0f, dropoutToleranceSeconds);
        }

        /// <summary>0..1 progress toward the hold requirement (for UI feedback).</summary>
        public float Progress01 { get; private set; }

        /// <summary>Feed one classification sample; returns true once the pose has
        /// been held long enough (and keeps returning true while it lasts).</summary>
        public bool Update(bool poseActive, float now)
        {
            if (poseActive)
            {
                if (_activeSince < 0f) _activeSince = now;
                _lastActiveAt = now;
            }
            else if (_activeSince >= 0f && now - _lastActiveAt > _dropoutTolerance)
            {
                Reset();
            }

            if (_activeSince < 0f)
            {
                Progress01 = 0f;
                return false;
            }
            float held = now - _activeSince;
            Progress01 = _holdSeconds <= 0f ? 1f : Mathf.Clamp01(held / _holdSeconds);
            return held >= _holdSeconds;
        }

        public void Reset()
        {
            _activeSince = -1f;
            _lastActiveAt = -1f;
            Progress01 = 0f;
        }
    }
}
