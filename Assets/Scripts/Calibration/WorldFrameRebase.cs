// World-frame re-basing for the experience flow (Plans/phase2-worldrebase-plan.md):
// the extrinsics.yaml global frame (calibration-board based) is re-expressed so
// that the origin is the floor projection of the 4-camera centroid and +X runs
// from camera 1 to camera 2 — yaw + XZ translation only, Y-up and the floor
// height stay as calibrated.
//
// Contract (parent plan "合成空間の契約"): the rebase Pose composes AFTER
// ExtrinsicsApply.ToUnityLocal(), in Unity space, left-applied to the local
// pose. Never compose in the OpenCV/mm space — ApplyToTransform deliberately
// leaves the per-GO localScale Y-flip alone, and an OpenCV-side composition
// would interact with that flip. The composed result is written to
// localPosition/localRotation, which equals a world rebase only while the
// parent transform is identity — callers gate on that (group-level, so a rig
// is never left half-rebased).
//
// Pure math, deterministic: live (SensorManager) and playback (SensorRecorder)
// compute the same Pose from the same yaml with no shared state.

using System.Collections.Generic;
using UnityEngine;

namespace Calibration
{
    public static class WorldFrameRebase
    {
        /// <summary>Reject the basis when the computed Ẑ disagrees with the
        /// camera2→3 direction by more than this — a swapped serial order or a
        /// mirrored rig must fail instead of shipping a suspicious frame.</summary>
        public const float MaxZAxisDisagreementDeg = 10f;

        private const float kMinAxisXZ = 0.01f;        // 1 cm: degenerate axis guard
        private const float kMaxPositionMagnitude = 100f; // yaml corruption guard (m)

        /// <summary>
        /// Compute the rebase Pose from the four camera positions (Unity space,
        /// i.e. already through <see cref="ExtrinsicsApply.ToUnityLocal"/>) in
        /// rig serial order 1..4. On success <paramref name="rebase"/> maps the
        /// old world to the new one (yaw + XZ translation, zero Y component).
        /// On failure nothing may be applied — callers keep every camera on the
        /// non-rebased pose (no partial application).
        /// </summary>
        public static bool TryCompute(IReadOnlyList<Vector3> camPosUnity,
                                      out Pose rebase, out string reason)
        {
            rebase = Pose.identity;
            if (camPosUnity == null || camPosUnity.Count != 4)
            { reason = $"need exactly 4 camera positions, got {camPosUnity?.Count ?? 0}"; return false; }

            for (int i = 0; i < 4; i++)
            {
                var p = camPosUnity[i];
                if (!IsFinite(p))
                { reason = $"camera {i + 1} position is not finite: {p}"; return false; }
                if (p.magnitude > kMaxPositionMagnitude)
                { reason = $"camera {i + 1} position implausibly far ({p.magnitude:0.0} m) — corrupt extrinsics?"; return false; }
            }

            Vector3 xAxis = ProjectXZ(camPosUnity[1] - camPosUnity[0]);
            if (xAxis.magnitude < kMinAxisXZ)
            { reason = "cameras 1 and 2 coincide in XZ — cannot define the X axis"; return false; }
            xAxis.Normalize();

            Vector3 zRef = ProjectXZ(camPosUnity[2] - camPosUnity[1]);
            if (zRef.magnitude < kMinAxisXZ)
            { reason = "cameras 2 and 3 coincide in XZ — cannot validate the Z axis"; return false; }
            zRef.Normalize();

            // Orthonormal left-handed basis: Ẑ = X̂ × Ŷ. The camera2→3 direction
            // only VALIDATES the sign — flipping Ẑ to match it would mirror the
            // basis and contradict the X̂ = camera1→2 spec, so disagreement fails.
            Vector3 zAxis = Vector3.Cross(xAxis, Vector3.up);
            float cosTol = Mathf.Cos(MaxZAxisDisagreementDeg * Mathf.Deg2Rad);
            if (Vector3.Dot(zAxis, zRef) < cosTol)
            {
                reason = $"camera2→3 direction is {Vector3.Angle(zAxis, zRef):0.0}° off the implied Z axis " +
                         $"(tolerance {MaxZAxisDisagreementDeg}°) — serial order swapped or mirrored rig?";
                return false;
            }

            Vector3 centroid = (camPosUnity[0] + camPosUnity[1] + camPosUnity[2] + camPosUnity[3]) * 0.25f;
            var rigPos = new Vector3(centroid.x, 0f, centroid.z); // floor projection: Y stays calibrated
            var rigRot = Quaternion.LookRotation(zAxis, Vector3.up);

            var invRot = Quaternion.Inverse(rigRot);
            rebase = new Pose(invRot * -rigPos, invRot);
            reason = null;
            return true;
        }

        /// <summary>
        /// Resolve the rig cameras out of parsed extrinsics by serial and run
        /// <see cref="TryCompute"/>. The single entry point for every call site
        /// (live and playback), so the math is never duplicated.
        /// </summary>
        public static bool TryComputeFromCalibrations(
            IReadOnlyList<(string serial, Vector3 posUnity)> cams,
            IReadOnlyList<string> rigSerialOrder,
            out Pose rebase, out string reason)
        {
            rebase = Pose.identity;
            if (rigSerialOrder == null || rigSerialOrder.Count != 4)
            { reason = $"rigSerialOrder must list exactly 4 serials, got {rigSerialOrder?.Count ?? 0}"; return false; }

            // A duplicated serial in the extrinsics themselves is corrupt input:
            // the rebase would be computed from one entry while the appliers later
            // match a different one — reject up front so nothing gets applied.
            for (int a = 0; a < cams.Count; a++)
                for (int b = a + 1; b < cams.Count; b++)
                    if (cams[a].serial == cams[b].serial)
                    { reason = $"extrinsics list serial {cams[a].serial} more than once — corrupt yaml?"; return false; }

            var ordered = new Vector3[4];
            for (int i = 0; i < 4; i++)
            {
                string serial = rigSerialOrder[i];
                if (string.IsNullOrEmpty(serial))
                { reason = $"rigSerialOrder[{i}] is empty"; return false; }
                for (int j = 0; j < i; j++)
                    if (rigSerialOrder[j] == serial)
                    { reason = $"rigSerialOrder lists serial {serial} twice"; return false; }

                bool found = false;
                for (int c = 0; c < cams.Count; c++)
                {
                    if (cams[c].serial != serial) continue;
                    ordered[i] = cams[c].posUnity;
                    found = true;
                    break;
                }
                if (!found)
                { reason = $"serial {serial} has no extrinsics entry"; return false; }
            }
            return TryCompute(ordered, out rebase, out reason);
        }

        /// <summary>
        /// The one composition point (used by the rebase-aware
        /// <see cref="ExtrinsicsApply.ApplyToTransform(Transform, in Orbbec.ObExtrinsic, in Pose)"/>
        /// overload): left-apply the rebase to a ToUnityLocal pose, Unity space.
        /// </summary>
        public static void ComposeUnityPoseAfterToUnityLocal(
            in Pose unityLocal, in Pose rebase, out Vector3 position, out Quaternion rotation)
        {
            position = rebase.rotation * unityLocal.position + rebase.position;
            rotation = rebase.rotation * unityLocal.rotation;
        }

        /// <summary>
        /// Group-level gate for the localPosition/localRotation contract: the
        /// composed pose equals a world rebase only under an identity parent.
        /// Call once per rig BEFORE applying any rebased pose, so cameras are
        /// never left in a mixed rebased/non-rebased state.
        /// </summary>
        public static bool ParentIsIdentity(Transform parent)
        {
            if (parent == null) return true;
            return parent.position.sqrMagnitude < 1e-8f &&
                   Quaternion.Angle(parent.rotation, Quaternion.identity) < 0.01f &&
                   (parent.lossyScale - Vector3.one).sqrMagnitude < 1e-8f;
        }

        private static Vector3 ProjectXZ(Vector3 v) => new Vector3(v.x, 0f, v.z);

        private static bool IsFinite(Vector3 v) =>
            !(float.IsNaN(v.x) || float.IsNaN(v.y) || float.IsNaN(v.z) ||
              float.IsInfinity(v.x) || float.IsInfinity(v.y) || float.IsInfinity(v.z));
    }
}
