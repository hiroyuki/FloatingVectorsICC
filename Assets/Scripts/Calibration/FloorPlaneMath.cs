// Floor-plane levelling math for the interactive "click 3+ floor points → make
// the floor horizontal at y=0" tune (see CalibrationRuntimeUI floor tune).
//
// All of this operates in Unity WORLD space and is decoupled from the extrinsics
// / camera-based WorldFrameRebase: the operator picks floor points in the world
// they currently see (already rebased + already levelled), we fit a plane, and
// produce a world-space correction Pose that is LEFT-composed after the rebase
// — exactly like WorldFrameRebase's rebase Pose is left-composed after
// ToUnityLocal. Composing another world Pose in front is therefore legal and
// keeps the localScale Y-flip untouched (we never touch per-renderer scale).
//
// Pure, deterministic, no Unity object state — unit-testable.

using System.Collections.Generic;
using UnityEngine;

namespace Calibration
{
    public static class FloorPlaneMath
    {
        // Reject a fit whose points are (near) collinear / coincident: the largest
        // covariance sub-determinant must clear this, otherwise the normal is noise.
        private const float kMinDeterminant = 1e-9f;
        private const float kMaxPositionMagnitude = 100f; // sanity guard (m)

        /// <summary>
        /// Least-squares plane fit through <paramref name="pts"/> (≥3, not
        /// collinear). Returns a point on the plane (the centroid) and the unit
        /// normal. Uses the robust determinant-weighted method (no eigen solver):
        /// pick the covariance axis with the largest sub-determinant and read the
        /// normal off it, so a floor that is nearly axis-aligned in any direction
        /// stays well-conditioned. The normal sign is arbitrary here — callers that
        /// need "up" should re-orient (see <see cref="ComputeLeveling"/>).
        /// </summary>
        public static bool TryFitPlane(IReadOnlyList<Vector3> pts, out Vector3 point, out Vector3 normal)
        {
            point = Vector3.zero;
            normal = Vector3.up;
            if (pts == null || pts.Count < 3) return false;

            Vector3 centroid = Vector3.zero;
            for (int i = 0; i < pts.Count; i++)
            {
                // Reject non-finite or implausibly-far picks (e.g. an invalid-depth
                // sentinel that slipped through) before they poison the covariance.
                if (!IsFinite(pts[i]) || pts[i].magnitude > kMaxPositionMagnitude) return false;
                centroid += pts[i];
            }
            centroid /= pts.Count;

            // Symmetric covariance accumulation (of centered points).
            float xx = 0f, xy = 0f, xz = 0f, yy = 0f, yz = 0f, zz = 0f;
            for (int i = 0; i < pts.Count; i++)
            {
                Vector3 r = pts[i] - centroid;
                xx += r.x * r.x; xy += r.x * r.y; xz += r.x * r.z;
                yy += r.y * r.y; yz += r.y * r.z; zz += r.z * r.z;
            }

            float detX = yy * zz - yz * yz;
            float detY = xx * zz - xz * xz;
            float detZ = xx * yy - xy * xy;
            float detMax = Mathf.Max(detX, Mathf.Max(detY, detZ));
            if (detMax < kMinDeterminant) return false; // degenerate (collinear points)

            Vector3 n;
            if (detMax == detX)
                n = new Vector3(detX, xz * yz - xy * zz, xy * yz - xz * yy);
            else if (detMax == detY)
                n = new Vector3(xz * yz - xy * zz, detY, xy * xz - yz * xx);
            else
                n = new Vector3(xy * yz - xz * yy, xy * xz - yz * xx, detZ);

            if (n.sqrMagnitude < 1e-12f || !IsFinite(n)) return false;
            normal = n.normalized;
            point = centroid;
            return true;
        }

        /// <summary>
        /// World-space correction Pose that rotates the plane (given by a point on
        /// it and its unit normal, both in the CURRENT world) so its normal maps to
        /// +Y — the shortest-arc rotation about the world origin — and then shifts Y
        /// so the plane sits at y=0. The normal is first re-oriented to point up
        /// (dot with +Y &gt; 0) so a downward-pointing fit does not flip the world.
        /// XZ translation is deliberately zero (only the tilt and height change), so
        /// the rig stays centred where the camera-based rebase put it.
        ///
        /// Result semantics: <c>levelled = L.rotation * p + L.position</c> for any
        /// world point p.
        /// </summary>
        public static Pose ComputeLeveling(Vector3 planePoint, Vector3 planeNormal)
        {
            Vector3 n = planeNormal;
            if (n.sqrMagnitude < 1e-12f) return Pose.identity;
            n.Normalize();
            if (Vector3.Dot(n, Vector3.up) < 0f) n = -n; // keep the world right-side-up

            Quaternion rot = Quaternion.FromToRotation(n, Vector3.up);
            // Height of the (rotated) plane point becomes the Y offset to null out.
            float yAfter = (rot * planePoint).y;
            return new Pose(new Vector3(0f, -yAfter, 0f), rot);
        }

        /// <summary>
        /// Compose two world-space poses: apply <paramref name="before"/> first,
        /// then <paramref name="after"/>. Equivalent to the 4x4 product
        /// after·before. Used to (a) fold a fresh levelling delta onto the stored
        /// total so repeated picks converge, and (b) fold the total levelling onto
        /// the camera rebase before it is applied.
        /// </summary>
        public static Pose ComposeWorld(in Pose after, in Pose before)
        {
            return new Pose(
                after.rotation * before.position + after.position,
                after.rotation * before.rotation);
        }

        private static bool IsFinite(Vector3 v) =>
            float.IsFinite(v.x) && float.IsFinite(v.y) && float.IsFinite(v.z);

        /// <summary>True when the pose is within tolerance of identity (no tilt, no
        /// shift) — lets callers skip composing a no-op levelling.</summary>
        public static bool IsApproximatelyIdentity(in Pose p)
        {
            return p.position.sqrMagnitude < 1e-10f &&
                   Quaternion.Angle(p.rotation, Quaternion.identity) < 1e-3f;
        }
    }
}
