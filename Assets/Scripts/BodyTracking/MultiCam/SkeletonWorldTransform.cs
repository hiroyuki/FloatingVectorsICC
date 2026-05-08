// Converts a k4abt joint (camera-local mm, OpenCV: +x right, +y down, +z fwd)
// into world-space Unity coordinates using the per-camera renderer transform.
//
// Why we do not call rendererTransform.TransformPoint directly:
//   PointCloudRenderer sets transform.localScale.y = -|y| as the per-mesh
//   OpenCV→Unity Y flip for point cloud vertices. K4AmmToUnity already does
//   the same Y flip on the joint position (BodyTrackingLive.K4AmmToUnity).
//   Calling rendererTransform.TransformPoint would apply the mesh scale a
//   second time, producing a mirrored result. Instead we compose a transform
//   from localPosition + localRotation + Vector3.one (skipping localScale)
//   and walk up through the parent chain.
//
// See Plans/issue-11-multicam-skeleton-merge.md (座標系 section) and
// Plans/issue-9-multicam-extrinsic-calibration.md (Transform 規約).

using UnityEngine;

namespace BodyTracking.MultiCam
{
    public static class SkeletonWorldTransform
    {
        /// <summary>
        /// Convert a k4abt joint position (mm, OpenCV camera-local) to world space (m, Unity).
        /// When <paramref name="rendererTransform"/> is null or the renderer is at the
        /// world origin, the result reduces to <see cref="BodyTrackingLive.K4AmmToUnity"/>.
        /// </summary>
        public static Vector3 ToWorld(in k4a_float3_t jointMm, Transform rendererTransform)
        {
            // 1. mm/OpenCV → m/Unity-local (Y flip, mm→m).
            Vector3 unityLocal = BodyTrackingLive.K4AmmToUnity(jointMm);

            if (rendererTransform == null) return unityLocal;

            // 2. Compose world ← unity-camera-local using only localPosition/localRotation
            //    of the renderer (skip its localScale, which is the mesh-vertex Y flip
            //    handled separately by the point cloud Mesh and not relevant to skeletons).
            Matrix4x4 worldFromUnityLocal = WorldFromRenderer(rendererTransform);
            return worldFromUnityLocal.MultiplyPoint3x4(unityLocal);
        }

        /// <summary>
        /// Convert a k4abt joint orientation (k4a quaternion, OpenCV camera-local) to a
        /// Unity-world Quaternion. Mirrors the basis change that
        /// <see cref="Calibration.ExtrinsicsApply"/> applies (S * q * S^-1 with
        /// S = diag(1,-1,1)) so the rotated frame matches Unity's left-handed axes.
        /// </summary>
        public static Quaternion ToWorldRotation(in k4a_quaternion_t orient, Transform rendererTransform)
        {
            // k4a_quaternion_t is wxyz; build a Unity Quaternion (xyzw constructor).
            // Apply S = diag(1,-1,1) basis change to the local rotation: this negates
            // the y component of the axis-angle representation and the y component of
            // the imaginary part. For unit quaternions q = (w, x, y, z):
            //   S * q * S^-1 = (w, x, -y, z) when S = diag(1,-1,1)
            // (derivable from the rotation matrix conjugation in ExtrinsicsApply).
            Quaternion unityLocal = new Quaternion(orient.X, -orient.Y, orient.Z, orient.W).normalized;

            if (rendererTransform == null) return unityLocal;

            // World rotation: parent.rotation * localRotation * unityLocal. We take only
            // localRotation from the renderer (not its world rotation, because that would
            // include localScale.y=-1 reflection and produce a left-handed-rotation issue).
            Quaternion worldFromRendererRot = ParentRotation(rendererTransform) * rendererTransform.localRotation;
            return (worldFromRendererRot * unityLocal).normalized;
        }

        /// <summary>
        /// Build the 4x4 matrix that maps a Unity-camera-local point (i.e. the output of
        /// <see cref="BodyTrackingLive.K4AmmToUnity"/>) to world coordinates, using the
        /// renderer's localPosition/localRotation and the parent chain's full
        /// localToWorldMatrix. Renderer's own localScale is intentionally ignored.
        /// </summary>
        private static Matrix4x4 WorldFromRenderer(Transform rendererTransform)
        {
            Matrix4x4 rendererLocal = Matrix4x4.TRS(
                rendererTransform.localPosition,
                rendererTransform.localRotation,
                Vector3.one);

            Transform parent = rendererTransform.parent;
            return parent != null
                ? parent.localToWorldMatrix * rendererLocal
                : rendererLocal;
        }

        private static Quaternion ParentRotation(Transform t)
            => t.parent != null ? t.parent.rotation : Quaternion.identity;
    }
}
