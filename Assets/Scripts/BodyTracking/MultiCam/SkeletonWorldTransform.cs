// Converts a k4abt joint (camera-local mm, OpenCV: +x right, +y down, +z fwd)
// into world-space Unity coordinates using the per-camera renderer transform.
//
// Why we do not call rendererTransform.TransformPoint directly:
//   PointCloudRenderer sets transform.localScale.y = -|y| as the per-mesh
//   OpenCV→Unity Y flip for point cloud vertices. K4AmmToUnity already does
//   the same Y flip on the joint position (BodyTrackingShared.K4AmmToUnity).
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
        /// world origin, the result reduces to <see cref="BodyTrackingShared.K4AmmToUnity"/>.
        /// </summary>
        public static Vector3 ToWorld(in k4a_float3_t jointMm, Transform rendererTransform)
            => ToWorld(jointMm, Matrix4x4.identity, rendererTransform);

        /// <summary>
        /// Same as <see cref="ToWorld(in k4a_float3_t, Transform)"/> but first maps the
        /// k4abt joint (which is expressed in the DEPTH camera 3D frame) into the COLOR
        /// camera frame via the depth→color extrinsic. This is required because the point
        /// cloud is reconstructed in color space (PointCloudReconstruct.compute applies the
        /// same R,T), while raw k4abt joints stay in depth space. The Femto Bolt / Azure
        /// Kinect depth and color sensors are tilted ~6° relative to each other, so
        /// omitting this makes the skeleton float ~30–40 cm above the point cloud at a few
        /// metres depth. Pass <see cref="Matrix4x4.identity"/> when no extrinsic is known.
        /// </summary>
        /// <param name="depthToColorMm">
        /// Rigid depth→color transform in the sensor's mm / OpenCV frame
        /// (color_mm = R · depth_mm + T), matching ObCameraParam.Transform.
        /// </param>
        public static Vector3 ToWorld(in k4a_float3_t jointMm, in Matrix4x4 depthToColorMm,
                                       Transform rendererTransform)
        {
            // 0. depth-camera mm → color-camera mm (identity is a no-op).
            Vector3 colorMm = depthToColorMm.MultiplyPoint3x4(
                new Vector3(jointMm.X, jointMm.Y, jointMm.Z));

            // 1. mm/OpenCV → m/Unity-local (Y flip, mm→m); mirrors BodyTrackingShared.K4AmmToUnity.
            Vector3 unityLocal = new Vector3(colorMm.x, -colorMm.y, colorMm.z) * 0.001f;

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
            => ToWorldRotation(orient, Quaternion.identity, rendererTransform);

        /// <summary>
        /// Same as <see cref="ToWorldRotation(in k4a_quaternion_t, Transform)"/> but first
        /// rotates the joint orientation from the DEPTH camera frame into the COLOR camera
        /// frame. <paramref name="depthToColorRotUnity"/> is the depth→color rotation with
        /// the S = diag(1,-1,1) basis change already applied (so it composes directly with
        /// the basis-changed joint quaternion). Pass <see cref="Quaternion.identity"/> when
        /// no extrinsic is known. See <see cref="ToWorld(in k4a_float3_t, in Matrix4x4, Transform)"/>.
        /// </summary>
        public static Quaternion ToWorldRotation(in k4a_quaternion_t orient,
                                                  in Quaternion depthToColorRotUnity,
                                                  Transform rendererTransform)
        {
            // k4a_quaternion_t is wxyz; build a Unity Quaternion (xyzw constructor).
            // Apply S = diag(1,-1,1) basis change to the local rotation: this negates
            // the y component of the axis-angle representation and the y component of
            // the imaginary part. For unit quaternions q = (w, x, y, z):
            //   S * q * S^-1 = (w, x, -y, z) when S = diag(1,-1,1)
            // (derivable from the rotation matrix conjugation in ExtrinsicsApply).
            // depthToColorRotUnity (already basis-changed) left-multiplies to move the
            // joint frame depth→color before the renderer/world transform.
            Quaternion unityLocal =
                (depthToColorRotUnity * new Quaternion(orient.X, -orient.Y, orient.Z, orient.W)).normalized;

            if (rendererTransform == null) return unityLocal;

            // World rotation: parent.rotation * localRotation * unityLocal. We take only
            // localRotation from the renderer (not its world rotation, because that would
            // include localScale.y=-1 reflection and produce a left-handed-rotation issue).
            Quaternion worldFromRendererRot = ParentRotation(rendererTransform) * rendererTransform.localRotation;
            return (worldFromRendererRot * unityLocal).normalized;
        }

        /// <summary>
        /// Build the 4x4 matrix that maps a Unity-camera-local point (i.e. the output of
        /// <see cref="BodyTrackingShared.K4AmmToUnity"/>) to world coordinates, using the
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
