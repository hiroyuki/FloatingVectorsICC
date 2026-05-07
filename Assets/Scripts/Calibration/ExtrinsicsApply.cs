using Orbbec;
using UnityEngine;

namespace Calibration
{
    /// <summary>
    /// Converts an <see cref="ObExtrinsic"/> stored in the OpenCV camera convention
    /// (+x right, +y down, +z forward, right-handed; <c>Trans</c> in millimeters)
    /// into a Unity-native local position + rotation, applying the basis change
    /// <c>M_unity = S * M_ocv * S^-1</c> with <c>S = diag(1, -1, 1)</c>.
    ///
    /// Per Plans/issue-9-multicam-extrinsic-calibration.md (Transform 規約 → 座標系):
    /// this helper handles the OpenCV→Unity basis change for the *world ← camera*
    /// transform. The per-renderer / per-playback-GO <c>localScale.y = -1</c> is
    /// orthogonal (it converts in-mesh OpenCV camera-local points to Unity); leave
    /// it untouched.
    /// </summary>
    public static class ExtrinsicsApply
    {
        /// <summary>
        /// Decompose an <see cref="ObExtrinsic"/> (mm/ocv) into a Unity local
        /// position (meters) and rotation, applying the similarity basis change.
        /// </summary>
        public static void ToUnityLocal(in ObExtrinsic ocv, out Vector3 position, out Quaternion rotation)
        {
            // Translation: meters in Unity. Apply S = diag(1,-1,1) by negating Y;
            // also convert mm → m.
            position = new Vector3(
                 ocv.Trans[0] * 0.001f,
                -ocv.Trans[1] * 0.001f,
                 ocv.Trans[2] * 0.001f);

            // Rotation: build R_ocv as 3x3 then conjugate by S.
            // S * R * S^T flips the y-row and y-column signs (since S=diag(1,-1,1)).
            //   R'[i,j] = s[i] * R[i,j] * s[j]
            // Equivalent to: negate any element where exactly one of {i,j} == 1.
            float r00 = ocv.Rot[0], r01 = ocv.Rot[1], r02 = ocv.Rot[2];
            float r10 = ocv.Rot[3], r11 = ocv.Rot[4], r12 = ocv.Rot[5];
            float r20 = ocv.Rot[6], r21 = ocv.Rot[7], r22 = ocv.Rot[8];

            float u00 =  r00, u01 = -r01, u02 =  r02;
            float u10 = -r10, u11 =  r11, u12 = -r12;
            float u20 =  r20, u21 = -r21, u22 =  r22;

            // Convert 3x3 row-major rotation matrix to Quaternion. Standard algorithm.
            float trace = u00 + u11 + u22;
            float qw, qx, qy, qz;
            if (trace > 0f)
            {
                float s = Mathf.Sqrt(trace + 1f) * 2f;
                qw = 0.25f * s;
                qx = (u21 - u12) / s;
                qy = (u02 - u20) / s;
                qz = (u10 - u01) / s;
            }
            else if (u00 > u11 && u00 > u22)
            {
                float s = Mathf.Sqrt(1f + u00 - u11 - u22) * 2f;
                qw = (u21 - u12) / s;
                qx = 0.25f * s;
                qy = (u01 + u10) / s;
                qz = (u02 + u20) / s;
            }
            else if (u11 > u22)
            {
                float s = Mathf.Sqrt(1f + u11 - u00 - u22) * 2f;
                qw = (u02 - u20) / s;
                qx = (u01 + u10) / s;
                qy = 0.25f * s;
                qz = (u12 + u21) / s;
            }
            else
            {
                float s = Mathf.Sqrt(1f + u22 - u00 - u11) * 2f;
                qw = (u10 - u01) / s;
                qx = (u02 + u20) / s;
                qy = (u12 + u21) / s;
                qz = 0.25f * s;
            }
            rotation = new Quaternion(qx, qy, qz, qw).normalized;
        }

        /// <summary>
        /// Apply the extrinsic to <paramref name="t"/> as <c>localPosition</c> +
        /// <c>localRotation</c>. Does NOT touch <c>localScale</c> — the per-renderer
        /// Y flip stays.
        /// </summary>
        public static void ApplyToTransform(Transform t, in ObExtrinsic ocv)
        {
            ToUnityLocal(in ocv, out var pos, out var rot);
            t.localPosition = pos;
            t.localRotation = rot;
        }
    }
}
