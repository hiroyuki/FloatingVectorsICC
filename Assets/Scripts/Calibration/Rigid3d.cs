using System;

namespace Calibration
{
    /// <summary>
    /// Rigid-body transform (rotation + translation) in double precision.
    /// Notation: <c>A_tr_B</c> transforms a point from frame B into frame A
    /// (per project convention defined in Plans/issue-9-multicam-extrinsic-calibration.md).
    /// Rotation is row-major 3x3 (R[0..8] = R00, R01, R02, R10, R11, R12, R20, R21, R22).
    /// </summary>
    public readonly struct Rigid3d
    {
        public readonly double[] Rotation;    // length 9, row-major
        public readonly double[] Translation; // length 3

        public Rigid3d(double[] rotation, double[] translation)
        {
            if (rotation == null || rotation.Length != 9)
                throw new ArgumentException("rotation must be length 9 (row-major 3x3)", nameof(rotation));
            if (translation == null || translation.Length != 3)
                throw new ArgumentException("translation must be length 3", nameof(translation));
            Rotation = rotation;
            Translation = translation;
        }

        public static Rigid3d Identity => new Rigid3d(
            new[] { 1.0, 0, 0, 0, 1, 0, 0, 0, 1 },
            new[] { 0.0, 0, 0 });

        public static Rigid3d Translate(double tx, double ty, double tz) => new Rigid3d(
            new[] { 1.0, 0, 0, 0, 1, 0, 0, 0, 1 },
            new[] { tx, ty, tz });

        /// <summary>
        /// Inverse of a rigid-body transform: <c>R' = R^T, t' = -R^T * t</c>.
        /// </summary>
        public Rigid3d Inverse()
        {
            double[] rt = new double[9];
            // Transpose rotation.
            rt[0] = Rotation[0]; rt[1] = Rotation[3]; rt[2] = Rotation[6];
            rt[3] = Rotation[1]; rt[4] = Rotation[4]; rt[5] = Rotation[7];
            rt[6] = Rotation[2]; rt[7] = Rotation[5]; rt[8] = Rotation[8];

            // -R^T * t.
            double[] tInv = new double[3];
            for (int r = 0; r < 3; r++)
            {
                tInv[r] = -(rt[r * 3 + 0] * Translation[0]
                          + rt[r * 3 + 1] * Translation[1]
                          + rt[r * 3 + 2] * Translation[2]);
            }
            return new Rigid3d(rt, tInv);
        }

        /// <summary>
        /// Composition: <c>A_tr_C = A_tr_B * B_tr_C</c>. Equivalent to applying
        /// <paramref name="b"/> first, then <paramref name="a"/>.
        /// </summary>
        public static Rigid3d Compose(Rigid3d a, Rigid3d b)
        {
            double[] r = new double[9];
            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    double s = 0.0;
                    for (int k = 0; k < 3; k++)
                        s += a.Rotation[i * 3 + k] * b.Rotation[k * 3 + j];
                    r[i * 3 + j] = s;
                }
            }
            double[] t = new double[3];
            for (int i = 0; i < 3; i++)
            {
                t[i] = a.Rotation[i * 3 + 0] * b.Translation[0]
                     + a.Rotation[i * 3 + 1] * b.Translation[1]
                     + a.Rotation[i * 3 + 2] * b.Translation[2]
                     + a.Translation[i];
            }
            return new Rigid3d(r, t);
        }

        /// <summary>Apply this transform to a 3D point: <c>p' = R*p + t</c>.</summary>
        public void TransformPoint(double px, double py, double pz, out double qx, out double qy, out double qz)
        {
            qx = Rotation[0] * px + Rotation[1] * py + Rotation[2] * pz + Translation[0];
            qy = Rotation[3] * px + Rotation[4] * py + Rotation[5] * pz + Translation[1];
            qz = Rotation[6] * px + Rotation[7] * py + Rotation[8] * pz + Translation[2];
        }

        /// <summary>
        /// Build a rotation-only Rigid3d from axis-angle (radians) using Rodrigues' formula.
        /// Provided as a test convenience; production code uses <c>Calib3d.Rodrigues</c>.
        /// </summary>
        public static Rigid3d FromAxisAngleRadians(double ax, double ay, double az)
        {
            double theta = Math.Sqrt(ax * ax + ay * ay + az * az);
            if (theta < 1e-12) return Identity;
            double cx = ax / theta, cy = ay / theta, cz = az / theta;
            double c = Math.Cos(theta), s = Math.Sin(theta), C = 1.0 - c;
            double[] r = new double[]
            {
                cx * cx * C + c,       cx * cy * C - cz * s,  cx * cz * C + cy * s,
                cy * cx * C + cz * s,  cy * cy * C + c,       cy * cz * C - cx * s,
                cz * cx * C - cy * s,  cz * cy * C + cx * s,  cz * cz * C + c,
            };
            return new Rigid3d(r, new[] { 0.0, 0, 0 });
        }
    }
}
