// Unity-side helpers for ObExtrinsic (row-major 3x3 rotation + mm translation,
// see OrbbecNative.ObExtrinsic). Previously each consumer hand-unpacked the
// Rot[9]/Trans[3] arrays into a Matrix4x4 (SkeletonMerger.SetSlotExtrinsic and
// TSDFIntegrator.ComputeDepthFromWorld built the same matrix two different
// ways); this is the single shared implementation.

using UnityEngine;

namespace Orbbec
{
    public static class ObExtrinsicExtensions
    {
        /// <summary>
        /// The extrinsic as a homogeneous transform: row-major 3x3 rotation in
        /// the upper-left, translation (millimetres) in the last column, bottom
        /// row (0,0,0,1). Applies to column vectors in millimetres.
        /// </summary>
        public static Matrix4x4 ToMatrixMm(this in ObExtrinsic e)
        {
            var m = Matrix4x4.identity;
            m.SetRow(0, new Vector4(e.Rot[0], e.Rot[1], e.Rot[2], e.Trans[0]));
            m.SetRow(1, new Vector4(e.Rot[3], e.Rot[4], e.Rot[5], e.Trans[1]));
            m.SetRow(2, new Vector4(e.Rot[6], e.Rot[7], e.Rot[8], e.Trans[2]));
            return m;
        }
    }
}
