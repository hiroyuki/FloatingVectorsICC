// Sensor-space unit conversion constants shared by PointCloud / BodyTracking /
// TSDF. OrbbecSDK and k4abt both report positions in millimetres; Unity world
// space is metres. Compute shaders necessarily replicate this convention in
// HLSL — when editing a kernel, cross-reference these constants rather than
// introducing a new literal.
//
// The K4A/Orbbec -> Unity Y-flip lives in BodyTracking.BodyTrackingShared
// (K4AmmToUnity / UnityToK4Amm), not here: Shared has no reference to the
// k4a struct types.

using UnityEngine;

namespace Shared
{
    public static class Units
    {
        public const float MmToM = 0.001f;
        public const float MToMm = 1000f;

        /// <summary>Scale matrix metres -> millimetres (world m to sensor mm).</summary>
        public static readonly Matrix4x4 MToMmScale =
            Matrix4x4.Scale(new Vector3(1000f, 1000f, 1000f));

        /// <summary>Scale matrix millimetres -> metres (sensor mm to world m).</summary>
        public static readonly Matrix4x4 MmToMScale =
            Matrix4x4.Scale(new Vector3(0.001f, 0.001f, 0.001f));
    }
}
