// Derives the person-selection capture volume from the camera rig itself,
// in the fusion world frame (extrinsics "global" frame, OpenCV convention,
// millimetres — the frame RtmPoseAdapter.SetCaptureVolume speaks).
//
// Why this is not a constant: the volume used to be hard-coded as
// center (0, 200, 3000) / half (1100, 1500, 1100), which was really "the rig
// centroid, 3 m in front of the origin camera" — true only while extrinsics
// put one camera at the origin. Once the solve moved to a room-centred world
// frame the cameras sit at the four corners around (0, ·, 0), the constant
// pointed 3 m outside the room, and SelectPerson rejected EVERY detection:
// YOLOX found the visitor on all four cameras, the volume test threw them all
// away, and fusion emitted nothing (live and offline conversion alike).
//
// Deriving from the rig is frame-agnostic — it reproduces the old constant on
// old takes and follows the new frame without a second machine-local file.
// It is the same construction ExperienceSpaceBuilder uses for the sensing
// area (cameras' XZ bounding rectangle, pulled in by a perpendicular inset),
// but with a SMALLER inset on purpose: this box only has to reject bystanders
// standing outside the rig, so it must comfortably CONTAIN the sensing area
// rather than match it — a visitor at the carpet edge must not be rejected.

using System.Collections.Generic;
using PointCloud;
using UnityEngine;

namespace BodyTracking.Eval.Rtmpose
{
    public static class RigCaptureVolume
    {
        /// <summary>Perpendicular clearance (mm) kept between each camera and the
        /// nearest face. Half of ExperienceSpaceBuilder.insetMeters (0.8 m), so
        /// the volume is a superset of the sensing area.</summary>
        public const float DefaultInsetMm = 400f;

        /// <summary>Vertical half-extent (mm) around the cameras' mean height.
        /// Carried over from the original constant (±1500): the volume test only
        /// has to admit a standing person, and the cameras bracket them.</summary>
        public const float DefaultHalfHeightMm = 1500f;

        /// <summary>
        /// XZ bounding rectangle of the cameras' world positions, inset on all four
        /// sides; Y centred on their mean height. Returns false when fewer than two
        /// cameras carry a world transform, or the inset collapses the rectangle —
        /// callers then keep their configured fallback volume.
        /// </summary>
        public static bool TryDerive(IReadOnlyList<PointCloudRecording.DeviceCalibration> calib,
                                     out Vector3 centerMm, out Vector3 halfMm,
                                     float insetMm = DefaultInsetMm,
                                     float halfHeightMm = DefaultHalfHeightMm)
        {
            centerMm = default; halfMm = default;
            if (calib == null) return false;

            float minX = float.MaxValue, maxX = float.MinValue;
            float minZ = float.MaxValue, maxZ = float.MinValue;
            float sumY = 0f;
            int n = 0;
            foreach (var c in calib)
            {
                if (!c.GlobalTrColorCamera.HasValue) continue;
                var t = c.GlobalTrColorCamera.Value.Trans;
                if (t == null || t.Length < 3) continue;
                minX = Mathf.Min(minX, t[0]); maxX = Mathf.Max(maxX, t[0]);
                minZ = Mathf.Min(minZ, t[2]); maxZ = Mathf.Max(maxZ, t[2]);
                sumY += t[1];
                n++;
            }
            if (n < 2) return false;

            minX += insetMm; maxX -= insetMm;
            minZ += insetMm; maxZ -= insetMm;
            // A rig standing closer together than 2*inset would invert the box.
            if (maxX - minX < 100f || maxZ - minZ < 100f) return false;

            centerMm = new Vector3(0.5f * (minX + maxX), sumY / n, 0.5f * (minZ + maxZ));
            halfMm = new Vector3(0.5f * (maxX - minX), halfHeightMm, 0.5f * (maxZ - minZ));
            return true;
        }
    }
}
