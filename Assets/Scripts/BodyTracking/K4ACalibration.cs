// Builds a k4a_calibration_t blob from an Orbbec v2 ObCameraParam so we can
// hand it to k4abt_tracker_create without ever opening the Femto Bolt via
// k4a_device_open (which would conflict with the v2 ob_pipeline that owns
// the device for point cloud capture).
//
// The k4a_calibration_t layout is large (~1032 bytes) and contains a 4x4
// inline array of extrinsics. Rather than declaring the whole struct in C#
// with fixed/unsafe arrays, we allocate native memory and write each piece
// at its known byte offset. Layout sourced from
//   D:\OrbbecSDK_K4A_Wrapper_v2.0.11_windows_202510221441\include\k4a\k4atypes.h
// Re-verify the sizes if you ever upgrade the K4A Wrapper.

using System;
using System.Runtime.InteropServices;
using Orbbec;
using UnityEngine;

namespace BodyTracking
{
    public static class K4ACalibration
    {
        // Sizes (bytes) — keep in sync with k4atypes.h.
        private const int SizeofExtrinsics = 48;          // float[9] + float[3]
        private const int SizeofIntrinsicsParams = 60;    // float[15]
        private const int SizeofIntrinsics = 68;          // int + uint + 60
        private const int SizeofCamera = 128;             // 48 + 68 + 4 + 4 + 4
        private const int SizeofCalibration = 1032;       // 128 + 128 + 16*48 + 4 + 4

        // Top-level offsets inside k4a_calibration_t.
        private const int OffsetDepthCamera = 0;
        private const int OffsetColorCamera = SizeofCamera;             // 128
        private const int OffsetExtrinsicsTable = 2 * SizeofCamera;     // 256
        private const int OffsetDepthMode = OffsetExtrinsicsTable + 16 * SizeofExtrinsics; // 1024
        private const int OffsetColorResolution = OffsetDepthMode + 4;  // 1028

        // Offsets inside k4a_calibration_camera_t.
        private const int CamOffsetExtrinsics = 0;
        private const int CamOffsetIntrinsicsType = SizeofExtrinsics;        // 48
        private const int CamOffsetIntrinsicsCount = CamOffsetIntrinsicsType + 4;
        private const int CamOffsetIntrinsicsParams = CamOffsetIntrinsicsCount + 4; // 56
        private const int CamOffsetWidth = CamOffsetIntrinsicsParams + SizeofIntrinsicsParams; // 116
        private const int CamOffsetHeight = CamOffsetWidth + 4;
        private const int CamOffsetMetricRadius = CamOffsetHeight + 4;

        // K4A intrinsic_parameters_t order (k4atypes.h around line 1080):
        //   cx, cy, fx, fy, k1, k2, k3, k4, k5, k6, codx, cody, p2, p1, metric_radius
        // Note p2 comes before p1 in the C union — easy to flip if you trust v2 names blindly.

        /// <summary>
        /// Allocate and populate a k4a_calibration_t from v2 camera parameters and
        /// the actual depth / color resolutions used at capture time. Caller owns the
        /// returned pointer and must call <see cref="Free"/> when done.
        /// </summary>
        public static IntPtr Build(in ObCameraParam param, int depthWidth, int depthHeight,
                                    int colorWidth, int colorHeight)
        {
            IntPtr buf = Marshal.AllocHGlobal(SizeofCalibration);
            // Zero-initialize (identity rotations / zero translations need explicit fill below).
            for (int i = 0; i < SizeofCalibration; i++) Marshal.WriteByte(buf, i, 0);

            // Depth camera: extrinsics = identity (depth is the reference frame for k4abt).
            WriteCamera(buf + OffsetDepthCamera,
                        rot: Identity3x3, trans: Zero3,
                        intr: param.DepthIntrinsic, dist: param.DepthDistortion,
                        resW: depthWidth, resH: depthHeight, metricRadius: 0f);

            // Color camera: extrinsics inside the camera struct are also depth->color
            // per the Azure Kinect convention (camera.extrinsics describes the camera's
            // pose relative to the depth camera origin).
            WriteCamera(buf + OffsetColorCamera,
                        rot: param.Transform.Rot, trans: param.Transform.Trans,
                        intr: param.RgbIntrinsic, dist: param.RgbDistortion,
                        resW: colorWidth, resH: colorHeight, metricRadius: 0f);

            // 4x4 extrinsics table. Diagonal is identity, [DEPTH][COLOR] is the v2
            // depth->color transform, [COLOR][DEPTH] its inverse, all other off-diagonal
            // entries (gyro/accel pairs we don't have) stay identity.
            for (int i = 0; i < 4; i++)
                for (int j = 0; j < 4; j++)
                    WriteExtrinsics(buf + OffsetExtrinsicsTable + (i * 4 + j) * SizeofExtrinsics,
                                    Identity3x3, Zero3);

            int depthIdx = (int)k4a_calibration_type_t.K4A_CALIBRATION_TYPE_DEPTH;
            int colorIdx = (int)k4a_calibration_type_t.K4A_CALIBRATION_TYPE_COLOR;

            WriteExtrinsics(
                buf + OffsetExtrinsicsTable + (depthIdx * 4 + colorIdx) * SizeofExtrinsics,
                param.Transform.Rot, param.Transform.Trans);

            InvertRigid(param.Transform.Rot, param.Transform.Trans, out var rotInv, out var transInv);
            WriteExtrinsics(
                buf + OffsetExtrinsicsTable + (colorIdx * 4 + depthIdx) * SizeofExtrinsics,
                rotInv, transInv);

            // Depth mode + color resolution enums (4 bytes each).
            Marshal.WriteInt32(buf, OffsetDepthMode, (int)PickDepthMode(depthWidth, depthHeight));
            Marshal.WriteInt32(buf, OffsetColorResolution, (int)PickColorResolution(colorWidth, colorHeight));

            return buf;
        }

        public static void Free(IntPtr calibrationHandle)
        {
            if (calibrationHandle != IntPtr.Zero) Marshal.FreeHGlobal(calibrationHandle);
        }

        // --- helpers ---

        private static readonly float[] Identity3x3 = new float[]
        {
            1f, 0f, 0f,
            0f, 1f, 0f,
            0f, 0f, 1f,
        };

        private static readonly float[] Zero3 = new float[] { 0f, 0f, 0f };

        private static void WriteExtrinsics(IntPtr at, float[] rot9, float[] trans3)
        {
            for (int i = 0; i < 9; i++) WriteFloat(at, i * 4, rot9[i]);
            for (int i = 0; i < 3; i++) WriteFloat(at, 36 + i * 4, trans3[i]);
        }

        private static void WriteCamera(IntPtr cam,
                                         float[] rot, float[] trans,
                                         ObCameraIntrinsic intr, ObCameraDistortion dist,
                                         int resW, int resH, float metricRadius)
        {
            // Extrinsics block.
            WriteExtrinsics(cam + CamOffsetExtrinsics, rot, trans);

            // Intrinsics: model type + parameter count + parameters[15].
            //
            // v2 BrownConrady, KannalaBrandt4, etc. → k4a only knows BROWN_CONRADY
            // for live use. We fold all v2 distortion models we're likely to see (BC and
            // Modified BC, which are essentially the same coefficients as far as the
            // body tracker model cares) into BROWN_CONRADY.
            Marshal.WriteInt32(cam, CamOffsetIntrinsicsType,
                (int)k4a_calibration_model_type_t.K4A_CALIBRATION_LENS_DISTORTION_MODEL_BROWN_CONRADY);
            Marshal.WriteInt32(cam, CamOffsetIntrinsicsCount, 14); // cx..p1 = 14, exclude metric_radius

            // Parameter union laid out as 15 floats: cx, cy, fx, fy, k1, k2, k3, k4, k5, k6, codx, cody, p2, p1, metric_radius.
            int p = CamOffsetIntrinsicsParams;
            WriteFloat(cam, p + 0 * 4, intr.Cx);
            WriteFloat(cam, p + 1 * 4, intr.Cy);
            WriteFloat(cam, p + 2 * 4, intr.Fx);
            WriteFloat(cam, p + 3 * 4, intr.Fy);
            WriteFloat(cam, p + 4 * 4, dist.K1);
            WriteFloat(cam, p + 5 * 4, dist.K2);
            WriteFloat(cam, p + 6 * 4, dist.K3);
            WriteFloat(cam, p + 7 * 4, dist.K4);
            WriteFloat(cam, p + 8 * 4, dist.K5);
            WriteFloat(cam, p + 9 * 4, dist.K6);
            WriteFloat(cam, p + 10 * 4, 0f);          // codx — unused for Brown-Conrady
            WriteFloat(cam, p + 11 * 4, 0f);          // cody — unused for Brown-Conrady
            WriteFloat(cam, p + 12 * 4, dist.P2);     // tangential 2 (note: p2 is listed first in k4a)
            WriteFloat(cam, p + 13 * 4, dist.P1);
            WriteFloat(cam, p + 14 * 4, metricRadius);

            Marshal.WriteInt32(cam, CamOffsetWidth, resW);
            Marshal.WriteInt32(cam, CamOffsetHeight, resH);
            WriteFloat(cam, CamOffsetMetricRadius, metricRadius);
        }

        private static void WriteFloat(IntPtr basePtr, int offset, float value)
        {
            unsafe
            {
                byte* dst = (byte*)basePtr.ToPointer() + offset;
                *(float*)dst = value;
            }
        }

        // Inverts a rigid-body transform (R, t): R' = R^T, t' = -R^T * t.
        private static void InvertRigid(float[] rotRowMajor, float[] trans, out float[] rotInv, out float[] transInv)
        {
            // Transpose rotation.
            rotInv = new float[]
            {
                rotRowMajor[0], rotRowMajor[3], rotRowMajor[6],
                rotRowMajor[1], rotRowMajor[4], rotRowMajor[7],
                rotRowMajor[2], rotRowMajor[5], rotRowMajor[8],
            };
            // -R^T * t.
            transInv = new float[]
            {
                -(rotInv[0] * trans[0] + rotInv[1] * trans[1] + rotInv[2] * trans[2]),
                -(rotInv[3] * trans[0] + rotInv[4] * trans[1] + rotInv[5] * trans[2]),
                -(rotInv[6] * trans[0] + rotInv[7] * trans[1] + rotInv[8] * trans[2]),
            };
        }

        // Femto Bolt depth modes mirror Azure Kinect, but if the user picks a non-K4A
        // resolution we fall back to OFF and warn. BT will still use the supplied
        // intrinsics; the depth_mode is used by k4abt mainly to pick a model variant.
        private static k4a_depth_mode_t PickDepthMode(int w, int h)
        {
            if (w == 320 && h == 288) return k4a_depth_mode_t.K4A_DEPTH_MODE_NFOV_2X2BINNED;
            if (w == 640 && h == 576) return k4a_depth_mode_t.K4A_DEPTH_MODE_NFOV_UNBINNED;
            if (w == 512 && h == 512) return k4a_depth_mode_t.K4A_DEPTH_MODE_WFOV_2X2BINNED;
            if (w == 1024 && h == 1024) return k4a_depth_mode_t.K4A_DEPTH_MODE_WFOV_UNBINNED;
            Debug.LogWarning(
                $"[K4ACalibration] depth resolution {w}x{h} doesn't match a known K4A mode. " +
                "BT may reject the calibration; consider switching depth to 640x576 (NFOV unbinned).");
            return k4a_depth_mode_t.K4A_DEPTH_MODE_NFOV_UNBINNED;
        }

        private static k4a_color_resolution_t PickColorResolution(int w, int h)
        {
            if (w == 1280 && h == 720) return k4a_color_resolution_t.K4A_COLOR_RESOLUTION_720P;
            if (w == 1920 && h == 1080) return k4a_color_resolution_t.K4A_COLOR_RESOLUTION_1080P;
            if (w == 2560 && h == 1440) return k4a_color_resolution_t.K4A_COLOR_RESOLUTION_1440P;
            if (w == 2048 && h == 1536) return k4a_color_resolution_t.K4A_COLOR_RESOLUTION_1536P;
            if (w == 3840 && h == 2160) return k4a_color_resolution_t.K4A_COLOR_RESOLUTION_2160P;
            if (w == 4096 && h == 3072) return k4a_color_resolution_t.K4A_COLOR_RESOLUTION_3072P;
            return k4a_color_resolution_t.K4A_COLOR_RESOLUTION_OFF;
        }
    }
}
