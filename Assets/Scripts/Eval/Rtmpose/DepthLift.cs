// Lift 2D keypoints (color-image pixels) to 3D camera-space millimeters using
// the recorded depth aligned to the color camera.
//
// The recorded depth is in the DEPTH camera frame; RTMPose keypoints are in the
// COLOR image. We forward-project every depth pixel into the color image (via
// intrinsics + the depth->color extrinsic), z-buffer the nearest, then sample a
// median window at each keypoint and back-project (u,v,d) to color-camera mm
// (OpenCV frame: +x right, +y down, +z forward). Requires ObCameraParam
// (intrinsics + D2C); recordings without calibration cannot be lifted.

using Orbbec;
using UnityEngine;

namespace BodyTracking.Eval.Rtmpose
{
    public sealed class DepthLift
    {
        private ushort[] _colorDepth; // aligned depth (mm) in color image space
        private int _cw, _ch;

        /// <summary>Build the color-aligned depth image for this frame. Returns false if inputs are unusable.</summary>
        public bool BuildAligned(byte[] depthY16, int dw, int dh, in ObCameraParam cam)
        {
            int cw = cam.RgbIntrinsic.Width, ch = cam.RgbIntrinsic.Height;
            if (depthY16 == null || dw <= 0 || dh <= 0 || cw <= 0 || ch <= 0) return false;
            if (depthY16.Length < dw * dh * 2) return false;

            if (_colorDepth == null || _cw != cw || _ch != ch) { _colorDepth = new ushort[cw * ch]; _cw = cw; _ch = ch; }
            System.Array.Clear(_colorDepth, 0, _colorDepth.Length);

            float fxd = cam.DepthIntrinsic.Fx, fyd = cam.DepthIntrinsic.Fy, cxd = cam.DepthIntrinsic.Cx, cyd = cam.DepthIntrinsic.Cy;
            float fxc = cam.RgbIntrinsic.Fx, fyc = cam.RgbIntrinsic.Fy, cxc = cam.RgbIntrinsic.Cx, cyc = cam.RgbIntrinsic.Cy;
            var R = cam.Transform.Rot;    // row-major 3x3, depth->color
            var T = cam.Transform.Trans;  // mm

            for (int vd = 0; vd < dh; vd++)
            {
                for (int ud = 0; ud < dw; ud++)
                {
                    int di = vd * dw + ud;
                    ushort d = (ushort)(depthY16[di * 2] | (depthY16[di * 2 + 1] << 8));
                    if (d == 0) continue;
                    float Xd = (ud - cxd) / fxd * d, Yd = (vd - cyd) / fyd * d, Zd = d;
                    float Xc = R[0] * Xd + R[1] * Yd + R[2] * Zd + T[0];
                    float Yc = R[3] * Xd + R[4] * Yd + R[5] * Zd + T[1];
                    float Zc = R[6] * Xd + R[7] * Yd + R[8] * Zd + T[2];
                    if (Zc <= 0f) continue;
                    int uc = Mathf.RoundToInt(fxc * Xc / Zc + cxc);
                    int vc = Mathf.RoundToInt(fyc * Yc / Zc + cyc);
                    if (uc < 0 || uc >= cw || vc < 0 || vc >= ch) continue;
                    int ci = vc * cw + uc;
                    ushort zc = (ushort)Mathf.Clamp(Zc, 0, 65535);
                    if (_colorDepth[ci] == 0 || zc < _colorDepth[ci]) _colorDepth[ci] = zc;
                }
            }
            return true;
        }

        /// <summary>Median depth (mm) in a (2*half+1) window around (u,v); 0 if none valid.</summary>
        public float SampleMm(int u, int v, int half)
        {
            if (_colorDepth == null) return 0f;
            System.Span<ushort> buf = stackalloc ushort[(2 * half + 1) * (2 * half + 1)];
            int n = 0;
            for (int dy = -half; dy <= half; dy++)
            {
                int y = v + dy; if (y < 0 || y >= _ch) continue;
                for (int dx = -half; dx <= half; dx++)
                {
                    int x = u + dx; if (x < 0 || x >= _cw) continue;
                    ushort z = _colorDepth[y * _cw + x];
                    if (z != 0) buf[n++] = z;
                }
            }
            if (n == 0) return 0f;
            // insertion sort (window is tiny) then median
            for (int i = 1; i < n; i++) { ushort key = buf[i]; int j = i - 1; while (j >= 0 && buf[j] > key) { buf[j + 1] = buf[j]; j--; } buf[j + 1] = key; }
            return buf[n / 2];
        }

        /// <summary>Back-project a color pixel + depth (mm) to color-camera-space mm (OpenCV frame).</summary>
        public static Vector3 Backproject(float u, float v, float dMm, in ObCameraParam cam)
        {
            float fxc = cam.RgbIntrinsic.Fx, fyc = cam.RgbIntrinsic.Fy, cxc = cam.RgbIntrinsic.Cx, cyc = cam.RgbIntrinsic.Cy;
            return new Vector3((u - cxc) / fxc * dMm, (v - cyc) / fyc * dMm, dMm);
        }
    }
}
