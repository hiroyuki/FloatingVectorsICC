// RTMPose top-down pipeline: deterministic, inference-backend-agnostic pieces.
//
//   color crop (top-down affine) -> normalized NCHW input
//   -> [inference backend produces simcc_x / simcc_y]
//   -> SimCC decode -> COCO-17 keypoints (image px + score)
//   -> map to the common 15-joint subset (Pelvis/Neck derived)
//
// 3D lift (image px + depth -> camera-space mm) lives in DepthLift.cs. Nothing
// here touches ONNX Runtime, so it compiles/tests without the inference plugin.

using UnityEngine;

namespace BodyTracking.Eval.Rtmpose
{
    /// <summary>RTMPose-m/body7 model geometry (256x192 input, SimCC split ratio 2).</summary>
    public struct RtmposeSpec
    {
        public int InW;          // model input width  (192)
        public int InH;          // model input height (256)
        public float SplitRatio; // SimCC split ratio  (2.0)
        public static RtmposeSpec Body256x192 => new RtmposeSpec { InW = 192, InH = 256, SplitRatio = 2f };
    }

    /// <summary>COCO-17 keypoint indices (RTMPose body7 output order).</summary>
    public static class Coco
    {
        public const int Nose = 0, LEye = 1, REye = 2, LEar = 3, REar = 4,
            LShoulder = 5, RShoulder = 6, LElbow = 7, RElbow = 8, LWrist = 9, RWrist = 10,
            LHip = 11, RHip = 12, LKnee = 13, RKnee = 14, LAnkle = 15, RAnkle = 16;
        public const int Count = 17;
    }

    /// <summary>A person bbox in image pixels plus the center/scale used to warp it.</summary>
    public struct PersonRoi
    {
        public float Cx, Cy;   // center (image px)
        public float Bw, Bh;   // padded, aspect-fixed box size (image px)
    }

    public static class RtmposePreprocess
    {
        public const float Padding = 1.25f;
        // ImageNet mean/std in RGB order (matches RTMPose body7 export).
        static readonly float[] Mean = { 123.675f, 116.28f, 103.53f };
        static readonly float[] Std = { 58.395f, 57.12f, 57.375f };

        /// <summary>
        /// bbox (xyxy, image px) -> padded, model-aspect-corrected center/scale ROI.
        /// </summary>
        public static PersonRoi RoiFromBox(float x1, float y1, float x2, float y2, in RtmposeSpec spec)
        {
            float cx = 0.5f * (x1 + x2), cy = 0.5f * (y1 + y2);
            float bw = (x2 - x1) * Padding, bh = (y2 - y1) * Padding;
            float aspect = (float)spec.InW / spec.InH;
            if (bw > aspect * bh) bh = bw / aspect; else bw = bh * aspect;
            return new PersonRoi { Cx = cx, Cy = cy, Bw = bw, Bh = bh };
        }

        /// <summary>
        /// Warp the ROI out of an RGB8 image into a normalized NCHW float tensor
        /// (length 3*InH*InW), bilinear sampled. RGB channel order.
        /// </summary>
        /// <summary>A/B numerics probe (editor tooling flips this): false runs the
        /// pre-parallelization serial loop verbatim.</summary>
        public static bool RowParallel = true;

        public static float[] BuildInput(byte[] rgb, int imgW, int imgH, in PersonRoi roi, in RtmposeSpec spec, float[] reuse = null)
        {
            int ow = spec.InW, oh = spec.InH, plane = ow * oh;
            var dst = (reuse != null && reuse.Length == 3 * plane) ? reuse : new float[3 * plane];
            float x0 = roi.Cx - 0.5f * roi.Bw, y0 = roi.Cy - 0.5f * roi.Bh;
            float bw = roi.Bw, bh = roi.Bh;
            // A/B probe: false = the pre-parallelization loop VERBATIM, for
            // bit-exactness checks against archived exports (numerics can differ
            // between the inline loop and the lambda via JIT FP contraction).
            if (!RowParallel)
            {
                for (int dy = 0; dy < oh; dy++)
                {
                    float sy0 = y0 + (dy + 0.5f) / oh * roi.Bh - 0.5f;
                    for (int dx = 0; dx < ow; dx++)
                    {
                        float sx0 = x0 + (dx + 0.5f) / ow * roi.Bw - 0.5f;
                        SampleBilinear(rgb, imgW, imgH, sx0, sy0, out float r0, out float g0, out float b0);
                        int idx0 = dy * ow + dx;
                        dst[0 * plane + idx0] = (r0 - Mean[0]) / Std[0];
                        dst[1 * plane + idx0] = (g0 - Mean[1]) / Std[1];
                        dst[2 * plane + idx0] = (b0 - Mean[2]) / Std[2];
                    }
                }
                return dst;
            }
            // Row-parallel: each dy writes a disjoint slice of dst, rgb is read-only.
            // This was the hot CPU stage of the live pipeline (~7.6ms/frame serial,
            // 4 cameras × 26fps saturated the worker thread — see PLAN_live_gpu Phase 3).
            System.Threading.Tasks.Parallel.For(0, oh, dy =>
            {
                float sy = y0 + (dy + 0.5f) / oh * bh - 0.5f;
                for (int dx = 0; dx < ow; dx++)
                {
                    float sx = x0 + (dx + 0.5f) / ow * bw - 0.5f;
                    SampleBilinear(rgb, imgW, imgH, sx, sy, out float r, out float g, out float b);
                    int idx = dy * ow + dx;
                    dst[0 * plane + idx] = (r - Mean[0]) / Std[0];
                    dst[1 * plane + idx] = (g - Mean[1]) / Std[1];
                    dst[2 * plane + idx] = (b - Mean[2]) / Std[2];
                }
            });
            return dst;
        }

        static void SampleBilinear(byte[] rgb, int w, int h, float x, float y, out float r, out float g, out float b)
        {
            r = g = b = 0f;
            if (rgb == null || rgb.Length < w * h * 3) return;
            int x0 = Mathf.Clamp(Mathf.FloorToInt(x), 0, w - 1);
            int y0 = Mathf.Clamp(Mathf.FloorToInt(y), 0, h - 1);
            int x1 = Mathf.Min(x0 + 1, w - 1), y1 = Mathf.Min(y0 + 1, h - 1);
            float fx = Mathf.Clamp01(x - x0), fy = Mathf.Clamp01(y - y0);
            Px(rgb, w, x0, y0, out float r00, out float g00, out float b00);
            Px(rgb, w, x1, y0, out float r10, out float g10, out float b10);
            Px(rgb, w, x0, y1, out float r01, out float g01, out float b01);
            Px(rgb, w, x1, y1, out float r11, out float g11, out float b11);
            r = Lerp2(r00, r10, r01, r11, fx, fy);
            g = Lerp2(g00, g10, g01, g11, fx, fy);
            b = Lerp2(b00, b10, b01, b11, fx, fy);
        }

        static void Px(byte[] rgb, int w, int x, int y, out float r, out float g, out float b)
        {
            int o = (y * w + x) * 3; r = rgb[o]; g = rgb[o + 1]; b = rgb[o + 2];
        }

        static float Lerp2(float a, float b, float c, float d, float fx, float fy)
            => Mathf.Lerp(Mathf.Lerp(a, b, fx), Mathf.Lerp(c, d, fx), fy);
    }

    public static class SimccDecoder
    {
        /// <summary>
        /// Decode SimCC logits to COCO keypoints in ORIGINAL image pixels.
        /// simccX length = K*Wx (Wx = InW*SplitRatio), simccY length = K*Wy.
        /// Writes into kpts[K] (image px) and scores[K].
        /// </summary>
        public static void Decode(float[] simccX, float[] simccY, int k, in PersonRoi roi, in RtmposeSpec spec,
                                  Vector2[] kpts, float[] scores)
        {
            int wx = simccX.Length / k, wy = simccY.Length / k;
            float x0 = roi.Cx - 0.5f * roi.Bw, y0 = roi.Cy - 0.5f * roi.Bh;
            for (int j = 0; j < k; j++)
            {
                ArgMax(simccX, j * wx, wx, out int xi, out float xmax);
                ArgMax(simccY, j * wy, wy, out int yi, out float ymax);
                float mx = xi / spec.SplitRatio; // model-input px [0,InW]
                float my = yi / spec.SplitRatio; // model-input px [0,InH]
                // invert the top-down affine using the SAME pixel-center convention
                // as BuildInput (which samples at (d+0.5)/N*B - 0.5), so forward and
                // inverse are exact inverses (no systematic half-pixel bias).
                kpts[j] = new Vector2(
                    x0 + (mx + 0.5f) / spec.InW * roi.Bw - 0.5f,
                    y0 + (my + 0.5f) / spec.InH * roi.Bh - 0.5f);
                // Score is the larger of the two SimCC axis peaks. NOTE: this is the
                // raw SimCC head activation, not a softmaxed 0..1 probability — its
                // scale must be checked empirically before trusting confThreshold.
                scores[j] = Mathf.Max(xmax, ymax);
            }
        }

        static void ArgMax(float[] a, int off, int len, out int idx, out float max)
        {
            idx = 0; max = float.NegativeInfinity;
            for (int i = 0; i < len; i++) { float v = a[off + i]; if (v > max) { max = v; idx = i; } }
        }
    }

    public static class CocoToEval
    {
        /// <summary>
        /// Map COCO-17 (image px + score) to the 15-joint subset (still image px + score;
        /// z is filled later by DepthLift). Pelvis = hip midpoint, Neck = shoulder
        /// midpoint, Head = nose. Derived-joint score = min of parents.
        /// </summary>
        public static void Map(Vector2[] coco, float[] score, float confThreshold,
                               Vector2[] outXy, float[] outScore, bool[] outValid)
        {
            void Set(EvalJointId id, Vector2 p, float s)
            {
                int i = (int)id; outXy[i] = p; outScore[i] = s; outValid[i] = s >= confThreshold;
            }
            Vector2 Mid(int a, int b) => 0.5f * (coco[a] + coco[b]);
            float MinS(int a, int b) => Mathf.Min(score[a], score[b]);

            Set(EvalJointId.Pelvis, Mid(Coco.LHip, Coco.RHip), MinS(Coco.LHip, Coco.RHip));
            Set(EvalJointId.Neck, Mid(Coco.LShoulder, Coco.RShoulder), MinS(Coco.LShoulder, Coco.RShoulder));
            Set(EvalJointId.Head, coco[Coco.Nose], score[Coco.Nose]);
            Set(EvalJointId.ShoulderL, coco[Coco.LShoulder], score[Coco.LShoulder]);
            Set(EvalJointId.ElbowL, coco[Coco.LElbow], score[Coco.LElbow]);
            Set(EvalJointId.WristL, coco[Coco.LWrist], score[Coco.LWrist]);
            Set(EvalJointId.ShoulderR, coco[Coco.RShoulder], score[Coco.RShoulder]);
            Set(EvalJointId.ElbowR, coco[Coco.RElbow], score[Coco.RElbow]);
            Set(EvalJointId.WristR, coco[Coco.RWrist], score[Coco.RWrist]);
            Set(EvalJointId.HipL, coco[Coco.LHip], score[Coco.LHip]);
            Set(EvalJointId.KneeL, coco[Coco.LKnee], score[Coco.LKnee]);
            Set(EvalJointId.AnkleL, coco[Coco.LAnkle], score[Coco.LAnkle]);
            Set(EvalJointId.HipR, coco[Coco.RHip], score[Coco.RHip]);
            Set(EvalJointId.KneeR, coco[Coco.RKnee], score[Coco.RKnee]);
            Set(EvalJointId.AnkleR, coco[Coco.RAnkle], score[Coco.RAnkle]);
        }
    }
}
