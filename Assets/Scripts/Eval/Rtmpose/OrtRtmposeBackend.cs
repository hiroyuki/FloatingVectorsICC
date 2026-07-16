// Real inference backend: YOLOX person detector + RTMPose, via ONNX Runtime
// (asus4 vendored Microsoft.ML.OnnxRuntime, DirectML EP on Windows).
//
// YOLOX end2end.onnx bakes NMS in: input "input" [1,3,640,640] -> "dets"[1,N,5]
// (x1,y1,x2,y2,score in letterboxed coords) + "labels"[1,N]. RTMPose end2end:
// input "input" [1,3,256,192] -> "simcc_x"[1,17,384], "simcc_y"[1,17,512].
//
// Channel-order / normalization assumptions are flagged; verify empirically
// (research warned this is a silent-accuracy risk):
//   YOLOX  : BGR, raw 0-255, letterbox pad 114 (mmdet convention)
//   RTMPose: RGB, ImageNet mean/std (done in RtmposePipeline)

using System;
using System.Collections.Generic;
using System.IO;
using Microsoft.ML.OnnxRuntime;
using Microsoft.ML.OnnxRuntime.Tensors;
using UnityEngine;

namespace BodyTracking.Eval.Rtmpose
{
    public sealed class OrtRtmposeBackend : IRtmposeBackend
    {
        public bool Ready { get; private set; }
        public RtmposeSpec Spec { get; } = RtmposeSpec.Body256x192;
        public int NumKeypoints => Coco.Count;

        public float detScoreThreshold = 0.3f;
        public bool yoloxBgr = true;      // mmdet YOLOX default channel order
        public const int PersonLabel = 0;

        private const int YoloxSize = 640;
        private readonly SessionOptions _opt;
        private readonly InferenceSession _yolox, _rtmpose;
        private readonly float[] _yoloxInput = new float[3 * YoloxSize * YoloxSize];
        // Output indices resolved by NAME (output order is not guaranteed).
        private readonly int _detsIdx, _labelsIdx, _simccXIdx, _simccYIdx;
        // Hoisted allocations (reused every frame to avoid GC in the hot path).
        private readonly long[] _yoloxShape = { 1, 3, YoloxSize, YoloxSize };
        private readonly long[] _poseShape;
        private readonly OrtValue[] _in1 = new OrtValue[1];
        private readonly RunOptions _runOpt = new RunOptions(); // vendored ORT NREs on a null RunOptions

        private float _lbScale; private int _lbPadX, _lbPadY;

        public OrtRtmposeBackend(string yoloxPath, string rtmposePath, bool useDirectML = true)
        {
            _opt = new SessionOptions { GraphOptimizationLevel = GraphOptimizationLevel.ORT_ENABLE_ALL };
            if (useDirectML)
            {
                try { _opt.AppendExecutionProvider_DML(0); }
                catch (Exception e) { Debug.LogWarning($"[rtmpose] DirectML unavailable, using CPU: {e.Message}"); }
            }
            _yolox = new InferenceSession(File.ReadAllBytes(yoloxPath), _opt);
            _rtmpose = new InferenceSession(File.ReadAllBytes(rtmposePath), _opt);
            _detsIdx = FindOr(_yolox.OutputNames, "dets", 0);
            _labelsIdx = FindOr(_yolox.OutputNames, "labels", 1);
            _simccXIdx = FindOr(_rtmpose.OutputNames, "simcc_x", 0);
            _simccYIdx = FindOr(_rtmpose.OutputNames, "simcc_y", 1);
            _poseShape = new long[] { 1, 3, Spec.InH, Spec.InW };
            Debug.Log($"[rtmpose] yolox in={Join(_yolox.InputNames)} out={Join(_yolox.OutputNames)} " +
                      $"(dets@{_detsIdx},labels@{_labelsIdx}); rtmpose in={Join(_rtmpose.InputNames)} " +
                      $"out={Join(_rtmpose.OutputNames)} (x@{_simccXIdx},y@{_simccYIdx})");
            Ready = true;
        }

        public int Detect(byte[] rgb, int w, int h, DetBox[] outBoxes)
        {
            if (!Ready) return 0;
            Letterbox(rgb, w, h, _yoloxInput, out _lbScale, out _lbPadX, out _lbPadY);
            using var inp = OrtValue.CreateTensorValueFromMemory(_yoloxInput, _yoloxShape);
            _in1[0] = inp;
            using var res = _yolox.Run(_runOpt, _yolox.InputNames, _in1, _yolox.OutputNames);

            var dets = res[_detsIdx].GetTensorDataAsSpan<float>();
            var shape = res[_detsIdx].GetTensorTypeAndShape().Shape; // [1,N,5]
            int n = shape.Length >= 2 ? (int)shape[shape.Length - 2] : 0;

            var lblVal = res[_labelsIdx];
            var lblType = lblVal.GetTensorTypeAndShape().ElementDataType;
            ReadOnlySpan<long> lblL = lblType == TensorElementType.Int64 ? lblVal.GetTensorDataAsSpan<long>() : default;
            ReadOnlySpan<int> lblI = lblType == TensorElementType.Int64 ? default : lblVal.GetTensorDataAsSpan<int>();

            int count = 0;
            for (int i = 0; i < n && count < outBoxes.Length; i++)
            {
                float score = dets[i * 5 + 4];
                if (score < detScoreThreshold) continue;
                long label = lblType == TensorElementType.Int64 ? lblL[i] : lblI[i];
                if (label != PersonLabel) continue;
                outBoxes[count++] = new DetBox
                {
                    X1 = (dets[i * 5 + 0] - _lbPadX) / _lbScale,
                    Y1 = (dets[i * 5 + 1] - _lbPadY) / _lbScale,
                    X2 = (dets[i * 5 + 2] - _lbPadX) / _lbScale,
                    Y2 = (dets[i * 5 + 3] - _lbPadY) / _lbScale,
                    Score = score,
                };
            }
            return count;
        }

        public bool Pose(float[] nchwInput, float[] simccX, float[] simccY)
        {
            if (!Ready) return false;
            using var inp = OrtValue.CreateTensorValueFromMemory(nchwInput, _poseShape);
            _in1[0] = inp;
            using var res = _rtmpose.Run(_runOpt, _rtmpose.InputNames, _in1, _rtmpose.OutputNames);
            var sx = res[_simccXIdx].GetTensorDataAsSpan<float>();
            var sy = res[_simccYIdx].GetTensorDataAsSpan<float>();
            if (sx.Length > simccX.Length || sy.Length > simccY.Length) return false;
            sx.CopyTo(simccX);
            sy.CopyTo(simccY);
            return true;
        }

        public void Dispose()
        {
            // Ready gates Detect/Pose, so flipping it first turns any call on a
            // disposed backend into a no-op instead of a native crash inside ORT.
            // This matters because RtmPoseAdapter.Dispose() disposes the backend
            // it was HANDED — callers sharing one backend across adapters (Frame
            // Inspector's s_backend cache, ad-hoc eval scripts) hard-crashed the
            // editor when a later adapter used the already-disposed session.
            Ready = false;
            _yolox?.Dispose();
            _rtmpose?.Dispose();
            _runOpt?.Dispose();
            _opt?.Dispose();
        }

        // ------------------------------------------------------------------

        /// <summary>Resize RGB8 into a 640x640 NCHW buffer keeping aspect (pad 114), YOLOX channel order.</summary>
        private void Letterbox(byte[] rgb, int w, int h, float[] dst, out float scale, out int padX, out int padY)
        {
            scale = Mathf.Min((float)YoloxSize / w, (float)YoloxSize / h);
            int newW = Mathf.RoundToInt(w * scale), newH = Mathf.RoundToInt(h * scale);
            padX = (YoloxSize - newW) / 2; padY = (YoloxSize - newH) / 2;
            int plane = YoloxSize * YoloxSize;
            int c0 = yoloxBgr ? 2 : 0, c2 = yoloxBgr ? 0 : 2; // swap R/B if BGR
            for (int dy = 0; dy < YoloxSize; dy++)
            {
                for (int dx = 0; dx < YoloxSize; dx++)
                {
                    int o = dy * YoloxSize + dx;
                    float r = 114f, g = 114f, b = 114f;
                    int sx = dx - padX, sy = dy - padY;
                    if (sx >= 0 && sy >= 0 && sx < newW && sy < newH)
                    {
                        int ix = Mathf.Min((int)(sx / scale), w - 1);
                        int iy = Mathf.Min((int)(sy / scale), h - 1);
                        int si = (iy * w + ix) * 3;
                        r = rgb[si]; g = rgb[si + 1]; b = rgb[si + 2];
                    }
                    dst[c0 * plane + o] = r;
                    dst[1 * plane + o] = g;
                    dst[c2 * plane + o] = b;
                }
            }
        }

        private static int FindOr(IReadOnlyList<string> names, string name, int fallback)
        {
            for (int i = 0; i < names.Count; i++) if (names[i] == name) return i;
            return Mathf.Clamp(fallback, 0, Mathf.Max(0, names.Count - 1));
        }

        private static string Join(IReadOnlyList<string> names)
        {
            var sb = new System.Text.StringBuilder();
            for (int i = 0; i < names.Count; i++) { if (i > 0) sb.Append(','); sb.Append(names[i]); }
            return sb.ToString();
        }
    }
}
