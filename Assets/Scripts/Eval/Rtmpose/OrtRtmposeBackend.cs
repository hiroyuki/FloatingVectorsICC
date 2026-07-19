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
    /// <summary>Execution provider request for OrtRtmposeBackend. The backend
    /// falls back DOWNWARD only from the requested provider (TensorRt→Cuda→
    /// DirectML→Cpu); a higher-tier EP is never tried unless requested.</summary>
    public enum OrtProvider { Cpu, DirectML, Cuda, TensorRt }

    public sealed class OrtRtmposeBackend : IRtmposeBackend
    {
        public bool Ready { get; private set; }

        /// <summary>The EP both sessions actually constructed with. Only valid
        /// after the ctor returns; differs from the requested provider when the
        /// fallback chain kicked in (also surfaced as a LogError).</summary>
        public OrtProvider ActiveProvider { get; private set; } = OrtProvider.Cpu;
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
        // Used ONLY by Detect (always under _detectLock) — a RunOptions handle has
        // mutable native state, so concurrent Run calls must not share one.
        // Pose creates a per-call RunOptions instead (vendored ORT NREs on null).
        private readonly RunOptions _runOpt = new RunOptions();

        // Pose is called CONCURRENTLY (one caller per camera, see
        // FusedRtmposeAdapter.SubmitBurst): InferenceSession.Run supports
        // concurrent calls, so Pose only uses locals + caller-owned buffers.
        // Detect keeps shared scratch (_yoloxInput, letterbox params) and is
        // serialized by _detectLock — detects are rare (redetectEveryN).
        private readonly object _detectLock = new object();

        private float _lbScale; private int _lbPadX, _lbPadY;

        public OrtRtmposeBackend(string yoloxPath, string rtmposePath, bool useDirectML = true)
            : this(yoloxPath, rtmposePath, useDirectML ? OrtProvider.DirectML : OrtProvider.Cpu) { }

        public OrtRtmposeBackend(string yoloxPath, string rtmposePath, OrtProvider requested)
        {
            // ActiveProvider is only assigned once BOTH sessions constructed with
            // the candidate EP — AppendExecutionProvider alone is not proof (missing
            // cuDNN/CUDA DLLs surface at InferenceSession creation). On a partial
            // failure the surviving session is disposed too, so a mixed-EP pair can
            // never exist.
            byte[] yoloxBytes = File.ReadAllBytes(yoloxPath);
            byte[] rtmposeBytes = File.ReadAllBytes(rtmposePath);
            foreach (OrtProvider p in FallbackChain(requested))
            {
                SessionOptions opt = null;
                InferenceSession yolox = null, rtmpose = null;
                try
                {
                    opt = new SessionOptions { GraphOptimizationLevel = GraphOptimizationLevel.ORT_ENABLE_ALL };
                    // ORT 1.25 dropped the legacy OrtSessionOptionsAppendExecutionProvider_*
                    // C exports (the int-overload C# wrappers P/Invoke those directly and
                    // throw EntryPointNotFound). DML goes through the generic named-EP API;
                    // CUDA/TensorRT go through the V2 options API — both ride the OrtApi
                    // function table, which is version-stable. The V2 append copies the
                    // options into SessionOptions, so disposing them right after is safe.
                    switch (p)
                    {
                        case OrtProvider.TensorRt:
                            using (var trtOpts = new OrtTensorRTProviderOptions())
                                opt.AppendExecutionProvider_Tensorrt(trtOpts);
                            using (var cudaOpts = new OrtCUDAProviderOptions())
                                opt.AppendExecutionProvider_CUDA(cudaOpts); // TRT-unsupported nodes run on CUDA
                            break;
                        case OrtProvider.Cuda:
                            using (var cudaOpts = new OrtCUDAProviderOptions())
                            {
                                // Match the CPU (v11s reference) numerics as closely
                                // as CUDA allows — both knobs measurably reduced
                                // wrist spike rate in the offline A/B (eval/results/
                                // offline_ab_*.md):
                                //  - use_tf32=0: Ampere+ otherwise runs FP32 convs
                                //    as TF32 (10-bit mantissa)
                                //  - cudnn_conv_algo_search: EXHAUSTIVE picks
                                //    Winograd-class algos whose larger FP error flips
                                //    SimCC argmax on motion-blurred frames. DEFAULT
                                //    matches CPU numerics but is pathologically slow
                                //    on ORT 1.26 + cuDNN 9 (~505ms/inference);
                                //    HEURISTIC keeps the deterministic GEMM-class
                                //    choice at full speed (verify parity via
                                //    LiveV11sVerify.CompareTakes after ORT bumps).
                                cudaOpts.UpdateOptions(new Dictionary<string, string>
                                {
                                    { "use_tf32", "0" },
                                    { "cudnn_conv_algo_search", "HEURISTIC" },
                                });
                                opt.AppendExecutionProvider_CUDA(cudaOpts);
                            }
                            break;
                        case OrtProvider.DirectML: opt.AppendExecutionProvider("DML"); break;
                    }
                    yolox = new InferenceSession(yoloxBytes, opt);
                    rtmpose = new InferenceSession(rtmposeBytes, opt);
                    _opt = opt; _yolox = yolox; _rtmpose = rtmpose;
                    ActiveProvider = p;
                    break;
                }
                catch (Exception e)
                {
                    rtmpose?.Dispose(); yolox?.Dispose(); opt?.Dispose();
                    if (p == OrtProvider.Cpu) throw; // terminal: nothing left to try
                    Debug.LogWarning($"[rtmpose] EP {p} failed, trying next in chain: {e.Message}");
                }
            }
            if (ActiveProvider == requested)
                Debug.Log($"[rtmpose] EP={ActiveProvider} (requested={requested})");
            else
                Debug.LogError($"[rtmpose] EP fallback: requested={requested} actual={ActiveProvider} — " +
                               "any benchmark on this backend does NOT measure the requested provider");
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
            lock (_detectLock)
            {
                // an async detect (thread pool, not joined by session teardown) can
                // pass the outer Ready check and then lose the lock race to
                // Dispose — recheck under the lock so it no-ops on dead sessions
                if (!Ready) return 0;
                return DetectLocked(rgb, w, h, outBoxes);
            }
        }

        private int DetectLocked(byte[] rgb, int w, int h, DetBox[] outBoxes)
        {
            Letterbox(rgb, w, h, _yoloxInput, out _lbScale, out _lbPadX, out _lbPadY);
            using var inp = OrtValue.CreateTensorValueFromMemory(_yoloxInput, _yoloxShape);
            var in1 = new OrtValue[] { inp };
            using var res = _yolox.Run(_runOpt, _yolox.InputNames, in1, _yolox.OutputNames);

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

        /// <summary>A/B probe: serialize concurrent Pose calls. Live fusion runs the
        /// four cameras' Pose concurrently on ONE session; if the CUDA EP's shared
        /// per-session scratch corrupts overlapping runs, serializing restores the
        /// offline (sequential) numerics at ~4x pose latency per burst.</summary>
        public static bool SerializePose = false;
        private readonly object _poseLock = new object();

        public bool Pose(float[] nchwInput, float[] simccX, float[] simccY)
        {
            if (!Ready) return false;
            if (SerializePose) { lock (_poseLock) return PoseUnlocked(nchwInput, simccX, simccY); }
            return PoseUnlocked(nchwInput, simccX, simccY);
        }

        private bool PoseUnlocked(float[] nchwInput, float[] simccX, float[] simccY)
        {
            using var inp = OrtValue.CreateTensorValueFromMemory(nchwInput, _poseShape);
            var in1 = new OrtValue[] { inp }; // local: Pose runs concurrently per camera
            using var runOpt = new RunOptions(); // per-call: RunOptions native state must not be shared across concurrent Runs
            using var res = _rtmpose.Run(runOpt, _rtmpose.InputNames, in1, _rtmpose.OutputNames);
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
            // Async detects run on the THREAD POOL and are not joined by session
            // teardown (LiveFusedBodySource joins only its worker). Taking the
            // detect lock waits out an in-flight Detect; one that lost the lock
            // race rechecks Ready inside the lock and no-ops. Poses cannot be in
            // flight here: they only run inside SubmitBurst on the (already
            // joined) worker, and the stuck-worker path abandons instead of
            // disposing.
            lock (_detectLock)
            {
                _yolox?.Dispose();
                _rtmpose?.Dispose();
                _runOpt?.Dispose();
                _opt?.Dispose();
            }
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
            // A/B numerics probe — see RtmposePreprocess.RowParallel
            if (!RtmposePreprocess.RowParallel)
            {
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
                return;
            }
            float s = scale; int px = padX, py = padY; // no out params inside the lambda
            // Row-parallel: disjoint dst rows, read-only rgb (same pattern as
            // RtmposePreprocess.BuildInput — this was ~half of the 39ms detect cost).
            System.Threading.Tasks.Parallel.For(0, YoloxSize, dy =>
            {
                for (int dx = 0; dx < YoloxSize; dx++)
                {
                    int o = dy * YoloxSize + dx;
                    float r = 114f, g = 114f, b = 114f;
                    int sx = dx - px, sy = dy - py;
                    if (sx >= 0 && sy >= 0 && sx < newW && sy < newH)
                    {
                        int ix = Mathf.Min((int)(sx / s), w - 1);
                        int iy = Mathf.Min((int)(sy / s), h - 1);
                        int si = (iy * w + ix) * 3;
                        r = rgb[si]; g = rgb[si + 1]; b = rgb[si + 2];
                    }
                    dst[c0 * plane + o] = r;
                    dst[1 * plane + o] = g;
                    dst[c2 * plane + o] = b;
                }
            });
        }

        private static IEnumerable<OrtProvider> FallbackChain(OrtProvider requested)
        {
            if (requested >= OrtProvider.TensorRt) yield return OrtProvider.TensorRt;
            if (requested >= OrtProvider.Cuda) yield return OrtProvider.Cuda;
            if (requested >= OrtProvider.DirectML) yield return OrtProvider.DirectML;
            yield return OrtProvider.Cpu;
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
