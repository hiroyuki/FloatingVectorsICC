// Inference backend abstraction for the RTMPose adapter: a person detector
// (YOLOX) + the RTMPose estimator. The real implementation runs ONNX Runtime and
// is added together with the ORT Unity plugin; NullRtmposeBackend lets the whole
// Track-B pipeline compile and wire up before the plugin/models are in place.

using System;

namespace BodyTracking.Eval.Rtmpose
{
    public struct DetBox
    {
        public float X1, Y1, X2, Y2, Score;
    }

    public interface IRtmposeBackend : IDisposable
    {
        /// <summary>True once models are loaded and inference can run.</summary>
        bool Ready { get; }
        RtmposeSpec Spec { get; }
        int NumKeypoints { get; }

        /// <summary>Detect people in an RGB8 image; fill outBoxes; return count.</summary>
        int Detect(byte[] rgb, int width, int height, DetBox[] outBoxes);

        /// <summary>Run RTMPose on a prepared NCHW input; fill simccX/simccY; return success.</summary>
        bool Pose(float[] nchwInput, float[] simccX, float[] simccY);
    }

    /// <summary>Placeholder backend — reports not-ready so the adapter emits nothing.</summary>
    public sealed class NullRtmposeBackend : IRtmposeBackend
    {
        public bool Ready => false;
        public RtmposeSpec Spec => RtmposeSpec.Body256x192;
        public int NumKeypoints => Coco.Count;
        public int Detect(byte[] rgb, int width, int height, DetBox[] outBoxes) => 0;
        public bool Pose(float[] nchwInput, float[] simccX, float[] simccY) => false;
        public void Dispose() { }
    }
}
