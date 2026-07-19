// Tracker abstraction for the evaluation harness. Each approach (k4abt /
// Nuitrack / RTMPose) implements this so EvalRunner can drive them uniformly
// and swap implementations without touching the metrics/replay layers.

using System;
using Orbbec;
using PointCloud;

namespace BodyTracking.Eval
{
    /// <summary>
    /// Per-camera context handed to an adapter before frames start. CameraParam
    /// carries intrinsics + D2C extrinsic when available (may be null for
    /// recordings without embedded calibration — RTMPose needs a fallback).
    /// </summary>
    public readonly struct EvalCameraContext
    {
        public readonly string Serial;
        public readonly int DepthWidth, DepthHeight;
        public readonly int ColorWidth, ColorHeight;
        public readonly ObCameraParam? CameraParam;

        public EvalCameraContext(string serial, int depthW, int depthH, int colorW, int colorH, ObCameraParam? cam)
        {
            Serial = serial;
            DepthWidth = depthW; DepthHeight = depthH;
            ColorWidth = colorW; ColorHeight = colorH;
            CameraParam = cam;
        }
    }

    /// <summary>
    /// A body tracker being evaluated. Frames are pushed via SubmitFrame; results
    /// come back asynchronously through OnSkeletons (a synchronous adapter may
    /// raise it inside SubmitFrame). Latency is measured by the runner between
    /// the two, keyed on the source frame timestamp.
    /// </summary>
    public interface ITrackerAdapter : IDisposable
    {
        /// <summary>Stable short name used as the metrics/CSV key (e.g. "k4abt").</summary>
        string Name { get; }

        /// <summary>Announce a camera before its frames arrive. Called once per serial.</summary>
        void Configure(in EvalCameraContext ctx);

        /// <summary>
        /// Push one source frame. The adapter must copy anything it needs to keep
        /// past this call — driver buffers are reused on the next frame.
        /// </summary>
        void SubmitFrame(string serial, in RawFrameData frame, ulong tsNs);

        /// <summary>
        /// Some adapters (e.g. the recorded-bodies baseline) consume a pre-tracked
        /// bodies_main payload instead of running inference on the frame. Adapters
        /// that run their own inference ignore this. Payload bytes are reused after
        /// the call — copy if retained.
        /// </summary>
        void SubmitRecordedBodies(string serial, byte[] payload, int byteCount, ulong tsNs);

        /// <summary>Raised (main thread, via Pump) with normalized skeletons for a frame.</summary>
        event Action<EvalSkeletonFrame> OnSkeletons;

        /// <summary>
        /// Drain any completed async results and raise OnSkeletons for them.
        /// Called by the runner every Update on the main thread.
        /// </summary>
        void Pump();
    }
}
