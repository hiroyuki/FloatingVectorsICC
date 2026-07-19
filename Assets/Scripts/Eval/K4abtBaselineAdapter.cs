// Baseline adapter: the recorded k4abt output (bodies_main) replayed as-is.
//
// This is the reference every alternative is compared against. It runs no
// inference — it decodes the pre-tracked bodies_main payload the recording
// already contains (which IS current k4abt's output) and normalizes it into the
// common EvalSkeleton format. Because decode is synchronous, its measured
// latency is ~0 and NOT representative of live k4abt inference latency; use the
// live-worker mode (future) to measure that. All other metrics (jitter,
// continuity, multi-person) are valid from this passthrough.

using System;

namespace BodyTracking.Eval
{
    public sealed class K4abtBaselineAdapter : ITrackerAdapter
    {
        public string Name => "k4abt";

        public event Action<EvalSkeletonFrame> OnSkeletons;

        // bodies_main allows up to 6 bodies/frame (K4abtWorkerSharedLayout.MaxBodies).
        private const int MaxBodies = 6;
        private readonly BodySnapshot[] _decodeBuf = new BodySnapshot[MaxBodies];
        private readonly EvalSkeletonFrame _frame = new EvalSkeletonFrame();
        // Reusable EvalSkeleton pool so we don't allocate per frame.
        private readonly EvalSkeleton[] _pool = new EvalSkeleton[MaxBodies];

        public K4abtBaselineAdapter()
        {
            for (int i = 0; i < MaxBodies; i++)
            {
                _decodeBuf[i] = new BodySnapshot();
                _pool[i] = new EvalSkeleton();
            }
        }

        public void Configure(in EvalCameraContext ctx) { /* baseline needs no calibration */ }

        public void SubmitFrame(string serial, in PointCloud.RawFrameData frame, ulong tsNs) { /* ignored: uses recorded bodies */ }

        public void SubmitRecordedBodies(string serial, byte[] payload, int byteCount, ulong tsNs)
        {
            if (payload == null || byteCount <= 0) return;
            int count = RecordedBodySerializer.Decode(payload, byteCount, _decodeBuf);
            if (count <= 0) return;

            _frame.Reset(Name, serial, tsNs);
            for (int i = 0; i < count && i < MaxBodies; i++)
            {
                var skel = _pool[i];
                EvalSkeletonMap.FromK4abt(_decodeBuf[i], tsNs, skel);
                _frame.Bodies.Add(skel);
            }
            OnSkeletons?.Invoke(_frame);
        }

        public void Pump() { /* synchronous — nothing queued */ }

        public void Dispose() { }
    }
}
