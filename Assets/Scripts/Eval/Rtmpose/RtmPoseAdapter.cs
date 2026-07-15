// Track B adapter: RTMPose (top-down) + depth back-projection, normalized into
// the common EvalSkeleton. Single-person installation, so we take the highest-
// score detection per frame. Inference is synchronous, so OnSkeletons is raised
// inside SubmitFrame and measured latency includes detect+pose+lift.
//
// Requires a ready IRtmposeBackend (ORT) and per-camera ObCameraParam for the 3D
// lift. With the NullRtmposeBackend, or a frame without calibration, it emits
// nothing / no valid 3D — by design until the plugin + a calibrated recording
// are in place.

using System;
using System.Collections.Generic;
using Orbbec;
using PointCloud;
using UnityEngine;

namespace BodyTracking.Eval.Rtmpose
{
    public sealed class RtmPoseAdapter : ITrackerAdapter
    {
        public string Name => "rtmpose";
        public event Action<EvalSkeletonFrame> OnSkeletons;

        [Range(0f, 1f)] public float confThreshold = 0.3f;
        public int depthWindowHalf = 3; // 7x7 median

        private readonly IRtmposeBackend _backend;
        private readonly DepthLift _lift = new DepthLift();
        private readonly Dictionary<string, ObCameraParam?> _cam = new Dictionary<string, ObCameraParam?>();

        // reusable buffers
        private const int MaxDet = 8;
        private readonly DetBox[] _boxes = new DetBox[MaxDet];
        private float[] _input, _simccX, _simccY;
        private readonly Vector2[] _coco;
        private readonly float[] _cocoScore;
        private readonly Vector2[] _xy = new Vector2[EvalSkeleton.JointCount];
        private readonly float[] _score = new float[EvalSkeleton.JointCount];
        private readonly bool[] _valid = new bool[EvalSkeleton.JointCount];
        private readonly EvalSkeleton _skel = new EvalSkeleton();
        private readonly EvalSkeletonFrame _frame = new EvalSkeletonFrame();

        public RtmPoseAdapter(IRtmposeBackend backend = null)
        {
            _backend = backend ?? new NullRtmposeBackend();
            int k = _backend.NumKeypoints;
            _coco = new Vector2[k];
            _cocoScore = new float[k];
            var spec = _backend.Spec;
            _input = new float[3 * spec.InH * spec.InW];
            _simccX = new float[k * Mathf.RoundToInt(spec.InW * spec.SplitRatio)];
            _simccY = new float[k * Mathf.RoundToInt(spec.InH * spec.SplitRatio)];
        }

        public void Configure(in EvalCameraContext ctx) => _cam[ctx.Serial] = ctx.CameraParam;

        public void SubmitRecordedBodies(string serial, byte[] payload, int byteCount, ulong tsNs) { /* RTMPose runs inference */ }

        public void SubmitFrame(string serial, in RawFrameData frame, ulong tsNs)
        {
            if (!_backend.Ready) return;
            if (frame.ColorBytes == null || frame.ColorByteCount <= 0 || frame.ColorWidth <= 0) return;

            int nDet = _backend.Detect(frame.ColorBytes, frame.ColorWidth, frame.ColorHeight, _boxes);
            if (nDet <= 0) return;

            // single person: highest-score detection
            int best = 0;
            for (int i = 1; i < nDet && i < MaxDet; i++) if (_boxes[i].Score > _boxes[best].Score) best = i;
            var box = _boxes[best];

            var spec = _backend.Spec;
            var roi = RtmposePreprocess.RoiFromBox(box.X1, box.Y1, box.X2, box.Y2, spec);
            RtmposePreprocess.BuildInput(frame.ColorBytes, frame.ColorWidth, frame.ColorHeight, roi, spec, _input);
            if (!_backend.Pose(_input, _simccX, _simccY)) return;

            int k = _backend.NumKeypoints;
            SimccDecoder.Decode(_simccX, _simccY, k, roi, spec, _coco, _cocoScore);
            CocoToEval.Map(_coco, _cocoScore, confThreshold, _xy, _score, _valid);

            // 3D lift via aligned depth (needs calibration)
            ObCameraParam? cam = _cam.TryGetValue(serial, out var c) ? c : null;
            bool have3d = cam.HasValue && _lift.BuildAligned(frame.DepthBytes, frame.DepthWidth, frame.DepthHeight, cam.Value);

            _skel.Reset(0, tsNs);
            for (int j = 0; j < EvalSkeleton.JointCount; j++)
            {
                ref var oj = ref _skel.Joints[j];
                oj.Confidence = _score[j];
                if (!_valid[j] || !have3d) { oj.Valid = false; continue; }
                int u = Mathf.RoundToInt(_xy[j].x), v = Mathf.RoundToInt(_xy[j].y);
                float d = _lift.SampleMm(u, v, depthWindowHalf);
                if (d <= 0f) { oj.Valid = false; continue; }
                oj.PositionMm = DepthLift.Backproject(_xy[j].x, _xy[j].y, d, cam.Value);
                oj.Valid = true;
            }

            _frame.Reset(Name, serial, tsNs);
            _frame.Bodies.Add(_skel);
            OnSkeletons?.Invoke(_frame);
        }

        public void Pump() { /* synchronous */ }

        public void Dispose() => _backend.Dispose();
    }
}
