// Track B adapter: RTMPose (top-down) + depth back-projection, normalized into
// the common EvalSkeleton. Inference is synchronous, so OnSkeletons is raised
// inside SubmitFrame and measured latency includes detect+pose+lift.
//
// Person selection: the installation is single-person in a known space, but a
// recording can contain bystanders. When a WORLD capture volume + a per-camera
// color->world transform are configured, detections are filtered to the person
// whose (box-center depth) 3D position falls inside the volume — this rejects
// bystanders and stops the skeleton from jumping between people. Without a
// volume it falls back to the highest-score detection.
//
// Requires a ready IRtmposeBackend (ORT) and per-camera ObCameraParam for the 3D
// lift; with NullRtmposeBackend / no calibration it emits nothing.

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
        private readonly Dictionary<string, ObExtrinsic> _colorToWorld = new Dictionary<string, ObExtrinsic>();

        // World capture volume (axis-aligned in world space, millimeters).
        private Vector3 _volCenter, _volHalf;
        private bool _hasVolume;

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

        /// <summary>World-space axis-aligned capture volume (mm). Detections outside it are rejected.</summary>
        public void SetCaptureVolume(Vector3 centerMm, Vector3 halfExtentsMm)
        {
            _volCenter = centerMm; _volHalf = halfExtentsMm; _hasVolume = true;
        }

        /// <summary>Per-camera color-camera -> world extrinsic (for volume tests).</summary>
        public void SetWorldTransform(string serial, ObExtrinsic colorToWorld) => _colorToWorld[serial] = colorToWorld;

        public void SubmitRecordedBodies(string serial, byte[] payload, int byteCount, ulong tsNs) { /* RTMPose runs inference */ }

        public void SubmitFrame(string serial, in RawFrameData frame, ulong tsNs)
        {
            if (!_backend.Ready) return;
            if (frame.ColorBytes == null || frame.ColorByteCount <= 0 || frame.ColorWidth <= 0) return;

            ObCameraParam? cam = _cam.TryGetValue(serial, out var c) ? c : null;
            bool have3d = cam.HasValue && _lift.BuildAligned(
                frame.DepthBytes, frame.DepthWidth, frame.DepthHeight,
                frame.ColorWidth, frame.ColorHeight, cam.Value);

            int nDet = _backend.Detect(frame.ColorBytes, frame.ColorWidth, frame.ColorHeight, _boxes);
            if (nDet <= 0) return;
            if (nDet > MaxDet) nDet = MaxDet;

            int sel = SelectPerson(nDet, serial, cam, have3d, frame.ColorWidth, frame.ColorHeight);
            if (sel < 0) return; // no person in the capture volume this frame
            var box = _boxes[sel];

            var spec = _backend.Spec;
            var roi = RtmposePreprocess.RoiFromBox(box.X1, box.Y1, box.X2, box.Y2, spec);
            RtmposePreprocess.BuildInput(frame.ColorBytes, frame.ColorWidth, frame.ColorHeight, roi, spec, _input);
            if (!_backend.Pose(_input, _simccX, _simccY)) return;

            int k = _backend.NumKeypoints;
            SimccDecoder.Decode(_simccX, _simccY, k, roi, spec, _coco, _cocoScore);
            CocoToEval.Map(_coco, _cocoScore, confThreshold, _xy, _score, _valid);

            _skel.Reset(0, tsNs);
            for (int j = 0; j < EvalSkeleton.JointCount; j++)
            {
                ref var oj = ref _skel.Joints[j];
                oj.Confidence = _score[j];
                if (!_valid[j] || !have3d) { oj.Valid = false; continue; }
                int u = Mathf.RoundToInt(_xy[j].x), v = Mathf.RoundToInt(_xy[j].y);
                float d = _lift.SampleMm(u, v, depthWindowHalf);
                if (d <= 0f) { oj.Valid = false; continue; }
                oj.PositionMm = DepthLift.Backproject(_xy[j].x, _xy[j].y, d, frame.ColorWidth, frame.ColorHeight, cam.Value);
                oj.Valid = true;
            }

            _frame.Reset(Name, serial, tsNs);
            _frame.Bodies.Add(_skel);
            OnSkeletons?.Invoke(_frame);
        }

        /// <summary>
        /// Choose the performer detection. If a capture volume + world transform +
        /// depth are available, pick the highest-score detection whose box-center
        /// 3D world position is inside the volume; otherwise the highest score.
        /// Returns -1 if a volume is configured but no detection falls inside it.
        /// </summary>
        private int SelectPerson(int nDet, string serial, ObCameraParam? cam, bool have3d, int cw, int ch)
        {
            bool useVolume = _hasVolume && have3d && cam.HasValue && _colorToWorld.TryGetValue(serial, out var _);
            if (!useVolume)
            {
                int best = 0;
                for (int i = 1; i < nDet; i++) if (_boxes[i].Score > _boxes[best].Score) best = i;
                return best;
            }

            var e = _colorToWorld[serial];
            int sel = -1; float selScore = -1f;
            for (int i = 0; i < nDet; i++)
            {
                float bx = 0.5f * (_boxes[i].X1 + _boxes[i].X2);
                float by = 0.5f * (_boxes[i].Y1 + _boxes[i].Y2);
                float d = _lift.SampleMm(Mathf.RoundToInt(bx), Mathf.RoundToInt(by), 5);
                if (d <= 0f) continue;
                Vector3 camMm = DepthLift.Backproject(bx, by, d, cw, ch, cam.Value);
                Vector3 world = ToWorld(camMm, e);
                if (!InVolume(world)) continue;
                if (_boxes[i].Score > selScore) { selScore = _boxes[i].Score; sel = i; }
            }
            return sel;
        }

        private bool InVolume(Vector3 pMm) =>
            Mathf.Abs(pMm.x - _volCenter.x) <= _volHalf.x &&
            Mathf.Abs(pMm.y - _volCenter.y) <= _volHalf.y &&
            Mathf.Abs(pMm.z - _volCenter.z) <= _volHalf.z;

        private static Vector3 ToWorld(Vector3 camMm, in ObExtrinsic e)
        {
            var R = e.Rot; var T = e.Trans;
            return new Vector3(
                R[0] * camMm.x + R[1] * camMm.y + R[2] * camMm.z + T[0],
                R[3] * camMm.x + R[4] * camMm.y + R[5] * camMm.z + T[1],
                R[6] * camMm.x + R[7] * camMm.y + R[8] * camMm.z + T[2]);
        }

        public void Pump() { /* synchronous */ }

        public void Dispose() => _backend.Dispose();
    }
}
