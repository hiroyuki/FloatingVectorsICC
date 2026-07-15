// Track B adapter: RTMPose (top-down) + depth back-projection, normalized into
// the common EvalSkeleton. Inference is synchronous, so OnSkeletons is raised
// inside SubmitFrame and measured latency includes detect+pose+lift.
//
// Person selection uses a WORLD capture volume (single-person, known space):
// detections are filtered to the person whose box-center 3D world position is
// inside the volume, rejecting bystanders.
//
// Latency: YOLOX detection dominates (~126ms vs ~20ms RTMPose). So we DETECT
// only to (re)acquire the person, then TRACK by deriving the next frame's box
// from the current keypoints — re-detecting every redetectEveryN frames, when
// pose confidence drops, or when the tracked person leaves the volume. Most
// frames skip YOLOX entirely (~6x faster).
//
// Requires a ready IRtmposeBackend and per-camera ObCameraParam for the 3D lift.

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
        public int depthWindowHalf = 3;      // 7x7 median
        public int redetectEveryN = 30;      // safety re-detection interval
        public float minTrackConf = 0.35f;   // below this mean keypoint score, re-detect
        public float trackPad = 0.15f;       // keypoint-bbox padding fraction for the next box

        private readonly IRtmposeBackend _backend;
        private readonly DepthLift _lift = new DepthLift();
        private readonly Dictionary<string, ObCameraParam?> _cam = new Dictionary<string, ObCameraParam?>();
        private readonly Dictionary<string, ObExtrinsic> _colorToWorld = new Dictionary<string, ObExtrinsic>();

        private Vector3 _volCenter, _volHalf;
        private bool _hasVolume;

        private struct Track { public float X1, Y1, X2, Y2; public int Age; public bool Valid; }
        private readonly Dictionary<string, Track> _track = new Dictionary<string, Track>();

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
        public void SetCaptureVolume(Vector3 centerMm, Vector3 halfExtentsMm) { _volCenter = centerMm; _volHalf = halfExtentsMm; _hasVolume = true; }
        public void SetWorldTransform(string serial, ObExtrinsic colorToWorld) => _colorToWorld[serial] = colorToWorld;
        public void SubmitRecordedBodies(string serial, byte[] payload, int byteCount, ulong tsNs) { }

        public void SubmitFrame(string serial, in RawFrameData frame, ulong tsNs)
        {
            if (!_backend.Ready) return;
            if (frame.ColorBytes == null || frame.ColorByteCount <= 0 || frame.ColorWidth <= 0) return;
            int cw = frame.ColorWidth, ch = frame.ColorHeight;

            ObCameraParam? cam = _cam.TryGetValue(serial, out var c) ? c : null;
            bool have3d = cam.HasValue && _lift.BuildAligned(frame.DepthBytes, frame.DepthWidth, frame.DepthHeight, cw, ch, cam.Value);

            // detect (to acquire) vs track (reuse previous keypoint box)
            DetBox box; bool tracking = false;
            if (_track.TryGetValue(serial, out var tr) && tr.Valid && tr.Age < redetectEveryN)
            {
                box = new DetBox { X1 = tr.X1, Y1 = tr.Y1, X2 = tr.X2, Y2 = tr.Y2, Score = 1f };
                tracking = true;
            }
            else
            {
                int nDet = _backend.Detect(frame.ColorBytes, cw, ch, _boxes);
                if (nDet <= 0) { _track.Remove(serial); return; }
                if (nDet > MaxDet) nDet = MaxDet;
                int sel = SelectPerson(nDet, serial, cam, have3d, cw, ch);
                if (sel < 0) { _track.Remove(serial); return; }
                box = _boxes[sel];
            }

            var spec = _backend.Spec;
            var roi = RtmposePreprocess.RoiFromBox(box.X1, box.Y1, box.X2, box.Y2, spec);
            RtmposePreprocess.BuildInput(frame.ColorBytes, cw, ch, roi, spec, _input);
            if (!_backend.Pose(_input, _simccX, _simccY)) { _track.Remove(serial); return; }

            int k = _backend.NumKeypoints;
            SimccDecoder.Decode(_simccX, _simccY, k, roi, spec, _coco, _cocoScore);
            CocoToEval.Map(_coco, _cocoScore, confThreshold, _xy, _score, _valid);

            _skel.Reset(0, tsNs);
            float x1 = float.MaxValue, y1 = float.MaxValue, x2 = float.MinValue, y2 = float.MinValue;
            float confSum = 0; int confN = 0;
            for (int j = 0; j < EvalSkeleton.JointCount; j++)
            {
                ref var oj = ref _skel.Joints[j];
                oj.Confidence = _score[j];
                if (_valid[j])
                {
                    confSum += _score[j]; confN++;
                    x1 = Mathf.Min(x1, _xy[j].x); y1 = Mathf.Min(y1, _xy[j].y);
                    x2 = Mathf.Max(x2, _xy[j].x); y2 = Mathf.Max(y2, _xy[j].y);
                }
                if (!_valid[j] || !have3d) { oj.Valid = false; continue; }
                float d = _lift.SampleMm(Mathf.RoundToInt(_xy[j].x), Mathf.RoundToInt(_xy[j].y), depthWindowHalf);
                if (d <= 0f) { oj.Valid = false; continue; }
                oj.PositionMm = DepthLift.Backproject(_xy[j].x, _xy[j].y, d, cw, ch, cam.Value);
                oj.Valid = true;
            }

            // volume re-check while tracking: if the tracked person left the volume, re-detect next frame
            if (tracking && _hasVolume && have3d && _colorToWorld.TryGetValue(serial, out var e))
            {
                ref var pelvis = ref _skel.Joints[(int)EvalJointId.Pelvis];
                if (pelvis.Valid && !InVolume(ToWorld(pelvis.PositionMm, e))) { _track.Remove(serial); return; }
            }

            // update the tracked box from the keypoint bounds (padded) for the next frame
            float meanConf = confN > 0 ? confSum / confN : 0f;
            if (confN >= 3 && x2 > x1 && y2 > y1)
            {
                float pw = (x2 - x1) * trackPad, ph = (y2 - y1) * trackPad;
                _track[serial] = new Track
                {
                    X1 = x1 - pw, Y1 = y1 - ph, X2 = x2 + pw, Y2 = y2 + ph,
                    Age = tracking ? tr.Age + 1 : 0,
                    Valid = meanConf >= minTrackConf,
                };
            }
            else _track.Remove(serial);

            _frame.Reset(Name, serial, tsNs);
            _frame.Bodies.Add(_skel);
            OnSkeletons?.Invoke(_frame);
        }

        private int SelectPerson(int nDet, string serial, ObCameraParam? cam, bool have3d, int cw, int ch)
        {
            bool useVolume = _hasVolume && have3d && cam.HasValue && _colorToWorld.ContainsKey(serial);
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
                float bx = 0.5f * (_boxes[i].X1 + _boxes[i].X2), by = 0.5f * (_boxes[i].Y1 + _boxes[i].Y2);
                float d = _lift.SampleMm(Mathf.RoundToInt(bx), Mathf.RoundToInt(by), 5);
                if (d <= 0f) continue;
                Vector3 world = ToWorld(DepthLift.Backproject(bx, by, d, cw, ch, cam.Value), e);
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

        public void Pump() { }
        public void Dispose() => _backend.Dispose();
    }
}
