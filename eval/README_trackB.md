# Track B — RTMPose + depth back-projection (eval/rtmpose-depth)

Top-down 2D pose (RTMPose) on the color image, lifted to 3D via the recorded
depth aligned to the color camera, normalized into the shared `EvalSkeleton`.

## Code (`Assets/Scripts/Eval/Rtmpose/`, asmdef `BodyTracking.Eval.Rtmpose`)
- `RtmposePipeline.cs` — preprocess (top-down affine + ImageNet norm, RGB), SimCC decode, COCO-17→15-joint map (Pelvis=hip mid, Neck=shoulder mid, Head=nose).
- `DepthLift.cs` — forward-project depth→color (z-buffer), median-sample window, back-project (u,v,d)→color-camera mm.
- `IPoseInference.cs` — `IRtmposeBackend` (YOLOX detect + RTMPose) with `NullRtmposeBackend` stub.
- `RtmPoseAdapter.cs` — `ITrackerAdapter`; single-person (top-score box), synchronous, emits `EvalSkeleton`.

Status: **compiles green with the stub backend.** Produces no skeletons until a
real backend is wired and a calibrated recording is used.

## Models (downloaded, gitignored — `eval/models/`)
- RTMPose-m body7 256×192: `rtmpose-m/20230831/rtmpose_onnx/rtmpose-m_simcc-body7_pt-body7_420e-256x192-e48f03d0_20230504/end2end.onnx`
- YOLOX-m person (Human-Art): `yolox-m/20230928/yolox_onnx/yolox_m_8xb8-300e_humanart-c2c7a14a/end2end.onnx`
- (RTMPose-l available to swap in if accuracy is short.)

## Remaining work
1. **ORT plugin + real backend** — add `asus4/onnxruntime-unity` (UPM, ORT 1.26; DirectML GPU out of the box), implement `IRtmposeBackend` (YOLOX decode + RTMPose SimCC) against it. CUDA/TensorRT EP later (needs CUDA 12.x + cuDNN 9.x; box currently has 11.6/cuDNN8).
2. **Calibrated recording** — 3D lift needs `ObCameraParam` (intrinsics + D2C). Current local recordings lack `extrinsics.yaml`, so lift yields nothing. Needs the long calibrated recording.
3. **Verify BGR vs RGB** empirically against the exported model (research flagged this as a silent-accuracy risk). Preprocess currently assumes RGB.
4. **YOLOX output decode** (grid/stride, NMS) in the backend.
5. Register the adapter with `EvalRunner` (a small setup MonoBehaviour) once the backend is real.

## Frame note
RTMPose 3D is in the **color-camera** frame; k4abt baseline is depth-camera frame.
Per-tracker metrics (jitter/continuity) are frame-relative so this is fine; absolute
cross-tracker position comparison would need a common frame.
