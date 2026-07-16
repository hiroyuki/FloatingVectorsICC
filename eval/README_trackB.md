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

## Backend implementation spec (confirmed, ready to code)

**Install** — add to `Packages/manifest.json`:
```json
"scopedRegistries": [{ "name": "NPM", "url": "https://registry.npmjs.com", "scopes": ["com.github.asus4"] }],
"dependencies": {
  "com.github.asus4.onnxruntime": "0.4.8",
  "com.github.asus4.onnxruntime.unity": "0.4.8"
}
```
(win-x64-gpu package only for CUDA/TensorRT; DirectML ships in core.) API namespace is
`Microsoft.ML.OnnxRuntime` (asus4 core = vendored Microsoft ORT).

**Session** (DirectML):
```csharp
var o = new SessionOptions(); o.AppendExecutionProvider_DML(0);
var session = new InferenceSession(File.ReadAllBytes(path), o);
using var input = OrtValue.CreateTensorValueFromMemory(chw, new long[]{1,3,H,W});
using var res = session.Run(null, new[]{"input"}, new[]{input}, outputNames);
ReadOnlySpan<float> x = res[0].GetTensorDataAsSpan<float>();
```

**Model I/O (verified from the .onnx):**
- RTMPose `end2end.onnx`: input `input` [1,3,256,192] → `simcc_x` [1,17,384], `simcc_y` [1,17,512] (split ratio 2).
- YOLOX `end2end.onnx`: input `input` [1,3,640,640] → `dets` [1,N,5] (x1,y1,x2,y2,score, in letterboxed 640 coords), `labels` [1,N]. NMS is baked in → detection = read dets, keep label==0 (person) & score>thr, undo letterbox (subtract pad / divide scale) to original px.
- Verify at runtime with `session.InputNames`/`OutputNames`; check `labels` dtype (int32/int64).

**Preprocess gotchas:** RTMPose = ImageNet mean/std, RGB (already in RtmposePipeline). YOLOX = letterbox to 640, likely raw 0-255 (verify BGR vs RGB and normalization empirically — silent-accuracy risk).

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
