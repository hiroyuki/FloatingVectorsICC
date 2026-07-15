# Body-Tracking Evaluation — k4abt vs RTMPose (Nuitrack deferred)

Fact-based comparison for the FloatingVectorsICC installation (1 person, 4× Femto
Bolt, Unity 6 / Windows). Recommendation is separated from measured facts.

## Setup
- **Session**: `2026-07-14_15-50-24` — 4 cameras, ~53 s, depth 320×288 (Y16) + color
  1280×960 (RGB8), with `calibration/extrinsics.yaml` (intrinsics + D2C + world).
- **Harness**: shared `EvalReplayDriver` replays the recorded RCSV to each tracker;
  metrics computed **per (tracker, camera)**, **single-camera** (no fusion yet).
- **k4abt** = the recorded `bodies_main` (the current system's own output).
- **RTMPose** = ONNX Runtime (asus4, **DirectML** EP), top-down: YOLOX-m person
  detector → RTMPose-m (SimCC, COCO-17) → depth back-projection to 3D, with a
  **world capture-volume** person selection. Mapped to the common 15-joint subset.
- **Nuitrack (Track A): NOT evaluated.** Nuitrack is sensor-only (no API to feed
  recorded frames; only OpenNI2 `.oni`/RealSense `.bag`, not our Orbbec RCSV), so it
  can't run on the same recording. It needs a **live 4-camera rig** — deferred until
  hardware is connected (a separate live A-1 run: Nuitrack live while recording RCSV).
- **Metrics**: 600 frames (150/camera). **Jitter is whole-range** (no static-hold
  segment is labeled in this recording) → it reflects real motion + noise, **not
  pure sensor jitter**. RTMPose latency is **warmup-excluded** (DirectML shader
  compile on the first frames dropped).

## Results (per camera, 150 frames each)

| cam | tracker | continuity | valid joints | spike% | jitter (mm) | latency mean / p95 (ms) |
|-----|---------|-----------:|-------------:|-------:|------------:|------------------------:|
| N  | k4abt   | 100% | 15.0 | 6.9% | 124 | — (recorded) |
| N  | rtmpose | 95%  | 14.8 | 5.0% | 191 | 291 / 375 |
| L  | k4abt   | 100% | 15.0 | 8.6% | 122 | — |
| L  | rtmpose | 100% | 14.6 | 5.3% | **118** | 284 / 356 |
| EG | k4abt   | 100% | 15.0 | 9.1% | 127 | — |
| EG | rtmpose | 98%  | 14.6 | 4.8% | 285 | 290 / 354 |
| Z  | k4abt   | 100% | 15.0 | 8.6% | 117 | — |
| Z  | rtmpose | 100% | 14.1 | 5.0% | 158 | 285 / 358 |

k4abt latency is N/A here (replayed recorded output); live k4abt is typically
~15–30 ms/frame. idSwaps = 0 for both (single tracked person after volume selection).

## Findings (measured)
- **Latency**: RTMPose ≈ **285 ms/frame** (p95 ≈ 360 ms) as implemented — YOLOX-640 +
  RTMPose + a CPU/C# depth-align (92k-px loop). Roughly **10× slower than live
  k4abt**. Not real-time (~3.5 fps) without optimization.
- **Accuracy (whole-range jitter)**: RTMPose ≈ k4abt on the frontal camera **L (118 vs
  122 mm)**; noisier on oblique/edge views (Z 158, N 191, **EG 285** vs k4abt
  117–127). k4abt is more **viewpoint-consistent** (117–127 across all cameras).
- **Temporal stability (spike rate)**: RTMPose is actually **lower** (4.8–5.3%) than
  k4abt (6.9–9.1%) — smooth frame-to-frame once person-switching is removed.
- **Continuity**: both ≥95%; k4abt 100%. RTMPose loses a few frames where the person
  is at/beyond the frame edge (EG) — recoverable by **multi-camera fusion**.
- **Joints**: k4abt reports 15/15 (incl. predicted); RTMPose 14.1–14.8 (drops
  low-confidence joints at conf-threshold 0.3).
- **Multi-person — the decisive lesson**: the recording contains **bystanders** (up to
  **8 detections/frame**). Naive highest-score selection made RTMPose **jump between
  people** (cam L jitter **1079 mm**). A **world capture-volume person selection**
  (implemented) fixed it: **L jitter 1079 → 118 mm**, matching k4abt. Top-down RTMPose
  **requires** a capture ROI/volume + identity handling in production; k4abt was
  stable here without it.
- **Depth surface vs joint center**: RTMPose lifts to the body **surface** depth;
  k4abt estimates the joint **center** (slightly inside). A visible, systematic offset
  — evaluated raw (uncorrected) per the task.

## Effort / gotchas / production work
- **Effort**: harness + baseline reused directly; RTMPose backend ~1 day equivalent
  (ONNX models are pre-exported; asus4 ORT = vendored Microsoft.ML.OnnxRuntime).
- **Gotchas hit**: vendored ORT NREs on a `null` RunOptions; SimCC scores are **raw
  (un-normalized, can exceed 1)** so the confidence threshold is on a raw scale;
  BGR-vs-RGB had to be verified (YOLOX BGR, RTMPose RGB); intrinsics must be scaled to
  the actual frame resolution; batchmode `-nographics` disables DirectML (runs needed
  the interactive Editor / GPU).
- **To make RTMPose production-viable**: (1) **CUDA/TensorRT EP + GPU depth-align +
  detection crop/skip** to hit real-time; (2) **per-camera capture ROI/volume** (done
  in eval) for multi-person robustness *and* speed; (3) **multi-camera
  confidence-weighted fusion** for edge coverage; (4) optional joint-center depth
  correction.

## Recommendation (opinion, grounded in the above)
- **k4abt** remains a strong baseline here: stable, viewpoint-consistent, fast, 32
  joints, free, no per-camera ROI needed. The original motivation (dissatisfaction with
  joint accuracy) is **not directly quantified** in this run — it needs ground truth;
  on jitter/continuity/stability k4abt is comparable-to-better than RTMPose *as
  implemented*.
- **RTMPose** matches k4abt accuracy on good viewpoints and has a lower spike rate, but
  is **~10× slower as-implemented** and only works cleanly with the capture volume +
  (for full coverage) fusion. It is a viable path **if** the latency/ROI/fusion work is
  funded; otherwise it does not beat k4abt on this data.
- **Nuitrack**: cannot be judged offline; schedule a **live trial** on the rig (trial
  license from cognitive.3divi.com; Femto Bolt officially supported; Unity 6 unverified
  by the vendor). Only then can it enter the comparison.

## Reproduce
Interactive Editor on `../FloatingVectorsICC-eval-rtmpose`, then MCP execute_code:
`RtmposeCompareChunked.Start(<session>,150,0.3,15)` → `Step(80)` (repeat) → `Finish("")`.
Single-frame visual check: `RtmposeVerify.Run(<session>,"PAN-SHI",<serial>,<frame>,true,0.3,true,true)`.
CSV: `eval/results/compare/`.
