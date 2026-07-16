# Body-Tracking Evaluation Harness

Shared, tracker-agnostic harness for comparing body-tracking approaches
(k4abt baseline / Nuitrack / RTMPose) on the same recorded input.

Lives in `Assets/Scripts/Eval/` (asmdef `BodyTracking.Eval`). This branch
(`eval/harness-base`) is the common ancestor of `eval/nuitrack-ai` and
`eval/rtmpose-depth`; fixes here merge into both.

## Pieces
- `EvalSkeleton.cs` — common 15-joint skeleton format (`EvalJointId`), camera-space mm, OpenCV frame.
- `ITrackerAdapter.cs` — tracker abstraction (`SubmitFrame` → `OnSkeletons`), swappable.
- `EvalReplayDriver.cs` — deterministic RCSV replay (reuses `PointCloudRecording.RcsvFrameStream`).
- `K4abtBaselineAdapter.cs` — decodes recorded `bodies_main` (current k4abt output) as the baseline.
- `EvalMetrics.cs` — jitter / spikes / continuity / latency / ID-swaps → CSV.
- `EvalRunner.cs` — orchestrator (`RequireComponent(EvalReplayDriver)`).
- `Editor/EvalRunnerEditor.cs` — Load / Run / Export buttons.

## Run a baseline (recorded k4abt) from the editor
1. Empty scene → GameObject with **EvalRunner** (auto-adds **EvalReplayDriver**).
2. In the EvalRunner inspector set **Session Root** to a recorded session dir, e.g.
   `D:\Dropbox\projects\ICC\Recordings\RecordingBase\2026-07-14_11-29-59`
   (the dir containing `dataset/` and `calibration/`).
3. Enter Play. Click **Load & Run (single pass)**. When the pass ends it auto-finishes;
   or click **Finish & Export CSV**.
4. CSVs land in `<project>/eval/results/` (`summary.csv`, `jitter_k4abt.csv`).

## Metrics notes
- Computed on RAW output — no One-Euro / smoothing.
- Baseline latency from recorded bodies is ~0 (synchronous decode) and NOT
  representative of live k4abt inference; measure that with the live-worker mode (TODO).
- Jitter window: set `staticStartSec`/`staticEndSec` on EvalRunner to a static-pose
  segment; leave equal for whole-recording.

## Adding a tracker (per track branch)
Implement `ITrackerAdapter` and register it, e.g. in a small MonoBehaviour:
```csharp
GetComponent<EvalRunner>().Register(new RtmPoseAdapter(...));
```
Keep `includeBaseline = true` so every run is compared against k4abt.

## Current data caveat
Only short (~1-2 s), single-person sessions with no extrinsics are present locally.
Full metric numbers need a longer multi-segment recording (static hold / occlusion /
2-person) — see `eval/PLAN_eval.md`.
