# Implementation Plan: Higher-resolution live point-cloud mesh (TSDF + Marching Cubes)

> Status: reviewed and APPROVED by Codex (4 rounds).
> IMPLEMENTED: Task 0-0 (per-pass ProfilerMarkers, VRAM + frame-ms per-second diag,
>   `TSDFView.useFullGridMC` A/B toggle) and Task 0-1 (persistent per-buffer active-block
>   occupancy set, write-time marking with near-surface band + halo in
>   integrate/fold/trail-bake, CompactBlocks + BuildDispatchArgs + MarchCubesActive
>   indirect dispatch). New file `Resources/TSDFBlockActive.hlsl` holds the shared
>   MarkVoxelActive. `TSDFClear.compute` gained a `ClearUint` kernel.
> VERIFIED (Mac/Metal, synthetic sphere AND real Rec2_jump playback):
>   - Real data caught a marking bug: a SYMMETRIC |sdf|<k*voxelSize band drops
>     straddling cells whose corners are all outside the band (steep real TSDF). FIXED
>     to ONE-SIDED marking (mark all sdf<0 down to -tau + a ~1*voxelSize positive rim);
>     every emitting cell has an inside corner, so completeness is gradient-independent.
>   - After fix: full-grid vs active-block are triangle-count IDENTICAL (diff=0) across
>     frames and modes. Zero-active-blocks dispatch is a safe no-op.
>   - Perf (MC pass, dispatch+GPU sync): live-follow @voxelSize 0.01 = 1% blocks active,
>     1.80ms→0.68ms (2.67x, grows finer); accumulate smeared @0.01 = 100% blocks,
>     5.51→4.66ms (1.18x); accumulate @0.02 = 26% blocks, 1.21→1.47ms (slightly slower —
>     fixed overhead not amortized when full-grid is already ~1ms). So the win is for the
>     LIVE finer-density path; at coarse voxelSize it's neutral/slightly negative.
>   `useFullGridMC` defaults to true (old path) — flip to false to enable the win.
> NOT DONE: 0-2 (integration frustum culling), 0-3 (SDF smoothing), 0-4 (gradient
>   normals), 0-5 (voxelSize sweep / exit gate). End-to-end verification with the real
>   jump recording must run on the Windows box (recordings live at D:\...).

## Project context

Unity 6 URP real-time installation. A dense TSDF volume is integrated from multi-camera
depth+color (Orbbec Femto Bolt, up to 4 cams) and re-meshed on the GPU via Marching Cubes.
Single-person use case. The user wants finer meshes without paying the full compute cost of
uniformly shrinking voxels ("coarse everywhere, refine/compute only where the surface is").

## Confirmed current architecture (read from source)

- **Storage**: DENSE flat `ComputeBuffer<float2>` (x=sdf metres, y=weight), one element per
  voxel over the whole bbox. Double-buffered + a parallel dense `float4` color buffer. The
  double-buffered SDF+color totals ~48 bytes per voxel (NOT per buffer). Allocated in
  `TSDFVolume.RebuildIfNeeded()` (`TSDFVolume.cs:534-549`). `voxelSize` default 0.05 m, range
  0.001–0.1. Hard guard clamps voxelSize UP if total exceeds `MaxVoxels = 150,000,000`
  (`TSDFVolume.cs:497-520`). Tau = tauMultiplier(4) * voxelSize.
  - There is a single-buffer / accumulate path where `FrontBuffer == WriteBuffer`
    (`_instSdf/_instColor` scratch + fold, `TSDFVolume.cs:239-275`). Any read-modify step that
    writes the front buffer would feed back into future integration — see smoothing task.
- **Marching Cubes**: GPU compute, no CPU readback. `Resources/TSDFMarchingCubes.compute`,
  kernel `MarchCubes` `[numthreads(4,4,4)]`, dispatched over ALL `(dim-1)^3` cells every
  publish (`TSDFView.cs:371-374`). Per-cell early-outs only: weight gate + empty edge mask.
  Emits ONE `Tri` (3 verts, each float3 pos + float3 color, stride 72) per Append to avoid
  cross-cell interleaving (fixed a real artifact bug — invariant must be preserved). `ScaleArgs`
  multiplies tri count ×3 and clamps to `_MaxTriangles` (default 333,333). Drawn via
  `DrawProceduralIndirect` reading the append buffer directly — triangle soup, no shared
  vertices, no index buffer.
- **Integration**: `TSDFIntegrator` → `TSDFIntegrate.compute` dispatched over the FULL volume
  `Dim.x*Dim.y*Dim.z` once PER CAMERA PER FRAME (`TSDFIntegrator.cs:437` area). With 4 cameras
  this is 4×O(volume) per frame — a first-class cost at fine voxelSize.
- **Cadence**: MC re-extracts when `TSDFVolume.PublishVersion` bumps (per multi-cam batch),
  gated by `mcEveryNFrames` (default 1). `TSDFTrailBaker` (analytic capsules) is out of scope,
  unchanged.

## Key reframing (drives the phase split)

At the user's likely scale (≈2 m box, target voxelSize 0.025–0.03 m → ≈80^3 ≈ 0.5 M voxels
≈ 24 MB TOTAL for the double-buffered SDF+color), the `MaxVoxels` memory ceiling
(dim ≈ 530^3 ≈ 7.2 GB) is NOT the wall. Target VRAM is 12–16 GB. The real cost that grows is
**compute**, and it has TWO sources, both currently O(volume):
  (a) MC dispatches every cell each publish;
  (b) integration dispatches every voxel per camera per frame (×4 cams).
The goal is to make BOTH surface/observation-proportional without a sparse-storage rewrite.

Honest complexity note: a naive `MarkActiveBlocks` that scans the whole grid is still
O(volume) for occupancy discovery (plus O(surface) MC). That alone can still beat full MC, but
does not make discovery surface-proportional. To actually cut the O(volume) term we maintain a
PERSISTENT per-physical-buffer active-block occupancy set, updated AT WRITE TIME by the kernels
that already touch voxels (integrate, fold, trail-bake), so MC consumes a compacted list of
currently-active FRONT blocks produced as a near-free side effect — no separate full-grid scan.
The full-grid `MarkActiveBlocks` is kept only as a correctness fallback / A-B reference. Exit
conditions below are defined by measurement, not assertion.

CRITICAL correctness principle (per review round 2): MC rebuilds the WHOLE triangle buffer each
publish, so the active set it consumes must contain EVERY block that currently emits triangles
in the authoritative front volume — NOT merely blocks changed in the latest batch. "Dirty this
batch" and "active in the front volume" are different sets; conflating them drops unchanged-but-
visible geometry (especially in accumulate mode, where `TSDFFold` preserves old surfaces while
the current instant touches only new voxels). Therefore the active-block set is treated as an
occupancy bitset with the SAME clear/build/publish/swap lifecycle as the SDF buffer it
describes.

Also: MC output is a triangle soup with no vertex connectivity, so mesh-domain smoothing
(Laplacian/Taubin) is not applicable. Smoothing must be done in the SDF (volume) domain, and
NON-DESTRUCTIVELY (never write the authoritative front buffer).

## Scope

- In: `TSDFIntegrator → TSDFVolume → TSDFView(MC)`. Lower voxelSize; keep MC AND integration
  cost proportional to surface/observation; improve perceived smoothness at a given resolution.
- Out: octree/Dual Contouring/Transvoxel, multi-person, TrailBaker high-res, mesh-domain smoothing.

---

## Phase 0 — make COMPUTE sparse (primary; storage layout unchanged; low risk)

### Task 0-0 (FIRST): Per-pass instrumentation + A/B toggle (prerequisite for every claim)
Before any optimization, add real measurement so exit conditions are data-driven.
- Use `ProfilerRecorder` for CPU-side per-pass markers (reliable across backends). GPU per-
  dispatch timing is best-effort: attempt `CommandBuffer` timestamp queries where the backend
  supports them, otherwise fall back to clearly-labeled CPU-marker timing. Time each pass
  SEPARATELY: integration, occupancy-compact, MC, smoothing. Log ms/pass/frame.
- Add real VRAM/buffer-memory reporting computed from actual `ComputeBuffer.count * stride`
  summed over all TSDF buffers (not a hand estimate).
- Add an Inspector A/B toggle `useFullGridMC` that forces the OLD full-`(dim-1)^3` dispatch, so
  every new path can be compared for triangle-count + visual parity on the SAME published volume.
- Capture a baseline table at voxelSize 0.05 and 0.03 (integration ms, MC ms, tris, VRAM)
  BEFORE optimizing. This baseline defines the Phase 0 exit target.

### Task 0-1: Active-block MC via a persistent per-buffer active-block set
Cut both the MC full-grid dispatch AND the occupancy full-grid scan, WITHOUT dropping
unchanged-but-visible geometry.
1. Define a block grid of 8^3 cells: `bdim = ceil((dim-1)/8)` per axis. Allocate a persistent
   `RWStructuredBuffer<uint> _BlockActive` occupancy bitset of length `bdim.x*bdim.y*bdim.z`,
   DOUBLE-BUFFERED as `_BlockActiveFront` / `_BlockActiveWrite` — one pair mirroring the SDF
   `FrontBuffer`/`WriteBuffer`. The active set describes a physical buffer and travels with it.
2. **Lifecycle = identical to the SDF buffer it describes**:
   - `Clear()` (`TSDFVolume.cs:296`) clears BOTH SDF buffers and bumps `PublishVersion` WITHOUT
     a swap, so it must zero BOTH `_BlockActiveFront` AND `_BlockActiveWrite` (else the front
     active set no longer matches the cleared front SDF). `ClearWrite()` (live-follow's pre-batch
     clear) zeros ONLY `_BlockActiveWrite` in double-buffer mode.
   - **Single-buffer / accumulate mode semantics**: when `FrontBuffer == WriteBuffer`, there is
     ONE active set (front and write ALIAS it); `Publish()` only bumps the version and does NOT
     swap active sets. Old active bits PERSIST across publishes (they describe accumulated
     surfaces `TSDFFold` preserves) — the implementer must NOT clear the active set per instant,
     or preserved accumulated surfaces are dropped. The active set follows the SDF buffer's mode
     exactly: double-buffered → paired + swapped; single-buffer → one set, no swap.
   - **Mark at write time** in EVERY kernel that establishes an observable voxel in the write
     buffer: `TSDFIntegrate.compute` (live-follow), `TSDFFold.compute` (`TSDFVolume.cs:239-259`,
     accumulate mode — MUST mark, or accumulated surfaces are missed), and `TSDFTrailBake.compute`.
     When a voxel becomes observable (weight≥gate), set its owning block AND the negative-
     direction neighbor blocks it is a corner-cell of (halo, see 2a) in `_BlockActiveWrite`
     (atomic-OR / store 1).
   - `Publish()` (`TSDFVolume.cs:337`): swap `_BlockActiveFront`↔`_BlockActiveWrite` ATOMICALLY
     with the SDF/color front/write swap, so the front active set always matches the front SDF.
   - MC consumes ONLY `_BlockActiveFront`. Nothing "clears dirty after MC" — the set persists
     with its buffer and is reset only by Clear/ClearWrite, exactly like the SDF data.
   2a. **Halo**: an MC cell straddles block borders (cell `(cx,cy,cz)` reads corners `cx..cx+1`).
   A written voxel at index `v` is a corner of cells owned by blocks `floor((v-1)/8)` and
   `floor(v/8)` per axis, so mark the owning block plus its negative-direction neighbors (up to
   the 8 cell-blocks the voxel is a corner of). This guarantees no border-crossing surface is
   dropped.
3. **Compact**: a small kernel over the block grid reads `_BlockActiveFront` and appends set-bit
   block indices into `AppendStructuredBuffer<uint> _ActiveBlocks`. (Fallback A/B path:
   `MarkActiveBlocks` that recomputes occupancy by scanning all cells of a block via a per-block
   thread-group reduction — 512 threads cooperating, NOT one thread scanning 512 cells — used
   only when `useFullGridMC`/reference mode is on, to validate the persistent set drops nothing.)
4. **Indirect MC** — explicit plumbing:
   - Reset `_ActiveBlocks` counter (`SetCounterValue(0)`) every extraction.
   - `_ActiveBlocks` capacity = `bdim.x*bdim.y*bdim.z` (worst case all blocks active).
   - `CopyCount(_ActiveBlocks, _dispatchArgs, 0)` into a SEPARATE 3-uint
     `ComputeBufferType.IndirectArguments` `_dispatchArgs` (distinct from the 4-uint DRAW args
     buffer). A tiny kernel writes `_dispatchArgs = [activeCount, 1, 1]`.
   - `DispatchIndirect(MarchCubes, _dispatchArgs)` with `[numthreads(8,8,8)]` = one group per
     active block. Map `groupID → _ActiveBlocks[gid] → block base cell`,
     `groupThreadID → cell within block`. Keep existing per-cell weight/edge gates + the
     one-Append-per-triangle invariant.
   - **Zero active blocks**: guard so the DRAW args vertex count is cleared to 0 (draw nothing)
     rather than reading a stale count. Verify `DispatchIndirect` with count 0 is safe on the
     target GPU; if not, skip the dispatch when `activeCount==0`.
   - Add all new buffers (`_BlockActiveFront`/`_BlockActiveWrite` — or a single aliased set in
     single-buffer mode — plus `_ActiveBlocks`, `_dispatchArgs`) to `EnsureBuffers`/
     `ReleaseBuffers`; reallocate when `bdim` changes (voxelSize/bbox change).
- Files: `TSDFMarchingCubes.compute` (+kernels, rewrite indexing), `TSDFIntegrate.compute`
  (active-mark + halo), `TSDFFold.compute` (active-mark + halo, accumulate mode),
  `TSDFTrailBake.compute` (active-mark + halo), `TSDFView.cs` (dispatch plumbing, buffers),
  `TSDFVolume.cs` (own `_BlockActiveFront`/`_BlockActiveWrite`, swap them inside `Publish()`;
  `Clear()` zeros BOTH sets, `ClearWrite()` zeros only the write set; single-buffer mode uses one
  aliased set with no swap).
- Validation: mesh matches full-grid MC (tri count + visual parity via A/B toggle); MC ms and
  occupancy ms both drop; confirm no missed thin surfaces (see risk note).

### Task 0-2: Integration cost reduction (only if 0-0 shows integration dominates)
Don't declare done while 4×O(volume) integration dominates. Gated on measurement.
- If integration ms >> MC ms at target voxelSize, add camera-frustum block culling: skip
  integrate threadgroups whose block AABB is fully outside the camera frustum / beyond depth
  range (dispatch integration per active-or-frustum block, mirroring 0-1's indirect scheme), or
  lower integration cadence / stagger cameras across frames.
- If integration is already minor vs MC, record that and skip — keep Phase 0 lean.

### Task 0-3: SDF-domain smoothing (non-destructive; new kernel)
`TSDFSmoothUnion`'s SmoothUnion blends two SDFs, not a blur. Add a separable Gaussian blur.
- **Non-destructive**: never write the authoritative front buffer. Read front → write a
  dedicated smoothed scratch buffer; bind the SCRATCH only to MC. Voxel/Cell views keep reading
  raw front. In single-buffer accumulate mode this is mandatory (front==write feeds integration).
- Separable blur needs ping-pong: allocate TWO scratch buffers (X pass front→scratchA,
  Y pass scratchA→scratchB, Z pass scratchB→scratchA), MC binds the final one. Weight-gated:
  only average observed voxels (weight≥gate); never pull in the +tau/weight=0 seed; renormalize
  by summed weights.
- Restrict blur to active blocks (reuse 0-1's `_ActiveBlocks`) so it stays surface-proportional.
- Strength = Inspector slider, 0 = off (binds raw front, zero overhead). Conservative default;
  blur thins fine features.
- Files: new `TSDFResmooth.compute`, scratch buffers + hook in `TSDFView.cs`/`TSDFVolume.cs`.

### Task 0-4: SDF-gradient normals for shading (robust)
`Tri` carries pos+color only (stride 72). Add per-vertex normals for smooth shading at low res.
- Compute the SDF gradient at the 8 CELL CORNERS (central difference), then interpolate the
  corner gradients along each crossed MC edge with the same `InterpT` used for position/color —
  do NOT sample one nearest voxel per emitted vertex.
- Robustness: clamp sample indices at grid edges (one-sided differences at borders); skip
  neighbors with weight<gate (use one-sided from valid side); if the gradient is degenerate
  (‖∇‖≈0) fall back to the triangle FACE normal. Normalize, then transform to world space with
  the inverse-transpose of `_WorldFromVoxel`.
- `Tri` stride 72→108 (add n0/n1/n2). Update in lockstep: `TSDFMarchingCubes.compute` struct,
  `TSDFView.cs` `triStride`/reallocation, `TSDFMesh.shader` (read normal + lighting). The
  72-byte contract asserted by comments in both files must be updated to 108 everywhere.

### Task 0-5: Measure & tune voxelSize (exit gate)
Via Unity MCP: `Application.runInBackground=true` → EnterPlaymode → Recorder Play (playback).
Sweep voxelSize 0.05 → 0.03 → 0.025; record the 0-0 table (integration ms, MC ms, smooth ms,
tris, VRAM). **Phase 0 exit condition (explicit)**: at the chosen voxelSize, total frame time
stays within the real-time budget (define target fps, e.g. ≥ the current 0.05 baseline fps),
integration is not the silent bottleneck (else do 0-2), memory has headroom, and the mesh is
visibly finer + smoother than the 0.05 baseline. If the exit condition can't be met by tuning,
that is the documented trigger for Phase 1 — not an assumption.

---

## Phase 1 — sparse storage (conditional fallback; ONLY if Phase 0 measurements hit the memory ceiling)

Block-hashing narrow-band TSDF: 8^3 blocks allocated lazily where depth/capsules touch; all
kernels indirect-dispatched over active blocks; MC/CC block boundaries handled via 1-voxel
apron. Integrate path first; TrailBaker on dense fallback. Detailed design done separately.
NOT begun unless Phase 0 data justifies it.

---

## Validation (all phases)
Per project CLAUDE.md, AI-driven: `Application.runInBackground=true`, EnterPlaymode, Recorder
Play, `read_console` for compile/errors, `Object.FindObjectsByType` + the 0-0 instrumentation to
read per-pass ms / tri count / VRAM. Use the A/B `useFullGridMC` toggle for parity checks.
Console must be error-free before any merge to main. Stop before recompiling (Play blocks
recompilation); the Recorder Play state is lost across EnterPlaymode/ExitPlaymode and must be
re-armed.

## Risks / notes
- **Active-vs-dirty (0-1, review round 2 core)**: MC needs ALL currently-active FRONT blocks,
  not blocks changed this batch. Mitigation is structural: `_BlockActive` is a persistent
  occupancy set per physical buffer with the SAME clear/build/publish/swap lifecycle as the SDF
  buffer (zeroed on Clear/ClearWrite, marked by integrate/fold/trail-bake, swapped in Publish).
  In live-follow the write buffer is cleared and fully rebuilt each batch, so its active set is
  too; in accumulate mode `TSDFFold` preserves surfaces AND marks them active, so nothing is
  dropped. Validate parity against the full-grid `MarkActiveBlocks` reference via the A/B toggle.
- **Border-crossing surface**: an MC cell reads corners from neighboring blocks. Mitigation: the
  write-time halo marks the owning block plus negative-direction neighbor cell-blocks (2a), so a
  surface crossing a block boundary is always covered. The fallback reduction independently
  scans all block cells and is used to confirm no blocks are missed.
- **Swap atomicity**: `_BlockActiveFront/_BlockActiveWrite` MUST swap together with the SDF/color
  front/write inside `Publish()`; a partial swap desynchronizes the mesh from the volume.
- **Stride/args hazards**: distinct dispatch-args (3-uint) vs draw-args (4-uint); reset append
  counters each extraction; keep the 108-byte `Tri` contract in lockstep; clear draw args on
  zero active blocks.
- **Smoothing**: non-destructive scratch only; thins fine features → conservative default.
- Preserve the one-Append-per-triangle MC invariant through all refactors.
