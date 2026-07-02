#ifndef TSDF_BLOCK_ACTIVE_INCLUDED
#define TSDF_BLOCK_ACTIVE_INCLUDED

// Shared write-time active-block marking for the active-block Marching Cubes path
// (tsdf-hires-mesh-plan.md, Phase 0 task 0-1). Every kernel that establishes an
// observable voxel in a buffer MC will consume (integrate, fold, trail-bake) calls
// MarkVoxelActive so the per-buffer occupancy set is built as a near-free side
// effect — MC then dispatches one thread-group per active 8^3-cell block instead of
// over the whole (dim-1)^3 grid, keeping cost surface-proportional.
//
// Block grid: 8^3 CELLS per block; block coord = cellIndex >> 3. _BDim = ceil((_Dim-1)/8).
// The buffer is one uint per block ("1 = active"); marking is an idempotent plain
// store of 1 (all threads writing the same value → race-free, no atomics needed).
//
// Halo (CRITICAL, plan 2a): an MC cell (cx,cy,cz) reads corners cx..cx+1, so a voxel
// v is a corner of cells (v-1) and (v) per axis — up to 8 cells spanning up to 8
// blocks. We mark ALL of them, so a surface crossing a block boundary is never
// dropped. Owning block + negative-direction neighbours = the halo.
//
// Marking is ONE-SIDED: mark every observed voxel at/below the surface (sdf <
// _MarkBand, with NO lower bound). Rationale: a cell emits a triangle only if its
// corners straddle iso=0, so it always has >=1 corner with sdf < 0; that corner,
// via the halo below, marks the cell's owning block — guaranteeing completeness
// regardless of how steep the SDF is. A SYMMETRIC band (|sdf| < k*voxelSize) is NOT
// safe: real fused TSDF (RetainGhost shells, noise, truncation kinks) can jump many
// voxelSizes across one cell, so a straddling cell can have ALL corners outside a
// symmetric band and get dropped (observed as ~0.02% missing triangles on real data).
// _MarkBand is a small positive rim (~1*voxelSize) past iso for margin; the negative
// side is unbounded down to -tau. Free space (sdf clamped to +tau) still never marks,
// so the bulk of the volume stays inactive. Interior cells (all corners sdf<0) get
// marked but emit nothing (edgeTable==0 early-out), so triangle work stays surface-
// proportional even though a solid interior inflates the active-block COUNT.
//
// ISO ASSUMPTION: completeness holds for MC iso == 0 (the default and normal case:
// the marking's inside-corner invariant is "sdf < iso"). A large POSITIVE meshIsoLevel
// could put an emitting cell's inside corner above _MarkBand and get its block dropped.
// The write kernels don't know the view's iso, so for a non-trivial nonzero iso use
// full-grid MC (TSDFView.useFullGridMC = true). _MarkBand's positive rim gives margin
// for small iso offsets.

RWStructuredBuffer<uint> _BlockActive;   // one uint per block; 1 = active
int3 _BDim;                              // block-grid dims = ceil((_Dim-1)/8)
float _MarkBand;                         // mark when sdf < this (metres); one-sided

void MarkVoxelActive(int3 v, int3 dim, float sdf, float weight)
{
    if (weight <= 0.0) return;
    if (sdf >= _MarkBand) return;   // one-sided: all sdf<0 (down to -tau) + thin +rim

    int cellsX = max(dim.x - 1, 1);
    int cellsY = max(dim.y - 1, 1);
    int cellsZ = max(dim.z - 1, 1);

    // Cell indices this voxel is a corner of, per axis (clamped in-range).
    int cx[2] = { clamp(v.x - 1, 0, cellsX - 1), clamp(v.x, 0, cellsX - 1) };
    int cy[2] = { clamp(v.y - 1, 0, cellsY - 1), clamp(v.y, 0, cellsY - 1) };
    int cz[2] = { clamp(v.z - 1, 0, cellsZ - 1), clamp(v.z, 0, cellsZ - 1) };

    // Mark the (up to 8) owning blocks. Duplicates collapse to the same store of 1.
    [unroll] for (int a = 0; a < 2; a++)
    [unroll] for (int b = 0; b < 2; b++)
    [unroll] for (int c = 0; c < 2; c++)
    {
        int bx = cx[a] >> 3;
        int by = cy[b] >> 3;
        int bz = cz[c] >> 3;
        int bi = bx + _BDim.x * (by + _BDim.y * bz);
        _BlockActive[bi] = 1u;
    }
}

#endif
