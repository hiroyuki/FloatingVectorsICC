// 3D-print export: turn the displayed TSDF mesh + the motion curves into a
// watertight binary STL. Operated from its own panel, Window > Print Export
// (Shared.EditorTools.PrintExportPanel — settings + the four operations):
//
//   [Fuse curves]  snapshot the front volume (first time), then bake the current
//                  curves as capsule tubes + body bridges into the DISPLAYED
//                  volume (min-union, same path as TSDFTrailBaker's fuse). Runs
//                  restore-then-fuse every time, so repeated clicks don't stack.
//   [Close holes]  morphological closing + outside flood fill (TSDFFloodFill):
//                  seals camera-unseen openings up to ~2*closeRadiusVoxels wide
//                  and fills the enclosed interior. Then a CPU connected-
//                  component pass reports islands and (keepLargestOnly) culls
//                  everything not attached to the main sculpture.
//   [Export STL]   full-grid Marching Cubes in Z slabs, CPU readback, binary STL
//                  scaled to targetHeightMm (mm units, slicer convention), with
//                  a signed-volume winding check. Optional colour PLY.
//   [Restore]      put the pre-print front buffer back (undo all of the above).
//
// All ops are one-shot and synchronous (GPU readbacks): print path, not per-frame.

using System;
using System.Collections.Generic;
using System.IO;
using BodyTracking;
using UnityEngine;

namespace TSDF
{
    [DisallowMultipleComponent]
    public class TSDFPrintExporter : MonoBehaviour
    {
        [Tooltip("Volume to export. Auto-resolves the first TSDFVolume at OnEnable.")]
        public TSDFVolume volume;

        [Tooltip("Curves source. Auto-resolves the first PointCloudMotionCurves at OnEnable.")]
        public PointCloudMotionCurves curves;

        [Header("Curve tubes")]
        [Min(1)]
        [Tooltip("Emit every Nth drawn curve. 20000 seeds / 40 = 500 printed tubes; " +
                 "thicker tubes need fewer curves or they fuse into a blob.")]
        public int printSeedStride = 40;

        [Range(0.002f, 0.05f)]
        [Tooltip("Curve tube radius (m). Print minimum: at 30 cm for a 1.8 m capture " +
                 "(1/6), 6 mm here prints as a 2 mm-diameter wire — the SLS/MJF floor. " +
                 "Keep >= ~1 voxel or Marching Cubes breaks the tube up.")]
        public float printRadius = 0.006f;

        [Range(1f, 3f)]
        [Tooltip("Bridge body-side radius multiplier — a fillet root where the tube " +
                 "meets the body, for strength.")]
        public float bridgeRadiusScale = 1.5f;

        [Range(0.05f, 1f)]
        [Tooltip("Skip bridges longer than this (m): a long strut means the seed was " +
                 "misclassified to a far bone.")]
        public float maxBridgeLength = 0.4f;

        [Min(16)]
        [Tooltip("Capsule segments per bake dispatch (GPU watchdog guard, same idea as " +
                 "TSDFTrailBaker.batchSize).")]
        public int bakeBatchSize = 256;

        [Header("Hole closing")]
        [Range(1, 16)]
        [Tooltip("Morphological closing radius in voxels: seals openings up to ~2x this " +
                 "wide (5 voxels @ 7 mm ≈ 7 cm openings). Bigger also rounds off concavities.")]
        public int closeRadiusVoxels = 5;

        [Tooltip("Cull every solid component except the largest after closing. The removed " +
                 "voxel count is the 'floating debris / unattached curves' detector.")]
        public bool keepLargestOnly = true;

        [Header("Export")]
        [Range(50f, 1000f)]
        [Tooltip("Printed height (mm) of the sculpture's bounding box — STL is written in mm.")]
        public float targetHeightMm = 300f;

        [Tooltip("Also write a binary PLY with per-vertex colours (full-colour print services).")]
        public bool exportPlyWithColor = false;

        [Range(0, 30)]
        [Tooltip("Taubin smoothing iterations on export (0 = off). Non-shrinking smoothing " +
                 "that removes the Marching-Cubes staircase from tubes and surfaces; ~10 is " +
                 "a good start. Runs on the welded mesh, purely at export — the displayed " +
                 "TSDF mesh is untouched.")]
        public int smoothIterations = 10;

        [Min(100_000)]
        [Tooltip("Triangle capacity of the per-slab MC readback buffer (72 B each).")]
        public int triangleBudgetPerSlab = 2_000_000;

        // ---- snapshot of the pre-print front buffer (typed arrays, RAM) ----
        private float[] _snapSdf;      // float2 per voxel
        private float[] _snapColor;    // float4 per voxel
        private uint[] _snapBlocks;    // front block-active set
        private int _snapTotal;        // voxel count the snapshot was taken at
        // PublishVersion the volume had after the snapshot / our last print op. If the
        // volume published since (the integrator is still live-updating — the moment
        // wasn't frozen), the snapshot is STALE: restoring it would resurrect an older
        // pose over the current one. Print ops retake it; Restore refuses.
        private int _snapVersion;

        private string _status = "";

        private ComputeShader _bakeShader;   // Resources/TSDFTrailBake
        private int _bakeKernel = -1;
        private ComputeShader _floodShader;  // Resources/TSDFFloodFill
        private int _kBuildMask = -1, _kDilate = -1, _kSeedOutside = -1, _kPropagate = -1,
                    _kMarkFilled = -1, _kErode = -1, _kFillWrite = -1, _kCullMask = -1;
        private ComputeShader _mcShader;     // Resources/TSDFMarchingCubes
        private int _mcKernel = -1;

        private static readonly Vector3 kFillColor = new Vector3(0.5f, 0.5f, 0.5f);

        // ---------------- panel surface (PrintExportPanel) ----------------
        /// <summary>One-line state readout for the panel: last op result, prefixed
        /// while the displayed volume carries print modifications.</summary>
        public string StatusText =>
            (HasSnapshot ? "print-modified (Restore available) — " : "") + _status;

        // ---------------- lifecycle ----------------
        private void OnEnable()
        {
            if (volume == null) volume = FindFirstObjectByType<TSDFVolume>();
            if (curves == null) curves = FindFirstObjectByType<PointCloudMotionCurves>();
        }

        private void OnDisable()
        {
            // Snapshot is CPU-side and cheap to keep, but it describes a Play-mode
            // volume that no longer exists after Stop — drop it.
            _snapSdf = null; _snapColor = null; _snapBlocks = null; _snapTotal = 0;
        }

        public bool VolumeReady =>
            volume != null && volume.Dim.x > 0 && volume.FrontBuffer != null;

        public bool HasSnapshot => _snapSdf != null;

        private bool SnapshotFresh => HasSnapshot && _snapVersion == volume.PublishVersion;

        private void DropSnapshot()
        {
            _snapSdf = null; _snapColor = null; _snapBlocks = null; _snapTotal = 0;
        }

        // ---------------- snapshot / restore ----------------
        private bool EnsureSnapshot()
        {
            if (SnapshotFresh) return true;
            if (HasSnapshot)
            {
                Debug.LogWarning("[TSDFPrintExporter] volume published since the last snapshot " +
                                 "(playback / integrator still running?) — retaking the snapshot from " +
                                 "the current content. Pause playback to freeze the moment first.", this);
                DropSnapshot();
            }
            int total = volume.Dim.x * volume.Dim.y * volume.Dim.z;
            var sdf = volume.FrontBuffer;
            var col = volume.FrontColorBuffer;
            if (sdf == null || col == null || sdf.count != total || col.count != total)
            {
                Fail($"snapshot aborted: buffer size mismatch (dim says {total}, sdf {sdf?.count}, color {col?.count})");
                return false;
            }
            _snapSdf = new float[total * 2];
            _snapColor = new float[total * 4];
            sdf.GetData(_snapSdf);
            col.GetData(_snapColor);
            if (volume.FrontBlockActive != null)
            {
                _snapBlocks = new uint[volume.FrontBlockActive.count];
                volume.FrontBlockActive.GetData(_snapBlocks);
            }
            _snapTotal = total;
            _snapVersion = volume.PublishVersion;
            Debug.Log($"[TSDFPrintExporter] snapshot taken ({total} voxels, " +
                      $"{(_snapSdf.Length + _snapColor.Length) * 4L / (1024 * 1024)} MB)", this);
            return true;
        }

        public void RestoreSnapshot()
        {
            if (!HasSnapshot) { Fail("nothing to restore"); return; }
            if (!SnapshotFresh)
            {
                Fail("restore aborted: the volume moved on since the snapshot (playback running?) — " +
                     "the snapshot no longer matches this moment; dropping it");
                DropSnapshot();
                return;
            }
            int total = volume.Dim.x * volume.Dim.y * volume.Dim.z;
            if (total != _snapTotal)
            {
                Fail($"restore aborted: volume was rebuilt since the snapshot ({_snapTotal} -> {total} voxels); dropping snapshot");
                DropSnapshot();
                return;
            }
            volume.FrontBuffer.SetData(_snapSdf);
            volume.FrontColorBuffer.SetData(_snapColor);
            if (_snapBlocks != null && volume.FrontBlockActive != null &&
                volume.FrontBlockActive.count == _snapBlocks.Length)
                volume.FrontBlockActive.SetData(_snapBlocks);
            volume.MarkFrontDirty();
            _snapVersion = volume.PublishVersion; // our own bump — snapshot still matches this moment
            _status = "restored pre-print volume";
            Debug.Log("[TSDFPrintExporter] " + _status, this);
        }

        // ---------------- 1: fuse curves ----------------
        public void FuseCurves()
        {
            if (!Guard(needCurves: true)) return;
            if (!EnsureBakeShader()) return;
            if (!EnsureSnapshot()) return;

            // Idempotent: always start from the snapshot so repeated clicks (or a
            // stride/radius retune) replace the previous fuse instead of stacking.
            RestoreSnapshot();

            int segCap = curves.MaxPrintSegs(printSeedStride);
            var segBuf = new ComputeBuffer(segCap, sizeof(float) * 12); // TrailSeg/Seg, 48B
            try
            {
                if (!curves.EmitPrintSegs(segBuf, printSeedStride, printRadius,
                                          bridgeRadiusScale, maxBridgeLength, out var st))
                    return;
                if (st.emitted == 0)
                {
                    Fail("no curve segments emitted (no curves on screen? seeds all culled?)");
                    return;
                }
                if (st.dropped > 0)
                    Debug.LogWarning($"[TSDFPrintExporter] seg buffer overflow: {st.dropped} segments dropped " +
                                     $"(cap {segCap}) — raise printSeedStride.", this);

                volume.CopyFrontToWrite();
                BakeSegs(segBuf, st.emitted);
                volume.Publish();
                _snapVersion = volume.PublishVersion; // our own bump

                _status = $"fused {st.emitted} curve segs (bridges skipped: {st.bridgesSkipped}, dropped: {st.dropped})";
                Debug.Log("[TSDFPrintExporter] " + _status, this);
            }
            finally { segBuf.Release(); }
        }

        // Same dispatch pattern as TSDFTrailBaker.BakeCore: full grid, batched over
        // segments so a single dispatch never trips the GPU watchdog.
        private void BakeSegs(ComputeBuffer segs, int segCount)
        {
            var dim = volume.Dim;
            int total = dim.x * dim.y * dim.z;
            DispatchGrid(total, out int gx, out int gy);
            _bakeShader.SetInts("_Dim", dim.x, dim.y, dim.z);
            _bakeShader.SetInt("_DispatchWidth", gx * 64);
            _bakeShader.SetFloat("_Tau", volume.Tau);
            _bakeShader.SetMatrix("_WorldFromVoxel", volume.WorldFromVoxel);
            _bakeShader.SetBuffer(_bakeKernel, "_Segs", segs);
            _bakeShader.SetBuffer(_bakeKernel, "_VoxelsOut", volume.WriteBuffer);
            _bakeShader.SetBuffer(_bakeKernel, "_ColorsOut", volume.WriteColorBuffer);
            volume.BindBlockMarking(_bakeShader, _bakeKernel, volume.WriteBlockActive);

            for (int off = 0; off < segCount; off += bakeBatchSize)
            {
                _bakeShader.SetInt("_SegOffset", off);
                _bakeShader.SetInt("_SegCount", Mathf.Min(bakeBatchSize, segCount - off));
                _bakeShader.Dispatch(_bakeKernel, Mathf.Max(1, gx), Mathf.Max(1, gy), 1);
            }
        }

        // ---------------- 2: close holes ----------------
        public void CloseHoles()
        {
            if (!Guard(needCurves: false)) return;
            if (!EnsureFloodShader()) return;
            if (!EnsureSnapshot()) return;

            var dim = volume.Dim;
            int total = dim.x * dim.y * dim.z;
            var maskA = new ComputeBuffer(total, sizeof(uint));
            var maskB = new ComputeBuffer(total, sizeof(uint));
            var changed = new ComputeBuffer(1, sizeof(uint));
            var fillStats = new ComputeBuffer(2, sizeof(uint));
            try
            {
                volume.CopyFrontToWrite();
                BindFloodCommon();

                // 1) solid mask off the displayed content (write == front copy).
                _floodShader.SetBuffer(_kBuildMask, "_VoxelsIn", volume.WriteBuffer);
                _floodShader.SetBuffer(_kBuildMask, "_MaskOut", maskA);
                Dispatch3D(_kBuildMask, dim);

                // 2) dilate k (ping-pong; cur ends on the buffer holding the result).
                var cur = maskA; var other = maskB;
                for (int i = 0; i < closeRadiusVoxels; i++)
                {
                    _floodShader.SetBuffer(_kDilate, "_MaskRead", cur);
                    _floodShader.SetBuffer(_kDilate, "_MaskOut", other);
                    Dispatch3D(_kDilate, dim);
                    (cur, other) = (other, cur);
                }

                // 3) outside flood fill (in place on cur).
                _floodShader.SetBuffer(_kSeedOutside, "_Mask", cur);
                Dispatch3D(_kSeedOutside, dim);
                int maxIters = dim.x + dim.y + dim.z + 16;
                var changedHost = new uint[1];
                int iters = 0;
                _floodShader.SetBuffer(_kPropagate, "_Mask", cur);
                _floodShader.SetBuffer(_kPropagate, "_Changed", changed);
                while (iters < maxIters)
                {
                    changed.SetData(new uint[] { 0u });
                    for (int c = 0; c < 8 && iters < maxIters; c++, iters++)
                        Dispatch3D(_kPropagate, dim);
                    changed.GetData(changedHost);
                    if (changedHost[0] == 0u) break;
                }
                if (iters >= maxIters)
                    Debug.LogWarning($"[TSDFPrintExporter] outside flood fill hit the iteration cap ({maxIters}) — " +
                                     "closing may be incomplete.", this);

                // 4) outside->0, rest->1, then erode k back.
                _floodShader.SetBuffer(_kMarkFilled, "_Mask", cur);
                Dispatch3D(_kMarkFilled, dim);
                for (int i = 0; i < closeRadiusVoxels; i++)
                {
                    _floodShader.SetBuffer(_kErode, "_MaskRead", cur);
                    _floodShader.SetBuffer(_kErode, "_MaskOut", other);
                    Dispatch3D(_kErode, dim);
                    (cur, other) = (other, cur);
                }

                // 5) fill interior + synthesise the +tau rim, then publish.
                fillStats.SetData(new uint[] { 0u, 0u });
                _floodShader.SetBuffer(_kFillWrite, "_MaskRead", cur);
                _floodShader.SetBuffer(_kFillWrite, "_VoxelsOut", volume.WriteBuffer);
                _floodShader.SetBuffer(_kFillWrite, "_ColorsOut", volume.WriteColorBuffer);
                _floodShader.SetBuffer(_kFillWrite, "_FillStats", fillStats);
                volume.BindBlockMarking(_floodShader, _kFillWrite, volume.WriteBlockActive);
                Dispatch3D(_kFillWrite, dim);
                volume.Publish();

                var stats = new uint[2];
                fillStats.GetData(stats);

                // Connectivity: label solid voxels on the CPU, report islands, cull
                // all but the largest (this is the unattached-curve detector).
                string ccReport = AnalyzeComponents(maskA, maskB);
                _snapVersion = volume.PublishVersion; // our own bumps

                _status = $"closed holes: filled {stats[0]} voxels (+{stats[1]} rim), " +
                          $"flood {iters} iters; {ccReport}";
                Debug.Log("[TSDFPrintExporter] " + _status, this);
            }
            finally
            {
                maskA.Release(); maskB.Release(); changed.Release(); fillStats.Release();
            }
        }

        private void BindFloodCommon()
        {
            var dim = volume.Dim;
            _floodShader.SetInts("_Dim", dim.x, dim.y, dim.z);
            _floodShader.SetFloat("_Tau", volume.Tau);
            _floodShader.SetVector("_FillColor", new Vector4(kFillColor.x, kFillColor.y, kFillColor.z, 0f));
        }

        // CPU 6-connectivity labelling over the post-fill solid mask. Returns a
        // human-readable summary; culls non-largest components when keepLargestOnly.
        private string AnalyzeComponents(ComputeBuffer scratchA, ComputeBuffer scratchB)
        {
            var dim = volume.Dim;
            int total = dim.x * dim.y * dim.z;

            // Rebuild the solid mask from the just-published front and read it back.
            _floodShader.SetBuffer(_kBuildMask, "_VoxelsIn", volume.FrontBuffer);
            _floodShader.SetBuffer(_kBuildMask, "_MaskOut", scratchA);
            Dispatch3D(_kBuildMask, dim);
            var mask = new uint[total];
            scratchA.GetData(mask);

            int dx = dim.x, dxy = dim.x * dim.y;
            var visited = new bool[total];
            var stack = new int[total];
            int compCount = 0, largestSeed = -1, largestSize = 0; long solidTotal = 0;

            for (int i = 0; i < total; i++)
            {
                if (mask[i] == 0u || visited[i]) continue;
                compCount++;
                int size = 0, sp = 0;
                stack[sp++] = i; visited[i] = true;
                while (sp > 0)
                {
                    int v = stack[--sp];
                    size++;
                    int x = v % dx, y = (v / dx) % dim.y, z = v / dxy;
                    if (x + 1 < dim.x && mask[v + 1] != 0u && !visited[v + 1]) { visited[v + 1] = true; stack[sp++] = v + 1; }
                    if (x > 0 && mask[v - 1] != 0u && !visited[v - 1]) { visited[v - 1] = true; stack[sp++] = v - 1; }
                    if (y + 1 < dim.y && mask[v + dx] != 0u && !visited[v + dx]) { visited[v + dx] = true; stack[sp++] = v + dx; }
                    if (y > 0 && mask[v - dx] != 0u && !visited[v - dx]) { visited[v - dx] = true; stack[sp++] = v - dx; }
                    if (z + 1 < dim.z && mask[v + dxy] != 0u && !visited[v + dxy]) { visited[v + dxy] = true; stack[sp++] = v + dxy; }
                    if (z > 0 && mask[v - dxy] != 0u && !visited[v - dxy]) { visited[v - dxy] = true; stack[sp++] = v - dxy; }
                }
                solidTotal += size;
                if (size > largestSize) { largestSize = size; largestSeed = i; }
            }

            if (compCount <= 1)
                return $"components: {compCount} (all connected, {solidTotal} solid voxels)";

            long removed = solidTotal - largestSize;
            if (!keepLargestOnly)
            {
                Debug.LogWarning($"[TSDFPrintExporter] {compCount - 1} disconnected island(s), {removed} voxels " +
                                 "— these would fall off the print (keepLargestOnly is OFF).", this);
                return $"components: {compCount}, largest {largestSize}, UNCULLED islands {removed} voxels";
            }

            // Second pass: mark the largest component, then flag everything else solid
            // for removal and cull it on the GPU.
            Array.Clear(visited, 0, visited.Length);
            {
                int sp = 0;
                stack[sp++] = largestSeed; visited[largestSeed] = true;
                while (sp > 0)
                {
                    int v = stack[--sp];
                    int x = v % dx, y = (v / dx) % dim.y, z = v / dxy;
                    if (x + 1 < dim.x && mask[v + 1] != 0u && !visited[v + 1]) { visited[v + 1] = true; stack[sp++] = v + 1; }
                    if (x > 0 && mask[v - 1] != 0u && !visited[v - 1]) { visited[v - 1] = true; stack[sp++] = v - 1; }
                    if (y + 1 < dim.y && mask[v + dx] != 0u && !visited[v + dx]) { visited[v + dx] = true; stack[sp++] = v + dx; }
                    if (y > 0 && mask[v - dx] != 0u && !visited[v - dx]) { visited[v - dx] = true; stack[sp++] = v - dx; }
                    if (z + 1 < dim.z && mask[v + dxy] != 0u && !visited[v + dxy]) { visited[v + dxy] = true; stack[sp++] = v + dxy; }
                    if (z > 0 && mask[v - dxy] != 0u && !visited[v - dxy]) { visited[v - dxy] = true; stack[sp++] = v - dxy; }
                }
            }
            for (int i = 0; i < total; i++)
                mask[i] = (mask[i] != 0u && !visited[i]) ? 1u : 0u;

            scratchB.SetData(mask);
            volume.CopyFrontToWrite();
            _floodShader.SetBuffer(_kCullMask, "_MaskRead", scratchB);
            _floodShader.SetBuffer(_kCullMask, "_VoxelsOut", volume.WriteBuffer);
            _floodShader.SetBuffer(_kCullMask, "_ColorsOut", volume.WriteColorBuffer);
            BindFloodCommon();
            Dispatch3D(_kCullMask, dim);
            volume.Publish();

            Debug.LogWarning($"[TSDFPrintExporter] culled {compCount - 1} disconnected island(s) " +
                             $"({removed} voxels) — unattached curves / debris. If curves vanished, " +
                             "check bridge stats on Fuse.", this);
            return $"components: {compCount} -> 1 (culled {removed} voxels of islands)";
        }

        // ---------------- 3: STL export ----------------
        public void ExportStl()
        {
            if (!Guard(needCurves: false)) return;
            if (!EnsureMcShader()) return;

            var dim = volume.Dim;
            // Match what the user sees: reuse the view's iso/weight gates if present.
            var view = FindFirstObjectByType<TSDFView>();
            float iso = view != null ? view.meshIsoLevel : 0f;
            float minWeight = view != null ? view.meshMinWeight : 0.5f;

            int triCap = Mathf.Max(100_000, triangleBudgetPerSlab);
            var triBuf = new ComputeBuffer(triCap, 72, ComputeBufferType.Append); // Tri
            var countBuf = new ComputeBuffer(1, sizeof(uint), ComputeBufferType.Raw);
            var slabs = new List<(float[] data, int tris)>();
            long totalTris = 0;
            try
            {
                _mcShader.SetInts("_Dim", dim.x, dim.y, dim.z);
                _mcShader.SetMatrix("_WorldFromVoxel", volume.WorldFromVoxel);
                _mcShader.SetFloat("_IsoLevel", iso);
                _mcShader.SetFloat("_MinWeight", minWeight);
                _mcShader.SetBuffer(_mcKernel, "_Voxels", volume.FrontBuffer);
                _mcShader.SetBuffer(_mcKernel, "_Colors", volume.FrontColorBuffer);
                _mcShader.SetBuffer(_mcKernel, "_Triangles", triBuf);

                int gx = Mathf.Max(1, Mathf.CeilToInt((dim.x - 1) / 4f));
                int gy = Mathf.Max(1, Mathf.CeilToInt((dim.y - 1) / 4f));
                int cellsZ = Mathf.Max(1, dim.z - 1);
                var countHost = new uint[1];

                int zBase = 0, thickness = 32; // multiple of 4: dispatch covers exactly gz*4 cells
                while (zBase < cellsZ)
                {
                    int gz = Mathf.Max(1, Mathf.CeilToInt(Mathf.Min(thickness, cellsZ - zBase) / 4f));
                    triBuf.SetCounterValue(0);
                    _mcShader.SetInt("_CellZBase", zBase);
                    _mcShader.Dispatch(_mcKernel, gx, gy, gz);
                    ComputeBuffer.CopyCount(triBuf, countBuf, 0);
                    countBuf.GetData(countHost);
                    int n = (int)countHost[0];

                    if (n >= triCap)
                    {
                        if (thickness <= 4)
                        {
                            Fail($"slab at z={zBase} overflows {triCap} triangles even at minimum thickness — " +
                                 "raise triangleBudgetPerSlab");
                            return;
                        }
                        thickness = Mathf.Max(4, thickness / 2);
                        continue; // retry the same slab thinner
                    }

                    if (n > 0)
                    {
                        var data = new float[n * 18]; // Tri = 18 floats
                        triBuf.GetData(data, 0, 0, n * 18);
                        slabs.Add((data, n));
                        totalTris += n;
                    }
                    zBase += gz * 4;
                    thickness = 32;
                }
                _mcShader.SetInt("_CellZBase", 0); // hygiene: shared shader asset state

                if (totalTris == 0) { Fail("Marching Cubes produced 0 triangles — empty volume?"); return; }
                WriteMeshFiles(slabs, totalTris);
            }
            finally
            {
                triBuf.Release(); countBuf.Release();
            }
        }

        private void WriteMeshFiles(List<(float[] data, int tris)> slabs, long totalTris)
        {
            // Weld the MC soup into an indexed mesh: adjacency for Taubin smoothing,
            // and a much smaller PLY. 0.1 mm quantisation merges the shared-edge
            // vertices adjacent cells emit (not guaranteed bitwise identical) while
            // staying far below any feature at 4-7 mm voxels.
            WeldSlabs(slabs, out var pos, out var col, out var tri);
            slabs.Clear(); // free the soup copies before smoothing allocates

            if (smoothIterations > 0)
                TaubinSmooth(pos, tri, smoothIterations);

            Vector3 min = Vector3.positiveInfinity, max = Vector3.negativeInfinity;
            foreach (var p in pos) { min = Vector3.Min(min, p); max = Vector3.Max(max, p); }
            Vector3 center = (min + max) * 0.5f;
            Vector3 size = max - min;
            if (size.y < 1e-4f) { Fail("degenerate mesh bounds"); return; }

            // Count non-degenerate triangles (0.1 mm weld can collapse slivers) and
            // measure the signed volume: MC emits inside = sdf < iso, so a negative
            // volume means the winding faces inward — flip on write.
            int triCount = tri.Length / 3;
            int written = 0;
            double signedVol = 0;
            for (int t = 0; t < triCount; t++)
            {
                int i0 = tri[t * 3], i1 = tri[t * 3 + 1], i2 = tri[t * 3 + 2];
                if (i0 == i1 || i1 == i2 || i0 == i2) continue;
                written++;
                Vector3 a = pos[i0] - center, b = pos[i1] - center, c = pos[i2] - center;
                signedVol += Vector3.Dot(a, Vector3.Cross(b, c)) / 6.0;
            }
            bool flip = signedVol < 0;

            float scale = targetHeightMm / size.y; // metres -> printed mm
            string dir = Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.UserProfile),
                                      "Documents", "FloatingVectorsPrints");
            string stamp = DateTime.Now.ToString("yyyyMMdd_HHmmss");
            string stlPath = Path.Combine(dir, $"print_{stamp}.stl");
            try
            {
                Directory.CreateDirectory(dir);
                using (var bw = new BinaryWriter(File.Create(stlPath)))
                {
                    var header = new byte[80];
                    var tag = System.Text.Encoding.ASCII.GetBytes("FloatingVectorsICC print export");
                    Array.Copy(tag, header, Math.Min(tag.Length, 80));
                    bw.Write(header);
                    bw.Write((uint)written);
                    for (int t = 0; t < triCount; t++)
                    {
                        int i0 = tri[t * 3], i1 = tri[t * 3 + 1], i2 = tri[t * 3 + 2];
                        if (i0 == i1 || i1 == i2 || i0 == i2) continue;
                        if (flip) (i1, i2) = (i2, i1);
                        Vector3 a = (pos[i0] - center) * scale;
                        Vector3 b = (pos[i1] - center) * scale;
                        Vector3 c = (pos[i2] - center) * scale;
                        Vector3 nrm = Vector3.Cross(b - a, c - a);
                        float len = nrm.magnitude;
                        nrm = len > 1e-12f ? nrm / len : Vector3.up;
                        WriteV(bw, nrm); WriteV(bw, a); WriteV(bw, b); WriteV(bw, c);
                        bw.Write((ushort)0);
                    }
                }

                string plyNote = "";
                if (exportPlyWithColor)
                {
                    string plyPath = Path.Combine(dir, $"print_{stamp}.ply");
                    WritePly(plyPath, pos, col, tri, written, center, scale, flip);
                    plyNote = " + PLY";
                }

                long bytes = new FileInfo(stlPath).Length;
                _status = $"STL: {written} tris / {pos.Length} welded verts, " +
                          $"{bytes / (1024 * 1024)} MB{plyNote} -> {stlPath} " +
                          $"(height {targetHeightMm:0} mm, smooth x{smoothIterations}, " +
                          $"winding {(flip ? "flipped" : "ok")}, " +
                          $"signedVol {(signedVol >= 0 ? "+" : "")}{signedVol:0.0000} m^3)";
                Debug.Log("[TSDFPrintExporter] " + _status, this);
            }
            catch (Exception e)
            {
                Fail($"write failed: {e.Message}");
            }
        }

        private static void WriteV(BinaryWriter bw, Vector3 v)
        {
            bw.Write(v.x); bw.Write(v.y); bw.Write(v.z);
        }

        // Weld the per-slab triangle soup (Tri = p0 c0 p1 c1 p2 c2, 3 floats each)
        // into unique vertices + index list. Vertex colours are averaged over all
        // soup occurrences. Keys are positions quantised to 0.1 mm packed into a
        // long (21 bits/axis = ±104 m range).
        private static void WeldSlabs(List<(float[] data, int tris)> slabs,
                                      out Vector3[] positions, out Vector3[] colors, out int[] triangles)
        {
            var map = new Dictionary<long, int>();
            var pos = new List<Vector3>();
            var colSum = new List<Vector3>();
            var colCnt = new List<int>();
            var tri = new List<int>();

            foreach (var (data, tris) in slabs)
                for (int t = 0; t < tris; t++)
                {
                    int o = t * 18;
                    for (int v = 0; v < 3; v++)
                    {
                        int p = o + v * 6;
                        var w = new Vector3(data[p], data[p + 1], data[p + 2]);
                        long kx = (long)Math.Round(w.x * 10000.0) + 1048576;
                        long ky = (long)Math.Round(w.y * 10000.0) + 1048576;
                        long kz = (long)Math.Round(w.z * 10000.0) + 1048576;
                        long key = (kx << 42) | (ky << 21) | kz;
                        if (!map.TryGetValue(key, out int idx))
                        {
                            idx = pos.Count;
                            map.Add(key, idx);
                            pos.Add(w);
                            colSum.Add(Vector3.zero);
                            colCnt.Add(0);
                        }
                        colSum[idx] += new Vector3(data[p + 3], data[p + 4], data[p + 5]);
                        colCnt[idx]++;
                        tri.Add(idx);
                    }
                }

            positions = pos.ToArray();
            colors = new Vector3[positions.Length];
            for (int i = 0; i < colors.Length; i++)
                colors[i] = colCnt[i] > 0 ? colSum[i] / colCnt[i] : Vector3.one * 0.5f;
            triangles = tri.ToArray();
        }

        // Taubin λ|μ smoothing (non-shrinking): alternating positive/negative
        // uniform-Laplacian steps over the unique-edge adjacency. In-place on pos.
        private static void TaubinSmooth(Vector3[] pos, int[] tri, int iterations)
        {
            int n = pos.Length;

            // Unique undirected edges -> CSR adjacency.
            var edges = new HashSet<long>();
            for (int t = 0; t < tri.Length; t += 3)
            {
                AddEdge(edges, tri[t], tri[t + 1]);
                AddEdge(edges, tri[t + 1], tri[t + 2]);
                AddEdge(edges, tri[t + 2], tri[t]);
            }
            var degree = new int[n];
            foreach (long e in edges) { degree[(int)(e >> 32)]++; degree[(int)(e & 0xffffffffL)]++; }
            var offset = new int[n + 1];
            for (int i = 0; i < n; i++) offset[i + 1] = offset[i] + degree[i];
            var adj = new int[offset[n]];
            var cursor = (int[])offset.Clone();
            foreach (long e in edges)
            {
                int a = (int)(e >> 32), b = (int)(e & 0xffffffffL);
                adj[cursor[a]++] = b;
                adj[cursor[b]++] = a;
            }

            const float lambda = 0.5f, mu = -0.53f;
            var tmp = new Vector3[n];
            var src = pos;
            for (int it = 0; it < iterations; it++)
            {
                LaplacianPass(src, tmp, offset, adj, lambda);
                LaplacianPass(tmp, src, offset, adj, mu);
            }
        }

        private static void AddEdge(HashSet<long> edges, int a, int b)
        {
            if (a == b) return;
            long lo = Math.Min(a, b), hi = Math.Max(a, b);
            edges.Add((lo << 32) | hi);
        }

        private static void LaplacianPass(Vector3[] src, Vector3[] dst, int[] offset, int[] adj, float factor)
        {
            for (int v = 0; v < src.Length; v++)
            {
                int a = offset[v], b = offset[v + 1];
                if (b == a) { dst[v] = src[v]; continue; }
                Vector3 avg = Vector3.zero;
                for (int i = a; i < b; i++) avg += src[adj[i]];
                avg /= (b - a);
                dst[v] = src[v] + factor * (avg - src[v]);
            }
        }

        // Binary little-endian PLY: welded vertices with averaged colours + indexed faces.
        private static void WritePly(string path, Vector3[] pos, Vector3[] col, int[] tri,
                                     int faceCount, Vector3 center, float scale, bool flip)
        {
            using var fs = File.Create(path);
            string header = "ply\nformat binary_little_endian 1.0\n" +
                            $"element vertex {pos.Length}\n" +
                            "property float x\nproperty float y\nproperty float z\n" +
                            "property uchar red\nproperty uchar green\nproperty uchar blue\n" +
                            $"element face {faceCount}\n" +
                            "property list uchar int vertex_indices\nend_header\n";
            var hb = System.Text.Encoding.ASCII.GetBytes(header);
            fs.Write(hb, 0, hb.Length);
            using var bw = new BinaryWriter(fs);
            for (int i = 0; i < pos.Length; i++)
            {
                Vector3 w = (pos[i] - center) * scale;
                bw.Write(w.x); bw.Write(w.y); bw.Write(w.z);
                bw.Write((byte)Mathf.Clamp(Mathf.RoundToInt(col[i].x * 255f), 0, 255));
                bw.Write((byte)Mathf.Clamp(Mathf.RoundToInt(col[i].y * 255f), 0, 255));
                bw.Write((byte)Mathf.Clamp(Mathf.RoundToInt(col[i].z * 255f), 0, 255));
            }
            for (int t = 0; t < tri.Length; t += 3)
            {
                int i0 = tri[t], i1 = tri[t + 1], i2 = tri[t + 2];
                if (i0 == i1 || i1 == i2 || i0 == i2) continue;
                if (flip) (i1, i2) = (i2, i1);
                bw.Write((byte)3);
                bw.Write(i0); bw.Write(i1); bw.Write(i2);
            }
        }

        // ---------------- shared ----------------
        private bool Guard(bool needCurves)
        {
            if (!Application.isPlaying) { Fail("Play mode only"); return false; }
            if (!VolumeReady) { Fail("no initialised TSDFVolume"); return false; }
            if (needCurves && curves == null) { Fail("no PointCloudMotionCurves in the scene"); return false; }
            return true;
        }

        private void Fail(string msg)
        {
            _status = "ERROR: " + msg;
            Debug.LogError("[TSDFPrintExporter] " + msg, this);
        }

        private bool EnsureBakeShader()
        {
            if (_bakeShader != null && _bakeKernel >= 0) return true;
            _bakeShader = Resources.Load<ComputeShader>("TSDFTrailBake");
            if (_bakeShader == null) { Fail("Resources/TSDFTrailBake.compute not found"); return false; }
            _bakeKernel = _bakeShader.FindKernel("BakeTrail");
            return _bakeKernel >= 0;
        }

        private bool EnsureFloodShader()
        {
            if (_floodShader != null && _kFillWrite >= 0) return true;
            _floodShader = Resources.Load<ComputeShader>("TSDFFloodFill");
            if (_floodShader == null) { Fail("Resources/TSDFFloodFill.compute not found"); return false; }
            _kBuildMask = _floodShader.FindKernel("BuildMask");
            _kDilate = _floodShader.FindKernel("Dilate");
            _kSeedOutside = _floodShader.FindKernel("SeedOutside");
            _kPropagate = _floodShader.FindKernel("Propagate");
            _kMarkFilled = _floodShader.FindKernel("MarkFilled");
            _kErode = _floodShader.FindKernel("Erode");
            _kFillWrite = _floodShader.FindKernel("FillWrite");
            _kCullMask = _floodShader.FindKernel("CullMask");
            return true;
        }

        private bool EnsureMcShader()
        {
            if (_mcShader != null && _mcKernel >= 0) return true;
            _mcShader = Resources.Load<ComputeShader>("TSDFMarchingCubes");
            if (_mcShader == null) { Fail("Resources/TSDFMarchingCubes.compute not found"); return false; }
            _mcKernel = _mcShader.FindKernel("MarchCubes");
            return _mcKernel >= 0;
        }

        private void Dispatch3D(int kernel, Vector3Int dim)
        {
            _floodShader.Dispatch(kernel,
                Mathf.Max(1, Mathf.CeilToInt(dim.x / 4f)),
                Mathf.Max(1, Mathf.CeilToInt(dim.y / 4f)),
                Mathf.Max(1, Mathf.CeilToInt(dim.z / 4f)));
        }

        private static void DispatchGrid(int total, out int gx, out int gy)
        {
            int groups = Mathf.CeilToInt(total / 64f);
            gx = Mathf.Max(1, Mathf.Min(groups, 65535));
            gy = Mathf.Max(1, Mathf.CeilToInt(groups / (float)gx));
        }
    }
}
