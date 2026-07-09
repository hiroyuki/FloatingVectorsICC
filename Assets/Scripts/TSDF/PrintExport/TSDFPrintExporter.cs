// Mesh export: turn the displayed TSDF mesh + the motion curves into web/AR
// files. Operated from its own panel, Window > Print Export
// (Shared.EditorTools.PrintExportPanel — settings + the operations):
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
//   [Export Web]   the MC mesh at real-world scale (metres, Y-up right-handed,
//                  base on y=0) PLUS the curved lines as display-resolution tube
//                  meshes (read back from the draw buffer — no voxel fuse, so no
//                  Fuse curves needed and no resolution loss): binary glTF
//                  (.glb, vertex colours) for web viewers, plus .usdz for iPhone
//                  AR Quick Look — Quick Look ignores vertex colours, so the
//                  .usdz bakes them into a per-triangle texture atlas
//                  (GlbWriter / UsdzWriter).
//   [Restore]      put the pre-print front buffer back (undo all of the above).
//
// (The binary STL / colour PLY physical-print path was removed 2026-07-09 —
//  restore from git history if a print run ever comes back.)
// All ops are one-shot and synchronous (GPU readbacks): export path, not per-frame.

using System;
using System.Collections.Generic;
using System.IO;
using System.Threading.Tasks;
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
        [Range(0, 30)]
        [Tooltip("Taubin smoothing iterations on export (0 = off). Non-shrinking smoothing " +
                 "that removes the Marching-Cubes staircase from tubes and surfaces; ~10 is " +
                 "a good start. Runs on the welded mesh, purely at export — the displayed " +
                 "TSDF mesh is untouched.")]
        public int smoothIterations = 10;

        [Min(100_000)]
        [Tooltip("Triangle capacity of the per-slab MC readback buffer (72 B each).")]
        public int triangleBudgetPerSlab = 2_000_000;

        [Header("Web export (GLB + USDZ)")]
        [Tooltip("Include the curved lines in the web files as real tube meshes tessellated " +
                 "from the drawn polylines (full display resolution). Unlike the print path " +
                 "there is NO voxel fuse — Fuse curves is not needed and would double them up.")]
        public bool webIncludeCurves = true;

        [Min(1)]
        [Tooltip("Emit every Nth drawn curve into the web files. Each curve becomes a tube of " +
                 "~1000 triangles, so 20000 seeds / 40 = 500 tubes ≈ 0.5M triangles. Lower = " +
                 "denser but much bigger files (the .usdz is ASCII).")]
        public int webCurveStride = 40;

        [Range(3, 12)]
        [Tooltip("Tube cross-section sides. 4-6 reads as round at ribbon width.")]
        public int webCurveSides = 4;

        [Range(0.05f, 1f)]
        [Tooltip("Tube radius scale at the OLD end of each trail (the past tip). 1 = constant " +
                 "thickness; 0.25 = the tail tapers to a quarter of the ribbon radius, so the " +
                 "stroke reads as fading in from the past.")]
        public float webCurveTipTaper = 0.25f;

        [Min(0)]
        [Tooltip("Decimate the TSDF mesh to this many triangles for the web files (quadric edge " +
                 "collapse, colours kept — the STL keeps full resolution). 0 = no decimation. " +
                 "150k mesh + default curves lands both files under ~10 MB.")]
        public int webMeshTargetTris = 150_000;

        [Range(0f, 0.01f)]
        [Tooltip("Curve polyline simplification tolerance (m, Douglas-Peucker) before tubing. " +
                 "The Catmull-Rom polylines are oversampled where motion is slow; 1.5 mm halves " +
                 "the tube triangles with no visible change at 6 mm ribbons. 0 = off.")]
        public float webCurveTolerance = 0.0015f;

        [Tooltip("Python with the pxr module (pip: usd-core), used to convert the .usdz payload " +
                 "to binary usdc (3-4x smaller). Empty = auto-detect: ~/.venvs/usd, then system " +
                 "python3/python. One-time setup — macOS: python3 -m venv ~/.venvs/usd && " +
                 "~/.venvs/usd/bin/pip install usd-core / Windows: py -3 -m venv " +
                 "%USERPROFILE%\\.venvs\\usd + ...\\Scripts\\pip install usd-core. " +
                 "Not found -> the usdz ships ASCII usda (bigger, still valid).")]
        public string usdPythonPath = "";

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
                    _kMarkFilled = -1, _kErode = -1, _kFillWrite = -1, _kCapBoundary = -1,
                    _kCullMask = -1;
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
            var segBuf = new ComputeBuffer(segCap, TrailBakeOps.SegStride); // TrailSeg/Seg, 48B
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
                // Shared full-grid batched bake (3-7 — was a copy of TrailBaker.BakeCore's loop).
                TrailBakeOps.BakeSegments(_bakeShader, _bakeKernel, volume,
                                          segBuf, st.emitted, bakeBatchSize);
                volume.Publish();
                _snapVersion = volume.PublishVersion; // our own bump

                _status = $"fused {st.emitted} curve segs (bridges skipped: {st.bridgesSkipped}, dropped: {st.dropped})";
                Debug.Log("[TSDFPrintExporter] " + _status, this);
            }
            finally { segBuf.Release(); }
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

                // Watertight bbox cut: cap anything crossing a boundary face (the
                // floor crop cuts legs/trail/curves) one voxel inside.
                _floodShader.SetBuffer(_kCapBoundary, "_VoxelsOut", volume.WriteBuffer);
                _floodShader.SetBuffer(_kCapBoundary, "_ColorsOut", volume.WriteColorBuffer);
                Dispatch3D(_kCapBoundary, dim);
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

        // ---------------- 3: exports ----------------
        // (The binary STL / colour PLY print path was removed 2026-07-09 by owner
        //  decision — glTF (.glb) + .usdz are the only export targets. Restore
        //  ExportStl/WriteMeshFiles + StlWriter/PlyWriter from git history if a
        //  physical print run ever comes back.)

        /// <summary>Web/AR export: .glb (vertex colours, web viewers) and .usdz
        /// (texture-atlas colours, iPhone AR Quick Look), both real-world metres.</summary>
        public void ExportWeb()
        {
            if (!Guard(needCurves: false)) return;
            if (!EnsureMcShader()) return;
            var sw = System.Diagnostics.Stopwatch.StartNew();
            var slabs = RunMarchingCubes(out long totalTris);
            if (slabs == null) return;
            WriteWebFiles(slabs, sw.ElapsedMilliseconds);
        }

        // Full-grid Marching Cubes in Z slabs with CPU readback: the shared front
        // half of every mesh export. Returns the per-slab triangle soup
        // (Tri = 18 floats each), or null after Fail().
        private List<(float[] data, int tris)> RunMarchingCubes(out long totalTris)
        {
            totalTris = 0;
            var dim = volume.Dim;
            // Match what the user sees: reuse the view's iso/weight gates if present.
            var view = FindFirstObjectByType<TSDFView>();
            float iso = view != null ? view.meshIsoLevel : 0f;
            float minWeight = view != null ? view.meshMinWeight : 0.5f;

            int triCap = Mathf.Max(100_000, triangleBudgetPerSlab);
            var triBuf = new ComputeBuffer(triCap, 72, ComputeBufferType.Append); // Tri
            var countBuf = new ComputeBuffer(1, sizeof(uint), ComputeBufferType.Raw);
            var slabs = new List<(float[] data, int tris)>();
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
                            return null;
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
                    // Progress heartbeat: a long export must stay diagnosable from
                    // Editor.log (the editor blocks while this runs).
                    if (slabs.Count % 5 == 0)
                        Debug.Log($"[TSDFPrintExporter] MC slab {zBase}/{cellsZ} — {totalTris} tris so far", this);
                }
                _mcShader.SetInt("_CellZBase", 0); // hygiene: shared shader asset state

                if (totalTris == 0) { Fail("Marching Cubes produced 0 triangles — empty volume?"); return null; }
                Debug.Log($"[TSDFPrintExporter] MC done: {totalTris} tris — welding...", this);
                return slabs;
            }
            finally
            {
                triBuf.Release(); countBuf.Release();
            }
        }

        // ---------------- web/AR export (.glb + .usdz) ----------------
        // Same weld+smooth as the STL path, then written at real-world scale:
        // metres, XZ centred on the origin, base on y=0 (AR Quick Look places the
        // model on the floor). Unity is left-handed, glTF/USD are right-handed
        // Y-up, so X is mirrored and the winding re-checked by signed volume
        // (CCW = outward in RH; negative volume means flip on write).
        private void WriteWebFiles(List<(float[] data, int tris)> slabs, long mcMs)
        {
            var total = System.Diagnostics.Stopwatch.StartNew();
            var phase = System.Diagnostics.Stopwatch.StartNew();
            MeshOps.WeldSlabs(slabs, out var pos, out var col, out var tri);
            slabs.Clear(); // free the soup copies before smoothing allocates
            long weldMs = phase.ElapsedMilliseconds;
            Debug.Log($"[TSDFPrintExporter] welded to {pos.Length} verts in {weldMs} ms " +
                      $"— smoothing x{smoothIterations}...", this);
            phase.Restart();
            if (smoothIterations > 0)
                MeshOps.TaubinSmooth(pos, tri, smoothIterations);
            long smoothMs = phase.ElapsedMilliseconds;

            // Decimate to the web triangle budget (web files only; the STL path
            // keeps the full MC resolution).
            long decimateMs = 0;
            int trisBeforeDec = tri.Length / 3;
            if (webMeshTargetTris > 0 && trisBeforeDec > webMeshTargetTris)
            {
                phase.Restart();
                MeshDecimator.Simplify(ref pos, ref col, ref tri, webMeshTargetTris);
                decimateMs = phase.ElapsedMilliseconds;
                Debug.Log($"[TSDFPrintExporter] decimated {trisBeforeDec} -> {tri.Length / 3} tris " +
                          $"({pos.Length} verts) in {decimateMs} ms", this);
            }
            int meshTris = tri.Length / 3;
            phase.Restart();

            Vector3 min = Vector3.positiveInfinity, max = Vector3.negativeInfinity;
            foreach (var p in pos) { min = Vector3.Min(min, p); max = Vector3.Max(max, p); }
            Vector3 center = (min + max) * 0.5f;
            if ((max - min).y < 1e-4f) { Fail("degenerate mesh bounds"); return; }

            for (int i = 0; i < pos.Length; i++)
                pos[i] = new Vector3(-(pos[i].x - center.x), pos[i].y - min.y, pos[i].z - center.z);

            // Drop weld-collapsed slivers, then orient: positive signed volume in
            // the mirrored (right-handed) space means the winding is already
            // CCW-outward; negative means every triangle flips.
            var idxList = new List<int>(tri.Length);
            double signedVol = 0;
            for (int t = 0; t < tri.Length; t += 3)
            {
                int i0 = tri[t], i1 = tri[t + 1], i2 = tri[t + 2];
                if (i0 == i1 || i1 == i2 || i0 == i2) continue;
                idxList.Add(i0); idxList.Add(i1); idxList.Add(i2);
                signedVol += Vector3.Dot(pos[i0], Vector3.Cross(pos[i1], pos[i2])) / 6.0;
            }
            var idx = idxList.ToArray();
            if (idx.Length == 0) { Fail("no non-degenerate triangles after weld"); return; }
            bool flip = signedVol < 0;
            if (flip)
                for (int t = 0; t < idx.Length; t += 3)
                    (idx[t + 1], idx[t + 2]) = (idx[t + 2], idx[t + 1]);

            var nrm = MeshOps.ComputeVertexNormals(pos, idx);

            // ---- curved lines: direct tube meshes at display resolution ----
            // NOT the Fuse-curves voxel path: the drawn Catmull-Rom polylines are
            // read back and swept into tubes, so the curves keep their on-screen
            // smoothness instead of being quantised to voxel size.
            int curveCount = 0;
            long curveMs = 0;
            phase.Restart();
            if (webIncludeCurves && curves != null)
            {
                var lines = new List<Vector3[]>();
                var lineCols = new List<Vector3>();
                if (curves.TryReadCurvePolylines(webCurveStride, lines, lineCols, out string why))
                {
                    var tp = new List<Vector3>(); var tn = new List<Vector3>();
                    var tc = new List<Vector3>(); var ti = new List<int>();
                    curveCount = CurveTubeBuilder.AppendCurveTubes(lines, lineCols, curves.brightness,
                        Mathf.Max(0.0005f, curves.ribbonWidth * 0.5f),
                        Mathf.Clamp(webCurveSides, 3, 12), webCurveTolerance, center, min.y, tp, tn, tc, ti,
                        webCurveTipTaper);
                    if (curveCount > 0)
                    {
                        int vOff = pos.Length, iOff = idx.Length;
                        Array.Resize(ref pos, vOff + tp.Count);
                        Array.Resize(ref nrm, vOff + tn.Count);
                        Array.Resize(ref col, vOff + tc.Count);
                        Array.Resize(ref idx, iOff + ti.Count);
                        tp.CopyTo(pos, vOff); tn.CopyTo(nrm, vOff); tc.CopyTo(col, vOff);
                        for (int i = 0; i < ti.Count; i++) idx[iOff + i] = ti[i] + vOff;
                        Debug.Log($"[TSDFPrintExporter] curves: {curveCount} tubes, " +
                                  $"{tp.Count} verts / {ti.Count / 3} tris (stride {webCurveStride})", this);
                    }
                }
                else
                    Debug.LogWarning($"[TSDFPrintExporter] web export: curves skipped — {why}", this);
            }
            curveMs = phase.ElapsedMilliseconds;

            string dir = Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.UserProfile),
                                      "Documents", "FloatingVectorsPrints");
            string stamp = DateTime.Now.ToString("yyyyMMdd_HHmmss");
            string glbPath = Path.Combine(dir, $"web_{stamp}.glb");
            string usdzPath = Path.Combine(dir, $"web_{stamp}.usdz");
            try
            {
                Directory.CreateDirectory(dir);
                phase.Restart();
                long glbBytes = GlbWriter.Write(glbPath, pos, nrm, col, idx);
                long glbMs = phase.ElapsedMilliseconds;
                phase.Restart();
                var ur = UsdzWriter.Write(usdzPath, pos, nrm, col, idx, usdPythonPath);
                long usdzMs = phase.ElapsedMilliseconds;
                if (!ur.binaryUsdc)
                    Debug.LogWarning("[TSDFPrintExporter] USDZ written as ASCII usda (no python with " +
                                     "usd-core found — 3-4x larger than binary usdc). One-time setup: " +
                                     "macOS: python3 -m venv ~/.venvs/usd && ~/.venvs/usd/bin/pip install usd-core / " +
                                     "Windows: py -3 -m venv %USERPROFILE%\\.venvs\\usd + " +
                                     "%USERPROFILE%\\.venvs\\usd\\Scripts\\pip install usd-core — " +
                                     "or set Usd Python Path on the exporter.", this);
                _status = $"Web: {idx.Length / 3} tris / {pos.Length} verts " +
                          $"(mesh {meshTris} + {curveCount} tubes) — " +
                          $"GLB {glbBytes / 1048576.0:0.0} MB + USDZ {ur.bytes / 1048576.0:0.0} MB " +
                          $"({(ur.binaryUsdc ? "usdc" : "usda fallback")}, atlas {ur.texSize}px) -> {dir} | " +
                          $"time: MC {mcMs / 1000.0:0.0}s, weld {weldMs / 1000.0:0.0}s, " +
                          $"smooth {smoothMs / 1000.0:0.0}s, decimate {decimateMs / 1000.0:0.0}s " +
                          $"({trisBeforeDec / 1000}k->{meshTris / 1000}k), curves {curveMs / 1000.0:0.0}s, " +
                          $"GLB {glbMs / 1000.0:0.0}s, USDZ {usdzMs / 1000.0:0.0}s" +
                          $"{(ur.binaryUsdc ? $" (usdc conv {ur.convertMs / 1000.0:0.0}s)" : "")}, " +
                          $"total {(total.ElapsedMilliseconds + mcMs) / 1000.0:0.0}s";
                Debug.Log("[TSDFPrintExporter] " + _status, this);
            }
            catch (Exception e)
            {
                Fail($"web export failed: {e.Message}");
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
            if (!TSDFComputeUtil.TryLoad(ref _bakeShader, "TSDFTrailBake", "TSDFPrintExporter", this))
            { Fail("Resources/TSDFTrailBake.compute not found"); return false; }
            _bakeKernel = _bakeShader.FindKernel("BakeTrail");
            return _bakeKernel >= 0;
        }

        private bool EnsureFloodShader()
        {
            if (_floodShader != null && _kFillWrite >= 0) return true;
            if (!TSDFComputeUtil.TryLoad(ref _floodShader, "TSDFFloodFill", "TSDFPrintExporter", this))
            { Fail("Resources/TSDFFloodFill.compute not found"); return false; }
            _kBuildMask = _floodShader.FindKernel("BuildMask");
            _kDilate = _floodShader.FindKernel("Dilate");
            _kSeedOutside = _floodShader.FindKernel("SeedOutside");
            _kPropagate = _floodShader.FindKernel("Propagate");
            _kMarkFilled = _floodShader.FindKernel("MarkFilled");
            _kErode = _floodShader.FindKernel("Erode");
            _kFillWrite = _floodShader.FindKernel("FillWrite");
            _kCapBoundary = _floodShader.FindKernel("CapBoundary");
            _kCullMask = _floodShader.FindKernel("CullMask");
            return true;
        }

        private bool EnsureMcShader()
        {
            if (_mcShader != null && _mcKernel >= 0) return true;
            if (!TSDFComputeUtil.TryLoad(ref _mcShader, "TSDFMarchingCubes", "TSDFPrintExporter", this))
            { Fail("Resources/TSDFMarchingCubes.compute not found"); return false; }
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
    }
}
