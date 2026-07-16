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
//   [Export STL]   the MC mesh (full resolution) as binary STL scaled to
//                  targetHeightMm (mm units, slicer convention), every shell
//                  wound outward via its own signed volume. By default
//                  (stlIncludeCurveTubes) the curved lines ride along as
//                  full-resolution CLOSED tube meshes + tapered body bridges,
//                  ALL rebuilt from ONE CSEmitSegs readback (the exact seed
//                  set Fuse curves would bake, so tubes and bridges match) —
//                  overlapping shells the slicer unions, no voxel fuse, so
//                  Fuse curves is NOT needed (and would double them up).
//                  Removed 2026-07-09, restored 2026-07-13 for the dancer-
//                  session print run (the colour-PLY variant stayed in history);
//                  curve tubes added 2026-07-16 (the voxel-fused curves came out
//                  mushy: ~1-voxel tubes + closing + Taubin ate the detail).
//   [Restore]      put the pre-print front buffer back (undo all of the above).
//
// All ops are one-shot and synchronous (GPU readbacks): export path, not per-frame.

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

        [Range(0, 30)]
        [Tooltip("Taubin smoothing iterations on export (0 = off). Non-shrinking smoothing " +
                 "that removes the Marching-Cubes staircase from tubes and surfaces; ~10 is " +
                 "a good start. Runs on the welded mesh, purely at export — the displayed " +
                 "TSDF mesh is untouched.")]
        public int smoothIterations = 10;

        [Min(100_000)]
        [Tooltip("Triangle capacity of the per-slab MC readback buffer (72 B each).")]
        public int triangleBudgetPerSlab = 2_000_000;

        [Header("STL curve tubes")]
        [Tooltip("Append the curved lines to the STL as closed tube meshes plus their " +
                 "body bridges as tapered tubes, all rebuilt from one CSEmitSegs pass " +
                 "(the exact chains + anchors Fuse curves would bake — full curve " +
                 "resolution, no voxel fuse). The slicer unions the overlapping shells. " +
                 "Fuse curves is then NOT needed for print (and would double the curves " +
                 "up), and Close holes only ever sees the body. Off = legacy: curves " +
                 "reach the STL only via Fuse curves.")]
        public bool stlIncludeCurveTubes = true;

        [Range(3, 12)]
        [Tooltip("Tube cross-section sides for the STL tubes. The print is physical — " +
                 "6 reads round at a 2 mm printed wire.")]
        public int stlCurveSides = 6;

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

        /// <summary>Print export: binary STL scaled to targetHeightMm (mm units,
        /// slicer convention). The body is the same MC + weld + Taubin pipeline
        /// as the web export (TSDFSnapshotBuilder) at full resolution (no
        /// decimation). With stlIncludeCurveTubes the curved lines are appended
        /// as closed tube meshes + tapered body bridges, all rebuilt from one
        /// CSEmitSegs readback (no voxel fuse, no resolution loss — see
        /// BuildCurveAndBridgeTubes); the slicer unions the overlapping shells.
        /// Every shell is wound outward via its own signed volume.</summary>
        public void ExportStl()
        {
            if (!Guard(needCurves: false)) return;
            var sw = System.Diagnostics.Stopwatch.StartNew();
            var opt = WebCaptureOptions();
            opt.meshTargetTris = 0;     // print wants full resolution
            opt.includeCurves = false;  // STL curves come from the CSEmitSegs readback below,
                                        // NOT the drawn ribbon buffer — see BuildCurveAndBridgeTubes
            var snap = TSDFSnapshotBuilder.Capture(volume, null, opt, out string err);
            if (snap == null) { Fail(err); return; }

            // Body shell: drop weld-collapsed slivers, then wind outward (MC
            // emits inside = sdf < iso, so a negative volume means inward).
            var pos = new List<Vector3>(snap.pos);
            var tri = new List<int>(snap.tri.Length);
            for (int t = 0; t + 2 < snap.tri.Length; t += 3)
            {
                int i0 = snap.tri[t], i1 = snap.tri[t + 1], i2 = snap.tri[t + 2];
                if (i0 == i1 || i1 == i2 || i0 == i2) continue;
                tri.Add(i0); tri.Add(i1); tri.Add(i2);
            }
            if (tri.Count == 0) { Fail("no non-degenerate triangles after weld"); return; }
            double bodyVol = OrientOutward(pos, tri, 0, tri.Count);
            int bodyTris = tri.Count / 3;

            // Curve tubes + body bridges as their own outward-wound shells. Both
            // come from ONE CSEmitSegs readback (the exact seed set Fuse curves
            // would bake), so every tube has its matching bridge — the drawn
            // ribbon buffer is a DIFFERENT nondeterministic seed subset and is
            // deliberately not used here.
            int tubeCount = 0, bridgeCount = 0;
            if (stlIncludeCurveTubes)
            {
                var tp = new List<Vector3>(); var tn = new List<Vector3>();
                var tc = new List<Vector3>(); var ti = new List<int>();
                BuildCurveAndBridgeTubes(tp, tn, tc, ti, out tubeCount, out bridgeCount);
                if (ti.Count > 0)
                {
                    int vOff = pos.Count, iBase = tri.Count;
                    pos.AddRange(tp);
                    for (int i = 0; i < ti.Count; i++) tri.Add(ti[i] + vOff);
                    OrientOutward(pos, tri, iBase, tri.Count - iBase);
                }
                else
                    Debug.LogWarning("[TSDFPrintExporter] STL: no curve tubes emitted " +
                                     "(no curves component / seeds all culled?) — exporting " +
                                     "the body only.", this);
            }

            // Bounds over EVERYTHING: tubes can reach past the body bbox and
            // they must land inside the printed height too.
            Vector3 min = Vector3.positiveInfinity, max = Vector3.negativeInfinity;
            foreach (var p in pos) { min = Vector3.Min(min, p); max = Vector3.Max(max, p); }
            Vector3 center = (min + max) * 0.5f;
            Vector3 size = max - min;
            if (size.y < 1e-4f) { Fail("degenerate mesh bounds"); return; }

            // Preflight: StlWriter builds the whole file in one MemoryStream, so
            // its hard ceiling is int.MaxValue bytes (~42M triangles at 50 B each).
            long stlBytes = 84L + 50L * (tri.Count / 3);
            if (stlBytes > int.MaxValue)
            {
                Fail($"STL would be {stlBytes / (1024 * 1024)} MB ({tri.Count / 3} tris) — over the " +
                     "2 GB writer limit. Raise Print Seed Stride / Web Curve Tolerance or lower " +
                     "STL Curve Sides.");
                return;
            }

            float scale = targetHeightMm / size.y; // metres -> printed mm
            string dir = Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.UserProfile),
                                      "Documents", "FloatingVectorsPrints");
            string stamp = DateTime.Now.ToString("yyyyMMdd_HHmmss");
            string stlPath = Path.Combine(dir, $"print_{stamp}.stl");
            try
            {
                Directory.CreateDirectory(dir);
                StlWriter.Write(stlPath, pos.ToArray(), tri.ToArray(), tri.Count / 3,
                                center, scale, flip: false);
                long bytes = new FileInfo(stlPath).Length;
                _status = $"STL: {tri.Count / 3} tris ({bodyTris} body + {tubeCount} tubes / " +
                          $"{bridgeCount} bridges), {pos.Count} verts, " +
                          $"{bytes / (1024 * 1024)} MB -> {stlPath} " +
                          $"(height {targetHeightMm:0} mm, smooth x{smoothIterations}, " +
                          $"bodyVol {bodyVol:0.0000} m^3, " +
                          $"total {sw.ElapsedMilliseconds / 1000.0:0.0}s)";
                Debug.Log("[TSDFPrintExporter] " + _status, this);
            }
            catch (Exception e)
            {
                Fail($"write failed: {e.Message}");
            }
        }

        /// <summary>Signed volume of tri[start..start+count) about the submesh
        /// bbox centre; flips the range in place when negative so it winds
        /// outward. Returns the absolute volume (m^3). The reference point only
        /// matters for open shells (an un-closed body), where it keeps the
        /// estimate stable — closed tube shells are translation-invariant.</summary>
        private static double OrientOutward(List<Vector3> pos, List<int> tri, int start, int count)
        {
            Vector3 min = Vector3.positiveInfinity, max = Vector3.negativeInfinity;
            for (int i = start; i < start + count; i++)
            {
                var p = pos[tri[i]];
                min = Vector3.Min(min, p); max = Vector3.Max(max, p);
            }
            Vector3 c = (min + max) * 0.5f;
            double vol = 0;
            for (int t = start; t + 2 < start + count; t += 3)
            {
                Vector3 a = pos[tri[t]] - c, b = pos[tri[t + 1]] - c, d = pos[tri[t + 2]] - c;
                vol += Vector3.Dot(a, Vector3.Cross(b, d)) / 6.0;
            }
            if (vol < 0)
            {
                for (int t = start; t + 2 < start + count; t += 3)
                    (tri[t + 1], tri[t + 2]) = (tri[t + 2], tri[t + 1]);
                vol = -vol;
            }
            return vol;
        }

        /// <summary>Curve + bridge tubes for the STL, ALL from one CSEmitSegs
        /// readback — the exact seed set (and bridge anchors) Fuse curves would
        /// bake, so every tube has its matching bridge. Reading the drawn ribbon
        /// buffer instead would pair tubes with a different nondeterministic seed
        /// subset (GPU append order — see PointCloudMotionCurves.Update), leaving
        /// bridges to curves that aren't in the STL and tubes with no attachment.
        /// Segments carry tag = seed*2 + isBridge in their pad; a single thread's
        /// appends get monotonically increasing buffer indices, so per-seed chain
        /// order is just buffer order. Chains become closed constant-radius tubes
        /// (polyline [a0, b0, b1, ...]); each bridge becomes a 2-point tapered
        /// tube — printRadius at the curve tip, ×bridgeRadiusScale at the body
        /// side (fillet root, like the voxel-fuse path had).</summary>
        private void BuildCurveAndBridgeTubes(List<Vector3> tp, List<Vector3> tn,
                                              List<Vector3> tc, List<int> ti,
                                              out int tubeCount, out int bridgeCount)
        {
            tubeCount = 0; bridgeCount = 0;
            if (curves == null) return;
            int segCap = curves.MaxPrintSegs(printSeedStride);
            var segBuf = new ComputeBuffer(segCap, TrailBakeOps.SegStride);
            try
            {
                if (!curves.EmitPrintSegs(segBuf, printSeedStride, printRadius,
                                          bridgeRadiusScale, maxBridgeLength, out var st))
                    return;
                int n = Mathf.Min(st.emitted, segCap);
                if (n <= 0) return;
                if (st.dropped > 0)
                    Debug.LogWarning($"[TSDFPrintExporter] STL curves: {st.dropped} segments dropped " +
                                     $"(cap {segCap}) — some tubes will be truncated or jump; raise " +
                                     "printSeedStride.", this);
                if (st.bridgesSkipped > 0)
                    Debug.LogWarning($"[TSDFPrintExporter] STL bridges: {st.bridgesSkipped} skipped " +
                                     $"(over maxBridgeLength {maxBridgeLength} m) — those curves " +
                                     "hang on their tip contact only.", this);
                var segs = new TrailBakeOps.TrailSeg[n];
                segBuf.GetData(segs, 0, 0, n);

                // Regroup the interleaved appends: chains keyed by seed (buffer
                // order within a seed = emission order), bridges as-is.
                var chains = new Dictionary<uint, List<int>>();
                var bridges = new List<int>();
                for (int i = 0; i < n; i++)
                {
                    uint code = (uint)segs[i].pad; // seed*2 + isBridge
                    if ((code & 1u) != 0u) { bridges.Add(i); continue; }
                    uint seed = code >> 1;
                    if (!chains.TryGetValue(seed, out var list))
                        chains[seed] = list = new List<int>(64);
                    list.Add(i);
                }

                var line = new List<Vector3[]> { null };
                var col = new List<Vector3> { Vector3.one };
                var pts = new List<Vector3>(64);
                foreach (var kv in chains)
                {
                    var idxs = kv.Value;
                    pts.Clear();
                    pts.Add(segs[idxs[0]].a);
                    foreach (int i in idxs) pts.Add(segs[i].b);
                    if (pts.Count < 2) continue;
                    line[0] = pts.ToArray();
                    col[0] = segs[idxs[0]].color;
                    // Constant radius: a tapered tail would print below the wire floor.
                    tubeCount += CurveTubeBuilder.AppendCurveTubes(line, col, 1f, printRadius,
                        stlCurveSides, webCurveTolerance, Vector3.zero, 0f, tp, tn, tc, ti,
                        tipTaper: 1f, exportSpace: false);
                }

                foreach (int i in bridges)
                {
                    if ((segs[i].b - segs[i].a).sqrMagnitude < 1e-10f) continue;
                    line[0] = new[] { segs[i].a, segs[i].b };
                    col[0] = segs[i].color;
                    float rb = Mathf.Max(segs[i].rb, 1e-4f);
                    CurveTubeBuilder.AppendCurveTubes(line, col, 1f, rb, stlCurveSides,
                        0f, Vector3.zero, 0f, tp, tn, tc, ti,
                        tipTaper: Mathf.Clamp01(segs[i].ra / rb), exportSpace: false);
                    bridgeCount++;
                }
            }
            finally { segBuf.Release(); }
        }

        /// <summary>The web/AR capture options from the panel-tuned fields —
        /// shared by ExportWeb and the operator publish flow (OperatorPublisher).</summary>
        public TSDFSnapshotBuilder.CaptureOptions WebCaptureOptions() => new TSDFSnapshotBuilder.CaptureOptions
        {
            smoothIterations = smoothIterations,
            triangleBudgetPerSlab = triangleBudgetPerSlab,
            meshTargetTris = webMeshTargetTris,
            includeCurves = webIncludeCurves,
            curveStride = webCurveStride,
            curveSides = webCurveSides,
            curveTipTaper = webCurveTipTaper,
            curveTolerance = webCurveTolerance,
        };

        /// <summary>Web/AR export: .glb (vertex colours, web viewers) and .usdz
        /// (texture-atlas colours, iPhone AR Quick Look), both real-world metres.
        /// The heavy lifting (MC readback, weld/smooth/decimate, curve tubes,
        /// file writing) lives in TSDFSnapshotBuilder — this wires the panel
        /// settings through and reports the timings.</summary>
        public void ExportWeb()
        {
            if (!Guard(needCurves: false)) return;
            var sw = System.Diagnostics.Stopwatch.StartNew();
            var snap = TSDFSnapshotBuilder.Capture(volume, curves, WebCaptureOptions(), out string err);
            if (snap == null) { Fail(err); return; }

            string dir = Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.UserProfile),
                                      "Documents", "FloatingVectorsPrints");
            string stamp = DateTime.Now.ToString("yyyyMMdd_HHmmss");
            string glbPath = Path.Combine(dir, $"web_{stamp}.glb");
            string usdzPath = Path.Combine(dir, $"web_{stamp}.usdz");
            if (!TSDFSnapshotBuilder.ExportFiles(snap, glbPath, usdzPath, usdPythonPath,
                                                 out var st, out err))
            { Fail(err); return; }

            if (!st.usdzBinary)
                Debug.LogWarning("[TSDFPrintExporter] USDZ written as ASCII usda (no python with " +
                                 "usd-core found — 3-4x larger than binary usdc). One-time setup: " +
                                 "macOS: python3 -m venv ~/.venvs/usd && ~/.venvs/usd/bin/pip install usd-core / " +
                                 "Windows: py -3 -m venv %USERPROFILE%\\.venvs\\usd + " +
                                 "%USERPROFILE%\\.venvs\\usd\\Scripts\\pip install usd-core — " +
                                 "or set Usd Python Path on the exporter.", this);
            _status = $"Web: {st.finalTris} tris / {st.finalVerts} verts " +
                      $"(mesh {snap.MeshTris} + {st.curveCount} tubes) — " +
                      $"GLB {st.glbBytes / 1048576.0:0.0} MB + USDZ {st.usdzBytes / 1048576.0:0.0} MB " +
                      $"({(st.usdzBinary ? "usdc" : "usda fallback")}, atlas {st.usdzTexSize}px) -> {dir} | " +
                      $"time: MC {snap.mcMs / 1000.0:0.0}s, weld {snap.weldMs / 1000.0:0.0}s, " +
                      $"smooth {snap.smoothMs / 1000.0:0.0}s, decimate {snap.decimateMs / 1000.0:0.0}s " +
                      $"({snap.trisBeforeDecimate / 1000}k->{snap.MeshTris / 1000}k), curves {st.curveMs / 1000.0:0.0}s, " +
                      $"GLB {st.glbMs / 1000.0:0.0}s, USDZ {st.usdzMs / 1000.0:0.0}s" +
                      $"{(st.usdzBinary ? $" (usdc conv {st.usdzConvertMs / 1000.0:0.0}s)" : "")}, " +
                      $"total {sw.ElapsedMilliseconds / 1000.0:0.0}s";
            Debug.Log("[TSDFPrintExporter] " + _status, this);
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

        private void Dispatch3D(int kernel, Vector3Int dim)
        {
            _floodShader.Dispatch(kernel,
                Mathf.Max(1, Mathf.CeilToInt(dim.x / 4f)),
                Mathf.Max(1, Mathf.CeilToInt(dim.y / 4f)),
                Mathf.Max(1, Mathf.CeilToInt(dim.z / 4f)));
        }
    }
}
