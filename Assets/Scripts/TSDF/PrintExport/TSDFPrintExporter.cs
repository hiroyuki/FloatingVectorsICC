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
//                  (stlIncludeFloorPlane) adds a square floor plate as one more
//                  closed shell, centred on the dancer's floor contact.
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

        [Range(0.05f, 1f)]
        [Tooltip("Radius scale at the tube's PAST end (trail tail). 1 = constant " +
                 "radius; smaller = the trail thins out into the past. Watch the " +
                 "print minimum: taper * printRadius must stay above the wire floor.")]
        public float tubeTailTaper = 1f;

        [Tooltip("Raindrop silhouette along each tube: spherical head at the " +
                 "NEWEST end (motion direction), chopstick tail thinning into " +
                 "the past (tail tip = Tube Tail Taper * head radius). Off = " +
                 "plain linear taper (with Curve Sides 4 + low taper, each " +
                 "trail reads as a long square pyramid — the chosen look).")]
        public bool tubeRaindrop = false;

        [Tooltip("Align the cross-section to WORLD UP instead of the twisting " +
                 "parallel-transport frame: with 4 sides one vertex always points " +
                 "up, so horizontal runs print as self-supporting 45° diamond " +
                 "roofs instead of flat overhangs (FDM).")]
        public bool tubeAlignUp = true;

        [Tooltip("Weld bead at the closest approach of every touching chain " +
                 "pair: a short fat strut buried in the joint, so graze contacts " +
                 "print as a real bond instead of a knife-edge seam. White span.")]
        public bool stlContactBeads = true;

        [Range(1f, 2f)]
        [Tooltip("Bead strut radius as a multiple of the thicker contact radius.")]
        public float stlContactBeadScale = 1.15f;

        [Header("Crop (test prints)")]
        [Tooltip("Export only the chains inside the crop box (chains are split " +
                 "into their in-box runs). The floor plate shrinks to the box " +
                 "footprint and the print SCALE comes from Full Height Ref, so a " +
                 "crop prints at exactly the full sculpture's thickness.")]
        public bool cropEnabled = false;

        public Vector3 cropCenter = new Vector3(0f, 0.3f, 0f);

        public Vector3 cropSize = new Vector3(0.6f, 0.6f, 0.6f);

        [Tooltip("Full-sculpture height (m) used for the crop's print scale: " +
                 "scale = Target Height Mm / this, identical to the full export.")]
        public float stlFullHeightRef = 1.9f;

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
                 "wide. Bigger also rounds off concavities. With direct curve tubes " +
                 "(stlIncludeCurveTubes) the closing no longer needs to swallow curves " +
                 "into the body volume, so a small radius that just seals camera-unseen " +
                 "gaps is enough — 2 by default (was 5 in the voxel-fuse era).")]
        public int closeRadiusVoxels = 2;

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

        [Header("STL shells")]
        [Tooltip("Include the body (TSDF Marching-Cubes shell) in the STL. Off = " +
                 "curve tubes only — an aesthetic variant without the body and " +
                 "WITHOUT the body bridges (they'd be floating spikes with nothing " +
                 "to attach to). The floor plate is ALWAYS included (the print " +
                 "cannot stand without it); with the body off it centres on the " +
                 "tubes' floor-contact band instead.")]
        public bool stlIncludeBody = true;

        [Header("STL curve supports")]
        [Tooltip("Self-made supports for the curves (slicer support inside the wire " +
                 "tangle cannot be removed — FDM/A2L). From each curve endpoint " +
                 "(plus interior points every Support Spacing), a SHORT strut " +
                 "connects to the nearest point of another curve inside a 45° " +
                 "downward cone — struts hide inside the curve cloud instead of " +
                 "forming a pillar forest. Only points with nothing below them " +
                 "drop a pillar to the floor plate.")]
        public bool stlCurveSupportPillars = true;

        [Range(0.05f, 0.5f)]
        [Tooltip("Horizontal search radius (m) for the strut target below. Wider " +
                 "finds more curve-to-curve struts (fewer floor pillars) but allows " +
                 "longer, more visible struts.")]
        public float stlSupportSearchRadius = 0.15f;

        [Range(0f, 1f)]
        [Tooltip("Arc-length spacing (m) for interior support pillars along each " +
                 "curve. 0 = endpoints only.")]
        public float stlSupportSpacing = 0f;

        [Range(0.001f, 0.02f)]
        [Tooltip("Support pillar radius (m). Print Radius-sized reads as part of " +
                 "the artwork; thinner reads as strings holding the curves.")]
        public float stlSupportRadius = 0.004f;

        [Tooltip("Chains that touch neither another chain nor the floor band get " +
                 "a strut (Support Radius) to the nearest other chain — rendered " +
                 "in the BODY colour so the joins read as armature, not artwork.")]
        public bool stlLinkIsolatedChains = true;

        [Range(0f, 0.2f)]
        [Tooltip("Floor anchoring: chains whose lowest point is within this band " +
                 "(m) above the global minimum get a thick pillar down to the " +
                 "plate, bonding the feet-area curves to it. 0 = off.")]
        public float stlFloorPillarBand = 0.05f;

        [Range(0.004f, 0.03f)]
        [Tooltip("Floor anchor pillar radius (m) — structural, thicker than the " +
                 "curve-to-curve struts.")]
        public float stlFloorPillarRadius = 0.012f;

        [Header("3MF colours (Bambu multi-filament)")]
        [Tooltip("Body + floor plate colour in the 3MF export. Bambu Studio's " +
                 "Standard-3MF colour parsing maps it to the closest filament.")]
        public Color threeMfBodyColor = Color.black;

        [Tooltip("Curve tubes + bridges + wireframe colour in the 3MF export.")]
        public Color threeMfCurveColor = Color.white;

        [Header("STL root wireframe (tubes-only mode)")]
        [Tooltip("With the body OFF, connect the curves' body-side anchor points to " +
                 "each other with thin wires: a minimum spanning tree (guarantees ONE " +
                 "connected piece — powder printing drops disconnected parts) plus " +
                 "N-nearest-neighbour edges for density. The anchors sit on the " +
                 "dancer's surface at bone-classified points, so the graph reads as a " +
                 "ghost-body wireframe. Bridges then connect each curve to it.")]
        public bool stlRootWireframe = true;

        [Range(0, 4)]
        [Tooltip("Extra wires from every anchor to its N nearest anchors (on top of " +
                 "the spanning tree). 0 = tree only (sparsest — the picked look, " +
                 "2026-07-17); 2 reads as a mesh-y wireframe.")]
        public int stlWireframeNeighbors = 0;

        [Range(0.05f, 0.5f)]
        [Tooltip("Skip neighbour wires longer than this (m) — stops e.g. hand-to-hip " +
                 "chords cutting across the body. Spanning-tree edges ignore the cap " +
                 "(connectivity beats aesthetics).")]
        public float stlWireframeMaxEdge = 0.25f;

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

        [Header("STL floor plane")]
        // The floor plate is NOT optional: the print cannot stand without it
        // (user rule 2026-07-17). A square plate is always appended as its own
        // closed shell (the slicer unions whatever overlaps it). Centred on the
        // dancer — the centroid of the body shell's floor-contact band (tubes'
        // band when the body is off), NOT the mesh bbox centre, which trails and
        // outstretched arms would drag off the body. Lies in the volume grid
        // frame, so it stays parallel to the sculpture's floor-crop plane even
        // with the bbox tilted to the real floor.

        [Range(0.5f, 3f)]
        [Tooltip("Floor plate side length (m, square, real-world scale — printed size " +
                 "follows targetHeightMm like everything else).")]
        public float stlFloorSize = 1.5f;

        [Range(0.005f, 0.1f)]
        [Tooltip("Floor plate thickness (m, real-world scale). 0.02 m at a 1/8 print " +
                 "scale ≈ 2.5 mm plate.")]
        public float stlFloorThickness = 0.02f;

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
        /// <summary>Everything ExportStl/Export3mf share: the assembled shells
        /// (body + curve tubes/bridges/wires + floor plate) with the triangle
        /// spans of each colour family, plus bounds. Tri-index boundaries:
        /// body = [0, CurveTriStart), curves = [CurveTriStart, FloorTriStart),
        /// floor = [FloorTriStart, Tri.Count).</summary>
        private sealed class ShellAssembly
        {
            public readonly List<Vector3> Pos = new List<Vector3>();
            public readonly List<int> Tri = new List<int>();
            public int CurveTriStart, WireTriStart, SupportTriStart, FloorPillarTriStart, FloorTriStart;
            public int BodyTris, TubeCount, BridgeCount, WireCount, SupportCount, FloorTris;
            public double BodyVol;
            public Vector2 FloorAt;
            public Vector3 Center, Size;
        }

        private ShellAssembly AssembleShells()
        {
            var asm = new ShellAssembly();
            var pos = asm.Pos; var tri = asm.Tri;

            // Body shell: drop weld-collapsed slivers, then wind outward (MC
            // emits inside = sdf < iso, so a negative volume means inward).
            // stlIncludeBody=false skips the whole capture — tubes-only variant.
            if (stlIncludeBody)
            {
                var opt = WebCaptureOptions();
                opt.meshTargetTris = 0;     // print wants full resolution
                opt.includeCurves = false;  // STL curves come from the CSEmitSegs readback below,
                                            // NOT the drawn ribbon buffer — see BuildCurveAndBridgeTubes
                var snap = TSDFSnapshotBuilder.Capture(volume, null, opt, out string err);
                if (snap == null) { Fail(err); return null; }
                pos.AddRange(snap.pos);
                tri.Capacity = snap.tri.Length;
                for (int t = 0; t + 2 < snap.tri.Length; t += 3)
                {
                    int i0 = snap.tri[t], i1 = snap.tri[t + 1], i2 = snap.tri[t + 2];
                    if (i0 == i1 || i1 == i2 || i0 == i2) continue;
                    tri.Add(i0); tri.Add(i1); tri.Add(i2);
                }
                if (tri.Count == 0) { Fail("no non-degenerate triangles after weld"); return null; }
                asm.BodyVol = OrientOutward(pos, tri, 0, tri.Count);
            }
            asm.BodyTris = tri.Count / 3;
            int bodyVerts = pos.Count; // floor-plate contact band scans pos[0..this)
            asm.CurveTriStart = tri.Count;

            // Curve tubes + body bridges as their own outward-wound shells. Both
            // come from ONE CSEmitSegs readback (the exact seed set Fuse curves
            // would bake), so every tube has its matching bridge — the drawn
            // ribbon buffer is a DIFFERENT nondeterministic seed subset and is
            // deliberately not used here.
            if (stlIncludeCurveTubes)
            {
                var tp = new List<Vector3>(); var tn = new List<Vector3>();
                var tc = new List<Vector3>(); var ti = new List<int>();
                // bridges attach tubes to the body — or, in tubes-only mode with
                // the root wireframe, to the anchor graph. Without either target
                // they would be floating spikes, so they are skipped.
                bool wireframe = !stlIncludeBody && stlRootWireframe;
                var supportPts = stlCurveSupportPillars ? new List<(Vector3 p, int chain)>() : null;
                var supportCands = stlCurveSupportPillars ? new List<(Vector3 p, int chain)>() : null;
                var chainLows = stlFloorPillarBand > 1e-4f ? new List<Vector3>() : null;
                BuildCurveAndBridgeTubes(tp, tn, tc, ti, out asm.TubeCount, out asm.BridgeCount,
                                         includeBridges: stlIncludeBody || wireframe,
                                         rootWireframe: wireframe, out asm.WireCount,
                                         out int wireTriStartLocal,
                                         supportPts, supportCands, chainLows);
                if (ti.Count > 0)
                {
                    int vOff = pos.Count, iBase = tri.Count;
                    pos.AddRange(tp);
                    for (int i = 0; i < ti.Count; i++) tri.Add(ti[i] + vOff);
                    OrientOutward(pos, tri, iBase, tri.Count - iBase);
                    asm.WireTriStart = wireTriStartLocal >= 0 ? iBase + wireTriStartLocal : tri.Count;
                }
                else
                {
                    asm.WireTriStart = tri.Count;
                    Debug.LogWarning("[TSDFPrintExporter] STL: no curve tubes emitted " +
                                     "(no curves component / seeds all culled?) — exporting " +
                                     "the body only.", this);
                }

                // Self-made supports: from each support point, a SHORT strut to
                // the nearest OTHER curve's point inside a 45° downward cone —
                // struts hide inside the curve cloud instead of forming a pillar
                // forest ("地面から脚を生やすと全部埋もれる", 2026-07-17). Points
                // with no target below get NO strut ("床に落ちているサポートの柱
                // いらない") — the slicer's own supports handle the bottom layer.
                asm.SupportTriStart = tri.Count;
                if (supportPts != null && supportPts.Count > 0 && pos.Count > 0)
                {
                    float floorMinY = float.PositiveInfinity;
                    foreach (var p in pos) if (p.y < floorMinY) floorMinY = p.y;

                    // XZ spatial hash over the strut target candidates
                    float cell = Mathf.Max(0.05f, stlSupportSearchRadius);
                    var grid = new Dictionary<long, List<int>>();
                    long CellKey(float x, float z)
                        => ((long)Mathf.FloorToInt(x / cell) << 32) ^ (uint)Mathf.FloorToInt(z / cell);
                    for (int i = 0; i < supportCands.Count; i++)
                    {
                        long k = CellKey(supportCands[i].p.x, supportCands[i].p.z);
                        if (!grid.TryGetValue(k, out var list)) grid[k] = list = new List<int>(8);
                        list.Add(i);
                    }

                    var sp = new List<Vector3>(); var sn = new List<Vector3>();
                    var sc = new List<Vector3>(); var si = new List<int>();
                    var strutLine = new List<Vector3[]> { null };
                    var strutCol = new List<Vector3> { Vector3.one };
                    float minDrop = stlSupportRadius * 3f;
                    int floorFallbacks = 0;
                    foreach (var (p, chain) in supportPts)
                    {
                        // nearest other-chain point inside the 45° downward cone
                        Vector3 best = default; float bestSq = float.MaxValue; bool found = false;
                        int cx = Mathf.FloorToInt(p.x / cell), cz = Mathf.FloorToInt(p.z / cell);
                        for (int gx = cx - 1; gx <= cx + 1; gx++)
                            for (int gz = cz - 1; gz <= cz + 1; gz++)
                            {
                                if (!grid.TryGetValue(((long)gx << 32) ^ (uint)gz, out var list)) continue;
                                foreach (int ci in list)
                                {
                                    var (q, qChain) = supportCands[ci];
                                    if (qChain == chain) continue;
                                    float dy = p.y - q.y;
                                    if (dy < minDrop) continue;                  // must be below
                                    float hx = q.x - p.x, hz = q.z - p.z;
                                    float horizSq = hx * hx + hz * hz;
                                    if (horizSq > dy * dy) continue;             // outside 45° cone
                                    if (horizSq > stlSupportSearchRadius * stlSupportSearchRadius) continue;
                                    float dSq = horizSq + dy * dy;
                                    if (dSq < bestSq) { bestSq = dSq; best = q; found = true; }
                                }
                            }

                        if (!found)
                        {
                            // printability fallback: a minimum with NOTHING below
                            // in the cone is a mid-air island — it must drop a
                            // pillar to the plate or the print fails there
                            if (p.y - floorMinY < minDrop) continue; // already at the plate
                            best = new Vector3(p.x, floorMinY, p.z);
                            floorFallbacks++;
                        }
                        // hourglass join: wide waist, flares buried in both lines
                        AppendFlaredStrut(p, best,
                            printRadius * 0.85f,
                            found ? printRadius * 0.85f : stlSupportRadius * 1.5f,
                            Mathf.Max(stlSupportRadius, printRadius * 0.55f),
                            strutLine, strutCol, sp, sn, sc, si);
                        asm.SupportCount++;
                    }
                    if (si.Count > 0)
                    {
                        int vOff = pos.Count, iBase = tri.Count;
                        pos.AddRange(sp);
                        for (int i = 0; i < si.Count; i++) tri.Add(si[i] + vOff);
                        OrientOutward(pos, tri, iBase, tri.Count - iBase);
                        Debug.Log($"[TSDFPrintExporter] self-support: {asm.SupportCount} struts, " +
                                  $"{floorFallbacks} of them floor pillars (no target below).", this);
                    }
                }

                asm.FloorPillarTriStart = tri.Count;
                // Floor anchors: chains whose lowest point sits within the band
                // above the global minimum get a THICK pillar down to the plate
                // ("足元がプレートにくっついてない…太めのサポートでいい") — the
                // plate top sinks 5 mm into the lowest geometry, so a pillar to
                // minY is embedded, not merely touching.
                if (chainLows != null && chainLows.Count > 0 && pos.Count > 0)
                {
                    float minY = float.PositiveInfinity;
                    foreach (var p in pos) if (p.y < minY) minY = p.y;

                    var sp = new List<Vector3>(); var sn = new List<Vector3>();
                    var sc = new List<Vector3>(); var si = new List<int>();
                    var pilLine = new List<Vector3[]> { null };
                    var pilCol = new List<Vector3> { Vector3.one };
                    int pillars = 0;
                    foreach (var p in chainLows)
                    {
                        float above = p.y - minY;
                        if (above > stlFloorPillarBand) continue;      // not a feet-area chain
                        if (above < stlFloorPillarRadius) continue;    // already inside the plate sink
                        // pillar top runs 5 cm past the chain's lowest point so it
                        // embeds INTO the line — the exported tube centerline is
                        // simplified and can drift off the raw low point, which
                        // left some pillars floating next to their line.
                        pilLine[0] = new[] { new Vector3(p.x, p.y + 0.05f, p.z), new Vector3(p.x, minY, p.z) };
                        CurveTubeBuilder.AppendCurveTubes(pilLine, pilCol, 1f, stlFloorPillarRadius,
                            stlCurveSides, 0f, Vector3.zero, 0f, sp, sn, sc, si,
                            tipTaper: 1f, exportSpace: false);
                        pillars++;
                    }
                    if (si.Count > 0)
                    {
                        int vOff = pos.Count, iBase = tri.Count;
                        pos.AddRange(sp);
                        for (int i = 0; i < si.Count; i++) tri.Add(si[i] + vOff);
                        OrientOutward(pos, tri, iBase, tri.Count - iBase);
                        asm.SupportCount += pillars;
                        Debug.Log($"[TSDFPrintExporter] floor anchors: {pillars} pillars " +
                                  $"(band {stlFloorPillarBand} m, r {stlFloorPillarRadius} m).", this);
                    }
                }
            }
            if (tri.Count == 0) { Fail("nothing to export (body and tubes both off/empty)"); return null; }
            if (!stlIncludeCurveTubes)
            { asm.WireTriStart = tri.Count; asm.SupportTriStart = tri.Count; asm.FloorPillarTriStart = tri.Count; }
            asm.FloorTriStart = tri.Count;

            // Floor plate: after the tubes so its top can sink into the lowest
            // geometry of EVERYTHING (a tube dipping below the feet would
            // otherwise poke through the plate).
            // always present — the print cannot stand without its floor plate.
            // contact band scans the body verts; tubes-only falls back to all verts
            asm.FloorTris = AppendFloorPlane(pos, tri, bodyVerts > 0 ? bodyVerts : pos.Count, out asm.FloorAt);

            // Bounds over EVERYTHING: tubes can reach past the body bbox and
            // they must land inside the printed height too.
            Vector3 min = Vector3.positiveInfinity, max = Vector3.negativeInfinity;
            foreach (var p in pos) { min = Vector3.Min(min, p); max = Vector3.Max(max, p); }
            asm.Center = (min + max) * 0.5f;
            asm.Size = max - min;
            if (asm.Size.y < 1e-4f) { Fail("degenerate mesh bounds"); return null; }
            return asm;
        }

        public void ExportStl()
        {
            if (!Guard(needCurves: !stlIncludeBody)) return;
            var sw = System.Diagnostics.Stopwatch.StartNew();
            var asm = AssembleShells();
            if (asm == null) return;
            var pos = asm.Pos; var tri = asm.Tri;
            int bodyTris = asm.BodyTris, tubeCount = asm.TubeCount, bridgeCount = asm.BridgeCount,
                wireCount = asm.WireCount, floorTris = asm.FloorTris;
            double bodyVol = asm.BodyVol;
            Vector2 floorAt = asm.FloorAt;
            Vector3 center = asm.Center, size = asm.Size;

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

            float scale = ExportScale(size.y); // metres -> printed mm
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
                string floorNote = floorTris > 0
                    ? $" + {stlFloorSize:0.0#} m floor @ ({floorAt.x:0.00}, {floorAt.y:0.00})"
                    : "";
                string wireNote = wireCount > 0 ? $" + {wireCount} wires" : "";
                string supNote = asm.SupportCount > 0 ? $" + {asm.SupportCount} supports" : "";
                _status = $"STL: {tri.Count / 3} tris ({bodyTris} body + {tubeCount} tubes / " +
                          $"{bridgeCount} bridges{wireNote}{supNote}{floorNote}), {pos.Count} verts, " +
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

        /// <summary>Colour 3MF for multi-filament printing (Bambu AMS): the same
        /// shells as ExportStl with per-triangle face colours — body + floor in
        /// threeMfBodyColor, curves/bridges/wires in threeMfCurveColor. Bambu
        /// Studio's Standard-3MF colour parsing dialog maps the two colours to
        /// AMS filaments on import. Geometry/scale/orientation are identical to
        /// the STL (mm, Unity axes, outward winding).</summary>
        public void Export3mf()
        {
            if (!Guard(needCurves: !stlIncludeBody)) return;
            var sw = System.Diagnostics.Stopwatch.StartNew();
            var asm = AssembleShells();
            if (asm == null) return;

            float scale = ExportScale(asm.Size.y); // metres -> printed mm
            var spans = BuildColourSpans(asm);
            int total = asm.Tri.Count / 3;

            string dir = Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.UserProfile),
                                      "Documents", "FloatingVectorsPrints");
            string path3mf = Path.Combine(dir, $"print_{DateTime.Now:yyyyMMdd_HHmmss}.3mf");
            try
            {
                Directory.CreateDirectory(dir);
                ThreeMfWriter.Write(path3mf, asm.Pos, asm.Tri, spans,
                    new[] { ("Body", threeMfBodyColor), ("Curves", threeMfCurveColor) },
                    asm.Center, scale);
                long bytes = new FileInfo(path3mf).Length;
                string wireNote = asm.WireCount > 0 ? $" + {asm.WireCount} wires" : "";
                string supNote = asm.SupportCount > 0 ? $" + {asm.SupportCount} supports" : "";
                _status = $"3MF: {total} tris ({asm.BodyTris} body + {asm.TubeCount} tubes / " +
                          $"{asm.BridgeCount} bridges{wireNote}{supNote} + floor), 2 colours, " +
                          $"{bytes / (1024 * 1024)} MB -> {path3mf} " +
                          $"(height {targetHeightMm:0} mm, total {sw.ElapsedMilliseconds / 1000.0:0.0}s)";
                Debug.Log("[TSDFPrintExporter] " + _status, this);
            }
            catch (Exception e)
            {
                Fail($"3MF write failed: {e.Message}");
            }
        }

        /// <summary>Colour spans over the assembled shells. Material 0 = Body
        /// colour (body mesh, root wireframe, isolation links, floor pillars and
        /// the floor plate — the black armature), material 1 = Curve colour
        /// (tubes, bridges, curve-to-curve struts — the white artwork).</summary>
        private static List<ThreeMfWriter.Span> BuildColourSpans(ShellAssembly asm)
        {
            var spans = new List<ThreeMfWriter.Span>(5);
            void Add(int fromTriIdx, int toTriIdx, int mat)
            {
                int start = fromTriIdx / 3, count = (toTriIdx - fromTriIdx) / 3;
                if (count > 0) spans.Add(new ThreeMfWriter.Span { StartTri = start, TriCount = count, Material = mat });
            }
            Add(0, asm.CurveTriStart, 0);                          // body
            Add(asm.CurveTriStart, asm.WireTriStart, 1);           // tubes + bridges
            Add(asm.WireTriStart, asm.SupportTriStart, 0);         // wireframe + isolation links
            Add(asm.SupportTriStart, asm.FloorPillarTriStart, 1);  // curve-to-curve struts
            Add(asm.FloorPillarTriStart, asm.Tri.Count, 0);        // floor pillars + plate
            return spans;
        }

        /// <summary>Two-material OBJ+MTL (same shells and spans as Export3mf) —
        /// the PREVIEWABLE colour variant: MeshLab/Blender render the materials,
        /// and Bambu Studio's colored-OBJ import maps them to filaments too.
        /// Check colours here first, then ship the 3MF.</summary>
        public void ExportObj()
        {
            if (!Guard(needCurves: !stlIncludeBody)) return;
            var sw = System.Diagnostics.Stopwatch.StartNew();
            var asm = AssembleShells();
            if (asm == null) return;

            float scale = ExportScale(asm.Size.y);
            var spans = BuildColourSpans(asm);
            int total = asm.Tri.Count / 3;

            string dir = Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.UserProfile),
                                      "Documents", "FloatingVectorsPrints");
            string pathObj = Path.Combine(dir, $"print_{DateTime.Now:yyyyMMdd_HHmmss}.obj");
            try
            {
                Directory.CreateDirectory(dir);
                ObjMaterialWriter.Write(pathObj, asm.Pos, asm.Tri, spans,
                    new[] { ("Body", threeMfBodyColor), ("Curves", threeMfCurveColor) },
                    asm.Center, scale);
                long bytes = new FileInfo(pathObj).Length;
                string wireNote = asm.WireCount > 0 ? $" + {asm.WireCount} wires" : "";
                string supNote = asm.SupportCount > 0 ? $" + {asm.SupportCount} supports" : "";
                _status = $"OBJ: {total} tris ({asm.BodyTris} body + {asm.TubeCount} tubes / " +
                          $"{asm.BridgeCount} bridges{wireNote}{supNote} + floor), 2 materials, " +
                          $"{bytes / (1024 * 1024)} MB -> {pathObj} " +
                          $"(height {targetHeightMm:0} mm, total {sw.ElapsedMilliseconds / 1000.0:0.0}s)";
                Debug.Log("[TSDFPrintExporter] " + _status, this);
            }
            catch (Exception e)
            {
                Fail($"OBJ write failed: {e.Message}");
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
                                              out int tubeCount, out int bridgeCount,
                                              bool includeBridges, bool rootWireframe,
                                              out int wireCount, out int wireTriStartLocal,
                                              List<(Vector3 p, int chain)> supportPts = null,
                                              List<(Vector3 p, int chain)> supportCandidates = null,
                                              List<Vector3> chainLows = null)
        {
            tubeCount = 0; bridgeCount = 0; wireCount = 0; wireTriStartLocal = -1;
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
                bool wantLinkData = stlLinkIsolatedChains || stlContactBeads;
                var linkPts = wantLinkData ? new List<(Vector3 p, float r)>() : null;
                var linkRanges = wantLinkData ? new List<(int start, int count)>() : null;
                int emittedChains = 0, tubesOut = 0;

                // one emitted tube — a whole chain, or one in-crop run of it
                void EmitChain(List<Vector3> cp, Vector3 colour)
                {
                    if (cp.Count < 2) return;
                    int chainId = emittedChains++;
                    line[0] = cp.ToArray();
                    col[0] = colour;
                    tubesOut += CurveTubeBuilder.AppendCurveTubes(line, col, 1f, printRadius,
                        stlCurveSides, webCurveTolerance, Vector3.zero, 0f, tp, tn, tc, ti,
                        tipTaper: Mathf.Clamp01(tubeTailTaper), exportSpace: false,
                        raindrop: tubeRaindrop, alignUp: tubeAlignUp);

                    if (chainLows != null)
                    {
                        Vector3 low = cp[0];
                        for (int i = 1; i < cp.Count; i++) if (cp[i].y < low.y) low = cp[i];
                        chainLows.Add(low);
                    }

                    if (linkPts != null)
                    {
                        linkRanges.Add((linkPts.Count, cp.Count));
                        float taper = Mathf.Clamp01(tubeTailTaper);
                        for (int i = 0; i < cp.Count; i++)
                            linkPts.Add((cp[i], printRadius * Mathf.Lerp(taper, 1f,
                                cp.Count > 1 ? (float)i / (cp.Count - 1) : 1f)));
                    }

                    // support-strut anchor points (endpoints + interior samples
                    // every stlSupportSpacing of arc length; 0 = endpoints only)
                    // and strut TARGET candidates (all chain points, tagged with
                    // the chain id so a curve never "supports" itself)
                    if (supportPts != null)
                    {
                        // an endpoint is an island only when the chain DESCENDS
                        // into it — upward-pointing ends rest on their own tube
                        if (cp[0].y < cp[1].y) supportPts.Add((cp[0], chainId));
                        if (cp[cp.Count - 1].y < cp[cp.Count - 2].y)
                            supportPts.Add((cp[cp.Count - 1], chainId));
                        // local Y-minima with real PROMINENCE: FDM islands start
                        // at minima, but a jitter dip self-bridges — only valleys
                        // hanging >= 3 cm below their surroundings (within 12 cm
                        // of arc) need an anchor, or the strut count explodes on
                        // wiggly 32-sample chains
                        const float promArc = 0.12f, promDepth = 0.03f;
                        float[] arcCum = new float[cp.Count];
                        for (int i = 1; i < cp.Count; i++)
                            arcCum[i] = arcCum[i - 1] + (cp[i] - cp[i - 1]).magnitude;
                        float lastKeptArc = float.NegativeInfinity;
                        for (int i = 1; i < cp.Count - 1; i++)
                        {
                            if (!(cp[i].y < cp[i - 1].y && cp[i].y <= cp[i + 1].y)) continue;
                            float rise = float.PositiveInfinity;
                            float maxL = float.NegativeInfinity, maxR = float.NegativeInfinity;
                            for (int j = i - 1; j >= 0 && arcCum[i] - arcCum[j] <= promArc; j--)
                                maxL = Mathf.Max(maxL, cp[j].y);
                            for (int j = i + 1; j < cp.Count && arcCum[j] - arcCum[i] <= promArc; j++)
                                maxR = Mathf.Max(maxR, cp[j].y);
                            rise = Mathf.Min(maxL, maxR) - cp[i].y;
                            if (rise < promDepth) continue;
                            if (arcCum[i] - lastKeptArc < promArc) continue; // dedupe close valleys
                            lastKeptArc = arcCum[i];
                            supportPts.Add((cp[i], chainId));
                        }
                        if (stlSupportSpacing > 1e-4f)
                        {
                            float since = 0f;
                            for (int i = 1; i < cp.Count - 1; i++)
                            {
                                since += (cp[i] - cp[i - 1]).magnitude;
                                if (since >= stlSupportSpacing) { supportPts.Add((cp[i], chainId)); since = 0f; }
                            }
                        }
                        if (supportCandidates != null)
                            for (int i = 0; i < cp.Count; i++)
                                supportCandidates.Add((cp[i], chainId));
                    }
                }

                var cropB = new Bounds(cropCenter, cropSize);
                var run = new List<Vector3>(64);
                foreach (var kv in chains)
                {
                    var idxs = kv.Value;
                    pts.Clear();
                    pts.Add(segs[idxs[0]].a);
                    foreach (int i in idxs) pts.Add(segs[i].b);
                    if (pts.Count < 2) continue;
                    Vector3 colour = segs[idxs[0]].color;
                    if (!cropEnabled) { EmitChain(pts, colour); continue; }
                    // crop: each maximal in-box run becomes its own tube; drop
                    // stubs under 5 cm — chopped fragments read as debris
                    void EmitRun()
                    {
                        float arc = 0f;
                        for (int i = 1; i < run.Count; i++) arc += (run[i] - run[i - 1]).magnitude;
                        if (arc >= 0.05f) EmitChain(run, colour);
                        run.Clear();
                    }
                    run.Clear();
                    foreach (var q in pts)
                    {
                        if (cropB.Contains(q)) { run.Add(q); continue; }
                        EmitRun();
                    }
                    EmitRun();
                }
                tubeCount = tubesOut;

                if (stlContactBeads)
                    AppendContactBeads(linkPts, linkRanges, line, col, tp, tn, tc, ti);

                if (includeBridges)
                {
                    var anchors = new List<Vector3>(bridges.Count);
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
                        anchors.Add(segs[i].b); // body-side anchor = wireframe node
                    }

                    if (rootWireframe)
                    {
                        wireTriStartLocal = ti.Count; // wires get their own colour span
                        wireCount = AppendRootWireframe(anchors, line, col, tp, tn, tc, ti);
                    }
                }

                if (stlLinkIsolatedChains)
                    AppendIsolationLinks(linkPts, linkRanges, line, col, tp, tn, tc, ti,
                                         ref wireTriStartLocal);
            }
            finally { segBuf.Release(); }
        }

        /// <summary>Hourglass strut: a thin waist flaring smoothly (parabolic)
        /// out to rA/rB at the ends, which are extended half a radius INTO the
        /// tubes they join — the join reads as a smooth membrane growing out of
        /// the lines, not a stick jammed between them ("つなぎ目はもっと滑らかに
        /// つながっていていい", 2026-07-17).</summary>
        private void AppendFlaredStrut(Vector3 a, Vector3 b, float rA, float rB, float waist,
                                       List<Vector3[]> line, List<Vector3> col,
                                       List<Vector3> tp, List<Vector3> tn, List<Vector3> tc, List<int> ti)
        {
            Vector3 ab = b - a;
            float d = ab.magnitude;
            if (d < 1e-5f) return;
            Vector3 dir = ab / d;
            Vector3 ea = a - dir * (rA * 0.5f), eb = b + dir * (rB * 0.5f);
            const int N = 9;
            var strutPts = new Vector3[N];
            var radii = new float[N];
            for (int i = 0; i < N; i++)
            {
                float t = i / (float)(N - 1);
                float s = 2f * t - 1f;
                strutPts[i] = Vector3.Lerp(ea, eb, t);
                radii[i] = waist + (Mathf.Lerp(rA, rB, t) - waist) * s * s;
            }
            line[0] = strutPts;
            col[0] = Vector3.one;
            CurveTubeBuilder.AppendCurveTubes(line, col, 1f, waist, stlCurveSides, 0f,
                Vector3.zero, 0f, tp, tn, tc, ti, tipTaper: 1f, exportSpace: false,
                ringRadii: radii);
        }

        /// <summary>Weld beads: ONE short fat strut at the closest approach of
        /// every touching (or just-grazing) chain pair. A graze prints as a
        /// knife-edge seam that snaps in handling; the bead buries real material
        /// in the joint. Appended before the wire span, so it stays in the
        /// artwork (Curve) colour.</summary>
        private void AppendContactBeads(List<(Vector3 p, float r)> pts, List<(int start, int count)> ranges,
                                        List<Vector3[]> line, List<Vector3> col,
                                        List<Vector3> tp, List<Vector3> tn, List<Vector3> tc, List<int> ti)
        {
            if (pts == null || ranges == null || ranges.Count < 2) return;
            int n = pts.Count;
            var chainOf = new int[n];
            for (int c = 0; c < ranges.Count; c++)
                for (int i = 0; i < ranges[c].count; i++)
                    chainOf[ranges[c].start + i] = c;

            float cell = Mathf.Max(0.03f, printRadius * 4f);
            var grid = new Dictionary<long, List<int>>(n / 2 + 1);
            long Key(int kx, int ky, int kz) => (kx * 73856093L) ^ (ky * 19349663L) ^ (kz * 83492791L);
            int C(float v) => Mathf.FloorToInt(v / cell);
            for (int i = 0; i < n; i++)
            {
                long k = Key(C(pts[i].p.x), C(pts[i].p.y), C(pts[i].p.z));
                if (!grid.TryGetValue(k, out var l)) grid[k] = l = new List<int>(8);
                l.Add(i);
            }

            // closest approach per touching pair (visit each pair once: a < b)
            var best = new Dictionary<long, (float dSq, Vector3 a, Vector3 b, float r)>();
            for (int i = 0; i < n; i++)
            {
                var (p, r) = pts[i];
                int cx = C(p.x), cy = C(p.y), cz = C(p.z);
                for (int gx = cx - 1; gx <= cx + 1; gx++)
                    for (int gy = cy - 1; gy <= cy + 1; gy++)
                        for (int gz = cz - 1; gz <= cz + 1; gz++)
                        {
                            if (!grid.TryGetValue(Key(gx, gy, gz), out var l)) continue;
                            foreach (int qi in l)
                            {
                                if (chainOf[qi] <= chainOf[i]) continue;
                                var (q, qr) = pts[qi];
                                float lim = (r + qr) * 1.1f; // touching or just grazing
                                float dSq = (q - p).sqrMagnitude;
                                if (dSq > lim * lim) continue;
                                long pairKey = ((long)chainOf[i] << 32) | (uint)chainOf[qi];
                                if (!best.TryGetValue(pairKey, out var cur) || dSq < cur.dSq)
                                    best[pairKey] = (dSq, p, q, Mathf.Max(r, qr));
                            }
                        }
            }

            int beads = 0;
            foreach (var kvp in best)
            {
                var (dSq, a, b, r) = kvp.Value;
                float d = Mathf.Sqrt(dSq);
                // weld ONLY true grazes: solid overlaps (centrelines closer than
                // 75% of the touch distance) bond fine on their own, and beading
                // every contact reads as lumps all over the artwork
                if (d < r * 1.5f) continue;
                // smooth hourglass web between the two lines, wide waist
                AppendFlaredStrut(a, b, r, r, r * 0.6f, line, col, tp, tn, tc, ti);
                beads++;
            }
            if (beads > 0)
                Debug.Log($"[TSDFPrintExporter] contact beads: {beads} welds.", this);
        }

        /// <summary>Struts for chains that touch nothing: neither another chain
        /// (centreline distance <= r1+r2 anywhere along them) nor the floor-pillar
        /// band. Each isolated chain gets ONE strut to the nearest point of
        /// another chain, appended into the WIRE span so it prints in the Body
        /// colour — the joins read as black armature, not artwork.</summary>
        private void AppendIsolationLinks(List<(Vector3 p, float r)> pts, List<(int start, int count)> ranges,
                                          List<Vector3[]> line, List<Vector3> col,
                                          List<Vector3> tp, List<Vector3> tn, List<Vector3> tc, List<int> ti,
                                          ref int wireTriStartLocal)
        {
            if (pts == null || ranges == null || ranges.Count < 2) return;

            int n = pts.Count;
            var chainOf = new int[n];
            float minY = float.PositiveInfinity;
            for (int c = 0; c < ranges.Count; c++)
                for (int i = 0; i < ranges[c].count; i++)
                    chainOf[ranges[c].start + i] = c;
            for (int i = 0; i < n; i++) if (pts[i].p.y < minY) minY = pts[i].p.y;

            // 3D spatial hash (hash collisions only add distance checks)
            float cell = Mathf.Max(0.03f, printRadius * 4f);
            var grid = new Dictionary<long, List<int>>(n / 2 + 1);
            long Key(int kx, int ky, int kz)
                => (kx * 73856093L) ^ (ky * 19349663L) ^ (kz * 83492791L);
            int C(float v) => Mathf.FloorToInt(v / cell);
            for (int i = 0; i < n; i++)
            {
                long k = Key(C(pts[i].p.x), C(pts[i].p.y), C(pts[i].p.z));
                if (!grid.TryGetValue(k, out var l)) grid[k] = l = new List<int>(8);
                l.Add(i);
            }

            int links = 0, unresolved = 0;
            for (int c = 0; c < ranges.Count; c++)
            {
                var (start, count) = ranges[c];

                // floor band → the pillar pass bonds it to the plate
                float low = float.PositiveInfinity;
                for (int i = 0; i < count; i++) low = Mathf.Min(low, pts[start + i].p.y);
                if (stlFloorPillarBand > 1e-4f && low - minY <= stlFloorPillarBand) continue;

                bool touching = false;
                for (int i = 0; i < count && !touching; i++)
                {
                    var (p, r) = pts[start + i];
                    int cx = C(p.x), cy = C(p.y), cz = C(p.z);
                    for (int gx = cx - 1; gx <= cx + 1 && !touching; gx++)
                        for (int gy = cy - 1; gy <= cy + 1 && !touching; gy++)
                            for (int gz = cz - 1; gz <= cz + 1 && !touching; gz++)
                            {
                                if (!grid.TryGetValue(Key(gx, gy, gz), out var l)) continue;
                                foreach (int qi in l)
                                {
                                    if (chainOf[qi] == c) continue;
                                    var (q, qr) = pts[qi];
                                    float lim = r + qr;
                                    if ((q - p).sqrMagnitude <= lim * lim) { touching = true; break; }
                                }
                            }
                }
                if (touching) continue;

                // isolated: one strut from EACH end ("先頭としっぽ") to that
                // end's nearest other-chain point, expanding shell search
                // capped at 0.5 m
                int maxR = Mathf.CeilToInt(0.5f / cell);
                int made = 0;
                var probes = count > 1 ? new[] { start, start + count - 1 } : new[] { start };
                foreach (int pi in probes)
                {
                    var (p, _) = pts[pi];
                    Vector3 best = default; float bestSq = float.MaxValue;
                    int cx = C(p.x), cy = C(p.y), cz = C(p.z);
                    int foundAt = -1;
                    for (int R = 0; R <= maxR; R++)
                    {
                        if (foundAt >= 0 && R > foundAt + 1) break; // one shell past the hit
                        for (int gx = cx - R; gx <= cx + R; gx++)
                            for (int gy = cy - R; gy <= cy + R; gy++)
                                for (int gz = cz - R; gz <= cz + R; gz++)
                                {
                                    if (Mathf.Max(Mathf.Abs(gx - cx),
                                        Mathf.Max(Mathf.Abs(gy - cy), Mathf.Abs(gz - cz))) != R) continue;
                                    if (!grid.TryGetValue(Key(gx, gy, gz), out var l)) continue;
                                    foreach (int qi in l)
                                    {
                                        if (chainOf[qi] == c) continue;
                                        float dSq = (pts[qi].p - p).sqrMagnitude;
                                        if (dSq < bestSq)
                                        {
                                            bestSq = dSq; best = pts[qi].p;
                                            if (foundAt < 0) foundAt = R;
                                        }
                                    }
                                }
                    }
                    if (bestSq == float.MaxValue) continue;

                    if (wireTriStartLocal < 0) wireTriStartLocal = ti.Count;
                    // hourglass join, wide waist — same smooth language as the
                    // white struts, in the black armature span
                    AppendFlaredStrut(p, best, printRadius * 0.85f, printRadius * 0.85f,
                        printRadius * 0.55f, line, col, tp, tn, tc, ti);
                    links++; made++;
                }
                if (made == 0) unresolved++;
            }
            if (links > 0 || unresolved > 0)
                Debug.Log($"[TSDFPrintExporter] isolation links: {links} black struts" +
                          (unresolved > 0 ? $", {unresolved} chains had no neighbour within 0.5 m" : "") +
                          ".", this);
        }

        /// <summary>Tubes-only mode: wire the curves' body-side anchors into ONE
        /// connected graph — a minimum spanning tree (powder printing drops
        /// disconnected parts, so connectivity is structural, not aesthetic)
        /// plus stlWireframeNeighbors nearest-neighbour edges for density. The
        /// anchors lie on the dancer's surface, so the graph reads as a
        /// ghost-body wireframe. Returns the number of wires appended.</summary>
        private int AppendRootWireframe(List<Vector3> anchors,
                                        List<Vector3[]> line, List<Vector3> col,
                                        List<Vector3> tp, List<Vector3> tn,
                                        List<Vector3> tc, List<int> ti)
        {
            int n = anchors.Count;
            if (n < 2) return 0;

            var edges = new HashSet<long>();
            long Key(int i, int j) => i < j ? ((long)i << 32) | (uint)j : ((long)j << 32) | (uint)i;

            // Prim's MST (O(n^2), n ~ hundreds) — unconditional edges: the tree
            // is what makes the sculpture one piece.
            var inTree = new bool[n];
            var bestD = new float[n];
            var bestJ = new int[n];
            for (int i = 0; i < n; i++) { bestD[i] = float.MaxValue; bestJ[i] = -1; }
            inTree[0] = true;
            for (int i = 1; i < n; i++) { bestD[i] = (anchors[i] - anchors[0]).sqrMagnitude; bestJ[i] = 0; }
            for (int step = 1; step < n; step++)
            {
                int k = -1; float kd = float.MaxValue;
                for (int i = 0; i < n; i++)
                    if (!inTree[i] && bestD[i] < kd) { kd = bestD[i]; k = i; }
                if (k < 0) break;
                inTree[k] = true;
                if (kd > 1e-8f) edges.Add(Key(k, bestJ[k])); // skip coincident anchors
                for (int i = 0; i < n; i++)
                {
                    if (inTree[i]) continue;
                    float d = (anchors[i] - anchors[k]).sqrMagnitude;
                    if (d < bestD[i]) { bestD[i] = d; bestJ[i] = k; }
                }
            }

            // density pass: N nearest neighbours per anchor, capped by edge length
            float maxSq = stlWireframeMaxEdge * stlWireframeMaxEdge;
            if (stlWireframeNeighbors > 0)
            {
                var dist = new List<(float d, int j)>(n);
                for (int i = 0; i < n; i++)
                {
                    dist.Clear();
                    for (int j = 0; j < n; j++)
                    {
                        if (j == i) continue;
                        float d = (anchors[j] - anchors[i]).sqrMagnitude;
                        if (d > 1e-8f && d <= maxSq) dist.Add((d, j));
                    }
                    dist.Sort((a, b) => a.d.CompareTo(b.d));
                    for (int k = 0; k < Mathf.Min(stlWireframeNeighbors, dist.Count); k++)
                        edges.Add(Key(i, dist[k].j));
                }
            }

            int wires = 0;
            foreach (long key in edges)
            {
                int i = (int)(key >> 32), j = (int)(key & 0xFFFFFFFF);
                line[0] = new[] { anchors[i], anchors[j] };
                col[0] = Vector3.one;
                CurveTubeBuilder.AppendCurveTubes(line, col, 1f, printRadius, stlCurveSides,
                    0f, Vector3.zero, 0f, tp, tn, tc, ti,
                    tipTaper: 1f, exportSpace: false);
                wires++;
            }
            return wires;
        }

        // Floor plate: how deep the plate top sinks into the lowest geometry
        // (guarantees shell overlap for the slicer union) and how far above the
        // body's lowest point vertices still count as "standing on the floor".
        private const float kFloorEmbed = 0.005f;
        private const float kFloorContactBand = 0.15f;

        /// <summary>Append the floor plate: a stlFloorSize² × stlFloorThickness
        /// box built in the volume grid frame (the BoundingVolume rotation —
        /// TSDFVolume anchors its voxels to it, so the plate lies parallel to
        /// the sculpture's floor-crop plane even with the bbox tilted to the
        /// real floor). XZ-centred on the dancer: the centroid of body-shell
        /// vertices within kFloorContactBand of the body's lowest point — the
        /// mesh bbox centre would drift with trails or an outstretched arm, the
        /// feet don't. Top sits kFloorEmbed above the lowest geometry so the
        /// shells overlap; wound outward like every other shell. Returns the
        /// triangle count added; <paramref name="floorAt"/> gets the plate
        /// centre (grid-frame XZ, for the status line).</summary>
        private int AppendFloorPlane(List<Vector3> pos, List<int> tri, int bodyVerts,
                                     out Vector2 floorAt)
        {
            Quaternion rot = volume != null && volume.boundingBox != null
                ? volume.boundingBox.transform.rotation
                : Quaternion.identity;
            Quaternion inv = Quaternion.Inverse(rot);

            float bodyMinY = float.PositiveInfinity, allMinY = float.PositiveInfinity;
            for (int i = 0; i < pos.Count; i++)
            {
                float y = (inv * pos[i]).y;
                if (y < allMinY) allMinY = y;
                if (i < bodyVerts && y < bodyMinY) bodyMinY = y;
            }

            double sx = 0, sz = 0; long n = 0;
            for (int i = 0; i < bodyVerts; i++)
            {
                var l = inv * pos[i];
                if (l.y > bodyMinY + kFloorContactBand) continue;
                sx += l.x; sz += l.z; n++;
            }
            float cx = (float)(sx / n), cz = (float)(sz / n); // n >= 1: the min vertex is in band
            floorAt = new Vector2(cx, cz);

            // crop test prints get a plate matching the crop footprint
            float floorSize = cropEnabled
                ? Mathf.Min(stlFloorSize, Mathf.Max(cropSize.x, cropSize.z))
                : stlFloorSize;
            float half = floorSize * 0.5f;
            float topY = allMinY + kFloorEmbed;
            int iBase = tri.Count;
            // u × v = outward normal (component algebra) — all six faces
            // consistent, OrientOutward then only confirms the sign.
            void Face(Vector3 o, Vector3 u, Vector3 v)
            {
                int b = pos.Count;
                pos.Add(rot * o); pos.Add(rot * (o + u));
                pos.Add(rot * (o + u + v)); pos.Add(rot * (o + v));
                tri.Add(b); tri.Add(b + 1); tri.Add(b + 2);
                tri.Add(b); tri.Add(b + 2); tri.Add(b + 3);
            }
            var ux = new Vector3(floorSize, 0f, 0f);
            var uz = new Vector3(0f, 0f, floorSize);
            var ut = new Vector3(0f, stlFloorThickness, 0f);
            var c00 = new Vector3(cx - half, topY - stlFloorThickness, cz - half);
            Face(c00, ux, uz);                                            // bottom (-Y)
            Face(new Vector3(cx - half, topY, cz - half), uz, ux);        // top (+Y)
            Face(c00, uz, ut);                                            // -X
            Face(new Vector3(cx + half, c00.y, cz - half), ut, uz);       // +X
            Face(c00, ut, ux);                                            // -Z
            Face(new Vector3(cx - half, c00.y, cz + half), ux, ut);       // +Z
            OrientOutward(pos, tri, iBase, tri.Count - iBase);
            return (tri.Count - iBase) / 3;
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

        /// <summary>Print scale (mm per metre). A crop uses the FULL sculpture's
        /// reference height so test prints come out at the exact production
        /// thickness; the full export scales its own bounds to targetHeightMm.</summary>
        private float ExportScale(float sizeY)
            => targetHeightMm / (cropEnabled ? Mathf.Max(0.1f, stlFullHeightRef) : Mathf.Max(1e-4f, sizeY));

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
