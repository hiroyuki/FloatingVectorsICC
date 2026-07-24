// Frozen-moment capture of the displayed sculpture (TSDF mesh + motion-curve
// polylines), split out of TSDFPrintExporter so the experience flow can take
// three captures in a row and export them later:
//
//   Capture()            volume + curves -> TSDFSnapshot. Full-grid Marching
//                        Cubes readback, weld/smooth/decimate, curve polyline
//                        readback. Non-destructive: reads the front buffer
//                        only, the displayed volume is untouched.
//   BuildDisplayMeshes() TSDFSnapshot -> world-space Meshes for in-scene
//                        display (the selection screen's three sculptures).
//   ExportFiles()        TSDFSnapshot -> .glb + .usdz on disk, real-world
//                        metres, recentred/mirrored to right-handed Y-up.
//                        Atomic: writes <path>.tmp, renames on success.
//
// Coordinate contract: TSDFSnapshot holds everything in WORLD space (Unity,
// left-handed) plus the export-space metadata (center, minY) computed ONCE
// from the surface mesh at capture time. ExportFiles and the curve tubes both
// derive their export transform from that same metadata, so mesh and curves
// stay registered exactly as they did when TSDFPrintExporter.WriteWebFiles
// did everything inline.

using System;
using System.Collections.Generic;
using System.IO;
using BodyTracking;
using UnityEngine;

namespace TSDF
{
    /// <summary>One frozen capture: world-space surface mesh + curve polylines
    /// + the export metadata and per-stage timings.</summary>
    public sealed class TSDFSnapshot
    {
        // Surface mesh, WORLD space (welded + smoothed + decimated).
        public Vector3[] pos;
        public Vector3[] col;
        public int[] tri;

        // Curve polylines, WORLD space (oldest -> newest point order), one
        // colour per line. Empty lists when curves were skipped/unavailable.
        public List<Vector3[]> curveLines = new List<Vector3[]>();
        public List<Vector3> curveColors = new List<Vector3>();

        // Displayed point cloud, WORLD space + sRGB colour (BoundingVolume +
        // decimater applied). Null when the capture didn't include it. Exported
        // through the SAME center/minY as the mesh/curves, so it stays registered.
        public Vector3[] pcPos;
        public Vector3[] pcCol;

        // Curve tube parameters frozen at capture time (the curves component
        // may be retuned between capture and export).
        public float curveBrightness;
        public float curveRadius;
        public int curveSides;
        public float curveTolerance;
        public float curveTipTaper;

        // Export-space metadata: XZ centre and base height of the SURFACE mesh
        // bounds, computed once at capture. Both the mesh and the curve tubes
        // recentre/mirror through these same values on export.
        public Vector3 center;
        public float minY;

        // Stage timings / counts (surfaced in the exporter status line).
        public long mcMs, weldMs, smoothMs, decimateMs;
        public long mcTris;            // raw MC triangle count before weld
        public int trisBeforeDecimate; // welded+smoothed, pre-decimation

        public int MeshTris => tri.Length / 3;
    }

    public static class TSDFSnapshotBuilder
    {
        public struct CaptureOptions
        {
            public int smoothIterations;      // Taubin iterations (0 = off)
            public int triangleBudgetPerSlab; // per-slab MC readback capacity
            public int meshTargetTris;        // decimation target (0 = off)
            // True = Capture returns the UNDECIMATED mesh and the caller runs
            // Decimate(snap, meshTargetTris) itself, typically on a worker thread
            // so the main thread keeps rendering. Bounds are refreshed there.
            public bool deferDecimation;
            public bool includeCurves;
            public int curveStride;           // every Nth drawn curve
            public int curveSides;            // tube cross-section sides
            public float curveTipTaper;       // radius scale at the old end
            public float curveTolerance;      // polyline simplification (m)
            public bool includePointCloud;    // also grab the displayed point cloud
        }

        public struct ExportStats
        {
            public int finalTris, finalVerts; // mesh + tubes as written
            public int curveCount;            // tubes emitted
            public int pointCount;            // point-cloud points written
            public long curveMs, aoMs, glbMs, usdzMs;
            public long glbBytes, usdzBytes;
            public int usdzTexSize;
            public bool usdzBinary;           // usdc (true) vs usda fallback
            public long usdzConvertMs;
        }

        private static ComputeShader s_mcShader; // Resources/TSDFMarchingCubes
        private static int s_mcKernel = -1;

        // ---------------- capture ----------------

        /// <summary>Reads the displayed volume + curves into a frozen snapshot.
        /// Non-destructive (front buffer reads only). Returns null with
        /// <paramref name="error"/> set on failure. Curves failing to read is
        /// NOT an error: the capture proceeds mesh-only with a warning.</summary>
        /// <summary>
        /// Reduce the surface mesh to <paramref name="targetTris"/> and refresh the
        /// export bounds. Touches nothing but the snapshot's plain arrays, so it is
        /// safe to call from a worker thread — which is the point: at show sizes
        /// this is seconds of single-threaded work, and running it inline freezes
        /// rendering between the progress bar filling and the result appearing.
        /// No-op when the mesh is already at or under the target.
        /// </summary>
        public static void Decimate(TSDFSnapshot snap, int targetTris)
        {
            if (snap == null || targetTris <= 0) return;
            if (snap.tri.Length / 3 <= targetTris) return;
            var phase = System.Diagnostics.Stopwatch.StartNew();
            int before = snap.tri.Length / 3;
            MeshDecimator.Simplify(ref snap.pos, ref snap.col, ref snap.tri, targetTris);
            snap.decimateMs = phase.ElapsedMilliseconds;
            ComputeBounds(snap); // decimation moves vertices; bounds follow
            Debug.Log($"[TSDFSnapshotBuilder] decimated {before} -> {snap.tri.Length / 3} tris " +
                      $"({snap.pos.Length} verts) in {snap.decimateMs} ms");
        }

        // False when the mesh is flat enough to be unusable for export.
        static bool ComputeBounds(TSDFSnapshot snap)
        {
            Vector3 min = Vector3.positiveInfinity, max = Vector3.negativeInfinity;
            foreach (var p in snap.pos) { min = Vector3.Min(min, p); max = Vector3.Max(max, p); }
            if ((max - min).y < 1e-4f) return false;
            snap.center = (min + max) * 0.5f;
            snap.minY = min.y;
            return true;
        }

        public static TSDFSnapshot Capture(TSDFVolume volume, PointCloudMotionCurves curves,
                                           CaptureOptions opt, out string error)
        {
            error = null;
            if (volume == null || volume.Dim.x <= 0 || volume.FrontBuffer == null)
            { error = "no initialised TSDFVolume"; return null; }
            if (!EnsureMcShader()) { error = "Resources/TSDFMarchingCubes.compute not found"; return null; }

            var snap = new TSDFSnapshot();
            var sw = System.Diagnostics.Stopwatch.StartNew();
            var slabs = RunMarchingCubes(volume, opt.triangleBudgetPerSlab, out long totalTris, out error);
            if (slabs == null) return null;
            snap.mcMs = sw.ElapsedMilliseconds;
            snap.mcTris = totalTris;

            var phase = System.Diagnostics.Stopwatch.StartNew();
            MeshOps.WeldSlabs(slabs, out snap.pos, out snap.col, out snap.tri);
            slabs.Clear(); // free the soup copies before smoothing allocates
            snap.weldMs = phase.ElapsedMilliseconds;
            Debug.Log($"[TSDFSnapshotBuilder] welded to {snap.pos.Length} verts in {snap.weldMs} ms " +
                      $"— smoothing x{opt.smoothIterations}...");
            phase.Restart();
            if (opt.smoothIterations > 0)
                MeshOps.TaubinSmooth(snap.pos, snap.tri, opt.smoothIterations);
            snap.smoothMs = phase.ElapsedMilliseconds;

            snap.trisBeforeDecimate = snap.tri.Length / 3;
            // Decimation is by far the longest phase (seconds at show sizes) and
            // it is pure array math — callers that must keep rendering hand it to
            // a worker thread via Decimate() instead of paying it here.
            if (!opt.deferDecimation)
                Decimate(snap, opt.meshTargetTris);

            // Export metadata off the SURFACE mesh, once — the curve tubes use
            // the same values so mesh and curves stay registered. Recomputed by
            // Decimate() when the caller deferred it.
            if (!ComputeBounds(snap)) { error = "degenerate mesh bounds"; return null; }

            // Curve polylines at display resolution, world space. A paused pose
            // or hidden curves is a mesh-only capture, not a failure.
            if (opt.includeCurves && curves != null)
            {
                if (curves.TryReadCurvePolylines(opt.curveStride, snap.curveLines, snap.curveColors,
                                                 out string why))
                {
                    snap.curveBrightness = curves.brightness;
                    snap.curveRadius = Mathf.Max(0.0005f, curves.ribbonWidth * 0.5f);
                    snap.curveSides = Mathf.Clamp(opt.curveSides, 3, 12);
                    snap.curveTolerance = opt.curveTolerance;
                    snap.curveTipTaper = opt.curveTipTaper;
                }
                else
                    Debug.LogWarning($"[TSDFSnapshotBuilder] capture: curves skipped — {why}");
            }

            // Point cloud at the SAME frame, through the same content filters
            // the display shader runs (region, decimater, capsule person-mask,
            // floor mask). Kept in world space; ExportFiles recentres it through
            // snap.center/minY so it registers with the mesh and curves.
            if (opt.includePointCloud)
            {
                var bounds = UnityEngine.Object.FindFirstObjectByType<PointCloud.BoundingVolume>();
                var deci = UnityEngine.Object.FindFirstObjectByType<PointCloud.PointCloudDecimater>();
                var caps = UnityEngine.Object.FindFirstObjectByType<PointCloud.PointCloudCapsuleFilter>();
                var floor = UnityEngine.Object.FindFirstObjectByType<PointCloud.PointCloudFloorMask>();
                float keep = (deci != null && deci.Enabled) ? deci.KeepRatio : 1f;
                PointCloud.PointCloudSnapshot.Gather(bounds, keep, 12345, out snap.pcPos, out snap.pcCol,
                                                     caps, floor);
                Debug.Log($"[TSDFSnapshotBuilder] point cloud: {snap.pcPos.Length} pts " +
                          $"(bounds {(bounds != null ? bounds.Mode.ToString() : "none")}, keep {keep:0.00}, " +
                          $"caps {(caps != null ? caps.Mode.ToString() : "none")}, " +
                          $"floor {(floor != null && floor.isActiveAndEnabled && floor.MaskActive ? "on" : "off")})");
            }
            return snap;
        }

        // ---------------- display meshes ----------------

        /// <summary>World-space Meshes for in-scene display: the surface with
        /// vertex colours, and the curve tubes (null when the snapshot has no
        /// curves). No mirroring/recentring — render them where the sculpture
        /// stood. Tube winding is export-oriented, so display with a Cull Off
        /// material (unlit vertex colour).</summary>
        public static void BuildDisplayMeshes(TSDFSnapshot snap, out Mesh surface, out Mesh tubes)
        {
            surface = MakeMesh(snap.pos, MeshOps.ComputeVertexNormals(snap.pos, snap.tri),
                               snap.col, snap.tri, "SnapshotSurface");

            tubes = null;
            if (snap.curveLines.Count == 0) return;
            var tp = new List<Vector3>(); var tn = new List<Vector3>();
            var tc = new List<Vector3>(); var ti = new List<int>();
            int emitted = CurveTubeBuilder.AppendCurveTubes(snap.curveLines, snap.curveColors,
                snap.curveBrightness, snap.curveRadius, snap.curveSides, snap.curveTolerance,
                snap.center, snap.minY, tp, tn, tc, ti, snap.curveTipTaper, exportSpace: false);
            if (emitted == 0) return;
            tubes = MakeMesh(tp.ToArray(), tn.ToArray(), tc.ToArray(), ti.ToArray(), "SnapshotTubes");
        }

        private static Mesh MakeMesh(Vector3[] pos, Vector3[] nrm, Vector3[] col, int[] tri, string name)
        {
            var m = new Mesh { name = name };
            if (pos.Length > 65535) m.indexFormat = UnityEngine.Rendering.IndexFormat.UInt32;
            m.SetVertices(pos);
            m.SetNormals(nrm);
            var colors = new Color[col.Length];
            for (int i = 0; i < col.Length; i++) colors[i] = new Color(col[i].x, col[i].y, col[i].z, 1f);
            m.SetColors(colors);
            m.SetTriangles(tri, 0, calculateBounds: true);
            return m;
        }

        // ---------------- export ----------------

        /// <summary>How ExportFiles turns the snapshot into web files. default =
        /// legacy behaviour: mesh + all captured curves, no AO.</summary>
        public struct WebFileOptions
        {
            public MeshAO.Options ao;   // strength 0 = no AO bake
            public bool omitMesh;       // curves-only files (mesh still captured for
                                        // registration; falls back to mesh when a
                                        // capture has no curves)
            public long curveBudgetBytes; // >0: pick curves to fit this GLB size
            public float shortCurveShare; // 0..1 of the budget for short-curve accents
            public float curveRadiusScale; // tube-radius multiplier (0/1 = as captured)
            public bool includePointCloud; // write snap.pcPos as points (GLB) / cubes (USDZ)
            public float pointCloudSize;   // point size in metres (GLB point width hint / USDZ cube radius)
        }

        /// <summary>Writes the snapshot as .glb + .usdz (real-world metres,
        /// right-handed Y-up, base on y=0). The snapshot is not mutated —
        /// exporting twice gives identical files. Atomic: each file lands via
        /// a .tmp rename, and failed/cancelled writes leave no partials.</summary>
        public static bool ExportFiles(TSDFSnapshot snap, string glbPath, string usdzPath,
                                       string usdPythonPath, out ExportStats stats, out string error,
                                       WebFileOptions opt = default)
        {
            stats = default;
            error = null;
            var phase = System.Diagnostics.Stopwatch.StartNew();
            var ao = opt.ao;

            // Curves-only was asked for but this capture has none (hidden curves,
            // readback failure) — ship the mesh rather than an empty file.
            bool omitMesh = opt.omitMesh;
            if (omitMesh && snap.curveLines.Count == 0)
            {
                Debug.LogWarning("[TSDFSnapshotBuilder] curves-only export requested but the " +
                                 "snapshot has no curves — keeping the surface mesh instead.");
                omitMesh = false;
            }

            Vector3[] pos, col, nrm;
            int[] idx;
            if (!omitMesh)
            {
                // To export space (copy — the snapshot keeps its world-space mesh):
                // mirror X (LH -> RH), XZ centred on the origin, base on y=0.
                pos = new Vector3[snap.pos.Length];
                for (int i = 0; i < pos.Length; i++)
                    pos[i] = new Vector3(-(snap.pos[i].x - snap.center.x),
                                         snap.pos[i].y - snap.minY,
                                         snap.pos[i].z - snap.center.z);
                col = snap.col; // Array.Resize below copies before the tube append

                // Drop weld-collapsed slivers, then orient: positive signed volume
                // in the mirrored (right-handed) space means the winding is already
                // CCW-outward; negative means every triangle flips.
                var tri = snap.tri;
                var idxList = new List<int>(tri.Length);
                double signedVol = 0;
                for (int t = 0; t < tri.Length; t += 3)
                {
                    int i0 = tri[t], i1 = tri[t + 1], i2 = tri[t + 2];
                    if (i0 == i1 || i1 == i2 || i0 == i2) continue;
                    idxList.Add(i0); idxList.Add(i1); idxList.Add(i2);
                    signedVol += Vector3.Dot(pos[i0], Vector3.Cross(pos[i1], pos[i2])) / 6.0;
                }
                idx = idxList.ToArray();
                if (idx.Length == 0) { error = "no non-degenerate triangles after weld"; return false; }
                if (signedVol < 0)
                    for (int t = 0; t < idx.Length; t += 3)
                        (idx[t + 1], idx[t + 2]) = (idx[t + 2], idx[t + 1]);

                nrm = MeshOps.ComputeVertexNormals(pos, idx);
            }
            else
            {
                pos = new Vector3[0]; col = new Vector3[0]; nrm = new Vector3[0]; idx = new int[0];
            }
            int surfVertCount = pos.Length; // AO below darkens only these unless affectCurves

            // Curve tubes straight into export space, through the SAME
            // center/minY the mesh used (registration contract).
            if (snap.curveLines.Count > 0)
            {
                var lines = snap.curveLines;
                var lineCols = snap.curveColors;
                if (opt.curveBudgetBytes > 0)
                {
                    var selLines = new List<Vector3[]>();
                    var selCols = new List<Vector3>();
                    SelectCurvesForBudget(snap, opt.curveBudgetBytes, opt.shortCurveShare,
                                          selLines, selCols);
                    Debug.Log($"[TSDFSnapshotBuilder] curve budget {opt.curveBudgetBytes / 1048576.0:0.0} MB: " +
                              $"picked {selLines.Count}/{lines.Count} curves (longest-first, " +
                              $"{opt.shortCurveShare:P0} short-accent share)");
                    lines = selLines; lineCols = selCols;
                }
                var tp = new List<Vector3>(); var tn = new List<Vector3>();
                var tc = new List<Vector3>(); var ti = new List<int>();
                float radiusScale = opt.curveRadiusScale > 0f ? opt.curveRadiusScale : 1f;
                stats.curveCount = CurveTubeBuilder.AppendCurveTubes(lines, lineCols,
                    snap.curveBrightness, snap.curveRadius * radiusScale, snap.curveSides, snap.curveTolerance,
                    snap.center, snap.minY, tp, tn, tc, ti, snap.curveTipTaper);
                if (stats.curveCount > 0)
                {
                    int vOff = pos.Length, iOff = idx.Length;
                    Array.Resize(ref pos, vOff + tp.Count);
                    Array.Resize(ref nrm, vOff + tn.Count);
                    Array.Resize(ref col, vOff + tc.Count);
                    Array.Resize(ref idx, iOff + ti.Count);
                    tp.CopyTo(pos, vOff); tn.CopyTo(nrm, vOff); tc.CopyTo(col, vOff);
                    for (int i = 0; i < ti.Count; i++) idx[iOff + i] = ti[i] + vOff;
                    Debug.Log($"[TSDFSnapshotBuilder] curves: {stats.curveCount} tubes, " +
                              $"{tp.Count} verts / {ti.Count / 3} tris");
                }
            }
            stats.curveMs = phase.ElapsedMilliseconds;
            bool haveCloud = opt.includePointCloud && snap.pcPos != null && snap.pcPos.Length > 0;
            if (idx.Length == 0 && !haveCloud)
            { error = "nothing to export (no mesh, no usable curves, no point cloud)"; return false; }

            // AO bake into the vertex colours (linear-space multiply). Runs on
            // the FINAL export geometry so the tubes occlude the surface. The
            // snapshot's own colour array is never touched: clone when the tube
            // append didn't already copy it (Array.Resize above).
            int aoVertCount = ao.affectCurves ? pos.Length : surfVertCount;
            if (ao.strength > 0f && ao.samples > 0 && aoVertCount > 0)
            {
                phase.Restart();
                if (ReferenceEquals(col, snap.col)) col = (Vector3[])col.Clone();
                float[] open = MeshAO.Compute(pos, nrm, idx, ao.samples, Mathf.Max(0.01f, ao.maxDistance));
                MeshAO.ApplyToColors(col, open, ao.strength, aoVertCount);
                stats.aoMs = phase.ElapsedMilliseconds;
                Debug.Log($"[TSDFSnapshotBuilder] AO baked: {ao.samples} rays x {pos.Length} verts, " +
                          $"strength {ao.strength:0.00}, radius {ao.maxDistance:0.00} m " +
                          $"({(ao.affectCurves ? "mesh+tubes" : "mesh only; tubes occlude")}) " +
                          $"in {stats.aoMs} ms");
            }
            stats.finalTris = idx.Length / 3;
            stats.finalVerts = pos.Length;

            // Point cloud into the SAME export space (mirror X, recentre, base y=0)
            // as the mesh/curves. glTF renders it as a POINTS primitive; USDZ can't
            // (AR Quick Look ignores UsdGeomPoints), so each point becomes a tiny
            // octahedron merged into the mesh, coloured through the same atlas.
            Vector3[] pcExport = null, pcColExport = null;
            if (opt.includePointCloud && snap.pcPos != null && snap.pcPos.Length > 0)
            {
                pcExport = new Vector3[snap.pcPos.Length];
                for (int i = 0; i < pcExport.Length; i++)
                    pcExport[i] = new Vector3(-(snap.pcPos[i].x - snap.center.x),
                                              snap.pcPos[i].y - snap.minY,
                                              snap.pcPos[i].z - snap.center.z);
                pcColExport = snap.pcCol;
                stats.pointCount = pcExport.Length;
            }

            string glbTmp = glbPath + ".tmp", usdzTmp = usdzPath + ".tmp";
            try
            {
                Directory.CreateDirectory(Path.GetDirectoryName(glbPath));
                Directory.CreateDirectory(Path.GetDirectoryName(usdzPath));
                phase.Restart();
                stats.glbBytes = GlbWriter.Write(glbTmp, pos, nrm, col, idx, pcExport, pcColExport);
                stats.glbMs = phase.ElapsedMilliseconds;
                phase.Restart();
                // USDZ: append point octahedra to the triangle geometry. Every
                // point costs 8 triangles, and UsdzWriter throws once the total
                // exceeds the 4096px colour atlas (2048^2 = 4.19M triangles) —
                // which would abort BOTH files. So subsample the points for the
                // USDZ octahedra to fit the atlas; the GLB keeps the full cloud as
                // a POINTS primitive (no atlas, no cap).
                Vector3[] uPos = pos, uNrm = nrm, uCol = col; int[] uIdx = idx;
                if (pcExport != null && pcExport.Length > 0)
                {
                    const int atlasMaxTris = 2048 * 2048; // UsdzWriter's hard ceiling
                    int maxOctaPoints = Mathf.Max(0, (atlasMaxTris - idx.Length / 3) / 8);
                    Vector3[] octPos = pcExport, octCol = pcColExport;
                    if (pcExport.Length > maxOctaPoints)
                    {
                        int step = Mathf.CeilToInt(pcExport.Length / (float)Mathf.Max(1, maxOctaPoints));
                        var sp = new List<Vector3>(maxOctaPoints);
                        var sc = new List<Vector3>(maxOctaPoints);
                        for (int i = 0; i < pcExport.Length && sp.Count < maxOctaPoints; i += step)
                        { sp.Add(pcExport[i]); sc.Add(pcColExport[i]); }
                        octPos = sp.ToArray(); octCol = sc.ToArray();
                        Debug.LogWarning($"[TSDFSnapshotBuilder] USDZ point cloud capped " +
                                         $"{pcExport.Length} -> {octPos.Length} points to fit the colour " +
                                         "atlas (the GLB keeps the full cloud).");
                    }
                    if (octPos.Length > 0)
                        AppendPointOctahedra(ref uPos, ref uNrm, ref uCol, ref uIdx, octPos, octCol,
                                             opt.pointCloudSize > 0f ? opt.pointCloudSize * 0.5f : 0.006f);
                }
                var ur = UsdzWriter.Write(usdzTmp, uPos, uNrm, uCol, uIdx, usdPythonPath);
                stats.usdzMs = phase.ElapsedMilliseconds;
                stats.usdzBytes = ur.bytes;
                stats.usdzTexSize = ur.texSize;
                stats.usdzBinary = ur.binaryUsdc;
                stats.usdzConvertMs = ur.convertMs;
                // Both payloads written — promote them together. Refuse to
                // clobber existing finals (callers use timestamped paths, so a
                // collision is a caller bug — and deleting them up front would
                // leave a half-updated pair if the second rename failed). Roll
                // the .glb back if the .usdz promotion fails.
                if (File.Exists(glbPath)) throw new IOException($"destination already exists: {glbPath}");
                if (File.Exists(usdzPath)) throw new IOException($"destination already exists: {usdzPath}");
                File.Move(glbTmp, glbPath);
                try { File.Move(usdzTmp, usdzPath); }
                catch { TryDelete(glbPath); throw; }
                return true;
            }
            catch (Exception e)
            {
                TryDelete(glbTmp); TryDelete(usdzTmp);
                error = $"web export failed: {e.Message}";
                return false;
            }
        }

        // Pick which curve polylines ship, under a GLB byte budget: longest-first
        // until (1 - shortShare) of the budget is spent, then an even sample of
        // the shorter remainder fills the rest — "many long strokes, a few short
        // accents". Costs mirror CurveTubeBuilder exactly (dedup + Douglas-Peucker
        // at the snapshot's tolerance, then verts*28 + tris*12 GLB bytes), so the
        // written file lands at, not near, the budget. Deterministic — same
        // snapshot, same selection.
        private static void SelectCurvesForBudget(TSDFSnapshot snap, long budgetBytes,
                                                  float shortShare,
                                                  List<Vector3[]> outLines, List<Vector3> outCols)
        {
            int sides = Mathf.Clamp(snap.curveSides, 3, 12);
            int nc = snap.curveLines.Count;
            var cost = new long[nc];
            var length = new float[nc];
            var order = new List<int>(nc);
            var pts = new List<Vector3>(256);
            for (int i = 0; i < nc; i++)
            {
                var line = snap.curveLines[i];
                pts.Clear();
                for (int w = 0; w < line.Length; w++)
                    if (pts.Count == 0 || (line[w] - pts[pts.Count - 1]).sqrMagnitude > 1e-10f)
                        pts.Add(line[w]);
                if (snap.curveTolerance > 0f && pts.Count > 2)
                    MeshOps.SimplifyPolyline(pts, snap.curveTolerance);
                if (pts.Count < 2) continue; // degenerate — the builder would skip it too
                float len = 0f;
                for (int w = 1; w < pts.Count; w++) len += (pts[w] - pts[w - 1]).magnitude;
                length[i] = len;
                long verts = (long)pts.Count * sides + 2;
                long tris = (long)(pts.Count - 1) * sides * 2 + 2 * sides;
                cost[i] = verts * 28 + tris * 12; // f32 pos+nrm (24 B) + rgba8 (4 B) + u32 idx
                order.Add(i);
            }
            order.Sort((a, b) => length[b].CompareTo(length[a]));

            var picked = new bool[nc];
            long used = 0;
            long longBudget = (long)(budgetBytes * (1.0 - Mathf.Clamp01(shortShare)));
            int cut = 0; // boundary: first curve the long pass could not afford
            for (; cut < order.Count; cut++)
            {
                int i = order[cut];
                if (used + cost[i] > longBudget) break;
                picked[i] = true;
                used += cost[i];
            }
            if (cut < order.Count && used < budgetBytes)
            {
                long avg = 0;
                for (int k = cut; k < order.Count; k++) avg += cost[order[k]];
                avg /= order.Count - cut;
                long want = Math.Max(1, (budgetBytes - used) / Math.Max(1, avg));
                int step = Mathf.Max(1, (int)((order.Count - cut) / want));
                for (int k = cut; k < order.Count && used < budgetBytes; k += step)
                {
                    int i = order[k];
                    if (used + cost[i] > budgetBytes) continue;
                    picked[i] = true;
                    used += cost[i];
                }
            }
            for (int i = 0; i < nc; i++)
                if (picked[i]) { outLines.Add(snap.curveLines[i]); outCols.Add(snap.curveColors[i]); }
        }

        // Turn each point into a small 6-vertex / 8-triangle octahedron so it
        // survives as solid geometry in AR Quick Look (which drops true points).
        // Appended after the mesh/curves; the atlas colours each face from the
        // point's colour. Grows the four arrays in place.
        private static void AppendPointOctahedra(ref Vector3[] pos, ref Vector3[] nrm, ref Vector3[] col,
                                                 ref int[] idx, Vector3[] pcPos, Vector3[] pcCol, float r)
        {
            int n = pcPos.Length;
            int vOff = pos.Length, iOff = idx.Length;
            System.Array.Resize(ref pos, vOff + n * 6);
            System.Array.Resize(ref nrm, vOff + n * 6);
            System.Array.Resize(ref col, vOff + n * 6);
            System.Array.Resize(ref idx, iOff + n * 24);
            var dirs = new[] { Vector3.right, Vector3.left, Vector3.up, Vector3.down, Vector3.forward, Vector3.back };
            int[] faces = { 0, 2, 4, 2, 1, 4, 1, 3, 4, 3, 0, 4, 2, 0, 5, 1, 2, 5, 3, 1, 5, 0, 3, 5 };
            for (int p = 0; p < n; p++)
            {
                int vb = vOff + p * 6;
                Vector3 c = pcCol != null && p < pcCol.Length ? pcCol[p] : Vector3.one;
                for (int k = 0; k < 6; k++)
                {
                    pos[vb + k] = pcPos[p] + dirs[k] * r;
                    nrm[vb + k] = dirs[k];
                    col[vb + k] = c;
                }
                int ib = iOff + p * 24;
                for (int f = 0; f < 24; f++) idx[ib + f] = vb + faces[f];
            }
        }

        private static void TryDelete(string path)
        {
            try { if (File.Exists(path)) File.Delete(path); }
            catch { /* partial cleanup is best-effort */ }
        }

        // ---------------- marching cubes ----------------

        private static bool EnsureMcShader()
        {
            if (s_mcShader != null && s_mcKernel >= 0) return true;
            if (!TSDFComputeUtil.TryLoad(ref s_mcShader, "TSDFMarchingCubes", "TSDFSnapshotBuilder", null))
                return false;
            s_mcKernel = s_mcShader.FindKernel("MarchCubes");
            return s_mcKernel >= 0;
        }

        // Full-grid Marching Cubes in Z slabs with CPU readback (moved verbatim
        // from TSDFPrintExporter). Returns the per-slab triangle soup
        // (Tri = 18 floats each), or null with error set.
        private static List<(float[] data, int tris)> RunMarchingCubes(
            TSDFVolume volume, int triangleBudgetPerSlab, out long totalTris, out string error)
        {
            totalTris = 0;
            error = null;
            var dim = volume.Dim;
            // Match what the user sees: reuse the view's iso/weight gates if present.
            var view = UnityEngine.Object.FindFirstObjectByType<TSDFView>();
            float iso = view != null ? view.meshIsoLevel : 0f;
            float minWeight = view != null ? view.meshMinWeight : 0.5f;

            int triCap = Mathf.Max(100_000, triangleBudgetPerSlab);
            var triBuf = new ComputeBuffer(triCap, 72, ComputeBufferType.Append); // Tri
            var countBuf = new ComputeBuffer(1, sizeof(uint), ComputeBufferType.Raw);
            var slabs = new List<(float[] data, int tris)>();
            try
            {
                s_mcShader.SetInts("_Dim", dim.x, dim.y, dim.z);
                s_mcShader.SetMatrix("_WorldFromVoxel", volume.WorldFromVoxel);
                s_mcShader.SetFloat("_IsoLevel", iso);
                s_mcShader.SetFloat("_MinWeight", minWeight);
                s_mcShader.SetBuffer(s_mcKernel, "_Voxels", volume.FrontBuffer);
                s_mcShader.SetBuffer(s_mcKernel, "_Colors", volume.FrontColorBuffer);
                s_mcShader.SetBuffer(s_mcKernel, "_Triangles", triBuf);

                int gx = Mathf.Max(1, Mathf.CeilToInt((dim.x - 1) / 4f));
                int gy = Mathf.Max(1, Mathf.CeilToInt((dim.y - 1) / 4f));
                int cellsZ = Mathf.Max(1, dim.z - 1);
                var countHost = new uint[1];

                int zBase = 0, thickness = 32; // multiple of 4: dispatch covers exactly gz*4 cells
                while (zBase < cellsZ)
                {
                    int gz = Mathf.Max(1, Mathf.CeilToInt(Mathf.Min(thickness, cellsZ - zBase) / 4f));
                    triBuf.SetCounterValue(0);
                    s_mcShader.SetInt("_CellZBase", zBase);
                    s_mcShader.Dispatch(s_mcKernel, gx, gy, gz);
                    ComputeBuffer.CopyCount(triBuf, countBuf, 0);
                    countBuf.GetData(countHost);
                    int n = (int)countHost[0];

                    if (n >= triCap)
                    {
                        if (thickness <= 4)
                        {
                            error = $"slab at z={zBase} overflows {triCap} triangles even at minimum thickness — " +
                                    "raise triangleBudgetPerSlab";
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
                        Debug.Log($"[TSDFSnapshotBuilder] MC slab {zBase}/{cellsZ} — {totalTris} tris so far");
                }
                s_mcShader.SetInt("_CellZBase", 0); // hygiene: shared shader asset state

                if (totalTris == 0) { error = "Marching Cubes produced 0 triangles — empty volume?"; return null; }
                Debug.Log($"[TSDFSnapshotBuilder] MC done: {totalTris} tris — welding...");
                return slabs;
            }
            finally
            {
                triBuf.Release(); countBuf.Release();
            }
        }
    }
}
