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
        }

        public struct ExportStats
        {
            public int finalTris, finalVerts; // mesh + tubes as written
            public int curveCount;            // tubes emitted
            public long curveMs, glbMs, usdzMs;
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

        /// <summary>Writes the snapshot as .glb + .usdz (real-world metres,
        /// right-handed Y-up, base on y=0). The snapshot is not mutated —
        /// exporting twice gives identical files. Atomic: each file lands via
        /// a .tmp rename, and failed/cancelled writes leave no partials.</summary>
        public static bool ExportFiles(TSDFSnapshot snap, string glbPath, string usdzPath,
                                       string usdPythonPath, out ExportStats stats, out string error)
        {
            stats = default;
            error = null;
            var phase = System.Diagnostics.Stopwatch.StartNew();

            // To export space (copy — the snapshot keeps its world-space mesh):
            // mirror X (LH -> RH), XZ centred on the origin, base on y=0.
            var pos = new Vector3[snap.pos.Length];
            for (int i = 0; i < pos.Length; i++)
                pos[i] = new Vector3(-(snap.pos[i].x - snap.center.x),
                                     snap.pos[i].y - snap.minY,
                                     snap.pos[i].z - snap.center.z);
            var col = snap.col; // Array.Resize below copies before the tube append

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
            var idx = idxList.ToArray();
            if (idx.Length == 0) { error = "no non-degenerate triangles after weld"; return false; }
            if (signedVol < 0)
                for (int t = 0; t < idx.Length; t += 3)
                    (idx[t + 1], idx[t + 2]) = (idx[t + 2], idx[t + 1]);

            var nrm = MeshOps.ComputeVertexNormals(pos, idx);

            // Curve tubes straight into export space, through the SAME
            // center/minY the mesh used (registration contract).
            if (snap.curveLines.Count > 0)
            {
                var tp = new List<Vector3>(); var tn = new List<Vector3>();
                var tc = new List<Vector3>(); var ti = new List<int>();
                stats.curveCount = CurveTubeBuilder.AppendCurveTubes(snap.curveLines, snap.curveColors,
                    snap.curveBrightness, snap.curveRadius, snap.curveSides, snap.curveTolerance,
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
            stats.finalTris = idx.Length / 3;
            stats.finalVerts = pos.Length;

            string glbTmp = glbPath + ".tmp", usdzTmp = usdzPath + ".tmp";
            try
            {
                Directory.CreateDirectory(Path.GetDirectoryName(glbPath));
                Directory.CreateDirectory(Path.GetDirectoryName(usdzPath));
                phase.Restart();
                stats.glbBytes = GlbWriter.Write(glbTmp, pos, nrm, col, idx);
                stats.glbMs = phase.ElapsedMilliseconds;
                phase.Restart();
                var ur = UsdzWriter.Write(usdzTmp, pos, nrm, col, idx, usdPythonPath);
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
