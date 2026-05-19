using System;
using System.Collections.Generic;

namespace Calibration
{
    // Multi-camera extrinsic solver that does NOT require all cameras to see the
    // board in a single sample — necessary for ring setups around an occluder
    // (a person, etc.) where any single board pose is hidden from 2+ cameras.
    //
    // Input model:
    //   - cameraCount cameras, indexed [0..cameraCount).
    //   - Many capture samples; each sample may contribute zero or more per-camera
    //     cam_tr_marker observations (one per camera that detected the board THIS
    //     sample). All observations within a single sample share the same board pose.
    //   - From any sample where two cameras both detected, we derive an edge
    //     camA_tr_camB = camA_tr_marker * inverse(camB_tr_marker).
    //
    // Solver:
    //   - Pin camera 0 to world origin (identity). World axes = cam0 axes.
    //   - BFS the camera graph from cam0, propagating
    //     global_tr_camB = global_tr_camA * camA_tr_camB.
    //   - For pairs that have multiple supporting samples, use the most recent
    //     observation (highest sample index). Quaternion / SE(3) averaging is a
    //     future improvement; the user can iterate by capturing duplicates.
    //
    // Returns reachability per camera. Cameras not connected to cam0 cannot be
    // solved and the caller should prompt the user to capture additional samples
    // bridging them.
    public static class PairwiseCalibrationMath
    {
        // A single per-camera observation of the board within one capture sample.
        public struct Observation
        {
            public int SampleIndex;     // Which capture this came from (used to dedupe per-pair).
            public int CameraIndex;     // 0..cameraCount-1
            public Rigid3d CamTrMarker; // OpenCV camera frame, meters.
        }

        public struct SolveResult
        {
            // global_tr_camera[i] = transform that takes points in camera i frame
            // to the world frame. Identity for cam0. Garbage for unreachable cameras
            // — check Reachable[i] before consuming.
            public Rigid3d[] GlobalTrCamera;
            // Reachable[i] = true iff there is a chain of pair observations from
            // cam0 to cam i. Unreachable cameras keep Identity in GlobalTrCamera.
            public bool[] Reachable;
            // For diagnostics: per-camera chain of camera indices walked from cam0
            // (length 1 for cam0 itself, > 1 for descendants). Empty for unreachable.
            public List<int>[] PathFromCam0;
            // For diagnostics: per directed edge (a→b) the sample index whose
            // observations were used to derive camA_tr_camB.
            public Dictionary<(int from, int to), int> EdgeSampleIndex;
        }

        public static SolveResult Solve(int cameraCount, IReadOnlyList<Observation> observations)
        {
            if (cameraCount <= 0) throw new ArgumentException("cameraCount must be > 0", nameof(cameraCount));
            if (observations == null) throw new ArgumentNullException(nameof(observations));

            // Group observations by sample so we can extract per-sample camera pairs.
            var bySample = new Dictionary<int, List<Observation>>();
            foreach (var o in observations)
            {
                if (o.CameraIndex < 0 || o.CameraIndex >= cameraCount) continue;
                if (!bySample.TryGetValue(o.SampleIndex, out var list))
                {
                    list = new List<Observation>();
                    bySample[o.SampleIndex] = list;
                }
                list.Add(o);
            }

            // For each pair (a,b) build an undirected edge keyed by sorted pair, value =
            // (camA_tr_camB, source sample index). Later samples replace earlier ones.
            // We store directed lookups for both a→b and b→a for convenient BFS.
            var directedEdge = new Dictionary<(int, int), (Rigid3d Transform, int SampleIndex)>();
            // Sort samples ascending so newer (higher index) overwrites older.
            var sampleKeys = new List<int>(bySample.Keys);
            sampleKeys.Sort();
            foreach (var sampleIndex in sampleKeys)
            {
                var obs = bySample[sampleIndex];
                for (int i = 0; i < obs.Count; i++)
                {
                    for (int j = i + 1; j < obs.Count; j++)
                    {
                        var a = obs[i];
                        var b = obs[j];
                        // camA_tr_camB = camA_tr_marker * marker_tr_camB
                        //              = camA_tr_marker * inverse(camB_tr_marker)
                        var aTrB = Rigid3d.Compose(a.CamTrMarker, b.CamTrMarker.Inverse());
                        directedEdge[(a.CameraIndex, b.CameraIndex)] = (aTrB, sampleIndex);
                        directedEdge[(b.CameraIndex, a.CameraIndex)] = (aTrB.Inverse(), sampleIndex);
                    }
                }
            }

            // Build adjacency list for BFS.
            var adj = new List<int>[cameraCount];
            for (int i = 0; i < cameraCount; i++) adj[i] = new List<int>();
            foreach (var key in directedEdge.Keys)
                adj[key.Item1].Add(key.Item2);

            var global = new Rigid3d[cameraCount];
            var reachable = new bool[cameraCount];
            var path = new List<int>[cameraCount];
            for (int i = 0; i < cameraCount; i++) { global[i] = Rigid3d.Identity; path[i] = new List<int>(); }

            // BFS from cam0.
            global[0] = Rigid3d.Identity;
            reachable[0] = true;
            path[0].Add(0);
            var queue = new Queue<int>();
            queue.Enqueue(0);
            while (queue.Count > 0)
            {
                int cur = queue.Dequeue();
                foreach (var next in adj[cur])
                {
                    if (reachable[next]) continue;
                    var (curTrNext, _) = directedEdge[(cur, next)];
                    // global_tr_next = global_tr_cur * cur_tr_next
                    global[next] = Rigid3d.Compose(global[cur], curTrNext);
                    reachable[next] = true;
                    path[next].AddRange(path[cur]);
                    path[next].Add(next);
                    queue.Enqueue(next);
                }
            }

            // Note: an earlier version of this solver auto-recentered the
            // world origin onto the camera centroid for a more aesthetic
            // scene-view layout. That broke "WYSIWYG" — Live mode (which reads
            // the same yaml) saw the cameras with cam0 pinned at origin while
            // any post-processed yaml moved them, so Live and Playback drifted
            // apart. Convention is now plain: cam0 is the world origin.
            // RecenterOnCameraCentroid is kept below as an opt-in utility but
            // is not invoked automatically.

            return new SolveResult
            {
                GlobalTrCamera = global,
                Reachable = reachable,
                PathFromCam0 = path,
                EdgeSampleIndex = ToDirectedSampleMap(directedEdge),
            };
        }

        private static void RecenterOnCameraCentroid(Rigid3d[] global, bool[] reachable)
        {
            double cx = 0, cy = 0, cz = 0;
            int n = 0;
            for (int i = 0; i < global.Length; i++)
            {
                if (!reachable[i]) continue;
                cx += global[i].Translation[0];
                cy += global[i].Translation[1];
                cz += global[i].Translation[2];
                n++;
            }
            if (n == 0) return;
            cx /= n; cy /= n; cz /= n;
            for (int i = 0; i < global.Length; i++)
            {
                if (!reachable[i]) continue;
                var t = global[i].Translation;
                global[i] = new Rigid3d(global[i].Rotation,
                    new[] { t[0] - cx, t[1] - cy, t[2] - cz });
            }
        }

        private static Dictionary<(int from, int to), int> ToDirectedSampleMap(
            Dictionary<(int, int), (Rigid3d Transform, int SampleIndex)> directed)
        {
            var m = new Dictionary<(int from, int to), int>(directed.Count);
            foreach (var kv in directed) m[(kv.Key.Item1, kv.Key.Item2)] = kv.Value.SampleIndex;
            return m;
        }
    }
}
