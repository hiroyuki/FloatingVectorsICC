// Bakes a body-tracking motion TRAIL into a TSDFVolume as authored capsule
// geometry, then lets the existing Marching Cubes pass (TSDFView) re-mesh it —
// the "feed the BT trail back into the SDF and re-mesh" experiment.
//
// Source data: BodyTrackingPlayback.Trajectories — per-(body,joint) time-ordered
// position samples. IMPORTANT: those samples are in k4a-CAMERA-LOCAL space (mm->m
// + Y flip, no world transform; see BodyTrackingPlayback.Process). MotionLineRenderer
// only looks aligned with the point cloud because it parents its line meshes under
// its own transform, which carries the camera->world placement. So this baker takes
// the SAME source-space transform and applies TransformPoint to every sample before
// comparing to the volume's world-space voxels. Leave sourceSpace empty to default
// to a MotionLineRenderer in the scene (the proven-aligned frame).
//
// Each pair of consecutive samples becomes one analytic capsule segment; the
// kernel (TSDFTrailBake.compute) min-unions sdf = dist(p, segment) - radius into
// the volume write buffer in TDR-safe batches, then Publish() triggers re-extraction.
//
// Modes:
//   clearVolumeFirst = true  -> trail-only sculpture (ClearWrite, then bake).
//   clearVolumeFirst = false -> fuse: min-union the trail into whatever the volume
//                               already holds (e.g. an accumulated body TSDF).
//
// Multi-camera caveat: Process merges every device track by (bodyId, jointId), so
// mixing cameras puts conflicting camera-local coords into one trajectory. Set
// BodyTrackingPlayback.deviceSerialFilter to ONE device and point sourceSpace at
// that device's frame for a clean trail.

using System.Collections.Generic;
using BodyTracking;
using UnityEngine;

namespace TSDF
{
    [DisallowMultipleComponent]
    public class TSDFTrailBaker : MonoBehaviour
    {
        /// <summary>Where the per-joint ribbon centerlines come from.</summary>
        public enum TrailSource
        {
            /// <summary>Whole-recording per-joint trajectories from BodyTrackingPlayback (all frames).</summary>
            OfflineTrajectories,
            /// <summary>The short windowed trail SkeletonMerger currently draws (~trailDuration s, smoothed).</summary>
            LiveWindowedTrail,
        }

        [Header("Source")]
        [Tooltip("OfflineTrajectories: whole-recording per-joint paths (BodyTrackingPlayback). " +
                 "LiveWindowedTrail: the short, smoothed trail SkeletonMerger draws right now " +
                 "for the joints below — the on-screen ribbon, baked as it follows the motion.")]
        public TrailSource source = TrailSource.LiveWindowedTrail;

        [Header("Wiring")]
        [Tooltip("Target volume. Leave empty to auto-resolve the first TSDFVolume in the scene.")]
        public TSDFVolume volume;

        [Tooltip("OfflineTrajectories source. Leave empty to auto-resolve the first BodyTrackingPlayback.")]
        public BodyTrackingPlayback playback;

        [Tooltip("LiveWindowedTrail source. Leave empty to auto-resolve the first SkeletonMerger.")]
        public SkeletonMerger skeleton;

        [Tooltip("Joints baked as ribbons in LiveWindowedTrail mode. Default: wrists, hands, feet, head.")]
        public k4abt_joint_id_t[] ribbonJoints =
        {
            k4abt_joint_id_t.K4ABT_JOINT_WRIST_LEFT,
            k4abt_joint_id_t.K4ABT_JOINT_HAND_LEFT,
            k4abt_joint_id_t.K4ABT_JOINT_WRIST_RIGHT,
            k4abt_joint_id_t.K4ABT_JOINT_HAND_RIGHT,
            k4abt_joint_id_t.K4ABT_JOINT_FOOT_LEFT,
            k4abt_joint_id_t.K4ABT_JOINT_FOOT_RIGHT,
            k4abt_joint_id_t.K4ABT_JOINT_HEAD,
        };

        [Tooltip("OfflineTrajectories only: transform mapping camera-local trajectory samples " +
                 "into world. Leave empty to use the first MotionLineRenderer's transform.")]
        public Transform sourceSpace;

        [Header("Capsule")]
        [Min(0f)]
        [Tooltip("Capsule radius in metres. Keep >= ~1 voxel (voxelSize) or the tube " +
                 "aliases / disappears at the volume resolution.")]
        public float radius = 0.05f;

        [Tooltip("Drop trajectory samples below this confidence before linking segments. " +
                 "MEDIUM is the SDK ceiling for real observations; LOW lets predicted joints in.")]
        public k4abt_joint_confidence_level_t minConfidence =
            k4abt_joint_confidence_level_t.K4ABT_JOINT_CONFIDENCE_MEDIUM;

        [Min(1)]
        [Tooltip("Use every Nth trajectory sample as a capsule endpoint. >1 thins the " +
                 "segment count (cheaper bake) at the cost of a coarser trail.")]
        public int sampleStride = 1;

        [Header("Colour")]
        public bool perJointHue = true;
        [Tooltip("Trail colour when perJointHue is off (also tints the per-joint hues).")]
        public Color trailColor = Color.white;

        [Header("Compose")]
        [Tooltip("ON: clear the volume first so the mesh is the trail ALONE. " +
                 "OFF: min-union the trail into the volume's current contents (fuse with the body).")]
        public bool clearVolumeFirst = true;

        [Header("Performance")]
        [Min(1)]
        [Tooltip("Hard cap on capsule segments. Excess segments past this are dropped (logged).")]
        public int maxSegments = 20000;

        [Min(1)]
        [Tooltip("Segments processed per Dispatch. Smaller = more dispatches but each is " +
                 "shorter, avoiding the ~2s GPU TDR watchdog on dense trails.")]
        public int batchSize = 512;

        public string LastStatus { get; private set; } = "";

        // GPU-side capsule, must match struct Seg in TSDFTrailBake.compute (48 bytes).
        private struct TrailSeg
        {
            public Vector3 a;
            public Vector3 b;
            public float ra;
            public float rb;
            public Vector3 color;
            public float pad;
        }

        private ComputeShader _shader;
        private int _kernel = -1;

        [ContextMenu("Bake BT trail into volume")]
        public void BakeTrailIntoVolume()
        {
            if (volume == null) volume = FindFirstObjectByType<TSDFVolume>();
            if (volume == null) { Fail("no TSDFVolume found"); return; }

            var dim = volume.Dim;
            if (dim.x <= 0 || dim.y <= 0 || dim.z <= 0) { Fail("volume not initialised (Dim is 0)"); return; }

            List<TrailSeg> segs;
            int trajCount; bool capped;
            if (source == TrailSource.LiveWindowedTrail)
            {
                if (skeleton == null) skeleton = FindFirstObjectByType<SkeletonMerger>();
                if (skeleton == null) { Fail("no SkeletonMerger found (LiveWindowedTrail source)"); return; }
                segs = BuildWindowedSegments(out trajCount, out capped);
                if (segs.Count == 0) { Fail("no windowed trail segments (enable showBones so trails " +
                                            "accumulate, and let the body move a bit / check ribbonJoints)"); return; }
            }
            else
            {
                if (playback == null) playback = FindFirstObjectByType<BodyTrackingPlayback>();
                if (playback == null) { Fail("no BodyTrackingPlayback found (OfflineTrajectories source)"); return; }
                Transform space = sourceSpace;
                if (space == null)
                {
                    var mlr = FindFirstObjectByType<MotionLineRenderer>();
                    if (mlr != null) space = mlr.transform;
                }
                segs = BuildSegments(space, out trajCount, out capped);
                if (segs.Count == 0) { Fail("no segments built (Read+Process the recording first, " +
                                            "and check deviceSerialFilter / confidence)"); return; }
            }

            if (!EnsureShader()) { Fail("TSDFTrailBake.compute not found in Resources"); return; }

            var segBuf = new ComputeBuffer(segs.Count, sizeof(float) * 12);
            try
            {
                segBuf.SetData(segs);

                if (clearVolumeFirst) volume.ClearWrite();

                int total = dim.x * dim.y * dim.z;
                DispatchGrid(total, out int gx, out int gy);

                _shader.SetInts("_Dim", dim.x, dim.y, dim.z);
                _shader.SetInt("_DispatchWidth", gx * 64);
                _shader.SetFloat("_Tau", volume.Tau);
                _shader.SetMatrix("_WorldFromVoxel", volume.WorldFromVoxel);
                _shader.SetBuffer(_kernel, "_Segs", segBuf);
                _shader.SetBuffer(_kernel, "_VoxelsOut", volume.WriteBuffer);
                _shader.SetBuffer(_kernel, "_ColorsOut", volume.WriteColorBuffer);

                int batches = 0;
                for (int off = 0; off < segs.Count; off += batchSize)
                {
                    int count = Mathf.Min(batchSize, segs.Count - off);
                    _shader.SetInt("_SegOffset", off);
                    _shader.SetInt("_SegCount", count);
                    _shader.Dispatch(_kernel, Mathf.Max(1, gx), Mathf.Max(1, gy), 1);
                    batches++;
                }

                volume.Publish();

                LastStatus = $"baked {segs.Count} segs (from {trajCount} trajectories) " +
                             $"in {batches} batch(es){(capped ? $", CAPPED at {maxSegments}" : "")}" +
                             $"; mode={(clearVolumeFirst ? "trail-only" : "fuse")}, r={radius:0.000}m";
                Debug.Log($"[TSDFTrailBaker] {LastStatus}", this);
            }
            finally
            {
                segBuf.Release();
            }
        }

        private List<TrailSeg> BuildSegments(Transform space, out int trajCount, out bool capped)
        {
            var list = new List<TrailSeg>();
            capped = false;
            var trajectories = playback.Trajectories;
            trajCount = trajectories != null ? trajectories.Count : 0;
            if (trajectories == null) return list;

            int stride = Mathf.Max(1, sampleStride);
            foreach (var traj in trajectories)
            {
                var samples = traj.Samples;
                Color baseCol = perJointHue
                    ? HueFor((int)traj.JointId, K4ABTConsts.K4ABT_JOINT_COUNT) * trailColor
                    : trailColor;
                Vector3 colVec = new Vector3(baseCol.r, baseCol.g, baseCol.b);

                int prev = -1;
                for (int i = 0; i < samples.Count; i += stride)
                {
                    if (samples[i].Confidence < minConfidence) { prev = -1; continue; }
                    if (prev >= 0)
                    {
                        Vector3 a = ToWorld(space, samples[prev].Position);
                        Vector3 b = ToWorld(space, samples[i].Position);
                        if ((b - a).sqrMagnitude > 1e-10f)   // skip zero-length (still joints)
                        {
                            list.Add(new TrailSeg { a = a, b = b, ra = radius, rb = radius, color = colVec });
                            if (list.Count >= maxSegments) { capped = true; return list; }
                        }
                    }
                    prev = i;
                }
            }
            return list;
        }

        // Reused scratch so a per-frame bake doesn't allocate.
        private readonly List<Vector3> _trailScratch = new List<Vector3>();

        // Build capsule segments from the SkeletonMerger's current windowed trail for each
        // selected joint (the same ribbon centerline it draws). trajCount = joints that
        // contributed at least one segment.
        private List<TrailSeg> BuildWindowedSegments(out int trajCount, out bool capped)
        {
            var list = new List<TrailSeg>();
            capped = false;
            trajCount = 0;
            if (ribbonJoints == null) return list;

            int stride = Mathf.Max(1, sampleStride);
            foreach (var joint in ribbonJoints)
            {
                _trailScratch.Clear();
                int n = skeleton.CopyTrailWorldPoints(joint, _trailScratch);
                if (n < 2) continue;

                Color baseCol = perJointHue
                    ? HueFor((int)joint, K4ABTConsts.K4ABT_JOINT_COUNT) * trailColor
                    : trailColor;
                Vector3 colVec = new Vector3(baseCol.r, baseCol.g, baseCol.b);

                bool contributed = false;
                int prev = -1;
                for (int i = 0; i < n; i += stride)
                {
                    if (prev >= 0)
                    {
                        Vector3 a = _trailScratch[prev];
                        Vector3 b = _trailScratch[i];
                        if ((b - a).sqrMagnitude > 1e-10f)
                        {
                            list.Add(new TrailSeg { a = a, b = b, ra = radius, rb = radius, color = colVec });
                            contributed = true;
                            if (list.Count >= maxSegments) { capped = true; if (contributed) trajCount++; return list; }
                        }
                    }
                    prev = i;
                }
                if (contributed) trajCount++;
            }
            return list;
        }

        private static Vector3 ToWorld(Transform space, Vector3 local)
            => space != null ? space.TransformPoint(local) : local;

        private static Color HueFor(int index, int total)
        {
            float h = total > 0 ? Mathf.Repeat(index * 0.61803398875f, 1f) : 0f;
            return Color.HSVToRGB(h, 0.85f, 1f);
        }

        private bool EnsureShader()
        {
            if (_shader != null && _kernel >= 0) return true;
            _shader = Resources.Load<ComputeShader>("TSDFTrailBake");
            if (_shader == null) return false;
            _kernel = _shader.FindKernel("BakeTrail");
            return _kernel >= 0;
        }

        // Mirrors TSDFVolume.DispatchGrid: linearise total threads into a 2D grid
        // (gx, gy) of 64-thread groups, capped at the 65535-per-axis D3D limit.
        private static void DispatchGrid(int total, out int gx, out int gy)
        {
            int groups = Mathf.CeilToInt(total / 64f);
            gx = Mathf.Max(1, Mathf.Min(groups, 65535));
            gy = Mathf.Max(1, Mathf.CeilToInt(groups / (float)gx));
        }

        private void Fail(string why)
        {
            LastStatus = "FAILED: " + why;
            Debug.LogWarning("[TSDFTrailBaker] " + LastStatus, this);
        }
    }
}
