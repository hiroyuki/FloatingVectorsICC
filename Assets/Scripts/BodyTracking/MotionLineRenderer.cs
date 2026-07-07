// Renders the per-joint trajectories produced by BodyTrackingPlayback as
// connected line segments — one Mesh per (body id, joint id), all visible at
// once after Read+Process. Default coloring is per-joint hue with an optional
// time gradient (oldest → newest); the issue spec leaves this as a creative
// choice and per-joint coloring matches the "lines drawing through space"
// motion-study aesthetic best when there's one person in the recording.

using System.Collections.Generic;
using UnityEngine;

namespace BodyTracking
{
    [RequireComponent(typeof(BodyTrackingPlayback))]
    public class MotionLineRenderer : MonoBehaviour
    {
        public enum ColorMode { PerJoint, PerBody, FlatColor, AccelHeatmap }

        [Header("Display")]
        [Tooltip("Hide all motion lines without destroying the meshes.")]
        public bool show = true;

        [Tooltip("Color scheme. PerJoint matches the K4ABT joint id to a stable hue " +
                 "(useful when one person is recorded and you want each limb traceable). " +
                 "PerBody is one hue per body id. FlatColor uses the color field below.")]
        public ColorMode colorMode = ColorMode.PerJoint;

        [Tooltip("Color used in FlatColor mode. Also acts as a tint multiplier on " +
                 "PerJoint / PerBody.")]
        public Color flatColor = Color.white;

        [Tooltip("If true, fade vertex alpha from 0 (oldest sample) to 1 (newest) so " +
                 "the line reads as a directional trail.")]
        public bool timeGradient = true;

        [Tooltip("Drop trajectory samples whose confidence is below this level. " +
                 "LOW lets predicted/occluded joints into the trail; MEDIUM is the " +
                 "current SDK ceiling for actual observations.")]
        public k4abt_joint_confidence_level_t minConfidence =
            k4abt_joint_confidence_level_t.K4ABT_JOINT_CONFIDENCE_LOW;

        [Tooltip("Skip joints whose trajectories have fewer than this many valid samples.")]
        [Min(2)] public int minSamples = 4;

        [Range(0f, 0.99f)]
        [Tooltip("Lerp-based low-pass on the per-joint trajectory. 0 = raw samples, " +
                 "~0.9 = heavy smoothing. Forward-only EMA used only for offline playback " +
                 "trail visualization (live skeleton uses the 1€ filter in BodyVisual).")]
        public float smoothing = 0f;

        [Header("Accel heatmap (ColorMode.AccelHeatmap)")]
        [Tooltip("|a| value that maps to the cold (base) color. Anything below stays at base. " +
                 "0 means start coloring from any motion at all.")]
        public float accelMin = 0f;

        [Tooltip("|a| value that maps to the hot color. Sample p95 from the Rebuild log to " +
                 "pick this — defaulted to ~56 m/s^2 from the reference recording. " +
                 "Ignored when autoAccelMax is on.")]
        public float accelMax = 56f;

        [Tooltip("If true, accelMax is replaced by the p95 of |a| computed from the " +
                 "current trajectories every Rebuild. Avoids having to hand-tune accelMax " +
                 "for each recording.")]
        public bool autoAccelMax = false;

        [Tooltip("Hot end of the heatmap. The cold end is `flatColor`.")]
        public Color accelHotColor = Color.red;

        // --- runtime state ---

        private BodyTrackingPlayback _src;
        private readonly List<GameObject> _spawned = new List<GameObject>();
        private Material _sharedMaterial;

        private void Awake()
        {
            _src = GetComponent<BodyTrackingPlayback>();
        }

        private void OnEnable()
        {
            if (_src == null) _src = GetComponent<BodyTrackingPlayback>();
            if (_src != null) _src.OnTrajectoriesReady += Rebuild;
        }

        private void OnDisable()
        {
            if (_src != null) _src.OnTrajectoriesReady -= Rebuild;
            ClearSpawned();
        }

        private void OnValidate()
        {
            // Live-tweak visibility from the Inspector without re-running the BT pass.
            foreach (var go in _spawned) if (go != null) go.SetActive(show);
        }

        [ContextMenu("Rebuild from current trajectories")]
        public void Rebuild()
        {
            ClearSpawned();
            if (_src == null) return;

            var trajectories = _src.Trajectories;
            if (trajectories == null || trajectories.Count == 0) return;

            EnsureSharedMaterial();

            // Pass 1: per-trajectory raw positions / times / acceleration. Defer mesh
            // creation until we know the effective accelMax (so autoAccelMax can replace
            // it with the p95 of |a| computed across every trajectory).
            var accelMags = new List<float>();
            var prepared = new List<(BodyTrackingPlayback.JointTrajectory traj,
                Vector3[] verts, float[] trajAccel, int validCount)>();

            foreach (var traj in trajectories)
            {
                int validCount = 0;
                for (int i = 0; i < traj.Samples.Count; i++)
                    if (traj.Samples[i].Confidence >= minConfidence) validCount++;
                if (validCount < minSamples) continue;

                var verts = new Vector3[validCount];
                var times = new double[validCount];
                int v = 0;
                for (int i = 0; i < traj.Samples.Count; i++)
                {
                    var s = traj.Samples[i];
                    if (s.Confidence < minConfidence) continue;
                    verts[v] = s.Position;
                    times[v] = s.TimeSec;
                    v++;
                }

                var trajAccel = new float[validCount];
                for (int i = 1; i < validCount - 1; i++)
                {
                    float dt1 = (float)(times[i] - times[i - 1]);
                    float dt2 = (float)(times[i + 1] - times[i]);
                    if (dt1 <= 0f || dt2 <= 0f) continue;
                    Vector3 vel1 = (verts[i] - verts[i - 1]) / dt1;
                    Vector3 vel2 = (verts[i + 1] - verts[i]) / dt2;
                    Vector3 a = (vel2 - vel1) / (0.5f * (dt1 + dt2));
                    trajAccel[i] = a.magnitude;
                    accelMags.Add(trajAccel[i]);
                }
                if (validCount > 1)
                {
                    trajAccel[0] = trajAccel[1];
                    trajAccel[validCount - 1] = trajAccel[validCount - 2];
                }

                // 5-frame rolling median over |a| to kill central-difference spikes
                // (big-jump / id-swap artifacts produce huge isolated values that
                // would otherwise dominate the heatmap). Window of 5 is symmetric
                // around each sample; endpoints fall back to the available half.
                if (validCount > 1)
                    AccelMedian5(trajAccel, validCount);

                prepared.Add((traj, verts, trajAccel, validCount));
            }

            // Decide accelMax: auto picks p95 of |a| so colors don't collapse to base
            // when the recording happens to never hit the hand-tuned 56 m/s^2 ceiling.
            float effectiveAccelMax = accelMax;
            float p5 = 0f, p50 = 0f, p95 = 0f, mn = 0f, mx = 0f;
            if (accelMags.Count > 0)
            {
                accelMags.Sort();
                int n = accelMags.Count;
                float q(float t) => accelMags[Mathf.Clamp((int)(n * t), 0, n - 1)];
                mn = accelMags[0]; mx = accelMags[n - 1];
                p5 = q(0.05f); p50 = q(0.50f); p95 = q(0.95f);
                if (autoAccelMax) effectiveAccelMax = p95;
                Debug.Log($"[MotionLineRenderer] |a| samples={n} min={mn:F2} p5={p5:F2} p50={p50:F2} " +
                          $"p95={p95:F2} max={mx:F2} (m/s^2)  using accelMax={effectiveAccelMax:F2}" +
                          (autoAccelMax ? " (auto)" : ""));
            }

            // Pass 2: build colors with the final accelMax, smooth, and emit one mesh per trajectory.
            foreach (var item in prepared)
            {
                var traj = item.traj;
                int validCount = item.validCount;
                var verts = item.verts;
                var trajAccel = item.trajAccel;

                var colors = new Color[validCount];
                for (int i = 0; i < validCount; i++) colors[i] = ColorFor(traj, i, validCount);

                if (colorMode == ColorMode.AccelHeatmap)
                {
                    for (int i = 0; i < validCount; i++)
                    {
                        float t = Mathf.Clamp01(Mathf.InverseLerp(accelMin, effectiveAccelMax, trajAccel[i]));
                        float alpha = colors[i].a;
                        Color c = Color.Lerp(flatColor, accelHotColor, t);
                        c.a = alpha;
                        colors[i] = c;
                    }
                }

                if (smoothing > 0f && validCount >= 2)
                {
                    float a = 1f - smoothing;
                    for (int i = 1; i < validCount; i++)
                        verts[i] = Vector3.Lerp(verts[i - 1], verts[i], a);
                }

                int lineCount = validCount - 1;
                var indices = new int[lineCount * 2];
                for (int i = 0; i < lineCount; i++)
                {
                    indices[i * 2 + 0] = i;
                    indices[i * 2 + 1] = i + 1;
                }

                var mesh = new Mesh
                {
                    name = $"motion_b{traj.BodyId}_j{traj.JointId}",
                    indexFormat = validCount > ushort.MaxValue
                        ? UnityEngine.Rendering.IndexFormat.UInt32
                        : UnityEngine.Rendering.IndexFormat.UInt16,
                };
                mesh.vertices = verts;
                mesh.colors = colors;
                mesh.SetIndices(indices, MeshTopology.Lines, 0, true);

                var go = new GameObject(mesh.name);
                go.transform.SetParent(transform, false);
                var mf = go.AddComponent<MeshFilter>();
                mf.sharedMesh = mesh;
                var mr = go.AddComponent<MeshRenderer>();
                mr.sharedMaterial = _sharedMaterial;
                go.SetActive(show);
                _spawned.Add(go);
            }
        }

        private Color ColorFor(BodyTrackingPlayback.JointTrajectory traj, int sampleIndex, int sampleCount)
        {
            Color c;
            switch (colorMode)
            {
                case ColorMode.PerJoint:
                    c = HueFor((int)traj.JointId, K4ABTConsts.K4ABT_JOINT_COUNT);
                    break;
                case ColorMode.PerBody:
                    c = HueFor((int)(traj.BodyId % 12u), 12);
                    break;
                default:
                    c = flatColor;
                    break;
            }
            // Tint by flatColor (acts as a global multiplier).
            c *= flatColor;
            c.a = timeGradient && sampleCount > 1
                ? Mathf.Lerp(0.05f, 1f, sampleIndex / (float)(sampleCount - 1))
                : 1f;
            return c;
        }

        private static readonly float[] s_medianScratch = new float[5];
        private static void AccelMedian5(float[] arr, int count)
        {
            var copy = new float[count];
            System.Array.Copy(arr, copy, count);
            for (int i = 0; i < count; i++)
            {
                int lo = Mathf.Max(0, i - 2);
                int hi = Mathf.Min(count - 1, i + 2);
                int len = hi - lo + 1;
                for (int k = 0; k < len; k++) s_medianScratch[k] = copy[lo + k];
                System.Array.Sort(s_medianScratch, 0, len);
                arr[i] = s_medianScratch[len / 2];
            }
        }

        private static Color HueFor(int index, int total)
        {
            float h = (index % total) / (float)total;
            return Color.HSVToRGB(h, 0.85f, 1f);
        }

        private void EnsureSharedMaterial()
        {
            if (_sharedMaterial != null) return;
            // Vertex-color unlit material so the time-gradient alpha is visible.
            var shader = Shader.Find("Universal Render Pipeline/Particles/Unlit");
            if (shader == null) shader = Shader.Find("Sprites/Default");
            if (shader == null) shader = Shader.Find("Unlit/Color");
            _sharedMaterial = new Material(shader)
            {
                name = "MotionLineMat",
                color = Color.white,
            };
            // Try to enable vertex colors; for shaders that don't expose them this is a no-op.
            if (_sharedMaterial.HasProperty("_BaseColor"))
                _sharedMaterial.SetColor("_BaseColor", Color.white);
        }

        public void ClearSpawned()
        {
            foreach (var go in _spawned)
            {
                if (go == null) continue;
                var mf = go.GetComponent<MeshFilter>();
                if (mf != null && mf.sharedMesh != null) Object.Destroy(mf.sharedMesh);
                Object.Destroy(go);
            }
            _spawned.Clear();
        }
    }
}
