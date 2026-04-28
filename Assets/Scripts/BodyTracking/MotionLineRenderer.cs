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
        public enum ColorMode { PerJoint, PerBody, FlatColor }

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

            foreach (var traj in trajectories)
            {
                int validCount = 0;
                for (int i = 0; i < traj.Samples.Count; i++)
                    if (traj.Samples[i].Confidence >= minConfidence) validCount++;
                if (validCount < minSamples) continue;

                var verts = new Vector3[validCount];
                var colors = new Color[validCount];
                int v = 0;
                for (int i = 0; i < traj.Samples.Count; i++)
                {
                    var s = traj.Samples[i];
                    if (s.Confidence < minConfidence) continue;
                    verts[v] = s.Position;
                    colors[v] = ColorFor(traj, v, validCount);
                    v++;
                }

                // line list: each segment is two consecutive samples
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
