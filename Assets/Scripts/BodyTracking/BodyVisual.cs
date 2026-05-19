// Per-body skeleton visual: 32 joint markers + a single line-list mesh of
// bones + per-joint TrailRenderers. Used by BodyTrackingMultiLive's pool to
// draw merged and per-worker skeletons.

using System.Collections.Generic;
using UnityEngine;

namespace BodyTracking
{
    internal sealed class BodyVisual
    {
        private readonly uint _bodyId;
        private readonly GameObject _root;
        private readonly Transform[] _joints = new Transform[K4ABTConsts.K4ABT_JOINT_COUNT];
        private readonly Vector3[] _jointPositions = new Vector3[K4ABTConsts.K4ABT_JOINT_COUNT];
        private readonly bool[] _jointValid = new bool[K4ABTConsts.K4ABT_JOINT_COUNT];

        // Diagnostic bookkeeping for the "flying bones" investigation: the current
        // bone draw uses _jointPositions even when the underlying joint's confidence
        // has dropped to NONE (_jointValid never resets to false). These let a probe
        // distinguish "both endpoints fresh", "one stale one fresh", and "both stale".
        private readonly k4abt_joint_confidence_level_t[] _lastConfidence =
            new k4abt_joint_confidence_level_t[K4ABTConsts.K4ABT_JOINT_COUNT];
        private readonly int[] _lastFreshFrame = new int[K4ABTConsts.K4ABT_JOINT_COUNT];
        public k4abt_joint_confidence_level_t LastConfidence(int i) => _lastConfidence[i];
        public int LastFreshFrame(int i) => _lastFreshFrame[i];
        public bool JointValid(int i) => _jointValid[i];
        public Vector3 JointPosition(int i) => _jointPositions[i];

        // Per-joint inter-pop jump tracking. Set whenever a fresh-fresh transition
        // updates _jointPositions; lets us correlate visible "flying bones" with
        // which joint is actually moving a lot between pops.
        private readonly Vector3[] _prevPosForJump = new Vector3[K4ABTConsts.K4ABT_JOINT_COUNT];
        private readonly float[] _maxJumpThisWindow = new float[K4ABTConsts.K4ABT_JOINT_COUNT];
        private readonly float[] _lastJump = new float[K4ABTConsts.K4ABT_JOINT_COUNT];
        public float MaxJumpInWindow(int i) => _maxJumpThisWindow[i];
        public float LastJumpMeters(int i) => _lastJump[i];
        public void ResetJumpWindow() { for (int i = 0; i < _maxJumpThisWindow.Length; i++) _maxJumpThisWindow[i] = 0f; }

        private readonly GameObject _bonesGO;
        private readonly Mesh _bonesMesh;
        private readonly MeshRenderer _bonesRenderer;
        private readonly Material _bonesMat;
        private readonly Vector3[] _boneVerts;
        private readonly int[] _boneIndices;

        private JointTrailMesh[] _trails;
        private bool[] _jointEverValid; // first-valid bookkeeping for trail Clear() seeding

        // One-Euro filter state per joint. Smoothing parameters come from the
        // owning MonoBehaviour's Inspector (passed each pop via BodyVisualConfig).
        // This is the only joint-position smoother in the pipeline — mergeSmoothing
        // and trailSmoothing EMAs were removed because they were redundant flat
        // low-passes layered on top of this speed-adaptive filter.
        private readonly OneEuroVec3[] _oneEuro = new OneEuroVec3[K4ABTConsts.K4ABT_JOINT_COUNT];

        public BodyVisual(Transform parent, uint id, float jointRadius, Color color,
                          bool showTrails, float trailDuration, float trailWidth,
                          BodyTrackingShared.TrailColorMode trailColorMode, Color trailFlatColor)
        {
            _bodyId = id;
            _root = new GameObject($"Body_{id}");
            _root.transform.SetParent(parent, false);

            // Pre-resolve a shader. `new Material(null)` throws ArgumentNullException
            // immediately, so the old shader-then-fallback pattern (`new Material(Find(a));
            // if (mat.shader == null) mat = new Material(Find(b))`) crashes before the
            // fallback runs. Resolve the chain first, then construct the Material once.
            var unlitShader = ResolveUnlitShader();

            var jointMat = new Material(unlitShader);
            SetMaterialColor(jointMat, new Color(color.r * 1.2f, color.g * 1.2f, color.b * 1.2f, 1f));

            _trails = new JointTrailMesh[_joints.Length];
            _jointEverValid = new bool[_joints.Length];

            var trailMat = ResolveTrailMaterial();

            for (int i = 0; i < _joints.Length; i++)
            {
                var sphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                Object.Destroy(sphere.GetComponent<Collider>());
                sphere.transform.SetParent(_root.transform, false);
                sphere.transform.localScale = Vector3.one * (jointRadius * 2f);
                sphere.GetComponent<MeshRenderer>().sharedMaterial = jointMat;
                sphere.name = ((k4abt_joint_id_t)i).ToString();
                _joints[i] = sphere.transform;

                // Quad-strip trail mesh per joint. Lives as a child of _root so it shares
                // the body's local space with the joint sphere (we feed it sample positions
                // in the same local frame as _jointPositions).
                _trails[i] = new JointTrailMesh(_root.transform, $"Trail_{(k4abt_joint_id_t)i}", trailMat);
                var c = TrailColorFor(i, id, trailColorMode, trailFlatColor);
                _trails[i].Configure(showTrails, trailDuration, trailWidth, c, Color.red,
                    0f, 56f,
                    trailColorMode == BodyTrackingShared.TrailColorMode.AccelHeatmap
                        ? JointTrailMesh.ColorMode.AccelHeatmap
                        : JointTrailMesh.ColorMode.Base);
            }

            _bonesGO = new GameObject("Bones");
            _bonesGO.transform.SetParent(_root.transform, false);
            var mf = _bonesGO.AddComponent<MeshFilter>();
            _bonesRenderer = _bonesGO.AddComponent<MeshRenderer>();
            _bonesMesh = new Mesh { name = "Bones", indexFormat = UnityEngine.Rendering.IndexFormat.UInt16 };
            mf.sharedMesh = _bonesMesh;

            _bonesMat = new Material(unlitShader);
            SetMaterialColor(_bonesMat, color);
            _bonesRenderer.sharedMaterial = _bonesMat;

            _boneVerts = new Vector3[BodyTrackingShared.Bones.Length * 2];
            _boneIndices = new int[BodyTrackingShared.Bones.Length * 2];
            for (int i = 0; i < _boneIndices.Length; i++) _boneIndices[i] = i;
        }

        public void UpdateFromSkeleton(in k4abt_skeleton_t skel, in BodyVisualConfig cfg)
        {
            // Each pop refreshes the last-known position for every joint with at least
            // LOW confidence. Joints whose confidence flaps below LOW keep their previous
            // position (don't toggle SetActive — that's what produced per-joint flicker
            // when a body was being tracked at the edge of the depth model's range).
            // Position smoothing is delegated to the per-joint One-Euro filter below;
            // the old firstTime branch still snaps without filtering so the trail does
            // not draw a segment back to the origin on a body's first valid frame.
            float jointRadius = cfg.JointRadius;
            bool showBones = cfg.ShowAnatomicalBones;
            Color color = cfg.SkeletonColor;
            float dt = Time.deltaTime;

            for (int i = 0; i < _joints.Length; i++)
            {
                var j = skel.Joints[i];
                _lastConfidence[i] = j.ConfidenceLevel;
                if (j.ConfidenceLevel >= k4abt_joint_confidence_level_t.K4ABT_JOINT_CONFIDENCE_LOW)
                {
                    _lastFreshFrame[i] = Time.frameCount;
                    var rawPos = BodyTrackingShared.K4AmmToUnity(j.Position);
                    bool firstTime = _jointEverValid != null && !_jointEverValid[i];
                    _jointValid[i] = true;

                    if (firstTime)
                    {
                        // Snap the joint to its real position WITHOUT drawing a trail back
                        // to the origin. Also seed the 1€ filter with this sample so the
                        // next pop's velocity estimate is zero rather than huge.
                        _jointPositions[i] = rawPos;
                        _oneEuro[i].Reset();
                        if (cfg.UseOneEuroFilter)
                            _oneEuro[i].Filter(rawPos, dt, cfg.OneEuroMinCutoff, cfg.OneEuroBeta, cfg.OneEuroDerivCutoff);
                        _joints[i].localPosition = rawPos;
                        if (_trails != null && _trails[i] != null)
                        {
                            _trails[i].Clear();
                            _trails[i].AddSample(Time.timeAsDouble, rawPos);
                        }
                        if (_jointEverValid != null) _jointEverValid[i] = true;
                        _joints[i].localScale = Vector3.one * (jointRadius * 2f);
                        if (!_joints[i].gameObject.activeSelf) _joints[i].gameObject.SetActive(true);
                        continue;
                    }

                    Vector3 newPos = cfg.UseOneEuroFilter
                        ? _oneEuro[i].Filter(rawPos, dt, cfg.OneEuroMinCutoff, cfg.OneEuroBeta, cfg.OneEuroDerivCutoff)
                        : rawPos;

                    float jump = (newPos - _jointPositions[i]).magnitude;
                    _lastJump[i] = jump;
                    if (jump > _maxJumpThisWindow[i]) _maxJumpThisWindow[i] = jump;
                    _jointPositions[i] = newPos;
                    if (_trails != null && _trails[i] != null)
                        _trails[i].AddSample(Time.timeAsDouble, _jointPositions[i]);
                }
                // else: leave _jointPositions[i] / _jointValid[i] from previous pop.
                _joints[i].localPosition = _jointPositions[i];
                _joints[i].localScale = Vector3.one * (jointRadius * 2f);
                if (!_joints[i].gameObject.activeSelf && _jointValid[i])
                {
                    _joints[i].gameObject.SetActive(true);
                }
            }

            if (showBones)
            {
                int v = 0;
                for (int b = 0; b < BodyTrackingShared.Bones.Length; b++)
                {
                    var (a, c) = BodyTrackingShared.Bones[b];
                    if (_jointValid[(int)a] && _jointValid[(int)c])
                    {
                        _boneVerts[v++] = _jointPositions[(int)a];
                        _boneVerts[v++] = _jointPositions[(int)c];
                    }
                }
                // Pad unused slots to last valid vertex so the line list stays well-formed.
                if (v == 0)
                {
                    _bonesMesh.Clear();
                }
                else
                {
                    var verts = new Vector3[v];
                    var idx = new int[v];
                    System.Array.Copy(_boneVerts, verts, v);
                    for (int i = 0; i < v; i++) idx[i] = i;
                    _bonesMesh.Clear();
                    _bonesMesh.vertices = verts;
                    _bonesMesh.SetIndices(idx, MeshTopology.Lines, 0, true);
                }
                SetMaterialColor(_bonesMat, color);
                if (!_bonesGO.activeSelf) _bonesGO.SetActive(true);
            }
            else
            {
                if (_bonesGO.activeSelf) _bonesGO.SetActive(false);
            }
        }

        public int SetActiveCallsTrue { get; private set; }
        public int SetActiveCallsFalse { get; private set; }

        public void SetVisible(bool visible)
        {
            if (_root == null) return;
            if (_root.activeSelf == visible) return;
            _root.SetActive(visible);
            if (visible) SetActiveCallsTrue++;
            else SetActiveCallsFalse++;
        }

        public bool IsActive => _root != null && _root.activeInHierarchy;

        // Diagnostic accessors (used by the parent's per-second log).
        public Vector3 GetSamplePosition()
        {
            int idx = (int)k4abt_joint_id_t.K4ABT_JOINT_PELVIS;
            return idx < _jointPositions.Length ? _jointPositions[idx] : Vector3.zero;
        }

        public Vector3 WorldOf(Vector3 local)
        {
            return _root != null ? _root.transform.TransformPoint(local) : local;
        }

        // Diagnostic: how often did this body's visibility toggle and how big are
        // the per-pop position jumps? Reset by ResetDiagWindow.
        public int VisibilityToggles { get; private set; }
        public float MaxJumpThisWindow { get; private set; }
        public float SumJumpThisWindow { get; private set; }
        public int JumpSamples { get; private set; }
        private bool _wasActive;
        private Vector3 _prevPelvis;

        public void TickDiagAfterUpdate()
        {
            bool nowActive = _root != null && _root.activeSelf;
            if (nowActive != _wasActive) VisibilityToggles++;
            _wasActive = nowActive;

            int idx = (int)k4abt_joint_id_t.K4ABT_JOINT_PELVIS;
            if (idx < _jointPositions.Length)
            {
                var cur = _jointPositions[idx];
                if (_prevPelvis != Vector3.zero)
                {
                    float d = (cur - _prevPelvis).magnitude;
                    if (d > MaxJumpThisWindow) MaxJumpThisWindow = d;
                    SumJumpThisWindow += d;
                    JumpSamples++;
                }
                _prevPelvis = cur;
            }
        }

        public void ResetDiagWindow()
        {
            VisibilityToggles = 0;
            MaxJumpThisWindow = 0f;
            SumJumpThisWindow = 0f;
            JumpSamples = 0;
            SetActiveCallsTrue = 0;
            SetActiveCallsFalse = 0;
        }

        public void Destroy()
        {
            if (_trails != null)
                for (int i = 0; i < _trails.Length; i++)
                    if (_trails[i] != null) _trails[i].Destroy();
            if (_bonesMesh != null) Object.Destroy(_bonesMesh);
            if (_root != null) Object.Destroy(_root);
        }

        // Rebuild every joint's trail mesh against the given camera (for billboard
        // orientation). Called once per frame by the owning MonoBehaviour after the
        // skeleton update so vertex positions / colors are fresh.
        public void TickTrails(Camera cam)
        {
            if (_trails == null) return;
            double now = Time.timeAsDouble;
            for (int i = 0; i < _trails.Length; i++)
                if (_trails[i] != null) _trails[i].Rebuild(now, cam);
        }

        /// <summary>Feed every joint's latest |a| into <paramref name="sink"/> so the
        /// pool can build a rolling window across all visible bodies / joints.</summary>
        public void CollectLatestAccels(List<float> sink)
        {
            if (_trails == null) return;
            for (int i = 0; i < _trails.Length; i++)
                if (_trails[i] != null) sink.Add(_trails[i].LastAccel);
        }

        /// <summary>Override the trail accelMax for every joint of this body. Used
        /// when autoAccelMax is on; the pool computes one rolling p95 and pushes it
        /// here so all trails share the same hot-end calibration.</summary>
        public void SetTrailAccelMax(float v)
        {
            if (_trails == null) return;
            for (int i = 0; i < _trails.Length; i++)
                if (_trails[i] != null) _trails[i].SetAccelMax(v);
        }

        private static Shader ResolveUnlitShader()
        {
            // SkeletonOverlay forces ZTest Always + ZWrite Off + Overlay queue so the
            // joints/bones aren't occluded by the point cloud surface. URP-compatible
            // (HLSLPROGRAM, UniversalForward LightMode). Fallback to URP/Unlit if it
            // ever fails to compile (skeleton would lose the always-on-top behaviour
            // but at least render).
            Shader s = Shader.Find("BodyTracking/SkeletonOverlay");
            if (s == null) s = Shader.Find("Universal Render Pipeline/Unlit");
            if (s == null) s = Shader.Find("Unlit/Color");
            if (s == null) s = Shader.Find("Sprites/Default");
            return s;
        }

        // URP/Unlit binds the visible color to "_BaseColor"; legacy Material.color
        // writes "_Color" instead and gets ignored on URP shaders. Set both so the
        // material picks up the colour regardless of which pipeline is in use.
        private static void SetMaterialColor(Material m, Color c)
        {
            if (m == null) return;
            if (m.HasProperty("_BaseColor")) m.SetColor("_BaseColor", c);
            if (m.HasProperty("_Color")) m.SetColor("_Color", c);
            m.color = c;
        }

        // Single shared material for every JointTrailMesh. Uses TrailOverlay
        // (vertex-color + alpha-blend + ZTest Always) so the per-vertex
        // acceleration heatmap actually reaches the framebuffer; the
        // SkeletonOverlay shader used for joints/bones discards mesh.colors.
        private static Material s_trailMat;
        private static Material ResolveTrailMaterial()
        {
            if (s_trailMat != null) return s_trailMat;
            Shader s = Shader.Find("BodyTracking/TrailOverlay");
            if (s == null) s = Shader.Find("Universal Render Pipeline/Particles/Unlit");
            if (s == null) s = Shader.Find("Sprites/Default");
            s_trailMat = new Material(s) { name = "BT_Trail" };
            SetMaterialColor(s_trailMat, Color.white);
            return s_trailMat;
        }

        private static Color TrailColorFor(int jointIndex, uint bodyId, BodyTrackingShared.TrailColorMode mode, Color flat)
        {
            // AccelHeatmap mode treats flat as the cold (base) end; per-vertex
            // interpolation toward hot is done inside JointTrailMesh.
            if (mode == BodyTrackingShared.TrailColorMode.FlatColor ||
                mode == BodyTrackingShared.TrailColorMode.AccelHeatmap) return flat;
            int idx = mode == BodyTrackingShared.TrailColorMode.PerBody
                ? (int)(bodyId % 12u)
                : jointIndex;
            int total = mode == BodyTrackingShared.TrailColorMode.PerBody
                ? 12
                : K4ABTConsts.K4ABT_JOINT_COUNT;
            float h = (idx % total) / (float)total;
            Color hue = Color.HSVToRGB(h, 0.85f, 1f);
            return new Color(hue.r * flat.r, hue.g * flat.g, hue.b * flat.b, flat.a);
        }

        // Live-tweakable trail params from the Inspector (no need to re-spawn visuals).
        public void ApplyTrailParams(bool show, float duration, float width,
                                      BodyTrackingShared.TrailColorMode mode, Color flat,
                                      float accelMin, float accelMax, Color accelHotColor)
        {
            if (_trails == null) return;
            JointTrailMesh.ColorMode jmMode = mode == BodyTrackingShared.TrailColorMode.AccelHeatmap
                ? JointTrailMesh.ColorMode.AccelHeatmap
                : JointTrailMesh.ColorMode.Base;
            for (int i = 0; i < _trails.Length; i++)
            {
                var tr = _trails[i];
                if (tr == null) continue;
                var baseColor = TrailColorFor(i, _bodyId, mode, flat);
                tr.Configure(show, duration, width, baseColor, accelHotColor,
                    accelMin, accelMax, jmMode);
            }
        }
    }
}
