// Per-body skeleton visual: 32 joint markers + a single line-list mesh of
// bones + per-joint TrailRenderers. Extracted verbatim from BodyTrackingLive
// (issue #7) so multi-camera body tracking (issue #11) can reuse the same
// visual code path. Behavior is intentionally unchanged from the previous
// nested-class location — Phase 5b will introduce a BodyVisualPool
// abstraction on top.

using UnityEngine;

namespace BodyTracking
{
    internal sealed class BodyVisual
    {
        private readonly GameObject _root;
        private readonly Transform[] _joints = new Transform[K4ABTConsts.K4ABT_JOINT_COUNT];
        private readonly Vector3[] _jointPositions = new Vector3[K4ABTConsts.K4ABT_JOINT_COUNT];
        private readonly bool[] _jointValid = new bool[K4ABTConsts.K4ABT_JOINT_COUNT];

        private readonly GameObject _bonesGO;
        private readonly Mesh _bonesMesh;
        private readonly MeshRenderer _bonesRenderer;
        private readonly Material _bonesMat;
        private readonly Vector3[] _boneVerts;
        private readonly int[] _boneIndices;

        private TrailRenderer[] _trails;
        private bool[] _jointEverValid; // first-valid bookkeeping for trail Clear() seeding

        public BodyVisual(Transform parent, uint id, float jointRadius, Color color,
                          bool showTrails, float trailDuration, float trailWidth,
                          BodyTrackingLive.TrailColorMode trailColorMode, Color trailFlatColor)
        {
            _root = new GameObject($"Body_{id}");
            _root.transform.SetParent(parent, false);

            // Pre-resolve a shader. `new Material(null)` throws ArgumentNullException
            // immediately, so the old shader-then-fallback pattern (`new Material(Find(a));
            // if (mat.shader == null) mat = new Material(Find(b))`) crashes before the
            // fallback runs. Resolve the chain first, then construct the Material once.
            var unlitShader = ResolveUnlitShader();

            var jointMat = new Material(unlitShader);
            SetMaterialColor(jointMat, new Color(color.r * 1.2f, color.g * 1.2f, color.b * 1.2f, 1f));

            _trails = new TrailRenderer[_joints.Length];
            _jointEverValid = new bool[_joints.Length];

            for (int i = 0; i < _joints.Length; i++)
            {
                var sphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                Object.Destroy(sphere.GetComponent<Collider>());
                sphere.transform.SetParent(_root.transform, false);
                sphere.transform.localScale = Vector3.one * (jointRadius * 2f);
                sphere.GetComponent<MeshRenderer>().sharedMaterial = jointMat;
                sphere.name = ((k4abt_joint_id_t)i).ToString();
                _joints[i] = sphere.transform;

                // TrailRenderer per joint. Built-in component, renders in world space and
                // lays down a width-tapered strip behind the moving sphere. Joint-specific
                // hue makes individual limbs traceable in PerJointHue mode.
                var tr = sphere.AddComponent<TrailRenderer>();
                tr.time = trailDuration;
                tr.startWidth = trailWidth;
                tr.endWidth = 0f;
                tr.minVertexDistance = 0.005f;
                tr.numCornerVertices = 0;
                tr.numCapVertices = 0;
                tr.alignment = LineAlignment.View;
                tr.textureMode = LineTextureMode.Stretch;
                tr.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
                tr.receiveShadows = false;
                tr.sharedMaterial = ResolveTrailMaterial();
                var c = TrailColorFor(i, trailColorMode, trailFlatColor);
                var grad = new Gradient();
                grad.SetKeys(
                    new[] { new GradientColorKey(c, 0f), new GradientColorKey(c, 1f) },
                    new[] { new GradientAlphaKey(c.a, 0f), new GradientAlphaKey(0f, 1f) });
                tr.colorGradient = grad;
                tr.enabled = showTrails;
                _trails[i] = tr;
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

            _boneVerts = new Vector3[BodyTrackingLive.s_bones.Length * 2];
            _boneIndices = new int[BodyTrackingLive.s_bones.Length * 2];
            for (int i = 0; i < _boneIndices.Length; i++) _boneIndices[i] = i;
        }

        public void UpdateFromSkeleton(in k4abt_skeleton_t skel, float jointRadius,
                                        bool showBones, Color color)
        {
            // Each pop refreshes the last-known position for every joint with at least
            // LOW confidence. Joints whose confidence flaps below LOW keep their previous
            // position (don't toggle SetActive — that's what produced per-joint flicker
            // when a body was being tracked at the edge of the depth model's range).
            // Big single-frame jump that suggests the joint was re-detected at a new location
            // (id swap inside k4abt, brief tracking loss, etc.). Clearing the trail when this
            // happens prevents a long stray line from appearing through space. 0.4 m / frame
            // at 30 Hz = 12 m/s — well above natural human joint velocity.
            const float kTrailJumpResetMeters = 0.4f;

            for (int i = 0; i < _joints.Length; i++)
            {
                var j = skel.Joints[i];
                if (j.ConfidenceLevel >= k4abt_joint_confidence_level_t.K4ABT_JOINT_CONFIDENCE_LOW)
                {
                    var newPos = BodyTrackingLive.K4AmmToUnity(j.Position);
                    bool firstTime = _jointEverValid != null && !_jointEverValid[i];
                    bool bigJump = !firstTime &&
                                   (newPos - _jointPositions[i]).sqrMagnitude > kTrailJumpResetMeters * kTrailJumpResetMeters;
                    _jointPositions[i] = newPos;
                    _jointValid[i] = true;

                    if (firstTime || bigJump)
                    {
                        // Snap the joint to its real position WITHOUT drawing a trail back
                        // to the previous (or origin) location.
                        _joints[i].localPosition = newPos;
                        if (_trails != null && _trails[i] != null) _trails[i].Clear();
                        if (_jointEverValid != null) _jointEverValid[i] = true;
                        _joints[i].localScale = Vector3.one * (jointRadius * 2f);
                        if (!_joints[i].gameObject.activeSelf) _joints[i].gameObject.SetActive(true);
                        continue;
                    }
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
                for (int b = 0; b < BodyTrackingLive.s_bones.Length; b++)
                {
                    var (a, c) = BodyTrackingLive.s_bones[b];
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
            if (_bonesMesh != null) Object.Destroy(_bonesMesh);
            if (_root != null) Object.Destroy(_root);
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

        // Single shared material for every TrailRenderer — uses the same overlay shader so
        // trails draw on top of the point cloud. Vertex-color via the gradient handles the
        // per-trail color; we don't need per-instance materials.
        private static Material s_trailMat;
        private static Material ResolveTrailMaterial()
        {
            if (s_trailMat != null) return s_trailMat;
            s_trailMat = new Material(ResolveUnlitShader()) { name = "BT_Trail" };
            SetMaterialColor(s_trailMat, Color.white);
            return s_trailMat;
        }

        private static Color TrailColorFor(int jointIndex, BodyTrackingLive.TrailColorMode mode, Color flat)
        {
            if (mode == BodyTrackingLive.TrailColorMode.FlatColor) return flat;
            float h = (jointIndex % K4ABTConsts.K4ABT_JOINT_COUNT) / (float)K4ABTConsts.K4ABT_JOINT_COUNT;
            Color hue = Color.HSVToRGB(h, 0.85f, 1f);
            return new Color(hue.r * flat.r, hue.g * flat.g, hue.b * flat.b, flat.a);
        }

        // Live-tweakable trail params from the Inspector (no need to re-spawn visuals).
        public void ApplyTrailParams(bool show, float duration, float width,
                                      BodyTrackingLive.TrailColorMode mode, Color flat)
        {
            if (_trails == null) return;
            for (int i = 0; i < _trails.Length; i++)
            {
                var tr = _trails[i];
                if (tr == null) continue;
                tr.enabled = show;
                tr.time = duration;
                tr.startWidth = width;
                tr.endWidth = 0f;
                var c = TrailColorFor(i, mode, flat);
                var grad = new Gradient();
                grad.SetKeys(
                    new[] { new GradientColorKey(c, 0f), new GradientColorKey(c, 1f) },
                    new[] { new GradientAlphaKey(c.a, 0f), new GradientAlphaKey(0f, 1f) });
                tr.colorGradient = grad;
            }
        }
    }
}
