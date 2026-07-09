// Per-body skeleton visual: 32 joint markers + a single tube mesh of
// bones. Used by SkeletonMerger's pool to draw merged and per-worker
// skeletons.

using UnityEngine;

namespace BodyTracking
{
    internal sealed class BodyVisual
    {
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

        private readonly GameObject _bonesGO;
        private readonly Mesh _bonesMesh;
        private readonly MeshRenderer _bonesRenderer;
        private readonly Material _bonesMat;
        // Tube mesh buffers for the anatomical bones. Each bone segment owns a
        // contiguous slice of 2 * kBoneTubeSides vertices (head ring at jointA +
        // tail ring at jointC) and kBoneTubeSides * 6 triangle indices. Sized
        // once in the constructor against BodyTrackingShared.Bones.Length and
        // refilled in place every frame — triangle indices and vertex colors
        // never change so they're populated once up front.
        private readonly Vector3[] _boneVerts;
        private readonly Vector3[] _boneNormals;
        private readonly Color[] _boneColors;
        private readonly int[] _boneTris;

        // First-valid bookkeeping per joint. On a joint's first LOW+ frame we snap
        // to the real position and reset/seed the One-Euro filter so its first
        // velocity estimate is zero instead of a huge jump from the origin.
        private bool[] _jointEverValid;

        // One-Euro filter state per joint. Smoothing parameters come from the
        // owning MonoBehaviour's Inspector (passed each pop via BodyVisualConfig).
        // This is the only joint-position smoother in the pipeline — mergeSmoothing
        // and trailSmoothing EMAs were removed because they were redundant flat
        // low-passes layered on top of this speed-adaptive filter.
        private readonly OneEuroVec3[] _oneEuro = new OneEuroVec3[K4ABTConsts.K4ABT_JOINT_COUNT];

        public BodyVisual(Transform parent, uint id, float jointRadius, Color color)
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
            }

            _bonesGO = new GameObject("Bones");
            _bonesGO.transform.SetParent(_root.transform, false);
            var mf = _bonesGO.AddComponent<MeshFilter>();
            _bonesRenderer = _bonesGO.AddComponent<MeshRenderer>();
            // Bones max out at Bones.Length * 2 * kBoneTubeSides verts (744 with
            // the current 31-bone, 12-side config) so UInt16 is plenty.
            _bonesMesh = new Mesh { name = "Bones", indexFormat = UnityEngine.Rendering.IndexFormat.UInt16 };
            _bonesMesh.MarkDynamic();
            mf.sharedMesh = _bonesMesh;

            // Bone tubes are PBR-shaded via TrailLit so they catch scene lighting
            // and read as 3D cylinders. The material instance is per-BodyVisual
            // because skeletonColor lives in _BaseColor here while every bone
            // vertex carries a fixed white color — TrailLit multiplies them.
            _bonesMat = new Material(ResolveTrailLitShader()) { name = "BT_Bones" };
            SetMaterialColor(_bonesMat, color);
            _bonesRenderer.sharedMaterial = _bonesMat;

            int boneCount = BodyTrackingShared.Bones.Length;
            int vertsPerBone = 2 * kBoneTubeSides;
            int trisPerBone = kBoneTubeSides * 6;
            _boneVerts = new Vector3[boneCount * vertsPerBone];
            _boneNormals = new Vector3[boneCount * vertsPerBone];
            _boneColors = new Color[boneCount * vertsPerBone];
            _boneTris = new int[boneCount * trisPerBone];

            // Per-vertex color stays white forever: bone hue comes from the
            // material's _BaseColor (skeletonColor), and TrailLit multiplies the
            // two. Triangle indices are fixed by the tube topology so they're
            // populated once here too. Per frame we only rewrite positions and
            // normals (and collapse-to-point for invalid bones).
            for (int i = 0; i < _boneColors.Length; i++) _boneColors[i] = Color.white;
            for (int b = 0; b < boneCount; b++)
            {
                int vBase = b * vertsPerBone;
                int tBase = b * trisPerBone;
                int ringA = vBase;
                int ringB = vBase + kBoneTubeSides;
                for (int k = 0; k < kBoneTubeSides; k++)
                {
                    int kNext = (k + 1) % kBoneTubeSides;
                    int t = tBase + k * 6;
                    _boneTris[t + 0] = ringA + k;
                    _boneTris[t + 1] = ringB + k;
                    _boneTris[t + 2] = ringA + kNext;
                    _boneTris[t + 3] = ringA + kNext;
                    _boneTris[t + 4] = ringB + k;
                    _boneTris[t + 5] = ringB + kNext;
                }
            }
        }

        // Sides around each bone tube's circular cross-section. 12 reads as a
        // smoothly round cylinder under PBR lighting.
        private const int kBoneTubeSides = 12;

        // Pre-computed unit-circle offsets (cos, sin) reused every frame to
        // place each ring vertex without per-vertex trig.
        private static readonly float[] s_boneRingCos = BuildBoneRingCos();
        private static readonly float[] s_boneRingSin = BuildBoneRingSin();
        private static float[] BuildBoneRingCos()
        {
            var a = new float[kBoneTubeSides];
            for (int k = 0; k < kBoneTubeSides; k++) a[k] = Mathf.Cos(2f * Mathf.PI * k / kBoneTubeSides);
            return a;
        }
        private static float[] BuildBoneRingSin()
        {
            var a = new float[kBoneTubeSides];
            for (int k = 0; k < kBoneTubeSides; k++) a[k] = Mathf.Sin(2f * Mathf.PI * k / kBoneTubeSides);
            return a;
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
                // Hand joints past the wrist are excluded project-wide (see
                // BodyTrackingShared.IsDrawnJoint): hide the sphere so nothing draws
                // at the unreliable NONE-confidence prediction.
                if (!BodyTrackingShared.IsDrawnJoint((k4abt_joint_id_t)i))
                {
                    if (_joints[i] != null && _joints[i].gameObject.activeSelf) _joints[i].gameObject.SetActive(false);
                    _jointValid[i] = false;
                    continue;
                }
                if (j.ConfidenceLevel >= k4abt_joint_confidence_level_t.K4ABT_JOINT_CONFIDENCE_LOW)
                {
                    _lastFreshFrame[i] = Time.frameCount;
                    var rawPos = BodyTrackingShared.K4AmmToUnity(j.Position);
                    bool firstTime = _jointEverValid != null && !_jointEverValid[i];
                    _jointValid[i] = true;

                    if (firstTime)
                    {
                        // Snap the joint to its real position. Seed the 1€ filter with
                        // this sample so the next pop's velocity estimate is zero rather
                        // than huge.
                        _jointPositions[i] = rawPos;
                        _oneEuro[i].Reset();
                        if (cfg.UseOneEuroFilter)
                            _oneEuro[i].Filter(rawPos, dt, cfg.OneEuroMinCutoff, cfg.OneEuroBeta, cfg.OneEuroDerivCutoff);
                        _joints[i].localPosition = rawPos;
                        if (_jointEverValid != null) _jointEverValid[i] = true;
                        _joints[i].localScale = Vector3.one * (jointRadius * 2f);
                        // showBones gates DRAWING only; the position above is always updated so hidden
                        // consumers still read live poses.
                        if (_joints[i].gameObject.activeSelf != showBones) _joints[i].gameObject.SetActive(showBones);
                        continue;
                    }

                    Vector3 newPos = cfg.UseOneEuroFilter
                        ? _oneEuro[i].Filter(rawPos, dt, cfg.OneEuroMinCutoff, cfg.OneEuroBeta, cfg.OneEuroDerivCutoff)
                        : rawPos;

                    _jointPositions[i] = newPos;
                }
                // else: leave _jointPositions[i] / _jointValid[i] from previous pop.
                _joints[i].localPosition = _jointPositions[i];
                _joints[i].localScale = Vector3.one * (jointRadius * 2f);
                // Draw the sphere only when the skeleton is shown AND the joint is valid; positions are
                // updated above regardless, so hiding the skeleton doesn't stop data flowing to consumers.
                bool wantActive = showBones && _jointValid[i];
                if (_joints[i].gameObject.activeSelf != wantActive) _joints[i].gameObject.SetActive(wantActive);
            }

            if (showBones)
            {
                RebuildBoneTubes(Mathf.Max(0f, cfg.BoneWidth));
                SetMaterialColor(_bonesMat, color);
                if (!_bonesGO.activeSelf) _bonesGO.SetActive(true);
            }
            else
            {
                if (_bonesGO.activeSelf) _bonesGO.SetActive(false);
            }
        }

        /// <summary>
        /// Re-apply visual configuration without ingesting a new skeleton sample.
        /// Used by SkeletonMerger while the Editor is paused so Inspector
        /// tweaks (joint radius, bone width, color) reflect immediately instead of
        /// waiting for unpause. Does not touch _jointPositions or _jointValid.
        /// </summary>
        public void ApplyConfigOnly(in BodyVisualConfig cfg)
        {
            float r = Mathf.Max(0f, cfg.JointRadius);
            for (int i = 0; i < _joints.Length; i++)
            {
                if (_joints[i] == null) continue;
                _joints[i].localScale = Vector3.one * (r * 2f);
            }

            if (cfg.ShowAnatomicalBones)
            {
                RebuildBoneTubes(Mathf.Max(0f, cfg.BoneWidth));
                SetMaterialColor(_bonesMat, cfg.SkeletonColor);
                if (!_bonesGO.activeSelf) _bonesGO.SetActive(true);
            }
            else if (_bonesGO.activeSelf)
            {
                _bonesGO.SetActive(false);
            }
        }

        // Build the bones mesh as a single tube-tri soup: each bone segment gets
        // two rings of kBoneTubeSides verts (head=jointA, tail=jointC) stitched
        // by side triangles. No taper — both rings share radius=boneWidth. The
        // frame per bone is built fresh from the tangent (no parallel-transport
        // chain across bones since bones don't form a continuous curve). Bones
        // whose endpoints aren't both valid collapse to a single point so their
        // triangles degenerate (zero area, GPU draws nothing visible) while the
        // mesh keeps a fixed vertex/index layout — avoids reallocating arrays
        // every frame as bones come and go. Normals are analytical radial outward
        // so the shader reads a smooth cylinder under PBR lighting.
        private void RebuildBoneTubes(float radius)
        {
            int boneCount = BodyTrackingShared.Bones.Length;
            int vertsPerBone = 2 * kBoneTubeSides;
            bool anyValid = false;

            for (int b = 0; b < boneCount; b++)
            {
                var (ja, jc) = BodyTrackingShared.Bones[b];
                int ia = (int)ja, ic = (int)jc;
                int vBase = b * vertsPerBone;
                int ringA = vBase;
                int ringB = vBase + kBoneTubeSides;

                if (!_jointValid[ia] || !_jointValid[ic])
                {
                    // Collapse this bone to the origin so its triangles have zero
                    // area. Normal is arbitrary (Vector3.up) — won't render anyway.
                    for (int k = 0; k < vertsPerBone; k++)
                    {
                        _boneVerts[vBase + k] = Vector3.zero;
                        _boneNormals[vBase + k] = Vector3.up;
                    }
                    continue;
                }

                Vector3 pA = _jointPositions[ia];
                Vector3 pC = _jointPositions[ic];
                Vector3 tangent = pC - pA;
                tangent = tangent.sqrMagnitude > 1e-12f ? tangent.normalized : Vector3.forward;

                Vector3 seed = Mathf.Abs(Vector3.Dot(tangent, Vector3.up)) < 0.95f
                    ? Vector3.up
                    : Vector3.right;
                Vector3 right = Vector3.Cross(tangent, seed);
                if (right.sqrMagnitude < 1e-12f) right = Vector3.Cross(tangent, Vector3.right);
                right = right.sqrMagnitude > 1e-12f ? right.normalized : Vector3.right;
                Vector3 up = Vector3.Cross(right, tangent);
                up = up.sqrMagnitude > 1e-12f ? up.normalized : Vector3.up;

                for (int k = 0; k < kBoneTubeSides; k++)
                {
                    float cosk = s_boneRingCos[k];
                    float sink = s_boneRingSin[k];
                    Vector3 radial = right * cosk + up * sink;
                    _boneVerts[ringA + k] = pA + radial * radius;
                    _boneNormals[ringA + k] = radial;
                    _boneVerts[ringB + k] = pC + radial * radius;
                    _boneNormals[ringB + k] = radial;
                }
                anyValid = true;
            }

            if (!anyValid)
            {
                _bonesMesh.Clear();
                return;
            }

            // Clear first so the vertex count / triangle count swap atomically —
            // otherwise Unity warns when you assign vertices smaller than the
            // current triangle index range.
            // Use the array-with-range overloads so Unity reads directly out of
            // our pre-allocated buffers instead of copying via the legacy
            // property setters (which alloc transient garbage per frame).
            _bonesMesh.Clear();
            _bonesMesh.SetVertices(_boneVerts, 0, _boneVerts.Length);
            _bonesMesh.SetNormals(_boneNormals, 0, _boneNormals.Length);
            _bonesMesh.SetColors(_boneColors, 0, _boneColors.Length);
            _bonesMesh.SetTriangles(_boneTris, 0, true);
        }

        public bool IsActive => _root != null && _root.activeInHierarchy;

        public Vector3 WorldOf(Vector3 local)
        {
            return _root != null ? _root.transform.TransformPoint(local) : local;
        }

        /// <summary>Current merged world position of one joint, straight from the per-frame
        /// smoothed <c>_jointPositions</c> (NOT the trail history) so callers don't depend on
        /// the fading-trail buffer. Returns false if the joint isn't currently valid.</summary>
        public bool TryGetJointWorld(int jointIdx, out Vector3 world)
        {
            world = Vector3.zero;
            if (jointIdx < 0 || jointIdx >= _jointPositions.Length) return false;
            if (!_jointValid[jointIdx]) return false;
            world = WorldOf(_jointPositions[jointIdx]);
            return true;
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
            // Resources first: nothing else references these auto-material shaders, so
            // a bare Shader.Find returns null in a player build (they get stripped).
            Shader s = Resources.Load<Shader>("SkeletonOverlay");
            if (s == null) s = Shader.Find("BodyTracking/SkeletonOverlay");
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

        // Shader resolution chain for the TrailLit-shaded bone tubes. TrailLit is
        // URP PBR + per-vertex color tint + ShadowCaster + DepthNormals (so SSAO
        // from the PC_Renderer feature attaches automatically) so the bones catch
        // scene lighting and read as 3D cylinders. Falls back to TrailOverlay, then
        // to URP particle / sprite shaders as a last resort.
        private static Shader ResolveTrailLitShader()
        {
            Shader s = Resources.Load<Shader>("TrailLit");
            if (s == null) s = Shader.Find("BodyTracking/TrailLit");
            if (s == null) s = Resources.Load<Shader>("TrailOverlay");
            if (s == null) s = Shader.Find("BodyTracking/TrailOverlay");
            if (s == null) s = Shader.Find("Universal Render Pipeline/Particles/Unlit");
            if (s == null) s = Shader.Find("Sprites/Default");
            return s;
        }
    }
}
