// Per-body skeleton visual: 32 joint markers + a single tube mesh of
// bones + per-joint TrailRenderers. Used by SkeletonMerger's pool to
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

        private JointTrailMesh[] _trails;
        private bool[] _jointEverValid; // first-valid bookkeeping for trail Clear() seeding

        // Per-bone interpolation-point trails. One JointTrailMesh per parametric
        // interp slot per bone (excluding endpoints — those are already covered
        // by the per-joint trails). _currentBoneTrailStep records the step used
        // to build _boneTrails so ApplyTrailParams can detect a config change
        // and rebuild the array. _boneEverValid mirrors _jointEverValid so the
        // first sample for a freshly-valid bone seeds the trail without drawing
        // a segment from the origin.
        private List<JointTrailMesh>[] _boneTrails;
        private bool[] _boneEverValid;
        private float _currentBoneTrailStep;

        // One-Euro filter state per joint. Smoothing parameters come from the
        // owning MonoBehaviour's Inspector (passed each pop via BodyVisualConfig).
        // This is the only joint-position smoother in the pipeline — mergeSmoothing
        // and trailSmoothing EMAs were removed because they were redundant flat
        // low-passes layered on top of this speed-adaptive filter.
        private readonly OneEuroVec3[] _oneEuro = new OneEuroVec3[K4ABTConsts.K4ABT_JOINT_COUNT];

        public BodyVisual(Transform parent, uint id, float jointRadius, Color color,
                          bool showTrails, float trailDuration, float trailWidth,
                          BodyTrackingShared.TrailColorMode trailColorMode, Color trailFlatColor,
                          BodyTrackingShared.FrameHueParams frameHue)
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

                // Tube trail mesh per joint. Lives as a child of _root so it shares
                // the body's local space with the joint sphere (we feed it sample positions
                // in the same local frame as _jointPositions).
                _trails[i] = new JointTrailMesh(_root.transform, $"Trail_{(k4abt_joint_id_t)i}", trailMat);
                var c = TrailColorFor(i, id, trailColorMode, trailFlatColor, frameHue);
                _trails[i].Configure(showTrails, trailDuration, trailWidth, c, Color.red,
                    0f, 56f, ToJointTrailMode(trailColorMode), frameHue);
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

            // Bone tubes are PBR-shaded via TrailLit (same shader as the per-joint
            // trails) so they catch scene lighting and read as 3D cylinders. The
            // material instance is per-BodyVisual (not the s_trailMat singleton)
            // because skeletonColor lives in _BaseColor here while every bone
            // vertex carries a fixed white color — TrailLit multiplies them so a
            // shared white-_BaseColor singleton wouldn't pick up skeletonColor.
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

        // Sides around each bone tube's circular cross-section. Kept at 12 so it
        // matches JointTrailMesh.kTubeSides — joint trails and bone tubes share
        // visual silhouette. If you bump this, update the trail constant too.
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

        public void UpdateFromSkeleton(in k4abt_skeleton_t skel, in BodyVisualConfig cfg, double trailNow)
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
                            _trails[i].AddSample(trailNow, rawPos, Time.frameCount);
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
                        _trails[i].AddSample(trailNow, _jointPositions[i], Time.frameCount);
                }
                // else: leave _jointPositions[i] / _jointValid[i] from previous pop.
                _joints[i].localPosition = _jointPositions[i];
                _joints[i].localScale = Vector3.one * (jointRadius * 2f);
                if (!_joints[i].gameObject.activeSelf && _jointValid[i])
                {
                    _joints[i].gameObject.SetActive(true);
                }
            }

            // Feed interpolation-point trails along each bone (in addition to
            // the per-joint trails fed above). Lives in the same local space as
            // _jointPositions so positions just lerp directly. Sampling cadence
            // follows the joint loop (= one sample per merged pop).
            EnsureBoneTrails(cfg.BoneTrailStep);
            if (_boneTrails != null && _currentBoneTrailStep > 0f)
            {
                double now = trailNow;
                for (int b = 0; b < BodyTrackingShared.Bones.Length; b++)
                {
                    var (ja, jc) = BodyTrackingShared.Bones[b];
                    int ia = (int)ja, ic = (int)jc;
                    if (!_jointValid[ia] || !_jointValid[ic]) continue;
                    var meshList = _boneTrails[b];
                    if (meshList == null || meshList.Count == 0) continue;
                    Vector3 pA = _jointPositions[ia];
                    Vector3 pC = _jointPositions[ic];
                    bool firstTime = _boneEverValid != null && !_boneEverValid[b];
                    int count = meshList.Count;
                    for (int k = 0; k < count; k++)
                    {
                        float t = (k + 1) * _currentBoneTrailStep;
                        Vector3 p = Vector3.Lerp(pA, pC, t);
                        var tr = meshList[k];
                        if (tr == null) continue;
                        int frame = Time.frameCount;
                        if (firstTime)
                        {
                            tr.Clear();
                            tr.AddSample(now, p, frame);
                        }
                        else
                        {
                            tr.AddSample(now, p, frame);
                        }
                    }
                    if (firstTime && _boneEverValid != null) _boneEverValid[b] = true;
                }
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
        /// tweaks (joint radius, bone width, trail step / width / duration / color
        /// mode, frame-hue params) reflect immediately instead of waiting for
        /// unpause. Does not touch _jointPositions, _jointValid, or trail sample
        /// buffers — trail timestamps remain frozen because Time.timeAsDouble is
        /// frozen during pause, so no samples expire.
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

            // Step change destroys+recreates bone-interp trails; the empty array
            // is fine — new samples won't arrive until play resumes.
            EnsureBoneTrails(cfg.BoneTrailStep);

            ApplyTrailParams(cfg.ShowTrails, cfg.TrailDuration, cfg.TrailWidth,
                             cfg.TrailColorMode, cfg.TrailFlatColor,
                             cfg.AccelMin, cfg.AccelMax, cfg.AccelHotColor,
                             cfg.FrameHue);
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

        /// <summary>Append this body's windowed per-joint trail centerline (the same samples
        /// the JointTrailMesh ribbon draws) for <paramref name="jointIdx"/> to
        /// <paramref name="outWorld"/>, in WORLD space (oldest→newest). Returns the number
        /// appended. Used by the SDF trail baker to bake the ribbon as capsules.</summary>
        public int CopyTrailWorldPoints(int jointIdx, List<Vector3> outWorld)
        {
            if (_trails == null || jointIdx < 0 || jointIdx >= _trails.Length) return 0;
            var tr = _trails[jointIdx];
            if (tr == null) return 0;
            int n = tr.SampleCount;
            for (int i = 0; i < n; i++)
                outWorld.Add(WorldOf(tr.SampleLocalPos(i)));
            return n;
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
            DestroyBoneTrails();
            if (_bonesMesh != null) Object.Destroy(_bonesMesh);
            if (_root != null) Object.Destroy(_root);
        }

        // Rebuild every joint's trail mesh against the given camera (for billboard
        // orientation). Called once per frame by the owning MonoBehaviour after the
        // skeleton update so vertex positions / colors are fresh.
        public void TickTrails(Camera cam, double trailNow)
        {
            double now = trailNow;
            if (_trails != null)
            {
                for (int i = 0; i < _trails.Length; i++)
                    if (_trails[i] != null) _trails[i].Rebuild(now, cam);
            }
            if (_boneTrails != null)
            {
                for (int b = 0; b < _boneTrails.Length; b++)
                {
                    var list = _boneTrails[b];
                    if (list == null) continue;
                    for (int k = 0; k < list.Count; k++)
                        if (list[k] != null) list[k].Rebuild(now, cam);
                }
            }
        }

        /// <summary>Feed every joint's latest |a| into <paramref name="sink"/> so the
        /// pool can build a rolling window across all visible bodies / joints.</summary>
        public void CollectLatestAccels(List<float> sink)
        {
            if (_trails != null)
                for (int i = 0; i < _trails.Length; i++)
                    if (_trails[i] != null) sink.Add(_trails[i].LastAccel);
            if (_boneTrails != null)
            {
                for (int b = 0; b < _boneTrails.Length; b++)
                {
                    var list = _boneTrails[b];
                    if (list == null) continue;
                    for (int k = 0; k < list.Count; k++)
                        if (list[k] != null) sink.Add(list[k].LastAccel);
                }
            }
        }

        /// <summary>Override the trail accelMax for every joint of this body. Used
        /// when autoAccelMax is on; the pool computes one rolling p95 and pushes it
        /// here so all trails share the same hot-end calibration.</summary>
        public void SetTrailAccelMax(float v)
        {
            if (_trails != null)
                for (int i = 0; i < _trails.Length; i++)
                    if (_trails[i] != null) _trails[i].SetAccelMax(v);
            if (_boneTrails != null)
            {
                for (int b = 0; b < _boneTrails.Length; b++)
                {
                    var list = _boneTrails[b];
                    if (list == null) continue;
                    for (int k = 0; k < list.Count; k++)
                        if (list[k] != null) list[k].SetAccelMax(v);
                }
            }
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

        // Single shared material for every JointTrailMesh. Prefers TrailLit
        // (URP PBR + per-vertex color tint + ShadowCaster + DepthNormals so
        // SSAO from the PC_Renderer feature attaches automatically) so the
        // trail catches scene lighting and reads as a 3D shape instead of a
        // flat overlay. Falls back to TrailOverlay (the old unlit alpha-blend
        // path) if TrailLit failed to compile, then to URP particle / sprite
        // shaders as a last resort. The per-vertex acceleration heatmap is
        // preserved through the chain — TrailLit multiplies it into the
        // PBR albedo, TrailOverlay passes it through unlit.
        private static Material s_trailMat;
        private static Material ResolveTrailMaterial()
        {
            if (s_trailMat != null) return s_trailMat;
            s_trailMat = new Material(ResolveTrailLitShader()) { name = "BT_Trail" };
            SetMaterialColor(s_trailMat, Color.white);
            return s_trailMat;
        }

        // Shared shader resolution chain for TrailLit-shaded geometry (per-joint
        // trails AND bone tubes). Each caller still allocates its own Material
        // instance: bones write skeletonColor into _BaseColor with white vertex
        // colors, while the trail singleton keeps _BaseColor=white and uses the
        // per-vertex heatmap, so the two can't share one Material.
        private static Shader ResolveTrailLitShader()
        {
            Shader s = Shader.Find("BodyTracking/TrailLit");
            if (s == null) s = Shader.Find("BodyTracking/TrailOverlay");
            if (s == null) s = Shader.Find("Universal Render Pipeline/Particles/Unlit");
            if (s == null) s = Shader.Find("Sprites/Default");
            return s;
        }

        private static Color TrailColorFor(int jointIndex, uint bodyId, BodyTrackingShared.TrailColorMode mode, Color flat,
                                            BodyTrackingShared.FrameHueParams frameHue)
        {
            // AccelHeatmap mode treats flat as the cold (base) end; per-vertex
            // interpolation toward hot is done inside JointTrailMesh. FrameHue
            // is also resolved per-vertex inside JointTrailMesh from each
            // sample's stored frame counter — only flat.a (alpha multiplier)
            // matters here, the RGB is overwritten in Rebuild.
            if (mode == BodyTrackingShared.TrailColorMode.FlatColor ||
                mode == BodyTrackingShared.TrailColorMode.AccelHeatmap ||
                mode == BodyTrackingShared.TrailColorMode.FrameHue) return flat;
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

        private static JointTrailMesh.ColorMode ToJointTrailMode(BodyTrackingShared.TrailColorMode mode)
        {
            if (mode == BodyTrackingShared.TrailColorMode.AccelHeatmap) return JointTrailMesh.ColorMode.AccelHeatmap;
            if (mode == BodyTrackingShared.TrailColorMode.FrameHue) return JointTrailMesh.ColorMode.FrameHue;
            return JointTrailMesh.ColorMode.Base;
        }

        // Live-tweakable trail params from the Inspector (no need to re-spawn visuals).
        public void ApplyTrailParams(bool show, float duration, float width,
                                      BodyTrackingShared.TrailColorMode mode, Color flat,
                                      float accelMin, float accelMax, Color accelHotColor,
                                      BodyTrackingShared.FrameHueParams frameHue)
        {
            JointTrailMesh.ColorMode jmMode = ToJointTrailMode(mode);
            if (_trails != null)
            {
                for (int i = 0; i < _trails.Length; i++)
                {
                    var tr = _trails[i];
                    if (tr == null) continue;
                    var baseColor = TrailColorFor(i, _bodyId, mode, flat, frameHue);
                    tr.Configure(show, duration, width, baseColor, accelHotColor,
                        accelMin, accelMax, jmMode, frameHue);
                }
            }
            if (_boneTrails != null)
            {
                for (int b = 0; b < _boneTrails.Length; b++)
                {
                    var list = _boneTrails[b];
                    if (list == null) continue;
                    // Color each bone-interp trail from its bone's start-joint hue
                    // so PerJointHue keeps a recognizable gradient along each limb.
                    int colorJointIdx = (int)BodyTrackingShared.Bones[b].a;
                    var baseColor = TrailColorFor(colorJointIdx, _bodyId, mode, flat, frameHue);
                    for (int k = 0; k < list.Count; k++)
                    {
                        var tr = list[k];
                        if (tr == null) continue;
                        tr.Configure(show, duration, width, baseColor, accelHotColor,
                            accelMin, accelMax, jmMode, frameHue);
                    }
                }
            }
        }

        // Build (or rebuild) the per-bone interp-trail array to match the given
        // parametric step. step <= 0 destroys the array (feature off). Trails
        // are configured with default values; the caller (BodyVisualPool.Apply)
        // is expected to invoke ApplyTrailParams immediately after, which writes
        // the real show/duration/width/color. Same JointTrailMesh material is
        // shared with the per-joint trails so the heatmap palette renders the
        // same way for both.
        private void EnsureBoneTrails(float step)
        {
            // Treat NaN / negatives as off.
            if (!(step > 0f)) step = 0f;
            if (Mathf.Approximately(step, _currentBoneTrailStep)) return;

            DestroyBoneTrails();
            _currentBoneTrailStep = step;
            if (step <= 0f) return;

            int boneCount = BodyTrackingShared.Bones.Length;
            int interpCount = Mathf.Min(Mathf.FloorToInt((1f - 1e-6f) / step), 64);
            if (interpCount <= 0) return;

            var mat = ResolveTrailMaterial();
            _boneTrails = new List<JointTrailMesh>[boneCount];
            _boneEverValid = new bool[boneCount];
            for (int b = 0; b < boneCount; b++)
            {
                var list = new List<JointTrailMesh>(interpCount);
                var bone = BodyTrackingShared.Bones[b];
                for (int k = 0; k < interpCount; k++)
                {
                    string name = $"BoneTrail_{bone.a}_{bone.b}_{k}";
                    list.Add(new JointTrailMesh(_root.transform, name, mat));
                }
                _boneTrails[b] = list;
            }
        }

        private void DestroyBoneTrails()
        {
            if (_boneTrails == null) return;
            for (int b = 0; b < _boneTrails.Length; b++)
            {
                var list = _boneTrails[b];
                if (list == null) continue;
                for (int k = 0; k < list.Count; k++)
                    if (list[k] != null) list[k].Destroy();
            }
            _boneTrails = null;
            _boneEverValid = null;
            _currentBoneTrailStep = 0f;
        }
    }
}
