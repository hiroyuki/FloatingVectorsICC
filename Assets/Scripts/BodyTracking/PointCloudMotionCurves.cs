// Point-cloud motion curves (Phase 2): from each seeded point-cloud vertex,
// grow a curve that traces where that point-of-the-body was over the last K
// frames, producing a "motion sculpture" effect. Seeds are classified to the
// nearest bone, parameterised in that bone's stable frame (t, cv, cw), and
// reprojected through BonePoseHistory's world pose ring by MotionCurvesBuild.
//
// GPU path, fully real-time: seeds are read straight out of the point-cloud
// mesh vertex buffers (Raw target, ObColorPoint 24B) and curves are drawn with
// Graphics.DrawProceduralIndirect + MotionCurves.shader. Nothing is read back.
//
// Lives in the BodyTracking assembly (not PointCloud) because it needs both
// BonePoseHistory (BodyTracking) and the point-cloud meshes (PointCloud), and
// BodyTracking already references PointCloud (the reverse would be circular).
//
// Consumed in Update, one frame behind BonePoseHistory's LateUpdate publish —
// the same one-frame lag the other body->cloud feeders accept.

using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering;

namespace BodyTracking
{
    [DisallowMultipleComponent]
    public class PointCloudMotionCurves : MonoBehaviour
    {
        [Tooltip("Bone pose history source. Auto-resolves the first BonePoseHistory at OnEnable.")]
        public BonePoseHistory history;

        [Header("Seed source")]
        [Tooltip("Explicit point-cloud MeshFilters to seed from. If empty, all _Playback_* meshes " +
                 "in the scene are used automatically (playback path).")]
        public List<MeshFilter> sourceMeshes = new List<MeshFilter>();

        [Tooltip("When sourceMeshes is empty, auto-collect MeshFilters whose GameObject name starts " +
                 "with this prefix each frame (the recorder's per-device playback meshes).")]
        public string autoSourcePrefix = "_Playback_";

        [Header("Seeds / curves")]
        [Min(64)]
        [Tooltip("Total seed count (buffer capacity). Spread evenly across source meshes.")]
        public int seedCount = 20000;

        [Range(0.2f, 4f)]
        [Tooltip("Global multiplier on the standard per-bone radii — absorbs body-size differences.")]
        public float radiusScale = 1f;

        [Range(0f, 0.3f)]
        [Tooltip("Cull a seed as background if its nearest bone surface distance exceeds this (m).")]
        public float surfaceMargin = 0.05f;

        [Range(1, 4)]
        [Tooltip("How many nearest bones each seed blends (skinning-style). 1 = hard nearest-bone " +
                 "assignment (sharp boundaries); 3 smooths bone joints.")]
        public int boneBlendCount = 3;

        [Range(0.005f, 0.2f)]
        [Tooltip("Blend falloff sigma (m). Smaller = sharper (closer to nearest-bone); larger = softer, " +
                 "wider blend across the boundary.")]
        public float blendSharpness = 0.02f;

        [Range(0f, 3f)]
        public float brightness = 1f;

        [Header("Freeze")]
        [Tooltip("Hard hold: stop rebuilding and keep drawing the last curves, ignoring parameter and " +
                 "pose changes. For holding the sculpture during live play. NOTE: pausing playback " +
                 "already holds the pose (via BonePoseHistory) while still letting you retune params — " +
                 "leave this off if you want to compare blend settings on a paused frame.")]
        public bool freeze = false;

        [Tooltip("Coordinate magnitude (m) above which a source vertex is treated as an invalid-depth " +
                 "dummy and skipped (playback reconstruct emits 1e10 for holes).")]
        public float sanityRange = 100f;

        // --- GPU ---
        private ComputeShader _shader;
        private int _kernel = -1;
        private Material _mat;
        private GraphicsBuffer _outBuf;      // LineVert[seedCount*(K-1)*2]
        private GraphicsBuffer _argsBuf;     // 4-uint indirect args
        private GraphicsBuffer _radiusABuf;  // float[boneCount]
        private GraphicsBuffer _radiusBBuf;
        private GraphicsBuffer _colorBuf;    // float3[boneCount]
        private int _boneCount;
        private int _ringLen;
        private int _capacity;               // seedCount the buffers were sized for
        private bool _hasBuilt;              // _outBuf holds at least one real (non-empty) build
        private readonly List<MeshFilter> _srcScratch = new List<MeshFilter>();

        // Cached property ids.
        private static readonly int kSrc = Shader.PropertyToID("_Src");
        private static readonly int kHistory = Shader.PropertyToID("_History");
        private static readonly int kBoneCounts = Shader.PropertyToID("_BoneCounts");
        private static readonly int kRadiusA = Shader.PropertyToID("_BoneRadiusA");
        private static readonly int kRadiusB = Shader.PropertyToID("_BoneRadiusB");
        private static readonly int kBoneColor = Shader.PropertyToID("_BoneColor");
        private static readonly int kOut = Shader.PropertyToID("_Out");
        private static readonly int kSrcL2W = Shader.PropertyToID("_SrcLocalToWorld");
        private static readonly int kSrcCount = Shader.PropertyToID("_SrcCount");
        private static readonly int kSeedBase = Shader.PropertyToID("_SeedBase");
        private static readonly int kSeedCount = Shader.PropertyToID("_SeedCount");
        private static readonly int kBoneCount = Shader.PropertyToID("_BoneCount");
        private static readonly int kRingLen = Shader.PropertyToID("_RingLen");
        private static readonly int kRadiusScale = Shader.PropertyToID("_RadiusScale");
        private static readonly int kSurfaceMargin = Shader.PropertyToID("_SurfaceMargin");
        private static readonly int kSanityRange = Shader.PropertyToID("_SanityRange");
        private static readonly int kBlendCount = Shader.PropertyToID("_BlendCount");
        private static readonly int kBlendSigma = Shader.PropertyToID("_BlendSigma");
        private static readonly int kVerts = Shader.PropertyToID("_Verts");
        private static readonly int kBrightness = Shader.PropertyToID("_Brightness");

        private void OnEnable()
        {
            if (history == null) history = FindFirstObjectByType<BonePoseHistory>();
            if (_shader == null)
            {
                _shader = Resources.Load<ComputeShader>("MotionCurvesBuild");
                if (_shader != null) _kernel = _shader.FindKernel("CSBuild");
            }
            if (_mat == null)
            {
                var sh = Shader.Find("Orbbec/MotionCurves");
                if (sh != null) _mat = new Material(sh) { name = "MotionCurves (auto)", hideFlags = HideFlags.DontSave };
            }
        }

        private void OnDisable() => ReleaseBuffers();
        private void OnDestroy()
        {
            ReleaseBuffers();
            if (_mat != null) { DestroyImmediate(_mat); _mat = null; }
        }

        private void ReleaseBuffers()
        {
            _outBuf?.Release(); _outBuf = null;
            _argsBuf?.Release(); _argsBuf = null;
            _radiusABuf?.Release(); _radiusABuf = null;
            _radiusBBuf?.Release(); _radiusBBuf = null;
            _colorBuf?.Release(); _colorBuf = null;
            _capacity = 0; _ringLen = 0;
            _hasBuilt = false;
        }

        private void Update()
        {
            if (_shader == null || _kernel < 0 || _mat == null) return;

            // Manual hard hold: keep drawing the last built curves without rebuilding (ignores params).
            // Pausing playback does NOT take this path — BonePoseHistory holds the pose while paused, so
            // rebuilding stays stable yet still responds to parameter tweaks.
            if (freeze)
            {
                if (_outBuf != null && _hasBuilt) DrawCurves();
                return;
            }

            if (history == null) return;
            var histBuf = history.HistoryBuffer;
            var countBuf = history.CountBuffer;
            if (histBuf == null || countBuf == null) return;

            int boneCount = history.BoneCount;
            int ringLen = history.RingLength;
            if (boneCount <= 0 || ringLen < 2) return;

            EnsureBuffers(boneCount, ringLen);

            // Resolve source meshes for this frame.
            var sources = ResolveSources();
            if (sources.Count == 0) return; // nothing to seed from -> draw nothing

            int cap = _capacity;
            int nSrc = sources.Count;
            int seedBase = 0;

            _shader.SetBuffer(_kernel, kHistory, histBuf);
            _shader.SetBuffer(_kernel, kBoneCounts, countBuf);
            _shader.SetBuffer(_kernel, kRadiusA, _radiusABuf);
            _shader.SetBuffer(_kernel, kRadiusB, _radiusBBuf);
            _shader.SetBuffer(_kernel, kBoneColor, _colorBuf);
            _shader.SetBuffer(_kernel, kOut, _outBuf);
            _shader.SetInt(kBoneCount, boneCount);
            _shader.SetInt(kRingLen, ringLen);
            _shader.SetFloat(kRadiusScale, radiusScale);
            _shader.SetFloat(kSurfaceMargin, surfaceMargin);
            _shader.SetFloat(kSanityRange, sanityRange);
            _shader.SetInt(kBlendCount, Mathf.Clamp(boneBlendCount, 1, 4));
            _shader.SetFloat(kBlendSigma, blendSharpness);

            var borrowed = new List<GraphicsBuffer>(nSrc);
            for (int i = 0; i < nSrc; i++)
            {
                var mf = sources[i];
                var mesh = mf.sharedMesh;
                var vb = mesh.GetVertexBuffer(0);
                if (vb == null) continue;
                borrowed.Add(vb);

                // Last source takes the remainder so every slot [0,cap) is written.
                int seedsForThis = (i == nSrc - 1) ? (cap - seedBase) : (cap / nSrc);
                if (seedsForThis <= 0) { vb.Dispose(); borrowed.RemoveAt(borrowed.Count - 1); continue; }

                _shader.SetBuffer(_kernel, kSrc, vb);
                _shader.SetMatrix(kSrcL2W, mf.transform.localToWorldMatrix);
                _shader.SetInt(kSrcCount, mesh.vertexCount);
                _shader.SetInt(kSeedBase, seedBase);
                _shader.SetInt(kSeedCount, seedsForThis);
                int groups = (seedsForThis + 63) / 64;
                _shader.Dispatch(_kernel, groups, 1, 1);
                seedBase += seedsForThis;
            }

            foreach (var vb in borrowed) vb.Dispose();
            if (borrowed.Count == 0) return;

            _hasBuilt = true;
            DrawCurves();
        }

        private void DrawCurves()
        {
            _mat.SetBuffer(kVerts, _outBuf);
            _mat.SetFloat(kBrightness, brightness);
            var bounds = new Bounds(Vector3.zero, Vector3.one * 50f);
            Graphics.DrawProceduralIndirect(_mat, bounds, MeshTopology.Lines, _argsBuf, 0,
                camera: null, properties: null,
                castShadows: ShadowCastingMode.Off, receiveShadows: false,
                layer: gameObject.layer);
        }

        private List<MeshFilter> ResolveSources()
        {
            _srcScratch.Clear();
            if (sourceMeshes != null && sourceMeshes.Count > 0)
            {
                foreach (var mf in sourceMeshes)
                    if (mf != null && mf.sharedMesh != null && mf.sharedMesh.vertexCount > 0)
                        _srcScratch.Add(mf);
                return _srcScratch;
            }
            // Auto: per-device playback meshes.
            var all = FindObjectsByType<MeshFilter>(FindObjectsSortMode.None);
            foreach (var mf in all)
            {
                if (mf == null || mf.sharedMesh == null || mf.sharedMesh.vertexCount == 0) continue;
                if (!string.IsNullOrEmpty(autoSourcePrefix) && !mf.gameObject.name.StartsWith(autoSourcePrefix)) continue;
                _srcScratch.Add(mf);
            }
            return _srcScratch;
        }

        private void EnsureBuffers(int boneCount, int ringLen)
        {
            int cap = Mathf.Max(64, seedCount);
            if (_outBuf != null && _capacity == cap && _ringLen == ringLen && _boneCount == boneCount) return;

            ReleaseBuffers();
            _boneCount = boneCount;
            _ringLen = ringLen;
            _capacity = cap;

            int lineVerts = cap * (ringLen - 1) * 2;
            _outBuf = new GraphicsBuffer(GraphicsBuffer.Target.Structured, lineVerts, sizeof(float) * 6); // LineVert
            _argsBuf = new GraphicsBuffer(GraphicsBuffer.Target.IndirectArguments, 4, sizeof(uint));
            _argsBuf.SetData(new uint[] { (uint)lineVerts, 1, 0, 0 });

            // Standard per-bone radii (absolute m) + endpoint taper; static, scaled at runtime.
            var bones = BodyTrackingShared.Bones;
            var rA = new float[boneCount];
            var rB = new float[boneCount];
            var col = new Vector3[boneCount];
            for (int b = 0; b < boneCount; b++)
            {
                DefaultRadius(bones[b].a, bones[b].b, out rA[b], out rB[b]);
                Color c = HueFor(b, boneCount);
                col[b] = new Vector3(c.r, c.g, c.b);
            }
            _radiusABuf = new GraphicsBuffer(GraphicsBuffer.Target.Structured, boneCount, sizeof(float));
            _radiusBBuf = new GraphicsBuffer(GraphicsBuffer.Target.Structured, boneCount, sizeof(float));
            _colorBuf = new GraphicsBuffer(GraphicsBuffer.Target.Structured, boneCount, sizeof(float) * 3);
            _radiusABuf.SetData(rA);
            _radiusBBuf.SetData(rB);
            _colorBuf.SetData(col);
        }

        // Standard bone radii (m), parent side (a) -> child side (b). Categorised by the joint pair.
        private static void DefaultRadius(k4abt_joint_id_t a, k4abt_joint_id_t b, out float ra, out float rb)
        {
            // torso / spine
            if (IsSpine(a) && IsSpine(b)) { ra = 0.18f; rb = 0.16f; return; }
            // clavicle
            if (b == k4abt_joint_id_t.K4ABT_JOINT_CLAVICLE_LEFT || b == k4abt_joint_id_t.K4ABT_JOINT_CLAVICLE_RIGHT) { ra = 0.09f; rb = 0.07f; return; }
            // upper arm
            if (b == k4abt_joint_id_t.K4ABT_JOINT_SHOULDER_LEFT || b == k4abt_joint_id_t.K4ABT_JOINT_SHOULDER_RIGHT) { ra = 0.07f; rb = 0.06f; return; }
            if (b == k4abt_joint_id_t.K4ABT_JOINT_ELBOW_LEFT || b == k4abt_joint_id_t.K4ABT_JOINT_ELBOW_RIGHT) { ra = 0.06f; rb = 0.05f; return; }
            // forearm -> wrist
            if (b == k4abt_joint_id_t.K4ABT_JOINT_WRIST_LEFT || b == k4abt_joint_id_t.K4ABT_JOINT_WRIST_RIGHT) { ra = 0.05f; rb = 0.04f; return; }
            // head cluster
            if (b == k4abt_joint_id_t.K4ABT_JOINT_HEAD || a == k4abt_joint_id_t.K4ABT_JOINT_HEAD
                || b == k4abt_joint_id_t.K4ABT_JOINT_NOSE) { ra = 0.09f; rb = 0.08f; return; }
            // hips (pelvis -> hip)
            if (b == k4abt_joint_id_t.K4ABT_JOINT_HIP_LEFT || b == k4abt_joint_id_t.K4ABT_JOINT_HIP_RIGHT) { ra = 0.12f; rb = 0.10f; return; }
            // thigh -> knee
            if (b == k4abt_joint_id_t.K4ABT_JOINT_KNEE_LEFT || b == k4abt_joint_id_t.K4ABT_JOINT_KNEE_RIGHT) { ra = 0.10f; rb = 0.08f; return; }
            // shin -> ankle
            if (b == k4abt_joint_id_t.K4ABT_JOINT_ANKLE_LEFT || b == k4abt_joint_id_t.K4ABT_JOINT_ANKLE_RIGHT) { ra = 0.07f; rb = 0.05f; return; }
            // foot
            if (b == k4abt_joint_id_t.K4ABT_JOINT_FOOT_LEFT || b == k4abt_joint_id_t.K4ABT_JOINT_FOOT_RIGHT) { ra = 0.05f; rb = 0.04f; return; }
            ra = 0.06f; rb = 0.05f;
        }

        private static bool IsSpine(k4abt_joint_id_t j)
        {
            switch (j)
            {
                case k4abt_joint_id_t.K4ABT_JOINT_PELVIS:
                case k4abt_joint_id_t.K4ABT_JOINT_SPINE_NAVEL:
                case k4abt_joint_id_t.K4ABT_JOINT_SPINE_CHEST:
                case k4abt_joint_id_t.K4ABT_JOINT_NECK:
                    return true;
                default: return false;
            }
        }

        // Golden-ratio hue, matching TSDFTrailBaker.HueFor's look.
        private static Color HueFor(int index, int total)
        {
            float h = total > 0 ? Mathf.Repeat(index * 0.61803398875f, 1f) : 0f;
            return Color.HSVToRGB(h, 0.85f, 1f);
        }
    }
}
