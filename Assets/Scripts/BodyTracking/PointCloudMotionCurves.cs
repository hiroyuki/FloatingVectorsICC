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
    public class PointCloudMotionCurves : MonoBehaviour, global::Shared.IViewToggle
    {
        [Tooltip("Show/hide the motion curves. Exposed in the unified Views panel as \"Motion lines\", " +
                 "independent of the BT skeleton toggle.")]
        public bool visible = true;

        // ---- Shared.IViewToggle (unified Views panel) ----
        public string ViewLabel => "Motion lines";
        public bool Visible { get => visible; set => visible = value; }

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

        [Range(0f, 0.6f)]
        [Tooltip("Padding (m) added around the bone AABB when collecting seed candidates. Seeds are " +
                 "restricted to this box so the whole budget lands on the body, not the background. " +
                 "Should comfortably exceed the thickest bone radius + surfaceMargin.")]
        public float bboxPadding = 0.25f;

        [Range(1, 4)]
        [Tooltip("How many nearest bones each seed blends (skinning-style). 1 = hard nearest-bone " +
                 "assignment (sharp boundaries); 3 smooths bone joints.")]
        public int boneBlendCount = 3;

        [Range(0.005f, 0.2f)]
        [Tooltip("Blend falloff sigma (m). Smaller = sharper (closer to nearest-bone); larger = softer, " +
                 "wider blend across the boundary.")]
        public float blendSharpness = 0.02f;

        [Range(1, 8)]
        [Tooltip("Catmull-Rom subdivisions per control-point interval. 1 = straight polyline; higher = " +
                 "smoother curves (multiplies the vertex count / VRAM by this factor).")]
        public int smoothSubdiv = 4;

        [Range(0f, 3f)]
        [Tooltip("Overall brightness multiplier on the curves' original point-cloud colour.")]
        public float brightness = 1f;

        [Range(0f, 0.05f)]
        [Tooltip("Ribbon width (m). Each curve is drawn as a camera-facing ribbon of this width; " +
                 "0 collapses to a thin line. Wider ribbons make the smoothing / colour read clearly.")]
        public float ribbonWidth = 0.006f;

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
        private int _collectKernel = -1;
        private int _buildKernel = -1;
        private int _emitKernel = -1;   // CSEmitSegs (print export)
        private Material _mat;
        private GraphicsBuffer _outBuf;      // LineVert[seedCount*(K-1)*2]
        private GraphicsBuffer _argsBuf;     // 4-uint indirect args
        private GraphicsBuffer _radiusABuf;  // float[boneCount]
        private GraphicsBuffer _radiusBBuf;
        private GraphicsBuffer _collectBuf;      // SeedPoint[totalSourceVerts] compacted in-bbox points
        private GraphicsBuffer _collectCounter;  // uint[1]
        private int _collectCap;                 // _collectBuf capacity (total source verts sized)
        private GraphicsBuffer _segStatsBuf;     // uint[3] print-emit stats (appends / dropped / bridges skipped)
        private static readonly uint[] s_counterReset = { 0u };
        private static readonly uint[] s_segStatsReset = { 0u, 0u, 0u };
        private int _boneCount;
        private int _ringLen;
        private int _subdiv;                  // Catmull-Rom subdiv the output buffer was sized for
        private int _capacity;               // seedCount the buffers were sized for
        private bool _hasBuilt;              // _outBuf holds at least one real (non-empty) build
        private readonly List<MeshFilter> _srcScratch = new List<MeshFilter>();

        // Cached property ids.
        private static readonly int kSrc = Shader.PropertyToID("_Src");
        private static readonly int kHistory = Shader.PropertyToID("_History");
        private static readonly int kBoneCounts = Shader.PropertyToID("_BoneCounts");
        private static readonly int kRadiusA = Shader.PropertyToID("_BoneRadiusA");
        private static readonly int kRadiusB = Shader.PropertyToID("_BoneRadiusB");
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
        private static readonly int kCurveSamples = Shader.PropertyToID("_CurveSamples");
        private static readonly int kSubdiv = Shader.PropertyToID("_Subdiv");
        private static readonly int kCollectOut = Shader.PropertyToID("_CollectOut");
        private static readonly int kCollectCounter = Shader.PropertyToID("_CollectCounter");
        private static readonly int kCollectCap = Shader.PropertyToID("_CollectCap");
        private static readonly int kBboxMin = Shader.PropertyToID("_BboxMin");
        private static readonly int kBboxMax = Shader.PropertyToID("_BboxMax");
        private static readonly int kVerts = Shader.PropertyToID("_Verts");
        private static readonly int kBrightness = Shader.PropertyToID("_Brightness");
        private static readonly int kWidth = Shader.PropertyToID("_Width");
        private static readonly int kSegsOut = Shader.PropertyToID("_SegsOut");
        private static readonly int kSegStats = Shader.PropertyToID("_SegStats");
        private static readonly int kSegCap = Shader.PropertyToID("_SegCap");
        private static readonly int kPrintSeedStride = Shader.PropertyToID("_PrintSeedStride");
        private static readonly int kPrintRadius = Shader.PropertyToID("_PrintRadius");
        private static readonly int kBridgeRadiusScale = Shader.PropertyToID("_BridgeRadiusScale");
        private static readonly int kMaxBridgeLength = Shader.PropertyToID("_MaxBridgeLength");

        private void OnEnable()
        {
            if (history == null) history = FindFirstObjectByType<BonePoseHistory>();
            if (_shader == null)
            {
                _shader = Resources.Load<ComputeShader>("MotionCurvesBuild");
                if (_shader != null)
                {
                    _collectKernel = _shader.FindKernel("CSCollect");
                    _buildKernel = _shader.FindKernel("CSBuild");
                    _emitKernel = _shader.FindKernel("CSEmitSegs");
                }
            }
            if (_mat == null)
            {
                // Load from Resources so the shader survives player-build stripping (nothing else
                // references it); fall back to Shader.Find in case it is registered another way.
                var sh = Resources.Load<Shader>("MotionCurves");
                if (sh == null) sh = Shader.Find("Orbbec/MotionCurves");
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
            _collectBuf?.Release(); _collectBuf = null;
            _collectCounter?.Release(); _collectCounter = null;
            _segStatsBuf?.Release(); _segStatsBuf = null;
            _capacity = 0; _ringLen = 0; _subdiv = 0; _collectCap = 0;
            _hasBuilt = false;
        }

        private void Update()
        {
            if (_shader == null || _collectKernel < 0 || _buildKernel < 0 || _mat == null) return;
            if (!visible) return; // hidden via the Views panel -> don't build or draw this frame

            // Manual hard hold: keep drawing the last built curves without rebuilding (ignores params).
            // Pausing playback does NOT take this path — BonePoseHistory holds the pose while paused, so
            // rebuilding stays stable yet still responds to parameter tweaks.
            if (freeze)
            {
                if (_outBuf != null && _hasBuilt) DrawCurves();
                return;
            }

            if (!CollectSeeds(out _)) return;

            // --- build: one dispatch, seeds strided over the compacted points ---
            BindBuildParams(_buildKernel);
            _shader.SetBuffer(_buildKernel, kOut, _outBuf);
            _shader.Dispatch(_buildKernel, (_capacity + 63) / 64, 1, 1);

            _hasBuilt = true;
            DrawCurves();
        }

        // Shared front half of Update() and EmitPrintSegs(): validates inputs,
        // resolves sources, (re)sizes buffers and runs the CSCollect prepass.
        // Returns false with a human-readable reason when there is nothing to
        // build from this frame.
        private bool CollectSeeds(out string reason)
        {
            reason = null;
            if (_shader == null || _collectKernel < 0 || _buildKernel < 0)
            { reason = "MotionCurvesBuild compute shader not loaded"; return false; }
            if (history == null) { reason = "no BonePoseHistory in the scene"; return false; }
            var histBuf = history.HistoryBuffer;
            var countBuf = history.CountBuffer;
            if (histBuf == null || countBuf == null)
            { reason = "BonePoseHistory buffers not ready (no pose ingested yet)"; return false; }

            int boneCount = history.BoneCount;
            int ringLen = history.RingLength;
            if (boneCount <= 0 || ringLen < 2) { reason = "no bones / history ring too short"; return false; }
            int subdiv = Mathf.Clamp(smoothSubdiv, 1, 8);

            // Resolve source meshes and size the collect buffer to their total vertex count.
            var sources = ResolveSources();
            if (sources.Count == 0) { reason = "no usable point-cloud source meshes"; return false; }
            int totalSrcVerts = 0;
            foreach (var mf in sources) totalSrcVerts += mf.sharedMesh.vertexCount;

            EnsureBuffers(boneCount, ringLen, subdiv, totalSrcVerts);

            // Seed bounding box: the bone AABB grown by padding, so the prepass keeps only body points.
            // No pose yet -> fall back to an all-encompassing box (behaves like the old whole-cloud seed).
            Vector3 bmin, bmax;
            if (history.TryGetWorldBounds(out var bb))
            {
                Vector3 pad = Vector3.one * bboxPadding;
                bmin = bb.min - pad; bmax = bb.max + pad;
            }
            else { bmin = Vector3.one * -1e9f; bmax = Vector3.one * 1e9f; }

            // --- prepass: compact in-bbox valid points from every source into _collectBuf ---
            _collectCounter.SetData(s_counterReset);
            _shader.SetBuffer(_collectKernel, kCollectOut, _collectBuf);
            _shader.SetBuffer(_collectKernel, kCollectCounter, _collectCounter);
            _shader.SetInt(kCollectCap, _collectCap);
            _shader.SetVector(kBboxMin, bmin);
            _shader.SetVector(kBboxMax, bmax);
            _shader.SetFloat(kSanityRange, sanityRange);
            var borrowed = new List<GraphicsBuffer>(sources.Count);
            foreach (var mf in sources)
            {
                var mesh = mf.sharedMesh;
                var vb = mesh.GetVertexBuffer(0);
                if (vb == null) continue;
                borrowed.Add(vb);
                _shader.SetBuffer(_collectKernel, kSrc, vb);
                _shader.SetMatrix(kSrcL2W, mf.transform.localToWorldMatrix);
                _shader.SetInt(kSrcCount, mesh.vertexCount);
                _shader.Dispatch(_collectKernel, (mesh.vertexCount + 63) / 64, 1, 1);
            }
            foreach (var vb in borrowed) vb.Dispose();
            if (borrowed.Count == 0) { reason = "source mesh vertex buffers unavailable"; return false; }
            return true;
        }

        // Bind the collect/history/radius inputs + classify/reproject scalars shared
        // by CSBuild and CSEmitSegs. CollectSeeds() must have succeeded this frame.
        private void BindBuildParams(int kernel)
        {
            _shader.SetBuffer(kernel, kCollectOut, _collectBuf);
            _shader.SetBuffer(kernel, kCollectCounter, _collectCounter);
            _shader.SetInt(kCollectCap, _collectCap);
            _shader.SetBuffer(kernel, kHistory, history.HistoryBuffer);
            _shader.SetBuffer(kernel, kBoneCounts, history.CountBuffer);
            _shader.SetBuffer(kernel, kRadiusA, _radiusABuf);
            _shader.SetBuffer(kernel, kRadiusB, _radiusBBuf);
            _shader.SetInt(kBoneCount, _boneCount);
            _shader.SetInt(kRingLen, _ringLen);
            _shader.SetFloat(kRadiusScale, radiusScale);
            _shader.SetFloat(kSurfaceMargin, surfaceMargin);
            _shader.SetInt(kBlendCount, Mathf.Clamp(boneBlendCount, 1, 4));
            _shader.SetFloat(kBlendSigma, blendSharpness);
            _shader.SetInt(kCurveSamples, history.CurveSamples);
            _shader.SetInt(kSubdiv, _subdiv);
            _shader.SetInt(kSeedBase, 0);
            _shader.SetInt(kSeedCount, _capacity);
        }

        // ---- Print export (TSDFPrintExporter) ----

        /// <summary>Stats returned by <see cref="EmitPrintSegs"/>.</summary>
        public struct PrintSegStats
        {
            public int emitted;         // segments actually written to the buffer
            public int dropped;         // appends lost to capacity overflow
            public int bridgesSkipped;  // bridges over maxBridgeLength (misclassified seeds)
        }

        /// <summary>Worst-case segment count <see cref="EmitPrintSegs"/> can produce for
        /// a given stride — size the seg buffer with this.</summary>
        public int MaxPrintSegs(int printSeedStride)
        {
            int cap = Mathf.Max(64, seedCount);
            int stride = Mathf.Max(1, printSeedStride);
            int printSeeds = (cap + stride - 1) / stride;
            int subdiv = Mathf.Clamp(smoothSubdiv, 1, 8);
            int m = history != null ? Mathf.Clamp(history.CurveSamples, 2, 32) : 32;
            return printSeeds * ((m - 1) * subdiv + 1); // chain + 1 bridge per seed
        }

        /// <summary>One-shot print bake: re-runs the collect prepass on the current
        /// (paused) inputs and appends every _PrintSeedStride-th curve as capsule
        /// segments (TSDFTrailBake Seg layout, 48B) into segsOut, each with a bridge
        /// capsule to its blended bone-centerline anchor. Synchronous stats readback —
        /// print path only, not for per-frame use. Returns false (and logs why) when
        /// inputs are missing.</summary>
        public bool EmitPrintSegs(ComputeBuffer segsOut, int printSeedStride, float printRadius,
                                  float bridgeRadiusScale, float maxBridgeLength,
                                  out PrintSegStats stats)
        {
            stats = default;
            if (segsOut == null) { Debug.LogError("[MotionCurves] EmitPrintSegs: segsOut is null.", this); return false; }
            if (_emitKernel < 0) { Debug.LogError("[MotionCurves] CSEmitSegs kernel missing.", this); return false; }
            if (!CollectSeeds(out string reason))
            { Debug.LogError($"[MotionCurves] print emit aborted: {reason}.", this); return false; }

            if (_segStatsBuf == null)
                _segStatsBuf = new GraphicsBuffer(GraphicsBuffer.Target.Structured, 3, sizeof(uint));
            _segStatsBuf.SetData(s_segStatsReset);

            int stride = Mathf.Max(1, printSeedStride);
            BindBuildParams(_emitKernel);
            _shader.SetBuffer(_emitKernel, kSegsOut, segsOut);
            _shader.SetBuffer(_emitKernel, kSegStats, _segStatsBuf);
            _shader.SetInt(kSegCap, segsOut.count);
            _shader.SetInt(kPrintSeedStride, stride);
            _shader.SetFloat(kPrintRadius, printRadius);
            _shader.SetFloat(kBridgeRadiusScale, bridgeRadiusScale);
            _shader.SetFloat(kMaxBridgeLength, maxBridgeLength);
            int printSeeds = (_capacity + stride - 1) / stride;
            _shader.Dispatch(_emitKernel, (printSeeds + 63) / 64, 1, 1);

            var s = new uint[3];
            _segStatsBuf.GetData(s);
            stats.emitted = (int)Mathf.Min(s[0], (uint)segsOut.count);
            stats.dropped = (int)s[1];
            stats.bridgesSkipped = (int)s[2];
            return true;
        }

        private void DrawCurves()
        {
            _mat.SetBuffer(kVerts, _outBuf);
            _mat.SetFloat(kBrightness, brightness);
            _mat.SetFloat(kWidth, ribbonWidth);
            var bounds = new Bounds(Vector3.zero, Vector3.one * 50f);
            Graphics.DrawProceduralIndirect(_mat, bounds, MeshTopology.Triangles, _argsBuf, 0,
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
                    if (IsUsableSource(mf)) _srcScratch.Add(mf);
                return _srcScratch;
            }
            // Auto: per-device playback meshes.
            var all = FindObjectsByType<MeshFilter>(FindObjectsSortMode.None);
            foreach (var mf in all)
            {
                if (!IsUsableSource(mf)) continue;
                if (!string.IsNullOrEmpty(autoSourcePrefix) && !mf.gameObject.name.StartsWith(autoSourcePrefix)) continue;
                _srcScratch.Add(mf);
            }
            return _srcScratch;
        }

        // The compute reads vertex buffer 0 as a Raw ByteAddressBuffer of ObColorPoint (pos12 + col12,
        // 24B). Only meshes whose vertex buffer was created with the Raw target can be bound that way,
        // and the stride must match. Reject anything else (e.g. a CPU/live mesh without Raw, or an
        // unrelated mesh that happens to match autoSourcePrefix) to avoid GPU binding errors / garbage.
        private static bool IsUsableSource(MeshFilter mf)
        {
            if (mf == null) return false;
            var m = mf.sharedMesh;
            if (m == null || m.vertexCount == 0) return false;
            if ((m.vertexBufferTarget & GraphicsBuffer.Target.Raw) == 0) return false;
            if (m.GetVertexBufferStride(0) != kObColorPointStride) return false;
            return true;
        }

        private const int kObColorPointStride = 24; // ObColorPoint: 3 floats pos + 3 floats colour

        private void EnsureBuffers(int boneCount, int ringLen, int subdiv, int totalSrcVerts)
        {
            int cap = Mathf.Max(64, seedCount);
            bool seedStale = _outBuf == null || _capacity != cap || _ringLen != ringLen
                             || _boneCount != boneCount || _subdiv != subdiv;
            if (seedStale)
            {
                _outBuf?.Release(); _argsBuf?.Release();
                _radiusABuf?.Release(); _radiusBBuf?.Release();
                _hasBuilt = false;
                _boneCount = boneCount;
                _ringLen = ringLen;
                _subdiv = subdiv;
                _capacity = cap;

                int segments = cap * (ringLen - 1) * subdiv;
                int lineVerts = segments * 2;                   // _Verts entries (segment endpoint pairs)
                _outBuf = new GraphicsBuffer(GraphicsBuffer.Target.Structured, lineVerts, sizeof(float) * 6); // LineVert
                _argsBuf = new GraphicsBuffer(GraphicsBuffer.Target.IndirectArguments, 4, sizeof(uint));
                // Ribbon: each segment is expanded to a quad (2 tris = 6 verts) in the shader.
                _argsBuf.SetData(new uint[] { (uint)(segments * 6), 1, 0, 0 });

                // Standard per-bone radii (absolute m) + endpoint taper; static, scaled at runtime.
                var bones = BodyTrackingShared.Bones;
                var rA = new float[boneCount];
                var rB = new float[boneCount];
                for (int b = 0; b < boneCount; b++)
                    DefaultRadius(bones[b].a, bones[b].b, out rA[b], out rB[b]);
                _radiusABuf = new GraphicsBuffer(GraphicsBuffer.Target.Structured, boneCount, sizeof(float));
                _radiusBBuf = new GraphicsBuffer(GraphicsBuffer.Target.Structured, boneCount, sizeof(float));
                _radiusABuf.SetData(rA);
                _radiusBBuf.SetData(rB);
            }

            // Collect buffer holds the compacted in-bbox points (SeedPoint = pos + colour, 24B);
            // sized (grow-only) to the source total.
            int need = Mathf.Max(64, totalSrcVerts);
            if (_collectBuf == null || _collectCap < need)
            {
                _collectBuf?.Release();
                _collectBuf = new GraphicsBuffer(GraphicsBuffer.Target.Structured, need, sizeof(float) * 6);
                _collectCap = need;
            }
            if (_collectCounter == null)
                _collectCounter = new GraphicsBuffer(GraphicsBuffer.Target.Structured, 1, sizeof(uint));
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
    }
}
