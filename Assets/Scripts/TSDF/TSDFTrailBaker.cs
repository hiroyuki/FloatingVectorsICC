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
// Compose (manual bake):
//   clearVolumeFirst = true  -> trail-only sculpture (ClearWrite, then bake).
//   clearVolumeFirst = false -> fuse: min-union the trail into whatever the volume
//                               already holds (e.g. an accumulated body TSDF).
//
// Live modes (per-frame, follow playback/live integration):
//   Off           -> only the manual ContextMenu bakes run.
//   LiveTrailOnly -> every frame: clear the write buffer, bake the trail alone,
//                    publish. REQUIRES the integrator to be idle (it would otherwise
//                    clear/publish the body over the trail every frame) — warns if a
//                    live integrator is still driving the same volume.
//   LiveFuse      -> subscribe to TSDFIntegrator.BeforePublishCompleteBatch and
//                    min-union the trail into the just-integrated body WriteBuffer
//                    right before the integrator publishes it. The baker does NOT
//                    publish; the integrator's Publish carries the fused result.
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

        /// <summary>Per-frame live baking mode (Phase 2). Off = manual bakes only.</summary>
        public enum LiveMode
        {
            /// <summary>No per-frame baking; only the manual ContextMenu bakes fire.</summary>
            Off,
            /// <summary>Every frame: clear write, bake the trail alone, publish. Integrator must be idle.</summary>
            LiveTrailOnly,
            /// <summary>Every completed integrator batch: fuse the trail into the body just before its Publish.</summary>
            LiveFuse,
        }

        [Header("Source")]
        [Tooltip("OfflineTrajectories: whole-recording per-joint paths (BodyTrackingPlayback). " +
                 "LiveWindowedTrail: the short, smoothed trail SkeletonMerger draws right now " +
                 "for the joints below — the on-screen ribbon, baked as it follows the motion.")]
        public TrailSource source = TrailSource.LiveWindowedTrail;

        [Header("Live (follow)")]
        [Tooltip("Off: manual bakes / capture only. LiveTrailOnly: re-bake the trail alone every " +
                 "frame (integrator must be idle). LiveFuse: fuse the trail into the body every " +
                 "completed integrator batch (the animated follow look). Ignored while capturing.")]
        public LiveMode liveMode = LiveMode.Off;

        [Header("Capture (Start/Stop accumulation)")]
        [Tooltip("What the frozen sculpture contains. ON: also accumulate the body TSDF sweep " +
                 "alongside the trail (heavier, ghosted body). OFF: trail only (pure motion " +
                 "sculpture). Set before Start Capture.")]
        public bool accumulateBody = true;

        [Min(0f)]
        [Tooltip("Capture sampling density ALONG each bone (the old 'Bone Trail Step'). The capture " +
                 "accumulates not just the joints but points interpolated along every bone, so the " +
                 "whole limb/torso motion enters the sculpture. Parametric step: 0.15 ≈ 7 points per " +
                 "bone. 0 = bone endpoints (joints) only. Smaller = denser & heavier.")]
        public float boneStep = 0.15f;

        /// <summary>True between StartCapture() and StopCapture(): the volume is a single
        /// persistent accumulation buffer that only grows (never per-batch cleared), so the
        /// motion between Start and Stop builds up into one static mesh, then freezes.</summary>
        public bool IsCapturing { get; private set; }

        [Header("Wiring")]
        [Tooltip("Target volume. Leave empty to auto-resolve the first TSDFVolume in the scene.")]
        public TSDFVolume volume;

        [Tooltip("Integrator whose BeforePublishCompleteBatch hook LiveFuse rides. Leave empty " +
                 "to auto-resolve the first TSDFIntegrator in the scene.")]
        public TSDFIntegrator integrator;

        [Tooltip("OfflineTrajectories source. Leave empty to auto-resolve the first BodyTrackingPlayback.")]
        public BodyTrackingPlayback playback;

        [Tooltip("LiveWindowedTrail source. Leave empty to auto-resolve the first SkeletonMerger.")]
        public SkeletonMerger skeleton;

        [Tooltip("Joints baked as ribbons in LiveWindowedTrail mode. Default: wrists, feet, head. " +
                 "HAND/HANDTIP/THUMB are intentionally excluded — k4abt reports them at NONE " +
                 "confidence from every camera view (see BodyTrackingShared.IsDrawnJoint), so the " +
                 "wrist is the reliable arm tip.")]
        public k4abt_joint_id_t[] ribbonJoints =
        {
            k4abt_joint_id_t.K4ABT_JOINT_WRIST_LEFT,
            k4abt_joint_id_t.K4ABT_JOINT_WRIST_RIGHT,
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

        [Header("Compose (manual bake)")]
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

        [Min(0f)]
        [Tooltip("Capture only: skip (and reseed) a per-frame segment longer than this (metres). " +
                 "Guards against a joint teleport / low-confidence pop stamping a huge bridge. " +
                 "0 = no limit.")]
        public float maxSegmentStep = 0.5f;

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
        private int _boxKernel = -1;

        // Persistent, reused across bakes so a per-frame live bake never allocates
        // (Codex: GC / buffer churn would skew the perf read). The segment list is
        // Clear()ed and refilled each build; the ComputeBuffer only grows.
        private readonly List<TrailSeg> _segScratch = new List<TrailSeg>();
        private readonly List<Vector3> _trailScratch = new List<Vector3>();
        private ComputeBuffer _segBuf;

        // Incremental-accumulate capture state. We sample "sites" = points along every bone
        // (endpoints + boneStep interpolation), and remember each site's last stamped world
        // position so each frame only the NEW segment (last -> current) per site is baked, into
        // just its AABB. Sites are laid out per bone: site = boneIndex*_sitesPerBone + stepIndex.
        private Vector3[] _lastSite;
        private bool[] _hasSite;
        private int _stepsPerBone;   // segments per bone (points = steps+1)
        private int _sitesPerBone;   // = _stepsPerBone + 1
        private Vector3Int _accumDim;   // volume Dim the accumulate state is valid for

        // Per-bone batching of this frame's new segments: one small box dispatch per bone
        // (covering all that bone's new segments) instead of one per segment. Voxel-space AABB.
        private int[] _batchStart;
        private int[] _batchCount;
        private Vector3[] _batchVMin;   // voxel-space AABB min per bone
        private Vector3[] _batchVMax;

        // LiveFuse subscription bookkeeping so we hook exactly one integrator and
        // detach cleanly on mode change / disable.
        private TSDFIntegrator _subscribedIntegrator;
        private bool _warnedTrailOnlyConflict;
        private bool _scannedForOtherBakers;

        private void OnDisable()
        {
            IsCapturing = false;
            UnsubscribeFuse();
            _segBuf?.Release();
            _segBuf = null;
        }

        private void OnDestroy()
        {
            _segBuf?.Release();
            _segBuf = null;
        }

        private void Update()
        {
            if (IsCapturing) { CaptureTick(); return; }   // capture overrides the follow modes

            switch (liveMode)
            {
                case LiveMode.Off:
                    UnsubscribeFuse();
                    break;

                case LiveMode.LiveTrailOnly:
                    UnsubscribeFuse();
                    WarnIfMultipleBakers();
                    LiveTrailOnlyTick();
                    break;

                case LiveMode.LiveFuse:
                    WarnIfMultipleBakers();
                    EnsureFuseSubscribed();
                    break;
            }
        }

        // ---- Live: trail-only (baker owns clear/bake/publish) ----------------
        private void LiveTrailOnlyTick()
        {
            if (!ResolveVolume(silent: true)) return;

            // The integrator would clear + publish the body over our trail every
            // frame. LiveTrailOnly requires it idle; warn once if it is still driving.
            if (integrator == null) integrator = FindFirstObjectByType<TSDFIntegrator>();
            if (integrator != null && integrator.isActiveAndEnabled &&
                integrator.integrationEnabled && integrator.volume == volume)
            {
                if (!_warnedTrailOnlyConflict)
                {
                    Debug.LogWarning("[TSDFTrailBaker] LiveTrailOnly: a TSDFIntegrator is still " +
                                     "integrating into this volume — it will fight the trail bake " +
                                     "(last publish per frame wins → flicker). Disable the integrator " +
                                     "(or set integrationEnabled=false) for a clean trail-only view.", this);
                    _warnedTrailOnlyConflict = true;
                }
            }
            else _warnedTrailOnlyConflict = false;

            BakeCore(clearWrite: true, doPublish: true, silent: true);
        }

        // ---- Live: fuse into the body via the integrator's pre-publish hook -----
        private void EnsureFuseSubscribed()
        {
            if (integrator == null) integrator = FindFirstObjectByType<TSDFIntegrator>();
            if (integrator == null)
            {
                if (_subscribedIntegrator == null && !_warnedTrailOnlyConflict)
                {
                    // reuse the warn flag as a one-shot latch for "no integrator"
                    Debug.LogWarning("[TSDFTrailBaker] LiveFuse: no TSDFIntegrator found to hook. " +
                                     "The trail will not appear until an integrator is publishing batches.", this);
                    _warnedTrailOnlyConflict = true;
                }
                return;
            }
            _warnedTrailOnlyConflict = false;

            if (_subscribedIntegrator == integrator) return;   // already hooked the right one
            UnsubscribeFuse();
            integrator.BeforePublishCompleteBatch += OnBeforePublish;
            _subscribedIntegrator = integrator;
        }

        private void UnsubscribeFuse()
        {
            if (_subscribedIntegrator != null)
            {
                _subscribedIntegrator.BeforePublishCompleteBatch -= OnBeforePublish;
                _subscribedIntegrator = null;
            }
        }

        // Fired by the integrator immediately before it publishes a completed batch:
        // the just-integrated body is in WriteBuffer, so min-union the trail into it
        // and return. We must NOT publish — the integrator does that next. Empty trail
        // is a silent no-op so the body still publishes normally (Codex edge case).
        private void OnBeforePublish(TSDFIntegrator integ, TSDFVolume vol)
        {
            if (vol == null) return;
            volume = vol;   // fuse into exactly the volume the integrator is about to publish
            if (IsCapturing)
                StampNewSegments(doPublish: false);   // accumulate: only NEW segments, local box
            else
                BakeCore(clearWrite: false, doPublish: false, silent: true);   // follow: re-bake the window (volume is cleared each batch)
        }

        // ---- Capture: Start/Stop accumulation into a persistent (frozen) sculpture ----
        // The motion between Start and Stop accumulates into ONE never-cleared single buffer
        // (min-union), then Stop freezes it. accumulateBody chooses trail-only vs trail+body.
        [ContextMenu("Start capture")]
        public void StartCapture()
        {
            if (!ResolveVolume(silent: false)) return;
            if (integrator == null) integrator = FindFirstObjectByType<TSDFIntegrator>();

            // Switch the volume to a single persistent accumulation buffer. The integrator
            // forces volume.doubleBuffered = clearVolumeOnNewBatch every Update, so clearing
            // that flag is what keeps us single-buffered while it runs.
            if (integrator != null)
            {
                integrator.volume = volume;
                integrator.clearVolumeOnNewBatch = false;
            }
            volume.doubleBuffered = false;
            volume.ForceRebuild();   // rebuild as single buffer + clear to empty

            if (accumulateBody && integrator != null)
            {
                // Body sweep accumulates via the integrator's fold path; the trail rides the
                // fold's pre-publish hook so both land in the same accumulation buffer.
                integrator.integrationEnabled = true;
                integrator.BeginFreshBatch();
                EnsureFuseSubscribed();
            }
            else
            {
                // Trail only: keep the integrator from touching the volume; the baker
                // min-unions the current trail into the persistent buffer each frame.
                if (integrator != null) integrator.integrationEnabled = false;
                UnsubscribeFuse();
                volume.ClearWrite();
                volume.Publish();
            }

            _accumDim = volume.Dim;
            SetupAccumSites();         // fresh sweep: size per-site state + clear "seen" flags

            IsCapturing = true;
            LastStatus = $"CAPTURING ({(accumulateBody ? "trail + body sweep" : "trail only")})…";
            Debug.Log($"[TSDFTrailBaker] {LastStatus}", this);
        }

        [ContextMenu("Stop capture")]
        public void StopCapture()
        {
            if (!IsCapturing) return;
            IsCapturing = false;
            if (integrator != null) integrator.integrationEnabled = false;   // freeze the body sweep
            UnsubscribeFuse();                                               // stop adding trail
            if (volume != null) volume.Publish();                           // show the final accumulated mesh
            LastStatus = "CAPTURE stopped — frozen sculpture on screen.";
            Debug.Log($"[TSDFTrailBaker] {LastStatus}", this);
        }

        // Per-frame work while capturing. Body-sweep mode is driven by the integrator + hook,
        // so only trail-only mode needs to bake here (min-union into the never-cleared buffer).
        private void CaptureTick()
        {
            if (accumulateBody) return;   // body sweep + trail ride the integrator's pre-publish hook
            StampNewSegments(doPublish: true);
        }

        // Incremental accumulate: each frame, sample points along every bone (endpoints +
        // boneStep interpolation) and stamp only the NEW segment each site moved through, into
        // just the voxels inside that bone's AABB. Cost is independent of the total volume size
        // (unlike the full-volume BakeCore). Never clears — the persistent single buffer
        // accumulates the whole Start->Stop sweep of the skeleton (joints + along bones).
        private void StampNewSegments(bool doPublish)
        {
            if (volume == null) return;
            var dim = volume.Dim;
            if (dim.x <= 0 || dim.y <= 0 || dim.z <= 0) return;
            if (volume.WriteBuffer == null || volume.WriteColorBuffer == null) return;
            if (!EnsureShader()) return;
            if (skeleton == null) skeleton = FindFirstObjectByType<SkeletonMerger>();
            if (skeleton == null) return;

            if (_lastSite == null || _accumDim != dim) { _accumDim = dim; SetupAccumSites(); }

            var bones = BodyTrackingShared.Bones;
            _segScratch.Clear();
            int batchN = 0;

            for (int bi = 0; bi < bones.Length; bi++)
            {
                var (ja, jb) = bones[bi];
                // Bones already omit the hand joints, but guard defensively.
                if (!BodyTrackingShared.IsDrawnJoint(ja) || !BodyTrackingShared.IsDrawnJoint(jb)) continue;
                if (!skeleton.TryGetJointWorld(ja, out Vector3 wa)) continue;
                if (!skeleton.TryGetJointWorld(jb, out Vector3 wb)) continue;

                int start = _segScratch.Count;
                Vector3 vmin = new Vector3(float.PositiveInfinity, float.PositiveInfinity, float.PositiveInfinity);
                Vector3 vmax = new Vector3(float.NegativeInfinity, float.NegativeInfinity, float.NegativeInfinity);

                for (int s = 0; s <= _stepsPerBone; s++)
                {
                    float t = _stepsPerBone > 0 ? (float)s / _stepsPerBone : 0f;
                    Vector3 cur = Vector3.Lerp(wa, wb, t);
                    int site = bi * _sitesPerBone + s;
                    if (!_hasSite[site]) { _lastSite[site] = cur; _hasSite[site] = true; continue; }   // seed
                    Vector3 prev = _lastSite[site];
                    float len = (cur - prev).magnitude;
                    if (len < 1e-4f) continue;                                             // no movement
                    if (maxSegmentStep > 0f && len > maxSegmentStep) { _lastSite[site] = cur; continue; }  // teleport
                    var colJoint = (t < 0.5f) ? ja : jb;   // colour by the nearer bone endpoint
                    Color baseCol = perJointHue ? HueFor((int)colJoint, K4ABTConsts.K4ABT_JOINT_COUNT) * trailColor : trailColor;
                    _segScratch.Add(new TrailSeg { a = prev, b = cur, ra = radius, rb = radius,
                                                   color = new Vector3(baseCol.r, baseCol.g, baseCol.b) });
                    // accumulate this bone's AABB in VOXEL space (rotation-safe box).
                    Vector3 vp = volume.VoxelFromWorld.MultiplyPoint3x4(prev);
                    Vector3 vc = volume.VoxelFromWorld.MultiplyPoint3x4(cur);
                    vmin = Vector3.Min(vmin, Vector3.Min(vp, vc));
                    vmax = Vector3.Max(vmax, Vector3.Max(vp, vc));
                    _lastSite[site] = cur;
                }

                int count = _segScratch.Count - start;
                if (count > 0)
                {
                    _batchStart[batchN] = start; _batchCount[batchN] = count;
                    _batchVMin[batchN] = vmin; _batchVMax[batchN] = vmax;
                    batchN++;
                }
            }

            if (_segScratch.Count == 0) return;   // nothing moved this frame (quiet)
            if (!EnsureSegBuffer(_segScratch.Count)) return;
            _segBuf.SetData(_segScratch, 0, 0, _segScratch.Count);

            _shader.SetInts("_Dim", dim.x, dim.y, dim.z);
            _shader.SetFloat("_Tau", volume.Tau);
            _shader.SetMatrix("_WorldFromVoxel", volume.WorldFromVoxel);
            _shader.SetBuffer(_boxKernel, "_Segs", _segBuf);
            _shader.SetBuffer(_boxKernel, "_VoxelsOut", volume.WriteBuffer);
            _shader.SetBuffer(_boxKernel, "_ColorsOut", volume.WriteColorBuffer);

            float padVox = (radius + volume.Tau) / Mathf.Max(1e-6f, volume.voxelSize) + 1f;
            for (int b = 0; b < batchN; b++)
            {
                Vector3 vmin = _batchVMin[b] - Vector3.one * padVox;
                Vector3 vmax = _batchVMax[b] + Vector3.one * padVox;
                int i0 = Mathf.Clamp(Mathf.FloorToInt(vmin.x - 0.5f), 0, dim.x - 1);
                int j0 = Mathf.Clamp(Mathf.FloorToInt(vmin.y - 0.5f), 0, dim.y - 1);
                int k0 = Mathf.Clamp(Mathf.FloorToInt(vmin.z - 0.5f), 0, dim.z - 1);
                int i1 = Mathf.Clamp(Mathf.CeilToInt (vmax.x - 0.5f), 0, dim.x - 1);
                int j1 = Mathf.Clamp(Mathf.CeilToInt (vmax.y - 0.5f), 0, dim.y - 1);
                int k1 = Mathf.Clamp(Mathf.CeilToInt (vmax.z - 0.5f), 0, dim.z - 1);
                int nx = i1 - i0 + 1, ny = j1 - j0 + 1, nz = k1 - k0 + 1;
                if (nx <= 0 || ny <= 0 || nz <= 0) continue;
                DispatchGrid(nx * ny * nz, out int gx, out int gy);
                _shader.SetInts("_VoxMin", i0, j0, k0);
                _shader.SetInts("_VoxSize", nx, ny, nz);
                _shader.SetInt("_BoxDispatchWidth", gx * 64);
                _shader.SetInt("_SegOffset", _batchStart[b]);
                _shader.SetInt("_SegCount", _batchCount[b]);
                _shader.Dispatch(_boxKernel, Mathf.Max(1, gx), Mathf.Max(1, gy), 1);
            }

            if (doPublish) volume.Publish();
            LastStatus = $"accumulating: +{_segScratch.Count} seg/frame over {batchN} bone(s) (r={radius:0.000}m, boneStep={boneStep:0.00})";
        }

        // Size the per-site last-position + per-bone batch arrays for the current bone table
        // and boneStep, and clear the "seen" flags so a fresh capture starts from scratch.
        private void SetupAccumSites()
        {
            int bones = BodyTrackingShared.Bones.Length;
            _stepsPerBone = boneStep > 0f ? Mathf.Max(1, Mathf.CeilToInt(1f / boneStep)) : 1;
            _sitesPerBone = _stepsPerBone + 1;
            int sites = bones * _sitesPerBone;
            if (_lastSite == null || _lastSite.Length != sites)
            {
                _lastSite = new Vector3[sites];
                _hasSite = new bool[sites];
            }
            else System.Array.Clear(_hasSite, 0, _hasSite.Length);
            if (_batchStart == null || _batchStart.Length != bones)
            {
                _batchStart = new int[bones];
                _batchCount = new int[bones];
                _batchVMin = new Vector3[bones];
                _batchVMax = new Vector3[bones];
            }
        }

        // ---- Manual bakes (ContextMenu) — report failures loudly ---------------
        [ContextMenu("Bake BT trail into volume")]
        public void BakeTrailIntoVolume()
        {
            if (!ResolveVolume(silent: false)) return;
            BakeCore(clearWrite: clearVolumeFirst, doPublish: true, silent: false);
        }

        // Phase 1: fuse the trail into the CURRENTLY DISPLAYED body. After the
        // integrator's last Publish() the body is in FrontBuffer while WriteBuffer holds
        // stale scratch, so copy Front->Write first, then min-union the trail and publish.
        [ContextMenu("Fuse trail into displayed body")]
        public void FuseTrailIntoDisplayedBody()
        {
            if (!ResolveVolume(silent: false)) return;
            volume.CopyFrontToWrite();   // put the on-screen body back into the write buffer
            BakeCore(clearWrite: false, doPublish: true, silent: false);
        }

        // ---- Core bake routine -------------------------------------------------
        // Writes into volume.WriteBuffer/WriteColorBuffer. clearWrite wipes it first
        // (trail-only); doPublish swaps/bumps so views pick it up (false = a hook
        // subscriber that lets the integrator publish); silent suppresses the empty-
        // trail failure log for the live paths (Codex: live empty trail = quiet no-op).
        private bool BakeCore(bool clearWrite, bool doPublish, bool silent)
        {
            if (volume == null) { if (!silent) Fail("no TSDFVolume found"); return false; }

            var dim = volume.Dim;
            if (dim.x <= 0 || dim.y <= 0 || dim.z <= 0)
            {
                if (!silent) Fail("volume not initialised (Dim is 0)");
                return false;
            }
            if (volume.WriteBuffer == null || volume.WriteColorBuffer == null)
            {
                if (!silent) Fail("volume buffers not allocated");
                return false;
            }

            int trajCount; bool capped;
            if (!BuildSegments(silent, out trajCount, out capped))
            {
                // BuildSegments already Failed() with a specific reason for the
                // manual (non-silent) path; silent live path just bails quietly.
                return false;
            }
            if (_segScratch.Count == 0)
            {
                if (!silent) Fail(source == TrailSource.LiveWindowedTrail
                    ? "no windowed trail segments (enable showBones so trails accumulate, and let " +
                      "the body move a bit / check ribbonJoints)"
                    : "no segments built (Read+Process the recording first, and check " +
                      "deviceSerialFilter / confidence)");
                return false;   // empty trail: leave the volume as-is (never stalls a body publish)
            }

            if (!EnsureShader()) { if (!silent) Fail("TSDFTrailBake.compute not found in Resources"); return false; }
            if (!EnsureSegBuffer(_segScratch.Count)) return false;

            _segBuf.SetData(_segScratch, 0, 0, _segScratch.Count);

            if (clearWrite) volume.ClearWrite();

            int total = dim.x * dim.y * dim.z;
            DispatchGrid(total, out int gx, out int gy);

            _shader.SetInts("_Dim", dim.x, dim.y, dim.z);
            _shader.SetInt("_DispatchWidth", gx * 64);
            _shader.SetFloat("_Tau", volume.Tau);
            _shader.SetMatrix("_WorldFromVoxel", volume.WorldFromVoxel);
            _shader.SetBuffer(_kernel, "_Segs", _segBuf);
            _shader.SetBuffer(_kernel, "_VoxelsOut", volume.WriteBuffer);
            _shader.SetBuffer(_kernel, "_ColorsOut", volume.WriteColorBuffer);

            int batches = 0;
            for (int off = 0; off < _segScratch.Count; off += batchSize)
            {
                int count = Mathf.Min(batchSize, _segScratch.Count - off);
                _shader.SetInt("_SegOffset", off);
                _shader.SetInt("_SegCount", count);
                _shader.Dispatch(_kernel, Mathf.Max(1, gx), Mathf.Max(1, gy), 1);
                batches++;
            }

            if (doPublish) volume.Publish();

            LastStatus = $"baked {_segScratch.Count} segs (from {trajCount} trajectories) " +
                         $"in {batches} batch(es){(capped ? $", CAPPED at {maxSegments}" : "")}" +
                         $"; clearWrite={clearWrite}, publish={doPublish}, r={radius:0.000}m";
            if (!silent) Debug.Log($"[TSDFTrailBaker] {LastStatus}", this);
            return true;
        }

        // Fill _segScratch with the current source's capsule segments. Returns false
        // only on a hard source-missing failure (which it logs when !silent via Fail);
        // an empty-but-valid source returns true with _segScratch.Count == 0.
        private bool BuildSegments(bool silent, out int trajCount, out bool capped)
        {
            _segScratch.Clear();
            trajCount = 0;
            capped = false;

            if (source == TrailSource.LiveWindowedTrail)
            {
                if (skeleton == null) skeleton = FindFirstObjectByType<SkeletonMerger>();
                if (skeleton == null) { if (!silent) Fail("no SkeletonMerger found (LiveWindowedTrail source)"); return false; }
                BuildWindowedSegments(out trajCount, out capped);
                return true;
            }

            if (playback == null) playback = FindFirstObjectByType<BodyTrackingPlayback>();
            if (playback == null) { if (!silent) Fail("no BodyTrackingPlayback found (OfflineTrajectories source)"); return false; }
            Transform space = sourceSpace;
            if (space == null)
            {
                var mlr = FindFirstObjectByType<MotionLineRenderer>();
                if (mlr != null) space = mlr.transform;
            }
            BuildOfflineSegments(space, out trajCount, out capped);
            return true;
        }

        private void BuildOfflineSegments(Transform space, out int trajCount, out bool capped)
        {
            capped = false;
            var trajectories = playback.Trajectories;
            trajCount = trajectories != null ? trajectories.Count : 0;
            if (trajectories == null) return;

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
                            _segScratch.Add(new TrailSeg { a = a, b = b, ra = radius, rb = radius, color = colVec });
                            if (_segScratch.Count >= maxSegments) { capped = true; return; }
                        }
                    }
                    prev = i;
                }
            }
        }

        // Build capsule segments from the SkeletonMerger's current windowed trail for each
        // selected joint (the same ribbon centerline it draws). trajCount = joints that
        // contributed at least one segment.
        private void BuildWindowedSegments(out int trajCount, out bool capped)
        {
            capped = false;
            trajCount = 0;
            if (ribbonJoints == null) return;

            int stride = Mathf.Max(1, sampleStride);
            foreach (var joint in ribbonJoints)
            {
                // Enforce the project-wide hand exclusion even if the Inspector list still
                // contains a hand joint — k4abt gives those NONE confidence (garbage pos).
                if (!BodyTrackingShared.IsDrawnJoint(joint)) continue;
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
                            _segScratch.Add(new TrailSeg { a = a, b = b, ra = radius, rb = radius, color = colVec });
                            contributed = true;
                            if (_segScratch.Count >= maxSegments) { capped = true; if (contributed) trajCount++; return; }
                        }
                    }
                    prev = i;
                }
                if (contributed) trajCount++;
            }
        }

        private bool ResolveVolume(bool silent)
        {
            if (volume == null) volume = FindFirstObjectByType<TSDFVolume>();
            if (volume == null) { if (!silent) Fail("no TSDFVolume found"); return false; }
            return true;
        }

        // Grow-only reusable segment buffer (48-byte TrailSeg). Never shrinks so a
        // steady-state live bake stops reallocating once it hits its peak count.
        private bool EnsureSegBuffer(int count)
        {
            if (count <= 0) return false;
            if (_segBuf == null || _segBuf.count < count)
            {
                _segBuf?.Release();
                _segBuf = new ComputeBuffer(Mathf.Max(count, 256), sizeof(float) * 12);
            }
            return true;
        }

        // One-shot scan (first live frame): warn if a second baker is also live, since
        // they would both min-union into the same volume and stack / fight.
        private void WarnIfMultipleBakers()
        {
            if (_scannedForOtherBakers) return;
            _scannedForOtherBakers = true;
            var all = FindObjectsByType<TSDFTrailBaker>(FindObjectsSortMode.None);
            int live = 0;
            foreach (var b in all) if (b.liveMode != LiveMode.Off) live++;
            if (live > 1)
                Debug.LogWarning("[TSDFTrailBaker] More than one baker is in a live mode. Multiple " +
                                 "bakers min-union into the same volume and will stack / fight. Keep " +
                                 "one live baker per volume.", this);
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
            if (_shader != null && _kernel >= 0 && _boxKernel >= 0) return true;
            if (_shader == null) _shader = Resources.Load<ComputeShader>("TSDFTrailBake");
            if (_shader == null) return false;
            if (_kernel < 0) _kernel = _shader.FindKernel("BakeTrail");
            if (_boxKernel < 0) _boxKernel = _shader.FindKernel("BakeTrailBox");
            return _kernel >= 0 && _boxKernel >= 0;
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
