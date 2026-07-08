// Bakes a body-tracking motion TRAIL into a TSDFVolume as authored capsule
// geometry, then lets the existing Marching Cubes pass (TSDFView) re-mesh it —
// the "feed the BT trail back into the SDF and re-mesh" experiment.
//
// Two ways to bake:
//   1. Start/Stop CAPTURE (the main path): accumulate the motion between Start and
//      Stop into ONE persistent single buffer (never cleared) so it builds up into a
//      static "motion sculpture", then freeze. Each frame stamps only the NEW segment
//      each sample site moved through, into just its AABB (BakeTrailBox) — cost is
//      independent of the total volume size. Sites = points along every bone (endpoints
//      + boneStep interpolation) read from the merged skeleton's CURRENT pose
//      (SkeletonMerger.TryGetJointWorld — no dependency on any fading trail). Hand
//      joints are excluded project-wide (BodyTrackingShared.IsDrawnJoint).
//      accumulateBody also folds the body TSDF sweep in (via the integrator + the
//      BeforePublishCompleteBatch hook) so trail + body land in the same buffer.
//   2. Manual one-shot bakes (ContextMenu) from BodyTrackingPlayback.Trajectories —
//      the whole-recording per-joint paths. Those samples are k4a-CAMERA-LOCAL
//      (mm->m + Y flip); sourceSpace (default: a MotionLineRenderer transform) maps
//      them to world before comparing to the volume's world-space voxels.
//
// Each pair of consecutive samples becomes one analytic capsule segment; the kernel
// (TSDFTrailBake.compute) min-unions sdf = dist(p, segment) - radius into the volume
// write buffer, then Publish() triggers re-extraction.

using System.Collections.Generic;
using BodyTracking;
using Shared;
using UnityEngine;

namespace TSDF
{
    [DisallowMultipleComponent]
    public class TSDFTrailBaker : MonoBehaviour, Shared.IAccumulationController, Shared.IAccumulationExtraActions
    {
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

        [Min(0f)]
        [Tooltip("Capture only: skip (and reseed) a per-frame segment longer than this (metres). " +
                 "Guards against a joint teleport / low-confidence pop stamping a huge bridge. " +
                 "0 = no limit.")]
        public float maxSegmentStep = 0.5f;

        [Min(0f)]
        [Tooltip("Seconds between capture stamps. 0 = continuous (trail: every frame; body sweep: " +
                 "every complete batch — the historical behaviour). > 0 = discrete stamps: the " +
                 "trail becomes a coarser polyline connecting the stamped poses, and with " +
                 "accumulateBody the body is folded in stroboscopically. NOTE: fast motion over " +
                 "a long interval can exceed Max Segment Step and leave trail gaps (reseeds).")]
        public float intervalSeconds = 0f;

        [Tooltip("On Stop capture, also PAUSE the recording playback so the whole scene freezes " +
                 "with the sculpture. Resume live un-pauses it.")]
        public bool pausePlaybackOnStop = true;

        /// <summary>True between StartCapture() and StopCapture(): the volume is a single
        /// persistent accumulation buffer that only grows (never per-batch cleared), so the
        /// motion between Start and Stop builds up into one static mesh, then freezes.</summary>
        public bool IsCapturing { get; private set; }

        // ---- Shared.IAccumulationController (unified accumulation UI) ----
        // Start/Stop = capture. CanClear=false ON PURPOSE: this component's "clear"
        // (ResumeLive) has heavier side effects (discard sculpture + rebuild double-
        // buffered + re-enable integration + resume playback), so it keeps its own
        // dedicated "Resume live" button instead of riding the shared Clear slot.
        public bool IsAccumulating => IsCapturing;
        public float IntervalSeconds
        {
            get => intervalSeconds;
            set => intervalSeconds = Mathf.Max(0f, value);
        }
        public string StatusText => LastStatus;
        public bool CanStart => !IsCapturing;
        public string StartLabel => "Start capture";
        public void StartAccumulate() => StartCapture();
        public bool CanStop => IsCapturing;
        public void StopAccumulate() => StopCapture();
        public bool CanClear => false;
        public string ClearLabel => "";
        public void ClearAccumulated() { }

        // ---- Shared.IAccumulationExtraActions (Control Panel) ----
        // Resume live is this component's exit action; exposing it here lets the
        // central Control Panel offer it without selecting the GameObject.
        public int ExtraActionCount => 1;
        public string ExtraActionLabel(int index) => "Resume live (body back)";
        public void RunExtraAction(int index) => ResumeLive();

        [Header("Wiring")]
        [Tooltip("Target volume. Leave empty to auto-resolve the first TSDFVolume in the scene.")]
        public TSDFVolume volume;

        [Tooltip("Integrator to drive for the body sweep (accumulateBody) and whose " +
                 "BeforePublishCompleteBatch hook the trail rides. Auto-resolved if empty.")]
        public TSDFIntegrator integrator;

        [Tooltip("Skeleton the capture reads current joint positions from. Auto-resolved if empty.")]
        public SkeletonMerger skeleton;

        [Tooltip("Manual-bake source: whole-recording per-joint trajectories. Auto-resolved if empty.")]
        public BodyTrackingPlayback playback;

        [Tooltip("Recorder paused on Stop capture / resumed on Resume live. Auto-resolved if empty.")]
        public PointCloud.SensorRecorder recorder;

        [Tooltip("Manual bake: transform mapping camera-local trajectory samples into world. " +
                 "Leave empty to use the first MotionLineRenderer's transform.")]
        public Transform sourceSpace;

        [Header("Capsule")]
        [Min(0f)]
        [Tooltip("Capsule radius in metres. Keep >= ~1 voxel (voxelSize) or the tube " +
                 "aliases / disappears at the volume resolution.")]
        public float radius = 0.05f;

        [Tooltip("Manual bake: drop trajectory samples below this confidence before linking " +
                 "segments. MEDIUM is the SDK ceiling for real observations.")]
        public k4abt_joint_confidence_level_t minConfidence =
            k4abt_joint_confidence_level_t.K4ABT_JOINT_CONFIDENCE_MEDIUM;

        [Min(1)]
        [Tooltip("Manual bake: use every Nth trajectory sample as a capsule endpoint.")]
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
        [Tooltip("Manual bake: hard cap on capsule segments (excess dropped, logged).")]
        public int maxSegments = 20000;

        [Min(1)]
        [Tooltip("Manual bake: segments processed per Dispatch (TDR-safe batching).")]
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
        private int _boxKernel = -1;

        // Reused so a per-frame capture bake never allocates. The list is Clear()ed and
        // refilled each build; the ComputeBuffer only grows.
        private readonly List<TrailSeg> _segScratch = new List<TrailSeg>();
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

        // Integrator hook subscription (used only while capturing with accumulateBody).
        private TSDFIntegrator _subscribedIntegrator;

        // Interval gating (intervalSeconds > 0): see BatchIntervalGate. Trail-only mode
        // uses its clock as a plain stamp throttle (_lastSite then connects the last
        // STAMPED pose to the current one, so the polyline just gets coarser). Body-
        // sweep mode closes the integrator gate right after each complete-batch fold
        // (inside OnBeforePublish — the only race-free batch boundary) and CaptureTick
        // re-opens it once the interval elapses; the trail rides those folds so both
        // stamp on the same clock.
        private readonly BatchIntervalGate _gate = new BatchIntervalGate();

        private void OnDisable()
        {
            IsCapturing = false;
            _gate.Reset();
            UnsubscribeFuse();
            GpuBuf.Release(ref _segBuf);
        }

        private void OnDestroy()
        {
            GpuBuf.Release(ref _segBuf);
        }

        private void Update()
        {
            if (IsCapturing) CaptureTick();   // capture is the only per-frame path; else button-driven
        }

        // ---- Integrator pre-publish hook (body-sweep capture) -------------------
        private void EnsureFuseSubscribed()
        {
            if (integrator == null) integrator = FindFirstObjectByType<TSDFIntegrator>();
            if (integrator == null) return;
            if (_subscribedIntegrator == integrator) return;
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

        // Fired just before the integrator publishes a completed batch: the freshly-folded
        // body is in WriteBuffer, so min-union the current trail into it (we don't publish —
        // the integrator does that next). Only active during a body-sweep capture.
        private void OnBeforePublish(TSDFIntegrator integ, TSDFVolume vol)
        {
            if (!IsCapturing || vol == null) return;
            volume = vol;
            StampNewSegments(doPublish: false);
            if (intervalSeconds > 0f)
            {
                // Close the gate at this batch boundary: the fold that just happened
                // still publishes, then whole frames are dropped until CaptureTick
                // re-opens the gate after the interval. No partial instants can mix.
                _gate.Close(integ);
            }
        }

        // ---- Capture: Start/Stop accumulation into a persistent (frozen) sculpture ----
        [ContextMenu("Start capture")]
        public void StartCapture()
        {
            if (!ResolveVolume(silent: false)) return;
            if (integrator == null) integrator = FindFirstObjectByType<TSDFIntegrator>();

            // Single persistent accumulation buffer (no ping-pong clear). The integrator
            // forces volume.doubleBuffered = clearVolumeOnNewBatch every Update, so clearing
            // that flag keeps us single-buffered while it runs.
            if (integrator != null)
            {
                integrator.volume = volume;
                integrator.clearVolumeOnNewBatch = false;
            }
            volume.doubleBuffered = false;
            volume.ForceRebuild();   // rebuild as single buffer + clear to empty

            if (accumulateBody && integrator != null)
            {
                integrator.integrationEnabled = true;   // fold the body sweep each instant
                integrator.BeginFreshBatch();
                EnsureFuseSubscribed();                 // trail rides the fold's pre-publish hook
            }
            else
            {
                if (integrator != null) integrator.integrationEnabled = false;   // trail only
                UnsubscribeFuse();
                volume.ClearWrite();
                volume.Publish();
            }

            _accumDim = volume.Dim;
            SetupAccumSites();         // fresh sweep: size per-site state + clear "seen" flags

            _gate.Reset(immediateNextStamp: true);   // first stamp fires immediately
            IsCapturing = true;
            LastStatus = $"CAPTURING ({(accumulateBody ? "trail + body sweep" : "trail only")})…";
            Debug.Log($"[TSDFTrailBaker] {LastStatus}", this);
        }

        [ContextMenu("Stop capture")]
        public void StopCapture()
        {
            if (!IsCapturing) return;
            IsCapturing = false;
            _gate.Reset();
            if (integrator != null) integrator.integrationEnabled = false;   // freeze the body sweep
            UnsubscribeFuse();                                               // stop adding trail
            if (volume != null) volume.Publish();                           // show the final accumulated mesh

            // Freeze the recording playback too, so the whole scene holds with the sculpture.
            if (pausePlaybackOnStop)
            {
                if (recorder == null) recorder = FindFirstObjectByType<PointCloud.SensorRecorder>();
                if (recorder != null && recorder.CurrentState == PointCloud.SensorRecorder.State.Playing && !recorder.IsPaused)
                    recorder.PausePlayback();
            }

            LastStatus = "CAPTURE stopped — frozen sculpture on screen. (Resume live to get the body back.)";
            Debug.Log($"[TSDFTrailBaker] {LastStatus}", this);
        }

        // Exit capture and return to normal live-follow: the frozen sculpture is cleared and
        // the body TSDF mesh follows the current frame again. A trail-only capture disables the
        // body integrator, so the surface mesh stays gone until you resume — this brings it back.
        [ContextMenu("Resume live (exit capture)")]
        public void ResumeLive()
        {
            if (!ResolveVolume(silent: false)) return;
            if (integrator == null) integrator = FindFirstObjectByType<TSDFIntegrator>();
            IsCapturing = false;
            _gate.Reset();
            UnsubscribeFuse();
            volume.doubleBuffered = true;
            if (integrator != null)
            {
                integrator.volume = volume;
                integrator.clearVolumeOnNewBatch = true;   // live-follow (clears each batch)
                integrator.integrationEnabled = true;      // integrate the current body again
            }
            volume.ForceRebuild();                         // drop the frozen sculpture, rebuild double-buffered
            if (integrator != null) integrator.BeginFreshBatch();

            // Un-pause the playback that Stop capture froze, so the live body actually updates.
            if (recorder == null) recorder = FindFirstObjectByType<PointCloud.SensorRecorder>();
            if (recorder != null && recorder.CurrentState == PointCloud.SensorRecorder.State.Playing && recorder.IsPaused)
                recorder.ResumePlayback();

            LastStatus = "Resumed live — body follows the current frame again.";
            Debug.Log($"[TSDFTrailBaker] {LastStatus}", this);
        }

        // Per-frame work while capturing. Body-sweep mode is driven by the integrator + hook,
        // so only trail-only mode needs to bake here (min-union into the never-cleared buffer).
        private void CaptureTick()
        {
            if (accumulateBody)
            {
                // Body sweep + trail ride the integrator's pre-publish hook; here we
                // only re-open the interval gate that OnBeforePublish closed (also
                // when the user sets the interval back to 0 mid-capture).
                _gate.TryReopen(integrator, intervalSeconds);
                return;
            }
            if (!_gate.IntervalElapsed(intervalSeconds)) return;
            _gate.MarkNow();
            StampNewSegments(doPublish: true);
        }

        // Incremental accumulate: each frame, sample points along every bone (endpoints +
        // boneStep interpolation) and stamp only the NEW segment each site moved through, into
        // just the voxels inside that bone's AABB. Cost is independent of the total volume size.
        // Never clears — the persistent single buffer accumulates the whole Start->Stop sweep.
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
            // Active-block marking (task 0-1): baked trail voxels mark the write set so
            // active-block MC meshes the sculpture (rides the same Publish/swap).
            volume.BindBlockMarking(_shader, _boxKernel, volume.WriteBlockActive);

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

        // ---- Manual one-shot bakes (ContextMenu) from BodyTrackingPlayback ------
        [ContextMenu("Bake BT trail into volume")]
        public void BakeTrailIntoVolume()
        {
            if (!ResolveVolume(silent: false)) return;
            BakeCore(clearWrite: clearVolumeFirst, doPublish: true, silent: false);
        }

        // Fuse the trail into the CURRENTLY DISPLAYED body: after the last Publish() the body
        // is in FrontBuffer while WriteBuffer holds stale scratch, so copy Front->Write first,
        // then min-union the trail and publish.
        [ContextMenu("Fuse trail into displayed body")]
        public void FuseTrailIntoDisplayedBody()
        {
            if (!ResolveVolume(silent: false)) return;
            volume.CopyFrontToWrite();
            BakeCore(clearWrite: false, doPublish: true, silent: false);
        }

        // Full-volume bake of the offline recording trajectories (manual path).
        private bool BakeCore(bool clearWrite, bool doPublish, bool silent)
        {
            if (volume == null) { if (!silent) Fail("no TSDFVolume found"); return false; }

            var dim = volume.Dim;
            if (dim.x <= 0 || dim.y <= 0 || dim.z <= 0) { if (!silent) Fail("volume not initialised (Dim is 0)"); return false; }
            if (volume.WriteBuffer == null || volume.WriteColorBuffer == null) { if (!silent) Fail("volume buffers not allocated"); return false; }

            int trajCount; bool capped;
            if (!BuildOfflineSegments(silent, out trajCount, out capped)) return false;
            if (_segScratch.Count == 0)
            {
                if (!silent) Fail("no segments built (Read+Process the recording first, and check " +
                                  "deviceSerialFilter / confidence)");
                return false;
            }

            if (!EnsureShader()) { if (!silent) Fail("TSDFTrailBake.compute not found in Resources"); return false; }
            if (!EnsureSegBuffer(_segScratch.Count)) return false;
            _segBuf.SetData(_segScratch, 0, 0, _segScratch.Count);

            if (clearWrite) volume.ClearWrite();

            int total = dim.x * dim.y * dim.z;
            _shader.SetInts("_Dim", dim.x, dim.y, dim.z);
            _shader.SetFloat("_Tau", volume.Tau);
            _shader.SetMatrix("_WorldFromVoxel", volume.WorldFromVoxel);
            _shader.SetBuffer(_kernel, "_Segs", _segBuf);
            _shader.SetBuffer(_kernel, "_VoxelsOut", volume.WriteBuffer);
            _shader.SetBuffer(_kernel, "_ColorsOut", volume.WriteColorBuffer);
            // Active-block marking (task 0-1): baked trail voxels mark the write set so
            // active-block MC meshes the sculpture (rides the Publish/swap).
            volume.BindBlockMarking(_shader, _kernel, volume.WriteBlockActive);

            int batches = 0;
            for (int off = 0; off < _segScratch.Count; off += batchSize)
            {
                int count = Mathf.Min(batchSize, _segScratch.Count - off);
                _shader.SetInt("_SegOffset", off);
                _shader.SetInt("_SegCount", count);
                TSDFComputeUtil.DispatchLinear(_shader, _kernel, total);
                batches++;
            }

            if (doPublish) volume.Publish();

            LastStatus = $"baked {_segScratch.Count} segs (from {trajCount} trajectories) " +
                         $"in {batches} batch(es){(capped ? $", CAPPED at {maxSegments}" : "")}" +
                         $"; clearWrite={clearWrite}, publish={doPublish}, r={radius:0.000}m";
            if (!silent) Debug.Log($"[TSDFTrailBaker] {LastStatus}", this);
            return true;
        }

        // Fill _segScratch from BodyTrackingPlayback.Trajectories (camera-local -> world).
        private bool BuildOfflineSegments(bool silent, out int trajCount, out bool capped)
        {
            _segScratch.Clear();
            trajCount = 0;
            capped = false;

            if (playback == null) playback = FindFirstObjectByType<BodyTrackingPlayback>();
            if (playback == null) { if (!silent) Fail("no BodyTrackingPlayback found"); return false; }
            Transform space = sourceSpace;
            if (space == null)
            {
                var mlr = FindFirstObjectByType<MotionLineRenderer>();
                if (mlr != null) space = mlr.transform;
            }

            var trajectories = playback.Trajectories;
            trajCount = trajectories != null ? trajectories.Count : 0;
            if (trajectories == null) return true;

            int stride = Mathf.Max(1, sampleStride);
            foreach (var traj in trajectories)
            {
                if (!BodyTrackingShared.IsDrawnJoint(traj.JointId)) continue;   // never bake the hand joints
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
                        if ((b - a).sqrMagnitude > 1e-10f)
                        {
                            _segScratch.Add(new TrailSeg { a = a, b = b, ra = radius, rb = radius, color = colVec });
                            if (_segScratch.Count >= maxSegments) { capped = true; return true; }
                        }
                    }
                    prev = i;
                }
            }
            return true;
        }

        private bool ResolveVolume(bool silent)
        {
            if (volume == null) volume = FindFirstObjectByType<TSDFVolume>();
            if (volume == null) { if (!silent) Fail("no TSDFVolume found"); return false; }
            return true;
        }

        // Grow-only reusable segment buffer (48-byte TrailSeg).
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
            if (!TSDFComputeUtil.TryLoad(ref _shader, "TSDFTrailBake", "TSDFTrailBaker", this)) return false;
            if (_kernel < 0) _kernel = _shader.FindKernel("BakeTrail");
            if (_boxKernel < 0) _boxKernel = _shader.FindKernel("BakeTrailBox");
            return _kernel >= 0 && _boxKernel >= 0;
        }

        // Linearise total threads into a 2D grid (gx, gy) of 64-thread groups,
        // capped at the 65535-per-axis D3D limit.
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
