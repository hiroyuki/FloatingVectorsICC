// Debug-only controller. Caches the latest depth frame per camera while the
// recorder plays, then lets the user step through views with keyboard
// shortcuts WITHOUT advancing the playback clock:
//
//   1..4  show only that camera's latest cached frame
//   0     show all four cameras integrated at the same cached timestamp
//   P     toggle the existing point cloud display
//
// Pairs with TSDFDebugVoxelViz for the single-frame inspection workflow.
// NOT for production. Lives under TSDF/Debug/ so it's trivial to delete.

using System.Collections.Generic;
using Orbbec;
using PointCloud;
using UnityEngine;

namespace TSDF.DebugTools
{
    [AddComponentMenu("TSDF/Debug/Session")]
    public sealed class TSDFDebugSession : MonoBehaviour
    {
        [Header("References (auto-located on enable)")]
        public TSDFVolume volume;
        public TSDFIntegrator integrator;
        public PointCloudRecorder recorder;

        [Header("Keyboard shortcuts (Game View must have focus)")]
        [Tooltip("Number keys 1..4 select the indexed camera from cameraKeys. " +
                 "Leave empty to auto-populate on first key press.")]
        public string[] cameraKeys = new string[0];

        [Tooltip("0 = integrate all cached cameras at once.")]
        public KeyCode allCamsKey = KeyCode.Alpha0;

        [Tooltip("Toggle the existing point-cloud display.")]
        public KeyCode pointCloudToggleKey = KeyCode.P;

        [Tooltip("Toggle the TSDF mesh display (TSDFView.showMesh).")]
        public KeyCode meshToggleKey = KeyCode.M;

        [Header("Frame seek (hardcoded for now)")]
        [Tooltip("If >= 0 and seekRequested is ticked, jumps every cam's " +
                 "PlaybackCursor to this index and pauses. Use to lock the " +
                 "scene at a known-bad frame for cell-by-cell verification.")]
        public int seekToCursor = -1;
        // One-shot trigger via the "Seek Now" context menu. NonSerialized so a
        // ticked state can never bake into the scene and fire on every Play.
        [System.NonSerialized] public bool seekRequested = false;

        // Header "Compare two instants (bench)" + the toggle button are drawn by
        // TSDFDebugSessionEditor so the button sits at the top of this section.
        [Tooltip("Camera serial whose frames to accumulate. LEAVE EMPTY to use ALL " +
                 "cameras (every serial gets folded in at each playhead — multi-view).")]
        public string validateSerial = "";
        [Tooltip("Start playhead in SECONDS from the recording start (the value shown " +
                 "in 'Current Playhead Sec'). The bench integrates the frame at this " +
                 "instant PLUS the frame 'skipFrames' later — two shells in ONE volume " +
                 "with RetainGhost. The result holds (integrator frozen) so you can " +
                 "inspect the raw two-shell state before bridging it.")]
        public double startPlayheadSec = 0.0;
        [Tooltip("How many frames after the start instant the SECOND shell is taken " +
                 "from. 1 = the immediately-next frame; larger = a wider motion gap. " +
                 "The second playhead time is COMPUTED from this camera's frame " +
                 "timestamps (uses validateSerial, or the first camera when blank).")]
        [Min(1)] public int skipFrames = 1;
        [Tooltip("Colour-code the two instants in the baked mesh (dev aid): instant 1 " +
                 "= instant1Color, instant 2 = instant2Color, instead of camera RGB. " +
                 "Visible in the MESH view — the Voxel view colours by SDF, not by " +
                 "this. Off = keep camera colour.")]
        public bool colorByInstant = true;
        public Color instant1Color = Color.red;
        public Color instant2Color = Color.blue;
        [Header("Smooth-union (issue #27)")]
        [Tooltip("Method B: instead of folding the two instants into ONE RetainGhost " +
                 "buffer, integrate them as TWO SEPARATE SDFs and blend with " +
                 "smin(A,B,k) so the shells grow an organic neck. While the bench is " +
                 "frozen, dragging smoothUnionK re-blends the cached pair live (no " +
                 "re-integration).")]
        public bool smoothUnion = false;
        [Tooltip("smin neck radius k in METRES. Larger = fatter / longer neck (and more " +
                 "outward bulge when the two shells sit far apart — the documented " +
                 "caveat). ~1-4 voxels is a sensible start. Drag while frozen to watch " +
                 "the neck change live.")]
        [Range(0.001f, 0.3f)] public float smoothUnionK = 0.03f;
        // One-shot trigger fired by the Compare toggle / "Compare Two Instants"
        // context menu. NonSerialized so a ticked state can never bake into the scene
        // and run the compare (seek + freeze) on every Play — that was the cause of
        // "Play stops on a frame and shows the red/blue bench".
        [System.NonSerialized] public bool buildRequested = false;
        [Tooltip("On entering Play, automatically Read + Play the recorder and run the " +
                 "bench once (no need to press the recorder's Play button by hand — its " +
                 "playback state is otherwise lost on every EnterPlaymode). The build " +
                 "leaves playback paused/frozen at the two fixed instants.")]
        public bool autoRunOnPlay = false;
        [Tooltip("Run the connected-component noise filter (TSDFVolume.RemoveSmallComponents) " +
                 "on the current frozen result — drops small floating islands, keeps the " +
                 "thin main surface. Run while paused (it's a one-shot, not per-frame).")]
        public KeyCode removeNoiseKey = KeyCode.N;
        [Tooltip("Toggle colorByInstant live (camera RGB <-> red/blue instant tags) and " +
                 "immediately re-tag the current result, so you can flip the colour mode " +
                 "during Play without rebuilding by hand.")]
        public KeyCode colorModeKey = KeyCode.C;
        [Tooltip("Toggle the smin SEAM HIGHLIGHT: recolour just the blended neck " +
                 "(where the two instants actually merge) to seamColor and leave the " +
                 "rest of the mesh as-is, so you can see at a glance where smooth-union " +
                 "connected the two shells. Smooth-union bench only. Game View focus.")]
        public KeyCode highlightSeamKey = KeyCode.H;
        [Tooltip("When on, the smin seam is painted seamColor for visual inspection. " +
                 "Toggled live with highlightSeamKey; re-blends the cached pair without " +
                 "re-integrating.")]
        public bool highlightSeam = false;
        [Tooltip("Colour the smin seam is painted when highlightSeam is on.")]
        public Color seamColor = Color.magenta;
        [Tooltip("Blend-strength (0..1) at which a voxel starts counting as seam: lower " +
                 "= a wider band of the transition is painted, higher = only the tightest " +
                 "neck. Drag while frozen to watch the highlighted band change live.")]
        [Range(0f, 1f)] public float seamThreshold = 0.35f;

        [Header("Cache status (read-only)")]
        [SerializeField] private int cachedCount = 0;
        [SerializeField] private string lastReplayKind = "(none)";
        [SerializeField] private ulong lastReplayTimestampUs = 0;
        [Tooltip("Live cursor readout per camera (frame index / total). " +
                 "Use the numbers shown to fill in seekToCursor.")]
        [SerializeField] private string[] currentCursors = new string[0];
        [Tooltip("Live playhead readout in seconds from the recording start — the " +
                 "master timeline value. Copy the numbers shown into startPlayheadSec.")]
        [SerializeField] private string currentPlayheadSec = "(idle)";

        // Per-serial snapshot of the most recently observed depth frame. The
        // recorder's RcsvFrameStream re-uses its scratch byte[], so we copy
        // into our own buffer on every fire; subsequent fires of the same
        // serial overwrite the buffer in place when sizes match.
        private sealed class Snapshot
        {
            public string Serial;
            public ObCameraParam CamParam;
            public Transform SourceTransform;
            public byte[] DepthBytes;
            public int DepthByteCount;
            public int DepthWidth;
            public int DepthHeight;
            public byte[] ColorBytes;   // RGB8; ColorByteCount==0 means no colour this frame
            public int ColorByteCount;
            public int ColorWidth;
            public int ColorHeight;
            public ulong TimestampUs;
            public int Cursor;          // playback cursor this snapshot was emitted at (stale-cache guard)
        }
        private readonly Dictionary<string, Snapshot> _cache = new Dictionary<string, Snapshot>();

        private PointCloudRecorder _subscribedRecorder;
        private System.Action<string, ObCameraParam?, Transform, RawFrameData> _handler;

        // --- Smooth-union (issue #27): two instants held as separate SDFs ---
        // _suVoxA/_suColA and _suVoxB/_suColB cache the two integrated instants so
        // the smin compose can be re-run for a new k without re-integrating. The
        // result lands in the volume's write buffer (so MC extracts it normally).
        private ComputeShader _suShader;
        private int _suCopyKernel = -1, _suUnionKernel = -1;
        private ComputeBuffer _suVoxA, _suColA, _suVoxB, _suColB;
        private int _suBufCount = -1;
        private bool _haveInstants;
        private float _lastComposedK = float.NaN;
        // Seam-highlight params the cache was last composed with, so a live toggle /
        // threshold drag triggers a re-compose just like a smoothUnionK drag does.
        private bool _lastHighlightSeam;
        private float _lastSeamThreshold = float.NaN;
        private bool _kClampActive;   // true while smoothUnionK is being clamped below 4*Tau
        // Grid mapping the cached instants were integrated against. If the volume
        // rebuilds (bbox transform / voxelSize) with the SAME voxel count, the cached
        // SDFs no longer match the new world<->voxel mapping, so invalidate them.
        private Matrix4x4 _suWorldFromVoxel;
        private float _suVoxelSizeAtBuild;

        private void OnEnable()
        {
            if (volume == null)     volume = FindAnyObjectByType<TSDFVolume>(FindObjectsInactive.Include);
            if (integrator == null) integrator = FindAnyObjectByType<TSDFIntegrator>(FindObjectsInactive.Include);
            if (recorder == null)   recorder = FindAnyObjectByType<PointCloudRecorder>(FindObjectsInactive.Include);
            if (recorder == null) { Debug.LogWarning("[TSDFDebugSession] No PointCloudRecorder.", this); return; }

            _handler = HandlePlaybackRawFrame;
            recorder.OnPlaybackRawFrame += _handler;
            _subscribedRecorder = recorder;

            if (autoRunOnPlay && Application.isPlaying)
                StartCoroutine(AutoRunCo());
        }

        // Wait a couple of frames so the recorder's own enable/start ran, then
        // Read+Play+Build automatically. CompareTwoInstants self-starts playback,
        // so this only needs to fire it once.
        private System.Collections.IEnumerator AutoRunCo()
        {
            yield return null;
            yield return null;
            CompareTwoInstants();   // seek to startPlayheadSec, build the two fixed instants, paused
        }

        private void OnDisable()
        {
            if (_subscribedRecorder != null && _handler != null)
            {
                _subscribedRecorder.OnPlaybackRawFrame -= _handler;
                _subscribedRecorder = null;
                _handler = null;
            }
            ReleaseSmoothUnionBuffers();
        }

        // Cache every cam's latest depth as it arrives. The main TSDFIntegrator
        // is still subscribed so live multi-cam accumulation runs in parallel;
        // when the user hits a debug key we override the buffer with our own
        // single-shot integration (1..4 / 0 replay, or the fixed-frame bench).
        private void HandlePlaybackRawFrame(string serial, ObCameraParam? camParam,
                                            Transform sourceTransform, RawFrameData raw)
        {
            if (string.IsNullOrEmpty(serial)) return;
            if (!camParam.HasValue) return;
            if (raw.DepthBytes == null || raw.DepthByteCount <= 0) return;

            if (!_cache.TryGetValue(serial, out var snap))
            {
                snap = new Snapshot { Serial = serial };
                _cache[serial] = snap;
            }
            if (snap.DepthBytes == null || snap.DepthBytes.Length < raw.DepthByteCount)
                snap.DepthBytes = new byte[raw.DepthByteCount];
            System.Buffer.BlockCopy(raw.DepthBytes, 0, snap.DepthBytes, 0, raw.DepthByteCount);
            snap.DepthByteCount = raw.DepthByteCount;
            snap.DepthWidth = raw.DepthWidth;
            snap.DepthHeight = raw.DepthHeight;
            snap.CamParam = camParam.Value;
            snap.SourceTransform = sourceTransform;
            snap.TimestampUs = raw.TimestampUs;
            snap.Cursor = recorder != null ? recorder.GetPlaybackCursor(serial) : -1;
            // Cache the colour frame too so colorByInstant=false bakes real camera RGB
            // (without this the integrator gets no colour and SampleColor returns grey).
            bool hasColor = raw.ColorBytes != null && raw.ColorByteCount > 0
                            && raw.ColorWidth > 0 && raw.ColorHeight > 0;
            if (hasColor)
            {
                if (snap.ColorBytes == null || snap.ColorBytes.Length < raw.ColorByteCount)
                    snap.ColorBytes = new byte[raw.ColorByteCount];
                System.Buffer.BlockCopy(raw.ColorBytes, 0, snap.ColorBytes, 0, raw.ColorByteCount);
                snap.ColorByteCount = raw.ColorByteCount;
                snap.ColorWidth = raw.ColorWidth;
                snap.ColorHeight = raw.ColorHeight;
            }
            else snap.ColorByteCount = 0;
            cachedCount = _cache.Count;
        }

        private void Update()
        {
            HandleKeyboard();
            UpdateCursorReadout();
            if (seekRequested) PerformSeek();
            if (Input.GetKeyDown(removeNoiseKey) && volume != null) volume.RemoveSmallComponents();
            if (Input.GetKeyDown(colorModeKey))
            {
                // Flip camera-RGB <-> red/blue tags and re-tag immediately so the
                // change is visible without a manual rebuild (works paused too).
                colorByInstant = !colorByInstant;
                CompareTwoInstants();
            }
            if (Input.GetKeyDown(highlightSeamKey))
            {
                // Recolour just the smin neck. Only the smooth-union bench carries
                // seam info; re-compose immediately so it shows without a k drag.
                highlightSeam = !highlightSeam;
                Debug.Log($"[TSDFDebugSession] smin seam highlight = {highlightSeam}", this);
                if (smoothUnion && SmoothUnionCacheValid()) ComposeSmoothUnion();
                else Debug.Log("[TSDFDebugSession] (no smooth-union cache; enable smoothUnion " +
                               "and toggle Compare on first)", this);
            }
            if (buildRequested) CompareTwoInstants();   // context-menu / tick = one-shot frozen pair

            // Live k tuning: while the smin bench is frozen (paused / idle), dragging
            // smoothUnionK re-blends the cached A/B pair without re-integrating. Skip
            // while actively playing so we don't fight the realtime trailing path (the
            // trailing path also clears _haveInstants when it runs, so this only ever
            // touches a freshly-built, still-current smin cache).
            if (smoothUnion && SmoothUnionCacheValid()
                && !IsRecorderActivelyPlaying()
                && (!Mathf.Approximately(smoothUnionK, _lastComposedK)
                    || highlightSeam != _lastHighlightSeam
                    || !Mathf.Approximately(seamThreshold, _lastSeamThreshold)))
                ComposeSmoothUnion();
        }

        private bool IsRecorderActivelyPlaying()
            => recorder != null
               && recorder.CurrentState == PointCloudRecorder.State.Playing
               && !recorder.IsPaused;

        // The cached instants are usable only while the volume still has the same
        // voxel count AND the same world<->voxel mapping they were integrated against
        // (a bbox/voxelSize edit rebuilds the grid and the cache would project wrong).
        private bool SmoothUnionCacheValid()
        {
            if (!_haveInstants || volume == null) return false;
            int total = volume.Dim.x * volume.Dim.y * volume.Dim.z;
            if (_suBufCount != total) return false;
            if (!Mathf.Approximately(_suVoxelSizeAtBuild, volume.voxelSize)) return false;
            return _suWorldFromVoxel == volume.WorldFromVoxel;
        }

        [ContextMenu("Remove Small Components (noise)")]
        public void TriggerRemoveNoise() { if (volume != null) volume.RemoveSmallComponents(); }

        private void UpdateCursorReadout()
        {
            if (recorder == null) { currentCursors = System.Array.Empty<string>(); currentPlayheadSec = "(no recorder)"; return; }
            currentPlayheadSec = recorder.CurrentState == PointCloudRecorder.State.Playing
                ? recorder.CurrentPlayheadSeconds.ToString("0.000")
                : "(not playing)";
            if (cameraKeys == null || cameraKeys.Length == 0) AutoPopulateCameraKeys();
            if (cameraKeys == null) return;
            if (currentCursors == null || currentCursors.Length != cameraKeys.Length)
                currentCursors = new string[cameraKeys.Length];
            for (int i = 0; i < cameraKeys.Length; i++)
            {
                string s = cameraKeys[i];
                if (string.IsNullOrEmpty(s)) { currentCursors[i] = "(empty)"; continue; }
                int cur = recorder.GetPlaybackCursor(s);
                int tot = recorder.GetTrackFrameCount(s);
                string tail = s.Length > 6 ? s.Substring(s.Length - 6) : s;
                currentCursors[i] = (cur < 0 || tot < 0) ? $"{tail}: unknown" : $"{tail}: {cur} / {tot}";
            }
        }

        [ContextMenu("Seek Now")]
        public void TriggerSeek() { seekRequested = true; }

        private void PerformSeek()
        {
            seekRequested = false;
            if (recorder == null || seekToCursor < 0) return;
            bool moved = recorder.SeekAllTracksTo(seekToCursor);
            Debug.Log($"[TSDFDebugSession] SeekAllTracksTo({seekToCursor}) moved={moved}", this);
        }

        /// <summary>True while the two-instant compare is held (integrator frozen).
        /// The Inspector exposes this as a single toggle button.</summary>
        public bool IsComparing { get; private set; }

        /// <summary>One-button compare: enter the two-instant compare if not in it,
        /// otherwise exit back to normal playback.</summary>
        public void ToggleCompare()
        {
            if (IsComparing) ResumePlayback();   // exit -> resume normal playback
            else buildRequested = true;          // enter -> built on the next Update tick
        }

        [ContextMenu("Compare Two Instants")]
        public void TriggerCompareTwoInstants() { buildRequested = true; }

        /// <summary>
        /// Get the recorder into Playing state without the user touching its Play
        /// button: Read (Load) the recording if nothing is loaded yet, then start
        /// playback. No-op if already playing.
        /// </summary>
        private void EnsureRecorderPlaying()
        {
            if (recorder == null || recorder.CurrentState == PointCloudRecorder.State.Playing) return;
            var tracks = recorder.GetRecordedDepthTracks();
            if (tracks == null || tracks.Count == 0) recorder.Load();   // "Read"
            recorder.TogglePlay();                                      // Idle -> Playing
        }

        /// <summary>
        /// Accumulate two instants — the frame at <see cref="startPlayheadSec"/> and
        /// the frame <see cref="skipFrames"/> later — into a single volume with
        /// RetainGhost, then freeze. The second playhead is COMPUTED from a reference
        /// camera's frame timestamps (so you give a frame offset, not a second time).
        /// Each instant is seeked by TIME (every camera lands on its own
        /// timestamp-matched frame) and integrated WITHOUT clearing between, so the
        /// two instants land as two shells — the raw "do they overlap or sit apart?"
        /// state we want to inspect before choosing how to bridge them. When
        /// <see cref="validateSerial"/> is blank, ALL cameras are folded in at each
        /// instant (multi-view) and the first camera is the timing reference. The
        /// live integrator is gated OFF during the seeks so it can't fold frames in
        /// twice; our IntegrateRawFrame calls bypass that gate.
        /// </summary>
        public void CompareTwoInstants()
        {
            buildRequested = false;
            if (recorder == null || volume == null || integrator == null)
            {
                Debug.LogWarning("[TSDFDebugSession] Build needs recorder + volume + integrator.", this);
                return;
            }
            if (recorder.CurrentState != PointCloudRecorder.State.Playing)
            {
                EnsureRecorderPlaying();
                if (recorder.CurrentState != PointCloudRecorder.State.Playing)
                {
                    Debug.LogWarning("[TSDFDebugSession] Could not start playback — is a recording set/loaded " +
                                     "(recorder folderPath)? Press Read manually if needed.", this);
                    return;
                }
            }

            if (cameraKeys == null || cameraKeys.Length == 0) AutoPopulateCameraKeys();

            // Cameras to fold in (one serial, or ALL when blank) and the reference
            // camera used to convert the skipFrames offset into a playhead time.
            string[] serials = !string.IsNullOrEmpty(validateSerial)
                ? new[] { validateSerial }
                : cameraKeys;
            if (serials == null || serials.Length == 0 || string.IsNullOrEmpty(serials[0]))
            {
                Debug.LogWarning("[TSDFDebugSession] No validateSerial set and no cameraKeys to fall back on.", this);
                return;
            }
            string refSerial = serials[0];

            // Freeze live integration during the seeks (its event handler honors
            // this gate; our direct IntegrateRawFrame calls do not) and force
            // RetainGhost so the frames accumulate as separate shells. Save the prior
            // state first so an early abort below (bad reference cursor) can restore
            // it — otherwise a failed build silently leaves live integration frozen.
            bool priorIntegEnabled = integrator.integrationEnabled;
            var priorAccum = volume.accumulationMode;
            integrator.integrationEnabled = false;
            volume.accumulationMode = TSDFVolume.AccumulationMode.RetainGhost;

            PauseRecorder();

            // Instant 1: seek to the start playhead, then compute instant 2 from a
            // frame offset on the reference camera's track.
            recorder.SeekToPlayheadSeconds(startPlayheadSec);
            int refCursor = recorder.GetPlaybackCursor(refSerial);
            if (refCursor < 0)
            {
                Debug.LogWarning($"[TSDFDebugSession] Reference camera '{refSerial}' has no cursor at " +
                                 $"{startPlayheadSec:0.000}s — aborting.", this);
                integrator.integrationEnabled = priorIntegEnabled;   // restore — nothing was integrated
                volume.accumulationMode = priorAccum;
                return;
            }
            int total = recorder.GetTrackFrameCount(refSerial);
            int targetCursor = refCursor + Mathf.Max(1, skipFrames);
            if (total > 0 && targetCursor > total - 1)
            {
                Debug.LogWarning($"[TSDFDebugSession] start cursor {refCursor} + {skipFrames} exceeds last " +
                                 $"frame {total - 1} on '{refSerial}' — clamping. Pick an earlier start.", this);
                targetCursor = total - 1;
            }
            double secondSec = recorder.GetFramePlayheadSeconds(refSerial, targetCursor);
            if (targetCursor == refCursor)
                Debug.LogWarning("[TSDFDebugSession] Second instant equals the first (skip clamped) — " +
                                 "you'll see only one shell.", this);

            int done = 0;
            bool smin = smoothUnion && EnsureSmoothUnionShader();
            if (smin)
            {
                // Method B (issue #27): integrate each instant into its OWN sdf+colour
                // buffer (clear the write buffer BEFORE each one), snapshot it out, then
                // smin(A,B,k) -> write buffer. The two shells stay independent so the
                // blend produces an organic neck instead of two flat RetainGhost shells.
                int voxelTotal = volume.Dim.x * volume.Dim.y * volume.Dim.z;
                EnsureSmoothUnionBuffers(voxelTotal);

                volume.ClearWrite();
                if (colorByInstant) integrator.colorOverride = instant1Color;
                done += IntegrateAllCached(serials, $"instant1 @ {startPlayheadSec:0.000}s (SDF A)");
                SnapshotWriteBuffer(_suVoxA, _suColA, voxelTotal);

                recorder.SeekToPlayheadSeconds(secondSec);
                volume.ClearWrite();
                if (colorByInstant) integrator.colorOverride = instant2Color;
                done += IntegrateAllCached(serials, $"instant2 @ {secondSec:0.000}s (+{skipFrames}f, SDF B)");
                SnapshotWriteBuffer(_suVoxB, _suColB, voxelTotal);

                if (colorByInstant) integrator.colorOverride = new Color(0f, 0f, 0f, 0f);

                _haveInstants = true;
                _suWorldFromVoxel = volume.WorldFromVoxel;   // mapping the cache is valid for
                _suVoxelSizeAtBuild = volume.voxelSize;
                ComposeSmoothUnion();   // smin(A,B,k) -> write buffer + Publish
            }
            else
            {
                // Method-neutral baseline: both instants fold into ONE RetainGhost
                // buffer as two co-existing shells (the raw "overlap or apart?" view).
                _haveInstants = false;

                volume.ClearWrite();   // clear ONCE; both instants accumulate

                if (colorByInstant) integrator.colorOverride = instant1Color;
                done += IntegrateAllCached(serials, $"instant1 @ {startPlayheadSec:0.000}s");

                recorder.SeekToPlayheadSeconds(secondSec);
                if (colorByInstant) integrator.colorOverride = instant2Color;
                done += IntegrateAllCached(serials, $"instant2 @ {secondSec:0.000}s (+{skipFrames}f)");

                // Reset so live integration / later builds use camera colour again.
                if (colorByInstant) integrator.colorOverride = new Color(0f, 0f, 0f, 0f);

                volume.Publish();
            }

            string who = !string.IsNullOrEmpty(validateSerial)
                ? validateSerial.Substring(System.Math.Max(0, validateSerial.Length - 6))
                : $"ALL x{serials.Length}";
            string mode = smin ? $"smooth-union k={smoothUnionK:0.000}m (separate SDFs)" : "RetainGhost (one buffer)";
            lastReplayKind = $"fixed[{done}] ({who}) {(smin ? "smin" : "ghost")}";
            IsComparing = true;
            Debug.Log($"[TSDFDebugSession] Compared two instants on {who}: " +
                      $"start={startPlayheadSec:0.000}s (cursor {refCursor}) + {skipFrames}f " +
                      $"-> {secondSec:0.000}s (cursor {targetCursor}). {done} integrations, {mode}. " +
                      "Integrator FROZEN — toggle Compare off (or Resume Playback) to go live.", this);
        }

        // ---------------- Smooth-union (issue #27) ----------------

        private bool EnsureSmoothUnionShader()
        {
            if (_suShader != null) return true;
            _suShader = Resources.Load<ComputeShader>("TSDFSmoothUnion");
            if (_suShader == null)
            {
                Debug.LogError("[TSDFDebugSession] Compute shader \"Resources/TSDFSmoothUnion.compute\" " +
                               "not found — falling back to RetainGhost.", this);
                return false;
            }
            _suCopyKernel = _suShader.FindKernel("Copy");
            _suUnionKernel = _suShader.FindKernel("SmoothUnion");
            return true;
        }

        private void EnsureSmoothUnionBuffers(int total)
        {
            if (total <= 0) return;
            if (_suBufCount == total && _suVoxA != null) return;
            ReleaseSmoothUnionBuffers();
            _suVoxA = new ComputeBuffer(total, sizeof(float) * 2);
            _suColA = new ComputeBuffer(total, sizeof(float) * 4);
            _suVoxB = new ComputeBuffer(total, sizeof(float) * 2);
            _suColB = new ComputeBuffer(total, sizeof(float) * 4);
            _suBufCount = total;
            _haveInstants = false;   // freshly (re)allocated — no valid instants cached yet
        }

        private void ReleaseSmoothUnionBuffers()
        {
            _suVoxA?.Release(); _suColA?.Release();
            _suVoxB?.Release(); _suColB?.Release();
            _suVoxA = _suColA = _suVoxB = _suColB = null;
            _suBufCount = -1;
            _haveInstants = false;
        }

        // Copy the freshly-integrated instant out of the volume's write buffer into
        // a scratch (sdf, colour) pair so the next instant can reuse the write buffer.
        private void SnapshotWriteBuffer(ComputeBuffer dstVox, ComputeBuffer dstCol, int total)
        {
            if (_suShader == null || _suCopyKernel < 0) return;
            if (volume == null || volume.WriteBuffer == null) return;
            SuDispatchDims(total, out int gx, out int gy);
            _suShader.SetInts("_Dim", volume.Dim.x, volume.Dim.y, volume.Dim.z);
            _suShader.SetInt("_DispatchWidth", gx * 64);
            _suShader.SetBuffer(_suCopyKernel, "_SrcVoxels", volume.WriteBuffer);
            _suShader.SetBuffer(_suCopyKernel, "_SrcColors", volume.WriteColorBuffer);
            _suShader.SetBuffer(_suCopyKernel, "_DstVoxels", dstVox);
            _suShader.SetBuffer(_suCopyKernel, "_DstColors", dstCol);
            _suShader.Dispatch(_suCopyKernel, Mathf.Max(1, gx), Mathf.Max(1, gy), 1);
        }

        // smin(A, B, k) -> volume write buffer, then publish so MC re-extracts. Used
        // both by CompareTwoInstants and by the live-k recompose in Update.
        private void ComposeSmoothUnion()
        {
            if (!_haveInstants || volume == null || volume.WriteBuffer == null) return;
            if (!EnsureSmoothUnionShader()) return;
            int total = volume.Dim.x * volume.Dim.y * volume.Dim.z;
            if (total <= 0 || _suBufCount != total) return;   // grid changed under us — re-run Build

            SuDispatchDims(total, out int gx, out int gy);
            _suShader.SetInts("_Dim", volume.Dim.x, volume.Dim.y, volume.Dim.z);
            _suShader.SetFloat("_Tau", volume.Tau);
            // The smin seam dip is k/4 at its deepest. Co-observed free space sits at
            // +Tau, so k >= 4*Tau would pull it below the iso and Marching Cubes would
            // emit phantom geometry in observed open space (issue #27 caveat). Keep k
            // strictly under 4*Tau with a margin; the shader also guards this per-voxel.
            float kMax = 3.8f * volume.Tau;
            float kEff = Mathf.Min(smoothUnionK, kMax);
            bool clamped = kEff < smoothUnionK;
            if (clamped && !_kClampActive)
                Debug.LogWarning($"[TSDFDebugSession] smoothUnionK {smoothUnionK:0.000}m exceeds the safe " +
                                 $"limit {kMax:0.000}m (=3.8*Tau, Tau={volume.Tau:0.000}m); clamped to avoid " +
                                 "phantom free-space geometry. Lower smoothUnionK or raise the volume's Tau.", this);
            _kClampActive = clamped;
            _suShader.SetFloat("_K", kEff);
            // Debug seam highlight: paint the blended neck seamColor (visual only —
            // sdf is untouched). Map [threshold..1] blend strength onto the ramp.
            _suShader.SetInt("_HighlightSeam", highlightSeam ? 1 : 0);
            _suShader.SetVector("_SeamColor", seamColor);
            _suShader.SetVector("_SeamRange", new Vector4(Mathf.Clamp01(seamThreshold), 1f, 0f, 0f));
            _suShader.SetInt("_DispatchWidth", gx * 64);
            _suShader.SetBuffer(_suUnionKernel, "_VoxelsA", _suVoxA);
            _suShader.SetBuffer(_suUnionKernel, "_ColorsA", _suColA);
            _suShader.SetBuffer(_suUnionKernel, "_VoxelsB", _suVoxB);
            _suShader.SetBuffer(_suUnionKernel, "_ColorsB", _suColB);
            _suShader.SetBuffer(_suUnionKernel, "_VoxelsOut", volume.WriteBuffer);
            _suShader.SetBuffer(_suUnionKernel, "_ColorsOut", volume.WriteColorBuffer);
            _suShader.Dispatch(_suUnionKernel, Mathf.Max(1, gx), Mathf.Max(1, gy), 1);

            volume.Publish();
            _lastComposedK = smoothUnionK;
            _lastHighlightSeam = highlightSeam;
            _lastSeamThreshold = seamThreshold;
        }

        // 2D dispatch grid (mirrors TSDFVolume/Integrator): keeps large volumes under
        // the 65535 threadgroup-per-axis D3D limit; the kernels linearise via
        // _DispatchWidth = gx * 64.
        private static void SuDispatchDims(int total, out int gx, out int gy)
        {
            int groups = Mathf.CeilToInt(total / 64f);
            gx = Mathf.Max(1, Mathf.Min(groups, 65535));
            gy = Mathf.Max(1, Mathf.CeilToInt(groups / (float)gx));
        }

        /// <summary>Integrate the currently-cached snapshot of every serial in
        /// <paramref name="serials"/>. Returns how many integrated.</summary>
        private int IntegrateAllCached(string[] serials, string label)
        {
            int n = 0;
            foreach (string s in serials)
            {
                if (string.IsNullOrEmpty(s)) continue;
                if (!_cache.TryGetValue(s, out var snap) || snap == null)
                {
                    Debug.LogWarning($"[TSDFDebugSession] No cached frame for '{s}' ({label}).", this);
                    continue;
                }
                // A seek that doesn't move a track leaves _cache at its last-emitted
                // cursor; only integrate when the cache is actually the frame now at
                // the playhead (guards stale data on repeated/all-cam/end seeks).
                int curIdx = recorder.GetPlaybackCursor(s);
                if (snap.Cursor != curIdx)
                {
                    Debug.LogWarning($"[TSDFDebugSession] stale cache for '{s}' (cache@{snap.Cursor}, " +
                                     $"playhead@{curIdx}) — skipping ({label}).", this);
                    continue;
                }
                IntegrateSnapshot(snap);   // RetainGhost accumulate, no clear between
                n++;
            }
            return n;
        }

        private void HandleKeyboard()
        {
            if (Input.GetKeyDown(pointCloudToggleKey))
            {
                var pcv = FindAnyObjectByType<PointCloudView>(FindObjectsInactive.Include);
                if (pcv != null)
                {
                    pcv.showPointClouds = !pcv.showPointClouds;
                    Debug.Log($"[TSDFDebugSession] PointCloudView.showPointClouds = {pcv.showPointClouds}", this);
                }
            }

            if (Input.GetKeyDown(meshToggleKey))
            {
                var view = FindAnyObjectByType<TSDFView>(FindObjectsInactive.Include);
                if (view != null)
                {
                    view.showMesh = !view.showMesh;
                    Debug.Log($"[TSDFDebugSession] TSDFView.showMesh = {view.showMesh}", this);
                }
            }

            // 1..4 — single-cam view from cache, time frozen.
            for (int i = 0; i < 4; i++)
            {
                KeyCode key = (KeyCode)((int)KeyCode.Alpha1 + i);
                if (!Input.GetKeyDown(key)) continue;
                if (cameraKeys == null || cameraKeys.Length == 0) AutoPopulateCameraKeys();
                if (cameraKeys == null || i >= cameraKeys.Length)
                {
                    Debug.LogWarning($"[TSDFDebugSession] cameraKeys[{i}] not configured.", this);
                    continue;
                }
                string serial = cameraKeys[i];
                PauseRecorder();
                ReplaySingleCam(serial);
            }

            // 0 — all cams from cache at the cached timestamps. Pause first so
            // the live integrator stops competing with us.
            if (Input.GetKeyDown(allCamsKey))
            {
                PauseRecorder();
                ReplayAllCams();
            }
        }

        private void AutoPopulateCameraKeys()
        {
            if (recorder == null) return;
            var tracks = recorder.GetRecordedDepthTracks();
            if (tracks == null) return;
            var list = new List<string>(tracks.Count);
            foreach (var t in tracks)
                if (!string.IsNullOrEmpty(t.Serial)) list.Add(t.Serial);
            cameraKeys = list.ToArray();
            Debug.Log($"[TSDFDebugSession] Auto-populated cameraKeys: [{string.Join(", ", cameraKeys)}]", this);
        }

        private void PauseRecorder()
        {
            if (recorder != null
                && recorder.CurrentState == PointCloudRecorder.State.Playing
                && !recorder.IsPaused)
            {
                recorder.PausePlayback();
            }
        }

        private void ReplaySingleCam(string serial)
        {
            if (!_cache.TryGetValue(serial, out var snap))
            {
                Debug.LogWarning($"[TSDFDebugSession] No cached frame yet for serial '{serial}'.", this);
                return;
            }
            if (volume == null || integrator == null) return;
            volume.ClearWrite();
            IntegrateSnapshot(snap);
            volume.Publish();
            lastReplayKind = $"single ({serial.Substring(System.Math.Max(0, serial.Length - 6))})";
            lastReplayTimestampUs = snap.TimestampUs;
            Debug.Log($"[TSDFDebugSession] Replayed single cam '{serial}' @ {snap.TimestampUs}us", this);
        }

        private void ReplayAllCams()
        {
            if (_cache.Count == 0)
            {
                Debug.LogWarning("[TSDFDebugSession] Cache empty — let playback run first to fill it.", this);
                return;
            }
            if (volume == null || integrator == null) return;
            volume.ClearWrite();
            ulong minTs = ulong.MaxValue, maxTs = 0;
            foreach (var snap in _cache.Values)
            {
                IntegrateSnapshot(snap);
                if (snap.TimestampUs < minTs) minTs = snap.TimestampUs;
                if (snap.TimestampUs > maxTs) maxTs = snap.TimestampUs;
            }
            volume.Publish();
            lastReplayKind = $"all ({_cache.Count})";
            lastReplayTimestampUs = maxTs;
            ulong spreadUs = maxTs > minTs ? (maxTs - minTs) : 0;
            Debug.Log($"[TSDFDebugSession] Replayed {_cache.Count} cams (ts spread={spreadUs / 1000.0:F1}ms)", this);
        }

        private void IntegrateSnapshot(Snapshot snap)
        {
            var raw = new RawFrameData(
                depthBytes: snap.DepthBytes, depthByteCount: snap.DepthByteCount,
                depthWidth: snap.DepthWidth, depthHeight: snap.DepthHeight,
                colorBytes: snap.ColorByteCount > 0 ? snap.ColorBytes : null,
                colorByteCount: snap.ColorByteCount,
                colorWidth: snap.ColorWidth, colorHeight: snap.ColorHeight,
                irBytes: null, irByteCount: 0, irWidth: 0, irHeight: 0,
                timestampUs: snap.TimestampUs);
            integrator.IntegrateRawFrame(snap.Serial, snap.CamParam, snap.SourceTransform, raw);
        }

        // Exit the two-instant compare back to normal continuous playback: re-enable
        // integration, clear the frozen compare mesh, and resume the recorder so the
        // volume refills from live frames in camera colour.
        [ContextMenu("Resume Playback (exit compare)")]
        public void ResumePlayback()
        {
            if (integrator != null)
            {
                integrator.integrationEnabled = true;
                integrator.BeginFreshBatch();   // drop the frozen compare result
            }
            if (recorder != null && recorder.IsPaused) recorder.ResumePlayback();
            IsComparing = false;
        }
    }
}
