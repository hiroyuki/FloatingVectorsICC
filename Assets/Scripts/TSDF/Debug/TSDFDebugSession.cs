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

        [Header("Frame seek (hardcoded for now)")]
        [Tooltip("If >= 0 and seekRequested is ticked, jumps every cam's " +
                 "PlaybackCursor to this index and pauses. Use to lock the " +
                 "scene at a known-bad frame for cell-by-cell verification.")]
        public int seekToCursor = -1;
        [Tooltip("Tick true (or hit \"Seek Now\" in the context menu) to apply " +
                 "seekToCursor. Auto-resets after the seek runs.")]
        public bool seekRequested = false;

        [Header("Cache status (read-only)")]
        [SerializeField] private int cachedCount = 0;
        [SerializeField] private string lastReplayKind = "(none)";
        [SerializeField] private ulong lastReplayTimestampUs = 0;
        [Tooltip("Live cursor readout per camera (frame index / total). " +
                 "Use the numbers shown to fill in seekToCursor.")]
        [SerializeField] private string[] currentCursors = new string[0];

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
            public ulong TimestampUs;
        }
        private readonly Dictionary<string, Snapshot> _cache = new Dictionary<string, Snapshot>();

        private PointCloudRecorder _subscribedRecorder;
        private System.Action<string, ObCameraParam?, Transform, RawFrameData> _handler;

        private void OnEnable()
        {
            if (volume == null)     volume = FindAnyObjectByType<TSDFVolume>(FindObjectsInactive.Include);
            if (integrator == null) integrator = FindAnyObjectByType<TSDFIntegrator>(FindObjectsInactive.Include);
            if (recorder == null)   recorder = FindAnyObjectByType<PointCloudRecorder>(FindObjectsInactive.Include);
            if (recorder == null) { Debug.LogWarning("[TSDFDebugSession] No PointCloudRecorder.", this); return; }

            _handler = HandlePlaybackRawFrame;
            recorder.OnPlaybackRawFrame += _handler;
            _subscribedRecorder = recorder;
        }

        private void OnDisable()
        {
            if (_subscribedRecorder != null && _handler != null)
            {
                _subscribedRecorder.OnPlaybackRawFrame -= _handler;
                _subscribedRecorder = null;
                _handler = null;
            }
        }

        // Cache every cam's latest depth as it arrives. The main TSDFIntegrator
        // is still subscribed so live multi-cam accumulation runs in parallel;
        // when the user hits a debug key we override the buffer with our own
        // single-shot integration in LateUpdate.
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
            cachedCount = _cache.Count;
        }

        private void Update()
        {
            HandleKeyboard();
            UpdateCursorReadout();
            if (seekRequested) PerformSeek();
        }

        private void UpdateCursorReadout()
        {
            if (recorder == null) { currentCursors = System.Array.Empty<string>(); return; }
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
            volume.Clear();
            IntegrateSnapshot(snap);
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
            volume.Clear();
            ulong minTs = ulong.MaxValue, maxTs = 0;
            foreach (var snap in _cache.Values)
            {
                IntegrateSnapshot(snap);
                if (snap.TimestampUs < minTs) minTs = snap.TimestampUs;
                if (snap.TimestampUs > maxTs) maxTs = snap.TimestampUs;
            }
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
                colorBytes: null, colorByteCount: 0, colorWidth: 0, colorHeight: 0,
                irBytes: null, irByteCount: 0, irWidth: 0, irHeight: 0,
                timestampUs: snap.TimestampUs);
            integrator.IntegrateRawFrame(snap.Serial, snap.CamParam, snap.SourceTransform, raw);
        }

        [ContextMenu("Resume Playback (let live integration run)")]
        public void ResumePlayback()
        {
            if (recorder != null && recorder.IsPaused) recorder.ResumePlayback();
        }
    }
}
