// Bone-verify mode: one-switch orchestrator that stands up the whole v11
// bone-verification rig and tears it down cleanly. Lives as a single standby
// GameObject in the scene (always enabled, listening for the hotkeys); the
// diagnostic VIEWS are off until entered.
//
// Two modes:
//  - LIVE (F8):   display[gridDisplay] = colour grid + per-camera 2D detection
//                 (+ REC dot while recording); display[sceneDisplay] = point
//                 cloud + merged v11 bones + motion curves.
//  - REVIEW (F7): loads a recorded take, pauses, and steps frame-by-frame
//                 (←/→). The colour grid shows BOTH the recorded fused skeleton
//                 reprojected (cyan, deterministic) and the per-camera 2D
//                 re-detection (orange); the 3D view shows the recorded cloud +
//                 merged bones + curves, all advancing one frame per step.
//
// Invocation (all four, per design): F8/F7 + ←/→ hotkeys, an operator button
// panel on display 1, and reflection-callable EnterLive/EnterPlaybackReview/
// Step/Exit for AI/MCP driving.
//
// Display assignment is machine-local (display2/3 are physically swapped on the
// 5080 set), so gridDisplay/sceneDisplay are fields, never hardcoded downstream.

using System;
using System.IO;
using PointCloud;
using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.UI;

namespace BodyTracking.Eval.Rtmpose
{
    [DisallowMultipleComponent]
    public class BoneVerifyController : MonoBehaviour
    {
        public enum Mode { Off, Live, Review }

        [Header("Displays (machine-local; display2/3 are swapped on the 5080 set)")]
        [Tooltip("OS display index for the colour grid (physical display2 = 2 here).")]
        public int gridDisplay = 2;
        [Tooltip("OS display index that keeps the 3D point cloud + merged bones + curves.")]
        public int sceneDisplay = 1;

        [Header("Hotkeys")]
        public KeyCode liveKey = KeyCode.F8;
        public KeyCode reviewKey = KeyCode.F7;
        public KeyCode stepForwardKey = KeyCode.RightArrow;
        public KeyCode stepBackwardKey = KeyCode.LeftArrow;

        [Header("Review")]
        [Tooltip("Take folder for review. Empty = the recorder's playbackFolderPath, else " +
                 "the newest take under the recorder's folderPath.")]
        public string reviewTakePath = "";

        [Header("Live sync (display3 timestamp-matched cloud delay)")]
        [Tooltip("Delay the live point cloud so it lines up with the ~130ms-delayed " +
                 "skeleton + curves (measured cloud-skel lag). Display-only: BT and " +
                 "recording keep getting the newest frame. Off = cloud stays real-time " +
                 "(skeleton visibly trails the body during motion).")]
        public bool enableLiveRenderSync = true;

        [Tooltip("Extra delay (ms) ADDED on top of the automatic timestamp match. The " +
                 "auto-match aligns the cloud to the skeleton's frame timestamp (~130-166ms " +
                 "measured); this nudges it further to compensate the drawn skeleton's " +
                 "smoothing lag. Tune by eye while moving: raise until the cloud sits on " +
                 "the body. 0 = pure timestamp match.")]
        [Range(0f, 120f)] public float extraDelayMs = 0f;

        [Header("References (auto-resolve when empty)")]
        public SensorManager sensorManager;
        public SensorRecorder recorder;
        public SkeletonMerger merger;
        public LiveFusedBodySource fused;
        public PointCloudMotionCurves curves;

        public Mode CurrentMode { get; private set; } = Mode.Off;

        private CameraColorGridOverlay _overlay;
        private GameObject _overlayGo;

        // saved scene state (captured on the first Off->non-Off transition)
        private bool _saved;
        private bool _sShowBones, _sExternal, _sCurvesVisible, _sUseV11s;
        private bool _sFusedLiveOnly, _sFusedPlaybackOnly, _sFusedSubmit, _sFusedSyncReview;

        // operator UI
        private Canvas _opCanvas;
        private Text _opStatus;

        private void OnEnable()
        {
            ResolveRefs();
            BuildOperatorUI();
        }

        private void OnDisable()
        {
            if (CurrentMode != Mode.Off) Exit();
            if (_opCanvas != null) { Destroy(_opCanvas.gameObject); _opCanvas = null; }
        }

        private void ResolveRefs()
        {
            if (sensorManager == null) sensorManager = FindFirstObjectByType<SensorManager>();
            if (recorder == null) recorder = FindFirstObjectByType<SensorRecorder>();
            if (merger == null) merger = FindFirstObjectByType<SkeletonMerger>();
            if (fused == null) fused = FindFirstObjectByType<LiveFusedBodySource>();
            if (curves == null) curves = FindFirstObjectByType<PointCloudMotionCurves>();
        }

        private void Update()
        {
            if (Input.GetKeyDown(liveKey)) { if (CurrentMode == Mode.Live) Exit(); else EnterLive(); }
            if (Input.GetKeyDown(reviewKey)) { if (CurrentMode == Mode.Review) Exit(); else EnterPlaybackReview(null); }
            if (CurrentMode == Mode.Review)
            {
                if (Input.GetKeyDown(stepForwardKey)) Step(+1);
                if (Input.GetKeyDown(stepBackwardKey)) Step(-1);
            }
            if (CurrentMode == Mode.Live && enableLiveRenderSync) UpdateLiveRenderSync();
            UpdateOperatorStatus();
        }

        // Feed every live renderer the source timestamp of the currently-shown
        // skeleton so its timestamp-matched render delay draws the matching cloud
        // frame. TryGetLatestSkeletonTimestampNs holds through brief tracking gaps
        // (its freshness window) and reports false on a sustained absence — then we
        // target 0 so the cloud returns to real-time instead of freezing at the
        // stale target once it falls behind the whole delay ring.
        private void UpdateLiveRenderSync()
        {
            ulong tsUs = 0UL; // 0 = show newest (real-time) — no fresh skeleton to sync to
            if (merger != null && merger.TryGetLatestSkeletonTimestampNs(out ulong tsNs))
            {
                tsUs = tsNs / 1000UL;
                if (extraDelayMs > 0f)
                {
                    ulong off = (ulong)(extraDelayMs * 1000f); // ms -> µs, shift target earlier = more delay
                    tsUs = tsUs > off ? tsUs - off : 0UL;
                }
            }
            foreach (var r in FindObjectsByType<PointCloudRenderer>(FindObjectsSortMode.None))
                if (r != null) r.targetDisplayTimestampUs = tsUs;
        }

        private void SetLiveRenderDelay(bool on)
        {
            foreach (var r in FindObjectsByType<PointCloudRenderer>(FindObjectsSortMode.None))
            {
                if (r == null) continue;
                r.renderDelayEnabled = on;
                if (!on) r.targetDisplayTimestampUs = 0;
            }
        }

        // ---------------- public API (reflection / UI / hotkey) ----------------

        public void EnterLive()
        {
            ResolveRefs();
            SaveStateIfNeeded();

            // live cameras up and visible; not suppressed as sculpture source
            if (sensorManager != null)
            {
                if (!HasLiveRenderers()) TryStartLive();
                sensorManager.SetLiveVisualsVisible(true);
                sensorManager.SetLiveSuppressedAsSource(false);
            }
            if (recorder != null && recorder.IsPlaying) recorder.TogglePlay(); // stop any playback

            if (merger != null) merger.showBones = true;
            if (curves != null) curves.visible = true;

            // live fusion feeding the merger (v11 -> merged bones + bodies_v11s while recording)
            if (fused != null)
            {
                fused.liveFramesOnly = false;
                fused.playbackFramesOnly = false;
                fused.syncReviewFusion = false;
                fused.SetSubmitToMerger(true);
            }

            RecreateOverlay(review: false);
            SetLiveRenderDelay(enableLiveRenderSync);
            CurrentMode = Mode.Live;
            Debug.Log($"[{nameof(BoneVerifyController)}] LIVE verify — grid on display{gridDisplay + 1}, 3D+bones+curves on display{sceneDisplay + 1}.", this);
        }

        public void EnterPlaybackReview(string path)
        {
            ResolveRefs();
            if (recorder == null) { Debug.LogError($"[{nameof(BoneVerifyController)}] no SensorRecorder.", this); return; }
            SaveStateIfNeeded();

            string take = !string.IsNullOrEmpty(path) ? path
                        : !string.IsNullOrEmpty(reviewTakePath) ? reviewTakePath
                        : !string.IsNullOrEmpty(recorder.playbackFolderPath) ? recorder.playbackFolderPath
                        : NewestTakeUnder(recorder.folderPath);
            if (string.IsNullOrEmpty(take) || !Directory.Exists(take))
            {
                Debug.LogError($"[{nameof(BoneVerifyController)}] review take not found: '{take}'.", this);
                return;
            }
            recorder.playbackFolderPath = take;
            recorder.useV11sBodies = true; // recorded fused v11 owns the merge (RecordedV11sOwnsPlayback)

            // recorded take is the sculpture: hide live visuals + drop them as source
            if (sensorManager != null)
            {
                sensorManager.SetLiveVisualsVisible(false);
                sensorManager.SetLiveSuppressedAsSource(true);
            }
            SetLiveRenderDelay(false); // review is frame-locked already; no live delay
            // per-camera 2D must track the RECORDED frames, and must NOT submit to the
            // merger (recorded v11 bodies_v11s own the merge)
            if (fused != null)
            {
                fused.liveFramesOnly = false;
                fused.playbackFramesOnly = true;
                fused.syncReviewFusion = true; // deterministic per-step per-camera 2D
                fused.SetSubmitToMerger(false);
            }
            if (merger != null) merger.showBones = true;
            if (curves != null) curves.visible = true;

            // load + start + pause at frame 0
            if (recorder.CurrentState != SensorRecorder.State.Idle && recorder.IsPlaying) recorder.TogglePlay();
            recorder.Load();
            if (!recorder.IsPlaying) recorder.TogglePlay();
            recorder.PausePlayback();
            recorder.SeekToFrame(0);

            RecreateOverlay(review: true);
            CurrentMode = Mode.Review;
            Debug.Log($"[{nameof(BoneVerifyController)}] REVIEW — {Path.GetFileName(take)}: ←/→ to step. " +
                      $"grid(display{gridDisplay + 1})=cyan recorded-fused + orange per-camera 2D; " +
                      $"3D(display{sceneDisplay + 1})=cloud+merged bones+curves.", this);
        }

        /// <summary>Step the review playback by <paramref name="delta"/> frames
        /// (negative = backward). No-op outside review.</summary>
        public void Step(int delta)
        {
            if (CurrentMode != Mode.Review || recorder == null || delta == 0) return;
            int n = Mathf.Abs(delta);
            for (int i = 0; i < n; i++)
            {
                if (delta > 0) recorder.StepForward();
                else recorder.StepBackward();
            }
        }

        public void Exit()
        {
            if (recorder != null && recorder.IsPlaying) recorder.TogglePlay(); // stop playback

            SetLiveRenderDelay(false);
            if (_overlayGo != null) { Destroy(_overlayGo); _overlayGo = null; _overlay = null; }

            RestoreState();
            CurrentMode = Mode.Off;
            Debug.Log($"[{nameof(BoneVerifyController)}] exited bone-verify — scene restored.", this);
        }

        // ---------------- state save / restore ----------------

        private void SaveStateIfNeeded()
        {
            if (_saved || CurrentMode != Mode.Off) return;
            _sShowBones = merger != null && merger.showBones;
            _sExternal = merger != null && merger.useExternalBodies;
            _sCurvesVisible = curves != null && curves.visible;
            _sUseV11s = recorder != null && recorder.useV11sBodies;
            _sFusedLiveOnly = fused != null && fused.liveFramesOnly;
            _sFusedPlaybackOnly = fused != null && fused.playbackFramesOnly;
            _sFusedSubmit = fused != null && fused.submitToMerger;
            _sFusedSyncReview = fused != null && fused.syncReviewFusion;
            _saved = true;
        }

        private void RestoreState()
        {
            if (!_saved) return;
            if (sensorManager != null)
            {
                sensorManager.SetLiveVisualsVisible(true);
                sensorManager.SetLiveSuppressedAsSource(false);
            }
            if (fused != null)
            {
                fused.liveFramesOnly = _sFusedLiveOnly;
                fused.playbackFramesOnly = _sFusedPlaybackOnly;
                fused.syncReviewFusion = _sFusedSyncReview;
                fused.SetSubmitToMerger(_sFusedSubmit);
            }
            if (merger != null) { merger.showBones = _sShowBones; merger.useExternalBodies = _sExternal; }
            if (curves != null) curves.visible = _sCurvesVisible;
            if (recorder != null) recorder.useV11sBodies = _sUseV11s;
            _saved = false;
        }

        // ---------------- helpers ----------------

        private void RecreateOverlay(bool review)
        {
            if (_overlayGo != null) { Destroy(_overlayGo); _overlayGo = null; _overlay = null; }
            // Build the GO INACTIVE so the overlay's OnEnable runs only AFTER its
            // fields are set (AddComponent would otherwise fire OnEnable immediately
            // with the default targetDisplay).
            _overlayGo = new GameObject("BoneVerify.ColorGrid");
            _overlayGo.transform.SetParent(transform, false);
            _overlayGo.SetActive(false);
            _overlay = _overlayGo.AddComponent<CameraColorGridOverlay>();
            _overlay.targetDisplay = gridDisplay;
            _overlay.fused = fused;
            _overlay.merger = merger;
            _overlay.consumePlayback = review;
            _overlay.showMergedReproj = review;
            _overlayGo.SetActive(true);
        }

        private bool HasLiveRenderers()
        {
            foreach (var r in FindObjectsByType<PointCloudRenderer>(FindObjectsSortMode.None))
                if (r != null && !string.IsNullOrEmpty(r.deviceSerial)) return true;
            return false;
        }

        private void TryStartLive()
        {
            try { sensorManager.GetType().GetMethod("StartLive")?.Invoke(sensorManager, null); }
            catch (Exception e) { Debug.LogException(e, this); }
        }

        private static string NewestTakeUnder(string root)
        {
            try
            {
                if (string.IsNullOrEmpty(root) || !Directory.Exists(root)) return null;
                string best = null; DateTime bestT = DateTime.MinValue;
                foreach (var d in Directory.GetDirectories(root))
                {
                    var t = Directory.GetLastWriteTimeUtc(d);
                    if (t > bestT) { bestT = t; best = d; }
                }
                return best;
            }
            catch { return null; }
        }

        // ---------------- operator UI (display 1) ----------------

        private void BuildOperatorUI()
        {
            if (_opCanvas != null) return;
            EnsureEventSystem();

            var go = new GameObject("BoneVerify.OperatorUI");
            go.transform.SetParent(transform, false);
            _opCanvas = go.AddComponent<Canvas>();
            _opCanvas.renderMode = RenderMode.ScreenSpaceOverlay;
            _opCanvas.targetDisplay = 0; // operator display
            var scaler = go.AddComponent<CanvasScaler>();
            scaler.uiScaleMode = CanvasScaler.ScaleMode.ScaleWithScreenSize;
            scaler.referenceResolution = new Vector2(1920, 1080);
            go.AddComponent<GraphicRaycaster>();

            var font = Resources.GetBuiltinResource<Font>("LegacyRuntime.ttf");

            var panel = new GameObject("panel").AddComponent<RectTransform>();
            panel.SetParent(_opCanvas.transform, false);
            panel.anchorMin = new Vector2(0f, 1f); panel.anchorMax = new Vector2(0f, 1f);
            panel.pivot = new Vector2(0f, 1f);
            panel.anchoredPosition = new Vector2(16f, -16f);
            panel.sizeDelta = new Vector2(560f, 132f);
            var bg = panel.gameObject.AddComponent<Image>();
            bg.color = new Color(0f, 0f, 0f, 0.55f);

            _opStatus = MakeText(panel, font, "Bone Verify: OFF", new Vector2(12f, -8f), 26, TextAnchor.UpperLeft);
            _opStatus.rectTransform.sizeDelta = new Vector2(536f, 40f);

            MakeButton(panel, font, "Live (F8)", new Vector2(12f, -56f), () => { if (CurrentMode == Mode.Live) Exit(); else EnterLive(); });
            MakeButton(panel, font, "Review (F7)", new Vector2(150f, -56f), () => { if (CurrentMode == Mode.Review) Exit(); else EnterPlaybackReview(null); });
            MakeButton(panel, font, "◀", new Vector2(300f, -56f), () => Step(-1), 48f);
            MakeButton(panel, font, "▶", new Vector2(352f, -56f), () => Step(+1), 48f);
            MakeButton(panel, font, "Exit", new Vector2(430f, -56f), Exit);
        }

        private void UpdateOperatorStatus()
        {
            if (_opStatus == null) return;
            if (CurrentMode == Mode.Review && recorder != null)
                _opStatus.text = $"Bone Verify: REVIEW  frame {recorder.CurrentPlaybackFrame + 1}/{recorder.RecordedFrameCount}" +
                                 (recorder.IsPaused ? "  (paused)" : "");
            else
                _opStatus.text = $"Bone Verify: {CurrentMode.ToString().ToUpperInvariant()}";
        }

        private static void EnsureEventSystem()
        {
            if (FindFirstObjectByType<EventSystem>() != null) return;
            var es = new GameObject("EventSystem");
            es.AddComponent<EventSystem>();
            es.AddComponent<StandaloneInputModule>();
        }

        private static Text MakeText(RectTransform parent, Font font, string s, Vector2 pos, int size, TextAnchor anchor)
        {
            var go = new GameObject("text");
            var rt = go.AddComponent<RectTransform>();
            rt.SetParent(parent, false);
            rt.anchorMin = new Vector2(0f, 1f); rt.anchorMax = new Vector2(0f, 1f);
            rt.pivot = new Vector2(0f, 1f);
            rt.anchoredPosition = pos;
            rt.sizeDelta = new Vector2(200f, 36f);
            var t = go.AddComponent<Text>();
            t.font = font; t.fontSize = size; t.color = Color.white; t.text = s; t.alignment = anchor;
            t.horizontalOverflow = HorizontalWrapMode.Overflow; t.verticalOverflow = VerticalWrapMode.Overflow;
            return t;
        }

        private static void MakeButton(RectTransform parent, Font font, string label, Vector2 pos,
                                       UnityEngine.Events.UnityAction onClick, float width = 132f)
        {
            var go = new GameObject("btn_" + label);
            var rt = go.AddComponent<RectTransform>();
            rt.SetParent(parent, false);
            rt.anchorMin = new Vector2(0f, 1f); rt.anchorMax = new Vector2(0f, 1f);
            rt.pivot = new Vector2(0f, 1f);
            rt.anchoredPosition = pos;
            rt.sizeDelta = new Vector2(width, 56f);
            var img = go.AddComponent<Image>();
            img.color = new Color(0.20f, 0.22f, 0.26f, 0.95f);
            var btn = go.AddComponent<Button>();
            btn.targetGraphic = img;
            btn.onClick.AddListener(onClick);
            var t = MakeText(rt, font, label, Vector2.zero, 24, TextAnchor.MiddleCenter);
            t.rectTransform.anchorMin = Vector2.zero; t.rectTransform.anchorMax = Vector2.one;
            t.rectTransform.offsetMin = Vector2.zero; t.rectTransform.offsetMax = Vector2.zero;
        }
    }
}
