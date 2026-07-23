// "起動中" / "終了中" full-screen splash for the seconds when the app is busy and
// the picture is not.
//
// Why this exists: opening the four cameras takes ~6.4s and every bit of it runs
// synchronously inside one frame's Start phase, so the player presented its FIRST
// frame 8.9s after launch (measured on the 4070 rig with StartupProfiler). Until
// then the window is blank and the operator cannot tell a slow start from a hung
// one. Quitting is the same story from the other end: ~4.5s of ob_pipeline_stop
// with a frozen last frame on screen. Unity cannot draw during either block, so
// no amount of UI helps by itself — the blocking work has to be pushed past a
// PRESENTED frame. SensorManager.StartLiveStaged does that for boot and
// AppQuitHotkey's wantsToQuit hook does it for shutdown; this is what those
// frames show.
//
// Drawn as a uGUI Canvas per display, NOT as IMGUI: IMGUI only reaches the
// primary display in a standalone build, and it renders underneath every
// ScreenSpaceOverlay canvas (same reasoning as DisplayDiagnosticsOverlay).
//
// Self-installing at BeforeSceneLoad so the canvas exists before any scene
// component's Start can block, with nothing to wire in the scene.

using UnityEngine;
using UnityEngine.UI;

namespace Shared
{
    [DisallowMultipleComponent]
    public class BootOverlay : MonoBehaviour
    {
        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.BeforeSceneLoad)]
        private static void Install()
        {
            var go = new GameObject("[BootOverlay]");
            Instance = go.AddComponent<BootOverlay>();
            DontDestroyOnLoad(go);
        }

        public static BootOverlay Instance { get; private set; }

        [Tooltip("Headline shown while the rig starts.")]
        public string title = "起動中";

        [Tooltip("Displays to cover, including the primary one. Matches the " +
                 "MultiDisplayActivator count (operator + two visitor displays).")]
        public int displayCount = 3;

        [Tooltip("OS font names tried in order (the text is Japanese, and the " +
                 "built-in LegacyRuntime.ttf has no CJK glyphs — it would render as " +
                 "boxes). First installed one wins; none → LegacyRuntime.ttf + the " +
                 "romaji fallback title.")]
        public string[] fontCandidates =
            { "Yu Gothic Medium", "游ゴシック Medium", "Yu Gothic", "Meiryo", "MS Gothic" };

        [Tooltip("Title used when no Japanese-capable OS font was found.")]
        public string fallbackTitle = "STARTING UP";

        [Tooltip("Headline shown while the app shuts the rig down and exits.")]
        public string quitTitle = "終了中";

        [Tooltip("Shutdown title used when no Japanese-capable OS font was found.")]
        public string quitFallbackTitle = "SHUTTING DOWN";

        [Tooltip("Frames to wait for someone to Claim() the overlay before hiding it. " +
                 "A scene with no live rig (playback, calibration, a test scene) never " +
                 "claims it, and must not be left under a splash.")]
        public int unclaimedFrames = 3;

        [Tooltip("Hide unconditionally after this long, however the boot went. A splash " +
                 "that outlives a failed startup would hide the very screen the operator " +
                 "needs in order to see what went wrong.")]
        public float safetyTimeoutSeconds = 120f;

        private Canvas[] _canvases;
        private Text[] _titleTexts;
        private Text[] _statusTexts;
        private bool _claimed;
        // Set once ShowShutdown() takes over; makes Hide() a no-op from then on.
        private bool _shutdownOwned;
        private bool _hidden;
        private bool _japaneseCapable;
        private int _frames;
        private float _shownAt;
        private string _pendingStatus;

        /// <summary>Announce that a boot sequence owns this overlay and will Hide() it.
        /// Without a claim the splash takes itself down after a few frames.</summary>
        public static void Claim()
        {
            if (Instance != null) Instance._claimed = true;
        }

        /// <summary>Replace the line under the headline (e.g. "カメラを起動しています… 2/4").
        /// Safe to call before the overlay has built its canvases.</summary>
        public static void SetStatus(string status)
        {
            if (Instance == null) return;
            Instance._pendingStatus = status;
            Instance.ApplyStatus();
        }

        /// <summary>Take the splash down. Idempotent, and ignored once shutdown owns the
        /// overlay: a boot sequence still unwinding when the operator quits would otherwise
        /// pull the 終了中 splash off the screen on its way out, leaving the displays blank
        /// for the seconds the camera stop takes. Shutdown is terminal — nothing needs to
        /// hide it, the process exit does.</summary>
        public static void Hide()
        {
            if (Instance != null && !Instance._shutdownOwned) Instance.HideNow();
        }

        /// <summary>Bring the splash back up for shutdown. Claims it too, so the
        /// unclaimed-frames auto-hide can never pull it out from under a teardown
        /// that takes seconds.</summary>
        public static void ShowShutdown(string status = "")
        {
            if (Instance == null) return;
            Instance.ShowShutdownNow(status);
        }

        private void ShowShutdownNow(string status)
        {
            _shutdownOwned = true;
            _claimed = true;
            _hidden = false;
            _shownAt = Time.realtimeSinceStartup;
            OperatorOverlayGate.BootActive = true;
            string headline = _japaneseCapable ? quitTitle : quitFallbackTitle;
            if (_titleTexts != null)
                foreach (var t in _titleTexts)
                    if (t != null) t.text = headline;
            _pendingStatus = status;
            ApplyStatus();
            if (_canvases == null) return;
            foreach (var c in _canvases)
                if (c != null) c.gameObject.SetActive(true);
        }

        private void Awake()
        {
            if (Instance != null && Instance != this) { Destroy(gameObject); return; }
            Instance = this;
            _shownAt = Time.realtimeSinceStartup;
            OperatorOverlayGate.BootActive = true;
            Build();
        }

        private void Update()
        {
            if (_hidden) return;
            _frames++;
            // Unclaimed after a few frames: no live boot in this scene, so nothing is
            // ever going to hide it.
            if (!_claimed && _frames > Mathf.Max(1, unclaimedFrames)) { HideNow(); return; }
            if (Time.realtimeSinceStartup - _shownAt > safetyTimeoutSeconds)
            {
                Debug.LogWarning($"[{nameof(BootOverlay)}] still up after " +
                                 $"{safetyTimeoutSeconds:0}s — hiding so the screen is usable. " +
                                 "Startup either failed or is much slower than expected.");
                HideNow();
            }
        }

        private void HideNow()
        {
            if (_hidden) return;
            _hidden = true;
            OperatorOverlayGate.BootActive = false;
            if (_canvases == null) return;
            foreach (var c in _canvases)
                if (c != null) c.gameObject.SetActive(false);
        }

        // null = leave the line alone; "" = clear it (shutdown has no per-step count).
        private void ApplyStatus()
        {
            if (_statusTexts == null || _pendingStatus == null) return;
            foreach (var t in _statusTexts)
                if (t != null) t.text = _pendingStatus;
        }

        private void Build()
        {
            Font font = ResolveFont(out _japaneseCapable);
            string headline = _japaneseCapable ? title : fallbackTitle;

            int count = Mathf.Max(1, displayCount);
            _canvases = new Canvas[count];
            _titleTexts = new Text[count];
            _statusTexts = new Text[count];
            for (int i = 0; i < count; i++)
                _canvases[i] = BuildCanvas(i, font, headline,
                                           out _titleTexts[i], out _statusTexts[i]);
            ApplyStatus();
        }

        private Font ResolveFont(out bool japaneseCapable)
        {
            var installed = new System.Collections.Generic.HashSet<string>(Font.GetOSInstalledFontNames());
            foreach (var name in fontCandidates)
            {
                if (string.IsNullOrEmpty(name) || !installed.Contains(name)) continue;
                var f = Font.CreateDynamicFontFromOSFont(name, 64);
                if (f == null) continue;
                japaneseCapable = true;
                return f;
            }
            Debug.LogWarning($"[{nameof(BootOverlay)}] none of the Japanese font candidates are " +
                             "installed; falling back to LegacyRuntime.ttf and the romaji title.");
            japaneseCapable = false;
            return Resources.GetBuiltinResource<Font>("LegacyRuntime.ttf");
        }

        // One canvas per display; targetDisplay is 0-based, so index 0 is "Display 1".
        private Canvas BuildCanvas(int display, Font font, string headline,
                                   out Text titleText, out Text status)
        {
            var go = new GameObject("BootOverlay.Canvas." + (display + 1));
            go.transform.SetParent(transform, false);
            var canvas = go.AddComponent<Canvas>();
            canvas.renderMode = RenderMode.ScreenSpaceOverlay;
            canvas.targetDisplay = display;
            // Above the fps readout and the operator UI, below nothing: while this is
            // up it IS the screen.
            canvas.sortingOrder = short.MaxValue;
            var scaler = go.AddComponent<CanvasScaler>();
            scaler.uiScaleMode = CanvasScaler.ScaleMode.ScaleWithScreenSize;
            scaler.referenceResolution = new Vector2(1920, 1080);
            scaler.matchWidthOrHeight = 0.5f;

            // Opaque: the scene behind is half-built during boot (cameras with no
            // frames yet, an empty sculpture) and showing that reads as a fault.
            var bgGo = new GameObject("bg").AddComponent<RectTransform>();
            bgGo.SetParent(canvas.transform, false);
            bgGo.anchorMin = Vector2.zero; bgGo.anchorMax = Vector2.one;
            bgGo.offsetMin = Vector2.zero; bgGo.offsetMax = Vector2.zero;
            var bg = bgGo.gameObject.AddComponent<Image>();
            bg.color = Color.black;
            bg.raycastTarget = false;

            titleText = AddText(bgGo, font, headline, 96, FontStyle.Bold,
                                new Vector2(0f, 40f), 160f, new Color(1f, 1f, 1f, 0.95f), "title");
            status = AddText(bgGo, font, "", 40, FontStyle.Normal,
                             new Vector2(0f, -70f), 70f, new Color(1f, 1f, 1f, 0.6f), "status");
            return canvas;
        }

        private static Text AddText(RectTransform parent, Font font, string content, int size,
                                    FontStyle style, Vector2 offset, float height, Color color,
                                    string name)
        {
            var go = new GameObject(name);
            go.transform.SetParent(parent, false);
            var rt = go.AddComponent<RectTransform>();
            rt.anchorMin = new Vector2(0f, 0.5f);
            rt.anchorMax = new Vector2(1f, 0.5f);
            rt.pivot = new Vector2(0.5f, 0.5f);
            rt.anchoredPosition = offset;
            rt.sizeDelta = new Vector2(0f, height);
            var text = go.AddComponent<Text>();
            text.font = font;
            text.fontSize = size;
            text.fontStyle = style;
            text.color = color;
            text.alignment = TextAnchor.MiddleCenter;
            text.horizontalOverflow = HorizontalWrapMode.Overflow;
            text.verticalOverflow = VerticalWrapMode.Overflow;
            text.raycastTarget = false;
            text.text = content;
            return text;
        }
    }
}
