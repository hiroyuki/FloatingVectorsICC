// REC indicator: a small blinking red dot in the top-right corner of the
// secondary displays (default Display 2 + 3, the visitor-facing side screens)
// while the recorder transport reports IsRecording. The operator's Display 1
// is deliberately left untouched — the Control Panel already shows transport
// state there; the side screens had no cue at all, so nobody standing in the
// space could tell a take was rolling.
//
// Self-installing (same pattern as DisplayDiagnosticsOverlay) so it works in
// any scene/build without wiring. One ScreenSpaceOverlay canvas per display —
// see the render-mode comment in BuildDisplayUi for why camera-space canvases
// are invisible on these displays.

using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

namespace Shared
{
    [DisallowMultipleComponent]
    public class RecordingIndicatorOverlay : MonoBehaviour
    {
        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
        private static void Install()
        {
            if (FindFirstObjectByType<RecordingIndicatorOverlay>() != null) return;
            var go = new GameObject("[RecordingIndicator]");
            go.AddComponent<RecordingIndicatorOverlay>();
            DontDestroyOnLoad(go);
        }

        [Tooltip("Displays to draw the dot on, 0-based (1 = Display 2, 2 = Display 3). " +
                 "Display 1 (index 0) is left to the operator UI.")]
        public int[] targetDisplays = { 1, 2 };

        [Min(8)]
        [Tooltip("Dot diameter in reference pixels (1920x1080 canvas scaling).")]
        public int dotDiameterPixels = 44;

        [Min(0)]
        [Tooltip("Distance from the top-right screen corner in reference pixels.")]
        public int cornerMargin = 28;

        [Min(0f)]
        [Tooltip("Blink rate. 0 = steady dot.")]
        public float blinkHz = 1f;

        private IRecorderTransport _transport;
        private float _nextTransportScan;
        private readonly List<GameObject> _roots = new List<GameObject>();
        private Sprite _dotSprite;

        private void Update()
        {
            // The transport is a plain interface over a MonoBehaviour, so a
            // destroyed recorder must be re-detected via the Unity null check.
            if (_transport is MonoBehaviour mb && mb == null) _transport = null;
            if (_transport == null)
            {
                if (Time.unscaledTime < _nextTransportScan) return;
                _nextTransportScan = Time.unscaledTime + 2f;
                foreach (var m in FindObjectsByType<MonoBehaviour>(FindObjectsSortMode.None))
                    if (m is IRecorderTransport t) { _transport = t; break; }
                if (_transport == null) return;
            }

            bool rec = _transport.IsRecording;
            bool on = rec && (blinkHz <= 0f
                || Mathf.Repeat(Time.unscaledTime * blinkHz, 1f) < 0.65f);
            if (on && _roots.Count == 0) BuildUi();
            for (int i = 0; i < _roots.Count; i++)
                if (_roots[i] != null && _roots[i].activeSelf != on) _roots[i].SetActive(on);
        }

        private void BuildUi()
        {
            var displays = (targetDisplays != null && targetDisplays.Length > 0)
                ? targetDisplays : new[] { 1, 2 };
            foreach (int display in displays)
                BuildDisplayUi(display);
        }

        private void BuildDisplayUi(int display)
        {
            var root = new GameObject($"_RecIndicator_Display{display}");
            root.transform.SetParent(transform, false);

            var canvas = root.AddComponent<Canvas>();
            // ScreenSpaceOverlay, NOT ScreenSpaceCamera: overlay canvases always
            // draw on top of camera-space ones, and the side displays are covered
            // by CalibrationRuntimeUI's fullscreen overlay color monitors
            // (_CalibColorDisplay, sortingOrder 100, black backdrop) — a
            // camera-space dot would sit invisibly behind them.
            canvas.renderMode = RenderMode.ScreenSpaceOverlay;
            canvas.targetDisplay = display;
            canvas.sortingOrder = 6500; // above _CalibColorDisplay (100) / QrOverlay (6000)

            var scaler = root.AddComponent<CanvasScaler>();
            scaler.uiScaleMode = CanvasScaler.ScaleMode.ScaleWithScreenSize;
            scaler.referenceResolution = new Vector2(1920f, 1080f);
            scaler.matchWidthOrHeight = 0.5f;

            var dotGo = new GameObject("RecDot", typeof(RectTransform));
            dotGo.transform.SetParent(root.transform, false);
            var rect = dotGo.GetComponent<RectTransform>();
            rect.anchorMin = rect.anchorMax = rect.pivot = Vector2.one; // top-right
            rect.sizeDelta = new Vector2(dotDiameterPixels, dotDiameterPixels);
            rect.anchoredPosition = new Vector2(-cornerMargin, -cornerMargin);
            var image = dotGo.AddComponent<Image>();
            image.sprite = GetDotSprite();
            image.color = new Color(0.92f, 0.11f, 0.11f, 1f);
            image.raycastTarget = false;

            root.SetActive(false);
            _roots.Add(root);
        }

        // Procedural anti-aliased disc — no dependency on builtin UI sprites,
        // which can be stripped from player builds.
        private Sprite GetDotSprite()
        {
            if (_dotSprite != null) return _dotSprite;
            const int n = 64;
            var tex = new Texture2D(n, n, TextureFormat.RGBA32, false)
            {
                name = "_RecDot",
                wrapMode = TextureWrapMode.Clamp,
            };
            float c = (n - 1) * 0.5f;
            float rOut = n * 0.5f - 1.5f;
            var px = new Color32[n * n];
            for (int y = 0; y < n; y++)
            {
                for (int x = 0; x < n; x++)
                {
                    float d = Mathf.Sqrt((x - c) * (x - c) + (y - c) * (y - c));
                    float a = Mathf.Clamp01(rOut - d + 0.5f);
                    px[y * n + x] = new Color32(255, 255, 255, (byte)(a * 255f + 0.5f));
                }
            }
            tex.SetPixels32(px);
            tex.Apply(false, true);
            _dotSprite = Sprite.Create(tex, new Rect(0, 0, n, n), new Vector2(0.5f, 0.5f));
            return _dotSprite;
        }

        private void OnDestroy()
        {
            if (_dotSprite != null)
            {
                Destroy(_dotSprite.texture);
                Destroy(_dotSprite);
            }
        }
    }
}
