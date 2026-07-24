// Stage-side debug HUD: mirrors a short diagnostic text onto the secondary
// displays (default Display 2 + 3) so it can be read FROM THE STAGE. The
// operator PC (Display 1) sits meters away from the sensing area — an OnGUI
// HUD there is useless while you are standing in the space being tracked.
//
// Static Show/Hide API so any system can push text without scene wiring; the
// backing GameObject is created on first Show and kept across scene loads.
// Text is uGUI (ScreenSpaceOverlay, high sorting order) so it draws above the
// fullscreen overlay canvases covering those displays (_CalibColorDisplay 100,
// QrOverlay 6000).

using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

namespace Shared
{
    [DisallowMultipleComponent]
    public sealed class StageHudOverlay : MonoBehaviour
    {
        private static StageHudOverlay s_instance;

        /// <summary>Show <paramref name="text"/> (uGUI rich text) on the stage
        /// displays. Creates the overlay on first call.</summary>
        public static void Show(string text)
        {
            if (s_instance == null)
            {
                var go = new GameObject("[StageHud]");
                DontDestroyOnLoad(go);
                s_instance = go.AddComponent<StageHudOverlay>();
            }
            s_instance.SetText(text);
        }

        /// <summary>Hide the overlay. No-op when it was never shown.</summary>
        public static void Hide()
        {
            if (s_instance != null) s_instance.SetText(null);
        }

        [Tooltip("Displays to draw on, 0-based (1 = Display 2, 2 = Display 3).")]
        public int[] targetDisplays = { 1, 2 };

        [Min(8)]
        [Tooltip("Font size in reference pixels (1920x1080 scaling). Large — this is " +
                 "read from meters away.")]
        public int fontSize = 34;

        private readonly List<GameObject> _roots = new List<GameObject>();
        private readonly List<Text> _texts = new List<Text>();
        private readonly List<Image> _backdrops = new List<Image>();

        private void SetText(string text)
        {
            bool show = !string.IsNullOrEmpty(text);
            if (show && _roots.Count == 0) BuildUi();
            for (int i = 0; i < _roots.Count; i++)
            {
                if (_roots[i] == null) continue;
                if (_roots[i].activeSelf != show) _roots[i].SetActive(show);
                if (show)
                {
                    _texts[i].text = text;
                    // Fit the backdrop to the text height (plus padding).
                    float h = _texts[i].preferredHeight + 24f;
                    var r = (RectTransform)_backdrops[i].transform;
                    r.sizeDelta = new Vector2(r.sizeDelta.x, h);
                }
            }
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
            var root = new GameObject($"_StageHud_Display{display}");
            root.transform.SetParent(transform, false);

            var canvas = root.AddComponent<Canvas>();
            // Overlay, not camera-space: see RecordingIndicatorOverlay — these
            // displays are covered by fullscreen overlay canvases.
            canvas.renderMode = RenderMode.ScreenSpaceOverlay;
            canvas.targetDisplay = display;
            canvas.sortingOrder = 6400; // above _CalibColorDisplay (100), below the REC dot (6500)

            var scaler = root.AddComponent<CanvasScaler>();
            scaler.uiScaleMode = CanvasScaler.ScaleMode.ScaleWithScreenSize;
            scaler.referenceResolution = new Vector2(1920f, 1080f);
            scaler.matchWidthOrHeight = 0.5f;

            var backGo = new GameObject("Backdrop", typeof(RectTransform));
            backGo.transform.SetParent(root.transform, false);
            var backRect = (RectTransform)backGo.transform;
            backRect.anchorMin = new Vector2(0f, 1f);
            backRect.anchorMax = new Vector2(1f, 1f); // stretch across the top
            backRect.pivot = new Vector2(0.5f, 1f);
            backRect.sizeDelta = new Vector2(0f, 200f); // height refit per text
            backRect.anchoredPosition = Vector2.zero;
            var back = backGo.AddComponent<Image>();
            back.color = new Color(0f, 0f, 0f, 0.7f);
            back.raycastTarget = false;

            var textGo = new GameObject("Text", typeof(RectTransform));
            textGo.transform.SetParent(backGo.transform, false);
            var textRect = (RectTransform)textGo.transform;
            textRect.anchorMin = Vector2.zero;
            textRect.anchorMax = Vector2.one;
            textRect.offsetMin = new Vector2(24f, 12f);
            textRect.offsetMax = new Vector2(-24f, -12f);
            var text = textGo.AddComponent<Text>();
            text.font = Resources.GetBuiltinResource<Font>("LegacyRuntime.ttf");
            text.fontSize = fontSize;
            text.color = Color.white;
            text.alignment = TextAnchor.UpperLeft;
            text.horizontalOverflow = HorizontalWrapMode.Overflow;
            text.verticalOverflow = VerticalWrapMode.Overflow;
            text.supportRichText = true;
            text.raycastTarget = false;

            root.SetActive(false);
            _roots.Add(root);
            _texts.Add(text);
            _backdrops.Add(back);
        }
    }
}
