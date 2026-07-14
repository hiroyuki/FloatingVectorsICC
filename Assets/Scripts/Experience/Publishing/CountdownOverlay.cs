// Big centre-screen countdown, one canvas per display (same ScreenSpaceCamera
// approach as QrOverlay — SensorRecorder's IMGUI OnGUI never showed up on the
// multi-display game-view setup). Driven by OperatorPublisher from
// SensorRecorder.FreezeCountdownRemaining so the dancer sees the freeze coming
// on every screen.

using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

namespace Experience.Publishing
{
    [DisallowMultipleComponent]
    public sealed class CountdownOverlay : MonoBehaviour
    {
        [Tooltip("Displays to show the countdown on, 0-based (default: all three).")]
        public int[] targetDisplays = { 0, 1, 2 };

        private readonly List<GameObject> _roots = new List<GameObject>();
        private readonly List<Text> _texts = new List<Text>();

        public void Show(string text)
        {
            EnsureUi();
            for (int i = 0; i < _roots.Count; i++)
            {
                _texts[i].text = text;
                _roots[i].SetActive(true);
            }
        }

        public void Hide()
        {
            foreach (var r in _roots)
                if (r != null) r.SetActive(false);
        }

        private void EnsureUi()
        {
            if (_roots.Count > 0) return;
            var displays = (targetDisplays != null && targetDisplays.Length > 0)
                ? targetDisplays : new[] { 0, 1, 2 };
            foreach (int display in displays)
                BuildDisplayUi(display);
        }

        private void BuildDisplayUi(int display)
        {
            var root = new GameObject($"_Countdown_Display{display}");
            root.transform.SetParent(transform, false);

            var canvas = root.AddComponent<Canvas>();
            Camera cam = null;
            foreach (var c in Camera.allCameras) // enabled cameras only
                if (c.targetDisplay == display) { cam = c; break; }
            if (cam != null)
            {
                canvas.renderMode = RenderMode.ScreenSpaceCamera;
                canvas.worldCamera = cam;
                canvas.planeDistance = 1f;
            }
            else
            {
                canvas.renderMode = RenderMode.ScreenSpaceOverlay;
            }
            canvas.targetDisplay = display;
            canvas.sortingOrder = 6100; // above the QR overlay (6000)

            var scaler = root.AddComponent<CanvasScaler>();
            scaler.uiScaleMode = CanvasScaler.ScaleMode.ScaleWithScreenSize;
            scaler.referenceResolution = new Vector2(1920f, 1080f);
            scaler.matchWidthOrHeight = 0.5f;

            var go = new GameObject("Number", typeof(RectTransform));
            go.transform.SetParent(root.transform, false);
            var rect = go.GetComponent<RectTransform>();
            rect.sizeDelta = new Vector2(1000f, 700f); // centered anchors by default
            var text = go.AddComponent<Text>();
            text.font = Resources.GetBuiltinResource<Font>("LegacyRuntime.ttf");
            text.fontSize = 500;
            text.resizeTextForBestFit = true; // the atlas caps huge glyphs — let it fit
            text.resizeTextMaxSize = 500;
            text.resizeTextMinSize = 100;
            text.fontStyle = FontStyle.Bold;
            text.color = Color.white;
            text.alignment = TextAnchor.MiddleCenter;
            text.horizontalOverflow = HorizontalWrapMode.Overflow;
            text.verticalOverflow = VerticalWrapMode.Overflow;
            text.raycastTarget = false;
            var shadow = go.AddComponent<Shadow>();
            shadow.effectColor = new Color(0f, 0f, 0f, 0.9f);
            shadow.effectDistance = new Vector2(6f, -6f);

            root.SetActive(false);
            _roots.Add(root);
            _texts.Add(text);
        }
    }
}
