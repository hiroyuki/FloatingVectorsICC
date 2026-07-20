// Operator QR overlay: a small top-right QR + URL caption for the F10 publish
// flow (OperatorPublisher). Unlike VisitorMessageUI this never occupies the
// operator's main view (Display 1) — it renders on the secondary displays
// (targetDisplays, default Display 2 + 3) so a phone can scan either side
// screen while Display 1 keeps showing the sculpture untouched.

using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

namespace Experience.Publishing
{
    [DisallowMultipleComponent]
    public sealed class QrOverlay : MonoBehaviour
    {
        [Tooltip("Displays to show the QR on, 0-based (1 = Display 2, 2 = Display 3). " +
                 "Display 1 (index 0) is deliberately left to the main sculpture view.")]
        public int[] targetDisplays = { 1, 2 };

        [Min(64)]
        [Tooltip("QR edge length in reference pixels (1920x1080 canvas scaling).")]
        public int qrSizePixels = 280;

        [Min(0)]
        [Tooltip("Distance from the top-right screen corner in reference pixels.")]
        public int cornerMargin = 24;

        private readonly List<GameObject> _roots = new List<GameObject>();
        private readonly List<RawImage> _images = new List<RawImage>();
        private readonly List<Text> _captions = new List<Text>();
        private Texture2D _tex; // owned: destroyed on replace / component destroy

        public bool IsShowing => _roots.Count > 0 && _roots[0].activeSelf;

        /// <summary>Show a QR (ownership of the texture transfers here) with a
        /// caption under it, on every target display. Replaces any previous QR.</summary>
        public void Show(Texture2D qr, string caption)
        {
            if (qr == null) return;
            EnsureUi();
            if (_tex != null && _tex != qr) Destroy(_tex);
            _tex = qr;
            for (int i = 0; i < _roots.Count; i++)
            {
                _images[i].texture = qr;
                _captions[i].text = caption ?? "";
                _roots[i].SetActive(true);
            }
        }

        public void Hide()
        {
            foreach (var r in _roots)
                if (r != null) r.SetActive(false);
        }

        private void OnDestroy()
        {
            if (_tex != null) Destroy(_tex);
        }

        private void EnsureUi()
        {
            if (_roots.Count > 0) return;
            var displays = (targetDisplays != null && targetDisplays.Length > 0)
                ? targetDisplays : new[] { 1, 2 };
            foreach (int display in displays)
                BuildDisplayUi(display);
        }

        private void BuildDisplayUi(int display)
        {
            var root = new GameObject($"_QrOverlay_Display{display}");
            root.transform.SetParent(transform, false);

            var canvas = root.AddComponent<Canvas>();
            // ScreenSpaceCamera through the display's own camera (same approach as
            // VisitorMessageUI): a ScreenSpaceOverlay canvas on a secondary display
            // sizes itself from the wrong screen when the game views differ, which
            // pushed the "top-right" QR toward the centre. Overlay is the fallback
            // when no camera renders to that display.
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
            canvas.sortingOrder = 6000; // above VisitorMessageUI (5000)

            var scaler = root.AddComponent<CanvasScaler>();
            scaler.uiScaleMode = CanvasScaler.ScaleMode.ScaleWithScreenSize;
            scaler.referenceResolution = new Vector2(1920f, 1080f);
            scaler.matchWidthOrHeight = 0.5f;

            var qrGo = new GameObject("QrImage", typeof(RectTransform));
            qrGo.transform.SetParent(root.transform, false);
            var rect = qrGo.GetComponent<RectTransform>();
            rect.anchorMin = rect.anchorMax = rect.pivot = Vector2.one; // top-right
            rect.sizeDelta = new Vector2(qrSizePixels, qrSizePixels);
            rect.anchoredPosition = new Vector2(-cornerMargin, -cornerMargin);
            var image = qrGo.AddComponent<RawImage>();
            image.color = Color.white;
            image.raycastTarget = false;

            var capGo = new GameObject("Caption", typeof(RectTransform));
            capGo.transform.SetParent(root.transform, false);
            var capRect = capGo.GetComponent<RectTransform>();
            capRect.anchorMin = capRect.anchorMax = capRect.pivot = Vector2.one;
            capRect.sizeDelta = new Vector2(qrSizePixels, 36f);
            capRect.anchoredPosition = new Vector2(-cornerMargin, -(cornerMargin + qrSizePixels + 2f));
            var caption = capGo.AddComponent<Text>();
            caption.font = Resources.GetBuiltinResource<Font>("LegacyRuntime.ttf");
            caption.fontSize = 16;
            caption.color = Color.white;
            caption.alignment = TextAnchor.UpperRight;
            caption.horizontalOverflow = HorizontalWrapMode.Overflow; // one line
            caption.verticalOverflow = VerticalWrapMode.Overflow;
            caption.resizeTextForBestFit = true; // shrink long URLs
            caption.resizeTextMaxSize = 16;
            caption.resizeTextMinSize = 8;
            caption.raycastTarget = false;
            capGo.AddComponent<Shadow>().effectColor = new Color(0f, 0f, 0f, 0.9f);

            root.SetActive(false);
            _roots.Add(root);
            _images.Add(image);
            _captions.Add(caption);
        }
    }
}
