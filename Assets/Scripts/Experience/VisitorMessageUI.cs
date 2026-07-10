// Visitor-facing overlay UI for the experience flow (Phase 3 of
// Plans/experience-flow-plan.md → Plans/phase3-visitor-ui-plan.md).
//
// One ScreenSpaceCamera canvas per visitor display (default 1 and 2), built
// programmatically — no scene/prefab dependency; the ExperienceDirector will
// own an instance in a later phase. ScreenSpaceCamera (not Overlay) so the
// per-display camera routes the UI to its physical display in builds AND an
// explicit cam.Render() captures the UI in Editor verification.
//
// The red fault alert must also cover the OPERATOR display (display 0), where
// the HUD and the 4-camera tiles are IMGUI — and IMGUI draws over uGUI. So
// the operator alert is drawn in IMGUI here (primary display only, which IS
// display 0), and Shared.OperatorOverlayGate.AlertActive tells the other
// IMGUI overlays to stand down while it is up.
//
// Priority inside the visitor canvases is sibling order: message < QR < alert
// (the alert group is the last sibling and has a full-screen backdrop).

using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

namespace Experience
{
    [DisallowMultipleComponent]
    public class VisitorMessageUI : MonoBehaviour
    {
        [Tooltip("Display indices that get a visitor overlay canvas (a camera with " +
                 "this targetDisplay must exist). Operator display 0 is IMGUI-only.")]
        public int[] visitorDisplays = { 1, 2 };

        [Tooltip("OS font names tried in order for the visitor text (kids-friendly " +
                 "Japanese). First installed one wins; none → LegacyRuntime.ttf + log.")]
        public string[] fontCandidates =
            { "Yu Gothic Medium", "游ゴシック Medium", "Yu Gothic", "Hiragino Kaku Gothic ProN" };

        [Min(10)] public int messageFontSize = 72;
        [Min(10)] public int countdownFontSize = 260;
        [Min(10)] public int alertFontSize = 80;
        [Min(10)] public int captionFontSize = 36;
        [Min(64f)] public float qrSizePixels = 480f;

        [Range(0f, 1f)]
        [Tooltip("Black backdrop opacity behind the red alert text.")]
        public float alertBackdropAlpha = 0.75f;

        private sealed class DisplayUi
        {
            public Canvas canvas;
            public Text message;
            public GameObject qrGroup;
            public RawImage qrImage;
            public Text qrCaption;
            public GameObject alertGroup;
            public Text alertText;
        }

        private readonly List<DisplayUi> _uis = new List<DisplayUi>();
        private Font _font;
        private bool _built;
        // Operator-side alert state. The gate is static/shared — only the
        // instance that raised it may clear it (multiple instances must not
        // stomp each other's alert).
        private bool _alertOwned;
        private string _alertMessage = "";
        private GUIStyle _operatorAlertStyle;
        private Texture2D _backdropTex;

        public bool AlertVisible => _alertOwned;

        // ---------------- lifecycle ----------------

        private void OnEnable()
        {
            EnsureBuilt();
            // Re-raise an alert this instance still owns (disable/enable cycle).
            if (_alertOwned) Shared.OperatorOverlayGate.AlertActive = true;
        }

        private void OnDisable()
        {
            // While disabled we can't draw the operator alert — release the gate
            // so the HUD/tiles aren't suppressed behind an invisible alert.
            if (_alertOwned) Shared.OperatorOverlayGate.AlertActive = false;
        }

        private void OnDestroy()
        {
            if (_alertOwned)
            {
                Shared.OperatorOverlayGate.AlertActive = false;
                _alertOwned = false;
            }
            foreach (var ui in _uis)
                if (ui.canvas != null) Destroy(ui.canvas.gameObject);
            _uis.Clear();
            if (_backdropTex != null) Destroy(_backdropTex);
            _built = false;
        }

        // ---------------- public API ----------------

        /// <summary>Centered white message on every visitor display.</summary>
        public void ShowMessage(string text)
        {
            EnsureBuilt();
            foreach (var ui in _uis)
            {
                ui.message.fontSize = messageFontSize;
                ui.message.text = text ?? "";
                ui.message.gameObject.SetActive(true);
                ui.qrGroup.SetActive(false);
            }
        }

        /// <summary>Big countdown digits (same slot as the message).</summary>
        public void ShowCountdown(int seconds)
        {
            EnsureBuilt();
            foreach (var ui in _uis)
            {
                ui.message.fontSize = countdownFontSize;
                ui.message.text = seconds.ToString();
                ui.message.gameObject.SetActive(true);
                ui.qrGroup.SetActive(false);
            }
        }

        /// <summary>QR + caption, centered. Hidden behind the alert if one is up.</summary>
        public void ShowQr(Texture2D qr, string caption)
        {
            EnsureBuilt();
            foreach (var ui in _uis)
            {
                ui.qrImage.texture = qr;
                ui.qrCaption.text = caption ?? "";
                ui.qrGroup.SetActive(true);
                ui.message.gameObject.SetActive(false);
            }
        }

        /// <summary>Full-screen red fault alert — every visitor display AND the
        /// operator display (IMGUI, see header). Highest priority; stays up
        /// through ClearAll, dropped only by ClearAlert / ClearEverything.</summary>
        public void ShowAlert(string text)
        {
            EnsureBuilt();
            _alertMessage = text ?? "";
            foreach (var ui in _uis)
            {
                ui.alertText.text = _alertMessage;
                ui.alertGroup.SetActive(true);
            }
            _alertOwned = true;
            Shared.OperatorOverlayGate.AlertActive = true;
        }

        public void ClearAlert()
        {
            foreach (var ui in _uis)
                if (ui.alertGroup != null) ui.alertGroup.SetActive(false);
            if (_alertOwned)
            {
                Shared.OperatorOverlayGate.AlertActive = false;
                _alertOwned = false;
            }
        }

        /// <summary>Clear message/countdown/QR. The alert survives (a fault
        /// outlives whatever the show was doing).</summary>
        public void ClearAll()
        {
            foreach (var ui in _uis)
            {
                if (ui.message != null) ui.message.gameObject.SetActive(false);
                if (ui.qrGroup != null) ui.qrGroup.SetActive(false);
            }
        }

        public void ClearEverything()
        {
            ClearAll();
            ClearAlert();
        }

        // ---------------- operator-side alert (IMGUI, display 0) ----------------

        private void OnGUI()
        {
            if (!_alertOwned) return;
            GUI.depth = -1000; // in front of any other IMGUI that ignored the gate

            if (_backdropTex == null)
            {
                _backdropTex = new Texture2D(1, 1, TextureFormat.RGBA32, false);
                _backdropTex.SetPixel(0, 0, new Color(0f, 0f, 0f, alertBackdropAlpha));
                _backdropTex.Apply();
            }
            _operatorAlertStyle ??= new GUIStyle
            {
                alignment = TextAnchor.MiddleCenter,
                wordWrap = true,
                normal = { textColor = Color.red }
            };
            _operatorAlertStyle.fontSize = Mathf.RoundToInt(alertFontSize * Screen.height / 1080f);

            GUI.DrawTexture(new Rect(0, 0, Screen.width, Screen.height), _backdropTex);
            GUI.Label(new Rect(Screen.width * 0.05f, 0, Screen.width * 0.9f, Screen.height),
                      _alertMessage, _operatorAlertStyle);
        }

        // ---------------- construction ----------------

        private void EnsureBuilt()
        {
            if (_built) return;
            _built = true;
            ResolveFont();
            foreach (int d in visitorDisplays)
            {
                var cam = FindDisplayCamera(d);
                if (cam == null)
                {
                    Debug.LogWarning($"[{nameof(VisitorMessageUI)}] no enabled camera with " +
                                     $"targetDisplay {d} — skipping that display.", this);
                    continue;
                }
                _uis.Add(BuildDisplayUi(d, cam));
            }
        }

        private void ResolveFont()
        {
            if (_font != null) return;
            string picked = null;
            var installed = new HashSet<string>(Font.GetOSInstalledFontNames());
            foreach (var candidate in fontCandidates)
                if (!string.IsNullOrEmpty(candidate) && installed.Contains(candidate))
                { picked = candidate; break; }

            if (picked != null)
            {
                _font = Font.CreateDynamicFontFromOSFont(picked, messageFontSize);
                Debug.Log($"[{nameof(VisitorMessageUI)}] visitor font: OS \"{picked}\".", this);
            }
            else
            {
                _font = Resources.GetBuiltinResource<Font>("LegacyRuntime.ttf");
                Debug.LogWarning($"[{nameof(VisitorMessageUI)}] none of the candidate OS fonts " +
                                 $"({string.Join(", ", fontCandidates)}) are installed — falling " +
                                 "back to LegacyRuntime.ttf (Japanese glyph coverage may vary).", this);
            }
        }

        private static Camera FindDisplayCamera(int display)
        {
            foreach (var cam in Camera.allCameras) // enabled cameras only
                if (cam.targetDisplay == display) return cam;
            return null;
        }

        private DisplayUi BuildDisplayUi(int display, Camera cam)
        {
            var root = new GameObject($"_VisitorUI_Display{display}");
            root.transform.SetParent(transform, false);

            var canvas = root.AddComponent<Canvas>();
            canvas.renderMode = RenderMode.ScreenSpaceCamera;
            canvas.worldCamera = cam;
            canvas.planeDistance = 1f;
            canvas.sortingOrder = 5000; // stay above any future scene canvases

            var scaler = root.AddComponent<CanvasScaler>();
            scaler.uiScaleMode = CanvasScaler.ScaleMode.ScaleWithScreenSize;
            scaler.referenceResolution = new Vector2(1920f, 1080f);
            scaler.matchWidthOrHeight = 0.5f;

            var ui = new DisplayUi { canvas = canvas };

            // -- message / countdown (one slot) --
            ui.message = MakeText(root.transform, "Message", messageFontSize, Color.white,
                                  new Vector2(1700f, 700f));
            ui.message.gameObject.SetActive(false);

            // -- QR group: image + caption --
            ui.qrGroup = new GameObject("QrGroup", typeof(RectTransform));
            ui.qrGroup.transform.SetParent(root.transform, false);
            Stretch(ui.qrGroup.GetComponent<RectTransform>());

            var qrGo = new GameObject("QrImage", typeof(RectTransform));
            qrGo.transform.SetParent(ui.qrGroup.transform, false);
            var qrRect = qrGo.GetComponent<RectTransform>();
            qrRect.sizeDelta = new Vector2(qrSizePixels, qrSizePixels);
            qrRect.anchoredPosition = new Vector2(0f, 60f);
            ui.qrImage = qrGo.AddComponent<RawImage>();
            ui.qrImage.color = Color.white;

            ui.qrCaption = MakeText(ui.qrGroup.transform, "QrCaption", captionFontSize, Color.white,
                                    new Vector2(1700f, 60f));
            var capRect = ui.qrCaption.rectTransform;
            capRect.anchoredPosition = new Vector2(0f, -(qrSizePixels * 0.5f) + 60f - 70f);
            ui.qrCaption.horizontalOverflow = HorizontalWrapMode.Overflow; // one line, no wrap
            ui.qrCaption.resizeTextForBestFit = true;                      // shrink long URLs
            ui.qrCaption.resizeTextMaxSize = captionFontSize;
            ui.qrCaption.resizeTextMinSize = 12;
            ui.qrGroup.SetActive(false);

            // -- alert group: backdrop + red text, LAST sibling = on top --
            ui.alertGroup = new GameObject("AlertGroup", typeof(RectTransform));
            ui.alertGroup.transform.SetParent(root.transform, false);
            Stretch(ui.alertGroup.GetComponent<RectTransform>());

            var backGo = new GameObject("Backdrop", typeof(RectTransform));
            backGo.transform.SetParent(ui.alertGroup.transform, false);
            Stretch(backGo.GetComponent<RectTransform>());
            var back = backGo.AddComponent<Image>();
            back.color = new Color(0f, 0f, 0f, alertBackdropAlpha);

            ui.alertText = MakeText(ui.alertGroup.transform, "AlertText", alertFontSize, Color.red,
                                    new Vector2(1700f, 800f));
            ui.alertGroup.SetActive(false);

            return ui;
        }

        private Text MakeText(Transform parent, string name, int size, Color color, Vector2 rectSize)
        {
            var go = new GameObject(name, typeof(RectTransform));
            go.transform.SetParent(parent, false);
            var rect = go.GetComponent<RectTransform>();
            rect.sizeDelta = rectSize; // centered anchors by default
            var text = go.AddComponent<Text>();
            text.font = _font;
            text.fontSize = size;
            text.color = color;
            text.alignment = TextAnchor.MiddleCenter;
            text.horizontalOverflow = HorizontalWrapMode.Wrap;
            text.verticalOverflow = VerticalWrapMode.Truncate;
            text.raycastTarget = false;
            return text;
        }

        private static void Stretch(RectTransform rect)
        {
            rect.anchorMin = Vector2.zero;
            rect.anchorMax = Vector2.one;
            rect.offsetMin = Vector2.zero;
            rect.offsetMax = Vector2.zero;
        }
    }
}
