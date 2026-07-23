// Visitor-facing overlay UI for the experience flow (Phase 3 of
// Plans/experience-flow-plan.md → Plans/phase3-visitor-ui-plan.md).
//
// One ScreenSpaceCamera canvas per visitor display (default 1 and 2), built
// programmatically. The ExperienceDirector prefers a scene-authored instance
// (Experience/VisitorUI) so layout fields (pose-guide position/size/displays)
// are tunable and persisted in the scene; it spawns a throwaway instance only
// when none exists. ScreenSpaceCamera (not Overlay) so the
// per-display camera routes the UI to its physical display in builds AND an
// explicit cam.Render() captures the UI in Editor verification.
//
// The red fault alert must also cover the OPERATOR display (Display 1), where
// the HUD and the 4-camera tiles are IMGUI — and IMGUI draws over uGUI. So
// the operator alert is drawn in IMGUI here (primary display only, which IS
// Display 1), and Shared.OperatorOverlayGate.AlertActive tells the other
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
                 "this targetDisplay must exist). Operator Display 1 is IMGUI-only.")]
        public int[] visitorDisplays = { 1, 2 };

        [Tooltip("Subset of visitorDisplays that show the visitor TEXT/content " +
                 "(message, countdown, QR, pose guide, progress, notice). Empty = all. " +
                 "The red fault ALERT deliberately ignores this and covers every " +
                 "visitorDisplay — a fault must be visible on every screen even though " +
                 "the wording lives on one. Default {1} = display2 only.")]
        public int[] messageDisplays = { 1 };

        [Tooltip("OS font names tried in order for the visitor text (kids-friendly " +
                 "Japanese). First installed one wins; none → LegacyRuntime.ttf + log.")]
        public string[] fontCandidates =
            { "Yu Gothic Medium", "游ゴシック Medium", "Yu Gothic", "Hiragino Kaku Gothic ProN" };

        // Visitor-flow text sizes. Scaled to 2/3 of the original (72/260/36)
        // after reading them on the real screens — at the old size the message
        // and the countdown crowded the sculpture they are supposed to caption.
        // noticeFontSize is deliberately NOT scaled with them (see below).
        [Min(10)] public int messageFontSize = 48;
        [Min(10)] public int countdownFontSize = 173;
        [Tooltip("Fault-alert text. Intentionally left at full size — the red " +
                 "alert is an operator-facing emergency, not part of the visitor " +
                 "flow, so it does not shrink with the flow text.")]
        [Min(10)] public int alertFontSize = 80;
        [Min(10)] public int captionFontSize = 24;
        [Min(64f)] public float qrSizePixels = 480f;

        [Tooltip("QR code image anchored position (reference px from canvas center). " +
                 "The caption rides 70 px below the code's bottom edge.")]
        public Vector2 qrPosition = new Vector2(0f, 60f);

        [Range(0f, 1f)]
        [Tooltip("Black backdrop opacity behind the red alert text.")]
        public float alertBackdropAlpha = 0.75f;

        [Header("Pose guide (adjust on the scene GO; live-applied while shown)")]
        [Tooltip("Displays that show the pose guide (subset of visitorDisplays). " +
                 "Other visitor displays keep the countdown/message only.")]
        public int[] poseGuideDisplays = { 1 };

        [Min(64f)]
        [Tooltip("Pose-guide figure size (reference pixels).")]
        public float poseGuideSizePixels = 420f;

        [Tooltip("Pose-guide figure anchored position (reference px from canvas center).")]
        public Vector2 poseImagePosition = new Vector2(0f, 130f);

        [Tooltip("Pose instruction text anchored position (reference px from canvas center).")]
        public Vector2 poseLabelPosition = new Vector2(0f, -200f);

        [Tooltip("Countdown digits anchored position (reference px from canvas center) — " +
                 "the message slot moves here while it is showing a countdown, so the " +
                 "digits can sit clear of the pose guide / visitor.")]
        public Vector2 countdownPosition = Vector2.zero;

        [Tooltip("Message text anchored position (reference px from canvas center).")]
        public Vector2 messagePosition = Vector2.zero;

        [Header("Notice panel (framed box — the privacy consent)")]
        [Min(10)]
        [Tooltip("Notice body size. Set independently of messageFontSize on purpose: " +
                 "the consent notice is READ up close, not glanced at from across the " +
                 "room, so it kept its size when the flow text was scaled to 2/3.")]
        public int noticeFontSize = 36;

        [Tooltip("Framed box size (reference px).")]
        public Vector2 noticeBoxSize = new Vector2(1120f, 400f);

        [Tooltip("Framed box anchored position (reference px from canvas center).")]
        public Vector2 noticePosition = Vector2.zero;

        [Min(0f)]
        [Tooltip("Frame line thickness (reference px).")]
        public float noticeBorderWidth = 4f;

        [Min(0f)]
        [Tooltip("Gap between the frame and the text (reference px).")]
        public float noticePadding = 56f;

        // No fill colour on purpose: the box is an outline, the middle stays
        // transparent so the scene behind it reads through.

        [Tooltip("Frame line colour.")]
        public Color noticeFrameColor = Color.white;

        [Tooltip("Headline above the QR (e.g. できたよ！) anchored position " +
                 "(reference px from canvas center).")]
        public Vector2 qrHeadlinePosition = new Vector2(0f, 420f);

        [Min(64f)]
        [Tooltip("Progress bar width (reference pixels).")]
        public float progressBarWidth = 800f;

        private sealed class DisplayUi
        {
            public int display;
            public Canvas canvas;
            public Text message;
            public GameObject qrGroup;
            public RawImage qrImage;
            public Text qrCaption;
            public Text qrHeadline;
            public GameObject poseGroup;
            public RawImage poseImage;
            public Text poseLabel;
            public GameObject progressGroup;
            public Text progressCaption;
            public RectTransform progressFill;
            public GameObject noticeGroup;
            public RectTransform noticeFrame;      // the box; itself draws nothing
            public Image[] noticeEdges;            // top, bottom, left, right
            public Text noticeText;
            public GameObject alertGroup;
            public Text alertText;

            // Screen size this canvas was laid out for. The CanvasScaler's factor is
            // fixed at build time from these; if the display later reports a
            // different size (activation, mode change, Game view resize) the canvas
            // is rebuilt. See RebuildResizedDisplays.
            public int builtW, builtH;
            public float staleSince;   // unscaled time the size mismatch started; 0 = matching
        }

        private readonly List<DisplayUi> _uis = new List<DisplayUi>();
        private Font _font;
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
            foreach (var ui in _uis) RetireCanvas(ui);
            _uis.Clear();
            _builtDisplays.Clear();
            _warnedDisplays.Clear();
            if (_backdropTex != null) Destroy(_backdropTex);
        }

        // ---------------- public API ----------------

        /// <summary>Centered white message on every visitor display. Replaces the
        /// whole non-alert screen (pose guide / progress / QR go down with it).</summary>
        // The content currently on screen, re-applied to canvases that a late
        // camera adds after the Show* call (startup ordering) so they don't come
        // up blank. Two layers: the base full-screen setter, plus an optional
        // countdown overlay (Calibrate shows digits ON TOP of the pose guide, so
        // the base must replay too). The alert replays separately from _alertOwned.
        private System.Action _replayBase;
        private int _countdownOverlay = -1; // -1 = no countdown on screen
        private bool _replaying;

        public void ShowMessage(string text)
        {
            EnsureBuilt();
            _replayBase = () => ShowMessage(text);
            _countdownOverlay = -1;
            _countdownMode = false;
            foreach (var ui in _uis)
            {
                if (!MessageShownOn(ui.display)) { HideContentGroups(ui); continue; }
                ui.message.fontSize = messageFontSize;
                ui.message.text = text ?? "";
                ui.message.rectTransform.anchoredPosition = messagePosition;
                ui.message.gameObject.SetActive(true);
                ui.qrGroup.SetActive(false);
                ui.poseGroup.SetActive(false);
                ui.progressGroup.SetActive(false);
                ui.noticeGroup.SetActive(false);
            }
        }

        /// <summary>Big countdown digits (same slot as the message, repositioned
        /// to countdownPosition so they can sit clear of the pose guide).</summary>
        public void ShowCountdown(int seconds)
        {
            EnsureBuilt();
            // Overlay only — the base (pose guide / cue message) stays as set.
            _countdownOverlay = seconds;
            _countdownMode = true;
            foreach (var ui in _uis)
            {
                if (!MessageShownOn(ui.display)) { HideContentGroups(ui); continue; }
                ui.message.fontSize = countdownFontSize;
                ui.message.text = seconds.ToString();
                ui.message.rectTransform.anchoredPosition = countdownPosition;
                ui.message.gameObject.SetActive(true);
                ui.qrGroup.SetActive(false);
            }
        }

        // Which layout the shared message slot is currently using (live-apply).
        private bool _countdownMode;

        /// <summary>
        /// Pose-guide figure (above center) + instruction text (below). The
        /// message/countdown slot stays free, so ShowCountdown can overlay the
        /// remaining seconds while the guide is up (Calibrate state).
        /// </summary>
        public void ShowPoseGuide(Texture2D guide, string text)
        {
            EnsureBuilt();
            _replayBase = () => ShowPoseGuide(guide, text);
            _countdownOverlay = -1; // director re-issues ShowCountdown each second
            foreach (var ui in _uis)
            {
                if (!MessageShownOn(ui.display)) { HideContentGroups(ui); continue; }
                bool shown = PoseGuideShownOn(ui.display);
                ui.poseImage.texture = guide;
                ui.poseLabel.text = text ?? "";
                ApplyPoseLayout(ui);
                ui.poseGroup.SetActive(shown);
                ui.qrGroup.SetActive(false);
                ui.progressGroup.SetActive(false);
                ui.noticeGroup.SetActive(false);
                // Drop any previous state's message; ShowCountdown re-activates
                // the slot when digits need to overlay the guide.
                ui.message.gameObject.SetActive(false);
            }
        }

        /// <summary>Framed notice box with small body text (Consent state). Put
        /// the line breaks in the text itself — the box is sized by
        /// noticeBoxSize, not by the content.</summary>
        public void ShowNotice(string text)
        {
            EnsureBuilt();
            _replayBase = () => ShowNotice(text);
            _countdownOverlay = -1;
            _countdownMode = false;
            foreach (var ui in _uis)
            {
                if (!MessageShownOn(ui.display)) { HideContentGroups(ui); continue; }
                ui.noticeText.text = text ?? "";
                ApplyNoticeLayout(ui);
                ui.noticeGroup.SetActive(true);
                ui.message.gameObject.SetActive(false);
                ui.qrGroup.SetActive(false);
                ui.poseGroup.SetActive(false);
                ui.progressGroup.SetActive(false);
            }
        }

        // Re-applied every frame while the notice is up so the box can be tuned
        // live in Play mode (same contract as ApplyPoseLayout).
        private void ApplyNoticeLayout(DisplayUi ui)
        {
            if (ui.noticeFrame == null) return;
            ui.noticeFrame.sizeDelta = noticeBoxSize;
            ui.noticeFrame.anchoredPosition = noticePosition;

            float b = noticeBorderWidth;
            float w = noticeBoxSize.x, h = noticeBoxSize.y;
            // top, bottom, left, right — the horizontals span the full width so
            // the corners are covered exactly once.
            SetEdge(ui.noticeEdges[0], new Vector2(w, b), new Vector2(0f, (h - b) * 0.5f));
            SetEdge(ui.noticeEdges[1], new Vector2(w, b), new Vector2(0f, -(h - b) * 0.5f));
            SetEdge(ui.noticeEdges[2], new Vector2(b, h - b * 2f), new Vector2(-(w - b) * 0.5f, 0f));
            SetEdge(ui.noticeEdges[3], new Vector2(b, h - b * 2f), new Vector2((w - b) * 0.5f, 0f));

            float p = noticePadding;
            ui.noticeText.rectTransform.offsetMin = new Vector2(p, p);
            ui.noticeText.rectTransform.offsetMax = new Vector2(-p, -p);
            ui.noticeText.fontSize = noticeFontSize;
        }

        private void SetEdge(Image edge, Vector2 size, Vector2 pos)
        {
            var r = edge.rectTransform;
            r.anchorMin = r.anchorMax = new Vector2(0.5f, 0.5f);
            r.pivot = new Vector2(0.5f, 0.5f);
            r.sizeDelta = size;
            r.anchoredPosition = pos;
            edge.color = noticeFrameColor;
        }

        private bool PoseGuideShownOn(int display)
        {
            if (poseGuideDisplays == null || poseGuideDisplays.Length == 0) return true;
            foreach (int d in poseGuideDisplays) if (d == display) return true;
            return false;
        }

        // Which displays show the visitor text/content. The alert path never calls
        // this — a fault covers every visitorDisplay regardless.
        private bool MessageShownOn(int display)
        {
            if (messageDisplays == null || messageDisplays.Length == 0) return true;
            foreach (int d in messageDisplays) if (d == display) return true;
            return false;
        }

        // Blank every non-alert group on a canvas that is not a message display, so
        // that display shows only the scene (and the alert, when one is up).
        private static void HideContentGroups(DisplayUi ui)
        {
            if (ui.message != null) ui.message.gameObject.SetActive(false);
            if (ui.qrGroup != null) ui.qrGroup.SetActive(false);
            if (ui.poseGroup != null) ui.poseGroup.SetActive(false);
            if (ui.progressGroup != null) ui.progressGroup.SetActive(false);
            if (ui.noticeGroup != null) ui.noticeGroup.SetActive(false);
        }

        // Re-applied every frame while a guide is up so the scene GO's Inspector
        // values can be tuned live in Play mode (copy component values to keep).
        private void ApplyPoseLayout(DisplayUi ui)
        {
            var rect = (RectTransform)ui.poseImage.transform;
            rect.sizeDelta = new Vector2(poseGuideSizePixels, poseGuideSizePixels);
            rect.anchoredPosition = poseImagePosition;
            ui.poseLabel.rectTransform.anchoredPosition = poseLabelPosition;
            ui.poseLabel.fontSize = messageFontSize;
        }

        // Same live-apply contract for the QR screen: code position/size, the
        // caption riding 70 px below the code's bottom edge, and the headline.
        private void ApplyQrLayout(DisplayUi ui)
        {
            var qrRect = (RectTransform)ui.qrImage.transform;
            qrRect.sizeDelta = new Vector2(qrSizePixels, qrSizePixels);
            qrRect.anchoredPosition = qrPosition;
            ui.qrCaption.rectTransform.anchoredPosition =
                new Vector2(qrPosition.x, qrPosition.y - qrSizePixels * 0.5f - 70f);
            ui.qrCaption.fontSize = captionFontSize;
            ui.qrCaption.resizeTextMaxSize = captionFontSize;
            ui.qrHeadline.rectTransform.anchoredPosition = qrHeadlinePosition;
            ui.qrHeadline.fontSize = messageFontSize;
        }

        private void Update()
        {
            // retry canvases for displays whose camera (or OS display) wasn't up at OnEnable
            if (_uis.Count < visitorDisplays.Length) EnsureBuilt();
            RebuildResizedDisplays();
            foreach (var ui in _uis)
            {
                if (ui.poseGroup != null && ui.poseGroup.activeSelf) ApplyPoseLayout(ui);
                if (ui.noticeGroup != null && ui.noticeGroup.activeSelf) ApplyNoticeLayout(ui);
                // live-apply countdown/message position + size (panel/Inspector
                // tuning in Play mode)
                if (ui.message != null && ui.message.gameObject.activeSelf)
                {
                    ui.message.rectTransform.anchoredPosition =
                        _countdownMode ? countdownPosition : messagePosition;
                    ui.message.fontSize = _countdownMode ? countdownFontSize : messageFontSize;
                }
                if (ui.qrGroup != null && ui.qrGroup.activeSelf) ApplyQrLayout(ui);
            }
        }

        /// <summary>Caption + horizontal progress bar (Processing state).</summary>
        public void ShowProgress(float value01, string caption)
        {
            EnsureBuilt();
            _replayBase = () => ShowProgress(value01, caption);
            _countdownOverlay = -1;
            float v = Mathf.Clamp01(value01);
            foreach (var ui in _uis)
            {
                if (!MessageShownOn(ui.display)) { HideContentGroups(ui); continue; }
                ui.progressCaption.text = caption ?? "";
                ui.progressFill.sizeDelta = new Vector2(progressBarWidth * v, 0f);
                ui.progressGroup.SetActive(true);
                ui.qrGroup.SetActive(false);
                ui.poseGroup.SetActive(false);
                ui.message.gameObject.SetActive(false);
                ui.noticeGroup.SetActive(false);
            }
        }

        /// <summary>The result screen BEFORE the QR exists (upload still
        /// running): the headline (できたよ！) already sits at its final
        /// QR-screen position so ShowQr later just adds the code beneath it —
        /// one seamless scene, no text jump.</summary>
        public void ShowResult(string headline)
        {
            EnsureBuilt();
            _replayBase = () => ShowResult(headline);
            _countdownOverlay = -1;
            foreach (var ui in _uis)
            {
                if (!MessageShownOn(ui.display)) { HideContentGroups(ui); continue; }
                ui.qrImage.gameObject.SetActive(false);
                ui.qrCaption.text = "";
                ui.qrHeadline.text = headline ?? "";
                ApplyQrLayout(ui);
                ui.qrHeadline.gameObject.SetActive(!string.IsNullOrEmpty(headline));
                ui.qrGroup.SetActive(true);
                ui.message.gameObject.SetActive(false);
                ui.poseGroup.SetActive(false);
                ui.progressGroup.SetActive(false);
                ui.noticeGroup.SetActive(false);
            }
        }

        /// <summary>QR + caption, centered, with an optional headline above the
        /// code (e.g. できたよ！ so the result cheer and the QR share one
        /// screen). Hidden behind the alert if one is up.</summary>
        public void ShowQr(Texture2D qr, string caption, string headline = null)
        {
            EnsureBuilt();
            _replayBase = () => ShowQr(qr, caption, headline);
            _countdownOverlay = -1;
            foreach (var ui in _uis)
            {
                if (!MessageShownOn(ui.display)) { HideContentGroups(ui); continue; }
                ui.noticeGroup.SetActive(false);
                ui.qrImage.texture = qr;
                ui.qrImage.gameObject.SetActive(true);
                ui.qrCaption.text = caption ?? "";
                ui.qrHeadline.text = headline ?? "";
                ApplyQrLayout(ui);
                ui.qrHeadline.gameObject.SetActive(!string.IsNullOrEmpty(headline));
                ui.qrGroup.SetActive(true);
                ui.message.gameObject.SetActive(false);
                ui.poseGroup.SetActive(false);
                ui.progressGroup.SetActive(false);
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
            _replayBase = null; // late canvases stay blank (Idle floor-grid only)
            _countdownOverlay = -1;
            foreach (var ui in _uis)
            {
                if (ui.message != null) ui.message.gameObject.SetActive(false);
                if (ui.qrGroup != null) ui.qrGroup.SetActive(false);
                if (ui.poseGroup != null) ui.poseGroup.SetActive(false);
                if (ui.progressGroup != null) ui.progressGroup.SetActive(false);
                if (ui.noticeGroup != null) ui.noticeGroup.SetActive(false);
            }
        }

        public void ClearEverything()
        {
            ClearAll();
            ClearAlert();
        }

        // ---------------- operator-side alert (IMGUI, Display 1) ----------------

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
                fontStyle = FontStyle.Bold,
                normal = { textColor = Color.red }
            };
            _operatorAlertStyle.fontSize = Mathf.RoundToInt(alertFontSize * Screen.height / 1080f);

            GUI.DrawTexture(new Rect(0, 0, Screen.width, Screen.height), _backdropTex);
            GUI.Label(new Rect(Screen.width * 0.05f, 0, Screen.width * 0.9f, Screen.height),
                      _alertMessage, _operatorAlertStyle);
        }

        // ---------------- construction ----------------

        // Per-display build with retry: a scene-authored instance's OnEnable can
        // run before the display cameras are enabled (startup ordering), so a
        // missing camera must NOT permanently skip that display — every call
        // (Show*, Update) retries the still-unbuilt ones until the camera shows up.
        private readonly HashSet<int> _builtDisplays = new HashSet<int>();
        private readonly HashSet<int> _warnedDisplays = new HashSet<int>();

        // Canvas GameObjects that survive without their bookkeeping. _uis and
        // _builtDisplays are plain runtime state, so an Editor domain reload
        // (recompiling while in play mode) wipes them while the canvases they
        // describe stay in the hierarchy — EnsureBuilt then builds a SECOND set and
        // the stale one keeps rendering underneath at whatever scale it was laid out
        // for, which reads as text at the wrong size and position. Adopt nothing,
        // just delete what we no longer track.
        //
        // Ownership is the VisitorUiCanvasTag component, not the name: the tag is
        // only ever added by BuildDisplayUi, it survives the domain reload that
        // loses the bookkeeping, and a scene object that merely shares the naming
        // convention is left alone.
        private void DestroyOrphanCanvases()
        {
            for (int i = transform.childCount - 1; i >= 0; i--)
            {
                var child = transform.GetChild(i);
                var tag = child.GetComponent<VisitorUiCanvasTag>();
                if (tag == null || tag.retired) continue;
                bool tracked = false;
                foreach (var ui in _uis)
                    if (ui.canvas != null && ui.canvas.transform == child) { tracked = true; break; }
                if (tracked) continue;
                Debug.LogWarning($"[{nameof(VisitorMessageUI)}] orphaned canvas '{child.name}' " +
                                 "(domain reload during play?) — destroying it.", this);
                RetireCanvas(tag, child.gameObject);
            }
        }

        // Single retirement path for a canvas we own. Unity's Destroy is deferred to
        // the end of frame, so the object stays reachable meanwhile: mark the tag
        // first (the orphan sweep skips retired tags, otherwise a canvas retired on
        // purpose — resize rebuild, teardown — is reported as a domain-reload orphan
        // and destroyed a second time) and hide it so it stops drawing right away.
        private void RetireCanvas(DisplayUi ui)
        {
            if (ui == null || ui.canvas == null) return;
            RetireCanvas(ui.canvas.GetComponent<VisitorUiCanvasTag>(), ui.canvas.gameObject);
        }

        private void RetireCanvas(VisitorUiCanvasTag tag, GameObject go)
        {
            if (go == null) return;
            if (tag != null) tag.retired = true;
            go.SetActive(false);
            Destroy(go);
        }

        private void EnsureBuilt()
        {
            ResolveFont();
            DestroyOrphanCanvases();
            bool addedAny = false;
            foreach (int d in visitorDisplays)
            {
                if (_builtDisplays.Contains(d)) continue;
                if (!DisplayReady(d)) continue; // OS display not activated yet
                var cam = FindDisplayCamera(d);
                if (cam == null)
                {
                    if (_warnedDisplays.Add(d))
                        Debug.LogWarning($"[{nameof(VisitorMessageUI)}] no enabled camera with " +
                                         $"targetDisplay {d} yet — will keep retrying.", this);
                    continue;
                }
                _uis.Add(BuildDisplayUi(d, cam));
                _builtDisplays.Add(d);
                addedAny = true;
                if (_warnedDisplays.Remove(d))
                    Debug.Log($"[{nameof(VisitorMessageUI)}] display {d} camera appeared — canvas built.", this);
            }

            // Newly built canvases start blank — replay the current visual onto
            // them so a late display shows the same thing as its siblings. Guard
            // re-entrancy: the Show* replays call EnsureBuilt again (no-op now).
            if (addedAny && !_replaying)
            {
                _replaying = true;
                int cd = _countdownOverlay; // _replayBase resets it — capture first
                _replayBase?.Invoke();
                if (cd >= 0) ShowCountdown(cd);
                if (_alertOwned) foreach (var ui in _uis)
                {
                    ui.alertText.text = _alertMessage;
                    ui.alertGroup.SetActive(true);
                }
                _replaying = false;
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

        // A canvas built before MultiDisplayActivator calls Display.Activate() gets
        // its CanvasScaler factor from whatever size the inactive display reported —
        // the text then renders small and pushed toward the bottom-left corner for
        // the whole run, while the 3D content (which does not go through the canvas)
        // looks perfectly fine. Nothing guarantees the Start order between the two,
        // so the same build did it only sometimes. Wait for the activation instead.
        //
        // Display 1 is always active. In the Editor Display.displays has a single
        // entry no matter how many Game views are open, so displays past the end are
        // let through there (they preview in a Game view) and skipped in a player,
        // where they would render nowhere until the monitor actually shows up —
        // EnsureBuilt keeps retrying either way.
        private static bool DisplayReady(int display)
        {
            if (display <= 0) return true;
            if (display < Display.displays.Length) return Display.displays[display].active;
            return Application.isEditor;
        }

        // Size the canvas for `display` should currently be laid out for.
        private static void GetDisplaySize(int display, out int w, out int h)
        {
            if (display > 0 && display < Display.displays.Length)
            {
                var d = Display.displays[display];
                w = d.renderingWidth; h = d.renderingHeight;
                return;
            }
            w = Screen.width; h = Screen.height;
        }

        // Rebuild any canvas whose display changed size since it was built (late
        // activation, resolution change, Game view resize). Debounced so dragging a
        // Game view edge doesn't rebuild once per frame.
        private const float kResizeSettleSeconds = 0.5f;

        private void RebuildResizedDisplays()
        {
            _resizedScratch.Clear();
            float now = Time.unscaledTime;
            foreach (var ui in _uis)
            {
                GetDisplaySize(ui.display, out int w, out int h);
                if (w == ui.builtW && h == ui.builtH) { ui.staleSince = 0f; continue; }
                if (ui.staleSince <= 0f) { ui.staleSince = now; continue; }
                if (now - ui.staleSince < kResizeSettleSeconds) continue;
                _resizedScratch.Add(ui);
            }
            if (_resizedScratch.Count == 0) return;

            foreach (var ui in _resizedScratch)
            {
                GetDisplaySize(ui.display, out int w, out int h);
                Debug.Log($"[{nameof(VisitorMessageUI)}] display {ui.display} resized " +
                          $"{ui.builtW}x{ui.builtH} -> {w}x{h}; rebuilding its canvas.", this);
                RetireCanvas(ui);
                _uis.Remove(ui);
                _builtDisplays.Remove(ui.display);
            }
            _resizedScratch.Clear();
            EnsureBuilt(); // rebuilds and replays the current visual onto them
        }

        private readonly List<DisplayUi> _resizedScratch = new List<DisplayUi>();

        private DisplayUi BuildDisplayUi(int display, Camera cam)
        {
            // `display` is Unity's 0-based targetDisplay; the name is 1-origin
            // (project convention) so the hierarchy reads the same as the
            // Inspector, the OS and the physical screens.
            var root = new GameObject($"_VisitorUI_Display{display + 1}");
            root.transform.SetParent(transform, false);
            root.AddComponent<VisitorUiCanvasTag>().display = display; // ownership marker for the orphan sweep

            var canvas = root.AddComponent<Canvas>();
            // Overlay, NOT ScreenSpaceCamera: the visitor displays are mirrored by
            // CameraControl.MirrorDisplayCamera, which negates the camera's
            // projection X. A camera-space canvas goes through that projection and
            // every glyph comes out reversed. Overlay bypasses the camera entirely,
            // which is exactly the assumption MirrorDisplayCamera documents.
            canvas.renderMode = RenderMode.ScreenSpaceOverlay;
            canvas.targetDisplay = display;
            canvas.sortingOrder = 5000; // stay above any future scene canvases

            var scaler = root.AddComponent<CanvasScaler>();
            scaler.uiScaleMode = CanvasScaler.ScaleMode.ScaleWithScreenSize;
            scaler.referenceResolution = new Vector2(1920f, 1080f);
            scaler.matchWidthOrHeight = 0.5f;

            GetDisplaySize(display, out int builtW, out int builtH);
            var ui = new DisplayUi { display = display, canvas = canvas, builtW = builtW, builtH = builtH };

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
            ui.qrImage = qrGo.AddComponent<RawImage>();
            ui.qrImage.color = Color.white;

            ui.qrCaption = MakeText(ui.qrGroup.transform, "QrCaption", captionFontSize, Color.white,
                                    new Vector2(1700f, 60f));
            ui.qrCaption.horizontalOverflow = HorizontalWrapMode.Overflow; // one line, no wrap
            ui.qrCaption.resizeTextForBestFit = true;                      // shrink long URLs
            ui.qrCaption.resizeTextMaxSize = captionFontSize;
            ui.qrCaption.resizeTextMinSize = 12;

            ui.qrHeadline = MakeText(ui.qrGroup.transform, "QrHeadline", messageFontSize,
                                     Color.white, new Vector2(1700f, 120f));
            ui.qrHeadline.gameObject.SetActive(false);
            ApplyQrLayout(ui); // qrPosition / qrSizePixels / caption / headline
            ui.qrGroup.SetActive(false);

            // -- pose-guide group: figure above center, instruction below.
            //    The message/countdown slot stays independent so the big digits
            //    can overlay the guide during Calibrate. --
            ui.poseGroup = new GameObject("PoseGroup", typeof(RectTransform));
            ui.poseGroup.transform.SetParent(root.transform, false);
            Stretch(ui.poseGroup.GetComponent<RectTransform>());

            var poseGo = new GameObject("PoseImage", typeof(RectTransform));
            poseGo.transform.SetParent(ui.poseGroup.transform, false);
            ui.poseImage = poseGo.AddComponent<RawImage>();
            ui.poseImage.color = Color.white;
            ui.poseImage.raycastTarget = false;

            ui.poseLabel = MakeText(ui.poseGroup.transform, "PoseLabel", messageFontSize, Color.white,
                                    new Vector2(1700f, 200f));
            ApplyPoseLayout(ui); // poseImagePosition / poseLabelPosition / size
            ui.poseGroup.SetActive(false);

            // -- progress group: caption + horizontal bar --
            ui.progressGroup = new GameObject("ProgressGroup", typeof(RectTransform));
            ui.progressGroup.transform.SetParent(root.transform, false);
            Stretch(ui.progressGroup.GetComponent<RectTransform>());

            ui.progressCaption = MakeText(ui.progressGroup.transform, "ProgressCaption",
                                          messageFontSize, Color.white, new Vector2(1700f, 200f));
            ui.progressCaption.rectTransform.anchoredPosition = new Vector2(0f, 90f);

            var barBackGo = new GameObject("BarBackground", typeof(RectTransform));
            barBackGo.transform.SetParent(ui.progressGroup.transform, false);
            var barBackRect = barBackGo.GetComponent<RectTransform>();
            barBackRect.sizeDelta = new Vector2(progressBarWidth, 40f);
            barBackRect.anchoredPosition = new Vector2(0f, -60f);
            var barBack = barBackGo.AddComponent<Image>();
            barBack.color = new Color(1f, 1f, 1f, 0.15f);
            barBack.raycastTarget = false;

            var fillGo = new GameObject("BarFill", typeof(RectTransform));
            fillGo.transform.SetParent(barBackGo.transform, false);
            ui.progressFill = fillGo.GetComponent<RectTransform>();
            // anchored to the left edge, height follows the bar, width set by ShowProgress
            ui.progressFill.anchorMin = new Vector2(0f, 0f);
            ui.progressFill.anchorMax = new Vector2(0f, 1f);
            ui.progressFill.pivot = new Vector2(0f, 0.5f);
            ui.progressFill.anchoredPosition = Vector2.zero;
            ui.progressFill.sizeDelta = new Vector2(0f, 0f);
            var fill = fillGo.AddComponent<Image>();
            fill.color = Color.white;
            fill.raycastTarget = false;
            ui.progressGroup.SetActive(false);

            // -- notice group: a framed box with small body text. Built from two
            //    nested Images (outer = frame colour, inner inset by the border
            //    width = fill) so no 9-sliced sprite asset is needed. --
            ui.noticeGroup = new GameObject("NoticeGroup", typeof(RectTransform));
            ui.noticeGroup.transform.SetParent(root.transform, false);
            Stretch(ui.noticeGroup.GetComponent<RectTransform>());

            var frameGo = new GameObject("NoticeFrame", typeof(RectTransform));
            frameGo.transform.SetParent(ui.noticeGroup.transform, false);
            ui.noticeFrame = frameGo.GetComponent<RectTransform>();

            // Four edge bars, NOT a filled rect behind a smaller one: the box is
            // an outline only, so whatever is behind it (the floor grid, the
            // sculpture) stays visible through the middle.
            ui.noticeEdges = new Image[4];
            string[] edgeNames = { "EdgeTop", "EdgeBottom", "EdgeLeft", "EdgeRight" };
            for (int e = 0; e < 4; e++)
            {
                var edgeGo = new GameObject(edgeNames[e], typeof(RectTransform));
                edgeGo.transform.SetParent(frameGo.transform, false);
                var img = edgeGo.AddComponent<Image>();
                img.raycastTarget = false;
                ui.noticeEdges[e] = img;
            }

            ui.noticeText = MakeText(frameGo.transform, "NoticeText", noticeFontSize,
                                     Color.white, Vector2.zero);
            Stretch(ui.noticeText.rectTransform); // inset per-frame by noticePadding
            ApplyNoticeLayout(ui);
            ui.noticeGroup.SetActive(false);

            // Countdown digits must draw OVER the pose guide (Calibrate overlays
            // both) — push the message slot above the content groups; the alert
            // group is created after and stays topmost.
            ui.message.transform.SetAsLastSibling();

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
            ui.alertText.fontStyle = FontStyle.Bold; // fault text carries weight
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
