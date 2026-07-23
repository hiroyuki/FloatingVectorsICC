// Frame-rate readout in the top-right corner of every display.
//
// Why this exists: in a standalone multi-display build there is no profiler and
// no console — the only way to tell whether the operator display or one of the
// two visitor displays is dropping frames is to see the number on the screen
// itself. All displays are driven by the one player loop, so the number is the
// same everywhere; showing it on each screen just means whichever screen you
// happen to be standing in front of can answer the question.
//
// Drawn as a uGUI Canvas per display, NOT as IMGUI: IMGUI only reaches the
// primary display in a standalone build, and it renders underneath every
// ScreenSpaceOverlay canvas (see DisplayDiagnosticsOverlay for the same
// reasoning).
//
// Self-installing so it is available in any build without scene wiring.

using UnityEngine;
using UnityEngine.UI;

namespace Shared
{
    [DisallowMultipleComponent]
    public class FpsOverlay : MonoBehaviour
    {
        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
        private static void Install()
        {
            if (FindFirstObjectByType<FpsOverlay>() != null) return;
            var go = new GameObject("[FpsOverlay]");
            go.AddComponent<FpsOverlay>();
            DontDestroyOnLoad(go);
        }

        [Tooltip("Show/hide the readout on every display.")]
        public bool visible = true;

        [Tooltip("Toggle the readout.")]
        public KeyCode toggleKey = KeyCode.F2;

        [Tooltip("Displays to draw on, including the primary one. Matches the " +
                 "MultiDisplayActivator count (operator + two visitor displays). " +
                 "Canvases for displays that are not connected simply never render, " +
                 "and in the Editor they show up in extra Game views.")]
        public int displayCount = 3;

        [Tooltip("Averaging window in seconds. Longer reads steadier, shorter reacts " +
                 "faster to a stall.")]
        public float windowSeconds = 0.5f;

        [Tooltip("Also show the worst single frame in the window (ms). That is what " +
                 "catches hitches an average hides.")]
        public bool showWorstFrame = true;

        [Tooltip("Also show the body-tracking cadence: 'pose' is how often the pose " +
                 "history advances (what the motion-curve trail steps at); 'fused' and " +
                 "'fresh' are the live RTMPose fusion rate and the part of it carrying a " +
                 "genuinely new inference. A healthy fps with a low skeleton rate is " +
                 "exactly what a stuttering trail looks like — the frame cap says nothing " +
                 "about how often the sculpture actually advances. Only the fields the " +
                 "running scene produces are drawn, and the line is hidden entirely until " +
                 "some skeleton source reports.")]
        public bool showPoseRate = true;

        private Text[] _texts;
        private int _lines = 1;      // label line count the panels are currently sized for
        private int _frames;
        private float _elapsed;
        private float _worstFrame;

        private void Update()
        {
            if (Input.GetKeyDown(toggleKey))
            {
                visible = !visible;
                ApplyVisible();
            }

            // Unscaled: a paused/frozen stage still has a real frame rate.
            float dt = Time.unscaledDeltaTime;
            _frames++;
            _elapsed += dt;
            if (dt > _worstFrame) _worstFrame = dt;

            if (!visible || _elapsed < Mathf.Max(0.05f, windowSeconds)) return;

            float fps = _frames / _elapsed;
            float avgMs = 1000f * _elapsed / _frames;
            string label = showWorstFrame
                ? $"{fps:0.0} fps   {avgMs:0.0} ms   max {1000f * _worstFrame:0.0}"
                : $"{fps:0.0} fps   {avgMs:0.0} ms";
            // Only the counters this scene actually produces. Keying the line to one
            // specific producer would blank it on a scene that runs the other, so each
            // field appears on its own terms.
            bool rateLine = showPoseRate && (RateProbe.AnyKnown || SkeletonSourceProbe.EverReported);
            if (rateLine)
            {
                // Which engine is driving comes first: the rates below mean different
                // things per engine, and a build that silently fell back to k4abt is
                // otherwise indistinguishable from one running RTMPose.
                label += "\nsrc " + (SkeletonSourceProbe.Current ?? "none");
                if (RateProbe.IsKnown(RateProbe.Fused))
                    label += $"   fused {RateProbe.Hz(RateProbe.Fused):0.0}"
                           + $"   fresh {RateProbe.Hz(RateProbe.FreshFused):0.0}";
                if (RateProbe.IsKnown(RateProbe.PoseIngest))
                    label += $"   pose {RateProbe.Hz(RateProbe.PoseIngest):0.0}";
                label += " Hz";
            }

            EnsureCanvases();
            // The rate line appears only once a skeleton source starts, so the backing
            // panel has to grow then — a fixed height would leave the second line drawn
            // outside the dark box and unreadable over the point cloud.
            int lines = rateLine ? 2 : 1;
            for (int i = 0; i < _texts.Length; i++)
            {
                if (_texts[i] == null) continue;
                _texts[i].text = label;
                if (lines != _lines && _texts[i].transform.parent is RectTransform panel)
                    panel.sizeDelta = new Vector2(panel.sizeDelta.x, PanelHeight(lines));
            }
            _lines = lines;

            _frames = 0;
            _elapsed = 0f;
            _worstFrame = 0f;
        }

        private void ApplyVisible()
        {
            if (_texts == null) return;
            foreach (var t in _texts)
                if (t != null) t.transform.parent.gameObject.SetActive(visible);
        }

        private void EnsureCanvases()
        {
            int count = Mathf.Max(1, displayCount);
            if (_texts != null && _texts.Length == count) return;

            _texts = new Text[count];
            for (int i = 0; i < count; i++) _texts[i] = BuildCanvas(i);
        }

        // 26pt text plus the 4px padding the text rect insets on each side.
        private static float PanelHeight(int lines) => 8f + 32f * Mathf.Max(1, lines);

        // One canvas per display; targetDisplay is 0-based, so index 0 is "Display 1".
        private Text BuildCanvas(int display)
        {
            var go = new GameObject("FpsOverlay.Canvas." + (display + 1));
            go.transform.SetParent(transform, false);
            var canvas = go.AddComponent<Canvas>();
            canvas.renderMode = RenderMode.ScreenSpaceOverlay;
            canvas.targetDisplay = display;
            // Just under DisplayDiagnosticsOverlay, above the operator/visitor UI.
            // Order at the top: BootOverlay > DisplayDiagnosticsOverlay > this.
            canvas.sortingOrder = short.MaxValue - 2;
            var scaler = go.AddComponent<CanvasScaler>();
            scaler.uiScaleMode = CanvasScaler.ScaleMode.ScaleWithScreenSize;
            scaler.referenceResolution = new Vector2(1920, 1080);
            scaler.matchWidthOrHeight = 0.5f;

            var panel = new GameObject("panel").AddComponent<RectTransform>();
            panel.SetParent(canvas.transform, false);
            panel.anchorMin = panel.anchorMax = panel.pivot = new Vector2(1f, 1f);
            panel.anchoredPosition = new Vector2(-16f, -16f);
            panel.sizeDelta = new Vector2(340f, PanelHeight(_lines));
            var bg = panel.gameObject.AddComponent<Image>();
            // Semi-opaque: readable over the point cloud without hiding it.
            bg.color = new Color(0f, 0f, 0f, 0.55f);
            bg.raycastTarget = false;

            var textGo = new GameObject("text");
            textGo.transform.SetParent(panel, false);
            var trt = textGo.AddComponent<RectTransform>();
            trt.anchorMin = Vector2.zero; trt.anchorMax = Vector2.one;
            trt.offsetMin = new Vector2(10f, 4f);
            trt.offsetMax = new Vector2(-10f, -4f);
            var text = textGo.AddComponent<Text>();
            text.font = Resources.GetBuiltinResource<Font>("LegacyRuntime.ttf");
            text.fontSize = 26;
            text.color = Color.white;
            text.alignment = TextAnchor.MiddleRight;
            text.horizontalOverflow = HorizontalWrapMode.Overflow;
            text.verticalOverflow = VerticalWrapMode.Overflow;
            text.raycastTarget = false;
            text.text = "-- fps";

            panel.gameObject.SetActive(visible);
            return text;
        }
    }
}
