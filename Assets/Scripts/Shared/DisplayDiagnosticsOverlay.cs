// F4: dump what the RUNNING process actually thinks the displays are.
//
// Why this exists: the Editor's Game-view "Display 2/3" tabs are just tabs, and
// a standalone build maps Display.displays[i] by OS enumeration order instead —
// so a display assignment verified in the Editor can land on a different physical
// screen in the build, and a Canvas can end up on a display that is not active or
// has no camera. None of that is visible from the Editor, and a fullscreen build
// has nowhere to print it.
//
// Read it as: does displays[i].active match what you expect, is there exactly one
// enabled camera per display, and is each Canvas on the display you meant.
//
// Drawn as a uGUI Canvas at the maximum sorting order, NOT as IMGUI: IMGUI renders
// underneath every ScreenSpaceOverlay canvas, so the operator UI covered it.
//
// Self-installing so it is available in any build without scene wiring.

using System.Text;
using UnityEngine;
using UnityEngine.UI;

namespace Shared
{
    [DisallowMultipleComponent]
    public class DisplayDiagnosticsOverlay : MonoBehaviour
    {
        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
        private static void Install()
        {
            if (FindFirstObjectByType<DisplayDiagnosticsOverlay>() != null) return;
            var go = new GameObject("[DisplayDiagnostics]");
            go.AddComponent<DisplayDiagnosticsOverlay>();
            DontDestroyOnLoad(go);
        }

        [Tooltip("Toggle the dump.")]
        public KeyCode toggleKey = KeyCode.F4;

        [Tooltip("Cycle which display the dump is drawn on. Use it when the operator " +
                 "display is not the one you are standing in front of.")]
        public KeyCode cycleDisplayKey = KeyCode.F3;

        [Tooltip("Display index the dump is drawn on. F3 cycles it at runtime.")]
        public int targetDisplay = 0;

        [Tooltip("Also write the dump to the log each time it is toggled on, so it " +
                 "survives into Player.log for after-the-fact reading.")]
        public bool logOnToggle = true;

        private bool _show;
        private float _lastBuild;
        private Canvas _canvas;
        private Text _text;

        private void Update()
        {
            if (Input.GetKeyDown(cycleDisplayKey))
            {
                int n = Mathf.Max(1, Display.displays.Length);
                targetDisplay = (targetDisplay + 1) % n;
                if (_canvas != null) _canvas.targetDisplay = targetDisplay;
                if (_show) Refresh();
            }

            if (Input.GetKeyDown(toggleKey))
            {
                _show = !_show;
                if (_show)
                {
                    EnsureCanvas();
                    Refresh();
                    if (logOnToggle) Debug.Log("[DisplayDiagnostics]\n" + _text.text);
                }
                if (_canvas != null) _canvas.gameObject.SetActive(_show);
            }

            // Cheap auto-refresh: cameras get enabled/disabled by mode changes.
            if (_show && Time.realtimeSinceStartup - _lastBuild > 1f) Refresh();
        }

        private void EnsureCanvas()
        {
            if (_canvas != null) return;

            var go = new GameObject("DisplayDiagnostics.Canvas");
            go.transform.SetParent(transform, false);
            _canvas = go.AddComponent<Canvas>();
            _canvas.renderMode = RenderMode.ScreenSpaceOverlay;
            _canvas.targetDisplay = targetDisplay;
            // Above every other overlay canvas except the boot / shutdown splash,
            // which owns the screen outright while it is up (see BootOverlay).
            _canvas.sortingOrder = short.MaxValue - 1;
            var scaler = go.AddComponent<CanvasScaler>();
            scaler.uiScaleMode = CanvasScaler.ScaleMode.ScaleWithScreenSize;
            scaler.referenceResolution = new Vector2(1920, 1080);

            var panel = new GameObject("panel").AddComponent<RectTransform>();
            panel.SetParent(_canvas.transform, false);
            panel.anchorMin = new Vector2(0f, 0f);
            panel.anchorMax = new Vector2(1f, 1f);
            panel.offsetMin = new Vector2(40f, 40f);
            panel.offsetMax = new Vector2(-40f, -40f);
            var bg = panel.gameObject.AddComponent<Image>();
            bg.color = Color.black;   // fully opaque: the scene behind it made the text unreadable

            var textGo = new GameObject("text");
            textGo.transform.SetParent(panel, false);
            var trt = textGo.AddComponent<RectTransform>();
            trt.anchorMin = Vector2.zero; trt.anchorMax = Vector2.one;
            trt.offsetMin = new Vector2(24f, 24f);
            trt.offsetMax = new Vector2(-24f, -24f);
            _text = textGo.AddComponent<Text>();
            _text.font = Resources.GetBuiltinResource<Font>("LegacyRuntime.ttf");
            _text.fontSize = 22;
            _text.color = Color.white;
            _text.alignment = TextAnchor.UpperLeft;
            _text.horizontalOverflow = HorizontalWrapMode.Overflow;
            _text.verticalOverflow = VerticalWrapMode.Overflow;
        }

        private void Refresh()
        {
            _lastBuild = Time.realtimeSinceStartup;
            if (_text == null) return;

            var sb = new StringBuilder();
            sb.Append("[F4] display diagnostics   (F3 = move this panel to the next display)\n\n");

            sb.Append("Display.displays.Length = ").Append(Display.displays.Length).Append('\n');
            for (int i = 0; i < Display.displays.Length; i++)
            {
                var d = Display.displays[i];
                sb.Append("  [").Append(i).Append("] \"Display ").Append(i + 1).Append('"')
                  .Append(d.active ? "  ACTIVE  " : "  inactive")
                  .Append("  rendering ").Append(d.renderingWidth).Append('x').Append(d.renderingHeight)
                  .Append("  system ").Append(d.systemWidth).Append('x').Append(d.systemHeight)
                  .Append(i == targetDisplay ? "   <-- this panel" : "")
                  .Append('\n');
            }

            sb.Append("\nCameras (targetDisplay is 0-based):\n");
            foreach (var cam in FindObjectsByType<Camera>(FindObjectsSortMode.None))
            {
                sb.Append("  disp ").Append(cam.targetDisplay)
                  .Append(cam.enabled && cam.gameObject.activeInHierarchy ? "  ON   " : "  off  ")
                  .Append(cam.name)
                  .Append("  depth ").Append(cam.depth);
                if (cam.targetTexture != null) sb.Append("  -> RenderTexture");
                sb.Append('\n');
            }

            sb.Append("\nCanvases:\n");
            foreach (var c in FindObjectsByType<Canvas>(FindObjectsSortMode.None))
            {
                if (!c.isRootCanvas) continue;
                int disp = c.renderMode == RenderMode.ScreenSpaceOverlay
                    ? c.targetDisplay
                    : (c.worldCamera != null ? c.worldCamera.targetDisplay : -1);
                sb.Append("  disp ").Append(disp)
                  .Append(c.enabled && c.gameObject.activeInHierarchy ? "  ON   " : "  off  ")
                  .Append(c.name).Append("  ").Append(c.renderMode).Append('\n');
            }

            sb.Append("\nScreen: ").Append(Screen.width).Append('x').Append(Screen.height)
              .Append("  fullScreenMode ").Append(Screen.fullScreenMode)
              .Append("\nGraphics: ").Append(SystemInfo.graphicsDeviceType)
              .Append("  ").Append(SystemInfo.graphicsDeviceName);

            _text.text = sb.ToString();
        }
    }
}
