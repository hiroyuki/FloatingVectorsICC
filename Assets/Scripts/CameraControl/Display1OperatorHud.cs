// Operator HUD drawn with IMGUI so it lands on Display 1 — the otherwise-black
// operator display — in a standalone multi-display build (IMGUI renders to the
// primary display only). In the Editor it draws over the Game view.
//
// Controls:
//   - History Samples slider: BonePoseHistory's motion-curve time window (same
//     knob as the Control Panel's "Motion history" tunable).
//   - Auto Orbit toggle: turns on the idle auto-orbit on every stage camera's
//     CameraOrbitController, keeping the controllers enabled regardless of the
//     pause state (via PauseOrbitGate.autoOrbitOverride on gated cameras).
//
// Lives in the CameraControl folder (Assembly-CSharp): it needs both
// BodyTracking (BonePoseHistory) and the orbit controllers, and only
// Assembly-CSharp can see both.

using BodyTracking;
using PointCloud;
using TSDF;
using UnityEngine;

namespace CameraControl
{
    public class Display1OperatorHud : MonoBehaviour
    {
        [Tooltip("Bone pose history whose History Samples value the slider drives. " +
                 "Auto-resolves the first BonePoseHistory when left empty.")]
        public BonePoseHistory history;

        [Tooltip("Show/hide the HUD.")]
        public bool visible = true;

        [Tooltip("Top-left corner of the HUD in scaled GUI pixels.")]
        public Vector2 position = new Vector2(24, 24);

        [Tooltip("Slider width in scaled GUI pixels.")]
        public float width = 420f;

        [Tooltip("Overall HUD scale (IMGUI's default skin is small on 4K displays).")]
        [Range(1f, 4f)] public float uiScale = 2f;

        [Tooltip("Orbit controllers the Auto Orbit toggle drives. Auto-resolves every " +
                 "CameraOrbitController in the scene (including disabled ones) when empty.")]
        public CameraOrbitController[] orbits;

        [Tooltip("Web/AR exporter driven by the Export button (writes .glb + .usdz to " +
                 "~/Documents/FloatingVectorsPrints). Auto-resolves when left empty. The " +
                 "export is synchronous and takes a few seconds — freeze (Space) first so " +
                 "the hitch doesn't show on the stage displays.")]
        public TSDFPrintExporter exporter;

        [Tooltip("4-camera tile overlay the view-switch key swaps in for the HUD. " +
                 "Auto-resolves the first MultiCameraDebugView when left empty.")]
        public MultiCameraDebugView debugView;

        [Tooltip("Experience-flow director driven by the Experience Mode checkbox. " +
                 "Auto-resolves when left empty.")]
        public Experience.ExperienceDirector experienceDirector;

        [Tooltip("Operator publisher (F10/F11/F12) whose live values the hotkey cheat " +
                 "sheet shows. Auto-resolves when left empty.")]
        public Experience.OperatorPublisher publisher;

        [Tooltip("Recorder whose countdown/auto-stop seconds the cheat sheet shows. " +
                 "Auto-resolves when left empty.")]
        public SensorRecorder recorder;

        [Tooltip("Key that switches the operator display between the HUD and the " +
                 "4-camera tiles (drives debugView.Visible; the HUD mirrors off it).")]
        public KeyCode viewSwitchKey = KeyCode.Tab;

        [Tooltip("Keep HUD and 4-camera tiles mutually exclusive: the HUD hides itself " +
                 "whenever the tiles are visible (whichever key toggled them).")]
        public bool exclusiveWithDebugView = true;

        // Where the cheat sheet starts, below the widgets (slider / toggles / export).
        private const float SheetTopOffset = 150f;

        /// <summary>Panel height in scaled GUI units as last measured in OnGUI.
        /// Exposed so a check can tell whether the panel still fits the display.</summary>
        public float MeasuredHeight { get; private set; }

        private bool _autoOrbit;

        private void OnEnable()
        {
            if (orbits == null || orbits.Length == 0)
                orbits = FindObjectsByType<CameraOrbitController>(FindObjectsSortMode.None);
            if (exporter == null) exporter = FindFirstObjectByType<TSDFPrintExporter>();
            if (debugView == null) debugView = FindFirstObjectByType<MultiCameraDebugView>();
            if (experienceDirector == null)
                experienceDirector = FindFirstObjectByType<Experience.ExperienceDirector>();
            if (publisher == null) publisher = FindFirstObjectByType<Experience.OperatorPublisher>();
            if (recorder == null) recorder = FindFirstObjectByType<SensorRecorder>();
            // Reflect an override that is already on (e.g. HUD re-enabled mid-session).
            foreach (var o in orbits)
                if (o != null && o.TryGetComponent(out PauseOrbitGate g) && g.autoOrbitOverride)
                    _autoOrbit = true;
        }

        private void Update()
        {
            // HUD <-> 4-camera tiles. debugView.Visible is the single source of
            // truth; mirroring every frame keeps the pair exclusive no matter
            // which key toggled the tiles (this one or the view's own).
            if (debugView != null)
            {
                if (Input.GetKeyDown(viewSwitchKey)) debugView.Visible = !debugView.Visible;
                if (exclusiveWithDebugView) visible = !debugView.Visible;
            }
            else if (Input.GetKeyDown(viewSwitchKey))
            {
                visible = !visible;
            }
        }

        private void OnGUI()
        {
            if (!visible) return;
            // A full-screen alert or floor tune owns the display — stand down.
            if (Shared.OperatorOverlayGate.Suppressed) return;
            if (history == null)
            {
                history = FindFirstObjectByType<BonePoseHistory>();
                if (history == null) return;
            }

            GUI.matrix = Matrix4x4.Scale(Vector3.one * uiScale);

            // Cheat sheet first: the backdrop has to be drawn before the widgets, and
            // its height comes from measuring the sheet, so the text is built up here.
            string sheet = BuildCheatSheet();
            var sheetContent = new GUIContent(sheet);
            float sheetTop = position.y + SheetTopOffset;
            // Measured, not a per-line guess: the skin's line height is not knowable
            // outside OnGUI, and a wrong guess clipped the last row off the panel.
            float sheetHeight = GUI.skin.label.CalcHeight(sheetContent, width);
            MeasuredHeight = sheetTop - position.y + sheetHeight + 34f;

            // Panel backdrop so the widgets read against the black operator display.
            GUI.Box(new Rect(position.x - 12, position.y - 10, width + 24, MeasuredHeight),
                    GUIContent.none);

            // Tunable 0 is History Samples (frames) — same knob as the Control Panel.
            float min = history.TunableMin(0);
            float max = history.TunableMax(0);
            int cur = Mathf.RoundToInt(history.TunableValue(0));
            // width is shared with the rest of the HUD, but the slider should not
            // stretch across the operator display. GUI coordinates are pre-scale,
            // so convert half of the physical screen width back into GUI units.
            float sliderWidth = Mathf.Min(width, Screen.width * 0.5f / uiScale);

            GUI.Label(new Rect(position.x, position.y, sliderWidth, 22),
                      $"{history.TunableName(0)}: {cur}");

            // Explicit track: the default skin's slider rail is nearly invisible on a
            // black background — only the thumb showed.
            GUI.color = new Color(1f, 1f, 1f, 0.35f);
            GUI.DrawTexture(new Rect(position.x, position.y + 24 + 8, sliderWidth, 4), Texture2D.whiteTexture);
            GUI.color = Color.white;

            float v = GUI.HorizontalSlider(
                new Rect(position.x, position.y + 24, sliderWidth, 20), cur, min, max);
            int next = Mathf.RoundToInt(v);
            if (next != cur) history.SetTunableValue(0, next);

            bool auto = GUI.Toggle(new Rect(position.x, position.y + 52, width, 24),
                                   _autoOrbit, " Auto Orbit (stage cams)");
            if (auto != _autoOrbit) ApplyAutoOrbit(auto);

            if (exporter != null)
            {
                if (GUI.Button(new Rect(position.x, position.y + 82, 190, 26), "Export GLB + USDZ"))
                    exporter.ExportWeb();
                GUI.Label(new Rect(position.x + 198, position.y + 84, width - 198, 44),
                          exporter.StatusText);
            }

            if (experienceDirector != null)
            {
                bool exp = GUI.Toggle(new Rect(position.x, position.y + 112, width, 24),
                                      experienceDirector.Visible,
                                      $" Experience Mode ({experienceDirector.CurrentState})");
                if (exp != experienceDirector.Visible) experienceDirector.Visible = exp;
            }

            GUI.color = new Color(1f, 1f, 1f, 0.35f);
            GUI.DrawTexture(new Rect(position.x, sheetTop - 8f, width, 2), Texture2D.whiteTexture);
            GUI.color = Color.white;
            GUI.Label(new Rect(position.x, sheetTop, width, sheetHeight), sheetContent);
            if (publisher != null && !string.IsNullOrEmpty(publisher.StatusText))
                GUI.Label(new Rect(position.x, sheetTop + sheetHeight + 4f, width, 22),
                          publisher.StatusText.Length > 60
                              ? publisher.StatusText.Substring(0, 60) + "…"
                              : publisher.StatusText);

            GUI.matrix = Matrix4x4.identity;
        }

        // Hotkey cheat sheet with the live values, grouped by what the operator is
        // doing: run the session, publish the result, then everything that only
        // changes what the operator display itself shows.
        private string BuildCheatSheet()
        {
            float cd = recorder != null ? recorder.liveFreezeCountdownSeconds : 5f;
            float stop = recorder != null ? recorder.stopRecAfterFreezeSeconds : 0f;
            int trail = publisher != null ? publisher.trailSamples : 0;
            string recStop = stop > 0f ? $" (REC auto-stops {stop:0}s later)" : "";
            string busy = publisher != null && publisher.IsBusy ? "  [PUBLISHING…]" : "";
            // The tiles have their own toggle key besides this HUD's; show both, since
            // either one gets the operator back from a screen full of camera tiles.
            string tiles = debugView != null
                ? $"{viewSwitchKey} / {debugView.toggleKey}"
                : viewSwitchKey.ToString();

            return "SESSION\n" +
                   $"  F9         REC start / stop\n" +
                   $"  Space      freeze in {cd:0}s{recStop}\n" +
                   $"  <- / ->    step frame (paused)\n" +
                   "PUBLISH\n" +
                   $"  F10        export -> upload -> QR on disp 2/3 (trail {trail}){busy}\n" +
                   "  F11        hide QR\n" +
                   "  F12        toggle: export preview <-> live\n" +
                   "OPERATOR DISPLAY\n" +
                   $"  {tiles,-9}  HUD <-> 4-cam tiles\n" +
                   "  F2         FPS readout (all displays)\n" +
                   "  F4         display diagnostics   (F3 moves it)\n" +
                   "  F1         calibration mode\n" +
                   "  Esc hold   quit (build only)\n" +
                   "EXPERIENCE (dev jump — mode auto-ON)\n" +
                   "  0          頭出し (run reset -> Idle)\n" +
                   "  1 2 3 4 5  Consent / Welcome / Calibrate / TestMove1 / TestMove2\n" +
                   "  6 7 8 9    Shoot / Processing / ResultShow / QrShow\n" +
                   "TSDF DEBUG\n" +
                   "  Q W E R    single-cam view (cam 1-4)\n" +
                   "  A          all cams";
        }

        // ON: every stage camera's orbit controller runs its idle auto-orbit,
        // regardless of pause (gated cameras get autoOrbitOverride, ungated ones are
        // enabled directly). OFF: back to the default policy — gated cameras follow
        // the pause state, ungated ones go back to disabled.
        private void ApplyAutoOrbit(bool on)
        {
            _autoOrbit = on;
            foreach (var o in orbits)
            {
                if (o == null) continue;
                o.autoOrbit = on;
                if (o.TryGetComponent(out PauseOrbitGate gate)) gate.autoOrbitOverride = on;
                else o.enabled = on;
            }
        }
    }
}
