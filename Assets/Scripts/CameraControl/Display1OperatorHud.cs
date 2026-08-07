// Operator HUD drawn with IMGUI so it lands on Display 1 — the otherwise-black
// operator display — in a standalone multi-display build (IMGUI renders to the
// primary display only). In the Editor it draws over the Game view.
//
// Controls (a runtime subset of Window > Control Panel — that panel is an
// EditorWindow and does not exist in a build, so the knobs recently added to it
// are mirrored here for the operator on the built app):
//   - History Samples slider: BonePoseHistory's motion-curve time window (same
//     knob as the Control Panel's "Motion history" tunable).
//   - Brightness slider: PointCloudView overall colour brightness.
//   - Web Curve Tol slider: TSDFPrintExporter web-export curve tolerance.
//   - Auto Orbit toggle: turns on the idle auto-orbit on every stage camera's
//     CameraOrbitController, keeping the controllers enabled regardless of the
//     pause state (via PauseOrbitGate.autoOrbitOverride on gated cameras).
//   - Experience Mode toggle + QR Dry Run toggle (ExperienceDirector).
//   - Save settings button: persists the tuning above to PlayerPrefs; reloaded
//     and applied on the next launch (Start).
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

        [Tooltip("Point cloud view whose Brightness slider is drawn. Auto-resolves the " +
                 "first PointCloudView when left empty.")]
        public PointCloudView pointCloudView;

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

        // Row heights (scaled GUI units) for the flowing widget column. The backdrop
        // box and the draw cursor both add these up, so keep them the single source.
        private const float SliderRowH = 48f;
        private const float ToggleRowH = 28f;
        private const float ButtonRowH = 32f;

        // Save-button persistence (PlayerPrefs). Built-app operators tune on Display 1
        // and press Save; the values reload on the next launch (Start applies them).
        private const string KeyBrightness = "FVUI.pc.brightness";
        private const string KeyCurveTol   = "FVUI.print.curveTol";
        private const string KeyQrDryRun   = "FVUI.exp.qrDryRun";
        private const string KeyHistory    = "FVUI.bt.historySamples";
        private string _saveStatus;

        /// <summary>Panel height in scaled GUI units as last measured in OnGUI.
        /// Exposed so a check can tell whether the panel still fits the display.</summary>
        public float MeasuredHeight { get; private set; }

        private bool _autoOrbit;

        private void OnEnable()
        {
            if (orbits == null || orbits.Length == 0)
                orbits = FindObjectsByType<CameraOrbitController>(FindObjectsSortMode.None);
            if (exporter == null) exporter = FindFirstObjectByType<TSDFPrintExporter>();
            if (pointCloudView == null) pointCloudView = FindFirstObjectByType<PointCloudView>();
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

        // Load persisted tuning (Save button) once the scene's components exist.
        // Only values whose reload applies cleanly at Start are persisted: brightness
        // and history apply anytime; curve tolerance is read at export; QR dry-run is
        // read at ResultShow, always after Start.
        private void Start() => LoadSettings();

        private void LoadSettings()
        {
            if (pointCloudView == null) pointCloudView = FindFirstObjectByType<PointCloudView>();
            if (exporter == null) exporter = FindFirstObjectByType<TSDFPrintExporter>();
            if (experienceDirector == null)
                experienceDirector = FindFirstObjectByType<Experience.ExperienceDirector>();
            if (history == null) history = FindFirstObjectByType<BonePoseHistory>();

            if (pointCloudView != null && PlayerPrefs.HasKey(KeyBrightness))
                pointCloudView.SetTunableValue(0, PlayerPrefs.GetFloat(KeyBrightness));
            if (exporter != null && PlayerPrefs.HasKey(KeyCurveTol))
                exporter.SetTunableValue(0, PlayerPrefs.GetFloat(KeyCurveTol));
            if (experienceDirector != null && PlayerPrefs.HasKey(KeyQrDryRun))
                experienceDirector.DryRunPublish = PlayerPrefs.GetInt(KeyQrDryRun) == 1;
            if (history != null && PlayerPrefs.HasKey(KeyHistory))
                history.SetTunableValue(0, PlayerPrefs.GetFloat(KeyHistory));
        }

        private void SaveSettings()
        {
            if (pointCloudView != null) PlayerPrefs.SetFloat(KeyBrightness, pointCloudView.TunableValue(0));
            if (exporter != null) PlayerPrefs.SetFloat(KeyCurveTol, exporter.TunableValue(0));
            if (experienceDirector != null)
                PlayerPrefs.SetInt(KeyQrDryRun, experienceDirector.DryRunPublish ? 1 : 0);
            if (history != null) PlayerPrefs.SetFloat(KeyHistory, history.TunableValue(0));
            PlayerPrefs.Save();
            _saveStatus = "Saved ✓";
        }

        // One tuning slider (label line + rail + thumb) at the running cursor y.
        // Returns the cursor advanced past the row so callers just chain the result.
        private float SliderRow(float y, float sliderWidth, string label,
                                float value, float min, float max, System.Action<float> set)
        {
            GUI.Label(new Rect(position.x, y, sliderWidth, 22), label);
            // Explicit track: the default skin's rail is nearly invisible on black.
            GUI.color = new Color(1f, 1f, 1f, 0.35f);
            GUI.DrawTexture(new Rect(position.x, y + 24 + 8, sliderWidth, 4), Texture2D.whiteTexture);
            GUI.color = Color.white;
            float v = GUI.HorizontalSlider(new Rect(position.x, y + 24, sliderWidth, 20), value, min, max);
            if (!Mathf.Approximately(v, value)) set(v);
            return y + SliderRowH;
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

            string sheet = BuildCheatSheet();
            var sheetContent = new GUIContent(sheet);
            // Measured, not a per-line guess: the skin's line height is not knowable
            // outside OnGUI, and a wrong guess clipped the last row off the panel.
            float sheetHeight = GUI.skin.label.CalcHeight(sheetContent, width);

            // Widget-column height = sum of present rows (same constants the draw
            // cursor advances by below), so the backdrop wraps exactly the widgets.
            float widgetsH = SliderRowH;                            // History
            if (pointCloudView != null) widgetsH += SliderRowH;     // Brightness
            if (exporter != null)       widgetsH += SliderRowH;     // Web curve tolerance
            widgetsH += ToggleRowH;                                 // Auto Orbit
            if (experienceDirector != null) widgetsH += 2 * ToggleRowH; // Experience Mode + QR Dry Run
            if (exporter != null)       widgetsH += ButtonRowH;     // Export
            widgetsH += ButtonRowH;                                 // Save

            float sheetTop = position.y + widgetsH + 10f;
            MeasuredHeight = sheetTop - position.y + sheetHeight + 34f;

            // Panel backdrop so the widgets read against the black operator display.
            GUI.Box(new Rect(position.x - 12, position.y - 10, width + 24, MeasuredHeight),
                    GUIContent.none);

            // width is shared with the rest of the HUD, but the sliders should not
            // stretch across the operator display. GUI coordinates are pre-scale,
            // so convert half of the physical screen width back into GUI units.
            float sliderWidth = Mathf.Min(width, Screen.width * 0.5f / uiScale);
            float y = position.y;

            // History Samples (frames) — same knob as the Control Panel.
            y = SliderRow(y, sliderWidth,
                $"{history.TunableName(0)}: {Mathf.RoundToInt(history.TunableValue(0))}",
                history.TunableValue(0), history.TunableMin(0), history.TunableMax(0),
                nv => history.SetTunableValue(0, Mathf.RoundToInt(nv)));

            // Point cloud brightness (39544fc).
            if (pointCloudView != null)
                y = SliderRow(y, sliderWidth,
                    $"Brightness: {pointCloudView.TunableValue(0):0.00}",
                    pointCloudView.TunableValue(0), pointCloudView.TunableMin(0), pointCloudView.TunableMax(0),
                    nv => pointCloudView.SetTunableValue(0, nv));

            // Print web-export curve tolerance (1e05bc1).
            if (exporter != null)
                y = SliderRow(y, sliderWidth,
                    $"Web Curve Tol: {exporter.TunableValue(0):0.0000} m",
                    exporter.TunableValue(0), exporter.TunableMin(0), exporter.TunableMax(0),
                    nv => exporter.SetTunableValue(0, nv));

            bool auto = GUI.Toggle(new Rect(position.x, y, width, 24),
                                   _autoOrbit, " Auto Orbit (stage cams)");
            if (auto != _autoOrbit) ApplyAutoOrbit(auto);
            y += ToggleRowH;

            if (experienceDirector != null)
            {
                bool exp = GUI.Toggle(new Rect(position.x, y, width, 24),
                                      experienceDirector.Visible,
                                      $" Experience Mode ({experienceDirector.CurrentState})");
                if (exp != experienceDirector.Visible) experienceDirector.Visible = exp;
                y += ToggleRowH;

                // QR Dry Run (a188523): ON = fake URL, OFF = real LFKS upload.
                bool dry = experienceDirector.DryRunPublish;
                bool ndry = GUI.Toggle(new Rect(position.x, y, width, 24), dry,
                    dry ? " QR Dry Run: ON (fake URL)" : " QR Dry Run: OFF (real upload)");
                if (ndry != dry) experienceDirector.DryRunPublish = ndry;
                y += ToggleRowH;
            }

            if (exporter != null)
            {
                if (GUI.Button(new Rect(position.x, y, 190, 26), "Export GLB + USDZ"))
                    exporter.ExportWeb();
                GUI.Label(new Rect(position.x + 198, y + 2, width - 198, 44), exporter.StatusText);
                y += ButtonRowH;
            }

            if (GUI.Button(new Rect(position.x, y, 190, 26), "Save settings"))
                SaveSettings();
            if (!string.IsNullOrEmpty(_saveStatus))
                GUI.Label(new Rect(position.x + 198, y + 2, width - 198, 22), _saveStatus);
            y += ButtonRowH;

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
