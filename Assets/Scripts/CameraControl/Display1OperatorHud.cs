// Operator HUD drawn with IMGUI so it lands on Display 1 — the otherwise-black
// operator display — in a standalone multi-display build (IMGUI renders to the
// primary display only). In the Editor it draws over the Game view.
//
// Controls (a runtime subset of Window > Control Panel — that panel is an
// EditorWindow and does not exist in a build, so the knobs recently added to it
// are mirrored here for the operator on the built app):
//   - History (live) slider: BonePoseHistory's motion-curve window at runtime.
//   - History (print) slider: OperatorPublisher.trailSamples — the window the
//     3D-data export temporarily overrides the live one with before baking.
//   - Brightness slider: PointCloudView overall colour brightness.
//   - Decimate slider: PointCloudDecimater reduction percent.
//   - Seed Count slider: PointCloudMotionCurves seed budget.
//   - Web Curve Tol slider: TSDFPrintExporter web-export curve tolerance.
//   - Auto Orbit toggle: turns on the idle auto-orbit on every stage camera's
//     CameraOrbitController, keeping the controllers enabled regardless of the
//     pause state (via PauseOrbitGate.autoOrbitOverride on gated cameras).
//   - Experience Mode toggle (ExperienceDirector).
//   - Keep recordings toggle: when ON, ExperienceDirector keeps the raw visitor
//     takes on disk instead of deleting them at the QR / next-run boundary.
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

        [Tooltip("Point cloud decimater whose Decimate (%) slider is drawn. " +
                 "Auto-resolves the first PointCloudDecimater when left empty.")]
        public PointCloudDecimater decimater;

        [Tooltip("Motion curves whose Seed Count slider is drawn. Auto-resolves the " +
                 "first PointCloudMotionCurves when left empty.")]
        public BodyTracking.PointCloudMotionCurves motionCurves;

        [Tooltip("Mirror component on the Display 3 camera the 'Flip Display 3 (X)' " +
                 "toggle drives. Auto-resolves the camera whose targetDisplay is 2 " +
                 "(Display 3) and adds a MirrorDisplayCamera if it has none.")]
        public MirrorDisplayCamera display3Mirror;

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
        private const string KeyBrightness   = "FVUI.pc.brightness";
        private const string KeyCurveTol     = "FVUI.print.curveTol";
        private const string KeyHistoryLive  = "FVUI.bt.historySamples";
        private const string KeyHistoryPrint = "FVUI.pub.trailSamples";
        private const string KeyDecimate     = "FVUI.pc.decimate";
        private const string KeySeedCount    = "FVUI.mc.seedCount";
        private string _saveStatus;

        // Seed Count is index 8 of PointCloudMotionCurves' IPanelTunable list.
        private const int SeedCountTunable = 8;

        // Display 3 is Unity targetDisplay index 2 (0-based). The flip toggle drives
        // the MirrorDisplayCamera on whichever camera renders there.
        private const int Display3Index = 2;
        private const string KeyFlipDisplay3 = "FVUI.disp3.flipX";

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
            if (decimater == null) decimater = FindFirstObjectByType<PointCloudDecimater>();
            if (motionCurves == null) motionCurves = FindFirstObjectByType<BodyTracking.PointCloudMotionCurves>();
            if (display3Mirror == null) display3Mirror = ResolveDisplay3Mirror();
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
        // Every persisted value reloads cleanly at Start: brightness / decimate /
        // seed count / both history windows apply anytime; curve tolerance and the
        // print history window are read at export, always after Start.
        private void Start() => LoadSettings();

        private void LoadSettings()
        {
            if (pointCloudView == null) pointCloudView = FindFirstObjectByType<PointCloudView>();
            if (exporter == null) exporter = FindFirstObjectByType<TSDFPrintExporter>();
            if (history == null) history = FindFirstObjectByType<BonePoseHistory>();
            if (publisher == null) publisher = FindFirstObjectByType<Experience.OperatorPublisher>();
            if (decimater == null) decimater = FindFirstObjectByType<PointCloudDecimater>();
            if (motionCurves == null) motionCurves = FindFirstObjectByType<BodyTracking.PointCloudMotionCurves>();

            if (pointCloudView != null && PlayerPrefs.HasKey(KeyBrightness))
                pointCloudView.SetTunableValue(0, PlayerPrefs.GetFloat(KeyBrightness));
            if (exporter != null && PlayerPrefs.HasKey(KeyCurveTol))
                exporter.SetTunableValue(0, PlayerPrefs.GetFloat(KeyCurveTol));
            if (history != null && PlayerPrefs.HasKey(KeyHistoryLive))
                history.SetTunableValue(0, PlayerPrefs.GetFloat(KeyHistoryLive));
            if (publisher != null && PlayerPrefs.HasKey(KeyHistoryPrint))
                publisher.trailSamples = Mathf.Clamp(PlayerPrefs.GetInt(KeyHistoryPrint), 2, 32);
            if (decimater != null && PlayerPrefs.HasKey(KeyDecimate))
                decimater.SetTunableValue(0, PlayerPrefs.GetFloat(KeyDecimate));
            if (motionCurves != null && PlayerPrefs.HasKey(KeySeedCount))
                motionCurves.SetTunableValue(SeedCountTunable, PlayerPrefs.GetInt(KeySeedCount));
        }

        private void SaveSettings()
        {
            if (pointCloudView != null) PlayerPrefs.SetFloat(KeyBrightness, pointCloudView.TunableValue(0));
            if (exporter != null) PlayerPrefs.SetFloat(KeyCurveTol, exporter.TunableValue(0));
            if (history != null) PlayerPrefs.SetFloat(KeyHistoryLive, history.TunableValue(0));
            if (publisher != null) PlayerPrefs.SetInt(KeyHistoryPrint, publisher.trailSamples);
            if (decimater != null) PlayerPrefs.SetFloat(KeyDecimate, decimater.TunableValue(0));
            if (motionCurves != null)
                PlayerPrefs.SetInt(KeySeedCount, Mathf.RoundToInt(motionCurves.TunableValue(SeedCountTunable)));
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
            float widgetsH = SliderRowH;                            // History (live)
            if (publisher != null)      widgetsH += SliderRowH;     // History (print)
            if (pointCloudView != null) widgetsH += SliderRowH;     // Brightness
            if (decimater != null)      widgetsH += SliderRowH;     // Decimate
            if (motionCurves != null)   widgetsH += SliderRowH;     // Seed Count
            if (exporter != null)       widgetsH += SliderRowH;     // Web curve tolerance
            widgetsH += ToggleRowH;                                 // Auto Orbit
            if (experienceDirector != null) widgetsH += ToggleRowH; // Experience Mode
            if (experienceDirector != null) widgetsH += ToggleRowH; // Keep recordings
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

            // History Samples — LIVE window (BonePoseHistory), the normal runtime knob.
            y = SliderRow(y, sliderWidth,
                $"History (live): {Mathf.RoundToInt(history.TunableValue(0))}",
                history.TunableValue(0), history.TunableMin(0), history.TunableMax(0),
                nv => history.SetTunableValue(0, Mathf.RoundToInt(nv)));

            // History Samples — 3D-EXPORT window (OperatorPublisher.trailSamples). The
            // capture temporarily overrides the live window with this before baking.
            if (publisher != null)
                y = SliderRow(y, sliderWidth,
                    $"History (print): {publisher.trailSamples}",
                    publisher.trailSamples, 2f, 32f,
                    nv => publisher.trailSamples = Mathf.Clamp(Mathf.RoundToInt(nv), 2, 32));

            // Point cloud brightness (39544fc).
            if (pointCloudView != null)
                y = SliderRow(y, sliderWidth,
                    $"Brightness: {pointCloudView.TunableValue(0):0.00}",
                    pointCloudView.TunableValue(0), pointCloudView.TunableMin(0), pointCloudView.TunableMax(0),
                    nv => pointCloudView.SetTunableValue(0, nv));

            // Point cloud decimate (%).
            if (decimater != null)
                y = SliderRow(y, sliderWidth,
                    $"Decimate: {decimater.TunableValue(0):0} %",
                    decimater.TunableValue(0), decimater.TunableMin(0), decimater.TunableMax(0),
                    nv => decimater.SetTunableValue(0, nv));

            // Motion-curve seed count (PointCloudMotionCurves tunable 8).
            if (motionCurves != null)
                y = SliderRow(y, sliderWidth,
                    $"Seed Count: {Mathf.RoundToInt(motionCurves.TunableValue(SeedCountTunable))}",
                    motionCurves.TunableValue(SeedCountTunable),
                    motionCurves.TunableMin(SeedCountTunable), motionCurves.TunableMax(SeedCountTunable),
                    nv => motionCurves.SetTunableValue(SeedCountTunable, nv));

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

                // Keep the raw visitor takes on disk (default OFF = visitor data never
                // outlives the show). ON persists every take under visitorRecordingRoot
                // for later playback.
                bool keep = GUI.Toggle(new Rect(position.x, y, width, 24),
                                       experienceDirector.KeepRecordings,
                                       " Keep recordings (raw takes on disk)");
                if (keep != experienceDirector.KeepRecordings) experienceDirector.KeepRecordings = keep;
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
