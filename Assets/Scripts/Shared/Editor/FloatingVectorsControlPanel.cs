// THE one-stop operation panel: Window > Control Panel.
//
// Everything operable lives here so you never have to remember which
// GameObject carries which Inspector:
//   - Recorder: record / play / pause transport (IRecorderTransport).
//   - Views: every IViewToggle (point cloud / TSDF mesh / BT skeleton) as
//     checkboxes + All on/off.
//   - Accumulation: every IAccumulationController listed with a CHECKBOX, and
//     ONE master ● Start / ■ Stop pair that applies to the checked ones (per
//     user request — no per-row button hunting). Per-row Clear / extra
//     actions (e.g. the baker's Resume live) stay inline on each row.
// Each row has a Select button that pings the owning GameObject when you DO
// want the full Inspector (all fields remain there).
//
// Discovery is interface-based (Shared.*), so this panel needs no references
// to the concrete PointCloud/TSDF/BodyTracking assemblies and picks up any
// future implementer automatically.

using System.Collections.Generic;
using UnityEditor;
using UnityEngine;

namespace Shared.EditorTools
{
    public class FloatingVectorsControlPanel : EditorWindow
    {
        [MenuItem("Window/Control Panel")]
        public static void Open() => GetWindow<FloatingVectorsControlPanel>("Control Panel");

        private readonly List<IRecorderTransport> _transports = new List<IRecorderTransport>();
        private readonly List<IViewToggle> _views = new List<IViewToggle>();
        private readonly List<IAccumulationController> _accums = new List<IAccumulationController>();
        private readonly List<IPanelTunable> _tunables = new List<IPanelTunable>();
        private readonly List<IStartupActivatable> _activatables = new List<IStartupActivatable>();
        private GUIStyle _bigToggleStyle;
        private Vector2 _scroll;

        private void OnGUI()
        {
            // Re-scan every draw: components appear/disappear with scene loads and
            // play mode, and the counts are tiny (a handful of MonoBehaviours).
            _transports.Clear();
            _views.Clear();
            _accums.Clear();
            _tunables.Clear();
            _activatables.Clear();
            foreach (var mb in FindObjectsByType<MonoBehaviour>(FindObjectsInactive.Include, FindObjectsSortMode.None))
            {
                if (mb is IRecorderTransport rt) _transports.Add(rt);
                if (mb is IViewToggle vt) _views.Add(vt);
                if (mb is IAccumulationController ac) _accums.Add(ac);
                if (mb is IPanelTunable tn) _tunables.Add(tn);
                if (mb is IStartupActivatable sa) _activatables.Add(sa);
            }
            // FindObjectsSortMode.None gives a nondeterministic order that changes
            // every Play — sort ALL sections so rows never jump around.
            _transports.Sort((a, b) => string.CompareOrdinal(GoName(a), GoName(b)));
            _views.Sort((a, b) => string.CompareOrdinal(a.ViewLabel, b.ViewLabel));
            _accums.Sort((a, b) => string.CompareOrdinal(DisplayName(a), DisplayName(b)));
            _tunables.Sort((a, b) =>
            {
                int c = string.CompareOrdinal(a.TuningLabel, b.TuningLabel);
                return c != 0 ? c : string.CompareOrdinal(GoName(a), GoName(b));
            });
            _activatables.Sort((a, b) => string.CompareOrdinal(a.ActivateLabel, b.ActivateLabel));

            _scroll = EditorGUILayout.BeginScrollView(_scroll);

            DrawActivateOnPlaySection();
            EditorGUILayout.Space(12);
            DrawRecorderSection();
            EditorGUILayout.Space(12);
            DrawViewsSection();
            EditorGUILayout.Space(12);
            DrawAccumulationSection();
            EditorGUILayout.Space(12);
            DrawTuningSection();

            EditorGUILayout.EndScrollView();
        }

        // ---------------- Activate On Play ----------------
        // The exhibition arming switch (e.g. ExperienceDirector.activateOnPlay):
        // ON means the next Play boots straight into the experience with no
        // operator action. Deliberately LARGE — this is the one flag that decides
        // whether Play starts a show or a dev session, so it must be impossible
        // to miss when closing up the exhibition machine.
        private void DrawActivateOnPlaySection()
        {
            EditorGUILayout.LabelField("Activate On Play (起動時に自動開始)", EditorStyles.boldLabel);
            if (_activatables.Count == 0)
            {
                EditorGUILayout.HelpBox("No IStartupActivatable components in the open scene(s).", MessageType.Info);
                return;
            }

            if (_bigToggleStyle == null)
                _bigToggleStyle = new GUIStyle(GUI.skin.button)
                {
                    fontSize = 18,
                    fontStyle = FontStyle.Bold,
                    fixedHeight = 48,
                };

            foreach (var sa in _activatables)
            {
                var comp = sa as Component;
                bool on = sa.ActivateOnPlay;
                using (new EditorGUILayout.HorizontalScope())
                {
                    Color saved = GUI.backgroundColor;
                    GUI.backgroundColor = on ? new Color(0.35f, 1f, 0.45f) : saved;
                    bool now = GUILayout.Toggle(on,
                        (on ? "⦿ " : "○ ") + sa.ActivateLabel + " — Activate On Play: " + (on ? "ON" : "OFF"),
                        _bigToggleStyle);
                    GUI.backgroundColor = saved;
                    if (now != on)
                    {
                        if (comp != null) Undo.RecordObject(comp, "Activate On Play");
                        sa.ActivateOnPlay = now;
                        if (comp != null) EditorUtility.SetDirty(comp);
                    }
                    DrawSelectButton(comp);
                }
            }
            EditorGUILayout.HelpBox("ON = 次の Play で自動的に体験モードへ入る（展示用）。OFF = Dev セッション。", MessageType.None);
        }

        // ---------------- Recorder ----------------
        private void DrawRecorderSection()
        {
            EditorGUILayout.LabelField("Recorder (収録/再生)", EditorStyles.boldLabel);
            if (_transports.Count == 0)
            {
                EditorGUILayout.HelpBox("No IRecorderTransport components in the open scene(s).", MessageType.Info);
                return;
            }

            foreach (var t in _transports)
            {
                var comp = t as Component;
                using (new EditorGUILayout.HorizontalScope())
                {
                    EditorGUILayout.LabelField(comp != null ? comp.gameObject.name : t.ToString(),
                                               EditorStyles.miniBoldLabel);
                    DrawSelectButton(comp);
                }

                if (!string.IsNullOrEmpty(t.TransportStatus))
                    EditorGUILayout.HelpBox(t.TransportStatus,
                        t.IsRecording ? MessageType.Warning : MessageType.Info);

                // Recording folder (folderPath). Editing is locked while recording
                // so the destination can't change mid-take. Delayed field so Undo /
                // the setter fire once per commit, not per keystroke. Below it, the
                // resolved absolute path so the operator sees where Rec/Play land.
                using (new EditorGUI.DisabledScope(t.IsRecording))
                using (new EditorGUILayout.HorizontalScope())
                {
                    GUILayout.Label(new GUIContent("設定ディレクトリ",
                        "録画/再生のルートフォルダ（SensorRecorder.folderPath）。相対パスは " +
                        "persistentDataPath 起点。空欄で既定の recording フォルダ。"),
                        GUILayout.Width(150));
                    string folder = EditorGUILayout.DelayedTextField(t.RecordingFolder ?? string.Empty);
                    if (folder != (t.RecordingFolder ?? string.Empty))
                    {
                        if (comp != null) Undo.RecordObject(comp, "Set recording folder");
                        t.RecordingFolder = folder;
                        if (comp != null) EditorUtility.SetDirty(comp);
                    }
                }
                if (!string.IsNullOrEmpty(t.ResolvedRecordingFolder))
                    EditorGUILayout.LabelField(" ", t.ResolvedRecordingFolder, EditorStyles.miniLabel);

                // Startup mode selector (SensorManager.playbackOnly). Set BEFORE
                // pressing Play: the scene then starts in the chosen mode. Editable
                // only outside Play mode — mid-session use the runtime switch below.
                using (new EditorGUI.DisabledScope(Application.isPlaying))
                using (new EditorGUILayout.HorizontalScope())
                {
                    GUILayout.Label(new GUIContent("起動モード (Startup)",
                        "Play を押す前に選ぶ起動時のモード（SensorManager.playbackOnly）。" +
                        "PLAYBACK = 録画フォルダを再生 / LIVE = カメラ接続。Play 中は変更不可。"),
                        GUILayout.Width(150));
                    int cur = t.StartInPlaybackMode ? 1 : 0;
                    int sel = GUILayout.Toolbar(cur, new[] { "LIVE ⦿", "PLAYBACK ▶" }, GUILayout.Height(22));
                    if (sel != cur) t.StartInPlaybackMode = sel == 1;
                }

                // While playing, just show which mode is actually running. Mid-session
                // switching isn't needed, so no switch button here — the startup mode
                // above is the single point of control.
                if (Application.isPlaying)
                    EditorGUILayout.LabelField("Now",
                        t.IsLiveMode ? "LIVE (realtime)" : "PLAYBACK", EditorStyles.miniBoldLabel);

                using (new EditorGUI.DisabledScope(!Application.isPlaying))
                using (new EditorGUILayout.HorizontalScope())
                {
                    if (GUILayout.Button(t.IsRecording ? "■ Stop Rec" : "● Rec", GUILayout.Height(24)))
                        t.ToggleRecord();
                    if (GUILayout.Button(t.IsPlaying ? "■ Stop Play" : "▶ Play", GUILayout.Height(24)))
                        t.TogglePlay();
                    using (new EditorGUI.DisabledScope(!t.IsPlaying))
                    {
                        if (GUILayout.Button(t.IsPaused ? "Resume" : "Pause", GUILayout.Height(24)))
                            t.TogglePause();
                    }
                }
            }
        }

        // ---------------- Views ----------------
        private void DrawViewsSection()
        {
            EditorGUILayout.LabelField("Views (表示)", EditorStyles.boldLabel);
            if (_views.Count == 0)
            {
                EditorGUILayout.HelpBox("No IViewToggle components in the open scene(s).", MessageType.Info);
                return;
            }

            foreach (var vt in _views)
            {
                using (new EditorGUILayout.HorizontalScope())
                {
                    var obj = vt as Object;
                    bool now = EditorGUILayout.ToggleLeft(vt.ViewLabel, vt.Visible);
                    if (now != vt.Visible)
                    {
                        if (obj != null) Undo.RecordObject(obj, (now ? "Show " : "Hide ") + vt.ViewLabel);
                        vt.Visible = now;
                        if (obj != null) EditorUtility.SetDirty(obj);
                    }
                    DrawSelectButton(vt as Component);
                }
            }

            using (new EditorGUILayout.HorizontalScope())
            {
                if (GUILayout.Button("All on")) SetAllViews(true);
                if (GUILayout.Button("All off")) SetAllViews(false);
            }
        }

        private void SetAllViews(bool visible)
        {
            foreach (var vt in _views)
            {
                var obj = vt as Object;
                if (obj != null) Undo.RecordObject(obj, visible ? "Show all views" : "Hide all views");
                vt.Visible = visible;
                if (obj != null) EditorUtility.SetDirty(obj);
            }
        }

        // ---------------- Accumulation ----------------
        // One master Start/Stop pair; the checkboxes choose which accumulators it
        // drives. Checkbox state persists for the editor session (SessionState).
        private void DrawAccumulationSection()
        {
            EditorGUILayout.LabelField("Accumulation (累積)", EditorStyles.boldLabel);
            if (_accums.Count == 0)
            {
                EditorGUILayout.HelpBox("No IAccumulationController components in the open scene(s).", MessageType.Info);
                return;
            }

            foreach (var c in _accums)
            {
                var comp = c as Component;
                using (new EditorGUILayout.HorizontalScope())
                {
                    bool was = IsChecked(comp);
                    bool now = EditorGUILayout.ToggleLeft(DisplayName(c), was);
                    if (now != was) SetChecked(comp, now);

                    // Stamp interval in seconds (0 = every frame / every batch =
                    // continuous). Delayed field so Undo records once per commit,
                    // not per keystroke.
                    GUILayout.Label(new GUIContent("Interval(s)",
                        "累積スタンプの間隔（秒）。0 = 毎フレーム/毎バッチ（連続）。"),
                        GUILayout.Width(62));
                    float iv = EditorGUILayout.DelayedFloatField(c.IntervalSeconds, GUILayout.Width(44));
                    if (!Mathf.Approximately(iv, c.IntervalSeconds))
                    {
                        if (comp != null) Undo.RecordObject(comp, "Accumulation interval");
                        c.IntervalSeconds = iv;
                        if (comp != null) EditorUtility.SetDirty(comp);
                    }

                    // Per-row auxiliary actions stay inline (they are component-specific).
                    if (c.CanClear && GUILayout.Button(c.ClearLabel, GUILayout.Width(150)))
                    {
                        // FullHierarchy covers the heaviest case (snapshot child GOs being
                        // destroyed); a superset of RecordObject for the rest, harmless.
                        if (comp != null) Undo.RegisterFullObjectHierarchyUndo(comp.gameObject, c.ClearLabel);
                        c.ClearAccumulated();
                        if (comp != null) EditorUtility.SetDirty(comp);
                    }
                    if (c is IAccumulationExtraActions ex)
                    {
                        using (new EditorGUI.DisabledScope(!Application.isPlaying))
                        {
                            for (int i = 0; i < ex.ExtraActionCount; i++)
                                if (GUILayout.Button(ex.ExtraActionLabel(i), GUILayout.Width(150)))
                                    ex.RunExtraAction(i);
                        }
                    }
                    DrawSelectButton(comp);
                }

                if (!string.IsNullOrEmpty(c.StatusText))
                    EditorGUILayout.HelpBox(c.StatusText, c.IsAccumulating ? MessageType.Warning : MessageType.Info);
            }

            EditorGUILayout.Space(4);
            using (new EditorGUI.DisabledScope(!Application.isPlaying))
            using (new EditorGUILayout.HorizontalScope())
            {
                if (GUILayout.Button("● Start (checked)", GUILayout.Height(26)))
                    foreach (var c in _accums)
                        if (IsChecked(c as Component) && c.CanStart) c.StartAccumulate();

                if (GUILayout.Button("■ Stop (checked)", GUILayout.Height(26)))
                    foreach (var c in _accums)
                        if (IsChecked(c as Component) && c.CanStop) c.StopAccumulate();
            }
            if (!Application.isPlaying)
                EditorGUILayout.HelpBox("Start/Stop はプレイ中のみ（チェックした累積に一括適用）。", MessageType.None);
        }

        // Session-persistent checkbox per component (survives domain reloads;
        // resets on editor restart). Default: checked.
        private static string CheckKey(Component comp) => "FVCP.accum." + (comp != null ? comp.GetInstanceID() : 0);
        private static bool IsChecked(Component comp) => SessionState.GetBool(CheckKey(comp), true);
        private static void SetChecked(Component comp, bool v) => SessionState.SetBool(CheckKey(comp), v);

        // ---------------- Tuning ----------------
        private void DrawTuningSection()
        {
            EditorGUILayout.LabelField("Tuning (調整)", EditorStyles.boldLabel);
            if (_tunables.Count == 0)
            {
                EditorGUILayout.HelpBox("No IPanelTunable components in the open scene(s).", MessageType.Info);
                return;
            }

            foreach (var tn in _tunables)
            {
                var comp = tn as Component;
                using (new EditorGUILayout.HorizontalScope())
                {
                    EditorGUILayout.LabelField(tn.TuningLabel, EditorStyles.miniBoldLabel);
                    DrawSelectButton(comp);
                }

                for (int i = 0; i < tn.TunableCount; i++)
                {
                    using (new EditorGUILayout.HorizontalScope())
                    {
                        GUILayout.Label(tn.TunableName(i), GUILayout.Width(150));
                        float v = tn.TunableValue(i);
                        float nv = tn.TunableIsInt(i)
                            ? EditorGUILayout.IntSlider(Mathf.RoundToInt(v),
                                Mathf.RoundToInt(tn.TunableMin(i)), Mathf.RoundToInt(tn.TunableMax(i)))
                            : EditorGUILayout.Slider(v, tn.TunableMin(i), tn.TunableMax(i));
                        if (!Mathf.Approximately(nv, v))
                        {
                            if (comp != null) Undo.RecordObject(comp, "Tune " + tn.TunableName(i));
                            tn.SetTunableValue(i, nv);
                            if (comp != null) EditorUtility.SetDirty(comp);
                        }
                    }
                }
            }
        }

        // ---------------- shared bits ----------------
        private static string DisplayName(IAccumulationController c)
        {
            var comp = c as Component;
            return comp != null ? comp.GetType().Name + "  (" + comp.gameObject.name + ")" : c.ToString();
        }

        private static string GoName(object o)
        {
            var comp = o as Component;
            return comp != null ? comp.gameObject.name : (o != null ? o.ToString() : string.Empty);
        }

        private static void DrawSelectButton(Component comp)
        {
            if (comp == null) return;
            if (GUILayout.Button("Select", GUILayout.Width(52)))
            {
                Selection.activeGameObject = comp.gameObject;
                EditorGUIUtility.PingObject(comp.gameObject);
            }
        }

        // Keep the panel live while in Play mode (state changes from code/runtime).
        private void OnInspectorUpdate() => Repaint();
    }
}
