// THE one-stop operation panel: Window > Floating Vectors > Control Panel.
//
// Everything operable lives here so you never have to remember which
// GameObject carries which Inspector:
//   - Views: every IViewToggle (point cloud / TSDF mesh / BT skeleton) as
//     checkboxes + All on/off.
//   - Accumulation: every IAccumulationController (point-cloud snapshots /
//     TSDF cumulative / motion-sculpture capture) with its status and
//     Start / Stop / Clear (+ extra actions like the baker's Resume live).
// Each row has a Select button that pings the owning GameObject when you DO
// want the full Inspector (all fields remain there).
//
// Discovery is interface-based (Shared.IViewToggle / IAccumulationController /
// IAccumulationExtraActions), so this panel needs no references to the
// concrete PointCloud/TSDF/BodyTracking assemblies and picks up any future
// implementer automatically.

using System.Collections.Generic;
using UnityEditor;
using UnityEngine;

namespace Shared.EditorTools
{
    public class FloatingVectorsControlPanel : EditorWindow
    {
        [MenuItem("Window/Floating Vectors/Control Panel")]
        public static void Open() => GetWindow<FloatingVectorsControlPanel>("FV Control");

        private readonly List<IViewToggle> _views = new List<IViewToggle>();
        private readonly List<IAccumulationController> _accums = new List<IAccumulationController>();
        private Vector2 _scroll;

        private void OnGUI()
        {
            // Re-scan every draw: components appear/disappear with scene loads and
            // play mode, and the counts are tiny (a handful of MonoBehaviours).
            _views.Clear();
            _accums.Clear();
            foreach (var mb in FindObjectsByType<MonoBehaviour>(FindObjectsInactive.Include, FindObjectsSortMode.None))
            {
                if (mb is IViewToggle vt) _views.Add(vt);
                if (mb is IAccumulationController ac) _accums.Add(ac);
            }
            _views.Sort((a, b) => string.CompareOrdinal(a.ViewLabel, b.ViewLabel));
            _accums.Sort((a, b) => string.CompareOrdinal(DisplayName(a), DisplayName(b)));

            _scroll = EditorGUILayout.BeginScrollView(_scroll);

            DrawViewsSection();
            EditorGUILayout.Space(12);
            DrawAccumulationSection();

            EditorGUILayout.EndScrollView();
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
        private void DrawAccumulationSection()
        {
            EditorGUILayout.LabelField("Accumulation (累積)", EditorStyles.boldLabel);
            if (_accums.Count == 0)
            {
                EditorGUILayout.HelpBox("No IAccumulationController components in the open scene(s).", MessageType.Info);
                return;
            }
            if (!Application.isPlaying)
                EditorGUILayout.HelpBox("Start/Stop はプレイ中のみ。Enter Play first.", MessageType.None);

            foreach (var c in _accums)
            {
                var comp = c as Component;
                EditorGUILayout.Space(4);
                using (new EditorGUILayout.HorizontalScope())
                {
                    EditorGUILayout.LabelField(DisplayName(c), EditorStyles.miniBoldLabel);
                    DrawSelectButton(comp);
                }

                if (!string.IsNullOrEmpty(c.StatusText))
                    EditorGUILayout.HelpBox(c.StatusText, c.IsAccumulating ? MessageType.Warning : MessageType.Info);

                using (new EditorGUILayout.HorizontalScope())
                {
                    // Start/Stop are runtime operations for every implementer; Clear may be
                    // valid in edit mode too (snapshot deletion) so it is gated only by CanClear.
                    using (new EditorGUI.DisabledScope(!Application.isPlaying || !c.CanStart))
                    {
                        if (GUILayout.Button("● " + c.StartLabel, GUILayout.Height(22)))
                            c.StartAccumulate();
                    }
                    using (new EditorGUI.DisabledScope(!Application.isPlaying || !c.CanStop))
                    {
                        if (GUILayout.Button("■ Stop", GUILayout.Height(22)))
                            c.StopAccumulate();
                    }
                    if (c.CanClear && GUILayout.Button(c.ClearLabel, GUILayout.Height(22)))
                    {
                        // FullHierarchy covers the heaviest case (snapshot child GOs being
                        // destroyed); a superset of RecordObject for the others, harmless.
                        if (comp != null) Undo.RegisterFullObjectHierarchyUndo(comp.gameObject, c.ClearLabel);
                        c.ClearAccumulated();
                        if (comp != null) EditorUtility.SetDirty(comp);
                    }
                    if (c is IAccumulationExtraActions ex)
                    {
                        using (new EditorGUI.DisabledScope(!Application.isPlaying))
                        {
                            for (int i = 0; i < ex.ExtraActionCount; i++)
                                if (GUILayout.Button(ex.ExtraActionLabel(i), GUILayout.Height(22)))
                                    ex.RunExtraAction(i);
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
