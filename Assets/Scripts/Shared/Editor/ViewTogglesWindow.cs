// One panel for every IViewToggle in the open scene(s): see at a glance what
// is currently drawn (point cloud / TSDF mesh / BT skeleton) and flip each —
// or all — without hunting through three different Inspectors. Window >
// Floating Vectors > Views.

using System.Collections.Generic;
using UnityEditor;
using UnityEngine;

namespace Shared.EditorTools
{
    public class ViewTogglesWindow : EditorWindow
    {
        [MenuItem("Window/Floating Vectors/Views")]
        public static void Open() => GetWindow<ViewTogglesWindow>("Views");

        private readonly List<IViewToggle> _toggles = new List<IViewToggle>();

        private void OnGUI()
        {
            // Re-scan every draw: components appear/disappear with scene loads and
            // play mode, and the count is tiny (a handful of MonoBehaviours).
            _toggles.Clear();
            foreach (var mb in FindObjectsByType<MonoBehaviour>(FindObjectsInactive.Include, FindObjectsSortMode.None))
                if (mb is IViewToggle vt) _toggles.Add(vt);

            if (_toggles.Count == 0)
            {
                EditorGUILayout.HelpBox("No IViewToggle components in the open scene(s).", MessageType.Info);
                return;
            }

            EditorGUILayout.LabelField("Views", EditorStyles.boldLabel);
            foreach (var vt in _toggles)
            {
                var obj = vt as Object;
                bool now = EditorGUILayout.ToggleLeft(vt.ViewLabel, vt.Visible);
                if (now != vt.Visible)
                {
                    if (obj != null) Undo.RecordObject(obj, (now ? "Show " : "Hide ") + vt.ViewLabel);
                    vt.Visible = now;
                    if (obj != null) EditorUtility.SetDirty(obj);
                }
            }

            EditorGUILayout.Space();
            using (new EditorGUILayout.HorizontalScope())
            {
                if (GUILayout.Button("All on")) SetAll(true);
                if (GUILayout.Button("All off")) SetAll(false);
            }
        }

        private void SetAll(bool visible)
        {
            foreach (var vt in _toggles)
            {
                var obj = vt as Object;
                if (obj != null) Undo.RecordObject(obj, visible ? "Show all views" : "Hide all views");
                vt.Visible = visible;
                if (obj != null) EditorUtility.SetDirty(obj);
            }
        }

        // Keep the panel live while in Play mode (toggles can be driven from code).
        private void OnInspectorUpdate() => Repaint();
    }
}
