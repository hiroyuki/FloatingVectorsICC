// Inspector for TSDFDebugSession. Draws the default fields but places the single
// "Compare Two Instants" toggle button inside the compare/bench group it controls
// (right after the smooth-union settings, before the read-only Cache status),
// instead of floating at the very bottom. The button freezes playback and overlays
// two moments; clicking it again resumes normal playback.

using UnityEditor;
using UnityEngine;
using TSDF.DebugTools;

namespace TSDF.EditorTools
{
    [CustomEditor(typeof(TSDFDebugSession))]
    public class TSDFDebugSessionEditor : Editor
    {
        public override void OnInspectorGUI()
        {
            serializedObject.Update();
            var t = (TSDFDebugSession)target;

            var it = serializedObject.GetIterator();
            bool enterChildren = true;
            bool buttonDrawn = false;
            while (it.NextVisible(enterChildren))
            {
                enterChildren = false;

                if (it.propertyPath == "m_Script")
                {
                    using (new EditorGUI.DisabledScope(true))
                        EditorGUILayout.PropertyField(it);
                    continue;
                }

                // The compare button belongs with the bench it drives: draw it just
                // before the "Cache status" block (cachedCount is its first field).
                if (!buttonDrawn && it.name == "cachedCount")
                {
                    DrawCompareButton(t);
                    buttonDrawn = true;
                }

                EditorGUILayout.PropertyField(it, true);
            }

            // Fallback if the cache-status fields are ever renamed/removed.
            if (!buttonDrawn) DrawCompareButton(t);

            serializedObject.ApplyModifiedProperties();

            if (Application.isPlaying) Repaint();
        }

        private static void DrawCompareButton(TSDFDebugSession t)
        {
            EditorGUILayout.Space();
            using (new EditorGUI.DisabledScope(!Application.isPlaying))
            {
                bool comparing = t.IsComparing;
                var prev = GUI.backgroundColor;
                GUI.backgroundColor = comparing ? new Color(1f, 0.55f, 0.55f) : prev;
                string label = comparing
                    ? "● Comparing — click to resume playback"
                    : "Compare Two Instants";
                if (GUILayout.Button(label, GUILayout.Height(26)))
                    t.ToggleCompare();
                GUI.backgroundColor = prev;
            }
            if (!Application.isPlaying)
                EditorGUILayout.HelpBox(
                    "In Play: toggle on to freeze and overlay two moments (red/blue); " +
                    "toggle off to resume normal playback.", MessageType.None);
            EditorGUILayout.Space();
        }
    }
}
