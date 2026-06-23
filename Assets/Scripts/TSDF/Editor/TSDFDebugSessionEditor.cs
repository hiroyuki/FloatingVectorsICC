// Inspector for TSDFDebugSession. The single "Compare Two Instants" toggle is the
// control for the compare/bench section, so it's drawn at the TOP of that section
// (right under its header, above the bench settings). The button freezes playback
// and overlays two moments; clicking it again resumes normal playback.

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

                // Open the compare/bench section with its header + the toggle button,
                // then a serial dropdown in place of the raw text field.
                if (!buttonDrawn && it.name == "validateSerial")
                {
                    EditorGUILayout.Space();
                    EditorGUILayout.LabelField("Compare two instants (bench)", EditorStyles.boldLabel);
                    DrawCompareButton(t);
                    buttonDrawn = true;
                    DrawValidateSerialPopup(t, it);
                    continue;   // popup replaces the default text field
                }

                EditorGUILayout.PropertyField(it, true);
            }

            // Fallback if the script field is ever not first.
            if (!buttonDrawn) DrawCompareButton(t);

            serializedObject.ApplyModifiedProperties();

            if (Application.isPlaying) Repaint();
        }

        // Dropdown of available camera serials (populated from the recorder after
        // Play) instead of a free-text field. Empty string = all cameras.
        private static void DrawValidateSerialPopup(TSDFDebugSession t, SerializedProperty prop)
        {
            string[] serials = t.cameraKeys;
            if (serials == null || serials.Length == 0)
            {
                // No serials yet (before Play / before the recording loads) — keep a
                // text field so it's still editable; the dropdown appears once known.
                EditorGUILayout.PropertyField(prop, new GUIContent("Validate Serial"));
                return;
            }

            var options = new string[serials.Length + 1];
            options[0] = "(ALL cameras)";
            for (int i = 0; i < serials.Length; i++) options[i + 1] = serials[i];

            string cur = prop.stringValue;
            int idx = 0;
            if (!string.IsNullOrEmpty(cur))
            {
                int found = System.Array.IndexOf(serials, cur);
                idx = found >= 0 ? found + 1 : 0;
            }
            int next = EditorGUILayout.Popup("Validate Serial", idx, options);
            if (next != idx)
                prop.stringValue = next == 0 ? "" : serials[next - 1];
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
