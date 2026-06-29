// Inspector for TSDFDebugSession. Each action button is drawn right under the
// fields it operates on, so the controls aren't hidden in the component's
// context-menu:
//   - "Compare Two Instants" sits at the top of the bench section (startPlayheadSec).
//   - "Integrate Frame Range" sits right under the Frame-range fields.

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
            bool compareDrawn = false;
            bool accumDrawn = false;
            while (it.NextVisible(enterChildren))
            {
                enterChildren = false;

                if (it.propertyPath == "m_Script")
                {
                    using (new EditorGUI.DisabledScope(true))
                        EditorGUILayout.PropertyField(it);
                    continue;
                }

                // Bench header + Compare button, just above the bench settings.
                if (!compareDrawn && it.name == "startPlayheadSec")
                {
                    EditorGUILayout.Space();
                    EditorGUILayout.LabelField("Compare two instants (bench)", EditorStyles.boldLabel);
                    DrawCompareButton(t);
                    compareDrawn = true;
                }

                // Accumulate header + Start/Stop button, just above the read-only cache status.
                if (!accumDrawn && it.name == "cachedCount")
                {
                    EditorGUILayout.Space();
                    EditorGUILayout.LabelField("Accumulate during playback (motion mesh)", EditorStyles.boldLabel);
                    DrawAccumulateButton(t);
                    accumDrawn = true;
                }

                EditorGUILayout.PropertyField(it, true);
            }

            if (!accumDrawn) DrawAccumulateButton(t);

            // Fallback if startPlayheadSec is ever not present.
            if (!compareDrawn) DrawCompareButton(t);

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
                    "In Play: toggle on to freeze and overlay two moments; " +
                    "toggle off to resume normal playback.", MessageType.None);
            EditorGUILayout.Space();
        }

        private static void DrawAccumulateButton(TSDFDebugSession t)
        {
            EditorGUILayout.Space();
            using (new EditorGUI.DisabledScope(!Application.isPlaying))
            {
                bool acc = t.IsAccumulating;
                var prev = GUI.backgroundColor;
                GUI.backgroundColor = acc ? new Color(1f, 0.55f, 0.55f) : new Color(0.6f, 0.95f, 0.6f);
                string label = acc ? "● Accumulating — click to STOP (freeze)" : "Start Accumulating";
                if (GUILayout.Button(label, GUILayout.Height(28)))
                    t.ToggleAccumulate();
                GUI.backgroundColor = prev;
            }
            if (!Application.isPlaying)
                EditorGUILayout.HelpBox(
                    "In Play: press Start, then every frame playback advances folds into one " +
                    "RetainGhost motion mesh (it builds up live). Press Stop to freeze.",
                    MessageType.None);
            EditorGUILayout.Space();
        }
    }
}
