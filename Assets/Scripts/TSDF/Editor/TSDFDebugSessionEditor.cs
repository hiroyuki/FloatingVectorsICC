// Inspector for TSDFDebugSession: the default fields plus a single Compare
// toggle button. "Compare Two Instants" freezes playback and overlays two
// moments; clicking it again resumes normal playback. Replaces the old
// confusing "Build Fixed Frames" + separate "Resume" split.

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
            DrawDefaultInspector();

            var t = (TSDFDebugSession)target;

            EditorGUILayout.Space();
            using (new EditorGUI.DisabledScope(!Application.isPlaying))
            {
                bool comparing = t.IsComparing;
                var prev = GUI.backgroundColor;
                GUI.backgroundColor = comparing ? new Color(1f, 0.55f, 0.55f) : prev;
                string label = comparing
                    ? "● Comparing — click to resume playback"
                    : "Compare Two Instants";
                if (GUILayout.Button(label, GUILayout.Height(28)))
                    t.ToggleCompare();
                GUI.backgroundColor = prev;
            }

            if (!Application.isPlaying)
                EditorGUILayout.HelpBox(
                    "Play: the recording plays normally. Toggle 'Compare Two Instants' to " +
                    "freeze and overlay two moments (red/blue); toggle it off to resume playback.",
                    MessageType.None);

            if (Application.isPlaying) Repaint();
        }
    }
}
