// Inspector buttons for driving an evaluation run from the editor without
// wiring anything at runtime: point the EvalReplayDriver at a session root,
// hit "Load & Run (single pass)", then "Finish & Export CSV".

using UnityEditor;
using UnityEngine;

namespace BodyTracking.Eval.Editor
{
    [CustomEditor(typeof(EvalRunner))]
    public sealed class EvalRunnerEditor : UnityEditor.Editor
    {
        private string _lastSummary = "";

        public override void OnInspectorGUI()
        {
            DrawDefaultInspector();

            var runner = (EvalRunner)target;
            var driver = runner.GetComponent<EvalReplayDriver>();

            EditorGUILayout.Space();
            EditorGUILayout.LabelField("Evaluation Run", EditorStyles.boldLabel);

            if (driver != null)
                driver.sessionRoot = EditorGUILayout.TextField("Session Root", driver.sessionRoot);

            using (new EditorGUI.DisabledScope(!Application.isPlaying))
            {
                if (!Application.isPlaying)
                    EditorGUILayout.HelpBox("Enter Play mode to run.", MessageType.Info);

                using (new EditorGUILayout.HorizontalScope())
                {
                    if (GUILayout.Button("Load & Run (single pass)"))
                    {
                        driver.loop = false;                 // single pass so metrics finalize
                        if (runner.LoadAndRun(driver.sessionRoot))
                            _lastSummary = "(running…)";
                    }
                    if (GUILayout.Button("Finish & Export CSV"))
                    {
                        _lastSummary = runner.Finish();
                    }
                }

                EditorGUILayout.LabelField($"Running: {runner.IsRunning}   " +
                                           (driver != null && driver.IsLoaded ? $"devices: {driver.Devices.Count}" : "not loaded"));
            }

            if (!string.IsNullOrEmpty(_lastSummary))
            {
                EditorGUILayout.Space();
                EditorGUILayout.LabelField("Summary", EditorStyles.boldLabel);
                EditorGUILayout.TextArea(_lastSummary, GUILayout.MinHeight(80));
            }

            if (Application.isPlaying) Repaint();
        }
    }
}
