// Inspector for SensorRecorder: default fields plus Rec/Play/Read buttons
// wired to the matching public methods on the target component (per issue #5).
//
// Recording streams straight to disk during Rec and finalizes on Stop Rec, so
// there is no separate "save the recording" step. Read loads a recording from
// the Inspector's folderPath for playback; "Rewrite metadata (yaml)" re-emits
// the extrinsics / hostinfo / dataset sidecars (useful after a Read + recalibrate
// cycle). All paths come from the folderPath field — the old native folder
// pickers were removed because the Windows picker hangs for seconds when the
// initial directory holds GB-sized recordings.

using UnityEditor;
using UnityEngine;

namespace PointCloud.EditorTools
{
    [CustomEditor(typeof(SensorRecorder))]
    public class SensorRecorderEditor : Editor
    {
        public override void OnInspectorGUI()
        {
            DrawDefaultInspector();

            var t = (SensorRecorder)target;

            EditorGUILayout.Space();
            EditorGUILayout.LabelField("State", t.CurrentState.ToString());
            EditorGUILayout.LabelField("Devices", t.DeviceCount.ToString());
            EditorGUILayout.LabelField("Recorded frames", t.RecordedFrameCount.ToString());
            EditorGUILayout.LabelField("Recorded duration", $"{t.RecordedDuration:0.00}s");
            if (!string.IsNullOrEmpty(t.StatusMessage))
                EditorGUILayout.HelpBox(t.StatusMessage, MessageType.None);

            EditorGUILayout.Space();

            using (new EditorGUILayout.HorizontalScope())
            {
                using (new EditorGUI.DisabledScope(!Application.isPlaying))
                {
                    string recLabel = t.CurrentState == SensorRecorder.State.Recording ? "Stop Rec" : "Rec";
                    if (GUILayout.Button(recLabel)) t.ToggleRecord();

                    string playLabel = t.CurrentState == SensorRecorder.State.Playing ? "Stop Play" : "Play";
                    // Play auto-Reads the configured folder if nothing is loaded, so it's
                    // clickable without a separate Read step.
                    if (GUILayout.Button(playLabel)) t.TogglePlay();

                    string pauseLabel = t.IsPaused ? "Resume" : "Pause";
                    using (new EditorGUI.DisabledScope(t.CurrentState != SensorRecorder.State.Playing))
                    {
                        if (GUILayout.Button(pauseLabel)) t.TogglePause();
                    }
                }
            }

            // Frame-step row (issue #19). Enabled only while playing; the
            // recorder auto-pauses on first press so the user can hold ←/→
            // without a separate Pause click.
            using (new EditorGUILayout.HorizontalScope())
            {
                using (new EditorGUI.DisabledScope(t.CurrentState != SensorRecorder.State.Playing))
                {
                    if (GUILayout.Button("◀ Step")) t.StepBackward();
                    if (GUILayout.Button("Step ▶")) t.StepForward();
                }
            }

            // Reload is now OPTIONAL — Play auto-Reads. Use it only to re-load the
            // folder without pressing Play (e.g. after editing folderPath or a
            // recalibrate). Recording needs no Save — Rec streams to disk and Stop
            // Rec finalizes the files.
            using (new EditorGUILayout.HorizontalScope())
            {
                using (new EditorGUI.DisabledScope(!Application.isPlaying || t.CurrentState != SensorRecorder.State.Idle))
                {
                    if (GUILayout.Button("Reload (optional)")) t.Load();
                }
                using (new EditorGUI.DisabledScope(t.RecordedFrameCount == 0))
                {
                    if (GUILayout.Button("Rewrite metadata (yaml)")) t.Save();
                }
            }

            if (Application.isPlaying) Repaint();
        }
    }
}
