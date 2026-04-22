// Inspector for PointCloudRecorder: default fields plus Rec/Play/Save/Read buttons
// wired to the matching public methods on the target component (per issue #5).

using UnityEditor;
using UnityEngine;

namespace PointCloud.EditorTools
{
    [CustomEditor(typeof(PointCloudRecorder))]
    public class PointCloudRecorderEditor : Editor
    {
        public override void OnInspectorGUI()
        {
            DrawDefaultInspector();

            var t = (PointCloudRecorder)target;

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
                    string recLabel = t.CurrentState == PointCloudRecorder.State.Recording ? "Stop Rec" : "Rec";
                    if (GUILayout.Button(recLabel)) t.ToggleRecord();

                    string playLabel = t.CurrentState == PointCloudRecorder.State.Playing ? "Stop Play" : "Play";
                    using (new EditorGUI.DisabledScope(t.RecordedFrameCount == 0 && t.CurrentState != PointCloudRecorder.State.Playing))
                    {
                        if (GUILayout.Button(playLabel)) t.TogglePlay();
                    }
                }
            }

            using (new EditorGUILayout.HorizontalScope())
            {
                using (new EditorGUI.DisabledScope(t.RecordedFrameCount == 0))
                {
                    if (GUILayout.Button("Save")) t.Save();
                }
                using (new EditorGUI.DisabledScope(!Application.isPlaying || t.CurrentState != PointCloudRecorder.State.Idle))
                {
                    if (GUILayout.Button("Read")) t.Load();
                }
            }

            if (Application.isPlaying) Repaint();
        }
    }
}
