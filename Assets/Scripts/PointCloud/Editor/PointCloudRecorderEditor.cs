// Inspector for PointCloudRecorder: default fields plus Rec/Play/Save/Read buttons
// wired to the matching public methods on the target component (per issue #5).
//
// Save / Read open a native folder picker so the recording root can be chosen at
// click time. The picked path is written back into PointCloudRecorder.folderPath
// so subsequent runtime ContextMenu calls reuse the same location.

using System.IO;
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

                    string pauseLabel = t.IsPaused ? "Resume" : "Pause";
                    using (new EditorGUI.DisabledScope(t.CurrentState != PointCloudRecorder.State.Playing))
                    {
                        if (GUILayout.Button(pauseLabel)) t.TogglePause();
                    }
                }
            }

            using (new EditorGUILayout.HorizontalScope())
            {
                using (new EditorGUI.DisabledScope(t.RecordedFrameCount == 0))
                {
                    if (GUILayout.Button("Save...")) PromptSave(t);
                }
                using (new EditorGUI.DisabledScope(!Application.isPlaying || t.CurrentState != PointCloudRecorder.State.Idle))
                {
                    if (GUILayout.Button("Read...")) PromptRead(t);
                }
            }

            // Direct-path variants that bypass EditorUtility.OpenFolderPanel /
            // SaveFolderPanel. The Windows native picker has been observed to
            // hang for several seconds before showing (and occasionally not show
            // at all) when the initial directory contains many GB-sized files —
            // use these to drive Save / Read off the current folderPath value
            // instead. Edit folderPath above to point at the desired recording.
            using (new EditorGUILayout.HorizontalScope())
            {
                using (new EditorGUI.DisabledScope(t.RecordedFrameCount == 0))
                {
                    if (GUILayout.Button("Save (current folderPath)")) t.Save();
                }
                using (new EditorGUI.DisabledScope(!Application.isPlaying || t.CurrentState != PointCloudRecorder.State.Idle))
                {
                    if (GUILayout.Button("Read (current folderPath)")) t.Load();
                }
            }

            if (Application.isPlaying) Repaint();
        }

        private void PromptSave(PointCloudRecorder t)
        {
            string initialDir = ResolveInitialDir(t);
            string defaultName = ResolveDefaultName(t);
            string picked = EditorUtility.SaveFolderPanel("Save Recording To", initialDir, defaultName);
            if (string.IsNullOrEmpty(picked)) return;

            ApplyFolderPath(picked);
            t.Save();
        }

        private void PromptRead(PointCloudRecorder t)
        {
            string initialDir = ResolveInitialDir(t);
            string picked = EditorUtility.OpenFolderPanel("Read Recording From", initialDir, "");
            if (string.IsNullOrEmpty(picked)) return;

            ApplyFolderPath(picked);
            t.Load();
        }

        private void ApplyFolderPath(string absolutePath)
        {
            var sp = serializedObject.FindProperty("folderPath");
            if (sp != null)
            {
                sp.stringValue = absolutePath;
                serializedObject.ApplyModifiedProperties();
            }
        }

        private static string ResolveInitialDir(PointCloudRecorder t)
        {
            string p = t.folderPath;
            if (!string.IsNullOrWhiteSpace(p))
            {
                if (!Path.IsPathRooted(p))
                    p = Path.Combine(Application.persistentDataPath, p);
                if (Directory.Exists(p)) return p;
                var parent = Path.GetDirectoryName(p);
                if (!string.IsNullOrEmpty(parent) && Directory.Exists(parent)) return parent;
            }
            string fallback = Path.Combine(Application.persistentDataPath, "Recordings");
            return Directory.Exists(fallback) ? fallback : Application.persistentDataPath;
        }

        private static string ResolveDefaultName(PointCloudRecorder t)
        {
            if (!string.IsNullOrWhiteSpace(t.datasetName)) return t.datasetName;
            if (!string.IsNullOrWhiteSpace(t.folderPath))
            {
                string name = Path.GetFileName(t.folderPath.TrimEnd(Path.DirectorySeparatorChar, Path.AltDirectorySeparatorChar));
                if (!string.IsNullOrEmpty(name)) return name;
            }
            return "recording";
        }
    }
}
