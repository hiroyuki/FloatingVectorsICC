// Inspector for TSDFCaptureSession: default fields plus Start / Stop Capture
// buttons and a live state / countdown readout.

using UnityEditor;
using UnityEngine;

namespace TSDF.EditorTools
{
    [CustomEditor(typeof(TSDFCaptureSession))]
    public class TSDFCaptureSessionEditor : Editor
    {
        public override void OnInspectorGUI()
        {
            DrawDefaultInspector();

            var t = (TSDFCaptureSession)target;

            EditorGUILayout.Space();
            EditorGUILayout.LabelField("State", t.State.ToString());
            if (t.State == TSDFCaptureSession.SessionState.Capturing)
                EditorGUILayout.LabelField("Remaining", $"{t.Remaining:0.0}s / {t.captureDuration:0.0}s");

            using (new EditorGUI.DisabledScope(!Application.isPlaying))
            using (new EditorGUILayout.HorizontalScope())
            {
                bool capturing = t.State == TSDFCaptureSession.SessionState.Capturing;
                if (GUILayout.Button(capturing ? "Restart Capture" : "Start Capture")) t.StartCapture();
                using (new EditorGUI.DisabledScope(!capturing))
                {
                    if (GUILayout.Button("Stop Capture")) t.StopCapture();
                }
            }

            if (!Application.isPlaying)
                EditorGUILayout.HelpBox("Enter Play, then Start Capture (live or while a recording is playing back).", MessageType.None);

            if (Application.isPlaying) Repaint();
        }
    }
}
