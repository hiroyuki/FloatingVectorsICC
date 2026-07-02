// Inspector for TSDFTrailBaker: default fields plus a Bake button and the last
// bake status, so the trail->SDF re-mesh can be triggered without the context menu.

using UnityEditor;
using UnityEngine;

namespace TSDF.EditorTools
{
    [CustomEditor(typeof(TSDFTrailBaker))]
    public class TSDFTrailBakerEditor : Editor
    {
        public override void OnInspectorGUI()
        {
            DrawDefaultInspector();

            var t = (TSDFTrailBaker)target;
            EditorGUILayout.Space();

            if (GUILayout.Button("Bake BT trail into volume", GUILayout.Height(28)))
                t.BakeTrailIntoVolume();

            if (GUILayout.Button("Fuse trail into displayed body", GUILayout.Height(28)))
                t.FuseTrailIntoDisplayedBody();

            EditorGUILayout.Space();
            EditorGUILayout.LabelField("Capture (Start/Stop accumulation)", EditorStyles.boldLabel);
            using (new EditorGUILayout.HorizontalScope())
            {
                GUI.enabled = !t.IsCapturing;
                if (GUILayout.Button("● Start capture", GUILayout.Height(28)))
                    t.StartCapture();
                GUI.enabled = t.IsCapturing;
                if (GUILayout.Button("■ Stop capture", GUILayout.Height(28)))
                    t.StopCapture();
                GUI.enabled = true;
            }
            if (t.IsCapturing)
                EditorGUILayout.HelpBox("Capturing… motion is accumulating into the frozen buffer.",
                                        MessageType.Warning);

            if (!string.IsNullOrEmpty(t.LastStatus))
                EditorGUILayout.HelpBox(t.LastStatus, MessageType.Info);
        }
    }
}
