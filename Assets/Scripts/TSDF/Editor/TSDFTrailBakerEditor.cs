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

            if (!string.IsNullOrEmpty(t.LastStatus))
                EditorGUILayout.HelpBox(t.LastStatus, MessageType.Info);
        }
    }
}
