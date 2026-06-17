// Inspector for MeshCumulative: default fields plus Begin / Freeze / Release
// buttons and a live state / countdown readout.

using UnityEditor;
using UnityEngine;

namespace TSDF.EditorTools
{
    [CustomEditor(typeof(MeshCumulative))]
    public class MeshCumulativeEditor : Editor
    {
        public override void OnInspectorGUI()
        {
            DrawDefaultInspector();

            var t = (MeshCumulative)target;

            EditorGUILayout.Space();
            EditorGUILayout.LabelField("State", t.State.ToString());
            if (t.State == MeshCumulative.CumulativeState.Accumulating)
                EditorGUILayout.LabelField("Remaining", $"{t.Remaining:0.0}s / {t.duration:0.0}s");

            using (new EditorGUI.DisabledScope(!Application.isPlaying))
            using (new EditorGUILayout.HorizontalScope())
            {
                bool accumulating = t.State == MeshCumulative.CumulativeState.Accumulating;
                bool frozen = t.State == MeshCumulative.CumulativeState.Frozen;

                if (GUILayout.Button(accumulating ? "Restart" : "Begin")) t.Begin();
                using (new EditorGUI.DisabledScope(!accumulating))
                {
                    if (GUILayout.Button("Freeze")) t.Freeze();
                }
                using (new EditorGUI.DisabledScope(!(frozen || accumulating)))
                {
                    if (GUILayout.Button("Release")) t.Release();
                }
            }

            if (!Application.isPlaying)
                EditorGUILayout.HelpBox("Enter Play, then Begin (live or while a recording is playing back). " +
                                        "Freeze holds the fused mesh; Release returns to live-follow.", MessageType.None);

            if (Application.isPlaying) Repaint();
        }
    }
}
