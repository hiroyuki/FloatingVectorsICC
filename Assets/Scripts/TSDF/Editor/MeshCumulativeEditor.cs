// Inspector for MeshCumulative: default fields plus the shared accumulation
// row (Start=Begin/Restart, Stop=Freeze, Clear=Release & resume live).

using Shared.EditorTools;
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
            AccumulationControllerGUI.Draw(t, t, new AccumulationControllerGUI.Options
            {
                requirePlayMode = true,    // runtime-only state machine (existing behaviour)
                clearUndo = AccumulationControllerGUI.UndoMode.None,
            });

            if (!Application.isPlaying)
                EditorGUILayout.HelpBox("Enter Play, then Begin (live or while a recording is playing back). " +
                                        "Stop (Freeze) holds the fused mesh; Clear (Release) returns to live-follow.",
                                        MessageType.None);

            if (Application.isPlaying) Repaint();
        }
    }
}
