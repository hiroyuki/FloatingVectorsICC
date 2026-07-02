// Inspector for TSDFTrailBaker: default fields, the manual bake buttons, the
// shared accumulation row (Start/Stop capture — status box shows LastStatus),
// and the dedicated "Resume live" button. Resume live intentionally does NOT
// ride the shared Clear slot (CanClear=false): its side effects (discard
// sculpture + rebuild double-buffered + re-enable integration + resume
// playback) deserve their own explicit button.

using Shared.EditorTools;
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

            AccumulationControllerGUI.Draw(t, t, new AccumulationControllerGUI.Options
            {
                requirePlayMode = true,
                clearUndo = AccumulationControllerGUI.UndoMode.None,
            });

            // After a capture the body integrator is left frozen (trail-only capture
            // disables it), so the live surface mesh stays gone. One click restores it.
            if (GUILayout.Button("Resume live (exit capture → body back)", GUILayout.Height(24)))
                t.ResumeLive();

            if (Application.isPlaying) Repaint();
        }
    }
}
