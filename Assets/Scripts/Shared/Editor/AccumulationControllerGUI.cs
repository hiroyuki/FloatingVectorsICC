// Shared Inspector row for IAccumulationController implementers: one uniform
// "Accumulation" section (status box + ● Start / ■ Stop / Clear buttons) so
// PointCloudCumulative, MeshCumulative and TSDFTrailBaker present the same
// operation surface (plan phase 3).
//
// This is a thin LAYOUT helper only — the interface can't carry Undo/dirty or
// play-mode gating (it isn't a UnityEngine.Object), so the calling editor
// passes the target Object plus Options:
//   - requirePlayMode: grey the whole row outside Play mode (MeshCumulative /
//     TSDFTrailBaker) or allow edit-time use (PointCloudCumulative's Clear).
//   - clearUndo: FullHierarchy for clears that destroy child GameObjects
//     (PointCloudCumulative snapshots — plain RecordObject is NOT enough),
//     RecordObject for field-only mutations, None for playmode-only runtime
//     actions that Undo can't meaningfully revert.

using UnityEditor;
using UnityEngine;

namespace Shared.EditorTools
{
    public static class AccumulationControllerGUI
    {
        public enum UndoMode { None, RecordObject, FullHierarchy }

        public struct Options
        {
            public bool requirePlayMode;
            public UndoMode clearUndo;
        }

        public static void Draw(Object target, IAccumulationController c, Options opts)
        {
            EditorGUILayout.Space();
            EditorGUILayout.LabelField("Accumulation", EditorStyles.boldLabel);

            if (!string.IsNullOrEmpty(c.StatusText))
                EditorGUILayout.HelpBox(c.StatusText,
                    c.IsAccumulating ? MessageType.Warning : MessageType.Info);

            using (new EditorGUI.DisabledScope(opts.requirePlayMode && !Application.isPlaying))
            using (new EditorGUILayout.HorizontalScope())
            {
                using (new EditorGUI.DisabledScope(!c.CanStart))
                {
                    if (GUILayout.Button("● " + c.StartLabel, GUILayout.Height(24)))
                        c.StartAccumulate();
                }
                using (new EditorGUI.DisabledScope(!c.CanStop))
                {
                    if (GUILayout.Button("■ Stop", GUILayout.Height(24)))
                        c.StopAccumulate();
                }
                if (c.CanClear)
                {
                    if (GUILayout.Button(c.ClearLabel, GUILayout.Height(24)))
                    {
                        var go = (target as Component) != null ? ((Component)target).gameObject : null;
                        if (opts.clearUndo == UndoMode.FullHierarchy && go != null)
                            Undo.RegisterFullObjectHierarchyUndo(go, c.ClearLabel);
                        else if (opts.clearUndo == UndoMode.RecordObject && target != null)
                            Undo.RecordObject(target, c.ClearLabel);
                        c.ClearAccumulated();
                        if (target != null) EditorUtility.SetDirty(target);
                    }
                }
            }
        }
    }
}
