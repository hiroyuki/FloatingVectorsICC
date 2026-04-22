// Inspector for PointCloudCumulative: default fields plus an inline Clear button
// (per issue #4, "clearボタンで全部削除") and a snapshot-count readout.

using UnityEditor;
using UnityEngine;

namespace PointCloud.EditorTools
{
    [CustomEditor(typeof(PointCloudCumulative))]
    public class PointCloudCumulativeEditor : Editor
    {
        public override void OnInspectorGUI()
        {
            DrawDefaultInspector();

            var t = (PointCloudCumulative)target;

            EditorGUILayout.Space();
            EditorGUILayout.LabelField("Snapshots", t.SnapshotCount.ToString());

            using (new EditorGUI.DisabledScope(t.SnapshotCount == 0))
            {
                if (GUILayout.Button("Clear"))
                {
                    Undo.RegisterFullObjectHierarchyUndo(t.gameObject, "Clear Point Cloud Snapshots");
                    t.Clear();
                    EditorUtility.SetDirty(t);
                }
            }
        }
    }
}
