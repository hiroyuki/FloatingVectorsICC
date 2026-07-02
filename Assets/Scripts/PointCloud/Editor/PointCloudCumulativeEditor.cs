// Inspector for PointCloudCumulative: default fields plus the shared
// accumulation row (Start/Stop = noErase toggle, Clear = delete snapshots).
// Clear uses FullHierarchy undo because it destroys the snapshot child GOs.

using Shared.EditorTools;
using UnityEditor;

namespace PointCloud.EditorTools
{
    [CustomEditor(typeof(PointCloudCumulative))]
    public class PointCloudCumulativeEditor : Editor
    {
        public override void OnInspectorGUI()
        {
            DrawDefaultInspector();

            var t = (PointCloudCumulative)target;
            AccumulationControllerGUI.Draw(t, t, new AccumulationControllerGUI.Options
            {
                requirePlayMode = false,   // edit-time Clear allowed (existing behaviour)
                clearUndo = AccumulationControllerGUI.UndoMode.FullHierarchy,
            });
        }
    }
}
