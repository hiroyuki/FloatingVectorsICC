// One-click setup for the body tracking pipeline. Adds a "BodyTracking" parent
// GameObject with two children — [SkeletonMerger] (skeleton overlay, single- or
// multi-camera) and Playback (motion line). Avoids hand-editing main.unity
// from outside the Editor.

using PointCloud;
using UnityEditor;
using UnityEngine;

namespace BodyTracking.EditorTools
{
    public static class BodyTrackingMenu
    {
        [MenuItem("GameObject/Body Tracking/Setup pipeline", false, 30)]
        public static void Setup()
        {
            var parent = GameObject.Find("BodyTracking");
            if (parent == null)
            {
                parent = new GameObject("BodyTracking");
                Undo.RegisterCreatedObjectUndo(parent, "Create BodyTracking root");
            }

            // Live skeleton overlay (single- or multi-camera via K4abtWorkerHost).
            var live = parent.transform.Find("[SkeletonMerger]")?.gameObject;
            if (live == null)
            {
                live = new GameObject("[SkeletonMerger]");
                live.transform.SetParent(parent.transform, false);
                Undo.RegisterCreatedObjectUndo(live, "Create SkeletonMerger");
            }
            var liveComp = live.GetComponent<SkeletonMerger>();
            if (liveComp == null) liveComp = Undo.AddComponent<SkeletonMerger>(live);
            if (liveComp.cameraManager == null)
            {
                liveComp.cameraManager = Object.FindFirstObjectByType<PointCloudCameraManager>();
                EditorUtility.SetDirty(liveComp);
            }
            if (liveComp.workerHost == null)
            {
                liveComp.workerHost = Object.FindFirstObjectByType<K4abtWorkerHost>();
                EditorUtility.SetDirty(liveComp);
            }

            // Playback + motion line.
            var pb = parent.transform.Find("Playback")?.gameObject;
            if (pb == null)
            {
                pb = new GameObject("Playback");
                pb.transform.SetParent(parent.transform, false);
                Undo.RegisterCreatedObjectUndo(pb, "Create Playback");
            }
            var pbComp = pb.GetComponent<BodyTrackingPlayback>();
            if (pbComp == null) pbComp = Undo.AddComponent<BodyTrackingPlayback>(pb);
            if (pbComp.recorder == null)
            {
                pbComp.recorder = Object.FindFirstObjectByType<PointCloudRecorder>();
                EditorUtility.SetDirty(pbComp);
            }
            if (pb.GetComponent<MotionLineRenderer>() == null)
            {
                Undo.AddComponent<MotionLineRenderer>(pb);
            }

            Selection.activeGameObject = parent;
            EditorSceneSelectionPing(parent);
        }

        private static void EditorSceneSelectionPing(Object obj)
        {
            EditorGUIUtility.PingObject(obj);
        }
    }
}
