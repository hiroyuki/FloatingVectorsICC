// One-click setup for the body tracking pipeline. Adds a "BodyTracking" parent
// GameObject with two children — Live (skeleton overlay) and Playback (motion
// line) — pre-wired to the first PointCloudRenderer / PointCloudRecorder in
// the scene. Avoids hand-editing main.unity from outside the Editor.

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

            // Live skeleton overlay.
            var live = parent.transform.Find("Live")?.gameObject;
            if (live == null)
            {
                live = new GameObject("Live");
                live.transform.SetParent(parent.transform, false);
                Undo.RegisterCreatedObjectUndo(live, "Create Live");
            }
            var liveComp = live.GetComponent<BodyTrackingLive>();
            if (liveComp == null) liveComp = Undo.AddComponent<BodyTrackingLive>(live);
            // PointCloudRenderer is runtime-spawned by PointCloudCameraManager, so we
            // wire the manager instead and let BodyTrackingLive late-bind to the first
            // renderer once it appears.
            if (liveComp.cameraManager == null)
            {
                liveComp.cameraManager = Object.FindFirstObjectByType<PointCloudCameraManager>();
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
