// Editor entry point for re-fitting the sensing box to the camera rig.
//
// The fit normally rides along with a calibration Solve, and the calibration UI
// has a key for it during Play. This menu covers the third case: the operator is
// not in a calibration session, may not even be in Play mode, and just wants the
// box re-derived after the cameras were moved. It writes the same
// calibration/sensing_area.yaml, so all three routes agree.

using Experience;
using PointCloud;
using UnityEditor;
using UnityEngine;

namespace Experience.EditorTools
{
    public static class SensingAreaMenu
    {
        [MenuItem("Window/Calibration/Rebuild Sensing Area")]
        public static void Rebuild()
        {
            var builder = Object.FindFirstObjectByType<ExperienceSpaceBuilder>(FindObjectsInactive.Include);
            bool temporary = false;

            if (builder == null)
            {
                // Same fallback the calibration UI uses: borrow the director's wiring
                // rather than making the operator add a component by hand.
                var dir = Object.FindFirstObjectByType<ExperienceDirector>(FindObjectsInactive.Include);
                if (dir == null || dir.boundingVolume == null)
                {
                    EditorUtility.DisplayDialog("Rebuild Sensing Area",
                        "No ExperienceSpaceBuilder and no ExperienceDirector with a BoundingVolume " +
                        "in the open scene — nothing to fit.", "OK");
                    return;
                }
                var go = new GameObject("__SensingAreaBuilder") { hideFlags = HideFlags.HideAndDontSave };
                builder = go.AddComponent<ExperienceSpaceBuilder>();
                builder.boundingVolume = dir.boundingVolume;
                builder.floorOrigin = dir.floorOrigin;
                builder.sensorManager = Object.FindFirstObjectByType<SensorManager>(FindObjectsInactive.Include);
                builder.sensorRecorder = Object.FindFirstObjectByType<SensorRecorder>(FindObjectsInactive.Include);
                temporary = true;
            }

            var box = builder.boundingVolume;
            if (box != null) Undo.RecordObject(box.transform, "Rebuild sensing area");

            builder.Apply();   // logs, and warns on its own when the layout is degenerate

            if (box != null)
            {
                EditorUtility.SetDirty(box.transform);
                Debug.Log($"[SensingArea] {box.transform.position:F3} size {box.transform.localScale:F3}. " +
                          "Saved to calibration/sensing_area.yaml — the scene copy is only a fallback, " +
                          "so it does not need committing.", box);
            }

            if (temporary) Object.DestroyImmediate(builder.gameObject);
        }

        [MenuItem("Window/Calibration/Rebuild Sensing Area", validate = true)]
        private static bool RebuildValidate()
        {
            // Camera poses come from live renderers or _Playback_<serial> objects,
            // neither of which exists before the scene is running or loaded.
            return Object.FindFirstObjectByType<ExperienceDirector>(FindObjectsInactive.Include) != null
                   || Object.FindFirstObjectByType<ExperienceSpaceBuilder>(FindObjectsInactive.Include) != null;
        }
    }
}
