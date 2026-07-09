// Enables the sibling CameraOrbitController only while the transport is holding
// still — a playback pause or a live freeze (both surface as SensorRecorder.IsPaused).
// While frames flow, the controller is disabled so the stage framing stays locked;
// pressing Space (pause/freeze) hands the camera to the mouse for inspection, and
// resuming locks it again at its current pose.

using PointCloud;
using UnityEngine;

namespace CameraControl
{
    [RequireComponent(typeof(CameraOrbitController))]
    public class PauseOrbitGate : MonoBehaviour
    {
        [Tooltip("Transport whose pause state gates the orbit control. Auto-resolves " +
                 "the first SensorRecorder when left empty.")]
        public SensorRecorder recorder;

        [Tooltip("Orbit controller to gate. Auto-resolves the sibling component.")]
        public CameraOrbitController orbit;

        [Tooltip("Keep the orbit controller enabled regardless of pause — driven by the " +
                 "Display 1 Auto Orbit toggle (Display1OperatorHud).")]
        public bool autoOrbitOverride;

        private void OnEnable()
        {
            if (orbit == null) orbit = GetComponent<CameraOrbitController>();
            if (recorder == null) recorder = FindFirstObjectByType<SensorRecorder>();
        }

        private void Update()
        {
            if (orbit == null) return;
            bool want = autoOrbitOverride || (recorder != null && recorder.IsPaused);
            if (orbit.enabled != want) orbit.enabled = want;
        }
    }
}
