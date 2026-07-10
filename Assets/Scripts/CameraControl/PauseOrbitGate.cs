// Enables the sibling CameraOrbitController while the transport is holding
// still — a playback pause or a live freeze (both surface as SensorRecorder.IsPaused) —
// or while the trail sculpture is being generated (TSDFTrailBaker.IsCapturing), so the
// growing sculpture can be inspected from any angle. Outside those states the controller
// is disabled and the stage framing stays locked at its current pose.

using PointCloud;
using TSDF;
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

        [Tooltip("Trail baker whose capture state also enables the orbit control " +
                 "(orbit while the sculpture is generating). Auto-resolves the first " +
                 "TSDFTrailBaker when left empty.")]
        public TSDFTrailBaker baker;

        [Tooltip("Keep the orbit controller enabled regardless of pause — driven by the " +
                 "Display 1 Auto Orbit toggle (Display1OperatorHud).")]
        public bool autoOrbitOverride;

        private void OnEnable()
        {
            if (orbit == null) orbit = GetComponent<CameraOrbitController>();
            if (recorder == null) recorder = FindFirstObjectByType<SensorRecorder>();
            if (baker == null) baker = FindFirstObjectByType<TSDFTrailBaker>();
        }

        private void Update()
        {
            if (orbit == null) return;
            bool want = autoOrbitOverride
                        || (recorder != null && recorder.IsPaused)
                        || (baker != null && baker.IsCapturing);
            if (orbit.enabled != want) orbit.enabled = want;
        }
    }
}
