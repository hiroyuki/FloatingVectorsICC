// Spec §7 session lifecycle: accumulate a moving subject's TSDF trail for a
// fixed time window, then freeze the result so the final "motion-to-mesh" holds
// on screen.
//
// Source-agnostic: the integration it drives (TSDFIntegrator) subscribes to BOTH
// live (PointCloudRenderer.OnRawFramesReady) and playback
// (PointCloudRecorder.OnPlaybackRawFrame), so a capture works identically on a
// live Femto Bolt rig OR on an already-recorded RCSV played back. Nothing here
// touches the record path — recording stays a separate concern.
//
// One capture = clear the volume, run RetainGhost accumulation (spec §3.2 A) for
// captureDuration seconds with the per-batch live-follow clear turned OFF, then
// freeze TSDFIntegrator so later frames stop changing the held mesh.

using UnityEngine;

namespace TSDF
{
    [AddComponentMenu("TSDF/Capture Session")]
    [DefaultExecutionOrder(-5)]
    public sealed class TSDFCaptureSession : MonoBehaviour
    {
        public enum SessionState { Idle, Capturing }

        [Header("References")]
        [Tooltip("Volume to accumulate into. Auto-located on enable.")]
        public TSDFVolume volume;
        [Tooltip("Integrator that feeds the volume. Auto-located on enable.")]
        public TSDFIntegrator integrator;

        [Header("Capture")]
        [Tooltip("Seconds to accumulate before freezing the mesh (spec §7). Wall-clock " +
                 "from StartCapture; on real-time playback this matches the played span.")]
        [Min(0.1f)] public float captureDuration = 10f;
        [Tooltip("Wipe the volume at capture start so each session begins empty. Note: " +
                 "starting from live-follow (clearVolumeOnNewBatch=true) flips the volume " +
                 "to single-buffer, which rebuilds + clears it on the next TSDFVolume " +
                 "update regardless — so clearOnStart=false cannot reliably preserve " +
                 "pre-existing content across that transition.")]
        public bool clearOnStart = true;
        [Tooltip("Force RetainGhost accumulation while capturing so the motion trail is " +
                 "kept instead of averaged away (spec §3.2 A vs B).")]
        public bool forceRetainGhost = true;

        [Header("Triggers")]
        [Tooltip("Start a capture automatically when this component enables (e.g. on Play).")]
        public bool autoStartOnEnable = false;
        [Tooltip("When the window elapses, immediately start a fresh capture instead of " +
                 "freezing — a rolling 'last captureDuration seconds' loop.")]
        public bool loop = false;
        public KeyCode startKey = KeyCode.C;
        public KeyCode stopKey = KeyCode.V;

        public SessionState State { get; private set; } = SessionState.Idle;
        /// <summary>Seconds elapsed in the current capture (0 when idle).</summary>
        public float Elapsed { get; private set; }
        public float Remaining => State == SessionState.Capturing
            ? Mathf.Max(0f, captureDuration - Elapsed) : 0f;

        private double _startTime;

        private void OnEnable()
        {
            if (volume == null) volume = FindAnyObjectByType<TSDFVolume>(FindObjectsInactive.Include);
            if (integrator == null) integrator = FindAnyObjectByType<TSDFIntegrator>(FindObjectsInactive.Include);
            if (volume == null || integrator == null)
            {
                Debug.LogError("[TSDFCaptureSession] Need both a TSDFVolume and a TSDFIntegrator in the scene.", this);
                enabled = false;
                return;
            }
            if (autoStartOnEnable) StartCapture();
        }

        private void OnDisable()
        {
            // If disabled mid-capture, freeze now. Update() stops running once the
            // component is disabled, so the duration timer would never fire and the
            // integrator would keep accumulating forever. StopCapture closes the
            // gate and holds the mesh.
            if (State == SessionState.Capturing) StopCapture();
        }

        /// <summary>
        /// Begin a fresh capture: take the integrator into accumulate mode
        /// (per-batch clear OFF, gate OPEN, RetainGhost) and wipe the volume.
        /// </summary>
        [ContextMenu("Start Capture")]
        public void StartCapture()
        {
            if (volume == null || integrator == null) return;

            // Accumulate, don't live-follow-clear. This also flips the volume to
            // single-buffer (the integrator drives volume.doubleBuffered from this
            // flag), which is what RetainGhost's read-modify-write needs.
            integrator.clearVolumeOnNewBatch = false;
            integrator.integrationEnabled = true;
            if (forceRetainGhost)
                volume.accumulationMode = TSDFVolume.AccumulationMode.RetainGhost;

            if (clearOnStart) volume.Clear();

            _startTime = Time.timeAsDouble;
            Elapsed = 0f;
            State = SessionState.Capturing;
        }

        /// <summary>
        /// Freeze integration so the accumulated mesh holds. Deliberately does NOT
        /// restore clearVolumeOnNewBatch: flipping it back to true would make the
        /// integrator re-enable double buffering, which rebuilds + clears the
        /// volume and wipes the very mesh we just captured. The frozen gate keeps
        /// the held buffer untouched until the next StartCapture.
        /// </summary>
        [ContextMenu("Stop Capture")]
        public void StopCapture()
        {
            if (integrator != null) integrator.integrationEnabled = false;
            State = SessionState.Idle;
            Elapsed = captureDuration;
        }

        private void Update()
        {
            if (Input.GetKeyDown(startKey)) StartCapture();
            if (Input.GetKeyDown(stopKey)) StopCapture();

            if (State != SessionState.Capturing) return;

            Elapsed = (float)(Time.timeAsDouble - _startTime);
            if (Elapsed >= captureDuration)
            {
                if (loop) StartCapture();
                else StopCapture();
            }
        }
    }
}
