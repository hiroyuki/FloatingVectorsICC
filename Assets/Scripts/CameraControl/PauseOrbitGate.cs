// Enables the sibling CameraOrbitController so the scene can be inspected from any angle.
// In dev (Experience Mode OFF) freeOrbitInDev keeps it enabled continuously, so the
// operator can move the camera at any time without pausing. It also turns on while the
// transport is holding still — a playback pause or a live freeze (both surface as
// SensorRecorder.IsPaused) — or while the trail sculpture is being generated
// (TSDFTrailBaker.IsCapturing). During the show the ExperienceDirector forces
// SuppressTransportOrbit on and owns the camera through the IOrbitOverride path, so none
// of these dev triggers fire and the stage framing stays under the director's control.

using PointCloud;
using TSDF;
using UnityEngine;

namespace CameraControl
{
    [RequireComponent(typeof(CameraOrbitController))]
    public class PauseOrbitGate : MonoBehaviour, Shared.IOrbitOverride
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
                 "Display 1 Auto Orbit toggle (Display1OperatorHud) and by the " +
                 "ExperienceDirector during playback/QR phases (via Shared.IOrbitOverride).")]
        public bool autoOrbitOverride;

        [Tooltip("Snapshot the camera pose as the orbit turns on and restore it as it " +
                 "turns off. Off by default (dev keeps the inspection angle); the " +
                 "ExperienceDirector forces it on for the show so the stage framing " +
                 "comes back for the next visitor.")]
        public bool restorePoseOnDisable;

        [Tooltip("Dev only: keep the orbit controller enabled while Experience Mode is " +
                 "OFF, so the operator can move the camera at any time without pausing. " +
                 "Suppressed during the show — the ExperienceDirector forces " +
                 "SuppressTransportOrbit on, so this never fights the presentation camera.")]
        public bool freeOrbitInDev = true;

        // ---- Shared.IOrbitOverride (ExperienceDirector, cross-asmdef) ----
        public bool OrbitOverride
        {
            get => autoOrbitOverride;
            set => autoOrbitOverride = value;
        }

        public bool RestorePoseOnDisable
        {
            get => restorePoseOnDisable;
            set => restorePoseOnDisable = value;
        }

        public bool AutoOrbit
        {
            get { ResolveOrbit(); return orbit != null && orbit.autoOrbit; }
            set { ResolveOrbit(); if (orbit != null) orbit.autoOrbit = value; }
        }

        // When set, the recorder-paused / baker-capturing auto-orbit triggers in
        // Update are ignored — only autoOrbitOverride drives the orbit. The
        // ExperienceDirector forces this on for the session so freezing the model
        // (which pauses the recorder) can't pop the camera into a dev pause-orbit.
        [System.NonSerialized] public bool suppressTransportOrbit;
        public bool SuppressTransportOrbit
        {
            get => suppressTransportOrbit;
            set => suppressTransportOrbit = value;
        }

        private void ResolveOrbit()
        {
            if (orbit == null) orbit = GetComponent<CameraOrbitController>();
        }

        // ---- presentation camera work (Shared.IOrbitOverride) ----
        // The show swaps the orbit target to a person-tracking anchor and
        // speeds the sweep up; the dev pivot (boundingBox) / speed / bob come
        // back when it hands over (null pivot).
        private BoundingVolume _savedBBox;
        private Transform _savedPivotTf;
        private float _savedYawSpeed, _savedBobAmp;
        private bool _presentationApplied;

        public void SetPresentationOrbit(Transform pivot, float yawSpeedDeg, float bobAmpMeters)
        {
            ResolveOrbit();
            if (orbit == null) return;
            if (pivot != null)
            {
                if (!_presentationApplied)
                {
                    _savedBBox = orbit.boundingBox;
                    _savedPivotTf = orbit.pivot;
                    _savedYawSpeed = orbit.autoOrbitYawSpeedDeg;
                    _savedBobAmp = orbit.autoOrbitBobAmpMeters;
                    _presentationApplied = true;
                }
                orbit.boundingBox = null; // boundingBox outranks pivot — clear it
                orbit.pivot = pivot;
                orbit.autoOrbitYawSpeedDeg = yawSpeedDeg;
                orbit.autoOrbitBobAmpMeters = bobAmpMeters;
            }
            else if (_presentationApplied)
            {
                orbit.boundingBox = _savedBBox;
                orbit.pivot = _savedPivotTf;
                orbit.autoOrbitYawSpeedDeg = _savedYawSpeed;
                orbit.autoOrbitBobAmpMeters = _savedBobAmp;
                _presentationApplied = false;
            }
        }

        /// <summary>Immediate off-edge: disable the controller and restore the
        /// saved pose without waiting for the next Update. Used by the
        /// ExperienceDirector's mode exit, where the deferred Update would run
        /// only after the gate's dev flags were already restored.</summary>
        public void ReleaseOrbit()
        {
            ResolveOrbit();
            if (orbit == null || !orbit.enabled) return;
            orbit.enabled = false;
            if (restorePoseOnDisable && _haveSavedPose)
            {
                transform.position = _savedPos;
                transform.rotation = _savedRot;
                _haveSavedPose = false;
            }
        }

        private Vector3 _savedPos;
        private Quaternion _savedRot;
        private bool _haveSavedPose;

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
                        || (!suppressTransportOrbit
                            && (freeOrbitInDev
                                || (recorder != null && recorder.IsPaused)
                                || (baker != null && baker.IsCapturing)));
            if (orbit.enabled == want) return;
            // Snapshot on the off→on edge / restore on the on→off edge. The
            // restore happens the same frame the controller is disabled, so its
            // LateUpdate can never re-apply an orbit pose over it.
            if (want)
            {
                _savedPos = transform.position;
                _savedRot = transform.rotation;
                _haveSavedPose = true;
            }
            orbit.enabled = want;
            if (!want && restorePoseOnDisable && _haveSavedPose)
            {
                transform.position = _savedPos;
                transform.rotation = _savedRot;
                _haveSavedPose = false;
            }
        }
    }
}
