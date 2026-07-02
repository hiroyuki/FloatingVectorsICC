// Mouse-driven orbit camera. Attach to a Unity Camera (or any Transform).
// Left drag: orbit (yaw/pitch) around the pivot.
// Right/Middle drag: pan the pivot in the view plane.
// Scroll wheel: zoom (change distance to pivot).
//
// Uses the legacy Input Manager (ProjectSettings/activeInputHandler: 0).

using PointCloud;
using UnityEngine;

namespace CameraControl
{
    public class CameraOrbitController : MonoBehaviour
    {
        [Header("Pivot")]
        [Tooltip("Bounding box whose world-space center is used as the orbit pivot. " +
                 "When assigned, it takes priority over 'pivot' each frame. The box " +
                 "itself is never moved; panning accumulates into an internal offset.")]
        public BoundingVolume boundingBox;

        [Tooltip("Fallback pivot transform. Used only when boundingBox is null. " +
                 "If both are null, the pivot is initialized to the point \"distance\" " +
                 "in front of this transform on first Update.")]
        public Transform pivot;

        [Tooltip("Initial distance from pivot to camera (world units = meters). " +
                 "Also used as the fallback when pivot is null.")]
        public float distance = 3f;

        [Header("Orbit")]
        [Tooltip("Degrees of rotation per pixel of left-drag mouse movement.")]
        public float orbitSpeed = 0.3f;
        [Tooltip("Minimum pitch in degrees. Avoid +/-90 to prevent gimbal flip.")]
        public float minPitch = -89f;
        [Tooltip("Maximum pitch in degrees.")]
        public float maxPitch = 89f;
        [Tooltip("Invert vertical mouse axis for orbit.")]
        public bool invertY;

        [Header("Pan")]
        [Tooltip("World units of pan per pixel, scaled by current distance.")]
        public float panSpeed = 0.0015f;

        [Header("Zoom")]
        [Tooltip("Fraction of current distance per scroll notch.")]
        public float zoomSpeed = 0.1f;
        public float minDistance = 0.1f;
        public float maxDistance = 100f;

        [Header("Reset")]
        [Tooltip("Max seconds between two left-clicks to count as a double-click and reset to the default pose.")]
        public float doubleClickInterval = 0.3f;

        [Header("Auto orbit (idle)")]
        [Tooltip("When no mouse input for autoOrbitIdleSeconds, slowly rotate the camera " +
                 "around the pivot for a 3D-revealing preview. Mouse drag instantly takes over.")]
        public bool autoOrbit = true;
        [Tooltip("Seconds of mouse idleness before auto-orbit kicks in.")]
        [Min(0f)] public float autoOrbitIdleSeconds = 0.5f;
        [Tooltip("Yaw degrees / second during auto-orbit. Sign sets direction.")]
        public float autoOrbitYawSpeedDeg = 8f;
        [Tooltip("Sine amplitude (meters) added vertically to the camera position.")]
        public float autoOrbitBobAmpMeters = 0.25f;
        [Tooltip("Frequency (Hz) of the vertical bob.")]
        public float autoOrbitBobFreqHz = 0.07f;
        [Tooltip("Sine amplitude (meters) added to the orbit distance — gentle push-in / pull-out.")]
        public float autoOrbitDollyAmpMeters = 0.6f;
        [Tooltip("Frequency (Hz) of the dolly bob. Pick something different from bob freq so they don't beat.")]
        public float autoOrbitDollyFreqHz = 0.05f;

        private float _lastInputTime;
        private float _autoTimeAccum;

        private Vector3 _pivotPoint;
        private Vector3 _panOffset;
        private float _yaw;
        private float _pitch;
        private bool _initialized;

        // Default pose captured on first initialization; restored on double-click.
        private float _defaultYaw;
        private float _defaultPitch;
        private float _defaultDistance;
        private Vector3 _defaultPivotPoint;
        private float _lastLeftClickTime = -1f;

        private void OnEnable()
        {
            _initialized = false;
        }

        private void LateUpdate()
        {
            if (!_initialized)
            {
                InitializeFromCurrentTransform();
                _initialized = true;
            }

            if (Input.GetMouseButtonDown(0))
            {
                float now = Time.unscaledTime;
                if (_lastLeftClickTime > 0f && now - _lastLeftClickTime <= doubleClickInterval)
                {
                    ResetToDefault();
                    _lastLeftClickTime = -1f;
                }
                else
                {
                    _lastLeftClickTime = now;
                }
            }

            if (boundingBox != null)
                _pivotPoint = boundingBox.transform.position + _panOffset;
            else if (pivot != null)
                _pivotPoint = pivot.position;

            float dx = Input.GetAxis("Mouse X");
            float dy = Input.GetAxis("Mouse Y");
            // GetAxis is frame-rate independent and already scaled; multiply pixel-based speeds
            // by a constant so inspector values stay intuitive.
            const float axisToPixels = 30f;
            dx *= axisToPixels;
            dy *= axisToPixels;

            // Treat any mouse button being held as user input even when motion delta
            // is zero — otherwise the auto-orbit timer expires mid-hold and the
            // camera starts drifting under the user's hand. Mouse-wheel scroll
            // updates this further below.
            bool hadInput = Input.GetMouseButton(0) || Input.GetMouseButton(1) || Input.GetMouseButton(2);
            if (Input.GetMouseButton(0))
            {
                _yaw += dx * orbitSpeed;
                _pitch += (invertY ? dy : -dy) * orbitSpeed;
                _pitch = Mathf.Clamp(_pitch, minPitch, maxPitch);
            }

            if (Input.GetMouseButton(1) || Input.GetMouseButton(2))
            {
                // Pan in the camera's local right/up plane, scaled by distance so
                // the feel is consistent regardless of zoom level.
                Vector3 right = transform.right;
                Vector3 up = transform.up;
                Vector3 delta = (-dx * right - dy * up) * panSpeed * distance;
                _pivotPoint += delta;
                if (boundingBox != null)
                    _panOffset += delta;
                else if (pivot != null)
                    pivot.position = _pivotPoint;
            }

            float scroll = Input.GetAxis("Mouse ScrollWheel");
            if (!Mathf.Approximately(scroll, 0f))
            {
                distance *= Mathf.Exp(-scroll * zoomSpeed * 10f);
                distance = Mathf.Clamp(distance, minDistance, maxDistance);
                hadInput = true;
            }

            // Auto-orbit when mouse has been idle for autoOrbitIdleSeconds. The bob
            // and dolly are added to the live values *for this frame only* (not
            // baked back into _yaw / distance) so resuming manual control after
            // auto-orbit doesn't drift the saved zoom.
            if (hadInput) _lastInputTime = Time.unscaledTime;
            float dollyOffset = 0f;
            float verticalBob = 0f;
            if (autoOrbit && Time.unscaledTime - _lastInputTime >= autoOrbitIdleSeconds)
            {
                _autoTimeAccum += Time.deltaTime;
                _yaw += autoOrbitYawSpeedDeg * Time.deltaTime;
                float twoPi = 2f * Mathf.PI;
                verticalBob = Mathf.Sin(twoPi * autoOrbitBobFreqHz * _autoTimeAccum) * autoOrbitBobAmpMeters;
                dollyOffset = Mathf.Sin(twoPi * autoOrbitDollyFreqHz * _autoTimeAccum) * autoOrbitDollyAmpMeters;
            }

            Quaternion rot = Quaternion.Euler(_pitch, _yaw, 0f);
            float effectiveDistance = Mathf.Max(0.01f, distance + dollyOffset);
            Vector3 offset = rot * new Vector3(0f, 0f, -effectiveDistance);
            transform.position = _pivotPoint + offset + Vector3.up * verticalBob;
            transform.rotation = rot;
        }

        private void InitializeFromCurrentTransform()
        {
            _panOffset = Vector3.zero;

            if (boundingBox != null)
            {
                _pivotPoint = boundingBox.transform.position;
            }
            else if (pivot != null)
            {
                _pivotPoint = pivot.position;
            }
            else
            {
                _pivotPoint = transform.position + transform.forward * distance;
            }

            Vector3 toPivot = _pivotPoint - transform.position;
            float camDist = toPivot.magnitude;
            if (camDist > 1e-4f)
                distance = camDist;

            Vector3 euler = transform.rotation.eulerAngles;
            _pitch = NormalizeAngle(euler.x);
            _yaw = euler.y;
            _pitch = Mathf.Clamp(_pitch, minPitch, maxPitch);

            _defaultYaw = _yaw;
            _defaultPitch = _pitch;
            _defaultDistance = distance;
            _defaultPivotPoint = _pivotPoint;
        }

        private void ResetToDefault()
        {
            _yaw = _defaultYaw;
            _pitch = _defaultPitch;
            distance = _defaultDistance;
            _panOffset = Vector3.zero;
            _pivotPoint = _defaultPivotPoint;
            if (boundingBox == null && pivot != null)
                pivot.position = _defaultPivotPoint;
        }

        private static float NormalizeAngle(float a)
        {
            a %= 360f;
            if (a > 180f) a -= 360f;
            if (a < -180f) a += 360f;
            return a;
        }
    }
}
