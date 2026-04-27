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
        public PointCloudBoundingBox boundingBox;

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
            }

            Quaternion rot = Quaternion.Euler(_pitch, _yaw, 0f);
            Vector3 offset = rot * new Vector3(0f, 0f, -distance);
            transform.position = _pivotPoint + offset;
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
