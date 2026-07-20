// Sensing-area builder for the experience flow (Phase 2 of
// Plans/experience-flow-plan.md): shrink-wraps the existing BoundingVolume to
// the rectangle spanned by the four camera positions pulled 1 m inward along
// their diagonals, and points FloorOrigin at it so the grid follows.
//
// Meant to run with world rebase active (cameras placed around the origin);
// the box is pinned to rotation = identity in that frame — FloorOrigin only
// follows the box's bottom-center POSITION, not rotation (FloorOrigin.cs), so
// an axis-aligned box is a hard requirement, not a style choice.
//
// Apply() snapshots what it touches and Restore() puts it back, so Dev mode is
// untouched when the experience mode exits (the ExperienceDirector will own
// the Apply/Restore calls in a later phase; ContextMenu for manual use now).

using System.Collections.Generic;
using Calibration;
using PointCloud;
using UnityEngine;

namespace Experience
{
    [DisallowMultipleComponent]
    public class ExperienceSpaceBuilder : MonoBehaviour
    {
        [Tooltip("The OBB to reshape into the sensing area (KeepInside volume the " +
                 "renderers/TSDF already filter through).")]
        public BoundingVolume boundingVolume;

        [Tooltip("Floor grid to attach to the reshaped box (grid position follows the " +
                 "box's bottom center). Optional.")]
        public FloorOrigin floorOrigin;

        [Tooltip("Live rig (camera positions come from its renderers when present).")]
        public SensorManager sensorManager;

        [Tooltip("Playback rig fallback — used when a serial has no live renderer " +
                 "(_Playback_<serial> children hold the camera poses).")]
        public SensorRecorder sensorRecorder;

        [Tooltip("The rig's 4 camera serials in order 1..4. Empty → inherited from " +
                 "sensorRecorder.rigSerialOrder, then sensorManager.rigSerialOrder.")]
        public string[] rigSerialOrder = new string[0];

        [Min(0f)]
        [Tooltip("How far each camera corner is pulled toward the rig centroid (along " +
                 "the diagonal) before the box is fitted — keeps the sensing area away " +
                 "from the camera stands.")]
        public float insetMeters = 1f;

        [Min(0.5f)]
        [Tooltip("Sensing area height (m). The box sits on floorY.")]
        public float areaHeight = 2.5f;

        [Tooltip("Floor height in the (rebased) world. The calibration keeps the " +
                 "board frame's floor level, so confirm against the real rig.")]
        public float floorY = 0f;

        // ---- Apply() snapshot (restored by Restore) ----
        private bool _applied;
        private Vector3 _savedPos;
        private Quaternion _savedRot;
        private Vector3 _savedScale;
        private BoundingVolume _savedFloorBox;

        public bool IsApplied => _applied;

        [ContextMenu("Apply sensing area")]
        public void Apply()
        {
            if (boundingVolume == null)
            { Debug.LogWarning($"[{nameof(ExperienceSpaceBuilder)}] no boundingVolume assigned.", this); return; }
            if (!TryGetCameraPositions(out var camPos)) return;

            // The box must stay world-axis-aligned (FloorOrigin follows position
            // only) — a rotated/scaled parent would bend the world-space values
            // we are about to write.
            if (!WorldFrameRebase.ParentIsIdentity(boundingVolume.transform.parent))
            {
                Debug.LogWarning($"[{nameof(ExperienceSpaceBuilder)}] boundingVolume parent transform " +
                                 "is not identity — the sensing area would not be world-axis-aligned. " +
                                 "Reparent or reset it first.", this);
                return;
            }

            Vector3 centroid = (camPos[0] + camPos[1] + camPos[2] + camPos[3]) * 0.25f;
            centroid.y = 0f;
            Vector3 min = Vector3.positiveInfinity, max = Vector3.negativeInfinity;
            for (int i = 0; i < 4; i++)
            {
                var p = new Vector3(camPos[i].x, 0f, camPos[i].z);
                Vector3 toCenter = centroid - p;
                if (toCenter.magnitude <= insetMeters)
                {
                    Debug.LogWarning($"[{nameof(ExperienceSpaceBuilder)}] camera {i + 1} is within " +
                                     $"{insetMeters} m of the rig centroid — inset would invert the area.", this);
                    return;
                }
                Vector3 inset = p + toCenter.normalized * insetMeters;
                min = Vector3.Min(min, inset);
                max = Vector3.Max(max, inset);
            }
            var size = max - min;
            if (size.x < 0.1f || size.z < 0.1f)
            {
                Debug.LogWarning($"[{nameof(ExperienceSpaceBuilder)}] degenerate sensing area " +
                                 $"({size.x:0.00} x {size.z:0.00} m) — check camera layout / inset.", this);
                return;
            }

            if (!_applied)
            {
                _savedPos = boundingVolume.transform.position;
                _savedRot = boundingVolume.transform.rotation;
                _savedScale = boundingVolume.transform.localScale;
                _savedFloorBox = floorOrigin != null ? floorOrigin.boundingBox : null;
                _applied = true;
            }

            var center = (min + max) * 0.5f;
            boundingVolume.transform.position = new Vector3(center.x, floorY + areaHeight * 0.5f, center.z);
            boundingVolume.transform.rotation = Quaternion.identity;
            boundingVolume.transform.localScale = new Vector3(size.x, areaHeight, size.z);
            if (floorOrigin != null) floorOrigin.boundingBox = boundingVolume;

            Debug.Log($"[{nameof(ExperienceSpaceBuilder)}] sensing area: center ({center.x:0.00}, {center.z:0.00}), " +
                      $"size {size.x:0.00} x {areaHeight:0.0} x {size.z:0.00} m, floorY {floorY:0.00}.", this);
        }

        [ContextMenu("Restore previous volume")]
        public void Restore()
        {
            if (!_applied) return;
            if (boundingVolume != null)
            {
                boundingVolume.transform.position = _savedPos;
                boundingVolume.transform.rotation = _savedRot;
                boundingVolume.transform.localScale = _savedScale;
            }
            if (floorOrigin != null) floorOrigin.boundingBox = _savedFloorBox;
            _savedFloorBox = null;
            _applied = false;
        }

        // Camera world positions in rig serial order: live renderer first, then
        // the _Playback_<serial> child of the recorder. Uses the PLACED scene
        // transforms, so whatever rebase is active is already included.
        private bool TryGetCameraPositions(out Vector3[] positions)
        {
            positions = null;
            var order = ResolveSerialOrder();
            if (order == null || order.Count != 4)
            {
                Debug.LogWarning($"[{nameof(ExperienceSpaceBuilder)}] need 4 serials in rigSerialOrder " +
                                 $"(got {order?.Count ?? 0}).", this);
                return false;
            }
            var result = new Vector3[4];
            for (int i = 0; i < 4; i++)
            {
                var t = FindCameraTransform(order[i]);
                if (t == null)
                {
                    Debug.LogWarning($"[{nameof(ExperienceSpaceBuilder)}] no live renderer or playback GO " +
                                     $"for serial {order[i]}.", this);
                    return false;
                }
                result[i] = t.position;
            }
            positions = result;
            return true;
        }

        private IReadOnlyList<string> ResolveSerialOrder()
        {
            string[] fallback =
                rigSerialOrder is { Length: 4 } ? rigSerialOrder
                : sensorRecorder != null && sensorRecorder.rigSerialOrder is { Length: 4 } rec ? rec
                : sensorManager != null && sensorManager.rigSerialOrder is { Length: 4 } man ? man
                : rigSerialOrder;

            // cameras.yaml beats every scene/config value (see PointCloudRecording.
            // ResolveRigSerialOrder). Live rig present → the machine-local map; pure
            // playback → the take's own map, so cross-set takes keep their rig.
            bool liveRig = sensorManager != null && sensorManager.Renderers != null
                           && sensorManager.Renderers.Count > 0;
            string root = liveRig ? sensorManager.ResolveExtrinsicsRoot()
                : sensorRecorder != null ? sensorRecorder.ResolvePlaybackRoot()
                : sensorManager != null ? sensorManager.ResolveExtrinsicsRoot()
                : null;
            if (string.IsNullOrEmpty(root)) return fallback;
            return PointCloudRecording.ResolveRigSerialOrder(root, fallback, out _);
        }

        private Transform FindCameraTransform(string serial)
        {
            if (sensorManager != null)
            {
                foreach (var r in sensorManager.Renderers)
                    if (r != null && r.deviceSerial == serial) return r.transform;
            }
            if (sensorRecorder != null)
            {
                var t = sensorRecorder.transform.Find($"_Playback_{serial}");
                if (t != null) return t;
            }
            return null;
        }
    }
}
