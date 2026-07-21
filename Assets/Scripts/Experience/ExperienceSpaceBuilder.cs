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
        [Tooltip("Clearance between each camera and the nearest face of the sensing " +
                 "box (m). The cameras' XZ bounding rectangle is pulled in by this " +
                 "much on every side, so it is a true perpendicular distance — not a " +
                 "diagonal one.")]
        public float insetMeters = 0.8f;

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
            bool saved = false;
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

            // Footprint = the cameras' XZ bounding rectangle, pulled in by
            // insetMeters on each of the four sides. The inset is PERPENDICULAR to
            // each side, i.e. it is the actual clearance between a camera and the
            // nearest face of the box — that is what "keep the sensing area N metres
            // off the camera stands" means. (Insetting along the centroid diagonal
            // instead, as this did originally, spends the distance on both axes at
            // once and leaves only inset/√2 ≈ 57 % of it as real clearance.)
            Vector3 min = Vector3.positiveInfinity, max = Vector3.negativeInfinity;
            for (int i = 0; i < 4; i++)
            {
                var p = new Vector3(camPos[i].x, 0f, camPos[i].z);
                min = Vector3.Min(min, p);
                max = Vector3.Max(max, p);
            }
            min += new Vector3(insetMeters, 0f, insetMeters);
            max -= new Vector3(insetMeters, 0f, insetMeters);
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

            // Persist next to cameras.yaml / floor.yaml rather than in the scene: the
            // two sets share main.unity but not their camera positions, so a box
            // committed to the scene lands wrong on the other machine.
            var worldCenter = boundingVolume.transform.position;
            var worldSize = boundingVolume.transform.localScale;
            try
            {
                string root = sensorManager != null ? sensorManager.ResolveExtrinsicsRoot() : null;
                if (!string.IsNullOrEmpty(root))
                {
                    PointCloudRecording.WriteSensingArea(root,
                        new[] { worldCenter.x, worldCenter.y, worldCenter.z },
                        new[] { worldSize.x, worldSize.y, worldSize.z },
                        insetMeters, areaHeight);
                    saved = true;
                }
            }
            catch (System.Exception e)
            {
                Debug.LogWarning($"[{nameof(ExperienceSpaceBuilder)}] sensing area not saved: {e.Message}", this);
            }

            Debug.Log($"[{nameof(ExperienceSpaceBuilder)}] sensing area: center ({center.x:0.00}, {center.z:0.00}), " +
                      $"size {size.x:0.00} x {areaHeight:0.0} x {size.z:0.00} m, floorY {floorY:0.00}" +
                      (saved ? " (saved to sensing_area.yaml)." : " (NOT saved)."), this);
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
