// Single point of control for point cloud visibility. Previously,
// PointCloudCameraManager and PointCloudRecorder each carried their own
// showPointClouds bool — Live and Playback meshes were toggled separately,
// and there was no central place to "hide all clouds while keeping the
// pipeline running".
//
// PointCloudView owns the single bool plus a registry of every MeshRenderer
// that represents a point cloud (live spawn + playback spawn both register
// here). The cumulative-snapshots toggle is driven from the same flag so the
// whole "point cloud visual" lever is one knob.
//
// Filters (boundingBox / decimater / capsuleFilter / jointMotionField) and
// material assignment stay where they are — those are not in this PR's scope.

using System.Collections.Generic;
using UnityEngine;

namespace PointCloud
{
    public sealed class PointCloudView : MonoBehaviour
    {
        [Tooltip("Hide / show every registered point cloud mesh (Live + Playback) and the " +
                 "cumulative snapshots, without stopping capture / reconstruction. Toggle at " +
                 "runtime; re-enabling resumes instantly because the underlying pipeline keeps " +
                 "running.")]
        public bool showPointClouds = true;

        [Tooltip("Optional explicit Cumulative reference. Auto-found via FindFirstObjectByType " +
                 "if left null.")]
        public PointCloudCumulative cumulative;

        private readonly List<MeshRenderer> _renderers = new List<MeshRenderer>();
        private bool _lastApplied = true;
        private bool _stateDirty = true; // force first-frame sync

        /// <summary>Register a renderer (Live or Playback). Idempotent — re-registering
        /// the same MR is a no-op. The MR's current enabled state is overridden on the
        /// next Update tick to match showPointClouds.</summary>
        public void Register(MeshRenderer mr)
        {
            if (mr == null) return;
            if (_renderers.Contains(mr)) return;
            _renderers.Add(mr);
            // Apply current state immediately so a freshly-spawned MR doesn't flash
            // visible for a frame when showPointClouds is off.
            if (mr.enabled != showPointClouds) mr.enabled = showPointClouds;
        }

        /// <summary>Unregister a renderer (e.g. on track teardown). Safe to call with
        /// a null or already-destroyed MR.</summary>
        public void Unregister(MeshRenderer mr)
        {
            if (mr == null) return;
            _renderers.Remove(mr);
        }

        private void Update()
        {
            if (showPointClouds == _lastApplied && !_stateDirty) return;
            for (int i = _renderers.Count - 1; i >= 0; i--)
            {
                var mr = _renderers[i];
                if (mr == null) { _renderers.RemoveAt(i); continue; }
                if (mr.enabled != showPointClouds) mr.enabled = showPointClouds;
            }
            if (cumulative == null) cumulative = FindFirstObjectByType<PointCloudCumulative>();
            if (cumulative != null) cumulative.SnapshotsVisible = showPointClouds;
            _lastApplied = showPointClouds;
            _stateDirty = false;
        }
    }
}
