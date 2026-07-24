// Single point of control for point cloud visibility. Previously,
// SensorManager and SensorRecorder each carried their own
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
    public sealed class PointCloudView : MonoBehaviour, Shared.IViewToggle, Shared.IPanelTunable
    {
        [Tooltip("Hide / show every registered point cloud mesh (Live + Playback) and the " +
                 "cumulative snapshots, without stopping capture / reconstruction. Toggle at " +
                 "runtime; re-enabling resumes instantly because the underlying pipeline keeps " +
                 "running.")]
        public bool showPointClouds = true;

        // ---- Shared.IViewToggle (unified Views panel) ----
        public string ViewLabel => "Point cloud";
        public bool Visible { get => showPointClouds; set => showPointClouds = value; }

        [Range(0f, 3f)]
        [Tooltip("Overall colour brightness for Live, Playback, and cumulative point clouds. " +
                 "1 = source colour, 0 = black, values above 1 brighten the cloud.")]
        public float brightness = 1f;

        // ---- Shared.IPanelTunable (Control Panel > Tuning) ----
        public string TuningLabel => "Point cloud appearance";
        public int TunableCount => 1;
        public string TunableName(int i) => i == 0 ? "Brightness" : string.Empty;
        public float TunableValue(int i) => i == 0 ? brightness : 0f;
        public float TunableMin(int i) => 0f;
        public float TunableMax(int i) => 3f;
        public bool TunableIsInt(int i) => false;
        public void SetTunableValue(int i, float value)
        {
            if (i != 0) return;
            brightness = Mathf.Clamp(value, 0f, 3f);
            _appearanceDirty = true;
            ApplyAppearance();
        }

        [Tooltip("Optional explicit Cumulative reference. Auto-found via FindFirstObjectByType " +
                 "if left null.")]
        public PointCloudCumulative cumulative;

        private static readonly int BrightnessId = Shader.PropertyToID("_Brightness");
        private readonly List<MeshRenderer> _renderers = new List<MeshRenderer>();
        private MaterialPropertyBlock _appearanceBlock;
        private bool _lastApplied = true;
        private float _lastBrightness = -1f;
        private bool _stateDirty = true; // force first-frame sync
        private bool _appearanceDirty = true;

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
            ApplyBrightness(mr);
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
            bool visibilityChanged = showPointClouds != _lastApplied || _stateDirty;
            bool brightnessChanged = !Mathf.Approximately(brightness, _lastBrightness)
                                     || _appearanceDirty;
            if (!visibilityChanged && !brightnessChanged) return;

            brightness = Mathf.Clamp(brightness, 0f, 3f);
            for (int i = _renderers.Count - 1; i >= 0; i--)
            {
                var mr = _renderers[i];
                if (mr == null) { _renderers.RemoveAt(i); continue; }
                if (visibilityChanged && mr.enabled != showPointClouds)
                    mr.enabled = showPointClouds;
                if (brightnessChanged) ApplyBrightness(mr);
            }
            if (cumulative == null) cumulative = FindFirstObjectByType<PointCloudCumulative>();
            if (cumulative != null)
            {
                if (visibilityChanged) cumulative.SnapshotsVisible = showPointClouds;
                if (brightnessChanged) cumulative.Brightness = brightness;
            }
            _lastApplied = showPointClouds;
            _lastBrightness = brightness;
            _stateDirty = false;
            _appearanceDirty = false;
        }

        private void ApplyAppearance()
        {
            for (int i = _renderers.Count - 1; i >= 0; i--)
            {
                var mr = _renderers[i];
                if (mr == null) { _renderers.RemoveAt(i); continue; }
                ApplyBrightness(mr);
            }
            if (cumulative == null) cumulative = FindFirstObjectByType<PointCloudCumulative>();
            if (cumulative != null) cumulative.Brightness = brightness;
            _lastBrightness = brightness;
            _appearanceDirty = false;
        }

        private void ApplyBrightness(MeshRenderer mr)
        {
            if (_appearanceBlock == null) _appearanceBlock = new MaterialPropertyBlock();
            mr.GetPropertyBlock(_appearanceBlock);
            _appearanceBlock.SetFloat(BrightnessId, brightness);
            mr.SetPropertyBlock(_appearanceBlock);
        }
    }
}
