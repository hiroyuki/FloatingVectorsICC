// Per-frame random decimation for point clouds. Attach to a GameObject and
// reference it from PointCloudRenderer.decimater. Each point is independently
// kept with probability (1 - reductionPercent/100), so the output count
// approximates the requested percentage per frame.

using UnityEngine;

namespace PointCloud
{
    [DisallowMultipleComponent]
    public class PointCloudDecimater : MonoBehaviour, Shared.IPanelTunable
    {
        [Range(0f, 100f)]
        [Tooltip("Percentage of points to randomly drop each frame. " +
                 "0 = keep all, 100 = drop all.")]
        public float reductionPercent = 0f;

        public bool Enabled => reductionPercent > 0f;

        public float KeepRatio => Mathf.Clamp01(1f - reductionPercent * 0.01f);

        // ---- Shared.IPanelTunable (one-stop Control Panel) ----
        // Runtime knob so the cloud density can be balanced against the motion
        // curves without leaving Play mode.
        public string TuningLabel => "Point cloud";
        public int TunableCount => 1;
        public string TunableName(int i) => "Decimate (%)";
        public float TunableValue(int i) => reductionPercent;
        public void SetTunableValue(int i, float value) =>
            reductionPercent = Mathf.Clamp(value, 0f, 100f);
        public float TunableMin(int i) => 0f;
        public float TunableMax(int i) => 100f;
        public bool TunableIsInt(int i) => false;
    }
}
