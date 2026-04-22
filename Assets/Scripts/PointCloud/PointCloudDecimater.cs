// Per-frame random decimation for point clouds. Attach to a GameObject and
// reference it from PointCloudRenderer.decimater. Each point is independently
// kept with probability (1 - reductionPercent/100), so the output count
// approximates the requested percentage per frame.

using UnityEngine;

namespace PointCloud
{
    [DisallowMultipleComponent]
    public class PointCloudDecimater : MonoBehaviour
    {
        [Range(0f, 100f)]
        [Tooltip("Percentage of points to randomly drop each frame. " +
                 "0 = keep all, 100 = drop all.")]
        public float reductionPercent = 0f;

        public bool Enabled => reductionPercent > 0f;

        public float KeepRatio => Mathf.Clamp01(1f - reductionPercent * 0.01f);
    }
}
