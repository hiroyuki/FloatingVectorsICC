using UnityEngine;

namespace PointCloud
{
    // Shared writer for the per-mesh shader properties consumed by
    // Assets/Scripts/PointCloud/PointCloudUnlit.shader:
    //   _ObbObjToBox, _ObbMode, _DecimKeep, _DecimFrame.
    //
    // PointCloudRenderer (live) and PointCloudRecorder (playback) both push
    // the same set every frame, so the actual MPB-fill logic lives here to
    // keep them in lock-step. Add a new shader filter? Edit one place.
    internal static class PointCloudShaderFilters
    {
        private static readonly int _ObbObjToBox = Shader.PropertyToID("_ObbObjToBox");
        private static readonly int _ObbMode     = Shader.PropertyToID("_ObbMode");
        private static readonly int _DecimKeep   = Shader.PropertyToID("_DecimKeep");
        private static readonly int _DecimFrame  = Shader.PropertyToID("_DecimFrame");

        // Push the current filter state into `mr`'s MaterialPropertyBlock.
        // `meshTransform` is the renderer's own Transform — used to build the
        // mesh-object → box-local matrix that the shader's PassObb expects.
        // `mpb` is a caller-owned scratch instance reused across frames.
        public static void Apply(MeshRenderer mr, MaterialPropertyBlock mpb,
                                  Transform meshTransform,
                                  PointCloudBoundingBox boundingBox,
                                  PointCloudDecimater decimater)
        {
            mr.GetPropertyBlock(mpb);

            float obbMode = 0f;
            if (boundingBox != null && boundingBox.Mode != PointCloudBoundingBox.FilterMode.Disabled)
            {
                obbMode = boundingBox.Mode == PointCloudBoundingBox.FilterMode.KeepInside ? 1f : 2f;
                var m = boundingBox.transform.worldToLocalMatrix * meshTransform.localToWorldMatrix;
                mpb.SetMatrix(_ObbObjToBox, m);
            }
            mpb.SetFloat(_ObbMode, obbMode);

            float decimKeep = (decimater != null && decimater.Enabled) ? Mathf.Clamp01(decimater.KeepRatio) : 1f;
            mpb.SetFloat(_DecimKeep, decimKeep);
            mpb.SetFloat(_DecimFrame, Time.frameCount);

            mr.SetPropertyBlock(mpb);
        }
    }
}
