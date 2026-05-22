using UnityEngine;

namespace PointCloud
{
    // Shared writer for the per-mesh shader properties consumed by
    // Assets/Scripts/PointCloud/PointCloudUnlit.shader:
    //   _ObbObjToBox, _ObbMode, _DecimKeep, _DecimFrame, _CapsMode,
    //   _CapsCount, _CapsA[], _CapsB[].
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
        private static readonly int _CapsMode    = Shader.PropertyToID("_CapsMode");
        private static readonly int _CapsCount   = Shader.PropertyToID("_CapsCount");
        private static readonly int _CapsA       = Shader.PropertyToID("_CapsA");
        private static readonly int _CapsB       = Shader.PropertyToID("_CapsB");

        // Padding buffers used when CapsuleCount == 0 — Unity refuses
        // SetVectorArray with length 0, so we always push the full array but
        // gate visibility on _CapsCount in the shader.
        private static readonly Vector4[] s_emptyCaps = new Vector4[PointCloudCapsuleFilter.MaxCapsules];

        // Push the current filter state into `mr`'s MaterialPropertyBlock.
        // `meshTransform` is the renderer's own Transform — used to build the
        // mesh-object → box-local matrix that the shader's PassObb expects.
        // `mpb` is a caller-owned scratch instance reused across frames.
        public static void Apply(MeshRenderer mr, MaterialPropertyBlock mpb,
                                  Transform meshTransform,
                                  PointCloudBoundingBox boundingBox,
                                  PointCloudDecimater decimater,
                                  PointCloudCapsuleFilter capsuleFilter)
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

            float capsMode = 0f;
            int capsCount = 0;
            if (capsuleFilter != null && capsuleFilter.Mode != PointCloudCapsuleFilter.FilterMode.Disabled)
            {
                capsMode = capsuleFilter.Mode == PointCloudCapsuleFilter.FilterMode.KeepInside ? 1f : 2f;
                capsCount = Mathf.Clamp(capsuleFilter.CapsuleCount, 0, PointCloudCapsuleFilter.MaxCapsules);
            }
            // Shader array uniforms have a fixed compile-time size. Always
            // push the full backing array (live or padding) so the binding
            // exists; the shader reads only the first _CapsCount entries.
            if (capsCount > 0)
            {
                mpb.SetVectorArray(_CapsA, capsuleFilter.CapsuleA);
                mpb.SetVectorArray(_CapsB, capsuleFilter.CapsuleB);
            }
            else
            {
                mpb.SetVectorArray(_CapsA, s_emptyCaps);
                mpb.SetVectorArray(_CapsB, s_emptyCaps);
            }
            mpb.SetFloat(_CapsMode, capsMode);
            mpb.SetFloat(_CapsCount, capsCount);

            mr.SetPropertyBlock(mpb);
        }
    }
}
