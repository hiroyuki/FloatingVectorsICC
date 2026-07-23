using UnityEngine;

namespace PointCloud
{
    // Shared writer for the per-mesh shader properties consumed by
    // Assets/Scripts/PointCloud/PointCloudUnlit.shader:
    //   _ObbObjToBox, _ObbMode, _DecimKeep, _DecimFrame, _CapsMode,
    //   _CapsCount, _CapsA[], _CapsB[], _MotionMode, _MotionCount,
    //   _MotionColorMode, _MotionMaxDist, _MotionDisplace, _MotionDisplaceScale,
    //   _MotionSpeedMax, _MotionHotColor, _MotionPos[], _MotionVel[],
    //   _FloorMaskY, _FloorMaskRadius, _FloorFootCount, _FloorFoot[].
    //
    // PointCloudRenderer (live) and SensorRecorder (playback) both push
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
        private static readonly int _MotionMode          = Shader.PropertyToID("_MotionMode");
        private static readonly int _MotionCount         = Shader.PropertyToID("_MotionCount");
        private static readonly int _MotionColorMode     = Shader.PropertyToID("_MotionColorMode");
        private static readonly int _MotionMaxDist       = Shader.PropertyToID("_MotionMaxDist");
        private static readonly int _MotionDisplace      = Shader.PropertyToID("_MotionDisplace");
        private static readonly int _MotionDisplaceScale = Shader.PropertyToID("_MotionDisplaceScale");
        private static readonly int _MotionSpeedMax      = Shader.PropertyToID("_MotionSpeedMax");
        private static readonly int _MotionHotColor      = Shader.PropertyToID("_MotionHotColor");
        private static readonly int _MotionPos           = Shader.PropertyToID("_MotionPos");
        private static readonly int _MotionVel           = Shader.PropertyToID("_MotionVel");
        private static readonly int _FloorMaskY      = Shader.PropertyToID("_FloorMaskY");
        private static readonly int _FloorMaskRadius = Shader.PropertyToID("_FloorMaskRadius");
        private static readonly int _FloorFootCount  = Shader.PropertyToID("_FloorFootCount");
        private static readonly int _FloorFoot       = Shader.PropertyToID("_FloorFoot");

        // Padding buffers used when CapsuleCount == 0 — Unity refuses
        // SetVectorArray with length 0, so we always push the full array but
        // gate visibility on _CapsCount in the shader.
        private static readonly Vector4[] s_emptyCaps = new Vector4[PointCloudCapsuleFilter.MaxCapsules];
        private static readonly Vector4[] s_emptyJoints = new Vector4[PointCloudJointMotionField.MaxJoints];
        private static readonly Vector4[] s_emptyFeet = new Vector4[PointCloudFloorMask.MaxFeet];

        // Push the current filter state into `mr`'s MaterialPropertyBlock.
        // `meshTransform` is the renderer's own Transform — used to build the
        // mesh-object → box-local matrix that the shader's PassObb expects.
        // `mpb` is a caller-owned scratch instance reused across frames.
        public static void Apply(MeshRenderer mr, MaterialPropertyBlock mpb,
                                  Transform meshTransform,
                                  BoundingVolume boundingBox,
                                  PointCloudDecimater decimater,
                                  PointCloudCapsuleFilter capsuleFilter,
                                  PointCloudJointMotionField jointMotionField = null,
                                  PointCloudFloorMask floorMask = null)
        {
            mr.GetPropertyBlock(mpb);

            float obbMode = 0f;
            if (boundingBox != null && boundingBox.Mode != BoundingVolume.FilterMode.Disabled)
            {
                obbMode = boundingBox.Mode == BoundingVolume.FilterMode.KeepInside ? 1f : 2f;
                var m = boundingBox.transform.worldToLocalMatrix * meshTransform.localToWorldMatrix;
                mpb.SetMatrix(_ObbObjToBox, m);
            }
            mpb.SetFloat(_ObbMode, obbMode);

            float decimKeep = (decimater != null && decimater.Enabled) ? Mathf.Clamp01(decimater.KeepRatio) : 1f;
            mpb.SetFloat(_DecimKeep, decimKeep);
            mpb.SetFloat(_DecimFrame, PointCloudDecimater.DecimFramePin >= 0
                ? PointCloudDecimater.DecimFramePin : Time.frameCount);

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

            // Joint motion field (issue #24). _MotionMode = 0 short-circuits
            // the nearest-joint inner loop in the shader entirely, so disabled
            // / unassigned costs only the uniform writes (which we keep small
            // — arrays are pushed unconditionally because Unity rejects 0-length
            // SetVectorArray, but the shader reads only the first _MotionCount
            // entries).
            float motionMode = 0f;
            int motionCount = 0;
            float motionColorMode = 0f;
            float motionMaxDist = 0f;
            float motionDisplace = 0f;
            float motionDisplaceScale = 0f;
            float motionSpeedMax = 1f;
            Color motionHotColor = Color.red;
            if (jointMotionField != null && jointMotionField.Mode != PointCloudJointMotionField.FilterMode.Disabled)
            {
                motionMode = 1f;
                motionCount = Mathf.Clamp(jointMotionField.JointCount, 0, PointCloudJointMotionField.MaxJoints);
                motionColorMode = (float)(int)jointMotionField.Coloring;
                motionMaxDist = Mathf.Max(0f, jointMotionField.maxAssignDistance);
                motionDisplace = jointMotionField.displaceVertices ? 1f : 0f;
                motionDisplaceScale = Mathf.Max(0f, jointMotionField.displaceScale);
                motionSpeedMax = Mathf.Max(1e-3f, jointMotionField.speedMax);
                motionHotColor = jointMotionField.hotColor;
            }
            if (motionCount > 0)
            {
                mpb.SetVectorArray(_MotionPos, jointMotionField.JointPos);
                mpb.SetVectorArray(_MotionVel, jointMotionField.JointVel);
            }
            else
            {
                mpb.SetVectorArray(_MotionPos, s_emptyJoints);
                mpb.SetVectorArray(_MotionVel, s_emptyJoints);
            }
            mpb.SetFloat(_MotionMode, motionMode);
            mpb.SetFloat(_MotionCount, motionCount);
            mpb.SetFloat(_MotionColorMode, motionColorMode);
            mpb.SetFloat(_MotionMaxDist, motionMaxDist);
            mpb.SetFloat(_MotionDisplace, motionDisplace);
            mpb.SetFloat(_MotionDisplaceScale, motionDisplaceScale);
            mpb.SetFloat(_MotionSpeedMax, motionSpeedMax);
            mpb.SetColor(_MotionHotColor, motionHotColor);

            // Floor mask. _FloorMaskY = 0 disables the test in the shader, so a
            // missing / disabled component costs nothing beyond these writes.
            // A live mask with zero feet is NOT disabled — that state hides the
            // floor entirely, which is the intended "nobody is here" look.
            // Parked far below any real geometry when off: the shader's only test is
            // "is this point above the mask height", so nothing is ever below this.
            // A height of exactly 0 therefore stays a usable threshold.
            const float kFloorMaskOff = -1e9f;
            float floorMaskY = kFloorMaskOff;
            float floorRadius = 0f;
            int footCount = 0;
            // isActiveAndEnabled matters as much as Mode: OnDisable clears the feet,
            // so a disabled-but-still-referenced mask would otherwise read as
            // "enabled with nobody tracked" and hide the whole floor.
            if (floorMask != null && floorMask.isActiveAndEnabled
                && floorMask.Mode != PointCloudFloorMask.FilterMode.Disabled
                && floorMask.MaskActive)
            {
                floorMaskY = floorMask.FloorHeight;
                floorRadius = floorMask.KeepRadius;
                footCount = Mathf.Clamp(floorMask.FootCount, 0, PointCloudFloorMask.MaxFeet);
            }
            mpb.SetVectorArray(_FloorFoot, footCount > 0 ? floorMask.Feet : s_emptyFeet);
            mpb.SetFloat(_FloorMaskY, floorMaskY);
            mpb.SetFloat(_FloorMaskRadius, floorRadius);
            mpb.SetFloat(_FloorFootCount, footCount);

            mr.SetPropertyBlock(mpb);
        }
    }
}
