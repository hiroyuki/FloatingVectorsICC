using UnityEngine;

namespace BodyTracking
{
    /// <summary>
    /// Post-merge hook: adjust selected merged joint WORLD positions after
    /// SkeletonMerger builds the merged skeleton and before BodyVisual receives
    /// it. Implemented on the TSDF side (asmdef: TSDF references BodyTracking,
    /// never the reverse) and attached via <see cref="SkeletonMerger.JointRefiner"/>.
    /// </summary>
    public interface IMergedJointRefiner
    {
        /// <summary>
        /// <paramref name="joints"/>[i] identifies the joint whose world position
        /// is in <paramref name="positionsWorld"/>[i] (in/out). <paramref name="valid"/>[i]
        /// false = the merged joint had no sample this frame (position is stale —
        /// read it if useful, but writes are ignored by the caller).
        /// </summary>
        void RefineJoints(uint clusterId, k4abt_joint_id_t[] joints, Vector3[] positionsWorld, bool[] valid);
    }
}
