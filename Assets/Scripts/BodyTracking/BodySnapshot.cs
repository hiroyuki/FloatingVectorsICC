// Pooled DTO that K4abtWorkerHost hands to subscribers via OnSkeletonsReady.
// Each frame the host fills up to K4ABT_MAX_BODIES of these from the MMF
// output slot and reuses the same instances on the next frame, so the field
// arrays must never be retained beyond the synchronous event handler — copy
// out anything you need to persist.

namespace BodyTracking
{
    public sealed class BodySnapshot
    {
        /// <summary>K4ABT body id (matches k4abt_frame_get_body_id).</summary>
        public uint Id;

        /// <summary>Joints in K4ABT order; always length K4ABTConsts.K4ABT_JOINT_COUNT.</summary>
        public readonly k4abt_joint_t[] Joints = new k4abt_joint_t[K4ABTConsts.K4ABT_JOINT_COUNT];
    }
}
