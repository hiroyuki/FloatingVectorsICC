// Pulls body skeletons out of a k4abt_frame_t handle and writes them into the
// output slot of the per-camera MMF using the layout defined in
// K4abtWorkerSharedLayout. The writer assumes the caller has already wrapped
// the slot in a seq-lock Begin/Commit pair (so partially written records are
// never observed).

using System;
using BodyTracking;
using BodyTracking.Shared;

namespace K4abtWorker
{
    internal static class SkeletonOutputWriter
    {
        /// <summary>
        /// Read up to MaxBodies skeletons from <paramref name="bodyFrame"/> and
        /// write them into the output slot at <paramref name="slotIdx"/> with the
        /// supplied <paramref name="frameId"/> and <paramref name="tsNs"/>.
        /// Returns the number of bodies written.
        /// </summary>
        public static unsafe uint Write(MmfRegion mmf, int slotIdx, IntPtr bodyFrame,
                                        ulong frameId, ulong tsNs)
        {
            byte* slot = mmf.OutputSlotPayloadPtr(slotIdx);

            // Common slot prefix (seq is owned by the caller; we write frame_id / ts_ns here).
            *(ulong*)(slot + K4abtWorkerSharedLayout.SlotOffsetFrameId) = frameId;
            *(ulong*)(slot + K4abtWorkerSharedLayout.SlotOffsetTsNs) = tsNs;

            uint nBodies = K4ABTNative.k4abt_frame_get_num_bodies(bodyFrame);
            if (nBodies > K4abtWorkerSharedLayout.MaxBodies) nBodies = (uint)K4abtWorkerSharedLayout.MaxBodies;
            *(uint*)(slot + K4abtWorkerSharedLayout.OutSlotOffsetBodyCount) = nBodies;
            *(uint*)(slot + K4abtWorkerSharedLayout.OutSlotOffsetReserved) = 0;

            byte* bodyBase = slot + K4abtWorkerSharedLayout.OutSlotOffsetBodies;
            uint actuallyWritten = 0;

            for (uint i = 0; i < nBodies; i++)
            {
                uint id = K4ABTNative.k4abt_frame_get_body_id(bodyFrame, i);
                if (id == K4ABTConsts.K4ABT_INVALID_BODY_ID) continue;

                if (K4ABTNative.k4abt_frame_get_body_skeleton(bodyFrame, i, out var skel)
                    != k4a_result_t.K4A_RESULT_SUCCEEDED)
                {
                    continue;
                }

                byte* rec = bodyBase + actuallyWritten * K4abtWorkerSharedLayout.BodyRecordBytes;

                // Body header: id + reserved
                *(uint*)(rec + 0) = id;
                *(uint*)(rec + 4) = 0;

                // 32 joints, each: pos(float3) + quat(float4) + confidence(uint32) = 32 B
                byte* jointPtr = rec + 8;
                for (int j = 0; j < K4abtWorkerSharedLayout.K4abtJointCount; j++)
                {
                    var jt = skel.Joints[j];
                    *(float*)(jointPtr + 0) = jt.Position.X;
                    *(float*)(jointPtr + 4) = jt.Position.Y;
                    *(float*)(jointPtr + 8) = jt.Position.Z;
                    *(float*)(jointPtr + 12) = jt.Orientation.W;
                    *(float*)(jointPtr + 16) = jt.Orientation.X;
                    *(float*)(jointPtr + 20) = jt.Orientation.Y;
                    *(float*)(jointPtr + 24) = jt.Orientation.Z;
                    *(uint*)(jointPtr + 28) = (uint)jt.ConfidenceLevel;
                    jointPtr += K4abtWorkerSharedLayout.JointRecordBytes;
                }

                actuallyWritten++;
            }

            // Patch the body_count if some bodies were filtered out (invalid id / skeleton fetch failure).
            if (actuallyWritten != nBodies)
            {
                *(uint*)(slot + K4abtWorkerSharedLayout.OutSlotOffsetBodyCount) = actuallyWritten;
            }
            return actuallyWritten;
        }
    }
}
