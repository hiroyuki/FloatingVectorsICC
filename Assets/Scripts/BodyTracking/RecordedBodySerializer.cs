// Encode / decode BodySnapshot[] to the bodies_main RCSV payload format. The
// byte layout intentionally matches the per-body region of the k4abt worker
// MMF (BodyTracking.Shared.K4abtWorkerSharedLayout): u32 bodyCount followed by
// fixed-size body records (id + reserved + 32 joints × {posXYZ, quatWXYZ, conf}).
//
// Kept Unity-free (only System / System.Buffers types) so it can compile into
// any subset of the project, and so unit-testing the round trip never needs an
// Editor instance.

using System;
using BodyTracking.Shared;

namespace BodyTracking
{
    public static class RecordedBodySerializer
    {
        // Byte layout constants — same names / values as K4abtWorkerSharedLayout
        // so a single grep finds both ends of the contract.
        public const int JointCount = K4abtWorkerSharedLayout.K4abtJointCount; // 32
        public const int JointRecordBytes = K4abtWorkerSharedLayout.JointRecordBytes; // 32
        public const int BodyRecordBytes = K4abtWorkerSharedLayout.BodyRecordBytes;   // 1032
        public const int HeaderBytes = 4; // u32 body count

        /// <summary>
        /// Exact byte size for a frame containing <paramref name="bodyCount"/>
        /// bodies. Use to pre-size the scratch buffer passed to <see cref="Encode"/>.
        /// </summary>
        public static int FrameSize(int bodyCount) => HeaderBytes + Math.Max(0, bodyCount) * BodyRecordBytes;

        /// <summary>
        /// Serialize <paramref name="bodies"/>[0..<paramref name="count"/>) into
        /// <paramref name="scratch"/>. The buffer must have room for at least
        /// <see cref="FrameSize"/>(<paramref name="count"/>) bytes; throws
        /// ArgumentException otherwise. Returns the number of bytes written.
        /// </summary>
        public static int Encode(BodySnapshot[] bodies, int count, byte[] scratch)
        {
            if (scratch == null) throw new ArgumentNullException(nameof(scratch));
            if (count < 0) throw new ArgumentOutOfRangeException(nameof(count));
            int needed = FrameSize(count);
            if (scratch.Length < needed)
                throw new ArgumentException(
                    $"scratch too small: {scratch.Length} < {needed} (count={count})", nameof(scratch));

            int offset = 0;
            WriteUInt32(scratch, ref offset, (uint)count);
            for (int i = 0; i < count; i++)
            {
                var body = bodies[i];
                WriteUInt32(scratch, ref offset, body.Id);
                WriteUInt32(scratch, ref offset, 0); // reserved (alignment to 8 B)
                for (int j = 0; j < JointCount; j++)
                {
                    var jt = body.Joints[j];
                    WriteFloat(scratch, ref offset, jt.Position.X);
                    WriteFloat(scratch, ref offset, jt.Position.Y);
                    WriteFloat(scratch, ref offset, jt.Position.Z);
                    WriteFloat(scratch, ref offset, jt.Orientation.W);
                    WriteFloat(scratch, ref offset, jt.Orientation.X);
                    WriteFloat(scratch, ref offset, jt.Orientation.Y);
                    WriteFloat(scratch, ref offset, jt.Orientation.Z);
                    WriteUInt32(scratch, ref offset, (uint)jt.ConfidenceLevel);
                }
            }
            return offset;
        }

        /// <summary>
        /// Parse a bodies_main RCSV payload into <paramref name="outBuffer"/>.
        /// Returns the number of bodies decoded; clamps to <paramref name="outBuffer"/>.Length
        /// to defend against a future format that reports a larger body count than
        /// the consumer can hold. Each occupied slot's Joints array must be
        /// pre-allocated (BodySnapshot's constructor does this).
        /// </summary>
        public static int Decode(byte[] bytes, int byteCount, BodySnapshot[] outBuffer)
        {
            if (bytes == null) throw new ArgumentNullException(nameof(bytes));
            if (outBuffer == null) throw new ArgumentNullException(nameof(outBuffer));
            if (byteCount < HeaderBytes) return 0;

            int offset = 0;
            uint declared = ReadUInt32(bytes, ref offset);
            int needed = HeaderBytes + (int)declared * BodyRecordBytes;
            if (byteCount < needed)
            {
                // Truncated payload — drop the frame rather than read past the end.
                return 0;
            }

            int outCount = Math.Min((int)declared, outBuffer.Length);
            for (int i = 0; i < (int)declared; i++)
            {
                if (i >= outBuffer.Length)
                {
                    // Skip remaining bodies that don't fit; advance offset so the
                    // next caller sees a clean cursor in case we ever stream
                    // multiple frames from one buffer.
                    offset += BodyRecordBytes;
                    continue;
                }
                var snap = outBuffer[i];
                snap.Id = ReadUInt32(bytes, ref offset);
                offset += 4; // reserved
                for (int j = 0; j < JointCount; j++)
                {
                    var jt = new k4abt_joint_t
                    {
                        Position = new k4a_float3_t
                        {
                            X = ReadFloat(bytes, ref offset),
                            Y = ReadFloat(bytes, ref offset),
                            Z = ReadFloat(bytes, ref offset),
                        },
                        Orientation = new k4a_quaternion_t
                        {
                            W = ReadFloat(bytes, ref offset),
                            X = ReadFloat(bytes, ref offset),
                            Y = ReadFloat(bytes, ref offset),
                            Z = ReadFloat(bytes, ref offset),
                        },
                        ConfidenceLevel = (k4abt_joint_confidence_level_t)ReadUInt32(bytes, ref offset),
                    };
                    snap.Joints[j] = jt;
                }
            }
            return outCount;
        }

        private static void WriteUInt32(byte[] buf, ref int offset, uint v)
        {
            buf[offset    ] = (byte)(v       & 0xFF);
            buf[offset + 1] = (byte)((v >> 8 ) & 0xFF);
            buf[offset + 2] = (byte)((v >> 16) & 0xFF);
            buf[offset + 3] = (byte)((v >> 24) & 0xFF);
            offset += 4;
        }

        private static unsafe void WriteFloat(byte[] buf, ref int offset, float v)
        {
            uint bits = *(uint*)&v;
            WriteUInt32(buf, ref offset, bits);
        }

        private static uint ReadUInt32(byte[] buf, ref int offset)
        {
            uint v = (uint)buf[offset]
                   | ((uint)buf[offset + 1] << 8)
                   | ((uint)buf[offset + 2] << 16)
                   | ((uint)buf[offset + 3] << 24);
            offset += 4;
            return v;
        }

        private static unsafe float ReadFloat(byte[] buf, ref int offset)
        {
            uint bits = ReadUInt32(buf, ref offset);
            return *(float*)&bits;
        }
    }
}
