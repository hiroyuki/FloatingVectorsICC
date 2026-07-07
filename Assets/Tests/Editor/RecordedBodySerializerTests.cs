// Round-trip and golden-bytes tests for the bodies_main payload format.
//
// The byte layout is a shared contract between three implementations:
//   - Workers/K4abtWorker/SkeletonOutputWriter.cs (worker -> MMF)
//   - K4abtWorkerHost (MMF -> BodySnapshot)
//   - RecordedBodySerializer (BodySnapshot <-> bodies_main on disk)
// The golden-bytes test pins the layout so any codec refactor (or LayoutVersion
// bump) that changes bytes fails loudly here instead of corrupting recordings.

using System;
using BodyTracking;
using BodyTracking.Shared;
using NUnit.Framework;

namespace Calibration.Tests
{
    public class RecordedBodySerializerTests
    {
        // ---- layout constants -------------------------------------------------

        [Test]
        public void LayoutConstants_MatchSharedLayout()
        {
            Assert.AreEqual(32, RecordedBodySerializer.JointCount);
            Assert.AreEqual(32, RecordedBodySerializer.JointRecordBytes);
            Assert.AreEqual(1032, RecordedBodySerializer.BodyRecordBytes);
            Assert.AreEqual(4, RecordedBodySerializer.HeaderBytes);

            Assert.AreEqual(K4abtWorkerSharedLayout.K4abtJointCount, RecordedBodySerializer.JointCount);
            Assert.AreEqual(K4abtWorkerSharedLayout.JointRecordBytes, RecordedBodySerializer.JointRecordBytes);
            Assert.AreEqual(K4abtWorkerSharedLayout.BodyRecordBytes, RecordedBodySerializer.BodyRecordBytes);
            Assert.AreEqual(K4ABTConsts.K4ABT_JOINT_COUNT, RecordedBodySerializer.JointCount);
        }

        [Test]
        public void FrameSize_IsHeaderPlusBodies()
        {
            Assert.AreEqual(4, RecordedBodySerializer.FrameSize(0));
            Assert.AreEqual(4 + 1032, RecordedBodySerializer.FrameSize(1));
            Assert.AreEqual(4 + 2 * 1032, RecordedBodySerializer.FrameSize(2));
            Assert.AreEqual(4, RecordedBodySerializer.FrameSize(-1)); // clamped
        }

        // ---- round trip -------------------------------------------------------

        [Test]
        public void EncodeDecode_RoundTripsAllFields()
        {
            var bodies = new[] { MakeBody(7u, seed: 100), MakeBody(42u, seed: 5000) };
            var scratch = new byte[RecordedBodySerializer.FrameSize(bodies.Length)];
            int written = RecordedBodySerializer.Encode(bodies, bodies.Length, scratch);
            Assert.AreEqual(scratch.Length, written);

            var decoded = new[] { new BodySnapshot(), new BodySnapshot() };
            int count = RecordedBodySerializer.Decode(scratch, written, decoded);
            Assert.AreEqual(2, count);

            for (int i = 0; i < 2; i++)
            {
                Assert.AreEqual(bodies[i].Id, decoded[i].Id, $"body {i} id");
                for (int j = 0; j < RecordedBodySerializer.JointCount; j++)
                {
                    var a = bodies[i].Joints[j];
                    var b = decoded[i].Joints[j];
                    Assert.AreEqual(a.Position.X, b.Position.X, $"body {i} joint {j} pos.X");
                    Assert.AreEqual(a.Position.Y, b.Position.Y, $"body {i} joint {j} pos.Y");
                    Assert.AreEqual(a.Position.Z, b.Position.Z, $"body {i} joint {j} pos.Z");
                    Assert.AreEqual(a.Orientation.W, b.Orientation.W, $"body {i} joint {j} quat.W");
                    Assert.AreEqual(a.Orientation.X, b.Orientation.X, $"body {i} joint {j} quat.X");
                    Assert.AreEqual(a.Orientation.Y, b.Orientation.Y, $"body {i} joint {j} quat.Y");
                    Assert.AreEqual(a.Orientation.Z, b.Orientation.Z, $"body {i} joint {j} quat.Z");
                    Assert.AreEqual(a.ConfidenceLevel, b.ConfidenceLevel, $"body {i} joint {j} conf");
                }
            }
        }

        [Test]
        public void EncodeDecode_RoundTripsNonFiniteFloats()
        {
            // NaN / infinity must survive bit-exactly (EMA seeds and confidence
            // gating downstream rely on bit patterns, not value comparison).
            var body = MakeBody(1u, seed: 0);
            var jt = body.Joints[0];
            jt.Position.X = float.NaN;
            jt.Position.Y = float.PositiveInfinity;
            jt.Position.Z = float.NegativeInfinity;
            body.Joints[0] = jt;

            var scratch = new byte[RecordedBodySerializer.FrameSize(1)];
            RecordedBodySerializer.Encode(new[] { body }, 1, scratch);
            var decoded = new[] { new BodySnapshot() };
            RecordedBodySerializer.Decode(scratch, scratch.Length, decoded);

            Assert.IsTrue(float.IsNaN(decoded[0].Joints[0].Position.X));
            Assert.IsTrue(float.IsPositiveInfinity(decoded[0].Joints[0].Position.Y));
            Assert.IsTrue(float.IsNegativeInfinity(decoded[0].Joints[0].Position.Z));
        }

        // ---- golden bytes -----------------------------------------------------

        [Test]
        public void Encode_GoldenBytes_PinsLayout()
        {
            // One body, id 0x0A0B0C0D. Joint 0 carries hand-picked float bit
            // patterns; joint 1 carries a distinct confidence so the +32 stride
            // is observable. All remaining joints are zero.
            var body = new BodySnapshot { Id = 0x0A0B0C0Du };
            body.Joints[0] = new k4abt_joint_t
            {
                Position = new k4a_float3_t { X = 1.5f, Y = -2.0f, Z = 0.25f },
                Orientation = new k4a_quaternion_t { W = 1.0f, X = -1.0f, Y = 2.0f, Z = -0.5f },
                ConfidenceLevel = (k4abt_joint_confidence_level_t)3,
            };
            body.Joints[1] = new k4abt_joint_t
            {
                ConfidenceLevel = (k4abt_joint_confidence_level_t)2,
            };

            var bytes = new byte[RecordedBodySerializer.FrameSize(1)];
            int written = RecordedBodySerializer.Encode(new[] { body }, 1, bytes);
            Assert.AreEqual(4 + 1032, written);

            // Header: u32 bodyCount = 1, little-endian.
            AssertBytesAt(bytes, 0, new byte[] { 0x01, 0x00, 0x00, 0x00 }, "header bodyCount");
            // Body id at +4, little-endian.
            AssertBytesAt(bytes, 4, new byte[] { 0x0D, 0x0C, 0x0B, 0x0A }, "body id");
            // Reserved u32 at +8 is zero.
            AssertBytesAt(bytes, 8, new byte[] { 0x00, 0x00, 0x00, 0x00 }, "reserved");

            // Joint 0 record starts at +12. IEEE-754 single, little-endian:
            //   1.5f  = 0x3FC00000, -2.0f = 0xC0000000, 0.25f = 0x3E800000
            //   1.0f  = 0x3F800000, -1.0f = 0xBF800000, 2.0f  = 0x40000000, -0.5f = 0xBF000000
            AssertBytesAt(bytes, 12, new byte[] { 0x00, 0x00, 0xC0, 0x3F }, "joint0 pos.X (1.5f)");
            AssertBytesAt(bytes, 16, new byte[] { 0x00, 0x00, 0x00, 0xC0 }, "joint0 pos.Y (-2.0f)");
            AssertBytesAt(bytes, 20, new byte[] { 0x00, 0x00, 0x80, 0x3E }, "joint0 pos.Z (0.25f)");
            AssertBytesAt(bytes, 24, new byte[] { 0x00, 0x00, 0x80, 0x3F }, "joint0 quat.W (1.0f)");
            AssertBytesAt(bytes, 28, new byte[] { 0x00, 0x00, 0x80, 0xBF }, "joint0 quat.X (-1.0f)");
            AssertBytesAt(bytes, 32, new byte[] { 0x00, 0x00, 0x00, 0x40 }, "joint0 quat.Y (2.0f)");
            AssertBytesAt(bytes, 36, new byte[] { 0x00, 0x00, 0x00, 0xBF }, "joint0 quat.Z (-0.5f)");
            AssertBytesAt(bytes, 40, new byte[] { 0x03, 0x00, 0x00, 0x00 }, "joint0 confidence");

            // Joint 1 record starts at +12 + 32; confidence sits at its +28.
            AssertBytesAt(bytes, 12 + 32 + 28, new byte[] { 0x02, 0x00, 0x00, 0x00 }, "joint1 confidence");

            // Everything after joint 1's record is zero (joints 2..31 untouched).
            for (int off = 12 + 2 * 32; off < written; off++)
                Assert.AreEqual(0, bytes[off], $"expected zero at offset {off}");
        }

        // ---- defensive decode -------------------------------------------------

        [Test]
        public void Decode_TruncatedPayload_ReturnsZero()
        {
            var body = MakeBody(1u, seed: 1);
            var scratch = new byte[RecordedBodySerializer.FrameSize(1)];
            RecordedBodySerializer.Encode(new[] { body }, 1, scratch);

            var outBuf = new[] { new BodySnapshot() };
            Assert.AreEqual(0, RecordedBodySerializer.Decode(scratch, scratch.Length - 1, outBuf));
            Assert.AreEqual(0, RecordedBodySerializer.Decode(scratch, 3, outBuf));
            Assert.AreEqual(0, RecordedBodySerializer.Decode(scratch, 0, outBuf));
        }

        [Test]
        public void Decode_MoreBodiesThanOutBuffer_ClampsToCapacity()
        {
            var bodies = new[] { MakeBody(1u, seed: 10), MakeBody(2u, seed: 20), MakeBody(3u, seed: 30) };
            var scratch = new byte[RecordedBodySerializer.FrameSize(3)];
            RecordedBodySerializer.Encode(bodies, 3, scratch);

            var outBuf = new[] { new BodySnapshot(), new BodySnapshot() };
            int count = RecordedBodySerializer.Decode(scratch, scratch.Length, outBuf);
            Assert.AreEqual(2, count);
            Assert.AreEqual(1u, outBuf[0].Id);
            Assert.AreEqual(2u, outBuf[1].Id);
        }

        [Test]
        public void Encode_ScratchTooSmall_Throws()
        {
            var bodies = new[] { MakeBody(1u, seed: 1) };
            var scratch = new byte[RecordedBodySerializer.FrameSize(1) - 1];
            Assert.Throws<ArgumentException>(() => RecordedBodySerializer.Encode(bodies, 1, scratch));
        }

        // ---- helpers ----------------------------------------------------------

        private static BodySnapshot MakeBody(uint id, int seed)
        {
            var body = new BodySnapshot { Id = id };
            for (int j = 0; j < RecordedBodySerializer.JointCount; j++)
            {
                float f = seed + j * 8;
                body.Joints[j] = new k4abt_joint_t
                {
                    Position = new k4a_float3_t { X = f + 0.125f, Y = -(f + 1.25f), Z = f + 2.5f },
                    Orientation = new k4a_quaternion_t { W = f + 3.0f, X = f + 4.0f, Y = -(f + 5.0f), Z = f + 6.0f },
                    ConfidenceLevel = (k4abt_joint_confidence_level_t)(j % 4),
                };
            }
            return body;
        }

        private static void AssertBytesAt(byte[] buf, int offset, byte[] expected, string what)
        {
            for (int i = 0; i < expected.Length; i++)
                Assert.AreEqual(expected[i], buf[offset + i], $"{what}: byte {i} at offset {offset + i}");
        }
    }
}
