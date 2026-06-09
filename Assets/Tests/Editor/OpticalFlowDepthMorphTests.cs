using NUnit.Framework;
using TSDF;

namespace Calibration.Tests
{
    public class OpticalFlowDepthMorphTests
    {
        private static byte[] MakeDepthBytes(int w, int h, int x0, int x1, ushort z)
        {
            var d = new ushort[w * h];
            for (int y = 0; y < h; y++)
            {
                int row = y * w;
                for (int x = x0; x <= x1; x++)
                    d[row + x] = z;
            }
            var bytes = new byte[w * h * 2];
            System.Buffer.BlockCopy(d, 0, bytes, 0, bytes.Length);
            return bytes;
        }

        private static ushort[] ToU16(byte[] bytes)
        {
            var v = new ushort[bytes.Length / 2];
            System.Buffer.BlockCopy(bytes, 0, v, 0, bytes.Length);
            return v;
        }

        [Test]
        public void WarpedIntermediate_ShiftsTowardB()
        {
            // A: vertical band at x=2..3, B: same band shifted right to x=4..5.
            int w = 8, h = 6;
            byte[] a = MakeDepthBytes(w, h, x0: 2, x1: 3, z: 1000);
            byte[] b = MakeDepthBytes(w, h, x0: 4, x1: 5, z: 1000);

            bool ok = OpticalFlowDepthMorph.TryBuildWarpedDepthFrame(
                a, a.Length, b, b.Length, w, h,
                t: 0.5f, windowRadius: 1, regularization: 1e-3f,
                out var mid);

            Assert.IsTrue(ok);
            var m = ToU16(mid);
            // Midpoint should include occupied pixels around the halfway shift.
            for (int y = 0; y < h; y++)
            {
                int row = y * w;
                Assert.Greater(m[row + 3], 0, $"row {y} x=3");
                Assert.Greater(m[row + 4], 0, $"row {y} x=4");
            }
        }

        [Test]
        public void InvalidInput_ReturnsFalse()
        {
            bool ok = OpticalFlowDepthMorph.TryBuildWarpedDepthFrame(
                depthABytes: null, depthAByteCount: 0,
                depthBBytes: null, depthBByteCount: 0,
                width: 4, height: 4, t: 0.5f,
                windowRadius: 1, regularization: 0f,
                out _);

            Assert.IsFalse(ok);
        }
    }
}
