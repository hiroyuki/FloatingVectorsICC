using UnityEngine;

namespace TSDF
{
    /// <summary>
    /// CPU fallback used by mesh-completion C (#28): estimate a dense 2D flow field
    /// between two depth images (A->B) and synthesize an intermediate depth image at t.
    /// Endpoints are not touched; callers integrate this only for bridge frames.
    /// </summary>
    public static class OpticalFlowDepthMorph
    {
        public static bool TryBuildWarpedDepthFrame(
            byte[] depthABytes, int depthAByteCount,
            byte[] depthBBytes, int depthBByteCount,
            int width, int height,
            float t, int windowRadius, float regularization,
            out byte[] warpedDepthBytes)
        {
            warpedDepthBytes = null;
            if (depthABytes == null || depthBBytes == null) return false;
            if (depthAByteCount <= 0 || depthBByteCount <= 0) return false;
            if (width <= 2 || height <= 2) return false;
            if (depthAByteCount < width * height * 2 || depthBByteCount < width * height * 2) return false;

            int px = width * height;
            var a16 = new ushort[px];
            var b16 = new ushort[px];
            System.Buffer.BlockCopy(depthABytes, 0, a16, 0, width * height * 2);
            System.Buffer.BlockCopy(depthBBytes, 0, b16, 0, width * height * 2);

            var a = new float[px];
            var b = new float[px];
            for (int i = 0; i < px; i++)
            {
                a[i] = a16[i];
                b[i] = b16[i];
            }

            var ix = new float[px];
            var iy = new float[px];
            var it = new float[px];
            for (int y = 1; y < height - 1; y++)
            {
                int row = y * width;
                for (int x = 1; x < width - 1; x++)
                {
                    int i = row + x;
                    float cA = a[i];
                    float cB = b[i];
                    if (cA <= 0f || cB <= 0f) continue;
                    ix[i] = 0.5f * (a[i + 1] - a[i - 1]);
                    iy[i] = 0.5f * (a[i + width] - a[i - width]);
                    it[i] = cB - cA;
                }
            }

            int r = Mathf.Max(1, windowRadius);
            float reg = Mathf.Max(0f, regularization);
            var flowU = new float[px];
            var flowV = new float[px];
            for (int y = r; y < height - r; y++)
            {
                for (int x = r; x < width - r; x++)
                {
                    float sumIx2 = 0f, sumIy2 = 0f, sumIxIy = 0f;
                    float sumIxIt = 0f, sumIyIt = 0f;
                    int valid = 0;
                    for (int wy = -r; wy <= r; wy++)
                    {
                        int row = (y + wy) * width;
                        for (int wx = -r; wx <= r; wx++)
                        {
                            int i = row + (x + wx);
                            float gx = ix[i];
                            float gy = iy[i];
                            float gt = it[i];
                            if (gt == 0f && gx == 0f && gy == 0f) continue;
                            sumIx2 += gx * gx;
                            sumIy2 += gy * gy;
                            sumIxIy += gx * gy;
                            sumIxIt += gx * gt;
                            sumIyIt += gy * gt;
                            valid++;
                        }
                    }
                    if (valid == 0) continue;
                    float det = (sumIx2 + reg) * (sumIy2 + reg) - sumIxIy * sumIxIy;
                    if (Mathf.Abs(det) < 1e-6f) continue;
                    int idx = y * width + x;
                    float u = ((-sumIy2 * sumIxIt) + (sumIxIy * sumIyIt)) / det;
                    float v = ((sumIxIy * sumIxIt) - (sumIx2 * sumIyIt)) / det;
                    flowU[idx] = Mathf.Clamp(u, -32f, 32f);
                    flowV[idx] = Mathf.Clamp(v, -32f, 32f);
                }
            }

            t = Mathf.Clamp01(t);
            var out16 = new ushort[px];
            for (int y = 0; y < height; y++)
            {
                int row = y * width;
                for (int x = 0; x < width; x++)
                {
                    int i = row + x;
                    ushort zA = a16[i];
                    if (zA == 0) continue;

                    float fx = x + flowU[i] * t;
                    float fy = y + flowV[i] * t;
                    int dx = Mathf.Clamp(Mathf.RoundToInt(fx), 0, width - 1);
                    int dy = Mathf.Clamp(Mathf.RoundToInt(fy), 0, height - 1);
                    int di = dy * width + dx;

                    int bx = Mathf.Clamp(Mathf.RoundToInt(x + flowU[i]), 0, width - 1);
                    int by = Mathf.Clamp(Mathf.RoundToInt(y + flowV[i]), 0, height - 1);
                    ushort zB = b16[by * width + bx];
                    float z = zB > 0 ? Mathf.Lerp(zA, zB, t) : zA;
                    ushort zOut = (ushort)Mathf.Clamp(Mathf.RoundToInt(z), 0, ushort.MaxValue);
                    if (out16[di] == 0 || zOut < out16[di]) out16[di] = zOut;
                }
            }

            for (int i = 0; i < px; i++)
            {
                if (out16[i] != 0) continue;
                ushort zA = a16[i];
                ushort zB = b16[i];
                if (zA > 0 && zB > 0)
                {
                    out16[i] = (ushort)Mathf.Clamp(Mathf.RoundToInt(Mathf.Lerp(zA, zB, t)), 0, ushort.MaxValue);
                }
                else
                {
                    out16[i] = zA > 0 ? zA : zB;
                }
            }

            warpedDepthBytes = new byte[width * height * 2];
            System.Buffer.BlockCopy(out16, 0, warpedDepthBytes, 0, warpedDepthBytes.Length);
            return true;
        }
    }
}
