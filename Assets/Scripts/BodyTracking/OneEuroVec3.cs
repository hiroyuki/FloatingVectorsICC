using UnityEngine;

namespace BodyTracking
{
    // 1€ low-pass for a 3D position. Cutoff = minCutoff + beta * |smoothed velocity|,
    // so the filter is heavy when the joint is near-stationary (kills camera-swap /
    // NN noise jitter) and releases as it moves fast (no visible lag).
    internal struct OneEuroVec3
    {
        public Vector3 XHat;
        public Vector3 DxHat;
        public bool HasPrev;

        public void Reset() { HasPrev = false; XHat = default; DxHat = default; }

        public Vector3 Filter(Vector3 x, float dt, float minCutoff, float beta, float derivCutoff)
        {
            if (!HasPrev || dt <= 0f)
            {
                XHat = x;
                DxHat = Vector3.zero;
                HasPrev = true;
                return x;
            }
            Vector3 dx = (x - XHat) / dt;
            float alphaD = Alpha(derivCutoff, dt);
            Vector3 dxHat = Vector3.Lerp(DxHat, dx, alphaD);
            float cutoff = minCutoff + beta * dxHat.magnitude;
            float alpha = Alpha(cutoff, dt);
            Vector3 xHat = Vector3.Lerp(XHat, x, alpha);
            XHat = xHat;
            DxHat = dxHat;
            return xHat;
        }

        private static float Alpha(float cutoff, float dt)
        {
            float tau = 1f / (2f * Mathf.PI * Mathf.Max(cutoff, 0.0001f));
            return 1f / (1f + tau / dt);
        }
    }
}
