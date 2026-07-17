// Programmatic pose-guide placeholders: a white stick figure rasterized once
// into a Texture2D (CPU distance-field over a handful of segments + a head
// circle, soft anti-aliased edge, transparent background). Used by the
// Calibrate ("この ポーズを とってね" — star pose) and BanzaiWait prompts
// until real artwork exists; ExperienceConfig.poseGuideTexture overrides it.

using UnityEngine;

namespace Experience
{
    public static class StickFigureTexture
    {
        /// <summary>Star pose (大の字): arms out slightly raised, legs spread.</summary>
        public static Texture2D DrawStarPose(int size = 512) => Draw(size, new[]
        {
            // spine
            (new Vector2(0.50f, 0.72f), new Vector2(0.50f, 0.42f)),
            // arms — out along the shoulder line, slightly raised
            (new Vector2(0.50f, 0.68f), new Vector2(0.16f, 0.78f)),
            (new Vector2(0.50f, 0.68f), new Vector2(0.84f, 0.78f)),
            // legs — spread
            (new Vector2(0.50f, 0.42f), new Vector2(0.30f, 0.10f)),
            (new Vector2(0.50f, 0.42f), new Vector2(0.70f, 0.10f)),
        });

        /// <summary>Banzai: both arms straight up, feet together-ish.</summary>
        public static Texture2D DrawBanzaiPose(int size = 512) => Draw(size, new[]
        {
            (new Vector2(0.50f, 0.72f), new Vector2(0.50f, 0.42f)),
            (new Vector2(0.50f, 0.68f), new Vector2(0.36f, 0.95f)),
            (new Vector2(0.50f, 0.68f), new Vector2(0.64f, 0.95f)),
            (new Vector2(0.50f, 0.42f), new Vector2(0.42f, 0.10f)),
            (new Vector2(0.50f, 0.42f), new Vector2(0.58f, 0.10f)),
        });

        const float HeadCenterY = 0.82f;
        const float HeadRadius = 0.075f;
        const float LimbRadius = 0.028f;
        const float SoftEdge = 0.012f;

        static Texture2D Draw(int size, (Vector2 a, Vector2 b)[] segments)
        {
            size = Mathf.Clamp(size, 64, 2048);
            var tex = new Texture2D(size, size, TextureFormat.RGBA32, false)
            {
                name = "StickFigure",
                wrapMode = TextureWrapMode.Clamp,
            };
            var pixels = new Color32[size * size];
            var head = new Vector2(0.5f, HeadCenterY);

            for (int y = 0; y < size; y++)
            {
                float v = (y + 0.5f) / size;
                for (int x = 0; x < size; x++)
                {
                    float u = (x + 0.5f) / size;
                    var p = new Vector2(u, v);

                    // head ring distance (outline circle reads better than a disc)
                    float d = Mathf.Abs(Vector2.Distance(p, head) - HeadRadius);
                    foreach (var (a, b) in segments)
                        d = Mathf.Min(d, DistanceToSegment(p, a, b));

                    float alpha = 1f - Mathf.SmoothStep(0f, 1f,
                        Mathf.InverseLerp(LimbRadius - SoftEdge, LimbRadius + SoftEdge, d));
                    pixels[y * size + x] = new Color32(255, 255, 255, (byte)(alpha * 255f));
                }
            }
            tex.SetPixels32(pixels);
            tex.Apply(false, true); // no mips, non-readable after upload
            return tex;
        }

        static float DistanceToSegment(Vector2 p, Vector2 a, Vector2 b)
        {
            Vector2 ab = b - a;
            float t = Mathf.Clamp01(Vector2.Dot(p - a, ab) / Mathf.Max(1e-6f, ab.sqrMagnitude));
            return Vector2.Distance(p, a + t * ab);
        }
    }
}
