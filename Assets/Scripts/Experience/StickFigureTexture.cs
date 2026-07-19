// Programmatic pose-guide placeholders: a white HUMAN SILHOUETTE (pictogram
// style — detached head + filled tapered-capsule body/limbs, soft anti-
// aliased edge, transparent background) rasterized once into a Texture2D.
// Used by the Calibrate ("この ポーズを とってね" — star pose) prompt until
// real artwork exists; ExperienceConfig.poseGuideTexture overrides it. The
// banzai silhouette is kept around unused in case a raised-arms guide returns.
//
// The shape is a distance-field union of "cone capsules" (segments whose
// radius tapers from a to b) plus the head disc — the standard restroom-sign
// look, deliberately outline-only in silhouette rather than a bone skeleton.

using UnityEngine;

namespace Experience
{
    public static class StickFigureTexture
    {
        // One silhouette part: a segment with a tapered radius (cone capsule).
        private readonly struct Part
        {
            public readonly Vector2 A, B;
            public readonly float Ra, Rb;
            public Part(float ax, float ay, float bx, float by, float ra, float rb)
            { A = new Vector2(ax, ay); B = new Vector2(bx, by); Ra = ra; Rb = rb; }
        }

        /// <summary>Star pose (大の字): arms out slightly raised, legs spread.</summary>
        public static Texture2D DrawStarPose(int size = 512) => Draw(size, new[]
        {
            // torso — shoulders wider than hips
            new Part(0.50f, 0.70f, 0.50f, 0.46f, 0.095f, 0.072f),
            // arms — from the shoulder edges, slightly raised, tapering to the wrists
            new Part(0.45f, 0.68f, 0.14f, 0.80f, 0.042f, 0.026f),
            new Part(0.55f, 0.68f, 0.86f, 0.80f, 0.042f, 0.026f),
            // legs — spread wide
            new Part(0.47f, 0.47f, 0.29f, 0.06f, 0.058f, 0.032f),
            new Part(0.53f, 0.47f, 0.71f, 0.06f, 0.058f, 0.032f),
        });

        /// <summary>Banzai: both arms straight up, feet under the hips.</summary>
        public static Texture2D DrawBanzaiPose(int size = 512) => Draw(size, new[]
        {
            new Part(0.50f, 0.68f, 0.50f, 0.44f, 0.092f, 0.070f),
            // arms raised high in a V, from the shoulder edges
            new Part(0.44f, 0.65f, 0.31f, 0.96f, 0.042f, 0.026f),
            new Part(0.56f, 0.65f, 0.69f, 0.96f, 0.042f, 0.026f),
            // legs — standing, slight stance
            new Part(0.465f, 0.45f, 0.41f, 0.05f, 0.056f, 0.032f),
            new Part(0.535f, 0.45f, 0.59f, 0.05f, 0.056f, 0.032f),
        });

        const float HeadRadius = 0.078f;
        const float SoftEdge = 0.008f;

        static Texture2D Draw(int size, Part[] parts)
        {
            size = Mathf.Clamp(size, 64, 2048);
            var tex = new Texture2D(size, size, TextureFormat.RGBA32, false)
            {
                name = "PoseSilhouette",
                wrapMode = TextureWrapMode.Clamp,
            };
            var pixels = new Color32[size * size];
            // Head sits above the torso with a pictogram-style neck gap.
            var headByPose = parts[0].A + new Vector2(0f, 0.155f);

            for (int y = 0; y < size; y++)
            {
                float v = (y + 0.5f) / size;
                for (int x = 0; x < size; x++)
                {
                    float u = (x + 0.5f) / size;
                    var p = new Vector2(u, v);

                    // Signed distance to the silhouette (negative = inside).
                    float d = Vector2.Distance(p, headByPose) - HeadRadius;
                    foreach (var part in parts)
                        d = Mathf.Min(d, ConeCapsuleDistance(p, part));

                    float alpha = 1f - Mathf.SmoothStep(0f, 1f,
                        Mathf.InverseLerp(-SoftEdge, SoftEdge, d));
                    pixels[y * size + x] = new Color32(255, 255, 255, (byte)(alpha * 255f));
                }
            }
            tex.SetPixels32(pixels);
            tex.Apply(false, true); // no mips, non-readable after upload
            return tex;
        }

        // Distance to a segment whose radius tapers linearly from Ra (at A) to
        // Rb (at B); negative inside the filled shape.
        static float ConeCapsuleDistance(Vector2 p, in Part part)
        {
            Vector2 ab = part.B - part.A;
            float t = Mathf.Clamp01(Vector2.Dot(p - part.A, ab) / Mathf.Max(1e-6f, ab.sqrMagnitude));
            float radius = Mathf.Lerp(part.Ra, part.Rb, t);
            return Vector2.Distance(p, part.A + t * ab) - radius;
        }
    }
}
