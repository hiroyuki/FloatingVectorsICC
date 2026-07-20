// UGUI vector overlay for the external calibration displays: draws the latest
// detection pass's marker outlines (cyan) and interpolated ChArUco corners
// (yellow) over the raw color feed without baking pixels into it — the feed
// stays at frame rate while the overlay updates at the detector's round-robin
// cadence. Coordinates come in as image pixels (top-left origin) and are
// mapped into this Graphic's rect, which must cover the fitted RawImage.

using UnityEngine;
using UnityEngine.UI;

namespace Calibration.RuntimeUI
{
    // RequireComponent is explicit: AddComponent on a GO built from
    // new GameObject(typeof(RectTransform)) did NOT inherit Graphic's own
    // CanvasRenderer requirement, and a Graphic without a CanvasRenderer
    // silently renders nothing.
    [RequireComponent(typeof(CanvasRenderer))]
    public class DetectionOverlay : MaskableGraphic
    {
        private static readonly Color32 MarkerColor = new Color32(0, 255, 255, 255);
        private static readonly Color32 CornerColor = new Color32(255, 255, 0, 255);

        private float[][] _quads;
        private float[] _corners;
        private int _imgW;
        private int _imgH;

        public void SetDetections(float[][] markerQuads, float[] cornerPoints, int imageWidth, int imageHeight)
        {
            _quads = markerQuads;
            _corners = cornerPoints;
            _imgW = imageWidth;
            _imgH = imageHeight;
            SetVerticesDirty();
        }

        protected override void OnPopulateMesh(VertexHelper vh)
        {
            vh.Clear();
            if (_imgW <= 0 || _imgH <= 0) return;

            var rect = GetPixelAdjustedRect();
            // The RawImage's texture rows were flipped on upload so it displays
            // upright; image y therefore maps to rect top-down.
            float lw = Mathf.Max(1.5f, rect.width / 640f);

            if (_quads != null)
            {
                foreach (var q in _quads)
                {
                    if (q == null || q.Length < 8) continue;
                    for (int i = 0; i < 4; i++)
                    {
                        int j = (i + 1) % 4;
                        AddLine(vh, Map(rect, q[i * 2], q[i * 2 + 1]),
                                    Map(rect, q[j * 2], q[j * 2 + 1]), lw, MarkerColor);
                    }
                }
            }

            if (_corners != null)
            {
                float s = lw * 2f;
                for (int i = 0; i + 1 < _corners.Length; i += 2)
                {
                    var c = Map(rect, _corners[i], _corners[i + 1]);
                    AddQuad(vh,
                        new Vector2(c.x - s, c.y - s), new Vector2(c.x + s, c.y - s),
                        new Vector2(c.x + s, c.y + s), new Vector2(c.x - s, c.y + s),
                        CornerColor);
                }
            }
        }

        private Vector2 Map(Rect rect, float x, float y) => new Vector2(
            rect.xMin + x / _imgW * rect.width,
            rect.yMin + (1f - y / _imgH) * rect.height);

        private static void AddLine(VertexHelper vh, Vector2 a, Vector2 b, float width, Color32 color)
        {
            Vector2 dir = b - a;
            if (dir.sqrMagnitude < 1e-6f) return;
            Vector2 n = new Vector2(-dir.y, dir.x).normalized * (width * 0.5f);
            AddQuad(vh, a - n, b - n, b + n, a + n, color);
        }

        private static void AddQuad(VertexHelper vh, Vector2 p0, Vector2 p1, Vector2 p2, Vector2 p3, Color32 color)
        {
            int start = vh.currentVertCount;
            var v = UIVertex.simpleVert;
            v.color = color;
            v.position = p0; vh.AddVert(v);
            v.position = p1; vh.AddVert(v);
            v.position = p2; vh.AddVert(v);
            v.position = p3; vh.AddVert(v);
            vh.AddTriangle(start, start + 1, start + 2);
            vh.AddTriangle(start, start + 2, start + 3);
        }
    }
}
