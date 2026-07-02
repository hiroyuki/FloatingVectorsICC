// Debug overlay: shows the color + depth image of every connected camera (live
// or playback) as an on-screen grid. Toggle with a key (default backquote `).
// (F1 is taken by macOS system keys, so backquote is the cross-platform default.)
//
// Subscribes to PointCloudRenderer.OnRawFramesReady (live, keyed by deviceSerial)
// and SensorRecorder.OnPlaybackRawFrame (playback, keyed by serial). Both fire
// on the main thread, so texture uploads here are safe. Raw byte arrays are pooled
// and reused next frame — we copy them straight into Texture2Ds during the callback.

using System.Collections.Generic;
using Orbbec;
using UnityEngine;

namespace PointCloud
{
    public class MultiCameraDebugView : MonoBehaviour
    {
        [Header("Toggle")]
        [Tooltip("Key that shows/hides the overlay at runtime.")]
        public KeyCode toggleKey = KeyCode.BackQuote;
        [Tooltip("Whether the overlay is visible when play starts.")]
        public bool showOnStart = true;

        [Header("Depth visualization")]
        [Tooltip("Depth (mm) mapped to grayscale: near = bright, far = dark, 0/invalid = black. " +
                 "This is the distance shown as full white.")]
        public float depthNearMm = 300f;
        [Tooltip("Depth (mm) shown as black (and anything beyond).")]
        public float depthFarMm = 4000f;

        [Header("Layout")]
        [Tooltip("Width in pixels of each image tile. Height follows the source aspect ratio.")]
        public int tileWidth = 320;
        [Tooltip("Pixels between the screen edge / panel and the tiles.")]
        public float margin = 8f;
        [Tooltip("Pixels between the color and depth tiles / rows.")]
        public float gap = 6f;
        [Tooltip("Flip tiles vertically. Turn on if the images appear upside down.")]
        public bool flipVertical = false;

        private class CamTextures
        {
            public Texture2D Color;
            public Texture2D Depth;
            public byte[] DepthScratch;
        }

        private bool _visible;
        private readonly Dictionary<string, CamTextures> _cams = new Dictionary<string, CamTextures>();
        private readonly HashSet<PointCloudRenderer> _subRenderers = new HashSet<PointCloudRenderer>();
        private readonly HashSet<SensorRecorder> _subRecorders = new HashSet<SensorRecorder>();
        private GUIStyle _labelStyle;
        private float _nextScan;

        private void OnEnable()
        {
            _visible = showOnStart;
            ScanAndSubscribe();
        }

        private void OnDisable()
        {
            UnsubscribeAll();
            foreach (var c in _cams.Values)
            {
                if (c.Color != null) Destroy(c.Color);
                if (c.Depth != null) Destroy(c.Depth);
            }
            _cams.Clear();
        }

        private void Update()
        {
            if (Input.GetKeyDown(toggleKey)) _visible = !_visible;

            // Renderers are spawned by SensorManager on Start, and playback
            // recorders may appear/disappear, so re-scan periodically for new sources.
            if (Time.unscaledTime >= _nextScan)
            {
                _nextScan = Time.unscaledTime + 1f;
                ScanAndSubscribe();
            }
        }

        private void ScanAndSubscribe()
        {
            var renderers = FindObjectsByType<PointCloudRenderer>(FindObjectsSortMode.None);
            foreach (var r in renderers)
            {
                if (r == null || _subRenderers.Contains(r)) continue;
                r.OnRawFramesReady += OnLiveFrame;
                _subRenderers.Add(r);
            }

            var recorders = FindObjectsByType<SensorRecorder>(FindObjectsSortMode.None);
            foreach (var rec in recorders)
            {
                if (rec == null || _subRecorders.Contains(rec)) continue;
                rec.OnPlaybackRawFrame += OnPlaybackFrame;
                _subRecorders.Add(rec);
            }
        }

        private void UnsubscribeAll()
        {
            foreach (var r in _subRenderers)
                if (r != null) r.OnRawFramesReady -= OnLiveFrame;
            _subRenderers.Clear();

            foreach (var rec in _subRecorders)
                if (rec != null) rec.OnPlaybackRawFrame -= OnPlaybackFrame;
            _subRecorders.Clear();
        }

        private void OnLiveFrame(PointCloudRenderer renderer, RawFrameData frame)
        {
            if (renderer == null || string.IsNullOrEmpty(renderer.deviceSerial)) return;
            UpdateCam(renderer.deviceSerial, frame);
        }

        private void OnPlaybackFrame(string serial, ObCameraParam? cameraParam, Transform source, RawFrameData frame)
        {
            if (string.IsNullOrEmpty(serial)) return;
            UpdateCam(serial, frame);
        }

        private void UpdateCam(string serial, RawFrameData frame)
        {
            // Skip the texture work entirely while hidden — events keep firing every frame.
            if (!_visible) return;

            if (!_cams.TryGetValue(serial, out var cam))
            {
                cam = new CamTextures();
                _cams[serial] = cam;
            }

            UpdateColor(cam, frame);
            UpdateDepth(cam, frame);
        }

        private void UpdateColor(CamTextures cam, RawFrameData frame)
        {
            int w = frame.ColorWidth, h = frame.ColorHeight;
            if (w <= 0 || h <= 0 || frame.ColorByteCount < w * h * 3) return;

            EnsureTex(ref cam.Color, w, h, TextureFormat.RGB24);
            // ColorBytes is RGB8 row-major (MJPG already decoded upstream). The pooled
            // array may be larger than w*h*3; SetPixelData copies only the mip size.
            cam.Color.SetPixelData(frame.ColorBytes, 0);
            cam.Color.Apply(false, false);
        }

        private void UpdateDepth(CamTextures cam, RawFrameData frame)
        {
            int w = frame.DepthWidth, h = frame.DepthHeight;
            int n = w * h;
            if (w <= 0 || h <= 0 || frame.DepthByteCount < n * 2) return;

            if (cam.DepthScratch == null || cam.DepthScratch.Length < n * 3)
                cam.DepthScratch = new byte[n * 3];

            var src = frame.DepthBytes;
            var dst = cam.DepthScratch;
            float near = depthNearMm;
            float far = Mathf.Max(depthNearMm + 1f, depthFarMm);
            float scale = 255f / (far - near);

            for (int i = 0; i < n; i++)
            {
                int d = src[i * 2] | (src[i * 2 + 1] << 8); // Y16 little-endian, millimeters
                byte v;
                if (d <= 0)
                {
                    v = 0; // invalid / no return
                }
                else
                {
                    float g = 255f - (d - near) * scale; // near -> 255, far -> 0
                    v = (byte)Mathf.Clamp(g, 0f, 255f);
                }
                int o = i * 3;
                dst[o] = v;
                dst[o + 1] = v;
                dst[o + 2] = v;
            }

            EnsureTex(ref cam.Depth, w, h, TextureFormat.RGB24);
            cam.Depth.SetPixelData(dst, 0);
            cam.Depth.Apply(false, false);
        }

        private static void EnsureTex(ref Texture2D tex, int w, int h, TextureFormat format)
        {
            if (tex != null && (tex.width != w || tex.height != h || tex.format != format))
            {
                Destroy(tex);
                tex = null;
            }
            if (tex == null)
            {
                tex = new Texture2D(w, h, format, false) { filterMode = FilterMode.Bilinear };
            }
        }

        private void OnGUI()
        {
            if (!_visible) return;

            if (_labelStyle == null)
            {
                _labelStyle = new GUIStyle(GUI.skin.label) { fontSize = 12 };
                _labelStyle.normal.textColor = Color.white;
            }

            float y = margin;
            GUI.Label(new Rect(margin, y, 700, 18),
                $"[{toggleKey}] camera debug   cams:{_cams.Count}   depth {depthNearMm:0}-{depthFarMm:0}mm (near=white)",
                _labelStyle);
            y += 20f;

            foreach (var kv in _cams)
            {
                var cam = kv.Value;
                string tag = ShortSerial(kv.Key);
                var aspectTex = cam.Color != null ? cam.Color : cam.Depth;
                float th = aspectTex != null
                    ? tileWidth * (float)aspectTex.height / aspectTex.width
                    : tileWidth * 0.5625f;

                float x2 = margin + tileWidth + gap;
                GUI.Label(new Rect(margin, y, tileWidth, 16f), tag + "  color", _labelStyle);
                GUI.Label(new Rect(x2, y, tileWidth, 16f), tag + "  depth", _labelStyle);
                float ty = y + 16f;

                DrawTile(new Rect(margin, ty, tileWidth, th), cam.Color);
                DrawTile(new Rect(x2, ty, tileWidth, th), cam.Depth);

                y = ty + th + gap;
            }
        }

        private void DrawTile(Rect rect, Texture2D tex)
        {
            // Dark backing so empty / black-heavy tiles stay visible against the scene.
            var prev = GUI.color;
            GUI.color = new Color(0f, 0f, 0f, 0.6f);
            GUI.DrawTexture(rect, Texture2D.whiteTexture);
            GUI.color = prev;

            if (tex == null) return;

            if (flipVertical)
                GUI.DrawTextureWithTexCoords(rect, tex, new Rect(0f, 1f, 1f, -1f));
            else
                GUI.DrawTexture(rect, tex);
        }

        private static string ShortSerial(string serial)
        {
            if (string.IsNullOrEmpty(serial)) return "?";
            return serial.Length > 4 ? serial.Substring(serial.Length - 4) : serial;
        }
    }
}
