// Live diagnostic view for display2: a 2x2 grid of the four cameras' colour
// images, each with its OWN RTMPose 2D detection (the raw, pre-fusion per-camera
// skeleton) drawn on top. Purpose: verify live v11 tracking per camera — you can
// see at a glance which camera sees the visitor cleanly and which is guessing.
//
// Self-contained: builds its own Canvas + four RawImages at runtime and, while
// enabled, SUPPRESSES the 3D point-cloud camera on the target display so the grid
// replaces it (restored on disable). Nothing to wire in the scene beyond adding
// this component.
//
// Data flow (all main-thread):
//  - colour bytes: PointCloudRenderer.OnRawFramesReady (copied in the callback —
//    the renderer's buffer is only valid for the call).
//  - per-camera pose: LiveFusedBodySource.TryGetPerCameraPose (camera-frame mm),
//    reprojected through the camera's colour intrinsics (pinhole), same math as
//    the offline CandOverlay.
//
// Orientation: colour bytes are top-down; bones are drawn in the same top-down
// pixel space and the RawImage uvRect flips V for display, so colour and skeleton
// always share one coordinate system (no per-frame byte flip).

using System.Collections.Generic;
using Orbbec;
using PointCloud;
using UnityEngine;
using UnityEngine.UI;

namespace BodyTracking.Eval.Rtmpose
{
    [DisallowMultipleComponent]
    public class CameraColorGridOverlay : MonoBehaviour
    {
        [Tooltip("OS display index (0 = primary/operator, 1 = display2, 2 = display3).")]
        public int targetDisplay = 1;

        [Tooltip("Per-camera 2D detection source. Auto-resolves when empty.")]
        public LiveFusedBodySource fused;

        [Tooltip("Skeleton line + joint colour.")]
        public Color boneColor = new Color(1f, 0.47f, 0.04f); // orange

        [Range(1, 8)] public int lineThickness = 3;
        [Range(2, 12)] public int jointRadius = 5;

        [Tooltip("Max colour-image redraws per second (throttles texture upload).")]
        [Range(5f, 60f)] public float maxRedrawHz = 25f;

        [Tooltip("Consume recorder PLAYBACK colour frames instead of live renderer frames " +
                 "(bone-verify review). Live frames are ignored while this is on so the grid " +
                 "shows the recorded take, not the live cameras.")]
        public bool consumePlayback = false;

        [Tooltip("Also overlay the merged / recorded-fused skeleton (reprojected per camera) " +
                 "in mergedBoneColor — the review shows this next to the per-camera 2D so " +
                 "recorded-fused vs raw-per-camera can be compared on each colour image.")]
        public bool showMergedReproj = false;

        [Tooltip("Colour for the merged / recorded-fused reprojection (showMergedReproj).")]
        public Color mergedBoneColor = new Color(0f, 0.85f, 1f); // cyan

        [Tooltip("Merged skeleton source for showMergedReproj. Auto-resolves when empty.")]
        public SkeletonMerger merger;

        // Joint validity already reflects the adapter's confThreshold (0.3), so a
        // valid joint is a confident one — no separate gate needed here.

        // EvalSkeleton 15-joint bone list (identical to CandOverlay).
        static readonly int[,] Bones = new int[,]
        {
            {0,1},{1,2},{1,3},{3,4},{4,5},{1,6},{6,7},{7,8},
            {0,9},{9,10},{10,11},{0,12},{12,13},{13,14}
        };

        sealed class Cell
        {
            public string Serial;
            public int Slot;             // 0..3 quadrant
            public RawImage Image;
            public Texture2D Tex;
            public Text Label;
            public byte[] Color;         // latest colour bytes (our own copy)
            public int W, H, Bpp;
            public ObCameraParam? Cam;
            public bool Dirty;
        }

        private readonly List<PointCloudRenderer> _subscribed = new List<PointCloudRenderer>();
        private readonly Dictionary<string, Cell> _cells = new Dictionary<string, Cell>();
        private readonly List<Cell> _order = new List<Cell>(4);
        private readonly RawImage[] _slotImages = new RawImage[4];
        private readonly Text[] _slotLabels = new Text[4];
        private Canvas _canvas;
        private RawImage _recDot;          // red REC circle (top-right), shown while recording
        private Texture2D _recDotTex;
        private SensorRecorder _recorder;
        private SensorRecorder _playbackSub;   // subscribed OnPlaybackRawFrame source
        private readonly List<Camera> _suppressed = new List<Camera>();
        private readonly Vector3[] _poseMm = new Vector3[(int)EvalJointId.Count];
        private readonly bool[] _poseValid = new bool[(int)EvalJointId.Count];
        private readonly Vector2[] _px = new Vector2[(int)EvalJointId.Count];
        private readonly bool[] _pxOk = new bool[(int)EvalJointId.Count];
        // merged / recorded-fused reprojection scratch (k4abt 32 -> eval 15)
        private readonly Vector3[] _mergedPos = new Vector3[K4ABTConsts.K4ABT_JOINT_COUNT];
        private readonly bool[] _mergedValid = new bool[K4ABTConsts.K4ABT_JOINT_COUNT];
        private readonly Vector3[] _mergedEvalPos = new Vector3[(int)EvalJointId.Count];
        private readonly bool[] _mergedEvalValid = new bool[(int)EvalJointId.Count];
        private float _lastRedraw;

        void OnEnable()
        {
            if (fused == null) fused = FindFirstObjectByType<LiveFusedBodySource>();
            if (_recorder == null) _recorder = FindFirstObjectByType<SensorRecorder>();
            if (merger == null) merger = FindFirstObjectByType<SkeletonMerger>();
            BuildCanvas();
            SuppressTargetDisplayCameras();
            SubscribeRenderers();
        }

        void OnDisable()
        {
            foreach (var r in _subscribed) if (r != null) r.OnRawFramesReady -= OnLiveFrame;
            _subscribed.Clear();
            if (_playbackSub != null) { _playbackSub.OnPlaybackRawFrame -= OnPlaybackColorFrame; _playbackSub = null; }

            foreach (var c in _suppressed) if (c != null) c.enabled = true;
            _suppressed.Clear();

            foreach (var c in _cells.Values) if (c.Tex != null) Destroy(c.Tex);
            _cells.Clear();
            _order.Clear();
            for (int i = 0; i < 4; i++) { _slotImages[i] = null; _slotLabels[i] = null; }
            _recDot = null;
            if (_recDotTex != null) { Destroy(_recDotTex); _recDotTex = null; }
            if (_canvas != null) { Destroy(_canvas.gameObject); _canvas = null; }
        }

        void Update()
        {
            SubscribeRenderers(); // live cameras open asynchronously

            // REC indicator: red dot in the top-right, visible only while recording.
            if (_recDot != null)
            {
                if (_recorder == null) _recorder = FindFirstObjectByType<SensorRecorder>();
                bool rec = _recorder != null && _recorder.IsRecording;
                if (_recDot.gameObject.activeSelf != rec) _recDot.gameObject.SetActive(rec);
            }

            float now = Time.realtimeSinceStartup;
            if (now - _lastRedraw < 1f / Mathf.Max(1f, maxRedrawHz)) return;
            _lastRedraw = now;

            for (int i = 0; i < _order.Count; i++)
            {
                var c = _order[i];
                if (!c.Dirty || c.Color == null || c.W <= 0 || c.H <= 0) continue;
                DrawCell(c);
                c.Dirty = false;
            }
        }

        // ---------------- canvas / camera setup ----------------

        void BuildCanvas()
        {
            var go = new GameObject($"CameraColorGrid(Display{targetDisplay + 1})");
            go.transform.SetParent(transform, false);
            _canvas = go.AddComponent<Canvas>();
            _canvas.renderMode = RenderMode.ScreenSpaceOverlay;
            _canvas.targetDisplay = targetDisplay;
            var scaler = go.AddComponent<CanvasScaler>();
            scaler.uiScaleMode = CanvasScaler.ScaleMode.ScaleWithScreenSize;
            scaler.referenceResolution = new Vector2(2560, 1440);

            // quadrant anchors: 0=TL 1=TR 2=BL 3=BR (screen space, +Y up)
            Vector2[] min = { new(0f, 0.5f), new(0.5f, 0.5f), new(0f, 0f), new(0.5f, 0f) };
            Vector2[] max = { new(0.5f, 1f), new(1f, 1f), new(0.5f, 0.5f), new(1f, 0.5f) };
            var font = Resources.GetBuiltinResource<Font>("LegacyRuntime.ttf");

            for (int i = 0; i < 4; i++)
            {
                var cellGo = new GameObject($"cell{i}");
                cellGo.transform.SetParent(_canvas.transform, false);
                var rt = cellGo.AddComponent<RectTransform>();
                rt.anchorMin = min[i]; rt.anchorMax = max[i];
                rt.offsetMin = Vector2.zero; rt.offsetMax = Vector2.zero;
                var img = cellGo.AddComponent<RawImage>();
                img.color = Color.black;                       // until a frame arrives
                img.uvRect = new Rect(0f, 1f, 1f, -1f);        // flip V (top-down bytes)
                _slotImages[i] = img;

                var labGo = new GameObject($"label{i}");
                labGo.transform.SetParent(cellGo.transform, false);
                var lrt = labGo.AddComponent<RectTransform>();
                lrt.anchorMin = new Vector2(0f, 1f); lrt.anchorMax = new Vector2(0f, 1f);
                lrt.pivot = new Vector2(0f, 1f);
                lrt.anchoredPosition = new Vector2(12f, -8f);
                lrt.sizeDelta = new Vector2(360f, 44f);
                var lab = labGo.AddComponent<Text>();
                lab.font = font; lab.fontSize = 28; lab.color = Color.white;
                lab.horizontalOverflow = HorizontalWrapMode.Overflow;
                lab.verticalOverflow = VerticalWrapMode.Overflow;
                lab.text = "";
                _slotLabels[i] = lab;
            }

            // REC indicator — red circle in the top-right corner of the whole view.
            // Added last so it draws above the cells. Hidden until recording starts.
            _recDotTex = MakeCircleTexture(64, Color.red);
            var dotGo = new GameObject("recDot");
            dotGo.transform.SetParent(_canvas.transform, false);
            var drt = dotGo.AddComponent<RectTransform>();
            drt.anchorMin = new Vector2(1f, 1f); drt.anchorMax = new Vector2(1f, 1f);
            drt.pivot = new Vector2(1f, 1f);
            drt.anchoredPosition = new Vector2(-40f, -40f);
            drt.sizeDelta = new Vector2(64f, 64f);
            _recDot = dotGo.AddComponent<RawImage>();
            _recDot.texture = _recDotTex;
            dotGo.SetActive(false);
        }

        // Solid red disc on transparent background (RGBA32), 1px soft edge.
        static Texture2D MakeCircleTexture(int size, Color col)
        {
            var tex = new Texture2D(size, size, TextureFormat.RGBA32, false);
            float r = size * 0.5f - 1f, cx = (size - 1) * 0.5f, cy = (size - 1) * 0.5f;
            var px = new Color32[size * size];
            var c = (Color32)col;
            for (int y = 0; y < size; y++)
                for (int x = 0; x < size; x++)
                {
                    float d = Mathf.Sqrt((x - cx) * (x - cx) + (y - cy) * (y - cy));
                    float a = Mathf.Clamp01(r - d + 0.5f); // 1 inside, 0 outside, AA on the 1px rim
                    px[y * size + x] = new Color32(c.r, c.g, c.b, (byte)(a * 255f));
                }
            tex.SetPixels32(px);
            tex.Apply(false);
            return tex;
        }

        void SuppressTargetDisplayCameras()
        {
            foreach (var cam in FindObjectsByType<Camera>(FindObjectsSortMode.None))
            {
                if (cam.enabled && cam.targetTexture == null && cam.targetDisplay == targetDisplay)
                {
                    cam.enabled = false;
                    _suppressed.Add(cam);
                }
            }
        }

        // ---------------- frame intake (main thread) ----------------

        void SubscribeRenderers()
        {
            foreach (var r in FindObjectsByType<PointCloudRenderer>(FindObjectsSortMode.None))
            {
                if (_subscribed.Contains(r)) continue;
                r.OnRawFramesReady += OnLiveFrame;
                _subscribed.Add(r);
            }
            if (consumePlayback && _playbackSub == null)
            {
                _playbackSub = _recorder != null ? _recorder : FindFirstObjectByType<SensorRecorder>();
                if (_playbackSub != null) _playbackSub.OnPlaybackRawFrame += OnPlaybackColorFrame;
            }
        }

        void OnLiveFrame(PointCloudRenderer r, RawFrameData f)
        {
            if (consumePlayback) return; // review mode: colour comes from the recorded take
            IngestColor(r.deviceSerial, r.CameraParam, f);
        }

        void OnPlaybackColorFrame(string serial, ObCameraParam? camParam, Transform tr, RawFrameData f)
        {
            if (!consumePlayback) return;
            IngestColor(serial, camParam, f);
        }

        void IngestColor(string serial, ObCameraParam? camParam, in RawFrameData f)
        {
            if (string.IsNullOrEmpty(serial) || f.ColorBytes == null || f.ColorByteCount <= 0) return;
            if (f.ColorWidth <= 0 || f.ColorHeight <= 0) return;

            if (!_cells.TryGetValue(serial, out var c))
            {
                if (_order.Count >= 4) return; // only four quadrants
                int slot = _order.Count;
                c = new Cell { Serial = serial, Slot = slot, Image = _slotImages[slot], Label = _slotLabels[slot] };
                _cells[serial] = c;
                _order.Add(c);
                if (c.Label != null) c.Label.text = serial;
            }

            int bpp = f.ColorByteCount / (f.ColorWidth * f.ColorHeight);
            if (bpp < 3) return; // unexpected format; skip
            c.W = f.ColorWidth; c.H = f.ColorHeight; c.Bpp = bpp;
            if (c.Color == null || c.Color.Length < f.ColorByteCount) c.Color = new byte[f.ColorByteCount];
            System.Buffer.BlockCopy(f.ColorBytes, 0, c.Color, 0, f.ColorByteCount);
            c.Cam = camParam ?? c.Cam;
            c.Dirty = true;
        }

        // ---------------- draw + upload ----------------

        void DrawCell(Cell c)
        {
            // per-camera 2D detection (orange)
            if (fused != null && c.Cam.HasValue &&
                fused.TryGetPerCameraPose(c.Serial, _poseMm, _poseValid))
                DrawSkeleton(c, _poseMm, _poseValid, boneColor);

            // merged / recorded-fused reprojection (cyan) — review mode. Mapped from
            // k4abt (32) joint order into the 15-joint eval order the Bones list uses.
            if (showMergedReproj && c.Cam.HasValue && merger != null &&
                merger.TryGetIngestedCameraFramePose(c.Serial, _mergedPos, _mergedValid))
            {
                for (int e = 0; e < _mergedEvalPos.Length; e++)
                {
                    int k = (int)EvalSkeletonMap.K4abtSource[e];
                    _mergedEvalValid[e] = _mergedValid[k];
                    _mergedEvalPos[e] = _mergedPos[k];
                }
                DrawSkeleton(c, _mergedEvalPos, _mergedEvalValid, mergedBoneColor);
            }

            if (c.Tex == null || c.Tex.width != c.W || c.Tex.height != c.H ||
                (c.Tex.format == TextureFormat.RGB24) != (c.Bpp == 3))
            {
                if (c.Tex != null) Destroy(c.Tex);
                c.Tex = new Texture2D(c.W, c.H, c.Bpp == 3 ? TextureFormat.RGB24 : TextureFormat.RGBA32, false);
                c.Image.texture = c.Tex;
                c.Image.color = Color.white;
            }
            c.Tex.LoadRawTextureData(c.Color);
            c.Tex.Apply(false);
        }

        void DrawSkeleton(Cell c, Vector3[] poseMm, bool[] poseValid, Color col)
        {
            var K = c.Cam.Value.RgbIntrinsic;
            float fx = K.Fx, fy = K.Fy, cx = K.Cx, cy = K.Cy;
            for (int j = 0; j < _px.Length; j++)
            {
                _pxOk[j] = false;
                if (!poseValid[j]) continue;
                var p = poseMm[j];
                if (p.z <= 1f) continue;
                _px[j] = new Vector2(fx * p.x / p.z + cx, fy * p.y / p.z + cy);
                _pxOk[j] = true;
            }
            byte r = (byte)(col.r * 255f), g = (byte)(col.g * 255f), b = (byte)(col.b * 255f);
            for (int e = 0; e < Bones.GetLength(0); e++)
            {
                int a = Bones[e, 0], d = Bones[e, 1];
                if (_pxOk[a] && _pxOk[d])
                    Line(c, _px[a].x, _px[a].y, _px[d].x, _px[d].y, lineThickness, r, g, b);
            }
            for (int j = 0; j < _px.Length; j++)
                if (_pxOk[j]) Dot(c, (int)_px[j].x, (int)_px[j].y, jointRadius, r, g, b);
        }

        void Put(Cell c, int x, int y, byte r, byte g, byte b)
        {
            if ((uint)x >= (uint)c.W || (uint)y >= (uint)c.H) return;
            int idx = (y * c.W + x) * c.Bpp;
            c.Color[idx] = r; c.Color[idx + 1] = g; c.Color[idx + 2] = b;
        }

        void Dot(Cell c, int x, int y, int rad, byte r, byte g, byte b)
        {
            for (int yy = -rad; yy <= rad; yy++)
                for (int xx = -rad; xx <= rad; xx++)
                    if (xx * xx + yy * yy <= rad * rad) Put(c, x + xx, y + yy, r, g, b);
        }

        void Line(Cell c, float x0, float y0, float x1, float y1, int th, byte r, byte g, byte b)
        {
            float dx = x1 - x0, dy = y1 - y0;
            int n = Mathf.Max(1, (int)Mathf.Sqrt(dx * dx + dy * dy));
            int half = Mathf.Max(0, th / 2);
            for (int i = 0; i <= n; i++)
            {
                float t = (float)i / n;
                Dot(c, (int)(x0 + dx * t), (int)(y0 + dy * t), half, r, g, b);
            }
        }
    }
}
