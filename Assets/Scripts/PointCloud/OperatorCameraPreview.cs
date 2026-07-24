// Small live colour preview of ONE camera in the top-right corner of the operator
// display (Display 1). Purpose: while running the show from the operator PC you can
// keep an eye on a single camera's raw colour feed (framing, exposure, whether it
// dropped out) without switching to the full four-camera diagnostic grid that lives
// on the visitor displays (CameraColorGridOverlay).
//
// Which camera: cameraIndex is the CAMERA ID (calibration/cameras.yaml order, the
// same id 0..3 the rest of the system uses — resolved via
// PointCloudRecording.ResolveRigSerialOrder), NOT the SDK enumeration order and NOT
// a serial (serials are machine-local and must not be hardcoded — see CLAUDE.md).
// cameraIndex N is therefore camera id N as assigned in cameras.yaml.
//
// Frame source, both handled (same as CameraColorGridOverlay):
//  - live:     PointCloudRenderer.OnRawFramesReady
//  - playback: SensorRecorder.OnPlaybackRawFrame
// Frames are filtered to the resolved target serial, so the two paths never fight —
// in practice only one is active (live renderers are destroyed during playback).
//
// Drawn as a uGUI Canvas, NOT IMGUI: IMGUI only reaches the primary display and
// renders under the ScreenSpaceOverlay canvases — same reasoning as FpsOverlay.
// Self-installing so it needs no scene wiring; positioned below the FPS readout so
// the two top-right overlays do not overlap. Lives in the PointCloud assembly (not
// Shared) because it consumes PointCloudRenderer, and PointCloud already references
// Shared — the reverse reference would be circular.

using System.Collections.Generic;
using Orbbec;
using UnityEngine;
using UnityEngine.UI;

namespace PointCloud
{
    [DisallowMultipleComponent]
    public class OperatorCameraPreview : MonoBehaviour
    {
        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
        private static void Install()
        {
            if (FindFirstObjectByType<OperatorCameraPreview>() != null) return;
            var go = new GameObject("[OperatorCameraPreview]");
            go.AddComponent<OperatorCameraPreview>();
            DontDestroyOnLoad(go);
        }

        [Tooltip("OS display index to draw on (0 = primary/operator display).")]
        public int targetDisplay = 0;

        [Tooltip("Which camera to preview, by CAMERA ID (calibration/cameras.yaml order, " +
                 "the same id 0..3 used across the system). Not a serial — serials are " +
                 "machine-local and must not be committed.")]
        public int cameraIndex = 3;

        [Tooltip("Show/hide the preview.")]
        public bool visible = true;

        [Tooltip("Toggle the preview.")]
        public KeyCode toggleKey = KeyCode.F3;

        [Tooltip("Preview width in reference pixels (1920x1080). Height follows the " +
                 "camera's aspect ratio.")]
        [Range(160f, 960f)] public float width = 480f;

        [Tooltip("Max colour redraws per second (throttles the texture upload).")]
        [Range(5f, 60f)] public float maxRedrawHz = 25f;

        private readonly List<PointCloudRenderer> _subscribed = new List<PointCloudRenderer>();
        private SensorManager _mgr;
        private SensorRecorder _recorder;
        private SensorRecorder _playbackSub;   // recorder we subscribed OnPlaybackRawFrame to
        private Canvas _canvas;
        private RawImage _image;
        private RectTransform _panel;
        private Text _label;
        private Texture2D _tex;

        private string _targetSerial;          // serial for camera id == cameraIndex
        private float _lastResolve;

        private byte[] _color;      // our own copy of the latest colour bytes
        private int _w, _h, _bpp;
        private string _serial;
        private bool _dirty;
        private float _lastRedraw;

        void OnEnable()
        {
            if (_mgr == null) _mgr = FindFirstObjectByType<SensorManager>();
            if (_recorder == null) _recorder = FindFirstObjectByType<SensorRecorder>();
            BuildCanvas();
            ResolveTargetSerial();
            SubscribeSources();
        }

        void OnDisable()
        {
            foreach (var r in _subscribed) if (r != null) r.OnRawFramesReady -= OnLiveFrame;
            _subscribed.Clear();
            if (_playbackSub != null) { _playbackSub.OnPlaybackRawFrame -= OnPlaybackFrame; _playbackSub = null; }
            if (_tex != null) { Destroy(_tex); _tex = null; }
            if (_canvas != null) { Destroy(_canvas.gameObject); _canvas = null; }
            _image = null; _panel = null; _label = null;
        }

        void Update()
        {
            if (Input.GetKeyDown(toggleKey))
            {
                visible = !visible;
                if (_panel != null) _panel.gameObject.SetActive(visible);
            }

            SubscribeSources(); // live cameras / recorder appear asynchronously

            // cameras.yaml (and the live-vs-playback root it is read from) settles a
            // little after start; re-resolve the target serial at low frequency until
            // and while it can change. File IO once/sec is negligible.
            if (Time.realtimeSinceStartup - _lastResolve > 1f)
            {
                _lastResolve = Time.realtimeSinceStartup;
                ResolveTargetSerial();
            }

            if (!visible) return;
            float now = Time.realtimeSinceStartup;
            if (now - _lastRedraw < 1f / Mathf.Max(1f, maxRedrawHz)) return;
            _lastRedraw = now;

            if (_dirty && _color != null && _w > 0 && _h > 0)
            {
                Redraw();
                _dirty = false;
            }
        }

        // ---------------- canvas ----------------

        void BuildCanvas()
        {
            var go = new GameObject($"OperatorCameraPreview.Canvas(Display{targetDisplay + 1})");
            go.transform.SetParent(transform, false);
            _canvas = go.AddComponent<Canvas>();
            _canvas.renderMode = RenderMode.ScreenSpaceOverlay;
            _canvas.targetDisplay = targetDisplay;
            // Just under the FPS overlay (short.MaxValue - 2) so the readout stays on top.
            _canvas.sortingOrder = short.MaxValue - 3;
            var scaler = go.AddComponent<CanvasScaler>();
            scaler.uiScaleMode = CanvasScaler.ScaleMode.ScaleWithScreenSize;
            scaler.referenceResolution = new Vector2(1920, 1080);
            scaler.matchWidthOrHeight = 0.5f;

            _panel = new GameObject("panel").AddComponent<RectTransform>();
            _panel.SetParent(_canvas.transform, false);
            _panel.anchorMin = _panel.anchorMax = _panel.pivot = new Vector2(1f, 1f);
            // Clear the FPS panel (top-right, ~72px tall + its 16px margin) by dropping
            // below it, keeping the same right margin.
            _panel.anchoredPosition = new Vector2(-16f, -96f);
            _panel.sizeDelta = new Vector2(width, width * 9f / 16f); // refit to real aspect on first frame

            var img = new GameObject("image").AddComponent<RawImage>();
            img.transform.SetParent(_panel, false);
            var irt = (RectTransform)img.transform;
            irt.anchorMin = Vector2.zero; irt.anchorMax = Vector2.one;
            irt.offsetMin = Vector2.zero; irt.offsetMax = Vector2.zero;
            img.color = Color.black;                  // until a frame arrives
            img.uvRect = new Rect(0f, 1f, 1f, -1f);   // flip V (colour bytes are top-down)
            img.raycastTarget = false;
            _image = img;

            var labGo = new GameObject("label");
            labGo.transform.SetParent(_panel, false);
            var lrt = labGo.AddComponent<RectTransform>();
            lrt.anchorMin = new Vector2(0f, 1f); lrt.anchorMax = new Vector2(0f, 1f);
            lrt.pivot = new Vector2(0f, 1f);
            lrt.anchoredPosition = new Vector2(8f, -8f);
            lrt.sizeDelta = new Vector2(width, 32f);
            _label = labGo.AddComponent<Text>();
            _label.font = Resources.GetBuiltinResource<Font>("LegacyRuntime.ttf");
            _label.fontSize = 22;
            _label.color = Color.white;
            _label.horizontalOverflow = HorizontalWrapMode.Overflow;
            _label.verticalOverflow = VerticalWrapMode.Overflow;
            _label.raycastTarget = false;
            _label.text = $"cam {cameraIndex}";

            _panel.gameObject.SetActive(visible);
        }

        // ---------------- target-serial resolution ----------------

        // Resolve which serial is camera id == cameraIndex, from cameras.yaml via the
        // same path the rest of the system uses. Live rig present -> the machine-local
        // map; pure playback -> the take's own map (so cross-set takes keep their rig).
        void ResolveTargetSerial()
        {
            if (_mgr == null) _mgr = FindFirstObjectByType<SensorManager>();
            if (_recorder == null) _recorder = FindFirstObjectByType<SensorRecorder>();

            bool liveRig = _mgr != null && _mgr.Renderers != null && _mgr.Renderers.Count > 0;
            string root = liveRig ? _mgr.ResolveExtrinsicsRoot()
                : _recorder != null ? _recorder.ResolvePlaybackRoot()
                : _mgr != null ? _mgr.ResolveExtrinsicsRoot()
                : null;
            if (string.IsNullOrEmpty(root)) return;

            var order = PointCloudRecording.ResolveRigSerialOrder(root, null, out _);
            if (order == null || cameraIndex < 0 || cameraIndex >= order.Length) return;
            _targetSerial = order[cameraIndex];
        }

        // ---------------- frame intake (main thread) ----------------

        void SubscribeSources()
        {
            foreach (var r in FindObjectsByType<PointCloudRenderer>(FindObjectsSortMode.None))
            {
                if (_subscribed.Contains(r)) continue;
                r.OnRawFramesReady += OnLiveFrame;
                _subscribed.Add(r);
            }
            if (_playbackSub == null)
            {
                if (_recorder == null) _recorder = FindFirstObjectByType<SensorRecorder>();
                if (_recorder != null) { _recorder.OnPlaybackRawFrame += OnPlaybackFrame; _playbackSub = _recorder; }
            }
        }

        void OnLiveFrame(PointCloudRenderer r, RawFrameData f)
        {
            if (r != null) Ingest(r.deviceSerial, f);
        }

        void OnPlaybackFrame(string serial, ObCameraParam? camParam, Transform tr, RawFrameData f)
        {
            Ingest(serial, f);
        }

        void Ingest(string serial, in RawFrameData f)
        {
            // No target yet -> accept nothing; a stray non-target camera must not paint
            // over the preview. Once resolved, only the chosen camera's frames pass.
            if (string.IsNullOrEmpty(_targetSerial) || serial != _targetSerial) return;
            if (f.ColorBytes == null || f.ColorByteCount <= 0) return;
            if (f.ColorWidth <= 0 || f.ColorHeight <= 0) return;

            int bpp = f.ColorByteCount / (f.ColorWidth * f.ColorHeight);
            if (bpp < 3) return; // unexpected format; skip

            _w = f.ColorWidth; _h = f.ColorHeight; _bpp = bpp;
            _serial = serial;
            if (_color == null || _color.Length < f.ColorByteCount) _color = new byte[f.ColorByteCount];
            System.Buffer.BlockCopy(f.ColorBytes, 0, _color, 0, f.ColorByteCount);
            _dirty = true;
        }

        // ---------------- draw + upload ----------------

        void Redraw()
        {
            // Keep the panel at the camera's real aspect ratio (colour is usually 16:9
            // but honour whatever the stream reports).
            float h = width * _h / Mathf.Max(1, _w);
            if (!Mathf.Approximately(_panel.sizeDelta.y, h))
                _panel.sizeDelta = new Vector2(width, h);

            if (_tex == null || _tex.width != _w || _tex.height != _h ||
                (_tex.format == TextureFormat.RGB24) != (_bpp == 3))
            {
                if (_tex != null) Destroy(_tex);
                _tex = new Texture2D(_w, _h, _bpp == 3 ? TextureFormat.RGB24 : TextureFormat.RGBA32, false);
                _image.texture = _tex;
                _image.color = Color.white;
            }
            _tex.LoadRawTextureData(_color);
            _tex.Apply(false);

            if (_label != null)
                _label.text = string.IsNullOrEmpty(_serial) ? $"cam {cameraIndex}" : $"cam {cameraIndex}  {_serial}";
        }
    }
}
