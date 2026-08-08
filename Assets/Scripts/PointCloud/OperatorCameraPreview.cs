// Live preview on the operator display (Display 1) of what the visitor is seeing,
// so the show can be run from the operator PC without turning to the stage monitor.
//
// Default source (mirrorDisplay2): a FULL-SCREEN MIRROR of Display 2 (the visitor
// screen). The Display-2 camera is cloned into a 1080p RenderTexture each frame and
// stretched across the whole display at the bottom sorting order, so the operator UI
// (the IMGUI HUD, which always draws above uGUI) layers on top of it. Caveat: only
// what a CAMERA renders is mirrored — ScreenSpaceOverlay visitor UI (QR code,
// messages) is composited per-display by Unity and is NOT captured.
//
// Fallback source (mirrorDisplay2 == false): a small top-right corner preview of the
// raw colour feed of ONE sensor camera (cameraIndex), the same per-camera check the
// four-camera diagnostic grid on the visitor displays (CameraColorGridOverlay) does.
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
using UnityEngine.Rendering;
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

        [Tooltip("Mirror what Display 2 (the visitor screen) renders instead of a raw " +
                 "sensor colour feed. Clones the Display-2 camera into a render texture; " +
                 "ScreenSpaceOverlay visitor UI (QR / messages) is NOT captured.")]
        public bool mirrorDisplay2 = true;

        [Tooltip("Unity targetDisplay index (0-based) of the display to mirror. 1 = Display 2.")]
        public int mirrorSourceDisplay = 1;

        [Tooltip("Fallback only (mirrorDisplay2 == false): which camera to preview, by " +
                 "CAMERA ID (calibration/cameras.yaml order, the same id 0..3 used across " +
                 "the system). Not a serial — serials are machine-local, never committed.")]
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

        // Display-2 mirror
        private Camera _mirrorSrc;
        private Camera _mirrorCam;
        private RenderTexture _mirrorRt;
        private bool _mirrorHorizontally;   // does the source flip its projection X?
        private bool _renderHooked;

        void OnEnable()
        {
            if (_mgr == null) _mgr = FindFirstObjectByType<SensorManager>();
            if (_recorder == null) _recorder = FindFirstObjectByType<SensorRecorder>();
            BuildCanvas();
            // Raw sensor feed is the fallback source only; in mirror mode nothing
            // subscribes to camera frames — the panel shows the Display-2 render texture.
            if (!mirrorDisplay2)
            {
                ResolveTargetSerial();
                SubscribeSources();
            }
            else if (!_renderHooked)
            {
                // The mirrored display flips its projection X, which reverses triangle
                // winding; GL.invertCulling must be toggled around the clone's render
                // to keep solids from turning inside out (same trick MirrorDisplayCamera
                // uses on the real display — which we cannot reference across asmdefs).
                RenderPipelineManager.beginCameraRendering += OnBeginCameraRendering;
                RenderPipelineManager.endCameraRendering += OnEndCameraRendering;
                _renderHooked = true;
            }
        }

        void OnDisable()
        {
            foreach (var r in _subscribed) if (r != null) r.OnRawFramesReady -= OnLiveFrame;
            _subscribed.Clear();
            if (_playbackSub != null) { _playbackSub.OnPlaybackRawFrame -= OnPlaybackFrame; _playbackSub = null; }
            if (_tex != null) { Destroy(_tex); _tex = null; }
            if (_renderHooked)
            {
                RenderPipelineManager.beginCameraRendering -= OnBeginCameraRendering;
                RenderPipelineManager.endCameraRendering -= OnEndCameraRendering;
                _renderHooked = false;
            }
            GL.invertCulling = false;
            if (_mirrorCam != null) { Destroy(_mirrorCam.gameObject); _mirrorCam = null; }
            if (_mirrorRt != null) { _mirrorRt.Release(); Destroy(_mirrorRt); _mirrorRt = null; }
            _mirrorSrc = null;
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

            if (mirrorDisplay2)
            {
                if (visible) UpdateMirror();
                return; // raw-sensor path is bypassed while mirroring Display 2
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

        // ---------------- Display-2 mirror ----------------

        // Clone the camera that draws the mirrored display into a render texture and
        // show it in the panel. CopyFrom every frame tracks the source's viewpoint and
        // projection (the stage cameras orbit), so the mirror stays in lock-step.
        void UpdateMirror()
        {
            if (_mirrorSrc == null || !_mirrorSrc.isActiveAndEnabled ||
                _mirrorSrc.targetDisplay != mirrorSourceDisplay)
                _mirrorSrc = FindCameraForDisplay(mirrorSourceDisplay);
            if (_mirrorSrc == null) return;

            if (_mirrorRt == null)
            {
                // Full-screen background needs real resolution, not a thumbnail.
                _mirrorRt = new RenderTexture(1920, 1080, 24) { name = "OperatorDisplay2Mirror.RT" };
                _mirrorRt.Create();
            }
            if (_mirrorCam == null)
            {
                var go = new GameObject("OperatorDisplay2Mirror.Cam");
                go.transform.SetParent(transform, false);
                _mirrorCam = go.AddComponent<Camera>();
            }

            _mirrorCam.CopyFrom(_mirrorSrc);
            // CopyFrom also copied the source's targetDisplay and cleared targetTexture —
            // re-route to our RT so the clone renders off-screen, not onto Display 2.
            _mirrorCam.targetDisplay = 0;
            _mirrorCam.targetTexture = _mirrorRt;
            _mirrorCam.transform.SetPositionAndRotation(
                _mirrorSrc.transform.position, _mirrorSrc.transform.rotation);

            // Reproduce MirrorDisplayCamera on the clone: the source negates its
            // projection's X (m00 < 0 while mirrored). Detect that from the live source
            // and re-derive the flip from the clone's own fov/aspect so it does not
            // depend on whether CopyFrom carried the custom projection matrix.
            _mirrorHorizontally = _mirrorSrc.projectionMatrix.m00 < 0f;
            _mirrorCam.ResetProjectionMatrix();
            if (_mirrorHorizontally)
            {
                Matrix4x4 p = _mirrorCam.projectionMatrix;
                p.m00 = -p.m00; p.m01 = -p.m01; p.m02 = -p.m02; p.m03 = -p.m03;
                _mirrorCam.projectionMatrix = p;
            }

            if (_image != null && _image.texture != _mirrorRt)
            {
                _image.texture = _mirrorRt;
                _image.color = Color.white;
                _image.uvRect = new Rect(0f, 0f, 1f, 1f); // RT is upright; no V flip
            }
            // The panel fills the screen via anchors in mirror mode — no per-frame
            // resize (that is only for the raw corner preview's aspect fit).
            if (_label != null) _label.text = $"Display {mirrorSourceDisplay + 1}";
        }

        // First enabled camera rendering to the given targetDisplay (excluding our own
        // clone, whose targetDisplay is 0). Camera.allCameras only lists enabled ones.
        Camera FindCameraForDisplay(int disp)
        {
            foreach (var c in Camera.allCameras)
                if (c != _mirrorCam && c.targetDisplay == disp) return c;
            return null;
        }

        // The clone's X-flipped projection reverses winding — invert culling for its
        // render only, then restore, so front faces stay front (point clouds are
        // winding-agnostic, but any solid sharing the view would otherwise invert).
        void OnBeginCameraRendering(ScriptableRenderContext ctx, Camera cam)
        {
            if (_mirrorHorizontally && cam == _mirrorCam) GL.invertCulling = true;
        }

        void OnEndCameraRendering(ScriptableRenderContext ctx, Camera cam)
        {
            if (cam == _mirrorCam) GL.invertCulling = false;
        }

        // ---------------- canvas ----------------

        void BuildCanvas()
        {
            var go = new GameObject($"OperatorCameraPreview.Canvas(Display{targetDisplay + 1})");
            go.transform.SetParent(transform, false);
            _canvas = go.AddComponent<Canvas>();
            _canvas.renderMode = RenderMode.ScreenSpaceOverlay;
            _canvas.targetDisplay = targetDisplay;
            // Mirror mode paints the whole display as a BACKGROUND, so it sits at the
            // bottom sorting order — every other Display-1 overlay (FPS, alerts, boot
            // splash) draws over it, and the IMGUI HUD always draws above uGUI anyway.
            // Raw corner preview keeps its old spot just under the FPS overlay.
            _canvas.sortingOrder = mirrorDisplay2 ? short.MinValue : (short.MaxValue - 3);
            var scaler = go.AddComponent<CanvasScaler>();
            scaler.uiScaleMode = CanvasScaler.ScaleMode.ScaleWithScreenSize;
            scaler.referenceResolution = new Vector2(1920, 1080);
            scaler.matchWidthOrHeight = 0.5f;

            _panel = new GameObject("panel").AddComponent<RectTransform>();
            _panel.SetParent(_canvas.transform, false);
            if (mirrorDisplay2)
            {
                // Full-screen: stretch to every edge so Display 1 shows exactly what the
                // visitor sees on Display 2, with the operator UI layered on top.
                _panel.anchorMin = Vector2.zero;
                _panel.anchorMax = Vector2.one;
                _panel.pivot = new Vector2(0.5f, 0.5f);
                _panel.offsetMin = Vector2.zero;
                _panel.offsetMax = Vector2.zero;
            }
            else
            {
                _panel.anchorMin = _panel.anchorMax = _panel.pivot = new Vector2(1f, 1f);
                // Clear the FPS panel (top-right, ~72px tall + its 16px margin) by dropping
                // below it, keeping the same right margin.
                _panel.anchoredPosition = new Vector2(-16f, -96f);
                _panel.sizeDelta = new Vector2(width, width * 9f / 16f); // refit to real aspect on first frame
            }

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

            // Restore the raw path's vertical flip (colour bytes are top-down). Guarded
            // so a runtime switch back from mirror mode — which leaves uvRect upright —
            // is corrected without dirtying the canvas every redraw.
            var rawFlip = new Rect(0f, 1f, 1f, -1f);
            if (_image != null && _image.uvRect != rawFlip) _image.uvRect = rawFlip;

            if (_label != null)
                _label.text = string.IsNullOrEmpty(_serial) ? $"cam {cameraIndex}" : $"cam {cameraIndex}  {_serial}";
        }
    }
}
