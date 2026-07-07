// Watches the live camera rig and shows an on-screen alert when an expected
// camera is missing or has stopped delivering frames.
//
// Expected serials come from calibration/cameras.yaml (the same stable id map
// the calibration UI writes) or an explicit Inspector override. With no serial
// list available at all it falls back to comparing the connected renderer
// count against expectedCameraCount.
//
// Alerts are suppressed while SensorManager.playbackOnly is set and while the
// SensorRecorder is playing back a recording (playback destroys the live
// renderers on purpose — that is not a camera fault).

using System;
using System.Collections.Generic;
using UnityEngine;

namespace PointCloud
{
    [AddComponentMenu("PointCloud/Camera Health Monitor")]
    public class CameraHealthMonitor : MonoBehaviour
    {
        [Header("Expected rig")]
        [Tooltip("Serials that must be connected and streaming. Empty = read calibration/" +
                 "cameras.yaml under the SensorManager's extrinsics root; if that file is " +
                 "missing too, only the connected-count check below runs.")]
        public string[] expectedSerials = Array.Empty<string>();
        [Tooltip("Fallback: required number of connected cameras when no serial list is " +
                 "available (neither Inspector nor cameras.yaml).")]
        [Min(0)]
        public int expectedCameraCount = 4;

        [Header("Detection")]
        [Tooltip("Seconds after enable before alerts can fire — device enumeration and " +
                 "pipeline start take a while on a 4-camera rig.")]
        public float startupGraceSeconds = 15f;
        [Tooltip("A camera whose frame timestamp has not advanced for this many seconds " +
                 "is flagged as abnormal (covers mid-run USB drops where the renderer GO " +
                 "still exists but frames stopped).")]
        public float staleFrameSeconds = 3f;
        [Tooltip("Seconds between health checks.")]
        public float checkIntervalSeconds = 1f;

        [Header("Display")]
        [Tooltip("Draw the on-screen alert banner (IMGUI, same style as the crowd alert).")]
        public bool showAlert = true;
        [Tooltip("Also log alert transitions to the console.")]
        public bool logAlerts = true;

        /// <summary>Current alert lines (empty = all expected cameras healthy).</summary>
        public IReadOnlyList<string> CurrentAlerts => _alerts;

        private struct ExpectedCam
        {
            public string Serial;
            public string Label;
        }

        private class Health
        {
            public ulong LastTs;
            public float LastProgressAt;
            public bool Seen;
        }

        private SensorManager _manager;
        private SensorRecorder _recorder;
        private readonly List<ExpectedCam> _expected = new List<ExpectedCam>();
        private readonly Dictionary<string, Health> _health = new Dictionary<string, Health>();
        private readonly List<string> _alerts = new List<string>();
        private bool _yamlTried;
        private float _enabledAt;
        private float _nextCheckAt;
        private string _lastLoggedState = string.Empty;
        private GUIStyle _alertStyle;

        private void OnEnable()
        {
            _enabledAt = Time.unscaledTime;
            _nextCheckAt = 0f;
            _health.Clear();
            _alerts.Clear();
        }

        private void Update()
        {
            if (_manager == null) _manager = FindFirstObjectByType<SensorManager>();
            if (Time.unscaledTime < _nextCheckAt) return;
            _nextCheckAt = Time.unscaledTime + Mathf.Max(0.2f, checkIntervalSeconds);
            RunCheck();
        }

        private void RunCheck()
        {
            _alerts.Clear();

            if (_manager == null || _manager.playbackOnly) { LogTransition(); return; }
            if (_recorder == null) _recorder = FindFirstObjectByType<SensorRecorder>();
            if (_recorder != null && _recorder.IsPlaying)
            {
                // Live renderers are deliberately torn down for playback; forget the
                // per-camera progress so nothing is flagged stale when live resumes.
                _health.Clear();
                LogTransition();
                return;
            }
            if (Time.unscaledTime - _enabledAt < startupGraceSeconds) return;

            ResolveExpected();
            var renderers = _manager.Renderers;

            if (_expected.Count > 0)
            {
                foreach (var cam in _expected)
                {
                    var r = FindRenderer(renderers, cam.Serial);
                    string name = $"{cam.Label} ({ShortSerial(cam.Serial)})";
                    if (r == null)
                    {
                        _alerts.Add($"カメラ {name} が異常です。接続を確認してください。（未検出）");
                        continue;
                    }
                    CheckStreaming(r, name);
                }
            }
            else if (expectedCameraCount > 0)
            {
                int live = 0;
                for (int i = 0; i < renderers.Count; i++)
                    if (renderers[i] != null) live++;
                if (live < expectedCameraCount)
                    _alerts.Add($"カメラが {live}/{expectedCameraCount} 台しか接続されていません。接続を確認してください。");
                for (int i = 0; i < renderers.Count; i++)
                {
                    var r = renderers[i];
                    if (r == null) continue;
                    CheckStreaming(r, ShortSerial(r.deviceSerial));
                }
            }

            LogTransition();
        }

        // Flags a live renderer whose capture thread died or whose frame timestamp
        // stopped advancing (USB drop mid-run leaves the GO alive but frameless).
        private void CheckStreaming(PointCloudRenderer r, string name)
        {
            if (!r.IsCapturing)
            {
                _alerts.Add($"カメラ {name} が異常です。接続を確認してください。（キャプチャ停止）");
                return;
            }

            if (!_health.TryGetValue(r.deviceSerial, out var h))
            {
                h = new Health();
                _health[r.deviceSerial] = h;
            }
            ulong ts = r.LastTimestampUs;
            if (!h.Seen || ts != h.LastTs)
            {
                h.Seen = true;
                h.LastTs = ts;
                h.LastProgressAt = Time.unscaledTime;
            }
            else if (Time.unscaledTime - h.LastProgressAt > staleFrameSeconds)
            {
                _alerts.Add($"カメラ {name} が異常です。接続を確認してください。（フレーム停止）");
            }
        }

        // Inspector override wins; otherwise cameras.yaml is read once (list order is
        // the stable camera id, so labels come out as ID0..IDn).
        private void ResolveExpected()
        {
            if (expectedSerials != null && expectedSerials.Length > 0)
            {
                _expected.Clear();
                for (int i = 0; i < expectedSerials.Length; i++)
                {
                    string s = expectedSerials[i];
                    if (string.IsNullOrWhiteSpace(s)) continue;
                    _expected.Add(new ExpectedCam { Serial = s.Trim(), Label = $"ID{i}" });
                }
                return;
            }

            if (_yamlTried) return;
            _yamlTried = true;
            try
            {
                var map = PointCloudRecording.ReadCamerasYaml(_manager.ResolveExtrinsicsRoot());
                if (map == null) return;
                for (int i = 0; i < map.Cameras.Count; i++)
                {
                    var e = map.Cameras[i];
                    if (e == null || string.IsNullOrEmpty(e.Serial)) continue;
                    string label = string.IsNullOrEmpty(e.Label) ? $"ID{i}" : e.Label;
                    _expected.Add(new ExpectedCam { Serial = e.Serial, Label = label });
                }
                if (logAlerts)
                    Debug.Log($"[{nameof(CameraHealthMonitor)}] cameras.yaml: monitoring " +
                              $"{_expected.Count} expected camera(s).");
            }
            catch (Exception e)
            {
                Debug.LogWarning($"[{nameof(CameraHealthMonitor)}] cameras.yaml read failed: {e.Message}", this);
            }
        }

        // Serials only differ in the tail (CL8F253004N → 4N); the last two chars
        // are enough to identify a camera on the alert banner.
        private static string ShortSerial(string serial) =>
            string.IsNullOrEmpty(serial) || serial.Length <= 2 ? serial : serial.Substring(serial.Length - 2);

        private static PointCloudRenderer FindRenderer(IReadOnlyList<PointCloudRenderer> renderers, string serial)
        {
            if (renderers == null) return null;
            for (int i = 0; i < renderers.Count; i++)
            {
                var r = renderers[i];
                if (r != null && r.deviceSerial == serial) return r;
            }
            return null;
        }

        private void LogTransition()
        {
            if (!logAlerts) return;
            string state = _alerts.Count == 0 ? string.Empty : string.Join("\n", _alerts);
            if (state == _lastLoggedState) return;
            if (state.Length > 0)
                Debug.LogWarning($"[{nameof(CameraHealthMonitor)}]\n{state}", this);
            else if (_lastLoggedState.Length > 0)
                Debug.Log($"[{nameof(CameraHealthMonitor)}] 全カメラ正常に復帰しました。", this);
            _lastLoggedState = state;
        }

        private void OnGUI()
        {
            if (!showAlert || _alerts.Count == 0) return;
            if (_alertStyle == null)
            {
                _alertStyle = new GUIStyle(GUI.skin.label)
                {
                    fontSize = 28,
                    alignment = TextAnchor.MiddleCenter,
                    fontStyle = FontStyle.Bold,
                    wordWrap = true,
                };
                // Tint via GUI.contentColor at draw time instead of the style's
                // textColor — the style-copy path rendered white on the first
                // frames after style creation; contentColor applies reliably.
                _alertStyle.normal.textColor = Color.white;
            }

            // 2-column grid (1 column for a single alert) — 4 stacked full-width
            // rows overflow the screen top area, a 2x2 grid stays visible.
            int cols = _alerts.Count >= 2 ? 2 : 1;
            int rows = (_alerts.Count + cols - 1) / cols;
            const float gap = 12f;
            const float pad = 12f;
            float totalW = Mathf.Min(Screen.width - 40, 1600f);
            float cellW = (totalW - gap * (cols - 1)) / cols;

            var contents = new GUIContent[_alerts.Count];
            var rowH = new float[rows];
            for (int i = 0; i < _alerts.Count; i++)
            {
                contents[i] = new GUIContent(_alerts[i]);
                float h = _alertStyle.CalcHeight(contents[i], cellW - pad * 2f) + pad * 2f;
                int row = i / cols;
                if (h > rowH[row]) rowH[row] = h;
            }

            float x0 = (Screen.width - totalW) * 0.5f;
            float y = Screen.height * 0.08f;
            var prevColor = GUI.color;
            var prevContent = GUI.contentColor;
            for (int row = 0; row < rows; row++)
            {
                for (int col = 0; col < cols; col++)
                {
                    int i = row * cols + col;
                    if (i >= _alerts.Count) break;
                    var rect = new Rect(x0 + col * (cellW + gap), y, cellW, rowH[row]);
                    GUI.color = new Color(0f, 0f, 0f, 0.8f);
                    GUI.DrawTexture(rect, Texture2D.whiteTexture);
                    GUI.color = prevColor;
                    GUI.contentColor = new Color(1f, 0.3f, 0.25f, 1f);
                    GUI.Label(rect, contents[i], _alertStyle);
                    GUI.contentColor = prevContent;
                }
                y += rowH[row] + gap;
            }
        }
    }
}
