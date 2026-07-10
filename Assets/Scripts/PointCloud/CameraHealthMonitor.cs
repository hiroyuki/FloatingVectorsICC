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

        [Header("Identical-content detection (experimental, default off)")]
        [Tooltip("Flag a camera whose depth payload hashes identically for the window " +
                 "below — a frozen USB device can keep advancing timestamps while " +
                 "delivering the same image. Verify against real hardware before " +
                 "trusting it on site.")]
        public bool detectIdenticalContent = false;
        [Min(1f)]
        public float identicalContentSeconds = 10f;

        [Header("Display")]
        [Tooltip("Draw the on-screen alert banner (IMGUI, same style as the crowd alert).")]
        public bool showAlert = true;
        [Tooltip("Also log alert transitions to the console.")]
        public bool logAlerts = true;

        /// <summary>Current alert lines (empty = all expected cameras healthy).</summary>
        public IReadOnlyList<string> CurrentAlerts => _alerts;

        /// <summary>True while every expected camera is healthy (or checks are
        /// suppressed). The experience director feeds this into the Fault state.</summary>
        public bool IsHealthy => _alerts.Count == 0;

        /// <summary>Fired when IsHealthy flips (arg = new health).</summary>
        public event Action<bool> OnHealthChanged;

        /// <summary>Text for the full-screen fault alert (operator/adult-facing,
        /// so kanji): names the first faulty camera as "カメラ（ID n）が異常です".</summary>
        public string FaultAlertText =>
            _alerts.Count == 0 ? "" : $"カメラ（{_firstFaultyLabel}）が異常です";

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
            // identical-content detection
            public ulong LastContentHash;
            public float LastContentChangeAt;
            public float NextHashAt;
            public bool ContentSeen;
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
        private string _firstFaultyLabel = "";
        private bool _lastHealthy = true;
        private GUIStyle _alertStyle;
        // identical-content subscriptions (resynced every RunCheck so renderer
        // churn — keepLiveRenderersOnLoad, reconnects — never leaks handlers)
        private readonly Dictionary<PointCloudRenderer, Action<PointCloudRenderer, RawFrameData>>
            _contentHandlers = new Dictionary<PointCloudRenderer, Action<PointCloudRenderer, RawFrameData>>();
        private readonly List<PointCloudRenderer> _handlerScratch = new List<PointCloudRenderer>();

        private void OnEnable()
        {
            _enabledAt = Time.unscaledTime;
            _nextCheckAt = 0f;
            _health.Clear();
            _alerts.Clear();
            _firstFaultyLabel = "";
            _lastHealthy = true;
        }

        private void OnDisable() => UnsubscribeAllContent();
        private void OnDestroy() => UnsubscribeAllContent();

        private void UnsubscribeAllContent()
        {
            foreach (var kv in _contentHandlers)
                if (kv.Key != null) kv.Key.OnRawFramesReady -= kv.Value;
            _contentHandlers.Clear();
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
            _firstFaultyLabel = "";

            // Suppression truth table (Plans/phase6-attract-watchdog-plan.md):
            //   playbackOnly                          -> suppress (dev, no live rig by design)
            //   recorder Playing/Paused AND 0 live    -> suppress (live torn down on purpose)
            //   recorder Playing/Paused AND live >= 1 -> MONITOR (attract coexistence)
            //   otherwise                             -> monitor
            if (_manager == null || _manager.playbackOnly) { LogTransition(); return; }
            if (_recorder == null) _recorder = FindFirstObjectByType<SensorRecorder>();
            int liveCount = 0;
            var renderersEarly = _manager.Renderers;
            for (int i = 0; i < renderersEarly.Count; i++)
                if (renderersEarly[i] != null) liveCount++;
            if (_recorder != null && (_recorder.IsPlaying || _recorder.IsPaused) && liveCount == 0)
            {
                // Playback-only usage (live renderers deliberately torn down /
                // frame consumption held): forget progress so nothing is flagged
                // stale when live resumes.
                _health.Clear();
                LogTransition();
                return;
            }
            if (_recorder != null && _recorder.IsPaused && liveCount > 0)
            {
                // Live freeze (Space): frame consumption is intentionally held
                // even though the rig is up — a frozen timestamp is not a fault.
                // (Refines the plan's table: Paused+live>=1 suppresses, only
                // Playing+live>=1 — the attract case — keeps monitoring.)
                _health.Clear();
                LogTransition();
                return;
            }
            if (Time.unscaledTime - _enabledAt < startupGraceSeconds) return;

            ResolveExpected();
            var renderers = _manager.Renderers;
            SyncContentSubscriptions(renderers);

            if (_expected.Count > 0)
            {
                foreach (var cam in _expected)
                {
                    var r = FindRenderer(renderers, cam.Serial);
                    string name = $"{cam.Label} ({PointCloudUtil.TailSerial(cam.Serial, 2)})";
                    if (r == null)
                    {
                        _alerts.Add($"カメラ {name} が異常です。接続を確認してください。（未検出）");
                        NoteFault(cam.Label);
                        continue;
                    }
                    CheckStreaming(r, name, cam.Label);
                }
            }
            else if (expectedCameraCount > 0)
            {
                int live = 0;
                for (int i = 0; i < renderers.Count; i++)
                    if (renderers[i] != null) live++;
                if (live < expectedCameraCount)
                {
                    _alerts.Add($"カメラが {live}/{expectedCameraCount} 台しか接続されていません。接続を確認してください。");
                    NoteFault($"{live}/{expectedCameraCount}");
                }
                for (int i = 0; i < renderers.Count; i++)
                {
                    var r = renderers[i];
                    if (r == null) continue;
                    string tail = PointCloudUtil.TailSerial(r.deviceSerial, 2);
                    CheckStreaming(r, tail, tail);
                }
            }

            LogTransition();
        }

        private void NoteFault(string label)
        {
            if (string.IsNullOrEmpty(_firstFaultyLabel)) _firstFaultyLabel = label;
        }

        // Flags a live renderer whose capture thread died or whose frame timestamp
        // stopped advancing (USB drop mid-run leaves the GO alive but frameless).
        private void CheckStreaming(PointCloudRenderer r, string name, string label)
        {
            if (!r.IsCapturing)
            {
                _alerts.Add($"カメラ {name} が異常です。接続を確認してください。（キャプチャ停止）");
                NoteFault(label);
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
                NoteFault(label);
            }

            // A frozen device can keep advancing timestamps while delivering the
            // same image — the payload hash (fed by OnRawFramesReady) must move.
            if (detectIdenticalContent && h.ContentSeen &&
                Time.unscaledTime - h.LastContentChangeAt > identicalContentSeconds)
            {
                _alerts.Add($"カメラ {name} が異常です。接続を確認してください。（映像フリーズ）");
                NoteFault(label);
            }
        }

        // ---- identical-content detection (subscription lifecycle) ----

        private void SyncContentSubscriptions(IReadOnlyList<PointCloudRenderer> renderers)
        {
            if (!detectIdenticalContent)
            {
                if (_contentHandlers.Count > 0) UnsubscribeAllContent();
                return;
            }
            // Drop dead / no-longer-managed renderers.
            _handlerScratch.Clear();
            foreach (var kv in _contentHandlers)
                if (kv.Key == null || !Contains(renderers, kv.Key)) _handlerScratch.Add(kv.Key);
            foreach (var dead in _handlerScratch)
            {
                if (dead != null) dead.OnRawFramesReady -= _contentHandlers[dead];
                _contentHandlers.Remove(dead);
            }
            // Subscribe new ones.
            for (int i = 0; i < renderers.Count; i++)
            {
                var r = renderers[i];
                if (r == null || _contentHandlers.ContainsKey(r)) continue;
                Action<PointCloudRenderer, RawFrameData> handler = HandleRawFramesForContent;
                r.OnRawFramesReady += handler;
                _contentHandlers[r] = handler;
            }
        }

        private static bool Contains(IReadOnlyList<PointCloudRenderer> list, PointCloudRenderer r)
        {
            for (int i = 0; i < list.Count; i++)
                if (ReferenceEquals(list[i], r)) return true;
            return false;
        }

        private void HandleRawFramesForContent(PointCloudRenderer r, RawFrameData raw)
        {
            if (!detectIdenticalContent || r == null) return;
            if (!_health.TryGetValue(r.deviceSerial, out var h))
            {
                h = new Health();
                _health[r.deviceSerial] = h;
            }
            float now = Time.unscaledTime;
            if (now < h.NextHashAt) return; // hash at most once per check interval
            h.NextHashAt = now + Mathf.Max(0.2f, checkIntervalSeconds);
            ulong hash = HashDepthPayload(in raw);
            if (!h.ContentSeen || hash != h.LastContentHash)
            {
                h.ContentSeen = true;
                h.LastContentHash = hash;
                h.LastContentChangeAt = now;
            }
        }

        // Stride-sampled FNV-1a over the depth payload — cheap (64 samples), and
        // any real scene motion flips at least one sampled depth byte.
        private static ulong HashDepthPayload(in RawFrameData raw)
        {
            var bytes = raw.DepthBytes;
            int count = raw.DepthByteCount;
            if (bytes == null || count <= 0) return 0UL;
            count = Math.Min(count, bytes.Length);
            ulong h = 14695981039346656037UL;
            int stride = Math.Max(1, count / 64);
            for (int i = 0; i < count; i += stride)
            {
                h ^= bytes[i];
                h *= 1099511628211UL;
            }
            return h;
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
            bool healthy = _alerts.Count == 0;
            if (healthy != _lastHealthy)
            {
                _lastHealthy = healthy;
                try { OnHealthChanged?.Invoke(healthy); }
                catch (Exception e) { Debug.LogException(e, this); }
            }
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
            // The full-screen operator alert (VisitorMessageUI) owns the display.
            if (Shared.OperatorOverlayGate.AlertActive) return;
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
