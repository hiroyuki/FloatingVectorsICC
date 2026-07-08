// Watches "which cameras should have a k4abt worker" against "which workers
// are actually alive" and raises an on-screen alert + console error when a
// serial stays uncovered past a grace window. Without this, a camera whose
// worker never spawned (first frame lost to a mode switch) or never recovered
// (StartWorker failing repeatedly) silently degrades the skeleton merge — the
// point cloud looks fine while one camera's bones are just missing.
//
// Complements CameraHealthMonitor, which covers the layer below (camera not
// connected / frames stopped). This component assumes the camera side is
// healthy and checks only the worker layer on top of it:
//   live      → every capturing PointCloudRenderer serial needs a Ready session
//   playback  → every played-back track that runs live k4abt (Windows default,
//               SkeletonMerger.ignoreRecordedBodies) needs a Ready session;
//               tracks restored from bodies_main are exempt (no worker by design)

using System.Collections.Generic;
using PointCloud;
using UnityEngine;

namespace BodyTracking
{
    [AddComponentMenu("BodyTracking/Worker Gap Monitor")]
    public class WorkerGapMonitor : MonoBehaviour
    {
        [Header("Sources (auto-found when null)")]
        public SensorManager cameraManager;
        public K4abtWorkerHost workerHost;
        public SkeletonMerger merger;

        [Header("Detection")]
        [Tooltip("Seconds a serial may stay expected-but-uncovered before the alert fires. " +
                 "Must exceed worker startup (exe spawn + ONNX model load, several seconds " +
                 "on first run) so a normally-booting worker never trips it.")]
        public float spawnGraceSeconds = 15f;
        [Tooltip("Seconds between checks.")]
        public float checkIntervalSeconds = 1f;

        [Header("Display")]
        [Tooltip("Draw the on-screen alert banner (IMGUI, same operator-feedback style as " +
                 "the crowd alert and CameraHealthMonitor).")]
        public bool showAlert = true;
        [Tooltip("Also log alert transitions to the console (as errors — a missing worker " +
                 "means silently degraded merge quality).")]
        public bool logAlerts = true;

        /// <summary>Current alert lines (empty = every expected worker is up).</summary>
        public IReadOnlyList<string> CurrentAlerts => _alerts;

        private SensorRecorder _recorder;
        // serial -> unscaledTime when it first became expected. A serial ages toward
        // the grace deadline only while continuously expected; dropping out (camera
        // gone, playback stopped) resets its clock.
        private readonly Dictionary<string, float> _expectedSince = new Dictionary<string, float>();
        private readonly HashSet<string> _expectedNow = new HashSet<string>();
        private readonly List<string> _removeScratch = new List<string>();
        private readonly List<string> _alerts = new List<string>();
        private string _lastLoggedState = string.Empty;
        private float _nextCheckAt;
        private GUIStyle _alertStyle;

        private void OnEnable()
        {
            _expectedSince.Clear();
            _alerts.Clear();
            _lastLoggedState = string.Empty;
            _nextCheckAt = 0f;
        }

        private void Update()
        {
            if (Time.unscaledTime < _nextCheckAt) return;
            _nextCheckAt = Time.unscaledTime + Mathf.Max(0.2f, checkIntervalSeconds);
            RunCheck();
        }

        private void RunCheck()
        {
            _alerts.Clear();

            // k4abt workers are Windows-only (StartWorker no-ops elsewhere); on Mac
            // the skeleton source is bodies_main playback, so there is nothing to watch.
            if (Application.platform != RuntimePlatform.WindowsEditor &&
                Application.platform != RuntimePlatform.WindowsPlayer)
            {
                LogTransition();
                return;
            }

            if (merger == null) merger = FindFirstObjectByType<SkeletonMerger>();
            if (workerHost == null) workerHost = FindFirstObjectByType<K4abtWorkerHost>();
            if (cameraManager == null) cameraManager = FindFirstObjectByType<SensorManager>();
            if (_recorder == null) _recorder = FindFirstObjectByType<SensorRecorder>();

            // Merger disabled (or BT switched off) → it stopped all workers on purpose.
            if (merger == null || !merger.isActiveAndEnabled ||
                workerHost == null || !workerHost.useWorker)
            {
                _expectedSince.Clear();
                LogTransition();
                return;
            }

            // Paused playback delivers no frames, so a not-yet-spawned worker cannot
            // spawn — don't age the grace timers toward a false alert.
            if (_recorder != null && _recorder.IsPaused)
            {
                _expectedSince.Clear();
                LogTransition();
                return;
            }

            BuildExpectedSet();

            foreach (var serial in _expectedNow)
                if (!_expectedSince.ContainsKey(serial))
                    _expectedSince[serial] = Time.unscaledTime;
            _removeScratch.Clear();
            foreach (var kv in _expectedSince)
                if (!_expectedNow.Contains(kv.Key)) _removeScratch.Add(kv.Key);
            foreach (var k in _removeScratch) _expectedSince.Remove(k);

            foreach (var kv in _expectedSince)
            {
                if (Time.unscaledTime - kv.Value < spawnGraceSeconds) continue;
                // Ready (not just HasSession): a session stuck in init past the grace
                // window is as silent a gap as no session at all.
                if (workerHost.IsReady(kv.Key)) continue;
                string detail = workerHost.HasSession(kv.Key) ? "worker応答なし" : "worker未起動";
                _alerts.Add($"BTワーカー異常: カメラ {TailSerial(kv.Key, 4)} の骨格が" +
                            $"欠けています（{detail}）");
            }

            LogTransition();
        }

        private void BuildExpectedSet()
        {
            _expectedNow.Clear();

            if (_recorder != null && _recorder.IsPlaying)
            {
                // Playback: on Windows the merger feeds played-back depth to live k4abt
                // (ignoreRecordedBodies). A track only skips the worker when the flag is
                // off AND recorded bodies exist — then bodies_main is the source by design.
                var tracks = _recorder.GetRecordedDepthTracks();
                for (int i = 0; i < tracks.Count; i++)
                {
                    string serial = tracks[i].Serial;
                    if (string.IsNullOrEmpty(serial)) continue;
                    if (!merger.ignoreRecordedBodies && _recorder.HasRecordedBodies(serial)) continue;
                    _expectedNow.Add(serial);
                }
                return;
            }

            // Live: every capturing renderer must be covered by a worker. Same serial
            // key fallback as SkeletonMerger.HandleRawFrame (GO name when serial empty)
            // so both sides of the comparison use identical keys.
            var renderers = cameraManager != null ? cameraManager.Renderers : null;
            if (renderers == null) return;
            for (int i = 0; i < renderers.Count; i++)
            {
                var r = renderers[i];
                if (r == null || !r.IsCapturing) continue;
                _expectedNow.Add(string.IsNullOrEmpty(r.deviceSerial) ? r.gameObject.name : r.deviceSerial);
            }
        }

        // Last-n-chars display form of a serial (PointCloudUtil.TailSerial is
        // internal to the PointCloud assembly, so keep a local copy).
        private static string TailSerial(string serial, int chars)
        {
            if (string.IsNullOrEmpty(serial)) return "?";
            return serial.Length <= chars ? serial : serial.Substring(serial.Length - chars);
        }

        private void LogTransition()
        {
            if (!logAlerts) return;
            string state = _alerts.Count == 0 ? string.Empty : string.Join("\n", _alerts);
            if (state == _lastLoggedState) return;
            if (state.Length > 0)
                Debug.LogError($"[{nameof(WorkerGapMonitor)}]\n{state}", this);
            else if (_lastLoggedState.Length > 0)
                Debug.Log($"[{nameof(WorkerGapMonitor)}] 全カメラの BT worker が復帰しました。", this);
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
                _alertStyle.normal.textColor = Color.white;
            }

            const float pad = 12f;
            float w = Mathf.Min(Screen.width - 40, 1100f);
            var content = new GUIContent(string.Join("\n", _alerts));
            float h = _alertStyle.CalcHeight(content, w - pad * 2f) + pad * 2f;
            // Below the CameraHealthMonitor / crowd-alert zone (both draw at 8%
            // from the top) so simultaneous alerts stay readable.
            var rect = new Rect((Screen.width - w) * 0.5f, Screen.height * 0.3f, w, h);
            var prevColor = GUI.color;
            var prevContent = GUI.contentColor;
            GUI.color = new Color(0f, 0f, 0f, 0.8f);
            GUI.DrawTexture(rect, Texture2D.whiteTexture);
            GUI.color = prevColor;
            GUI.contentColor = new Color(1f, 0.3f, 0.25f, 1f);
            GUI.Label(rect, content, _alertStyle);
            GUI.contentColor = prevContent;
        }
    }
}
