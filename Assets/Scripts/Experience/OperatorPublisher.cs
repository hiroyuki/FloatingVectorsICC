// Operator one-key publish (dancer-session workflow): F10 captures the current
// moment with an operator-chosen trail length, exports GLB+USDZ, uploads via
// LFKS and shows the QR in the top-right corner of the screen (QrOverlay) so a
// phone can AR-preview it without leaving the main view.
//
// The capture protocol mirrors ExperienceDirector's prompt capture: set
// BonePoseHistory.historySamples -> wait for PointCloudMotionCurves.BuildVersion
// to advance by 2 (full rebuild with the new window; the GPU ring always holds
// 32 frames, so this works while frozen/paused too) with a 2 s timeout ->
// TSDFSnapshotBuilder.Capture (sync, non-destructive) -> restore. Publish
// settings (SHA-pinned upload.ps1, remote dir, timeout, dry-run, URL kind) come
// from the same ExperienceConfig as the visitor flow.

using System;
using System.Collections;
using System.IO;
using System.Threading;
using System.Threading.Tasks;
using BodyTracking;
using Experience.Publishing;
using TSDF;
using UnityEngine;

namespace Experience
{
    [DisallowMultipleComponent]
    public class OperatorPublisher : MonoBehaviour
    {
        [Tooltip("Publish settings source (dryRunPublish / uploadScriptSha256 / " +
                 "lfksRemoteDirectory / publishTimeoutSeconds / qrUrlKind). When " +
                 "empty a default instance is created — defaults are dry-run.")]
        public ExperienceConfig config;

        [Header("Scene references (auto-resolved when empty)")]
        public TSDFPrintExporter printExporter; // capture/export settings + volume
        public PointCloudMotionCurves motionCurves;
        public BonePoseHistory poseHistory;
        public QrOverlay qrOverlay; // added to this GameObject when missing

        [Header("Operator controls")]
        [Range(2, 32)]
        [Tooltip("Trail window for the captured sculpture, in BT frames (the visitor " +
                 "flow uses 15 ≈ 0.65 s at ~23 Hz). The pose ring holds 32 frames, so " +
                 "this can be changed and re-published while frozen on the same moment.")]
        public int trailSamples = 15;

        [Tooltip("Capture + export + upload + QR. Works frozen (Space) or paused — " +
                 "that's the intended flow. None disables the hotkey.")]
        public KeyCode publishKey = KeyCode.F10;

        [Tooltip("Hide the QR overlay. None disables the hotkey.")]
        public KeyCode hideQrKey = KeyCode.F11;

        public string StatusText => _status;
        public bool IsBusy => _routine != null;

        private string _status = "";
        private Coroutine _routine;
        private CancellationTokenSource _cts;
        private int _savedHistorySamples = -1;

        private void OnEnable()
        {
            if (printExporter == null) printExporter = FindFirstObjectByType<TSDFPrintExporter>();
            if (motionCurves == null) motionCurves = FindFirstObjectByType<PointCloudMotionCurves>();
            if (poseHistory == null) poseHistory = FindFirstObjectByType<BonePoseHistory>();
            if (qrOverlay == null) qrOverlay = FindFirstObjectByType<QrOverlay>()
                                               ?? gameObject.AddComponent<QrOverlay>();
        }

        private void OnDisable()
        {
            if (_cts != null) { _cts.Cancel(); _cts.Dispose(); _cts = null; }
            if (_routine != null) { StopCoroutine(_routine); _routine = null; }
            RestoreHistorySamples();
        }

        private void Update()
        {
            if (publishKey != KeyCode.None && Input.GetKeyDown(publishKey)) GenerateAndPublish();
            if (hideQrKey != KeyCode.None && Input.GetKeyDown(hideQrKey)) qrOverlay?.Hide();
        }

        [ContextMenu("Generate And Publish")]
        public void GenerateAndPublish()
        {
            if (!Application.isPlaying) return;
            if (_routine != null)
            {
                Debug.LogWarning($"[{nameof(OperatorPublisher)}] publish already running — ignored.", this);
                return;
            }
            _routine = StartCoroutine(PublishRoutine());
        }

        private IEnumerator PublishRoutine()
        {
            var sw = System.Diagnostics.Stopwatch.StartNew();
            if (printExporter == null || printExporter.volume == null || !printExporter.VolumeReady)
            {
                Fail("no initialised TSDFPrintExporter/volume in the scene");
                yield break;
            }
            if (motionCurves != null && motionCurves.freeze)
            {
                Fail("PointCloudMotionCurves.freeze is ON — the trail can't rebuild. " +
                     "Use the transport pause (Space) instead.");
                yield break;
            }

            // -- capture with the operator trail window (Director's protocol) --
            SetStatus($"capture (trail {trailSamples})…");
            if (poseHistory != null)
            {
                _savedHistorySamples = poseHistory.historySamples;
                poseHistory.historySamples = Mathf.Clamp(trailSamples, 2, 32);
                if (motionCurves != null)
                {
                    int target = motionCurves.BuildVersion + 2;
                    float deadline = Time.realtimeSinceStartup + 2f;
                    while (motionCurves.BuildVersion < target && Time.realtimeSinceStartup < deadline)
                        yield return null;
                    if (motionCurves.BuildVersion < target)
                        Debug.LogWarning($"[{nameof(OperatorPublisher)}] curve rebuild wait timed " +
                                         "out — capturing anyway (mesh-only capture is valid).", this);
                }
            }
            var snap = TSDFSnapshotBuilder.Capture(printExporter.volume, motionCurves,
                                                   printExporter.WebCaptureOptions(), out string err);
            RestoreHistorySamples();
            if (snap == null) { Fail(err); yield break; }

            // -- export --
            SetStatus("export GLB + USDZ…");
            string dir = Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.UserProfile),
                                      "Documents", "FloatingVectorsPrints");
            string stamp = DateTime.Now.ToString("yyyyMMdd_HHmmss");
            string glbPath = Path.Combine(dir, $"web_{stamp}_t{trailSamples}.glb");
            string usdzPath = Path.Combine(dir, $"web_{stamp}_t{trailSamples}.usdz");
            if (!TSDFSnapshotBuilder.ExportFiles(snap, glbPath, usdzPath,
                                                 printExporter.usdPythonPath, out _, out err))
            {
                Fail($"export failed: {err}");
                yield break;
            }

            // -- publish (same trust boundary as the visitor flow) --
            var cfg = config != null ? config : ScriptableObject.CreateInstance<ExperienceConfig>();
            ISculptureResultPublisher publisher;
            if (cfg.dryRunPublish)
            {
                publisher = new DryRunPublisher(cfg.dryRunDelaySeconds);
                SetStatus("upload (DRY RUN)…");
            }
            else
            {
                string token = LfksToken.Resolve();
                if (string.IsNullOrEmpty(token))
                {
                    Fail($"no LFKS token (persistentDataPath/lfks-token.txt or LFKS_TOKEN env) — " +
                         $"files kept locally: {glbPath}");
                    yield break;
                }
                publisher = new LfksUploadPublisher(
                    Path.Combine(Application.streamingAssetsPath, "lfks", "upload.ps1"),
                    cfg.uploadScriptSha256, token,
                    cfg.lfksRemoteDirectory, cfg.publishTimeoutSeconds);
                SetStatus("upload…");
            }
            _cts = new CancellationTokenSource();
            Task<PublishResult> task = publisher.PublishAsync(glbPath, usdzPath, _cts.Token);
            while (!task.IsCompleted) yield return null;

            if (task.IsCanceled) { _routine = null; yield break; }
            if (task.IsFaulted || !task.Result.Success)
            {
                string why = task.IsFaulted
                    ? task.Exception?.GetBaseException().Message
                    : task.Result.Error;
                Fail($"publish failed: {why} (files kept for manual upload: {glbPath})");
                yield break;
            }

            // -- QR --
            var r = task.Result;
            string url = cfg.qrUrlKind switch
            {
                QrUrlKind.Glb => r.GlbUrl,
                QrUrlKind.Usdz => r.UsdzUrl,
                _ => string.IsNullOrEmpty(r.UsdzUrl) ? r.GlbUrl : r.UsdzUrl,
            };
            var tex = new QrUrlPresenter().Present(url);
            if (tex != null) qrOverlay?.Show(tex, $"t{trailSamples} {stamp}");
            SetStatus($"done in {sw.ElapsedMilliseconds / 1000.0:0.0}s" +
                      (cfg.dryRunPublish ? " (DRY RUN)" : "") + $" — {url}");
            _routine = null;
        }

        private void RestoreHistorySamples()
        {
            if (_savedHistorySamples >= 0 && poseHistory != null)
                poseHistory.historySamples = _savedHistorySamples;
            _savedHistorySamples = -1;
        }

        private void SetStatus(string s)
        {
            _status = s;
            Debug.Log($"[{nameof(OperatorPublisher)}] {s}", this);
        }

        private void Fail(string why)
        {
            _status = "FAILED: " + why;
            Debug.LogError($"[{nameof(OperatorPublisher)}] {why}", this);
            _routine = null;
        }
    }
}
