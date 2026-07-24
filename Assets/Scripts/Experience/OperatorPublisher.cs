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
using PointCloud;
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
        public SensorRecorder sensorRecorder; // freeze-countdown source
        public CountdownOverlay countdownOverlay; // added to this GameObject when missing

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

        [Tooltip("Toggle the export-data preview: the captured snapshot (decimated " +
                 "mesh + trail tubes — exactly what the GLB/USDZ contain) swapped in " +
                 "place of the live TSDF mesh + curves. Shown automatically after each " +
                 "capture; toggle off to get the live view back. None disables the hotkey.")]
        public KeyCode previewToggleKey = KeyCode.F12;

        public string StatusText => _status;
        public bool IsBusy => _routine != null;
        public bool PreviewShowing => _previewRoot != null;

        private string _status = "";
        private Coroutine _routine;
        private CancellationTokenSource _cts;
        private int _savedHistorySamples = -1;

        // export-data preview (in-place swap)
        private TSDFSnapshot _lastSnap;
        private GameObject _previewRoot;
        private Material _previewMat;
        private readonly System.Collections.Generic.List<TSDFView> _hiddenViews =
            new System.Collections.Generic.List<TSDFView>();
        private bool _savedCurvesVisible;
        private PointCloudMotionCurves _curvesHidden;

        private void OnEnable()
        {
            if (printExporter == null) printExporter = FindFirstObjectByType<TSDFPrintExporter>();
            if (motionCurves == null) motionCurves = FindFirstObjectByType<PointCloudMotionCurves>();
            if (poseHistory == null) poseHistory = FindFirstObjectByType<BonePoseHistory>();
            if (qrOverlay == null) qrOverlay = FindFirstObjectByType<QrOverlay>()
                                               ?? gameObject.AddComponent<QrOverlay>();
            if (sensorRecorder == null) sensorRecorder = FindFirstObjectByType<SensorRecorder>();
            if (countdownOverlay == null) countdownOverlay = FindFirstObjectByType<CountdownOverlay>()
                                                             ?? gameObject.AddComponent<CountdownOverlay>();
        }

        private void OnDisable()
        {
            if (_cts != null) { _cts.Cancel(); _cts.Dispose(); _cts = null; }
            if (_routine != null) { StopCoroutine(_routine); _routine = null; }
            RestoreHistorySamples();
            HidePreview();
            _lastSnap = null;
        }

        private void OnDestroy()
        {
            if (_previewMat != null) Destroy(_previewMat);
        }

        private void Update()
        {
            if (publishKey != KeyCode.None && Input.GetKeyDown(publishKey)) GenerateAndPublish();
            if (hideQrKey != KeyCode.None && Input.GetKeyDown(hideQrKey)) qrOverlay?.Hide();
            if (previewToggleKey != KeyCode.None && Input.GetKeyDown(previewToggleKey))
            {
                if (PreviewShowing) HidePreview();
                else ShowPreview();
            }

            // Freeze-countdown display (Space): drawn here via per-display canvases —
            // SensorRecorder's IMGUI OnGUI never showed on the multi-display setup.
            float cd = sensorRecorder != null ? sensorRecorder.FreezeCountdownRemaining : 0f;
            if (cd > 0f && countdownOverlay != null)
                countdownOverlay.Show(Mathf.CeilToInt(cd).ToString());
            else countdownOverlay?.Hide();
        }

        // ---------------- export-data preview ----------------

        /// <summary>Swap the live sculpture visuals (TSDF mesh + curves) for the last
        /// captured snapshot's display meshes — exactly the geometry the GLB/USDZ
        /// contain (decimated, smoothed, trail tubes), rendered where it stood.</summary>
        public void ShowPreview()
        {
            if (_lastSnap == null)
            {
                Debug.LogWarning($"[{nameof(OperatorPublisher)}] no snapshot captured yet — " +
                                 "press the publish key first.", this);
                return;
            }
            HidePreview();

            TSDFSnapshotBuilder.BuildDisplayMeshes(_lastSnap, out Mesh surface, out Mesh tubes);
            if (surface == null) { Debug.LogWarning($"[{nameof(OperatorPublisher)}] snapshot has no mesh.", this); return; }
            if (_previewMat == null)
            {
                var shader = Resources.Load<Shader>("SnapshotVertexColor");
                if (shader == null)
                {
                    Debug.LogError($"[{nameof(OperatorPublisher)}] Resources/SnapshotVertexColor " +
                                   "shader missing — cannot preview.", this);
                    Destroy(surface); if (tubes != null) Destroy(tubes);
                    return;
                }
                _previewMat = new Material(shader)
                { name = "Snapshot preview (auto)", hideFlags = HideFlags.DontSave };
            }

            _previewRoot = new GameObject("_SnapshotPreview");
            _previewRoot.transform.SetParent(transform, false); // meshes are world-space
            AddMeshChild(surface, "surface");
            if (tubes != null) AddMeshChild(tubes, "tubes");

            // hide what the preview replaces, remember what we touched
            foreach (var v in FindObjectsByType<TSDFView>(FindObjectsInactive.Exclude, FindObjectsSortMode.None))
                if (v.Visible) { v.Visible = false; _hiddenViews.Add(v); }
            if (motionCurves != null)
            {
                _savedCurvesVisible = motionCurves.visible;
                motionCurves.visible = false;
                _curvesHidden = motionCurves;
            }
            SetStatus($"preview: {surface.triangles.Length / 3 / 1000}k tris" +
                      (tubes != null ? " + tubes" : "") + $" (t{trailSamples}) — toggle {previewToggleKey}");
        }

        public void HidePreview()
        {
            if (_previewRoot != null)
            {
                foreach (var mf in _previewRoot.GetComponentsInChildren<MeshFilter>())
                    if (mf.sharedMesh != null) Destroy(mf.sharedMesh);
                Destroy(_previewRoot);
                _previewRoot = null;
            }
            foreach (var v in _hiddenViews)
                if (v != null) v.Visible = true;
            _hiddenViews.Clear();
            if (_curvesHidden != null)
            {
                _curvesHidden.visible = _savedCurvesVisible;
                _curvesHidden = null;
            }
        }

        private void AddMeshChild(Mesh mesh, string name)
        {
            var go = new GameObject(name);
            go.transform.SetParent(_previewRoot.transform, false);
            go.AddComponent<MeshFilter>().sharedMesh = mesh;
            var mr = go.AddComponent<MeshRenderer>();
            mr.sharedMaterial = _previewMat;
            mr.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
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
            // A visible preview hides the curves, which also stops their rebuild —
            // restore the live visuals before capturing the next snapshot.
            HidePreview();
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

            // in-place preview of exactly what the files will contain, while the
            // export + upload continue below
            _lastSnap = snap;
            ShowPreview();

            // -- export --
            SetStatus("export GLB + USDZ…");
            string dir = Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.UserProfile),
                                      "Documents", "FloatingVectorsPrints");
            string stamp = DateTime.Now.ToString("yyyyMMdd_HHmmss");
            string glbPath = Path.Combine(dir, $"web_{stamp}_t{trailSamples}.glb");
            string usdzPath = Path.Combine(dir, $"web_{stamp}_t{trailSamples}.usdz");
            if (!TSDFSnapshotBuilder.ExportFiles(snap, glbPath, usdzPath,
                                                 printExporter.usdPythonPath, out _, out err,
                                                 printExporter.WebExportOptions()))
            {
                Fail($"export failed: {err}");
                yield break;
            }

            // -- publish (same trust boundary as the visitor flow) --
            var cfg = config != null ? config : ScriptableObject.CreateInstance<ExperienceConfig>();
            ISculptureResultPublisher publisher;
            if (cfg.DryRunPublish)
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
                    cfg.lfksRemoteDirectory, cfg.publishTimeoutSeconds, cfg.lfksApiUrl);
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
            // Same as the visitor flow: upload to LFKS, but point the QR at the
            // ICC viewer page for the upload API's file id (?id=…).
            var r = task.Result;
            string url = cfg.BuildQrUrl(r.GlbId, r.UsdzId, r.GlbUrl, r.UsdzUrl);
            var tex = new QrUrlPresenter().Present(url);
            if (tex != null) qrOverlay?.Show(tex, $"t{trailSamples} {stamp}");
            SetStatus($"done in {sw.ElapsedMilliseconds / 1000.0:0.0}s" +
                      (cfg.DryRunPublish ? " (DRY RUN)" : "") + $" — {url}");
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
