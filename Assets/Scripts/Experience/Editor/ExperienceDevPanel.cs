// Dev panel for running the visitor experience WITHOUT live cameras (Mac):
// the show starts empty (Idle), and pressing 入場 plays the configured
// recording — its recorded bodies drive the SkeletonMerger, PresenceDetector
// sees a pelvis inside the sensing volume and the show advances to Calibrate
// through the SAME signal path as a real visitor. debugForcePresence is only
// the fallback for takes whose pelvis never enters the sensing volume.
//
// Window > Experience Dev Panel

using PointCloud;
using UnityEditor;
using UnityEngine;

namespace Experience.EditorTools
{
    public class ExperienceDevPanel : EditorWindow
    {
        [MenuItem("Window/Experience Dev Panel")]
        private static void Open() => GetWindow<ExperienceDevPanel>("Experience Dev");

        private ExperienceDirector _director;
        private SensorRecorder _recorder;
        private PresenceDetector _presence; // runtime-spawned (_ExperiencePresence)

        private void OnEnable() => EditorApplication.playModeStateChanged += OnPlayMode;
        private void OnDisable() => EditorApplication.playModeStateChanged -= OnPlayMode;
        private void OnPlayMode(PlayModeStateChange _) { _presence = null; Repaint(); }

        private void Update()
        {
            // live readouts (presence flips, state transitions) need repaints
            if (Application.isPlaying) Repaint();
        }

        private void ResolveRefs()
        {
            if (_director == null)
                _director = FindFirstObjectByType<ExperienceDirector>(FindObjectsInactive.Include);
            if (_recorder == null)
                _recorder = FindFirstObjectByType<SensorRecorder>(FindObjectsInactive.Include);
            if (_presence == null && Application.isPlaying)
                _presence = FindFirstObjectByType<PresenceDetector>();
        }

        private void OnGUI()
        {
            ResolveRefs();

            if (_director == null)
            {
                EditorGUILayout.HelpBox(
                    "ExperienceDirector がシーンにありません（Experience/Director を確認）。",
                    MessageType.Error);
                return;
            }

            // ---- mode / state ------------------------------------------------
            EditorGUILayout.LabelField("体験モード", EditorStyles.boldLabel);
            using (new EditorGUI.DisabledScope(!Application.isPlaying))
            {
                bool on = _director.IsActive;
                string label = Application.isPlaying
                    ? $"Experience mode   —   state: {_director.CurrentState}"
                    : "Experience mode（Play 中のみ操作可）";
                bool want = EditorGUILayout.ToggleLeft(label, on);
                if (want != on) _director.Visible = want;
            }

            EditorGUILayout.Space(8);

            // ---- entrance simulation ----------------------------------------
            EditorGUILayout.LabelField("入場シミュレーション（収録再生 = 来場者）",
                EditorStyles.boldLabel);
            if (_recorder == null)
            {
                EditorGUILayout.HelpBox("SensorRecorder がシーンにありません。", MessageType.Error);
            }
            else
            {
                EditorGUILayout.LabelField("Recorder", Application.isPlaying
                    ? _recorder.TransportStatus : "-");
                DrawPlaybackFolderRow();

                using (new EditorGUILayout.HorizontalScope())
                {
                    bool playing = Application.isPlaying &&
                                   _recorder.CurrentState == SensorRecorder.State.Playing;
                    bool busyRec = Application.isPlaying &&
                                   _recorder.CurrentState == SensorRecorder.State.Recording;
                    using (new EditorGUI.DisabledScope(!Application.isPlaying || playing || busyRec))
                        if (GUILayout.Button("入場（収録を Play）", GUILayout.Height(28)))
                            _recorder.TogglePlay();
                    using (new EditorGUI.DisabledScope(!playing))
                        if (GUILayout.Button("退場（Stop Play）", GUILayout.Height(28)))
                            _recorder.TogglePlay();
                }
            }

            EditorGUILayout.Space(8);

            // ---- presence ----------------------------------------------------
            EditorGUILayout.LabelField("在場検知", EditorStyles.boldLabel);
            if (_presence == null)
            {
                EditorGUILayout.HelpBox(
                    "PresenceDetector 未生成（Experience mode を ON にすると生成されます）。",
                    MessageType.Info);
            }
            else
            {
                EditorGUILayout.LabelField(
                    $"体内(BT): {Dot(_presence.IsPersonInside)}   " +
                    $"点群占有: {Dot(_presence.OccupancyActive)}   " +
                    $"→ Present: {Dot(_presence.IsPresent)}   " +
                    $"人数: {_presence.PersonCount}");
                _presence.debugForcePresence = EditorGUILayout.ToggleLeft(
                    "debugForcePresence（検知を無視して常に在場扱い / Playごとにリセット）",
                    _presence.debugForcePresence);
            }

            EditorGUILayout.Space(8);
            DrawPreviewSection();

            EditorGUILayout.Space(8);
            DrawLayoutSection();

            EditorGUILayout.Space(8);
            DrawConfigSection();

            EditorGUILayout.Space(8);
            DrawHints();
        }

        // One-shot screen previews on the scene VisitorUI — layout tuning per
        // state without running the sequence. Play mode only (canvases + font
        // are runtime-built). With Experience mode ON the director's state
        // side-effects overwrite these, so preview with the mode OFF.
        private Texture2D _previewStar, _previewQr;
        private ExperienceConfig _previewDefaults; // fallback texts when no config assigned

        private void DrawPreviewSection()
        {
            EditorGUILayout.LabelField("画面プレビュー（UIのみ・シーケンス不要）", EditorStyles.boldLabel);
            var ui = FindFirstObjectByType<VisitorMessageUI>(FindObjectsInactive.Include);
            if (ui == null)
            {
                EditorGUILayout.HelpBox("VisitorMessageUI（Experience/VisitorUI）がシーンにありません。",
                    MessageType.Warning);
                return;
            }
            var cfg = _director.config;
            if (cfg == null)
            {
                if (_previewDefaults == null)
                {
                    _previewDefaults = ScriptableObject.CreateInstance<ExperienceConfig>();
                    _previewDefaults.hideFlags = HideFlags.HideAndDontSave;
                }
                cfg = _previewDefaults;
            }

            using (new EditorGUI.DisabledScope(!Application.isPlaying))
            {
                using (new EditorGUILayout.HorizontalScope())
                {
                    if (GUILayout.Button("アイドル（全消去）")) ui.ClearAll();
                    if (GUILayout.Button("ようこそ")) ui.ShowMessage(cfg.welcomeText);
                    if (GUILayout.Button("キャリブ（星ポーズ）"))
                        ui.ShowPoseGuide(StarTex(cfg), cfg.calibrateText);
                    if (GUILayout.Button("カウントダウン")) ui.ShowCountdown(3);
                }
                using (new EditorGUILayout.HorizontalScope())
                {
                    if (GUILayout.Button("じゆうに (FreeMove)")) ui.ShowMessage(cfg.freeMoveText);
                    if (GUILayout.Button("さつえい (Shoot cue)")) ui.ShowMessage(cfg.shootCueText);
                    if (GUILayout.Button("しょりちゅう (Processing)"))
                        ui.ShowProgress(0.4f, cfg.processingText);
                }
                using (new EditorGUILayout.HorizontalScope())
                {
                    if (GUILayout.Button("できたよ (ResultShow)")) ui.ShowResult(cfg.resultText);
                    if (GUILayout.Button("QR"))
                    {
                        if (_previewQr == null)
                            _previewQr = new Publishing.QrUrlPresenter()
                                .Present("https://example.com/preview");
                        ui.ShowQr(_previewQr, !string.IsNullOrEmpty(cfg.qrScanText)
                            ? cfg.qrScanText : cfg.qrCaption, cfg.resultText);
                    }
                    if (GUILayout.Button("アラート")) ui.ShowAlert("カメラ（ID 1）が　いじょうです");
                    if (GUILayout.Button("全消去")) ui.ClearEverything();
                }
            }
            if (!Application.isPlaying)
                EditorGUILayout.HelpBox("プレビューは Play 中のみ（Canvas はランタイム生成）。", MessageType.None);
            else if (_director.IsActive)
                EditorGUILayout.HelpBox(
                    "Experience mode が ON — ステート遷移がプレビューを上書きします。調整中は OFF 推奨。",
                    MessageType.Warning);
        }

        // Position / size tuning for the visitor UI elements (VisitorMessageUI's
        // serialized layout fields). Everything live-applies while the element is
        // on screen, so the loop is: Play → preview button above → drag numbers
        // here and watch the visitor display. Values persist with the scene.
        private bool _layoutFoldout = true;

        private void DrawLayoutSection()
        {
            var ui = FindFirstObjectByType<VisitorMessageUI>(FindObjectsInactive.Include);
            if (ui == null) return; // the preview section already shows the warning

            _layoutFoldout = EditorGUILayout.Foldout(_layoutFoldout,
                "UI レイアウト（文字/画像/QR の位置・サイズ）", true, EditorStyles.foldoutHeader);
            if (!_layoutFoldout) return;

            EditorGUI.BeginChangeCheck();

            EditorGUILayout.LabelField("位置（画面中心からの px, 1920×1080 基準）",
                EditorStyles.miniBoldLabel);
            Vector2 msgPos = EditorGUILayout.Vector2Field("メッセージ文字", ui.messagePosition);
            Vector2 cdPos = EditorGUILayout.Vector2Field("カウントダウン数字", ui.countdownPosition);
            Vector2 poseImgPos = EditorGUILayout.Vector2Field("ポーズ画像", ui.poseImagePosition);
            Vector2 poseLblPos = EditorGUILayout.Vector2Field("ポーズ説明文字", ui.poseLabelPosition);
            Vector2 qrPos = EditorGUILayout.Vector2Field("QR コード", ui.qrPosition);
            Vector2 qrHeadPos = EditorGUILayout.Vector2Field("QR 見出し文字", ui.qrHeadlinePosition);
            Vector2 noticePos = EditorGUILayout.Vector2Field("注意書きボックス", ui.noticePosition);

            EditorGUILayout.Space(4);
            EditorGUILayout.LabelField("サイズ（文字 pt / 画像 px）", EditorStyles.miniBoldLabel);
            int msgFont = EditorGUILayout.IntField("メッセージ文字", ui.messageFontSize);
            int cdFont = EditorGUILayout.IntField("カウントダウン文字", ui.countdownFontSize);
            float poseSize = EditorGUILayout.FloatField("ポーズ画像", ui.poseGuideSizePixels);
            float qrSize = EditorGUILayout.FloatField("QR コード", ui.qrSizePixels);
            int capFont = EditorGUILayout.IntField("QR キャプション文字", ui.captionFontSize);
            int noticeFont = EditorGUILayout.IntField("注意書き文字", ui.noticeFontSize);

            if (EditorGUI.EndChangeCheck())
            {
                Undo.RecordObject(ui, "Visitor UI layout");
                ui.messagePosition = msgPos;
                ui.countdownPosition = cdPos;
                ui.poseImagePosition = poseImgPos;
                ui.poseLabelPosition = poseLblPos;
                ui.qrPosition = qrPos;
                ui.qrHeadlinePosition = qrHeadPos;
                ui.noticePosition = noticePos;
                // Same floors as the fields' [Min] attributes (this path bypasses
                // the Inspector's clamping).
                ui.messageFontSize = Mathf.Max(10, msgFont);
                ui.countdownFontSize = Mathf.Max(10, cdFont);
                ui.poseGuideSizePixels = Mathf.Max(64f, poseSize);
                ui.qrSizePixels = Mathf.Max(64f, qrSize);
                ui.captionFontSize = Mathf.Max(10, capFont);
                ui.noticeFontSize = Mathf.Max(10, noticeFont);
                EditorUtility.SetDirty(ui);
            }

            EditorGUILayout.HelpBox(
                "Play 中に上の画面プレビューで対象を表示しながら動かすと即反映（X/Y ラベルの" +
                "ドラッグで連続調整）。値は VisitorUI のシーン値 — 保存を忘れずに。",
                MessageType.None);
        }

        private Texture2D StarTex(ExperienceConfig cfg)
        {
            if (cfg.poseGuideTexture != null) return cfg.poseGuideTexture;
            if (_previewStar == null) _previewStar = StickFigureTexture.DrawStarPose();
            return _previewStar;
        }

        // Edits the SHARED ExperienceConfig asset (goes into git) — meant for the
        // Mac dev loop; restore skips before a Windows production run.
        private void DrawConfigSection()
        {
            var cfg = _director.config;
            if (cfg == null) return;
            EditorGUILayout.LabelField("Config（共有アセットを直接編集 — 本番前に戻す）",
                EditorStyles.boldLabel);

            EditorGUI.BeginChangeCheck();
            bool dry = EditorGUILayout.ToggleLeft(
                "dryRunPublish（実アップロードせずフェイク URL）", cfg.dryRunPublish);
            bool ss = EditorGUILayout.ToggleLeft(
                "skipShoot（Mac: 実録画の代わりに缶詰テイクを使用）", cfg.timings.skipShoot);
            bool sp = EditorGUILayout.ToggleLeft(
                "skipProcessing（v11s 変換を飛ばし録画済み BT で再生）", cfg.timings.skipProcessing);
            bool sq = EditorGUILayout.ToggleLeft("skipQr", cfg.timings.skipQr);
            float tm = EditorGUILayout.Slider(
                new GUIContent("timeMultiplier", "全タイミングの早回し倍率"),
                cfg.timings.timeMultiplier, 0.5f, 5f);

            string canned = cfg.devCannedTakeRoot;
            using (new EditorGUILayout.HorizontalScope())
            {
                canned = EditorGUILayout.TextField("缶詰テイク (skipShoot)", canned);
                if (GUILayout.Button("選択…", GUILayout.Width(60)))
                {
                    string picked = EditorUtility.OpenFolderPanel("缶詰テイクのルート", canned, "");
                    if (!string.IsNullOrEmpty(picked)) canned = picked;
                    GUI.FocusControl(null);
                }
            }
            if (EditorGUI.EndChangeCheck())
            {
                Undo.RecordObject(cfg, "Experience config (dev panel)");
                cfg.dryRunPublish = dry;
                cfg.timings.skipShoot = ss;
                cfg.timings.skipProcessing = sp;
                cfg.timings.skipQr = sq;
                cfg.timings.timeMultiplier = tm;
                cfg.devCannedTakeRoot = canned;
                EditorUtility.SetDirty(cfg);
            }
            if (cfg.timings.skipShoot && !System.IO.Directory.Exists(cfg.devCannedTakeRoot))
                EditorGUILayout.HelpBox(
                    "skipShoot ON ですが缶詰テイクのフォルダが存在しません: " +
                    cfg.devCannedTakeRoot, MessageType.Warning);
        }

        private void DrawPlaybackFolderRow()
        {
            bool mac = Application.platform == RuntimePlatform.OSXEditor;
            string current = mac && !string.IsNullOrEmpty(_recorder.playbackFolderPathMacOverride)
                ? _recorder.playbackFolderPathMacOverride
                : _recorder.playbackFolderPath;
            using (new EditorGUILayout.HorizontalScope())
            {
                EditorGUILayout.LabelField("再生フォルダ",
                    string.IsNullOrEmpty(current) ? "(未設定)" : current);
                if (GUILayout.Button("選択…", GUILayout.Width(60)))
                {
                    string picked = EditorUtility.OpenFolderPanel(
                        "収録テイクのルートフォルダ", current, "");
                    if (!string.IsNullOrEmpty(picked))
                    {
                        if (mac) _recorder.playbackFolderPathMacOverride = picked;
                        else _recorder.playbackFolderPath = picked;
                        EditorUtility.SetDirty(_recorder);
                    }
                }
            }
        }

        private void DrawHints()
        {
            if (!Application.isPlaying)
            {
                EditorGUILayout.HelpBox(
                    "手順: Play → Experience mode ON（Idle で無人待機）→ 入場ボタン" +
                    "（収録再生の bodies が来場者として検知され Calibrate へ進む）。",
                    MessageType.Info);
                return;
            }
            if (_director.IsActive && _director.CurrentState == ExperienceState.Idle
                && _recorder != null
                && _recorder.CurrentState != SensorRecorder.State.Playing)
            {
                EditorGUILayout.HelpBox(
                    "無人（Idle）。「入場」で収録を再生すると Calibrate が始まります。",
                    MessageType.Info);
            }
            var cfg = _director.config;
            if (cfg != null && !cfg.dryRunPublish)
                EditorGUILayout.HelpBox(
                    "dryRunPublish が OFF — ResultShow で実際に LFKS アップロードが走ります。",
                    MessageType.Warning);
        }

        private static string Dot(bool b) => b ? "●" : "○";
    }
}
