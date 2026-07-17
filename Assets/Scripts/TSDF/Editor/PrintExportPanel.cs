// Dedicated 3D-print panel: Window > Print Export.
//
// Everything for the print workflow in one place, in workflow order:
//   1. freeze the moment  — playback pause row (IRecorderTransport)
//   2. tune               — curve tube / hole closing / export settings
//   3. operate            — Fuse curves / Close holes / Export STL / Restore
//                           + Export Web (GLB for web viewers, USDZ for iPhone AR)
//
// The print operations used to live on the shared Control Panel's Actions
// section; they moved here so settings and buttons sit together. The panel
// binds TSDFPrintExporter directly (typed), unlike the shared panel's
// interface-based discovery — this is a single-component workflow UI.

using PointCloud;
using TSDF;
using UnityEditor;
using UnityEngine;

namespace TSDF.EditorTools
{
    public class PrintExportPanel : EditorWindow
    {
        [MenuItem("Window/Print Export")]
        public static void Open() => GetWindow<PrintExportPanel>("Print Export");

        private Vector2 _scroll;

        private void OnGUI()
        {
            var pe = FindFirstObjectByType<TSDFPrintExporter>(FindObjectsInactive.Include);
            if (pe == null)
            {
                EditorGUILayout.HelpBox("No TSDFPrintExporter in the open scene(s). " +
                                        "Add one and wire volume + curves.", MessageType.Info);
                return;
            }

            _scroll = EditorGUILayout.BeginScrollView(_scroll);

            DrawFreezeRow();
            EditorGUILayout.Space(10);
            DrawSettings(pe);
            EditorGUILayout.Space(10);
            DrawOperations(pe);

            EditorGUILayout.EndScrollView();
        }

        // ---- 1: freeze the moment ------------------------------------------------
        private void DrawFreezeRow()
        {
            EditorGUILayout.LabelField("1. Freeze the moment (再生を止めて瞬間を固定)", EditorStyles.boldLabel);
            var rec = FindFirstObjectByType<SensorRecorder>(FindObjectsInactive.Include);
            if (rec == null)
            {
                EditorGUILayout.HelpBox("No SensorRecorder in the scene.", MessageType.Info);
                return;
            }

            if (!string.IsNullOrEmpty(rec.TransportStatus))
                EditorGUILayout.HelpBox(rec.TransportStatus, MessageType.None);

            using (new EditorGUI.DisabledScope(!Application.isPlaying || !rec.IsPlaying))
            using (new EditorGUILayout.HorizontalScope())
            {
                if (GUILayout.Button(rec.IsPaused ? "Resume" : "Pause", GUILayout.Height(22)))
                    rec.TogglePause();
                if (GUILayout.Button("◀ Step", GUILayout.Height(22))) rec.StepBackward();
                if (GUILayout.Button("Step ▶", GUILayout.Height(22))) rec.StepForward();
            }
        }

        // ---- 2: settings ---------------------------------------------------------
        private void DrawSettings(TSDFPrintExporter pe)
        {
            EditorGUILayout.LabelField("2. Settings", EditorStyles.boldLabel);

            float voxel = pe.volume != null ? pe.volume.voxelSize : 0f;

            EditorGUI.BeginChangeCheck();

            EditorGUILayout.LabelField("Curve tubes (curved line)", EditorStyles.miniBoldLabel);
            int stride = EditorGUILayout.IntSlider(
                new GUIContent("Print Seed Stride", "間引き：何本置きに1本エクスポートするか。40 → 20000本中約500本。"),
                pe.printSeedStride, 1, 400);
            float radius = EditorGUILayout.Slider(
                new GUIContent("Print Radius (m)", "カーブチューブの半径。STL チューブ直接同梱なら解像度制約なし。" +
                    "Fuse curves（voxel 融合）を使う場合のみ直径 3 voxel 以上必要。"),
                pe.printRadius, 0.002f, 0.05f);
            if (voxel > 1e-6f && !pe.stlIncludeCurveTubes)
            {
                // The 3-voxel MC floor only applies to the voxel-fuse path — the
                // direct STL tubes never touch the volume.
                float diaVox = radius * 2f / voxel;
                EditorGUILayout.HelpBox(
                    $"直径 {radius * 2000f:0.0} mm ≈ {diaVox:0.0} voxel (voxelSize {voxel * 1000f:0.0} mm)" +
                    (diaVox < 3f ? " — 3 voxel 未満: 断面がガタガタになります" : " — OK"),
                    diaVox < 3f ? MessageType.Warning : MessageType.None);
            }
            float bridgeScale = EditorGUILayout.Slider(
                new GUIContent("Bridge Radius Scale", "体との接続部の根元の太らせ倍率（フィレット）。"),
                pe.bridgeRadiusScale, 1f, 3f);
            float maxBridge = EditorGUILayout.Slider(
                new GUIContent("Max Bridge Length (m)", "これより長い接続支柱は作らない（誤分類ガード）。"),
                pe.maxBridgeLength, 0.05f, 1f);

            EditorGUILayout.Space(4);
            EditorGUILayout.LabelField("Hole closing (穴塞ぎ)", EditorStyles.miniBoldLabel);
            int closeR = EditorGUILayout.IntSlider(
                new GUIContent("Close Radius Voxels", "2×この幅までの開口を塞ぐ。大きいと凹みも丸まる。"),
                pe.closeRadiusVoxels, 1, 16);
            if (voxel > 1e-6f)
                EditorGUILayout.HelpBox($"塞げる開口 ≈ {2 * closeR * voxel * 100f:0.0} cm まで", MessageType.None);
            bool keepLargest = EditorGUILayout.ToggleLeft(
                new GUIContent("Keep Largest Only (浮遊物を削除)",
                    "本体に接続していない成分を削除。削除量はログに出る（接続漏れ検出を兼ねる）。"),
                pe.keepLargestOnly);

            EditorGUILayout.Space(4);
            EditorGUILayout.LabelField("Export", EditorStyles.miniBoldLabel);
            float heightMm = EditorGUILayout.Slider(
                new GUIContent("Target Height Mm", "プリント時の高さ。STL は mm 単位で書き出される。"),
                pe.targetHeightMm, 50f, 1000f);
            int smooth = EditorGUILayout.IntSlider(
                new GUIContent("Smooth Iterations", "書き出し時の Taubin 平滑化（縮まない）。0=OFF。" +
                    "MC の階段状ガタつきを除去する。表示中の TSDF mesh には影響しない。"),
                pe.smoothIterations, 0, 30);
            bool stlBody = EditorGUILayout.ToggleLeft(
                new GUIContent("STL: Include Body (体のメッシュを同梱)",
                    "OFF = チューブのみの STL（審美バリアント）。体シェルと、体に接続する" +
                    "ブリッジ（接続先が無くなるため）を出力しない。床プレートは独立フラグのままで、" +
                    "体 OFF 時はチューブの接地帯を中心にする。"),
                pe.stlIncludeBody);
            bool stlTubes = EditorGUILayout.ToggleLeft(
                new GUIContent("STL: Include Curve Tubes (フル解像度のチューブで同梱)",
                    "カーブ＋体への接続ブリッジを 1 回の CSEmitSegs（Fuse curves と同一のシード集合）" +
                    "から閉じたチューブメッシュとして再構成し、STL に直接同梱する。voxel 化を" +
                    "通らないので解像度が落ちず、Fuse curves は不要（すると二重になる）。" +
                    "重なったシェルはスライサーが union する。" +
                    "OFF = 旧方式（Fuse curves で voxel に焼いた分だけが STL に入る）。"),
                pe.stlIncludeCurveTubes);
            int stlSides = EditorGUILayout.IntSlider(
                new GUIContent("STL Curve Sides", "STL チューブ断面の角数。2mm ワイヤ相当なら 6 で丸く見える。"),
                pe.stlCurveSides, 3, 12);
            bool wire = EditorGUILayout.ToggleLeft(
                new GUIContent("STL: Root Wireframe (体OFF時、根本同士をワイヤ接続)",
                    "体を出さないとき、カーブの体側アンカー点同士を細いワイヤで接続する。" +
                    "最小全域木で必ず1つの連結部品にし（粉末プリントは非連結パーツが脱落する）、" +
                    "さらに近傍N本を追加。アンカーはダンサー表面の点なので、" +
                    "薄っすら身体のワイヤーフレームに見える。ブリッジはカーブ→ワイヤ網の接続として残る。"),
                pe.stlRootWireframe);
            int wireN = EditorGUILayout.IntSlider(
                new GUIContent("Wireframe Neighbors", "全域木に加えて各アンカーから近い順に何本ワイヤを張るか。" +
                    "0=木のみ（最疎）、2でメッシュらしい密度。"),
                pe.stlWireframeNeighbors, 0, 4);
            float wireMax = EditorGUILayout.Slider(
                new GUIContent("Wireframe Max Edge (m)", "近傍ワイヤの最長（これ以上は張らない — " +
                    "手→腰のような体を横切る弦を防ぐ）。全域木の辺は連結優先でこの制限を受けない。"),
                pe.stlWireframeMaxEdge, 0.05f, 0.5f);
            // 床プレートは常に同梱（無いと自立しない — 2026-07-17 のルール）。トグルは無い。
            float stlFloorSz = EditorGUILayout.Slider(
                new GUIContent("STL Floor Size (m)", "床プレートの一辺（実寸 m の正方形、常に同梱 — " +
                    "無いと自立しない）。中心はダンサー（接地帯の重心）。" +
                    "プリント寸法は他と同じく Target Height Mm のスケールに従う。"),
                pe.stlFloorSize, 0.5f, 3f);
            float stlFloorTh = EditorGUILayout.Slider(
                new GUIContent("STL Floor Thickness (m)", "床プレートの厚み（実寸 m）。" +
                    "0.02m は 1/8 スケール出力でおよそ 2.5mm。"),
                pe.stlFloorThickness, 0.005f, 0.1f);

            EditorGUILayout.Space(4);
            EditorGUILayout.LabelField("Web export (GLB + USDZ)", EditorStyles.miniBoldLabel);
            bool webCurves = EditorGUILayout.ToggleLeft(
                new GUIContent("Include Curves (表示解像度のチューブで同梱)",
                    "カーブを描画バッファから読み戻してチューブメッシュ化し、TSDF mesh と一緒に書き出す。" +
                    "voxel 化を通らないので解像度は落ちない。Fuse curves は不要（すると二重になる）。"),
                pe.webIncludeCurves);
            int webStride = EditorGUILayout.IntSlider(
                new GUIContent("Web Curve Stride", "何本置きに1本チューブ化するか。1本≈1000三角形。" +
                    "40 → 20000本中約500本 ≈ 0.5M 三角形。下げると密になるがファイルが大きくなる（USDZ はテキスト形式）。"),
                pe.webCurveStride, 1, 400);
            int webSides = EditorGUILayout.IntSlider(
                new GUIContent("Web Curve Sides", "チューブ断面の角数。リボン幅なら 4-6 で丸く見える。"),
                pe.webCurveSides, 3, 12);
            int webTris = EditorGUILayout.IntField(
                new GUIContent("Web Mesh Target Tris", "TSDF mesh をこの三角形数まで削減して書き出す" +
                    "（QEM デシメーション、色保持）。0=削減なし。STL には影響しない。" +
                    "150k + デフォルトのカーブで両ファイル 10MB 以内が目安。"),
                pe.webMeshTargetTris);
            float webTol = EditorGUILayout.Slider(
                new GUIContent("Web Curve Tolerance (m)", "チューブ化前のポリライン簡略化の許容誤差" +
                    "（Douglas-Peucker）。1.5mm で三角形数が半分以下、見た目は変わらない。0=OFF。"),
                pe.webCurveTolerance, 0f, 0.01f);
            string usdPy = EditorGUILayout.TextField(
                new GUIContent("Usd Python Path", "usd-core 入り python のパス（空=自動検出: ~/.venvs/usd → " +
                    "システム python）。見つかれば USDZ をバイナリ usdc に変換（3-4倍小さい）、" +
                    "無ければ ASCII usda のまま。"),
                pe.usdPythonPath);

            if (EditorGUI.EndChangeCheck())
            {
                Undo.RecordObject(pe, "Print export settings");
                pe.printSeedStride = stride;
                pe.printRadius = radius;
                pe.bridgeRadiusScale = bridgeScale;
                pe.maxBridgeLength = maxBridge;
                pe.closeRadiusVoxels = closeR;
                pe.keepLargestOnly = keepLargest;
                pe.targetHeightMm = heightMm;
                pe.smoothIterations = smooth;
                pe.stlIncludeBody = stlBody;
                pe.stlIncludeCurveTubes = stlTubes;
                pe.stlCurveSides = stlSides;
                pe.stlRootWireframe = wire;
                pe.stlWireframeNeighbors = wireN;
                pe.stlWireframeMaxEdge = wireMax;
                pe.stlFloorSize = stlFloorSz;
                pe.stlFloorThickness = stlFloorTh;
                pe.webIncludeCurves = webCurves;
                pe.webCurveStride = webStride;
                pe.webCurveSides = webSides;
                pe.webMeshTargetTris = Mathf.Max(0, webTris);
                pe.webCurveTolerance = webTol;
                pe.usdPythonPath = usdPy;
                EditorUtility.SetDirty(pe);
            }
        }

        // ---- 3: operations -------------------------------------------------------
        private void DrawOperations(TSDFPrintExporter pe)
        {
            EditorGUILayout.LabelField("3. Operations", EditorStyles.boldLabel);

            bool ready = Application.isPlaying && pe.VolumeReady;
            using (new EditorGUILayout.HorizontalScope())
            {
                using (new EditorGUI.DisabledScope(!ready || pe.curves == null))
                {
                    if (GUILayout.Button(new GUIContent("Fuse curves",
                            "カーブをチューブ化して表示中の TSDF に融合（何度押しても累積しない）。" +
                            "STL: Include Curve Tubes が ON のときは表示確認用 — STL には不要（二重になる）"),
                            GUILayout.Height(26)))
                        pe.FuseCurves();
                }
                using (new EditorGUI.DisabledScope(!ready))
                {
                    if (GUILayout.Button(new GUIContent("Close holes",
                            "穴塞ぎ＋未接続チェック"), GUILayout.Height(26)))
                        pe.CloseHoles();
                    if (GUILayout.Button(new GUIContent("Export STL",
                            "~/Documents/FloatingVectorsPrints/ に書き出し。" +
                            "カーブは表示解像度のチューブ＋ブリッジとして自動同梱（Fuse 不要）"),
                            GUILayout.Height(26)))
                        pe.ExportStl();
                }
                using (new EditorGUI.DisabledScope(!ready || !pe.HasSnapshot))
                {
                    if (GUILayout.Button(new GUIContent("Restore",
                            "全プリント操作を取り消して融合前に戻す"), GUILayout.Height(26)))
                        pe.RestoreSnapshot();
                }
            }

            using (new EditorGUI.DisabledScope(!ready))
            {
                // With direct STL tubes the voxel fuse must be skipped (the tubes
                // would land twice: once meshed, once voxelised).
                bool direct = pe.stlIncludeCurveTubes;
                if (GUILayout.Button(new GUIContent(
                        direct ? "Close → Export STL (ワンクリック)"
                               : "Fuse → Close → Export STL (ワンクリック)",
                        direct ? "3Dプリント一連: 穴塞ぎ → STL 書き出し（カーブはチューブとして自動同梱）。" +
                                 "やり直しは Restore で穴塞ぎ前に戻せる。"
                               : "3Dプリント一連: カーブ融合 → 穴塞ぎ → STL 書き出しを連続実行。" +
                                 "やり直しは Restore で融合前に戻せる。"), GUILayout.Height(30)))
                {
                    if (!direct && pe.curves != null) pe.FuseCurves();
                    pe.CloseHoles();
                    pe.ExportStl();
                }
            }

            using (new EditorGUI.DisabledScope(!ready))
            {
                if (GUILayout.Button(new GUIContent("Export Web (GLB + USDZ)",
                        "ウェブ表示用 .glb（頂点カラー）と iPhone AR Quick Look 用 .usdz" +
                        "（色はテクスチャに焼き込み）を実寸(m)で書き出し。カーブは表示解像度のまま" +
                        "チューブとして自動同梱（Fuse curves 不要）。出力先は " +
                        "~/Documents/FloatingVectorsPrints/。"),
                        GUILayout.Height(26)))
                    pe.ExportWeb();
            }

            if (!Application.isPlaying)
                EditorGUILayout.HelpBox("操作はプレイ中のみ。再生 → Pause → Fuse → Close → Export の順。", MessageType.None);
            else if (!string.IsNullOrEmpty(pe.StatusText))
                EditorGUILayout.HelpBox(pe.StatusText, MessageType.Info);

            using (new EditorGUILayout.HorizontalScope())
            {
                GUILayout.FlexibleSpace();
                if (GUILayout.Button("出力フォルダを開く", GUILayout.Width(130)))
                {
                    string dir = System.IO.Path.Combine(
                        System.Environment.GetFolderPath(System.Environment.SpecialFolder.UserProfile),
                        "Documents", "FloatingVectorsPrints");
                    System.IO.Directory.CreateDirectory(dir);
                    EditorUtility.RevealInFinder(dir);
                }
                if (GUILayout.Button("Select exporter", GUILayout.Width(110)))
                {
                    Selection.activeGameObject = pe.gameObject;
                    EditorGUIUtility.PingObject(pe.gameObject);
                }
            }
        }

        // Keep live while playing (status/state changes from code).
        private void OnInspectorUpdate() => Repaint();
    }
}
