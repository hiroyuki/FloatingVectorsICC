// Dedicated 3D-print panel: Window > Print Export.
//
// Everything for the print workflow in one place, in workflow order:
//   1. freeze the moment  — playback pause row (IRecorderTransport)
//   2. tune               — curve tube / hole closing / export settings
//   3. operate            — Fuse curves / Close holes / Export STL / Restore
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
                new GUIContent("Print Radius (m)", "カーブチューブの半径。直径 3 voxel 以上でないと MC がガタガタになる。"),
                pe.printRadius, 0.002f, 0.05f);
            if (voxel > 1e-6f)
            {
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
            bool ply = EditorGUILayout.ToggleLeft(
                new GUIContent("Export Ply With Color", "フルカラー印刷サービス用の頂点色付き PLY も出力。"),
                pe.exportPlyWithColor);

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
                pe.exportPlyWithColor = ply;
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
                            "カーブをチューブ化して表示中の TSDF に融合（何度押しても累積しない）"),
                            GUILayout.Height(26)))
                        pe.FuseCurves();
                }
                using (new EditorGUI.DisabledScope(!ready))
                {
                    if (GUILayout.Button(new GUIContent("Close holes",
                            "穴塞ぎ＋未接続チェック"), GUILayout.Height(26)))
                        pe.CloseHoles();
                    if (GUILayout.Button(new GUIContent("Export STL",
                            "~/Documents/FloatingVectorsPrints/ に書き出し"), GUILayout.Height(26)))
                        pe.ExportStl();
                }
                using (new EditorGUI.DisabledScope(!ready || !pe.HasSnapshot))
                {
                    if (GUILayout.Button(new GUIContent("Restore",
                            "全プリント操作を取り消して融合前に戻す"), GUILayout.Height(26)))
                        pe.RestoreSnapshot();
                }
            }

            if (!Application.isPlaying)
                EditorGUILayout.HelpBox("操作はプレイ中のみ。再生 → Pause → Fuse → Close → Export の順。", MessageType.None);
            else if (!string.IsNullOrEmpty(pe.StatusText))
                EditorGUILayout.HelpBox(pe.StatusText, MessageType.Info);

            using (new EditorGUILayout.HorizontalScope())
            {
                GUILayout.FlexibleSpace();
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
