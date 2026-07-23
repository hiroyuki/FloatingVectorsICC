// Render UI — one window for every rendering/look parameter that is not part
// of the experience flow itself: scene lighting, shadows (URP pipeline asset),
// ambient, the floor grid + floor reflection, and the curve shading (the
// PBR-ish knobs of the ribbon shader). Companion to the Control Panel (which
// keeps transport/tuning) — this window owns the LOOK.
//
// Everything edits live scene objects (Undo-recorded, SetDirty), so tuning in
// Play mode shows instantly; values changed in Play are lost on Stop like any
// Inspector edit — use the Save Scene button while in EDIT mode to persist.
// The URP shadow settings live on the pipeline ASSET and persist immediately.
//
// Window > Render UI

using UnityEditor;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Rendering.Universal;

namespace FloatingVectors.EditorTools
{
    public class RenderUIWindow : EditorWindow
    {
        [MenuItem("Window/Render UI")]
        private static void Open() => GetWindow<RenderUIWindow>("Render UI");

        private Vector2 _scroll;

        private void Update()
        {
            if (Application.isPlaying) Repaint(); // live readouts while tuning
        }

        private void OnGUI()
        {
            using var scroll = new EditorGUILayout.ScrollViewScope(_scroll);
            _scroll = scroll.scrollPosition;

            DrawLights();
            EditorGUILayout.Space(8);
            DrawAmbient();
            EditorGUILayout.Space(8);
            DrawUrpShadows();
            EditorGUILayout.Space(8);
            DrawFloor();
            EditorGUILayout.Space(8);
            DrawCurveShading();
            EditorGUILayout.Space(10);

            using (new EditorGUI.DisabledScope(Application.isPlaying))
                if (GUILayout.Button("Save Scene（シーン値を保存）", GUILayout.Height(26)))
                    UnityEditor.SceneManagement.EditorSceneManager.SaveOpenScenes();
            if (Application.isPlaying)
                EditorGUILayout.HelpBox(
                    "Play 中の変更は Stop で消えます（URP シャドウ設定はアセットなので残る）。" +
                    "決まった値は控えて、Stop 後にここで再入力 → Save Scene。",
                    MessageType.Info);
        }

        // ---- lights ----

        private void DrawLights()
        {
            EditorGUILayout.LabelField("ライト", EditorStyles.boldLabel);
            var lights = FindObjectsByType<Light>(FindObjectsInactive.Include, FindObjectsSortMode.InstanceID);
            if (lights.Length == 0)
            {
                EditorGUILayout.HelpBox("シーンに Light がありません。", MessageType.Info);
                return;
            }
            foreach (var l in lights)
            {
                EditorGUILayout.LabelField($"{l.gameObject.name}（{l.type}）", EditorStyles.miniBoldLabel);
                EditorGUI.BeginChangeCheck();
                bool en = EditorGUILayout.ToggleLeft("有効", l.enabled && l.gameObject.activeInHierarchy);
                float intensity = EditorGUILayout.Slider("Intensity", l.intensity, 0f, 8f);
                Color color = EditorGUILayout.ColorField("Color", l.color);
                Vector3 euler = l.transform.rotation.eulerAngles;
                float pitch = EditorGUILayout.Slider("角度 上下（X）", Normalize(euler.x), -10f, 90f);
                float yaw = EditorGUILayout.Slider("角度 方位（Y）", Normalize(euler.y), -180f, 180f);
                var shadows = (LightShadows)EditorGUILayout.EnumPopup("Shadows", l.shadows);
                float shadowStrength = EditorGUILayout.Slider("Shadow Strength", l.shadowStrength, 0f, 1f);
                if (EditorGUI.EndChangeCheck())
                {
                    Undo.RecordObjects(new Object[] { l, l.gameObject, l.transform }, "Render UI light");
                    l.enabled = en;
                    if (l.gameObject.activeSelf != en && !l.gameObject.activeInHierarchy == en)
                        l.gameObject.SetActive(en);
                    l.intensity = intensity;
                    l.color = color;
                    l.transform.rotation = Quaternion.Euler(pitch, yaw, 0f);
                    l.shadows = shadows;
                    l.shadowStrength = shadowStrength;
                    EditorUtility.SetDirty(l);
                    EditorUtility.SetDirty(l.transform);
                }
            }
        }

        private static float Normalize(float a)
        {
            a %= 360f;
            if (a > 180f) a -= 360f;
            return a;
        }

        // ---- ambient ----

        private void DrawAmbient()
        {
            EditorGUILayout.LabelField("環境光（Ambient）", EditorStyles.boldLabel);
            EditorGUI.BeginChangeCheck();
            var mode = (AmbientMode)EditorGUILayout.EnumPopup("Mode", RenderSettings.ambientMode);
            float ambientIntensity = RenderSettings.ambientIntensity;
            Color ambientColor = RenderSettings.ambientLight;
            if (mode == AmbientMode.Skybox)
                ambientIntensity = EditorGUILayout.Slider("Intensity", ambientIntensity, 0f, 3f);
            else
                ambientColor = EditorGUILayout.ColorField("Ambient Color", ambientColor);
            if (EditorGUI.EndChangeCheck())
            {
                RenderSettings.ambientMode = mode;
                RenderSettings.ambientIntensity = ambientIntensity;
                RenderSettings.ambientLight = ambientColor;
            }
        }

        // ---- URP shadows (pipeline asset — persists immediately) ----

        private void DrawUrpShadows()
        {
            EditorGUILayout.LabelField("シャドウ（URP アセット — 即永続）", EditorStyles.boldLabel);
            if (GraphicsSettings.currentRenderPipeline is not UniversalRenderPipelineAsset urp)
            {
                EditorGUILayout.HelpBox("URP パイプラインアセットが見つかりません。", MessageType.Info);
                return;
            }
            EditorGUI.BeginChangeCheck();
            float dist = EditorGUILayout.Slider("Shadow Distance (m)", urp.shadowDistance, 0f, 100f);
            int cascades = EditorGUILayout.IntSlider("Cascade Count", urp.shadowCascadeCount, 1, 4);
            if (EditorGUI.EndChangeCheck())
            {
                Undo.RecordObject(urp, "Render UI URP shadows");
                urp.shadowDistance = dist;
                urp.shadowCascadeCount = cascades;
                EditorUtility.SetDirty(urp);
            }
            EditorGUILayout.LabelField("Asset", urp.name);
        }

        // ---- floor grid + reflection ----

        private void DrawFloor()
        {
            EditorGUILayout.LabelField("床（グリッド / リフレクション）", EditorStyles.boldLabel);
            var floor = FindFirstObjectByType<PointCloud.FloorOrigin>(FindObjectsInactive.Include);
            if (floor == null)
            {
                EditorGUILayout.HelpBox("FloorOrigin がシーンにありません。", MessageType.Info);
                return;
            }
            EditorGUI.BeginChangeCheck();
            bool showGrid = EditorGUILayout.ToggleLeft("グリッド表示", floor.showGrid);
            Color gridColor = EditorGUILayout.ColorField("グリッド色", floor.gridColor);
            float inner = EditorGUILayout.Slider("内側ライン輝度", floor.innerLineBrightness, 0f, 1f);
            bool showShadow = EditorGUILayout.ToggleLeft("床リフレクション表示", floor.showShadow);
            Color shadowColor = EditorGUILayout.ColorField("リフレクション色（αで濃さ）", floor.shadowColor);
            float blur = EditorGUILayout.Slider("ブラー半径 (m)", floor.shadowBlurRadius, 0f, 0.3f);
            int blurSamples = EditorGUILayout.IntSlider("ブラーサンプル", floor.shadowBlurSamples, 1, 16);
            if (EditorGUI.EndChangeCheck())
            {
                Undo.RecordObject(floor, "Render UI floor");
                floor.showGrid = showGrid;
                floor.gridColor = gridColor;
                floor.innerLineBrightness = inner;
                floor.showShadow = showShadow;
                floor.shadowColor = shadowColor;
                floor.shadowBlurRadius = blur;
                floor.shadowBlurSamples = blurSamples;
                EditorUtility.SetDirty(floor);
            }
        }

        // ---- curve shading (the ribbons' PBR-ish knobs) ----

        private void DrawCurveShading()
        {
            EditorGUILayout.LabelField("カーブ シェーディング", EditorStyles.boldLabel);
            var curves = FindFirstObjectByType<BodyTracking.PointCloudMotionCurves>(FindObjectsInactive.Include);
            if (curves == null)
            {
                EditorGUILayout.HelpBox("PointCloudMotionCurves がシーンにありません。", MessageType.Info);
                return;
            }
            EditorGUI.BeginChangeCheck();
            float brightness = EditorGUILayout.Slider("Brightness", curves.brightness, 0f, 3f);
            float round = EditorGUILayout.Slider("Round shading（0=フラット 1=円柱）", curves.round, 0f, 1f);
            float rim = EditorGUILayout.Slider("Rim Boost", curves.rimBoost, 0f, 2f);
            float tailAlpha = EditorGUILayout.Slider("Tail Alpha", curves.tailAlpha, 0f, 1f);
            float tailFade = EditorGUILayout.Slider("Tail Fade Length", curves.tailFadePow, 0.5f, 6f);
            if (EditorGUI.EndChangeCheck())
            {
                Undo.RecordObject(curves, "Render UI curves");
                curves.brightness = brightness;
                curves.round = round;
                curves.rimBoost = rim;
                curves.tailAlpha = tailAlpha;
                curves.tailFadePow = tailFade;
                EditorUtility.SetDirty(curves);
            }
            EditorGUILayout.HelpBox(
                "注意: tailAlpha は体験モード中フェーズ切替で上書きされます" +
                "（ライブ=シーン値 / できたよ再生=ExperienceConfig の presentation 値）。",
                MessageType.None);
        }
    }
}
