// Runtime slider for BonePoseHistory.historySamples (the motion-curve time window),
// drawn with IMGUI so it lands on Display 1 — the otherwise-black operator display —
// in a standalone multi-display build (IMGUI renders to the primary display only).
// In the Editor it draws over the Game view. Mirrors the Control Panel's
// "Motion history" tunable so the two UIs stay the same knob.

using UnityEngine;

namespace BodyTracking
{
    public class HistorySamplesHud : MonoBehaviour
    {
        [Tooltip("Bone pose history whose History Samples value the slider drives. " +
                 "Auto-resolves the first BonePoseHistory when left empty.")]
        public BonePoseHistory history;

        [Tooltip("Show/hide the HUD.")]
        public bool visible = true;

        [Tooltip("Top-left corner of the HUD in scaled GUI pixels.")]
        public Vector2 position = new Vector2(24, 24);

        [Tooltip("Slider width in scaled GUI pixels.")]
        public float width = 420f;

        [Tooltip("Overall HUD scale (IMGUI's default skin is small on 4K displays).")]
        [Range(1f, 4f)] public float uiScale = 2f;

        private void OnGUI()
        {
            if (!visible) return;
            if (history == null)
            {
                history = FindFirstObjectByType<BonePoseHistory>();
                if (history == null) return;
            }

            GUI.matrix = Matrix4x4.Scale(Vector3.one * uiScale);

            // Tunable 0 is History Samples (frames) — same knob as the Control Panel.
            float min = history.TunableMin(0);
            float max = history.TunableMax(0);
            int cur = Mathf.RoundToInt(history.TunableValue(0));

            GUI.Label(new Rect(position.x, position.y, width, 22),
                      $"{history.TunableName(0)}: {cur}");
            float v = GUI.HorizontalSlider(
                new Rect(position.x, position.y + 24, width, 20), cur, min, max);
            int next = Mathf.RoundToInt(v);
            if (next != cur) history.SetTunableValue(0, next);

            GUI.matrix = Matrix4x4.identity;
        }
    }
}
