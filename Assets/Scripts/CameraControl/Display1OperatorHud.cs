// Operator HUD drawn with IMGUI so it lands on Display 1 — the otherwise-black
// operator display — in a standalone multi-display build (IMGUI renders to the
// primary display only). In the Editor it draws over the Game view.
//
// Controls:
//   - History Samples slider: BonePoseHistory's motion-curve time window (same
//     knob as the Control Panel's "Motion history" tunable).
//   - Auto Orbit toggle: turns on the idle auto-orbit on every stage camera's
//     CameraOrbitController, keeping the controllers enabled regardless of the
//     pause state (via PauseOrbitGate.autoOrbitOverride on gated cameras).
//
// Lives in the CameraControl folder (Assembly-CSharp): it needs both
// BodyTracking (BonePoseHistory) and the orbit controllers, and only
// Assembly-CSharp can see both.

using BodyTracking;
using UnityEngine;

namespace CameraControl
{
    public class Display1OperatorHud : MonoBehaviour
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

        [Tooltip("Orbit controllers the Auto Orbit toggle drives. Auto-resolves every " +
                 "CameraOrbitController in the scene (including disabled ones) when empty.")]
        public CameraOrbitController[] orbits;

        private bool _autoOrbit;

        private void OnEnable()
        {
            if (orbits == null || orbits.Length == 0)
                orbits = FindObjectsByType<CameraOrbitController>(FindObjectsSortMode.None);
            // Reflect an override that is already on (e.g. HUD re-enabled mid-session).
            foreach (var o in orbits)
                if (o != null && o.TryGetComponent(out PauseOrbitGate g) && g.autoOrbitOverride)
                    _autoOrbit = true;
        }

        private void OnGUI()
        {
            if (!visible) return;
            if (history == null)
            {
                history = FindFirstObjectByType<BonePoseHistory>();
                if (history == null) return;
            }

            GUI.matrix = Matrix4x4.Scale(Vector3.one * uiScale);

            // Panel backdrop so the widgets read against the black operator display.
            GUI.Box(new Rect(position.x - 12, position.y - 10, width + 24, 96), GUIContent.none);

            // Tunable 0 is History Samples (frames) — same knob as the Control Panel.
            float min = history.TunableMin(0);
            float max = history.TunableMax(0);
            int cur = Mathf.RoundToInt(history.TunableValue(0));

            GUI.Label(new Rect(position.x, position.y, width, 22),
                      $"{history.TunableName(0)}: {cur}");

            // Explicit track: the default skin's slider rail is nearly invisible on a
            // black background — only the thumb showed.
            GUI.color = new Color(1f, 1f, 1f, 0.35f);
            GUI.DrawTexture(new Rect(position.x, position.y + 24 + 8, width, 4), Texture2D.whiteTexture);
            GUI.color = Color.white;

            float v = GUI.HorizontalSlider(
                new Rect(position.x, position.y + 24, width, 20), cur, min, max);
            int next = Mathf.RoundToInt(v);
            if (next != cur) history.SetTunableValue(0, next);

            bool auto = GUI.Toggle(new Rect(position.x, position.y + 52, width, 24),
                                   _autoOrbit, " Auto Orbit (stage cams)");
            if (auto != _autoOrbit) ApplyAutoOrbit(auto);

            GUI.matrix = Matrix4x4.identity;
        }

        // ON: every stage camera's orbit controller runs its idle auto-orbit,
        // regardless of pause (gated cameras get autoOrbitOverride, ungated ones are
        // enabled directly). OFF: back to the default policy — gated cameras follow
        // the pause state, ungated ones go back to disabled.
        private void ApplyAutoOrbit(bool on)
        {
            _autoOrbit = on;
            foreach (var o in orbits)
            {
                if (o == null) continue;
                o.autoOrbit = on;
                if (o.TryGetComponent(out PauseOrbitGate gate)) gate.autoOrbitOverride = on;
                else o.enabled = on;
            }
        }
    }
}
