// Escape-to-quit for the standalone build.
//
// Why a HOLD and not a tap: the build runs fullscreen on three displays with no
// window chrome, so there is no other way out — but Escape is already a normal
// key inside the calibration UI (exit assign mode / toggle floor tune). A tap
// keeps going to those; only a deliberate hold quits.
//
// In the Editor this is a no-op on purpose. Play mode must not be stopped from
// a hotkey: Inspector tweaks made while playing are lost on stop, and the
// recorder's playback state does not survive it.
//
// Self-installing (RuntimeInitializeOnLoadMethod) so there is nothing to wire in
// the scene — the whole point is a way out of a build that has no window chrome,
// and that must not depend on someone remembering to add a component.

using UnityEngine;

namespace Shared
{
    [DisallowMultipleComponent]
    public class AppQuitHotkey : MonoBehaviour
    {
        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
        private static void Install()
        {
            if (FindFirstObjectByType<AppQuitHotkey>() != null) return; // scene copy wins
            var go = new GameObject("[AppQuitHotkey]");
            go.AddComponent<AppQuitHotkey>();
            DontDestroyOnLoad(go);
        }

        [Tooltip("Key to hold to quit the built application.")]
        public KeyCode quitKey = KeyCode.Escape;

        [Tooltip("How long the key must be held. Short taps fall through to the " +
                 "calibration UI, which also uses Escape.")]
        [Range(0.2f, 5f)] public float holdSeconds = 1.5f;

        [Tooltip("Show a 'hold to quit' progress hint while the key is down.")]
        public bool showHint = true;

        private float _heldFor;

        private void Update()
        {
            if (Input.GetKey(quitKey)) _heldFor += Time.unscaledDeltaTime;
            else _heldFor = 0f;

            if (_heldFor < holdSeconds) return;
            _heldFor = 0f;

            Debug.Log($"[AppQuitHotkey] {quitKey} held {holdSeconds:0.0}s — quitting.");
            ShutdownProfiler.Mark("AppQuitHotkey: Application.Quit() requested");
#if UNITY_EDITOR
            // Deliberately does not stop play mode; see the header note.
            Debug.Log("[AppQuitHotkey] Editor: quit suppressed (build-only).");
#else
            Application.Quit();
#endif
        }

        // Operator display only (IMGUI renders to display 1 / index 0).
        private void OnGUI()
        {
            if (!showHint || _heldFor <= 0f) return;

            float t = Mathf.Clamp01(_heldFor / Mathf.Max(0.01f, holdSeconds));
            const float w = 320f, h = 46f;
            var r = new Rect((Screen.width - w) * 0.5f, Screen.height - h - 40f, w, h);

            GUI.color = new Color(0f, 0f, 0f, 0.6f);
            GUI.DrawTexture(r, Texture2D.whiteTexture);
            GUI.color = new Color(1f, 0.25f, 0.2f, 0.9f);
            GUI.DrawTexture(new Rect(r.x, r.yMax - 6f, r.width * t, 6f), Texture2D.whiteTexture);
            GUI.color = Color.white;

            var style = new GUIStyle(GUI.skin.label)
            {
                alignment = TextAnchor.MiddleCenter,
                fontSize = 20
            };
            GUI.Label(r, $"hold {quitKey} to quit", style);
        }
    }
}
