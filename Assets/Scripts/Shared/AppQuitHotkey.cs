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

using System.Collections;
using UnityEngine;

namespace Shared
{
    [DisallowMultipleComponent]
    public class AppQuitHotkey : MonoBehaviour
    {
        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
        private static void Install()
        {
            var existing = FindFirstObjectByType<AppQuitHotkey>();
            if (existing != null) { s_instance = existing; return; } // scene copy wins
            var go = new GameObject("[AppQuitHotkey]");
            s_instance = go.AddComponent<AppQuitHotkey>();
            DontDestroyOnLoad(go);
        }

        private static AppQuitHotkey s_instance;
        // Two states, not one flag: "started" must REJECT every further quit request
        // (a second Alt+F4, the window close button, another component calling
        // Application.Quit) because accepting one mid-teardown kills the process with
        // camera pipelines still open. Only the coroutine's own final request, made
        // after setting "complete", is allowed through.
        private static bool s_teardownStarted;
        private static bool s_teardownComplete;

        [Tooltip("Key to hold to quit the built application.")]
        public KeyCode quitKey = KeyCode.Escape;

        [Tooltip("How long the key must be held. Short taps fall through to the " +
                 "calibration UI, which also uses Escape.")]
        [Range(0.2f, 5f)] public float holdSeconds = 1.5f;

        [Tooltip("Show a 'hold to quit' progress hint while the key is down.")]
        public bool showHint = true;

        [Tooltip("Last-resort bound on the staged teardown. Must sit above whatever the " +
                 "staged subsystems allow themselves (SensorManager caps its pipeline " +
                 "stops at hardStopTimeoutSeconds) — this only fires when one of them " +
                 "never returns at all.")]
        public float teardownWatchdogSeconds = 90f;

        private float _heldFor;
        private float _teardownElapsed;

        // Every quit route — the Esc hold, Alt+F4, the window's close button, a
        // Application.Quit() from anywhere else — funnels through wantsToQuit, so the
        // "終了中" splash is armed here rather than next to the hotkey. Returning false
        // once postpones the quit by a frame; that frame is what puts the splash on
        // screen before ~4.5s of ob_pipeline_stop freezes everything.
        private void OnEnable()
        {
            // A scene copy's OnEnable runs before Install() gets to set this.
            if (s_instance == null) s_instance = this;
            Application.wantsToQuit -= OnWantsToQuit;
            Application.wantsToQuit += OnWantsToQuit;
        }

        private void OnDisable() => Application.wantsToQuit -= OnWantsToQuit;

        private static bool OnWantsToQuit()
        {
            if (s_teardownComplete) return true;  // our own final request
            if (s_teardownStarted) return false;  // teardown running — reject, don't race it
            if (s_instance == null) return true;  // nobody to run the coroutine
            s_teardownStarted = true;
            // Before anything is torn down: the pipelines are about to stop on
            // purpose, and the rig watchdog must not read that as a camera fault.
            AppShutdown.Begin();
            ShutdownProfiler.Mark("quit requested — showing the shutdown splash");
            BootOverlay.ShowShutdown("停止しています…");
            s_instance.StartCoroutine(s_instance.QuitAfterPresent());
            return false;
        }

        private IEnumerator QuitAfterPresent()
        {
            // Two frames, not one: the first WaitForEndOfFrame lands at the end of the
            // frame the splash was enabled in, and a canvas enabled mid-frame is not
            // guaranteed to have been rebuilt for that frame's render.
            yield return new WaitForEndOfFrame();
            yield return new WaitForEndOfFrame();
            ShutdownProfiler.Mark("shutdown splash presented");

            // Tear the staged subsystems down HERE rather than leaving it all to
            // OnApplicationQuit: once Application.Quit() is called the player loop
            // never runs again, so a progress count set during that teardown could
            // never be drawn. Yielding through it is the only way the operator sees
            // anything but a frozen frame for the next few seconds.
            foreach (var mb in FindObjectsByType<MonoBehaviour>(FindObjectsSortMode.None))
            {
                if (mb is not IStagedShutdown staged) continue;
                yield return StartCoroutine(DriveStaged(staged));
            }

            ShutdownProfiler.Mark("staged teardown done — quitting for real");
            s_teardownComplete = true;
            Application.Quit();
        }

        // Steps the teardown enumerator by hand instead of `yield return
        // StartCoroutine(staged.StopStaged(...))`.
        //
        // Why: Unity logs an exception thrown inside a child coroutine and stops THAT
        // coroutine, then resumes the parent as if it had finished. The quit would
        // therefore proceed — setting s_teardownComplete, past the watchdog's reach —
        // while the native pipeline stops the enumerator had already launched were
        // still running, and the exit would dispose the SDK context underneath them.
        // That is precisely the race AbandonForForcedExit exists to prevent, so a
        // teardown that dies must take that path rather than be mistaken for success.
        private IEnumerator DriveStaged(IStagedShutdown staged)
        {
            var it = staged.StopStaged((done, total) =>
                BootOverlay.SetStatus($"カメラを停止しています…  {done} / {total}"));
            while (true)
            {
                object current;
                try
                {
                    if (!it.MoveNext()) yield break;
                    current = it.Current;
                }
                catch (System.Exception e)
                {
                    Debug.LogError("[AppQuitHotkey] a staged teardown threw — treating the exit " +
                                   "as forced so no subsystem is left assuming a clean stop.");
                    Debug.LogException(e);
                    ShutdownProfiler.Mark("staged teardown threw — forcing the safe-exit path");
                    AbandonAllForForcedExit();
                    yield break;
                }
                yield return current;
            }
        }

        // Tell every staged subsystem the exit is being forced. Both the watchdog and a
        // teardown that threw come through here; implementations must not block.
        private static void AbandonAllForForcedExit()
        {
            foreach (var mb in FindObjectsByType<MonoBehaviour>(FindObjectsSortMode.None))
            {
                if (mb is not IStagedShutdown staged) continue;
                try { staged.AbandonForForcedExit(); }
                catch (System.Exception e) { Debug.LogException(e); }
            }
        }

        private void Update()
        {
            // Once the quit is armed the app KEEPS RUNNING for the few seconds of
            // staged teardown (wantsToQuit returned false), and the operator is
            // naturally still holding the key. Without this the hold would re-arm and
            // put the "hold to quit" bar back over 終了中 (and, before wantsToQuit
            // learned to reject mid-teardown requests, fire a second quit that killed
            // the process with the pipelines still open).
            if (s_teardownStarted)
            {
                _heldFor = 0f;
                if (s_teardownComplete) return;
                // Watchdog. Every quit request is now rejected until the coroutine
                // finishes, so a teardown that dies (an exception in a StopStaged
                // implementation) or wedges would leave an installation that CANNOT be
                // closed. Force the exit rather than strand the operator.
                _teardownElapsed += Time.unscaledDeltaTime;
                if (_teardownElapsed > teardownWatchdogSeconds)
                {
                    Debug.LogError($"[AppQuitHotkey] staged teardown did not finish within " +
                                   $"{teardownWatchdogSeconds:0}s — quitting anyway.");
                    ShutdownProfiler.Mark("teardown watchdog fired — forcing the quit");
                    // This bound and each subsystem's own bounds are set independently, so
                    // the watchdog can fire BEFORE an implementation reached its own safety
                    // path. Tell them the exit is being forced rather than assume they got
                    // there — for the rig that is what stops the SDK context being disposed
                    // under a native call that never returned.
                    AbandonAllForForcedExit();
                    s_teardownComplete = true;
                    Application.Quit();
                }
                return;
            }

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
            // The splash (or an alert / floor tune) owns the screen — stand down, same
            // contract every other IMGUI overlay on Display 1 follows.
            if (OperatorOverlayGate.Suppressed) return;

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
