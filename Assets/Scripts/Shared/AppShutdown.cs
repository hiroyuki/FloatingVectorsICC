// "The application is on its way out" flag.
//
// Teardown stops the camera pipelines one by one, which is exactly what the
// health monitor is built to notice: frames stop advancing, renderers vanish,
// and the operator gets 「カメラ（ID n）が異常です」 over the shutdown splash for
// a rig that is not faulty at all. Anything that watches the rig for faults
// should stand down once this is set.
//
// Set from every exit route we can see: the staged quit (AppQuitHotkey arms it
// before the first pipeline stop), Application.quitting for quits that bypass
// the hotkey, and ExitingPlayMode in the Editor.
//
// Never cleared during a run — a process that started quitting does not come
// back. It resets on the next domain load (BeforeSceneLoad) so entering play
// mode again with domain reload off starts clean.

using UnityEngine;

namespace Shared
{
    public static class AppShutdown
    {
        /// <summary>True from the moment a quit / play-mode exit begins.</summary>
        public static bool IsShuttingDown { get; private set; }

        /// <summary>Latch the flag. Idempotent.</summary>
        public static void Begin() => IsShuttingDown = true;

        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.BeforeSceneLoad)]
        private static void Install()
        {
            IsShuttingDown = false;
            Application.quitting += Begin;
#if UNITY_EDITOR
            UnityEditor.EditorApplication.playModeStateChanged += state =>
            {
                if (state == UnityEditor.PlayModeStateChange.ExitingPlayMode) Begin();
            };
#endif
        }
    }
}
