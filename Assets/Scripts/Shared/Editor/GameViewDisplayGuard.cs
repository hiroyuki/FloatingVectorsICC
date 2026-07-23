// Editor-only: stop two Game views from claiming the same Display.
//
// Why this exists: in the Editor Display.displays has exactly ONE entry no matter
// how many Game views are open, so Unity has to derive "how many pixels does
// Display N have" from the Game view showing it. With two Game views set to the
// same display it takes the size of the one that rendered last — in practice the
// small focused one. Every ScreenSpaceOverlay canvas targeting that display then
// gets that size as its Canvas.renderingDisplaySize, and an overlay canvas is
// drawn one canvas pixel to one device pixel from the bottom-left corner: the UI
// lands in the bottom-left corner of the big view at a fraction of its size,
// while the 3D content (which goes through the camera's own viewport, not the
// canvas) still fills the screen. Observed 2026-07-23: a 907x515 floating Game
// view and the 1920x1010 fullscreen one both on Display 2 shrank the whole
// visitor UI to 47%.
//
// This is not fixable from the canvas side. Canvas.renderingDisplaySize is
// read-only, and CanvasScaler only scales the layout INSIDE the rect Unity has
// already decided on, so no amount of "just assume 1920x1080" changes the pixels
// the canvas is allowed to occupy. The display assignment itself has to be right.
//
// A standalone build is unaffected — there Display.displays really does have one
// entry per monitor and each canvas gets its own display's size.

using System.Collections.Generic;
using UnityEditor;
using UnityEngine;

namespace Shared.EditorTools
{
    [InitializeOnLoad]
    internal static class GameViewDisplayGuard
    {
        private const string kAutoFixPref = "FloatingVectors.GameViewDisplayGuard.AutoFix";
        private const string kAutoFixMenu = "FloatingVectors/Displays/Fix duplicate Game view displays on Play";
        private const string kRunNowMenu = "FloatingVectors/Displays/Fix duplicate Game view displays now";

        // Unity supports 8 displays; a view moved out of a collision has to land
        // on one of those indices to stay a valid assignment.
        private const int kMaxDisplays = 8;

        static GameViewDisplayGuard()
        {
            EditorApplication.playModeStateChanged += OnPlayModeStateChanged;
        }

        private static bool AutoFix
        {
            get => EditorPrefs.GetBool(kAutoFixPref, true);
            set => EditorPrefs.SetBool(kAutoFixPref, value);
        }

        [MenuItem(kAutoFixMenu, priority = 100)]
        private static void ToggleAutoFix() => AutoFix = !AutoFix;

        [MenuItem(kAutoFixMenu, validate = true)]
        private static bool ToggleAutoFixValidate()
        {
            Menu.SetChecked(kAutoFixMenu, AutoFix);
            return true;
        }

        [MenuItem(kRunNowMenu, priority = 101)]
        private static void RunNow()
        {
            if (!Fix(out string report))
                Debug.Log("[GameViewDisplayGuard] no two Game views share a display.\n" + report);
        }

        private static void OnPlayModeStateChanged(PlayModeStateChange state)
        {
            // On entering play mode: the canvases are built in the first frames
            // after this, so the assignment has to be correct by now.
            if (state != PlayModeStateChange.EnteredPlayMode) return;
            if (!AutoFix) return;
            Fix(out _);
        }

        /// <summary>Move every Game view but one off any display that has more than
        /// one. Returns true if anything was changed; `report` always lists the
        /// resulting assignment.</summary>
        private static bool Fix(out string report)
        {
            var views = OpenGameViews();
            report = Describe(views);
            if (views.Count < 2) return false;

            // Group by display. The view that keeps the display is the largest one
            // — that is the fullscreen view sitting on the physical monitor, which
            // is the size the canvases should be laid out for. The small helper
            // window is the one that has to move.
            var byDisplay = new Dictionary<int, List<EditorWindow>>();
            foreach (var w in views)
            {
                int d = GetTargetDisplay(w);
                if (d < 0) continue;
                if (!byDisplay.TryGetValue(d, out var list))
                    byDisplay[d] = list = new List<EditorWindow>();
                list.Add(w);
            }

            bool changed = false;
            foreach (var kv in byDisplay)
            {
                var list = kv.Value;
                if (list.Count < 2) continue;

                EditorWindow keep = list[0];
                foreach (var w in list)
                    if (Area(w) > Area(keep)) keep = w;

                foreach (var w in list)
                {
                    if (w == keep) continue;
                    int free = FirstFreeDisplay(byDisplay);
                    if (free < 0)
                    {
                        Debug.LogWarning($"[GameViewDisplayGuard] Game view '{w.titleContent.text}' " +
                                         $"({Size(w)}) shares Display {kv.Key + 1} with " +
                                         $"'{keep.titleContent.text}' ({Size(keep)}) and every display " +
                                         "index is already taken — the UI on the bigger view will render " +
                                         "at the smaller view's size, in its bottom-left corner. Close " +
                                         "one of the Game views.");
                        continue;
                    }

                    SetTargetDisplay(w, free);
                    if (!byDisplay.TryGetValue(free, out var moved))
                        byDisplay[free] = moved = new List<EditorWindow>();
                    moved.Add(w);
                    w.Repaint();
                    changed = true;
                    Debug.Log($"[GameViewDisplayGuard] Game view '{w.titleContent.text}' ({Size(w)}) " +
                              $"shared Display {kv.Key + 1} with '{keep.titleContent.text}' " +
                              $"({Size(keep)}) — moved it to Display {free + 1}. Two views on one " +
                              "display make every Canvas on that display lay out for the smaller " +
                              "view and render in the bottom-left corner of the bigger one.");
                }
            }

            if (changed) report = Describe(OpenGameViews());
            return changed;
        }

        private static int FirstFreeDisplay(Dictionary<int, List<EditorWindow>> byDisplay)
        {
            for (int d = 0; d < kMaxDisplays; d++)
            {
                if (!byDisplay.TryGetValue(d, out var list) || list.Count == 0) return d;
            }
            return -1;
        }

        private static List<EditorWindow> OpenGameViews()
        {
            var result = new List<EditorWindow>();
            var gvType = System.Type.GetType("UnityEditor.GameView,UnityEditor");
            if (gvType == null) return result;
            foreach (var o in Resources.FindObjectsOfTypeAll(gvType))
                if (o is EditorWindow w && w != null) result.Add(w);
            return result;
        }

        // GameView.targetDisplay is declared on the internal PlayModeView base, so
        // it is reached by reflection rather than by a direct reference. Both the
        // getter and the setter are there in Unity 6; a version that renames it
        // degrades to "guard does nothing" instead of throwing.
        private static System.Reflection.PropertyInfo TargetDisplayProperty(object view)
        {
            return view.GetType().GetProperty("targetDisplay",
                System.Reflection.BindingFlags.Instance |
                System.Reflection.BindingFlags.Public |
                System.Reflection.BindingFlags.NonPublic);
        }

        private static int GetTargetDisplay(EditorWindow view)
        {
            var p = TargetDisplayProperty(view);
            if (p == null || !p.CanRead) return -1;
            return (int)p.GetValue(view);
        }

        private static void SetTargetDisplay(EditorWindow view, int display)
        {
            var p = TargetDisplayProperty(view);
            if (p == null || !p.CanWrite)
            {
                Debug.LogWarning("[GameViewDisplayGuard] this Unity version does not expose " +
                                 "GameView.targetDisplay for writing — set the Game view's display " +
                                 "dropdown by hand.");
                return;
            }
            p.SetValue(view, display);
        }

        private static float Area(EditorWindow w) => w.position.width * w.position.height;

        private static string Size(EditorWindow w) =>
            $"{Mathf.RoundToInt(w.position.width)}x{Mathf.RoundToInt(w.position.height)}";

        private static string Describe(List<EditorWindow> views)
        {
            var sb = new System.Text.StringBuilder("Game views:");
            foreach (var w in views)
                sb.Append($"\n  Display {GetTargetDisplay(w) + 1}  {Size(w)}  '{w.titleContent.text}'");
            return sb.ToString();
        }
    }
}
