// Cross-assembly coordination flags for the operator display (Display 1):
// while something owns that display full-screen, every other IMGUI overlay on
// it must stand down — IMGUI draws over uGUI, so without this gate the HUD /
// camera tiles would cover the owner.
//
// Display numbering in this project is 1-origin everywhere in prose and naming
// ("Display 1" is the operator screen), matching the Unity Inspector and the OS.
// Unity's Camera.targetDisplay / Canvas.targetDisplay API is 0-based, so
// Display 1 == targetDisplay 0. Call sites that touch the API spell out both.
//
// Two owners exist:
//   - AlertActive     — a full-screen fault alert (Experience.VisitorMessageUI)
//   - FloorTuneActive — the floor-levelling tune, which borrows Display 1 to
//                       show the live point cloud and needs it unobstructed so
//                       the operator can click floor points
//
// Lives in Shared because Experience references PointCloud (the reverse would
// be a cycle) and both Display1OperatorHud (Assembly-CSharp) and
// MultiCameraDebugView (PointCloud) need to read it.

namespace Shared
{
    public static class OperatorOverlayGate
    {
        /// <summary>True while a full-screen operator alert is being drawn.
        /// Owned by Experience.VisitorMessageUI.</summary>
        public static bool AlertActive;

        /// <summary>True while floor tune owns Display 1 to show the live point
        /// cloud for the 3-point floor pick. Owned by
        /// Calibration.RuntimeUI.CalibrationRuntimeUI.</summary>
        public static bool FloorTuneActive;

        /// <summary>True while the boot / shutdown splash covers every display.
        /// Owned by Shared.BootOverlay. The splash is a uGUI canvas at the top
        /// sorting order and IMGUI draws under those, so this is belt-and-braces —
        /// but "起動中" with an operator HUD punched through it would read as a
        /// half-started app, and the Editor's Game view does not order IMGUI the
        /// same way a player does.</summary>
        public static bool BootActive;

        /// <summary>Whatever the reason, Display 1 is spoken for: IMGUI overlays
        /// on the primary display check this at the top of OnGUI and skip
        /// drawing.</summary>
        public static bool Suppressed => AlertActive || FloorTuneActive || BootActive;
    }
}
