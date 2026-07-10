// Cross-assembly coordination flag for the operator display (display 0):
// while a full-screen fault alert is up (Experience.VisitorMessageUI), every
// other IMGUI overlay on that display must stand down — IMGUI draws over uGUI,
// so without this gate the HUD / camera tiles would cover the alert.
//
// Lives in Shared because Experience references PointCloud (the reverse would
// be a cycle) and both Display1OperatorHud (Assembly-CSharp) and
// MultiCameraDebugView (PointCloud) need to read it.

namespace Shared
{
    public static class OperatorOverlayGate
    {
        /// <summary>True while a full-screen operator alert is being drawn.
        /// IMGUI overlays on the primary display check this at the top of
        /// OnGUI and skip drawing. Owned by Experience.VisitorMessageUI.</summary>
        public static bool AlertActive;
    }
}
