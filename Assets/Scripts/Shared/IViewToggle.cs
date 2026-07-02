// Cross-subsystem contract for "show/hide this subsystem's visual output"
// (the follow-up to the accumulation unification). PointCloudView
// (showPointClouds), TSDFView (showMesh) and SkeletonMerger (showBones) each
// carried their own bool with no central place to see or flip what is
// currently drawn. Implementing this lets the Views editor window
// (Shared.EditorTools.ViewTogglesWindow) list every visual lever in one panel
// — e.g. hide the point cloud + skeleton to show a sculpture alone.
//
// Scope note: only the three main per-subsystem levers implement this.
// Debug-adjacent visibility (per-worker skeletons, debug HUD, session
// overlays) intentionally stays on its own components.

namespace Shared
{
    public interface IViewToggle
    {
        /// <summary>Display name in the Views panel (e.g. "Point cloud").</summary>
        string ViewLabel { get; }

        /// <summary>Show/hide the subsystem's visual. Setting this must be safe at
        /// runtime and take effect on the next frame (the underlying pipeline keeps
        /// running either way).</summary>
        bool Visible { get; set; }
    }
}
