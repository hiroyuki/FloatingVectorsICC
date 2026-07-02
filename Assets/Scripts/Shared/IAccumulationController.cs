// Cross-subsystem contract for "accumulate over time, then stop/clear" scene
// controls (Plans/plan-architecture-cleanup.md phase 3, Codex-approved).
//
// PointCloudCumulative (snapshot stacking), MeshCumulative (TSDF time fusion)
// and TSDFTrailBaker (Start/Stop motion-sculpture capture) are DIFFERENT
// mechanisms on purpose — this interface only unifies their Inspector
// operation surface so all three present the same Start / Stop / Clear row
// (drawn by Shared.EditorTools.AccumulationControllerGUI).
//
// The capability flags + labels exist so the shared UI can preserve each
// component's existing gating and NOT hide semantic differences:
//   - CanStart/StartLabel: MeshCumulative allows "Restart" while accumulating;
//     TSDFTrailBaker disables Start while capturing.
//   - CanClear/ClearLabel: Clear means "delete snapshots" vs "clear & resume
//     live" depending on the component; TSDFTrailBaker opts out entirely
//     (CanClear=false) and keeps its dedicated "Resume live" button instead.

namespace Shared
{
    public interface IAccumulationController
    {
        /// <summary>True while the component is actively accumulating.</summary>
        bool IsAccumulating { get; }

        /// <summary>One-line state readout for the Inspector (e.g. "34 snapshots",
        /// "Accumulating 3.2s / 10s", the last capture status).</summary>
        string StatusText { get; }

        bool CanStart { get; }
        /// <summary>Start-button label (e.g. "Begin", "Restart", "Start capture").</summary>
        string StartLabel { get; }
        void StartAccumulate();

        bool CanStop { get; }
        void StopAccumulate();

        /// <summary>Whether the shared row shows a Clear action at all. Components
        /// whose "clear" has heavier side effects keep their own dedicated button
        /// and return false here.</summary>
        bool CanClear { get; }
        /// <summary>Clear-button label — must state what it really does
        /// (e.g. "Clear snapshots", "Clear &amp; resume live").</summary>
        string ClearLabel { get; }
        void ClearAccumulated();
    }
}
