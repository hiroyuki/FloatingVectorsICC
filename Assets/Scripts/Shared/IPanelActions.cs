// Cross-subsystem contract for one-shot operation buttons on the Control
// Panel (Window > Control Panel).
//
// IAccumulationController models "accumulate over time, then stop" — a state
// machine. Components whose operations are single button presses with no
// running state (e.g. TSDFPrintExporter's Fuse curves / Close holes /
// Export STL / Restore) implement THIS instead, and the panel lists each
// action as a plain button under an "Actions" section.
//
// Actions may be unavailable in some states (nothing to restore yet, not in
// Play mode, ...): gate via ActionEnabled so the panel can grey the button
// out instead of hiding it.

namespace Shared
{
    public interface IPanelActions
    {
        /// <summary>Row heading shown above this component's buttons
        /// (e.g. "3D Print Export").</summary>
        string ActionsLabel { get; }

        int ActionCount { get; }

        /// <summary>Button label — must state what it really does
        /// (e.g. "Fuse curves", "Export STL").</summary>
        string ActionLabel(int index);

        /// <summary>Whether the button is currently pressable. The panel greys
        /// out disabled actions but keeps them visible.</summary>
        bool ActionEnabled(int index);

        void RunAction(int index);

        /// <summary>One-line state readout under the buttons (seg counts, last
        /// export path, ...). Empty/null = no status box.</summary>
        string ActionsStatusText { get; }
    }
}
