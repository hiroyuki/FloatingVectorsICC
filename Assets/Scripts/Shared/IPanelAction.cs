// Cross-subsystem contract for "expose one-shot operation buttons on the
// one-stop Control Panel" (the sibling of IPanelTunable / IViewToggle).
// Implementers surface buttons in the panel's Actions section so runtime
// operations don't live only on component-specific windows or hotkeys.
// Discovery is interface-based, so the panel needs no reference to the
// concrete component's assembly.

namespace Shared
{
    public interface IPanelAction
    {
        /// <summary>Row heading in the Actions section (e.g. "Publish").</summary>
        string ActionsLabel { get; }

        /// <summary>Number of buttons this component exposes.</summary>
        int ActionCount { get; }

        /// <summary>Button label for action <paramref name="i"/> — may change with
        /// runtime state (e.g. "Show preview" / "Hide preview").</summary>
        string ActionName(int i);

        /// <summary>False draws button <paramref name="i"/> greyed out (e.g. edit
        /// mode, prerequisite missing).</summary>
        bool ActionEnabled(int i);

        /// <summary>Run action <paramref name="i"/>. Must be safe to call from
        /// editor GUI code.</summary>
        void RunAction(int i);

        /// <summary>One-line status shown under the buttons (empty = none) —
        /// progress / result stats of the last action.</summary>
        string ActionsStatus { get; }
    }
}
