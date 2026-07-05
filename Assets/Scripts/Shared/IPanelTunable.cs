// Cross-subsystem contract for "expose a few tunable numeric parameters on the
// one-stop Control Panel" (the sibling of IViewToggle / IAccumulationController).
// Implementers surface sliders in the panel's Tuning section so the key knobs
// don't live only in the Inspector. Discovery is interface-based, so the panel
// needs no reference to the concrete component's assembly.

namespace Shared
{
    public interface IPanelTunable
    {
        /// <summary>Row heading in the Tuning section (e.g. "Motion history").</summary>
        string TuningLabel { get; }

        /// <summary>Number of sliders this component exposes.</summary>
        int TunableCount { get; }

        /// <summary>Display name of slider <paramref name="i"/>.</summary>
        string TunableName(int i);

        /// <summary>Current value of slider <paramref name="i"/>.</summary>
        float TunableValue(int i);

        /// <summary>Apply a new value to slider <paramref name="i"/> (implementer clamps/rounds).
        /// Must be safe at runtime and take effect on the next frame.</summary>
        void SetTunableValue(int i, float value);

        float TunableMin(int i);
        float TunableMax(int i);

        /// <summary>True if slider <paramref name="i"/> is an integer (rendered as an int slider).</summary>
        bool TunableIsInt(int i);
    }
}
