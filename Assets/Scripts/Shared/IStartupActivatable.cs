// Cross-subsystem contract for an "arm this on Play" switch — a serialized
// bool on a component that makes it self-activate when Play starts (e.g.
// ExperienceDirector.activateOnPlay boots the exhibition into experience mode
// with no operator action). Implementing this surfaces the switch in the
// Control Panel (Shared.EditorTools.FloatingVectorsControlPanel) as a big
// toggle, so the operator never has to find the owning GameObject before
// pressing Play.

namespace Shared
{
    public interface IStartupActivatable
    {
        /// <summary>Display name in the Control Panel (e.g. "Experience mode").</summary>
        string ActivateLabel { get; }

        /// <summary>The serialized activate-on-Play flag. The setter must mark the
        /// change persistent-safe (the panel handles Undo/SetDirty); reading and
        /// writing outside Play mode is the primary use.</summary>
        bool ActivateOnPlay { get; set; }
    }
}
