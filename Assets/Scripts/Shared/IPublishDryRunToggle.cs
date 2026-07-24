// Cross-subsystem contract for the publish dry-run switch (e.g.
// ExperienceConfig.dryRunPublish: ON = fake URLs / no network, OFF = real LFKS
// upload at ResultShow). Implementing this next to IStartupActivatable surfaces
// the switch in the Control Panel (Shared.EditorTools.FloatingVectorsControlPanel)
// as a big toggle beside the Experience-mode button, so the operator arms/disarms
// the real upload from the same place they arm the show.

namespace Shared
{
    public interface IPublishDryRunToggle
    {
        /// <summary>The serialized dry-run flag (true = fake URLs). The panel
        /// handles Undo/SetDirty against <see cref="DryRunUndoTarget"/>.</summary>
        bool DryRunPublish { get; set; }

        /// <summary>The Object that owns the serialized flag (the config asset,
        /// not the component). Null = no config assigned; the panel draws the
        /// toggle disabled.</summary>
        UnityEngine.Object DryRunUndoTarget { get; }
    }
}
