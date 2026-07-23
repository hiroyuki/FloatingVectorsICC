// Cross-assembly hook for the stage orbit cameras. The visitor-display
// cameras carry CameraControl.PauseOrbitGate (Assembly-CSharp), which
// force-enables their CameraOrbitController while this override is up — but
// the ExperienceDirector lives in the Experience asmdef, which cannot
// reference Assembly-CSharp. The director drives the gates through this
// interface instead: playback/QR phases orbit the model, and the gate itself
// restores the saved camera pose when the orbit turns off (the controller
// leaves the camera wherever the orbit ended).

using UnityEngine;

namespace Shared
{
    public interface IOrbitOverride
    {
        /// <summary>Force the orbit controller on regardless of the transport
        /// pause state (mirrors PauseOrbitGate.autoOrbitOverride).</summary>
        bool OrbitOverride { get; set; }

        /// <summary>When set, the gate snapshots the camera pose as the orbit
        /// controller turns on and restores it as it turns off — so the stage
        /// framing comes back for the next visitor. The ExperienceDirector
        /// forces this on for the session and restores the original value on
        /// mode exit.</summary>
        bool RestorePoseOnDisable { get; set; }

        /// <summary>The orbit controller's idle auto-rotation flag
        /// (CameraOrbitController.autoOrbit). The show forces it on for the
        /// session — the visitor has no mouse, so an orbit phase must rotate
        /// by itself — and restores the dev value on exit.</summary>
        bool AutoOrbit { get; set; }

        /// <summary>Disable the orbit controller NOW and restore the saved
        /// camera pose (when RestorePoseOnDisable is set). The gate normally
        /// processes the off-edge in its own next Update, but mode exit
        /// restores the gate's dev flags in the same frame — by the time that
        /// Update ran, the restore flag would already be back to the dev value
        /// and the camera would be stranded at the orbit pose. Call this
        /// BEFORE restoring the saved flags.</summary>
        void ReleaseOrbit();

        /// <summary>Presentation camera work: orbit around <paramref name="pivot"/>
        /// (the show feeds a chest-height anchor that tracks the person) at
        /// <paramref name="yawSpeedDeg"/> with a vertical bob of
        /// <paramref name="bobAmpMeters"/>. Pass a null pivot to restore the
        /// controller's own pivot/speed/bob (dev values). Idempotent.</summary>
        void SetPresentationOrbit(Transform pivot, float yawSpeedDeg, float bobAmpMeters);
    }
}
