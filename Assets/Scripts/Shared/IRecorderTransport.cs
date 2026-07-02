// Control Panel contract for the sensor recorder's transport: record / play /
// pause from one place without selecting the GameObject. Implemented by
// PointCloud.SensorRecorder; the panel (Shared.Editor) discovers it via this
// interface so it needs no reference to the PointCloud assembly.

namespace Shared
{
    public interface IRecorderTransport
    {
        bool IsRecording { get; }
        bool IsPlaying { get; }
        bool IsPaused { get; }

        /// <summary>One-line state readout (state, playhead, last status message).</summary>
        string TransportStatus { get; }

        void ToggleRecord();
        void TogglePlay();
        void TogglePause();
    }
}
