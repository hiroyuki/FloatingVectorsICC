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

        /// <summary>
        /// Recording root folder (SensorRecorder.folderPath). Relative paths resolve
        /// under persistentDataPath; empty = the default recording folder. Editable
        /// from the Control Panel so the operator can point Rec/Play at a dataset
        /// without selecting the GameObject.
        /// </summary>
        string RecordingFolder { get; set; }

        /// <summary>Absolute path RecordingFolder resolves to (read-only, for display).</summary>
        string ResolvedRecordingFolder { get; }

        /// <summary>True while live camera renderers exist; false in playback mode.</summary>
        bool IsLiveMode { get; }

        /// <summary>
        /// Startup mode selector, backed by SensorManager.playbackOnly. Set before
        /// entering Play mode: true → the scene auto-starts in PLAYBACK (plays the
        /// recording folder), false → LIVE capture. The Control Panel exposes this
        /// as a checkbox so the operator confirms the mode before pressing Play,
        /// without needing to switch mid-session.
        /// </summary>
        bool StartInPlaybackMode { get; set; }

        void ToggleRecord();
        void TogglePlay();
        void TogglePause();

        /// <summary>One-button live ⇄ playback switch (see SensorRecorder.SwitchMode).</summary>
        void SwitchMode();
    }
}
