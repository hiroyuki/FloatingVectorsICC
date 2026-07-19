// Pure-C# state machine for the visitor experience — the pose-driven
// sequence: Calibrate (star pose + body measurement) → Explore (10s take
// recording) → Processing (v11s conversion) → Watch (first playback loop) →
// BanzaiWait (looping playback until the banzai pose / loop fallback) →
// Exporting → QrShow. No scene access — the ExperienceDirector feeds an
// ExperienceInputs snapshot every Tick and reacts to Changed. Pure so every
// transition rule is EditMode-testable (ExperienceStateMachineTests).
//
// Latching contracts (carried over from the previous machine's Codex review):
//   - Per-state completion flags (CalibrationDone, RecordingDone,
//     ProcessingDone, CaptureDone) are latched INSIDE their owning state only
//     and cleared on that state's enter — a stale director flag can never
//     fast-forward a later visitor's run.
//   - ProcessingFailed / ExportFailed are latched: a one-frame pulse starts
//     the fail-notice timer; the machine holds the state until
//     exportFailNoticeSeconds then falls back to Attract (the director shows
//     the apology meanwhile).
//
// Skip flags mean "enter the state, then advance immediately on the first
// Tick" — the state is still entered so side effects and the E2E transition
// path stay exercised (dev-mode stage skipping).
//
// Duration ownership: Explore's 10s (and its 3s end countdown) and
// Processing's timeout/fallback are DIRECTOR-owned (they end in RecordingDone
// / ProcessingDone) because they wrap real device work; Calibrate's 10s
// window and QrShow's dwell are machine-owned timers.

using System;
using UnityEngine;

namespace Experience
{
    public enum ExperienceState
    {
        Attract, Calibrate, Explore, Processing, Watch, BanzaiWait,
        Exporting, QrShow, Fault
    }

    /// <summary>Per-tick inputs from the director (already debounced where
    /// applicable — Present comes from PresenceDetector's hysteresis, or the
    /// occupancy+live-feed blend during playback states).</summary>
    public struct ExperienceInputs
    {
        public bool Present;
        public bool Fault;
        public bool CalibrationDone;   // star pose held + metrics stored
        public bool RecordingDone;     // director stopped the Explore recording
        public bool ProcessingDone;    // conversion finished OR fallback decided
        public bool ProcessingFailed;  // unrecoverable (no take) — may be a pulse
        public int PlaybackLoops;      // completed loops since Watch entry
        public bool CaptureDone;       // banzai / fallback capture finished
        public bool ExportDone;
        public bool ExportFailed;      // may be a one-frame pulse
    }

    /// <summary>Timing + dev-skip knobs (plain POCO, filled from
    /// ExperienceConfig; separate type so tests need no ScriptableObject).</summary>
    [Serializable]
    public class ExperienceTimings
    {
        [Min(0f)]
        [Tooltip("Star-pose window. Expiry advances with DEFAULT body metrics.")]
        public float calibrateSeconds = 10f;
        [Min(1f)]
        [Tooltip("Explore recording length (director-owned; last countdownSeconds " +
                 "show digits).")]
        public float exploreSeconds = 10f;
        [Min(5f)]
        [Tooltip("Director-enforced conversion ceiling; expiry falls back to the " +
                 "recorded k4abt bodies and pulses ProcessingDone.")]
        public float processingTimeoutSeconds = 90f;
        [Range(1, 10)]
        [Tooltip("Banzai window loops after the Watch loop; expiry captures at a " +
                 "random playback point.")]
        public int banzaiFallbackLoops = 3;
        [Min(0f)] public float exportFailNoticeSeconds = 5f;
        [Min(0f)] public float qrShowSeconds = 30f;

        [Header("Dev stage skipping (enter, then advance immediately)")]
        public bool skipAttract;
        public bool skipCalibrate;   // default body metrics
        public bool skipExplore;     // director substitutes devCannedTakeRoot
        public bool skipProcessing;  // keep recorded k4abt bodies, play instantly
        public bool skipQr;          // Exporting success goes straight to Attract

        [Min(0.01f)]
        [Tooltip("Dev time multiplier applied to Tick dt (2 = twice as fast).")]
        public float timeMultiplier = 1f;
    }

    public sealed class ExperienceStateMachine
    {
        public ExperienceState State { get; private set; } = ExperienceState.Attract;
        public float TimeInState { get; private set; }

        public event Action<ExperienceState, ExperienceState> Changed;

        private bool _calibLatched;
        private bool _recordingLatched;
        private bool _processingLatched;
        private bool _processingFailLatched;
        private float _processingFailElapsed;
        private bool _captureLatched;
        private bool _exportFailLatched;
        private float _exportFailElapsed;

        public void Tick(float dt, in ExperienceInputs inputs, ExperienceTimings t)
        {
            dt *= Mathf.Max(0.01f, t.timeMultiplier);
            TimeInState += dt;

            // Fault preempts everything (full-screen red alert + show stops).
            if (inputs.Fault && State != ExperienceState.Fault)
            { Go(ExperienceState.Fault); return; }

            switch (State)
            {
                case ExperienceState.Fault:
                    if (!inputs.Fault) Go(ExperienceState.Attract);
                    break;

                case ExperienceState.Attract:
                    if (t.skipAttract || inputs.Present) Go(ExperienceState.Calibrate);
                    break;

                case ExperienceState.Calibrate:
                    if (LeftEarly(inputs)) break;
                    if (inputs.CalibrationDone) _calibLatched = true;
                    // Window expiry advances too — the director keeps default
                    // metrics; the show must never stall on a shy visitor.
                    if (t.skipCalibrate || _calibLatched || TimeInState >= t.calibrateSeconds)
                        Go(ExperienceState.Explore);
                    break;

                case ExperienceState.Explore:
                    if (LeftEarly(inputs)) break;
                    if (inputs.RecordingDone) _recordingLatched = true;
                    if (t.skipExplore || _recordingLatched) Go(ExperienceState.Processing);
                    break;

                case ExperienceState.Processing:
                    if (inputs.ProcessingFailed) _processingFailLatched = true;
                    if (_processingFailLatched)
                    {
                        _processingFailElapsed += dt;
                        if (_processingFailElapsed >= t.exportFailNoticeSeconds)
                            Go(ExperienceState.Attract);
                        break;
                    }
                    if (LeftEarly(inputs)) break;
                    if (inputs.ProcessingDone) _processingLatched = true;
                    if (t.skipProcessing || _processingLatched) Go(ExperienceState.Watch);
                    break;

                case ExperienceState.Watch:
                    if (LeftEarly(inputs)) break;
                    if (inputs.PlaybackLoops >= 1) Go(ExperienceState.BanzaiWait);
                    break;

                case ExperienceState.BanzaiWait:
                    if (LeftEarly(inputs)) break;
                    // The banzai trigger AND the loop fallback both end in the
                    // director's capture routine → CaptureDone.
                    if (inputs.CaptureDone) _captureLatched = true;
                    if (_captureLatched) Go(ExperienceState.Exporting);
                    break;

                case ExperienceState.Exporting:
                    // No early-leave: the export always runs to a conclusion.
                    if (inputs.ExportFailed) _exportFailLatched = true;
                    if (_exportFailLatched)
                    {
                        _exportFailElapsed += dt;
                        if (_exportFailElapsed >= t.exportFailNoticeSeconds)
                            Go(ExperienceState.Attract);
                        break;
                    }
                    if (inputs.ExportDone)
                        Go(t.skipQr ? ExperienceState.Attract : ExperienceState.QrShow);
                    break;

                case ExperienceState.QrShow:
                    if (TimeInState >= t.qrShowSeconds || !inputs.Present)
                        Go(ExperienceState.Attract);
                    break;
            }

            bool LeftEarly(in ExperienceInputs i)
            {
                if (i.Present) return false;
                Go(ExperienceState.Attract);
                return true;
            }
        }

        /// <summary>Director-side override (mode exit, debug jumps).</summary>
        public void ForceTransition(ExperienceState to, string reason)
        {
            if (to == State) return;
            Go(to);
        }

        /// <summary>Restart from a state without firing Changed (mode enter).</summary>
        public void ResetTo(ExperienceState state)
        {
            State = state;
            TimeInState = 0f;
            _calibLatched = false;
            _recordingLatched = false;
            _processingLatched = false;
            _processingFailLatched = false;
            _processingFailElapsed = 0f;
            _captureLatched = false;
            _exportFailLatched = false;
            _exportFailElapsed = 0f;
        }

        private void Go(ExperienceState to)
        {
            var from = State;
            State = to;
            TimeInState = 0f;
            // Clear each state's OWN latches on entry — stale director flags
            // must never fast-forward the next visitor's run.
            switch (to)
            {
                case ExperienceState.Calibrate:
                    _calibLatched = false;
                    break;
                case ExperienceState.Explore:
                    _recordingLatched = false;
                    break;
                case ExperienceState.Processing:
                    _processingLatched = false;
                    _processingFailLatched = false;
                    _processingFailElapsed = 0f;
                    break;
                case ExperienceState.BanzaiWait:
                    _captureLatched = false;
                    break;
                case ExperienceState.Exporting:
                    _exportFailLatched = false;
                    _exportFailElapsed = 0f;
                    break;
            }
            Changed?.Invoke(from, to);
        }
    }
}
