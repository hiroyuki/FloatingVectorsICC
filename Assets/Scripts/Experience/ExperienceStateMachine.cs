// Pure-C# state machine for the visitor experience (Phase 5,
// Plans/phase5-director-plan.md). No scene access — the ExperienceDirector
// feeds an ExperienceInputs snapshot every Tick and reacts to Changed. Pure so
// every transition rule is EditMode-testable (ExperienceStateMachineTests).
//
// Latching contracts (Codex review):
//   - CaptureDone is latched internally and CLEARED on every Prompt-state
//     enter: a stale director flag can never fast-forward a later prompt.
//   - ExportFailed is latched: a one-frame pulse starts the fail-notice timer;
//     the machine stays in Exporting until exportFailNoticeSeconds then falls
//     back to Attract (the director shows the apology meanwhile).
//
// Skip flags mean "enter the state, then advance immediately on the first
// Tick" — the state is still entered so side effects and the E2E transition
// path stay exercised (user request: dev-mode stage skipping).

using System;
using UnityEngine;

namespace Experience
{
    public enum ExperienceState
    {
        Attract, Welcome, FreePlay, Ready,
        PromptAnimal, PromptMantis, PromptFree,
        Select, Exporting, QrShow, Fault
    }

    /// <summary>Per-tick inputs from the director (already debounced where
    /// applicable — Present comes from PresenceDetector's hysteresis).</summary>
    public struct ExperienceInputs
    {
        public bool Present;
        public bool Fault;
        public bool CaptureDone;   // current prompt's capture finished
        public int SelectedIndex;  // -1 = none
        public bool ExportDone;
        public bool ExportFailed;  // may be a one-frame pulse
    }

    /// <summary>Timing + dev-skip knobs (plain POCO, filled from
    /// ExperienceConfig; separate type so tests need no ScriptableObject).</summary>
    [Serializable]
    public class ExperienceTimings
    {
        [Min(0f)] public float welcomeSeconds = 6f;
        [Min(0f)] public float freePlaySeconds = 20f;
        [Min(0f)] public float readySeconds = 5f;
        [Min(1f)] public float promptSeconds = 12f;
        [Min(0f)] public float selectTimeoutSeconds = 90f; // 0 = wait forever
        [Min(0f)] public float exportFailNoticeSeconds = 5f;
        [Min(0f)] public float qrShowSeconds = 30f;

        [Header("Dev stage skipping (enter, then advance immediately)")]
        public bool skipAttract;
        public bool skipWelcome;
        public bool skipFreePlay;
        public bool skipReady;
        public bool skipQr;      // Exporting success goes straight to Attract
        [Range(1, 3)] public int captureCount = 3;
        [Min(0.01f)]
        [Tooltip("Dev time multiplier applied to Tick dt (2 = twice as fast).")]
        public float timeMultiplier = 1f;
    }

    public sealed class ExperienceStateMachine
    {
        public ExperienceState State { get; private set; } = ExperienceState.Attract;
        public float TimeInState { get; private set; }

        public event Action<ExperienceState, ExperienceState> Changed;

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
                    if (t.skipAttract || inputs.Present) Go(ExperienceState.Welcome);
                    break;

                case ExperienceState.Welcome:
                    if (LeftEarly(inputs)) break;
                    if (t.skipWelcome || TimeInState >= t.welcomeSeconds) Go(ExperienceState.FreePlay);
                    break;

                case ExperienceState.FreePlay:
                    if (LeftEarly(inputs)) break;
                    if (t.skipFreePlay || TimeInState >= t.freePlaySeconds) Go(ExperienceState.Ready);
                    break;

                case ExperienceState.Ready:
                    if (LeftEarly(inputs)) break;
                    if (t.skipReady || TimeInState >= t.readySeconds) Go(ExperienceState.PromptAnimal);
                    break;

                case ExperienceState.PromptAnimal:
                    TickPrompt(inputs, t, t.captureCount >= 2 ? ExperienceState.PromptMantis
                                                              : ExperienceState.Select);
                    break;

                case ExperienceState.PromptMantis:
                    TickPrompt(inputs, t, t.captureCount >= 3 ? ExperienceState.PromptFree
                                                              : ExperienceState.Select);
                    break;

                case ExperienceState.PromptFree:
                    TickPrompt(inputs, t, ExperienceState.Select);
                    break;

                case ExperienceState.Select:
                    if (LeftEarly(inputs)) break;
                    if (inputs.SelectedIndex >= 0) { Go(ExperienceState.Exporting); break; }
                    if (t.selectTimeoutSeconds > 0f && TimeInState >= t.selectTimeoutSeconds)
                        Go(ExperienceState.Attract);
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

        private void TickPrompt(in ExperienceInputs inputs, ExperienceTimings t, ExperienceState next)
        {
            if (!inputs.Present) { Go(ExperienceState.Attract); return; }
            if (inputs.CaptureDone) _captureLatched = true;
            if (TimeInState >= t.promptSeconds && _captureLatched) Go(next);
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
            _captureLatched = false;
            _exportFailLatched = false;
            _exportFailElapsed = 0f;
        }

        private void Go(ExperienceState to)
        {
            var from = State;
            State = to;
            TimeInState = 0f;
            switch (to)
            {
                case ExperienceState.PromptAnimal:
                case ExperienceState.PromptMantis:
                case ExperienceState.PromptFree:
                    _captureLatched = false; // each prompt needs ITS OWN capture
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
