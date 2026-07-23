// Pure-C# state machine for the visitor experience — the one-second-take
// sequence: Idle (unattended: just the floor grid, no attract content) →
// Welcome (greeting on area entry, machine-owned timer) →
// Calibrate (star pose → per-visitor bone profile for the fusion) →
// TestMove1 / TestMove2 (practice rounds: intro → countdown → one second
// recorded → converted → played back with the ribbons, no upload; wholly
// director-owned, the machine waits on TestMoveDone) → Shoot (ほんばん cue →
// countdown → ONE second recorded at zero) → Processing (v11s conversion +
// capture of the final one-second curve window) → ResultShow (the take loops
// with the ribbons + orbit while glb/usdz export + upload run; advance waits
// on ResultShowDone) → QrShow. No scene access — the ExperienceDirector feeds
// an ExperienceInputs snapshot every Tick and reacts to Changed. Pure so
// every transition rule is EditMode-testable.
//
// Latching contracts (carried over from earlier machines' Codex reviews):
//   - Per-state completion flags (CalibrationDone, RecordingDone,
//     ProcessingDone) are latched INSIDE their owning state only and cleared
//     on that state's enter — a stale director flag can never fast-forward a
//     later visitor's run.
//   - ProcessingFailed / ExportFailed are latched: a one-frame pulse starts
//     the fail-notice timer; the machine holds the state until
//     exportFailNoticeSeconds then falls back to Idle (the director shows
//     the apology meanwhile).
//
// Skip flags mean "enter the state, then advance immediately on the first
// Tick" — the state is still entered so side effects and the E2E transition
// path stay exercised (dev-mode stage skipping).
//
// Duration ownership: the TestMove mini-cycles, Shoot's cue+countdown+capture
// and Processing's timeout/fallback are DIRECTOR-owned (they wrap real device
// work and end in TestMoveDone / RecordingDone / ProcessingDone); Calibrate's
// window, ResultShow's minimum dwell and QrShow's dwell are machine-owned
// timers (ResultShow additionally waits on the director's ResultShowDone).

using System;
using UnityEngine;

namespace Experience
{
    public enum ExperienceState
    {
        Idle, Consent, Welcome, Calibrate, TestMove1, TestMove2, Shoot, Processing, ResultShow, QrShow, Fault
    }

    /// <summary>Per-tick inputs from the director (already debounced where
    /// applicable — Present comes from PresenceDetector's hysteresis, or the
    /// occupancy+live-fused blend while recorded bodies own the merge).</summary>
    public struct ExperienceInputs
    {
        public bool Present;
        public bool Fault;
        public bool CalibrationDone;   // star pose held + bone profile stored
        public bool TestMoveDone;      // the practice mini-cycle (record→convert→playback) finished
        // The practice presentation (recording stopped → conversion → load →
        // looped playback) is in progress. LeftEarly is suspended while set:
        // the take's ghost owns the merge there, so presence falls back to
        // occupancy/live-fusion signals that flicker across the conversion and
        // load gaps — a visitor standing still must not be reset mid-できたよ！.
        // Director-owned and bounded (conversion timeout + playback deadline),
        // so the machine can never be held here forever.
        public bool TestMovePresenting;
        public bool RecordingDone;     // the one-second take is on disk
        public bool ProcessingDone;    // conversion + final capture finished
        public bool ProcessingFailed;  // unrecoverable (no take) — may be a pulse
        public bool ExportDone;
        public bool ExportFailed;      // may be a one-frame pulse
        public bool ResultShowDone;    // the ResultShow playback loops finished
    }

    /// <summary>Timing + dev-skip knobs (plain POCO, filled from
    /// ExperienceConfig; separate type so tests need no ScriptableObject).</summary>
    [Serializable]
    public class ExperienceTimings
    {
        [Min(0f)]
        [Tooltip("Privacy-consent notice shown on area entry, BEFORE the greeting: the " +
                 "sculpture will be published online, so a visitor who objects opts out " +
                 "by simply leaving the carpet while it shows. Auto-advances to Welcome " +
                 "when the window elapses.")]
        public float consentSeconds = 8f;
        [Min(0f)]
        [Tooltip("Welcome greeting duration on area entry, before the star-pose window.")]
        public float welcomeSeconds = 4f;
        [Min(0f)]
        [Tooltip("Star-pose window. Expiry advances with the DEFAULT bone profile.")]
        public float calibrateSeconds = 10f;
        [Min(0f)]
        [Tooltip("How long はかれたよ！ (pose matched) stays on screen after the star " +
                 "pose is held, before TestMove1 replaces it. Without this the match is " +
                 "a single frame: the FSM latches CalibrationDone and advances on the " +
                 "very next tick. The ribbons also appear on this beat, so it doubles " +
                 "as the sculpture reveal. Only the HELD-pose path waits — window " +
                 "expiry (shy visitor) and skipCalibrate still advance immediately.")]
        public float calibrateMatchedSeconds = 1.5f;
        [Min(5f)]
        [Tooltip("Director-enforced ceiling on Processing (conversion + capture); " +
                 "expiry falls back to whatever bodies the take carries.")]
        public float processingTimeoutSeconds = 60f;
        [Min(0f)]
        [Tooltip("Minimum time the finished sculpture stays on screen even when the " +
                 "export/upload finishes faster.")]
        public float resultMinSeconds = 5f;
        [Min(0f)] public float exportFailNoticeSeconds = 5f;
        [Min(0f)] public float qrShowSeconds = 30f;

        [Min(0f)]
        [Tooltip("Grace period after the visitor leaves the sensing area before the run " +
                 "resets to Idle. Absorbs presence chattering and momentary occlusion / " +
                 "brief step-outs — the visitor must stay outside this long for the reset " +
                 "to fire. Applies to every interactive stage (Consent..Processing).")]
        public float leaveGraceSeconds = 5f;

        [Header("Dev stage skipping (enter, then advance immediately)")]
        [UnityEngine.Serialization.FormerlySerializedAs("skipAttract")]
        public bool skipIdle;
        public bool skipConsent;
        public bool skipWelcome;
        public bool skipCalibrate;   // default bone profile
        [UnityEngine.Serialization.FormerlySerializedAs("skipFreeMove")]
        [Tooltip("Skip BOTH practice rounds (TestMove1/TestMove2).")]
        public bool skipTestMoves;
        public bool skipShoot;       // director substitutes devCannedTakeRoot
        [Tooltip("Shoot/TestMove play out in full (cue, countdown, capture window). " +
                 "With no cameras but a RUNNING playback (the entrance stand-in), the " +
                 "capture window is REALLY recorded by tapping the playback stream — a " +
                 "production-length take, bodies passed through, no conversion needed. " +
                 "Only when nothing is playing does devCannedTakeRoot substitute. For " +
                 "rigless (Mac) runs that still need the full UX.")]
        public bool dummyShoot;      // FSM-inert: advance still rides RecordingDone
        public bool skipProcessing;  // skip the v11s conversion (capture still runs)
        public bool skipQr;          // ResultShow success goes straight to Idle

        [Min(0f)]
        [Tooltip("Blank beat between state screens: on entering a state the visitor " +
                 "UI clears, and the new screen (message + its audio cue) lands this " +
                 "many seconds later. State logic and scene-side visuals are not " +
                 "delayed; Fault and Idle paint immediately. 0 = hard cut. " +
                 "FSM-inert — consumed by the director.")]
        public float stateGapSeconds = 1f;

        [Min(0.01f)]
        [Tooltip("Dev time multiplier applied to Tick dt (2 = twice as fast).")]
        public float timeMultiplier = 1f;
    }

    public sealed class ExperienceStateMachine
    {
        public ExperienceState State { get; private set; } = ExperienceState.Idle;
        public float TimeInState { get; private set; }

        public event Action<ExperienceState, ExperienceState> Changed;

        private bool _calibLatched;
        private float _matchedElapsed; // time held in Calibrate AFTER the star match
        private bool _testMoveLatched;
        private bool _resultShowLatched;
        private bool _recordingLatched;
        private bool _processingLatched;
        private bool _processingFailLatched;
        private float _processingFailElapsed;
        private bool _exportFailLatched;
        private float _exportFailElapsed;
        // How long the visitor has been continuously absent in an interactive
        // stage. The reset to Idle waits until this passes leaveGraceSeconds so
        // presence chattering / a brief step-out never drops the run.
        private float _absentElapsed;

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
                    if (!inputs.Fault) Go(ExperienceState.Idle);
                    break;

                case ExperienceState.Idle:
                    if (t.skipIdle || inputs.Present) Go(ExperienceState.Consent);
                    break;

                case ExperienceState.Consent:
                    // Privacy gate: the visitor reads that the sculpture goes online
                    // and opts out by leaving (LeftEarly → Idle). Staying through the
                    // window is the consent — advance to the greeting.
                    if (LeftEarly(inputs)) break;
                    if (t.skipConsent || TimeInState >= t.consentSeconds)
                        Go(ExperienceState.Welcome);
                    break;

                case ExperienceState.Welcome:
                    if (LeftEarly(inputs)) break;
                    if (t.skipWelcome || TimeInState >= t.welcomeSeconds)
                        Go(ExperienceState.Calibrate);
                    break;

                case ExperienceState.Calibrate:
                    if (LeftEarly(inputs)) break;
                    if (inputs.CalibrationDone) _calibLatched = true;
                    // skipCalibrate is always immediate. Window expiry advances ONLY
                    // a visitor who has not matched — a star pose that lands near the
                    // end of the window still earns its full はかれたよ！ beat below and
                    // is never cut short by the timeout firing on the next tick (the
                    // director keeps the default bone profile; the show must never
                    // stall on a shy visitor, but a real match is not "shy").
                    if (t.skipCalibrate || (!_calibLatched && TimeInState >= t.calibrateSeconds))
                    {
                        Go(ExperienceState.TestMove1);
                        break;
                    }
                    // A HELD star pose earns はかれたよ！ its own beat: hold Calibrate
                    // calibrateMatchedSeconds after the match so the matched text and
                    // the freshly-revealed ribbons are readable before TestMove1's
                    // message replaces them.
                    if (_calibLatched)
                    {
                        _matchedElapsed += dt;
                        if (_matchedElapsed >= t.calibrateMatchedSeconds)
                            Go(ExperienceState.TestMove1);
                    }
                    break;

                // The practice rounds are wholly director-owned (intro texts →
                // countdown+record → conversion → looped playback) — like Shoot,
                // the machine has no timeout of its own and rides the latched
                // completion flag. The leave reset applies only to the LIVE half
                // (intro/countdown/recording): once the presentation runs
                // (TestMovePresenting) it plays to completion, same rationale as
                // ResultShow's no-early-leave.
                case ExperienceState.TestMove1:
                    if (!inputs.TestMovePresenting && LeftEarly(inputs)) break;
                    if (inputs.TestMoveDone) _testMoveLatched = true;
                    if (t.skipTestMoves || _testMoveLatched)
                        Go(ExperienceState.TestMove2);
                    break;

                case ExperienceState.TestMove2:
                    if (!inputs.TestMovePresenting && LeftEarly(inputs)) break;
                    if (inputs.TestMoveDone) _testMoveLatched = true;
                    if (t.skipTestMoves || _testMoveLatched)
                        Go(ExperienceState.Shoot);
                    break;

                case ExperienceState.Shoot:
                    if (LeftEarly(inputs)) break;
                    if (inputs.RecordingDone) _recordingLatched = true;
                    if (t.skipShoot || _recordingLatched) Go(ExperienceState.Processing);
                    break;

                case ExperienceState.Processing:
                    if (inputs.ProcessingFailed) _processingFailLatched = true;
                    if (_processingFailLatched)
                    {
                        _processingFailElapsed += dt;
                        if (_processingFailElapsed >= t.exportFailNoticeSeconds)
                            Go(ExperienceState.Idle);
                        break;
                    }
                    if (LeftEarly(inputs)) break;
                    if (inputs.ProcessingDone) _processingLatched = true;
                    if (_processingLatched) Go(ExperienceState.ResultShow);
                    break;

                case ExperienceState.ResultShow:
                    // No early-leave: the export always runs to a conclusion and
                    // the finished piece deserves its moment on screen.
                    if (inputs.ExportFailed) _exportFailLatched = true;
                    if (_exportFailLatched)
                    {
                        _exportFailElapsed += dt;
                        if (_exportFailElapsed >= t.exportFailNoticeSeconds)
                            Go(ExperienceState.Idle);
                        break;
                    }
                    // The result presentation (the take looping with the ribbons)
                    // must finish alongside the export before the QR appears.
                    if (inputs.ResultShowDone) _resultShowLatched = true;
                    if (inputs.ExportDone && _resultShowLatched
                        && TimeInState >= t.resultMinSeconds)
                        Go(t.skipQr ? ExperienceState.Idle : ExperienceState.QrShow);
                    break;

                case ExperienceState.QrShow:
                    if (TimeInState >= t.qrShowSeconds || !inputs.Present)
                        Go(ExperienceState.Idle);
                    break;
            }

            // Absence in an interactive stage only resets the run after the
            // visitor has been continuously gone for leaveGraceSeconds — a
            // chattering signal or a momentary step-out holds the state instead
            // of dropping it. Returns true whenever the visitor is absent (hold
            // or reset), so the caller skips this tick's normal progression.
            bool LeftEarly(in ExperienceInputs i)
            {
                if (i.Present) { _absentElapsed = 0f; return false; }
                // Freeze the state's own clock while absent: undo this tick's
                // TimeInState advance so every dwell (above all the consent
                // window, and the director-owned Shoot cue/countdown/capture
                // which reads TimeInState) measures REAL presence, never
                // wall-clock that elapsed while the visitor was gone. Without
                // this a sub-grace step-out could expire the consent timer or
                // fire recording start / RecordingDone with nobody present.
                TimeInState -= dt;
                _absentElapsed += dt;
                if (_absentElapsed >= t.leaveGraceSeconds) Go(ExperienceState.Idle);
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
            _matchedElapsed = 0f;
            _testMoveLatched = false;
            _resultShowLatched = false;
            _recordingLatched = false;
            _processingLatched = false;
            _processingFailLatched = false;
            _processingFailElapsed = 0f;
            _exportFailLatched = false;
            _exportFailElapsed = 0f;
            _absentElapsed = 0f;
        }

        private void Go(ExperienceState to)
        {
            var from = State;
            State = to;
            TimeInState = 0f;
            _absentElapsed = 0f; // every state starts its own leave-grace clock
            // Clear each state's OWN latches on entry — stale director flags
            // must never fast-forward the next visitor's run.
            switch (to)
            {
                case ExperienceState.Calibrate:
                    _calibLatched = false;
                    _matchedElapsed = 0f;
                    break;
                case ExperienceState.TestMove1:
                case ExperienceState.TestMove2:
                    _testMoveLatched = false;
                    break;
                case ExperienceState.Shoot:
                    _recordingLatched = false;
                    break;
                case ExperienceState.Processing:
                    _processingLatched = false;
                    _processingFailLatched = false;
                    _processingFailElapsed = 0f;
                    break;
                case ExperienceState.ResultShow:
                    _exportFailLatched = false;
                    _exportFailElapsed = 0f;
                    _resultShowLatched = false;
                    break;
            }
            Changed?.Invoke(from, to);
        }
    }
}
