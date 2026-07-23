// EditMode tests for the one-second-take experience state machine: full happy
// path, the privacy-consent gate, per-state latch clearing, calibrate-timeout
// default path, the TestMove practice rounds (director-owned TestMoveDone),
// processing fail path, ResultShow minimum dwell + ResultShowDone gate +
// no-early-leave, one-frame failure pulses, the debounced leave grace
// (chattering / brief step-outs never drop the run), skip flags, timeMultiplier.

using Experience;
using NUnit.Framework;

namespace Calibration.Tests
{
    public class ExperienceStateMachineTests
    {
        private ExperienceStateMachine _sm;
        private ExperienceTimings _t;
        private ExperienceInputs _in;

        [SetUp]
        public void SetUp()
        {
            _sm = new ExperienceStateMachine();
            _t = new ExperienceTimings
            {
                consentSeconds = 6f,
                welcomeSeconds = 4f,
                calibrateSeconds = 10f,
                calibrateMatchedSeconds = 1.5f,
                processingTimeoutSeconds = 60f,
                resultMinSeconds = 4f,
                exportFailNoticeSeconds = 5f,
                qrShowSeconds = 30f,
                leaveGraceSeconds = 5f,
            };
            _in = new ExperienceInputs { Present = true };
            _sm.ResetTo(ExperienceState.Idle);
        }

        private void Tick(float dt = 0.1f) => _sm.Tick(dt, in _in, _t);

        private void TickUntil(ExperienceState expected, float dt = 0.5f, int maxTicks = 200)
        {
            for (int i = 0; i < maxTicks && _sm.State != expected; i++) Tick(dt);
            Assert.AreEqual(expected, _sm.State);
        }

        // ---- entry-stage helpers (Idle → Consent → Welcome → Calibrate) ----

        private void ReachConsent()
        {
            Tick(); // Present → Consent
            Assert.AreEqual(ExperienceState.Consent, _sm.State);
        }

        private void ReachWelcome()
        {
            ReachConsent();
            TickUntil(ExperienceState.Welcome, dt: 1f); // consentSeconds timer
        }

        private void ReachCalibrate()
        {
            ReachWelcome();
            TickUntil(ExperienceState.Calibrate, dt: 1f); // welcomeSeconds timer
        }

        // Star pose matched, then tick through the はかれたよ！ hold beat to TestMove1.
        private void MatchStarPose()
        {
            _in.CalibrationDone = true;
            TickUntil(ExperienceState.TestMove1, dt: 1f); // calibrateMatchedSeconds beat
            _in.CalibrationDone = false;
        }

        // Complete the current practice round (director-owned TestMoveDone pulse).
        private void FinishTestMove(ExperienceState expectedNext)
        {
            _in.TestMoveDone = true;
            Tick();
            _in.TestMoveDone = false;
            Assert.AreEqual(expectedNext, _sm.State);
        }

        // ---- happy path ----

        [Test]
        public void HappyPath_FullSequence()
        {
            ReachCalibrate(); // Present → Consent → Welcome → Calibrate

            MatchStarPose();
            Assert.AreEqual(ExperienceState.TestMove1, _sm.State);

            FinishTestMove(ExperienceState.TestMove2); // practice 1 → practice 2
            FinishTestMove(ExperienceState.Shoot);     // practice 2 → ほんばん

            _in.RecordingDone = true;
            Tick();
            Assert.AreEqual(ExperienceState.Processing, _sm.State);
            _in.RecordingDone = false;

            _in.ProcessingDone = true;
            Tick();
            Assert.AreEqual(ExperienceState.ResultShow, _sm.State);
            _in.ProcessingDone = false;

            _in.ExportDone = true;
            _in.ResultShowDone = true; // replay loops finished
            TickUntil(ExperienceState.QrShow, dt: 1f); // resultMinSeconds dwell
            _in.ExportDone = false;
            _in.ResultShowDone = false;

            _in.Present = false; // walked off after scanning
            Tick();
            Assert.AreEqual(ExperienceState.Idle, _sm.State);
        }

        // ---- Idle ----

        [Test]
        public void Idle_WaitsForPresence()
        {
            _in.Present = false;
            Tick();
            Assert.AreEqual(ExperienceState.Idle, _sm.State);
            _in.Present = true;
            Tick();
            Assert.AreEqual(ExperienceState.Consent, _sm.State);
        }

        // ---- Consent (privacy gate) ----

        [Test]
        public void Consent_ShownFirst_ThenAdvancesToWelcome()
        {
            ReachConsent();
            Tick(3f);
            Assert.AreEqual(ExperienceState.Consent, _sm.State); // 3s < 6s
            TickUntil(ExperienceState.Welcome, dt: 1f);
        }

        [Test]
        public void Consent_LeaveEarly_OptsOutToIdle()
        {
            ReachConsent();
            _in.Present = false;
            Tick(1f);
            Assert.AreEqual(ExperienceState.Consent, _sm.State); // 1s < 5s grace
            TickUntil(ExperienceState.Idle, dt: 1f);             // grace elapses
        }

        [Test]
        public void Consent_Skip_AdvancesImmediately()
        {
            _t.skipConsent = true;
            ReachConsent();
            Tick();
            Assert.AreEqual(ExperienceState.Welcome, _sm.State);
        }

        // ---- Welcome ----

        [Test]
        public void Welcome_TimerAdvancesToCalibrate()
        {
            ReachWelcome();
            Tick(2f);
            Assert.AreEqual(ExperienceState.Welcome, _sm.State); // 2s < 4s
            for (int i = 0; i < 3; i++) Tick(1f);
            Assert.AreEqual(ExperienceState.Calibrate, _sm.State);
        }

        [Test]
        public void Welcome_LeaveEarly_ReturnsToIdleAfterGrace()
        {
            ReachWelcome();
            _in.Present = false;
            Tick(1f);
            Assert.AreEqual(ExperienceState.Welcome, _sm.State); // grace holds
            TickUntil(ExperienceState.Idle, dt: 1f);
        }

        [Test]
        public void Welcome_Skip_AdvancesImmediately()
        {
            _t.skipWelcome = true;
            ReachWelcome();
            Tick();
            Assert.AreEqual(ExperienceState.Calibrate, _sm.State);
        }

        // ---- Calibrate ----

        [Test]
        public void Calibrate_TimeoutAdvancesWithoutDone()
        {
            ReachCalibrate();
            for (int i = 0; i < 25; i++) Tick(0.5f); // 12.5s > 10s
            Assert.AreEqual(ExperienceState.TestMove1, _sm.State);
        }

        [Test]
        public void Calibrate_MatchedPose_HeldForBeatBeforeTestMove()
        {
            // The star match must linger so はかれたよ！ + the revealed ribbons are
            // readable — the FSM holds Calibrate calibrateMatchedSeconds after the
            // match instead of advancing on the very next tick.
            ReachCalibrate();
            _in.CalibrationDone = true;
            Tick(0.5f); // matched, but within the 1.5s beat
            Assert.AreEqual(ExperienceState.Calibrate, _sm.State);
            Tick(0.5f); // 1.0s < 1.5s
            Assert.AreEqual(ExperienceState.Calibrate, _sm.State);
            Tick(0.6f); // 1.6s >= 1.5s → beat done
            Assert.AreEqual(ExperienceState.TestMove1, _sm.State);
        }

        [Test]
        public void Calibrate_MatchNearWindowEnd_StillGetsFullBeat()
        {
            // A star pose that lands right as the window closes must still get its
            // full matched beat — the timeout must never preempt a latched match.
            ReachCalibrate();
            for (int i = 0; i < 19; i++) Tick(0.5f); // 9.5s < 10s window
            Assert.AreEqual(ExperienceState.Calibrate, _sm.State);
            _in.CalibrationDone = true;
            Tick(0.5f); // TimeInState now 10.0s (>= window) but latched → hold, not timeout
            Assert.AreEqual(ExperienceState.Calibrate, _sm.State);
            Tick(1.2f); // matched beat 1.7s >= 1.5s
            Assert.AreEqual(ExperienceState.TestMove1, _sm.State);
        }

        [Test]
        public void Calibrate_MatchLatched_HoldsBeatEvenIfDoneFlagDrops()
        {
            // The match latches: even if CalibrationDone falls back to false during
            // the beat (BT confidence flicker), the hold still completes.
            ReachCalibrate();
            _in.CalibrationDone = true;
            Tick(0.5f);
            _in.CalibrationDone = false; // flicker off mid-beat
            Assert.AreEqual(ExperienceState.Calibrate, _sm.State);
            Tick(1.2f); // 1.7s total >= 1.5s
            Assert.AreEqual(ExperienceState.TestMove1, _sm.State);
        }

        [Test]
        public void Calibrate_StaleDoneFlagFromPreviousRun_DoesNotSkip()
        {
            // CalibrationDone pulses while still in Idle (stale director
            // flag) — must not fast-forward the Calibrate that follows.
            _in.Present = false;
            _in.CalibrationDone = true;
            Tick();
            Assert.AreEqual(ExperienceState.Idle, _sm.State);
            _in.CalibrationDone = false;
            _in.Present = true;
            ReachCalibrate();
            Tick();
            Assert.AreEqual(ExperienceState.Calibrate, _sm.State);
        }

        // ---- TestMove practice rounds ----

        [Test]
        public void TestMove_AdvancesOnlyOnDone_And_LeaveReturnsToIdle()
        {
            ReachCalibrate();
            MatchStarPose();
            Assert.AreEqual(ExperienceState.TestMove1, _sm.State);
            for (int i = 0; i < 60; i++) Tick(1f); // no machine-side timeout
            Assert.AreEqual(ExperienceState.TestMove1, _sm.State);
            _in.Present = false;
            TickUntil(ExperienceState.Idle, dt: 1f); // leave grace elapses

            _in.Present = true;
            ReachCalibrate();
            MatchStarPose();
            FinishTestMove(ExperienceState.TestMove2);
            FinishTestMove(ExperienceState.Shoot);
        }

        [Test]
        public void TestMove_StaleDoneFlag_ClearedOnEachEntry()
        {
            // The latch from practice 1 must not fast-forward practice 2.
            ReachCalibrate();
            MatchStarPose();
            _in.TestMoveDone = true;
            Tick();
            Assert.AreEqual(ExperienceState.TestMove2, _sm.State);
            _in.TestMoveDone = false;
            Tick(5f);
            Assert.AreEqual(ExperienceState.TestMove2, _sm.State); // waits for its own Done
            FinishTestMove(ExperienceState.Shoot);
        }

        [Test]
        public void TestMove_PresentingSuspendsLeaveReset()
        {
            // Once the presentation stretch runs (recording stopped → convert →
            // load → replay), a presence flicker must not reset the run — the
            // ghost owns the merge there and the fallback presence signals blink
            // across the conversion/load gaps.
            ReachCalibrate();
            MatchStarPose();
            _in.TestMovePresenting = true;
            _in.Present = false;
            for (int i = 0; i < 20; i++) Tick(1f); // 20s absent >> 5s grace
            Assert.AreEqual(ExperienceState.TestMove1, _sm.State);
            // Presentation over, visitor genuinely gone → normal grace reset.
            _in.TestMovePresenting = false;
            TickUntil(ExperienceState.Idle, dt: 1f);
        }

        [Test]
        public void TestMove_PresentingStillAdvancesOnDone()
        {
            ReachCalibrate();
            MatchStarPose();
            _in.TestMovePresenting = true;
            _in.Present = false; // walked off mid-presentation
            _in.TestMoveDone = true;
            Tick();
            _in.TestMoveDone = false;
            Assert.AreEqual(ExperienceState.TestMove2, _sm.State); // presentation completed normally
            _in.TestMovePresenting = false;
            _in.Present = true;
        }

        [Test]
        public void TestMove_SkipFlag_SkipsBothRounds()
        {
            _t.skipTestMoves = true;
            ReachCalibrate();
            MatchStarPose(); // lands in TestMove1
            Tick(); // TestMove1 → TestMove2 (skip)
            Tick(); // TestMove2 → Shoot (skip)
            Assert.AreEqual(ExperienceState.Shoot, _sm.State);
        }

        // ---- leave grace (chattering / brief step-outs) ----

        [Test]
        public void LeaveGrace_ChatteringDoesNotReset()
        {
            ReachCalibrate();
            // Presence flickers off/on, always returning before the grace window.
            for (int i = 0; i < 8; i++)
            {
                _in.Present = false; Tick(0.4f);
                Assert.AreEqual(ExperienceState.Calibrate, _sm.State);
                _in.Present = true; Tick(0.1f); // back inside → grace clock resets
            }
            Assert.AreEqual(ExperienceState.Calibrate, _sm.State);
        }

        [Test]
        public void LeaveGrace_ResetsOnlyAfterContinuousAbsence()
        {
            ReachCalibrate();
            _in.Present = false;
            Tick(2f);
            Assert.AreEqual(ExperienceState.Calibrate, _sm.State); // 2s < 5s
            Tick(2f);
            Assert.AreEqual(ExperienceState.Calibrate, _sm.State); // 4s < 5s
            Tick(2f);
            Assert.AreEqual(ExperienceState.Idle, _sm.State);      // 6s >= 5s
        }

        [Test]
        public void LeaveGrace_AbsenceDoesNotConsumeStateDwell()
        {
            // The consent window must count only real presence: a sub-grace
            // step-out pauses the clock instead of advancing it, so returning
            // does NOT let a barely-attended consent fast-forward to Welcome.
            ReachConsent();
            Tick(3f); // 3s present in Consent (< 6s)
            Assert.AreEqual(ExperienceState.Consent, _sm.State);
            _in.Present = false;
            Tick(4f); // 4s absent (< 5s grace) — must not consume the window
            Assert.AreEqual(ExperienceState.Consent, _sm.State);
            _in.Present = true;
            Tick(2f); // 3+2 = 5s present (< 6s) — still short (absence didn't count)
            Assert.AreEqual(ExperienceState.Consent, _sm.State);
            Tick(2f); // 7s present (>= 6s) → Welcome
            Assert.AreEqual(ExperienceState.Welcome, _sm.State);
        }

        [Test]
        public void LeaveGrace_ReturningResetsTheClock()
        {
            ReachCalibrate();
            _in.Present = false;
            Tick(4f); // 4s absent (< 5s)
            Assert.AreEqual(ExperienceState.Calibrate, _sm.State);
            _in.Present = true;
            Tick(0.1f); // back inside → clock resets
            _in.Present = false;
            Tick(4f); // 4s absent again — cumulative would be 8s, but the clock reset
            Assert.AreEqual(ExperienceState.Calibrate, _sm.State);
        }

        // ---- Shoot ----

        private void ReachShoot()
        {
            ReachCalibrate();
            MatchStarPose();
            FinishTestMove(ExperienceState.TestMove2);
            FinishTestMove(ExperienceState.Shoot);
        }

        [Test]
        public void Shoot_AdvancesOnlyOnRecordingDone()
        {
            ReachShoot();
            for (int i = 0; i < 100; i++) Tick(1f); // no machine-side timeout
            Assert.AreEqual(ExperienceState.Shoot, _sm.State);
            _in.RecordingDone = true;
            Tick();
            Assert.AreEqual(ExperienceState.Processing, _sm.State);
        }

        [Test]
        public void Shoot_StaleRecordingFlag_ClearedOnEntry()
        {
            ReachCalibrate();
            _in.RecordingDone = true; // stale pulse long before Shoot
            MatchStarPose();          // through the Calibrate beat → TestMove1
            _in.RecordingDone = false; // cleared before Shoot (latch clears on entry)
            FinishTestMove(ExperienceState.TestMove2);
            FinishTestMove(ExperienceState.Shoot);
            Tick();
            Assert.AreEqual(ExperienceState.Shoot, _sm.State); // did not skip ahead
        }

        // ---- Processing ----

        private void ReachProcessing()
        {
            ReachShoot();
            _in.RecordingDone = true; Tick(); _in.RecordingDone = false;
            Assert.AreEqual(ExperienceState.Processing, _sm.State);
        }

        [Test]
        public void Processing_FailPulse_ShowsNoticeThenIdle()
        {
            ReachProcessing();
            _in.ProcessingFailed = true;
            Tick();
            _in.ProcessingFailed = false; // one-frame pulse
            Assert.AreEqual(ExperienceState.Processing, _sm.State);
            // Nobody present after the failure — Idle must be reached by the
            // notice TIMER (the fail latch suppresses LeftEarly), and staying
            // absent keeps the machine parked there for the assert.
            _in.Present = false;
            Tick(0.5f);
            Assert.AreEqual(ExperienceState.Processing, _sm.State); // notice still showing
            for (int i = 0; i < 12; i++) Tick(0.5f); // > 5s notice
            Assert.AreEqual(ExperienceState.Idle, _sm.State);
        }

        [Test]
        public void Processing_LeaveEarly_ReturnsToIdleAfterGrace()
        {
            ReachProcessing();
            _in.Present = false;
            Tick(1f);
            Assert.AreEqual(ExperienceState.Processing, _sm.State); // grace holds
            TickUntil(ExperienceState.Idle, dt: 1f);
        }

        // ---- ResultShow ----

        private void ReachResultShow()
        {
            ReachProcessing();
            _in.ProcessingDone = true; Tick(); _in.ProcessingDone = false;
            Assert.AreEqual(ExperienceState.ResultShow, _sm.State);
        }

        [Test]
        public void ResultShow_WaitsMinimumDwellEvenWhenExportIsInstant()
        {
            ReachResultShow();
            _in.ExportDone = true;
            _in.ResultShowDone = true;
            Tick(1f);
            Assert.AreEqual(ExperienceState.ResultShow, _sm.State); // 1s < 4s dwell
            for (int i = 0; i < 8; i++) Tick(1f);
            Assert.AreEqual(ExperienceState.QrShow, _sm.State);
        }

        [Test]
        public void ResultShow_WaitsForReplayLoopsEvenWhenExportIsDone()
        {
            // The QR must not appear while the replay presentation still runs.
            ReachResultShow();
            _in.ExportDone = true;
            for (int i = 0; i < 10; i++) Tick(1f); // dwell satisfied, loops not
            Assert.AreEqual(ExperienceState.ResultShow, _sm.State);
            _in.ResultShowDone = true; // pulse — latched
            Tick();
            _in.ResultShowDone = false;
            Tick();
            Assert.AreEqual(ExperienceState.QrShow, _sm.State);
        }

        [Test]
        public void ResultShow_NoEarlyLeave()
        {
            ReachResultShow();
            _in.Present = false;
            for (int i = 0; i < 10; i++) Tick(1f);
            Assert.AreEqual(ExperienceState.ResultShow, _sm.State); // export still running
        }

        [Test]
        public void ResultShow_FailPulse_NoticeThenIdle()
        {
            ReachResultShow();
            _in.ExportFailed = true;
            Tick();
            _in.ExportFailed = false;
            Assert.AreEqual(ExperienceState.ResultShow, _sm.State);
            _in.Present = false; // keeps the post-notice Idle parked for the assert
            for (int i = 0; i < 12; i++) Tick(0.5f);
            Assert.AreEqual(ExperienceState.Idle, _sm.State);
        }

        [Test]
        public void ResultShow_SkipQr_GoesStraightToIdle()
        {
            _t.skipQr = true;
            ReachResultShow();
            _in.ExportDone = true;
            _in.ResultShowDone = true;
            _in.Present = false; // park the Idle that follows
            for (int i = 0; i < 10; i++) Tick(1f);
            Assert.AreEqual(ExperienceState.Idle, _sm.State);
        }

        // ---- QrShow ----

        [Test]
        public void QrShow_TimesOutToIdle()
        {
            ReachResultShow();
            _in.ExportDone = true;
            _in.ResultShowDone = true;
            TickUntil(ExperienceState.QrShow, dt: 1f);
            // Present stays true — Idle must be reached by the TIMER.
            // TickUntil stops the moment Idle appears (before Idle could
            // advance again on the next tick).
            TickUntil(ExperienceState.Idle, dt: 1f, maxTicks: 40);
        }

        // ---- fault ----

        [Test]
        public void Fault_PreemptsAnyState_AndRecovers()
        {
            ReachProcessing();
            _in.Fault = true;
            Tick();
            Assert.AreEqual(ExperienceState.Fault, _sm.State);
            _in.Fault = false;
            _in.Present = false;
            Tick();
            Assert.AreEqual(ExperienceState.Idle, _sm.State);
        }

        // ---- skips ----

        [Test]
        public void Skips_ReachProcessingThroughEveryState()
        {
            _t.skipIdle = true;
            _t.skipConsent = true;
            _t.skipWelcome = true;
            _t.skipCalibrate = true;
            _t.skipTestMoves = true;
            _t.skipShoot = true;

            var visited = new System.Collections.Generic.List<ExperienceState>();
            _sm.Changed += (from, to) => visited.Add(to);

            for (int i = 0; i < 8; i++) Tick();
            Assert.AreEqual(ExperienceState.Processing, _sm.State);
            CollectionAssert.AreEqual(
                new[]
                {
                    ExperienceState.Consent, ExperienceState.Welcome,
                    ExperienceState.Calibrate, ExperienceState.TestMove1,
                    ExperienceState.TestMove2, ExperienceState.Shoot,
                    ExperienceState.Processing,
                },
                visited);

            // skipShoot does NOT auto-advance Processing — the canned take still
            // goes through the director's play-through+capture, which ends in
            // ProcessingDone as usual.
            _in.ProcessingDone = true;
            Tick();
            Assert.AreEqual(ExperienceState.ResultShow, _sm.State);
        }

        // ---- time multiplier ----

        [Test]
        public void TimeMultiplier_ScalesConsentAndWelcomeTimeouts()
        {
            _t.timeMultiplier = 10f;
            Tick(); // Idle → Consent (presence)
            Assert.AreEqual(ExperienceState.Consent, _sm.State);
            Tick(0.7f); // 0.7s * 10 = 7s > 6s consent → Welcome
            Assert.AreEqual(ExperienceState.Welcome, _sm.State);
            Tick(0.5f); // 0.5s * 10 = 5s > 4s welcome → Calibrate
            Assert.AreEqual(ExperienceState.Calibrate, _sm.State);
            for (int i = 0; i < 3; i++) Tick(0.5f); // 1.5s * 10 = 15s > 10s
            Assert.AreEqual(ExperienceState.TestMove1, _sm.State);
        }
    }
}
