// EditMode tests for the one-second-take experience state machine: full happy
// path, per-state latch clearing, calibrate-timeout default path, FreeMove
// timer, processing fail path, ResultShow minimum dwell + no-early-leave,
// one-frame failure pulses, leave/fault interruptions, skip flags,
// timeMultiplier.

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
                calibrateSeconds = 10f,
                freeMoveSeconds = 20f,
                processingTimeoutSeconds = 60f,
                resultMinSeconds = 4f,
                exportFailNoticeSeconds = 5f,
                qrShowSeconds = 30f,
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

        // ---- happy path ----

        [Test]
        public void HappyPath_FullSequence()
        {
            Tick(); // Present → Calibrate
            Assert.AreEqual(ExperienceState.Calibrate, _sm.State);

            _in.CalibrationDone = true;
            Tick();
            Assert.AreEqual(ExperienceState.FreeMove, _sm.State);
            _in.CalibrationDone = false;

            TickUntil(ExperienceState.Shoot, dt: 2f); // freeMoveSeconds timer

            _in.RecordingDone = true;
            Tick();
            Assert.AreEqual(ExperienceState.Processing, _sm.State);
            _in.RecordingDone = false;

            _in.ProcessingDone = true;
            Tick();
            Assert.AreEqual(ExperienceState.ResultShow, _sm.State);
            _in.ProcessingDone = false;

            _in.ExportDone = true;
            TickUntil(ExperienceState.QrShow, dt: 1f); // resultMinSeconds dwell
            _in.ExportDone = false;

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
            Assert.AreEqual(ExperienceState.Calibrate, _sm.State);
        }

        // ---- Calibrate ----

        [Test]
        public void Calibrate_TimeoutAdvancesWithoutDone()
        {
            Tick(); // → Calibrate
            for (int i = 0; i < 25; i++) Tick(0.5f); // 12.5s > 10s
            Assert.AreEqual(ExperienceState.FreeMove, _sm.State);
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
            Tick(); // → Calibrate
            Tick();
            Assert.AreEqual(ExperienceState.Calibrate, _sm.State);
        }

        // ---- FreeMove ----

        [Test]
        public void FreeMove_TimerAdvances_And_LeaveReturnsToIdle()
        {
            Tick();
            _in.CalibrationDone = true; Tick(); _in.CalibrationDone = false;
            Assert.AreEqual(ExperienceState.FreeMove, _sm.State);
            Tick(5f);
            Assert.AreEqual(ExperienceState.FreeMove, _sm.State); // 5s < 20s
            _in.Present = false;
            Tick();
            Assert.AreEqual(ExperienceState.Idle, _sm.State);

            _in.Present = true;
            Tick(); // → Calibrate
            _in.CalibrationDone = true; Tick(); _in.CalibrationDone = false;
            TickUntil(ExperienceState.Shoot, dt: 2f);
        }

        // ---- Shoot ----

        private void ReachShoot()
        {
            Tick();
            _in.CalibrationDone = true; Tick(); _in.CalibrationDone = false;
            TickUntil(ExperienceState.Shoot, dt: 2f);
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
            Tick();
            _in.CalibrationDone = true;
            _in.RecordingDone = true; // stale pulse long before Shoot
            Tick(); // → FreeMove (latch for Shoot must clear on Shoot entry)
            _in.CalibrationDone = false;
            _in.RecordingDone = false;
            TickUntil(ExperienceState.Shoot, dt: 2f);
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
        public void Processing_LeaveEarly_ReturnsToIdle()
        {
            ReachProcessing();
            _in.Present = false;
            Tick();
            Assert.AreEqual(ExperienceState.Idle, _sm.State);
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
            Tick(1f);
            Assert.AreEqual(ExperienceState.ResultShow, _sm.State); // 1s < 4s dwell
            for (int i = 0; i < 8; i++) Tick(1f);
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
            _t.skipCalibrate = true;
            _t.skipFreeMove = true;
            _t.skipShoot = true;

            var visited = new System.Collections.Generic.List<ExperienceState>();
            _sm.Changed += (from, to) => visited.Add(to);

            for (int i = 0; i < 5; i++) Tick();
            Assert.AreEqual(ExperienceState.Processing, _sm.State);
            CollectionAssert.AreEqual(
                new[]
                {
                    ExperienceState.Calibrate, ExperienceState.FreeMove,
                    ExperienceState.Shoot, ExperienceState.Processing,
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
        public void TimeMultiplier_ScalesCalibrateTimeout()
        {
            _t.timeMultiplier = 10f;
            Tick(); // → Calibrate
            for (int i = 0; i < 3; i++) Tick(0.5f); // 1.5s * 10 = 15s > 10s
            Assert.AreEqual(ExperienceState.FreeMove, _sm.State);
        }
    }
}
