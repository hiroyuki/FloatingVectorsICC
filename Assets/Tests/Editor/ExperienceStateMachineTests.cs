// EditMode tests for the pose-driven experience state machine: full happy
// path, per-state latch clearing, calibrate-timeout default path, processing
// fail/timeout-fallback, loop-count transitions, one-frame failure pulses,
// leave/fault interruptions, skip flags, timeMultiplier.

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
                exploreSeconds = 10f,
                processingTimeoutSeconds = 90f,
                banzaiFallbackLoops = 3,
                exportFailNoticeSeconds = 5f,
                qrShowSeconds = 30f,
            };
            _in = new ExperienceInputs { Present = true };
            _sm.ResetTo(ExperienceState.Attract);
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
            Assert.AreEqual(ExperienceState.Explore, _sm.State);
            _in.CalibrationDone = false;

            _in.RecordingDone = true;
            Tick();
            Assert.AreEqual(ExperienceState.Processing, _sm.State);
            _in.RecordingDone = false;

            _in.ProcessingDone = true;
            Tick();
            Assert.AreEqual(ExperienceState.Watch, _sm.State);
            _in.ProcessingDone = false;

            _in.PlaybackLoops = 1;
            Tick();
            Assert.AreEqual(ExperienceState.BanzaiWait, _sm.State);

            _in.CaptureDone = true;
            Tick();
            Assert.AreEqual(ExperienceState.Exporting, _sm.State);
            _in.CaptureDone = false;

            _in.ExportDone = true;
            Tick();
            Assert.AreEqual(ExperienceState.QrShow, _sm.State);

            TickUntil(ExperienceState.Attract, dt: 5f); // qrShowSeconds timeout
        }

        // ---- Attract ----

        [Test]
        public void Attract_WaitsForPresence()
        {
            _in.Present = false;
            Tick();
            Assert.AreEqual(ExperienceState.Attract, _sm.State);
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
            Assert.AreEqual(ExperienceState.Explore, _sm.State);
        }

        [Test]
        public void Calibrate_StaleDoneFlagFromPreviousRun_DoesNotSkip()
        {
            // CalibrationDone pulses while still in Attract (stale director
            // flag) — must not fast-forward the Calibrate that follows.
            _in.Present = false;
            _in.CalibrationDone = true;
            Tick();
            Assert.AreEqual(ExperienceState.Attract, _sm.State);
            _in.CalibrationDone = false;
            _in.Present = true;
            Tick(); // → Calibrate
            Tick();
            Assert.AreEqual(ExperienceState.Calibrate, _sm.State);
        }

        [Test]
        public void Calibrate_DonePulse_IsLatched()
        {
            Tick(); // → Calibrate
            _in.CalibrationDone = true;
            Tick();
            Assert.AreEqual(ExperienceState.Explore, _sm.State);
        }

        // ---- Explore ----

        [Test]
        public void Explore_AdvancesOnlyOnRecordingDone()
        {
            Tick();
            _in.CalibrationDone = true; Tick(); _in.CalibrationDone = false;
            Assert.AreEqual(ExperienceState.Explore, _sm.State);
            for (int i = 0; i < 100; i++) Tick(1f); // no machine-side timeout
            Assert.AreEqual(ExperienceState.Explore, _sm.State);
            _in.RecordingDone = true;
            Tick();
            Assert.AreEqual(ExperienceState.Processing, _sm.State);
        }

        // ---- Processing ----

        private void ReachProcessing()
        {
            Tick();
            _in.CalibrationDone = true; Tick(); _in.CalibrationDone = false;
            _in.RecordingDone = true; Tick(); _in.RecordingDone = false;
            Assert.AreEqual(ExperienceState.Processing, _sm.State);
        }

        [Test]
        public void Processing_FailPulse_ShowsNoticeThenAttract()
        {
            ReachProcessing();
            _in.ProcessingFailed = true;
            Tick();
            _in.ProcessingFailed = false; // one-frame pulse
            Assert.AreEqual(ExperienceState.Processing, _sm.State);
            // Nobody present after the failure — Attract must be reached by the
            // notice TIMER (the fail latch suppresses LeftEarly), and staying
            // absent keeps the machine parked there for the assert.
            _in.Present = false;
            Tick(0.5f);
            Assert.AreEqual(ExperienceState.Processing, _sm.State); // notice still showing
            for (int i = 0; i < 12; i++) Tick(0.5f); // > 5s notice
            Assert.AreEqual(ExperienceState.Attract, _sm.State);
        }

        [Test]
        public void Processing_LeaveEarly_ReturnsToAttract()
        {
            ReachProcessing();
            _in.Present = false;
            Tick();
            Assert.AreEqual(ExperienceState.Attract, _sm.State);
        }

        [Test]
        public void Processing_DonePulse_AdvancesToWatch()
        {
            ReachProcessing();
            _in.ProcessingDone = true;
            Tick();
            Assert.AreEqual(ExperienceState.Watch, _sm.State);
        }

        // ---- Watch / BanzaiWait ----

        private void ReachWatch()
        {
            ReachProcessing();
            _in.ProcessingDone = true; Tick(); _in.ProcessingDone = false;
            Assert.AreEqual(ExperienceState.Watch, _sm.State);
        }

        [Test]
        public void Watch_AdvancesAfterFirstLoop()
        {
            ReachWatch();
            for (int i = 0; i < 20; i++) Tick(1f);
            Assert.AreEqual(ExperienceState.Watch, _sm.State); // loops still 0
            _in.PlaybackLoops = 1;
            Tick();
            Assert.AreEqual(ExperienceState.BanzaiWait, _sm.State);
        }

        [Test]
        public void BanzaiWait_StaleCaptureFlag_ClearedOnEntry()
        {
            ReachWatch();
            // capture flag pulses during Watch (stale) — BanzaiWait must clear it
            _in.CaptureDone = true;
            _in.PlaybackLoops = 1;
            Tick(); // → BanzaiWait (latch cleared on entry)
            Assert.AreEqual(ExperienceState.BanzaiWait, _sm.State);
            _in.CaptureDone = false;
            Tick();
            Assert.AreEqual(ExperienceState.BanzaiWait, _sm.State);
            _in.CaptureDone = true;
            Tick();
            Assert.AreEqual(ExperienceState.Exporting, _sm.State);
        }

        // ---- Exporting ----

        private void ReachExporting()
        {
            ReachWatch();
            _in.PlaybackLoops = 1; Tick();
            _in.CaptureDone = true; Tick(); _in.CaptureDone = false;
            Assert.AreEqual(ExperienceState.Exporting, _sm.State);
        }

        [Test]
        public void Exporting_NoEarlyLeave()
        {
            ReachExporting();
            _in.Present = false;
            for (int i = 0; i < 10; i++) Tick(1f);
            Assert.AreEqual(ExperienceState.Exporting, _sm.State);
        }

        [Test]
        public void Exporting_FailPulse_NoticeThenAttract()
        {
            ReachExporting();
            _in.ExportFailed = true;
            Tick();
            _in.ExportFailed = false;
            Assert.AreEqual(ExperienceState.Exporting, _sm.State);
            _in.Present = false; // keeps the post-notice Attract parked for the assert
            for (int i = 0; i < 12; i++) Tick(0.5f);
            Assert.AreEqual(ExperienceState.Attract, _sm.State);
        }

        [Test]
        public void Exporting_SkipQr_GoesStraightToAttract()
        {
            _t.skipQr = true;
            ReachExporting();
            _in.ExportDone = true;
            Tick();
            Assert.AreEqual(ExperienceState.Attract, _sm.State);
        }

        // ---- QrShow ----

        [Test]
        public void QrShow_LeaveEndsShow()
        {
            ReachExporting();
            _in.ExportDone = true; Tick();
            Assert.AreEqual(ExperienceState.QrShow, _sm.State);
            _in.Present = false;
            Tick();
            Assert.AreEqual(ExperienceState.Attract, _sm.State);
        }

        // ---- leave-early coverage ----

        [Test]
        public void LeaveEarly_Calibrate_And_Explore()
        {
            Tick(); // → Calibrate
            _in.Present = false;
            Tick();
            Assert.AreEqual(ExperienceState.Attract, _sm.State);

            _in.Present = true;
            Tick(); // → Calibrate
            _in.CalibrationDone = true; Tick(); _in.CalibrationDone = false;
            Assert.AreEqual(ExperienceState.Explore, _sm.State);
            _in.Present = false;
            Tick();
            Assert.AreEqual(ExperienceState.Attract, _sm.State);
        }

        [Test]
        public void LeaveEarly_Watch_And_BanzaiWait()
        {
            ReachWatch();
            _in.Present = false;
            Tick();
            Assert.AreEqual(ExperienceState.Attract, _sm.State);

            _in.Present = true;
            ReachWatch();
            _in.PlaybackLoops = 1; Tick();
            Assert.AreEqual(ExperienceState.BanzaiWait, _sm.State);
            _in.Present = false;
            Tick();
            Assert.AreEqual(ExperienceState.Attract, _sm.State);
        }

        // ---- fault ----

        [Test]
        public void Fault_PreemptsAnyState_AndRecovers()
        {
            ReachWatch();
            _in.Fault = true;
            Tick();
            Assert.AreEqual(ExperienceState.Fault, _sm.State);
            _in.Fault = false;
            Tick();
            Assert.AreEqual(ExperienceState.Attract, _sm.State);
        }

        // ---- skips ----

        [Test]
        public void Skips_ReachWatchThroughEveryState()
        {
            _t.skipAttract = true;
            _t.skipCalibrate = true;
            _t.skipExplore = true;
            _t.skipProcessing = true;

            var visited = new System.Collections.Generic.List<ExperienceState>();
            _sm.Changed += (from, to) => visited.Add(to);

            for (int i = 0; i < 6; i++) Tick();
            Assert.AreEqual(ExperienceState.Watch, _sm.State);
            CollectionAssert.AreEqual(
                new[]
                {
                    ExperienceState.Calibrate, ExperienceState.Explore,
                    ExperienceState.Processing, ExperienceState.Watch,
                },
                visited);
        }

        // ---- time multiplier ----

        [Test]
        public void TimeMultiplier_ScalesCalibrateTimeout()
        {
            _t.timeMultiplier = 10f;
            Tick(); // → Calibrate
            for (int i = 0; i < 3; i++) Tick(0.5f); // 1.5s * 10 = 15s > 10s
            Assert.AreEqual(ExperienceState.Explore, _sm.State);
        }
    }
}
