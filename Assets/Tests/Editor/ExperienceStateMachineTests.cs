// EditMode tests for the experience-flow state machine (Phase 5,
// Plans/phase5-director-plan.md): full happy path, per-prompt capture
// latching, one-frame ExportFailed pulses, leave/fault interruptions, skip
// flags, captureCount variants, timeMultiplier.

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
                welcomeSeconds = 2f,
                freePlaySeconds = 3f,
                readySeconds = 1f,
                promptSeconds = 4f,
                selectTimeoutSeconds = 10f,
                exportFailNoticeSeconds = 2f,
                qrShowSeconds = 5f,
                captureCount = 3,
                timeMultiplier = 1f,
            };
            _in = new ExperienceInputs { Present = true, SelectedIndex = -1 };
        }

        private void Step(float seconds, int steps = 1)
        {
            float dt = seconds / steps;
            for (int i = 0; i < steps; i++) _sm.Tick(dt, in _in, _t);
        }

        private void RunPromptWithCapture()
        {
            // capture completes mid-prompt (one-frame pulse is enough: latched)
            Step(1f);
            _in.CaptureDone = true;
            Step(0.01f);
            _in.CaptureDone = false;
            Step(5f); // past promptSeconds
        }

        [Test]
        public void HappyPath_FullLoop()
        {
            Assert.AreEqual(ExperienceState.Attract, _sm.State);
            Step(0.1f);
            Assert.AreEqual(ExperienceState.Welcome, _sm.State, "present -> welcome");
            Step(2.5f);
            Assert.AreEqual(ExperienceState.FreePlay, _sm.State);
            Step(3.5f);
            Assert.AreEqual(ExperienceState.Ready, _sm.State);
            Step(1.5f);
            Assert.AreEqual(ExperienceState.PromptAnimal, _sm.State);
            RunPromptWithCapture();
            Assert.AreEqual(ExperienceState.PromptMantis, _sm.State);
            RunPromptWithCapture();
            Assert.AreEqual(ExperienceState.PromptFree, _sm.State);
            RunPromptWithCapture();
            Assert.AreEqual(ExperienceState.Select, _sm.State);
            _in.SelectedIndex = 1;
            Step(0.1f);
            Assert.AreEqual(ExperienceState.Exporting, _sm.State);
            _in.SelectedIndex = -1;
            _in.ExportDone = true;
            Step(0.1f);
            Assert.AreEqual(ExperienceState.QrShow, _sm.State);
            _in.ExportDone = false;
            Step(6f);
            Assert.AreEqual(ExperienceState.Attract, _sm.State, "qr timer -> attract");
        }

        [Test]
        public void Prompt_RequiresItsOwnCapture()
        {
            _t.captureCount = 2;
            Step(0.1f); Step(2.5f); Step(3.5f); Step(1.5f); // -> PromptAnimal
            Assert.AreEqual(ExperienceState.PromptAnimal, _sm.State);
            // Director "forgets" to lower CaptureDone: latch must still reset.
            _in.CaptureDone = true;
            Step(5f);
            Assert.AreEqual(ExperienceState.PromptMantis, _sm.State);
            _in.CaptureDone = false; // pulse ended before the second prompt saw it? No —
            // it stayed true through the transition tick; the latch cleared on enter,
            // and CaptureDone was still true on the NEXT tick, so it re-latches. Drop
            // it BEFORE any Mantis tick to prove Mantis needs its own capture:
            Step(5f);
            Assert.AreEqual(ExperienceState.PromptMantis, _sm.State,
                "prompt must not advance on the previous prompt's completion");
            _in.CaptureDone = true;
            Step(0.01f);
            _in.CaptureDone = false;
            Step(5f);
            Assert.AreEqual(ExperienceState.Select, _sm.State);
        }

        [Test]
        public void ExportFailed_OneFramePulse_LatchesAndFallsBackToAttract()
        {
            _sm.ResetTo(ExperienceState.Exporting);
            _in.ExportFailed = true;
            Step(0.01f); // one frame
            _in.ExportFailed = false;
            Assert.AreEqual(ExperienceState.Exporting, _sm.State, "stays during the notice");
            Step(1f);
            Assert.AreEqual(ExperienceState.Exporting, _sm.State);
            Step(1.5f);
            Assert.AreEqual(ExperienceState.Attract, _sm.State, "notice elapsed -> attract");
        }

        [Test]
        public void Exporting_IgnoresPresenceLoss()
        {
            _sm.ResetTo(ExperienceState.Exporting);
            _in.Present = false;
            Step(30f, steps: 10);
            Assert.AreEqual(ExperienceState.Exporting, _sm.State);
            _in.ExportDone = true;
            Step(0.1f);
            Assert.AreEqual(ExperienceState.QrShow, _sm.State);
        }

        [Test]
        public void LeavingEarly_ReturnsToAttract_FromEveryEligibleState()
        {
            var leaveStates = new[]
            {
                ExperienceState.Welcome, ExperienceState.FreePlay, ExperienceState.Ready,
                ExperienceState.PromptAnimal, ExperienceState.PromptMantis,
                ExperienceState.PromptFree, ExperienceState.Select, ExperienceState.QrShow,
            };
            foreach (var s in leaveStates)
            {
                _sm.ResetTo(s);
                _in.Present = false;
                Step(0.1f);
                Assert.AreEqual(ExperienceState.Attract, _sm.State, $"leave from {s}");
                _in.Present = true;
            }
        }

        [Test]
        public void Fault_PreemptsAndRecovers()
        {
            Step(0.1f); // -> Welcome
            _in.Fault = true;
            Step(0.01f);
            Assert.AreEqual(ExperienceState.Fault, _sm.State);
            Step(60f);
            Assert.AreEqual(ExperienceState.Fault, _sm.State, "stays while fault holds");
            _in.Fault = false;
            Step(0.01f);
            Assert.AreEqual(ExperienceState.Attract, _sm.State);
        }

        [Test]
        public void SkipFlags_EnterThenAdvanceImmediately()
        {
            _t.skipAttract = true;
            _t.skipWelcome = true;
            _t.skipFreePlay = true;
            _t.skipReady = true;
            var visited = new System.Collections.Generic.List<ExperienceState>();
            _sm.Changed += (_, to) => visited.Add(to);
            // skipAttract must advance even with nobody there (one tick)...
            _in.Present = false;
            Step(0.01f);
            Assert.AreEqual(ExperienceState.Welcome, _sm.State, "skipAttract ignores Present");
            // ...but the leave rule still owns the later states, so presence
            // returns for the rest of the skip chain.
            _in.Present = true;
            Step(0.05f, steps: 5);
            CollectionAssert.AreEqual(
                new[] { ExperienceState.Welcome, ExperienceState.FreePlay,
                        ExperienceState.Ready, ExperienceState.PromptAnimal },
                visited.GetRange(0, 4),
                "every skipped state is still entered, in order");
        }

        [Test]
        public void CaptureCount_One_SkipsSecondAndThirdPrompt()
        {
            _t.captureCount = 1;
            _sm.ResetTo(ExperienceState.PromptAnimal);
            RunPromptWithCapture();
            Assert.AreEqual(ExperienceState.Select, _sm.State);
        }

        [Test]
        public void SelectTimeout_FallsBackToAttract()
        {
            _sm.ResetTo(ExperienceState.Select);
            Step(11f, steps: 4);
            Assert.AreEqual(ExperienceState.Attract, _sm.State);
        }

        [Test]
        public void SkipQr_ExportSuccessGoesStraightToAttract()
        {
            _t.skipQr = true;
            _sm.ResetTo(ExperienceState.Exporting);
            _in.ExportDone = true;
            Step(0.1f);
            Assert.AreEqual(ExperienceState.Attract, _sm.State);
        }

        [Test]
        public void TimeMultiplier_ScalesTimers()
        {
            _t.timeMultiplier = 10f;
            _sm.ResetTo(ExperienceState.Welcome);
            Step(0.25f); // 0.25 * 10 = 2.5 scaled > welcomeSeconds(2)
            Assert.AreEqual(ExperienceState.FreePlay, _sm.State);
        }
    }
}
