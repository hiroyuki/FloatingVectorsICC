// ScriptableObject configuration for the visitor experience. Everything the
// operator tunes lives here: timings/skips, visitor-facing texts (hiragana,
// kids), pose-detection thresholds, v11s conversion, dry-run publishing,
// sensing thresholds pushed into PresenceDetector on mode enter.
// The LFKS token is NOT here (gitignored local file / env var).

using UnityEngine;

namespace Experience
{
    public enum QrUrlKind { Usdz, Glb, First }

    [CreateAssetMenu(fileName = "ExperienceConfig", menuName = "FloatingVectors/Experience Config")]
    public class ExperienceConfig : ScriptableObject
    {
        [Header("Rig")]
        // No rigSerialOrder here on purpose: the rig order is machine-local and
        // comes from calibration/cameras.yaml (PointCloudRecording.ResolveRigSerialOrder,
        // which SensorManager / SensorRecorder / ExperienceSpaceBuilder each call
        // themselves). This asset is git-synced between the "4070" and "5080" sets,
        // so any serial stored here would be the WRONG set's on one of the two
        // machines — and pushing it on mode enter would overwrite the locally
        // correct order with it.

        [Tooltip("Floor height in the REBASED world. With rebaseFloorY set on " +
                 "SensorManager/SensorRecorder (always-on rebase), the floor IS y=0.")]
        public float floorY = 0f;

        [Header("Timings / dev skips")]
        public ExperienceTimings timings = new ExperienceTimings();

        [Min(1)] public int countdownSeconds = 5;

        [Header("Practice rounds (TestMove1 / TestMove2)")]
        [Min(0f)]
        [Tooltip("How long each practice intro message stays on screen before the " +
                 "next one (TestMove1 shows two, TestMove2 one) and the countdown " +
                 "begins.")]
        public float testIntroSeconds = 4f;

        [Min(1)]
        [Tooltip("How many times the recorded second plays back after できたよ！ — " +
                 "practice rounds AND the ResultShow presentation. 1 = single slow " +
                 "pass (see presentationPlaybackRate), ending frozen on the final frame.")]
        public int playbackLoops = 1;

        [Range(0.05f, 1f)]
        [Tooltip("Playback speed of the できたよ！ replays (1/3 = slow motion). The " +
                 "Processing play-through keeps 1x — only the presentation slows down.")]
        public float presentationPlaybackRate = 1f / 3f;

        [Min(0f)]
        [Tooltip("Hold the できたよ！text ALONE on the blank stage for this long before " +
                 "the model appears and the replay begins. Used by BOTH ResultShow and " +
                 "each practice round's shoot-end reveal. 0 = model appears with the " +
                 "text. (The pure-black beat BEFORE できたよ！ is resultBlackGapSeconds.)")]
        public float resultRevealDelaySeconds = 0.5f;

        [Min(0f)]
        [Tooltip("Practice rounds / shoot-end reveal only: length of the pure-black beat " +
                 "AFTER the model dissolves away and BEFORE できたよ！ appears (see " +
                 "ExperienceDirector's shoot-end dissolve → black → できたよ！ → reveal " +
                 "sequence). ResultShow uses stateGapSeconds for its equivalent blank " +
                 "beat. 0 = できたよ！appears the instant the dissolve finishes.")]
        public float resultBlackGapSeconds = 0.5f;

        [Min(0f)]
        [Tooltip("Shoot-end → できたよ！ transition: after 撮影中 the live model FREEZES " +
                 "in place and holds for this long before it dissolves away.")]
        public float shootEndFreezeHoldSeconds = 0.3f;

        [Min(0f)]
        [Tooltip("Shoot-end → できたよ！ transition: seconds the frozen model takes to " +
                 "DISSOLVE to nothing (dither fade of the point cloud + ribbons).")]
        public float shootEndDissolveSeconds = 0.5f;

        [Tooltip("DEBUG (editor only): auto-pause the Editor the instant the shoot-end " +
                 "dissolve begins, so you can step through it one frame at a time (⏭). " +
                 "Leave off for the show.")]
        public bool debugPauseAtShootDissolve;

        [Tooltip("DEBUG (editor only): auto-pause the Editor the instant the finished " +
                 "できたよ！ model is revealed (both the ResultShow result AND each practice " +
                 "round's preview), so the real presentation look — orbit framing, presentation " +
                 "seed/tail values, and the presentationSolidMesh choice — can be studied frozen " +
                 "instead of flashing past. Leave off for the show.")]
        public bool debugPauseAtReveal;

        [Min(0f)]
        [Tooltip("Orbit speed (yaw °/s) during the replays and the QR screen. The dev " +
                 "pause-orbit keeps the camera's own slower value.")]
        public float presentationOrbitYawSpeedDeg = 30f;

        [Min(0f)]
        [Tooltip("ResultShow: the camera stays STILL through the できたよ！replay and only " +
                 "starts orbiting once the replay ends, easing its yaw speed 0→full over " +
                 "this many seconds (SmoothStep). 0 = snap to full speed immediately.")]
        public float presentationOrbitEaseInSeconds = 1.5f;

        [Min(0f)]
        [Tooltip("Vertical bob amplitude (m) of the presentation orbit.")]
        public float presentationOrbitBobMeters = 0.4f;

        [Min(0f)]
        [Tooltip("できたよ！ result: after the replay freezes on the final decisive frame, the " +
                 "camera orbits the held sculpture for this long. ResultShow then advances to " +
                 "the QR (the orbit keeps running under it); each practice round hands back to " +
                 "the live body afterwards. Applies to BOTH the practice replays and ResultShow.")]
        public float presentationOrbitSeconds = 15f;

        [Header("Playback presentation look (できたよ！ replay → QrShow)")]
        // The LIVE phases use the scene's own (lighter) values — tuned so the
        // point cloud stays readable next to the curves. The replay
        // presentation switches to this richer look and the director restores
        // the scene values when the stage goes back to live.
        [Min(64)]
        [Tooltip("Curve seed count during the take replays (scene value stays for live).")]
        public int presentationSeedCount = 60000;
        [Range(0f, 1f)]
        [Tooltip("Curve tail alpha during the take replays.")]
        public float presentationTailAlpha = 1f;
        [Range(0f, 100f)]
        [Tooltip("Point-cloud decimation (%) during the take replays.")]
        public float presentationDecimatePercent = 7.9f;

        [Tooltip("Show the SOLID shaded TSDF mesh (with self-shadow) as part of the finished " +
                 "できたよ！ model, alongside the ribbons. OFF (default) = the current look: the " +
                 "mesh stays suppressed and ONLY the curves form the final model (the mesh is " +
                 "still shown as the white point cloud during the reveal replay either way). ON = the " +
                 "solid body sits inside the ribbons. A/B this to decide the final look — it only " +
                 "affects ResultShow/QrShow; every live/practice phase is unchanged.")]
        public bool presentationSolidMesh = false;

        [Tooltip("Processing screen: play the TSDF as a looping white point cloud (mesh forming " +
                 "with the motion) behind the progress bar, instead of a black stage. The " +
                 "mesh loops through the v11s conversion wait too. Turn OFF on machines " +
                 "where the wait is GPU-bound (Windows v11s/ONNX) and the extra render " +
                 "competes — Processing then falls back to the plain progress bar on black. " +
                 "The export is unaffected either way (a clean capture pass runs regardless).")]
        [UnityEngine.Serialization.FormerlySerializedAs("processingWireframe")]
        public bool processingWhitePointCloud = true;

        [Min(0f)]
        [Tooltip("processingWhitePointCloud: seconds to LOOP the recorded take as the white point cloud " +
                 "AFTER the v11s conversion finishes (the white point cloud runs post-conversion " +
                 "so the playback file handle can't block the converter). Long takes loop " +
                 "only their last processingWhitePointCloudMinSeconds. Adds this long to Processing.")]
        [UnityEngine.Serialization.FormerlySerializedAs("processingWireframeMinSeconds")]
        public float processingWhitePointCloudMinSeconds = 5f;

        [Header("Sensing overrides (pushed into PresenceDetector on enter)")]
        [Min(0f)] public float insetMeters = 0f;
        [Tooltip("Ignore points below this world height (m). 0.15 skips the floor " +
                 "surface itself — measured live: empty-area occupancy 2358 -> 140.")]
        public float yBandMin = 0.15f;
        public float yBandMax = 100f;
        [Min(1)] public int occupancyThreshold = 1500;

        [Header("Camera auto-recovery (Fault only)")]
        [Tooltip("While the show sits in Fault because a camera stopped streaming, " +
                 "tear the live rig down and bring it straight back up instead of " +
                 "waiting for an operator. Docs/camera-dropout-investigation.md pins " +
                 "the dropout on a start-up race, not a USB fault: every time a " +
                 "DIFFERENT camera drops, the OS logs no USB error, and re-entering " +
                 "Play fixes it — so a re-init is the real remedy. Only ever runs in " +
                 "Fault, where the run has already been aborted, so it can never " +
                 "interrupt a visitor mid-experience.")]
        public bool autoRecoverCameras = true;

        [Min(0.5f)]
        [Tooltip("Pause between destroying the renderers and re-opening them, so the " +
                 "OrbbecSDK pipelines finish releasing the USB devices. Re-opening " +
                 "before the previous session let go is the suspected cause of the " +
                 "dropout in the first place — do not shorten this without testing.")]
        public float cameraReleaseSeconds = 3f;

        [Min(5f)]
        [Tooltip("How long to keep WAITING for the re-opened rig to report healthy " +
                 "before calling the attempt a failure. Polled, so a fast recovery " +
                 "returns immediately — this is the ceiling, not a fixed delay. " +
                 "Measured on the rig: StartLive returns as soon as the renderers " +
                 "exist, but frames only start flowing ~10 s later, so a ceiling under " +
                 "that tears the cameras down mid-startup and can never succeed.")]
        public float cameraSettleSeconds = 25f;

        [Min(1)]
        [Tooltip("Re-init attempts before giving up and leaving the red alert for the " +
                 "operator.")]
        public int cameraRecoverAttempts = 3;

        [Header("Visitor take (Shoot recording)")]
        [Tooltip("Root folder visitor takes are recorded under (each take gets a " +
                 "timestamped subfolder). Keep on a fast disk.")]
        public string visitorRecordingRoot = "";

        [Range(0.5f, 2f)]
        [Tooltip("The ONE second: how long after the countdown hits zero the movement " +
                 "keeps recording — this window becomes the final curved line " +
                 "(clamped to the 32-frame pose ring ≈ 1.06s at 30Hz).")]
        public float captureSeconds = 1.0f;

        [Min(0f)]
        [Tooltip("How long the cue text (これから うごきを さつえいするよ) shows " +
                 "before the countdown starts.")]
        public float shootCueSeconds = 2.5f;

        [Range(0f, 1f)]
        [Tooltip("How long AFTER the countdown zero the captured second begins. The " +
                 "visitor starts moving when they hear the shutter, and a person takes " +
                 "0.2-0.3 s to react — with no offset the front of the window is still " +
                 "them standing still and the tail of the movement falls outside it. " +
                 "The recording is extended by the same amount, so the kept second is " +
                 "still exactly captureSeconds long; only where it sits moves.")]
        public float captureStartOffsetSeconds = 0.25f;

        [Range(0f, 1f)]
        [Tooltip("How far BEFORE the countdown zero the recorder starts — just enough " +
                 "that opening the four RCSV writers does not eat the first frames of " +
                 "the capture window. 0 = start exactly at zero. The countdown itself " +
                 "is deliberately NOT recorded: every recorded frame is a frame the " +
                 "v11s conversion pays full RTMPose inference on, and only the capture " +
                 "window survives into the sculpture.")]
        public float recordPreRollSeconds = 0.3f;

        [Tooltip("Dev: pre-recorded take used when timings.skipShoot or " +
                 "timings.dummyShoot is on (no cameras needed). Point at a full " +
                 "RCSV take root. Read through DevCannedTakeRoot, never directly — " +
                 "on macOS devCannedTakeRootMacOverride wins.")]
        public string devCannedTakeRoot =
            @"D:\Dropbox\projects\ICC\Recordings\RecordingBase\2026-07-14_15-50-24";

        [Tooltip("Optional override used only when running on macOS — this asset is " +
                 "git-tracked and shared between the Windows rigs and a Mac dev " +
                 "machine, so a single field would always be wrong on one of them " +
                 "(same rationale as SensorRecorder.folderPathMacOverride). Empty = " +
                 "use devCannedTakeRoot on every platform.")]
        public string devCannedTakeRootMacOverride = "";

        /// <summary>Platform-resolved <see cref="devCannedTakeRoot"/>. Always read
        /// the canned take through this — the raw field holds the Windows path and
        /// is meaningless on a Mac.</summary>
        public string DevCannedTakeRoot
        {
            get
            {
#if UNITY_EDITOR_OSX || UNITY_STANDALONE_OSX
                if (!string.IsNullOrWhiteSpace(devCannedTakeRootMacOverride))
                    return devCannedTakeRootMacOverride;
#endif
                return devCannedTakeRoot;
            }
        }

        [Tooltip("Dev: number keys (top row / keypad) jump the show — 0=Idle(頭出し/" +
                 "run reset) 1=Consent 2=Welcome 3=Calibrate 4=TestMove1 5=TestMove2 " +
                 "6=Shoot 7=Processing 8=ResultShow 9=QrShow. Works with the mode off too " +
                 "(the jump turns it on). TSDFDebugSession's per-camera views " +
                 "moved to Q/W/E/R (all: A) to free the digits. Forward jumps keep " +
                 "the run's artifacts (take, capture) so each screen can be checked " +
                 "in isolation. Turn OFF for production if stray keys are a risk.")]
        public bool devStateJumpHotkeys = true;

        [Tooltip("Dev (rig-less): keep the entrance recording playing whenever the " +
                 "recorder is free — state jumps and run resets stop/unload it, and " +
                 "without cameras the stage (and presence) would stay empty. Loops " +
                 "the take. Never touches the recorder while live renderers exist, " +
                 "during Processing, or while the visitor take is loaded. Turn OFF " +
                 "to test 退場 (leaving).")]
        public bool devLoopEntrancePlayback = true;

        [Min(0f)]
        [Tooltip("Dev: a canned take longer than this plays only its last N seconds " +
                 "during Processing. A real visitor take is capture-window-sized " +
                 "(~2 s), so this only affects the skipShoot/dummyShoot dev loop, " +
                 "where a full-length recording would stretch Processing by the " +
                 "whole take. 0 = always play the canned take in full.")]
        public float devCannedTakeTailSeconds = 6f;

        [Tooltip("Dev: allow the Processing state to run the v11s conversion on the " +
                 "CANNED take (skipShoot/dummyShoot). The conversion rewrites " +
                 "bodies_main IN PLACE — leave this OFF unless devCannedTakeRoot " +
                 "points at a disposable copy, or a canonical recording gets mutated.")]
        public bool allowCannedTakeConversion = false;

        [Range(0, 16)]
        [Tooltip("SensorRecorder.playbackRenderDelayFrames during visitor playback — " +
                 "delays the rendered point cloud to line up with the fused skeleton " +
                 "(~130ms fusion latency ≈ 4 frames).")]
        public int playbackRenderDelayFrames = 4;

        [Tooltip("Live preview (Calibrate / TestMove / Shoot): delay the DISPLAYED " +
                 "point cloud to the fused skeleton's own timestamp so the ribbons " +
                 "stop thinning out on fast limbs. Display-only — BT and the " +
                 "recording still get the newest frame. Cost: the preview becomes a " +
                 "~150ms-lagged mirror, so this is a look trade, not a fix.")]
        public bool liveRenderSync = false;

        [Header("Cloud reveal (はかれたよ！ → the point cloud rises from the feet)")]
        [Min(0f)]
        [Tooltip("How long the visitor's point cloud takes to rise from the floor to " +
                 "full height when calibration finishes. 0 = pop in instantly.")]
        public float cloudRevealSeconds = 1f;

        [Min(0.1f)]
        [Tooltip("Height (m above floorY) the reveal sweeps up to before the clip " +
                 "switches off. Keep above head height with arms raised, or the top " +
                 "pops in at the end of the sweep.")]
        public float cloudRevealHeight = 2.6f;

        [Min(0f)]
        [Tooltip("After the cloud reveal sweep lands and the bones hand over, the " +
                 "motion-line ribbons grow from nothing to their full trail length " +
                 "over this many seconds. Held at zero during the sweep so the " +
                 "rising-cloud wipe isn't hidden behind full-length lines. " +
                 "0 = snap to full length the instant the sweep finishes.")]
        public float curveGrowSeconds = 1f;

        [Header("Pose detection")]
        [Min(0f)] public float starHoldSeconds = 0.5f;

        // The star pose is judged as four independent limb angles — see
        // PoseClassifiers.IsStarPose. Nothing here is in metres or relative to the
        // shoulder axis, so the same numbers fit a child and an adult, facing
        // any direction.
        [Range(45f, 180f)]
        [Tooltip("Max angle of each shoulder→wrist line from +Y. 0 = straight up " +
                 "(banzai), 90 = out to the side or forward, 180 = hanging at the " +
                 "side. Only the hanging case is rejected — every other direction " +
                 "holds the arm clear of the torso, which is all the measurement " +
                 "needs.")]
        public float starArmAngleFromUpMaxDeg = 135f;
        [Range(0f, 60f)]
        [Tooltip("Min angle of each pelvis→ankle line from −Y (0 = standing " +
                 "straight, larger = feet further apart).")]
        public float starLegAngleFromDownMinDeg = 12f;
        [Min(0f)]
        [Tooltip("Pose-hold dropout forgiveness (s) — BT confidence flickers.")]
        public float poseHoldDropoutSeconds = 0.2f;
        [Min(1)]
        [Tooltip("Per-bone minimum sample count for the per-visitor bone profile " +
                 "measured during the star-pose hold (per-camera raw skeletons; " +
                 "4 cams × hold ≈ dozens of samples per bone). Fewer → the default " +
                 "profile stays in effect.")]
        public int profileMinSamplesPerBone = 12;

        [Header("v11s conversion (Processing state)")]
        [Tooltip("Folder with yolox-m/ and rtmpose-m/ ONNX models, project-root relative.")]
        public string conversionModelsDir = "eval/models";
        [Tooltip("Bone-length profile JSON for the fusion (NOT the visitor metrics). " +
                 "Empty = fusion runs without bone-length priors.")]
        public string conversionBodyProfilePath = "eval/body_profile.json";
        [Tooltip("ONNX Runtime execution provider; falls back downward automatically " +
                 "(Cuda→DirectML→Cpu).")]
        public BodyTracking.Eval.Rtmpose.OrtProvider conversionProvider =
            BodyTracking.Eval.Rtmpose.OrtProvider.Cuda;
        [Range(0f, 1f)] public float conversionConfThreshold = 0.3f;
        [Tooltip("Run the catch-up smoothing post-pass (the \"s\" in v11s).")]
        public bool runCatchupSmooth = true;

        // One clip per visitor-facing MESSAGE (文言), played ONCE the moment that
        // message first appears — mid-state message changes (TestMove1's second
        // intro, さつえいちゅう！, failures, the crowd alert) each have their own
        // slot, not just state entries. Idempotent repaints (crowd-notice clear)
        // never replay. null = silent. Idle is deliberately silent (unattended).
        // The countdown tick / shutter below are sub-state event cues, not
        // messages.
        [Header("Audio — per-message cue (optional; null = silent)")]
        [Tooltip("Privacy consent screen (consentText).")]
        public AudioClip consentSe;
        [Tooltip("Greeting screen (welcomeText).")]
        public AudioClip welcomeSe;
        [Tooltip("Pose-guide screen (calibrateText / the star silhouette).")]
        public AudioClip calibrateSe;
        [Tooltip("Star pose matched — はかれたよ！ (calibrateMatchedText).")]
        public AudioClip poseMatchedSe;
        [UnityEngine.Serialization.FormerlySerializedAs("testMoveSe")]
        [UnityEngine.Serialization.FormerlySerializedAs("freeMoveSe")]
        [Tooltip("TestMove1 first intro (testMove1IntroText).")]
        public AudioClip testMove1IntroSe;
        [Tooltip("TestMove1 second intro (testMove1Intro2Text).")]
        public AudioClip testMove1Intro2Se;
        [Tooltip("TestMove2 intro (testMove2IntroText).")]
        public AudioClip testMove2IntroSe;
        [Tooltip("Shoot cue screen (shootCueText), before the countdown.")]
        public AudioClip shootCueSe;
        [Tooltip("さつえいちゅう！ (shootingText), alongside the shutter.")]
        public AudioClip shootingSe;
        [Tooltip("Processing / progress screen (processingText).")]
        public AudioClip processingSe;
        [Tooltip("できたよ！ (resultText) — the real result AND the practice replays.")]
        public AudioClip resultSe;
        [Tooltip("QR screen (qrScanText).")]
        public AudioClip qrShowSe;
        [Tooltip("Processing/export/publish failure (exportFailedText).")]
        public AudioClip exportFailedSe;
        [Tooltip("Crowd alert (crowdText), on each appearance.")]
        public AudioClip crowdSe;

        [Header("Audio — countdown / shutter (event cues)")]
        [Tooltip("Played on each countdown digit during Shoot.")]
        public AudioClip countdownTickSe;
        [Tooltip("Played at countdown zero (shutter) and again when the second ends.")]
        public AudioClip recordEndSe;

        [Header("Pose guide artwork")]
        [Tooltip("Star-pose guide image. Empty = programmatic silhouette " +
                 "(StickFigureTexture.DrawStarPose).")]
        public Texture2D poseGuideTexture;

        [Header("Publishing")]
        [Tooltip("Use the dry-run publisher (fake URLs, no network). Off = real LFKS " +
                 "upload via the pinned StreamingAssets/lfks/upload.ps1. Windows only — " +
                 "macOS ignores this and always dry-runs (see DryRunPublish). Read " +
                 "through DryRunPublish, never directly.")]
        public bool dryRunPublish = true;

        /// <summary>Platform-resolved <see cref="dryRunPublish"/>. The real publisher
        /// shells out to <c>powershell.exe</c> (LfksUploadPublisher), which does not
        /// exist on a Mac, so macOS is pinned to the dry run no matter how the
        /// git-shared asset is set on the show rigs.</summary>
        public bool DryRunPublish
        {
            get
            {
#if UNITY_EDITOR_OSX || UNITY_STANDALONE_OSX
                return true;
#else
                return dryRunPublish;
#endif
            }
        }
        [Min(0f)] public float dryRunDelaySeconds = 1f;
        public QrUrlKind qrUrlKind = QrUrlKind.First;

        [Tooltip("SHA-256 (hex) of StreamingAssets/lfks/upload.ps1 — the publisher " +
                 "refuses to run a script whose bytes changed. Not a secret.")]
        public string uploadScriptSha256 =
            "46d660b6ae081b648fdbcead21bfeb44419df78410c3e89cb55302c65dfac687";

        [Tooltip("Remote subfolder in the LFKS directory the sculptures land in.")]
        public string lfksRemoteDirectory = "sculptures";

        [Tooltip("LFKS API origin override. Empty = the origin baked into the pinned " +
                 "upload.ps1 (https://ntticc.lfks.app, production). A token is issued " +
                 "PER ORIGIN: presenting a staging token to production fails with a " +
                 "misleading \"invalid or expired token\" 404, so this must match " +
                 "whichever deployment lfks-token.txt belongs to. Only origins on " +
                 "LfksUploadPublisher.AllowedApiOrigins are accepted — the token is " +
                 "never sent anywhere else. Example: " +
                 "https://lfks-staging.circuit-lab.workers.dev")]
        public string lfksApiUrl = "";

        [Min(5f)]
        [Tooltip("Per-file upload timeout (s); one retry after a failure.")]
        public float publishTimeoutSeconds = 60f;

        [Header("Visitor texts (hiragana)")]
        [TextArea] public string exportFailedText = "うまくいかなかったみたい　ごめんね";
        [TextArea]
        [Tooltip("Optional caption under the QR. Empty = QR only (the in-Unity QR is " +
                 "interim — the final URL presentation moves to a separate machine via " +
                 "an IUrlPresenter swap).")]
        public string qrCaption = "";
        [TextArea] public string crowdText = "じゅうたんのうえは　ひとりだけ　にしてね";

        [Header("Visitor texts — sequence (hiragana)")]
        [TextArea]
        [Tooltip("Privacy consent shown FIRST on entry (Consent state). A visitor who " +
                 "does not want their sculpture published leaves the carpet here.")]
        public string consentText =
            "このさくひんは　インターネットで　みられるように　なります。\n" +
            "それが　いやなひとは　たいけんが　できないので　そとに　でてね。";
        [TextArea] public string welcomeText = "ようこそ、からだをつかって　おもしろいかたちが　つくれるよ";
        [TextArea] public string calibrateText = "この　ポーズを　とってね";
        [TextArea] public string calibrateMatchedText = "はかれたよ！";
        [Header("Visitor texts — practice rounds (hiragana)")]
        [TextArea]
        [Tooltip("TestMove1 first intro message.")]
        public string testMove1IntroText = "じぶんの　うごきを\nとるれんしゅうを　しよう";
        [TextArea]
        [Tooltip("TestMove1 second intro message (how the take works).")]
        public string testMove1Intro2Text = "５びょうたったら\n１びょうかん　とるよ\nたくさん　うごいてね";
        [TextArea]
        [Tooltip("TestMove2 single intro message.")]
        public string testMove2IntroText = "どうだった？\nもういちど　れんしゅうするよ";
        [Header("Visitor texts — the real take (hiragana)")]
        [TextArea] public string shootCueText = "じゃぁ　ほんばんだよ";
        [TextArea] public string shootingText = "さつえいちゅう！";
        [TextArea] public string processingText = "きろくを　じゅんびしているよ　まってね";
        [Tooltip("Shown over the looping white point cloud during the practice (TestMove) analysis " +
                 "phase. The real-take Processing screen uses processingText instead.")]
        [TextArea] public string analyzingText = "ぶんせきちゅう";
        [TextArea] public string resultText = "できたよ！";
        [TextArea] public string qrScanText = "にじげんコードの　しゃしんを　とったら\nおうちでも　みれるよ";
    }
}
