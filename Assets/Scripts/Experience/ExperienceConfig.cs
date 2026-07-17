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
        [Tooltip("Camera serials in rig order 1..4 (defines +X = camera1→2). Pushed " +
                 "into SensorManager/SensorRecorder on mode enter.")]
        public string[] rigSerialOrder =
            { "CL8F253004N", "CL8F253004L", "CL8F25300EG", "CL8F253004Z" };

        [Tooltip("Floor height in the REBASED world. With rebaseFloorY set on " +
                 "SensorManager/SensorRecorder (always-on rebase), the floor IS y=0.")]
        public float floorY = 0f;

        [Header("Timings / dev skips")]
        public ExperienceTimings timings = new ExperienceTimings();

        [Min(1)] public int countdownSeconds = 3;

        [Header("Sensing overrides (pushed into PresenceDetector on enter)")]
        [Min(0f)] public float insetMeters = 0f;
        [Tooltip("Ignore points below this world height (m). 0.15 skips the floor " +
                 "surface itself — measured live: empty-area occupancy 2358 -> 140.")]
        public float yBandMin = 0.15f;
        public float yBandMax = 100f;
        [Min(1)] public int occupancyThreshold = 1500;

        [Header("Attract playback")]
        [Tooltip("Folder containing recorded take folders for the attract-mode ghost " +
                 "(e.g. D:\\FloatingVectorsICC\\RecordingBase). Empty = attract is text-only.")]
        public string attractRecordingRoot = "";

        [Header("Visitor take (Explore recording)")]
        [Tooltip("Root folder visitor takes are recorded under (each take gets a " +
                 "timestamped subfolder). Keep on a fast disk; separate from the " +
                 "attract root so the ghost never picks up visitor takes.")]
        public string visitorRecordingRoot = "";

        [Tooltip("Dev: pre-recorded take used when timings.skipExplore is on (no " +
                 "cameras needed). Point at a full RCSV take root.")]
        public string devCannedTakeRoot =
            @"D:\Dropbox\projects\ICC\Recordings\RecordingBase\2026-07-14_15-50-24";

        [Tooltip("Dev: allow the Processing state to run the v11s conversion on the " +
                 "CANNED take (skipExplore). The conversion rewrites bodies_main IN " +
                 "PLACE — leave this OFF unless devCannedTakeRoot points at a " +
                 "disposable copy, or a canonical recording gets mutated.")]
        public bool allowCannedTakeConversion = false;

        [Range(0, 16)]
        [Tooltip("SensorRecorder.playbackRenderDelayFrames during visitor playback — " +
                 "delays the rendered point cloud to line up with the fused skeleton " +
                 "(~130ms fusion latency ≈ 4 frames).")]
        public int playbackRenderDelayFrames = 4;

        [Header("Pose detection")]
        [Min(0f)] public float starHoldSeconds = 0.5f;
        [Range(0.5f, 1f)]
        [Tooltip("Min (wrist-shoulder)/(arm path) ratio for a straight arm.")]
        public float starArmStraightnessMin = 0.85f;
        [Range(0.05f, 1f)]
        [Tooltip("Max |wrist.y − shoulder.y| as a fraction of arm length (arms level).")]
        public float starArmLevelFactor = 0.35f;
        [Range(0.1f, 1f)]
        [Tooltip("Min wrist displacement along the shoulder axis as a fraction of arm " +
                 "length (rejects arms-forward).")]
        public float starArmLateralFactor = 0.6f;
        [Range(0.5f, 3f)]
        [Tooltip("Min ankle separation as a multiple of shoulder width (legs spread).")]
        public float starAnkleSpreadFactor = 1.3f;
        [Min(0f)] public float starAnkleSpreadMinMeters = 0.35f;
        [Min(0f)] public float banzaiHoldSeconds = 0.4f;
        [Min(0f)]
        [Tooltip("Absolute floor on the wrists-above-head margin (m).")]
        public float banzaiMarginMeters = 0.05f;
        [Range(0f, 0.5f)]
        [Tooltip("Wrists-above-head margin as a fraction of the measured arm length " +
                 "(personal adaptation from the calibration pose).")]
        public float banzaiMarginArmFraction = 0.15f;
        [Min(0f)]
        [Tooltip("Pose-hold dropout forgiveness (s) — BT confidence flickers.")]
        public float poseHoldDropoutSeconds = 0.2f;

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

        [Header("Playback / capture")]
        [Min(0f)]
        [Tooltip("Wait after the 3-loop-fallback random seek before capturing, so " +
                 "trails/pose history regrow past the seek seam (s).")]
        public float postSeekSettleSeconds = 1.5f;

        [Header("Audio (optional placeholders — null = silent)")]
        public AudioClip startSe;
        public AudioClip countdownTickSe;
        public AudioClip recordEndSe;
        public AudioClip poseMatchedSe;
        public AudioClip banzaiSe;
        public AudioClip qrSe;

        [Header("Pose guide artwork")]
        [Tooltip("Star-pose guide image. Empty = programmatic stick figure " +
                 "(StickFigureTexture.DrawStarPose).")]
        public Texture2D poseGuideTexture;
        [Tooltip("Banzai guide image. Empty = programmatic stick figure.")]
        public Texture2D banzaiGuideTexture;

        [Header("Publishing")]
        [Tooltip("Use the dry-run publisher (fake URLs, no network). Off = real LFKS " +
                 "upload via the pinned StreamingAssets/lfks/upload.ps1.")]
        public bool dryRunPublish = true;
        [Min(0f)] public float dryRunDelaySeconds = 1f;
        public QrUrlKind qrUrlKind = QrUrlKind.First;

        [Tooltip("SHA-256 (hex) of StreamingAssets/lfks/upload.ps1 — the publisher " +
                 "refuses to run a script whose bytes changed. Not a secret.")]
        public string uploadScriptSha256 =
            "2f9e84376551e547198576a65562d1829796cba36d99daf7fd6aac84fafcce96";

        [Tooltip("Remote subfolder in the LFKS directory the sculptures land in.")]
        public string lfksRemoteDirectory = "sculptures";

        [Min(5f)]
        [Tooltip("Per-file upload timeout (s); one retry after a failure.")]
        public float publishTimeoutSeconds = 60f;

        [Header("Visitor texts (hiragana)")]
        [TextArea] public string attractText = "あそびに　きてね！";
        [TextArea] public string exportingText = "いま　じゅんびしているよ　まってね";
        [TextArea] public string exportFailedText = "うまくいかなかったみたい　ごめんね";
        [TextArea]
        [Tooltip("Optional caption under the QR. Empty = QR only (the in-Unity QR is " +
                 "interim — the final URL presentation moves to a separate machine via " +
                 "an IUrlPresenter swap).")]
        public string qrCaption = "";
        [TextArea] public string crowdText = "じゅうたんのうえは　ひとりだけ　にしてね";

        [Header("Visitor texts — new sequence (hiragana)")]
        [TextArea] public string calibrateText = "この　ポーズを　とってね";
        [TextArea] public string calibrateMatchedText = "はかれたよ！";
        [TextArea] public string exploreText = "おもしろい　うごきを　さがしてみよう";
        [TextArea] public string processingText = "きろくを　じゅんびしているよ　まってね";
        [TextArea] public string watchText = "";
        [TextArea] public string banzaiText = "じぶんの　すきな　ところで　ばんざいの　ポーズを　してね";
        [TextArea] public string qrScanText = "いりぐちの　にじげんコードを　スキャンしてね";
    }
}
