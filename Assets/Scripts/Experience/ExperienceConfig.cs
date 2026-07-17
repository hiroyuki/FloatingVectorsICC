// ScriptableObject configuration for the visitor experience (Phase 5,
// Plans/phase5-director-plan.md). Everything the operator tunes lives here:
// timings/skips, visitor-facing texts (hiragana, kids), sphere layout, dry-run
// publishing, sensing thresholds pushed into PresenceDetector on mode enter.
// The LFKS token is NOT here (gitignored local file / env var — Phase 7).

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

        [Header("Selection spheres (rebased frame, x offsets on the +X axis)")]
        public float[] sphereXOffsets = { -1f, 0f, 1f };
        [Tooltip("Sphere centre height above the floor (m).")]
        public float sphereHeight = 1.2f;
        [Tooltip("Sphere row Z in the rebased frame (m, + = toward camera 3).")]
        public float sphereZ = 0.9f;
        [Min(0.05f)] public float sphereRadius = 0.25f; // user-tuned live 2026-07-10
        [Min(0.1f)] public float sphereDwellSeconds = 1f;
        [Tooltip("Miniature sculpture scale above each sphere.")]
        [Range(0.05f, 1f)] public float displayMiniatureScale = 0.25f;

        [Header("Attract playback")]
        [Tooltip("Folder containing recorded take folders for the attract-mode ghost " +
                 "(e.g. D:\\FloatingVectorsICC\\RecordingBase). Empty = attract is text-only.")]
        public string attractRecordingRoot = "";

        [Header("Publishing")]
        [Tooltip("Use the dry-run publisher (fake URLs, no network). Off = real LFKS " +
                 "upload via the pinned StreamingAssets/lfks/upload.ps1.")]
        public bool dryRunPublish = true;
        [Min(0f)] public float dryRunDelaySeconds = 1f;
        public QrUrlKind qrUrlKind = QrUrlKind.First;

        [Tooltip("SHA-256 (hex) of StreamingAssets/lfks/upload.ps1 — the publisher " +
                 "refuses to run a script whose bytes changed. Not a secret.")]
        public string uploadScriptSha256 =
            "46d660b6ae081b648fdbcead21bfeb44419df78410c3e89cb55302c65dfac687";

        [Tooltip("Remote subfolder in the LFKS directory the sculptures land in.")]
        public string lfksRemoteDirectory = "sculptures";

        [Min(5f)]
        [Tooltip("Per-file upload timeout (s); one retry after a failure.")]
        public float publishTimeoutSeconds = 60f;

        [Header("Visitor texts (hiragana)")]
        [TextArea] public string attractText = "あそびに　きてね！";
        [TextArea] public string welcomeText = "ようこそ！　からだを　うごかしてみてね";
        [TextArea] public string freePlayText = "すきに　うごいてみよう！";
        [TextArea] public string readyText = "これから　3かい　しゃしんを　とるよ！";
        [TextArea] public string promptAnimalText = "すきな　どうぶつの　まねを　してみて！";
        [TextArea] public string[] promptMantisVariants =
            { "カマキリの　ポーズ！", "つよそうな　ポーズを　してみて！" };
        [TextArea] public string[] promptFreeVariants =
            { "すきな　ポーズで　きめて！", "さいごは　じゆうに　うごいてみて！" };
        [TextArea] public string selectText = "すきな　かたちを　えらんで　てを　いれてね";
        [TextArea] public string exportingText = "いま　じゅんびしているよ　まってね";
        [TextArea] public string exportFailedText = "うまくいかなかったみたい　ごめんね";
        [TextArea]
        [Tooltip("Optional caption under the QR. Empty = QR only (the in-Unity QR is " +
                 "interim — the final URL presentation moves to a separate machine via " +
                 "an IUrlPresenter swap).")]
        public string qrCaption = "";
        [TextArea] public string crowdText = "じゅうたんのうえは　ひとりだけ　にしてね";
    }
}
