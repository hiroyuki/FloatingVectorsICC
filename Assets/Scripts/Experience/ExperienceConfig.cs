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

        [Tooltip("Floor height in the calibration frame (SpaceBuilder.floorY; " +
                 "measured -0.9 on the current rig).")]
        public float floorY = -0.9f;

        [Header("Timings / dev skips")]
        public ExperienceTimings timings = new ExperienceTimings();

        [Min(1)] public int countdownSeconds = 3;

        [Header("Sensing overrides (pushed into PresenceDetector on enter)")]
        [Min(0f)] public float insetMeters = 0f;
        public float yBandMin = -100f;
        public float yBandMax = 100f;
        [Min(1)] public int occupancyThreshold = 1500;

        [Header("Selection spheres (rebased frame, x offsets on the +X axis)")]
        public float[] sphereXOffsets = { -1f, 0f, 1f };
        [Tooltip("Sphere centre height above the floor (m).")]
        public float sphereHeight = 1.2f;
        [Tooltip("Sphere row Z in the rebased frame (m, + = toward camera 3).")]
        public float sphereZ = 0.9f;
        [Min(0.05f)] public float sphereRadius = 0.18f;
        [Min(0.1f)] public float sphereDwellSeconds = 1f;
        [Tooltip("Miniature sculpture scale above each sphere.")]
        [Range(0.05f, 1f)] public float displayMiniatureScale = 0.25f;

        [Header("Publishing")]
        [Tooltip("Use the dry-run publisher (fake URLs, no network). Phase 7 wires LFKS.")]
        public bool dryRunPublish = true;
        [Min(0f)] public float dryRunDelaySeconds = 1f;
        public QrUrlKind qrUrlKind = QrUrlKind.First;

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
        [TextArea] public string qrCaption = "おうちで　みられるよ！";
        [TextArea] public string crowdText = "じゅうたんのうえは　ひとりだけ　にしてね";
    }
}
