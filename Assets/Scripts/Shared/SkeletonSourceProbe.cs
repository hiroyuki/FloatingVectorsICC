// Which engine is actually producing skeletons right now.
//
// Why this exists: the project ships TWO complete skeleton paths — live RTMPose
// (LiveFusedBodySource drives the merger through SubmitExternalBodies) and k4abt
// (K4abtWorkerHost spawns a worker process per camera). Which one runs is decided
// at startup by whether RTMPose managed to load its ONNX models: on success it sets
// SkeletonMerger.useExternalBodies, which stops k4abt workers from ever spawning; on
// failure the flag stays false and k4abt takes over. The only trace is one line in
// the log, so a build silently running the other engine looks completely normal —
// which is exactly how an Editor session measuring RTMPose got compared against a
// build that was running k4abt.
//
// Reported by whichever path is live, so the readout states what IS happening rather
// than re-deriving it from flags. In Shared with no dependencies: producers live in
// BodyTracking and BodyTracking.Eval.Rtmpose (both reference Shared) and the consumer
// is FpsOverlay (also Shared).

using UnityEngine;

namespace Shared
{
    public static class SkeletonSourceProbe
    {
        public const string Rtmpose = "rtmpose";
        public const string K4abt = "k4abt";

        // Long enough that a slow-but-alive source doesn't flicker to "none", short
        // enough that a source going down is visible while the operator is still
        // looking at the screen.
        private const float StaleSeconds = 2f;

        private static string s_name;
        private static float s_lastReport = -1f;

        /// <summary>Report that <paramref name="name"/> produced a skeleton. Call it on
        /// every emission — staleness, not an explicit "stopped" call, is what clears
        /// the readout, so a source that dies without unwinding still shows up as gone.
        /// Main thread only.</summary>
        public static void Report(string name)
        {
            s_name = name;
            s_lastReport = Time.realtimeSinceStartup;
        }

        /// <summary>The engine currently producing skeletons, or null when nothing has
        /// reported recently. Null is meaningful: it means neither path is delivering,
        /// not merely that this readout is uninitialised.</summary>
        public static string Current
        {
            get
            {
                if (s_lastReport < 0f) return null;
                return Time.realtimeSinceStartup - s_lastReport > StaleSeconds ? null : s_name;
            }
        }

        /// <summary>True once any source has ever reported — lets a readout stay absent
        /// on a scene with no body tracking at all, rather than showing a permanent
        /// "none" that reads as a fault.</summary>
        public static bool EverReported => s_lastReport >= 0f;
    }
}
