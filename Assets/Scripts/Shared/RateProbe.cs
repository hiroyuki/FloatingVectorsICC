// Push-based Hz counters for cadences that are NOT the render frame rate.
//
// Why this exists: the render loop can sit at a comfortable 60fps while the body
// frames that actually drive the sculpture arrive far slower, and nothing on
// screen distinguishes the two. The motion-curve trail advances one history slot
// per pose ingest, so a low ingest rate reads as a stuttering trail even though
// the fps readout looks perfect — and the two show machines (4070 / 5080) run the
// same code, so the only way to find where they diverge is to see both numbers
// side by side on each machine.
//
// Deliberately in Shared with no dependencies: producers live in BodyTracking
// (which references Shared) and the consumer is FpsOverlay (also Shared), so a
// direct reference either way would be circular.

using System.Collections.Generic;
using UnityEngine;

namespace Shared
{
    public static class RateProbe
    {
        /// <summary>Pose ingests into BonePoseHistory — the cadence the motion-curve
        /// trail actually advances at.</summary>
        public const string PoseIngest = "pose";

        /// <summary>Individual body-tracker snapshots, summed over all cameras. With
        /// four hardware-synced cameras this runs ~4x PoseIngest when the workers keep
        /// up; if it collapses toward PoseIngest the snapshots are arriving bunched into
        /// one render frame rather than the workers being slow.</summary>
        public const string BtSnapshot = "bt";

        // Rolling window: count ticks until the window closes, then publish and reset.
        private sealed class Counter
        {
            public int Ticks;
            public float WindowStart;
            public float LastTick;
            public float Hz;
        }

        private const float WindowSeconds = 0.5f;
        // No tick for this long => report 0 rather than a stale rate. Generous enough
        // that a genuinely slow (but alive) tracker still reads as a small number.
        private const float StaleSeconds = 1.5f;

        private static readonly Dictionary<string, Counter> s_counters = new Dictionary<string, Counter>();

        /// <summary>Record one event. Call from the main thread.</summary>
        public static void Tick(string name)
        {
            float now = Time.realtimeSinceStartup;
            if (!s_counters.TryGetValue(name, out var c))
            {
                c = new Counter { WindowStart = now };
                s_counters[name] = c;
            }
            // Restarting after a quiet spell (pause, playback swap, tracker restart)
            // opens a fresh window. Otherwise the first tick back closes a window that
            // spans the whole gap and publishes a near-zero rate — the readout would
            // claim a stall for the next window just as the subsystem came back.
            // Hz goes too: the tick just made the counter non-stale again, so Hz() would
            // otherwise report the pre-gap cadence as current until the fresh window closes.
            if (c.LastTick > 0f && now - c.LastTick > StaleSeconds)
            {
                c.Ticks = 0;
                c.WindowStart = now;
                c.Hz = 0f;
            }
            c.Ticks++;
            c.LastTick = now;

            float span = now - c.WindowStart;
            if (span >= WindowSeconds)
            {
                c.Hz = c.Ticks / span;
                c.Ticks = 0;
                c.WindowStart = now;
            }
        }

        /// <summary>Most recent rate for <paramref name="name"/>, or 0 when it has never
        /// ticked or has gone quiet.</summary>
        public static float Hz(string name)
        {
            if (!s_counters.TryGetValue(name, out var c)) return 0f;
            if (Time.realtimeSinceStartup - c.LastTick > StaleSeconds) return 0f;
            return c.Hz;
        }

        /// <summary>True once <paramref name="name"/> has ever ticked — lets a readout
        /// hide the field entirely on a machine where that subsystem isn't running,
        /// instead of showing a permanent 0.</summary>
        public static bool IsKnown(string name) => s_counters.ContainsKey(name);
    }
}
