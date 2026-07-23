// Push-based Hz counters for cadences that are NOT the render frame rate.
//
// Why this exists: the render loop can sit at its frame cap while the body frames
// that actually drive the sculpture arrive far slower, and nothing on screen
// distinguishes the two. The motion-curve trail advances one history slot per pose
// ingest, so a low ingest rate reads as a stuttering trail even though the fps
// readout looks perfect — and the two show machines (4070 / 5080) run the same
// code, so the only way to find where they diverge is to see the numbers side by
// side on each machine.
//
// Name the counters after what actually produces them. The live skeleton source in
// this project is RTMPose (LiveFusedBodySource drives the merger through
// SubmitExternalBodies); k4abt is bypassed entirely in that mode. A counter labelled
// for the wrong engine is worse than no counter — it invites reading a healthy
// RTMPose rate as evidence about k4abt.
//
// Deliberately in Shared with no dependencies: producers live in BodyTracking and
// BodyTracking.Eval.Rtmpose (both reference Shared) and the consumer is FpsOverlay
// (also Shared), so a direct reference either way would be circular.

using System.Collections.Generic;
using UnityEngine;

namespace Shared
{
    public static class RateProbe
    {
        /// <summary>Pose ingests into BonePoseHistory — the cadence the motion-curve
        /// trail actually advances at.</summary>
        public const string PoseIngest = "pose";

        /// <summary>Fused skeletons emitted by the live RTMPose source, held frames
        /// included. Reported (not ticked) — the fusion session already measures its own
        /// rate, and its number is authoritative where a count of merger injections is
        /// not: the fused pose is injected once per camera serial, so counting those
        /// would read 4x the true rate.</summary>
        public const string Fused = "fused";

        /// <summary>Fused skeletons carrying a genuinely NEW inference, i.e. Fused minus
        /// the held/repeated emissions. The gap between the two is the tell: equal means
        /// every emission is fresh, and fresh collapsing while fused holds steady means
        /// the source is coasting on held frames because inference is falling behind.</summary>
        public const string FreshFused = "fresh";

        // A counter is either ticked per event (Tick) or handed a rate measured elsewhere
        // (Report). Ticks/WindowStart stay unused in the reported case; LastTick drives
        // staleness for both.
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

        /// <summary>Publish a rate that the producer already measures itself, instead of
        /// counting events here. Call it every frame while the producer is alive — that
        /// is what keeps the counter out of the stale branch, so reporting a genuine 0
        /// (source running but emitting nothing) stays distinguishable from the source
        /// being gone entirely. Call from the main thread.</summary>
        public static void Report(string name, float hz)
        {
            float now = Time.realtimeSinceStartup;
            if (!s_counters.TryGetValue(name, out var c))
            {
                c = new Counter { WindowStart = now };
                s_counters[name] = c;
            }
            c.Hz = hz;
            c.LastTick = now;
        }

        /// <summary>Most recent rate for <paramref name="name"/>, or 0 when it has never
        /// ticked or has gone quiet.</summary>
        public static float Hz(string name)
        {
            if (!s_counters.TryGetValue(name, out var c)) return 0f;
            if (Time.realtimeSinceStartup - c.LastTick > StaleSeconds) return 0f;
            return c.Hz;
        }

        /// <summary>True once <paramref name="name"/> has ever ticked or been reported —
        /// lets a readout hide the field entirely on a machine where that subsystem isn't
        /// running, instead of showing a permanent 0.</summary>
        public static bool IsKnown(string name) => s_counters.ContainsKey(name);

        /// <summary>True when any counter at all has been seen. A readout uses this to
        /// decide whether the whole rate line is worth drawing, rather than keying it to
        /// one particular producer that a given scene may not contain.</summary>
        public static bool AnyKnown => s_counters.Count > 0;
    }
}
