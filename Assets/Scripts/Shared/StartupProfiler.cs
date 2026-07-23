// Startup timeline recorder — the mirror of ShutdownProfiler.
//
// Launching the build freezes the window for seconds before anything is drawn:
// device open, extrinsics load and the ORT session construction all run
// synchronously on the main thread, and Unity cannot present a frame until they
// return. This records who spends that time so the freeze can be attributed
// instead of guessed at, exactly as the teardown one did for quitting.
//
// Usage: StartupProfiler.Mark("label") after a suspect step. The clock starts at
// BeforeSceneLoad, so every mark reads as "ms since the process began running
// managed code" — comparable across runs. Each mark is logged immediately, and a
// summary is dumped once the first frame is actually on screen (that instant is
// what the operator experiences as "the app started").
//
// Always on: a handful of marks per process start, never per frame.

using System.Collections.Generic;
using System.Diagnostics;
using System.Text;
using UnityEngine;
using Debug = UnityEngine.Debug;

namespace Shared
{
    public static class StartupProfiler
    {
        private struct Entry
        {
            public string Label;
            public double AtMs;    // since BeforeSceneLoad
            public double DeltaMs; // since the previous mark
        }

        private static readonly List<Entry> s_entries = new List<Entry>(64);
        private static readonly object s_lock = new object();
        private static readonly Stopwatch s_sw = new Stopwatch();
        private static double s_lastMs;
        private static bool s_dumped;

        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.BeforeSceneLoad)]
        private static void Install()
        {
            lock (s_lock)
            {
                s_entries.Clear();
                s_sw.Restart();
                s_lastMs = 0;
                s_dumped = false;
            }
            Mark("BeforeSceneLoad (managed startup begins)");
        }

        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
        private static void AfterSceneLoad()
        {
            Mark("AfterSceneLoad (scene Awake/OnEnable done)");
            // The first frame the player actually presents is the moment the freeze
            // ends for whoever is watching the screen, so that is where the summary
            // belongs. A GameObject is the only way to get a frame callback from here.
            var go = new GameObject("[StartupProfiler]");
            go.AddComponent<FirstFrameWatcher>();
            Object.DontDestroyOnLoad(go);
        }

        /// <summary>Record and log a timeline point.</summary>
        public static void Mark(string label)
        {
            string line;
            lock (s_lock)
            {
                if (!s_sw.IsRunning) s_sw.Restart();
                double at = s_sw.Elapsed.TotalMilliseconds;
                double delta = at - s_lastMs;
                s_entries.Add(new Entry { Label = label, AtMs = at, DeltaMs = delta });
                s_lastMs = at;
                line = $"[StartupProfile] +{delta,8:0.0}ms  @{at,7:0}ms  {label}";
            }
            Debug.Log(line);
        }

        /// <summary>Elapsed ms since managed startup.</summary>
        public static double NowMs
        {
            get { lock (s_lock) { return s_sw.IsRunning ? s_sw.Elapsed.TotalMilliseconds : 0; } }
        }

        public static void Dump(string reason)
        {
            var sb = new StringBuilder();
            lock (s_lock)
            {
                if (s_dumped || s_entries.Count == 0) return;
                s_dumped = true;
                sb.Append("[StartupProfile] SUMMARY (").Append(reason)
                  .Append(") — total ").Append(s_lastMs.ToString("0")).AppendLine("ms");
                for (int i = 0; i < s_entries.Count; i++)
                {
                    var e = s_entries[i];
                    sb.Append("  +").Append(e.DeltaMs.ToString("0.0").PadLeft(8)).Append("ms  @")
                      .Append(e.AtMs.ToString("0").PadLeft(6)).Append("ms  ")
                      .AppendLine(e.Label);
                }
            }
            Debug.Log(sb.ToString());
        }

        // Marks the first few rendered frames individually. The first is when the
        // window stops being blank; the ones after it matter because plenty of
        // first-use cost (shader / compute-kernel compilation, first GPU dispatch,
        // the first camera frames arriving) lands in frame 2 or 3 and is invisible
        // in a single "startup done" number.
        private const int WatchedFrames = 6;

        private sealed class FirstFrameWatcher : MonoBehaviour
        {
            private int _frames;

            private void Update()
            {
                _frames++;
                // Update runs BEFORE the frame is rendered, so frame 1's mark is when
                // the loop first turns, not when anything reached the screen — the
                // first present lands somewhere inside the gap to frame 2's mark.
                if (_frames == 1) Mark("frame 1 Update (nothing drawn yet)");
                else Mark($"frame {_frames} Update");
                if (_frames < WatchedFrames) return;
                Dump("startup complete");
                Destroy(gameObject);
            }
        }
    }
}
