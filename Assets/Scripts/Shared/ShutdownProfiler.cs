// Shutdown timeline recorder.
//
// Quitting the build (Esc-hold) freezes the window for seconds: every teardown
// step runs synchronously on the main thread. This records who spends that time
// so the freeze can be attributed instead of guessed at.
//
// Usage: ShutdownProfiler.Mark("label") after a suspect step. The first Mark
// starts the clock; every mark is logged IMMEDIATELY (one line) because the
// process may die before any end-of-run summary could flush. A collected
// summary is also dumped on play-mode exit / process exit when we get there.
//
// Always on: this fires a handful of times per process shutdown, never per frame.

using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Text;
using UnityEngine;
using Debug = UnityEngine.Debug;

namespace Shared
{
    public static class ShutdownProfiler
    {
        private struct Entry
        {
            public string Label;
            public double AtMs;    // since the first mark
            public double DeltaMs; // since the previous mark
        }

        private static readonly List<Entry> s_entries = new List<Entry>(64);
        private static readonly object s_lock = new object();
        private static readonly Stopwatch s_sw = new Stopwatch();
        private static double s_lastMs;

        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.BeforeSceneLoad)]
        private static void Install()
        {
            lock (s_lock)
            {
                s_entries.Clear();
                s_sw.Reset();
                s_lastMs = 0;
            }
            Application.quitting += () => Mark("Application.quitting (teardown begins)");
            AppDomain.CurrentDomain.ProcessExit += (_, __) => Dump("ProcessExit");
#if UNITY_EDITOR
            UnityEditor.EditorApplication.playModeStateChanged += state =>
            {
                if (state == UnityEditor.PlayModeStateChange.ExitingPlayMode) Mark("ExitingPlayMode (teardown begins)");
                if (state == UnityEditor.PlayModeStateChange.EnteredEditMode) Dump("EnteredEditMode");
            };
#endif
        }

        /// <summary>Record and log a timeline point. First call starts the clock.</summary>
        public static void Mark(string label)
        {
            string line;
            lock (s_lock)
            {
                if (!s_sw.IsRunning) { s_sw.Start(); s_lastMs = 0; }
                double at = s_sw.Elapsed.TotalMilliseconds;
                double delta = at - s_lastMs;
                s_entries.Add(new Entry { Label = label, AtMs = at, DeltaMs = delta });
                s_lastMs = at;
                line = $"[ShutdownProfile] +{delta,8:0.0}ms  @{at,7:0}ms  {label}";
            }
            Debug.Log(line);
        }

        /// <summary>Elapsed ms on the shared clock (0 before the first mark).</summary>
        public static double NowMs
        {
            get { lock (s_lock) { return s_sw.IsRunning ? s_sw.Elapsed.TotalMilliseconds : 0; } }
        }

        public static void Dump(string reason)
        {
            var sb = new StringBuilder();
            lock (s_lock)
            {
                if (s_entries.Count == 0) return;
                sb.Append("[ShutdownProfile] SUMMARY (").Append(reason)
                  .Append(") — total ").Append(s_lastMs.ToString("0")).AppendLine("ms");
                for (int i = 0; i < s_entries.Count; i++)
                {
                    var e = s_entries[i];
                    sb.Append("  +").Append(e.DeltaMs.ToString("0.0").PadLeft(8)).Append("ms  @")
                      .Append(e.AtMs.ToString("0").PadLeft(6)).Append("ms  ")
                      .AppendLine(e.Label);
                }
                s_entries.Clear();
                s_sw.Reset();
                s_lastMs = 0;
            }
            Debug.Log(sb.ToString());
        }
    }
}
