// Orchestrates an evaluation run: drives the RCSV replay, fans each frame out to
// every registered tracker adapter, measures per-adapter latency, and feeds the
// normalized skeletons to the metrics collector. Tracks (Nuitrack / RTMPose)
// register their adapter via Register() from their own branch; the k4abt
// baseline is registered here by default.

using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using Orbbec;
using PointCloud;
using UnityEngine;
using Debug = UnityEngine.Debug;

namespace BodyTracking.Eval
{
    [RequireComponent(typeof(EvalReplayDriver))]
    public sealed class EvalRunner : MonoBehaviour
    {
        [Tooltip("Register the recorded-k4abt baseline adapter automatically.")]
        public bool includeBaseline = true;

        [Tooltip("Directory for CSV output. Empty -> <project>/eval/results.")]
        public string resultsDir = "";

        [Header("Metrics window (jitter)")]
        public float staticStartSec = 0f;
        public float staticEndSec = 0f;      // <= start => whole recording
        public float spikeThresholdMm = 50f;

        public bool IsRunning { get; private set; }

        private EvalReplayDriver _driver;
        private readonly List<ITrackerAdapter> _adapters = new List<ITrackerAdapter>();
        private EvalMetrics _metrics;
        private readonly Dictionary<ulong, long> _submitTicks = new Dictionary<ulong, long>();
        private readonly HashSet<string> _configured = new HashSet<string>();

        private void Awake()
        {
            _driver = GetComponent<EvalReplayDriver>();
            if (includeBaseline) Register(new K4abtBaselineAdapter());
        }

        private void OnEnable()
        {
            _driver.OnFrame += HandleFrame;
            _driver.OnRecordedBodies += HandleRecordedBodies;
            _driver.OnLoaded += HandleLoaded;
            _driver.OnLoopComplete += HandleLoopComplete;
        }

        private void OnDisable()
        {
            _driver.OnFrame -= HandleFrame;
            _driver.OnRecordedBodies -= HandleRecordedBodies;
            _driver.OnLoaded -= HandleLoaded;
            _driver.OnLoopComplete -= HandleLoopComplete;
        }

        private void OnDestroy()
        {
            foreach (var a in _adapters) a.Dispose();
            _adapters.Clear();
        }

        /// <summary>Add a tracker adapter. Its OnSkeletons feeds metrics + latency.</summary>
        public void Register(ITrackerAdapter adapter)
        {
            if (adapter == null || _adapters.Contains(adapter)) return;
            _adapters.Add(adapter);
            adapter.OnSkeletons += frame => HandleSkeletons(adapter, frame);
        }

        public IReadOnlyList<ITrackerAdapter> Adapters => _adapters;
        public EvalMetrics Metrics => _metrics;

        // ------------------------------------------------------------------

        public bool LoadAndRun(string sessionRoot)
        {
            if (!_driver.Load(sessionRoot)) return false;
            StartRun();
            return true;
        }

        public void StartRun()
        {
            _metrics = new EvalMetrics();
            _metrics.Configure(new EvalMetrics.Config
            {
                StaticStartSec = staticStartSec,
                StaticEndSec = staticEndSec,
                SpikeThresholdMm = spikeThresholdMm,
            });
            _submitTicks.Clear();
            ConfigureAdaptersForLoadedDevices();
            _driver.ResetPlayhead();
            _driver.Play();
            IsRunning = true;
            Debug.Log($"[EvalRunner] run started with {_adapters.Count} adapter(s)");
        }

        public void StopRun()
        {
            _driver.Pause();
            IsRunning = false;
        }

        /// <summary>Compute + write CSVs, log the summary.</summary>
        public string Finish(bool writeCsv = true)
        {
            StopRun();
            if (_metrics == null) return "(no run)";
            string summary = _metrics.BuildSummary();
            if (writeCsv)
            {
                string dir = ResolveResultsDir();
                _metrics.WriteCsv(dir);
                Debug.Log($"[EvalRunner] wrote metrics CSV to {dir}");
            }
            Debug.Log("[EvalRunner] summary:\n" + summary);
            return summary;
        }

        private void LateUpdate()
        {
            // Drain async adapter results after the driver has emitted this frame.
            for (int i = 0; i < _adapters.Count; i++) _adapters[i].Pump();
        }

        // ------------------------------------------------------------------

        private void HandleLoaded() => ConfigureAdaptersForLoadedDevices();

        private void ConfigureAdaptersForLoadedDevices()
        {
            foreach (var dev in _driver.Devices)
            {
                if (!_configured.Add(dev.Serial)) continue;
                var ctx = new EvalCameraContext(dev.Serial, dev.DepthW, dev.DepthH, dev.ColorW, dev.ColorH, dev.CameraParam);
                foreach (var a in _adapters) a.Configure(in ctx);
            }
        }

        private void HandleFrame(string serial, RawFrameData frame, ObCameraParam? cam, ulong tsNs)
        {
            if (!IsRunning) return;
            _submitTicks[tsNs] = Stopwatch.GetTimestamp();
            for (int i = 0; i < _adapters.Count; i++)
            {
                _metrics.AddSubmitted(_adapters[i].Name, serial);
                _adapters[i].SubmitFrame(serial, in frame, tsNs);
            }
        }

        private void HandleRecordedBodies(string serial, byte[] payload, int byteCount, ulong tsNs)
        {
            if (!IsRunning) return;
            for (int i = 0; i < _adapters.Count; i++)
                _adapters[i].SubmitRecordedBodies(serial, payload, byteCount, tsNs);
        }

        private void HandleSkeletons(ITrackerAdapter adapter, EvalSkeletonFrame frame)
        {
            if (_metrics == null) return;
            if (_submitTicks.TryGetValue(frame.TimestampNs, out long t0))
            {
                double ms = (Stopwatch.GetTimestamp() - t0) * 1000.0 / Stopwatch.Frequency;
                _metrics.AddLatency(adapter.Name, frame.Serial, ms);
            }
            _metrics.AddFrame(adapter.Name, frame);
        }

        private void HandleLoopComplete()
        {
            // Timestamps recur on the next pass — drop the latency bookkeeping so it
            // doesn't grow unbounded across a long looping run.
            _submitTicks.Clear();

            // One full pass is enough for a metric run; stop looping and finalize.
            if (IsRunning && !_driver.loop)
                Finish();
        }

        private string ResolveResultsDir()
        {
            if (!string.IsNullOrWhiteSpace(resultsDir)) return resultsDir;
            // <project>/eval/results  (Application.dataPath == <project>/Assets)
            string project = Directory.GetParent(Application.dataPath)?.FullName ?? Application.dataPath;
            return Path.Combine(project, "eval", "results");
        }
    }
}
