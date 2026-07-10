// Attract-mode ghost playback (Phase 6, Plans/phase6-attract-watchdog-plan.md):
// picks a random recorded take from a root folder, plays it through the
// SensorRecorder, and re-rolls a different take every time the playback loops.
//
// The recorder fires OnPlaybackLooped from inside its own Update — switching
// takes synchronously from that callback would mutate playback state
// re-entrantly, so the handler only sets a pending flag and the switch runs
// from THIS component's Update on the next frame.

using System.Collections.Generic;
using System.IO;
using PointCloud;
using UnityEngine;

namespace Experience
{
    [DisallowMultipleComponent]
    public class AttractPlaybackController : MonoBehaviour
    {
        [Tooltip("Recorder that plays the takes. Auto-resolves when empty.")]
        public SensorRecorder sensorRecorder;

        [Tooltip("Folder containing take folders (e.g. the RecordingBase root). " +
                 "Empty = attract playback disabled.")]
        public string attractRootPath = "";

        public bool IsRunning { get; private set; }

        /// <summary>While false, loop events don't rotate takes (the dev/Mac
        /// fallback keeps the ghost playing through the experience — a mid-
        /// experience StopAndUnload would yank the only sculpture source).</summary>
        public bool RotationEnabled { get; set; } = true;

        private string _currentTake;
        private bool _switchPending;
        private bool _subscribed;

        private void OnEnable()
        {
            if (sensorRecorder == null) sensorRecorder = FindFirstObjectByType<SensorRecorder>();
        }

        private void OnDisable() => Stop();

        private void Update()
        {
            if (!IsRunning || !_switchPending || !RotationEnabled) return;
            _switchPending = false;
            PlayRandomTake(); // deferred out of the recorder's loop callback
        }

        /// <summary>Enumerate takes, pick one at random (avoiding the current
        /// take when possible) and start playing it. No-op with a warning when
        /// the root has no takes.</summary>
        public void PlayRandomTake()
        {
            if (sensorRecorder == null || string.IsNullOrEmpty(attractRootPath)) return;
            var takes = EnumerateTakes();
            if (takes.Count == 0)
            {
                Debug.LogWarning($"[{nameof(AttractPlaybackController)}] no takes under " +
                                 $"\"{attractRootPath}\" — attract stays text-only.", this);
                return;
            }

            string pick = takes[Random.Range(0, takes.Count)];
            if (takes.Count > 1 && pick == _currentTake)
                pick = takes[(takes.IndexOf(pick) + 1) % takes.Count];
            _currentTake = pick;

            if (sensorRecorder.IsPlaying) sensorRecorder.StopAndUnload();
            sensorRecorder.playbackFolderPath = pick;
            sensorRecorder.Load();
            sensorRecorder.TogglePlay();

            if (!_subscribed)
            {
                sensorRecorder.OnPlaybackLooped += HandleLooped;
                _subscribed = true;
            }
            IsRunning = true;
            Debug.Log($"[{nameof(AttractPlaybackController)}] attract take: {Path.GetFileName(pick)}", this);
        }

        /// <summary>Stop playback and tear the session down (attract→visitor).</summary>
        public void Stop()
        {
            if (_subscribed && sensorRecorder != null)
            {
                sensorRecorder.OnPlaybackLooped -= HandleLooped;
                _subscribed = false;
            }
            _switchPending = false;
            if (IsRunning && sensorRecorder != null) sensorRecorder.StopAndUnload();
            IsRunning = false;
        }

        private void HandleLooped()
        {
            // Fired from inside SensorRecorder.Update — defer (see header).
            _switchPending = true;
        }

        // A take folder = contains dataset/<host>/FemtoBolt_*/depth_main (the
        // RCSV layout SensorRecorder writes), or at least a dataset folder.
        private List<string> EnumerateTakes()
        {
            var takes = new List<string>();
            try
            {
                if (!Directory.Exists(attractRootPath)) return takes;
                foreach (var dir in Directory.GetDirectories(attractRootPath))
                {
                    if (Directory.Exists(Path.Combine(dir, "dataset")))
                        takes.Add(dir);
                }
                // The root itself may BE a take (dataset directly inside).
                if (takes.Count == 0 && Directory.Exists(Path.Combine(attractRootPath, "dataset")))
                    takes.Add(attractRootPath);
            }
            catch (System.Exception e)
            {
                Debug.LogWarning($"[{nameof(AttractPlaybackController)}] take enumeration failed: {e.Message}", this);
            }
            return takes;
        }
    }
}
