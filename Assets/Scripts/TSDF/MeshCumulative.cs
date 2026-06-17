// Mesh-side cumulative capture (spec §7). The point-cloud counterpart is
// PointCloud.PointCloudCumulative: that one STACKS discrete point-cloud
// snapshots; this one FUSES a moving subject's observations into a single TSDF
// volume and freezes the resulting Marching-Cubes surface. Same user intent
// ("keep the motion over a window"), different artifact — hence the parallel
// PointCloudCumulative / MeshCumulative naming.
//
// Source-agnostic: the integration it drives (TSDFIntegrator) subscribes to BOTH
// live (PointCloudRenderer.OnRawFramesReady) and playback
// (PointCloudRecorder.OnPlaybackRawFrame), so it works identically on a live
// Femto Bolt rig OR on an already-recorded RCSV played back. Nothing here touches
// the record path — recording stays a separate concern.
//
// One run = clear the volume, accumulate with RetainGhost (spec §3.2 A) and the
// per-batch live-follow clear turned OFF for `duration` seconds, then Freeze so
// later frames stop changing the held mesh. Release returns to live-follow.

using UnityEngine;
using UnityEngine.Serialization;

namespace TSDF
{
    [AddComponentMenu("Mesh/Cumulative")]
    [DefaultExecutionOrder(-5)]
    public sealed class MeshCumulative : MonoBehaviour
    {
        public enum CumulativeState { Idle, Accumulating, Frozen }

        [Header("References")]
        [Tooltip("Volume to accumulate into. Auto-located on enable.")]
        public TSDFVolume volume;
        [Tooltip("Integrator that feeds the volume. Auto-located on enable.")]
        public TSDFIntegrator integrator;

        [Header("Cumulative")]
        [Tooltip("Seconds to accumulate before freezing the mesh (spec §7). Wall-clock " +
                 "from Begin; on real-time playback this matches the played span.")]
        [FormerlySerializedAs("captureDuration")]
        [Min(0.1f)] public float duration = 10f;
        [Tooltip("Wipe the volume at start so each run begins empty. Note: starting from " +
                 "live-follow (clearVolumeOnNewBatch=true) flips the volume to " +
                 "single-buffer, which rebuilds + clears it on the next TSDFVolume update " +
                 "regardless — so clearOnStart=false cannot reliably preserve pre-existing " +
                 "content across that transition.")]
        public bool clearOnStart = true;
        [Tooltip("Force RetainGhost accumulation while running so the motion trail is kept " +
                 "instead of averaged away (spec §3.2 A vs B).")]
        public bool forceRetainGhost = true;

        [Header("Triggers")]
        [Tooltip("Begin a run automatically when this component enables (e.g. on Play).")]
        public bool autoStartOnEnable = false;
        [Tooltip("When the window elapses, immediately Begin a fresh run instead of " +
                 "freezing — a rolling 'last duration seconds' loop.")]
        public bool loop = false;
        [FormerlySerializedAs("startKey")]
        public KeyCode beginKey = KeyCode.C;
        [FormerlySerializedAs("stopKey")]
        public KeyCode freezeKey = KeyCode.V;
        [Tooltip("Release the frozen mesh and return the integrator to live-follow. " +
                 "R (not B) so it doesn't clash with TSDFDebugSession's B-mode key.")]
        public KeyCode releaseKey = KeyCode.R;

        public CumulativeState State { get; private set; } = CumulativeState.Idle;
        /// <summary>Seconds elapsed in the current run (0 when idle).</summary>
        public float Elapsed { get; private set; }
        public float Remaining => State == CumulativeState.Accumulating
            ? Mathf.Max(0f, duration - Elapsed) : 0f;

        private double _startTime;

        // Live-follow settings captured on the Idle->Begin transition so Release can
        // restore exactly what the integrator/volume were doing before this run.
        private bool _priorClearOnBatch = true;
        private TSDFVolume.AccumulationMode _priorAccumMode = TSDFVolume.AccumulationMode.StandardWeightedAvg;

        private void OnEnable()
        {
            if (volume == null) volume = FindAnyObjectByType<TSDFVolume>(FindObjectsInactive.Include);
            if (integrator == null) integrator = FindAnyObjectByType<TSDFIntegrator>(FindObjectsInactive.Include);
            if (volume == null || integrator == null)
            {
                Debug.LogError("[MeshCumulative] Need both a TSDFVolume and a TSDFIntegrator in the scene.", this);
                enabled = false;
                return;
            }
            if (autoStartOnEnable) Begin();
        }

        private void OnDisable()
        {
            // If disabled mid-run, freeze now. Update() stops running once the
            // component is disabled, so the duration timer would never fire and the
            // integrator would keep accumulating forever. Freeze closes the gate and
            // holds the mesh.
            if (State == CumulativeState.Accumulating) Freeze();
        }

        /// <summary>
        /// Begin a fresh run: take the integrator into accumulate mode (per-batch
        /// clear OFF, gate OPEN, RetainGhost) and wipe the volume.
        /// </summary>
        [ContextMenu("Begin")]
        public void Begin()
        {
            if (volume == null || integrator == null) return;

            // Remember the live-follow config only when coming from a true idle
            // (live) state, so Release restores live-follow — not the mid-run values
            // that a loop restart or a frozen->Begin would otherwise capture.
            if (State == CumulativeState.Idle)
            {
                _priorClearOnBatch = integrator.clearVolumeOnNewBatch;
                _priorAccumMode = volume.accumulationMode;
            }

            // Accumulate, don't live-follow-clear. This also flips the volume to
            // single-buffer (the integrator drives volume.doubleBuffered from this
            // flag), which is what RetainGhost's read-modify-write needs.
            integrator.clearVolumeOnNewBatch = false;
            integrator.integrationEnabled = true;
            if (forceRetainGhost)
                volume.accumulationMode = TSDFVolume.AccumulationMode.RetainGhost;

            if (clearOnStart) volume.Clear();

            _startTime = Time.timeAsDouble;
            Elapsed = 0f;
            State = CumulativeState.Accumulating;
        }

        /// <summary>
        /// Freeze integration so the accumulated mesh holds. Deliberately does NOT
        /// restore clearVolumeOnNewBatch: flipping it back to true would make the
        /// integrator re-enable double buffering, which rebuilds + clears the volume
        /// and wipes the very mesh we just captured. The frozen gate keeps the held
        /// buffer untouched until the next Begin or Release.
        /// </summary>
        [ContextMenu("Freeze")]
        public void Freeze()
        {
            if (integrator != null) integrator.integrationEnabled = false;
            State = CumulativeState.Frozen;
            Elapsed = duration;
        }

        /// <summary>
        /// Release the held mesh and return the integrator to live-follow. Restores
        /// the clear/accumulation settings saved at Begin and re-opens the gate, so
        /// the volume goes back to tracking the current point cloud. Clears the held
        /// content (live-follow rebuilds the volume anyway).
        /// </summary>
        [ContextMenu("Release")]
        public void Release()
        {
            if (volume == null || integrator == null) { State = CumulativeState.Idle; return; }

            volume.accumulationMode = _priorAccumMode;
            integrator.clearVolumeOnNewBatch = _priorClearOnBatch;
            integrator.integrationEnabled = true;
            volume.Clear();

            Elapsed = 0f;
            State = CumulativeState.Idle;
        }

        private void Update()
        {
            if (Input.GetKeyDown(beginKey)) Begin();
            if (Input.GetKeyDown(freezeKey)) Freeze();
            if (Input.GetKeyDown(releaseKey)) Release();

            if (State != CumulativeState.Accumulating) return;

            Elapsed = (float)(Time.timeAsDouble - _startTime);
            if (Elapsed >= duration)
            {
                if (loop) Begin();
                else Freeze();
            }
        }
    }
}
