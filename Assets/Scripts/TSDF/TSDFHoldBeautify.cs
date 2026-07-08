// Runs TSDFVolume.BeautifyFront (one-shot 3x3x3 sdf-median repair — kills the
// RetainGhost multi-cam ribbing / thin-limb fray on the HELD mesh) whenever the
// displayed mesh freezes:
//   - playback pause (SensorRecorder.IsPaused — the spacebar toggle, issue #17)
//   - MeshCumulative freeze (integrationEnabled gate closing)
//   - any future trigger (exhibition timer) via the public RunNow() entry point.
// The repair only touches the front buffer; the next live publish overwrites it,
// so resuming playback returns to the raw live mesh automatically — nothing to undo.

using PointCloud;
using UnityEngine;

namespace TSDF
{
    [AddComponentMenu("TSDF/Hold Beautify")]
    public sealed class TSDFHoldBeautify : MonoBehaviour
    {
        [Tooltip("Volume whose front buffer gets repaired. Auto-located on enable.")]
        public TSDFVolume volume;

        [Tooltip("Median passes per repair (rounded up to even so the ping-pong lands " +
                 "back in the front buffer). 2 fixes the ribbing; 4 is noticeably " +
                 "softer — go higher only if strands survive.")]
        [Range(2, 8)] public int medianPasses = 2;

        [Tooltip("Run automatically when playback pauses (spacebar / SensorRecorder). " +
                 "The exhibition timer can instead call RunNow() directly.")]
        public bool runOnPlaybackPause = true;

        [Tooltip("Run automatically when the integrator's gate closes " +
                 "(integrationEnabled=false). OFF by default: MeshCumulative's " +
                 "stroboscopic interval gate ALSO closes that flag briefly between " +
                 "stamps, which would fire the repair mid-accumulation. Prefer the " +
                 "pause trigger or an explicit RunNow() from the freeze/timer code.")]
        public bool runOnIntegrationFreeze = false;

        private SensorRecorder _recorder;
        private TSDFIntegrator _integrator;
        private bool _wasPaused;
        private bool _wasFrozen;

        private void OnEnable()
        {
            if (volume == null) volume = FindAnyObjectByType<TSDFVolume>(FindObjectsInactive.Include);
            _recorder = FindAnyObjectByType<SensorRecorder>(FindObjectsInactive.Include);
            _integrator = FindAnyObjectByType<TSDFIntegrator>(FindObjectsInactive.Include);
            _wasPaused = _recorder != null && _recorder.IsPaused;
            _wasFrozen = _integrator != null && !_integrator.integrationEnabled;
        }

        /// <summary>Repair the currently displayed mesh now. Public so the exhibition
        /// timer (or any other freeze trigger) can invoke the same one-shot.</summary>
        [ContextMenu("Run Now")]
        public void RunNow()
        {
            if (volume == null) return;
            float t0 = Time.realtimeSinceStartup;
            volume.BeautifyFront(medianPasses);
            Debug.Log($"[TSDFHoldBeautify] BeautifyFront({medianPasses}) dispatched in " +
                      $"{(Time.realtimeSinceStartup - t0) * 1000f:F1} ms (CPU issue time)", this);
        }

        private void Update()
        {
            // Edge-detect the two freeze signals so the repair fires ONCE per hold.
            bool paused = _recorder != null && _recorder.IsPaused;
            if (runOnPlaybackPause && paused && !_wasPaused) RunNow();
            _wasPaused = paused;

            bool frozen = _integrator != null && !_integrator.integrationEnabled;
            if (runOnIntegrationFreeze && frozen && !_wasFrozen) RunNow();
            _wasFrozen = frozen;
        }
    }
}
