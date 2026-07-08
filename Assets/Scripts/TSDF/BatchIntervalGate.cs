// Stroboscopic interval gate shared by MeshCumulative and TSDFTrailBaker (3-9 dedup —
// the two carried byte-identical copies of this mechanism, comments included).
//
// Both build a sculpture from DISCRETE poses: after each complete multi-cam batch
// folds, the integrator gate is closed (integrationEnabled = false) so whole frames
// are dropped until the interval elapses, then re-opened. Closing only at the
// integrator's BeforePublishCompleteBatch boundary is what keeps instants unmixed:
// the batch that just completed still publishes, and frames are rejected at
// DispatchIntegrate entry, so no partial instant can ever blend into the volume.
//
// The same clock doubles as the trail-only stamp throttle (IntervalElapsed/MarkNow)
// so trail stamps and body folds share one cadence.

using UnityEngine;

namespace TSDF
{
    /// <summary>
    /// Batch-boundary interval gate for stroboscopic accumulation. Owners call
    /// <see cref="Close"/> from the integrator's BeforePublishCompleteBatch hook and
    /// <see cref="TryReopen"/> from their Update, and use
    /// <see cref="IntervalElapsed"/>/<see cref="MarkNow"/> as a plain stamp throttle.
    /// </summary>
    public sealed class BatchIntervalGate
    {
        private double _lastTime = double.NegativeInfinity;
        private bool _gated;

        /// <summary>True while the integrator gate is held closed by <see cref="Close"/>.</summary>
        public bool IsGated => _gated;

        /// <summary>Has the interval passed since the last <see cref="MarkNow"/>/<see cref="Close"/>?
        /// An interval &lt;= 0 always counts as elapsed (continuous mode / mid-run reset to 0).</summary>
        public bool IntervalElapsed(float intervalSeconds)
            => intervalSeconds <= 0f || Time.timeAsDouble - _lastTime >= intervalSeconds;

        /// <summary>Stamp the throttle clock now (per-frame stamping paths).</summary>
        public void MarkNow() => _lastTime = Time.timeAsDouble;

        /// <summary>
        /// Close the gate at a batch boundary: the batch that just completed still
        /// publishes, then <paramref name="integrator"/> drops whole frames until
        /// <see cref="TryReopen"/> — no partial instant can mix.
        /// </summary>
        public void Close(TSDFIntegrator integrator)
        {
            MarkNow();
            if (integrator != null) integrator.integrationEnabled = false;
            _gated = true;
        }

        /// <summary>
        /// Re-open the integrator gate once the interval has elapsed (or the owner set
        /// the interval back to 0 mid-run). No-op while not gated.
        /// </summary>
        public void TryReopen(TSDFIntegrator integrator, float intervalSeconds)
        {
            if (!_gated || !IntervalElapsed(intervalSeconds)) return;
            _gated = false;
            if (integrator != null) integrator.integrationEnabled = true;
        }

        /// <summary>Forget the gated state WITHOUT touching the integrator (stop /
        /// unsubscribe paths manage integrationEnabled themselves).
        /// <paramref name="immediateNextStamp"/> also rewinds the throttle clock so the
        /// next <see cref="IntervalElapsed"/> fires immediately (fresh capture start).</summary>
        public void Reset(bool immediateNextStamp = false)
        {
            _gated = false;
            if (immediateNextStamp) _lastTime = double.NegativeInfinity;
        }
    }
}
