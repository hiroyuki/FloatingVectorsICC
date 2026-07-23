// Contract for a subsystem that can tear itself down ACROSS FRAMES instead of in
// one blocking call.
//
// Why this exists: the quit path stops four camera pipelines. Doing that inside
// OnApplicationQuit is correct but invisible — the main thread never returns to
// the player loop, so nothing can be drawn and the operator watches a frozen last
// frame for ~4.5s with no idea how far along it is. A staged teardown keeps the
// same concurrency (the secondaries still stop in parallel on worker threads) and
// merely yields between polls, which is enough for the shutdown splash to show a
// live count.
//
// Lives in Shared so AppQuitHotkey (Shared) can drive SensorManager (PointCloud)
// without the reference going the wrong way — PointCloud already references
// Shared, so the interface has to be on this side.

using System;
using System.Collections;

namespace Shared
{
    public interface IStagedShutdown
    {
        /// <summary>Stop everything, yielding often enough that the caller's frames
        /// keep rendering. <paramref name="progress"/> is (done, total) in whatever
        /// unit the implementer counts — for the rig that is cameras stopped.
        /// Implementations must be safe to call once and must make the ordinary
        /// blocking teardown a no-op afterwards, since the engine will still run its
        /// own quit callbacks after this completes.</summary>
        IEnumerator StopStaged(Action<int, int> progress);

        /// <summary>The quit is being forced while <see cref="StopStaged"/> was still
        /// running (its watchdog fired). Put native state into whatever configuration
        /// survives an exit that no longer waits — for the rig that means refusing to
        /// dispose the shared SDK context, since a pipeline stop may still be in
        /// flight on a worker thread. Must not block: the caller is about to quit.
        ///
        /// This exists because the watchdog's bound and each implementation's own
        /// bounds are configured independently, so the watchdog can win the race and
        /// must not depend on the implementation having reached its own safety path.</summary>
        void AbandonForForcedExit();
    }
}
