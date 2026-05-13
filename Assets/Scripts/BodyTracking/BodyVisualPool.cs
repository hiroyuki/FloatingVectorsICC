// Shared per-body visual lifecycle / GC owned by either BodyTrackingLive
// (single in-process tracker) or BodyTrackingMultiLive (merged multi-worker).
// Phase 5b of issue #11: dedupe ~80 lines of EvictIfFull + ApplyBodySkeleton
// + GC loop that used to live in both MonoBehaviours.

using System;
using System.Collections.Generic;
using UnityEngine;

namespace BodyTracking
{
    /// <summary>
    /// Visual / behavior params applied to BodyVisual on each ApplyBodySkeleton
    /// call. Carried as a struct so Inspector tweaks live-update without per-frame
    /// allocations.
    /// </summary>
    internal struct BodyVisualConfig
    {
        public float JointRadius;
        public Color SkeletonColor;
        public bool ShowAnatomicalBones;
        public bool ShowTrails;
        public float TrailDuration;
        public float TrailWidth;
        public BodyTrackingLive.TrailColorMode TrailColorMode;
        public Color TrailFlatColor;
        public int MaxBodies;
        public float TrailSmoothing;
        public float AccelMin;
        public float AccelMax;
        public Color AccelHotColor;
    }

    internal sealed class BodyVisualPool
    {
        private readonly Transform _parent;
        private readonly Dictionary<uint, BodyVisual> _bodies = new Dictionary<uint, BodyVisual>();
        private readonly Dictionary<uint, int> _lastSeenFrame = new Dictionary<uint, int>();
        private readonly List<uint> _toDestroy = new List<uint>();

        // Rolling window of |a| samples across every visible body + joint. Used by
        // TickTrails when autoAccelMax is on to pick a heatmap hot-end (p95) that
        // adapts to the recording instead of a hand-tuned Inspector constant.
        // Window size: 32 joints × ~30 Hz × ~1 s = ~960; 1024 keeps roughly one
        // second of context per body without unbounded growth.
        private const int kAccelWindowSize = 1024;
        private readonly Queue<float> _accelWindow = new Queue<float>(kAccelWindowSize);
        private readonly List<float> _accelScratch = new List<float>(kAccelWindowSize);
        private readonly List<float> _latestAccelsScratch = new List<float>(64);

        public BodyVisualPool(Transform parent) { _parent = parent; }

        public int Count => _bodies.Count;

        /// <summary>Iterate live visuals (key + value). Mutating during iteration is not supported.</summary>
        public Dictionary<uint, BodyVisual> Visuals => _bodies;

        /// <summary>
        /// Create-or-update the visual for <paramref name="id"/>, applying the
        /// given skeleton + trail params. Caller passes <paramref name="cfg"/>
        /// re-built each frame so Inspector edits are picked up live.
        /// <paramref name="onEvicted"/> fires once per evicted id when the pool
        /// has to free a slot to honor MaxBodies — callers use it to clean their
        /// own per-id state (e.g. continuity dictionaries).
        /// </summary>
        public void Apply(uint id, in k4abt_skeleton_t skel, in BodyVisualConfig cfg, Action<uint> onEvicted = null)
        {
            if (!_bodies.TryGetValue(id, out var visual))
            {
                EvictIfFull(cfg.MaxBodies, onEvicted);
                visual = new BodyVisual(_parent, id, cfg.JointRadius, cfg.SkeletonColor,
                    cfg.ShowTrails, cfg.TrailDuration, cfg.TrailWidth,
                    cfg.TrailColorMode, cfg.TrailFlatColor);
                _bodies[id] = visual;
            }
            visual.UpdateFromSkeleton(skel, cfg.JointRadius, cfg.ShowAnatomicalBones, cfg.SkeletonColor, cfg.TrailSmoothing);
            visual.ApplyTrailParams(cfg.ShowTrails, cfg.TrailDuration, cfg.TrailWidth, cfg.TrailColorMode, cfg.TrailFlatColor,
                cfg.AccelMin, cfg.AccelMax, cfg.AccelHotColor);
            _lastSeenFrame[id] = Time.frameCount;
        }

        /// <summary>
        /// Rebuild every body's per-joint trail mesh against <paramref name="cam"/> for
        /// billboard orientation. Called once per frame from the owning MonoBehaviour's
        /// LateUpdate so positions/colors are fresh and old samples expire on schedule
        /// even when k4abt doesn't pop a new body frame this Update. When
        /// <paramref name="autoAccelMax"/> is true, computes a rolling p95 across all
        /// joints and overrides every JointTrailMesh's accelMax with it so the
        /// AccelHeatmap palette stretches across the actual motion range.
        /// </summary>
        public void TickTrails(Camera cam, bool autoAccelMax)
        {
            // Feed the rolling window with the latest |a| from every joint.
            _latestAccelsScratch.Clear();
            foreach (var v in _bodies.Values) v.CollectLatestAccels(_latestAccelsScratch);
            for (int i = 0; i < _latestAccelsScratch.Count; i++)
            {
                float a = _latestAccelsScratch[i];
                if (a <= 0f) continue;
                _accelWindow.Enqueue(a);
                while (_accelWindow.Count > kAccelWindowSize) _accelWindow.Dequeue();
            }

            if (autoAccelMax && _accelWindow.Count > 0)
            {
                _accelScratch.Clear();
                _accelScratch.AddRange(_accelWindow);
                _accelScratch.Sort();
                int n = _accelScratch.Count;
                float p95 = _accelScratch[Mathf.Clamp((int)(n * 0.95f), 0, n - 1)];
                if (p95 > 0f)
                    foreach (var v in _bodies.Values) v.SetTrailAccelMax(p95);
            }

            foreach (var v in _bodies.Values) v.TickTrails(cam);
        }

        /// <summary>
        /// Tick diagnostics and destroy visuals not seen for the given number of frames.
        /// <paramref name="onEvicted"/> fires once per destroyed id so callers can clean
        /// their own per-id state (e.g. BodyTrackingMultiLive's continuity dictionaries).
        /// </summary>
        public void GcStale(int unseenFramesBeforeDestroy, Action<uint> onEvicted = null)
        {
            _toDestroy.Clear();
            foreach (var kv in _bodies)
            {
                int lastSeen = _lastSeenFrame.TryGetValue(kv.Key, out var f) ? f : -1;
                int sinceLastSeen = Time.frameCount - lastSeen;
                kv.Value.TickDiagAfterUpdate();
                if (sinceLastSeen > unseenFramesBeforeDestroy) _toDestroy.Add(kv.Key);
            }
            for (int i = 0; i < _toDestroy.Count; i++)
            {
                uint id = _toDestroy[i];
                _bodies[id].Destroy();
                _bodies.Remove(id);
                _lastSeenFrame.Remove(id);
                onEvicted?.Invoke(id);
            }
        }

        /// <summary>
        /// Hard cap on cached BodyVisuals. K4ABT body ids can flap between frames
        /// when tracking is unstable; without this the pool would allocate 32+
        /// GameObjects per id forever.
        /// </summary>
        public void EvictIfFull(int maxBodies, Action<uint> onEvicted = null)
        {
            while (_bodies.Count >= maxBodies)
            {
                uint oldestId = 0;
                int oldestFrame = int.MaxValue;
                bool any = false;
                foreach (var kv in _bodies)
                {
                    int f = _lastSeenFrame.TryGetValue(kv.Key, out var v) ? v : -1;
                    if (f < oldestFrame) { oldestFrame = f; oldestId = kv.Key; any = true; }
                }
                if (!any) break;
                _bodies[oldestId].Destroy();
                _bodies.Remove(oldestId);
                _lastSeenFrame.Remove(oldestId);
                onEvicted?.Invoke(oldestId);
            }
        }

        public void DestroyAll()
        {
            foreach (var v in _bodies.Values) v.Destroy();
            _bodies.Clear();
            _lastSeenFrame.Clear();
        }
    }
}
