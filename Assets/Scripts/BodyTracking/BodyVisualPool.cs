// Per-body visual lifecycle / GC owned by SkeletonMerger.
// Handles EvictIfFull + ApplyBodySkeleton + GC loop.

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

        // Radius (m) of the tube mesh drawn for each anatomical bone segment. Fed
        // to BodyVisual on every UpdateFromSkeleton so Inspector tweaks take effect
        // live without spawning a new visual.
        public float BoneWidth;
        public int MaxBodies;
        public bool UseOneEuroFilter;
        public float OneEuroMinCutoff;
        public float OneEuroBeta;
        public float OneEuroDerivCutoff;
    }

    internal sealed class BodyVisualPool
    {
        private readonly Transform _parent;
        private readonly Dictionary<uint, BodyVisual> _bodies = new Dictionary<uint, BodyVisual>();
        private readonly Dictionary<uint, int> _lastSeenFrame = new Dictionary<uint, int>();
        private readonly List<uint> _toDestroy = new List<uint>();

        public BodyVisualPool(Transform parent) { _parent = parent; }

        public int Count => _bodies.Count;

        /// <summary>Iterate live visuals (key + value). Mutating during iteration is not supported.</summary>
        public Dictionary<uint, BodyVisual> Visuals => _bodies;

        /// <summary>
        /// Create-or-update the visual for <paramref name="id"/>, applying the
        /// given skeleton. Caller passes <paramref name="cfg"/> re-built each frame
        /// so Inspector edits are picked up live. <paramref name="onEvicted"/> fires
        /// once per evicted id when the pool has to free a slot to honor MaxBodies —
        /// callers use it to clean their own per-id state (e.g. continuity dictionaries).
        /// </summary>
        public void Apply(uint id, in k4abt_skeleton_t skel, in BodyVisualConfig cfg, Action<uint> onEvicted = null)
        {
            if (!_bodies.TryGetValue(id, out var visual))
            {
                EvictIfFull(cfg.MaxBodies, onEvicted);
                visual = new BodyVisual(_parent, id, cfg.JointRadius, cfg.SkeletonColor);
                _bodies[id] = visual;
            }
            visual.UpdateFromSkeleton(skel, in cfg);
            _lastSeenFrame[id] = Time.frameCount;
        }

        /// <summary>
        /// Destroy visuals not seen for the given number of frames.
        /// <paramref name="onEvicted"/> fires once per destroyed id so callers can clean
        /// their own per-id state (e.g. SkeletonMerger's continuity dictionaries).
        /// </summary>
        public void GcStale(int unseenFramesBeforeDestroy, Action<uint> onEvicted = null)
        {
            _toDestroy.Clear();
            foreach (var kv in _bodies)
            {
                int lastSeen = _lastSeenFrame.TryGetValue(kv.Key, out var f) ? f : -1;
                int sinceLastSeen = Time.frameCount - lastSeen;
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

        /// <summary>
        /// Push <paramref name="cfg"/> into every live visual's geometry without
        /// ingesting new samples. Used by the owning MonoBehaviour
        /// during Editor pause so Inspector tweaks update the visuals live.
        /// </summary>
        public void ReapplyConfigToAll(in BodyVisualConfig cfg)
        {
            foreach (var v in _bodies.Values) v.ApplyConfigOnly(in cfg);
        }
    }
}
