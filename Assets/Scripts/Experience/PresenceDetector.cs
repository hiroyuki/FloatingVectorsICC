// Visitor presence for the experience flow (Phase 4,
// Plans/phase4-presence-dwell-plan.md). Two independent signals:
//
//   BT path (during the experience): the merged person's pelvis
//   (SkeletonMerger.TryGetPrimaryPerson) inside the sensing OBB, debounced.
//
//   Occupancy path (during attract): playback occupies the k4abt workers, so
//   presence falls back to counting LIVE point-cloud points inside the OBB on
//   the GPU (PresenceOccupancy.compute + AsyncGPUReadback). Live renderer
//   meshes are read regardless of visibility / suppressAsSource — hiding the
//   live visuals must not blind the detector.
//
// The inside predicate is shared 1:1 with the compute kernel (see the .compute
// header): XZ-only inset + OBB height guard + world-Y band. Thresholds, inset
// and band are all serialized for on-site tuning (carpet reflections, parents
// leaning over the edge).
//
// Single-person installation: PersonCount / CrowdActive exist only to drive
// the "one at a time please" warning, not multi-person tracking.

using System.Collections.Generic;
using BodyTracking;
using PointCloud;
using UnityEngine;
using UnityEngine.Rendering;

namespace Experience
{
    [DisallowMultipleComponent]
    public class PresenceDetector : MonoBehaviour
    {
        [Tooltip("Merged-skeleton source (BT path). Auto-resolves when empty.")]
        public SkeletonMerger merger;

        [Tooltip("Sensing volume (the experience-space OBB). Auto-resolves when empty.")]
        public BoundingVolume sensingVolume;

        [Tooltip("Live rig whose renderer meshes feed the occupancy counter. " +
                 "Auto-resolves when empty.")]
        public SensorManager sensorManager;

        [Tooltip("Occupancy sources override (testing / special rigs). Non-empty → " +
                 "used INSTEAD of the live renderers. Same GPU contract as the motion " +
                 "curves: Raw vertex buffer, 24B ObColorPoint stride.")]
        public MeshFilter[] overrideSources = new MeshFilter[0];

        [Header("Sensing volume trim")]
        [Min(0f)]
        [Tooltip("XZ-only inner margin (m) — trims carpet edges without cutting feet/head. " +
                 "Clamped to 0 with a warning when >= half the box extent.")]
        public float insetMeters = 0f;

        [Tooltip("World-space vertical band: only points/pelvis between these heights count.")]
        public float yBandMin = -100f;
        public float yBandMax = 100f;

        [Header("Debounce")]
        [Min(0f)] public float enterDebounceSeconds = 0.3f;
        [Min(0f)] public float exitDebounceSeconds = 1.0f;

        [Header("Occupancy (attract mode)")]
        [Tooltip("Run the GPU occupancy counter. Off → OccupancyActive stays false.")]
        public bool occupancyEnabled = true;
        [Min(1)]
        [Tooltip("Points inside the volume at/above which occupancy reads as a person. " +
                 "On-site tunable.")]
        public int occupancyThreshold = 1500;
        [Min(0.05f)] public float occupancyIntervalSeconds = 0.2f;
        [Min(1f)]
        [Tooltip("Reject |world coordinate| beyond this (the reconstructor's 1e10 hole dummies).")]
        public float sanityRange = 50f;

        [Header("Debug")]
        [Tooltip("Force every presence signal true (Mac / no-hardware dry runs).")]
        public bool debugForcePresence;

        // ---- outputs ----
        /// <summary>BT: debounced pelvis-inside-volume.</summary>
        public bool IsPersonInside => debugForcePresence || _personInside;
        /// <summary>Occupancy: latest GPU readback (points inside the volume).</summary>
        public int OccupancyCount { get; private set; }
        /// <summary>Occupancy: debounced count >= threshold.</summary>
        public bool OccupancyActive => debugForcePresence || _occupancyActive;
        /// <summary>Any presence signal.</summary>
        public bool IsPresent => debugForcePresence || _personInside || _occupancyActive;
        /// <summary>Raw merged person count this frame (no debounce).</summary>
        public int PersonCount => _debugRawPersonCount >= 0 ? _debugRawPersonCount
                                  : (merger != null ? merger.PersonCount : 0);
        /// <summary>&gt;1 person, debounced (SkeletonMerger's crowd-alert hysteresis).</summary>
        public bool CrowdActive => _debugRawPersonCount >= 0 ? _debugRawPersonCount > 1
                                   : merger != null && merger.CrowdActive;

        /// <summary>Primary person's hands (world). False when nobody is tracked.</summary>
        public bool TryGetHands(out Vector3 left, out bool leftTracked,
                                out Vector3 right, out bool rightTracked)
        {
            left = right = Vector3.zero;
            leftTracked = rightTracked = false;
            if (merger == null || !merger.TryGetPrimaryPerson(out var p)) return false;
            left = p.HandLeftWorld; leftTracked = p.HandLeftTracked;
            right = p.HandRightWorld; rightTracked = p.HandRightTracked;
            return true;
        }

        // ---- state ----
        private bool _personInside;
        private float _btInsideSince = -1f, _btOutsideSince = -1f;
        private bool _occupancyActive;
        private float _occInsideSince = -1f, _occOutsideSince = -1f;
        private float _nextOccupancyDispatch;
        private bool _readbackPending;
        private AsyncGPUReadbackRequest _readback;
        private ComputeShader _shader;
        private int _kernel = -1;
        private ComputeBuffer _counter;
        private bool _insetWarned;
        private int _debugRawPersonCount = -1; // test hook (RunCommand); -1 = off

        private static readonly uint[] s_counterReset = { 0u };

        // ---- lifecycle ----

        private void OnEnable()
        {
            if (merger == null) merger = FindFirstObjectByType<SkeletonMerger>();
            if (sensingVolume == null) sensingVolume = FindFirstObjectByType<BoundingVolume>();
            if (sensorManager == null) sensorManager = FindFirstObjectByType<SensorManager>();
        }

        private void OnDisable()
        {
            _counter?.Release();
            _counter = null;
            _readbackPending = false;
            _personInside = false;
            _occupancyActive = false;
            _btInsideSince = _btOutsideSince = _occInsideSince = _occOutsideSince = -1f;
            OccupancyCount = 0;
        }

        private void Update()
        {
            float now = Time.realtimeSinceStartup;
            UpdateBtPresence(now);
            UpdateOccupancy(now);
        }

        /// <summary>Test hook: override the raw person count feeding PersonCount /
        /// CrowdActive (-1 = live data). Editor verification only.</summary>
        public void DebugSetRawPersonCount(int count) => _debugRawPersonCount = count;

        // ---- shared inside predicate (must stay 1:1 with PresenceOccupancy.compute) ----

        /// <summary>XZ-inset OBB + world-Y band test, identical to the GPU kernel.</summary>
        public bool IsInsideSensingVolume(Vector3 world)
        {
            if (sensingVolume == null) return false;
            GetNormalizedInsets(out float insetX, out float insetZ);
            Vector3 b = sensingVolume.transform.InverseTransformPoint(world);
            bool insideXZ = Mathf.Abs(b.x) <= 0.5f - insetX && Mathf.Abs(b.z) <= 0.5f - insetZ;
            bool insideBoxY = Mathf.Abs(b.y) <= 0.5f;
            bool insideWorldY = world.y >= yBandMin && world.y <= yBandMax;
            return insideXZ && insideBoxY && insideWorldY;
        }

        private void GetNormalizedInsets(out float insetX, out float insetZ)
        {
            Vector3 size = sensingVolume.transform.lossyScale;
            float inset = float.IsFinite(insetMeters) ? Mathf.Max(0f, insetMeters) : 0f;
            insetX = size.x > 1e-4f ? inset / size.x : 0f;
            insetZ = size.z > 1e-4f ? inset / size.z : 0f;
            if (insetX >= 0.5f || insetZ >= 0.5f)
            {
                if (!_insetWarned)
                {
                    Debug.LogWarning($"[{nameof(PresenceDetector)}] insetMeters ({insetMeters} m) is >= half " +
                                     "the sensing box extent — the area would invert. Ignoring the inset.", this);
                    _insetWarned = true;
                }
                insetX = insetZ = 0f;
            }
        }

        // ---- BT path ----

        private void UpdateBtPresence(float now)
        {
            bool insideNow = merger != null && merger.TryGetPrimaryPerson(out var person)
                             && IsInsideSensingVolume(person.PelvisWorld);
            Debounce(insideNow, now, ref _personInside, ref _btInsideSince, ref _btOutsideSince);
        }

        private void Debounce(bool rawInside, float now, ref bool state,
                              ref float insideSince, ref float outsideSince)
        {
            if (rawInside)
            {
                outsideSince = -1f;
                if (insideSince < 0f) insideSince = now;
                if (!state && now - insideSince >= enterDebounceSeconds) state = true;
            }
            else
            {
                insideSince = -1f;
                if (outsideSince < 0f) outsideSince = now;
                if (state && now - outsideSince >= exitDebounceSeconds) state = false;
            }
        }

        // ---- occupancy path ----

        private void UpdateOccupancy(float now)
        {
            if (!occupancyEnabled || sensingVolume == null)
            {
                // Full reset — a re-enable must not inherit a stale count.
                _occupancyActive = false;
                OccupancyCount = 0;
                _occInsideSince = _occOutsideSince = -1f;
                return;
            }

            // Harvest a finished readback first. A failed readback zeroes the
            // count: a stale occupied value must not keep OccupancyActive true.
            if (_readbackPending && _readback.done)
            {
                _readbackPending = false;
                OccupancyCount = _readback.hasError ? 0 : (int)_readback.GetData<uint>()[0];
            }

            float interval = float.IsFinite(occupancyIntervalSeconds)
                ? Mathf.Max(0.05f, occupancyIntervalSeconds) : 0.2f;
            if (!_readbackPending && now >= _nextOccupancyDispatch)
            {
                if (DispatchOccupancy())
                    _nextOccupancyDispatch = now + interval;
                else
                    OccupancyCount = 0; // no usable sources this tick — same rule
            }

            Debounce(OccupancyCount >= occupancyThreshold, now,
                     ref _occupancyActive, ref _occInsideSince, ref _occOutsideSince);
        }

        private bool DispatchOccupancy()
        {
            if (!EnsureCompute()) return false;

            _counter.SetData(s_counterReset);
            GetNormalizedInsets(out float insetX, out float insetZ);
            _shader.SetBuffer(_kernel, "_Counter", _counter);
            _shader.SetMatrix("_WorldToBox", sensingVolume.transform.worldToLocalMatrix);
            _shader.SetFloat("_SanityRange", sanityRange);
            _shader.SetFloat("_InsetX", insetX);
            _shader.SetFloat("_InsetZ", insetZ);
            _shader.SetFloat("_YBandMin", float.IsFinite(yBandMin) ? yBandMin : -100f);
            _shader.SetFloat("_YBandMax", float.IsFinite(yBandMax) ? yBandMax : 100f);

            int dispatched = 0;
            var borrowed = new List<GraphicsBuffer>(4);
            foreach (var mf in ResolveSources())
            {
                var mesh = mf.sharedMesh;
                var vb = mesh.GetVertexBuffer(0);
                if (vb == null) continue;
                borrowed.Add(vb);
                _shader.SetBuffer(_kernel, "_Src", vb);
                _shader.SetMatrix("_SrcLocalToWorld", mf.transform.localToWorldMatrix);
                _shader.SetInt("_SrcCount", mesh.vertexCount);
                _shader.Dispatch(_kernel, (mesh.vertexCount + 63) / 64, 1, 1);
                dispatched++;
            }
            foreach (var vb in borrowed) vb.Dispose();
            if (dispatched == 0) return false;

            if (SystemInfo.supportsAsyncGPUReadback)
            {
                _readback = AsyncGPUReadback.Request(_counter);
                _readbackPending = true;
            }
            else
            {
                var host = new uint[1];
                _counter.GetData(host); // sync fallback (Mac editor without async support)
                OccupancyCount = (int)host[0];
            }
            return true;
        }

        // Same source contract as PointCloudMotionCurves.IsUsableSource: only
        // meshes whose vertex buffer is Raw-bindable at the 24B ObColorPoint
        // stride can feed the kernel.
        private readonly List<MeshFilter> _srcScratch = new List<MeshFilter>(4);

        private List<MeshFilter> ResolveSources()
        {
            _srcScratch.Clear();
            if (overrideSources != null && overrideSources.Length > 0)
            {
                foreach (var mf in overrideSources)
                    if (IsUsableSource(mf)) _srcScratch.Add(mf);
                return _srcScratch;
            }
            if (sensorManager == null) return _srcScratch;
            foreach (var r in sensorManager.Renderers)
            {
                if (r == null) continue;
                if (!r.TryGetComponent(out MeshFilter mf)) continue;
                if (IsUsableSource(mf)) _srcScratch.Add(mf);
            }
            return _srcScratch;
        }

        private const int kObColorPointStride = 24;

        private static bool IsUsableSource(MeshFilter mf)
        {
            if (mf == null) return false;
            var m = mf.sharedMesh;
            if (m == null || m.vertexCount == 0) return false;
            if ((m.vertexBufferTarget & GraphicsBuffer.Target.Raw) == 0) return false;
            if (m.GetVertexBufferStride(0) != kObColorPointStride) return false;
            return true;
        }

        private bool EnsureCompute()
        {
            if (_shader == null)
            {
                _shader = Resources.Load<ComputeShader>("PresenceOccupancy");
                if (_shader == null)
                {
                    Debug.LogError($"[{nameof(PresenceDetector)}] Resources/PresenceOccupancy.compute not found.", this);
                    occupancyEnabled = false;
                    return false;
                }
                _kernel = _shader.FindKernel("CSCount");
            }
            _counter ??= new ComputeBuffer(1, sizeof(uint));
            return _kernel >= 0;
        }
    }
}
