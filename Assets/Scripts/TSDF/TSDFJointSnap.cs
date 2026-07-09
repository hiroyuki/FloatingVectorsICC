// Snaps merged body-tracking foot joints onto the observed TSDF surface.
//
// Why: occluded-side cameras report a lifted foot at a floor-biased position
// with MEDIUM confidence, so the confidence-weighted merge lags below the real
// limb. The depth data — and therefore the TSDF shell — sees the actual foot in
// real time, so the volume is the ground truth to pull the joint toward. The
// continuity gate (SkeletonMerger) fixes sustained holds; this fixes the
// mid-motion lag the gate cannot (its prediction trails during acceleration).
//
// How: registers as SkeletonMerger.JointRefiner (interface lives in
// BodyTracking; this class sits in the TSDF assembly, which references it).
// Every merge the refiner applies a cached per-joint world-space CORRECTION;
// LateUpdate dispatches TSDFJointSnap.compute on the pre-correction positions
// and an AsyncGPUReadback updates the corrections one frame later. Delta (not
// absolute) application means that one-frame latency only affects the rate of
// change of the correction, and feeding the kernel pre-correction positions
// prevents self-feedback.

using System.Collections.Generic;
using BodyTracking;
using UnityEngine;
using UnityEngine.Rendering;

namespace TSDF
{
    public sealed class TSDFJointSnap : MonoBehaviour, IMergedJointRefiner
    {
        [Tooltip("TSDF volume to snap against. Auto-resolves when left empty.")]
        public TSDFVolume volume;
        [Tooltip("Merger whose merged joints are refined. Auto-resolves when left empty.")]
        public SkeletonMerger merger;

        [Header("Snap")]
        [Tooltip("Phase-1 search radius (metres) for the nearest observed shell voxel " +
                 "around the merged joint. Must exceed the worst merge lag (~0.2 during a " +
                 "fast lift) or the true limb shell is never found.")]
        public float searchRadius = 0.2f;
        [Tooltip("Phase-2 refinement radius (metres) around the projected target.")]
        public float refineRadius = 0.03f;
        [Tooltip("Reject snaps that would move a joint further than this (metres). Keep " +
                 ">= searchRadius + embed or legitimate far snaps get rejected after the fact.")]
        public float maxSnapDist = 0.25f;
        [Tooltip("Reject snaps that would move a joint DOWN by more than this (metres). " +
                 "The floor is part of the TSDF and the k4abt error mode is always 'too " +
                 "low', so only upward/lateral correction is wanted.")]
        public float maxDownwardSnap = 0.03f;
        [Tooltip("Voxel weight below this counts as unobserved (matches the MC gate).")]
        public float minWeight = 0.5f;
        [Tooltip("World height (metres) of the physical floor. Shell voxels below " +
                 "floorY + floorMargin are never snap candidates — the floor is part of " +
                 "the TSDF, and a floor-biased joint can sink close enough that the " +
                 "relative down-filter alone re-admits the floor shell. Set from the " +
                 "calibrated stage (recording 12-50-09: -0.88). Very negative = disabled.")]
        public float floorY = -10f;
        [Tooltip("Margin (metres) above floorY still treated as floor shell.")]
        public float floorMargin = 0.03f;
        [Tooltip("How deep (metres) inside the surface the ankle joint centre sits.")]
        public float ankleEmbedDepth = 0.045f;
        [Tooltip("How deep (metres) inside the surface the foot joint centre sits.")]
        public float footEmbedDepth = 0.025f;
        [Tooltip("How deep (metres) inside the surface the knee joint centre sits.")]
        public float kneeEmbedDepth = 0.06f;

        [Header("Correction dynamics")]
        [Tooltip("Blend rate of new snap results into the cached correction (per readback).")]
        [Range(0f, 1f)] public float correctionEma = 0.5f;
        [Tooltip("Readback older than this (seconds) counts as stale and the correction " +
                 "starts decaying — covers mid-clear frames of the single-buffer volume.")]
        public float staleAfterSeconds = 0.25f;
        [Tooltip("Exponential decay time constant (seconds) for stale corrections.")]
        public float decaySeconds = 0.15f;

        [Header("Debug")]
        [Tooltip("Use synchronous GetData instead of AsyncGPUReadback (ground-truth A/B; " +
                 "stalls the pipeline, editor only).")]
        public bool debugSyncReadback = false;
        [Tooltip("Draw input->snapped gizmo lines for the latest cluster.")]
        public bool debugDrawGizmos = false;
        [Tooltip("Log snap acceptance counters once per second.")]
        public bool logDiagnostics = false;

        private const int kJointCount = 6; // matches SkeletonMerger.s_refineJoints order:
                                           // ankleL, footL, ankleR, footR, kneeL, kneeR

        private sealed class SnapState
        {
            public readonly Vector3[] Correction = new Vector3[kJointCount];
            public readonly Vector3[] PendingInput = new Vector3[kJointCount];
            public readonly bool[] PendingValid = new bool[kJointCount];
            public bool HasPending;
            public float LastResultRealtime;
            public float LastTouchedRealtime;
        }

        private readonly Dictionary<uint, SnapState> _states = new();
        private readonly List<uint> _pruneScratch = new();
        private ComputeShader _shader;
        private int _kernel = -1;
        private ComputeBuffer _jointsIn, _jointsOut;
        private readonly Vector4[] _jointsInData = new Vector4[kJointCount];
        private readonly Vector4[] _jointsOutSync = new Vector4[kJointCount];
        private readonly Vector3[] _inFlightInput = new Vector3[kJointCount];
        private readonly bool[] _inFlightValid = new bool[kJointCount];
        private uint _inFlightClusterId;
        private bool _readbackInFlight;
        // Invalidates pending async callbacks across disable/re-enable: a stale
        // callback must neither clear the NEW request's in-flight flag nor apply
        // old GPU output with the new request's cluster/input snapshot.
        private int _requestGeneration;
        private uint _latestClusterId;
        private bool _haveCluster;
        private int _diagAccepted, _diagRejected;
        private float _diagWindowStart;

        private void OnEnable()
        {
            if (volume == null) volume = FindFirstObjectByType<TSDFVolume>();
            if (merger == null) merger = FindFirstObjectByType<SkeletonMerger>();
            if (volume == null || merger == null)
            {
                Debug.LogWarning("[TSDFJointSnap] missing TSDFVolume or SkeletonMerger — disabling.", this);
                enabled = false;
                return;
            }
            if (!TSDFComputeUtil.TryLoad(ref _shader, "TSDFJointSnap", "TSDFJointSnap", this))
            {
                enabled = false;
                return;
            }
            _kernel = _shader.FindKernel("SnapJoints");
            if (merger.JointRefiner != null && !ReferenceEquals(merger.JointRefiner, this))
                Debug.LogWarning("[TSDFJointSnap] replacing an existing JointRefiner.", this);
            merger.JointRefiner = this;
        }

        private void OnDisable()
        {
            if (merger != null && ReferenceEquals(merger.JointRefiner, this))
                merger.JointRefiner = null;
            _requestGeneration++; // orphan any pending async readback
            _jointsIn?.Release(); _jointsIn = null;
            _jointsOut?.Release(); _jointsOut = null;
            _states.Clear();
            _readbackInFlight = false;
            _haveCluster = false;
        }

        // Called by SkeletonMerger for every merged cluster, every Update.
        // Applies the cached correction and records the PRE-correction input for
        // the next dispatch.
        public void RefineJoints(uint clusterId, k4abt_joint_id_t[] joints, Vector3[] positionsWorld, bool[] valid)
        {
            if (!_states.TryGetValue(clusterId, out var state))
            {
                state = new SnapState();
                _states[clusterId] = state;
            }
            float now = Time.realtimeSinceStartup;
            state.LastTouchedRealtime = now;
            _latestClusterId = clusterId;
            _haveCluster = true;

            // Stale readback (volume mid-clear, GPU hiccup, playback pause):
            // bleed the correction out instead of freezing a possibly-wrong offset.
            if (now - state.LastResultRealtime > staleAfterSeconds)
            {
                float k = Mathf.Exp(-Time.deltaTime / Mathf.Max(0.01f, decaySeconds));
                for (int i = 0; i < kJointCount; i++) state.Correction[i] *= k;
            }

            for (int i = 0; i < kJointCount; i++)
            {
                state.PendingValid[i] = valid[i];
                if (!valid[i]) continue;
                state.PendingInput[i] = positionsWorld[i]; // pre-correction
                positionsWorld[i] += state.Correction[i];
            }
            state.HasPending = true;
        }

        private void LateUpdate()
        {
            PruneStaleStates();

            if (!_haveCluster || _readbackInFlight) return;
            if (!_states.TryGetValue(_latestClusterId, out var state) || !state.HasPending) return;
            if (volume == null || volume.FrontBuffer == null || !volume.FrontBuffer.IsValid()) return;

            EnsureBuffers();

            for (int i = 0; i < kJointCount; i++)
            {
                float embed = i switch
                {
                    0 or 2 => ankleEmbedDepth,
                    1 or 3 => footEmbedDepth,
                    _ => kneeEmbedDepth,
                };
                // Invalid joints still occupy a slot (fixed layout); their output
                // is ignored in ApplyResult.
                var p = state.PendingInput[i];
                _jointsInData[i] = new Vector4(p.x, p.y, p.z, embed);
                _inFlightInput[i] = p;
                _inFlightValid[i] = state.PendingValid[i];
            }
            _jointsIn.SetData(_jointsInData);

            float vs = Mathf.Max(1e-4f, volume.voxelSize);
            int searchVox = Mathf.Clamp(Mathf.RoundToInt(searchRadius / vs), 1, 48);
            int refineVox = Mathf.Clamp(Mathf.RoundToInt(refineRadius / vs), 1, 12);

            var dim = volume.Dim;
            _shader.SetInts("_Dim", dim.x, dim.y, dim.z);
            _shader.SetMatrix("_WorldFromVoxel", volume.WorldFromVoxel);
            _shader.SetMatrix("_VoxelFromWorld", volume.VoxelFromWorld);
            _shader.SetFloat("_MinWeight", minWeight);
            _shader.SetInt("_SearchRadiusVox", searchVox);
            _shader.SetInt("_RefineRadiusVox", refineVox);
            _shader.SetFloat("_MaxSnapDist", maxSnapDist);
            _shader.SetFloat("_MaxDownward", maxDownwardSnap);
            _shader.SetFloat("_FloorCutoffY", floorY + floorMargin);
            _shader.SetInt("_JointCount", kJointCount);
            _shader.SetBuffer(_kernel, "_Voxels", volume.FrontBuffer);
            _shader.SetBuffer(_kernel, "_JointsIn", _jointsIn);
            _shader.SetBuffer(_kernel, "_JointsOut", _jointsOut);
            _shader.Dispatch(_kernel, kJointCount, 1, 1);

            state.HasPending = false;
            _inFlightClusterId = _latestClusterId;

            if (debugSyncReadback || !SystemInfo.supportsAsyncGPUReadback)
            {
                _jointsOut.GetData(_jointsOutSync);
                ApplyResult(_inFlightClusterId, _jointsOutSync);
            }
            else
            {
                _readbackInFlight = true;
                int gen = ++_requestGeneration;
                AsyncGPUReadback.Request(_jointsOut, req =>
                {
                    // Stale callback (component was disabled or re-dispatched
                    // since): the in-flight snapshot fields belong to a newer
                    // request — touch nothing.
                    if (gen != _requestGeneration) return;
                    _readbackInFlight = false;
                    if (req.hasError) return;
                    // Only one readback is ever in flight, so the sync scratch
                    // array is free to reuse here.
                    req.GetData<Vector4>().CopyTo(_jointsOutSync);
                    ApplyResult(_inFlightClusterId, _jointsOutSync);
                });
            }
        }

        private void ApplyResult(uint clusterId, Vector4[] results)
        {
            if (!_states.TryGetValue(clusterId, out var state)) return; // evicted meanwhile
            state.LastResultRealtime = Time.realtimeSinceStartup;
            for (int i = 0; i < kJointCount; i++)
                ApplyOne(state, i, results[i]);
            LogDiagnosticsIfDue();
        }

        private void ApplyOne(SnapState state, int i, Vector4 result)
        {
            Vector3 targetCorrection = Vector3.zero;
            if (_inFlightValid[i] && result.w > 0.5f)
            {
                targetCorrection = new Vector3(result.x, result.y, result.z) - _inFlightInput[i];
                _diagAccepted++;
            }
            else
            {
                _diagRejected++;
            }
            state.Correction[i] = Vector3.Lerp(state.Correction[i], targetCorrection, correctionEma);
        }

        private void EnsureBuffers()
        {
            if (_jointsIn == null || !_jointsIn.IsValid())
                _jointsIn = new ComputeBuffer(kJointCount, sizeof(float) * 4);
            if (_jointsOut == null || !_jointsOut.IsValid())
                _jointsOut = new ComputeBuffer(kJointCount, sizeof(float) * 4);
        }

        private void PruneStaleStates()
        {
            if (_states.Count <= 4) return;
            float now = Time.realtimeSinceStartup;
            _pruneScratch.Clear();
            foreach (var kv in _states)
                if (now - kv.Value.LastTouchedRealtime > 5f) _pruneScratch.Add(kv.Key);
            for (int i = 0; i < _pruneScratch.Count; i++) _states.Remove(_pruneScratch[i]);
        }

        private void LogDiagnosticsIfDue()
        {
            if (!logDiagnostics)
            {
                _diagAccepted = 0;
                _diagRejected = 0;
                return;
            }
            float now = Time.realtimeSinceStartup;
            if (_diagWindowStart == 0f) _diagWindowStart = now;
            if (now - _diagWindowStart < 1f) return;
            Debug.Log($"[TSDFJointSnap] snaps/sec accepted={_diagAccepted} rejected={_diagRejected}", this);
            _diagAccepted = 0;
            _diagRejected = 0;
            _diagWindowStart = now;
        }

        private void OnDrawGizmos()
        {
            if (!debugDrawGizmos || !_haveCluster) return;
            if (!_states.TryGetValue(_latestClusterId, out var state)) return;
            for (int i = 0; i < kJointCount; i++)
            {
                if (!state.PendingValid[i]) continue;
                Vector3 input = state.PendingInput[i];
                Vector3 corrected = input + state.Correction[i];
                Gizmos.color = Color.yellow;
                Gizmos.DrawWireSphere(input, 0.02f);
                Gizmos.color = Color.green;
                Gizmos.DrawLine(input, corrected);
                Gizmos.DrawWireSphere(corrected, 0.02f);
            }
        }
    }
}
