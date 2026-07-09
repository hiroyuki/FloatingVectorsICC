// Ring buffer of per-bone WORLD pose history for the primary (single-person) body,
// used to grow curved motion trails from point-cloud points (see the point-cloud
// motion-curves plan). SkeletonMerger.TryGetJointWorld only exposes joint POSITIONS,
// not orientations, and raw k4abt orientations do not survive the merge path — so
// instead of storing a per-bone quaternion (whose roll would be arbitrary frame to
// frame) we store each bone's endpoint pair (a_k, b_k) plus an explicitly-stabilized
// frame (u,v,w). A point parameterized in a bone's CURRENT stable frame (t, cv, cw)
// is reprojected through the history by SampleCurve to trace an arc: when the bone
// rotates, u/v/w rotate with it and the reprojected point sweeps a curve.
//
// Fed every LateUpdate — SkeletonMerger updates skeletons in Update, so by LateUpdate
// the joint positions reflect the current frame; consumers pick the curve up next
// frame (the same one-frame lag BodyJointMotionFeeder / BodyTubeCapsuleFeeder accept).
// On tracking loss / staleness a bone's history is reset so trails don't freeze or jump.

using UnityEngine;
using PointCloud;

namespace BodyTracking
{
    [DisallowMultipleComponent]
    public class BonePoseHistory : MonoBehaviour, global::Shared.IPanelTunable
    {
        [Tooltip("Body-tracking source. Leave empty to auto-resolve the first SkeletonMerger at OnEnable.")]
        public SkeletonMerger bodyTracking;

        [Tooltip("Playback source. Used only to tell an intentional pause (hold the sculpture) apart " +
                 "from a body that stopped/left (clear it). Auto-resolves the first SensorRecorder at OnEnable.")]
        public SensorRecorder recorder;

        private ulong _lastPoseVersion;
        private bool _havePoseVersion;

        [Min(2)]
        [Tooltip("Ring-buffer length K: how many past frames form each bone's curve.")]
        public int historySamples = 12;

        [Min(1)]
        [Tooltip("An endpoint is treated as stale (bone history reset) if its last fresh BT " +
                 "frame is older than this many frames. Guards against dragging held/predicted joints.")]
        public int freshWindowFrames = 12;

        [Range(0f, 0.4f)]
        [Tooltip("Length (m) of the virtual hand bone appended past each wrist. k4abt's hand " +
                 "joints are unreliable and dropped project-wide, so the bone table ends at the " +
                 "wrist and motion curves stopped mid-arm; this extends each forearm rigidly by " +
                 "this much so hand points inherit the forearm's motion. 0 disables it.")]
        public float handExtension = 0.18f;

        [Range(0f, 0.3f)]
        [Tooltip("Length (m) of the virtual crown bone appended past the HEAD joint. HEAD sits " +
                 "near the head's centre, so points on top of the skull were beyond every bone's " +
                 "reach and got no motion curves; this extends the neck->head bone rigidly by " +
                 "this much so crown points inherit the head's motion. 0 disables it.")]
        public float crownExtension = 0.12f;

        [Header("Debug gizmos (Phase 1 validation)")]
        [Tooltip("Draw each bone's reprojected curve in the Scene view to verify arcs form.")]
        public bool drawGizmos = true;

        [Range(0f, 0.3f)]
        [Tooltip("Perpendicular offset (on +v) of the demo curve drawn per bone. Non-zero makes " +
                 "rotation show up as an arc rather than a straight midpoint trail.")]
        public float gizmoOffset = 0.12f;

        public Color gizmoColor = new Color(1f, 0.55f, 0.1f, 1f);

        // ---- Shared.IPanelTunable (one-stop Control Panel) ----
        // Exposes the time-window length (K = historySamples): how many past frames each motion curve
        // spans. EnsureBuffers reallocates the ring when this changes, so live edits take effect next frame.
        public string TuningLabel => "Motion history";
        public int TunableCount => 1;
        // Label must match the Inspector field name (History Samples) so the
        // two UIs are recognisably the same knob.
        public string TunableName(int i) => "History Samples (frames)";
        public float TunableValue(int i) => historySamples;
        public void SetTunableValue(int i, float value) => historySamples = Mathf.Clamp(Mathf.RoundToInt(value), 2, MaxK);
        public float TunableMin(int i) => 4f;
        public float TunableMax(int i) => MaxK; // curve length caps at the ring size / compute MAXK
        public bool TunableIsInt(int i) => true;

        /// <summary>One stored frame of a bone: world endpoints + stabilized orthonormal frame.</summary>
        public struct Sample
        {
            public Vector3 A, B;    // world endpoints (a -> b)
            public Vector3 U, V, W; // stable orthonormal frame; U = normalize(B - A)
        }

        // Per-bone ring buffers. _ring[b] has length K; _head[b] is the newest slot;
        // _count[b] is how many valid samples exist (<= K). Oldest = (_head - _count + 1).
        private Sample[][] _ring;
        private int[] _head;
        private int[] _count;
        private Vector3[] _prevV;                 // previous stable v per bone (roll continuity)
        private int[] _parentBone;                // bone whose child == this bone's parent joint, or -1
        private SkeletonMerger.BoneEndpoints[] _scratch;
        private int _boneCount;
        private int _ringLen;

        // Virtual tip bones appended after the real Bones table, for body extremities the
        // skeleton doesn't reach: the hands (HAND/HANDTIP/THUMB joints are dropped
        // project-wide — BodyTrackingShared.IsDrawnJoint — so the arm ends at the wrist) and
        // the crown (HEAD sits near the head's centre, nothing spans the top of the skull).
        // Point-cloud seeds there had no bone within surfaceMargin, so motion curves stopped
        // short. Each virtual bone rigidly extends the real bone whose CHILD joint is the
        // listed tip (forearm past the wrist, neck->head past the crown) along that bone's
        // axis, reusing its stable frame (U/V/W), so tip points sweep with the parent bone's
        // motion. Indices [_realBoneCount ..] in every per-bone array / the GPU buffers;
        // consumers see them only as extra bones (PointCloudMotionCurves picks their radii
        // off this table).
        public static readonly k4abt_joint_id_t[] VirtualTipSources =
        {
            k4abt_joint_id_t.K4ABT_JOINT_WRIST_LEFT,
            k4abt_joint_id_t.K4ABT_JOINT_WRIST_RIGHT,
            k4abt_joint_id_t.K4ABT_JOINT_HEAD,
        };
        private int[] _virtualSrcBone; // per virtual bone: index of the real bone it extends
        private int _realBoneCount;

        // Per-virtual-bone extension length (m): the crown uses its own, shorter knob.
        private float VirtualTipLength(int i)
            => VirtualTipSources[i] == k4abt_joint_id_t.K4ABT_JOINT_HEAD ? crownExtension : handExtension;

        // GPU mirror of the ring, consumed by the motion-curves compute. Sample is 5 * Vector3 = 60
        // bytes, tightly packed, matching an HLSL struct of 5 float3. Per bone b, samples are written
        // oldest->newest into _histBuf[b*K .. b*K + count-1]; _countBuf[b] holds that count so the
        // compute never touches stale slots and needs no ring-index math. Rebuilt each LateUpdate.
        private GraphicsBuffer _histBuf;   // StructuredBuffer<Sample>, length boneCount*K
        private GraphicsBuffer _countBuf;  // StructuredBuffer<uint>, length boneCount
        private Sample[] _histScratch;
        private int[] _countScratch;

        /// <summary>Per-bone stabilized pose history, laid out [bone*RingLength + sample], oldest first.</summary>
        public GraphicsBuffer HistoryBuffer => _histBuf;
        /// <summary>Valid sample count per bone (uint), length <see cref="BoneCount"/>.</summary>
        public GraphicsBuffer CountBuffer => _countBuf;
        /// <summary>Ring length K: the per-bone stride into <see cref="HistoryBuffer"/>.</summary>
        public int RingLength => _ringLen;

        private const float Eps = 1e-4f;
        private const float MinPerpSin = 0.34f; // sin(~20deg): reject references too near the bone axis
        private const int MaxK = 32;            // ring length; also the per-curve cap (MAXK) in the compute

        /// <summary>How many of the newest captured frames the curve should span (the draw-time length,
        /// 2..MaxK). The ring always holds MaxK, so this can be raised or lowered — even while paused —
        /// to lengthen/shorten the curves out of the already-captured history.</summary>
        public int CurveSamples => Mathf.Clamp(historySamples, 2, MaxK);

        private void OnEnable()
        {
            if (bodyTracking == null) bodyTracking = FindFirstObjectByType<SkeletonMerger>();
            if (recorder == null) recorder = FindFirstObjectByType<SensorRecorder>();
            EnsureBuffers();
            if (recorder != null) recorder.OnPlaybackLooped += HandlePlaybackLooped;
        }

        private void OnDisable()
        {
            if (recorder != null) recorder.OnPlaybackLooped -= HandlePlaybackLooped;
            ReleaseGpu();
        }
        private void OnDestroy()
        {
            if (recorder != null) recorder.OnPlaybackLooped -= HandlePlaybackLooped;
            ReleaseGpu();
        }

        // Playback wrapped to the start: the pose jumps discontinuously backward. Reset
        // every bone's ring so a curve can't span the loop seam (which would draw a long
        // garbage streak from the last loop-1 pose to the first loop-2 pose).
        private void HandlePlaybackLooped()
        {
            ResetAll();
            PublishGpu();
        }

        private void ReleaseGpu()
        {
            _histBuf?.Release(); _histBuf = null;
            _countBuf?.Release(); _countBuf = null;
        }

        private void EnsureBuffers()
        {
            var bones = BodyTrackingShared.Bones;
            int real = bones.Length;
            int n = real + VirtualTipSources.Length; // + virtual tip bones (hands, crown)
            // The ring is ALWAYS MAXK frames. historySamples doesn't size it — it selects, at draw time,
            // how many of the newest captured frames the curve uses (see CurveSamples). So tuning the
            // length (even while paused) shortens OR lengthens the curves instantly out of the already-
            // captured 32 frames, with no reallocation and no clearing.
            int k = MaxK;
            // Must also require live GPU buffers: OnDisable releases them (nulling _histBuf/_countBuf)
            // without touching the CPU ring, so on re-enable the shape still matches — without this
            // check EnsureBuffers would early-return and leave HistoryBuffer/CountBuffer null forever,
            // silently killing every downstream consumer.
            if (_ring != null && _boneCount == n && _ringLen == k && _histBuf != null) return;

            _realBoneCount = real;
            _boneCount = n;
            _ringLen = k;
            _ring = new Sample[n][];
            _head = new int[n];
            _count = new int[n];
            _prevV = new Vector3[n];
            _scratch = new SkeletonMerger.BoneEndpoints[real]; // merger only fills the real bones
            for (int b = 0; b < n; b++)
            {
                _ring[b] = new Sample[k];
                _head[b] = -1;
                _count[b] = 0;
                _prevV[b] = Vector3.zero;
            }

            // Parent-bone table: for bone (a,b), find the bone whose child joint (.b) == a.
            // Its direction is the stable-frame reference axis; degenerate/absent -> torso-up.
            _parentBone = new int[n];
            for (int b = 0; b < real; b++)
            {
                _parentBone[b] = -1;
                var aJoint = bones[b].a;
                for (int p = 0; p < real; p++)
                {
                    if (p != b && bones[p].b == aJoint) { _parentBone[b] = p; break; }
                }
            }

            // Virtual bone -> its source (the real bone whose child joint is that tip:
            // forearm for the wrists, neck->head for the crown).
            _virtualSrcBone = new int[VirtualTipSources.Length];
            for (int i = 0; i < VirtualTipSources.Length; i++)
            {
                _virtualSrcBone[i] = -1;
                for (int b = 0; b < real; b++)
                    if (bones[b].b == VirtualTipSources[i]) { _virtualSrcBone[i] = b; break; }
                _parentBone[real + i] = _virtualSrcBone[i];
            }

            // GPU mirror sized to the current layout; reallocated only when boneCount changes.
            ReleaseGpu();
            _histScratch = new Sample[n * k];
            _countScratch = new int[n];
            _histBuf = new GraphicsBuffer(GraphicsBuffer.Target.Structured, n * k, System.Runtime.InteropServices.Marshal.SizeOf<Sample>());
            _countBuf = new GraphicsBuffer(GraphicsBuffer.Target.Structured, n, sizeof(int));
            // Zero-initialise: a fresh GraphicsBuffer holds undefined data, and a consumer can dispatch
            // before our first PublishGpu (Update runs before our LateUpdate; or we start paused and the
            // LateUpdate pause early-return skips PublishGpu). Publishing the all-zero scratch now means
            // counts read as 0 (empty bones) instead of garbage that would drive out-of-bounds indexing.
            _countBuf.SetData(_countScratch);
            _histBuf.SetData(_histScratch);
        }

        private void LateUpdate()
        {
            EnsureBuffers();
            // Ingest a sample only when a genuinely new body frame arrived (PoseVersion changed). This
            // holds the ring steady while the playhead is static (paused, no stepping) — avoiding the
            // repeated-identical-sample collapse — yet still updates on live playback AND on paused
            // frame-stepping (each step ingests one new frame), so the curves follow the stepped pose.
            bool newFrame = true;
            if (bodyTracking != null)
            {
                ulong v = bodyTracking.PoseVersion;
                newFrame = !_havePoseVersion || v != _lastPoseVersion;
                _lastPoseVersion = v;
                _havePoseVersion = true;
            }

            if (newFrame)
            {
                // New pose (play / frame-step): push fresh samples, reset bones that went invalid/stale.
                UpdateHistory();
                PublishGpu();
            }
            else if (recorder == null || !recorder.IsPaused)
            {
                // Static version but NOT an intentional pause => the body stalled or left. Clear bones
                // whose freshness expired (or all, if no active visual) WITHOUT appending duplicate
                // samples, so a ghost can't linger past freshWindowFrames while the stale BodyVisual is
                // still alive. (TryReadBonePosesWorld stays true for a live-but-stale visual, so a
                // freshness check — not just its bool — is what clears here.)
                ClearStale();
                PublishGpu();
            }
            // else: intentional pause (recorder.IsPaused) with no new frame -> hold the ring so the
            // frozen sculpture stays put even after the joints age out (no collapse, no premature clear).
        }

        private void UpdateHistory()
        {
            if (bodyTracking == null) { ResetAll(); return; }
            if (!bodyTracking.TryReadBonePosesWorld(_scratch, freshWindowFrames)) { ResetAll(); return; }

            Vector3 torsoUp = ComputeTorsoUp();
            Vector3 torsoRight = ComputeTorsoRight(torsoUp);
            Vector3 torsoFwd = Vector3.Cross(torsoRight, torsoUp);
            if (torsoFwd.sqrMagnitude < Eps * Eps) torsoFwd = Vector3.forward;
            torsoFwd.Normalize();

            for (int b = 0; b < _realBoneCount; b++)
            {
                var e = _scratch[b];
                if (!e.Valid || !e.Fresh) { ResetBone(b); continue; }

                Vector3 ab = e.B - e.A;
                if (ab.sqrMagnitude < Eps * Eps) { ResetBone(b); continue; }
                Vector3 u = ab.normalized;

                // Secondary axis: a reference direction projected onto the plane perpendicular to u.
                // The reference MUST be well off the bone axis, otherwise the perpendicular residual is
                // noise and V rolls arbitrarily frame to frame (spine bones run parallel to torso-up and
                // did exactly this). PickPerp rejects any reference within ~20deg of u; we then fall
                // through body axes that are guaranteed transverse to the spine (left/right, then fwd)
                // BEFORE ever reusing prevV/world axes (which reintroduce the roll).
                Vector3 refDir = ReferenceAxis(b, torsoUp);
                Vector3 v = PickPerp(refDir, u);
                if (v == Vector3.zero) v = PickPerp(torsoRight, u);
                if (v == Vector3.zero) v = PickPerp(torsoFwd, u);
                if (v == Vector3.zero) v = PickPerp(_prevV[b], u);
                if (v == Vector3.zero) v = ProjectPerp(Vector3.up, u);
                if (v.sqrMagnitude < Eps * Eps) v = ProjectPerp(Vector3.right, u);
                v.Normalize();
                Vector3 w = Vector3.Cross(u, v).normalized;

                Push(b, new Sample { A = e.A, B = e.B, U = u, V = v, W = w });
                _prevV[b] = v;
            }

            // Virtual tip bones: rigid extensions sharing their source bone's frame.
            // A source that failed above was reset (count 0), so count > 0 here means
            // its newest ring sample is from THIS frame — extend that one.
            for (int i = 0; i < _virtualSrcBone.Length; i++)
            {
                int vb = _realBoneCount + i;
                int f = _virtualSrcBone[i];
                float len = VirtualTipLength(i);
                if (f < 0 || _count[f] == 0 || len <= Eps) { ResetBone(vb); continue; }
                Sample s = _ring[f][_head[f]];
                Push(vb, new Sample { A = s.B, B = s.B + s.U * len, U = s.U, V = s.V, W = s.W });
            }
        }

        private Vector3 ComputeTorsoUp()
        {
            if (bodyTracking != null
                && bodyTracking.TryGetJointWorld(k4abt_joint_id_t.K4ABT_JOINT_PELVIS, out var pelvis)
                && bodyTracking.TryGetJointWorld(k4abt_joint_id_t.K4ABT_JOINT_NECK, out var neck))
            {
                Vector3 up = neck - pelvis;
                if (up.sqrMagnitude > Eps * Eps) return up.normalized;
            }
            return Vector3.up;
        }

        // Transverse body axis (person's left<->right), used as the well-conditioned reference for
        // bones that run parallel to torso-up (the whole spine). Shoulders preferred, hips as backup;
        // projected perpendicular to torsoUp so it is a clean horizontal. World fallback if neither pair
        // is available.
        private Vector3 ComputeTorsoRight(Vector3 torsoUp)
        {
            Vector3 span;
            if (TryJointSpan(k4abt_joint_id_t.K4ABT_JOINT_SHOULDER_LEFT,
                             k4abt_joint_id_t.K4ABT_JOINT_SHOULDER_RIGHT, out span)
             || TryJointSpan(k4abt_joint_id_t.K4ABT_JOINT_HIP_LEFT,
                             k4abt_joint_id_t.K4ABT_JOINT_HIP_RIGHT, out span))
            {
                Vector3 p = span - torsoUp * Vector3.Dot(span, torsoUp);
                if (p.sqrMagnitude > Eps * Eps) return p.normalized;
            }
            Vector3 f = Vector3.Cross(torsoUp, Vector3.forward);
            if (f.sqrMagnitude < Eps * Eps) f = Vector3.Cross(torsoUp, Vector3.right);
            return f.sqrMagnitude > Eps * Eps ? f.normalized : Vector3.right;
        }

        private bool TryJointSpan(k4abt_joint_id_t a, k4abt_joint_id_t b, out Vector3 span)
        {
            span = Vector3.zero;
            if (bodyTracking != null
                && bodyTracking.TryGetJointWorld(a, out var pa)
                && bodyTracking.TryGetJointWorld(b, out var pb))
            {
                span = pb - pa;
                return span.sqrMagnitude > Eps * Eps;
            }
            return false;
        }

        private Vector3 ReferenceAxis(int bone, Vector3 torsoUp)
        {
            int p = _parentBone[bone];
            if (p >= 0)
            {
                var pe = _scratch[p];
                if (pe.Valid)
                {
                    Vector3 pd = pe.B - pe.A;
                    if (pd.sqrMagnitude > Eps * Eps) return pd.normalized;
                }
            }
            return torsoUp;
        }

        private static Vector3 ProjectPerp(Vector3 dir, Vector3 u)
        {
            return dir - u * Vector3.Dot(dir, u);
        }

        // Project dir perpendicular to the unit vector u, but reject it (return zero) when dir sits
        // within ~MinPerpSin of the bone axis. |dir - u*(dir.u)| == |dir|*sin(angle), so the residual
        // must exceed MinPerpSin*|dir| for the direction to be a stable, non-noise-dominated reference.
        private static Vector3 PickPerp(Vector3 dir, Vector3 u)
        {
            if (dir.sqrMagnitude < Eps * Eps) return Vector3.zero;
            Vector3 p = dir - u * Vector3.Dot(dir, u);
            if (p.sqrMagnitude < MinPerpSin * MinPerpSin * dir.sqrMagnitude) return Vector3.zero;
            return p;
        }

        private void Push(int bone, Sample s)
        {
            int head = _head[bone] + 1;
            if (head >= _ringLen) head = 0;
            _ring[bone][head] = s;
            _head[bone] = head;
            if (_count[bone] < _ringLen) _count[bone]++;
        }

        private void ResetBone(int bone)
        {
            _count[bone] = 0;
            _head[bone] = -1;
            _prevV[bone] = Vector3.zero;
        }

        private void ResetAll()
        {
            if (_ring == null) return;
            for (int b = 0; b < _boneCount; b++) ResetBone(b);
        }

        // Reset bones no longer valid/fresh WITHOUT appending a sample. Called on a non-paused static
        // frame so a stalled/lost body clears at freshWindowFrames rather than lingering as a ghost
        // until its BodyVisual is destroyed. Fresh bones are left held (they clear once they too age out).
        private void ClearStale()
        {
            if (bodyTracking == null || !bodyTracking.TryReadBonePosesWorld(_scratch, freshWindowFrames))
            {
                ResetAll();
                return;
            }
            for (int b = 0; b < _realBoneCount; b++)
            {
                var e = _scratch[b];
                if (!e.Valid || !e.Fresh) ResetBone(b);
            }
            // A virtual tip bone clears exactly when its source bone does.
            for (int i = 0; i < _virtualSrcBone.Length; i++)
            {
                int f = _virtualSrcBone[i];
                if (f < 0 || _count[f] == 0) ResetBone(_realBoneCount + i);
            }
        }

        // Ring index of the oldest of a bone's newest <count> samples: head - (count-1), wrapped.
        private int OldestIndex(int bone, int count)
        {
            int idx = _head[bone] - (count - 1);
            idx %= _ringLen;
            if (idx < 0) idx += _ringLen;
            return idx;
        }

        // Advance a ring index by one, wrapping at K.
        private int NextIndex(int idx)
        {
            idx++;
            if (idx >= _ringLen) idx = 0;
            return idx;
        }

        // Mirror the CPU ring to the GPU: per bone, copy its valid samples oldest->newest into the
        // contiguous [bone*K ..] slice and record the count. Called every LateUpdate (including reset
        // frames, so the compute sees empty bones as count 0 rather than stale data).
        private void PublishGpu()
        {
            if (_histBuf == null || _countBuf == null || _histScratch == null) return;
            for (int b = 0; b < _boneCount; b++)
            {
                int cnt = _count[b];
                _countScratch[b] = cnt;
                if (cnt == 0) continue;
                int baseIdx = b * _ringLen;
                int idx = OldestIndex(b, cnt);
                for (int i = 0; i < cnt; i++)
                {
                    _histScratch[baseIdx + i] = _ring[b][idx];
                    idx = NextIndex(idx);
                }
            }
            _histBuf.SetData(_histScratch);
            _countBuf.SetData(_countScratch);
        }

        /// <summary>Reproject a point given in bone <paramref name="bone"/>'s CURRENT stable frame
        /// (segment param <paramref name="t"/> along a->b, offsets <paramref name="cv"/>/<paramref name="cw"/>
        /// on v/w) through the stored history. Writes control points oldest-first into
        /// <paramref name="dst"/> and returns the count written (0 if the bone has no history).</summary>
        public int SampleCurve(int bone, float t, float cv, float cw, Vector3[] dst)
        {
            if (_ring == null || bone < 0 || bone >= _boneCount || dst == null) return 0;
            int count = Mathf.Min(_count[bone], dst.Length);
            if (count == 0) return 0;
            // Walk oldest -> newest.
            int idx = OldestIndex(bone, count);
            for (int i = 0; i < count; i++)
            {
                Sample s = _ring[bone][idx];
                dst[i] = Vector3.LerpUnclamped(s.A, s.B, t) + cv * s.V + cw * s.W;
                idx = NextIndex(idx);
            }
            return count;
        }

        /// <summary>Number of stored samples for a bone (0..K).</summary>
        public int SampleCount(int bone)
            => (_count != null && bone >= 0 && bone < _boneCount) ? _count[bone] : 0;

        /// <summary>World-space AABB over every bone's newest endpoints (the current pose extent).
        /// Returns false if no bone has history. Used to concentrate point-cloud seeds on the body.</summary>
        public bool TryGetWorldBounds(out Bounds bounds)
        {
            bounds = default;
            if (_ring == null) return false;
            Vector3 mn = new Vector3(1e9f, 1e9f, 1e9f), mx = -mn;
            bool any = false;
            for (int b = 0; b < _boneCount; b++)
            {
                if (_count[b] == 0) continue;
                Sample s = _ring[b][_head[b]]; // newest
                mn = Vector3.Min(mn, Vector3.Min(s.A, s.B));
                mx = Vector3.Max(mx, Vector3.Max(s.A, s.B));
                any = true;
            }
            if (!any) return false;
            bounds = new Bounds((mn + mx) * 0.5f, mx - mn);
            return true;
        }

        public int BoneCount => _boneCount;

        private Vector3[] _gizmoBuf;

        private void OnDrawGizmos()
        {
            if (!drawGizmos || !Application.isPlaying || _ring == null) return;
            if (_gizmoBuf == null || _gizmoBuf.Length < _ringLen) _gizmoBuf = new Vector3[Mathf.Max(2, _ringLen)];
            Gizmos.color = gizmoColor;
            for (int b = 0; b < _boneCount; b++)
            {
                // Demo curve at mid-bone, offset on +v so bone rotation shows as an arc.
                int n = SampleCurve(b, 0.5f, gizmoOffset, 0f, _gizmoBuf);
                for (int i = 1; i < n; i++) Gizmos.DrawLine(_gizmoBuf[i - 1], _gizmoBuf[i]);
                if (n > 0) Gizmos.DrawSphere(_gizmoBuf[n - 1], 0.01f);
            }
        }
    }
}
