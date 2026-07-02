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

namespace BodyTracking
{
    [DisallowMultipleComponent]
    public class BonePoseHistory : MonoBehaviour
    {
        [Tooltip("Body-tracking source. Leave empty to auto-resolve the first SkeletonMerger at OnEnable.")]
        public SkeletonMerger bodyTracking;

        [Min(2)]
        [Tooltip("Ring-buffer length K: how many past frames form each bone's curve.")]
        public int historySamples = 12;

        [Min(1)]
        [Tooltip("An endpoint is treated as stale (bone history reset) if its last fresh BT " +
                 "frame is older than this many frames. Guards against dragging held/predicted joints.")]
        public int freshWindowFrames = 12;

        [Header("Debug gizmos (Phase 1 validation)")]
        [Tooltip("Draw each bone's reprojected curve in the Scene view to verify arcs form.")]
        public bool drawGizmos = true;

        [Range(0f, 0.3f)]
        [Tooltip("Perpendicular offset (on +v) of the demo curve drawn per bone. Non-zero makes " +
                 "rotation show up as an arc rather than a straight midpoint trail.")]
        public float gizmoOffset = 0.12f;

        public Color gizmoColor = new Color(1f, 0.55f, 0.1f, 1f);

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

        private const float Eps = 1e-4f;

        private void OnEnable()
        {
            if (bodyTracking == null) bodyTracking = FindFirstObjectByType<SkeletonMerger>();
            EnsureBuffers();
        }

        private void EnsureBuffers()
        {
            var bones = BodyTrackingShared.Bones;
            int n = bones.Length;
            int k = Mathf.Max(2, historySamples);
            if (_ring != null && _boneCount == n && _ringLen == k) return;

            _boneCount = n;
            _ringLen = k;
            _ring = new Sample[n][];
            _head = new int[n];
            _count = new int[n];
            _prevV = new Vector3[n];
            _scratch = new SkeletonMerger.BoneEndpoints[n];
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
            for (int b = 0; b < n; b++)
            {
                _parentBone[b] = -1;
                var aJoint = bones[b].a;
                for (int p = 0; p < n; p++)
                {
                    if (p != b && bones[p].b == aJoint) { _parentBone[b] = p; break; }
                }
            }
        }

        private void LateUpdate()
        {
            EnsureBuffers();
            if (bodyTracking == null) { ResetAll(); return; }
            if (!bodyTracking.TryReadBonePosesWorld(_scratch, freshWindowFrames)) { ResetAll(); return; }

            Vector3 torsoUp = ComputeTorsoUp();

            for (int b = 0; b < _boneCount; b++)
            {
                var e = _scratch[b];
                if (!e.Valid || !e.Fresh) { ResetBone(b); continue; }

                Vector3 ab = e.B - e.A;
                if (ab.sqrMagnitude < Eps * Eps) { ResetBone(b); continue; }
                Vector3 u = ab.normalized;

                // Secondary axis: reference direction projected onto the plane perpendicular to u,
                // with a fallback chain that keeps roll continuous (Codex-approved).
                Vector3 refDir = ReferenceAxis(b, torsoUp);
                Vector3 v = ProjectPerp(refDir, u);
                if (v.sqrMagnitude < Eps * Eps) v = ProjectPerp(_prevV[b], u);
                if (v.sqrMagnitude < Eps * Eps) v = ProjectPerp(Vector3.up, u);
                if (v.sqrMagnitude < Eps * Eps) v = ProjectPerp(Vector3.right, u);
                v.Normalize();
                Vector3 w = Vector3.Cross(u, v).normalized;

                Push(b, new Sample { A = e.A, B = e.B, U = u, V = v, W = w });
                _prevV[b] = v;
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

        /// <summary>Reproject a point given in bone <paramref name="bone"/>'s CURRENT stable frame
        /// (segment param <paramref name="t"/> along a->b, offsets <paramref name="cv"/>/<paramref name="cw"/>
        /// on v/w) through the stored history. Writes control points oldest-first into
        /// <paramref name="dst"/> and returns the count written (0 if the bone has no history).</summary>
        public int SampleCurve(int bone, float t, float cv, float cw, Vector3[] dst)
        {
            if (_ring == null || bone < 0 || bone >= _boneCount || dst == null) return 0;
            int count = Mathf.Min(_count[bone], dst.Length);
            if (count == 0) return 0;
            // Walk oldest -> newest. Oldest index = head - (count-1) wrapped.
            int idx = _head[bone] - (count - 1);
            idx %= _ringLen;
            if (idx < 0) idx += _ringLen;
            for (int i = 0; i < count; i++)
            {
                Sample s = _ring[bone][idx];
                dst[i] = Vector3.LerpUnclamped(s.A, s.B, t) + cv * s.V + cw * s.W;
                idx++;
                if (idx >= _ringLen) idx = 0;
            }
            return count;
        }

        /// <summary>Number of stored samples for a bone (0..K).</summary>
        public int SampleCount(int bone)
            => (_count != null && bone >= 0 && bone < _boneCount) ? _count[bone] : 0;

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
