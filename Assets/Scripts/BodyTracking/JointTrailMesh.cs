// Per-joint quad-strip trail mesh. Replaces Unity's TrailRenderer in BodyVisual
// so we can paint per-vertex acceleration heatmaps (TrailRenderer only supports
// a single time-axis colorGradient). Behavior parity with TrailRenderer:
// camera-aligned billboard, width tapering from head to tail, sample TTL by
// trailDuration. Sample positions are kept in the joint's parent-local space so
// the trail follows skeleton transforms exactly like the sphere joints do.

using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering;

namespace BodyTracking
{
    internal sealed class JointTrailMesh
    {
        public enum ColorMode { Base, AccelHeatmap }

        private struct Sample
        {
            public double Time;
            public Vector3 Pos;
            public float Accel;
        }

        private readonly GameObject _go;
        private readonly Mesh _mesh;
        private readonly MeshFilter _mf;
        private readonly MeshRenderer _mr;
        private readonly List<Sample> _samples = new List<Sample>();

        private Vector3[] _verts;
        private Color[] _colors;
        private int[] _tris;
        private float[] _medianAccels;

        // 5-window median filter scratch shared across all trail instances. Single-
        // threaded usage (Rebuild runs on the main thread) so a static buffer is fine.
        private static readonly float[] s_medBuf = new float[5];

        private float _duration = 2f;
        private float _width = 0.005f;
        private Color _baseColor = Color.white;
        private Color _hotColor = Color.red;
        private float _accelMin = 0f;
        private float _accelMax = 56f;
        private ColorMode _mode = ColorMode.Base;
        private bool _show = true;

        // Safety cap: at 30 Hz with trailDuration=20s we expect ~600 samples.
        // 4096 gives generous headroom for longer durations / higher Editor frame
        // rates while still bounding memory (a few MB per body across 32 joints).
        private const int kMaxSamples = 4096;

        /// <summary>|a| computed for the most recent AddSample call. Zero until 3 samples are seen.</summary>
        public float LastAccel { get; private set; }

        /// <summary>Override the accelMax set via Configure. Called by BodyVisualPool when
        /// autoAccelMax is on so the rolling p95 controls the heatmap's hot end.</summary>
        public void SetAccelMax(float v) { _accelMax = v; }

        public JointTrailMesh(Transform parent, string name, Material mat)
        {
            _go = new GameObject(name);
            _go.transform.SetParent(parent, false);
            _mf = _go.AddComponent<MeshFilter>();
            _mr = _go.AddComponent<MeshRenderer>();
            // Don't cast shadows — the trail rebuilds every frame with thin
            // taper-to-zero geometry, which produces flickery self-shadows.
            // Do receive shadows so the trail darkens consistently with the
            // rest of the scene under the TrailLit PBR shader.
            _mr.shadowCastingMode = ShadowCastingMode.Off;
            _mr.receiveShadows = true;
            _mr.sharedMaterial = mat;
            _mesh = new Mesh { name = name + ".trail", indexFormat = IndexFormat.UInt32 };
            _mesh.MarkDynamic();
            _mf.sharedMesh = _mesh;
        }

        public void Configure(bool show, float duration, float width,
                              Color baseColor, Color hotColor,
                              float accelMin, float accelMax, ColorMode mode)
        {
            _show = show;
            _duration = Mathf.Max(0.0001f, duration);
            _width = Mathf.Max(0f, width);
            _baseColor = baseColor;
            _hotColor = hotColor;
            _accelMin = accelMin;
            _accelMax = accelMax;
            _mode = mode;
            _mr.enabled = show;
        }

        public void Clear()
        {
            _samples.Clear();
            if (_mesh != null) _mesh.Clear();
        }

        // Skip threshold: the joint must have moved at least this far in local
        // space since the last sample. Without this, every Unity Update tick
        // re-feeds the unchanged joint position (Update runs at ~60–90 Hz while
        // k4abt pops at ~30 Hz) and the ring buffer fills with duplicates,
        // pushing real history off the front of the trail before trailDuration
        // expires. Matches Unity TrailRenderer.minVertexDistance (0.005m).
        private const float kMinVertexDistance = 0.005f;

        public void AddSample(double t, Vector3 localPos)
        {
            int n = _samples.Count;
            if (n > 0 && (localPos - _samples[n - 1].Pos).sqrMagnitude < kMinVertexDistance * kMinVertexDistance)
                return;

            // Compute |a| using the previous two samples (central diff: between
            // the two previous segments) so the value is associated with the
            // sample that already has neighbors on both sides.
            float accel = 0f;
            if (n >= 2)
            {
                var s0 = _samples[n - 2];
                var s1 = _samples[n - 1];
                float dt1 = (float)(s1.Time - s0.Time);
                float dt2 = (float)(t - s1.Time);
                if (dt1 > 0f && dt2 > 0f)
                {
                    Vector3 vel1 = (s1.Pos - s0.Pos) / dt1;
                    Vector3 vel2 = (localPos - s1.Pos) / dt2;
                    accel = ((vel2 - vel1) / (0.5f * (dt1 + dt2))).magnitude;
                }
            }
            _samples.Add(new Sample { Time = t, Pos = localPos, Accel = accel });
            LastAccel = accel;

            DropOldSamples(t);
            if (_samples.Count > kMaxSamples)
                _samples.RemoveRange(0, _samples.Count - kMaxSamples);
        }

        public void Rebuild(double currentTime, Camera cam)
        {
            if (!_show) { _mesh.Clear(); return; }
            DropOldSamples(currentTime);

            int n = _samples.Count;
            if (n < 2) { _mesh.Clear(); return; }

            if (_verts == null || _verts.Length != n * 2)
            {
                _verts = new Vector3[n * 2];
                _colors = new Color[n * 2];
                _tris = new int[(n - 1) * 6];
            }
            if (_medianAccels == null || _medianAccels.Length < n)
                _medianAccels = new float[Mathf.Max(n, 64)];

            // 5-window centered median over Sample.Accel so isolated central-difference
            // spikes (id swap / brief tracking loss) don't dominate the heatmap.
            for (int i = 0; i < n; i++)
            {
                int lo = Mathf.Max(0, i - 2);
                int hi = Mathf.Min(n - 1, i + 2);
                int len = hi - lo + 1;
                for (int k = 0; k < len; k++) s_medBuf[k] = _samples[lo + k].Accel;
                System.Array.Sort(s_medBuf, 0, len);
                _medianAccels[i] = s_medBuf[len / 2];
            }

            // Billboard offset axis = perpendicular to (tangent, camera-forward).
            // Compute camera forward in the local space of _go so offsets compose
            // correctly with the parent transform hierarchy.
            Vector3 camFwdLocal = cam != null
                ? _go.transform.InverseTransformDirection(cam.transform.forward)
                : Vector3.forward;

            for (int i = 0; i < n; i++)
            {
                var s = _samples[i];
                double age = currentTime - s.Time;
                float headness = Mathf.Clamp01(1f - (float)(age / _duration));
                float w = _width * headness;

                Vector3 tangent;
                if (n == 1)             tangent = Vector3.right;
                else if (i == 0)        tangent = _samples[1].Pos - s.Pos;
                else if (i == n - 1)    tangent = s.Pos - _samples[n - 2].Pos;
                else                    tangent = _samples[i + 1].Pos - _samples[i - 1].Pos;
                tangent = tangent.sqrMagnitude > 1e-12f ? tangent.normalized : Vector3.right;

                Vector3 right = Vector3.Cross(tangent, camFwdLocal);
                if (right.sqrMagnitude < 1e-12f) right = Vector3.Cross(tangent, Vector3.up);
                right = right.sqrMagnitude > 1e-12f ? right.normalized : Vector3.right;

                _verts[i * 2 + 0] = s.Pos - right * w;
                _verts[i * 2 + 1] = s.Pos + right * w;

                Color c;
                if (_mode == ColorMode.AccelHeatmap)
                {
                    float t = Mathf.Clamp01(Mathf.InverseLerp(_accelMin, _accelMax, _medianAccels[i]));
                    c = Color.Lerp(_baseColor, _hotColor, t);
                }
                else
                {
                    c = _baseColor;
                }
                c.a *= headness;
                _colors[i * 2 + 0] = c;
                _colors[i * 2 + 1] = c;
            }

            for (int i = 0; i < n - 1; i++)
            {
                int v = i * 2;
                int t = i * 6;
                _tris[t + 0] = v + 0;
                _tris[t + 1] = v + 1;
                _tris[t + 2] = v + 2;
                _tris[t + 3] = v + 2;
                _tris[t + 4] = v + 1;
                _tris[t + 5] = v + 3;
            }

            _mesh.Clear();
            _mesh.vertices = _verts;
            _mesh.colors = _colors;
            _mesh.triangles = _tris;
            // Smoothed per-vertex normals so the TrailLit (URP Lit) shader has
            // something to light against. Cheap enough at the trail's vertex
            // counts (~1k verts/joint at the kMaxSamples ceiling), and works
            // for both the camera-aligned quad strip geometry and any future
            // tube swap-in without bespoke per-topology normal math.
            _mesh.RecalculateNormals();

            // Replace LastAccel with the medianed value at the head so the pool's
            // rolling p95 window reads the same value the vertex colors use.
            LastAccel = _medianAccels[n - 1];
        }

        public void Destroy()
        {
            if (_mesh != null) Object.Destroy(_mesh);
            if (_go != null) Object.Destroy(_go);
        }

        private void DropOldSamples(double currentTime)
        {
            double cutoff = currentTime - _duration;
            int removeUpTo = 0;
            while (removeUpTo < _samples.Count && _samples[removeUpTo].Time < cutoff) removeUpTo++;
            if (removeUpTo > 0) _samples.RemoveRange(0, removeUpTo);
        }
    }
}
