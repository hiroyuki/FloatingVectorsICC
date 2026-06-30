// Per-joint tube trail mesh. Replaces Unity's TrailRenderer in BodyVisual so
// we can paint per-vertex acceleration heatmaps (TrailRenderer only supports
// a single time-axis colorGradient). Each sample emits a ring of kTubeSides
// vertices around the centerline; consecutive rings are stitched with quads
// to form a closed tube. Frames are propagated by parallel transport so the
// tube does not twist as the curve bends. Behavior parity with the previous
// quad-strip: width (= tube radius here) tapers from head to tail, sample TTL
// by trailDuration, vertex-color heatmap. Sample positions stay in the joint's
// parent-local space so the trail follows skeleton transforms exactly like
// the sphere joints do.

using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering;

namespace BodyTracking
{
    internal sealed class JointTrailMesh
    {
        public enum ColorMode { Base, AccelHeatmap, FrameHue }

        private struct Sample
        {
            public double Time;
            public Vector3 Pos;
            public float Accel;
            // Frame counter at the moment AddSample was called. Used by
            // ColorMode.FrameHue so each ring of the tube paints the hue that
            // was current when *that* sample was captured — the tube becomes
            // a gradient from oldest to newest, instead of recoloring as a
            // single block every frame.
            public int Frame;
        }

        private readonly GameObject _go;
        private readonly Mesh _mesh;
        private readonly MeshFilter _mf;
        private readonly MeshRenderer _mr;
        private readonly List<Sample> _samples = new List<Sample>();

        private Vector3[] _verts;
        private Vector3[] _normals;
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
        private BodyTrackingShared.FrameHueParams _frameHue;

        // Safety cap: at 30 Hz with trailDuration=20s we expect ~600 samples.
        // 4096 gives generous headroom for longer durations / higher Editor frame
        // rates while still bounding memory (a few MB per body across 32 joints).
        private const int kMaxSamples = 4096;

        /// <summary>|a| computed for the most recent AddSample call. Zero until 3 samples are seen.</summary>
        public float LastAccel { get; private set; }

        /// <summary>Number of windowed centerline samples currently retained (oldest→newest).</summary>
        public int SampleCount => _samples.Count;

        /// <summary>Sample i's position in the trail's PARENT-LOCAL space (caller maps to world).
        /// Used by the SDF trail baker to read the ribbon centerline.</summary>
        public Vector3 SampleLocalPos(int i) => _samples[i].Pos;

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
                              float accelMin, float accelMax, ColorMode mode,
                              BodyTrackingShared.FrameHueParams frameHue = default)
        {
            _show = show;
            _duration = Mathf.Max(0.0001f, duration);
            _width = Mathf.Max(0f, width);
            _baseColor = baseColor;
            _hotColor = hotColor;
            _accelMin = accelMin;
            _accelMax = accelMax;
            _mode = mode;
            _frameHue = frameHue;
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

        public void AddSample(double t, Vector3 localPos, int frame)
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
            _samples.Add(new Sample { Time = t, Pos = localPos, Accel = accel, Frame = frame });
            LastAccel = accel;

            DropOldSamples(t);
            if (_samples.Count > kMaxSamples)
                _samples.RemoveRange(0, _samples.Count - kMaxSamples);
        }

        // Number of sides around the tube's circular cross-section. 12 reads as
        // smoothly round under PBR lighting — the previous 6 left a visible
        // hex silhouette at typical viewing distances. Inter-sample index
        // count is kTubeSides × 6.
        private const int kTubeSides = 12;

        // Pre-computed unit-circle offsets (cos, sin) for each side. Filled once
        // and reused every Rebuild to skip per-vertex trig.
        private static readonly float[] s_ringCos = BuildRingCos();
        private static readonly float[] s_ringSin = BuildRingSin();
        private static float[] BuildRingCos()
        {
            var a = new float[kTubeSides];
            for (int k = 0; k < kTubeSides; k++) a[k] = Mathf.Cos(2f * Mathf.PI * k / kTubeSides);
            return a;
        }
        private static float[] BuildRingSin()
        {
            var a = new float[kTubeSides];
            for (int k = 0; k < kTubeSides; k++) a[k] = Mathf.Sin(2f * Mathf.PI * k / kTubeSides);
            return a;
        }

        public void Rebuild(double currentTime, Camera cam)
        {
            // cam was used by the old billboard quad-strip; tube geometry is
            // camera-independent so we ignore it. The parameter stays for API
            // parity with callers (BodyVisualPool.TickTrails).
            _ = cam;
            if (!_show) { _mesh.Clear(); return; }

            // Clamp "now" to the latest sample's time so the trail's age math
            // freezes when no fresh samples are arriving (playback paused,
            // playback stopped, BT pipeline silent). Without this, Time
            // .timeAsDouble keeps advancing → DropOldSamples eats the front of
            // the trail and headness fades the alpha toward zero even though
            // the user explicitly paused. When samples flow normally, latest
            // is essentially current so this is a no-op.
            if (_samples.Count > 0)
            {
                double latest = _samples[_samples.Count - 1].Time;
                if (currentTime > latest) currentTime = latest;
            }

            DropOldSamples(currentTime);

            int n = _samples.Count;
            if (n < 2) { _mesh.Clear(); return; }

            int vertsPerRing = kTubeSides;
            int totalVerts = n * vertsPerRing;
            int totalTris = (n - 1) * kTubeSides * 6;
            // Grow-only, power-of-two capacity. The trail sample count fluctuates
            // every frame as DropOldSamples / AddSample run, and the previous
            // `Length != totalVerts` test reallocated four arrays per Rebuild —
            // ~200 MB/s of GC churn across all trails at 30 fps. Allocating to
            // the next power of two and only growing keeps the steady-state
            // alloc rate effectively zero. The Mesh upload below passes the
            // actual filled length so unused tail slots are not uploaded.
            if (_verts == null || _verts.Length < totalVerts)
            {
                int cap = _verts == null ? 64 : _verts.Length;
                while (cap < totalVerts) cap <<= 1;
                _verts = new Vector3[cap];
                _normals = new Vector3[cap];
                _colors = new Color[cap];
            }
            if (_tris == null || _tris.Length < totalTris)
            {
                int cap = _tris == null ? 64 : _tris.Length;
                while (cap < totalTris) cap <<= 1;
                _tris = new int[cap];
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

            // Parallel-transport frame: pick an initial 'right' perpendicular to
            // the first tangent, then for every subsequent sample project the
            // previous 'right' onto the plane perpendicular to the new tangent.
            // This keeps the tube from twisting as the curve bends and avoids
            // the discontinuity you get from rebuilding the frame from a fixed
            // up-vector at every sample.
            Vector3 prevRight = Vector3.zero;
            Vector3 prevUp = Vector3.zero;
            bool haveFrame = false;

            for (int i = 0; i < n; i++)
            {
                var s = _samples[i];
                double age = currentTime - s.Time;
                float headness = Mathf.Clamp01(1f - (float)(age / _duration));
                float radius = _width * headness;

                Vector3 tangent;
                if (i == 0)             tangent = _samples[1].Pos - s.Pos;
                else if (i == n - 1)    tangent = s.Pos - _samples[n - 2].Pos;
                else                    tangent = _samples[i + 1].Pos - _samples[i - 1].Pos;
                tangent = tangent.sqrMagnitude > 1e-12f ? tangent.normalized : Vector3.forward;

                Vector3 right, up;
                if (!haveFrame)
                {
                    // Seed with a perpendicular that isn't parallel to tangent.
                    Vector3 seed = Mathf.Abs(Vector3.Dot(tangent, Vector3.up)) < 0.95f
                        ? Vector3.up
                        : Vector3.right;
                    right = Vector3.Cross(tangent, seed);
                    if (right.sqrMagnitude < 1e-12f) right = Vector3.Cross(tangent, Vector3.right);
                    right = right.sqrMagnitude > 1e-12f ? right.normalized : Vector3.right;
                    up = Vector3.Cross(right, tangent);
                    up = up.sqrMagnitude > 1e-12f ? up.normalized : Vector3.up;
                    haveFrame = true;
                }
                else
                {
                    // Project prevRight onto plane perpendicular to new tangent.
                    right = prevRight - tangent * Vector3.Dot(prevRight, tangent);
                    if (right.sqrMagnitude < 1e-12f)
                    {
                        // Fallback if prevRight became parallel to new tangent
                        // (sharp ~180° turn). Use prevUp instead, then reseed
                        // if that also degenerates.
                        right = prevUp - tangent * Vector3.Dot(prevUp, tangent);
                        if (right.sqrMagnitude < 1e-12f)
                        {
                            Vector3 seed = Mathf.Abs(Vector3.Dot(tangent, Vector3.up)) < 0.95f
                                ? Vector3.up
                                : Vector3.right;
                            right = Vector3.Cross(tangent, seed);
                        }
                    }
                    right = right.normalized;
                    up = Vector3.Cross(right, tangent);
                    up = up.sqrMagnitude > 1e-12f ? up.normalized : Vector3.up;
                }
                prevRight = right;
                prevUp = up;

                Color c;
                if (_mode == ColorMode.AccelHeatmap)
                {
                    float t = Mathf.Clamp01(Mathf.InverseLerp(_accelMin, _accelMax, _medianAccels[i]));
                    c = Color.Lerp(_baseColor, _hotColor, t);
                }
                else if (_mode == ColorMode.FrameHue)
                {
                    // Per-ring hue from the frame counter captured when this
                    // sample was added — so the tube is a gradient over the
                    // hues that were current along the trail's lifetime. The
                    // tip (newest sample) shows the current frame's hue; older
                    // rings retain their original hues.
                    Color rgb = BodyTrackingShared.FrameHueRGB(in _frameHue, s.Frame);
                    c = new Color(rgb.r, rgb.g, rgb.b, _baseColor.a);
                }
                else
                {
                    c = _baseColor;
                }
                c.a *= headness;

                int baseIdx = i * vertsPerRing;
                for (int k = 0; k < kTubeSides; k++)
                {
                    float cosk = s_ringCos[k];
                    float sink = s_ringSin[k];
                    // Analytical radial normal: for a circular cross-section the
                    // outward normal at each ring vertex is exactly the unit
                    // radial direction. This is continuous around the ring (so
                    // PBR shading reads as a smooth cylinder instead of the
                    // faceted hex Mesh.RecalculateNormals produced) and is
                    // exact rather than averaged.
                    Vector3 radial = right * cosk + up * sink;
                    _verts[baseIdx + k] = s.Pos + radial * radius;
                    _normals[baseIdx + k] = radial;
                    _colors[baseIdx + k] = c;
                }
            }

            // Stitch consecutive rings with two triangles per side. Wrap the
            // side index modulo kTubeSides to close the tube.
            for (int i = 0; i < n - 1; i++)
            {
                int ringA = i * vertsPerRing;
                int ringB = (i + 1) * vertsPerRing;
                int triBase = i * kTubeSides * 6;
                for (int k = 0; k < kTubeSides; k++)
                {
                    int kNext = (k + 1) % kTubeSides;
                    int t = triBase + k * 6;
                    _tris[t + 0] = ringA + k;
                    _tris[t + 1] = ringB + k;
                    _tris[t + 2] = ringA + kNext;
                    _tris[t + 3] = ringA + kNext;
                    _tris[t + 4] = ringB + k;
                    _tris[t + 5] = ringB + kNext;
                }
            }

            // Upload only the filled prefix [0, totalVerts) / [0, totalTris) of
            // our grow-only capacity buffers; the SetXxx(arr, start, length)
            // overloads do not allocate the way the legacy `mesh.vertices = arr`
            // property setters did.
            _mesh.Clear();
            _mesh.SetVertices(_verts, 0, totalVerts);
            _mesh.SetNormals(_normals, 0, totalVerts);
            _mesh.SetColors(_colors, 0, totalVerts);
            _mesh.SetTriangles(_tris, 0, totalTris, submesh: 0, calculateBounds: true);

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
