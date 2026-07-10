// Hand-dwell selection sphere (Phase 4, Plans/phase4-presence-dwell-plan.md):
// a wireframe sphere (three orthogonal circles) the visitor selects by holding
// a hand inside for dwellSeconds. Idle shows partial arcs; while a hand is
// inside, the arcs sweep toward full circles (the dwell progress bar IS the
// shape). On completion: OnSelected fires once, the wire flashes an HDR glow
// (vertex colour * glowBrightness decaying over glowSeconds — visible bloom
// when Bloom is enabled, still a readable brightness pop without), and the
// optional AudioClip plays. Locked until ResetSphere().
//
// Subclasses WireVisualizationBehaviour (same machinery as BoundingVolume /
// CameraPoseMarker). IMPORTANT: the base's Update() is private — defining
// Update() here would HIDE it and kill the visualization sync. All per-frame
// work therefore lives in the UpdateMesh() hook, which the base calls every
// frame while Show is true (keep showVisualization on; the director toggles
// the whole GameObject instead).

using System;
using System.Collections.Generic;
using PointCloud;
using UnityEngine;
using UnityEngine.Events;

namespace Experience
{
    [DisallowMultipleComponent]
    public class DwellSphere : WireVisualizationBehaviour
    {
        [Tooltip("Hand source. Auto-resolves the first PresenceDetector when empty.")]
        public PresenceDetector presence;

        [Min(0.02f)]
        [Tooltip("Selection radius (m) around this transform's position.")]
        public float radius = 0.18f;

        [Min(0.05f)]
        [Tooltip("Seconds a hand must stay inside to select.")]
        public float dwellSeconds = 1f;

        [Range(1.02f, 1.6f)]
        [Tooltip("Progress ring radius as a multiple of the sphere radius. The sphere " +
                 "wireframe is always complete; the dwell draws this horizontal ring " +
                 "around it from 0 to a full circle — full circle = selected.")]
        public float progressRingScale = 1.15f;

        [Min(8)]
        [Tooltip("Line segments per full circle.")]
        public int segmentsPerCircle = 64;

        [Range(0, 3)]
        [Tooltip("Icosphere subdivision level for the wireframe (Blender's Ico " +
                 "Sphere): 0 = icosahedron (30 edges), 1 = 120, 2 = 480 edges.")]
        public int icoSubdivisions = 1;

        [Header("Colors / glow")]
        public Color idleColor = new Color(1f, 1f, 1f, 0.6f);
        public Color activeColor = new Color(0.55f, 1f, 0.9f, 1f);
        [Min(1f)]
        [Tooltip("HDR brightness multiplier at the moment of selection.")]
        public float glowBrightness = 6f;
        [Min(0.05f)]
        [Tooltip("Glow decay time (s).")]
        public float glowSeconds = 0.4f;

        [Header("Selection")]
        [Tooltip("Played once on selection (slot only — asset comes later).")]
        public AudioClip selectedClip;
        public UnityEvent onSelected;

        [Header("Debug")]
        [Tooltip("Advance the dwell as if a hand were inside (Mac / no-BT runs).")]
        public bool debugForceDwell;

        [Tooltip("Show/hide the wire sphere.")]
        public bool showVisualization = true;

        /// <summary>Fired once per completed dwell (until ResetSphere).</summary>
        public event Action<DwellSphere> OnSelected;

        public float Progress { get; private set; }
        public bool Locked { get; private set; }

        private float _glowRemaining;
        private int _builtSegments = -1; // drawn segment count of the current mesh
        private AudioSource _audio;
        // Own wire material (Experience/DwellWire): mesh vertex colours are
        // UNorm8-clamped, so the HDR glow rides the material's _Tint instead.
        private Material _wireMat;
        private MeshRenderer _vizRenderer;
        private static readonly int kTintId = Shader.PropertyToID("_Tint");

        // ---- WireVisualizationBehaviour surface ----
        protected override bool Show => showVisualization;
        protected override string VizObjectName => "_DwellSphereViz";
        protected override string MaterialName => "DwellSphere Wire (auto)";

        // Base-sphere tone (the progress ring is coloured separately in
        // ApplyMeshColors). Vertex colours stay <= 1; the glow is the material tint.
        protected override Color WireColor => Locked ? activeColor : idleColor;

        protected override Mesh CreateMesh()
        {
            var mesh = new Mesh { name = "_DwellSphere", hideFlags = HideFlags.DontSave };
            _builtSegments = -1;
            RebuildArcs(mesh);
            return mesh;
        }

        // Per-frame hook (base Update -> SyncVisualization -> here while shown).
        // Dwell progression + glow decay live here on purpose — see header.
        protected override bool UpdateMesh(Mesh mesh)
        {
            Tick(Time.deltaTime);
            ApplyWireMaterial();
            return RebuildArcs(mesh);
        }

        protected override void OnDisable()
        {
            base.OnDisable(); // base viz teardown (protected virtual for exactly this)
            _vizRenderer = null;
        }

        protected override void OnDestroy()
        {
            base.OnDestroy();
            if (_wireMat != null) { Destroy(_wireMat); _wireMat = null; }
        }

        // Swap the base's default wire material for our HDR-tint one (the base
        // recreates the viz child whenever Show toggles, so re-check cheaply).
        private void ApplyWireMaterial()
        {
            if (_vizRenderer == null)
            {
                var viz = transform.Find(VizObjectName);
                if (viz == null) return;
                _vizRenderer = viz.GetComponent<MeshRenderer>();
                if (_vizRenderer == null) return;
            }
            if (_wireMat == null)
            {
                var shader = Resources.Load<Shader>("DwellWire");
                if (shader == null) return; // keep the base material; glow just won't be HDR
                _wireMat = new Material(shader) { name = "DwellSphere Wire (HDR)", hideFlags = HideFlags.DontSave };
            }
            if (_vizRenderer.sharedMaterial != _wireMat)
                _vizRenderer.sharedMaterial = _wireMat;

            float t = Locked && glowSeconds > 0f ? Mathf.Clamp01(_glowRemaining / glowSeconds) : 0f;
            float gain = Mathf.LerpUnclamped(1f, glowBrightness, t);
            _wireMat.SetColor(kTintId, new Color(gain, gain, gain, 1f));
        }

        /// <summary>Back to idle (director calls this between visitors/states).</summary>
        public void ResetSphere()
        {
            Locked = false;
            Progress = 0f;
            _glowRemaining = 0f;
        }

        // ---- dwell state ----

        private void Tick(float dt)
        {
            if (Locked)
            {
                _glowRemaining = Mathf.Max(0f, _glowRemaining - dt);
                return;
            }

            bool handInside = debugForceDwell || AnyTrackedHandInside();
            float dwell = float.IsFinite(dwellSeconds) ? Mathf.Max(0.05f, dwellSeconds) : 1f;
            if (handInside)
            {
                Progress = Mathf.Min(1f, Progress + dt / dwell);
                if (Progress >= 1f) Select();
            }
            else
            {
                Progress = 0f; // spec: leaving resets instantly
            }
        }

        private bool AnyTrackedHandInside()
        {
            if (presence == null)
            {
                presence = FindFirstObjectByType<PresenceDetector>();
                if (presence == null) return false;
            }
            if (!presence.TryGetHands(out var left, out bool leftTracked,
                                      out var right, out bool rightTracked)) return false;
            float r2 = radius * radius;
            Vector3 c = transform.position;
            if (leftTracked && (left - c).sqrMagnitude <= r2) return true;
            if (rightTracked && (right - c).sqrMagnitude <= r2) return true;
            return false;
        }

        private void Select()
        {
            Locked = true;
            _glowRemaining = glowSeconds;
            if (selectedClip != null)
            {
                if (_audio == null && !TryGetComponent(out _audio))
                    _audio = gameObject.AddComponent<AudioSource>();
                _audio.PlayOneShot(selectedClip);
            }
            onSelected?.Invoke();
            OnSelected?.Invoke(this);
        }

        // ---- geometry: icosphere wireframe + a growing progress ring ----
        //
        // The sphere wireframe is ALWAYS complete (it marks the touch target):
        // a triangle-mesh icosphere (Blender's Ico Sphere), rendered as its
        // unique edges. Dwell progress draws a separate horizontal ring around
        // it (radius * progressRingScale) from 0 to a full circle — full circle
        // = selected. Horizontal so it reads from both visitor displays.

        private int _baseVertCount;
        private int _builtSubdivisions = -1;
        private readonly List<Vector3> _icoEdgeVerts = new List<Vector3>(1024); // unit sphere, line pairs

        private int RingSegments() =>
            Locked ? segmentsPerCircle
                   : Mathf.Clamp(Mathf.RoundToInt(segmentsPerCircle * Progress), 0, segmentsPerCircle);

        // Rebuild only when the ring's drawn segment count changes (quantized
        // progress), so slow dwell doesn't rewrite the mesh every frame.
        private bool RebuildArcs(Mesh mesh)
        {
            int ringSegs = RingSegments();
            if (ringSegs == _builtSegments && _builtSubdivisions == icoSubdivisions
                && mesh.vertexCount > 0) return false;
            _builtSegments = ringSegs;

            if (_builtSubdivisions != icoSubdivisions)
            {
                BuildIcosphereEdges(icoSubdivisions, _icoEdgeVerts);
                _builtSubdivisions = icoSubdivisions;
            }

            var verts = new List<Vector3>(_icoEdgeVerts.Count + ringSegs * 2);
            for (int i = 0; i < _icoEdgeVerts.Count; i++)
                verts.Add(_icoEdgeVerts[i] * radius);
            _baseVertCount = verts.Count;

            // Progress ring (XZ, slightly larger).
            AppendArc(verts, ringSegs, radius * progressRingScale,
                      (c, s) => new Vector3(c, 0f, s));

            var idx = new int[verts.Count];
            for (int i = 0; i < idx.Length; i++) idx[i] = i;

            mesh.Clear();
            mesh.SetVertices(verts);
            mesh.SetIndices(idx, MeshTopology.Lines, 0);
            mesh.RecalculateBounds();
            return true;
        }

        // Unit icosphere as unique wire edges (line vertex pairs): icosahedron
        // subdivided N times, midpoints pushed onto the sphere.
        private static void BuildIcosphereEdges(int subdivisions, List<Vector3> outLinePairs)
        {
            float t = (1f + Mathf.Sqrt(5f)) / 2f;
            var v = new List<Vector3>
            {
                new Vector3(-1,  t,  0), new Vector3( 1,  t,  0), new Vector3(-1, -t,  0), new Vector3( 1, -t,  0),
                new Vector3( 0, -1,  t), new Vector3( 0,  1,  t), new Vector3( 0, -1, -t), new Vector3( 0,  1, -t),
                new Vector3( t,  0, -1), new Vector3( t,  0,  1), new Vector3(-t,  0, -1), new Vector3(-t,  0,  1),
            };
            for (int i = 0; i < v.Count; i++) v[i] = v[i].normalized;
            var faces = new List<(int a, int b, int c)>
            {
                (0,11,5), (0,5,1), (0,1,7), (0,7,10), (0,10,11),
                (1,5,9), (5,11,4), (11,10,2), (10,7,6), (7,1,8),
                (3,9,4), (3,4,2), (3,2,6), (3,6,8), (3,8,9),
                (4,9,5), (2,4,11), (6,2,10), (8,6,7), (9,8,1),
            };

            var midCache = new Dictionary<long, int>();
            int Midpoint(int a, int b)
            {
                long key = a < b ? ((long)a << 32) | (uint)b : ((long)b << 32) | (uint)a;
                if (midCache.TryGetValue(key, out int m)) return m;
                m = v.Count;
                v.Add(((v[a] + v[b]) * 0.5f).normalized);
                midCache.Add(key, m);
                return m;
            }

            for (int s = 0; s < subdivisions; s++)
            {
                var next = new List<(int, int, int)>(faces.Count * 4);
                foreach (var (a, b, c) in faces)
                {
                    int ab = Midpoint(a, b), bc = Midpoint(b, c), ca = Midpoint(c, a);
                    next.Add((a, ab, ca)); next.Add((b, bc, ab));
                    next.Add((c, ca, bc)); next.Add((ab, bc, ca));
                }
                faces = next;
            }

            var edges = new HashSet<long>();
            void Edge(int a, int b)
            {
                long key = a < b ? ((long)a << 32) | (uint)b : ((long)b << 32) | (uint)a;
                edges.Add(key);
            }
            foreach (var (a, b, c) in faces) { Edge(a, b); Edge(b, c); Edge(c, a); }

            outLinePairs.Clear();
            foreach (long key in edges)
            {
                outLinePairs.Add(v[(int)(key >> 32)]);
                outLinePairs.Add(v[(int)(key & 0xFFFFFFFF)]);
            }
        }

        private void AppendArc(List<Vector3> verts, int segs, float r,
                               Func<float, float, Vector3> shape)
        {
            float step = 2f * Mathf.PI / segmentsPerCircle; // constant density: arcs grow
            for (int i = 0; i < segs; i++)
            {
                float a0 = i * step, a1 = (i + 1) * step;
                verts.Add(shape(Mathf.Cos(a0), Mathf.Sin(a0)) * r);
                verts.Add(shape(Mathf.Cos(a1), Mathf.Sin(a1)) * r);
            }
        }

        // Base sphere = the WireColor tone; the progress ring is always the
        // active colour so the sweep pops against the idle sphere.
        protected override void ApplyMeshColors(Mesh mesh, Color color)
        {
            var colors = new Color[mesh.vertexCount];
            for (int i = 0; i < colors.Length; i++)
                colors[i] = i < _baseVertCount ? color : activeColor;
            mesh.SetColors(colors);
        }
    }
}
