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
                 "wireframe is always complete; the dwell draws this camera-facing ring " +
                 "over it from 0 to a full circle — full circle = selected.")]
        public float progressRingScale = 1.15f;

        [Range(0.005f, 0.1f)]
        [Tooltip("Progress ring width (m) — drawn as a solid ribbon, not a 1px line.")]
        public float progressRingWidth = 0.03f;

        [Tooltip("Opaque core inside the wireframe: hides the far-side wires and " +
                 "occludes the point cloud when a hand goes BEHIND the sphere — " +
                 "that occlusion is what sells the depth.")]
        public Color coreColor = Color.black;

        [Range(0.8f, 1f)]
        [Tooltip("Core diameter as a fraction of the wire sphere (slightly inside " +
                 "so the front wires stay crisp).")]
        public float coreScale = 0.97f;

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
            if (_ringMat != null) { Destroy(_ringMat); _ringMat = null; }
            if (_ringMesh != null) { Destroy(_ringMesh); _ringMesh = null; }
            if (_ringGo != null) { Destroy(_ringGo); _ringGo = null; }
            if (_coreMat != null) { Destroy(_coreMat); _coreMat = null; }
            if (_coreGo != null) { Destroy(_coreGo); _coreGo = null; }
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
            var tint = new Color(gain, gain, gain, 1f);
            _wireMat.SetColor(kTintId, tint);
            if (_ringMat != null) _ringMat.SetColor(kTintId, tint); // ring glows with the sphere
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

        // ---- geometry: icosphere wireframe + a billboard progress ring ----
        //
        // The sphere wireframe is ALWAYS complete (it marks the touch target):
        // a triangle-mesh icosphere (Blender's Ico Sphere), rendered as its
        // unique edges. Dwell progress draws a separate CAMERA-FACING ring
        // (clock-style, 12 o'clock start, clockwise) over the sphere — a
        // billboard shader re-expands it per display camera, so both visitor
        // screens see the circle face-on. Full circle = selected.

        private int _builtSubdivisions = -1;
        private float _builtRadius = -1f;
        private readonly List<Vector3> _icoEdgeVerts = new List<Vector3>(1024); // unit sphere, line pairs
        // Smallest edge-midpoint radius on the unit sphere: icosphere edges are
        // CHORDS that dip below the surface, so the opaque core must stay under
        // this or it swallows the wireframe (bit us live at subdiv 1 / core 0.97).
        private float _chordMinRadius = 0.85f; // icosahedron worst case until measured

        // Ring child (own mesh + billboard material — see DwellRing.shader).
        private GameObject _ringGo;
        private Mesh _ringMesh;
        private Material _ringMat;

        // Opaque occluder core (primitive sphere, URP unlit).
        private GameObject _coreGo;
        private Material _coreMat;

        private int RingSegments() =>
            Locked ? segmentsPerCircle
                   : Mathf.Clamp(Mathf.RoundToInt(segmentsPerCircle * Progress), 0, segmentsPerCircle);

        // Base sphere mesh: rebuilt when the subdivision level OR radius changes.
        private bool RebuildArcs(Mesh mesh)
        {
            UpdateRing();
            bool rebuilt = false;
            if (_builtSubdivisions != icoSubdivisions
                || !Mathf.Approximately(_builtRadius, radius)
                || mesh.vertexCount == 0)
            {
                if (_builtSubdivisions != icoSubdivisions)
                {
                    BuildIcosphereEdges(icoSubdivisions, _icoEdgeVerts);
                    _builtSubdivisions = icoSubdivisions;
                    // Measure how deep the chords dip (unit sphere) — the core
                    // clamp in UpdateCore depends on it.
                    float min = 1f;
                    for (int i = 0; i < _icoEdgeVerts.Count; i += 2)
                        min = Mathf.Min(min, ((_icoEdgeVerts[i] + _icoEdgeVerts[i + 1]) * 0.5f).magnitude);
                    _chordMinRadius = min;
                }
                _builtRadius = radius;

                var verts = new List<Vector3>(_icoEdgeVerts.Count);
                for (int i = 0; i < _icoEdgeVerts.Count; i++)
                    verts.Add(_icoEdgeVerts[i] * radius);

                var idx = new int[verts.Count];
                for (int i = 0; i < idx.Length; i++) idx[i] = i;

                mesh.Clear();
                mesh.SetVertices(verts);
                mesh.SetIndices(idx, MeshTopology.Lines, 0);
                mesh.RecalculateBounds();
                rebuilt = true;
            }
            UpdateCore(); // after the rebuild so the chord clamp uses fresh data
            return rebuilt;
        }

        // Opaque core: hides far-side wires and occludes anything (hands, point
        // cloud) that moves behind the sphere — the depth cue.
        private void UpdateCore()
        {
            if (_coreGo == null)
            {
                _coreGo = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                _coreGo.name = "_DwellCore";
                _coreGo.hideFlags = HideFlags.DontSave;
                Destroy(_coreGo.GetComponent<Collider>());
                _coreGo.transform.SetParent(transform, false);
                var shader = Shader.Find("Universal Render Pipeline/Unlit");
                if (shader != null)
                {
                    _coreMat = new Material(shader) { name = "DwellCore (auto)", hideFlags = HideFlags.DontSave };
                    _coreMat.SetColor("_BaseColor", coreColor);
                    _coreGo.GetComponent<MeshRenderer>().sharedMaterial = _coreMat;
                }
                var mr = _coreGo.GetComponent<MeshRenderer>();
                mr.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
            }
            _coreGo.SetActive(Show);
            // The wire edges are chords that dip inside the sphere — clamp the
            // core under the shallowest chord or it swallows the wireframe.
            float effective = Mathf.Min(coreScale, _chordMinRadius * 0.99f);
            float d = radius * 2f * effective;
            if (!Mathf.Approximately(_coreGo.transform.localScale.x, d))
                _coreGo.transform.localScale = Vector3.one * d;
        }

        // Clock-style arc in LOCAL XY meters (the billboard shader re-expands it
        // along each camera's right/up). Rebuilt when the segment count changes.
        private void UpdateRing()
        {
            int segs = RingSegments();
            if (_ringGo == null)
            {
                _ringGo = new GameObject("_DwellRing") { hideFlags = HideFlags.DontSave };
                _ringGo.transform.SetParent(transform, false);
                _ringMesh = new Mesh { name = "_DwellRing", hideFlags = HideFlags.DontSave };
                _ringGo.AddComponent<MeshFilter>().sharedMesh = _ringMesh;
                var mr = _ringGo.AddComponent<MeshRenderer>();
                mr.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
                var shader = Resources.Load<Shader>("DwellRing");
                if (shader != null)
                {
                    _ringMat = new Material(shader) { name = "DwellRing (auto)", hideFlags = HideFlags.DontSave };
                    mr.sharedMaterial = _ringMat;
                }
                _builtSegments = -1;
            }
            _ringGo.SetActive(segs > 0 && Show);
            if (segs == _builtSegments) return;
            _builtSegments = segs;
            if (segs == 0) { _ringMesh.Clear(); return; }

            // Solid ribbon arc (triangles): inner/outer vertex pair per step,
            // 12 o'clock start, clockwise on screen (x = sin, y = cos).
            float rOuter = radius * progressRingScale + progressRingWidth * 0.5f;
            float rInner = rOuter - progressRingWidth;
            float step = 2f * Mathf.PI / segmentsPerCircle;
            var verts = new Vector3[(segs + 1) * 2];
            var colors = new Color[verts.Length];
            for (int i = 0; i <= segs; i++)
            {
                float a = i * step;
                var dir = new Vector3(Mathf.Sin(a), Mathf.Cos(a), 0f);
                verts[i * 2] = dir * rInner;
                verts[i * 2 + 1] = dir * rOuter;
                colors[i * 2] = activeColor;
                colors[i * 2 + 1] = activeColor;
            }
            var tris = new int[segs * 6];
            for (int i = 0; i < segs; i++)
            {
                int v = i * 2, t = i * 6;
                tris[t] = v; tris[t + 1] = v + 1; tris[t + 2] = v + 2;
                tris[t + 3] = v + 1; tris[t + 4] = v + 3; tris[t + 5] = v + 2;
            }
            _ringMesh.Clear();
            _ringMesh.vertices = verts;
            _ringMesh.colors = colors;
            _ringMesh.SetTriangles(tris, 0);
            // Bounds must survive billboarding from any angle.
            _ringMesh.bounds = new Bounds(Vector3.zero, Vector3.one * (rOuter * 2.2f));
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

    }
}
