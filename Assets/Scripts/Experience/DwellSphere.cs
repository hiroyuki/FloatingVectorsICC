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

        [Range(0f, 0.9f)]
        [Tooltip("Arc fraction shown while idle (0.25 = quarter circles). Dwell " +
                 "progress sweeps this to full circles.")]
        public float idleArcFraction = 0.25f;

        [Min(8)]
        [Tooltip("Line segments per full circle.")]
        public int segmentsPerCircle = 64;

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

        // Vertex colours stay <= 1 (UNorm8); the glow is the material tint.
        protected override Color WireColor =>
            Locked ? activeColor : Color.LerpUnclamped(idleColor, activeColor, Progress);

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

        // ---- geometry: three orthogonal circles, arc length = progress ----

        private float CurrentArcFraction() =>
            Locked ? 1f : Mathf.Lerp(idleArcFraction, 1f, Progress);

        // Rebuild only when the drawn segment count changes (quantized progress),
        // so slow dwell doesn't rewrite the mesh every frame.
        private bool RebuildArcs(Mesh mesh)
        {
            int segs = Mathf.Clamp(Mathf.RoundToInt(segmentsPerCircle * CurrentArcFraction()),
                                   1, segmentsPerCircle);
            if (segs == _builtSegments) return false;
            _builtSegments = segs;

            int vertsPerArc = segs * 2; // Lines topology: 2 verts per segment
            var verts = new Vector3[vertsPerArc * 3];
            var idx = new int[vertsPerArc * 3];
            int w = 0;
            AppendArc(verts, ref w, segs, (c, s) => new Vector3(c, s, 0f)); // XY
            AppendArc(verts, ref w, segs, (c, s) => new Vector3(0f, c, s)); // YZ
            AppendArc(verts, ref w, segs, (c, s) => new Vector3(s, 0f, c)); // ZX
            for (int i = 0; i < idx.Length; i++) idx[i] = i;

            mesh.Clear();
            mesh.vertices = verts;
            mesh.SetIndices(idx, MeshTopology.Lines, 0);
            mesh.RecalculateBounds();
            return true;
        }

        private void AppendArc(Vector3[] verts, ref int w, int segs, Func<float, float, Vector3> plane)
        {
            float step = 2f * Mathf.PI / segmentsPerCircle; // full-circle step: arcs grow, density stays
            for (int i = 0; i < segs; i++)
            {
                float a0 = i * step, a1 = (i + 1) * step;
                verts[w++] = plane(Mathf.Cos(a0), Mathf.Sin(a0)) * radius;
                verts[w++] = plane(Mathf.Cos(a1), Mathf.Sin(a1)) * radius;
            }
        }
    }
}
