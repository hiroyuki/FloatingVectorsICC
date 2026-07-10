// Shared runtime wire-mesh visualization machinery (3-6 dedup): BoundingVolume and
// CameraPoseMarker carried ~80%-identical plumbing (viz child GO + line mesh +
// vertex-colour unlit material, synced every Update, destroyed with the component —
// CameraPoseMarker.cs even said "Same pattern as BoundingVolume"). The subclasses
// keep their own serialized fields (show flag / colour / shape — tooltips and
// defaults differ) and expose them through the abstract properties; this base owns
// the lifecycle so the wire shows in the Game view / builds, not just the Editor.

using UnityEngine;

namespace PointCloud
{
    public abstract class WireVisualizationBehaviour : MonoBehaviour
    {
        // Runtime wire rendering. Built as a child so it rides this transform.
        private GameObject _vizObject;
        private MeshFilter _vizFilter;
        private MeshRenderer _vizRenderer;
        private Mesh _vizMesh;
        private Material _vizMaterial;
        private Color _lastAppliedColor;

        /// <summary>Subclass's show toggle (serialized field stays in the subclass).</summary>
        protected abstract bool Show { get; }
        /// <summary>Subclass's wire colour (serialized field stays in the subclass).</summary>
        protected abstract Color WireColor { get; }
        /// <summary>Name of the hidden child GameObject (e.g. "_BoundsViz").</summary>
        protected abstract string VizObjectName { get; }
        /// <summary>Name given to the runtime material.</summary>
        protected abstract string MaterialName { get; }

        /// <summary>Build the line mesh (named, HideFlags.DontSave). Colour is applied
        /// by the base right after.</summary>
        protected abstract Mesh CreateMesh();

        /// <summary>Per-frame hook while visible — rebuild the mesh when subclass shape
        /// parameters changed (CameraPoseMarker). Return true if the mesh was rewritten
        /// so the base reapplies the vertex colour. Default: static mesh, no-op.</summary>
        protected virtual bool UpdateMesh(Mesh mesh) => false;

        // Protected virtual so subclasses can extend teardown (e.g. DwellSphere's
        // own material) — a subclass defining these privately would HIDE the base
        // message and silently kill the visualization lifecycle. Update stays
        // private on purpose: per-frame subclass work goes through UpdateMesh.
        protected virtual void OnEnable() => SyncVisualization();
        protected virtual void OnDisable() => DestroyVisualization();
        protected virtual void OnDestroy() => DestroyVisualization();
        private void Update() => SyncVisualization();

        private void OnValidate()
        {
            // Defer the actual visualization sync: Unity forbids DestroyImmediate (and
            // friends) during OnValidate, and toggling the show flag off here would call
            // DestroyVisualization synchronously and log an error every time. Update()
            // runs the same SyncVisualization() in a context where Destroy is legal, so
            // just let the next tick pick up the new Inspector value.
        }

        private void SyncVisualization()
        {
            if (Show)
            {
                if (_vizObject == null) CreateVisualization();
                if (_vizMesh != null && UpdateMesh(_vizMesh))
                {
                    ApplyVertexColor(_vizMesh, WireColor);
                    _lastAppliedColor = WireColor;
                }
                if (_vizMesh != null && _lastAppliedColor != WireColor)
                {
                    ApplyVertexColor(_vizMesh, WireColor);
                    _lastAppliedColor = WireColor;
                }
            }
            else if (_vizObject != null)
            {
                DestroyVisualization();
            }
        }

        private void CreateVisualization()
        {
            _vizObject = new GameObject(VizObjectName)
            {
                hideFlags = HideFlags.DontSave | HideFlags.NotEditable,
            };
            _vizObject.transform.SetParent(transform, worldPositionStays: false);

            _vizFilter = _vizObject.AddComponent<MeshFilter>();
            _vizRenderer = _vizObject.AddComponent<MeshRenderer>();
            PointCloudUtil.ConfigureUnlitRenderer(_vizRenderer);

            _vizMesh = CreateMesh();
            ApplyVertexColor(_vizMesh, WireColor);
            _lastAppliedColor = WireColor;
            _vizFilter.sharedMesh = _vizMesh;

            // Reuse the project's vertex-color unlit shader so the wire blends into the
            // same rendering setup as the points. Fall back to Unity built-ins if absent.
            var shader = Shader.Find("Orbbec/PointCloudUnlit");
            if (shader == null) shader = Shader.Find("Hidden/Internal-Colored");
            if (shader != null)
            {
                _vizMaterial = new Material(shader)
                {
                    name = MaterialName,
                    hideFlags = HideFlags.DontSave,
                };
                _vizRenderer.sharedMaterial = _vizMaterial;
            }
        }

        private void DestroyVisualization()
        {
            if (_vizObject != null)
            {
                PointCloudUtil.DestroySafe(_vizObject);
                _vizObject = null;
                _vizFilter = null;
                _vizRenderer = null;
            }
            if (_vizMesh != null)
            {
                PointCloudUtil.DestroySafe(_vizMesh);
                _vizMesh = null;
            }
            if (_vizMaterial != null)
            {
                PointCloudUtil.DestroySafe(_vizMaterial);
                _vizMaterial = null;
            }
        }

        protected static void ApplyVertexColor(Mesh mesh, Color c)
        {
            var colors = new Color[mesh.vertexCount];
            for (int i = 0; i < colors.Length; i++) colors[i] = c;
            mesh.SetColors(colors);
        }
    }
}
