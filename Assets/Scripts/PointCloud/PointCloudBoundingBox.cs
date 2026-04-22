// Oriented bounding box used by PointCloudRenderer to cull points inside or
// outside the box. Attach to a GameObject; its Transform defines the box
// center (position), orientation (rotation), and size (localScale). The box
// occupies the unit cube [-0.5, +0.5]^3 in the GameObject's local space, so
// a Transform with localScale == (1,1,1) is a 1 m cube.

using UnityEngine;

namespace PointCloud
{
    [DisallowMultipleComponent]
    public class PointCloudBoundingBox : MonoBehaviour
    {
        public enum FilterMode
        {
            Disabled = 0,
            KeepInside = 1,
            KeepOutside = 2,
        }

        [Tooltip("Disabled = pass all points through. KeepInside = keep points inside the box. " +
                 "KeepOutside = keep points outside the box.")]
        public FilterMode filterMode = FilterMode.KeepInside;

        [Header("Visualization")]
        [Tooltip("Show a wireframe of the box in both Scene view (Gizmos) and Game view (runtime mesh).")]
        public bool showVisualization = true;

        [Tooltip("Color of the wireframe box.")]
        public Color color = new Color(0.2f, 1f, 0.4f, 1f);

        // Runtime wire-cube rendering (so it shows in Game view / builds, not just the Editor).
        private GameObject _vizObject;
        private MeshFilter _vizFilter;
        private MeshRenderer _vizRenderer;
        private Mesh _vizMesh;
        private Material _vizMaterial;
        private Color _lastAppliedColor;

        public FilterMode Mode => filterMode;

        private void OnEnable()
        {
            SyncVisualization();
        }

        private void OnDisable()
        {
            DestroyVisualization();
        }

        private void OnDestroy()
        {
            DestroyVisualization();
        }

        private void Update()
        {
            SyncVisualization();
        }

        private void OnValidate()
        {
            if (isActiveAndEnabled) SyncVisualization();
        }

        private void SyncVisualization()
        {
            if (showVisualization)
            {
                if (_vizObject == null) CreateVisualization();
                if (_vizMesh != null && _lastAppliedColor != color)
                {
                    ApplyVertexColor(_vizMesh, color);
                    _lastAppliedColor = color;
                }
            }
            else if (_vizObject != null)
            {
                DestroyVisualization();
            }
        }

        private void CreateVisualization()
        {
            _vizObject = new GameObject("_BoundsViz")
            {
                hideFlags = HideFlags.DontSave | HideFlags.NotEditable,
            };
            _vizObject.transform.SetParent(transform, worldPositionStays: false);

            _vizFilter = _vizObject.AddComponent<MeshFilter>();
            _vizRenderer = _vizObject.AddComponent<MeshRenderer>();
            _vizRenderer.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
            _vizRenderer.receiveShadows = false;
            _vizRenderer.lightProbeUsage = UnityEngine.Rendering.LightProbeUsage.Off;
            _vizRenderer.reflectionProbeUsage = UnityEngine.Rendering.ReflectionProbeUsage.Off;

            _vizMesh = BuildWireCubeMesh();
            ApplyVertexColor(_vizMesh, color);
            _lastAppliedColor = color;
            _vizFilter.sharedMesh = _vizMesh;

            // Reuse the project's vertex-color unlit shader so the box blends into the
            // same rendering setup as the points. Fall back to Unity built-ins if absent.
            var shader = Shader.Find("Orbbec/PointCloudUnlit");
            if (shader == null) shader = Shader.Find("Hidden/Internal-Colored");
            if (shader != null)
            {
                _vizMaterial = new Material(shader)
                {
                    name = "BoundsViz",
                    hideFlags = HideFlags.DontSave,
                };
                _vizRenderer.sharedMaterial = _vizMaterial;
            }
        }

        private void DestroyVisualization()
        {
            if (_vizObject != null)
            {
                if (Application.isPlaying) Destroy(_vizObject); else DestroyImmediate(_vizObject);
                _vizObject = null;
                _vizFilter = null;
                _vizRenderer = null;
            }
            if (_vizMesh != null)
            {
                if (Application.isPlaying) Destroy(_vizMesh); else DestroyImmediate(_vizMesh);
                _vizMesh = null;
            }
            if (_vizMaterial != null)
            {
                if (Application.isPlaying) Destroy(_vizMaterial); else DestroyImmediate(_vizMaterial);
                _vizMaterial = null;
            }
        }

        private static Mesh BuildWireCubeMesh()
        {
            var mesh = new Mesh
            {
                name = "BoundsWireCube",
                hideFlags = HideFlags.DontSave,
            };
            var verts = new Vector3[8];
            int idx = 0;
            for (int z = 0; z <= 1; z++)
            for (int y = 0; y <= 1; y++)
            for (int x = 0; x <= 1; x++)
                verts[idx++] = new Vector3(x - 0.5f, y - 0.5f, z - 0.5f);

            // 12 cube edges -> 24 indices (LineList). Corners indexed by (x,y,z) bits 0..7.
            var indices = new int[]
            {
                0, 1, 2, 3, 4, 5, 6, 7, // along X
                0, 2, 1, 3, 4, 6, 5, 7, // along Y
                0, 4, 1, 5, 2, 6, 3, 7, // along Z
            };

            mesh.SetVertices(verts);
            mesh.SetIndices(indices, MeshTopology.Lines, 0);
            mesh.bounds = new Bounds(Vector3.zero, Vector3.one);
            return mesh;
        }

        private static void ApplyVertexColor(Mesh mesh, Color c)
        {
            var colors = new Color[mesh.vertexCount];
            for (int i = 0; i < colors.Length; i++) colors[i] = c;
            mesh.SetColors(colors);
        }

        private void OnDrawGizmos()
        {
            if (!showVisualization) return;
            var prevMatrix = Gizmos.matrix;
            var prevColor = Gizmos.color;
            Gizmos.matrix = transform.localToWorldMatrix;
            Gizmos.color = filterMode == FilterMode.Disabled ? new Color(color.r, color.g, color.b, 0.4f) : color;
            Gizmos.DrawWireCube(Vector3.zero, Vector3.one);
            Gizmos.matrix = prevMatrix;
            Gizmos.color = prevColor;
        }
    }
}
