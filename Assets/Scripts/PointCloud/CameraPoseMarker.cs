// Wireframe square-pyramid (camera frustum) marker. Attach to a GameObject whose
// local origin is the camera optical center and whose +Z is the view direction
// (this is exactly how PointCloudRenderer GameObjects are oriented once extrinsics
// are applied). The apex sits at the origin and the square base opens out along +Z,
// so each camera's position AND aim are visible inside the merged point cloud.
//
// Rendering mirrors BoundingVolume: a runtime line-mesh (so it shows in the
// Game view / builds, not just the Editor) plus a Scene-view Gizmo.

using UnityEngine;

namespace PointCloud
{
    [DisallowMultipleComponent]
    public class CameraPoseMarker : MonoBehaviour
    {
        [Header("Visualization")]
        [Tooltip("Show the wireframe frustum in both Scene view (Gizmos) and Game view (runtime mesh).")]
        public bool showVisualization = true;

        [Tooltip("Color of the wireframe frustum.")]
        public Color color = new Color(1f, 0.85f, 0.2f, 1f);

        [Header("Shape (metres, in the camera's local frame)")]
        [Tooltip("Distance from the apex (camera origin) to the base square, along +Z (view direction).")]
        public float length = 0.2f;

        [Tooltip("Half-width of the base square (local X).")]
        public float baseHalfWidth = 0.12f;

        [Tooltip("Half-height of the base square (local Y).")]
        public float baseHalfHeight = 0.09f;

        // Runtime line-mesh rendering. Built as a child so it rides this transform
        // (and therefore the applied extrinsics) automatically.
        private GameObject _vizObject;
        private MeshFilter _vizFilter;
        private MeshRenderer _vizRenderer;
        private Mesh _vizMesh;
        private Material _vizMaterial;
        private Color _lastAppliedColor;
        private Vector3 _lastShape; // (length, baseHalfWidth, baseHalfHeight) the mesh was built for

        private void OnEnable() => SyncVisualization();
        private void OnDisable() => DestroyVisualization();
        private void OnDestroy() => DestroyVisualization();
        private void Update() => SyncVisualization();

        private void OnValidate()
        {
            // Defer to Update(): Unity forbids DestroyImmediate during OnValidate, and
            // rebuilding here would be illegal. Same pattern as BoundingVolume.
        }

        private void SyncVisualization()
        {
            if (showVisualization)
            {
                if (_vizObject == null) CreateVisualization();

                var shape = new Vector3(length, baseHalfWidth, baseHalfHeight);
                if (_vizMesh != null && _lastShape != shape)
                {
                    RebuildFrustum(_vizMesh, length, baseHalfWidth, baseHalfHeight);
                    ApplyVertexColor(_vizMesh, color);
                    _lastAppliedColor = color;
                    _lastShape = shape;
                }
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
            _vizObject = new GameObject("_CamFrustumViz")
            {
                hideFlags = HideFlags.DontSave | HideFlags.NotEditable,
            };
            _vizObject.transform.SetParent(transform, worldPositionStays: false);

            _vizFilter = _vizObject.AddComponent<MeshFilter>();
            _vizRenderer = _vizObject.AddComponent<MeshRenderer>();
            PointCloudUtil.ConfigureUnlitRenderer(_vizRenderer);

            _vizMesh = new Mesh { name = "CamFrustumWire", hideFlags = HideFlags.DontSave };
            RebuildFrustum(_vizMesh, length, baseHalfWidth, baseHalfHeight);
            ApplyVertexColor(_vizMesh, color);
            _lastAppliedColor = color;
            _lastShape = new Vector3(length, baseHalfWidth, baseHalfHeight);
            _vizFilter.sharedMesh = _vizMesh;

            // Reuse the project's vertex-color unlit shader so the frustum blends into
            // the same rendering setup as the points. Fall back to a built-in if absent.
            var shader = Shader.Find("Orbbec/PointCloudUnlit");
            if (shader == null) shader = Shader.Find("Hidden/Internal-Colored");
            if (shader != null)
            {
                _vizMaterial = new Material(shader)
                {
                    name = "CamFrustumViz",
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

        // Square pyramid: apex at the origin, base square at z == len.
        // 5 verts, 8 line segments (4 apex->corner + 4 base edges).
        private static void RebuildFrustum(Mesh mesh, float len, float hw, float hh)
        {
            var verts = new Vector3[5];
            verts[0] = Vector3.zero;                  // apex = camera origin
            verts[1] = new Vector3(-hw, -hh, len);    // base corners
            verts[2] = new Vector3(hw, -hh, len);
            verts[3] = new Vector3(hw, hh, len);
            verts[4] = new Vector3(-hw, hh, len);

            var indices = new[]
            {
                0, 1, 0, 2, 0, 3, 0, 4, // apex to each base corner
                1, 2, 2, 3, 3, 4, 4, 1, // base square loop
            };

            mesh.Clear();
            mesh.SetVertices(verts);
            mesh.SetIndices(indices, MeshTopology.Lines, 0);
            mesh.bounds = new Bounds(new Vector3(0, 0, len * 0.5f),
                                     new Vector3(hw * 2f, hh * 2f, len));
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
            Gizmos.color = color;

            Vector3 apex = Vector3.zero;
            Vector3 c1 = new Vector3(-baseHalfWidth, -baseHalfHeight, length);
            Vector3 c2 = new Vector3(baseHalfWidth, -baseHalfHeight, length);
            Vector3 c3 = new Vector3(baseHalfWidth, baseHalfHeight, length);
            Vector3 c4 = new Vector3(-baseHalfWidth, baseHalfHeight, length);
            Gizmos.DrawLine(apex, c1); Gizmos.DrawLine(apex, c2);
            Gizmos.DrawLine(apex, c3); Gizmos.DrawLine(apex, c4);
            Gizmos.DrawLine(c1, c2); Gizmos.DrawLine(c2, c3);
            Gizmos.DrawLine(c3, c4); Gizmos.DrawLine(c4, c1);

            Gizmos.matrix = prevMatrix;
            Gizmos.color = prevColor;
        }
    }
}
