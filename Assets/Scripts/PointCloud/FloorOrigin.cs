// Defines the installation's floor plane and renders helper geometry on it.
//
// Convention (issue #22): the bottom-center of the assigned PointCloudBoundingBox
// is the floor; treat that point as the origin (y=0). FloorOrigin tracks the
// bounding box every frame and renders:
//
//   - A line-mesh grid at the floor plane (default ON), so the operator sees
//     the floor in both Scene and Game views without dropping a real plane mesh
//     into the scene.
//   - A drop shadow for every active PointCloudRenderer: each mesh is drawn a
//     second time with PointCloudShadow.shader (per-vertex Y collapsed to
//     _FloorY in world space, fragment = shadow color). The same OBB /
//     decimation filters as the live cloud are pushed through a per-draw
//     MaterialPropertyBlock so culled points don't cast a shadow either.
//
// Why DrawMesh instead of a second pass on PointCloudUnlit: keeps the existing
// material/shader pipeline (live + playback + cumulative snapshots) untouched
// and lets the shadow be opt-in per scene by simply removing the FloorOrigin GO.

using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering;

namespace PointCloud
{
    [DisallowMultipleComponent]
    [ExecuteAlways]
    public class FloorOrigin : MonoBehaviour
    {
        [Header("Floor anchor")]
        [Tooltip("Bounding volume whose bottom-center defines the floor plane. " +
                 "When assigned, FloorOrigin moves to that point every frame so the grid " +
                 "and drop-shadow plane track the box. Leave empty to use this GameObject's " +
                 "Transform position directly.")]
        public PointCloudBoundingBox boundingBox;

        [Header("Grid")]
        [Tooltip("Draw a wireframe grid at the floor plane.")]
        public bool showGrid = true;

        [Tooltip("Number of grid cells along each side (the grid is square). " +
                 "Total extent = gridCells * gridSpacing meters.")]
        [Min(1)]
        public int gridCells = 20;

        [Tooltip("Spacing between adjacent grid lines, in meters.")]
        [Min(0.001f)]
        public float gridSpacing = 0.5f;

        [Tooltip("Color of the regular grid lines.")]
        public Color gridColor = new Color(0.4f, 0.4f, 0.4f, 1f);

        [Tooltip("Color of the two axis lines through the origin (X and Z). Drawn on top of regular grid lines.")]
        public Color axisColor = new Color(0.85f, 0.85f, 0.85f, 1f);

        [Header("Drop shadow")]
        [Tooltip("Project active PointCloudRenderers onto the floor plane as a drop shadow.")]
        public bool showShadow = true;

        [Tooltip("RGBA color of the shadow points. Alpha controls per-point opacity " +
                 "(stacked points darken via additive alpha-blending).")]
        public Color shadowColor = new Color(0f, 0f, 0f, 0.5f);

        [Header("Rendering")]
        [Tooltip("Layer used for the grid and shadow draws. Set to a layer the camera renders.")]
        public int renderLayer = 0;

        // --- Internal ---
        private Mesh _gridMesh;
        private Material _gridMaterial;
        private Material _shadowMaterial;
        private MaterialPropertyBlock _shadowMpb;

        // Tracks whatever was last used to build the grid, so we can rebuild only when those change.
        private int _builtCells = -1;
        private float _builtSpacing = -1f;
        private Color _builtGridColor;
        private Color _builtAxisColor;

        // Cached list reused each LateUpdate to avoid the FindObjectsByType allocation.
        private static readonly List<PointCloudRenderer> s_rendererScratch = new List<PointCloudRenderer>(8);

        // Shader property IDs (mirror PointCloudShaderFilters + PointCloudShadow.shader).
        private static readonly int kObbObjToBox = Shader.PropertyToID("_ObbObjToBox");
        private static readonly int kObbMode     = Shader.PropertyToID("_ObbMode");
        private static readonly int kDecimKeep   = Shader.PropertyToID("_DecimKeep");
        private static readonly int kDecimFrame  = Shader.PropertyToID("_DecimFrame");
        private static readonly int kFloorY      = Shader.PropertyToID("_FloorY");
        private static readonly int kShadowColor = Shader.PropertyToID("_ShadowColor");

        private void OnEnable()
        {
            if (_shadowMpb == null) _shadowMpb = new MaterialPropertyBlock();
            UpdateAnchor();
        }

        private void OnDisable()
        {
            DestroyOwnedAssets();
        }

        private void OnDestroy()
        {
            DestroyOwnedAssets();
        }

        private void LateUpdate()
        {
            UpdateAnchor();

            if (showGrid)
            {
                EnsureGridMesh();
                EnsureGridMaterial();
                if (_gridMesh != null && _gridMaterial != null)
                {
                    // Grid mesh lives in this Transform's local space, so the grid follows the
                    // anchor position and any rotation you give the FloorOrigin GameObject.
                    Graphics.DrawMesh(_gridMesh, transform.localToWorldMatrix, _gridMaterial,
                                      renderLayer, null, 0, null, ShadowCastingMode.Off, false);
                }
            }

            if (showShadow)
            {
                EnsureShadowMaterial();
                if (_shadowMaterial != null) DrawShadows();
            }
        }

        // Floor anchor = bottom-center of the bounding box (local (0,-0.5,0) -> world).
        // Captures arbitrary OBB rotation correctly: TransformPoint applies the full TRS.
        private void UpdateAnchor()
        {
            if (boundingBox == null) return;
            Vector3 bottomCenter = boundingBox.transform.TransformPoint(new Vector3(0f, -0.5f, 0f));
            if (transform.position != bottomCenter) transform.position = bottomCenter;
        }

        private void EnsureGridMesh()
        {
            if (_gridMesh != null
                && _builtCells == gridCells
                && Mathf.Approximately(_builtSpacing, gridSpacing)
                && _builtGridColor == gridColor
                && _builtAxisColor == axisColor)
            {
                return;
            }

            if (_gridMesh == null)
            {
                _gridMesh = new Mesh
                {
                    name = "FloorGrid",
                    hideFlags = HideFlags.DontSave,
                };
            }
            BuildGridMesh(_gridMesh, gridCells, gridSpacing, gridColor, axisColor);
            _builtCells = gridCells;
            _builtSpacing = gridSpacing;
            _builtGridColor = gridColor;
            _builtAxisColor = axisColor;
        }

        private void EnsureGridMaterial()
        {
            if (_gridMaterial != null) return;
            var shader = Shader.Find("Orbbec/PointCloudUnlit");
            if (shader == null)
            {
                Debug.LogWarning(
                    $"[{nameof(FloorOrigin)}] Shader \"Orbbec/PointCloudUnlit\" not found; grid disabled.",
                    this);
                return;
            }
            _gridMaterial = new Material(shader)
            {
                name = "FloorGrid",
                hideFlags = HideFlags.DontSave,
            };
        }

        private void EnsureShadowMaterial()
        {
            if (_shadowMaterial != null) return;
            var shader = Shader.Find("Orbbec/PointCloudShadow");
            if (shader == null)
            {
                Debug.LogWarning(
                    $"[{nameof(FloorOrigin)}] Shader \"Orbbec/PointCloudShadow\" not found; drop shadow disabled.",
                    this);
                return;
            }
            _shadowMaterial = new Material(shader)
            {
                name = "PointCloudShadow",
                hideFlags = HideFlags.DontSave,
            };
        }

        private void DrawShadows()
        {
            float floorY = transform.position.y;
            s_rendererScratch.Clear();
            // Only live PointCloudRenderers cast a shadow. Snapshots / playback meshes are
            // out of scope for the initial drop-shadow implementation.
            var found = FindObjectsByType<PointCloudRenderer>(FindObjectsSortMode.None);
            for (int i = 0; i < found.Length; i++)
            {
                var pcr = found[i];
                if (pcr == null || !pcr.isActiveAndEnabled) continue;
                var mf = pcr.GetComponent<MeshFilter>();
                var mesh = mf != null ? mf.sharedMesh : null;
                if (mesh == null) continue;

                BuildShadowMpb(pcr, floorY);
                Graphics.DrawMesh(mesh, pcr.transform.localToWorldMatrix, _shadowMaterial,
                                  renderLayer, null, 0, _shadowMpb, ShadowCastingMode.Off, false);
            }
        }

        // Mirrors PointCloudShaderFilters.Apply so the shadow sees the same culled points the
        // live cloud does. Reads filter refs straight from the renderer's public fields.
        private void BuildShadowMpb(PointCloudRenderer pcr, float floorY)
        {
            _shadowMpb.Clear();

            float obbMode = 0f;
            var bb = pcr.boundingBox;
            if (bb != null && bb.Mode != PointCloudBoundingBox.FilterMode.Disabled)
            {
                obbMode = bb.Mode == PointCloudBoundingBox.FilterMode.KeepInside ? 1f : 2f;
                var m = bb.transform.worldToLocalMatrix * pcr.transform.localToWorldMatrix;
                _shadowMpb.SetMatrix(kObbObjToBox, m);
            }
            _shadowMpb.SetFloat(kObbMode, obbMode);

            var decim = pcr.decimater;
            float decimKeep = (decim != null && decim.Enabled) ? Mathf.Clamp01(decim.KeepRatio) : 1f;
            _shadowMpb.SetFloat(kDecimKeep, decimKeep);
            _shadowMpb.SetFloat(kDecimFrame, Time.frameCount);

            _shadowMpb.SetFloat(kFloorY, floorY);
            _shadowMpb.SetColor(kShadowColor, shadowColor);
        }

        private void DestroyOwnedAssets()
        {
            if (_gridMesh != null)
            {
                if (Application.isPlaying) Destroy(_gridMesh); else DestroyImmediate(_gridMesh);
                _gridMesh = null;
            }
            if (_gridMaterial != null)
            {
                if (Application.isPlaying) Destroy(_gridMaterial); else DestroyImmediate(_gridMaterial);
                _gridMaterial = null;
            }
            if (_shadowMaterial != null)
            {
                if (Application.isPlaying) Destroy(_shadowMaterial); else DestroyImmediate(_shadowMaterial);
                _shadowMaterial = null;
            }
            _builtCells = -1;
            _builtSpacing = -1f;
        }

        // Square line grid centered on (0,0,0) in the local XZ plane. The two axis lines
        // (x=0 and z=0) get the axisColor, every other line gets the regular gridColor.
        private static void BuildGridMesh(Mesh mesh, int cells, float spacing, Color gridColor, Color axisColor)
        {
            int lineCount = (cells + 1) * 2; // (cells+1) lines along X plus (cells+1) along Z
            int vertCount = lineCount * 2;
            var verts = new Vector3[vertCount];
            var colors = new Color[vertCount];
            var indices = new int[vertCount];

            float halfExtent = cells * spacing * 0.5f;
            int v = 0;

            // Lines parallel to the Z axis (constant X).
            for (int i = 0; i <= cells; i++)
            {
                float x = -halfExtent + i * spacing;
                bool isAxis = Mathf.Abs(x) < spacing * 0.001f;
                Color c = isAxis ? axisColor : gridColor;
                verts[v]   = new Vector3(x, 0f, -halfExtent);
                verts[v+1] = new Vector3(x, 0f,  halfExtent);
                colors[v] = c; colors[v+1] = c;
                indices[v] = v; indices[v+1] = v+1;
                v += 2;
            }
            // Lines parallel to the X axis (constant Z).
            for (int j = 0; j <= cells; j++)
            {
                float z = -halfExtent + j * spacing;
                bool isAxis = Mathf.Abs(z) < spacing * 0.001f;
                Color c = isAxis ? axisColor : gridColor;
                verts[v]   = new Vector3(-halfExtent, 0f, z);
                verts[v+1] = new Vector3( halfExtent, 0f, z);
                colors[v] = c; colors[v+1] = c;
                indices[v] = v; indices[v+1] = v+1;
                v += 2;
            }

            mesh.Clear();
            mesh.SetVertices(verts);
            mesh.SetColors(colors);
            mesh.SetIndices(indices, MeshTopology.Lines, 0);
            mesh.bounds = new Bounds(Vector3.zero, new Vector3(cells * spacing, 0f, cells * spacing));
        }
    }
}
