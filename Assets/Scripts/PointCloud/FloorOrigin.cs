// Defines the installation's floor plane and renders helper geometry on it.
//
// Convention (issue #22): the bottom-center of the assigned BoundingVolume
// is the floor; treat that point as the origin (y=0). FloorOrigin tracks the
// bounding box every frame and renders:
//
//   - A line-mesh grid at the floor plane (default ON), so the operator sees
//     the floor in both Scene and Game views without dropping a real plane mesh
//     into the scene.
//   - A drop shadow ("floor reflection") of what is VISIBLE right now: every
//     enabled point-cloud / playback / skeleton MeshRenderer is drawn a second
//     time with PointCloudShadow.shader (per-vertex Y collapsed to _FloorY in
//     world space, fragment = shadow color). The source renderer's own
//     MaterialPropertyBlock is copied per draw, so every display filter (OBB,
//     decimation, capsules, floor mask, motion field, reveal clip) applies to
//     the reflection identically — hidden meshes and culled points don't cast.
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
        public BoundingVolume boundingBox;

        [Header("Grid")]
        [Tooltip("Draw a wireframe grid at the floor plane.")]
        public bool showGrid = true;

        [Tooltip("Fit the grid to the bounding box footprint: lines snapped to WORLD " +
                 "multiples of gridSpacing (the world origin is always a line crossing) " +
                 "and clipped to the box interior. Off = legacy free-floating square " +
                 "grid of gridCells x gridSpacing centred on the anchor.")]
        public bool fitToBoundingBox = false;

        [Tooltip("Number of grid cells along each side (the grid is square). " +
                 "Total extent = gridCells * gridSpacing meters. Ignored when " +
                 "fitToBoundingBox is on.")]
        [Min(1)]
        public int gridCells = 20;

        [Tooltip("Spacing between adjacent grid lines, in meters.")]
        [Min(0.001f)]
        public float gridSpacing = 0.5f;

        [Tooltip("Color of the regular grid lines.")]
        public Color gridColor = new Color(0.4f, 0.4f, 0.4f, 1f);

        [Tooltip("Brightness multiplier applied to interior grid lines. The border " +
                 "rectangle (outermost lines) and the axis lines keep their full color.")]
        [Range(0f, 1f)]
        public float innerLineBrightness = 0.4f;

        [Tooltip("Color of the two axis lines through the origin (X and Z). Drawn on top of regular grid lines.")]
        public Color axisColor = new Color(0.85f, 0.85f, 0.85f, 1f);

        [Header("Drop shadow")]
        [Tooltip("Project active PointCloudRenderers onto the floor plane as a drop shadow.")]
        public bool showShadow = true;

        [Tooltip("RGBA color of the shadow. Alpha controls per-pixel opacity " +
                 "(stacked geometry darkens via alpha-blending).")]
        public Color shadowColor = new Color(0f, 0f, 0f, 0.5f);

        [Tooltip("Multi-tap soft-shadow radius in world-space meters (XZ). The " +
                 "shadow mesh is redrawn shadowBlurSamples times with each tap " +
                 "offset into a Gauss disc of this radius. 0 = hard shadow.")]
        [Min(0f)] public float shadowBlurRadius = 0.05f;

        [Tooltip("Number of jittered taps when shadowBlurRadius > 0. More = " +
                 "smoother but linearly more draw calls per BT mesh. 1 = no blur.")]
        [Range(1, 16)] public int shadowBlurSamples = 8;

        public enum ShadowLightMode { OrthographicDown, Directional, Positional }

        [Header("Shadow light")]
        [Tooltip("OrthographicDown = vertices dropped straight to the floor plane " +
                 "(no light position needed). Directional = parallel rays along " +
                 "lightDirection (sun-style; shadows stretch toward the opposite " +
                 "direction). Positional = perspective projection from lightPosition " +
                 "(point-light style; shadows grow larger with vertex height).")]
        public ShadowLightMode lightMode = ShadowLightMode.OrthographicDown;

        [Tooltip("World-space light direction (will be normalized). Only used in " +
                 "Directional mode. Y must be negative (light pointing down). " +
                 "lightTransform.forward overrides this if assigned.")]
        public Vector3 lightDirection = new Vector3(0.2f, -1f, 0.1f);

        [Tooltip("World-space light position. Only used in Positional mode. Y must " +
                 "be above the casting geometry. lightTransform.position overrides " +
                 "this if assigned.")]
        public Vector3 lightPosition = new Vector3(0f, 3f, 0f);

        [Tooltip("Optional driver Transform. When assigned, its world position and " +
                 "forward direction take precedence over the manual lightPosition / " +
                 "lightDirection values — drop in a Light component or empty GO and " +
                 "move it in the Scene view to position the shadow interactively.")]
        public Transform lightTransform;

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
        private float _builtInnerBrightness = -1f;
        private bool _builtFit;
        private Vector3 _builtFitMin, _builtFitMax; // world AABB of the fitted grid

        // Cached list reused each LateUpdate to avoid the FindObjectsByType allocation.
        private static readonly List<PointCloudRenderer> s_rendererScratch = new List<PointCloudRenderer>(8);
        // Reused per-MultiLive sweep so GetComponentsInChildren doesn't allocate per frame.
        private static readonly List<MeshRenderer> s_meshRendererScratch = new List<MeshRenderer>(128);

        // Shader property IDs (shadow-only stamps; the filter uniforms come in
        // by copying the source renderer's own MaterialPropertyBlock).
        private static readonly int kFloorY      = Shader.PropertyToID("_FloorY");
        private static readonly int kUseReveal   = Shader.PropertyToID("_ShadowUseReveal");
        private static readonly int kShadowColor = Shader.PropertyToID("_ShadowColor");
        private static readonly int kBlurOffset  = Shader.PropertyToID("_BlurOffset");
        private static readonly int kLightMode   = Shader.PropertyToID("_LightMode");
        private static readonly int kLightDir    = Shader.PropertyToID("_LightDir");
        private static readonly int kLightPos    = Shader.PropertyToID("_LightPos");

        // Pre-computed Poisson-disc-like offsets in [-1,1] XZ space, scaled at
        // draw-time by shadowBlurRadius. Up to 16 entries are referenced; we
        // only consume the first shadowBlurSamples of them per draw.
        private static readonly Vector2[] s_blurOffsets =
        {
            new Vector2( 0.000f,  0.000f),
            new Vector2( 0.707f,  0.000f),
            new Vector2(-0.707f,  0.000f),
            new Vector2( 0.000f,  0.707f),
            new Vector2( 0.000f, -0.707f),
            new Vector2( 0.500f,  0.500f),
            new Vector2(-0.500f,  0.500f),
            new Vector2( 0.500f, -0.500f),
            new Vector2(-0.500f, -0.500f),
            new Vector2( 0.965f,  0.260f),
            new Vector2(-0.260f,  0.965f),
            new Vector2(-0.965f, -0.260f),
            new Vector2( 0.260f, -0.965f),
            new Vector2( 0.866f, -0.500f),
            new Vector2(-0.866f,  0.500f),
            new Vector2( 0.130f,  0.225f),
        };

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
                    // Fitted grid is built in WORLD space (lines snapped to world
                    // coordinates), so it draws with identity; the legacy grid
                    // lives in this Transform's local space and follows the anchor.
                    Matrix4x4 m = fitToBoundingBox && boundingBox != null
                        ? Matrix4x4.identity : transform.localToWorldMatrix;
                    Graphics.DrawMesh(_gridMesh, m, _gridMaterial,
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
            bool fit = fitToBoundingBox && boundingBox != null;
            Vector3 fitMin = Vector3.zero, fitMax = Vector3.zero;
            if (fit) ComputeBoxFootprint(out fitMin, out fitMax);

            if (_gridMesh != null
                && _builtFit == fit
                && _builtCells == gridCells
                && Mathf.Approximately(_builtSpacing, gridSpacing)
                && _builtGridColor == gridColor
                && _builtAxisColor == axisColor
                && Mathf.Approximately(_builtInnerBrightness, innerLineBrightness)
                && (!fit || ((_builtFitMin - fitMin).sqrMagnitude < 1e-8f &&
                             (_builtFitMax - fitMax).sqrMagnitude < 1e-8f)))
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
            Color innerColor = new Color(gridColor.r * innerLineBrightness,
                                         gridColor.g * innerLineBrightness,
                                         gridColor.b * innerLineBrightness,
                                         gridColor.a);
            if (fit) BuildFitGridMesh(_gridMesh, fitMin, fitMax, gridSpacing, gridColor, innerColor, axisColor);
            else BuildGridMesh(_gridMesh, gridCells, gridSpacing, gridColor, innerColor, axisColor);
            _builtFit = fit;
            _builtFitMin = fitMin;
            _builtFitMax = fitMax;
            _builtCells = gridCells;
            _builtSpacing = gridSpacing;
            _builtGridColor = gridColor;
            _builtAxisColor = axisColor;
            _builtInnerBrightness = innerLineBrightness;
        }

        // World AABB of the box's bottom face (handles a rotated OBB as its AABB;
        // our rebased convention keeps the box axis-aligned anyway). Y = floor.
        private void ComputeBoxFootprint(out Vector3 min, out Vector3 max)
        {
            var t = boundingBox.transform;
            min = Vector3.positiveInfinity;
            max = Vector3.negativeInfinity;
            for (int i = 0; i < 4; i++)
            {
                var corner = new Vector3((i & 1) == 0 ? -0.5f : 0.5f, -0.5f, (i & 2) == 0 ? -0.5f : 0.5f);
                Vector3 w = t.TransformPoint(corner);
                min = Vector3.Min(min, w);
                max = Vector3.Max(max, w);
            }
        }

        private void EnsureGridMaterial()
        {
            if (_gridMaterial != null) return;
            // Resources first — a bare Shader.Find returns null in a player build
            // (nothing references the shader, so it is stripped).
            var shader = Resources.Load<Shader>("PointCloudUnlit");
            if (shader == null) shader = Shader.Find("Orbbec/PointCloudUnlit");
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
            // Resources first: no serialized material references this shader, so a bare
            // Shader.Find returns null in a player build (stripped).
            var shader = Resources.Load<Shader>("PointCloudShadow");
            if (shader == null) shader = Shader.Find("Orbbec/PointCloudShadow");
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

            // Light parameters are constant for every shadow draw this frame, so
            // push them onto the shared shadow material once instead of stamping
            // every MPB. Transform overrides take precedence over the Vector3
            // Inspector values so the user can drag a Light / empty GO around the
            // Scene view and watch the shadow follow live.
            Vector3 lightDir = lightTransform != null
                ? lightTransform.forward
                : lightDirection;
            if (lightDir.sqrMagnitude < 1e-8f) lightDir = Vector3.down;
            lightDir.Normalize();
            Vector3 lightPos = lightTransform != null
                ? lightTransform.position
                : lightPosition;
            float lightModeFloat = lightMode == ShadowLightMode.OrthographicDown ? 0f
                                  : lightMode == ShadowLightMode.Directional ? 1f
                                  : 2f;
            _shadowMaterial.SetFloat(kLightMode, lightModeFloat);
            _shadowMaterial.SetVector(kLightDir, new Vector4(lightDir.x, lightDir.y, lightDir.z, 0f));
            _shadowMaterial.SetVector(kLightPos, new Vector4(lightPos.x, lightPos.y, lightPos.z, 0f));

            int blurSamples = Mathf.Clamp(shadowBlurSamples, 1, s_blurOffsets.Length);
            // When blur is disabled (radius=0 or samples=1) we still do one tap
            // at offset (0,0) with full alpha — the loop below handles both.
            bool doBlur = shadowBlurRadius > 0f && blurSamples > 1;
            if (!doBlur) blurSamples = 1;

            // 1) Live PointCloudRenderers — typical when devices are connected.
            // The MR check is what keeps the reflection honest: PointCloudView
            // (and the experience's per-state show/hide) toggles MeshRenderer
            // .enabled, so a hidden cloud must not reflect either.
            var live = FindObjectsByType<PointCloudRenderer>(FindObjectsSortMode.None);
            for (int i = 0; i < live.Length; i++)
            {
                var pcr = live[i];
                if (pcr == null || !pcr.isActiveAndEnabled) continue;
                var mr = pcr.GetComponent<MeshRenderer>();
                if (mr == null || !mr.enabled) continue;
                var mf = pcr.GetComponent<MeshFilter>();
                var mesh = mf != null ? mf.sharedMesh : null;
                if (mesh == null) continue;

                DrawWithBlurTaps(mesh, pcr.transform.localToWorldMatrix, mr, floorY,
                                 blurSamples, doBlur, applyReveal: true);
            }

            // 2) Playback meshes — `_Playback_<serial>` children of every active
            // SensorRecorder. Their MPB is filled by the same
            // PointCloudShaderFilters.Apply as the live path, so copying it
            // (DrawWithBlurTaps) keeps the shadow point-for-point with the
            // visible cloud.
            var recorders = FindObjectsByType<SensorRecorder>(FindObjectsSortMode.None);
            for (int r = 0; r < recorders.Length; r++)
            {
                var rec = recorders[r];
                if (rec == null || !rec.isActiveAndEnabled) continue;
                int childCount = rec.transform.childCount;
                for (int c = 0; c < childCount; c++)
                {
                    var child = rec.transform.GetChild(c);
                    if (child == null || !child.gameObject.activeInHierarchy) continue;
                    if (!child.name.StartsWith("_Playback_")) continue;
                    var mr = child.GetComponent<MeshRenderer>();
                    if (mr == null || !mr.enabled) continue;
                    var mf = child.GetComponent<MeshFilter>();
                    var mesh = mf != null ? mf.sharedMesh : null;
                    if (mesh == null || mesh.vertexCount == 0) continue;

                    DrawWithBlurTaps(mesh, child.localToWorldMatrix, mr, floorY,
                                     blurSamples, doBlur, applyReveal: true);
                }
            }

            // 3) Body-tracking visuals — joint spheres, bone tubes, joint trails,
            // bone-interp trails. All live as MeshRenderer descendants of every
            // SkeletonMerger transform. The BodyTracking asmdef references
            // PointCloud (not the other way round), so we can't type-reference
            // SkeletonMerger here without a cyclic asmdef. Match by full
            // type name instead — there's only ever one or two MultiLive
            // instances so the FindObjectsByType<MonoBehaviour> walk is cheap.
            // The shadow uses the same PointCloudShadow shader with filters
            // disabled: the shader still collapses every vertex Y to _FloorY,
            // so triangle meshes (skeleton geometry) flatten to their XZ
            // footprint on the floor plane.
            var allMb = FindObjectsByType<MonoBehaviour>(FindObjectsSortMode.None);
            for (int b = 0; b < allMb.Length; b++)
            {
                var mb = allMb[b];
                if (mb == null || !mb.isActiveAndEnabled) continue;
                if (mb.GetType().FullName != "BodyTracking.SkeletonMerger") continue;
                mb.GetComponentsInChildren(false, s_meshRendererScratch);
                for (int i = 0; i < s_meshRendererScratch.Count; i++)
                {
                    var mr = s_meshRendererScratch[i];
                    if (mr == null || !mr.enabled) continue;
                    var mf = mr.GetComponent<MeshFilter>();
                    var mesh = mf != null ? mf.sharedMesh : null;
                    if (mesh == null || mesh.vertexCount == 0) continue;

                    // applyReveal: false — skeleton meshes are triangles, and a
                    // per-vertex reveal clip stretches straddling triangles into
                    // screen-crossing slivers; displayed bones aren't reveal-
                    // clipped either, so the reflection matches by skipping it.
                    DrawWithBlurTaps(mesh, mr.transform.localToWorldMatrix, mr,
                                     floorY, blurSamples, doBlur, applyReveal: false);
                }
                s_meshRendererScratch.Clear();
            }
        }

        // Draws `mesh` `blurSamples` times via the shadow material, each tap
        // jittered into a Gauss disc of `shadowBlurRadius` meters in world XZ.
        // Per-tap alpha is shadowColor.a / blurSamples so the integral over taps
        // matches a single hard-shadow tap. doBlur=false collapses to one draw
        // with offset (0,0) and full alpha.
        //
        // The filter uniforms are not rebuilt here: the source renderer's own
        // MaterialPropertyBlock — filled by PointCloudShaderFilters.Apply on the
        // display path (OBB, decimation, capsules, floor mask, motion field) —
        // is copied wholesale, so the reflection keeps exactly the vertices the
        // visible mesh keeps. Renderers that never set one (skeleton meshes)
        // yield an empty block and the shadow shader's pass-through defaults.
        private void DrawWithBlurTaps(Mesh mesh, Matrix4x4 modelMatrix,
                                      MeshRenderer sourceRenderer, float floorY,
                                      int blurSamples, bool doBlur, bool applyReveal)
        {
            float perTapAlpha = doBlur ? (shadowColor.a / blurSamples) : shadowColor.a;
            for (int t = 0; t < blurSamples; t++)
            {
                Vector2 offsetUnit = doBlur ? s_blurOffsets[t] : Vector2.zero;
                Vector4 offsetWorld = new Vector4(offsetUnit.x * shadowBlurRadius, 0f,
                                                  offsetUnit.y * shadowBlurRadius, 0f);
                sourceRenderer.GetPropertyBlock(_shadowMpb);
                _shadowMpb.SetFloat(kFloorY, floorY);
                _shadowMpb.SetFloat(kUseReveal, applyReveal ? 1f : 0f);
                _shadowMpb.SetColor(kShadowColor,
                    new Color(shadowColor.r, shadowColor.g, shadowColor.b, perTapAlpha));
                _shadowMpb.SetVector(kBlurOffset, offsetWorld);
                Graphics.DrawMesh(mesh, modelMatrix, _shadowMaterial,
                                  renderLayer, null, 0, _shadowMpb, ShadowCastingMode.Off, false);
            }
        }

        private void DestroyOwnedAssets()
        {
            if (_gridMesh != null)
            {
                PointCloudUtil.DestroySafe(_gridMesh);
                _gridMesh = null;
            }
            if (_gridMaterial != null)
            {
                PointCloudUtil.DestroySafe(_gridMaterial);
                _gridMaterial = null;
            }
            if (_shadowMaterial != null)
            {
                PointCloudUtil.DestroySafe(_shadowMaterial);
                _shadowMaterial = null;
            }
            _builtCells = -1;
            _builtSpacing = -1f;
        }

        // WORLD-space grid clipped to the box footprint: interior lines sit on
        // world multiples of `spacing` (so the world origin is always a line
        // crossing when inside the box), plus the border rectangle. The world
        // axis lines (x=0 / z=0) get axisColor; interior non-axis lines get
        // innerColor, the border keeps gridColor.
        private static void BuildFitGridMesh(Mesh mesh, Vector3 min, Vector3 max, float spacing,
                                             Color gridColor, Color innerColor, Color axisColor)
        {
            spacing = Mathf.Max(0.001f, spacing);
            float y = min.y;
            var verts = new List<Vector3>(128);
            var colors = new List<Color>(128);

            void AddLine(Vector3 a, Vector3 b, Color c)
            {
                verts.Add(a); verts.Add(b);
                colors.Add(c); colors.Add(c);
            }

            // interior lines snapped to world multiples of spacing
            for (int k = Mathf.CeilToInt(min.x / spacing); k * spacing <= max.x + 1e-4f; k++)
            {
                float x = k * spacing;
                AddLine(new Vector3(x, y, min.z), new Vector3(x, y, max.z),
                        k == 0 ? axisColor : innerColor);
            }
            for (int k = Mathf.CeilToInt(min.z / spacing); k * spacing <= max.z + 1e-4f; k++)
            {
                float z = k * spacing;
                AddLine(new Vector3(min.x, y, z), new Vector3(max.x, y, z),
                        k == 0 ? axisColor : innerColor);
            }
            // border rectangle so the grid reads as the sensing area's edge
            AddLine(new Vector3(min.x, y, min.z), new Vector3(max.x, y, min.z), gridColor);
            AddLine(new Vector3(min.x, y, max.z), new Vector3(max.x, y, max.z), gridColor);
            AddLine(new Vector3(min.x, y, min.z), new Vector3(min.x, y, max.z), gridColor);
            AddLine(new Vector3(max.x, y, min.z), new Vector3(max.x, y, max.z), gridColor);

            var indices = new int[verts.Count];
            for (int i = 0; i < indices.Length; i++) indices[i] = i;

            mesh.Clear();
            mesh.SetVertices(verts);
            mesh.SetColors(colors);
            mesh.SetIndices(indices, MeshTopology.Lines, 0);
            mesh.RecalculateBounds();
        }

        // Square line grid centered on (0,0,0) in the local XZ plane. The two axis lines
        // (x=0 and z=0) get the axisColor, the outermost (border) lines keep gridColor,
        // every other line gets innerColor.
        private static void BuildGridMesh(Mesh mesh, int cells, float spacing,
                                          Color gridColor, Color innerColor, Color axisColor)
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
                bool isBorder = i == 0 || i == cells;
                Color c = isAxis ? axisColor : isBorder ? gridColor : innerColor;
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
                bool isBorder = j == 0 || j == cells;
                Color c = isAxis ? axisColor : isBorder ? gridColor : innerColor;
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
