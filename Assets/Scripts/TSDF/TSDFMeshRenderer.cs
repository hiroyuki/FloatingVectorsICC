// Marching Cubes + DrawProceduralIndirect renderer for the TSDF volume.
// Holds an AppendStructuredBuffer<VertexOut> sized to a vertex budget, runs
// the MarchCubes kernel each frame, then queues a DrawProceduralIndirect
// against a vertex-count args buffer populated via ComputeBuffer.CopyCount.
//
// Step 1c of FloatingVectorsICC_v2_TSDF_spec.md §9 — first end-to-end shot
// where the user actually sees something. Performance is intentionally not
// tuned; spec §5 mentions "重い場合は数フレームおきに間引くフォールバック" and that
// hook (mcEveryNFrames) is exposed below.

using UnityEngine;
using UnityEngine.Rendering;

namespace TSDF
{
    /// <summary>
    /// Dispatches Marching Cubes and renders the result. Attach next to a
    /// <see cref="TSDFVolume"/> (or wire one via the Inspector).
    /// </summary>
    [DefaultExecutionOrder(10)]
    public sealed class TSDFMeshRenderer : MonoBehaviour
    {
        [Tooltip("Volume to extract the iso-surface from. Auto-located on enable.")]
        public TSDFVolume volume;

        [Tooltip("Material that uses the TSDF/TSDFMesh shader. Auto-created on " +
                 "enable if left null. Exposed so the user can swap in a custom " +
                 "shader later (e.g. URP Lit + buffer access).")]
        public Material material;

        [Tooltip("Max output vertices. 3 verts per triangle — 1 M = 333k tris, " +
                 "~12 MB at 12 bytes per vertex. Bump up if the volume + voxel " +
                 "size yields more surface than this can hold (the Append silently " +
                 "drops once the buffer is full).")]
        [Min(1024)] public int maxVertices = 1_000_000;

        [Tooltip("Iso-level for MC. 0 = the surface where sdf changes sign. " +
                 "Match the TSDF integration convention; only override for debug.")]
        public float isoLevel = 0f;

        [Tooltip("Minimum corner weight required for an MC cell to emit triangles. " +
                 "Spec §5: weight == 0 voxels are invalid. > 0 also rejects cells " +
                 "that sit on the integration boundary where corner data is partial.")]
        [Min(0f)] public float minWeight = 0.5f;

        [Tooltip("MC dispatch cadence. 1 = every frame. Higher values skip frames " +
                 "for slower GPUs (spec §5 fallback flag).")]
        [Min(1)] public int mcEveryNFrames = 1;

        [Tooltip("Log per-second vertex / triangle counts.")]
        public bool diagnosticLogging = false;

        private ComputeShader _shader;
        private int _kernel;
        private ComputeBuffer _vertices;   // AppendStructuredBuffer<VertexOut>
        private ComputeBuffer _args;       // IndirectArguments: [vertCount, 1, 0, 0]
        private int _frameCounter;
        private Bounds _drawBounds;

        // Diagnostics — read back the vertex count once per second via
        // ComputeBuffer.CopyCount → small CPU-visible buffer to avoid the
        // GetData stall on the main draw buffer.
        private ComputeBuffer _diagCountBuf;
        private float _diagWindowStart;
        private int _diagDispatchesThisWindow;

        private void OnEnable()
        {
            if (volume == null) volume = FindAnyObjectByType<TSDFVolume>(FindObjectsInactive.Include);
            if (volume == null)
            {
                Debug.LogError("[TSDFMeshRenderer] No TSDFVolume in scene; disabling.", this);
                enabled = false;
                return;
            }

            EnsureShader();
            EnsureMaterial();
            EnsureBuffers();
        }

        private void OnDisable()
        {
            ReleaseBuffers();
        }

        private void EnsureShader()
        {
            if (_shader != null) return;
            _shader = Resources.Load<ComputeShader>("TSDFMarchingCubes");
            if (_shader == null)
            {
                Debug.LogError("[TSDFMeshRenderer] Compute shader \"Resources/TSDFMarchingCubes.compute\" not found.", this);
                enabled = false;
                return;
            }
            _kernel = _shader.FindKernel("MarchCubes");
        }

        private void EnsureMaterial()
        {
            if (material != null) return;
            var shader = Shader.Find("TSDF/TSDFMesh");
            if (shader == null)
            {
                Debug.LogError("[TSDFMeshRenderer] Shader \"TSDF/TSDFMesh\" not found. " +
                               "Verify TSDFMesh.shader is under Assets/Scripts/TSDF/ and " +
                               "imported.", this);
                enabled = false;
                return;
            }
            material = new Material(shader) { name = "TSDFMesh (auto)", hideFlags = HideFlags.DontSave };
        }

        private void EnsureBuffers()
        {
            // VertexOut = 1x float3 = 12 bytes.
            const int vertexStride = 12;
            if (_vertices == null || _vertices.count != maxVertices)
            {
                _vertices?.Release();
                _vertices = new ComputeBuffer(maxVertices, vertexStride, ComputeBufferType.Append);
            }
            if (_args == null)
            {
                _args = new ComputeBuffer(4, sizeof(uint), ComputeBufferType.IndirectArguments);
                _args.SetData(new uint[] { 0, 1, 0, 0 });
            }
            if (_diagCountBuf == null)
            {
                _diagCountBuf = new ComputeBuffer(1, sizeof(uint), ComputeBufferType.Raw);
            }
        }

        private void ReleaseBuffers()
        {
            _vertices?.Release(); _vertices = null;
            _args?.Release(); _args = null;
            _diagCountBuf?.Release(); _diagCountBuf = null;
        }

        private void Update()
        {
            if (volume == null || volume.VoxelBuffer == null) return;
            _frameCounter++;
            if (_frameCounter % mcEveryNFrames != 0) return;

            EnsureBuffers();
            DispatchMarchCubes();
            QueueDraw();

            if (diagnosticLogging) PerSecondDiag();
        }

        private void DispatchMarchCubes()
        {
            _vertices.SetCounterValue(0);

            _shader.SetBuffer(_kernel, "_Voxels", volume.VoxelBuffer);
            _shader.SetInts("_Dim", volume.Dim.x, volume.Dim.y, volume.Dim.z);
            _shader.SetMatrix("_WorldFromVoxel", volume.WorldFromVoxel);
            _shader.SetFloat("_IsoLevel", isoLevel);
            _shader.SetFloat("_MinWeight", minWeight);
            _shader.SetBuffer(_kernel, "_Vertices", _vertices);

            // One thread per cell ((dim-1)^3 cells). 4x4x4 threadgroups.
            int gx = Mathf.CeilToInt((volume.Dim.x - 1) / 4f);
            int gy = Mathf.CeilToInt((volume.Dim.y - 1) / 4f);
            int gz = Mathf.CeilToInt((volume.Dim.z - 1) / 4f);
            _shader.Dispatch(_kernel, Mathf.Max(1, gx), Mathf.Max(1, gy), Mathf.Max(1, gz));

            // Copy the Append counter directly into args[0] — that IS the
            // vertex count (one Append per emitted vertex in the kernel).
            ComputeBuffer.CopyCount(_vertices, _args, 0);
            _diagDispatchesThisWindow++;
        }

        private void QueueDraw()
        {
            material.SetBuffer("_Vertices", _vertices);
            UpdateDrawBounds();
            // bounds is in WORLD space; DrawProceduralIndirect uses it for
            // frustum culling only — over-sizing is fine.
            Graphics.DrawProceduralIndirect(material, _drawBounds, MeshTopology.Triangles, _args, 0,
                                            camera: null, properties: null,
                                            castShadows: ShadowCastingMode.Off, receiveShadows: false,
                                            layer: gameObject.layer);
        }

        private void UpdateDrawBounds()
        {
            // The bbox-aligned grid: cover the world AABB of the rotated voxel
            // volume conservatively. Use TSDFVolume's WorldFromVoxel matrix to
            // get the eight grid corners and take the AABB.
            Vector3 min = Vector3.positiveInfinity;
            Vector3 max = Vector3.negativeInfinity;
            for (int b = 0; b < 8; b++)
            {
                float i = (b & 1) != 0 ? volume.Dim.x : 0f;
                float j = (b & 2) != 0 ? volume.Dim.y : 0f;
                float k = (b & 4) != 0 ? volume.Dim.z : 0f;
                Vector3 w = volume.WorldFromVoxel.MultiplyPoint(new Vector3(i, j, k));
                min = Vector3.Min(min, w);
                max = Vector3.Max(max, w);
            }
            _drawBounds = new Bounds((min + max) * 0.5f, max - min);
        }

        private void PerSecondDiag()
        {
            float now = Time.realtimeSinceStartup;
            if (_diagWindowStart == 0f) _diagWindowStart = now;
            if (now - _diagWindowStart < 1f) return;

            // Synchronously read the latest args[0] = vertex count. One-time
            // per-second stall, only when diagnosticLogging is on.
            var args = new uint[4];
            _args.GetData(args);
            int verts = (int)args[0];
            int tris = verts / 3;
            Debug.Log($"[TSDFMeshRenderer] dispatches={_diagDispatchesThisWindow}/s vertices={verts} triangles={tris} (cap={maxVertices})", this);
            _diagDispatchesThisWindow = 0;
            _diagWindowStart = now;
        }
    }
}
