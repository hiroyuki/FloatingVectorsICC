// Debug visualiser. One sphere instance per voxel, driven by Graphics
// .DrawMeshInstancedIndirect. Reads the TSDF volume's StructuredBuffer
// directly so the colour reflects the live (or frozen) buffer state without
// any CPU readback.
//
// Colour ramp:
//   sdf ≈  0       → red          (near surface — Marching Cubes will look here)
//   sdf →  +tau    → white        (free space)
//   sdf →  -tau    → black        (behind surface)
//   weight == 0    → offscreened  (no observation; sphere skipped via SV_POSITION
//                                  outside the clip volume so the rasteriser drops it)

Shader "TSDF/Voxel"
{
    Properties
    {
        _Scale ("Sphere scale (relative to voxelSize)", Range(0.05, 1.0)) = 0.30
    }
    SubShader
    {
        Tags { "RenderType"="Opaque" "Queue"="Geometry+10" "IgnoreProjector"="True" }
        LOD 100

        Pass
        {
            Cull Back
            ZWrite On
            ZTest LEqual

            CGPROGRAM
            #pragma vertex   vert
            #pragma fragment frag
            #pragma multi_compile_instancing
            #include "UnityCG.cginc"

            StructuredBuffer<float2> _Voxels;
            float4x4 _WorldFromVoxel;
            // float3 (not int3) so Material.SetVector binds cleanly; the kernel-
            // side counterpart in TSDFIntegrate uses int3 because compute shaders
            // can use SetInts, but Material.SetInts doesn't exist for vectors.
            float3 _Dim;
            float _VoxelSize;
            float _Tau;
            float _Scale;
            // sdf >= _HideAboveSdf => skip drawing (push offscreen). Used to hide
            // the "free space" sphere shell so the surface band stays visible.
            // Default tau so nothing is hidden unless the user opts in.
            float _HideAboveSdf;

            struct appdata
            {
                float4 vertex : POSITION;
                float3 normal : NORMAL;
                uint   inst   : SV_InstanceID;
            };

            struct v2f
            {
                float4 pos        : SV_POSITION;
                float3 col        : TEXCOORD0;
                float3 normalWS   : TEXCOORD1;
            };

            v2f vert(appdata IN)
            {
                v2f o;

                // Linear instance id -> voxel (i, j, k). Matches the kernel /
                // C# linear index = x + dimX*(y + dimY*z).
                uint id = IN.inst;
                uint dx = (uint)_Dim.x;
                uint dy = (uint)_Dim.y;
                uint i = id % dx;
                uint j = (id / dx) % dy;
                uint k = id / (dx * dy);

                float2 data = _Voxels[id];
                float sdf = data.x;
                float w   = data.y;

                // Per-instance world centre via the same matrix the integrate
                // kernel uses, so this debug position lines up exactly with what
                // was sampled.
                float3 idxC = float3((float)i + 0.5, (float)j + 0.5, (float)k + 0.5);
                float3 wp = mul(_WorldFromVoxel, float4(idxC, 1.0)).xyz;

                // Sphere mesh is centred at origin in object space; scale by
                // voxelSize * _Scale then offset to world centre.
                float3 worldVtx = wp + IN.vertex.xyz * _VoxelSize * _Scale;

                // Skip unobserved voxels (weight 0) AND voxels whose sdf is
                // above the user-set "free space" cull threshold. Both routes
                // punt the vertex out of the clip volume.
                bool hide = (w <= 0.0) || (sdf >= _HideAboveSdf);
                if (hide)
                    o.pos = float4(2.0, 2.0, 2.0, 1.0);
                else
                    o.pos = mul(UNITY_MATRIX_VP, float4(worldVtx, 1.0));

                // Color ramp via the signed normalised sdf t in [-1, +1].
                float t = clamp(sdf / max(_Tau, 1e-5), -1.0, 1.0);
                float3 red   = float3(1.0, 0.20, 0.20);
                float3 white = float3(1.0, 1.0,  1.0);
                float3 black = float3(0.0, 0.0,  0.0);
                o.col = (t >= 0.0)
                    ? lerp(red, white, t)
                    : lerp(red, black, -t);

                o.normalWS = IN.normal;
                return o;
            }

            fixed4 frag(v2f IN) : SV_Target
            {
                // Cheap shading so the spheres read as 3D rather than flat
                // blobs. Light from above-front.
                float3 L = normalize(float3(0.4, 0.85, 0.35));
                float diffuse = saturate(dot(normalize(IN.normalWS), L)) * 0.55 + 0.55;
                return fixed4(IN.col * diffuse, 1.0);
            }
            ENDCG
        }
    }
}
