// Procedural shader for the TSDF marching-cubes mesh. Reads vertex world
// positions out of a StructuredBuffer (set per-frame by TSDFMeshRenderer)
// and shades flat using ddx/ddy-derived face normals — enough to see the
// surface during the step 1 smoke phase. Cull Off so the user can inspect
// both sides while the iso level / sign convention is still being verified.

Shader "TSDF/TSDFMesh"
{
    Properties
    {
        _Color      ("Base Color (tint)", Color)  = (1, 1, 1, 1)
        _RimColor   ("Rim Color",  Color)  = (1, 1, 1, 1)
        _RimPower   ("Rim Power",  Float)  = 2.5
        _Saturation ("Saturation", Range(0, 3))   = 1.0
        _Brightness ("Brightness", Range(0, 3))   = 1.0
        _Gamma      ("Gamma",      Range(0.2, 3)) = 1.0
    }
    SubShader
    {
        Tags { "RenderType"="Opaque" "Queue"="Geometry" "IgnoreProjector"="True" }
        LOD 100

        Pass
        {
            Cull Off
            ZWrite On
            ZTest LEqual

            CGPROGRAM
            #pragma vertex   vert
            #pragma fragment frag
            #include "UnityCG.cginc"

            // Matches the Tri struct emitted by TSDFMarchingCubes.compute.
            // Per vertex: position + colour. 6 float3 per triangle (72 bytes);
            // SV_VertexID indexes vertices across all triangles, decompose into
            // (triIdx, cornerIdx).
            struct Tri { float3 p0; float3 c0; float3 p1; float3 c1; float3 p2; float3 c2; };
            StructuredBuffer<Tri> _Triangles;

            float4 _Color;
            float4 _RimColor;
            float  _RimPower;
            float  _Saturation;
            float  _Brightness;
            float  _Gamma;

            struct V2F
            {
                float4 pos      : SV_POSITION;
                float3 worldPos : TEXCOORD0;
                float3 viewDir  : TEXCOORD1;
                float3 vcol     : TEXCOORD2;
            };

            V2F vert(uint vid : SV_VertexID)
            {
                V2F o;
                uint triIdx = vid / 3u;
                uint cornerIdx = vid - triIdx * 3u; // = vid % 3
                Tri t = _Triangles[triIdx];
                float3 wp  = (cornerIdx == 0u) ? t.p0 : (cornerIdx == 1u) ? t.p1 : t.p2;
                float3 col = (cornerIdx == 0u) ? t.c0 : (cornerIdx == 1u) ? t.c1 : t.c2;
                o.pos = mul(UNITY_MATRIX_VP, float4(wp, 1.0));
                o.worldPos = wp;
                o.viewDir = normalize(_WorldSpaceCameraPos.xyz - wp);
                o.vcol = col;
                return o;
            }

            fixed4 frag(V2F i) : SV_Target
            {
                // Flat normal from screen-space derivatives — no per-vertex
                // normal buffer needed, and stable across MC reconnections.
                float3 dx = ddx(i.worldPos);
                float3 dy = ddy(i.worldPos);
                float3 N = normalize(cross(dx, dy));
                // Always face the camera (because we Cull Off both sides).
                if (dot(N, i.viewDir) < 0) N = -N;

                float3 L = normalize(float3(0.35, 0.85, 0.40));
                float diffuse = saturate(dot(N, L)) * 0.7 + 0.3;
                float rim = pow(1.0 - saturate(dot(N, i.viewDir)), _RimPower);

                // Per-vertex camera colour (baked at integration), colour-graded
                // then shaded by the flat normal. Grading order: gamma (fix
                // sRGB/linear darkness) -> saturation (luma-preserving boost) ->
                // tint -> brightness, then lighting.
                float3 albedo = saturate(i.vcol);
                albedo = pow(albedo, _Gamma);
                float luma = dot(albedo, float3(0.2126, 0.7152, 0.0722));
                albedo = max(0.0, lerp(float3(luma, luma, luma), albedo, _Saturation));
                albedo *= _Color.rgb;
                float3 rgb = albedo * diffuse * _Brightness + _RimColor.rgb * rim * 0.35;
                return fixed4(rgb, 1.0);
            }
            ENDCG
        }
    }
}
