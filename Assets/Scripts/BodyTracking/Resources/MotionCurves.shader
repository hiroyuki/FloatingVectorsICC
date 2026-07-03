// Procedural line shader for the point-cloud motion curves (Phase 2).
// Reads per-vertex position + colour from a StructuredBuffer filled by
// MotionCurvesBuild.compute and drawn with Graphics.DrawProceduralIndirect
// (MeshTopology.Lines). Built-in RP, matching the rest of the project.
// Zero-length segments (culled / unused seed slots) produce no fragments.

Shader "Orbbec/MotionCurves"
{
    Properties
    {
        _Brightness ("Brightness", Range(0, 3)) = 1.0
    }
    SubShader
    {
        Tags { "RenderType" = "Opaque" "Queue" = "Geometry" "IgnoreProjector" = "True" }
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

            // Matches LineVert in MotionCurvesBuild.compute (pos + colour, 24B).
            struct LineVert { float3 p; float3 c; };
            StructuredBuffer<LineVert> _Verts;

            float _Brightness;

            struct V2F
            {
                float4 pos  : SV_POSITION;
                float3 vcol : TEXCOORD0;
            };

            V2F vert(uint vid : SV_VertexID)
            {
                V2F o;
                LineVert v = _Verts[vid];
                o.pos = mul(UNITY_MATRIX_VP, float4(v.p, 1.0));
                o.vcol = v.c;
                return o;
            }

            fixed4 frag(V2F i) : SV_Target
            {
                return fixed4(saturate(i.vcol) * _Brightness, 1.0);
            }
            ENDCG
        }
    }
}
