// Procedural ribbon shader for the point-cloud motion curves.
// The compute (MotionCurvesBuild) still fills _Verts as line-segment endpoint
// pairs (LineVert = pos + colour, 24B). This shader expands each segment into a
// camera-facing quad (2 triangles, 6 verts) of width _Width, so the curves read
// as ribbons instead of 1px lines — the Catmull-Rom smoothness and colour show.
// Drawn with Graphics.DrawProceduralIndirect (MeshTopology.Triangles), vertex
// count = segments * 6. Degenerate (zero-length) segments are clipped offscreen.
// Built-in RP, matching the rest of the project.

Shader "Orbbec/MotionCurves"
{
    Properties
    {
        _Brightness ("Brightness", Range(0, 3)) = 1.0
        _Width      ("Ribbon Width (m)", Range(0, 0.05)) = 0.004
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
            float _Width;

            struct V2F
            {
                float4 pos  : SV_POSITION;
                float3 vcol : TEXCOORD0;
            };

            V2F vert(uint vid : SV_VertexID)
            {
                V2F o;
                uint seg    = vid / 6u;
                uint corner = vid - seg * 6u;

                LineVert A = _Verts[seg * 2u];
                LineVert B = _Verts[seg * 2u + 1u];
                float3 a = A.p;
                float3 b = B.p;

                float3 axis = b - a;
                float  len  = length(axis);
                if (len < 1e-6)
                {
                    // Degenerate / unused slot -> push offscreen so nothing rasterises.
                    o.pos = float4(2, 2, 2, 1);
                    o.vcol = float3(0, 0, 0);
                    return o;
                }

                // Camera-facing ribbon: offset each end perpendicular to the segment
                // tangent within the plane facing the camera.
                float3 tang = axis / len;
                float3 mid  = (a + b) * 0.5;
                float3 view = normalize(_WorldSpaceCameraPos.xyz - mid);
                float3 side = normalize(cross(tang, view)) * (_Width * 0.5);

                float3 a0 = a - side, a1 = a + side;
                float3 b0 = b - side, b1 = b + side;

                // Two triangles: (a0, a1, b0), (b0, a1, b1).
                float3 wp; float3 col;
                if      (corner == 0u) { wp = a0; col = A.c; }
                else if (corner == 1u) { wp = a1; col = A.c; }
                else if (corner == 2u) { wp = b0; col = B.c; }
                else if (corner == 3u) { wp = b0; col = B.c; }
                else if (corner == 4u) { wp = a1; col = A.c; }
                else                   { wp = b1; col = B.c; }

                o.pos = mul(UNITY_MATRIX_VP, float4(wp, 1.0));
                o.vcol = col;
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
