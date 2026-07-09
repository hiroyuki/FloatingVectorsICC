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
        // Fake round ribbon: shade the flat camera-facing quad with a cylinder-
        // equivalent normal so the curve reads as a tube (no extra geometry).
        // 0 = original flat emissive look, 1 = full rounded shading. Lets the
        // artist A/B against the old look (see realtime-performance-tuning.md).
        _Round      ("Round shading", Range(0, 1)) = 1.0
        // Rim brightens the tube edge in the curve's OWN colour (no white add),
        // so rounding stays saturated instead of washing out.
        _RimPower   ("Rim Power", Float) = 2.5
        _RimBoost   ("Rim Boost", Range(0, 2)) = 0.5
        // Alpha at the OLD end of each curve (age 0). 1 = no fade; 0 = the past
        // tip dissolves completely, so trails read as fading in from the past.
        _TailAlpha  ("Tail Alpha (past end)", Range(0, 1)) = 0.0
    }
    SubShader
    {
        // Transparent so the tail fade actually blends over the mesh/cloud behind
        // it. ZWrite off (standard for transparents); ribbons stop occluding each
        // other, which for thin dense strokes reads fine and avoids sort artefacts.
        Tags { "RenderType" = "Transparent" "Queue" = "Transparent" "IgnoreProjector" = "True" }
        LOD 100

        Pass
        {
            Cull Off
            ZWrite Off
            ZTest LEqual
            Blend SrcAlpha OneMinusSrcAlpha

            CGPROGRAM
            #pragma vertex   vert
            #pragma fragment frag
            #include "UnityCG.cginc"

            // Matches LineVert in MotionCurvesBuild.compute (pos + age + colour + pad, 32B).
            struct LineVert { float3 p; float age; float3 c; float pad; };
            StructuredBuffer<LineVert> _Verts;

            float _Brightness;
            float _Width;
            float _Round;
            float _RimPower;
            float _RimBoost;
            float _TailAlpha;

            struct V2F
            {
                float4 pos  : SV_POSITION;
                float3 vcol : TEXCOORD0;
                float3 worldPos : TEXCOORD1;
                float3 sideDir  : TEXCOORD2; // unit ribbon-width direction (world)
                float  u        : TEXCOORD3; // cross-ribbon coord in [-1, +1]
                float  age      : TEXCOORD4; // 0 = oldest history end, 1 = newest
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
                    o.worldPos = float3(0, 0, 0);
                    o.sideDir = float3(0, 1, 0);
                    o.u = 0;
                    o.age = 0;
                    return o;
                }

                // Camera-facing ribbon: offset each end perpendicular to the segment
                // tangent within the plane facing the camera.
                float3 tang = axis / len;
                float3 mid  = (a + b) * 0.5;
                float3 view = normalize(_WorldSpaceCameraPos.xyz - mid);
                // Guard the degenerate case where the segment points at the camera (cross -> ~0):
                // fall back to a world axis that isn't parallel to the tangent so the ribbon keeps a width.
                float3 sideRaw = cross(tang, view);
                if (dot(sideRaw, sideRaw) < 1e-10)
                {
                    sideRaw = cross(tang, float3(0, 1, 0));
                    if (dot(sideRaw, sideRaw) < 1e-10) sideRaw = cross(tang, float3(1, 0, 0));
                }
                float3 sideUnit = normalize(sideRaw);
                float3 side = sideUnit * (_Width * 0.5);

                float3 a0 = a - side, a1 = a + side;
                float3 b0 = b - side, b1 = b + side;

                // Two triangles: (a0, a1, b0), (b0, a1, b1).
                // u = -1 on the a0/b0 edge, +1 on the a1/b1 edge (ribbon width).
                float3 wp; float3 col; float uu; float age;
                if      (corner == 0u) { wp = a0; col = A.c; uu = -1; age = A.age; }
                else if (corner == 1u) { wp = a1; col = A.c; uu = +1; age = A.age; }
                else if (corner == 2u) { wp = b0; col = B.c; uu = -1; age = B.age; }
                else if (corner == 3u) { wp = b0; col = B.c; uu = -1; age = B.age; }
                else if (corner == 4u) { wp = a1; col = A.c; uu = +1; age = A.age; }
                else                   { wp = b1; col = B.c; uu = +1; age = B.age; }

                o.pos = mul(UNITY_MATRIX_VP, float4(wp, 1.0));
                o.vcol = col;
                o.worldPos = wp;
                o.sideDir = sideUnit;
                o.u = uu;
                o.age = age;
                return o;
            }

            fixed4 frag(V2F i) : SV_Target
            {
                float3 albedo = saturate(i.vcol);
                float3 flatCol = albedo * _Brightness;

                // Cylinder-equivalent normal: the flat quad faces the camera, so
                // the visible half-cylinder's normal sweeps from +side (u=+1)
                // through the view direction (u=0) to -side (u=-1). No geometry.
                float3 view = normalize(_WorldSpaceCameraPos.xyz - i.worldPos);
                float3 sideUnit = normalize(i.sideDir);
                float uu = clamp(i.u, -1.0, 1.0);
                float3 N = normalize(sideUnit * uu + view * sqrt(saturate(1.0 - uu * uu)));

                // Value-only shading: RGB * scalar preserves HSV saturation, so
                // the tube reads round via brightness variation without dulling
                // the colour. The rim lifts the edge in the curve's own colour
                // (never adds white), keeping it vivid. High ambient floor (0.55)
                // so the shaded side stays close to the original emissive look.
                float3 L = normalize(float3(0.35, 0.85, 0.40));
                float shade = saturate(dot(N, L)) * 0.45 + 0.55;   // 0.55..1.0
                float rim   = pow(1.0 - saturate(dot(N, view)), _RimPower);
                float3 rounded = albedo * _Brightness * (shade + rim * _RimBoost);

                float3 rgb = lerp(flatCol, rounded, saturate(_Round));
                // Tail fade: alpha ramps from _TailAlpha at the oldest history
                // point up to opaque at the newest, so the trail dissolves into
                // the past instead of ending in a hard cut.
                float alpha = lerp(saturate(_TailAlpha), 1.0, saturate(i.age));
                return fixed4(rgb, alpha);
            }
            ENDCG
        }
    }
}
