// Progress ring for DwellSphere: a camera-facing (billboard) circle drawn
// clock-style over the sphere. The mesh holds the arc in LOCAL XY meters
// around the object origin; the vertex shader re-expands it along the
// rendering camera's right/up axes, so EACH visitor display sees the ring
// face-on regardless of where its stage camera orbits. Vertex colour × HDR
// _Tint (shared with the sphere's glow).

Shader "Experience/DwellRing"
{
    Properties
    {
        _Tint ("Tint (HDR)", Color) = (1, 1, 1, 1)
    }
    SubShader
    {
        Tags { "RenderType" = "Transparent" "Queue" = "Transparent" "IgnoreProjector" = "True" }
        LOD 100

        Pass
        {
            Cull Off
            ZWrite Off
            ZTest Always        // the ring is UI-like: never hidden by the sphere/cloud
            Blend SrcAlpha OneMinusSrcAlpha

            CGPROGRAM
            #pragma vertex   vert
            #pragma fragment frag
            #include "UnityCG.cginc"

            float4 _Tint;

            struct appdata
            {
                float4 vertex : POSITION; // xy = arc coords (m), z unused
                float4 color  : COLOR;
            };

            struct v2f
            {
                float4 pos   : SV_POSITION;
                float4 color : COLOR0;
            };

            v2f vert(appdata v)
            {
                v2f o;
                float3 centerWS = mul(unity_ObjectToWorld, float4(0, 0, 0, 1)).xyz;
                // View-matrix rows = camera basis in world space.
                float3 camRight = normalize(float3(UNITY_MATRIX_V._m00, UNITY_MATRIX_V._m01, UNITY_MATRIX_V._m02));
                float3 camUp    = normalize(float3(UNITY_MATRIX_V._m10, UNITY_MATRIX_V._m11, UNITY_MATRIX_V._m12));
                float3 worldPos = centerWS + camRight * v.vertex.x + camUp * v.vertex.y;
                o.pos = mul(UNITY_MATRIX_VP, float4(worldPos, 1.0));
                o.color = v.color;
                return o;
            }

            float4 frag(v2f i) : SV_Target
            {
                return float4(i.color.rgb * _Tint.rgb, i.color.a * _Tint.a);
            }
            ENDCG
        }
    }
}
