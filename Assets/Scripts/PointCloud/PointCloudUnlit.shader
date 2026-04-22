// Minimal vertex-color shader for point cloud rendering. Targets the
// Built-in render pipeline (currently active in this project). Vertex
// layout matches OBColorPoint: float3 position + float3 color, 24 bytes.
// Each point renders as one fragment (1-pixel dot); upgrade later if you
// need point sprites or screen-space billboards.

Shader "Orbbec/PointCloudUnlit"
{
    SubShader
    {
        Tags { "RenderType" = "Opaque" "IgnoreProjector" = "True" }
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

            struct appdata
            {
                float4 vertex : POSITION;
                float3 color  : COLOR;
            };

            struct v2f
            {
                float4 vertex : SV_POSITION;
                float3 color  : TEXCOORD0;
            };

            v2f vert(appdata v)
            {
                v2f o;
                o.vertex = UnityObjectToClipPos(v.vertex);
                o.color  = v.color;
                return o;
            }

            fixed4 frag(v2f i) : SV_Target
            {
                return fixed4(i.color, 1.0);
            }
            ENDCG
        }
    }
}
