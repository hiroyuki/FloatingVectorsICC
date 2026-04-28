// Always-on-top unlit shader for the body tracking skeleton (joint spheres
// and bone line meshes). The default URP/Unlit shader hardcodes ZTest LEqual
// and doesn't expose _ZTest as a serialized property, so material.SetInt
// can't override it. Hardcoding ZTest Always + ZWrite Off here ensures the
// skeleton always draws on top of the point cloud regardless of depth.

Shader "BodyTracking/SkeletonOverlay"
{
    Properties
    {
        _Color ("Color", Color) = (1, 1, 1, 1)
    }

    SubShader
    {
        Tags { "Queue" = "Overlay" "RenderType" = "Overlay" "IgnoreProjector" = "True" }

        ZTest Always
        ZWrite Off
        Cull Off
        Lighting Off

        Pass
        {
            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #include "UnityCG.cginc"

            struct appdata
            {
                float4 vertex : POSITION;
            };

            struct v2f
            {
                float4 pos : SV_POSITION;
            };

            fixed4 _Color;

            v2f vert(appdata v)
            {
                v2f o;
                o.pos = UnityObjectToClipPos(v.vertex);
                return o;
            }

            fixed4 frag(v2f i) : SV_Target
            {
                return _Color;
            }
            ENDCG
        }
    }

    Fallback Off
}
