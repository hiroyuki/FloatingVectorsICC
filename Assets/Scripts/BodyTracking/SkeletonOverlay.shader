// Always-on-top unlit shader for the body tracking skeleton (joint spheres
// + bone line meshes). Works under Unity Built-in render pipeline (URP is
// not installed in this project, despite the project being structured as
// "Universal 3D" — there is no com.unity.render-pipelines.universal in
// Packages/manifest.json or Library/PackageCache).
// ZTest Always + ZWrite Off + Overlay queue keep the skeleton on top of
// the point cloud regardless of distance.

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
