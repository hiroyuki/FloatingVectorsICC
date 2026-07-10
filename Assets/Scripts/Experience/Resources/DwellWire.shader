// Wire shader for DwellSphere: vertex colour × HDR _Tint. Mesh vertex colours
// are stored UNorm8 (clamped at 1.0), so the selection glow cannot ride the
// vertex colours — the script drives _Tint above 1 instead (visible bloom when
// Bloom is enabled, still a brightness pop without). Lives in Resources so a
// player build doesn't strip it (nothing references it from a scene —
// project_build_shader_stripping).

Shader "Experience/DwellWire"
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
            ZTest LEqual
            Blend SrcAlpha OneMinusSrcAlpha

            CGPROGRAM
            #pragma vertex   vert
            #pragma fragment frag
            #include "UnityCG.cginc"

            float4 _Tint;

            struct appdata
            {
                float4 vertex : POSITION;
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
                o.pos = UnityObjectToClipPos(v.vertex);
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
