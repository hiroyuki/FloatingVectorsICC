// Snapshot display shader (Phase 5): unlit vertex colour, Cull Off — the
// world-space snapshot meshes (TSDFSnapshotBuilder.BuildDisplayMeshes) keep
// export-oriented tube winding, so both faces must draw. In Resources so a
// player build doesn't strip it (project_build_shader_stripping).

Shader "Experience/SnapshotVertexColor"
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
                return float4(i.color.rgb, 1.0);
            }
            ENDCG
        }
    }
}
