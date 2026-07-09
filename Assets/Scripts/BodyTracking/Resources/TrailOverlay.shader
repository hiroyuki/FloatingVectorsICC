// Vertex-color + alpha-blended overlay shader for JointTrailMesh. The
// SkeletonOverlay shader used for joint spheres / bones discards mesh.colors,
// so the per-vertex acceleration heatmap painted by JointTrailMesh.Rebuild
// shows up as plain white. This shader pipes COLOR through to the fragment
// and adds standard alpha blending so trail tails fade out cleanly.

Shader "BodyTracking/TrailOverlay"
{
    Properties
    {
        _Color ("Tint", Color) = (1, 1, 1, 1)
    }

    SubShader
    {
        Tags
        {
            "RenderPipeline" = "UniversalPipeline"
            "Queue" = "Overlay"
            "RenderType" = "Overlay"
            "IgnoreProjector" = "True"
        }

        Pass
        {
            Name "TrailOverlay"
            Tags { "LightMode" = "UniversalForward" }

            ZTest Always
            ZWrite Off
            Cull Off
            Blend SrcAlpha OneMinusSrcAlpha

            HLSLPROGRAM
            #pragma vertex vert
            #pragma fragment frag

            #include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Core.hlsl"

            CBUFFER_START(UnityPerMaterial)
                half4 _Color;
            CBUFFER_END

            struct Attributes
            {
                float3 positionOS : POSITION;
                half4  color      : COLOR;
            };

            struct Varyings
            {
                float4 positionHCS : SV_POSITION;
                half4  color       : TEXCOORD0;
            };

            Varyings vert(Attributes IN)
            {
                Varyings OUT;
                OUT.positionHCS = TransformObjectToHClip(IN.positionOS);
                OUT.color = IN.color;
                return OUT;
            }

            half4 frag(Varyings IN) : SV_Target
            {
                return IN.color * _Color;
            }
            ENDHLSL
        }
    }

    Fallback Off
}
