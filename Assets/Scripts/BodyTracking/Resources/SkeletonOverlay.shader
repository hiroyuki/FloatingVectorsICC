// URP-compatible always-on-top shader for the body tracking skeleton (joint
// spheres + bone line meshes). Required pieces for URP forward rendering:
//   - HLSLPROGRAM (not CGPROGRAM) and Core.hlsl include
//   - Pass tag "LightMode" = "UniversalForward"
//   - TransformObjectToHClip (not UnityObjectToClipPos)
// ZTest Always + ZWrite Off + Overlay queue keep the skeleton on top of the
// point cloud regardless of distance.

Shader "BodyTracking/SkeletonOverlay"
{
    Properties
    {
        _Color ("Color", Color) = (1, 1, 1, 1)
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
            Name "SkeletonOverlay"
            Tags { "LightMode" = "UniversalForward" }

            ZTest Always
            ZWrite Off
            Cull Off

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
            };

            struct Varyings
            {
                float4 positionHCS : SV_POSITION;
            };

            Varyings vert(Attributes IN)
            {
                Varyings OUT;
                OUT.positionHCS = TransformObjectToHClip(IN.positionOS);
                return OUT;
            }

            half4 frag(Varyings IN) : SV_Target
            {
                return _Color;
            }
            ENDHLSL
        }
    }

    Fallback Off
}
