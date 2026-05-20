// URP Lit replacement for the body-tracking trail material. Pipes mesh.colors
// through as a multiplicative tint on the base color so JointTrailMesh's
// per-vertex acceleration heatmap still reaches the framebuffer, but evaluates
// real PBR lighting (UniversalFragmentPBR) so the geometry catches main-light
// + additional lights and reads as a 3D shape instead of a flat overlay.
//
// Passes provided so SSAO works out of the box on the project's PC_Renderer
// (which has ScreenSpaceAmbientOcclusion enabled with Source=DepthNormals):
//   - UniversalForward : main PBR evaluation
//   - ShadowCaster     : present so a caller can opt the trail in by setting
//                        MeshRenderer.shadowCastingMode != Off (the
//                        JointTrailMesh renderer leaves it Off because per-
//                        frame rebuilt thin tubes produce flickery shadows)
//   - DepthOnly        : depth prepass
//   - DepthNormals     : feeds SSAO normal reconstruction
//
// Opaque (no alpha blend) so SSAO applies. The visual tail-fade is carried by
// JointTrailMesh's radius taper (_width * headness) collapsing the tube to
// zero at the oldest sample.

Shader "BodyTracking/TrailLit"
{
    Properties
    {
        [MainColor] _BaseColor      ("Base Color (multiplied by vertex color)", Color) = (1, 1, 1, 1)
        _Smoothness                 ("Smoothness", Range(0, 1)) = 0.55
        _Metallic                   ("Metallic", Range(0, 1)) = 0.0
        _EmissionStrength           ("Emission Strength (boost vertex color)", Range(0, 4)) = 0.0
    }

    SubShader
    {
        Tags
        {
            "RenderPipeline" = "UniversalPipeline"
            "RenderType"     = "Opaque"
            "Queue"          = "Geometry"
            "IgnoreProjector"= "True"
        }
        LOD 200

        // ---------- Forward Lit (PBR) ----------
        Pass
        {
            Name "ForwardLit"
            Tags { "LightMode" = "UniversalForward" }

            Cull Off
            ZTest LEqual
            ZWrite On

            HLSLPROGRAM
            #pragma vertex   vert
            #pragma fragment frag
            #pragma target 3.0

            // URP lighting keywords. Keep this list aligned with what URP/Lit
            // declares so SSAO, shadows, fog, lightmaps and Forward+ clustering
            // all hook in without surprises.
            #pragma multi_compile _ _MAIN_LIGHT_SHADOWS _MAIN_LIGHT_SHADOWS_CASCADE _MAIN_LIGHT_SHADOWS_SCREEN
            #pragma multi_compile _ _ADDITIONAL_LIGHTS_VERTEX _ADDITIONAL_LIGHTS
            #pragma multi_compile_fragment _ _ADDITIONAL_LIGHT_SHADOWS
            #pragma multi_compile_fragment _ _REFLECTION_PROBE_BLENDING
            #pragma multi_compile_fragment _ _REFLECTION_PROBE_BOX_PROJECTION
            #pragma multi_compile_fragment _ _SHADOWS_SOFT
            #pragma multi_compile_fragment _ _SCREEN_SPACE_OCCLUSION
            #pragma multi_compile_fragment _ DEBUG_DISPLAY
            #pragma multi_compile _ LIGHTMAP_SHADOW_MIXING
            #pragma multi_compile _ SHADOWS_SHADOWMASK
            #pragma multi_compile _ DIRLIGHTMAP_COMBINED
            #pragma multi_compile _ LIGHTMAP_ON
            #pragma multi_compile _ DYNAMICLIGHTMAP_ON
            #pragma multi_compile _ USE_LEGACY_LIGHTMAPS
            #pragma multi_compile_fog
            #pragma multi_compile_instancing
            #pragma multi_compile _ _FORWARD_PLUS

            #include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Core.hlsl"
            #include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Lighting.hlsl"

            CBUFFER_START(UnityPerMaterial)
                half4 _BaseColor;
                half  _Smoothness;
                half  _Metallic;
                half  _EmissionStrength;
            CBUFFER_END

            struct Attributes
            {
                float3 positionOS : POSITION;
                float3 normalOS   : NORMAL;
                half4  color      : COLOR;
                UNITY_VERTEX_INPUT_INSTANCE_ID
            };

            struct Varyings
            {
                float4 positionHCS : SV_POSITION;
                float3 positionWS  : TEXCOORD0;
                float3 normalWS    : TEXCOORD1;
                half4  color       : TEXCOORD2;
                float4 screenPos   : TEXCOORD3;
                half   fogFactor   : TEXCOORD4;
                UNITY_VERTEX_INPUT_INSTANCE_ID
                UNITY_VERTEX_OUTPUT_STEREO
            };

            Varyings vert(Attributes IN)
            {
                Varyings OUT = (Varyings)0;
                UNITY_SETUP_INSTANCE_ID(IN);
                UNITY_TRANSFER_INSTANCE_ID(IN, OUT);
                UNITY_INITIALIZE_VERTEX_OUTPUT_STEREO(OUT);

                VertexPositionInputs vpi = GetVertexPositionInputs(IN.positionOS);
                VertexNormalInputs   vni = GetVertexNormalInputs(IN.normalOS);

                OUT.positionHCS = vpi.positionCS;
                OUT.positionWS  = vpi.positionWS;
                OUT.normalWS    = vni.normalWS;
                OUT.color       = IN.color;
                OUT.screenPos   = ComputeScreenPos(vpi.positionCS);
                OUT.fogFactor   = ComputeFogFactor(vpi.positionCS.z);
                return OUT;
            }

            half4 frag(Varyings IN, half facing : VFACE) : SV_Target
            {
                UNITY_SETUP_INSTANCE_ID(IN);
                UNITY_SETUP_STEREO_EYE_INDEX_POST_VERTEX(IN);

                half3 albedo = _BaseColor.rgb * IN.color.rgb;

                // Always-double-sided: flip the normal for back faces so the
                // rear of a camera-aligned billboard (or the inside of a tube
                // if the camera enters it) isn't lit by the inverted normal.
                float3 N = normalize(IN.normalWS);
                if (facing < 0) N = -N;

                InputData inputData = (InputData)0;
                inputData.positionWS               = IN.positionWS;
                inputData.normalWS                 = N;
                inputData.viewDirectionWS          = GetWorldSpaceNormalizeViewDir(IN.positionWS);
                inputData.shadowCoord              = TransformWorldToShadowCoord(IN.positionWS);
                inputData.fogCoord                 = IN.fogFactor;
                inputData.vertexLighting           = 0.0;
                inputData.bakedGI                  = SampleSH(N);
                inputData.normalizedScreenSpaceUV  = IN.screenPos.xy / max(IN.screenPos.w, 1e-5);
                inputData.shadowMask               = half4(1, 1, 1, 1);

                SurfaceData surfaceData = (SurfaceData)0;
                surfaceData.albedo               = albedo;
                surfaceData.metallic             = _Metallic;
                surfaceData.specular             = 0;
                surfaceData.smoothness           = _Smoothness;
                surfaceData.normalTS             = half3(0, 0, 1);
                surfaceData.occlusion            = 1.0;
                surfaceData.emission             = albedo * _EmissionStrength;
                surfaceData.alpha                = 1.0;
                surfaceData.clearCoatMask        = 0;
                surfaceData.clearCoatSmoothness  = 0;

                half4 color = UniversalFragmentPBR(inputData, surfaceData);
                color.rgb   = MixFog(color.rgb, IN.fogFactor);
                color.a     = 1.0;
                return color;
            }
            ENDHLSL
        }

        // ---------- ShadowCaster ----------
        Pass
        {
            Name "ShadowCaster"
            Tags { "LightMode" = "ShadowCaster" }

            ZWrite On
            ZTest LEqual
            ColorMask 0
            Cull Off

            HLSLPROGRAM
            #pragma vertex   ShadowPassVertex
            #pragma fragment ShadowPassFragment
            #pragma target 3.0
            #pragma multi_compile_instancing
            #pragma multi_compile_vertex _ _CASTING_PUNCTUAL_LIGHT_SHADOW

            #include "Packages/com.unity.render-pipelines.universal/Shaders/LitInput.hlsl"
            #include "Packages/com.unity.render-pipelines.universal/Shaders/ShadowCasterPass.hlsl"
            ENDHLSL
        }

        // ---------- DepthOnly ----------
        Pass
        {
            Name "DepthOnly"
            Tags { "LightMode" = "DepthOnly" }

            ZWrite On
            ColorMask R
            Cull Off

            HLSLPROGRAM
            #pragma vertex   DepthOnlyVertex
            #pragma fragment DepthOnlyFragment
            #pragma target 3.0
            #pragma multi_compile_instancing

            #include "Packages/com.unity.render-pipelines.universal/Shaders/LitInput.hlsl"
            #include "Packages/com.unity.render-pipelines.universal/Shaders/DepthOnlyPass.hlsl"
            ENDHLSL
        }

        // ---------- DepthNormals (feeds SSAO Source=DepthNormals) ----------
        Pass
        {
            Name "DepthNormals"
            Tags { "LightMode" = "DepthNormals" }

            ZWrite On
            Cull Off

            HLSLPROGRAM
            #pragma vertex   DepthNormalsVertex
            #pragma fragment DepthNormalsFragment
            #pragma target 3.0
            #pragma multi_compile_instancing

            #include "Packages/com.unity.render-pipelines.universal/Shaders/LitInput.hlsl"
            #include "Packages/com.unity.render-pipelines.universal/Shaders/DepthNormalsPass.hlsl"
            ENDHLSL
        }
    }

    Fallback "Hidden/Universal Render Pipeline/FallbackError"
}
