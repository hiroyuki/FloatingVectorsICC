// Drop-shadow companion for PointCloudUnlit. Same per-vertex OBB / decimation
// culling, but each surviving vertex is collapsed to a fixed world-space Y
// (_FloorY) and the fragment outputs _ShadowColor instead of the per-vertex
// color. FloorOrigin issues a second draw of every live point-cloud mesh with
// this material so the visible points cast a flat silhouette onto the floor.
//
// MaterialPropertyBlock contract (set by FloorOrigin.UpdateShadowMpb):
//   _ObbObjToBox / _ObbMode / _DecimKeep / _DecimFrame  -- mirror the live
//      PointCloudRenderer's filter state so the shadow matches the visible
//      cloud point-for-point.
//   _FloorY      -- world-space Y of the floor plane.
//   _ShadowColor -- RGBA shadow tint. Alpha is consumed via SrcAlpha blending.

Shader "Orbbec/PointCloudShadow"
{
    Properties
    {
        _ShadowColor("Shadow Color", Color) = (0,0,0,0.5)
        _FloorY     ("Floor Y (world)", Float) = 0
        _ObbMode    ("OBB Mode (0=Disabled 1=KeepInside 2=KeepOutside)", Float) = 0
        _DecimKeep  ("Decimation Keep Ratio [0..1]", Float) = 1
        _DecimFrame ("Decimation Frame Counter",   Float) = 0
        // World-space XZ offset added to each vertex after floor projection. Used
        // by FloorOrigin's multi-tap soft shadow to jitter each tap into a Gauss
        // disc; per-tap alpha is folded into _ShadowColor.a by the caller so N
        // taps integrate to the same total opacity as a single hard tap.
        _BlurOffset ("Blur Offset (XZ world)", Vector) = (0, 0, 0, 0)

        // Light model. 0 = orthographic projection straight down to _FloorY
        // (the original behavior). 1 = directional light: rays along _LightDir.
        // 2 = positional point light at _LightPos. _LightDir.y must be < 0 in
        // mode 1, and _LightPos.y must be above the vertex Y in mode 2; the
        // vertex shader rejects (= offscreens) any vertex that fails these.
        _LightMode  ("Light Mode (0=Ortho 1=Dir 2=Point)", Float) = 0
        _LightDir   ("Light Direction (world, downward)", Vector) = (0, -1, 0, 0)
        _LightPos   ("Light Position (world)", Vector)            = (0, 3, 0, 0)
    }
    SubShader
    {
        Tags { "RenderType" = "Transparent" "Queue" = "Transparent-50" "IgnoreProjector" = "True" }
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

            float4x4 _ObbObjToBox;
            float    _ObbMode;
            float    _DecimKeep;
            float    _DecimFrame;
            float4   _ShadowColor;
            float    _FloorY;
            float4   _BlurOffset;
            float    _LightMode;
            float4   _LightDir;
            float4   _LightPos;

            struct appdata
            {
                float4 vertex : POSITION;
                float3 color  : COLOR;
                uint   vid    : SV_VertexID;
            };

            struct v2f
            {
                float4 vertex : SV_POSITION;
                // Metal requires VS to write point size or it's undefined
                // (renders as huge squares for Points topology). D3D ignores
                // PSIZE when the topology is non-Points.
                float  psize  : PSIZE;
            };

            bool PassObb(float3 objPos)
            {
                if (_ObbMode < 0.5) return true;
                float3 b = mul(_ObbObjToBox, float4(objPos, 1.0)).xyz;
                bool inside = all(abs(b) <= 0.5);
                return (_ObbMode < 1.5) ? inside : !inside;
            }

            bool PassDecim(uint vid)
            {
                if (_DecimKeep >= 1.0) return true;
                if (_DecimKeep <= 0.0) return false;
                uint h = vid * 2654435761u + (uint)_DecimFrame;
                h ^= h << 13;
                h ^= h >> 17;
                h ^= h << 5;
                float u = (h & 0x00FFFFFFu) * (1.0 / 16777216.0);
                return u < _DecimKeep;
            }

            v2f vert(appdata v)
            {
                v2f o;
                float keep = (PassObb(v.vertex.xyz) && PassDecim(v.vid)) ? 1.0 : 0.0;

                float3 V = mul(unity_ObjectToWorld, v.vertex).xyz;
                float3 floored;
                if (_LightMode < 0.5)
                {
                    // Orthographic — drop Y straight to the floor plane.
                    floored = float3(V.x, _FloorY, V.z);
                }
                else if (_LightMode < 1.5)
                {
                    // Directional — march along _LightDir until y = _FloorY.
                    // Guards: dy must be solidly negative (light pointing down)
                    // AND t positive (vertex above floor). Either failure → cull
                    // by zeroing keep so the vertex is offscreened below.
                    float dy = _LightDir.y;
                    float t  = (dy < -0.001) ? (_FloorY - V.y) / dy : -1.0;
                    keep    *= (t > 0.0) ? 1.0 : 0.0;
                    floored  = V + t * _LightDir.xyz;
                }
                else
                {
                    // Positional point light — ray from _LightPos through V to floor.
                    // Guards: vertex must be below the light AND projection in front
                    // of the light. Same offscreen-on-failure pattern.
                    float3 L  = _LightPos.xyz;
                    float  vy = V.y - L.y;
                    float  s  = (vy < -0.001) ? (_FloorY - L.y) / vy : -1.0;
                    keep     *= (s > 0.0) ? 1.0 : 0.0;
                    floored   = L + s * (V - L);
                }
                floored.x += _BlurOffset.x;
                floored.z += _BlurOffset.z;

                o.vertex = (keep > 0.5)
                    ? mul(UNITY_MATRIX_VP, float4(floored, 1.0))
                    : float4(2.0, 2.0, 2.0, 1.0);
                o.psize = 1.0;
                return o;
            }

            fixed4 frag(v2f i) : SV_Target
            {
                return _ShadowColor;
            }
            ENDCG
        }
    }
}
