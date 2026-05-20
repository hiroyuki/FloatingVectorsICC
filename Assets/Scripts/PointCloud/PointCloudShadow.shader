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

            struct appdata
            {
                float4 vertex : POSITION;
                float3 color  : COLOR;
                uint   vid    : SV_VertexID;
            };

            struct v2f
            {
                float4 vertex : SV_POSITION;
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
                bool keep = PassObb(v.vertex.xyz) && PassDecim(v.vid);
                float4 worldPos = mul(unity_ObjectToWorld, v.vertex);
                worldPos.y = _FloorY;
                o.vertex = keep
                    ? mul(UNITY_MATRIX_VP, worldPos)
                    : float4(2.0, 2.0, 2.0, 1.0);
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
