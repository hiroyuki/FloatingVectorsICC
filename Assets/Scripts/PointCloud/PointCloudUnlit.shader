// Minimal vertex-color shader for point cloud rendering. Targets the
// Built-in render pipeline (currently active in this project). Vertex
// layout matches OBColorPoint: float3 position + float3 color, 24 bytes.
// Each point renders as one fragment (1-pixel dot); upgrade later if you
// need point sprites or screen-space billboards.
//
// Per-vertex culling: OBB filter and random decimation are both applied in
// the VS by writing an out-of-clip-volume SV_POSITION, so the CPU side can
// ship the raw point buffer without pre-filtering (main-thread throughput).
// Defaults make the filters no-ops, so snapshot GameObjects that don't set
// MaterialPropertyBlock keep every point.

Shader "Orbbec/PointCloudUnlit"
{
    Properties
    {
        _ObbMode    ("OBB Mode (0=Disabled 1=KeepInside 2=KeepOutside)", Float) = 0
        _DecimKeep  ("Decimation Keep Ratio [0..1]", Float) = 1
        _DecimFrame ("Decimation Frame Counter",   Float) = 0
        _CapsMode   ("Capsule Mode (0=Disabled 1=KeepInside 2=KeepOutside)", Float) = 0
        _CapsCount  ("Capsule Count", Float) = 0
        _MotionMode         ("Motion Mode (0=Disabled 1=Enabled)", Float) = 0
        _MotionCount        ("Motion Joint Count",                Float) = 0
        _MotionColorMode    ("Motion Color (0=None 1=Magnitude 2=Direction)", Float) = 0
        _MotionMaxDist      ("Motion Max Assign Distance (m, 0=off)", Float) = 0
        _MotionDisplace     ("Motion Displace Vertices (0=off 1=on)", Float) = 0
        _MotionDisplaceScale("Motion Displace Scale",             Float) = 0
        _MotionSpeedMax     ("Motion Speed Max (m/s)",            Float) = 1
        _MotionHotColor     ("Motion Hot Color",                  Color) = (1, 0.2, 0.05, 1)
    }
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

            // Renderer-object-space -> OBB-local-space ([-0.5, +0.5]^3).
            // Set per-frame by PointCloudRenderer via MaterialPropertyBlock.
            float4x4 _ObbObjToBox;
            float    _ObbMode;
            float    _DecimKeep;
            float    _DecimFrame;
            // Capsule-union filter. _CapsMode == 0 -> disabled (pass-through);
            // 1 -> KeepInside (pass if inside any capsule); 2 -> KeepOutside.
            // _CapsA[i].xyz = world endpoint A, _CapsA[i].w = capsule radius.
            // _CapsB[i].xyz = world endpoint B, _CapsB[i].w unused. _CapsCount
            // is the live count (0..PointCloudCapsuleFilter.MaxCapsules). Bump
            // the array length in lockstep with that constant.
            float    _CapsMode;
            float    _CapsCount;
            float4   _CapsA[64];
            float4   _CapsB[64];
            // Joint motion field (issue #24). _MotionMode == 0 -> pass-through
            // (no nearest-joint loop). When enabled, the vertex shader assigns
            // each point its nearest joint, then optionally:
            //   - culls the point if the distance exceeds _MotionMaxDist (>0)
            //   - displaces the world position by nearestVel * _MotionDisplaceScale
            //   - tints the color from speed (Magnitude) or direction (Direction)
            // _MotionPos[i].xyz = world joint position, .w unused.
            // _MotionVel[i].xyz = world joint velocity (m/s), .w = precomputed speed.
            // Array length 64 must match PointCloudJointMotionField.MaxJoints.
            float    _MotionMode;
            float    _MotionCount;
            float    _MotionColorMode;
            float    _MotionMaxDist;
            float    _MotionDisplace;
            float    _MotionDisplaceScale;
            float    _MotionSpeedMax;
            float4   _MotionHotColor;
            float4   _MotionPos[64];
            float4   _MotionVel[64];

            struct appdata
            {
                float4 vertex : POSITION;
                float3 color  : COLOR;
                uint   vid    : SV_VertexID;
            };

            struct v2f
            {
                float4 vertex : SV_POSITION;
                float3 color  : TEXCOORD0;
            };

            bool PassObb(float3 objPos)
            {
                if (_ObbMode < 0.5) return true; // Disabled
                float3 b = mul(_ObbObjToBox, float4(objPos, 1.0)).xyz;
                bool inside = all(abs(b) <= 0.5);
                return (_ObbMode < 1.5) ? inside : !inside;
            }

            bool PassDecim(uint vid)
            {
                if (_DecimKeep >= 1.0) return true;
                if (_DecimKeep <= 0.0) return false;
                // xorshift32 seeded from vertexID mixed with the frame counter,
                // so the drop pattern varies between frames like the CPU version.
                uint h = vid * 2654435761u + (uint)_DecimFrame;
                h ^= h << 13;
                h ^= h >> 17;
                h ^= h << 5;
                float u = (h & 0x00FFFFFFu) * (1.0 / 16777216.0);
                return u < _DecimKeep;
            }

            // True iff worldPos is inside the union of the first _CapsCount
            // capsules. Each capsule is a segment (_CapsA[i].xyz -> _CapsB[i].xyz)
            // with radius _CapsA[i].w. Mode 1 keeps inside, mode 2 keeps outside,
            // mode 0 is pass-through.
            bool PassCapsules(float3 worldPos)
            {
                if (_CapsMode < 0.5) return true;
                int n = (int)_CapsCount;
                bool insideAny = false;
                for (int i = 0; i < n; i++)
                {
                    float3 a = _CapsA[i].xyz;
                    float3 b = _CapsB[i].xyz;
                    float  r = _CapsA[i].w;
                    float3 ab = b - a;
                    float ab2 = dot(ab, ab);
                    float3 closest;
                    if (ab2 < 1e-12)
                    {
                        closest = a;
                    }
                    else
                    {
                        float t = saturate(dot(worldPos - a, ab) / ab2);
                        closest = a + t * ab;
                    }
                    float3 d = worldPos - closest;
                    if (dot(d, d) <= r * r) { insideAny = true; break; }
                }
                return (_CapsMode < 1.5) ? insideAny : !insideAny;
            }

            // Locate the closest joint to `worldPos`. Returns the joint index
            // (-1 if no joints), its squared distance, and its velocity / speed.
            // Squared distance avoids a per-iteration sqrt; the caller compares
            // against maxDist^2 before taking sqrt for display logic.
            void NearestJoint(float3 worldPos, out int idx, out float distSq, out float3 vel, out float speed)
            {
                idx = -1;
                distSq = 1e30;
                vel = float3(0.0, 0.0, 0.0);
                speed = 0.0;
                int n = (int)_MotionCount;
                if (n <= 0) return;
                for (int i = 0; i < n; i++)
                {
                    float3 jp = _MotionPos[i].xyz;
                    float3 d = worldPos - jp;
                    float dd = dot(d, d);
                    if (dd < distSq)
                    {
                        distSq = dd;
                        idx = i;
                        vel = _MotionVel[i].xyz;
                        speed = _MotionVel[i].w;
                    }
                }
            }

            v2f vert(appdata v)
            {
                v2f o;
                float3 wp = mul(unity_ObjectToWorld, float4(v.vertex.xyz, 1.0)).xyz;
                bool keep = PassObb(v.vertex.xyz) && PassDecim(v.vid) && PassCapsules(wp);
                float3 outColor = v.color;
                float3 wpOut = wp;

                if (_MotionMode >= 0.5 && _MotionCount >= 0.5)
                {
                    int njIdx; float njDistSq; float3 njVel; float njSpeed;
                    NearestJoint(wp, njIdx, njDistSq, njVel, njSpeed);
                    if (njIdx >= 0)
                    {
                        // Max-assign distance filter (vertex filter from issue
                        // #24). 0 means disabled. Compares squared values to
                        // skip the sqrt.
                        if (_MotionMaxDist > 0.0)
                        {
                            float maxSq = _MotionMaxDist * _MotionMaxDist;
                            if (njDistSq > maxSq) keep = false;
                        }
                        if (_MotionDisplace >= 0.5)
                        {
                            wpOut = wp + njVel * _MotionDisplaceScale;
                        }
                        if (_MotionColorMode >= 0.5 && _MotionColorMode < 1.5)
                        {
                            // Magnitude heatmap: lerp v.color -> hotColor by t.
                            float t = saturate(njSpeed / _MotionSpeedMax);
                            outColor = lerp(v.color, _MotionHotColor.rgb, t);
                        }
                        else if (_MotionColorMode >= 1.5)
                        {
                            // Direction encoding: RGB = (normalize(vel)+1)/2.
                            // Stationary joints render mid-grey.
                            float3 dir = (njSpeed > 1e-4)
                                ? (njVel / njSpeed) * 0.5 + 0.5
                                : float3(0.5, 0.5, 0.5);
                            outColor = dir;
                        }
                    }
                }

                // x > w => outside clip volume => primitive culled on every GPU.
                // Use the (possibly displaced) world position when motion mode
                // is on so displacement actually moves the vertex on-screen.
                float4 clipPos = (_MotionMode >= 0.5 && _MotionDisplace >= 0.5)
                    ? mul(UNITY_MATRIX_VP, float4(wpOut, 1.0))
                    : UnityObjectToClipPos(v.vertex);
                o.vertex = keep ? clipPos : float4(2.0, 2.0, 2.0, 1.0);
                o.color = outColor;
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
