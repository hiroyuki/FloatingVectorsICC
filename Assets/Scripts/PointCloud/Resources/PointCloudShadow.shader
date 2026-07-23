// Drop-shadow companion for PointCloudUnlit. Same per-vertex culling as the
// displayed cloud — OBB, decimation, capsule union, floor mask, joint motion
// field and the global bottom-up reveal — but each surviving vertex is
// collapsed to a fixed world-space Y (_FloorY) and the fragment outputs
// _ShadowColor instead of the per-vertex color. FloorOrigin issues a second
// draw of every VISIBLE point-cloud / skeleton mesh with this material so the
// floor mirrors exactly what is on screen at that moment.
//
// MaterialPropertyBlock contract: FloorOrigin COPIES the source renderer's own
// property block (filled by PointCloudShaderFilters.Apply on the display path)
// so every filter uniform below matches the visible mesh point-for-point, then
// stamps the shadow-only properties on top:
//   _FloorY      -- world-space Y of the floor plane.
//   _ShadowColor -- RGBA shadow tint. Alpha is consumed via SrcAlpha blending.
//   _BlurOffset  -- per-tap soft-shadow jitter.
// _PcRevealMode/_PcRevealY are globals (set by ExperienceDirector), picked up
// here automatically so the reflection rises with the cloud.

Shader "Orbbec/PointCloudShadow"
{
    Properties
    {
        _ShadowColor("Shadow Color", Color) = (0,0,0,0.5)
        _FloorY     ("Floor Y (world)", Float) = 0
        _ObbMode    ("OBB Mode (0=Disabled 1=KeepInside 2=KeepOutside)", Float) = 0
        _DecimKeep  ("Decimation Keep Ratio [0..1]", Float) = 1
        _DecimFrame ("Decimation Frame Counter",   Float) = 0
        _CapsMode   ("Capsule Mode (0=Disabled 1=KeepInside 2=KeepOutside)", Float) = 0
        _CapsCount  ("Capsule Count", Float) = 0
        _MotionMode         ("Motion Mode (0=Disabled 1=Enabled)", Float) = 0
        _MotionCount        ("Motion Joint Count",                Float) = 0
        _MotionMaxDist      ("Motion Max Assign Distance (m, 0=off)", Float) = 0
        _MotionDisplace     ("Motion Displace Vertices (0=off 1=on)", Float) = 0
        _MotionDisplaceScale("Motion Displace Scale",             Float) = 0
        // Same park-below-everything default as PointCloudUnlit: a source MR
        // whose property block never set the mask (skeleton meshes) must not
        // get its shadow clipped at y=0.
        _FloorMaskY      ("Floor Mask Height (m)",   Float) = -1000000000
        _FloorMaskRadius ("Floor Keep Radius Around Feet (m)", Float) = 0.5
        _FloorFootCount  ("Floor Foot Count",                  Float) = 0
        // FloorOrigin sets this to 1 only for point-cloud sources. The global
        // _PcRevealMode clip culls per-VERTEX, which is safe for point topology
        // but stretches triangles that straddle the reveal plane into screen-
        // crossing slivers — and displayed skeleton meshes are not reveal-
        // clipped anyway, so their reflection must not be either.
        _ShadowUseReveal ("Apply Reveal Clip (point clouds only)", Float) = 0
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
            // Mirror of PointCloudUnlit's display filters (see that shader for
            // the semantics). Array lengths must stay in lockstep with
            // PointCloudCapsuleFilter.MaxCapsules / PointCloudJointMotionField
            // .MaxJoints / PointCloudFloorMask.MaxFeet.
            float    _CapsMode;
            float    _CapsCount;
            float4   _CapsA[64];
            float4   _CapsB[64];
            float    _MotionMode;
            float    _MotionCount;
            float    _MotionMaxDist;
            float    _MotionDisplace;
            float    _MotionDisplaceScale;
            float4   _MotionPos[64];
            float4   _MotionVel[64];
            float    _FloorMaskY;
            float    _FloorMaskRadius;
            float    _FloorFootCount;
            float4   _FloorFoot[8];
            // Globals (never in a Properties block — see PointCloudUnlit note).
            float    _PcRevealMode;
            float    _PcRevealY;
            // Global dissolve fade — mirrors PointCloudUnlit so the shadow thins out
            // with the cloud. 0 = shown (default), 1 = fully dissolved.
            float    _PcFadeCull;
            float    _ShadowUseReveal;
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

            // The three world-space culls below are copied verbatim from
            // PointCloudUnlit so the shadow keeps exactly the vertices the
            // display keeps.
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

            bool PassFloor(float3 worldPos)
            {
                if (worldPos.y >= _FloorMaskY) return true;
                int n = (int)_FloorFootCount;
                float r2 = _FloorMaskRadius * _FloorMaskRadius;
                for (int i = 0; i < n; i++)
                {
                    float2 d = worldPos.xz - _FloorFoot[i].xz;
                    if (dot(d, d) <= r2) return true;
                }
                return false;
            }

            bool PassReveal(float3 worldPos)
            {
                if (_ShadowUseReveal < 0.5 || _PcRevealMode < 0.5) return true;
                return worldPos.y <= _PcRevealY;
            }

            bool PassFade(uint vid)
            {
                if (_PcFadeCull <= 0.0) return true;
                uint h = vid * 2654435761u;
                h ^= h << 13; h ^= h >> 17; h ^= h << 5;
                float u = (h & 0x00FFFFFFu) * (1.0 / 16777216.0);
                return u >= _PcFadeCull;
            }

            // Motion-field mirror: cull past _MotionMaxDist and displace by the
            // nearest joint's velocity, so the shadow tracks the displaced
            // silhouette the viewer actually sees. Color logic is irrelevant
            // here (fragment is a flat tint) and skipped.
            void ApplyMotion(inout float3 worldPos, inout float keep)
            {
                if (_MotionMode < 0.5 || _MotionCount < 0.5) return;
                int idx = -1;
                float distSq = 1e30;
                float3 vel = float3(0.0, 0.0, 0.0);
                int n = (int)_MotionCount;
                for (int i = 0; i < n; i++)
                {
                    float3 d = worldPos - _MotionPos[i].xyz;
                    float dd = dot(d, d);
                    if (dd < distSq) { distSq = dd; idx = i; vel = _MotionVel[i].xyz; }
                }
                if (idx < 0) return;
                if (_MotionMaxDist > 0.0 && distSq > _MotionMaxDist * _MotionMaxDist)
                    keep = 0.0;
                if (_MotionDisplace >= 0.5)
                    worldPos += vel * _MotionDisplaceScale;
            }

            v2f vert(appdata v)
            {
                v2f o;
                float3 V = mul(unity_ObjectToWorld, v.vertex).xyz;
                float keep = (PassObb(v.vertex.xyz) && PassDecim(v.vid)
                              && PassCapsules(V) && PassFloor(V) && PassReveal(V)
                              && PassFade(v.vid))
                             ? 1.0 : 0.0;
                ApplyMotion(V, keep);
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
