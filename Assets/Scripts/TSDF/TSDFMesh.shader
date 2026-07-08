// Procedural shader for the TSDF marching-cubes mesh. Reads vertex world
// positions out of a StructuredBuffer (set per-frame by TSDFMeshRenderer)
// and shades flat using ddx/ddy-derived face normals — enough to see the
// surface during the step 1 smoke phase. Cull Off so the user can inspect
// both sides while the iso level / sign convention is still being verified.

Shader "TSDF/TSDFMesh"
{
    Properties
    {
        _Color      ("Base Color (tint)", Color)  = (1, 1, 1, 1)
        _RimColor   ("Rim Color",  Color)  = (1, 1, 1, 1)
        _RimPower   ("Rim Power",  Float)  = 2.5
        _Saturation ("Saturation", Range(0, 3))   = 1.0
        _Brightness ("Brightness", Range(0, 3))   = 1.0
        _Gamma      ("Gamma",      Range(0.2, 3)) = 1.0
        // Smooth (gradient) normals: sample the TSDF volume's SDF gradient at the
        // fragment instead of the faceted ddx/ddy face normal. 0 = flat facets
        // (original), 1 = smooth. Falls back to flat when the volume is unbound
        // or a neighbour voxel is unobserved, so print export etc. stay untouched.
        _GradNormals ("Gradient normals", Range(0, 1)) = 1.0
    }
    SubShader
    {
        Tags { "RenderType"="Opaque" "Queue"="Geometry" "IgnoreProjector"="True" }
        LOD 100

        Pass
        {
            Cull Off
            ZWrite On
            ZTest LEqual

            CGPROGRAM
            #pragma vertex   vert
            #pragma fragment frag
            #pragma target 4.5
            #include "UnityCG.cginc"

            // Matches the Tri struct emitted by TSDFMarchingCubes.compute.
            // Per vertex: position + colour. 6 float3 per triangle (72 bytes);
            // SV_VertexID indexes vertices across all triangles, decompose into
            // (triIdx, cornerIdx).
            struct Tri { float3 p0; float3 c0; float3 p1; float3 c1; float3 p2; float3 c2; };
            StructuredBuffer<Tri> _Triangles;

            float4 _Color;
            float4 _RimColor;
            float  _RimPower;
            float  _Saturation;
            float  _Brightness;
            float  _Gamma;

            // Gradient-normal inputs (bound by TSDFView when _GradNormals > 0).
            // _Voxels is the same displayed front SDF buffer the mesh was extracted
            // from; _VoxelFromWorld inverts TSDFMarchingCubes' _WorldFromVoxel so a
            // world point maps to voxel-centre coords (index + 0.5).
            StructuredBuffer<float2> _Voxels;   // x = sdf (m), y = weight
            float4x4 _WorldFromVoxel;           // linear part maps a voxel-space gradient to world
            float4x4 _VoxelFromWorld;
            float4   _VDim;                      // (x,y,z) = volume dims, w unused
            float    _MinWeight;
            float    _GradNormals;

            struct V2F
            {
                float4 pos      : SV_POSITION;
                float3 worldPos : TEXCOORD0;
                float3 viewDir  : TEXCOORD1;
                float3 vcol     : TEXCOORD2;
            };

            V2F vert(uint vid : SV_VertexID)
            {
                V2F o;
                uint triIdx = vid / 3u;
                uint cornerIdx = vid - triIdx * 3u; // = vid % 3
                Tri t = _Triangles[triIdx];
                float3 wp  = (cornerIdx == 0u) ? t.p0 : (cornerIdx == 1u) ? t.p1 : t.p2;
                float3 col = (cornerIdx == 0u) ? t.c0 : (cornerIdx == 1u) ? t.c1 : t.c2;
                o.pos = mul(UNITY_MATRIX_VP, float4(wp, 1.0));
                o.worldPos = wp;
                o.viewDir = normalize(_WorldSpaceCameraPos.xyz - wp);
                o.vcol = col;
                return o;
            }

            // Sample the SDF at an integer voxel corner; returns false if the
            // corner is out of range or unobserved (weight below the gate).
            bool SampleSdf(int3 c, out float sdf)
            {
                sdf = 0;
                if (any(c < 0) || c.x >= (int)_VDim.x || c.y >= (int)_VDim.y || c.z >= (int)_VDim.z)
                    return false;
                int idx = c.x + (int)_VDim.x * (c.y + (int)_VDim.y * c.z);
                float2 v = _Voxels[idx];
                if (v.y < _MinWeight) return false;
                sdf = v.x;
                return true;
            }

            // Central-difference SDF gradient at the fragment, mapped to world.
            // Returns false when any neighbour is missing so the caller can fall
            // back to the flat face normal.
            bool GradientNormal(float3 worldPos, out float3 N)
            {
                N = float3(0, 0, 1);
                float3 vc = mul(_VoxelFromWorld, float4(worldPos, 1.0)).xyz; // index + 0.5
                int3 ib = int3(floor(vc));
                float xp, xm, yp, ym, zp, zm;
                if (!SampleSdf(ib + int3(1,0,0), xp) || !SampleSdf(ib - int3(1,0,0), xm)) return false;
                if (!SampleSdf(ib + int3(0,1,0), yp) || !SampleSdf(ib - int3(0,1,0), ym)) return false;
                if (!SampleSdf(ib + int3(0,0,1), zp) || !SampleSdf(ib - int3(0,0,1), zm)) return false;
                float3 g = float3(xp - xm, yp - ym, zp - zm);       // voxel-space gradient
                if (dot(g, g) < 1e-12) return false;
                // Map to world (rotation + uniform voxel scale) and normalise.
                N = normalize(mul((float3x3)_WorldFromVoxel, g));
                return true;
            }

            fixed4 frag(V2F i) : SV_Target
            {
                // Flat normal from screen-space derivatives — no per-vertex
                // normal buffer needed, and stable across MC reconnections.
                float3 dx = ddx(i.worldPos);
                float3 dy = ddy(i.worldPos);
                float3 N = normalize(cross(dx, dy));

                // Smooth normal from the SDF gradient (blended by _GradNormals).
                if (_GradNormals > 0.0)
                {
                    float3 Ng;
                    if (GradientNormal(i.worldPos, Ng))
                        N = normalize(lerp(N, Ng, saturate(_GradNormals)));
                }

                // Always face the camera (because we Cull Off both sides).
                if (dot(N, i.viewDir) < 0) N = -N;

                float3 L = normalize(float3(0.35, 0.85, 0.40));
                float diffuse = saturate(dot(N, L)) * 0.7 + 0.3;
                float rim = pow(1.0 - saturate(dot(N, i.viewDir)), _RimPower);

                // Per-vertex camera colour (baked at integration), colour-graded
                // then shaded by the flat normal. Grading order: gamma (fix
                // sRGB/linear darkness) -> saturation (luma-preserving boost) ->
                // tint -> brightness, then lighting.
                float3 albedo = saturate(i.vcol);
                albedo = pow(albedo, _Gamma);
                float luma = dot(albedo, float3(0.2126, 0.7152, 0.0722));
                albedo = max(0.0, lerp(float3(luma, luma, luma), albedo, _Saturation));
                albedo *= _Color.rgb;
                float3 rgb = albedo * diffuse * _Brightness + _RimColor.rgb * rim * 0.35;
                return fixed4(rgb, 1.0);
            }
            ENDCG
        }
    }
}
