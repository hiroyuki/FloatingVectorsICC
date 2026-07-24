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
        // Rim lifts the silhouette in the surface's own colour (no white add),
        // so shading stays saturated instead of washing to grey.
        _RimPower   ("Rim Power",  Float)  = 2.5
        _RimBoost   ("Rim Boost",  Range(0, 2)) = 0.5
        _Saturation ("Saturation", Range(0, 3))   = 1.0
        _Brightness ("Brightness", Range(0, 3))   = 1.0
        _Gamma      ("Gamma",      Range(0.2, 3)) = 1.0
        // Smooth (gradient) normals: sample the TSDF volume's SDF gradient at the
        // fragment instead of the faceted ddx/ddy face normal. 0 = flat facets
        // (original), 1 = smooth. Falls back to flat when the volume is unbound
        // or a neighbour voxel is unobserved, so print export etc. stay untouched.
        _GradNormals ("Gradient normals", Range(0, 1)) = 1.0
        // Self-shadow: darken fragments occluded from the light by another part of
        // the body (arm over torso, etc). 0 = off (original flat-lit look).
        _ShadowStrength ("Self-shadow strength", Range(0, 1)) = 0.7
        _ShadowSteps    ("Self-shadow reach (voxel steps)", Range(1, 64)) = 24
        _ShadowBiasVox  ("Self-shadow start bias (voxels)", Range(0, 12)) = 3
        _ShadowIsoFrac  ("Self-shadow occluder threshold (x tau)", Range(0, 1)) = 0.3
        // White point cloud: draw only the triangle edges (interior discarded) so the
        // marching-cubes surface reads as a net of white points over the black stage. Driven
        // by the experience during the ResultShow / practice replay.
        [Toggle] _WhitePointCloud  ("White point cloud", Float) = 0
        _WhitePointCloudColor     ("White point cloud colour", Color)  = (1, 1, 1, 1)
        _WhitePointCloudThickness ("White point cloud thickness (px)", Range(0.2, 4)) = 1.2
        // Thin the net by drawing only every Nth marching-cubes triangle (the mesh
        // is far denser than a pixel, so a 1:1 net reads as a solid fill).
        _WhitePointCloudStride    ("White point cloud triangle stride (1 = all)", Range(1, 32)) = 8
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
            float  _RimPower;
            float  _RimBoost;
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

            // Lighting + self-shadow (bound by TSDFView). _LightDir is the shared
            // light direction used for BOTH the diffuse term and the shadow march,
            // so the cast direction always matches the shading. _VoxelSize / _Tau
            // give the march its step length and occluder threshold in metres.
            float3   _LightDir;
            float    _VoxelSize;
            float    _Tau;
            float    _ShadowStrength;
            float    _ShadowSteps;
            float    _ShadowBiasVox;
            float    _ShadowIsoFrac;

            // White point cloud. Each MC triangle is 3 consecutive procedural vertices, so
            // a per-corner barycentric ((1,0,0)/(0,1,0)/(0,0,1)) interpolates to the
            // distance-to-edge across the face — no geometry shader, no vertex attrs.
            float    _WhitePointCloud;
            float4   _WhitePointCloudColor;
            float    _WhitePointCloudThickness;
            float    _WhitePointCloudStride;

            struct V2F
            {
                float4 pos      : SV_POSITION;
                float3 worldPos : TEXCOORD0;
                float3 viewDir  : TEXCOORD1;
                float3 vcol     : TEXCOORD2;
                float3 bary     : TEXCOORD3;
                nointerpolation float pointKeep : TEXCOORD4; // 1 = draw this tri's edges
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
                o.bary = (cornerIdx == 0u) ? float3(1, 0, 0)
                       : (cornerIdx == 1u) ? float3(0, 1, 0) : float3(0, 0, 1);
                // Keep only every Nth triangle for the white point cloud (thinning). Flat so
                // all three corners agree on whether the triangle is drawn.
                uint stride = (uint)max(1.0, _WhitePointCloudStride);
                o.pointKeep = ((triIdx % stride) == 0u) ? 1.0 : 0.0;
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

            // Self-shadow by marching the TSDF toward the light. The TSDF is only
            // valid in a thin ±tau shell around each surface (interior and free
            // space are unobserved, weight 0), so this is NOT sphere tracing: it
            // fixed-steps along L one voxel at a time and darkens as soon as the ray
            // enters ANOTHER observed surface shell (sdf <= isoFrac*tau, weight ok).
            // The ray starts biased along L past the fragment's own shell so a
            // surface can't shadow itself; nearer occluders darken more (soft ramp).
            // Returns occlusion in [0,1]; 0 when shadows are off or the volume is
            // unbound (SampleSdf fails everywhere -> stays lit).
            float SelfShadow(float3 worldPos, float3 L)
            {
                if (_ShadowStrength <= 0.0 || _VoxelSize <= 0.0) return 0.0;
                int steps = (int)_ShadowSteps;
                float stepLen = _VoxelSize;                 // one voxel per step
                float iso = _ShadowIsoFrac * _Tau;          // enter-shell threshold (m)
                float3 ro = worldPos + L * (_VoxelSize * _ShadowBiasVox);
                [loop] for (int s = 1; s <= steps; s++)
                {
                    float3 wp = ro + L * (stepLen * (float)s);
                    float3 vc = mul(_VoxelFromWorld, float4(wp, 1.0)).xyz;
                    int3 ib = int3(floor(vc));
                    float sdf;
                    if (SampleSdf(ib, sdf) && sdf <= iso)
                        return 1.0 - (float)(s - 1) / (float)steps; // nearer = darker
                }
                return 0.0;
            }

            fixed4 frag(V2F i) : SV_Target
            {
                // White point cloud: keep only the fragments near a triangle edge; discard
                // the interior so the black stage shows through and the surface
                // reads as a net of white points. Screen-space AA via fwidth keeps the line a
                // constant pixel width regardless of distance.
                if (_WhitePointCloud > 0.5)
                {
                    if (i.pointKeep < 0.5) discard;       // thinned-out triangle
                    float3 d = fwidth(i.bary) * max(0.1, _WhitePointCloudThickness);
                    float3 s = smoothstep(float3(0, 0, 0), d, i.bary);
                    float edge = min(s.x, min(s.y, s.z)); // ~0 on an edge, ~1 interior
                    if (edge > 0.5) discard;
                    return fixed4(_WhitePointCloudColor.rgb, 1.0);
                }

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

                float3 L = normalize(_LightDir);
                // Value-only shading: RGB * scalar preserves HSV saturation, and
                // the rim lifts the silhouette in the surface's own colour (no
                // white add), so smooth normals read as form without washing to
                // grey. Higher ambient floor (0.4) keeps the shaded side vivid.
                float shade = saturate(dot(N, L)) * 0.6 + 0.4;   // 0.4..1.0
                // Self-shadow: parts occluded from the light (arm over torso) go
                // darker. Multiplies the diffuse term so an unshadowed fragment is
                // unchanged and a fully shadowed one drops toward the ambient floor.
                float occ = SelfShadow(i.worldPos, L);
                shade *= (1.0 - occ * _ShadowStrength);
                float rim = pow(1.0 - saturate(dot(N, i.viewDir)), _RimPower);

                // Per-vertex camera colour (baked at integration), colour-graded
                // then shaded by the normal. Grading order: gamma (fix
                // sRGB/linear darkness) -> saturation (luma-preserving boost) ->
                // tint -> brightness, then lighting.
                float3 albedo = saturate(i.vcol);
                albedo = pow(albedo, _Gamma);
                float luma = dot(albedo, float3(0.2126, 0.7152, 0.0722));
                albedo = max(0.0, lerp(float3(luma, luma, luma), albedo, _Saturation));
                albedo *= _Color.rgb;
                float3 rgb = albedo * _Brightness * (shade + rim * _RimBoost);
                return fixed4(rgb, 1.0);
            }
            ENDCG
        }
    }
}
