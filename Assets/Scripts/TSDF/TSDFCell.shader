// Per-cell visualiser. Renders one small cube at the centre of every MC cell
// that would emit at least one triangle — i.e. cells whose 8 corner voxels
// have BOTH +sdf and -sdf among them AND all weights >= _MinWeight (the same
// gate TSDFMarchingCubes.compute applies). Hides cells that are uniformly
// inside, uniformly outside, or have any unobserved corner.
//
// Use this view to verify that "MC would extract a triangle here" matches the
// surface bands you expect from the underlying SDF before drilling into the
// per-cell triTable / edgeTable output.

Shader "TSDF/Cell"
{
    Properties
    {
        _Scale ("Cube scale (relative to voxelSize)", Range(0.05, 1.0)) = 0.30
        _Color ("Boundary cell color", Color) = (0.20, 1.0, 0.50, 1.0)
    }
    SubShader
    {
        Tags { "RenderType"="Opaque" "Queue"="Geometry+10" "IgnoreProjector"="True" }
        LOD 100

        Pass
        {
            Cull Back
            ZWrite On
            ZTest LEqual

            CGPROGRAM
            #pragma vertex   vert
            #pragma fragment frag
            #pragma multi_compile_instancing
            #include "UnityCG.cginc"

            StructuredBuffer<float2> _Voxels;   // (sdf, weight)
            float4x4 _WorldFromVoxel;
            // x,y,z = voxel grid dimensions; w unused. Matches the TSDF/Voxel
            // shader binding so Material.SetVector reaches both with one call.
            float3 _Dim;
            float _VoxelSize;
            float _Scale;
            float _MinWeight;                   // gate; matches MC kernel
            float _IsoLevel;                    // = 0 for the standard MC iso
            float4 _Color;

            struct appdata
            {
                float4 vertex : POSITION;
                float3 normal : NORMAL;
                uint   inst   : SV_InstanceID;
            };

            struct v2f
            {
                float4 pos      : SV_POSITION;
                float3 col      : TEXCOORD0;
                float3 normalWS : TEXCOORD1;
            };

            uint VoxLin(int3 c)
            {
                uint dx = (uint)_Dim.x;
                uint dy = (uint)_Dim.y;
                return (uint)c.x + dx * ((uint)c.y + dy * (uint)c.z);
            }

            v2f vert(appdata IN)
            {
                v2f o;

                // Cell grid has (dimX-1) x (dimY-1) x (dimZ-1) cells. Decompose
                // the linear instance id into a 3D cell index.
                uint id = IN.inst;
                uint cdx = (uint)max(1.0, _Dim.x - 1.0);
                uint cdy = (uint)max(1.0, _Dim.y - 1.0);
                uint cx = id % cdx;
                uint cy = (id / cdx) % cdy;
                uint cz = id / (cdx * cdy);

                // 8 corner voxels of this cell (must match the MC kernel's
                // corner numbering in TSDFMarchingCubes.compute).
                int3 c0 = int3((int)cx,     (int)cy,     (int)cz    );
                int3 c1 = int3((int)cx + 1, (int)cy,     (int)cz    );
                int3 c2 = int3((int)cx + 1, (int)cy + 1, (int)cz    );
                int3 c3 = int3((int)cx,     (int)cy + 1, (int)cz    );
                int3 c4 = int3((int)cx,     (int)cy,     (int)cz + 1);
                int3 c5 = int3((int)cx + 1, (int)cy,     (int)cz + 1);
                int3 c6 = int3((int)cx + 1, (int)cy + 1, (int)cz + 1);
                int3 c7 = int3((int)cx,     (int)cy + 1, (int)cz + 1);

                float2 d0 = _Voxels[VoxLin(c0)];
                float2 d1 = _Voxels[VoxLin(c1)];
                float2 d2 = _Voxels[VoxLin(c2)];
                float2 d3 = _Voxels[VoxLin(c3)];
                float2 d4 = _Voxels[VoxLin(c4)];
                float2 d5 = _Voxels[VoxLin(c5)];
                float2 d6 = _Voxels[VoxLin(c6)];
                float2 d7 = _Voxels[VoxLin(c7)];

                // Weight gate — matches MC. Any unobserved corner kills the cell.
                bool weightOK = d0.y >= _MinWeight && d1.y >= _MinWeight
                             && d2.y >= _MinWeight && d3.y >= _MinWeight
                             && d4.y >= _MinWeight && d5.y >= _MinWeight
                             && d6.y >= _MinWeight && d7.y >= _MinWeight;

                // Case index from the 8 corner signs (same bit assignment as
                // the MC kernel). 0 = all outside, 255 = all inside → no triangles.
                int ci = 0;
                if (d0.x < _IsoLevel) ci |= 1;
                if (d1.x < _IsoLevel) ci |= 2;
                if (d2.x < _IsoLevel) ci |= 4;
                if (d3.x < _IsoLevel) ci |= 8;
                if (d4.x < _IsoLevel) ci |= 16;
                if (d5.x < _IsoLevel) ci |= 32;
                if (d6.x < _IsoLevel) ci |= 64;
                if (d7.x < _IsoLevel) ci |= 128;

                bool boundary = (ci != 0) && (ci != 255) && weightOK;

                // Cell centre in world = WorldFromVoxel * (cx+1, cy+1, cz+1)
                // because each voxel centre is at index+0.5, so the midpoint
                // of opposite voxel corners is at integer + 1.0.
                float3 cellIdxC = float3((float)cx + 1.0, (float)cy + 1.0, (float)cz + 1.0);
                float3 wp = mul(_WorldFromVoxel, float4(cellIdxC, 1.0)).xyz;

                // Cube primitive is centred at origin in object space; scale by
                // voxelSize * _Scale then translate to cell centre.
                float3 worldVtx = wp + IN.vertex.xyz * _VoxelSize * _Scale;

                if (!boundary)
                    o.pos = float4(2.0, 2.0, 2.0, 1.0); // clip-cull
                else
                    o.pos = mul(UNITY_MATRIX_VP, float4(worldVtx, 1.0));

                o.col = _Color.rgb;
                o.normalWS = IN.normal;
                return o;
            }

            fixed4 frag(v2f IN) : SV_Target
            {
                float3 L = normalize(float3(0.4, 0.85, 0.35));
                float diffuse = saturate(dot(normalize(IN.normalWS), L)) * 0.55 + 0.55;
                return fixed4(IN.col * diffuse, 1.0);
            }
            ENDCG
        }
    }
}
