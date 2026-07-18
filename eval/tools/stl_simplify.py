#!/usr/bin/env python3
"""Re-simplify an already-unioned watertight STL with Manifold.simplify.

Usage:  python stl_simplify.py input.stl tolerance_mm [output.stl]
"""
import sys, time
import numpy as np
import manifold3d as m3d
from stl_union import read_binary_stl, weld, write_binary_stl


def main():
    src = sys.argv[1]
    tol = float(sys.argv[2])
    dst = sys.argv[3] if len(sys.argv) > 3 else src.replace(".stl", f"_s{tol:g}.stl")
    t0 = time.time()
    tris = read_binary_stl(src)
    print(f"read {len(tris)} tris  ({time.time()-t0:.1f}s)")
    verts, tv = weld(tris)
    solid = m3d.Manifold(m3d.Mesh(vert_properties=verts, tri_verts=tv))
    print(f"constructed, status={solid.status()}  ({time.time()-t0:.1f}s)")
    solid = solid.simplify(tol)
    print(f"simplified @ {tol} mm -> {solid.num_tri()} tris  ({time.time()-t0:.1f}s)")
    out = solid.to_mesh()
    overts = np.asarray(out.vert_properties, dtype=np.float32)[:, :3]
    otris = np.asarray(out.tri_verts, dtype=np.uint32).reshape(-1, 3)
    write_binary_stl(dst, overts, otris)
    print(f"wrote {dst} ({len(otris)} tris, {100.0*len(otris)/max(1,len(tris)):.0f}% of input)")


if __name__ == "__main__":
    main()
