#!/usr/bin/env python3
"""Boolean-union a tube-soup STL with the Manifold library.

Blender's exact boolean freezes on ~29k mutually intersecting shells; Manifold
(the OpenSCAD backend) is built for exactly this. Reads a binary STL of many
closed shells, welds vertices, unions everything, writes <name>_union.stl.

Usage:  python stl_union.py input.stl [output.stl]
Needs:  pip install manifold3d numpy   (Python 3.8+)
"""
import sys, time
import numpy as np
import manifold3d as m3d


def read_binary_stl(path):
    with open(path, "rb") as f:
        f.seek(80)
        n = np.frombuffer(f.read(4), dtype=np.uint32)[0]
        rec = np.dtype([("n", "<3f4"), ("v", "<9f4"), ("attr", "<u2")])
        data = np.frombuffer(f.read(int(n) * 50), dtype=rec, count=int(n))
    tris = data["v"].reshape(-1, 3, 3).astype(np.float64)
    return tris


def weld(tris, decimals=4):
    """Weld duplicated corner vertices by rounded position (0.1 um at mm scale)."""
    flat = tris.reshape(-1, 3)
    key = np.round(flat, decimals)
    uniq, inv = np.unique(key, axis=0, return_inverse=True)
    return uniq.astype(np.float32), inv.reshape(-1, 3).astype(np.uint32)


def write_binary_stl(path, verts, tris):
    v = verts[tris]                                   # (n,3,3)
    n1 = np.cross(v[:, 1] - v[:, 0], v[:, 2] - v[:, 0])
    ln = np.linalg.norm(n1, axis=1, keepdims=True)
    n1 = np.where(ln > 1e-20, n1 / np.maximum(ln, 1e-20), 0.0)
    rec = np.zeros(len(tris), dtype=np.dtype(
        [("n", "<3f4"), ("v", "<9f4"), ("attr", "<u2")]))
    rec["n"] = n1.astype(np.float32)
    rec["v"] = v.reshape(-1, 9).astype(np.float32)
    with open(path, "wb") as f:
        f.write(b"manifold union".ljust(80, b"\0"))
        f.write(np.uint32(len(tris)).tobytes())
        f.write(rec.tobytes())


def main():
    src = sys.argv[1]
    dst = sys.argv[2] if len(sys.argv) > 2 else src.replace(".stl", "_union.stl")
    t0 = time.time()
    tris = read_binary_stl(src)
    print(f"read {len(tris)} tris  ({time.time()-t0:.1f}s)")

    verts, tv = weld(tris)
    print(f"welded -> {len(verts)} verts  ({time.time()-t0:.1f}s)")

    # our own connected-component split (Manifold.decompose took 20 min on 29k
    # shells; a plain union-find over welded verts does it in seconds)
    parent = np.arange(len(verts), dtype=np.int64)

    def find(x):
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    for a, b, c in tv:
        ra, rb, rc = find(a), find(b), find(c)
        if rb != ra: parent[rb] = ra
        if rc != ra: parent[rc] = ra
    roots = np.fromiter((find(i) for i in tv[:, 0]), dtype=np.int64, count=len(tv))
    comp_ids, tri_comp = np.unique(roots, return_inverse=True)
    print(f"{len(comp_ids)} shells (own union-find)  ({time.time()-t0:.1f}s)")

    parts = []
    order = np.argsort(tri_comp, kind="stable")
    bounds = np.searchsorted(tri_comp[order], np.arange(len(comp_ids)))
    bounds = np.append(bounds, len(order))
    for c in range(len(comp_ids)):
        tsel = tv[order[bounds[c]:bounds[c + 1]]]
        vids, remap = np.unique(tsel, return_inverse=True)
        part = m3d.Manifold(m3d.Mesh(
            vert_properties=verts[vids],
            tri_verts=remap.reshape(-1, 3).astype(np.uint32)))
        if part.status() == m3d.Error.NoError:
            parts.append(part)
    print(f"built {len(parts)} manifold shells  ({time.time()-t0:.1f}s)")

    union = m3d.Manifold.batch_boolean(parts, m3d.OpType.Add)
    print(f"union done, status={union.status()}, {union.num_tri()} tris  ({time.time()-t0:.1f}s)")

    # the union ADDS intersection-curve triangles; the real reduction comes from
    # merging the coplanar bar faces back together (0.05 mm tolerance)
    union = union.simplify(0.05)
    print(f"simplified -> {union.num_tri()} tris  ({time.time()-t0:.1f}s)")

    out = union.to_mesh()
    overts = np.asarray(out.vert_properties, dtype=np.float32)[:, :3]
    otris = np.asarray(out.tri_verts, dtype=np.uint32).reshape(-1, 3)
    print(f"result: {len(otris)} tris / {len(overts)} verts "
          f"({100.0 * len(otris) / max(1, len(tris)):.0f}% of input)")

    write_binary_stl(dst, overts, otris)
    print(f"wrote {dst}  ({time.time()-t0:.1f}s total)")


if __name__ == "__main__":
    main()
