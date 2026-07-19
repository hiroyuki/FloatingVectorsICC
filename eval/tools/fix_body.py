#!/usr/bin/env python3
"""Repair the HUMAN body span of a print-export OBJ with MeshFix.

The TSDF body has camera blind spots (crown, under-arms) whose holes
neither the voxel closing nor naive fan caps fully seal. MeshFix is a
dedicated scanned-surface repair tool: it rebuilds a single watertight
manifold from the largest component and fills all holes smoothly.

Reads the OBJ, repairs ONLY the Human span, and writes
<name>_fixedbody.obj with the other spans (Curves/Body) untouched.

Usage:  python fix_body.py input.obj [output.obj]
Needs:  pip install pymeshfix numpy   (Python 3.8+)
"""
import sys, time
import numpy as np
import pymeshfix


def main():
    src = sys.argv[1]
    dst = sys.argv[2] if len(sys.argv) > 2 else src.replace(".obj", "_fixedbody.obj")
    t0 = time.time()

    verts = []
    spans = {}          # material -> list of face index triples
    order = []          # material order of appearance
    cur = None
    with open(src, encoding="utf-8") as f:
        for line in f:
            if line.startswith("v "):
                p = line.split()
                verts.append((float(p[1]), float(p[2]), float(p[3])))
            elif line.startswith("usemtl"):
                cur = line.split()[1]
                if cur not in spans:
                    spans[cur] = []
                    order.append(cur)
            elif line.startswith("f "):
                p = line.split()
                idx = [int(x.split("/")[0]) - 1 for x in p[1:4]]
                spans[cur].append(idx)
    print(f"read {len(verts)} verts, spans: "
          + ", ".join(f"{m}:{len(t)}" for m, t in spans.items())
          + f"  ({time.time()-t0:.1f}s)")
    if "Human" not in spans:
        print("no Human span found"); sys.exit(1)

    v = np.asarray(verts, dtype=np.float64)
    hf = np.asarray(spans["Human"], dtype=np.int64)
    used = np.unique(hf)
    remap = np.full(len(v), -1, dtype=np.int64)
    remap[used] = np.arange(len(used))
    hv = v[used]
    hfr = remap[hf]

    fix = pymeshfix.MeshFix(hv, hfr)
    fix.repair()
    rv, rf = fix.points, fix.faces
    print(f"MeshFix: {len(hv)}v/{len(hfr)}f -> {len(rv)}v/{len(rf)}f watertight "
          f"({time.time()-t0:.1f}s)")

    # write output: repaired Human verts appended after the originals so the
    # untouched spans keep their indices
    base = len(v)
    with open(dst, "w", encoding="utf-8") as w:
        w.write(f"# FloatingVectorsICC print export (MeshFix body)\n")
        mtl = src.rsplit("/", 1)[-1].rsplit("\\", 1)[-1].replace(".obj", ".mtl")
        w.write(f"mtllib {mtl}\n")
        for p in verts:
            w.write(f"v {p[0]:.3f} {p[1]:.3f} {p[2]:.3f}\n")
        for p in rv:
            w.write(f"v {p[0]:.3f} {p[1]:.3f} {p[2]:.3f}\n")
        # smooth vertex normals for everything (viewers shade correctly)
        n = np.zeros((base + len(rv), 3))
        def acc(faces, off):
            a = faces[:, 0] + off; b = faces[:, 1] + off; c = faces[:, 2] + off
            allv = np.vstack([verts, rv]) if len(rv) else np.asarray(verts)
            fn = np.cross(allv[b] - allv[a], allv[c] - allv[a])
            np.add.at(n, a, fn); np.add.at(n, b, fn); np.add.at(n, c, fn)
        for m in order:
            if m == "Human": continue
            acc(np.asarray(spans[m], dtype=np.int64), 0)
        if len(rf): acc(np.asarray(rf, dtype=np.int64), base)
        ln = np.linalg.norm(n, axis=1, keepdims=True)
        n = np.where(ln > 1e-20, n / np.maximum(ln, 1e-20), [0, 1, 0])
        for p in n:
            w.write(f"vn {p[0]:.3f} {p[1]:.3f} {p[2]:.3f}\n")
        for m in order:
            if m == "Human":
                w.write("usemtl Human\n")
                for f3 in rf:
                    i, j, k = int(f3[0]) + base + 1, int(f3[1]) + base + 1, int(f3[2]) + base + 1
                    w.write(f"f {i}//{i} {j}//{j} {k}//{k}\n")
            else:
                w.write(f"usemtl {m}\n")
                for f3 in spans[m]:
                    i, j, k = f3[0] + 1, f3[1] + 1, f3[2] + 1
                    w.write(f"f {i}//{i} {j}//{j} {k}//{k}\n")
    print(f"wrote {dst}  ({time.time()-t0:.1f}s total)")


if __name__ == "__main__":
    main()
