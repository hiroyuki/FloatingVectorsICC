#!/usr/bin/env python3
"""Repair the HUMAN body span of a print-export OBJ with MeshFix.

The TSDF body has camera blind spots (crown, under-arms) whose holes
neither the voxel closing nor naive fan caps fully seal. MeshFix is a
dedicated scanned-surface repair tool: it rebuilds a single watertight
manifold from the largest component and fills all holes smoothly.

Reads the OBJ, repairs ONLY the Human span, and writes
<name>_fixedbody.obj with the other spans (Curves/Body) untouched.

Usage:  python fix_body.py input.obj [output.obj|output.3mf]
        (a .3mf output writes a Bambu-compatible Standard 3MF with
         basematerials Body=black / Curves=white / Human=blue — the
         compact ~1/6-size delivery format)
Needs:  pip install pymeshfix numpy   (Python 3.8+)
"""
import sys, time, zipfile
import numpy as np
import pymeshfix

MAT_COLORS = {"Body": "000000FF", "Curves": "FFFFFFFF", "Human": "669EFFFF"}


def write_3mf(path, verts, span_faces):
    """verts: (n,3) float mm; span_faces: list of (materialName, (m,3) int)."""
    mats = list(MAT_COLORS.keys())
    parts = []
    parts.append('<?xml version="1.0" encoding="UTF-8"?>\n'
                 '<model unit="millimeter" xml:lang="en-US" '
                 'xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02">\n'
                 ' <resources>\n  <basematerials id="1">\n')
    for m in mats:
        parts.append(f'   <base name="{m}" displaycolor="#{MAT_COLORS[m]}"/>\n')
    parts.append('  </basematerials>\n'
                 '  <object id="2" type="model" pid="1" pindex="0">\n   <mesh>\n    <vertices>\n')
    coords = np.round(verts, 3)
    for x, y, z in coords:
        parts.append(f'     <vertex x="{x:g}" y="{y:g}" z="{z:g}"/>\n')
    parts.append('    </vertices>\n    <triangles>\n')
    for name, faces in span_faces:
        p1 = mats.index(name)
        for a, b, c in faces:
            parts.append(f'     <triangle v1="{a}" v2="{b}" v3="{c}" p1="{p1}"/>\n')
    parts.append('    </triangles>\n   </mesh>\n  </object>\n </resources>\n'
                 ' <build>\n  <item objectid="2"/>\n </build>\n</model>\n')
    model = "".join(parts)
    types = ('<?xml version="1.0" encoding="UTF-8"?>\n'
             '<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">\n'
             ' <Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>\n'
             ' <Default Extension="model" ContentType="application/vnd.ms-package.3dmanufacturing-3dmodel+xml"/>\n'
             '</Types>\n')
    rels = ('<?xml version="1.0" encoding="UTF-8"?>\n'
            '<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">\n'
            ' <Relationship Target="/3D/3dmodel.model" Id="rel-1" '
            'Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel"/>\n'
            '</Relationships>\n')
    with zipfile.ZipFile(path, "w", zipfile.ZIP_DEFLATED, compresslevel=9) as z:
        z.writestr("[Content_Types].xml", types)
        z.writestr("_rels/.rels", rels)
        z.writestr("3D/3dmodel.model", model)


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

    if dst.lower().endswith(".stl"):
        from stl_union import write_binary_stl
        base3 = len(v)
        allv = np.vstack([v, rv]) if len(rv) else v
        chunks = []
        for m in order:
            if m == "Human":
                chunks.append(np.asarray(rf, dtype=np.int64) + base3)
            else:
                chunks.append(np.asarray(spans[m], dtype=np.int64))
        allf = np.vstack(chunks)
        write_binary_stl(dst, allv.astype(np.float32), allf.astype(np.uint32))
        import os
        print(f"wrote {dst} ({os.path.getsize(dst)/1048576:.1f} MB, {len(allf)} tris)"
              f"  ({time.time()-t0:.1f}s total)")
        return

    if dst.lower().endswith(".3mf"):
        base3 = len(v)
        allv = np.vstack([v, rv]) if len(rv) else v
        span_list = []
        for m in order:
            if m == "Human":
                span_list.append(("Human", np.asarray(rf, dtype=np.int64) + base3))
            else:
                span_list.append((m, np.asarray(spans[m], dtype=np.int64)))
        write_3mf(dst, allv, span_list)
        import os
        print(f"wrote {dst} ({os.path.getsize(dst)/1048576:.1f} MB)  ({time.time()-t0:.1f}s total)")
        return

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
