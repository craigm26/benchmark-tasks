"""Print files for the one custom part: the Infant/Child Key holder that sits in the open FRx carry case lid.

Run:  python3 fabricate.py [out_dir]   writes key_holder.stl (ASCII, mm). Nothing is written on import.

Same dimensions as the scene (aed_infant_key_before_shock.py): a 25 x 56 x 22 mm block with a 10 x 30 mm blade slot
through it, and 3 mm walls 12 mm tall round a 19 x 50 mm recess on top, where the key's 13 x 44 mm paddle rests.
Print in PETG, 0.2 mm layers, 30 % infill, slot vertical; stick it to the case lid with 3M Dual Lock.
"""

import os
import sys

SLOT_HX, SLOT_HY = 5.0, 15.0  # half sizes of the 10 x 30 mm blade slot
OX, OY = 12.5, 28.0  # outer half sizes, 25 x 56 mm
BLOCK_H = 22.0
WALL, WALL_H = 3.0, 12.0
IN_HX, IN_HY = 9.5, 25.0  # recess half sizes, 19 x 50 mm


def boxes():
    """(x0, y0, z0, x1, y1, z1) in mm, holder frame: origin at the block's base centre."""
    b = [
        (-OX, -OY, 0.0, -SLOT_HX, OY, BLOCK_H),
        (SLOT_HX, -OY, 0.0, OX, OY, BLOCK_H),
        (-SLOT_HX, SLOT_HY, 0.0, SLOT_HX, OY, BLOCK_H),
        (-SLOT_HX, -OY, 0.0, SLOT_HX, -SLOT_HY, BLOCK_H),
    ]
    top = BLOCK_H + WALL_H
    b += [
        (IN_HX, -OY, BLOCK_H, OX, OY, top),
        (-OX, -OY, BLOCK_H, -IN_HX, OY, top),
        (-IN_HX, IN_HY, BLOCK_H, IN_HX, OY, top),
        (-IN_HX, -OY, BLOCK_H, IN_HX, -IN_HY, top),
    ]
    return b


def box_facets(x0, y0, z0, x1, y1, z1):
    v = [(x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0), (x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)]
    quads = [((0, 3, 2, 1), (0, 0, -1)), ((4, 5, 6, 7), (0, 0, 1)), ((0, 1, 5, 4), (0, -1, 0)),
             ((2, 3, 7, 6), (0, 1, 0)), ((1, 2, 6, 5), (1, 0, 0)), ((3, 0, 4, 7), (-1, 0, 0))]
    for (a, b, c, d), n in quads:
        yield n, (v[a], v[b], v[c])
        yield n, (v[a], v[c], v[d])


def write_stl(path, name, solids):
    with open(path, "w") as f:
        f.write(f"solid {name}\n")
        for s in solids:
            for n, tri in box_facets(*s):
                f.write(f"  facet normal {n[0]} {n[1]} {n[2]}\n    outer loop\n")
                for p in tri:
                    f.write(f"      vertex {p[0]:.3f} {p[1]:.3f} {p[2]:.3f}\n")
                f.write("    endloop\n  endfacet\n")
        f.write(f"endsolid {name}\n")


def main(out_dir="."):
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, "key_holder.stl")
    write_stl(path, "key_holder", boxes())
    print("wrote", path)


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else ".")
