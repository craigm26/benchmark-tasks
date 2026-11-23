"""Print and cut files for the real trial's custom parts, from the same dimensions as the scene (mm).

    python3 fabricate.py [out_dir]

writes ASCII STL files (nothing happens on import):
  door_section.stl   the 300 x 370 x 45 mm door leaf section with its 76 x 210 mm mail-slot cut, its floor 125 mm up (a router or CNC
                     template for 44-45 mm solid-core door blank or glued plywood)
  door_foot.stl      one of two 45 mm wide L feet that clamp the section upright to the table (print two, PETG)
The mail slot itself (plate, flap and spring) is the bought Salsbury 4085; the mailer is the bought Uline S-16530.
"""

import os
import sys

DOOR_W, DOOR_H, DOOR_T = 300.0, 370.0, 45.0
SLOT_Y_FROM_LEFT_EDGE = 90.0 + 60.0  # door y +90 mm edge to slot centre y -60 mm (seen from the robot)
CUT_W, CUT_H, CUT_FLOOR = 76.0, 210.0, 125.0
FOOT_L, FOOT_W, FOOT_H, FOOT_T = 120.0, 45.0, 80.0, 6.0


def _box(x0, y0, z0, x1, y1, z1):
    v = [(x, y, z) for x in (x0, x1) for y in (y0, y1) for z in (z0, z1)]
    q = [(0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)]
    tris = []
    for a, b, c, d in q:
        tris += [(v[a], v[b], v[c]), (v[a], v[c], v[d])]
    return tris


def _write(path, name, boxes):
    with open(path, "w") as f:
        f.write(f"solid {name}\n")
        for b in boxes:
            for t in _box(*b):
                f.write("  facet normal 0 0 0\n    outer loop\n")
                for p in t:
                    f.write(f"      vertex {p[0]:.3f} {p[1]:.3f} {p[2]:.3f}\n")
                f.write("    endloop\n  endfacet\n")
        f.write(f"endsolid {name}\n")


def door_section():
    """x across the door (width), y through it (thickness), z up. Four blocks round the cut (they share faces)."""
    c0 = SLOT_Y_FROM_LEFT_EDGE - CUT_W / 2
    c1 = SLOT_Y_FROM_LEFT_EDGE + CUT_W / 2
    return [(0, 0, 0, c0, DOOR_T, DOOR_H), (c1, 0, 0, DOOR_W, DOOR_T, DOOR_H),
            (c0, 0, 0, c1, DOOR_T, CUT_FLOOR), (c0, 0, CUT_FLOOR + CUT_H, c1, DOOR_T, DOOR_H)]


def door_foot():
    """An L foot: base plate on the table and an upright web the door face is screwed to."""
    return [(0, 0, 0, FOOT_L, FOOT_W, FOOT_T), (FOOT_L / 2 - FOOT_T / 2, 0, 0, FOOT_L / 2 + FOOT_T / 2, FOOT_W, FOOT_H)]


def main(out_dir="."):
    os.makedirs(out_dir, exist_ok=True)
    for name, boxes in (("door_section", door_section()), ("door_foot", door_foot())):
        path = os.path.join(out_dir, f"{name}.stl")
        _write(path, name, boxes)
        print("wrote", path)


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else ".")
