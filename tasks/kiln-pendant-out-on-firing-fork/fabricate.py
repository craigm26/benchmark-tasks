"""Print or cut files for the real-trial kiln stand-in, from the same dimensions as the scene.

    python3 fabricate.py [out_dir]

Writes ASCII STL files (mm) of the cold kiln shell: its base with the chamber floor, the two side walls, the back
wall, the roof slab and the 6 mm shelf, the door panel, and a rack-corner bending gauge (12 mm legs). Each part is a
union of boxes, so it can be printed in sections or cut from 12 mm plywood or 25 mm fibre board by the listed sizes.
Nothing is written on import.
"""

import os
import sys

# Same numbers as kiln_pendant_out_on_firing_fork.py (metres there, millimetres here).
CASE_D, CASE_W, CASE_H, STAND_H = 356.0, 337.0, 394.0, 35.0
CH_D, CH_W, CH_FLOOR, CH_ROOF = 197.0, 203.0, 160.0, 306.0
SHELF_T = 6.0
LEG_H, RACK_W = 12.0, 76.0


def _box(x0, y0, z0, x1, y1, z1):
    v = [(x, y, z) for x in (x0, x1) for y in (y0, y1) for z in (z0, z1)]
    quads = [(0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)]
    tris = []
    for a, b, c, d in quads:
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


def parts():
    hw, cw = CASE_W / 2, CH_W / 2
    top = STAND_H + CASE_H
    return {
        "kiln_base_and_floor": [(0, -hw, STAND_H, CASE_D, hw, CH_FLOOR)],
        "kiln_side_wall_left": [(0, cw, CH_FLOOR, CASE_D, hw, CH_ROOF)],
        "kiln_side_wall_right": [(0, -hw, CH_FLOOR, CASE_D, -cw, CH_ROOF)],
        "kiln_back_wall": [(CH_D, -cw, CH_FLOOR, CASE_D, cw, CH_ROOF)],
        "kiln_roof": [(0, -hw, CH_ROOF, CASE_D, hw, top)],
        "kiln_shelf": [(5, -cw + 5, 0, CH_D - 5, cw - 5, SHELF_T)],
        "kiln_door": [(0, 0, 0, 40, CASE_W, 300)],
        "rack_corner_gauge": [(0, 0, 0, RACK_W, RACK_W, 3), (0, 0, 3, 8, 8, 3 + LEG_H)],
    }


def main(out_dir="stl"):
    os.makedirs(out_dir, exist_ok=True)
    for name, boxes in parts().items():
        _write(os.path.join(out_dir, f"{name}.stl"), name, boxes)
        print(f"wrote {out_dir}/{name}.stl")


if __name__ == "__main__":
    main(*sys.argv[1:2])
