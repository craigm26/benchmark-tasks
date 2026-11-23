"""Write print-ready ASCII STL files (mm) for the custom parts of the EPO reset task, from the same dimensions as the
scene in ext_dc_bess_microgrid_battery_container_service_9.py. Standard library only; nothing is written on import.

    python3 fabricate.py [out_dir]

The mechanism itself is bought, not printed: a Schneider Harmony XALK178E control station (turn-to-release 40 mm
mushroom head, 1 NO + 1 NC). The custom parts only hold it where the scene puts it:

- wall_foot.stl (print two): an L foot that clamps the 12 mm plywood wall panel upright on the table; its slot is the
  panel thickness plus 0.4 mm. PETG, 0.2 mm layers, 40 % infill.
- drill_template.stl: a 3 mm plate laid on the panel's front face, flush with its bottom edge, with a 10 mm pilot hole
  at the station's centre (140 mm up, centred across) and the 68 mm square outline of the station as a raised rim, so
  the station goes on at the scene's height.
"""

import os
import sys

# Scene dimensions in mm (match the task file).
WALL_T, WALL_W, WALL_H = 12.0, 300.0, 220.0
BOX_W = 68.0
AXIS_Z = 140.0
FOOT_L, FOOT_W, FOOT_H = 100.0, 40.0, 12.0
SLOT = WALL_T + 0.4
UPRIGHT_H = 50.0


def box(x0, x1, y0, y1, z0, z1):
    v = [(x, y, z) for x in (x0, x1) for y in (y0, y1) for z in (z0, z1)]
    q = [(0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)]
    return [(v[a], v[b], v[c]) for a, b, c, d in q] + [(v[a], v[c], v[d]) for a, b, c, d in q]


def parts():
    w = 8.0  # upright jaw thickness either side of the slot
    foot = (box(0, FOOT_L, 0, FOOT_W, 0, FOOT_H)                                    # base, screwed to the table
            + box(-SLOT - 2 * w, -SLOT - w, 0, FOOT_W, 0, UPRIGHT_H)               # front jaw
            + box(-w, 0, 0, FOOT_W, 0, UPRIGHT_H)                                  # back jaw
            + box(-SLOT - 2 * w, 0, 0, FOOT_W, -3.0, 0.0))                         # floor under the slot
    t, hole = 3.0, 5.0
    cx, cz = WALL_W / 2, AXIS_Z
    plate = (box(0, WALL_W, 0, t, 0, cz - hole) + box(0, WALL_W, 0, t, cz + hole, cz + BOX_W / 2 + 20)
             + box(0, cx - hole, 0, t, cz - hole, cz + hole) + box(cx + hole, WALL_W, 0, t, cz - hole, cz + hole))
    h = BOX_W / 2
    rim = (box(cx - h - 2, cx + h + 2, -2, 0, cz - h - 2, cz - h) + box(cx - h - 2, cx + h + 2, -2, 0, cz + h, cz + h + 2)
           + box(cx - h - 2, cx - h, -2, 0, cz - h, cz + h) + box(cx + h, cx + h + 2, -2, 0, cz - h, cz + h))
    return {"wall_foot": foot, "drill_template": plate + rim}


def write_stl(path, name, tris):
    with open(path, "w") as f:
        f.write(f"solid {name}\n")
        for a, b, c in tris:
            f.write("  facet normal 0 0 0\n    outer loop\n")
            for p in (a, b, c):
                f.write(f"      vertex {p[0]:.3f} {p[1]:.3f} {p[2]:.3f}\n")
            f.write("    endloop\n  endfacet\n")
        f.write(f"endsolid {name}\n")


if __name__ == "__main__":
    out = sys.argv[1] if len(sys.argv) > 1 else "stl"
    os.makedirs(out, exist_ok=True)
    for name, tris in parts().items():
        write_stl(os.path.join(out, name + ".stl"), name, tris)
        print(f"wrote {name}.stl ({len(tris)} triangles)")
