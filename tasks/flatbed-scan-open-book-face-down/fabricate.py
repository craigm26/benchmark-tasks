"""Print-ready file for the one custom part of a real trial: a 2 mm start template that sets the scanner and the book's
V in the task's fixed layout. The scanner (Canon CanoScan LiDE 400) and the book (Everyman's Library Pocket Poets
hardcover) are bought, not printed.

    python3 fabricate.py            # writes start_template.stl (mm) beside this file

The template is a flat plate with two raised rails: an L-stop the scanner's front-right corner sits against, and a
130 degree V of 3 mm tall rails the book's two tail edges stand inside. Dimensions come from the task file's layout
(scanner front edge 21.5 cm, right edge -18.35 cm; spine 20.0 cm, -3.6 cm; halves 113 mm at 130 degrees).
Nothing is written on import.
"""

import math
import os

# layout, mm, in the station frame (x forward from the arm bases, y left)
SCANNER_FRONT_X = 215.0
SCANNER_RIGHT_Y = -183.5
SPINE = (200.0, -36.0)
HALF_W = 113.0
V_DEG = 130.0
PLATE_T = 2.0
RAIL_H = 3.0
RAIL_W = 4.0


def _box(x0, x1, y0, y1, z0, z1):
    v = [(x, y, z) for x in (x0, x1) for y in (y0, y1) for z in (z0, z1)]
    f = [(0, 1, 3), (0, 3, 2), (4, 6, 7), (4, 7, 5), (0, 4, 5), (0, 5, 1),
         (2, 3, 7), (2, 7, 6), (0, 2, 6), (0, 6, 4), (1, 5, 7), (1, 7, 3)]
    return [(v[a], v[b], v[c]) for a, b, c in f]


def _rail(p0, p1, w, z0, z1):
    """A straight rail of width w from p0 to p1 (x, y), as a box turned about z."""
    dx, dy = p1[0] - p0[0], p1[1] - p0[1]
    length = math.hypot(dx, dy)
    c, s = dx / length, dy / length
    tris = _box(0.0, length, -w / 2, w / 2, z0, z1)
    out = []
    for tri in tris:
        out.append(tuple((p0[0] + c * x - s * y, p0[1] + s * x + c * y, z) for x, y, z in tri))
    return out


def template():
    tris = []
    # plate under the book's V and the scanner's front-right corner, x 120..235, y -200..70 (relative layout)
    tris += _box(120.0, 235.0, -200.0, 70.0, 0.0, PLATE_T)
    # L-stop for the scanner corner: along its front edge and its right edge
    tris += _box(SCANNER_FRONT_X - RAIL_W, SCANNER_FRONT_X, SCANNER_RIGHT_Y - RAIL_W, SCANNER_RIGHT_Y + 60.0,
                 PLATE_T, PLATE_T + RAIL_H)
    tris += _box(SCANNER_FRONT_X - RAIL_W, SCANNER_FRONT_X + 20.0, SCANNER_RIGHT_Y - RAIL_W, SCANNER_RIGHT_Y,
                 PLATE_T, PLATE_T + RAIL_H)
    # V rails just outside each board's tail edge (the boards stand against them)
    half = math.radians(V_DEG / 2)
    for sgn in (1.0, -1.0):
        u = (-math.cos(half), sgn * math.sin(half))
        out = (math.sin(half), sgn * math.cos(half))           # outward from the V, behind the board
        off = 2.5 + RAIL_W / 2
        p0 = (SPINE[0] + 8.0 * u[0] + off * out[0], SPINE[1] + 8.0 * u[1] + off * out[1])
        p1 = (SPINE[0] + HALF_W * u[0] + off * out[0], SPINE[1] + HALF_W * u[1] + off * out[1])
        tris += _rail(p0, p1, RAIL_W, PLATE_T, PLATE_T + RAIL_H)
    return tris


def write_stl(path, name, tris):
    with open(path, "w") as f:
        f.write(f"solid {name}\n")
        for a, b, c in tris:
            ux, uy, uz = (b[i] - a[i] for i in range(3))
            vx, vy, vz = (c[i] - a[i] for i in range(3))
            n = (uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx)
            m = math.sqrt(sum(k * k for k in n)) or 1.0
            f.write(f"  facet normal {n[0] / m:.6f} {n[1] / m:.6f} {n[2] / m:.6f}\n    outer loop\n")
            for p in (a, b, c):
                f.write(f"      vertex {p[0]:.4f} {p[1]:.4f} {p[2]:.4f}\n")
            f.write("    endloop\n  endfacet\n")
        f.write(f"endsolid {name}\n")


if __name__ == "__main__":
    here = os.path.dirname(os.path.abspath(__file__))
    out = os.path.join(here, "start_template.stl")
    write_stl(out, "start_template", template())
    print("wrote", out)
