"""Print files for the real trial's custom parts, from the same dimensions as the scene (mm).

    python3 fabricate.py [out_dir]

Writes ASCII STL files (only when run as a script, never on import):
  tube_template.stl   1:1 dip tube template: a 3 mm plate with a 7 mm groove along the tube's factory curve
                      (100 mm straight, then R 170 mm for 84 mm). A tube that lies in the groove has no new bend.
  bottle_locator.stl  a 2 mm ring (inner 72 mm) taped to the tray under each bottle, so both stand where marked.
  seat_gauge.stl      a 7.5 mm feeler: slides between the neck bead and the collar skirt (1.5 mm apart seated)
                      only where the seal face is over 6 mm up (not seated).
The bottles, sprayer and tray are bought (see README); nothing printed carries the mechanism.
"""

import math
import os
import sys

STRAIGHT, BEND_R, LENGTH, TUBE_OD = 100.0, 170.0, 184.0, 6.0


def _tri(f, a, b, c):
    ux, uy, uz = (b[i] - a[i] for i in range(3))
    vx, vy, vz = (c[i] - a[i] for i in range(3))
    n = (uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx)
    m = math.sqrt(sum(v * v for v in n)) or 1.0
    f.write(f"facet normal {n[0] / m:.6f} {n[1] / m:.6f} {n[2] / m:.6f}\n outer loop\n")
    for p in (a, b, c):
        f.write(f"  vertex {p[0]:.4f} {p[1]:.4f} {p[2]:.4f}\n")
    f.write(" endloop\nendfacet\n")


def _box(f, x0, y0, z0, x1, y1, z1):
    p = [(x, y, z) for z in (z0, z1) for y in (y0, y1) for x in (x0, x1)]
    for a, b, c, d in ((0, 2, 3, 1), (4, 5, 7, 6), (0, 1, 5, 4), (2, 6, 7, 3), (0, 4, 6, 2), (1, 3, 7, 5)):
        _tri(f, p[a], p[b], p[c])
        _tri(f, p[a], p[c], p[d])


def _ring(f, r_in, r_out, h, n=96):
    for k in range(n):
        a0, a1 = 2 * math.pi * k / n, 2 * math.pi * (k + 1) / n
        pi0, pi1 = (r_in * math.cos(a0), r_in * math.sin(a0)), (r_in * math.cos(a1), r_in * math.sin(a1))
        po0, po1 = (r_out * math.cos(a0), r_out * math.sin(a0)), (r_out * math.cos(a1), r_out * math.sin(a1))
        for z, flip in ((0.0, True), (h, False)):
            q = [(*pi0, z), (*po0, z), (*po1, z), (*pi1, z)]
            if flip:
                q = q[::-1]
            _tri(f, q[0], q[1], q[2])
            _tri(f, q[0], q[2], q[3])
        _tri(f, (*po0, 0), (*po1, 0), (*po1, h))
        _tri(f, (*po0, 0), (*po1, h), (*po0, h))
        _tri(f, (*pi1, 0), (*pi0, 0), (*pi0, h))
        _tri(f, (*pi1, 0), (*pi0, h), (*pi1, h))


def tube_centreline(step=4.0):
    pts, s = [], 0.0
    while s <= LENGTH + 1e-9:
        if s <= STRAIGHT:
            pts.append((0.0, s))
        else:
            a = (s - STRAIGHT) / BEND_R
            pts.append((BEND_R * (1 - math.cos(a)), STRAIGHT + BEND_R * math.sin(a)))
        s += step
    return pts


def write_template(path):
    """Plate 3 mm, with 2 mm walls either side of a 7 mm channel along the centreline (boxes per 4 mm step)."""
    with open(path, "w") as f:
        f.write("solid tube_template\n")
        _box(f, -15.0, -5.0, 0.0, 45.0, 195.0, 3.0)
        pts = tube_centreline()
        half = TUBE_OD / 2 + 0.5
        for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
            for side in (-1, 1):
                cx = (x0 + x1) / 2 + side * (half + 1.0)
                _box(f, cx - 1.0, min(y0, y1), 3.0, cx + 1.0, max(y0, y1) + 0.5, 7.0)
        f.write("endsolid tube_template\n")


def write_locator(path):
    with open(path, "w") as f:
        f.write("solid bottle_locator\n")
        _ring(f, 36.0, 40.0, 2.0)
        f.write("endsolid bottle_locator\n")


def write_gauge(path):
    with open(path, "w") as f:
        f.write("solid seat_gauge\n")
        _box(f, 0.0, 0.0, 0.0, 60.0, 15.0, 7.5)
        _box(f, 60.0, 0.0, 0.0, 90.0, 15.0, 12.0)
        f.write("endsolid seat_gauge\n")


if __name__ == "__main__":
    out = sys.argv[1] if len(sys.argv) > 1 else "."
    os.makedirs(out, exist_ok=True)
    write_template(os.path.join(out, "tube_template.stl"))
    write_locator(os.path.join(out, "bottle_locator.stl"))
    write_gauge(os.path.join(out, "seat_gauge.stl"))
    print("wrote tube_template.stl, bottle_locator.stl, seat_gauge.stl to", out)
