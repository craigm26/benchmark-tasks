"""Print-ready files for the custom parts of the charge port mock-up (ASCII STL, millimetres).

The mechanism itself is the car maker's own parts (door, pocket with its push-push latch, CCS1 inlet with its DC flap);
see README.md. This script writes the two custom pieces that hold them at the scene's dimensions:

- pocket_mount_frame.stl: a 4 mm frame screwed to the back of the 14 mm MDF fender panel, around the 176 x 181 mm
  opening, with two ears that clamp the pocket's flange (so the door sits flush in a 3 mm reveal).
- hinge_protractor.stl: a flat 3 mm scoring plate laid on the fender top edge over the door hinge, notched at
  0, -6, -75, -90 and -100 degrees, for the overhead photo.

Run it as a script: python3 fabricate.py [out_dir]. Importing it writes nothing.
"""

from __future__ import annotations

import math
import os
import sys

# Same numbers as the task file (metres there, millimetres here).
CUT_W, CUT_H = 176.0, 181.0
PANEL_W, PANEL_H, PANEL_T = 256.0, 198.0, 14.0
FRAME_BAND, FRAME_T = 18.0, 4.0
HINGE_TO_EDGE = 175.0


def _box(x0, y0, z0, x1, y1, z1):
    v = [(x, y, z) for x in (x0, x1) for y in (y0, y1) for z in (z0, z1)]
    quads = [(0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)]
    tris = []
    for a, b, c, d in quads:
        tris += [(v[a], v[b], v[c]), (v[a], v[c], v[d])]
    return tris


def _write(path, name, tris):
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


def pocket_mount_frame():
    w2, h2, b = CUT_W / 2, CUT_H / 2, FRAME_BAND
    t = []
    t += _box(-w2 - b, -h2 - b, 0, w2 + b, -h2, FRAME_T)  # bottom band
    t += _box(-w2 - b, h2, 0, w2 + b, h2 + b, FRAME_T)  # top band
    t += _box(-w2 - b, -h2, 0, -w2, h2, FRAME_T)  # left band
    t += _box(w2, -h2, 0, w2 + b, h2, FRAME_T)  # right band
    for sx in (-1, 1):  # clamp ears that hold the pocket flange 32 mm behind the panel
        x0 = sx * (w2 + 2.0)
        t += _box(min(x0, x0 + sx * 12), -20, FRAME_T, max(x0, x0 + sx * 12), 20, FRAME_T + 32)
    return t


def hinge_protractor():
    t = _box(-10, -10, 0, 10, 190, 3)  # strip along the shut door
    r = HINGE_TO_EDGE + 10
    for deg in (0, -6, -75, -90, -100):  # a 4 mm post at each scoring angle, r from the hinge
        a = math.radians(deg)
        x, y = -r * math.sin(a) * -1, r * math.cos(a)
        t += _box(x - 2, y - 2, 0, x + 2, y + 2, 6)
    return t


def main(out: str) -> None:
    os.makedirs(out, exist_ok=True)
    for name, fn in (("pocket_mount_frame", pocket_mount_frame), ("hinge_protractor", hinge_protractor)):
        path = os.path.join(out, name + ".stl")
        _write(path, name, fn())
        print("wrote", path)


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "stl")
