"""Print files for the real trial's custom parts (ASCII STL, millimetres). Run: python3 fabricate.py [out_dir]

Every part that carries the mechanism is the commercial faucet itself (InSinkErator F-HC3300); the two printed parts
only make the dry trial measurable:

- angle_card.stl: a 3 mm card that clips round the faucet body (57 mm) under the handle, with raised ridges at
  10 deg back (hot line) and 30 deg and 45 deg forward (cool lines) from off, read against the handle in the top video.
- outlet_target.stl: a 3 mm disc with a 30 mm hole (the 15 mm axis rule) that lies on the counter under the outlet, so
  the video shows the cup's rim around the outlet; it is lifted away before a trial starts, leaving a pencil ring.

Dimensions are the scene's: body radius 28.5 mm, handle pivot 38.5 mm off the body axis, handle 60 mm long.
"""

import math
import os
import sys

BODY_R = 28.5
PIVOT_Y = 38.5
CARD_R = 75.0
T = 3.0


def _box(x0, y0, z0, x1, y1, z1):
    v = [(x, y, z) for x in (x0, x1) for y in (y0, y1) for z in (z0, z1)]
    q = [(0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)]
    return [(v[a], v[b], v[c]) for a, b, c, d in q] + [(v[a], v[c], v[d]) for a, b, c, d in q]


def _prism(poly, z0, z1):
    """Triangles of a straight prism over a convex polygon [(x, y), ...] (counter-clockwise)."""
    tris = []
    n = len(poly)
    for i in range(1, n - 1):
        tris.append(((*poly[0], z0), (*poly[i + 1], z0), (*poly[i], z0)))
        tris.append(((*poly[0], z1), (*poly[i], z1), (*poly[i + 1], z1)))
    for i in range(n):
        a, b = poly[i], poly[(i + 1) % n]
        tris.append(((*a, z0), (*b, z0), (*b, z1)))
        tris.append(((*a, z0), (*b, z1), (*a, z1)))
    return tris


def _sector(r0, r1, a0, a1, z0, z1, cx=0.0, cy=0.0):
    """Annular sector built from convex quads (angles from +y, positive toward -x)."""
    tris = []
    steps = max(1, int(abs(a1 - a0) / math.radians(10)))
    for i in range(steps):
        b0 = a0 + (a1 - a0) * i / steps
        b1 = a0 + (a1 - a0) * (i + 1) / steps
        pts = [(cx - r * math.sin(b), cy + r * math.cos(b)) for r, b in ((r0, b0), (r1, b0), (r1, b1), (r0, b1))]
        area = sum(pts[k][0] * pts[(k + 1) % 4][1] - pts[(k + 1) % 4][0] * pts[k][1] for k in range(4))
        tris += _prism(pts if area > 0 else pts[::-1], z0, z1)
    return tris


def angle_card():
    tris = _sector(BODY_R + 0.5, CARD_R, math.radians(-60), math.radians(60), 0.0, T, 0.0, -PIVOT_Y)
    for deg in (-10.0, 30.0, 45.0):  # ridges: hot line 10 deg back; cool lines 30 and 45 deg forward
        a = math.radians(deg)
        tris += _sector(40.0, CARD_R, a - math.radians(0.8), a + math.radians(0.8), T, T + 1.5, 0.0, -PIVOT_Y)
    return tris


def outlet_target():
    tris = []
    for i in range(36):
        a0, a1 = 2 * math.pi * i / 36, 2 * math.pi * (i + 1) / 36
        tris += _sector(15.0, 40.0, a0, a1, 0.0, T)
    return tris


def write_stl(path, name, tris):
    def nrm(a, b, c):
        u = [b[k] - a[k] for k in range(3)]
        v = [c[k] - a[k] for k in range(3)]
        n = (u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0])
        m = math.sqrt(sum(x * x for x in n)) or 1.0
        return tuple(x / m for x in n)

    with open(path, "w") as f:
        f.write(f"solid {name}\n")
        for a, b, c in tris:
            f.write("  facet normal %.6f %.6f %.6f\n    outer loop\n" % nrm(a, b, c))
            for p in (a, b, c):
                f.write("      vertex %.4f %.4f %.4f\n" % p)
            f.write("    endloop\n  endfacet\n")
        f.write(f"endsolid {name}\n")


def main(out_dir="stl"):
    os.makedirs(out_dir, exist_ok=True)
    for name, fn in (("angle_card", angle_card), ("outlet_target", outlet_target)):
        path = os.path.join(out_dir, name + ".stl")
        write_stl(path, name, fn())
        print("wrote", path)


if __name__ == "__main__":
    main(*sys.argv[1:2])
