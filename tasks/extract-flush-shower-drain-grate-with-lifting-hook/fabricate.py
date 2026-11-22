"""Writes print-ready ASCII STL files (millimetres) of the task's custom parts, from the same dimensions as the scene:

    python3 fabricate.py [output_dir]

hook.stl         the lifting hook: 22 x 7 x 4 mm foot, 6 mm round shaft (12-sided), 22 x 30 x 14 mm T-handle (PETG, 100 %
                 infill or turned from 304 stainless rod)
grate.stl        the 136 x 136 x 15 mm slotted grate (11 bars, no clips); print with the ribs up
frame.stl        the 152 mm frame with its 10 mm ledge, 142 mm recess and 122 mm well (print in two halves if the bed is small)
deck_tile.stl    the 176 x 216 x 40 mm tile deck with its 152 mm square pocket
holder.stl       the 60 x 44 x 34 mm hook holder with its 30 x 13 mm slot, 30 mm deep (PETG, or cast in silicone)
mat.stl          the 176 x 154 x 10 mm service mat (cast silicone or cut from 10 mm EVA foam)

Nothing is written when this file is imported, only when it is run as a script.
"""
import math
import os
import sys


def _box(c, s):
    """Twelve triangles of an axis-aligned box centred at c with edge lengths s."""
    x0, y0, z0 = (c[i] - s[i] / 2 for i in range(3))
    x1, y1, z1 = (c[i] + s[i] / 2 for i in range(3))
    v = [(x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0), (x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)]
    f = [(0, 3, 2), (0, 2, 1), (4, 5, 6), (4, 6, 7), (0, 1, 5), (0, 5, 4), (2, 3, 7), (2, 7, 6), (1, 2, 6), (1, 6, 5),
         (3, 0, 4), (3, 4, 7)]
    return [(v[a], v[b], v[d]) for a, b, d in f]


def _prism(c, r, h, sides=12):
    """A vertical prism of ``sides`` sides, vertices on radius r, centred at c (its axis along z)."""
    ring = [(c[0] + r * math.cos(2 * math.pi * i / sides), c[1] + r * math.sin(2 * math.pi * i / sides)) for i in range(sides)]
    z0, z1 = c[2] - h / 2, c[2] + h / 2
    tris = []
    for i in range(sides):
        a, b = ring[i], ring[(i + 1) % sides]
        tris += [((a[0], a[1], z0), (b[0], b[1], z0), (b[0], b[1], z1)), ((a[0], a[1], z0), (b[0], b[1], z1), (a[0], a[1], z1))]
        tris += [((c[0], c[1], z0), (b[0], b[1], z0), (a[0], a[1], z0)), ((c[0], c[1], z1), (a[0], a[1], z1), (b[0], b[1], z1))]
    return tris


def _normal(t):
    (ax, ay, az), (bx, by, bz), (cx, cy, cz) = t
    n = ((by - ay) * (cz - az) - (bz - az) * (cy - ay), (bz - az) * (cx - ax) - (bx - ax) * (cz - az),
         (bx - ax) * (cy - ay) - (by - ay) * (cx - ax))
    m = math.sqrt(sum(v * v for v in n)) or 1.0
    return tuple(v / m for v in n)


def write_stl(path, name, tris):
    with open(path, "w") as f:
        f.write(f"solid {name}\n")
        for t in tris:
            n = _normal(t)
            f.write(f"  facet normal {n[0]:.6f} {n[1]:.6f} {n[2]:.6f}\n    outer loop\n")
            for p in t:
                f.write(f"      vertex {p[0]:.4f} {p[1]:.4f} {p[2]:.4f}\n")
            f.write("    endloop\n  endfacet\n")
        f.write(f"endsolid {name}\n")


def hook():
    """Printed standing on its handle: shaft along z, foot at the top (the foot's long side along x)."""
    t = _box((8.0, 0, 88 - 2.0), (22, 7, 4)) + _prism((0, 0, 88 - 37.0), 3.0, 74.0) + _box((0, 0, 7.0), (22, 30, 14))
    return t


def grate():
    t = []
    h, g, sk = 15.0, 68.0, 7.0
    for sx in (-1, 1):
        t += _box((sx * (g - sk / 2), 0, h / 2), (sk, 2 * g, h))
        t += _box((0, sx * (g - sk / 2), h / 2), (2 * (g - sk), sk, h))
    for y, w in ((0, 12), (-24, 10), (24, 10), (-46.5, 9), (46.5, 9)):
        t += _box((0, y, 12.5), (2 * (g - sk), w, 5))
    for x in (-21, 21):
        t += _box((x, 0, 12.5), (6, 2 * (g - sk), 5))
    return t


def frame():
    t = []
    W, O, I = 76.0, 71.0, 61.0
    t += _box((-(W + I) / 2, 0, 12.5), (W - I, 2 * W, 25)) + _box(((W + I) / 2, 0, 12.5), (W - I, 2 * W, 25))
    t += _box((0, -(W + I) / 2, 12.5), (2 * I, W - I, 25)) + _box((0, (W + I) / 2, 12.5), (2 * I, W - I, 25))
    t += _box((-(W + O) / 2, 0, 32.5), (W - O, 2 * W, 15)) + _box(((W + O) / 2, 0, 32.5), (W - O, 2 * W, 15))
    t += _box((0, -(W + O) / 2, 32.5), (2 * O, W - O, 15)) + _box((0, (W + O) / 2, 32.5), (2 * O, W - O, 15))
    return t


def deck_tile():
    t = []
    t += _box((-82, -24, 20), (12, 216, 40)) + _box((82, -24, 20), (12, 216, 40))
    t += _box((0, 80, 20), (152, 8, 40)) + _box((0, -104, 20), (152, 56, 40))
    return t


def holder():
    t = _box((0, 0, 2), (60, 44, 4))
    t += _box((0, -14.25, 17), (60, 15.5, 34)) + _box((0, 14.25, 17), (60, 15.5, 34))
    t += _box((-22.5, 0, 17), (15, 13, 34)) + _box((22.5, 0, 17), (15, 13, 34))
    return t


def mat():
    return _box((0, 0, 5), (176, 154, 10))


if __name__ == "__main__":
    out = sys.argv[1] if len(sys.argv) > 1 else "stl"
    os.makedirs(out, exist_ok=True)
    for name, fn in (("hook", hook), ("grate", grate), ("frame", frame), ("deck_tile", deck_tile), ("holder", holder),
                     ("mat", mat)):
        tris = fn()
        write_stl(os.path.join(out, name + ".stl"), name, tris)
        print(f"{name}.stl: {len(tris)} triangles")
