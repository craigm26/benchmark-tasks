"""Print files for the one custom part: the probe holster (40 x 40 x 135 mm block, 12 mm blind hole 130 mm deep with a
2 mm entry chamfer, 5 mm floor), from the same dimensions as the scene. Run as a script; importing it writes nothing.

    python3 fabricate.py [out_dir]     # writes probe_holster.stl (ASCII, millimetres)
"""

import math
import os
import sys

W, H = 40.0, 135.0  # block, mm
HOLE_R, LEAD_IN, FLOOR = 6.0, 2.0, 5.0
SIDES = 24  # multiple of 4 so the square's corners fall on the ring's angles


def _rings():
    """Matching rings at the same angles: the hole's edge and the block's outline."""
    hole, outer = [], []
    for k in range(SIDES):
        a = 2 * math.pi * k / SIDES
        c, s = math.cos(a), math.sin(a)
        t = (W / 2) / max(abs(c), abs(s))
        hole.append((c, s))
        outer.append((t * c, t * s))
    return hole, outer


def triangles():
    hole, outer = _rings()
    tris = []
    n = len(hole)
    r_top = HOLE_R + LEAD_IN
    for k in range(n):
        m = (k + 1) % n
        (c0, s0), (c1, s1) = hole[k], hole[m]
        o0, o1 = outer[k], outer[m]
        top_in0, top_in1 = (r_top * c0, r_top * s0, H), (r_top * c1, r_top * s1, H)
        mid0, mid1 = (HOLE_R * c0, HOLE_R * s0, H - LEAD_IN), (HOLE_R * c1, HOLE_R * s1, H - LEAD_IN)
        bot0, bot1 = (HOLE_R * c0, HOLE_R * s0, FLOOR), (HOLE_R * c1, HOLE_R * s1, FLOOR)
        ot0, ot1 = (o0[0], o0[1], H), (o1[0], o1[1], H)
        ob0, ob1 = (o0[0], o0[1], 0.0), (o1[0], o1[1], 0.0)
        tris += [(top_in0, ot0, ot1), (top_in0, ot1, top_in1)]  # top face between hole and outline
        tris += [(ot0, ob0, ob1), (ot0, ob1, ot1)]  # outer wall
        tris += [(top_in0, top_in1, mid1), (top_in0, mid1, mid0)]  # entry chamfer
        tris += [(mid0, mid1, bot1), (mid0, bot1, bot0)]  # hole wall
        tris += [((0.0, 0.0, FLOOR), bot0, bot1)]  # hole floor
        tris += [((0.0, 0.0, 0.0), ob1, ob0)]  # bottom
    return tris


def _normal(a, b, c):
    u = [b[i] - a[i] for i in range(3)]
    v = [c[i] - a[i] for i in range(3)]
    n = (u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0])
    length = math.sqrt(sum(x * x for x in n)) or 1.0
    return tuple(x / length for x in n)


def write_stl(path, name, tris):
    with open(path, "w") as f:
        f.write(f"solid {name}\n")
        for a, b, c in tris:
            n = _normal(a, b, c)
            f.write(f"  facet normal {n[0]:.6f} {n[1]:.6f} {n[2]:.6f}\n    outer loop\n")
            for p in (a, b, c):
                f.write(f"      vertex {p[0]:.4f} {p[1]:.4f} {p[2]:.4f}\n")
            f.write("    endloop\n  endfacet\n")
        f.write(f"endsolid {name}\n")


def _edges_closed(tris):
    """Every edge used exactly twice, in opposite directions: a watertight, consistently wound mesh."""
    seen = {}
    key = lambda p: tuple(round(x, 6) for x in p)
    for t in tris:
        for i in range(3):
            e = (key(t[i]), key(t[(i + 1) % 3]))
            seen[e] = seen.get(e, 0) + 1
    return all(seen.get((b, a), 0) == n for (a, b), n in seen.items())


if __name__ == "__main__":
    out = sys.argv[1] if len(sys.argv) > 1 else "."
    os.makedirs(out, exist_ok=True)
    t = triangles()
    assert _edges_closed(t), "holster mesh is not watertight"
    write_stl(os.path.join(out, "probe_holster.stl"), "probe_holster", t)
    print(f"wrote {os.path.join(out, 'probe_holster.stl')} ({len(t)} triangles)")
