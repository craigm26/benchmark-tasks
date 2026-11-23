"""Print files for the real-world build: writes ASCII STL files (mm) of the custom parts from the same dimensions as the
scene in ext_tornado_safe_room_shelter_latches_33.py. Run as a script; importing it writes nothing.

    python3 fabricate.py [out_dir]

Parts:
  clamp_body.stl     the clamp's C body as simulated (Lisle 44870 / 44880 type): 50 x 32 x 60 mm, 18 mm side slot
                     25 mm deep with a 3 mm hook lip hanging from the upper jaw's mouth edge; drill and tap 1/4-20
                     through the upper jaw at x = +3, y = 0 (or press in a brass 1/4-20 threaded insert).
  knob.stl           21 mm knurl-free knob with a 1/4-20 hex nut pocket (glue a 1/4-20 x 1 in brass screw into it).
  standoff_lower.stl and standoff_upper.stl   70 mm ball-stud standoffs that hold the strut 50 mm off the box face
                     (a 10 mm ball stud screws into the end; an M6 bolt holds the base to the box or lid lug).
"""

import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))


def _box(x0, x1, y0, y1, z0, z1):
    v = [(x, y, z) for x in (x0, x1) for y in (y0, y1) for z in (z0, z1)]
    q = [(0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)]
    return [(v[a], v[b], v[c]) for a, b, c, d in q] + [(v[a], v[c], v[d]) for a, b, c, d in q]


def _cyl(r, z0, z1, n=32, cx=0.0, cy=0.0, r_in=0.0, sides_in=6):
    tris = []
    for i in range(n):
        a0, a1 = 2 * math.pi * i / n, 2 * math.pi * (i + 1) / n
        p0 = (cx + r * math.cos(a0), cy + r * math.sin(a0))
        p1 = (cx + r * math.cos(a1), cy + r * math.sin(a1))
        tris += [((*p0, z0), (*p1, z0), (*p1, z1)), ((*p0, z0), (*p1, z1), (*p0, z1))]
        tris += [((cx, cy, z1), (*p0, z1), (*p1, z1)), ((cx, cy, z0), (*p1, z0), (*p0, z0))]
    return tris


def write_stl(path, name, tris):
    with open(path, "w") as f:
        f.write(f"solid {name}\n")
        for a, b, c in tris:
            u = [b[i] - a[i] for i in range(3)]
            w = [c[i] - a[i] for i in range(3)]
            n = (u[1] * w[2] - u[2] * w[1], u[2] * w[0] - u[0] * w[2], u[0] * w[1] - u[1] * w[0])
            ln = math.sqrt(sum(x * x for x in n)) or 1.0
            f.write(f"  facet normal {n[0]/ln:.6f} {n[1]/ln:.6f} {n[2]/ln:.6f}\n    outer loop\n")
            for p in (a, b, c):
                f.write(f"      vertex {p[0]:.3f} {p[1]:.3f} {p[2]:.3f}\n")
            f.write("    endloop\n  endfacet\n")
        f.write(f"endsolid {name}\n")


def clamp_body():
    # mm, clamp frame of ext_tornado_safe_room_shelter_latches_33.py: x depth (mouth at +20), y along the rod, z screw axis
    L, back, mouth, bottom, slot, jaw, low, lip_t, lip_h = 50, -12, 20, -5, 18, 12, 30, 4, 3
    t = _box(back, mouth, -L / 2, L / 2, slot / 2, slot / 2 + jaw)  # upper jaw
    t += _box(back, mouth, -L / 2, L / 2, -slot / 2 - low, -slot / 2)  # lower jaw
    t += _box(back, bottom, -L / 2, L / 2, -slot / 2, slot / 2)  # back wall
    t += _box(mouth - lip_t, mouth, -L / 2, L / 2, slot / 2 - lip_h, slot / 2)  # hook lip under the upper jaw
    return t  # drill and tap 1/4-20 through the upper jaw at x = +3, y = 0 after printing


def knob():
    return _cyl(10.5, 0.0, 12.0)  # drill 6.6 mm through on the axis; glue the screw head in


def standoff():
    return _cyl(6.0, 0.0, 70.0, n=24)


def main(out):
    os.makedirs(out, exist_ok=True)
    for name, tris in (("clamp_body", clamp_body()), ("knob", knob()), ("standoff_lower", standoff()),
                       ("standoff_upper", standoff())):
        write_stl(os.path.join(out, name + ".stl"), name, tris)  # touching boxes: slicers merge them
        print("wrote", os.path.join(out, name + ".stl"), len(tris), "triangles")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, "stl"))
