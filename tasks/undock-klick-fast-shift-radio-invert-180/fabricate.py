"""Print files for the two nylon parts that carry the mechanism, as ASCII STL in millimetres, from the same numbers the
scene uses (undock_klick_fast_shift_radio_invert_180.py). Standard library only; writes only when run as a script:

    python3 fabricate.py [output_dir]

writes dock_05_model.stl (backplate, channel, cradle, lips, lug) and stud_model.stl (base plate, neck, flange disc,
C collar). Print both in PA12 nylon or PETG at 0.12 mm layers, 4 walls, 100 % infill, with the stud's plate flat on the
bed and the dock's backplate flat on the bed. The commercial parts (Peter Jones Klick Fast Dock 05 FG-42-000013 and
Stick-On Stud FG-41-000076) are the purchased route; these files are the cheaper printed route for the same fit.

Frame: x along the stud axis with x = 0 on the lips' front face (negative toward the radio), y across, z up, origin on
the stud axis; the stud is drawn in its seated position so the two files overlay.
"""

import math
import os
import sys

# ---- dimensions (mm), from the scene -----------------------------------------------------------------------------
# axial positions measured from the front face of the stud's base plate / the dock's front lips (x = 0 there)
DOCK_LIP = (0.0, 3.0)  # front lips, 3 mm; the 20 mm slot between them
DOCK_CHAN = (3.0, 10.5)  # channel walls and cradle, 7.5 mm deep
DOCK_BACK = (10.5, 14.5)  # backplate, 4 mm
DOCK_LUG = (6.5, 10.5)  # latching lug, 4 mm, 6 x 6 mm, on the backplate at the axis
STUD_PLATE = (-3.0, -1.0)  # base plate 32.8 x 39.8 x 2, 1 mm in front of the lips (x = 0 is the lips' front face)
STUD_NECK = (-1.0, 4.0)  # neck r 7 mm through the slot
STUD_DISC = (4.0, 5.0)  # flange disc r 14.5 mm, 1 mm behind the lips
STUD_RING = (5.0, 8.5)  # C collar r 8.5 / 15 mm, 3.5 mm, gate 90 degrees open at the top (+z)
SEGMENTS = 48


def box(x0, x1, y0, y1, z0, z1):
    v = [(x, y, z) for x in (x0, x1) for y in (y0, y1) for z in (z0, z1)]
    q = lambda a, b, c, d: [(v[a], v[b], v[c]), (v[a], v[c], v[d])]  # noqa: E731
    faces = []
    for quad in ((0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)):
        faces += q(*quad)
    return faces


def sector(r_in, r_out, a0, a1, x0, x1, n=SEGMENTS):
    """A ring sector about the x axis, angles from +y toward +z (radians); r_in 0 makes a disc sector."""
    tris = []
    steps = max(2, int(n * (a1 - a0) / (2 * math.pi)))
    p = lambda r, a, x: (x, r * math.cos(a), r * math.sin(a))  # noqa: E731
    for i in range(steps):
        a, b = a0 + (a1 - a0) * i / steps, a0 + (a1 - a0) * (i + 1) / steps
        for r, flip in ((r_out, False), (r_in, True)):
            if r <= 0:
                continue
            q = [p(r, a, x0), p(r, b, x0), p(r, b, x1), p(r, a, x1)]
            tris += [(q[0], q[2], q[1]), (q[0], q[3], q[2])] if not flip else [(q[0], q[1], q[2]), (q[0], q[2], q[3])]
        for x, flip in ((x0, True), (x1, False)):
            ia, ib, oa, ob = p(r_in, a, x), p(r_in, b, x), p(r_out, a, x), p(r_out, b, x)
            t = [(ia, oa, ob), (ia, ob, ib)]
            tris += [(c, b_, a_) for a_, b_, c in t] if flip else t
    if abs((a1 - a0) - 2 * math.pi) > 1e-9:  # end faces of a partial ring
        for a in (a0, a1):
            ia, oa = p(r_in, a, x0), p(r_out, a, x0)
            ib, ob = p(r_in, a, x1), p(r_out, a, x1)
            tris += [(ia, oa, ob), (ia, ob, ib)]
    return tris


def write_stl(path, name, tris):
    with open(path, "w") as f:
        f.write(f"solid {name}\n")
        for a, b, c in tris:
            ux, uy, uz = (b[i] - a[i] for i in range(3))
            vx, vy, vz = (c[i] - a[i] for i in range(3))
            n = (uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx)
            m = math.sqrt(sum(k * k for k in n)) or 1.0
            f.write(f" facet normal {n[0] / m:.5f} {n[1] / m:.5f} {n[2] / m:.5f}\n  outer loop\n")
            for p_ in (a, b, c):
                f.write(f"   vertex {p_[0]:.4f} {p_[1]:.4f} {p_[2]:.4f}\n")
            f.write("  endloop\n endfacet\n")
        f.write(f"endsolid {name}\n")


def dock():
    t = []
    t += box(*DOCK_BACK, -23, 23, -23, 23)
    for sy in (-1, 1):  # side walls above the axis, 5 mm thick, inner faces 36 mm apart
        y0, y1 = (18, 23) if sy > 0 else (-23, -18)
        t += box(*DOCK_CHAN, y0, y1, 0, 23)
    t += sector(18.0, 23.0, math.pi, 2 * math.pi, *DOCK_CHAN)  # half-round cradle below the axis
    for y0, y1 in ((10, 23), (-23, -10)):  # front lips leaving the 20 mm slot
        t += box(*DOCK_LIP, y0, y1, -23, 23)
    t += box(*DOCK_LUG, -3, 3, -3, 3)
    return t


def stud():
    gate = math.radians(45.0)
    t = []
    t += box(*STUD_PLATE, -16.4, 16.4, -19.9, 19.9)
    t += sector(0.0, 7.0, 0.0, 2 * math.pi, *STUD_NECK)
    t += sector(0.0, 14.5, 0.0, 2 * math.pi, *STUD_DISC)
    t += sector(8.5, 15.0, math.pi / 2 + gate, math.pi / 2 - gate + 2 * math.pi, *STUD_RING)
    return t


def main(out):
    os.makedirs(out, exist_ok=True)
    write_stl(os.path.join(out, "dock_05_model.stl"), "dock_05_model", dock())
    write_stl(os.path.join(out, "stud_model.stl"), "stud_model", stud())
    print("wrote dock_05_model.stl and stud_model.stl to", out)


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "stl")
