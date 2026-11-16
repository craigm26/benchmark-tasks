"""Print-ready ASCII STL files (mm) for the custom parts of the hand shower docking task, from the scene's dimensions.

Run:  python3 fabricate.py [out_dir]      (default: ./stl next to this file)

Standard library only; importing this file writes nothing. Every part is one or more closed shells, each built as a
sweep of rings (watertight, outward facing); where a part has several shells (the holder's cup, arm and rosette) they
overlap and the slicer joins them into one solid (PrusaSlicer, Bambu Studio and OrcaSlicer always do; Cura with its
default "Union Overlapping Volumes"). The README gives material, print orientation and the assembly.
"""

import math
import os
import sys

# ---- dimensions, mm (the same values as the task file) ----------------------------------------------------------------
TILT = 15.0  # deg: the cup leans its rim toward the robot
CUP_H, R_SEAT, R_RIM, CUP_WALL = 32.0, 13.0, 17.0, 4.0
SLOT_A = 2.374  # rad: the cup wall spans -SLOT_A..+SLOT_A from the back; the gap is the front slot (18.0 mm at the ring)
C0 = (395.0, 25.0, 151.0)  # bottom-ring centre in the scene (x forward, y left, z above the table)
WALL_X = 470.0  # tile face
ARM_X0 = 407.8 - C0[0]  # the arm starts inside the cup's back wall
ARM_W, ARM_T, ARM_Z = 24.0, 16.0, 21.0  # arm width (y), thickness (z), centre height above the ring centre
ROSETTE_R, ROSETTE_T = 25.0, 6.0

NUT_R, NUT_L = 10.0, 10.0
HANDLE_R0, HANDLE_R, TAPER_END, HANDLE_END = 11.0, 14.0, 31.0, 160.0
HEAD_AT, HEAD_R, HEAD_T = 205.0, 45.0, 28.0
BAND = (9.0, 15.0)  # red depth band, mm from the nut's tip (paint it)

SPOUT = (458.0, 165.0, 172.7)  # where the hose leaves the outlet elbow, pointing down
ELBOW_R, ELBOW_ROSETTE_R, ELBOW_ROSETTE_T = 10.0, 22.0, 6.0
ELBOW_Z = SPOUT[2] + 24.0  # the elbow's horizontal bar, out of the wall

LEDGE = (158.0, 90.0, 100.0)  # stone ledge block, x by y by height
LEDGE_SHELL = 3.0

# hose socket with a barb (both ends of the hose): 1/2 in OD x 1/4 in ID neoprene tube (12.7 x 6.35 mm)
SOCKET_R, SOCKET_D = 6.5, 15.0  # 13.0 mm bore, 15 mm deep
BARB_R, BARB_RIDGE_R = 3.3, 3.7  # 6.6 mm shaft, 7.4 mm ridge: the tube grips it
BARB_TIP = 2.0  # the barb stops 2 mm inside the mouth

STEP = math.radians(3.0)


# ---- geometry helpers ----------------------------------------------------------------------------------------------------
def circle(r, z, n=120):
    if r is None:
        return [(0.0, 0.0, z)]
    return [(r * math.cos(2 * math.pi * i / n), r * math.sin(2 * math.pi * i / n), z) for i in range(n)]


def sweep(rings, caps=False):
    """Join consecutive rings (same point count, or a single axis point) into a surface. caps=True closes an open
    sweep's first and last rings with fans (the rings must be convex polygons)."""
    tris = []
    for a, b in zip(rings, rings[1:]):
        if len(a) == 1:
            n = len(b)
            tris += [(a[0], b[(i + 1) % n], b[i]) for i in range(n)]
        elif len(b) == 1:
            n = len(a)
            tris += [(a[i], a[(i + 1) % n], b[0]) for i in range(n)]
        else:
            n = len(a)
            for i in range(n):
                j = (i + 1) % n
                tris += [(a[i], a[j], b[j]), (a[i], b[j], b[i])]
    if caps:
        a, b = rings[0], rings[-1]
        tris += [(a[0], a[k + 1], a[k]) for k in range(1, len(a) - 1)]
        tris += [(b[0], b[k], b[k + 1]) for k in range(1, len(b) - 1)]
    return tris


def lathe(profile, n=120):
    """Solid of revolution about z from [(r or None, z), ...], walked round the section from axis to axis."""
    return sweep([circle(r, z, n) for r, z in profile])


def volume(tris):
    return sum((p[0] * (q[1] * r[2] - q[2] * r[1]) - p[1] * (q[0] * r[2] - q[2] * r[0])
                + p[2] * (q[0] * r[1] - q[1] * r[0])) for p, q, r in tris) / 6.0


def outward(tris):
    """One closed shell, wound so its signed volume is positive."""
    return tris if volume(tris) > 0 else [(p, r, q) for p, q, r in tris]


def move(tris, f):
    return [tuple(f(v) for v in t) for t in tris]


def rot_y(deg):
    """Rotation about y: +deg turns +z toward +x (the cup's -15 deg tilt leans its rim toward -x)."""
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return lambda v: (c * v[0] + s * v[2], v[1], -s * v[0] + c * v[2])


def z_to_x(v):
    """Turn a part built along +z so it points along +x (a proper rotation about y)."""
    return (v[2], v[1], -v[0])


def z_to_minus_x(v):
    return (-v[2], v[1], v[0])


def shift(dx, dy, dz):
    return lambda v: (v[0] + dx, v[1] + dy, v[2] + dz)


def box(x0, x1, y0, y1, z0, z1):
    ring = lambda z: [(x0, y0, z), (x1, y0, z), (x1, y1, z), (x0, y1, z)]  # noqa: E731
    return outward(sweep([ring(z0), ring(z1)], caps=True))


def edges_ok(tris):
    """Every directed edge appears once and its reverse once: the shells are closed and consistently wound."""
    seen = {}
    for t in tris:
        for a, b in ((t[0], t[1]), (t[1], t[2]), (t[2], t[0])):
            k = (tuple(round(c, 5) for c in a), tuple(round(c, 5) for c in b))
            seen[k] = seen.get(k, 0) + 1
    return all(c == 1 and seen.get((b, a)) == 1 for (a, b), c in seen.items())


def bounds(tris):
    lo = [min(v[i] for t in tris for v in t) for i in range(3)]
    hi = [max(v[i] for t in tris for v in t) for i in range(3)]
    return lo, hi


def to_bed(tris):
    z0 = bounds(tris)[0][2]
    return move(tris, shift(0.0, 0.0, -z0))


def write_stl(path, name, tris):
    with open(path, "w") as f:
        f.write(f"solid {name}\n")
        for p, q, r in tris:
            ux, uy, uz = (q[i] - p[i] for i in range(3))
            vx, vy, vz = (r[i] - p[i] for i in range(3))
            nx, ny, nz = uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx
            m = math.sqrt(nx * nx + ny * ny + nz * nz) or 1.0
            f.write(f"  facet normal {nx / m:.6e} {ny / m:.6e} {nz / m:.6e}\n    outer loop\n")
            for v in (p, q, r):
                f.write(f"      vertex {v[0]:.4f} {v[1]:.4f} {v[2]:.4f}\n")
            f.write("    endloop\n  endfacet\n")
        f.write(f"endsolid {name}\n")


# ---- the parts (scene frame: origin as noted, x toward the wall, y left, z up) ---------------------------------------------
def holder_shells():
    """Slotted conical cup leaning 15 deg toward the robot, its arm and the 50 mm wall rosette. Origin: the bottom
    ring's centre; the rosette's back face is the tile face (x = 75 mm)."""
    n = max(8, int(2 * SLOT_A / STEP))
    rings = []
    for i in range(n + 1):
        t = -SLOT_A + 2 * SLOT_A * i / n
        c, s = math.cos(t), math.sin(t)
        sec = [(R_SEAT, 0.0), (R_SEAT + CUP_WALL, 0.0), (R_RIM + CUP_WALL, CUP_H), (R_RIM, CUP_H)]
        rings.append([(r * c, r * s, z) for r, z in sec])
    cup = outward(move(sweep(rings, caps=True), rot_y(-TILT)))
    x_wall = WALL_X - C0[0]
    arm = box(ARM_X0, x_wall - ROSETTE_T + 0.5, -ARM_W / 2, ARM_W / 2, ARM_Z - ARM_T / 2, ARM_Z + ARM_T / 2)
    disc = lathe([(None, 0.0), (ROSETTE_R, 0.0), (ROSETTE_R, ROSETTE_T), (None, ROSETTE_T)])
    rosette = outward(move(move(disc, z_to_x), shift(x_wall - ROSETTE_T, 0.0, ARM_Z)))
    return [cup, arm, rosette]


def handset_shells():
    """Hand shower, its own frame: origin at the nut's tip, +x along the handle to the head, the spray face at -z.
    The nut has a 13 mm socket 15 mm deep with a barb for the hose."""
    body = lathe([(None, BARB_TIP), (2.8, BARB_TIP), (BARB_RIDGE_R, BARB_TIP + 2.0), (BARB_RIDGE_R, BARB_TIP + 4.0),
                  (BARB_R, BARB_TIP + 4.5), (BARB_R, SOCKET_D), (SOCKET_R, SOCKET_D), (SOCKET_R, 0.0),
                  (NUT_R, 0.0), (NUT_R, NUT_L), (HANDLE_R0, NUT_L), (HANDLE_R, TAPER_END),
                  (HANDLE_R, HANDLE_END + 4.0), (None, HANDLE_END + 4.0)])
    body = outward(move(body, z_to_x))
    head = lathe([(None, -HEAD_T / 2), (HEAD_R, -HEAD_T / 2), (HEAD_R, HEAD_T / 2), (None, HEAD_T / 2)])
    head = outward(move(head, shift(HEAD_AT, 0.0, 0.0)))
    return [body, head]


def elbow_shells():
    """Outlet elbow; origin: the tile face behind the spout, at table height. The hose's anchored end pushes into the
    socket at the bottom of the down bar (the spout)."""
    dx = SPOUT[0] - WALL_X  # -12 mm: the down bar's axis, in front of the tile
    down = lathe([(None, BARB_TIP), (2.8, BARB_TIP), (BARB_RIDGE_R, BARB_TIP + 2.0), (BARB_RIDGE_R, BARB_TIP + 4.0),
                  (BARB_R, BARB_TIP + 4.5), (BARB_R, SOCKET_D), (SOCKET_R, SOCKET_D), (SOCKET_R, 0.0),
                  (ELBOW_R, 0.0), (ELBOW_R, ELBOW_Z - SPOUT[2] + 8.0), (None, ELBOW_Z - SPOUT[2] + 8.0)])
    down = outward(move(down, shift(dx, 0.0, SPOUT[2])))
    bar = lathe([(None, 0.0), (ELBOW_R, 0.0), (ELBOW_R, -dx - 0.5), (None, -dx - 0.5)])
    bar = outward(move(move(bar, z_to_minus_x), shift(-0.5, 0.0, ELBOW_Z)))  # from the down bar's axis to the tile
    disc = lathe([(None, 0.0), (ELBOW_ROSETTE_R, 0.0), (ELBOW_ROSETTE_R, ELBOW_ROSETTE_T), (None, ELBOW_ROSETTE_T)])
    rosette = outward(move(move(disc, z_to_x), shift(-ELBOW_ROSETTE_T, 0.0, ELBOW_Z)))
    return [down, bar, rosette]


def ledge_shells():
    """The ledge block as an open-bottom box with 3 mm walls (print upside down, weight it with sand bags or clamp it)."""
    lx, ly, lh = LEDGE
    t = LEDGE_SHELL
    outer = [(0, 0), (lx, 0), (lx, ly), (0, ly)]
    inner = [(t, t), (lx - t, t), (lx - t, ly - t), (t, ly - t)]
    ring = lambda pts, z: [(x, y, z) for x, y in pts]  # noqa: E731
    shell = sweep([ring(inner, 0.0), ring(inner, lh - t)])  # inner walls
    top_in = ring(inner, lh - t)
    shell += [(top_in[0], top_in[k], top_in[k + 1]) for k in (1, 2)]  # underside of the top plate
    top_out = ring(outer, lh)
    shell += sweep([ring(outer, lh), ring(outer, 0.0)])  # outer walls
    shell += [(top_out[0], top_out[k + 1], top_out[k]) for k in (1, 2)]  # top face
    bot_o, bot_i = ring(outer, 0.0), ring(inner, 0.0)
    for i in range(4):  # the bottom rim, between the outer and inner walls
        j = (i + 1) % 4
        shell += [(bot_o[i], bot_i[j], bot_i[i]), (bot_o[i], bot_o[j], bot_i[j])]
    return [outward(shell)]


PARTS = [  # name, shells, count, material and orientation (also in the README)
    ("holder", holder_shells, 1, "black PETG, 4 walls, 40% infill, rim up as in the scene, supports under the arm and rosette"),
    ("handset", handset_shells, 1, "white PETG, spray face down, supports under the handle and nut, 3 walls, infill set for 100 g"),
    ("outlet_elbow", elbow_shells, 1, "black PETG, rosette back on the bed, supports under the down bar"),
    ("ledge", ledge_shells, 1, "grey PLA, top face on the bed, 3 walls"),
]


def main(out):
    os.makedirs(out, exist_ok=True)
    for name, make, count, how in PARTS:
        shells = make()
        tris = to_bed([t for s in shells for t in s])
        if not edges_ok(tris):
            raise SystemExit(f"{name}: mesh is not closed")
        write_stl(os.path.join(out, name + ".stl"), name, tris)
        lo, hi = bounds(tris)
        size = " x ".join(f"{h - l:.1f}" for l, h in zip(lo, hi))
        vol = sum(volume(s) for s in shells) / 1000.0
        print(f"{name}.stl  print {count}  {size} mm  shells {len(shells)}  {vol:.1f} cm3 solid (overlaps counted)  "
              f"{len(tris)} triangles  [{how}]")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.path.dirname(os.path.abspath(__file__)), "stl"))
