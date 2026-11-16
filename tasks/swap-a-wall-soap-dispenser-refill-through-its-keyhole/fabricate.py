"""Print-ready ASCII STL files (mm) for the custom parts of the soap dispenser task, from the scene's dimensions.

Run:  python3 fabricate.py [out_dir]      (default: ./stl next to this file)

Standard library only. Box parts are meshed as a union of boxes on a shared grid (watertight); round and chamfered
parts are swept through star-shaped sections. Positions are the scene's (x forward, y left, z above the table, in mm)
moved so each part sits on z = 0; the README gives the print orientation, material and the assembly.
"""

import math
import os
import sys

# ---- dimensions, mm (the same values as the task file) ----------------------------------------------------------------
LEDGE, LEDGE_T = 150.0, 6.0
ENTR_X, ENTR_W = 310.0, 44.0
SLOT_W, SLOT_END = 34.0, 367.0
WALL_Y0, WALL_Y1 = 57.5, 62.5
FRONT_X, BACK_X = 262.0, 397.0
HINGE = (262.0, 0.0, 90.0)
PIVOT = (290.0, 0.0, 125.5)
FORK_TOP, RIDGE_TOP = 120.0, 123.0
RIDGE_X, RIDGE_Y = (349.0, 357.0), (12.5, 17.5)
COVER_H, COVER_T, COVER_DEEP = 231.0, 5.0, 143.0
BODY_X, BODY_Y, BODY_H, CHAMFER = 76.0, 64.0, 150.0, 6.0
NECK_R, PUMP_R, COLLAR_R, NOZZLE_R = 14.0, 11.0, 19.0, 5.0
STAND_GAP, STAND_H = 46.0, 50.0
BIN_IN, BIN_H, BIN_T = 180.0, 150.0, 3.0
TRAY_X, TRAY_Y, TRAY_H, TRAY_T = (270.0, 390.0), 50.0, 15.0, 2.0

PIN = 3.4  # square pin holes for a 3 mm steel rod
WALL = 1.2  # refill shell wall
FIT = 0.25  # sliding clearance on a radius


# ---- box unions on a grid ---------------------------------------------------------------------------------------------
def box(x0, x1, y0, y1, z0, z1):
    return (min(x0, x1), max(x0, x1), min(y0, y1), max(y0, y1), min(z0, z1), max(z0, z1))


def box_union(add, sub=()):
    """Watertight triangles of (union of add) minus (union of sub), all axis-aligned boxes."""
    cuts = [sorted({round(v, 6) for b in list(add) + list(sub) for v in (b[2 * a], b[2 * a + 1])}) for a in range(3)]
    n = [len(c) - 1 for c in cuts]

    def inside(b, p):
        return all(b[2 * a] < p[a] < b[2 * a + 1] for a in range(3))

    full = set()
    for i in range(n[0]):
        for j in range(n[1]):
            for k in range(n[2]):
                p = ((cuts[0][i] + cuts[0][i + 1]) / 2, (cuts[1][j] + cuts[1][j + 1]) / 2,
                     (cuts[2][k] + cuts[2][k + 1]) / 2)
                if any(inside(b, p) for b in add) and not any(inside(b, p) for b in sub):
                    full.add((i, j, k))
    tris = []
    for (i, j, k) in full:
        lo = (cuts[0][i], cuts[1][j], cuts[2][k])
        hi = (cuts[0][i + 1], cuts[1][j + 1], cuts[2][k + 1])
        for a in range(3):
            for side in (0, 1):
                nb = [i, j, k]
                nb[a] += 1 if side else -1
                if tuple(nb) in full:
                    continue
                u, v = (a + 1) % 3, (a + 2) % 3
                c = [None] * 4
                for m, (su, sv) in enumerate(((0, 0), (1, 0), (1, 1), (0, 1))):
                    q = [0.0, 0.0, 0.0]
                    q[a] = hi[a] if side else lo[a]
                    q[u] = hi[u] if su else lo[u]
                    q[v] = hi[v] if sv else lo[v]
                    c[m] = tuple(q)
                if not side:
                    c.reverse()
                tris += [(c[0], c[1], c[2]), (c[0], c[2], c[3])]
    return tris


# ---- sweeps through star-shaped sections -------------------------------------------------------------------------------
def chamfer_rect(hx, hy, c):
    """Edges (normal, offset) of a rectangle with 45 degree chamfered corners."""
    s = 1 / math.sqrt(2)
    e = [((1, 0), hx), ((-1, 0), hx), ((0, 1), hy), ((0, -1), hy)]
    e += [((sx * s, sy * s), (hx + hy - c) * s) for sx in (1, -1) for sy in (1, -1)]
    return e


def corners(hx, hy, c):
    return [math.atan2(sy * a, sx * b) for sx in (1, -1) for sy in (1, -1) for a, b in ((hy - c, hx), (hy, hx - c))]


def section(shape, z, angles):
    """Ring of points at height z: shape is a radius (circle), a list of edges (convex polygon) or None (axis)."""
    if shape is None:
        return [(0.0, 0.0, z)]
    pts = []
    for t in angles:
        u = (math.cos(t), math.sin(t))
        if isinstance(shape, (int, float)):
            r = float(shape)
        else:
            r = min(d / (nx * u[0] + ny * u[1]) for (nx, ny), d in shape if nx * u[0] + ny * u[1] > 1e-9)
        pts.append((r * u[0], r * u[1], z))
    return pts


def sweep(rings, closed=False):
    """Join consecutive rings (same point count, or a single axis point) into a surface; closed joins last to first."""
    tris = []
    pairs = list(zip(rings, rings[1:])) + ([(rings[-1], rings[0])] if closed else [])
    for a, b in pairs:
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
    return tris


def angle_set(extra=()):
    a = {round(2 * math.pi * i / 96 - math.pi, 9) for i in range(96)}
    a |= {round(v, 9) for v in extra}
    return sorted(a)


# ---- mesh helpers ---------------------------------------------------------------------------------------------------
def volume(tris):
    return sum((p[0] * (q[1] * r[2] - q[2] * r[1]) - p[1] * (q[0] * r[2] - q[2] * r[0])
                + p[2] * (q[0] * r[1] - q[1] * r[0])) for p, q, r in tris) / 6.0


def finish(tris):
    """Outward winding, then sit the part on z = 0."""
    if volume(tris) < 0:
        tris = [(p, r, q) for p, q, r in tris]
    z0 = min(v[2] for t in tris for v in t)
    return [tuple((v[0], v[1], v[2] - z0) for v in t) for t in tris]


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


# ---- the parts --------------------------------------------------------------------------------------------------------
def ledge():
    """6 mm keyhole plate between the side walls: 44 x 44 mm entrance at x 310, 34 mm slot back to x 367."""
    e0, e1, s, z0 = ENTR_X - ENTR_W / 2, ENTR_X + ENTR_W / 2, SLOT_W / 2, LEDGE - LEDGE_T
    return box_union([box(FRONT_X, BACK_X, -WALL_Y0, WALL_Y0, z0, LEDGE)],
                     [box(e0, e1, -ENTR_W / 2, ENTR_W / 2, z0, LEDGE), box(e1, SLOT_END, -s, s, z0, LEDGE)])


def back_plate():
    return box_union([box(BACK_X, 405.0, -WALL_Y1, WALL_Y1, 50.0, 315.0)])


def side_wall():
    """Left wall (print two; the right one is the same part turned over). Pin holes for the lever and the cover."""
    hx, hz = HINGE[0], HINGE[2]
    px, pz = PIVOT[0], PIVOT[2]
    add = [box(FRONT_X, BACK_X, WALL_Y0, WALL_Y1, 50.0, LEDGE),
           box(hx - 6, hx + 6, WALL_Y0, WALL_Y1, hz - 6, hz + 6)]  # hinge boss in front of the wall
    sub = [box(hx - PIN / 2, hx + PIN / 2, WALL_Y0, WALL_Y1, hz - PIN / 2, hz + PIN / 2),
           box(px - PIN / 2, px + PIN / 2, WALL_Y0, WALL_Y1, pz - PIN / 2, pz + PIN / 2)]
    return box_union(add, sub)


def cover():
    """Cover in its shut pose, hinge line at the origin: face, top plate, side returns, two knuckles on the pin."""
    add = [box(-COVER_T, 0, -WALL_Y1, WALL_Y1, 0, COVER_H - COVER_T),
           box(-COVER_T, COVER_DEEP, -WALL_Y1, WALL_Y1, COVER_H - COVER_T, COVER_H),
           box(0, 25, WALL_Y0, WALL_Y1, 65, COVER_H - COVER_T), box(0, 25, -WALL_Y1, -WALL_Y0, 65, COVER_H - COVER_T),
           box(-6, 6, 51, 57, -6, 6), box(-6, 6, -57, -51, -6, 6)]
    sub = [box(-COVER_T, 0, 57, WALL_Y1, 0, 9), box(-COVER_T, 0, -WALL_Y1, -57, 0, 9),  # clear the wall bosses
           box(-PIN / 2, PIN / 2, -57, 57, -PIN / 2, PIN / 2)]
    return box_union(add, sub)


def lever():
    """Push lever in its rest pose: bar, side arms, pivot lugs, U fork with ridges, counterweight."""
    px, pz = PIVOT[0], PIVOT[2]
    add = [box(252, 264, -55, 55, 50, 80),
           box(264, 274, 40, 50, 50, 122), box(264, 274, -50, -40, 50, 122),
           box(274, 334, 40, 50, 112, 122), box(274, 334, -50, -40, 112, 122),
           box(px - 6, px + 6, 40, 50, 112, pz + 6), box(px - 6, px + 6, -50, -40, 112, pz + 6),  # pivot lugs
           box(334, 380, 12, 50, FORK_TOP - 5, FORK_TOP), box(334, 380, -50, -12, FORK_TOP - 5, FORK_TOP),
           box(366, 380, -12, 12, FORK_TOP - 5, FORK_TOP),
           box(RIDGE_X[0], RIDGE_X[1], RIDGE_Y[0], RIDGE_Y[1], FORK_TOP, RIDGE_TOP),
           box(RIDGE_X[0], RIDGE_X[1], -RIDGE_Y[1], -RIDGE_Y[0], FORK_TOP, RIDGE_TOP),
           box(364, 390, -45, 45, 93, 115)]
    sub = [box(px - PIN / 2, px + PIN / 2, -50, 50, pz - PIN / 2, pz + PIN / 2)]
    return box_union(add, sub)


def lever_rest_stop():
    """Block glued to the back plate's face under the counterweight: the lever rests on it (joint 0)."""
    return box_union([box(380, BACK_X, -10, 10, 83, 93)])


def lever_travel_stop():
    """Block glued under the ledge's back strip: the counterweight's top corner meets it at 0.12 rad of push."""
    return box_union([box(388, BACK_X, -10, 10, 127, LEDGE - LEDGE_T)])


def pin_spacer():
    """7.5 mm spacer on the pivot pin between each lever arm and its wall (print two)."""
    a = angle_set()
    r_in, r_out, h = PIN / 2, 4.0, WALL_Y0 - 50.0
    return sweep([section(r_in, 0, a), section(r_out, 0, a), section(r_out, h, a), section(r_in, h, a)], closed=True)


_OUT = chamfer_rect(BODY_X / 2, BODY_Y / 2, CHAMFER)
_IN = [(n, d - WALL) for n, d in _OUT]
_PLUG = [(n, d - WALL - 0.2) for n, d in _OUT]
_TOP = chamfer_rect(BODY_X / 2 - CHAMFER, BODY_Y / 2 - CHAMFER, CHAMFER / 2)
_ANG = angle_set(corners(BODY_X / 2, BODY_Y / 2, CHAMFER) + corners(BODY_X / 2 - CHAMFER, BODY_Y / 2 - CHAMFER, CHAMFER / 2))


def refill_bottle():
    """Bottle shell, shoulder face down (refill frame: shoulder face z 0, bottle up to 144, neck below to -11), open at
    the top for the ballast; a 22.5 mm bore through the neck for the pump plunger."""
    a = _ANG
    return sweep([section(_IN, BODY_H - CHAMFER, a), section(_IN, WALL, a), section(PUMP_R + FIT, WALL, a),
                  section(PUMP_R + FIT, -11.0, a), section(NECK_R, -11.0, a), section(NECK_R, 0.0, a),
                  section(_OUT, 0.0, a), section(_OUT, BODY_H - CHAMFER, a)], closed=True)


def refill_lid():
    """Chamfered base of the bottle with a 4 mm plug that presses into the shell's open end."""
    a = _ANG
    z = BODY_H - CHAMFER
    return sweep([section(None, z - 4, a), section(_PLUG, z - 4, a), section(_PLUG, z, a), section(_OUT, z, a),
                  section(_TOP, BODY_H, a), section(None, BODY_H, a)])


def refill_plunger():
    """Nozzle, 38 mm collar and the 22 mm pump plunger with its retaining flange (rests on the shell's inner face)."""
    a = angle_set()
    return sweep([section(None, -38, a), section(NOZZLE_R, -38, a), section(NOZZLE_R, -26, a),
                  section(COLLAR_R, -26, a), section(COLLAR_R, -22, a), section(PUMP_R, -22, a),
                  section(PUMP_R, WALL, a), section(13.5, WALL, a), section(13.5, WALL + 2, a),
                  section(None, WALL + 2, a)])


def refill_spring_cup():
    """Cup glued over the bore inside the bottle: holds the pump spring down on the plunger's flange."""
    a = angle_set()
    top_in = WALL + 2 + 31  # flange top + installed spring length
    return sweep([section(None, top_in, a), section(14.5, top_in, a), section(14.5, WALL, a), section(16.5, WALL, a),
                  section(16.5, top_in + 2, a), section(None, top_in + 2, a)])


def cradle_rail():
    """One of the two cradle rails (print two), 100 x 27 x 50 mm; set them 46 mm apart."""
    return box_union([box(0, 100, 0, 27, 0, STAND_H)])


def drip_tray():
    x0, x1 = TRAY_X
    return box_union([box(x0, x1, -TRAY_Y, TRAY_Y, 0, TRAY_H)],
                     [box(x0 + TRAY_T, x1 - TRAY_T, -TRAY_Y + TRAY_T, TRAY_Y - TRAY_T, TRAY_T, TRAY_H)])


def waste_bin():
    o = BIN_IN / 2 + BIN_T
    return box_union([box(-o, o, -o, o, 0, BIN_H)], [box(-BIN_IN / 2, BIN_IN / 2, -BIN_IN / 2, BIN_IN / 2, BIN_T, BIN_H)])


PARTS = [("ledge", ledge, 1), ("back_plate", back_plate, 1), ("side_wall", side_wall, 2), ("cover", cover, 1),
         ("lever", lever, 1), ("lever_rest_stop", lever_rest_stop, 1),
         ("lever_travel_stop", lever_travel_stop, 1), ("pin_spacer", pin_spacer, 2), ("refill_bottle", refill_bottle, 2),
         ("refill_lid", refill_lid, 2), ("refill_plunger", refill_plunger, 2),
         ("refill_spring_cup", refill_spring_cup, 2), ("cradle_rail", cradle_rail, 2), ("drip_tray", drip_tray, 1),
         ("waste_bin", waste_bin, 1)]


def main(out):
    os.makedirs(out, exist_ok=True)
    for name, make, count in PARTS:
        tris = finish(make())
        write_stl(os.path.join(out, name + ".stl"), name, tris)
        lo = [min(v[i] for t in tris for v in t) for i in range(3)]
        hi = [max(v[i] for t in tris for v in t) for i in range(3)]
        size = " x ".join(f"{h - l:.1f}" for l, h in zip(lo, hi))
        print(f"{name}.stl  print {count}  {size} mm  {volume(tris) / 1000:.1f} cm3  {len(tris)} triangles")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.path.dirname(os.path.abspath(__file__)), "stl"))
