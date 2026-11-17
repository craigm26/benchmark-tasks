"""Print-ready ASCII STL files (mm) for the swing-arm wall mirror's custom parts, from the scene's dimensions.
The parts are a full-size 6 inch two-arm wall mirror of the Jerdon JP7508C's class (150 mm head, about 10 inch reach;
head tilt left out); link lengths, the stops (+-90, +-178, +-95 deg) and hinge friction are this reference build's own.

Run:  python3 fabricate.py [out_dir]      (default: ./stl next to this file)

Standard library only; importing this file writes nothing. Each part is one or more closed shells (sweeps of rings,
watertight and outward facing); where a part has several shells they overlap and the slicer joins them into one solid
(PrusaSlicer, Bambu Studio and OrcaSlicer always do; Cura with its default "Union Overlapping Volumes"). The README gives
the hardware, print settings and the assembly.

Frames: every part is drawn in its own hinge frame (origin on its hinge axis, +x along the link or the glass normal,
z up, z = 0 at the part's own height in the scene) and then dropped onto the bed.
"""

import math
import os
import sys

# ---- dimensions, mm (the same values as the task file) ----------------------------------------------------------------
HOLE_R = 2.65  # 5.3 mm clearance hole for the M5 hinge screws
L1, L2 = 110.0, 100.0  # link lengths, axis to axis
TUBE_R = 9.0  # 18 mm link tubes
# shoulder: wall plate and bracket (shoulder axis 30 mm in front of the tile; plate 80 mm across, 15 mm thick)
PLATE_R, PLATE_X0, PLATE_X1 = 40.0, 15.0, 30.0
KNUCKLE_R, KNUCKLE_H = 11.0, 26.0  # shoulder knuckle 22 mm, 26 mm tall: a clevis round link 1's 18 mm eye
EAR_GAP = 19.0  # between the bracket's ears (link 1's eye is 18 mm tall)
BLOCK_X0 = 11.5  # the bracket's back block starts 0.5 mm outside link 1's eye
CHEEK = (9.0, 15.5, 12.0, 20.0, 9.0)  # stop cheeks: x0, x1, |y| from, |y| to, half height; link 1's flank lands at +-90
# elbow: link 1's knuckle (20 mm) below, link 2's eye above, joint face 16.5 mm above link 1's axis
ELBOW_R, ELBOW_Z0, ELBOW_FACE, ELBOW_Z1 = 10.0, -13.0, 16.5, 39.0  # heights from link 1's axis; link 2's axis is at 26
Z_LINK2 = 26.0
# elbow stop: 3 mm dowels 86 mm from the elbow axis, on link 1's top and link 2's underside, meet at +-178 deg
PIN_R, PIN_D = 86.0, 3.0
# swivel: link 2's boss (22 mm, 18 tall) is a clevis round the head's 5.5 mm eye; its back web stops the neck at +-95
BOSS_R, BOSS_H, SLOT_H = 11.0, 18.0, 6.0
WEB_R0, WEB_HALF = 8.0, 55.0  # web from r 8 mm, +-55 deg about the back of the boss
EYE_R, EYE_H = 7.5, 5.5
BAR_W, BAR_X1 = 8.0, 13.5  # flat neck bar inside the slot
NECK_R = 7.0
# head: one-sided round mirror, bezel 150 mm across and 25 mm thick, its back 17.5 mm from the swivel axis
HEAD_R, BEZEL_X0, BEZEL_X1 = 75.0, 17.5, 42.5
MIRROR_R, MIRROR_T, PROUD = 68.0, 3.0, 0.6  # 136 mm silver acrylic mirror disc, 3 mm, 0.6 mm proud of the bezel
RECESS_R = 68.3
BACK_STEP = 0.6  # raised back disc: keeps the head 26.2 mm thick, as in the scene

N = 96


# ---- geometry helpers ----------------------------------------------------------------------------------------------------
def circle(r, z, n=N, a0=0.0, a1=None):
    if r is None:
        return [(0.0, 0.0, z)]
    if a1 is None:
        return [(r * math.cos(2 * math.pi * i / n), r * math.sin(2 * math.pi * i / n), z) for i in range(n)]
    return [(r * math.cos(a0 + (a1 - a0) * i / n), r * math.sin(a0 + (a1 - a0) * i / n), z) for i in range(n + 1)]


def sweep(rings, caps=False):
    """Join consecutive closed rings (same point count, or a single axis point) into a surface; caps=True closes the
    first and last rings with fans (they must be convex)."""
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


def volume(tris):
    return sum((p[0] * (q[1] * r[2] - q[2] * r[1]) - p[1] * (q[0] * r[2] - q[2] * r[0])
                + p[2] * (q[0] * r[1] - q[1] * r[0])) for p, q, r in tris) / 6.0


def outward(tris):
    return tris if volume(tris) > 0 else [(p, r, q) for p, q, r in tris]


def move(tris, f):
    return [tuple(f(v) for v in t) for t in tris]


def shift(dx, dy, dz):
    return lambda v: (v[0] + dx, v[1] + dy, v[2] + dz)


def z_to_x(v):
    """Turn a part built along +z so it points along +x (a proper rotation about y)."""
    return (v[2], v[1], -v[0])


def lathe(profile, n=N):
    """Solid of revolution about z from [(r or None, z), ...], walked round the section from axis to axis."""
    return outward(sweep([circle(r, z, n) for r, z in profile]))


def ring_solid(r_in, r_out, z0, z1, n=N):
    """A washer: annulus r_in..r_out from z0 to z1 (a closed torus-like shell)."""
    pts = [(r_in, z0), (r_out, z0), (r_out, z1), (r_in, z1)]
    rings = [circle(r, z, n) for r, z in pts]
    return outward(sweep(rings + [rings[0]]))


def rod_x(r, x0, x1, y=0.0, z=0.0):
    """A solid round bar along +x from x0 to x1."""
    return outward(move(move(lathe([(None, 0.0), (r, 0.0), (r, x1 - x0), (None, x1 - x0)]), z_to_x),
                        shift(x0, y, z)))


def box(x0, x1, y0, y1, z0, z1):
    ring = lambda z: [(x0, y0, z), (x1, y0, z), (x1, y1, z), (x0, y1, z)]  # noqa: E731
    return outward(sweep([ring(z0), ring(z1)], caps=True))


def sector(r0, r1, a0, a1, z0, z1, n=24):
    """Annular sector r0..r1 between angles a0..a1 (deg), z0..z1: a sweep over angle of a 4-point section, capped."""
    rings = []
    for i in range(n + 1):
        t = math.radians(a0 + (a1 - a0) * i / n)
        c, s = math.cos(t), math.sin(t)
        rings.append([(r * c, r * s, z) for r, z in ((r0, z0), (r1, z0), (r1, z1), (r0, z1))])
    return outward(sweep(rings, caps=True))


def edges_ok(tris):
    """Every directed edge appears once and its reverse once: the shell is closed and consistently wound."""
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


# ---- the parts ------------------------------------------------------------------------------------------------------------
def wall_bracket():
    """Wall plate, bracket ears (a clevis round link 1's eye), back block and the two shoulder stop cheeks. Origin: the
    shoulder axis at link 1's height; +x toward the tile (the plate's back face, x = 30, goes on the tile)."""
    plate = outward(move(move(lathe([(None, 0.0), (PLATE_R, 0.0), (PLATE_R, PLATE_X1 - PLATE_X0),
                                     (None, PLATE_X1 - PLATE_X0)]), z_to_x), shift(PLATE_X0, 0.0, 0.0)))
    h, g = KNUCKLE_H / 2, EAR_GAP / 2
    shells = [plate]
    for z0, z1 in ((g, h), (-h, -g)):  # the two ears: a ring round the screw plus an arm back to the plate
        shells += [ring_solid(HOLE_R, KNUCKLE_R, z0, z1), box(6.0, PLATE_X0 + 0.5, -KNUCKLE_R, KNUCKLE_R, z0, z1)]
    shells.append(box(BLOCK_X0, PLATE_X0 + 0.5, -CHEEK[3], CHEEK[3], -g - 0.1, g + 0.1))  # back block joins the ears
    x0, x1, y0, y1, hz = CHEEK
    shells += [box(x0, x1, y0, y1, -hz, hz), box(x0, x1, -y1, -y0, -hz, hz)]
    return shells


def link1():
    """Link 1: the shoulder eye (between the bracket's ears), the 18 mm tube and the lower half of the elbow knuckle.
    Origin: the shoulder axis; +x toward the elbow. Pin A (a 3 mm dowel) goes into a hole drilled in the tube's top
    86 mm from the elbow axis (24 mm from the shoulder axis)."""
    return [ring_solid(HOLE_R, KNUCKLE_R, -EAR_GAP / 2 + 0.5, EAR_GAP / 2 - 0.5),
            rod_x(TUBE_R, 8.0, L1 - 8.0),
            outward(move(ring_solid(HOLE_R, ELBOW_R, ELBOW_Z0, ELBOW_FACE), shift(L1, 0.0, 0.0)))]


def link2():
    """Link 2: the elbow eye (on top of link 1's knuckle), the tube, and the swivel boss: two ears round the head's
    eye and a back web that stops the head's neck bar at +-95 deg. Origin: the elbow axis at link 2's height; +x toward
    the swivel. Pin B (a 3 mm dowel) goes into a hole drilled in the tube's underside 86 mm from the elbow axis."""
    e0, e1 = ELBOW_FACE - Z_LINK2, ELBOW_Z1 - Z_LINK2
    h, s = BOSS_H / 2, SLOT_H / 2
    boss = [ring_solid(HOLE_R, BOSS_R, s, h), ring_solid(HOLE_R, BOSS_R, -h, -s),
            sector(WEB_R0, BOSS_R, 180.0 - WEB_HALF, 180.0 + WEB_HALF, -s - 0.1, s + 0.1)]
    return [ring_solid(HOLE_R, ELBOW_R, e0, e1), rod_x(TUBE_R, 8.0, L2 - 9.0),
            *[outward(move(b, shift(L2, 0.0, 0.0))) for b in boss]]


def head():
    """Head: the eye (in link 2's boss slot), the flat neck bar, the round neck and the bezel with its front recess for
    the 136 mm mirror disc. Origin: the swivel axis at the head's centre height; +x along the glass normal."""
    bezel = lathe([(None, BEZEL_X0 - BACK_STEP), (MIRROR_R, BEZEL_X0 - BACK_STEP), (MIRROR_R, BEZEL_X0),
                   (HEAD_R, BEZEL_X0), (HEAD_R, BEZEL_X1), (RECESS_R, BEZEL_X1),
                   (RECESS_R, BEZEL_X1 - (MIRROR_T - PROUD)), (None, BEZEL_X1 - (MIRROR_T - PROUD))])
    bezel = outward(move(bezel, z_to_x))
    return [ring_solid(HOLE_R, EYE_R, -EYE_H / 2, EYE_H / 2),
            box(4.0, BAR_X1, -BAR_W / 2, BAR_W / 2, -EYE_H / 2, EYE_H / 2),
            rod_x(NECK_R, 13.0, BEZEL_X0 - BACK_STEP + 1.0), bezel]


PARTS = [  # name, shells, count, print settings and orientation (also in the README)
    ("wall_bracket", wall_bracket, 1, "black PETG, 100% infill, plate back on the bed, supports under the ears"),
    ("link1", link1, 1, "black PETG, 100% infill, on its side (tube axis flat), tree supports"),
    ("link2", link2, 1, "black PETG, 100% infill, on its side (tube axis flat), tree supports"),
    ("head", head, 1, "black PETG, 4 walls, infill set for 395 g (about 70%), standing on its rim, tree supports"),
]
PETG = 1.27  # g/cm3


def main(out):
    os.makedirs(out, exist_ok=True)
    for name, make, count, how in PARTS:
        shells = make()
        for k, s in enumerate(shells):
            if not edges_ok(s):
                raise SystemExit(f"{name}: shell {k} is not closed")
        tris = [t for s in shells for t in s]
        z0 = bounds(tris)[0][2]
        tris = move(tris, shift(0.0, 0.0, -z0))
        write_stl(os.path.join(out, name + ".stl"), name, tris)
        lo, hi = bounds(tris)
        size = " x ".join(f"{b - a:.1f}" for a, b in zip(lo, hi))
        vol = sum(volume(s) for s in shells) / 1000.0
        print(f"{name}.stl  print {count}  {size} mm  shells {len(shells)}  {vol:.1f} cm3 solid (overlaps counted, "
              f"about {vol * PETG:.0f} g PETG at 100%)  {len(tris)} triangles  [{how}]")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.path.dirname(os.path.abspath(__file__)), "stl"))
