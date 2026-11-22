"""Print files for the real trial, from the same dimensions as the scene (mm).

Bought, not printed: the Brunson 1.5THDN magnetic drift nest (aluminium, 40.6 mm across, 12.7 mm tall, SMR centre
28.6 mm above its foot) and the SMR stand-in, a 1-1/2 in (38.1 mm) 440C stainless steel ball (McMaster-Carr 9529K51,
magnetic), the same diameter and the same steel as the housing of FARO's standard 1.5 in SMR (440C, 150 g). The
stand-in has no optics; its face is painted on. Nothing printed carries the turning mechanism: the parts below are
painting guides, gauges and the measuring collar.

- face_stencil.stl: a spherical cap that sits on the ball with a hole 22 mm across (the SMR's open face, cut flat
  15.55 mm from the centre on a real SMR); paint the face black through it.
- ring_guide_10/20/30.stl: the same cap with a hole at 10, 20 and 30 degrees from the face axis (plan radius
  19.05 x sin(angle) = 3.31, 6.52 and 9.53 mm); trace each hole's edge with a fine white paint pen. A laser pointer
  aimed at the ball centre lands where the surface faces it, so the spot's ring reads the aim error directly.
- lift_cal_cap.stl: a cap whose flat top sits exactly 3.0 mm above the top of the ball. Set it on the seated ball, take
  one frame from each side camera and mark the cap's top edge: that line is the 3 mm lift limit.
- aim_stem.stl: a cap that centres on the painted face, carrying a 40 mm stem along the face axis, for the reset.
- start_wedge.stl: a 60 degree wedge, set on the plate pointing +y; the stem lies along its slope at the start pose.
- collar_half.stl (print two): a clamshell collar round the ball's equator (two M3 x 16 screws), with a top eye for
  the pull test and a lug 50 mm from the centre for the turning-torque test (README, "Validate the stand-in").

Run: python3 fabricate.py  (writes the STL files beside this script; importing it writes nothing).
"""

import math
import os

R_BALL = 19.05          # mm, 1.5 in SMR and the 1-1/2 in 440C ball
FACE_R = 11.0           # open face 22 mm across
FACE_Z = math.sqrt(R_BALL ** 2 - FACE_R ** 2)     # 15.55 mm: the face plane on a real SMR
RINGS = {a: R_BALL * math.sin(math.radians(a)) for a in (10, 20, 30)}
CLR = 0.15              # print clearance on the ball
SHELL = 2.0             # cap wall
LIFT = 3.0              # mm: the scored lift limit


def revolve(profile, n=96, a0=0.0, a1=2 * math.pi):
    """Triangles of a solid of revolution about z from a closed polyline [(r, z), ...]. Over a full turn, points
    with r = 0 close the solid on the axis; over a part turn, both end faces are fanned from the profile's centroid."""
    full = abs(a1 - a0 - 2 * math.pi) < 1e-9
    steps = n if full else max(2, int(n * (a1 - a0) / (2 * math.pi)))
    tris = []
    closed = not full or profile[0][0] > 0 or profile[-1][0] > 0
    pts = list(profile) + ([profile[0]] if closed else [])
    for i in range(len(pts) - 1):
        (r0, z0), (r1, z1) = pts[i], pts[i + 1]
        for k in range(steps):
            b0, b1 = a0 + (a1 - a0) * k / steps, a0 + (a1 - a0) * (k + 1) / steps
            p00 = (r0 * math.cos(b0), r0 * math.sin(b0), z0)
            p01 = (r0 * math.cos(b1), r0 * math.sin(b1), z0)
            p10 = (r1 * math.cos(b0), r1 * math.sin(b0), z1)
            p11 = (r1 * math.cos(b1), r1 * math.sin(b1), z1)
            if r0 > 0:
                tris.append((p00, p01, p11))
            if r1 > 0:
                tris.append((p00, p11, p10))
    if not full:
        cr = sum(p[0] for p in profile) / len(profile)
        cz = sum(p[1] for p in profile) / len(profile)
        for b, flip in ((a0, True), (a1, False)):
            c, s = math.cos(b), math.sin(b)
            ctr = (cr * c, cr * s, cz)
            for i in range(len(profile)):
                (r0, z0), (r1, z1) = profile[i], profile[(i + 1) % len(profile)]
                p, q = (r0 * c, r0 * s, z0), (r1 * c, r1 * s, z1)
                tris.append((ctr, p, q) if flip else (ctr, q, p))
    return tris


def arc(r, a_from, a_to, n=24):
    """(r, z) points on a sphere of radius r, polar angle a_from .. a_to (0 = top)."""
    return [(r * math.sin(a_from + (a_to - a_from) * i / n), r * math.cos(a_from + (a_to - a_from) * i / n))
            for i in range(n + 1)]


def cap(hole_plan_r, rim_deg=55.0):
    """A spherical cap shell on the ball's top (axis z = a face or ring axis), with a centred hole of plan radius
    hole_plan_r on the ball's surface, and its rim rim_deg from the axis."""
    ri, ro = R_BALL + CLR, R_BALL + CLR + SHELL
    a_hole = math.asin(hole_plan_r / R_BALL) if hole_plan_r > 0 else 0.0
    a_rim = math.radians(rim_deg)
    inner = arc(ri, a_hole, a_rim)
    outer = arc(ro, a_rim, a_hole)
    if hole_plan_r <= 0:
        inner[0], outer[-1] = (0.0, ri), (0.0, ro)
    return inner + outer


def lift_cal_cap():
    """Flat-topped cap: inner surface on the ball from the top to 45 deg, flat top LIFT mm above the ball's top."""
    ri = R_BALL + 0.02                                     # snug: its flat top must sit exactly 3.0 mm above the ball
    a_rim = math.radians(45.0)
    inner = [(0.0, ri)] + arc(ri, 0.0, a_rim)[1:]
    r_out = ri * math.sin(a_rim) + 2.0
    return inner + [(r_out, ri * math.cos(a_rim)), (r_out, R_BALL + LIFT), (0.0, R_BALL + LIFT)]


def box(c, size):
    (x, y, z), (dx, dy, dz) = c, [s / 2 for s in size]
    v = [(x + sx * dx, y + sy * dy, z + sz * dz) for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]
    f = [(0, 1, 3), (0, 3, 2), (4, 6, 7), (4, 7, 5), (0, 4, 5), (0, 5, 1), (2, 3, 7), (2, 7, 6), (0, 2, 6),
         (0, 6, 4), (1, 5, 7), (1, 7, 3)]
    return [(v[a], v[b], v[c2]) for a, b, c2 in f]


def aim_stem():
    """Face-centring cap (hole 22 mm, the painted face) with a 4 mm stem 40 mm long on the face axis, held by three
    spokes across the hole that bridge over the face without touching it."""
    tris = revolve(cap(FACE_R, rim_deg=50.0))
    top = R_BALL + CLR + SHELL
    tris += revolve([(0.0, FACE_Z + 4.0), (2.0, FACE_Z + 4.0), (2.0, top + 40.0), (0.0, top + 40.0)])
    for k in range(3):
        a = 2 * math.pi * k / 3
        mid = (R_BALL + 2.0) / 2
        c = (mid * math.cos(a), mid * math.sin(a), top + 1.0)
        tris += [tuple(_rotz(p, a, c) for p in t) for t in box(c, (R_BALL + 2.0, 3.0, 2.0))]
    return tris


def _rotz(p, a, c):
    x, y = p[0] - c[0], p[1] - c[1]
    return (c[0] + x * math.cos(a) - y * math.sin(a), c[1] + x * math.sin(a) + y * math.cos(a), p[2])


def wedge(length=60.0, width=20.0, angle=60.0):
    h = length * math.tan(math.radians(angle)) / 2
    v = [(0, 0, 0), (length / 2, 0, 0), (0, 0, h), (0, width, 0), (length / 2, width, 0), (0, width, h)]
    f = [(0, 1, 2), (3, 5, 4), (0, 4, 1), (0, 3, 4), (0, 5, 3), (0, 2, 5), (1, 5, 2), (1, 4, 5)]
    return [(v[a], v[b], v[c]) for a, b, c in f]


def collar_half():
    """Half of a 16 mm tall clamshell band round the ball's equator (inner surface on the sphere, outer radius 27 mm),
    with screw bosses, a top eye post and a 50 mm torque lug with an end lip for the thread."""
    ri, ro, hz = R_BALL + 0.05, 27.0, 8.0
    a_lo, a_hi = math.acos(hz / ri), math.acos(-hz / ri)
    prof = arc(ri, a_lo, a_hi, 16) + [(ro, -hz), (ro, hz)]
    tris = revolve(prof, n=96, a0=0.0, a1=math.pi)
    for y in (ri + 4.0, -(ri + 4.0)):                      # screw bosses at the split (drill 3.2 mm through in z)
        tris += box((4.0, y, 0.0), (8.0, 8.0, 2 * hz))
    tris += box((ro + 2.0, 0.0, hz + 6.0), (4.0, 8.0, 12.0))     # eye post for the pull test (drill 3 mm across)
    tris += box((ro + (50.0 - ro) / 2, 0.0, 0.0), (50.0 - ro + 2.0, 6.0, 6.0))    # torque lug out to 50 mm
    tris += box((50.0, 0.0, 0.0), (2.0, 6.0, 12.0))               # lip that keeps the thread at 50 mm
    return tris


def write_stl(path, name, tris):
    with open(path, "w") as f:
        f.write(f"solid {name}\n")
        for a, b, c in tris:
            u = [b[i] - a[i] for i in range(3)]
            w = [c[i] - a[i] for i in range(3)]
            n = (u[1] * w[2] - u[2] * w[1], u[2] * w[0] - u[0] * w[2], u[0] * w[1] - u[1] * w[0])
            ln = math.sqrt(sum(k * k for k in n)) or 1.0
            f.write(f"  facet normal {n[0] / ln:.6f} {n[1] / ln:.6f} {n[2] / ln:.6f}\n    outer loop\n")
            for p in (a, b, c):
                f.write(f"      vertex {p[0]:.4f} {p[1]:.4f} {p[2]:.4f}\n")
            f.write("    endloop\n  endfacet\n")
        f.write(f"endsolid {name}\n")


def parts():
    out = [("face_stencil", revolve(cap(FACE_R)))]
    out += [(f"ring_guide_{a}", revolve(cap(r))) for a, r in RINGS.items()]
    out += [("lift_cal_cap", revolve(lift_cal_cap())), ("aim_stem", aim_stem()), ("start_wedge", wedge()),
            ("collar_half", collar_half())]
    return out


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    for name, tris in parts():
        path = os.path.join(here, f"{name}.stl")
        write_stl(path, name, tris)
        print(f"wrote {path} ({len(tris)} facets)")


if __name__ == "__main__":
    main()
