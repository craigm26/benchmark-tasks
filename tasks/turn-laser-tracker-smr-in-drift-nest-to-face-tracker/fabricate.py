"""Print files for the real trial, from the same dimensions as the scene (mm).

The nest is bought (Brunson 1.5THDN magnetic drift nest). The SMR may be a real 1.5 in red-ring SMR, or, to keep the
trial cheap and spare a real one the gripper pads, this printed replica:

- smr_replica_shell.stl: a 38.1 mm ball cut flat at 15.55 mm from its centre (the 22 mm face), with a 25.4 mm
  (1 in) bore from the back for a 1 in chrome steel bearing ball, so the nest's magnet grips it and it weighs about
  150 g with the shell (PETG, 100 % infill round the bore).
- aim_face.stl: a 22 mm disc, 1.5 mm thick, glued on the flat face (print black), with raised rings at 2.7, 5.7 and
  9.0 mm radius: a laser pointer aimed at the ball centre lands inside the 10, 20 or 30 degree ring
  (15.55 mm x tan(angle)).
- start_gauge.stl: a 60 degree wedge, 40 mm long, laid against the face to reset the start elevation.

Run: python3 fabricate.py  (writes the STL files beside this script; importing it writes nothing).
"""

import math
import os

R_BALL = 19.05          # mm, 1.5 in SMR
FACE_Z = math.sqrt(R_BALL ** 2 - 11.0 ** 2)     # 15.55 mm: the face plane
CORE_R = 12.7           # 1 in steel bearing ball
RINGS = [15.55 * math.tan(math.radians(a)) for a in (10, 20, 30)]


def revolve(profile, n=64):
    """Triangles of a closed solid of revolution about z from a polyline [(r, z), ...] (r = 0 at both ends)."""
    tris = []
    for i in range(len(profile) - 1):
        (r0, z0), (r1, z1) = profile[i], profile[i + 1]
        for k in range(n):
            a0, a1 = 2 * math.pi * k / n, 2 * math.pi * (k + 1) / n
            p00 = (r0 * math.cos(a0), r0 * math.sin(a0), z0)
            p01 = (r0 * math.cos(a1), r0 * math.sin(a1), z0)
            p10 = (r1 * math.cos(a0), r1 * math.sin(a0), z1)
            p11 = (r1 * math.cos(a1), r1 * math.sin(a1), z1)
            if r0 > 0:
                tris.append((p00, p01, p11))
            if r1 > 0:
                tris.append((p00, p11, p10))
    return tris


def shell_profile():
    """Outer sphere from the bottom pole to the face rim, the face, then the core bore back down (blind, 1 in)."""
    out = []
    a_end = math.acos(-FACE_Z / R_BALL)
    for i in range(41):
        a = a_end * i / 40
        out.append((R_BALL * math.sin(a), -R_BALL * math.cos(a)))
    out.append((CORE_R + 0.1, FACE_Z))                       # face annulus in to the bore
    out.append((CORE_R + 0.1, -2.0))                         # bore wall down past the centre (core drops in from the face)
    out.append((0.0, -2.0))                                  # bore floor
    return out


def ring_disc():
    prof = [(0.0, 0.0), (11.0, 0.0), (11.0, 1.5)]
    for r in reversed(RINGS):
        prof += [(r + 0.3, 1.5), (r + 0.3, 2.1), (r - 0.3, 2.1), (r - 0.3, 1.5)]
    prof.append((0.0, 1.5))
    return prof


def wedge(length=40.0, width=20.0, angle=60.0):
    h = length * math.tan(math.radians(angle)) / 2
    v = [(0, 0, 0), (length, 0, 0), (0, 0, h), (0, width, 0), (length, width, 0), (0, width, h)]
    f = [(0, 2, 1), (3, 4, 5), (0, 1, 4), (0, 4, 3), (0, 3, 5), (0, 5, 2), (1, 2, 5), (1, 5, 4)]
    return [(v[a], v[b], v[c]) for a, b, c in f]


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


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    for name, tris in (("smr_replica_shell", revolve(shell_profile())), ("aim_face", revolve(ring_disc())),
                       ("start_gauge", wedge())):
        path = os.path.join(here, f"{name}.stl")
        write_stl(path, name, tris)
        print(f"wrote {path} ({len(tris)} facets)")


if __name__ == "__main__":
    main()
