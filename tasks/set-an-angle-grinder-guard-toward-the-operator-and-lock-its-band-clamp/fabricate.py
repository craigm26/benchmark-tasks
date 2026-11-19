"""Print file for the one custom part of this task: the guard-angle gauge used to score a real trial.

Every part that carries the mechanism is a stock part (Makita XAG03 grinder, Makita 123144-0 tool-less guard with its
band and clamp lever, any Type 27 115 x 6 x 22.23 mm wheel), so nothing here is needed to run a trial. The gauge only
makes the score quick to read: after the run, set it on the clamp nut (its cup fits over the 38 mm nut), turn its tall
fin to point along the motor housing, and read where the guard's orange tape falls against the 20 and 60 degree fins.
It sits inside the skirt (55 mm radius against the skirt's 62 mm) and clear of the wheel (cup floor 2 mm above it).

    python3 fabricate.py [out_dir]      writes q_gauge.stl (ASCII, millimetres) into out_dir (default ./stl)

Standard library only; importing this file writes nothing. Dimensions match the scene in
set_an_angle_grinder_guard_toward_the_operator_and_lock_its_band_clamp.py (nut radius 19 mm, 8 mm tall; wheel top
83 mm and skirt inner radius 62 mm above the table with the grinder upside down).
"""

import math
import os
import sys

# Gauge dimensions (mm).
NUT_R, NUT_H = 19.0, 8.0          # clamp nut it sits on
CUP_RIN, CUP_ROUT, CUP_DEPTH = 19.5, 23.0, 6.0
DISC_R, DISC_T = 55.0, 3.0         # under the skirt's 62 mm inner radius
FIN_R0, FIN_R1, FIN_W, FIN_H = 40.0, 54.0, 2.0, 6.0
POINTER_H = 14.0                   # the 0 fin, sighted along the motor housing
MARKS_DEG = (0.0, 20.0, -20.0, 60.0, -60.0)
SIDES = 96


def _ring(r, z, n=SIDES):
    return [(r * math.cos(2 * math.pi * k / n), r * math.sin(2 * math.pi * k / n), z) for k in range(n)]


def prism_shell(outline_bottom, outline_top):
    """A closed prism from two matching convex outlines (counter-clockwise from above). Returns triangles."""
    n = len(outline_bottom)
    tris = []
    cb = tuple(sum(p[i] for p in outline_bottom) / n for i in range(3))
    ct = tuple(sum(p[i] for p in outline_top) / n for i in range(3))
    for k in range(n):
        a, b = outline_bottom[k], outline_bottom[(k + 1) % n]
        c, d = outline_top[k], outline_top[(k + 1) % n]
        tris += [(cb, b, a), (ct, c, d), (a, b, d), (a, d, c)]
    return tris


def tube_shell(r_in, r_out, z0, z1, n=SIDES):
    """A closed annular prism (a cup wall)."""
    ib, ob, it, ot = _ring(r_in, z0, n), _ring(r_out, z0, n), _ring(r_in, z1, n), _ring(r_out, z1, n)
    tris = []
    for k in range(n):
        j = (k + 1) % n
        tris += [(ob[k], ob[j], ot[j]), (ob[k], ot[j], ot[k])]          # outer wall
        tris += [(ib[k], it[j], ib[j]), (ib[k], it[k], it[j])]          # inner wall
        tris += [(it[k], ot[k], ot[j]), (it[k], ot[j], it[j])]          # top
        tris += [(ib[k], ob[j], ob[k]), (ib[k], ib[j], ob[j])]          # bottom
    return tris


def fin_shell(angle_deg, r0, r1, w, z0, z1):
    a = math.radians(angle_deg)
    u, v = (math.cos(a), math.sin(a)), (-math.sin(a), math.cos(a))
    corners = [(r0, -w / 2), (r1, -w / 2), (r1, w / 2), (r0, w / 2)]
    pts = [(p * u[0] + q * v[0], p * u[1] + q * v[1]) for p, q in corners]
    return prism_shell([(x, y, z0) for x, y in pts], [(x, y, z1) for x, y in pts])


def gauge_shells():
    """The gauge as closed shells, z = 0 at the cup's open rim (it hangs over the nut from there)."""
    disc_z0 = CUP_DEPTH
    shells = [
        tube_shell(CUP_RIN, CUP_ROUT, 0.0, CUP_DEPTH + 0.5),                                  # cup over the nut
        prism_shell(_ring(DISC_R, disc_z0), _ring(DISC_R, disc_z0 + DISC_T)),                  # disc (cup floor)
    ]
    for m in MARKS_DEG:
        h = POINTER_H if m == 0.0 else FIN_H
        shells.append(fin_shell(m, FIN_R0, FIN_R1, FIN_W, disc_z0 + DISC_T - 0.5, disc_z0 + DISC_T + h))
    return shells


def check_closed(tris):
    """Every directed edge appears once and its reverse once: a closed, consistently wound shell."""
    edges = {}
    for t in tris:
        for i in range(3):
            e = (tuple(round(c, 6) for c in t[i]), tuple(round(c, 6) for c in t[(i + 1) % 3]))
            edges[e] = edges.get(e, 0) + 1
    return all(n == 1 and edges.get((b, a)) == 1 for (a, b), n in edges.items())


def signed_volume(tris):
    return sum((a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (b[0] * c[2] - b[2] * c[0])
                + a[2] * (b[0] * c[1] - b[1] * c[0])) for a, b, c in tris) / 6.0


def write_stl(path, name, shells):
    with open(path, "w") as f:
        f.write(f"solid {name}\n")
        for tris in shells:
            for a, b, c in tris:
                ux, uy, uz = (b[i] - a[i] for i in range(3))
                vx, vy, vz = (c[i] - a[i] for i in range(3))
                nx, ny, nz = uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx
                ln = math.sqrt(nx * nx + ny * ny + nz * nz) or 1.0
                f.write(f"  facet normal {nx / ln:.6e} {ny / ln:.6e} {nz / ln:.6e}\n    outer loop\n")
                for p in (a, b, c):
                    f.write(f"      vertex {p[0]:.4f} {p[1]:.4f} {p[2]:.4f}\n")
                f.write("    endloop\n  endfacet\n")
        f.write(f"endsolid {name}\n")


def main(out_dir="stl"):
    os.makedirs(out_dir, exist_ok=True)
    shells = gauge_shells()
    for s in shells:
        if not check_closed(s) or signed_volume(s) <= 0:
            raise SystemExit("q_gauge: a shell is open or inside out")
    path = os.path.join(out_dir, "q_gauge.stl")
    write_stl(path, "q_gauge", shells)
    print(f"wrote {path}: {len(shells)} closed shells (slice with union of overlapping volumes), "
          f"{2 * DISC_R:.0f} mm across, {CUP_DEPTH + DISC_T + POINTER_H:.0f} mm tall")
    return path


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "stl")
