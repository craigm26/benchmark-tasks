"""Print files for the custom parts of the real-trial rig, from the same dimensions as the simulated scene (mm).

    python3 fabricate.py [out_dir]

writes ASCII STL files (standard library only; nothing is written on import):

- shoulder_dome.stl: the cylinder shoulder shell, 133 mm across at the bench and 48 mm at the neck, 66 mm tall, 3 mm
  wall, open at the top for the valve and at the bottom over the floor flange. Print in PETG, paint dark green with a
  white band 14 to 22 mm up.
- bench_plate.stl: the 160 x 160 x 16 mm plate the rig stands on, with four 6.5 mm holes for the floor flange's
  M6 bolts on a 70 mm square and four 5 mm corner holes for clamping to the bench.
- witness_gauge.stl: a 6.4 mm thick U-shaped feeler (30 x 24 mm, 10.5 mm slot) for scoring: slid over the exposed
  3/8 in stem against the locknut, it must fit under the T-handle hub once the screw is 6.4 mm out.

The cylinder valve, regulator, flange, nipple and mannequin head are bought parts (see the README parts list).
"""

import math
import os
import sys

DOME = [(66.5, 0.0), (64.0, 16.0), (57.0, 32.0), (46.0, 46.0), (34.0, 58.0), (24.0, 66.0)]  # (r, z) mm, outside
WALL = 3.0
SIDES = 96


def _facet(f, a, b, c):
    ux, uy, uz = (b[i] - a[i] for i in range(3))
    vx, vy, vz = (c[i] - a[i] for i in range(3))
    n = (uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx)
    m = math.sqrt(sum(v * v for v in n)) or 1.0
    f.write(f"  facet normal {n[0] / m:.6f} {n[1] / m:.6f} {n[2] / m:.6f}\n    outer loop\n")
    for p in (a, b, c):
        f.write(f"      vertex {p[0]:.4f} {p[1]:.4f} {p[2]:.4f}\n")
    f.write("    endloop\n  endfacet\n")


def _write(path, name, tris):
    with open(path, "w") as f:
        f.write(f"solid {name}\n")
        for t in tris:
            _facet(f, *t)
        f.write(f"endsolid {name}\n")


def revolve(loop, sides=SIDES):
    """Triangles of a closed (r, z) loop (counter-clockwise in the r-z plane) turned about z."""
    tris = []
    for k in range(sides):
        a0, a1 = 2 * math.pi * k / sides, 2 * math.pi * (k + 1) / sides
        for i in range(len(loop)):
            (r0, z0), (r1, z1) = loop[i], loop[(i + 1) % len(loop)]
            p00 = (r0 * math.cos(a0), r0 * math.sin(a0), z0)
            p01 = (r0 * math.cos(a1), r0 * math.sin(a1), z0)
            p10 = (r1 * math.cos(a0), r1 * math.sin(a0), z1)
            p11 = (r1 * math.cos(a1), r1 * math.sin(a1), z1)
            tris += [(p00, p10, p11), (p00, p11, p01)]
    return tris


def dome_loop():
    """The shell's section: outside profile up, inside profile (offset WALL inward) down."""
    inner = [(r - WALL, z) for r, z in DOME]
    inner[-1] = (DOME[-1][0] - WALL, DOME[-1][1])
    return [(r, z) for r, z in DOME] + list(reversed(inner))


def box_tris(x0, y0, z0, x1, y1, z1):
    c = [(x, y, z) for z in (z0, z1) for y in (y0, y1) for x in (x0, x1)]
    q = [(0, 2, 3, 1), (4, 5, 7, 6), (0, 1, 5, 4), (2, 6, 7, 3), (0, 4, 6, 2), (1, 3, 7, 5)]
    out = []
    for a, b, cc, d in q:
        out += [(c[a], c[b], c[cc]), (c[a], c[cc], c[d])]
    return out


def plate_with_holes(size=160.0, thick=16.0, holes=None, sides=24):
    """A square plate with round through holes, built as a grid of solid cells around polygonal hole tubes."""
    holes = holes or []
    tris = []
    step = 2.0
    n = int(size / step)
    h = size / 2

    def inside_hole(x, y):
        return any(math.hypot(x - hx, y - hy) < r + step * 0.75 for hx, hy, r in holes)

    filled = [[not inside_hole(-h + (i + 0.5) * step, -h + (j + 0.5) * step) for j in range(n)] for i in range(n)]
    for i in range(n):
        for j in range(n):
            if not filled[i][j]:
                continue
            x0, y0 = -h + i * step, -h + j * step
            x1, y1 = x0 + step, y0 + step
            faces = box_tris(x0, y0, 0.0, x1, y1, thick)
            keep = []
            for k in range(0, 12, 2):  # drop faces shared with a filled neighbour (6 quads, 2 triangles each)
                quad = k // 2
                nb = {2: (i, j - 1), 3: (i, j + 1), 4: (i - 1, j), 5: (i + 1, j)}.get(quad)
                if nb and 0 <= nb[0] < n and 0 <= nb[1] < n and filled[nb[0]][nb[1]]:
                    continue
                keep += faces[k:k + 2]
            tris += keep
    return tris


def witness_gauge():
    """U-shaped feeler, 6.4 mm thick: 30 x 24 mm with a 10.5 mm slot 16 mm deep that straddles the 3/8 in stem."""
    t = 6.4
    return (box_tris(0.0, 0.0, 0.0, 30.0, 6.75, t) + box_tris(0.0, 17.25, 0.0, 30.0, 24.0, t)
            + box_tris(16.0, 6.75, 0.0, 30.0, 17.25, t))


def main(out="."):
    os.makedirs(out, exist_ok=True)
    _write(os.path.join(out, "shoulder_dome.stl"), "shoulder_dome", revolve(dome_loop()))
    bolt = [(35.0 * sx, 35.0 * sy, 3.25) for sx in (-1, 1) for sy in (-1, 1)]
    clamp = [(70.0 * sx, 70.0 * sy, 2.5) for sx in (-1, 1) for sy in (-1, 1)]
    _write(os.path.join(out, "bench_plate.stl"), "bench_plate", plate_with_holes(160.0, 16.0, bolt + clamp))
    _write(os.path.join(out, "witness_gauge.stl"), "witness_gauge", witness_gauge())
    print(f"wrote shoulder_dome.stl, bench_plate.stl, witness_gauge.stl to {os.path.abspath(out)}")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else ".")
