"""Print files for the custom parts of the real-trial rig, from the same dimensions as the simulated scene (mm).

    python3 fabricate.py [out_dir]

writes ASCII STL files (standard library only; nothing is written on import):

- shoulder_dome.stl: the cylinder shoulder shell, 133 mm across at the bench and 48 mm at the neck, 66 mm tall, 3 mm
  wall, open at the top for the valve and at the bottom over the floor flange. Print in PETG, paint dark green with a
  white band 14 to 22 mm up.
- bench_plate.stl: the 160 x 160 x 16 mm plate the rig stands on, with four 6.5 mm holes for the floor flange's
  M6 bolts on a 70 mm square and four 5 mm corner holes for clamping to the bench.
- degree_ring.stl: the scoring ring under the handwheel, 90 mm across with a 34 mm bore, 3 mm thick, with raised
  ticks every 15 degrees from 0 to 150 (the 0, 15 and 150 degree ticks run the full ring width). It lies on the shoulder
  dome round the valve body (put it on before the valve is threaded in), 0 degrees under the handwheel's white stripe
  when the valve is closed. Print in black PETG and paint the tick tops white; the trial video reads the stripe against
  it to time the opening.

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


RING_R = (17.0, 45.0)  # mm, bore and outside radius (scene RING_R)
RING_T = 3.0


def _turn(tris, a):
    c, s = math.cos(a), math.sin(a)
    return [tuple((p[0] * c - p[1] * s, p[0] * s + p[1] * c, p[2]) for p in t) for t in tris]


def degree_ring():
    """The flat ring (a revolved rectangle) plus 0.6 mm raised ticks, 1.6 mm wide, every 15 deg from 0 to 150."""
    tris = revolve([(RING_R[0], 0.0), (RING_R[1], 0.0), (RING_R[1], RING_T), (RING_R[0], RING_T)])
    for k in range(11):
        r0 = 33.0 if k in (0, 1, 10) else 38.0
        tick = box_tris(r0, -0.8, RING_T - 0.01, RING_R[1] - 0.3, 0.8, RING_T + 0.6)
        tris += _turn(tick, math.radians(15.0 * k))
    return tris


def main(out="."):
    os.makedirs(out, exist_ok=True)
    _write(os.path.join(out, "shoulder_dome.stl"), "shoulder_dome", revolve(dome_loop()))
    bolt = [(35.0 * sx, 35.0 * sy, 3.25) for sx in (-1, 1) for sy in (-1, 1)]
    clamp = [(70.0 * sx, 70.0 * sy, 2.5) for sx in (-1, 1) for sy in (-1, 1)]
    _write(os.path.join(out, "bench_plate.stl"), "bench_plate", plate_with_holes(160.0, 16.0, bolt + clamp))
    _write(os.path.join(out, "degree_ring.stl"), "degree_ring", degree_ring())
    print(f"wrote shoulder_dome.stl, bench_plate.stl, degree_ring.stl to {os.path.abspath(out)}")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else ".")
