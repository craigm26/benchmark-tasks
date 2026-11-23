"""Print or cut files for the only custom parts of this task: the plywood backer the box is screwed to, its foot, and a
corner gusset. Everything that carries the task's mechanism (box, in-use cover, receptacle, plug and cord) is a stock
commercial part; see README.md. Run as a script to write ASCII STL files (mm) beside this file:

    python3 fabricate.py

The sizes are the scene's: backer 300 x 300 x 19 mm (3/4 in plywood), foot 300 x 150 x 19 mm, gusset 100 x 100 x
19 mm right triangle. Cut them from plywood or print them; the STL is the cutting template.
"""

import os

BACKER = (19.0, 300.0, 300.0)  # x (thickness), y (width), z (height), mm
FOOT = (150.0, 300.0, 19.0)
GUSSET = 100.0
T = 19.0


def _facet(f, a, b, c):
    ux, uy, uz = (b[i] - a[i] for i in range(3))
    vx, vy, vz = (c[i] - a[i] for i in range(3))
    n = (uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx)
    m = sum(x * x for x in n) ** 0.5 or 1.0
    f.write(f"  facet normal {n[0] / m:.6f} {n[1] / m:.6f} {n[2] / m:.6f}\n    outer loop\n")
    for p in (a, b, c):
        f.write(f"      vertex {p[0]:.3f} {p[1]:.3f} {p[2]:.3f}\n")
    f.write("    endloop\n  endfacet\n")


def _prism(name, outline, depth):
    """Watertight prism: a convex (x, z) outline, counter-clockwise, extruded along y by depth."""
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), f"{name}.stl")
    lo = [(x, 0.0, z) for x, z in outline]
    hi = [(x, depth, z) for x, z in outline]
    n = len(outline)
    with open(path, "w") as f:
        f.write(f"solid {name}\n")
        for i in range(1, n - 1):
            _facet(f, lo[0], lo[i], lo[i + 1])
            _facet(f, hi[0], hi[i + 1], hi[i])
        for i in range(n):
            j = (i + 1) % n
            _facet(f, lo[i], hi[i], hi[j])
            _facet(f, lo[i], hi[j], lo[j])
        f.write(f"endsolid {name}\n")
    return path


def main():
    bx, by, bz = BACKER
    fx, fy, fz = FOOT
    out = [
        _prism("backer", [(0, 0), (bx, 0), (bx, bz), (0, bz)], by),
        _prism("foot", [(0, 0), (fx, 0), (fx, fz), (0, fz)], fy),
        _prism("gusset", [(0, 0), (GUSSET, 0), (0, GUSSET)], T),
    ]
    for p in out:
        print("wrote", os.path.basename(p))


if __name__ == "__main__":
    main()
