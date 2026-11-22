"""Print-ready and cut-ready STL files (ASCII, mm) of the custom parts of the toe-kick vacuum task's real trial, from
the same dimensions as the scene (vacuum_toe_kick_swivel_brush.py). The brush tool, vacuum and cabinet are store
bought; these parts only rebuild the cabinet's toe kick for a bench trial and put the lentils back on their marks.

- lentil_template.stl: a 3 mm plate, 260 x 120 mm, that butts against the kick board face and the cabinet's right
  end; ten 8 mm holes mark the lentil centres (drop one lentil in each, then lift the plate straight up).
- kick_board.stl: the 4 in (101.6 mm) x 229 mm toe kick, 19 mm (3/4 in) plywood or a 1x4 pine board.
- deck.stl: the cabinet bottom over the recess, 229 x 102 x 19 mm plywood, standing on the kick board and two
  19 mm spacer blocks so it overhangs the board by 76 mm (the standard 3 in toe-kick depth).
- spacer.stl: one of the two 19 x 26 x 101.6 mm blocks behind the kick board at its ends.
- door_skirt.stl: 229 x 78.4 x 19 mm white board standing on the deck flush with its front (the door bottom; the
  fixture is 180 mm tall like the scene's cut-off cabinet).

Run as a script: python3 fabricate.py [out_dir]. Importing this module writes nothing.
"""

import os
import sys

CAB_W = 229.0  # 9 in base cabinet width
TOE_H, TOE_D = 101.6, 76.0
BOARD_T = 19.0
DECK_D = TOE_D + BOARD_T + 7.0  # 102: door face to the deck's back edge
CAB_TOP = 180.0
TEMPLATE = (260.0, 120.0, 3.0)
HOLE_R = 4.0
# lentil centres from the scene (m), measured here from the cabinet's right end (x 0.155) and the kick board face
# (y 0.236): template x = (x - 0.155) * 1000, template y = (0.236 - y) * 1000 (toward the room)
LENTILS_M = [(0.185, 0.150), (0.245, 0.150), (0.305, 0.150), (0.200, 0.195), (0.260, 0.195), (0.320, 0.195),
               (0.180, 0.225), (0.235, 0.225), (0.290, 0.225), (0.376, 0.225)]
HOLES = [((x - 0.155) * 1000.0, (0.236 - y) * 1000.0) for x, y in LENTILS_M]


def _facet(f, a, b, c):
    ux, uy, uz = (b[i] - a[i] for i in range(3))
    vx, vy, vz = (c[i] - a[i] for i in range(3))
    n = (uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx)
    m = (n[0] ** 2 + n[1] ** 2 + n[2] ** 2) ** 0.5 or 1.0
    f.write(f"facet normal {n[0] / m:.6f} {n[1] / m:.6f} {n[2] / m:.6f}\n outer loop\n")
    for p in (a, b, c):
        f.write(f"  vertex {p[0]:.4f} {p[1]:.4f} {p[2]:.4f}\n")
    f.write(" endloop\nendfacet\n")


def _box(f, x0, y0, z0, x1, y1, z1):
    """A closed box shell."""
    p = [(x, y, z) for z in (z0, z1) for y in (y0, y1) for x in (x0, x1)]
    for a, b, c, d in ((0, 2, 3, 1), (4, 5, 7, 6), (0, 1, 5, 4), (2, 6, 7, 3), (0, 4, 6, 2), (1, 3, 7, 5)):
        _facet(f, p[a], p[b], p[c])
        _facet(f, p[a], p[c], p[d])


def _plate_with_holes(f, w, d, t, holes, r, cell=2.0):
    """The plate as a grid of cells (a box union) with every cell inside a hole left out; watertight per cell."""
    nx, ny = int(round(w / cell)), int(round(d / cell))
    for i in range(nx):
        for j in range(ny):
            cx, cy = (i + 0.5) * cell, (j + 0.5) * cell
            if any((cx - hx) ** 2 + (cy - hy) ** 2 <= r * r for hx, hy in holes):
                continue
            _box(f, i * cell, j * cell, 0.0, (i + 1) * cell, (j + 1) * cell, t)


def write_all(out_dir):
    os.makedirs(out_dir, exist_ok=True)
    parts = {
        "kick_board": lambda f: _box(f, 0, 0, 0, CAB_W, BOARD_T, TOE_H),
        "deck": lambda f: _box(f, 0, 0, 0, CAB_W, DECK_D, BOARD_T),
        "spacer": lambda f: _box(f, 0, 0, 0, BOARD_T, 26.0, TOE_H),
        "door_skirt": lambda f: _box(f, 0, 0, 0, CAB_W, BOARD_T, CAB_TOP - TOE_H),
        "lentil_template": lambda f: _plate_with_holes(f, *TEMPLATE, HOLES, HOLE_R),
    }
    for name, build in parts.items():
        path = os.path.join(out_dir, name + ".stl")
        with open(path, "w") as f:
            f.write(f"solid {name}\n")
            build(f)
            f.write(f"endsolid {name}\n")
        print("wrote", path)


if __name__ == "__main__":
    write_all(sys.argv[1] if len(sys.argv) > 1 else "stl")
