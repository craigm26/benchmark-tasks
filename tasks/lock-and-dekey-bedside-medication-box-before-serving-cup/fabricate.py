"""Print or cut files for the two custom parts that carry the lock's action, from the scene's own dimensions.

    python3 fabricate.py [out_dir]

writes ASCII STL files (millimetres) only when run as a script:

- lid_catch.stl: the L catch screwed under the lid's front edge. Stem 10 x 5 x 34.5 mm down from the lid, foot
  10 x 20.5 x 4 mm running toward the lock wall; the locked cam lies 3 mm over the foot. Print in PETG (100 % infill)
  or bend from 2 mm steel strip to the same outline; two M3 x 6 screws into the lid (or VHB tape 4910).
- cam_tongue.stl: a 7 x 6.5 mm cam, 10 mm long beyond the lock's 18 mm plug, its 10 mm double-D hole for the lock's cam post
  drilled to suit the lock. Use it if the cam supplied with the lock is longer than 18.5 mm (it would strike the lid) or shorter than
  14 mm (it would miss the catch). Cut from 3 mm steel or print in PETG.

Everything else is bought (see the README parts list).
"""

import os
import sys

MM = 1000.0

# Same numbers as the task file (metres there, millimetres here).
CATCH_X = (-22.5, -12.5)  # across the box
STEM_Y = (22.0, 27.0)  # from the box centre toward the lock wall
FOOT_Y = (22.0, 42.5)
FOOT_Z = (49.5, 53.5)  # above the box floor's underside
LID_Z = 84.0

CAM_W, CAM_T, CAM_R0, CAM_R1 = 7.0, 6.5, 8.5, 18.5


def _box_tris(x0, x1, y0, y1, z0, z1):
    v = [(x, y, z) for x in (x0, x1) for y in (y0, y1) for z in (z0, z1)]
    faces = [(0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)]
    tris = []
    for a, b, c, d in faces:
        tris += [(v[a], v[b], v[c]), (v[a], v[c], v[d])]
    return tris


def _write_stl(path, name, tris):
    def normal(t):
        (ax, ay, az), (bx, by, bz), (cx, cy, cz) = t
        ux, uy, uz, wx, wy, wz = bx - ax, by - ay, bz - az, cx - ax, cy - ay, cz - az
        n = (uy * wz - uz * wy, uz * wx - ux * wz, ux * wy - uy * wx)
        m = sum(c * c for c in n) ** 0.5 or 1.0
        return tuple(c / m for c in n)

    with open(path, "w") as f:
        f.write(f"solid {name}\n")
        for t in tris:
            n = normal(t)
            f.write(f"  facet normal {n[0]:.6f} {n[1]:.6f} {n[2]:.6f}\n    outer loop\n")
            for p in t:
                f.write(f"      vertex {p[0]:.4f} {p[1]:.4f} {p[2]:.4f}\n")
            f.write("    endloop\n  endfacet\n")
        f.write(f"endsolid {name}\n")


def lid_catch():
    """Stem and foot as one L: two overlapping closed blocks, which slicers and CAM merge into one solid."""
    stem = _box_tris(*CATCH_X, *STEM_Y, FOOT_Z[0], LID_Z)
    foot = _box_tris(*CATCH_X, *FOOT_Y, *FOOT_Z)
    return stem + foot


def cam_tongue():
    """The tongue as a plate in the x-z plane: hub 18 mm square with the post hole left to drill (10 mm, flats 8 mm),
    arm out to 18.5 mm from the lock axis."""
    hub = _box_tris(-9.0, 9.0, 0.0, CAM_T, -9.0, 9.0)
    arm = _box_tris(-CAM_W / 2, CAM_W / 2, 0.0, CAM_T, 9.0, CAM_R1)
    return hub + arm


def main(out_dir):
    os.makedirs(out_dir, exist_ok=True)
    for name, tris in (("lid_catch", lid_catch()), ("cam_tongue", cam_tongue())):
        path = os.path.join(out_dir, name + ".stl")
        _write_stl(path, name, tris)
        print(f"wrote {path} ({len(tris)} triangles)")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "stl")
