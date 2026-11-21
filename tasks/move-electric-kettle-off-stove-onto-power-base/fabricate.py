"""Print-ready ASCII STL helper fixtures for the electric-kettle-off-stove task, in mm, from the task file's numbers.

Nothing carrying the task's physical mechanism is custom: the cordless electric kettle and 360-degree power base are a
commercial Hamilton Beach 40998 (1.0 L), and the countertop single burner is a commercial Proctor Silex 34107 (White).
This script writes two optional bench-setup helpers for safe, repeatable dry/unplugged real-world trials:

  dummy_outlet_block.stl   unpowered 50 x 80 x 56 mm duplex-outlet mounting block taped behind the power base so the
                           kettle base's NEMA 5-15P plug seats into a realistic rear receptacle with zero mains power
  layout_spacing_jig.stl   38.5 mm wide alignment spacer bar with side lip that sets the exact separation between the
                           Proctor Silex 34107 housing right edge (y = +6.0 mm) and the Hamilton Beach 40998 power-base
                           outer rim (y = -32.5 mm) when taping both appliances to the table

Run as a script (python3 fabricate.py [out_dir]); importing writes nothing. Standard library only.
"""

import os
import sys

# Dimensions in millimetres, matching move_electric_kettle_off_stove_onto_power_base.py
OUTLET_X, OUTLET_Y, OUTLET_Z = 50.0, 80.0, 56.0
WALL_T = 4.0
SLOT_DEPTH = 18.0  # accepts NEMA 5-15P plug blades (15.9 mm long) inside an unpowered blind pocket

GAP_Y = 38.5  # mm between BURNER housing (y=+6.0 mm) and POWER_BASE outer rim (y=-32.5 mm)
JIG_LEN_X = 120.0
JIG_H = 12.0
TAB_H = 16.0


def _box(x0, y0, z0, x1, y1, z1):
    """Twelve triangles of an axis-aligned box with outward normals."""
    v = [
        (x0, y0, z0),
        (x1, y0, z0),
        (x1, y1, z0),
        (x0, y1, z0),
        (x0, y0, z1),
        (x1, y0, z1),
        (x1, y1, z1),
        (x0, y1, z1),
    ]
    faces = [
        (0, 2, 1),
        (0, 3, 2),
        (4, 5, 6),
        (4, 6, 7),
        (0, 1, 5),
        (0, 5, 4),
        (1, 2, 6),
        (1, 6, 5),
        (2, 3, 7),
        (2, 7, 6),
        (3, 0, 4),
        (3, 4, 7),
    ]
    return [(v[a], v[b], v[c]) for a, b, c in faces]


def _write_stl(path, name, tris):
    with open(path, "w") as f:
        f.write(f"solid {name}\n")
        for a, b, c in tris:
            ux, uy, uz = (b[i] - a[i] for i in range(3))
            vx, vy, vz = (c[i] - a[i] for i in range(3))
            nx = uy * vz - uz * vy
            ny = uz * vx - ux * vz
            nz = ux * vy - uy * vx
            mag = (nx * nx + ny * ny + nz * nz) ** 0.5 or 1.0
            f.write(f"  facet normal {nx / mag:.6f} {ny / mag:.6f} {nz / mag:.6f}\n    outer loop\n")
            for p in (a, b, c):
                f.write(f"      vertex {p[0]:.3f} {p[1]:.3f} {p[2]:.3f}\n")
            f.write("    endloop\n  endfacet\n")
        f.write(f"endsolid {name}\n")


def _dummy_outlet_block():
    """Hollow-front duplex outlet housing (50 x 80 x 56 mm) with a recessed blade pocket for an unpowered NEMA plug."""
    tris = []
    # Solid rear core (x = SLOT_DEPTH..OUTLET_X)
    tris += _box(SLOT_DEPTH, 0.0, 0.0, OUTLET_X, OUTLET_Y, OUTLET_Z)
    # Front surround walls framing a 30 x 36 mm plug-blade recess on the -y duplex half (y = 8..38, z = 10..46)
    tris += _box(0.0, 0.0, 0.0, SLOT_DEPTH, 8.0, OUTLET_Z)
    tris += _box(0.0, 38.0, 0.0, SLOT_DEPTH, OUTLET_Y, OUTLET_Z)
    tris += _box(0.0, 8.0, 0.0, SLOT_DEPTH, 38.0, 10.0)
    tris += _box(0.0, 8.0, 46.0, SLOT_DEPTH, 38.0, OUTLET_Z)
    return tris


def _layout_spacing_jig():
    """38.5 mm wide spacing bar with an upright pull tab to set burner-to-power-base separation before taping down."""
    tris = _box(0.0, 0.0, 0.0, JIG_LEN_X, GAP_Y, JIG_H)
    tris += _box(JIG_LEN_X / 2 - 12.0, GAP_Y / 2 - 4.0, JIG_H, JIG_LEN_X / 2 + 12.0, GAP_Y / 2 + 4.0, JIG_H + TAB_H)
    return tris


def main(out_dir="."):
    os.makedirs(out_dir, exist_ok=True)
    _write_stl(os.path.join(out_dir, "dummy_outlet_block.stl"), "dummy_outlet_block", _dummy_outlet_block())
    _write_stl(os.path.join(out_dir, "layout_spacing_jig.stl"), "layout_spacing_jig", _layout_spacing_jig())
    print(f"wrote dummy_outlet_block.stl and layout_spacing_jig.stl to {os.path.abspath(out_dir)}")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else ".")
