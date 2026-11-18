"""Print files for the two reset gauges of this task, from the same numbers as the scene.

Both objects in the task are store-bought (laptop and webcam); nothing that carries the mechanism is printed. These two
flat gauges only make the reset repeatable:

- lid_gauge.stl: a wedge that stands on the table against the lid back; its slope is the lid's 110 deg opening
  (20 deg past upright), 120 mm tall, so the lid is reset by laying it flat on the slope.
- foot_gauge.stl: a card with a 93 deg notch: the webcam's foot is swung open until its plate lies along the notch.

Run as a script to write the STL files (ASCII, millimetres) beside this file:  python3 fabricate.py
Standard library only; importing this module writes nothing.
"""

import math
import os

LID_OPEN_DEG = 110.0  # lid opening the task starts at (clip_a_webcam_onto_a_laptop_screen.OPEN_DEG)
FOOT_OPEN_DEG = 93.0  # foot opening at the start (clip_a_webcam_onto_a_laptop_screen.FOOT_START)
WEDGE_H, WEDGE_W, WEDGE_BASE = 120.0, 60.0, 80.0  # mm
CARD_T, CARD_ARM = 3.0, 60.0  # mm


def _tri(f, a, b, c):
    ux, uy, uz = (b[i] - a[i] for i in range(3))
    vx, vy, vz = (c[i] - a[i] for i in range(3))
    n = (uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx)
    m = math.sqrt(sum(v * v for v in n)) or 1.0
    f.write(f"  facet normal {n[0] / m:.6f} {n[1] / m:.6f} {n[2] / m:.6f}\n    outer loop\n")
    for p in (a, b, c):
        f.write(f"      vertex {p[0]:.4f} {p[1]:.4f} {p[2]:.4f}\n")
    f.write("    endloop\n  endfacet\n")


def prism(path, name, outline, depth):
    """Extrude a convex or simple 2-D outline [(x, z), ...] (counter-clockwise) along y by ``depth`` mm."""
    n = len(outline)
    lo = [(x, 0.0, z) for x, z in outline]
    hi = [(x, depth, z) for x, z in outline]
    with open(path, "w") as f:
        f.write(f"solid {name}\n")
        for i in range(1, n - 1):  # end caps (fan; outlines here are convex)
            _tri(f, lo[0], lo[i + 1], lo[i])
            _tri(f, hi[0], hi[i], hi[i + 1])
        for i in range(n):
            j = (i + 1) % n
            _tri(f, lo[i], lo[j], hi[j])
            _tri(f, lo[i], hi[j], hi[i])
        f.write(f"endsolid {name}\n")


def lid_wedge():
    """Right-angled wedge: base along x, its sloped face leaning (LID_OPEN_DEG - 90) deg past upright."""
    lean = math.radians(LID_OPEN_DEG - 90.0)
    top_x = WEDGE_BASE - WEDGE_H * math.tan(lean)
    return [(0.0, 0.0), (WEDGE_BASE, 0.0), (top_x, WEDGE_H), (0.0, WEDGE_H)]


def foot_card():
    """Card with two arms at FOOT_OPEN_DEG: one along the clip's lip line, one along the open foot."""
    a = math.radians(FOOT_OPEN_DEG)
    return [(0.0, 0.0), (CARD_ARM, 0.0), (CARD_ARM * math.cos(a) + 6.0, CARD_ARM * math.sin(a)),
            (CARD_ARM * math.cos(a), CARD_ARM * math.sin(a))]


if __name__ == "__main__":
    here = os.path.dirname(os.path.abspath(__file__))
    prism(os.path.join(here, "lid_gauge.stl"), "lid_gauge", lid_wedge(), WEDGE_W)
    prism(os.path.join(here, "foot_gauge.stl"), "foot_gauge", foot_card(), CARD_T)
    print("wrote lid_gauge.stl and foot_gauge.stl (mm)")
