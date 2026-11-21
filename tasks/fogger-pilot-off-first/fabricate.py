"""Print-ready STL files (ASCII, mm) of the custom parts of the fogger task's real trial, from the same dimensions as the
scene (fogger_pilot_off_first.py):

- door_plate.stl: the burner access door, 100 x 70 x 10 mm, with the 28 mm sight window the amber pilot LED shines
  through (the LED's 5 mm body fits the 5.2 mm hole in the backing tab);
- cam_flange.stl: the pilot-lamp cam, a 3 mm flange glued round the gas control knob's skirt 6 mm above the bezel
  (38.4 mm bore for the 38 mm knob, 21.5 mm rim radius, one lobe out to 30 mm over 20 deg); at OFF the lobe presses
  the lever of the Omron SS-5GL2 microswitch, opening its NC contact so the LED goes dark;
- switch_bracket.stl: an L bracket taped to the control's top on the knob's left, with two 2.4 mm slots that let the
  SS-5GL2 slide 8 mm toward the knob to set the trip point exactly at OFF;
- knob.stl and bezel_plate.stl: only for the cheaper mock-up route without a real White-Rodgers control: the 38 x 42 mm
  knob with its 6 x 6.5 x 6 mm interlock tooth on a 6 mm stem, and a 90 x 150 x 5 mm top plate carrying the 68 mm
  bezel and the interlock lip that blocks the tooth between PILOT and OFF unless the knob is pushed 4.5 mm.

Run as a script: python3 fabricate.py [out_dir]. Importing this module writes nothing.
"""

import math
import os
import sys

DOOR = (100.0, 70.0, 10.0)
WINDOW_R = 14.0
KNOB_R, KNOB_H = 19.0, 42.0
TOOTH = (19.0, 25.5, 6.0, 6.0)  # r_in, r_out, width, height (bottom of the knob skirt)
STEM_R, STEM_L = 3.0, 20.0
BEZEL = (27.0, 34.0, 19.0)  # r_in, r_out, height
LIP = (22.5, 27.0, 8.5, 12.0)  # r_in, r_out, height, ledge end (deg) past the tooth centre at PILOT and OFF
PLATE = (90.0, 150.0, 5.0)
CAM = (19.2, 21.5, 30.0, 3.0, 20.0)  # bore radius, rim radius, lobe radius, thickness, lobe width (deg)
BRACKET = (30.0, 20.0, 24.0, 3.0)  # foot length, width, upright height, thickness


def _facet(f, a, b, c):
    ux, uy, uz = (b[i] - a[i] for i in range(3))
    vx, vy, vz = (c[i] - a[i] for i in range(3))
    n = (uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx)
    m = math.sqrt(sum(x * x for x in n)) or 1.0
    f.write(f"facet normal {n[0] / m:.6f} {n[1] / m:.6f} {n[2] / m:.6f}\n outer loop\n")
    for p in (a, b, c):
        f.write(f"  vertex {p[0]:.4f} {p[1]:.4f} {p[2]:.4f}\n")
    f.write(" endloop\nendfacet\n")


def _quad(f, a, b, c, d):
    _facet(f, a, b, c)
    _facet(f, a, c, d)


def _ring_sector(f, r0, r1, z0, z1, a0, a1, n):
    """A closed ring sector (watertight): outer, inner, top, bottom and the two end faces."""
    angs = [a0 + (a1 - a0) * i / n for i in range(n + 1)]

    def p(r, a, z):
        return (r * math.cos(a), r * math.sin(a), z)

    for a, b in zip(angs[:-1], angs[1:]):
        _quad(f, p(r1, a, z0), p(r1, b, z0), p(r1, b, z1), p(r1, a, z1))
        _quad(f, p(r0, b, z0), p(r0, a, z0), p(r0, a, z1), p(r0, b, z1))
        _quad(f, p(r0, a, z1), p(r1, a, z1), p(r1, b, z1), p(r0, b, z1))
        _quad(f, p(r0, b, z0), p(r1, b, z0), p(r1, a, z0), p(r0, a, z0))
    if a1 - a0 < 2 * math.pi - 1e-9:
        _quad(f, p(r0, a0, z0), p(r0, a0, z1), p(r1, a0, z1), p(r1, a0, z0))
        _quad(f, p(r1, a1, z0), p(r1, a1, z1), p(r0, a1, z1), p(r0, a1, z0))


def _box(f, x0, x1, y0, y1, z0, z1):
    c = [(x, y, z) for z in (z0, z1) for y in (y0, y1) for x in (x0, x1)]
    for a, b, cc, d in ((0, 2, 3, 1), (4, 5, 7, 6), (0, 1, 5, 4), (2, 6, 7, 3), (0, 4, 6, 2), (1, 3, 7, 5)):
        _quad(f, c[a], c[b], c[cc], c[d])


def door_plate(f):
    """Plate with a square-cornered window approximated by four bars round a 28 mm opening (print flat), and a backing
    tab behind the window's lower edge with a 5.2 mm hole for the 5 mm LED."""
    w, h, t = DOOR
    s = WINDOW_R
    _box(f, -w / 2, w / 2, -h / 2, -s, 0, t)
    _box(f, -w / 2, w / 2, s, h / 2, 0, t)
    _box(f, -w / 2, -s, -s, s, 0, t)
    _box(f, s, w / 2, -s, s, 0, t)
    # LED tab: 20 x 12 x 3 mm below the window, its hole as four bars round a 5.2 mm square
    _box(f, -10, -2.6, -s - 12, -s, -3, 0)
    _box(f, 2.6, 10, -s - 12, -s, -3, 0)
    _box(f, -2.6, 2.6, -s - 12, -s - 8.6, -3, 0)
    _box(f, -2.6, 2.6, -s - 3.4, -s, -3, 0)


def cam_flange(f):
    r0, r1, rl, t, wdeg = CAM
    lobe = math.radians(wdeg)
    _ring_sector(f, r0, r1, 0.0, t, lobe / 2, 2 * math.pi - lobe / 2, 60)
    _ring_sector(f, r0, rl, 0.0, t, -lobe / 2, lobe / 2, 6)  # lobe: glued under the knob's pointer


def switch_bracket(f):
    a, w, h, t = BRACKET
    _box(f, 0, a, 0, w, 0, t)  # foot, taped to the control top
    # upright with two 2.4 x 8 mm slots (bars round them), the switch body bolts to its face
    _box(f, 0, t, 0, w, t, t + 4)
    _box(f, 0, t, 0, w, t + 6.4, t + 12)
    _box(f, 0, t, 0, w, t + 14.4, h)
    for y0, y1 in ((0, 3), (5.4, 14.6), (17, w)):
        _box(f, 0, t, y0, y1, t + 4, t + 6.4)
        _box(f, 0, t, y0, y1, t + 12, t + 14.4)


def knob(f):
    _ring_sector(f, 0.001, KNOB_R, 0.0, KNOB_H, 0.0, 2 * math.pi, 48)
    r0, r1, w, h = TOOTH
    _box(f, -w / 2, w / 2, r0 - 0.5, r1, 0.0, h)  # tooth at +y (its direction at ON)
    _ring_sector(f, 0.001, STEM_R, -STEM_L, 0.0, 0.0, 2 * math.pi, 16)


def bezel_plate(f):
    x, y, t = PLATE
    r0, r1, h = BEZEL
    # plate with a 54 mm square opening under the bezel (four bars), bezel ring standing on it
    _box(f, -x / 2, x / 2, -y / 2, -r0, 0, t)
    _box(f, -x / 2, x / 2, r0, y / 2, 0, t)
    _box(f, -x / 2, -r0, -r0, r0, 0, t)
    _box(f, r0, x / 2, -r0, r0, 0, t)
    _ring_sector(f, r0, r1, t, t + h, 0.0, 2 * math.pi, 48)
    li, lo, lh, gap = LIP
    # the ledge spans from the tooth's OFF position + gap to its PILOT position - gap (tooth at +90 deg at ON)
    a0 = math.radians(90.0 - 90.0 + gap)
    a1 = math.radians(90.0 - 45.0 - gap)
    _ring_sector(f, li, lo + 0.5, t + h - lh, t + h, a0, a1, 6)


PARTS = {"door_plate": door_plate, "cam_flange": cam_flange, "switch_bracket": switch_bracket, "knob": knob,
         "bezel_plate": bezel_plate}


def main(out_dir="."):
    os.makedirs(out_dir, exist_ok=True)
    for name, fn in PARTS.items():
        path = os.path.join(out_dir, name + ".stl")
        with open(path, "w") as f:
            f.write(f"solid {name}\n")
            fn(f)
            f.write(f"endsolid {name}\n")
        print("wrote", path)


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else ".")
