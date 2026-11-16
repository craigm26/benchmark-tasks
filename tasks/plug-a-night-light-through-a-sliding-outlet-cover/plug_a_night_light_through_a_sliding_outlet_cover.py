"""A plug-in night light and a duplex outlet behind a universal self-closing sliding cover.

On the far side of the table stands a pale blue wall panel with a white duplex cover plate. Behind the plate's top
recess a sprung sliding door (4 mm thick) covers both receptacle openings: at rest its solid face is over them and its
own two slots lie over solid backing. The only way in is the way the cover's sheet says: put the blade tips into the
door's slots, slide the door 16 mm across to its stop, then push the plug in. A white night light (45 g, NEMA 1-15
blades) stands on a small black rubber mat on the table, in front of
the right arm and inboard of it.

Requires -E object_jitter=0. Run it:

  inspect-robots run --embodiment yam_arena --policy yam_scripted \\
      -E task=plug_a_night_light_through_a_sliding_outlet_cover.py -E object_jitter=0 -E control_hz=20 \\
      --instruction "Plug the night light into the top outlet through the sliding childproof cover, push it fully in,
      then let go." --scorer task_score ...
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import dataclasses

import numpy as np

import belimits  # noqa: F401  shared limits kit (this scorer reads Mech states directly)
import mechkit
from closedloop import grip_rot, placeholder_demo
from mechkit import Joint, Mech, box, cyl, hull, lamp, on_reset, on_step, set_lamp
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 8.0  # the episode runs on 8 s after success, so the video shows the release and the parked arm

WALL_BLUE, WHITE, RIM, BACKING = (0.62, 0.70, 0.80), (0.95, 0.95, 0.93), (0.30, 0.30, 0.32), (0.22, 0.22, 0.24)
DOOR_GREY, SLOT_BLACK, SCREW = (0.78, 0.80, 0.83), (0.10, 0.10, 0.10), (0.70, 0.70, 0.68)
HOUSING, AMBER, GLOW, NICKEL, CELL = (0.96, 0.96, 0.94), (0.98, 0.70, 0.25), (1.0, 0.93, 0.55), (0.78, 0.78, 0.75), (0.35, 0.35, 0.38)


def _look(shape):
    """A look-only copy of a shape (no collider)."""
    return dataclasses.replace(shape, collide=False)


def _holed(x, y, z, holes, color, contact=None):
    """Boxes filling the block x0..x1, y0..y1, z0..z1 (outlet frame) except rectangular through holes along x, all in
    one z band: holes = [(y_lo, y_hi), ...] sorted, sharing (z_lo, z_hi) = holes' band given as z[2:]."""
    (x0, x1), (y0, y1), (z0, z1, hz0, hz1) = x, y, z
    xc, dx = (x0 + x1) / 2, x1 - x0
    out = [box((xc, (y0 + y1) / 2, (z0 + hz0) / 2), (dx, y1 - y0, hz0 - z0), color, contact=contact),
           box((xc, (y0 + y1) / 2, (hz1 + z1) / 2), (dx, y1 - y0, z1 - hz1), color, contact=contact)]
    edges = [y0] + [v for h in holes for v in h] + [y1]
    for a, b in zip(edges[0::2], edges[1::2]):
        if b - a > 1e-6:
            out.append(box((xc, (a + b) / 2, (hz0 + hz1) / 2), (dx, b - a, hz1 - hz0), color, contact=contact))
    return out


def _rounded_yz(x0, x1, y0, y1, z0, z1, r, round_top, round_bottom):
    """Corner points of a block in the y-z plane with 4 mm (r) rounded corners on the top and/or bottom edge, as both
    x faces, for hull()."""
    pts2 = []
    for (yc, zc, sy, sz, rnd) in ((y0, z0, 1, 1, round_bottom), (y1, z0, -1, 1, round_bottom),
                                  (y1, z1, -1, -1, round_top), (y0, z1, 1, -1, round_top)):
        if rnd:
            cy, cz = yc + sy * r, zc + sz * r
            for k in range(5):
                a = k * (math.pi / 2) / 4
                pts2.append((cy - sy * r * math.cos(a), cz - sz * r * math.sin(a)))
        else:
            pts2.append((yc, zc))
    return [(x, py, pz) for x in (x0, x1) for (py, pz) in pts2]


# == the outlet: wall panel, cover plate, backing, receptacle pockets and the sprung door ===========================
# Outlet frame: origin at (WALL_X, 0, TOP), axes as the station's. All numbers below are station x, y and z - TOP.
WALL_X = 0.345          # wall face, and the back of the cover plate
PLATE_X = 0.337         # cover plate front face (d is measured from here)
DOOR_X = (0.338, 0.342)  # the door slab
BACK_X = (0.342, 0.345)  # backing, with the receptacle openings
POCKET_END = 0.362      # blind end of the receptacle pockets
ZB = 0.100              # top outlet's slot line above the table (Z = TOP + ZB)
PIN_Y = 0.00635         # the blades and pockets: y = +-6.35 mm (12.7 mm centres)
OPEN_W, OPEN_H = 0.0045, 0.0093   # receptacle opening, 1.5 mm a side round a 1.5 x 6.3 mm blade
LEAD = 0.0015           # opening lead-in, 1.5 mm all round
PLATE_W, PLATE_H, PLATE_ZC = 0.070, 0.115, 0.081
REC_YC, REC_W, REC_H = 0.0015, 0.060, 0.030  # door recess (both outlets)
LOWER_ZB = ZB - 0.0381  # lower outlet's slot line (look only)
DOOR_W, DOOR_H = 0.042, 0.024
SLOT_W, SLOT_H = 0.0055, 0.0103  # door slots, 2 mm a side round a blade
SLOT_OFF = 0.0085       # slot-pair centre is this far +y of the door centre
TRAVEL = 0.016          # door travel, shut (rest) to open stop
SLACK = 0.002           # open stop: each slot 2 mm +y of its pocket, so a blade on the slot's -y wall is centred
SPRING = 225.0          # N/m: 3.6 N at the open stop
DOOR_REST_Y = SLACK + TRAVEL - SLOT_OFF  # door centre y at rest (slot-pair centre +0.018)
BLADE_LEN, BLADE_T, BLADE_W = 0.015, 0.0015, 0.0063


def _ox(x):
    return x - WALL_X


def _outlet_shapes():
    s = []
    # wall panel 22 x 240 x 200 mm with the two blind pockets in its face, and a foot behind it
    s.append(box((_ox(0.3645), 0.0, 0.100), (0.005, 0.240, 0.200), WALL_BLUE))  # back skin, x 0.362..0.367
    band = (ZB - OPEN_H / 2, ZB + OPEN_H / 2)
    holes = [(-PIN_Y - OPEN_W / 2, -PIN_Y + OPEN_W / 2), (PIN_Y - OPEN_W / 2, PIN_Y + OPEN_W / 2)]
    s += _holed((_ox(WALL_X), _ox(POCKET_END)), (-0.120, 0.120), (0.0, 0.200, *band), holes, WALL_BLUE)
    s.append(box((_ox(0.417), 0.0, 0.0075), (0.100, 0.240, 0.015), WALL_BLUE))  # foot, x 0.367..0.467
    # backing behind the plate, x 0.342..0.345: a 1.5 mm layer with the openings, and a 1.5 mm front layer whose holes
    # are 3 mm bigger, filled back to the openings by lead-in wedges
    by, bz = (-0.034, 0.034), (PLATE_ZC - PLATE_H / 2 + 0.001, PLATE_ZC + PLATE_H / 2 - 0.001)
    xm = BACK_X[0] + LEAD
    s += _holed((_ox(xm), _ox(BACK_X[1])), by, (bz[0], bz[1], *band), holes, BACKING)
    big = [(a - LEAD, b + LEAD) for a, b in holes]
    s += _holed((_ox(BACK_X[0]), _ox(xm)), by, (bz[0], bz[1], band[0] - LEAD, band[1] + LEAD), big, BACKING)
    xa, xb = _ox(BACK_X[0]), _ox(xm)
    for yc in (-PIN_Y, PIN_Y):
        y0, y1, z0, z1 = yc - OPEN_W / 2, yc + OPEN_W / 2, band[0], band[1]
        Y0, Y1, Z0, Z1 = y0 - LEAD, y1 + LEAD, z0 - LEAD, z1 + LEAD
        s.append(hull((0, 0, 0), [(xa, Y0, Z1), (xa, Y1, Z1), (xb, y0, z1), (xb, y1, z1), (xb, Y0, Z1), (xb, Y1, Z1)], BACKING))
        s.append(hull((0, 0, 0), [(xa, Y0, Z0), (xa, Y1, Z0), (xb, y0, z0), (xb, y1, z0), (xb, Y0, Z0), (xb, Y1, Z0)], BACKING))
        s.append(hull((0, 0, 0), [(xa, Y0, Z0), (xa, Y0, Z1), (xb, y0, z0), (xb, y0, z1), (xb, Y0, Z0), (xb, Y0, Z1)], BACKING))
        s.append(hull((0, 0, 0), [(xa, Y1, Z0), (xa, Y1, Z1), (xb, y1, z0), (xb, y1, z1), (xb, Y1, Z0), (xb, Y1, Z1)], BACKING))
    # cover plate front layer, x 0.337..0.342, 70 x 115 mm with rounded corners, open round the top door recess
    px0, px1 = _ox(PLATE_X), _ox(BACK_X[0])
    rz0, rz1 = ZB - REC_H / 2, ZB + REC_H / 2
    ry0, ry1 = REC_YC - REC_W / 2, REC_YC + REC_W / 2
    pz0, pz1 = PLATE_ZC - PLATE_H / 2, PLATE_ZC + PLATE_H / 2
    s.append(hull((0, 0, 0), _rounded_yz(px0, px1, -PLATE_W / 2, PLATE_W / 2, rz1, pz1, 0.004, True, False), WHITE))
    s.append(hull((0, 0, 0), _rounded_yz(px0, px1, -PLATE_W / 2, PLATE_W / 2, pz0, rz0, 0.004, False, True), WHITE))
    s.append(box(((px0 + px1) / 2, (-PLATE_W / 2 + ry0) / 2, ZB), (px1 - px0, ry0 + PLATE_W / 2, REC_H), WHITE))
    s.append(box(((px0 + px1) / 2, (ry1 + PLATE_W / 2) / 2, ZB), (px1 - px0, PLATE_W / 2 - ry1, REC_H), WHITE))
    # look only: dark recess rims on both outlets, the lower outlet's door and slots, and the plate screw
    fx = px0 - 0.0002
    for zc in (ZB, LOWER_ZB):
        for (cy, cz, w, h) in ((REC_YC, zc + REC_H / 2 - 0.0005, REC_W, 0.001), (REC_YC, zc - REC_H / 2 + 0.0005, REC_W, 0.001),
                               (ry0 + 0.0005, zc, 0.001, REC_H), (ry1 - 0.0005, zc, 0.001, REC_H)):
            s.append(_look(box((fx, cy, cz), (0.0004, w, h), RIM)))
    s.append(_look(box((fx + 0.0001, REC_YC, LOWER_ZB), (0.0003, REC_W - 0.002, REC_H - 0.002), BACKING)))
    s.append(_look(box((fx - 0.0001, DOOR_REST_Y, LOWER_ZB), (0.0003, DOOR_W, DOOR_H), DOOR_GREY)))
    for yc in (SLACK + TRAVEL - PIN_Y, SLACK + TRAVEL + PIN_Y):
        s.append(_look(box((fx - 0.0003, yc, LOWER_ZB), (0.0003, SLOT_W, SLOT_H), SLOT_BLACK)))
    s.append(_look(cyl((px0 - 0.0010, 0.0, PLATE_ZC), 0.0035, 0.0020, SCREW, axis="x")))  # screw between the outlets
    return s


def _door_shapes():
    """The sliding door in its own frame (centre of the slab at rest): 42 x 24 x 4 mm, two 5.5 x 10.3 mm slots."""
    sy = (SLOT_OFF - PIN_Y, SLOT_OFF + PIN_Y)
    holes = [(c - SLOT_W / 2, c + SLOT_W / 2) for c in sy]
    t = DOOR_X[1] - DOOR_X[0]
    return _holed((-t / 2, t / 2), (-DOOR_W / 2, DOOR_W / 2), (-DOOR_H / 2, DOOR_H / 2, -SLOT_H / 2, SLOT_H / 2), holes,
                  DOOR_GREY)


DOOR = Joint("door", "slide", axis="y", at=(_ox((DOOR_X[0] + DOOR_X[1]) / 2), DOOR_REST_Y, ZB), low=-TRAVEL, high=0.0,
             spring=SPRING, damping=2.0, mass=0.006, friction=0.4, shapes=_door_shapes())
OUTLET = Mech("outlet", pos=(WALL_X, 0.0, TOP), mass=None, shapes=_outlet_shapes(), joints=[DOOR])

# == the night light: origin at the centre of its back face, on the blade line; +x is the way the blades point =====
L_W, L_H, L_D = 0.045, 0.065, 0.028   # housing: y, z, x
BLADE_UP = 0.047                      # blade line above the housing's bottom
LENS_T = 0.002
LIGHT_MASS = 0.045
_hz0, _hz1 = -BLADE_UP, L_H - BLADE_UP
LIGHT_SHAPES = [
    hull((0, 0, 0), _rounded_yz(-L_D, 0.0, -L_W / 2, L_W / 2, _hz0, _hz1, 0.004, True, True), HOUSING),
    hull((0, 0, 0), _rounded_yz(-L_D - LENS_T, -L_D, -0.0205, 0.0205, _hz0 + 0.0025, _hz0 + 0.0625, 0.004, True, True),
         AMBER),
    _look(cyl((-L_D - LENS_T - 0.0004, 0.0, _hz1 - 0.008), 0.003, 0.0012, CELL, axis="x")),  # photocell window
    *lamp("glow", (-L_D - LENS_T - 0.0006, 0.0, _hz0 + 0.028), 0.014, {"lit": GLOW}, off=AMBER, height=0.0006, axis="x"),
    box((BLADE_LEN / 2, -PIN_Y, 0.0), (BLADE_LEN, BLADE_T, BLADE_W), NICKEL),
    box((BLADE_LEN / 2, PIN_Y, 0.0), (BLADE_LEN, BLADE_T, BLADE_W), NICKEL),
]
# A black rubber mat, 90 x 75 x 3 mm, fixed on the table under the light: the white housing stands out against it in
# the top camera. Both stand in open light, inboard of the parked right arm's shadow (the station lamp is high on the
# right, so that arm's shadow falls 13 to 23 cm right of centre in front of it).
MAT_T = 0.003
MAT_XY = (0.240, -0.055)
MAT = Box("mat", pos=(float(MAT_XY[0]), float(MAT_XY[1]), float(TOP + MAT_T / 2)), size=(0.090, 0.075, MAT_T),
          color=(0.12, 0.12, 0.13))
LIGHT_START = (0.254, -0.055, TOP + MAT_T + BLADE_UP + 0.0005)  # standing on the mat, housing centre at x 0.240
LIGHT = Mech("light", pos=(float(LIGHT_START[0]), float(LIGHT_START[1]), float(LIGHT_START[2])), mass=LIGHT_MASS,
             friction=0.8, shapes=LIGHT_SHAPES)

OBJECTS = [OUTLET, MAT, LIGHT]

# The instruction forbids no area: the wall and plate are the work. The task's declared limit is a speed limit on both
# tool points (0.10 m/s), enforced by its gate envelope; sweep.py has no region to check.
ARM_KEEP_OUT = []

# == scoring =======================================================================================================
Z = TOP + ZB
SEAT_TIP_X = PLATE_X + BLADE_LEN  # 0.352: tips when the back face is on the plate (d = 15 mm)
D_FULL, D_HALF = 0.013, 0.009
CARRY_Q = 0.006          # the door carried 10 of its 16 mm
SQUARE_DEG = 4.0
RELEASE = 0.030
STILL_V, STILL_S = 0.003, 1.0
STEP_S = 1.0 / 20.0


def door_q(outlet_state) -> float:
    """Door opening reading q (m): 0 at the open stop, 0.016 shut (slot-pair centre y minus 2 mm)."""
    return float(outlet_state.joints.get("door", 0.0)) + TRAVEL


def blade_tips(light_state):
    """World points of the two blade tips (centre of each tip face), -y blade first."""
    return [np.asarray(light_state.to_world((BLADE_LEN, s * PIN_Y, 0.0)), dtype=float) for s in (-1, 1)]


def light_axis(light_state):
    return np.asarray(light_state.rotation(), dtype=float)[:, 0]


def depth(light_state) -> float:
    """d: how far the shallower blade tip is past the plate's front face (m)."""
    return min(float(t[0]) for t in blade_tips(light_state)) - PLATE_X


def _in_rect(p, yc, zc, w, h, margin=0.0) -> bool:
    return abs(float(p[1]) - yc) <= w / 2 + margin and abs(float(p[2]) - zc) <= h / 2 + margin


def in_pockets(light_state) -> bool:
    """Both blades in their own receptacle openings: each tip past the backing's front face, inside its 4.5 x 9.3 mm
    opening (y and z), and the -y blade in the -y opening."""
    tips = blade_tips(light_state)
    return all(float(t[0]) > BACK_X[0] and _in_rect(t, s * PIN_Y, Z, OPEN_W, OPEN_H)
               for t, s in zip(tips, (-1, 1)))


def tips_in_door_slots(light_state, q) -> bool:
    """Both blade tips at least 0.5 mm into the door, each inside its own door slot (0.5 mm margin)."""
    yc = SLACK + q  # slot-pair centre y
    tips = blade_tips(light_state)
    return all(float(t[0]) >= DOOR_X[0] + 0.0005 and _in_rect(t, yc + s * PIN_Y, Z, SLOT_W, SLOT_H, 0.0005)
               for t, s in zip(tips, (-1, 1)))


def square_deg(light_state) -> float:
    a = light_axis(light_state)
    return math.degrees(math.acos(float(np.clip(a[0], -1.0, 1.0))))


def _released(light_state) -> bool:
    """Every finger pad point of both arms 30 mm or more from the light's bounding box (False offline)."""
    return mechkit.released(light_state, clearance=RELEASE)


def _fresh():
    return {"last": None, "prev": None, "still": 0.0, "fell": False, "carried": False, "clear": False,
            "lit": False}


_S = _fresh()


@on_reset
def _reset_scorer(env=None):
    _S.clear()
    _S.update(_fresh())


@on_step
def _lamp(env=None):
    """Scorer side, look only: the night light's lens glows once both blades are in the receptacle."""
    set_lamp(LIGHT, "glow", "lit" if _S["lit"] else "off")


def _track(objects) -> None:
    """Whole-run latches and the stillness clock, once per control step (success() and score() share one dict)."""
    if objects is _S["last"]:
        return
    _S["last"] = objects
    light, outlet = objects["light"], objects["outlet"]
    q = door_q(outlet)
    if float(np.asarray(light.low)[2]) < TOP - 0.02:  # off the table
        _S["fell"] = True
    if q <= CARRY_Q and tips_in_door_slots(light, q):
        _S["carried"] = True
    p = np.asarray(light.origin, dtype=float)
    if _S["prev"] is not None:
        v = float(np.linalg.norm(p - _S["prev"])) / STEP_S
        _S["still"] = _S["still"] + STEP_S if v < STILL_V else 0.0
    _S["prev"] = p
    _S["clear"] = _released(light)
    _S["lit"] = in_pockets(light)


def score(objects):
    """0 for the run if the light ever went off the table. At the end, the light must be released (every finger 30 mm
    clear) and still (under 3 mm/s for 1 s), else 0. Then: 1.0 both blades in their receptacle openings with d >= 13 mm
    and the light square within 4 deg; 0.5 both in with d 9 to 13 mm; 0.25 if the door was ever carried 10 of its 16 mm
    with both tips in its slots (and neither above applies); else 0."""
    _track(objects)
    if _S["fell"] or not _S["clear"] or _S["still"] < STILL_S:
        return 0.0
    light = objects["light"]
    d = depth(light)
    if in_pockets(light):
        if d >= D_FULL and square_deg(light) <= SQUARE_DEG:
            return 1.0
        if d >= D_HALF:
            return 0.5
    return 0.25 if _S["carried"] else 0.0


def success(objects):
    return score(objects) == 1.0


# == the scripted demo (right arm only; the left arm holds READY) ==================================================
PITCH = math.radians(25.0)
GRIP_ROT = grip_rot(pitch=PITCH)        # yaw 0: pads close along y on the housing's sides; wrist leans back to -x
APPROACH = GRIP_ROT[:, 2]               # the gripper's approach axis (down and toward +x)
PAD_AT = np.array([-0.024, 0.0, 0.008])  # tool point in the light's frame: 24 mm in front of the back face
GRIP_OPEN = -0.0325                     # one finger: pads 65 mm apart round the 45 mm housing
GRIP_RELEASE = -0.0350                  # 70 mm apart to let go: 12.5 mm clear of each side
V_CARRY, V_SLOW, V_DRAG, V_PUSH, V_PARK = 0.06, 0.02, 0.004, 0.005, 0.085
TIP_SHORT = 0.005                       # pre-door: tips 5 mm short of the door face
TIP_IN = DOOR_X[1] - 0.0005             # tips held 3.5 mm into the 4 mm door, 0.5 mm short of the backing
demo = placeholder_demo


def _mv(arm, pos, vmax, finger=None):
    """Smoothstep move whose peak speed (1.5 x the mean) stays under vmax."""
    d = float(np.linalg.norm(np.asarray(pos, dtype=float) - arm.pos))
    yield from arm.move(pos, max(0.3, 1.5 * d / vmax), finger, rot=GRIP_ROT)


def _ry(t):
    c, s = math.cos(t), math.sin(t)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def _rz(t):
    c, s = math.cos(t), math.sin(t)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _tips_mid(ctx):
    """Midpoint of the two blade tips and the light's blade axis, read from the simulator; None offline (the dry
    run's states never move)."""
    if ctx.offline:
        return None
    st = ctx.obj("light")
    a, b = blade_tips(st)
    return (a + b) / 2, light_axis(st)


def _q(ctx, est):
    """The door's opening reading q from the simulator; offline the estimate est (from the commanded carry)."""
    return est if ctx.offline else door_q(ctx.obj("outlet"))


class _Carry:
    """Moves the gripped light so the midpoint of its blade tips follows a goal, correcting the arm's command each step
    from the light's read pose (arm sag, the door spring's pull and any slip in the pads all show there). Offline the
    correction is zero and the tips are taken to be where the command puts them."""

    def __init__(self, ctx, arm):
        self.ctx, self.arm = ctx, arm
        live = _tips_mid(ctx)
        t = live[0] if live is not None else np.asarray(arm.pos, dtype=float) - PAD_AT + np.array([BLADE_LEN, 0, 0])
        self.off = arm.pos - t  # tool point minus tip midpoint, as gripped (includes the sag)
        self.corr = np.zeros(3)
        self.att = np.zeros(2)  # trim (pitch about y, yaw about z), rad, so the blades stay square to the wall
        self.goal = t.copy()

    def rot(self):
        return _rz(self.att[1]) @ _ry(self.att[0]) @ GRIP_ROT

    def tips(self):
        live = _tips_mid(self.ctx)
        return live[0] if live is not None else self.goal.copy()

    def step(self, goal, gain=0.12, mask=(1.0, 1.0, 1.0)):
        self.goal = np.asarray(goal, dtype=float)
        live = _tips_mid(self.ctx)
        if live is not None:
            e = np.clip((self.goal - live[0]) * np.asarray(mask), -0.003, 0.003)
            self.corr = np.clip(self.corr + gain * e, -0.040, 0.040)
            a = live[1]
            err = np.array([math.asin(float(np.clip(a[2], -1.0, 1.0))), -math.atan2(float(a[1]), float(a[0]))])
            self.att = np.clip(self.att + 0.10 * np.clip(err, -0.02, 0.02), -0.12, 0.12)
        self.arm._set_cartesian(self.goal + self.off + self.corr, self.rot(), self.arm.finger)
        yield

    def go(self, goal, vmax, settle=0.6, **kw):
        """Straight line of the tips to goal at peak speed vmax, then settle there."""
        a, b = self.goal.copy(), np.asarray(goal, dtype=float)
        steps = max(1, round(1.5 * float(np.linalg.norm(b - a)) / vmax / self.arm.dt))
        for i in range(1, steps + 1):
            s = i / steps
            yield from self.step(a + (s * s * (3 - 2 * s)) * (b - a), **kw)
        for _ in range(round(settle / self.arm.dt)):
            yield from self.step(b, **kw)



def _push_home(ctx, R, carry):
    """Step 5: push the blades home on the live tip pose; True once seated (or offline after one try)."""
    n3 = round(3.0 / R.dt)
    for attempt in range(3):
        x = float(carry.tips()[0])
        hist, extra = [x], 0
        while True:
            tips = carry.tips()
            off = max(abs(float(tips[1])), abs(float(tips[2]) - Z))
            if ctx.offline or float(tips[0]) < BACK_X[0] - 0.0015 or off < 0.001:
                x = min(SEAT_TIP_X, x + V_PUSH * R.dt)
            yield from carry.step(np.array([x, 0.0, Z]), gain=0.12)
            tx = float(carry.tips()[0])
            if tx >= SEAT_TIP_X - 0.0003:
                break
            hist.append(tx)
            if x >= SEAT_TIP_X:
                extra += 1
            if (len(hist) > n3 and tx - hist[-1 - n3] < 0.0002) or extra > round(12.0 / R.dt):
                break
        for _ in range(round(0.5 / R.dt)):
            yield from carry.step(np.array([SEAT_TIP_X, 0.0, Z]), gain=0.12)
        if ctx.offline:
            return True
        light = ctx.obj("light")
        if in_pockets(light) and depth(light) >= D_FULL + 0.0005:
            return True
        yield from carry.go(np.array([BACK_X[0] + 0.0025, 0.0, Z]), V_SLOW, settle=0.8)
    return False


def demo_script(ctx):
    """Right arm: grip the light's housing sides, carry it to the door, put the blade tips into the door's slots, drag
    the door to its open stop, push the blades home, let go and park."""
    R = ctx.right
    o0 = np.array(LIGHT_START) if ctx.offline else np.asarray(ctx.obj("light").origin, dtype=float)
    grasp = o0 + PAD_AT
    above = grasp - 0.060 * APPROACH

    # 1) pick: open to 65 mm, come down the tilted approach axis onto the housing's sides, close
    yield from R.grip(GRIP_OPEN, 0.6)
    yield from _mv(R, above, V_CARRY)
    yield from R.trim_to_measured(above)
    yield from _mv(R, grasp, V_SLOW)
    yield from R.trim_to_measured(grasp, max_comp=0.010)
    for _ in range(2):
        yield from R.grip(FINGER_CLOSED, 0.8)
        yield from R.hold(0.4)
        if R.holding(0.040):
            break
        yield from R.grip(GRIP_OPEN, 0.6)
        yield from R.trim_to_measured(grasp, max_comp=0.010)
    else:
        yield from _mv(R, above, V_SLOW, GRIP_OPEN)
        yield from R.home(6.0)
        return

    # 2) lift the blade line to the slot line, then carry to the door: tips 5 mm short, on the door's slots
    carry = _Carry(ctx, R)
    t = carry.goal.copy()
    yield from carry.go(np.array([t[0], t[1], Z]), V_SLOW)
    seated, lag_free = False, np.zeros(3)
    for attempt in range(2):
        slot_y = SLACK + _q(ctx, TRAVEL)  # slot-pair centre, read from the door
        pre = np.array([DOOR_X[0] - TIP_SHORT, slot_y, Z])
        yield from carry.go(pre, V_CARRY, settle=1.0)
        lag_free = R.pos - R.measured_pos  # the arm's sag holding the light in free space, before any contact

        # 3) tips into the door's slots: in at 5 mm/s until they stop (on the backing, or on the door face if they
        #    missed), then hold them 3.5 mm in, 0.5 mm short of the backing
        res = yield from R.press_until(R.pos.copy(), max_depth=TIP_SHORT + 0.0045, err_mm=3.0,
                                       min_depth=TIP_SHORT + 0.001, direction=(1.0, 0.0, 0.0), speed=V_PUSH,
                                       rot=carry.rot(), settle=0.2, hold=0.3)
        tips = carry.tips() if not ctx.offline else pre + np.array([res.depth, 0.0, 0.0])
        carry.goal, carry.corr, carry.off = tips.copy(), np.zeros(3), R.pos - tips
        if tips[0] < DOOR_X[0] + 0.0015:  # stopped on the door face: the blades missed the slots
            yield from carry.go(pre - np.array([0.010, 0.0, 0.0]), V_SLOW)
            continue
        yield from carry.go(np.array([TIP_IN, slot_y, Z]), V_SLOW, settle=0.3, mask=(1.0, 0.0, 1.0))

        # 4) drag the door across: the tips led -y at 4 mm/s, closed loop on the light's read pose (so the arm's lag
        #    under the door spring is taken out), until the door reads its open stop
        q0 = _q(ctx, TRAVEL)
        qmin, travel, ok = q0, 0.0, False
        start = carry.goal.copy()
        y0 = float(carry.tips()[1])
        while travel < 0.030:
            travel += V_DRAG * R.dt
            yield from carry.step(start - np.array([0.0, travel, 0.0]), gain=0.10)
            q = _q(ctx, max(0.0, TRAVEL - max(0.0, travel - SLACK)))
            qmin = min(qmin, q)
            moved = y0 - float(carry.tips()[1])
            if q < 0.0003:
                ok = True
                break
            if (moved >= SLACK + 0.004 and q0 - q < 0.002) or q > qmin + 0.002:  # blades out of the slots, or the
                break                                                           # door slipped back
        if not ok:
            yield from carry.go(carry.goal - np.array([0.010, 0.0, 0.0]), V_SLOW, mask=(1.0, 0.0, 1.0))
            continue

        # 5) push in. The tips are held on the receptacle openings' centre line (y 0, where a blade bearing on its
        #    door slot's -y wall is centred) by the live correction on the light's read pose, so no side load builds
        #    up against the door stop, and z on the slot line. Near the backing the x goal waits while the tips are
        #    more than 1 mm off in y or z. The goal runs in at 5 mm/s to the seat; the x integrator keeps adding push
        #    while blade friction slows the tips. A stall (under 0.2 mm in 3 s) ends a try; the demo then reads the
        #    depth, backs the tips out to 2.5 mm into the backing (still through the door, so it stays open),
        #    re-centres and pushes again, up to three tries.
        seated = yield from _push_home(ctx, R, carry)
        break

    # 6) let go. First unload: the arm has been leaning -y against the door spring and +x into the plate. Walk the
    #    y command over 1 s to the encoder-measured tool point plus the sag it had in free space, so the spring sets
    #    the blades on the +y walls of the receptacle slots; then walk x back the same way, stopping as soon as the
    #    tips start to come out (0.3 mm). Then open to 70 mm over 2 s while holding the measured tool point still
    #    along x, so nothing lunges at the plate when the pads let go. Back straight out 60 mm, move up and away from
    #    the wall, park at READY and stay still.
    if seated:
        n = round(1.0 / R.dt)
        for axis in (1, 0):
            p0 = R.pos.copy()
            target = float(R.measured_pos[axis] + lag_free[axis])
            x_start = float(carry.tips()[0])
            for i in range(1, n + 1):
                p = R.pos.copy()
                p[axis] = p0[axis] + (i / n) * (target - p0[axis])
                R._set_cartesian(p, R.rot.copy(), R.finger)
                yield
                if axis == 0 and float(carry.tips()[0]) < x_start - 0.0003:
                    break
            yield from R.hold(0.3)
        # open slowly (2 s) while holding the encoder-measured tool point where it is: any push the arm still stores
        # bleeds off as the pads slip, instead of throwing the open fingers at the plate
        hold_at, f0 = R.measured_pos.copy(), float(R.finger)
        n = round(2.0 / R.dt)
        for i in range(1, n + 1):
            err = np.zeros(3)
            err[0] = np.clip(float(R.measured_pos[0]) - hold_at[0], -0.003, 0.003)
            R._set_cartesian(R.pos - 0.6 * err, R.rot.copy(), f0 + (i / n) * (GRIP_RELEASE - f0))
            yield
    yield from R.grip(GRIP_RELEASE, 1.0 if not seated else 0.3)
    yield from R.hold(0.3)
    yield from _mv(R, R.pos - np.array([0.060, 0.0, 0.0]), V_CARRY)
    yield from _mv(R, np.array([0.240, -0.120, Z + 0.080]), V_PARK)  # up and away from the wall, then the joint swing
    yield from R.home(4.5)
    yield from ctx.wait(10.0 if seated else 2.0)
