"""On the station table: a compact push-bar wall soap dispenser (125 x 148 x 271 mm) screwed to a splashback board at
table height, its cover hanging open on its stop. A spent 600 mL refill sits in it, pump down: the bottle rests on a
ledge, its 28 mm neck at the end of a 34 mm slot and its 38 mm pump collar under the ledge, 1 mm over the push
lever's fork. A full refill (0.66 kg) stands on a cradle to the robot's right; a waste bin stands to its left.

The job: draw the spent refill forward to the 44 mm keyhole entrance, lift it out and bin it; lower the new refill
until its collar drops through the entrance, slide it 43 mm back to the slot end, shut the cover, push the bar once.
The fork strokes the pump collar only when the collar is over it, so one push is the functional check.

Requires -E object_jitter=0.

Run it:  inspect-robots run --embodiment yam_arena -E task=swap_a_wall_soap_dispenser_refill_through_its_keyhole.py \\
           -E object_jitter=0 --instruction "Swap the empty soap refill for the new one, put the empty in the bin,
           shut the cover and test it with one push." --scorer task_score ...
"""

import dataclasses
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402,F401
from yam_arena.task_file import TABLE_TOP  # noqa: E402

import belimits  # noqa: E402
import mechkit  # noqa: E402
from closedloop import grip_rot, placeholder_demo  # noqa: E402
from mechkit import Joint, Mech, box, cyl, extrude, hull, lathe, on_reset, released  # noqa: E402

SCENE = "station"
TOP = float(TABLE_TOP[SCENE])
SUCCESS_HOLD_S = 8.0

INSTRUCTION = ("Swap the empty soap refill for the new one, put the empty in the bin, shut the cover and test it with "
               "one push.")

WHITE, TILE = (0.93, 0.93, 0.91), (0.62, 0.78, 0.88)
DARK, MID = (0.20, 0.21, 0.23), (0.45, 0.46, 0.48)
BLUE, SPENT_GREY, BIN_GREY = (0.10, 0.30, 0.75), (0.66, 0.66, 0.64), (0.30, 0.32, 0.35)

# ---- the dispenser (station frame, metres; z above the table top) ---------------------------------------------------
DISP_AT = (0.330, 0.0, TOP)  # Mech origin: on the table under the dispenser
LEDGE = 0.150  # top of the ledge the bottle rests on
LEDGE_T = 0.006
ENTR_X, ENTR_W = 0.310, 0.044  # keyhole entrance: 44 x 44 mm, centred here
SLOT_W, SLOT_END = 0.034, 0.367  # 34 mm slot running back from the entrance
NECK_R, PUMP_R, COLLAR_R, NOZZLE_R = 0.014, 0.011, 0.019, 0.005
SEAT_X = SLOT_END - NECK_R  # 0.353: the neck against the slot end, the collar over the fork
WALL_Y0, WALL_Y1 = 0.0575, 0.0625
FRONT_X, BACK_X = 0.262, 0.397
HINGE = (0.262, 0.0, 0.090)  # cover hinge line (along y)
COVER_OPEN_DEG = 60.0
COVER_SHUT = math.radians(COVER_OPEN_DEG)  # joint value when shut; 0 is open on its stop
COVER_SPRING, COVER_MASS = 0.35, 0.26
PIVOT = (0.290, 0.0, 0.1255)  # push lever pivot (along y)
LEVER_TRAVEL, LEVER_SPRING, LEVER_MASS = 0.12, 0.3, 0.08
FORK_TOP, RIDGE_TOP = 0.120, 0.123  # the plate stays under the collar; only the 3 mm ridges touch it
RIDGE_X = (0.349, 0.357)
RIDGE_Y = (0.0125, 0.0175)  # outside the 22 mm pump, inside the 38 mm collar
COLLAR_SPRING, COLLAR_TRAVEL, COLLAR_MASS = 250.0, 0.010, 0.02


def _d(x, y, z):
    """Station coordinates (z above the table) to the dispenser Mech's frame."""
    return (x - DISP_AT[0], y, z)


def _dbox(x0, x1, y0, y1, z0, z1, color):
    return box(_d((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2), (x1 - x0, y1 - y0, z1 - z0), color)


def _look(shape):
    return dataclasses.replace(shape, collide=False)


_e0, _e1, _s = ENTR_X - ENTR_W / 2, ENTR_X + ENTR_W / 2, SLOT_W / 2
LEDGE_STRIPS = [  # six flat strips round the keyhole: entrance x 0.288..0.332, slot back to x 0.367
    _dbox(FRONT_X, _e0, -WALL_Y0, WALL_Y0, LEDGE - LEDGE_T, LEDGE, DARK),
    _dbox(_e0, _e1, ENTR_W / 2, WALL_Y0, LEDGE - LEDGE_T, LEDGE, DARK),
    _dbox(_e0, _e1, -WALL_Y0, -ENTR_W / 2, LEDGE - LEDGE_T, LEDGE, DARK),
    _dbox(_e1, SLOT_END, _s, WALL_Y0, LEDGE - LEDGE_T, LEDGE, DARK),
    _dbox(_e1, SLOT_END, -WALL_Y0, -_s, LEDGE - LEDGE_T, LEDGE, DARK),
    _dbox(SLOT_END, BACK_X, -WALL_Y0, WALL_Y0, LEDGE - LEDGE_T, LEDGE, DARK),
]
BOARD = _dbox(0.405, 0.423, -0.20, 0.20, 0.0, 0.36, TILE)
BACK_PLATE = _dbox(BACK_X, 0.405, -WALL_Y1, WALL_Y1, 0.05, 0.315, WHITE)
SIDE_WALLS = [
    _dbox(FRONT_X, BACK_X, WALL_Y0, WALL_Y1, 0.05, LEDGE, WHITE),
    _dbox(FRONT_X, BACK_X, -WALL_Y1, -WALL_Y0, 0.05, LEDGE, WHITE),
]
TILE_LINES = [_look(_dbox(0.4045, 0.405, -0.20, 0.20, z - 0.001, z + 0.001, WHITE)) for z in (0.12, 0.24)] + [
    _look(_dbox(0.4045, 0.405, y - 0.001, y + 0.001, 0.0, 0.36, WHITE)) for y in (-0.13, 0.13)]
BODY_SHAPES = [BOARD, BACK_PLATE, *SIDE_WALLS, *LEDGE_STRIPS]

# The cover, written in its shut pose relative to the hinge line, then turned 60 degrees forward: joint 0 is open.
_A = -COVER_SHUT
_TILT = (math.cos(_A / 2), 0.0, math.sin(_A / 2), 0.0)
_RC = mechkit._axis_matrix("y", _A)


def _cbox(x0, x1, y0, y1, z0, z1, color):
    c = _RC @ np.array([(x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2])
    return box((float(c[0]), float(c[1]), float(c[2])), (x1 - x0, y1 - y0, z1 - z0), color, tilt=_TILT)


COVER_H, COVER_T, COVER_DEEP = 0.231, 0.005, 0.143
_top = [(-COVER_T, COVER_H - COVER_T), (-COVER_T, COVER_H - 0.003), (-0.002, COVER_H), (COVER_DEEP, COVER_H),
        (COVER_DEEP, COVER_H - COVER_T)]  # (x, z) section of the top plate with its rounded front edge
_top_pts = [tuple(float(v) for v in _RC @ np.array([x, y, z])) for y in (-WALL_Y1, WALL_Y1) for x, z in _top]
COVER_SHAPES = [
    _cbox(-COVER_T, 0.0, -WALL_Y1, WALL_Y1, 0.0, COVER_H - COVER_T, WHITE),  # face, 125 x 226 mm
    hull((0.0, 0.0, 0.0), _top_pts, WHITE),  # top plate, 143 mm deep
    _cbox(0.0, 0.025, WALL_Y0, WALL_Y1, 0.065, COVER_H - COVER_T, WHITE),  # side returns, 25 mm
    _cbox(0.0, 0.025, -WALL_Y1, -WALL_Y0, 0.065, COVER_H - COVER_T, WHITE),
    _look(_cbox(-COVER_T - 0.0006, -COVER_T, -0.015, 0.015, 0.09, 0.19, DARK)),  # sight window (look only)
]
COVER = Joint("cover", "hinge", axis="y", at=_d(*HINGE), low=0.0, high=COVER_SHUT, spring=COVER_SPRING,
              damping=0.05, mass=COVER_MASS, detents=(0.0, COVER_SHUT), shapes=COVER_SHAPES)


def _lbox(x0, x1, y0, y1, z0, z1, color):
    return box(((x0 + x1) / 2 - PIVOT[0], (y0 + y1) / 2, (z0 + z1) / 2 - PIVOT[2]), (x1 - x0, y1 - y0, z1 - z0), color)


_FORK_OUT = [(0.334, -0.050), (0.380, -0.050), (0.380, 0.050), (0.334, 0.050), (0.334, 0.012), (0.366, 0.012),
             (0.366, -0.012), (0.334, -0.012)]  # U fork, 24 mm slot open to the front, closed at x 0.366: the pump passes


def _fork():
    s = extrude((0.0, 0.0, 0.0), [(x - PIVOT[0], y) for x, y in _FORK_OUT], 0.005, DARK, axis="z")
    lo, hi = mechkit.shape_aabb(s)
    dz = (FORK_TOP - PIVOT[2]) - float(hi[2])
    s = extrude((0.0, 0.0, dz), [(x - PIVOT[0], y) for x, y in _FORK_OUT], 0.005, DARK, axis="z")
    lo, hi = mechkit.shape_aabb(s)
    want = (0.334 - PIVOT[0], -0.050, FORK_TOP - 0.005 - PIVOT[2], 0.380 - PIVOT[0], 0.050, FORK_TOP - PIVOT[2])
    if max(abs(a - b) for a, b in zip((*lo, *hi), want)) > 1e-6:
        raise ValueError(f"fork: unexpected bounds {lo} {hi}")
    return s


LEVER_SHAPES = [
    _lbox(0.252, 0.264, -0.055, 0.055, 0.050, 0.080, DARK),  # push bar, 110 x 30 mm
    _lbox(0.264, 0.274, 0.040, 0.050, 0.050, 0.122, MID),  # side arms up to the pivot height
    _lbox(0.264, 0.274, -0.050, -0.040, 0.050, 0.122, MID),
    _lbox(0.274, 0.334, 0.040, 0.050, 0.112, 0.122, MID),
    _lbox(0.274, 0.334, -0.050, -0.040, 0.112, 0.122, MID),
    _fork(),
    _lbox(RIDGE_X[0], RIDGE_X[1], RIDGE_Y[0], RIDGE_Y[1], FORK_TOP, RIDGE_TOP, DARK),  # contact ridges beside the nozzle
    _lbox(RIDGE_X[0], RIDGE_X[1], -RIDGE_Y[1], -RIDGE_Y[0], FORK_TOP, RIDGE_TOP, DARK),
    _lbox(0.364, 0.390, -0.045, 0.045, 0.093, 0.115, MID),  # counterweight behind the pivot
]
# Pushing the bar in turns the lever about -y (the fork rises), so the joint runs 0 .. -0.12 rad.
LEVER = Joint("lever", "hinge", axis="y", at=_d(*PIVOT), low=-LEVER_TRAVEL, high=0.0, spring=LEVER_SPRING,
              damping=0.02, mass=LEVER_MASS, shapes=LEVER_SHAPES)
DISPENSER = Mech("dispenser", pos=DISP_AT, mass=None, friction=0.3, shapes=[*BODY_SHAPES, *TILE_LINES],
                 joints=[COVER, LEVER])

# ---- the refills (refill frame: origin at the centre of the bottle's shoulder face, z up; the pump hangs below) ------
BODY_X, BODY_Y, BODY_H, CHAMFER = 0.076, 0.064, 0.150, 0.006
REFILL_H = 0.188
COLLAR_Z = -0.024  # centre of the 4 mm collar, 22 to 26 mm below the shoulder face


def _octagon(hx, hy, c, z):
    return [(sx * (hx - (c if k else 0.0)), sy * (hy - (0.0 if k else c)), z)
            for sx in (1, -1) for sy in (1, -1) for k in (0, 1)]


def _refill(name, at, mass, body_color, band_color):
    hx, hy = BODY_X / 2, BODY_Y / 2
    body = hull((0.0, 0.0, 0.0), _octagon(hx, hy, CHAMFER, 0.0) + _octagon(hx, hy, CHAMFER, BODY_H - CHAMFER)
                + _octagon(hx - CHAMFER, hy - CHAMFER, CHAMFER / 2, BODY_H), body_color)
    band = _look(hull((0.0, 0.0, 0.0), _octagon(hx + 0.0004, hy + 0.0004, CHAMFER, 0.045)
                      + _octagon(hx + 0.0004, hy + 0.0004, CHAMFER, 0.095), band_color))
    neck = lathe((0.0, 0.0, -0.022), [(PUMP_R, 0.0), (PUMP_R, 0.009), (NECK_R, 0.011), (NECK_R, 0.022)], WHITE)
    collar = Joint("collar", "slide", axis="z", at=(0.0, 0.0, COLLAR_Z), low=0.0, high=COLLAR_TRAVEL,
                   spring=COLLAR_SPRING, damping=2.0, mass=COLLAR_MASS,
                   shapes=[cyl((0.0, 0.0, 0.0), COLLAR_R, 0.004, DARK), cyl((0.0, 0.0, -0.008), NOZZLE_R, 0.012, DARK)])
    return Mech(name, pos=at, mass=mass - COLLAR_MASS, friction=0.4, shapes=[body, neck, band], joints=[collar])


STAND_XY, STAND_H, STAND_GAP = (0.220, -0.190), 0.050, 0.046
NEW_MASS, SPENT_MASS = 0.66, 0.12
NEW_AT = (STAND_XY[0], STAND_XY[1], TOP + STAND_H)
SPENT_AT = (SEAT_X, 0.0, TOP + LEDGE)
REFILL_NEW = _refill("refill_new", NEW_AT, NEW_MASS, WHITE, BLUE)
REFILL_SPENT = _refill("refill_spent", SPENT_AT, SPENT_MASS, SPENT_GREY, MID)

STAND = Mech("stand", pos=(STAND_XY[0], STAND_XY[1], TOP), mass=None, friction=0.5, shapes=[
    box((0.0, (STAND_GAP + 0.027) / 2, STAND_H / 2), (0.100, 0.027, STAND_H), BLUE),  # two rails either side of the pump
    box((0.0, -(STAND_GAP + 0.027) / 2, STAND_H / 2), (0.100, 0.027, STAND_H), BLUE),
])

BIN_XY, BIN_IN, BIN_H, BIN_T = (0.220, 0.170), 0.180, 0.150, 0.003
_bo = BIN_IN / 2 + BIN_T / 2
BIN = Mech("bin", pos=(BIN_XY[0], BIN_XY[1], TOP), mass=None, friction=0.5, shapes=[
    box((0.0, 0.0, BIN_T / 2), (BIN_IN + 2 * BIN_T, BIN_IN + 2 * BIN_T, BIN_T), BIN_GREY),
    box((_bo, 0.0, BIN_H / 2), (BIN_T, BIN_IN + 2 * BIN_T, BIN_H), BIN_GREY),
    box((-_bo, 0.0, BIN_H / 2), (BIN_T, BIN_IN + 2 * BIN_T, BIN_H), BIN_GREY),
    box((0.0, _bo, BIN_H / 2), (BIN_IN, BIN_T, BIN_H), BIN_GREY),
    box((0.0, -_bo, BIN_H / 2), (BIN_IN, BIN_T, BIN_H), BIN_GREY),
])

OBJECTS = [DISPENSER, STAND, BIN, REFILL_NEW, REFILL_SPENT]

# ---- scoring ----------------------------------------------------------------------------------------------------------
SEAT_TOL, BASE_TOL, SEAT_TILT = 0.004, 0.002, 5.0  # N: neck axis, base height, tilt
SHUT_TOL = math.radians(2.0)  # L
PUSH_RAD, STROKE = 0.08, 0.005  # P
HELD_STEPS = 20  # N and L held 1 s before the push counts
DROP, DROP_STEPS, TIP_DEG = 0.05, 10, 45.0  # D: 5 cm within 0.5 s while released, or tilted past 45 degrees
STILL_SPEED, REST_STEPS = 0.01, 20
RELEASE_CLEAR = 0.03
RIM_MARGIN = 0.010
OFFLINE_FINGERS_CLEAR = True  # what the release test returns with no simulator (the offline checker flips it)

_L = {"pumped": False, "dropped": False, "still": 0, "held": 0, "zs": [], "seen": None}


@on_reset
def _clear(_env=None):
    _L.update(pumped=False, dropped=False, still=0, held=0, zs=[], seen=None)


def _clear_of(objects, name) -> bool:
    if mechkit._ENV["env"] is None:
        return OFFLINE_FINGERS_CLEAR
    return released(objects[name], RELEASE_CLEAR)


def _fingers_clear(objects) -> bool:
    return _clear_of(objects, "refill_new") and _clear_of(objects, "refill_spent")


def spent_binned(objects) -> bool:
    """S: the spent refill's centre inside the bin's inner footprint, its lowest point 1 cm or more below the rim."""
    s = objects["refill_spent"]
    c = np.asarray(s.pos, dtype=float)
    inside = abs(c[0] - BIN_XY[0]) <= BIN_IN / 2 and abs(c[1] - BIN_XY[1]) <= BIN_IN / 2
    return bool(inside and float(s.low[2]) <= TOP + BIN_H - RIM_MARGIN and float(s.low[2]) >= TOP - 0.02)


def new_pose(objects):
    """(neck axis distance from the slot-end seat in m, base height above the ledge top in m, tilt in degrees)."""
    n = objects["refill_new"]
    o = np.asarray(n.origin, dtype=float)
    return (float(math.hypot(o[0] - SEAT_X, o[1])), float(o[2] - (TOP + LEDGE)), float(belimits.tilt_deg(n)))


def new_seated(objects) -> bool:
    """N: neck axis within 4 mm of the slot end seat, base within 2 mm of the ledge top, tilt 5 degrees or less."""
    d, h, tilt = new_pose(objects)
    return d <= SEAT_TOL and abs(h) <= BASE_TOL and tilt <= SEAT_TILT


def cover_shut(objects) -> bool:
    return float(objects["dispenser"].joints["cover"]) >= COVER_SHUT - SHUT_TOL


def _update(objects) -> None:
    """Latch what happened this control step. yam_arena calls success() and score() on the same dict each step, so a
    dict already seen changes nothing."""
    if objects is _L["seen"]:
        return
    _L["seen"] = objects
    new, spent, disp = objects["refill_new"], objects["refill_spent"], objects["dispenser"]
    free = _clear_of(objects, "refill_new")
    # D: dropped 5 cm or more within 0.5 s with the fingers clear the whole time, or tipped past 45 degrees
    z = float(np.asarray(new.origin, dtype=float)[2])
    _L["zs"] = (_L["zs"] + [z])[-(DROP_STEPS + 1):] if free else [z]
    if max(_L["zs"]) - z >= DROP or belimits.tilt_deg(new) > TIP_DEG:
        _L["dropped"] = True
    # P: one push after N and L have held for 1 s
    _L["held"] = _L["held"] + 1 if (new_seated(objects) and free and cover_shut(objects)) else 0
    if (_L["held"] >= HELD_STEPS and -float(disp.joints["lever"]) >= PUSH_RAD
            and float(new.joints["collar"]) >= STROKE):
        _L["pumped"] = True
    still = all(float(np.linalg.norm(o.lin_vel)) < STILL_SPEED for o in (new, spent))
    _L["still"] = _L["still"] + 1 if still else 0


def score(objects):
    """0 unless both refills have been still for 1 s and every finger point is 3 cm clear of both. Then 1.0 for S, N, L
    and P; 0.75 for S, N, L; 0.5 for S, N; 0.25 for S alone or N without S; a dropped or tipped new refill (D) caps
    the score at 0.25."""
    _update(objects)
    if _L["still"] < REST_STEPS or not _fingers_clear(objects):
        return 0.0
    s, n, shut = spent_binned(objects), new_seated(objects), cover_shut(objects)
    if s and n and shut and _L["pumped"]:
        out = 1.0
    elif s and n and shut:
        out = 0.75
    elif s and n:
        out = 0.5
    elif s or n:
        out = 0.25
    else:
        out = 0.0
    return min(out, 0.25) if _L["dropped"] else out


def success(objects):
    return score(objects) == 1.0


# The declared limit: the pinch line along the dispenser's top back edge, where the closing cover's top plate meets
# the back plate. No arm part goes into this footprint below the top of the band.
PINCH = {"x": (0.375, 0.410), "y": (-0.075, 0.075), "z": (TOP + 0.290, TOP + 0.340)}
ARM_KEEP_OUT = [{"name": "cover_pinch_line", "x": PINCH["x"], "y": PINCH["y"], "z_top": PINCH["z"][1]}]

# ---- the scripted demo ------------------------------------------------------------------------------------------------
PITCH = 1.0  # top grasp: hand leaned 57 degrees forward, pads closing across y on the bottle's flat sides
GRIP_H, GRIP_X = 0.125, 0.006  # tool point above the shoulder face, and ahead of the bottle's axis
OPEN_GAP, CLOSE_GAP, HOLD_GAP = 0.088, 0.048, 0.056  # pad gaps on the 64 mm bottle
LIFT_OUT = 0.080  # spent refill lifted this far at the entrance
CARRY_Z = LEDGE + 0.065  # new refill's base while carried: nozzle 27 mm over the side walls
BIN_DROP_Z = BIN_T + 0.020 + 0.038  # release with the nozzle 2 cm over the bin floor
PUSH_ROT = grip_rot(yaw=-0.7, pitch=0.6)  # left hand for the cover push
PUSH_S, PUSH_LEAD, PUSH_OFF = 0.210, 0.15, 0.015  # contact 21 cm from the hinge, aimed 0.15 rad ahead
BAR_Z = 0.060
X_GATE = PINCH["x"][0] - 0.003  # commanded tool x never passes this
demo = placeholder_demo


def _finger(gap: float) -> float:
    return FINGER_OPEN * gap / (2.0 * abs(FINGER_OPEN))


def demo_script(ctx):
    L, R = ctx.left, ctx.right
    off = L.offline
    dt = ctx.dt
    rot = grip_rot(pitch=PITCH)
    nom = {"refill_spent": np.array([SEAT_X, 0.0, TOP + LEDGE]), "refill_new": np.array(NEW_AT, dtype=float)}

    def base(name):
        """The refill's shoulder-face centre: read from the simulator, or the plan's nominal value offline."""
        p = np.asarray(ctx.obj(name).origin, dtype=float)
        return nom[name].copy() if off else p

    def joint(mech, name, nominal):
        v = float(ctx.obj(mech).joints[name])
        return nominal if off else v

    def step_to(arm, p):
        p = np.array(p, dtype=float)
        p[0] = min(p[0], X_GATE)
        yield from arm.move(p, dt, rot=rot)

    def grasp(arm, name):
        """Top grasp: come down over the bottle, trim wrist sag, centre the pads on the read pose, close; one retry."""
        b = base(name)
        g = np.array([b[0] + GRIP_X, b[1], b[2] + GRIP_H])
        yield from arm.move(g + [0.0, 0.0, 0.070], 2.5, _finger(OPEN_GAP), rot=rot)
        yield from arm.move(g, 2.0, rot=rot)
        yield from arm.trim_to_measured(g)
        for _ in range(2):
            yield from arm.grip(_finger(CLOSE_GAP), 0.8)
            yield from arm.hold(0.3)
            if arm.holding(HOLD_GAP):
                break
            yield from arm.grip(_finger(OPEN_GAP), 0.5)
            yield from arm.trim_to_measured(g)

    def carry(arm, name, goal, speed, tol=0.0015, steps=400, axes=(0, 1, 2), stall=None):
        """Move the held refill until its read pose reaches ``goal`` on ``axes``: each step the command moves by a
        clipped share of the error, so wrist sag and pad slip are cancelled. Returns the last error."""
        goal = np.asarray(goal, dtype=float)
        best, since = 1.0, 0
        err = np.zeros(3)
        for _ in range(steps):
            b = base(name)
            err = np.zeros(3)
            for a in axes:
                err[a] = goal[a] - b[a]
            n = float(np.linalg.norm(err))
            if n <= tol:
                break
            if n < best - 0.0003:
                best, since = n, 0
            else:
                since += 1
            if stall is not None and since >= stall:
                break
            d = 0.2 * err
            m = float(np.linalg.norm(d))
            if m > speed * dt:
                d *= speed * dt / m
            if off:
                nom[name] = nom[name] + d
            yield from step_to(arm, arm.pos + d)
        return err

    def left_bin():
        """Lift the spent refill out at the entrance, carry it over the side wall to the bin, lower, let go, park."""
        p = L.pos.copy()
        yield from L.move(p + [0.0, 0.0, LIFT_OUT], 2.0, rot=rot)
        yield from L.move((p[0], 0.110, p[2] + LIFT_OUT), 2.0, rot=rot)
        over = np.array([BIN_XY[0] + GRIP_X, BIN_XY[1], p[2] + LIFT_OUT])
        yield from L.move(over, 2.0, rot=rot)
        yield from L.move((over[0], over[1], TOP + BIN_DROP_Z + GRIP_H), 3.0, rot=rot)
        yield from L.grip(_finger(OPEN_GAP), 0.8)
        yield from L.hold(0.4)
        yield from L.move(over, 2.0, rot=rot)
        yield from L.home(3.0)
        nom["refill_spent"] = np.array([BIN_XY[0], BIN_XY[1], TOP + BIN_DROP_Z])

    def right_pick():
        yield from R.move((STAND_XY[0] + GRIP_X, STAND_XY[1], TOP + STAND_H + GRIP_H + 0.070), 3.0, _finger(OPEN_GAP),
                          rot=rot)
        yield from grasp(R, "refill_new")
        p = R.pos.copy()
        yield from R.move((p[0], p[1], p[2] + CARRY_Z - STAND_H), 3.5, rot=rot)
        nom["refill_new"][2] = TOP + CARRY_Z
        yield from R.move((ENTR_X + GRIP_X, p[1], p[2] + CARRY_Z - STAND_H), 2.0, rot=rot)
        nom["refill_new"][0] = ENTR_X

    # 1) Spent refill out (left): top grasp, draw it 43 mm forward to the keyhole entrance on the read pose.
    yield from L.move((0.300, 0.200, TOP + 0.345), 2.5, _finger(OPEN_GAP), rot=rot)
    yield from grasp(L, "refill_spent")
    yield from carry(L, "refill_spent", (ENTR_X, 0.0, 0.0), 0.015, axes=(0, 1), stall=40)
    yield from L.hold(0.3)

    # 2) Left bins the spent refill while the right picks the new one off its cradle and lifts it to carry height.
    yield from ctx.both(left_bin(), right_pick())
    p = R.pos.copy()
    yield from R.move((p[0], 0.0, p[2]), 3.0, rot=rot)
    nom["refill_new"][1] = 0.0
    # Over the entrance: centre on the read pose, then lower with the pose fed back until the base is on the ledge.
    yield from carry(R, "refill_new", (ENTR_X, 0.0, TOP + CARRY_Z), 0.02, tol=0.001, steps=80)
    yield from carry(R, "refill_new", (ENTR_X, 0.0, TOP + LEDGE + 0.030), 0.03, tol=0.001, steps=120)
    yield from carry(R, "refill_new", (ENTR_X, 0.0, TOP + LEDGE + 0.0005), 0.01, tol=0.001, steps=160, stall=30)

    # 3) Slide home (right, still holding): 43 mm back along the slot until the neck meets the slot end.
    for _ in range(2):
        yield from carry(R, "refill_new", (SEAT_X + 0.002, 0.0, 0.0), 0.01, tol=0.0025, steps=240, axes=(0, 1), stall=30)
        if off or base("refill_new")[0] >= SEAT_X - 0.003:
            break
    yield from R.grip(_finger(OPEN_GAP), 0.8)
    yield from R.hold(0.4)
    p = R.pos.copy()
    yield from R.move((p[0] - 0.050, 0.0, p[2] + 0.080), 2.0, rot=rot)
    yield from R.move((0.200, -0.150, TOP + 0.200), 2.5, FINGER_CLOSED, rot=rot)

    # 4) Shut (left): closed fingers push the open cover's face from the front, 21 cm from its hinge, past the detent's
    #    midpoint along the closing arc; the detent spring finishes. The hinge is read each step; one retry if it falls back open.
    yield from L.move((0.100, 0.200, TOP + 0.300), 2.5, FINGER_CLOSED, rot=PUSH_ROT)
    tip = 0.013 * (PUSH_ROT @ np.array([0.0, 0.0, 1.0]))  # the closed pad tips, 13 mm past the tool point

    def cover_tcp(q, lead, back=0.0):
        """Tool point that puts the pad tips on the cover's outer face 21 cm from the hinge, for hinge value q."""
        phi = max(COVER_SHUT - q - lead, -0.06)
        c = np.array([HINGE[0] - PUSH_S * math.sin(phi), 0.0, TOP + HINGE[2] + PUSH_S * math.cos(phi)])
        n = np.array([-math.cos(phi), 0.0, -math.sin(phi)])
        return c + n * (COVER_T + PUSH_OFF + back) - tip

    qn = 0.0
    for _ in range(2):
        yield from L.move(cover_tcp(0.0, 0.0, 0.040), 2.0, rot=PUSH_ROT)
        lead, q_last = PUSH_LEAD, 0.0
        for _ in range(360):
            q = joint("dispenser", "cover", qn)
            # the arm gives under load: aim further ahead while the hinge is not turning, back to 0.15 rad when it is
            lead = PUSH_LEAD if q - q_last > 0.002 else min(0.50, lead + 0.005)
            q_last = q
            if q >= COVER_SHUT / 2 + 0.10:  # past the detent's midpoint: let go, the spring finishes
                break
            qn = min(COVER_SHUT, qn + 0.006)
            d = cover_tcp(q, lead) - L.pos
            m = float(np.linalg.norm(d))
            if m > 0.03 * dt:
                d *= 0.03 * dt / m
            yield from L.move(L.pos + d, dt, rot=PUSH_ROT)
        yield from L.move(L.pos + [-0.070, 0.0, -0.030], 1.0, rot=PUSH_ROT)
        yield from L.hold(1.0)
        if off or joint("dispenser", "cover", 0.0) >= COVER_SHUT - math.radians(1.0):
            break
    yield from L.home(3.0)

    # 5) Test (right): closed fingertips push the bar in until the lever reads 0.105 rad, hold 1.5 s, release.
    yield from R.move((0.205, 0.0, TOP + 0.120), 2.0, rot=rot)
    yield from R.move((0.215, 0.0, TOP + BAR_Z), 1.5, rot=rot)
    x = 0.215
    while x < 0.285:
        if -joint("dispenser", "lever", 0.0) >= 0.105:
            break
        if off and x >= 0.250:
            break
        x += 0.010 * dt
        yield from R.move((x, 0.0, TOP + BAR_Z), dt, rot=rot)
    yield from R.hold(1.5)
    yield from R.move((0.200, 0.0, TOP + BAR_Z + 0.010), 1.5, rot=rot)
    yield from R.move((0.200, -0.150, TOP + 0.200), 1.5, rot=rot)
    yield from R.home(3.0)
    yield from ctx.wait(2.0)
    if not off:
        st = ctx.states()
        d, h, tilt = new_pose(st)
        print(f"SOAP end: neck {d * 1000:.1f} mm from the seat, base {h * 1000:+.1f} mm, tilt {tilt:.1f} deg, "
              f"cover {math.degrees(st['dispenser'].joints['cover']):.1f} deg, binned {spent_binned(st)}", flush=True)
