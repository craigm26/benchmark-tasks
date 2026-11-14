"""On the station table: a blue-grey processor shipping tray with a server processor in its black carrier lying loose
in the pocket (green substrate, silver lid, four sprung corner clips standing up at the ends), and to its left a finned
heat sink (copper base, aluminium fins, four steel corner nuts) on a dark stand, a quarter turn away from the carrier.
Gold triangles mark pin 1 on the heat sink label, the carrier and the tray corner.

Requires -E object_jitter=0 (fixed layout).

Run it:  inspect-robots run --embodiment yam_arena -E task=snap_a_heat_sink_onto_a_processor_carrier_pin_1_matched.py \\
           -E object_jitter=0 --instruction "Fit the heat sink onto the processor carrier in the tray, gold triangles
           together, and press till all four clips catch." --scorer task_score ...
"""

import dataclasses
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

import mechkit  # noqa: E402
from belimits import tilt_deg  # noqa: E402
from closedloop import grip_rot, placeholder_demo  # noqa: E402
from mechkit import Joint, Mech, box, cyl, hull, on_reset, released  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 8.0  # the demo grips: leave time to lift off and park after the first 1.0

TRAY_BLUE, BLACK, GREEN = (0.38, 0.48, 0.62), (0.05, 0.05, 0.06), (0.10, 0.50, 0.22)
SILVER, GOLD, COPPER = (0.80, 0.81, 0.82), (0.98, 0.76, 0.08), (0.85, 0.42, 0.16)
ALU, GAP, STEEL = (0.84, 0.86, 0.88), (0.20, 0.21, 0.23), (0.62, 0.64, 0.68)
WHITE, CLIP_GREY, STAND_GREY = (0.96, 0.96, 0.94), (0.16, 0.16, 0.18), (0.27, 0.28, 0.30)


def _mark(corner, leg, z, thick=0.0006):
    """A gold right triangle (look only): the right angle at ``corner`` (x, y), legs ``leg`` long toward -x and -y."""
    x, y = corner
    pts = [(px, py, pz) for px, py in ((x, y), (x - leg, y), (x, y - leg)) for pz in (0.0, thick)]
    return dataclasses.replace(hull((0.0, 0.0, z), pts, GOLD), collide=False)


# ---- the shipping tray (fixed): a slab with a pocket 70 x 127 mm, 6 mm deep ------------------------------------------
TRAY_XY = (0.275, -0.030)
TRAY_YAW = math.pi / 2  # tray and carrier lie long side along x: their own +y points toward the robot (world -x)
SLAB = (0.100, 0.147, 0.024)
POCKET, WALL_H = (0.070, 0.127), 0.006
SB = TOP + SLAB[2]  # the pocket floor
_wx, _wy = (SLAB[0] - POCKET[0]) / 2, (SLAB[1] - POCKET[1]) / 2
TRAY = Mech("tray", pos=(*TRAY_XY, TOP), yaw=TRAY_YAW, mass=None, friction=0.5, shapes=[
    box((0, 0, SLAB[2] / 2), SLAB, TRAY_BLUE),
    *[box((s * (POCKET[0] + _wx) / 2, 0, SLAB[2] + WALL_H / 2), (_wx, SLAB[1], WALL_H), TRAY_BLUE) for s in (-1, 1)],
    *[box((0, s * (POCKET[1] + _wy) / 2, SLAB[2] + WALL_H / 2), (POCKET[0], _wy, WALL_H), TRAY_BLUE) for s in (-1, 1)],
    _mark((0.048, 0.0715), 0.012, SLAB[2] + WALL_H),  # pin 1 mark on the tray corner (near left in the world)
])

# ---- the processor in its carrier (free, 0.13 kg with its clips), loose in the pocket ----------------------------------
# Carrier frame: origin at the bottom centre, on the pocket floor, y along the long side. Pin 1 is its (+x, +y) corner,
# which the quarter turn of the tray puts at the near left in the world.
FRAME = (0.064, 0.109, 0.007)
LID_TOP = 0.009  # the seat: the heat sink base rests on the processor lid
BASE = (0.078, 0.108, 0.006)  # heat sink base: x across, y along, thickness
CLIP_X, CLIP_Y, HINGE_Z = 0.024, 0.057, 0.002
ARM = (0.012, 0.005, 0.020)  # clip arm: its inner face 0.5 mm outside the heat sink base end
NOSE = 0.0035  # the nose stands this far in from the arm: 0.5 mm of gap, then 3.0 mm over the base
RAMP_DEG = 55.0  # the nose's top face, from the horizontal
OVERLAP = CLIP_Y - ARM[1] / 2 - NOSE
OVERLAP = BASE[1] / 2 - OVERLAP  # 3.0 mm of nose over the base end
_TAN = math.tan(math.radians(RAMP_DEG))
CATCH_Z = 0.016  # catch face, 1.0 mm above the seated base top (9 + 6 = 15 mm)
TIP_FLAT = 0.0003
RAMP_Z = CATCH_Z + TIP_FLAT + OVERLAP * _TAN  # carrier z at which the base's lower edge first meets the ramps
CLIP_OPEN, CLIP_SPRING, CLIP_MASS = 0.40, 0.45, 0.020
CLIPS = {"clip_near_left": (1, 1), "clip_far_left": (1, -1), "clip_near_right": (-1, 1), "clip_far_right": (-1, -1)}


def _clip(name, sx, sy):
    """One corner clip: an arm hinged at its foot, sprung shut against its stop, with a nose whose flat underside is the
    catch face and whose top is a 55 degree ramp. It opens outward (away from the heat sink)."""
    yin, ytip = -sy * ARM[1] / 2, -sy * (ARM[1] / 2 + NOSE)
    zc = CATCH_Z - HINGE_Z
    nose = [(sxx * ARM[0] / 2, y, z) for sxx in (-1, 1)
            for y, z in ((yin, zc), (ytip, zc), (ytip, zc + TIP_FLAT), (yin, zc + TIP_FLAT + NOSE * _TAN))]
    return Joint(name, "hinge", axis="x", at=(sx * CLIP_X, sy * CLIP_Y, HINGE_Z),
                 low=-CLIP_OPEN if sy > 0 else 0.0, high=0.0 if sy > 0 else CLIP_OPEN,
                 spring=CLIP_SPRING, damping=0.01, mass=CLIP_MASS, friction=0.1,
                 shapes=[box((0, 0, ARM[2] / 2), ARM, CLIP_GREY), hull((0, 0, 0), nose, BLACK)])


CARRIER = Mech("carrier", pos=(*TRAY_XY, SB), yaw=TRAY_YAW, mass=0.050, friction=0.5, shapes=[
    box((0, 0, FRAME[2] / 2), FRAME, BLACK),  # carrier frame
    box((0, 0, 0.004), (0.0565, 0.0775, 0.008), GREEN),  # processor substrate, 1 mm proud of the frame
    box((0, 0, LID_TOP / 2), (0.050, 0.064, LID_TOP), SILVER),  # processor lid (heat spreader), 1 mm proud again
    _mark((0.028, 0.0385), 0.014, 0.008),  # pin 1 mark on the substrate's pin 1 corner
], joints=[_clip(n, sx, sy) for n, (sx, sy) in CLIPS.items()])

# ---- the heat sink (free, 0.45 kg). Frame: origin at the centre of the base underside, y along the long side. ---------
FIN_H, FIN_X, FIN_Y, NOTCH = 0.019, 0.078, 0.092, (0.014, 0.009)
HS_H = BASE[2] + FIN_H  # 25 mm to the fin tops
NUT_XY, NUT_R, NUT_H = (0.032, 0.042), 0.0045, 0.021  # four captive corner nuts, their tops 2 mm above the fins
NUT_TOP = BASE[2] + NUT_H
_mid_y, _end_x = FIN_Y - 2 * NOTCH[1], FIN_X - 2 * NOTCH[0]
_fz = BASE[2] + FIN_H / 2
_fin_x = [-0.038 + i * 0.076 / 12 for i in range(13)]
HS_SHAPES = [
    box((0, 0, BASE[2] / 2), BASE, COPPER),
    # the fin stack's collider: one block, notched at the four corners for the nuts (the drawn fins do not collide)
    dataclasses.replace(box((0, 0, _fz), (FIN_X, _mid_y, FIN_H), ALU), visible=False),
    *[dataclasses.replace(box((0, s * (FIN_Y - NOTCH[1]) / 2, _fz), (_end_x, NOTCH[1], FIN_H), ALU), visible=False)
      for s in (-1, 1)],
    *[cyl((sx * NUT_XY[0], sy * NUT_XY[1], BASE[2] + NUT_H / 2), NUT_R, NUT_H, STEEL) for sx in (-1, 1) for sy in (-1, 1)],
    # what shows: 13 aluminium fins 2 mm thick at 6.3 mm pitch along the long side, over a dark core
    dataclasses.replace(box((0, 0, BASE[2] + 0.008), (FIN_X - 0.004, _mid_y - 0.001, 0.016), GAP), collide=False),
    *[dataclasses.replace(box((0, s * (FIN_Y - NOTCH[1]) / 2, BASE[2] + 0.008), (_end_x - 0.002, NOTCH[1], 0.016), GAP),
                          collide=False) for s in (-1, 1)],
    *[dataclasses.replace(box((x, 0, _fz), (0.002, FIN_Y if abs(x) < _end_x / 2 else _mid_y, FIN_H), ALU), collide=False)
      for x in _fin_x],
    dataclasses.replace(box((0.006, 0.010, HS_H + 0.0003), (0.036, 0.050, 0.0006), WHITE), collide=False),  # label
    _mark((0.023, 0.034), 0.020, HS_H + 0.0006),  # pin 1 mark on the label, toward the (+x, +y) corner
    _mark((0.016, 0.0535), 0.007, BASE[2]),  # pin 1 mark on the base ledge at that end
]
STAND_XY, STAND = (0.275, 0.135), (0.090, 0.120, 0.030)
STAND_BOX = Box("stand", pos=(*STAND_XY, TOP + STAND[2] / 2), size=STAND, color=STAND_GREY)
HS_YAW0 = 0.0  # long side along y, pin 1 at the far left: a quarter turn counterclockwise (from above) is needed
HEAT_SINK = Mech("heat_sink", pos=(*STAND_XY, TOP + STAND[2]), yaw=HS_YAW0, mass=0.45, friction=0.5, shapes=HS_SHAPES)

OBJECTS = [TRAY, STAND_BOX, CARRIER, HEAT_SINK]

# ---- scoring ----------------------------------------------------------------------------------------------------------
YAW_TOL = 5.0  # degrees
SHUT, SEAT_TOL, FLAT_DEG = 0.05, 0.001, 1.5
C_LIFT, C_TILT = 0.005, 10.0
D_HIGH, D_TILT = 0.015, 15.0
RELEASE_CLEAR, REST_STEPS = 0.03, 20
OFFLINE_FINGERS_CLEAR = True  # what the end-of-run release test returns with no simulator (the checker flips it)
OFFLINE_LET_GO = None  # with no simulator the drop latch has no finger points to test; the checker sets True or False

_L = {"R": False, "C": False, "D": False, "still": 0, "seen": None}


@on_reset
def _clear(_env=None):
    _L.update(R=False, C=False, D=False, still=0, seen=None)


def rel_yaw_deg(hs, ca) -> float:
    """The heat sink's turn about the vertical relative to the carrier, degrees in -180..180 (0 is pin 1 to pin 1)."""
    r = ca.rotation().T @ hs.rotation()
    return math.degrees(math.atan2(r[1, 0], r[0, 0]))


def matched(hs, ca) -> bool:
    return abs(rel_yaw_deg(hs, ca)) <= YAW_TOL


def reversed_fit(hs, ca) -> bool:
    return abs(abs(rel_yaw_deg(hs, ca)) - 180.0) <= YAW_TOL


def base_height(hs, ca, sx, sy) -> float:
    """Height of the heat sink base underside above the processor lid at the carrier corner (sx, sy), m. The corner is
    taken in the carrier's frame, so a reversed heat sink is measured at the clip that actually holds it."""
    turn = -1.0 if reversed_fit(hs, ca) else 1.0
    p = ca.to_local(hs.to_world((turn * sx * CLIP_X, turn * sy * (BASE[1] / 2 - 0.004), 0.0)))
    return float(p[2] - LID_TOP)


def caught(hs, ca, name) -> bool:
    """This clip is shut on its stop and the base underside under it is down on the lid: its nose is over the base."""
    sx, sy = CLIPS[name]
    p = ca.to_local(hs.to_world((0.0, 0.0, 0.0)))
    over = abs(p[0]) <= 0.006 and abs(p[1]) <= 0.004  # the base is on the carrier, not beside it
    return bool(over and abs(ca.joints[name]) <= SHUT and abs(base_height(hs, ca, sx, sy)) <= SEAT_TOL)


def n_caught(hs, ca) -> int:
    return sum(caught(hs, ca, n) for n in CLIPS)


def flat_deg(hs, ca) -> float:
    c = float((ca.rotation()[:, 2]) @ (hs.rotation()[:, 2]))
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


def _fingers_clear(hs, ca) -> bool:
    if mechkit._ENV["env"] is None:
        return OFFLINE_FINGERS_CLEAR
    return released(hs, RELEASE_CLEAR) and released(ca, RELEASE_CLEAR)


def _let_go(hs):
    """True when no finger point is within 3 cm of the heat sink, False when one is, None when it cannot be told."""
    if mechkit._ENV["env"] is None:
        return OFFLINE_LET_GO
    return released(hs, RELEASE_CLEAR)


def _update(objects) -> None:
    """Latch what happened this control step. yam_arena calls success() and score() on the same dict each step, so a
    dict already seen changes nothing."""
    if objects is _L["seen"]:
        return
    _L["seen"] = objects
    hs, ca = objects["heat_sink"], objects["carrier"]
    if reversed_fit(hs, ca) and n_caught(hs, ca) > 0:
        _L["R"] = True
    if ca.origin[2] - SB > C_LIFT or tilt_deg(ca) > C_TILT:
        _L["C"] = True
    if _let_go(hs) and (hs.origin[2] - (ca.origin[2] + LID_TOP) > D_HIGH or tilt_deg(hs) > D_TILT):
        _L["D"] = True
    _L["still"] = _L["still"] + 1 if (hs.resting() and ca.resting()) else 0


def score(objects):
    """0 if a clip ever caught a reversed heat sink (R), the carrier was ever lifted 5 mm off the pocket floor or
    tilted 10 degrees (C), or the heat sink was ever let go more than 15 mm above the lid or tilted over 15 degrees (D).
    Otherwise, with the pin 1 corners matched within 5 degrees, both grippers 3 cm clear and everything still for 1 s:
    0.25 per clip caught, the fourth quarter only if the heat sink is flat on the lid within 1.5 degrees."""
    _update(objects)
    hs, ca = objects["heat_sink"], objects["carrier"]
    if _L["R"] or _L["C"] or _L["D"]:
        return 0.0
    if not matched(hs, ca) or _L["still"] < REST_STEPS or not _fingers_clear(hs, ca):
        return 0.0
    n = n_caught(hs, ca)
    if n == 4 and flat_deg(hs, ca) > FLAT_DEG:
        return 0.75
    return 0.25 * n


def success(objects):
    return score(objects) == 1.0


# ---- the robot keeps its hands out of the tray pocket ------------------------------------------------------------------
# The pocket holds the processor and its carrier: the hands work on the heat sink only. No arm point may go into the
# pocket below the level of the seated heat sink's base top plus 6 mm.
POCKET_X = (TRAY_XY[0] - POCKET[1] / 2, TRAY_XY[0] + POCKET[1] / 2)
POCKET_Y = (TRAY_XY[1] - POCKET[0] / 2, TRAY_XY[1] + POCKET[0] / 2)
POCKET_TOP = SB + LID_TOP + BASE[2] + 0.006  # TOP + 0.045
ARM_KEEP_OUT = [{"name": "tray pocket (processor and carrier)", "x": POCKET_X, "y": POCKET_Y, "z_top": POCKET_TOP}]

# ---- the scripted demo ------------------------------------------------------------------------------------------------
Z3 = np.array([0.0, 0.0, 1.0])
PICK_YAW, SEAT_YAW = -math.pi / 2, 0.0  # jaws along x at the pick; a quarter turn counterclockwise later, along y
PICK_ROT, SEAT_ROT = grip_rot(yaw=PICK_YAW), grip_rot(yaw=SEAT_YAW)
GRIP_UP = 0.010 + 0.013  # tool point above the base underside: pad tips 4 mm above the base, on the outer fins
CARRY_Z = TOP + 0.135
LEAN = math.radians(20.0)  # each pressing hand leans outward so the two wrists clear each other
PRESS_YAW = 0.0
OVERDRIVE = 0.022  # how far past the seated height the press command may go (the arm is a spring: about 1 N/mm)
demo = placeholder_demo


def _rx(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


PRESS_ROT = {"left": _rx(-LEAN) @ grip_rot(yaw=PRESS_YAW), "right": _rx(LEAN) @ grip_rot(yaw=PRESS_YAW)}


def _finger(gap: float) -> float:
    """Finger joint target for a pad gap in metres (94 mm is fully open)."""
    return FINGER_OPEN * gap / (2.0 * abs(FINGER_OPEN))


def _nominal(seated: bool):
    """Where the heat sink is by plan (origin, rotation): resting on the four ramps, or seated on the lid."""
    z = SB + (LID_TOP if seated else RAMP_Z)
    return np.array([TRAY_XY[0], TRAY_XY[1], z]), mechkit._axis_matrix("z", TRAY_YAW)


def _hs_pose(ctx, seated=False):
    if ctx.left.offline:
        return _nominal(seated)
    hs = ctx.obj("heat_sink")
    return np.asarray(hs.origin, dtype=float), hs.rotation()


def _clips_caught(ctx, names) -> bool:
    if ctx.left.offline:
        return False
    hs, ca = ctx.obj("heat_sink"), ctx.obj("carrier")
    return all(caught(hs, ca, n) for n in names)


def _press(ctx, jobs):
    """Closed fingertips of one or both hands press down on corner nuts until those corners' clips catch.
    jobs: {arm: clip name}. Each tool point starts 10 mm above its nut, leaned outward."""
    arms = {"left": ctx.left, "right": ctx.right}
    o, r = _hs_pose(ctx)
    start = {}
    for side, name in jobs.items():
        sx, sy = CLIPS[name]
        nut = o + r @ np.array([sx * NUT_XY[0], sy * NUT_XY[1], NUT_TOP])
        start[side] = nut - PRESS_ROT[side][:, 2] * 0.013 + Z3 * 0.010
    gens = {s: arms[s].move(start[s] + Z3 * 0.030, 2.8, FINGER_CLOSED, rot=PRESS_ROT[s]) for s in jobs}
    yield from ctx.both(gens.get("left", iter(())), gens.get("right", iter(())))
    gens = {s: arms[s].trim_to_measured(start[s] + Z3 * 0.030) for s in jobs}
    yield from ctx.both(gens.get("left", iter(())), gens.get("right", iter(())))
    trim = {s: arms[s].pos - (start[s] + Z3 * 0.030) for s in jobs}
    gens = {s: arms[s].move(start[s] + trim[s], 0.8, rot=PRESS_ROT[s]) for s in jobs}
    yield from ctx.both(gens.get("left", iter(())), gens.get("right", iter(())))
    travel = 0.010 + (RAMP_Z - LID_TOP)  # down to the seated nut top
    reach = travel + (0.002 if ctx.left.offline else OVERDRIVE)
    depth = 0.0
    while depth < reach:
        depth = min(reach, depth + 0.0006)  # 6 mm/s
        gens = {s: arms[s].move(start[s] + trim[s] - Z3 * depth, 0.1, rot=PRESS_ROT[s]) for s in jobs}
        yield from ctx.both(gens.get("left", iter(())), gens.get("right", iter(())))
        if depth > 0.010 and _clips_caught(ctx, list(jobs.values())):
            break
    yield from ctx.wait(0.3)
    gens = {s: arms[s].move(start[s] + trim[s] + Z3 * 0.030, 1.0, rot=PRESS_ROT[s]) for s in jobs}
    yield from ctx.both(gens.get("left", iter(())), gens.get("right", iter(())))


def demo_script(ctx):
    L, R = ctx.left, ctx.right

    # 1) Pick: the left hand takes the heat sink across its 78 mm of fins, jaws along x, 8 mm to spare a side.
    o, _ = (np.array([STAND_XY[0], STAND_XY[1], TOP + STAND[2]]), None) if L.offline else _hs_pose(ctx)
    grip = o + Z3 * GRIP_UP
    yield from L.move(grip + Z3 * 0.08, 2.5, FINGER_OPEN, rot=PICK_ROT)
    yield from L.trim_to_measured(grip + Z3 * 0.08)
    trim = L.pos - (grip + Z3 * 0.08)
    yield from L.move(grip + trim, 1.5, rot=PICK_ROT)
    yield from L.trim_to_measured(grip, max_duration=0.6)
    yield from L.grip(_finger(0.064), 0.8)
    yield from L.hold(0.3)
    if not L.holding(0.074):  # the pads did not stop on the 78 mm fin stack: reopen, recentre, try once more
        yield from L.grip(FINGER_OPEN, 0.5)
        o, _ = (o, None) if L.offline else _hs_pose(ctx)
        yield from L.move(o + Z3 * GRIP_UP + trim, 0.6, rot=PICK_ROT)
        yield from L.grip(_finger(0.064), 0.8)
        yield from L.hold(0.3)

    # 2) Turn and carry: lift, and on the way over the tray turn the wrist a quarter turn counterclockwise (seen
    # from above), so the long side lies along x and the gold triangles are at the same corner.
    lift = np.array([grip[0], grip[1], CARRY_Z]) + trim
    yield from L.move(lift, 1.5, rot=PICK_ROT)
    over = np.array([TRAY_XY[0], TRAY_XY[1], CARRY_Z])
    yield from L.move(over, 3.5, rot=SEAT_ROT)
    yield from L.hold(0.5)

    # 3) Lower: bring the heat sink's centre over the carrier's and its yaw onto the carrier's (both read from the
    # simulator), come down until the base is 1 mm above the four ramps, set it on them and let go.
    yaw_fix, target = 0.0, over.copy()
    for z_above, secs in ((0.040, 1.2), (0.012, 1.5), (0.001, 1.2)):
        target[2] = SB + RAMP_Z + z_above + GRIP_UP
        if not L.offline:
            hs, ca = ctx.obj("heat_sink"), ctx.obj("carrier")
            err = np.asarray(ca.origin, dtype=float) - np.asarray(hs.origin, dtype=float)
            err[2] = (ca.origin[2] + RAMP_Z + z_above) - hs.origin[2]
            target = L.pos + np.clip(err, -0.03, 0.03)
            yaw_fix -= math.radians(max(-10.0, min(10.0, rel_yaw_deg(hs, ca))))
        yield from L.move(target, secs, rot=grip_rot(yaw=SEAT_YAW + yaw_fix))
        yield from L.hold(0.4)
    rot = grip_rot(yaw=SEAT_YAW + yaw_fix)
    yield from L.move(L.pos - Z3 * 0.003, 0.8, rot=rot)  # onto the ramps
    yield from L.grip(FINGER_OPEN, 0.8)
    yield from L.move(L.pos + Z3 * 0.06, 1.2, rot=rot)
    yield from ctx.wait(0.5)

    # 4) and 5) Press: closed fingertips on the corner nuts, one diagonal pair of corners, then the other.
    yield from _press(ctx, {"left": "clip_far_left", "right": "clip_near_right"})
    yield from _press(ctx, {"left": "clip_near_left", "right": "clip_far_right"})
    for _ in range(2):  # any corner not caught is pressed once more, by the hand on its side
        if L.offline:
            break
        hs, ca = ctx.obj("heat_sink"), ctx.obj("carrier")
        todo = [n for n in CLIPS if not caught(hs, ca, n)]
        if not todo:
            break
        for n in todo:
            yield from _press(ctx, {"left" if CLIPS[n][0] > 0 else "right": n})

    # 6) Park: both hands up and back to the ready pose, grippers open.
    yield from ctx.both(L.move(L.pos + Z3 * 0.05, 1.0, rot=L.rot), R.move(R.pos + Z3 * 0.05, 1.0, rot=R.rot))
    yield from ctx.both(L.home(), R.home())
    yield from ctx.wait(1.0)
