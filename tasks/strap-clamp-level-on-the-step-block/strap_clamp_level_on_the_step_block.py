"""On the station table: a small mill table (a dark grey plate with two stops) holds a blue 6 x 6 x 4 cm part against the
stops. Beside it stands a threaded stud. On the stud, from the bottom: a lift spring with a spring washer (it holds a strap
off the part until the nut is tightened), a brass thrust washer that cannot turn, and an orange nut with a yellow handle.
An overhead mill arm with a black cutter hangs over the part. On the bare table lie a red strap clamp (a slotted bar with
a white grip tab) in front of the right arm, and a yellow step block (four treads, 2 to 5 cm tall) in front of the left
arm. The strap, the block and the part are loose; the mill (plate, stops, stud, arm) is fixed except the nut, which
turns, the thrust washer, which the nut's thread drives down, and the spring washer.

The thread is a helix worked by contact, not a slide: a lug under the nut climbs a helical ramp on the thrust washer, so
turning the nut clockwise wedges the washer down, and the washer cannot push the nut back. It is coarse (150 degrees of
turn give 2.4 cm, where an M12 thread gives 1.75 mm a turn) and cut as 80 flat steps of 0.34 mm. The washer hangs 2 mm
below the lug, so the nut turns about 15 degrees before the thread takes up. The thread is slippery (friction 0.15), so
a lug left part way up a step's edge settles back onto the flat below it, at most 0.34 mm, and rests there.

Requires -E object_jitter=0.

Run it:  inspect-robots run --embodiment yam_arena -E task=strap_clamp_level_on_the_step_block.py -E object_jitter=0 \\
           --instruction "Clamp the blue part to the mill table with the red strap clamp." --scorer task_score ...
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402
from belimits import PeakTracker, ResetOnJumpTracker, tilt_deg  # noqa: E402,F401
from closedloop import placeholder_demo  # noqa: E402
from mechkit import Mech, Joint, box, cyl, on_reset  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 12.0  # the demo goes on tightening for about 7 s after the clamp first scores, then lets go and parks
SCORE_OVER = "end"

DARK, GREY, LIGHT, BLACK = (0.22, 0.22, 0.25), (0.55, 0.55, 0.58), (0.78, 0.78, 0.80), (0.07, 0.07, 0.08)
RED, WHITE, YELLOW, ORANGE, BLUE = (0.85, 0.10, 0.10), (0.97, 0.97, 0.97), (0.96, 0.80, 0.08), (0.96, 0.45, 0.04), (0.12, 0.28, 0.85)

PLATE_T = 0.01  # the mill table plate stands 1 cm proud of the table
STUD_XY = (0.27, -0.02)
CUT_XY = (0.20, -0.02)

# ---- the mill: fixed. Its frame is at the stud's foot on the table top. -----------------------------------------------
BRASS = (0.80, 0.62, 0.20)

# Lift spring: a spring washer on the stud that stands 4 mm above the part's top and gives 1.5 N per mm (6 N with the
# strap seated on the part).
LIFT_REST = 0.054  # top of the spring washer above TOP, unloaded
LIFT_K = 1500.0  # N/m
LIFT_LOW = -0.008

# Thrust washer: slides on the stud, cannot turn. It hangs on a light spring with its underside 8.1 cm above TOP.
WASH_Z0 = 0.081
WASH_T = 0.004  # plate thickness
WASH_K, WASH_M = 100.0, 0.02  # a light spring (N/m) carries the 20 g washer up under the nut's lug
WASH_SAG = WASH_M * 9.81 / WASH_K  # the washer hangs this far below the spring's free length (2 mm)
WASH_SPAWN = WASH_Z0 + WASH_SAG  # the simulator starts the washer at the spring's free length, 8.3 cm, before it settles
# The thread: a stepped helical ramp on the washer, under a lug on the nut. 150 degrees of turn drive the washer down 2.4 cm.
TURN_MAX = math.radians(150.0)
THREAD_TRAVEL = 0.024
LEAD = THREAD_TRAVEL / TURN_MAX  # metres per radian (5.8 cm per turn)
RAMP_R, RAMP_W = 0.026, 0.012  # mean radius and radial width of the ramp
RAMP_T0 = 0.004  # ramp top above the washer plate under the lug at rest
LUG_A0 = math.radians(-105.0)  # where the lug starts, seen from above (0 = +x, counter-clockwise positive)
LUG_W = 0.006  # the lug's width across the ramp
LUG_CLEAR = 0.003  # the lug's post stops this far above its lowest edge: only the swept toe ever meets the ramp
LUG_GAP = 0.001  # the lug's lowest edge above the ramp with the washer at WASH_SPAWN: nothing overlaps when the scene is built
# Friction of the nut and the washer (an oiled thread). It is below tan 25 deg = 0.47 on purpose: the lug cannot hang on a
# step's edge by friction, where it would creep; it slides back onto the flat below and rests there.
THREAD_MU = 0.15
WASH_LOW = -(THREAD_TRAVEL + 0.0005)  # from WASH_SPAWN. Lowest: 5.85 cm above TOP, still above the spring washer (5.4)

# Nut: turns on the stud, clockwise (seen from above) to tighten. Its frame is the underside of its arms.
NUT_Z = 0.118
HANDLE_A0 = math.radians(80.0)  # the handle starts pointing to the robot's left and a little forward
HANDLE_R, HANDLE_RAD, HANDLE_H = 0.055, 0.009, 0.028
HANDLE_DROP = 0.018  # the handle hangs this far below the nut's arms, outside the washer
ARM_T = 0.010


def _l(x, y):  # world xy -> mill frame
    return (x - STUD_XY[0], y - STUD_XY[1])


def _qz(a):
    return (math.cos(a / 2), 0.0, 0.0, math.sin(a / 2))


def _qzy(a, b):  # turn by b about y, then by a about z; (w, x, y, z)
    cz, sz, cy, sy = math.cos(a / 2), math.sin(a / 2), math.cos(b / 2), math.sin(b / 2)
    return (cz * cy, -sz * sy, cz * sy, sz * cy)


RAMP_STEPS = 80  # the helix is cut as 80 flat steps, 2.125 degrees and 0.34 mm each: the lug rests on a flat, so it holds
RAMP_STEP = math.radians(2.125)
RAMP_LEAD_IN = 4  # steps before the one the lug starts on


def _step_top(k):
    """Height of step k's top above the washer's underside (steps rise clockwise; the lug starts on step RAMP_LEAD_IN)."""
    return WASH_T + RAMP_T0 + LEAD * RAMP_STEP * (k - RAMP_LEAD_IN)


def _washer_shapes():
    shapes = [
        box((0, 0, WASH_T / 2), (0.034, 0.042, WASH_T), BRASS),  # the pad that bears on the strap
        box((-0.016, 0, WASH_T / 2), (0.034, 0.066, WASH_T), BRASS),  # carries the ramp, on the side away from the strap's tab
    ]
    for k in range(RAMP_STEPS):
        psi = LUG_A0 - (k - RAMP_LEAD_IN) * RAMP_STEP  # the step's middle; the lug starts over step RAMP_LEAD_IN
        h = _step_top(k) - WASH_T
        shapes.append(box((RAMP_R * math.cos(psi), RAMP_R * math.sin(psi), WASH_T + h / 2), (0.0014, RAMP_W, h), BRASS,
                          tilt=_qz(psi - math.pi / 2)))
    return tuple(shapes)


def _nut_shapes():
    # The lug's lowest edge clears the ramp with the washer at its highest (WASH_SPAWN), so no shape of the nut starts inside
    # a step. Hanging, the washer is 2 mm lower, and the nut turns about 15 degrees before the lug meets the ramp.
    lug_bottom = WASH_SPAWN + WASH_T + RAMP_T0 + LUG_GAP - NUT_Z
    ca, sa = math.cos(LUG_A0), math.sin(LUG_A0)
    ch, sh = math.cos(HANDLE_A0), math.sin(HANDLE_A0)
    cw = np.array([sa, -ca, 0.0])  # clockwise along the ring at the lug
    apex = np.array([RAMP_R * ca, RAMP_R * sa, lug_bottom])
    # The lug meets the ramp only with its toe, which is swept up 25 degrees ahead of its lowest edge: the swept face
    # climbs each step's edge (wedging the washer down) and the lowest edge then rests on the step's flat top. The post
    # above it stops LUG_CLEAR short, so no upright face of the lug can come up against the upright face of a step.
    phi, sweep, tip_l, tip_t = LUG_A0 - math.pi / 2, math.radians(25.0), 0.006, 0.004
    along = np.array([cw[0] * math.cos(sweep), cw[1] * math.cos(sweep), math.sin(sweep)])
    up = np.array([-cw[0] * math.sin(sweep), -cw[1] * math.sin(sweep), math.cos(sweep)])
    post_h = -(lug_bottom + LUG_CLEAR)
    body = apex - cw * 0.0025 + np.array([0.0, 0.0, LUG_CLEAR + post_h / 2])
    return (
        cyl((0, 0, 0.008), 0.013, 0.016, ORANGE),  # the nut's body on the stud
        box((0.016 * ca, 0.016 * sa, ARM_T / 2), (0.032, 0.010, ARM_T), ORANGE, tilt=_qz(LUG_A0)),  # arm out to the lug
        box(tuple(body), (LUG_W, 0.005, post_h), ORANGE, tilt=_qz(LUG_A0)),  # the lug's post
        box(tuple(apex + along * tip_l / 2 + up * tip_t / 2), (tip_l, LUG_W, tip_t), ORANGE, tilt=_qzy(phi, -sweep)),  # its swept-up toe
        box((HANDLE_R / 2 * ch, HANDLE_R / 2 * sh, ARM_T / 2), (HANDLE_R, 0.016, ARM_T), ORANGE, tilt=_qz(HANDLE_A0)),  # handle arm
        cyl((HANDLE_R * ch, HANDLE_R * sh, HANDLE_H / 2 - HANDLE_DROP), HANDLE_RAD, HANDLE_H, YELLOW),  # handle: 1.8 cm across, 2.8 cm tall
    )


MILL = Mech(
    "mill",
    pos=(STUD_XY[0], STUD_XY[1], TOP),
    mass=None,
    shapes=[
        box((0, 0, PLATE_T / 2), (0.22, 0.16, PLATE_T), DARK),  # plate x 0.16..0.38, y -0.10..0.06
        box((*_l(0.167, -0.02), PLATE_T + 0.0125), (0.01, 0.06, 0.025), GREY),  # stop on the part's -x face
        box((*_l(0.202, -0.055), PLATE_T + 0.0125), (0.06, 0.01, 0.025), GREY),  # stop on the part's -y face
        cyl((0, 0, PLATE_T + 0.06), 0.006, 0.12, LIGHT),  # stud: 1.2 cm across, top 13 cm up
        cyl((*_l(*CUT_XY), 0.16), 0.008, 0.08, BLACK),  # end mill, z 0.12..0.20
        box((*_l(0.155, -0.02), 0.2075), (0.11, 0.02, 0.015), GREY),  # overhead arm holding the cutter
        box((*_l(0.115, -0.02), 0.1075), (0.03, 0.03, 0.215), GREY),  # its column, in front of the arms
    ],
    joints=[
        Joint("nut", "hinge", axis="z", at=(0, 0, NUT_Z), low=-TURN_MAX, high=0.0, spring=0.0, damping=0.5, mass=0.06,
              friction=THREAD_MU, shapes=_nut_shapes()),
        Joint("washer", "slide", axis="z", at=(0, 0, WASH_SPAWN), low=WASH_LOW, high=0.0, spring=WASH_K, damping=2.0,
              mass=WASH_M, friction=THREAD_MU, shapes=_washer_shapes()),
        Joint("lift", "slide", axis="z", at=(0, 0, LIFT_REST), low=LIFT_LOW, high=0.0, spring=LIFT_K, damping=30.0,
              mass=0.04, shapes=(
                  box((-0.005, 0, -0.002), (0.020, 0.044, 0.004), LIGHT),  # spring washer the strap sits on, x 0.255..0.275
                  cyl((0, 0, -0.012), 0.009, 0.016, LIGHT),  # the spring's sleeve round the stud
              )),
    ],
    friction=0.8,
)

# ---- the part: a blue 6 x 6 x 4 cm block against both stops --------------------------------------------------------------
PART_POS = (0.202, -0.02, TOP + PLATE_T + 0.02)
PART = Box("blue_part", pos=PART_POS, size=(0.06, 0.06, 0.04), mass=0.39, color=BLUE)  # aluminium
PART_TOP = TOP + PLATE_T + 0.04

# ---- the strap: 11.5 x 4.2 x 1.2 cm, slot 1.8 wide and 7.0 deep open at the toe (-x) end; frame at the bar's centre -------
SL, SW, ST = 0.115, 0.042, 0.012
SLOT_W, SLOT_D = 0.018, 0.07
TAB_X = SL / 2 - 0.01 - 0.009  # tab centre, 1 cm from the heel end
STRAP_SPAWN = (0.30, -0.17, TOP + ST / 2)
_prong_w = (SW - SLOT_W) / 2
STRAP = Mech(
    "strap",
    pos=STRAP_SPAWN,
    mass=0.115,  # aluminium bar less its slot
    shapes=[
        box((-SL / 2 + SLOT_D + (SL - SLOT_D) / 2 - 0.0, 0, 0), (SL - SLOT_D, SW, ST), RED),  # solid heel end
        box((-SL / 2 + SLOT_D / 2, SLOT_W / 2 + _prong_w / 2, 0), (SLOT_D, _prong_w, ST), RED),
        box((-SL / 2 + SLOT_D / 2, -SLOT_W / 2 - _prong_w / 2, 0), (SLOT_D, _prong_w, ST), RED),
        box((TAB_X, 0, ST / 2 + 0.015), (0.018, 0.030, 0.030), WHITE),  # grip tab
    ],
    friction=0.8,
)

# ---- the step block: four 2 cm treads, 2 to 5 cm tall, rising in +x; frame at the middle of its underside ---------------
TREAD_H = (0.02, 0.03, 0.04, 0.05)
TREAD_X = tuple(-0.03 + 0.02 * k for k in range(4))  # tread centres in the block frame
BLOCK_SPAWN = (0.24, 0.15, TOP)
BLOCK = Mech(
    "step_block",
    pos=BLOCK_SPAWN,
    mass=0.12,  # ABS plastic
    shapes=[box((TREAD_X[k], 0, TREAD_H[k] / 2), (0.02, 0.04, TREAD_H[k]), YELLOW) for k in range(4)],
    friction=0.8,
)

OBJECTS = [MILL, PART, STRAP, BLOCK]

demo = placeholder_demo

SNUG = 0.0037  # the demo tightens until the lift spring is pressed down this far (5.55 N)
CARRY_UNDER = 0.065  # the strap's underside above TOP while it is carried in
BLOCK_X, BLOCK_Y = 0.322, -0.02  # block frame (middle of its underside) when set on the plate

# ---- scoring ------------------------------------------------------------------------------------------------------------
PART_MOVE_MAX = 0.003  # part within 3 mm of its start
TOE_OVERLAP = (0.010, 0.025)  # toe on the part, 1.0 to 2.5 cm
LEVEL_TOL = 0.002  # clamped: toe underside within 2 mm of the part's top
HEEL_RISE = (-0.0015, 0.004)  # heel end 0 to 4 mm higher than the toe end (1.5 mm for sag in the contacts)
TREAD_MATCH = (-0.0015, 0.004)  # the heel's tread top is 0 to 4 mm above the part's top (same allowance)
LEVEL_TILT = 3.0
SET_TOE_UP = 0.010  # set but not clamped: the lift spring holds the toe up to 1 cm above the part
STAGE1_TILT = 15.0
WASHER_GAP = 0.001  # the thrust washer bears on the strap: gap at most 1 mm
# Held: the lift spring is pressed down at least 3 mm, which is 4.5 N. The strap's own weight presses it about 1 mm, so
# the nut presses the strap with 3.8 N or more (moments about the heel: 4.5 N x 7.0 cm less 1.1 N x 5.75 cm, over 6.5 cm).
# Against a pull across the strap that is 3.8 N x 0.8 on its top face and 5.0 N x 0.8 underneath, 7 N; with the top face
# at 0.475 (midway between the washer's 0.15 and the strap's 0.8) it is still 5.8 N. This is an estimate that stands in for
# the 5 N pull test, which the simulator does not run.
CLAMP_MIN = 0.003
# The nut did it: it has turned at least 110 degrees, and the thrust washer is as far down as that angle puts it, within
# 3 mm. The washer rests up against the nut's lug on its own spring, so only something pushing it down can put it lower
# than the thread does; a washer pushed onto the strap by a gripper has no matching nut angle.
NUT_MIN = math.radians(110.0)
THREAD_TOL = 0.003
AIR = 0.01

_tracker = ResetOnJumpTracker({"blue_part": PART_POS})


@on_reset
def _clear(_env=None):
    _tracker.clear()


def _strap_pts(strap):
    toe = strap.to_world((-SL / 2, 0.0, -ST / 2))
    heel = strap.to_world((SL / 2, 0.0, -ST / 2))
    return toe, heel


def _tread_under(block, pt, tol):
    """Index of the tread whose top is within ``tol`` of world point ``pt`` and which lies under it, else None."""
    x, y, z = block.to_local(pt)
    if abs(y) > 0.022 or tilt_deg(block) > 6.0:
        return None
    for k in range(4):
        x0 = TREAD_X[k] - 0.01
        if x0 + 0.003 <= x <= x0 + 0.02 + 0.004 and abs(z - TREAD_H[k]) <= tol:
            return k
    return None


def _block_airborne(block):
    return bool(block.low[2] > TOP + PLATE_T + AIR)


def clamp_force(objects):
    """Newtons the lift spring pushes back with: 1.5 N per mm it is pressed below its free height."""
    return LIFT_K * max(0.0, -float(objects["mill"].joints["lift"]))


def thread_depth(turn):
    """Metres the nut's thread holds the thrust washer below where the scene builds it, with the nut turned ``turn``
    radians clockwise: the lead times the turn, less the 1 mm the lug starts above the ramp."""
    return max(0.0, LEAD * turn - LUG_GAP)


def _facts(objects):
    strap, block, part, mill = objects["strap"], objects["step_block"], objects["blue_part"], objects["mill"]
    toe, heel = _strap_pts(strap)
    part_top = float(part.high[2])
    overlap = float(part.high[0] - toe[0])
    on_part_xy = (part.low[1] - 0.002 <= toe[1] <= part.high[1] + 0.002) and part.low[0] < toe[0] < part.high[0]
    stud = strap.to_local((STUD_XY[0], STUD_XY[1], strap.origin[2]))
    in_slot = bool(-SL / 2 + 0.004 < stud[0] < -SL / 2 + SLOT_D - 0.006 and abs(stud[1]) <= 0.004)
    strap_tilt = tilt_deg(strap)
    block_ok = not _block_airborne(block)
    k_heel = _tread_under(block, heel, 0.002) if block_ok else None
    k_loose = _tread_under(block, heel, 0.003) if block_ok else None
    rise = float(heel[2] - toe[2])
    toe_up = float(toe[2] - part_top)
    lift_top = float(mill.origin[2]) + LIFT_REST + float(mill.joints["lift"])
    supports = [TOP, TOP + PLATE_T, part_top, lift_top]
    if block_ok:
        supports += [float(block.origin[2]) + h for h in TREAD_H]
    airborne = bool(all(float(strap.low[2]) - z > AIR for z in supports))
    # the tread under the heel is the one level with the part's top (0 to 4 mm above it)
    tread_ok = k_heel is not None and TREAD_MATCH[0] <= float(block.origin[2]) + TREAD_H[k_heel] - part_top <= TREAD_MATCH[1]
    set_up = bool(
        not airborne and tread_ok and in_slot and on_part_xy and TOE_OVERLAP[0] <= overlap <= TOE_OVERLAP[1]
        and -LEVEL_TOL <= toe_up <= SET_TOE_UP
    )
    level = bool(set_up and abs(toe_up) <= LEVEL_TOL and HEEL_RISE[0] <= rise <= HEEL_RISE[1] and strap_tilt <= LEVEL_TILT)
    stage1 = bool(
        not airborne and k_loose is not None and on_part_xy and overlap >= 0.005
        and toe_up >= -0.003 and strap_tilt <= STAGE1_TILT
    )
    washer_bottom = float(mill.origin[2]) + WASH_Z0 + WASH_SAG + float(mill.joints["washer"])
    top_at_stud = strap.to_world((stud[0], 0.0, ST / 2))[2]
    bearing = bool(in_slot and -0.003 <= washer_bottom - top_at_stud <= WASHER_GAP)
    turn = -float(mill.joints["nut"])
    driven = bool(turn >= NUT_MIN and abs(-float(mill.joints["washer"]) - thread_depth(turn)) <= THREAD_TOL)
    clamped = bool(level and bearing and driven and -float(mill.joints["lift"]) >= CLAMP_MIN)
    return {"set": set_up, "level": level, "stage1": stage1, "bearing": bearing, "driven": driven, "clamped": clamped,
            "airborne": airborne}


def _part_kept(objects):
    peaks = _tracker.update(
        objects,
        {"part_move": float(np.linalg.norm(np.asarray(objects["blue_part"].pos, float) - np.asarray(PART_POS)))},
    )
    return peaks["part_move"] <= PART_MOVE_MAX


def score(objects):
    """Part kept (never more than 3 mm from its start, latched over the run), then: 1.0 clamped: the strap is set (stud
    in the slot, toe 1.0 to 2.5 cm over the part, heel on the tread level with the part's top), level (toe down on the
    part, heel 0 to 4 mm above the toe, tilt <= 3 deg) and the nut holds it there: the thrust washer bears on the
    strap, the nut has turned 110 degrees or more with the washer as deep as that angle puts it (within 3 mm), and the
    lift spring under the strap is pressed down 3 mm or more (4.5 N or more). 0.5: set, not clamped (the spring holds
    the toe up to 1 cm off the part; a washer pushed down by a gripper is not clamped). 0.25: toe over the part and heel
    on any tread, tilt <= 15 deg. Strap off the table or airborne: 0."""
    kept = _part_kept(objects)
    f = _facts(objects)
    if not kept or f["airborne"] or objects["strap"].origin[2] < TOP - 0.05:
        return 0.0
    if f["clamped"]:
        return 1.0
    if f["set"]:
        return 0.5
    if f["stage1"]:
        return 0.25
    return 0.0


def success(objects):
    """Everything scored 1.0 and the strap, block and part are still."""
    return score(objects) == 1.0 and all(objects[n].resting() for n in ("strap", "step_block", "blue_part"))


def distance(objects):
    """Metres from the strap's toe to its clamped position (0.22 m), along x, plus its height error."""
    toe, _ = _strap_pts(objects["strap"])
    return float(math.hypot(toe[0] - 0.22, toe[2] - PART_TOP))


# Parts of either arm stay out of the cutter column, the cutter arm and its column.
ARM_KEEP_OUT = [
    {"name": "under the cutter", "x": (0.155, 0.245), "y": (-0.065, 0.025), "z_top": TOP + 0.25},
    {"name": "cutter arm and column", "x": (0.085, 0.215), "y": (-0.05, 0.01), "z_top": TOP + 0.25},
]


# ---- the demo ------------------------------------------------------------------------------------------------------------
def _origin(ctx, name):
    return np.array(ctx.states()[name].origin, dtype=float)


def _mock(ctx) -> bool:
    """The offline dry run hands the script one frozen dict; the simulator builds a new one every call."""
    return ctx.states() is ctx.states()


def _down(arm, ctx, pos_xy, z0, z1, stop, speed, yaw=0.0):
    """Lower the TCP from z0 toward z1 at ``speed`` m/s until stop() is true (then hold)."""
    z = z0
    n = max(1, int(abs(z0 - z1) / speed / ctx.dt))
    for _ in range(n):
        if stop():
            break
        z -= speed * ctx.dt
        yield from arm.servo(lambda: (pos_xy[0], pos_xy[1], z), ctx.dt, yaw=yaw, max_speed=0.5)


def demo_script(ctx):
    L, R = ctx.left, ctx.right
    OVER = TOP + 0.155  # fingertips 14.2 cm up: over the nut (13.4), as high as the right arm reaches there
    HOV = TOP + 0.14
    # ---- left arm: carry the step block onto the plate, 4 cm tread under where the heel will rest ----------------------
    b = _origin(ctx, "step_block")
    gx, gy = b[0] + 0.025, b[1]  # grip over the 4 and 5 cm treads: the hand stays well +x of the nut
    yield from L.move((gx, gy, HOV), 2.0, FINGER_OPEN, yaw=0.0)
    yield from L.move((gx, gy, TOP + 0.025), 1.5, FINGER_OPEN, yaw=0.0)
    yield from L.trim_to_measured((gx, gy, TOP + 0.025))
    yield from L.grip(FINGER_CLOSED, 0.8)
    yield from L.move((gx, gy, TOP + 0.057), 1.0, FINGER_CLOSED, yaw=0.0)
    # carry in from the +y side at x = BLOCK_X, nudging until the block itself (not the hand) is on its mark
    off = _origin(ctx, "step_block")[:2] - np.array([L.pos[0], L.pos[1]])
    nominal = np.array([BLOCK_X, BLOCK_Y]) - off
    yield from L.move((nominal[0], 0.09, TOP + 0.057), 2.0, FINGER_CLOSED, yaw=0.0)
    yield from L.move((nominal[0], nominal[1], TOP + 0.057), 1.5, FINGER_CLOSED, yaw=0.0)
    for _i in range(6):
        o = _origin(ctx, "step_block")
        err = np.array([BLOCK_X - o[0], BLOCK_Y - o[1]])
        if np.hypot(*err) < 0.0015:
            break
        goal = np.clip(np.array([L.pos[0], L.pos[1]]) + err, nominal - 0.012, nominal + 0.012)
        yield from L.move((goal[0], goal[1], TOP + 0.057), 0.6, FINGER_CLOSED, yaw=0.0)
    tx, ty = L.pos[0], L.pos[1]
    yield from _down(L, ctx, (tx, ty), TOP + 0.057, TOP + 0.03, lambda: _origin(ctx, "step_block")[2] < TOP + PLATE_T + 0.0015, 0.01)
    yield from L.grip(FINGER_OPEN, 0.8)
    yield from L.move((L.pos[0], L.pos[1], TOP + 0.09), 1.0, FINGER_OPEN, yaw=0.0)
    yield from L.move((L.pos[0] - 0.03, 0.09, TOP + 0.10), 1.5, FINGER_OPEN, yaw=0.0)
    yield from L.home(3.0)

    # ---- right arm: thread the strap onto the stud under the thrust washer, heel on the 4 cm tread --------------------
    s = _origin(ctx, "strap")
    tab = s + np.array([TAB_X, 0.0, ST / 2 + 0.015 - 0.0])
    gz = TOP + 0.032
    yield from R.move((tab[0], tab[1], HOV), 2.0, FINGER_OPEN, yaw=0.0)
    yield from R.move((tab[0], tab[1], gz), 1.5, FINGER_OPEN, yaw=0.0)
    yield from R.trim_to_measured((tab[0], tab[1], gz))
    yield from R.grip(FINGER_CLOSED, 0.8)
    carry = gz + CARRY_UNDER  # strap underside 6.5 cm up: over the 5 cm tread (6.0), top 7.7 under the washer (8.1)
    yield from R.move((tab[0], tab[1], carry), 1.5, FINGER_CLOSED, yaw=0.0)
    toff = tab[0] - (_origin(ctx, "strap")[0] - SL / 2)  # TCP x less toe x
    yaw_y = _origin(ctx, "strap")[1] - R.pos[1]
    yield from R.move((0.290 + toff, R.pos[1], carry), 2.0, FINGER_CLOSED, yaw=0.0)
    yield from R.move((0.290 + toff, STUD_XY[1] - yaw_y, carry), 2.0, FINGER_CLOSED, yaw=0.0)
    yield from R.trim_to_measured((0.290 + toff, STUD_XY[1] - yaw_y, carry))
    yield from R.servo(lambda: (0.22 + toff + 0.0, STUD_XY[1] - yaw_y, carry), 9.0, FINGER_CLOSED, max_speed=0.01)
    yield from R.trim_to_measured((0.22 + toff, STUD_XY[1] - yaw_y, carry))
    # lower it: the strap comes to rest on the spring washer and, at its heel, on the tread
    yield from _down(R, ctx, (0.22 + toff, STUD_XY[1] - yaw_y), carry, gz + 0.047,
                     lambda: _strap_pts(ctx.states()["strap"])[1][2] < TOP + 0.0508, 0.005)
    yield from R.grip(FINGER_OPEN, 0.8)
    yield from R.move((R.pos[0], R.pos[1], OVER), 1.5, FINGER_OPEN, yaw=0.0)

    # ---- right arm: turn the nut clockwise by its handle until the strap is clamped ----------------------------------
    mock = _mock(ctx)
    # Closed fingers touch the handle 18 to 27 degrees behind its centre (they are wider along x than along y). The
    # script aims up to 12 degrees past the centre, so the arm can lean on the handle with 3 to 4 cm of reach. If the nut
    # stops short of clamped with the arm already leaning that far, it leans further, up to 24 degrees past.
    lag, floor, hard = math.radians(27.0), math.radians(-12.0), math.radians(-24.0)
    pz = TOP + NUT_Z - HANDLE_DROP + 0.016  # fingertips reach 1.3 cm below this: 3 mm above the handle's foot

    def at(c, z):
        return (STUD_XY[0] + HANDLE_R * math.cos(c), STUD_XY[1] + HANDLE_R * math.sin(c), z)

    c = HANDLE_A0 + lag
    yield from R.move(at(c, OVER), 2.0, FINGER_CLOSED, yaw=0.0)
    yield from R.move(at(c, pz), 1.5, FINGER_CLOSED, yaw=0.0)
    yield from R.trim_to_measured(at(c, pz))
    trim = np.array(R.pos, dtype=float) - np.array(at(c, pz))  # what the arm sags by here; carried round the turn
    turn = 0.0  # the nut's angle the script assumes offline
    rate = math.radians(8.0)  # per second
    seen = []  # the nut's angle over the last second, to tell when it has stopped turning
    for _ in range(int(60.0 / ctx.dt)):
        st = ctx.states()
        theta = turn if mock else float(st["mill"].joints["nut"])
        seen = (seen + [theta])[-int(1.0 / ctx.dt):]
        stopped = len(seen) >= int(1.0 / ctx.dt) and seen[0] - seen[-1] < math.radians(0.5)  # the nut has stopped turning
        tight = not mock and _facts(st)["clamped"] and (
            -float(st["mill"].joints["lift"]) >= SNUG  # the spring washer is down to the part's height: the toe is on the part
            or stopped
        )
        if tight or (mock and turn <= -math.radians(142.0)):
            break
        if not mock and stopped and c <= HANDLE_A0 + theta + floor + math.radians(1.0):
            floor = max(hard, floor - math.radians(4.0) * ctx.dt)  # stalled at full lean, not clamped: lean harder
        c = max(c - rate * ctx.dt, HANDLE_A0 + theta + floor)  # never drive far past the handle if it stalls
        turn = min(0.0, c - lag - HANDLE_A0)
        if not mock:  # hold the fingertips' height from the arm's own joint readings, so they stay over the strap's tab
            trim[2] = float(np.clip(trim[2] + np.clip(0.3 * (pz - R.measured_pos[2]), -0.0004, 0.0004), -0.01, 0.035))
        R._set_cartesian(np.array(at(c, pz)) + trim, R.rot, FINGER_CLOSED)
        yield
    yield from R.hold(0.3)
    # let go of the handle: back to 32 degrees behind it, which is clear of where the fingers touch it
    c = HANDLE_A0 + (turn if mock else float(ctx.states()["mill"].joints["nut"])) + math.radians(32.0)
    yield from R.move(np.array(at(c, pz)) + trim, 0.8, FINGER_CLOSED, yaw=0.0)
    yield from R.move(at(c, OVER), 1.0, FINGER_CLOSED, yaw=0.0)
    yield from R.home(2.5)
    yield from ctx.wait(1.0)

