"""On the station table: a small mill table (a dark grey plate with two stops) holds a blue 6 x 6 x 4 cm part against the
stops. Beside it stands a grey stud with an orange nut raised near its top (the nut has a yellow handle), and an overhead
mill arm with a black cutter hangs over the part. On the bare table lie a red strap clamp (a slotted bar with a white
grip tab) in front of the right arm, and a yellow step block (four treads, 2 to 5 cm tall) in front of the left arm.
The strap, the block and the part are loose; the mill (plate, stops, stud, nut, arm) is fixed except the nut, which
slides down the stud.

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
SUCCESS_HOLD_S = 8.0  # the demo grips things; keep the episode running until the release is on video
SCORE_OVER = "end"

DARK, GREY, LIGHT, BLACK = (0.22, 0.22, 0.25), (0.55, 0.55, 0.58), (0.78, 0.78, 0.80), (0.07, 0.07, 0.08)
RED, WHITE, YELLOW, ORANGE, BLUE = (0.85, 0.10, 0.10), (0.97, 0.97, 0.97), (0.96, 0.80, 0.08), (0.96, 0.45, 0.04), (0.12, 0.28, 0.85)

PLATE_T = 0.01  # the mill table plate stands 1 cm proud of the table
STUD_XY = (0.27, -0.02)
CUT_XY = (0.20, -0.02)

# ---- the mill: fixed. Its frame is at the stud's foot on the table top. -----------------------------------------------
NUT_Z0 = 0.110  # nut underside above TOP when raised
NUT_LOW = -0.058  # stop: the nut's underside can go 5.2 cm above TOP, 1 cm below the strap's top (6.2 cm)
NUT_ON_STRAP = 0.062 - NUT_Z0  # J with the nut resting on the strap's top (strap underside 5 cm up, 1.2 cm thick)


def _l(x, y):  # world xy -> mill frame
    return (x - STUD_XY[0], y - STUD_XY[1])


def _nut_shapes():
    h = 0.008  # hole half width: 2 mm clear of the 1.2 cm stud
    ring = [
        box((0, h + 0.0035, 0.006), (0.03, 0.007, 0.012), ORANGE),
        box((0, -h - 0.0035, 0.006), (0.03, 0.007, 0.012), ORANGE),
        # the two x-side walls start 4 mm up, so they cannot drop into the strap's slot
        box((h + 0.0035, 0, 0.008), (0.007, 2 * h, 0.008), ORANGE),
        box((-h - 0.0035, 0, 0.008), (0.007, 2 * h, 0.008), ORANGE),
        box((0, -0.0225, 0.006), (0.018, 0.015, 0.012), ORANGE),  # arm off the -y face, 1.5 cm long
        box((0, -0.039, 0.015), (0.018, 0.018, 0.030), YELLOW),  # handle: 1.8 x 1.8 x 3.0
    ]
    return tuple(ring)


NUT_DAMPING = 800.0  # N s/m: the nut holds its height (creeps about 0.05 mm/s) and stays where it is pushed
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
        Joint("nut", "slide", axis="z", at=(0, 0, NUT_Z0), low=NUT_LOW, high=0.0, spring=0.0, damping=NUT_DAMPING,
              mass=0.004, shapes=_nut_shapes())
    ],
    friction=0.8,
)

# ---- the part: a blue 6 x 6 x 4 cm block against both stops --------------------------------------------------------------
PART_POS = (0.202, -0.02, TOP + PLATE_T + 0.02)
PART = Box("blue_part", pos=PART_POS, size=(0.06, 0.06, 0.04), mass=0.35, color=BLUE)
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
    mass=0.06,
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
    mass=0.08,
    shapes=[box((TREAD_X[k], 0, TREAD_H[k] / 2), (0.02, 0.04, TREAD_H[k]), YELLOW) for k in range(4)],
    friction=0.8,
)

OBJECTS = [MILL, PART, STRAP, BLOCK]

demo = placeholder_demo

BLOCK_X, BLOCK_Y = 0.322, -0.02  # block frame (middle of its underside) when set on the plate

# ---- scoring ------------------------------------------------------------------------------------------------------------
PART_MOVE_MAX = 0.003  # part within 3 mm of its start
TOE_OVERLAP = (0.010, 0.025)  # toe on the part, 1.0 to 2.5 cm
LEVEL_TOL = 0.002  # toe underside within 2 mm of the part's top
HEEL_RISE = (-0.0015, 0.004)  # heel end 0 to 4 mm higher than the toe end (1.5 mm for sag in the contacts)
LEVEL_TILT = 3.0
STAGE1_TILT = 8.0
NUT_TOL = 0.003
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
    supports = [TOP, TOP + PLATE_T, part_top]
    if block_ok:
        supports += [float(block.origin[2]) + h for h in TREAD_H]
    airborne = bool(float(strap.low[2]) - max(supports) > AIR and all(float(strap.low[2]) - z > AIR for z in supports))
    level = bool(
        not airborne and k_heel is not None and in_slot and on_part_xy
        and abs(toe[2] - part_top) <= LEVEL_TOL and TOE_OVERLAP[0] <= overlap <= TOE_OVERLAP[1]
        and HEEL_RISE[0] <= rise <= HEEL_RISE[1] and strap_tilt <= LEVEL_TILT
    )
    stage1 = bool(
        not airborne and k_loose is not None and on_part_xy and abs(toe[2] - part_top) <= 0.003
        and overlap >= 0.005 and strap_tilt <= STAGE1_TILT
    )
    nut_bottom = float(mill.origin[2]) + NUT_Z0 + float(mill.joints["nut"])
    top_at_stud = strap.to_world((stud[0], 0.0, ST / 2))[2]
    nut_down = bool(abs(nut_bottom - top_at_stud) <= NUT_TOL and in_slot)
    return {"level": level, "stage1": stage1, "nut_down": nut_down, "airborne": airborne}


def _part_kept(objects):
    peaks = _tracker.update(
        objects,
        {"part_move": float(np.linalg.norm(np.asarray(objects["blue_part"].pos, float) - np.asarray(PART_POS)))},
    )
    return peaks["part_move"] <= PART_MOVE_MAX


def score(objects):
    """Part kept (never more than 3 mm from its start, latched over the run), then: 1.0 strap level (toe on the part,
    heel on a tread, stud in the slot, heel 0 to 4 mm above the toe) with the nut run down onto it; 0.5 level, nut up;
    0.25 strap resting on the part and a tread (any tread, tilt <= 8 deg); else 0. Strap off the table or airborne: 0."""
    kept = _part_kept(objects)
    f = _facts(objects)
    if not kept or f["airborne"] or objects["strap"].origin[2] < TOP - 0.05:
        return 0.0
    if f["level"] and f["nut_down"]:
        return 1.0
    if f["level"]:
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


def _nut_j(ctx):
    return float(ctx.states()["mill"].joints["nut"])


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

    # ---- right arm: thread the strap onto the stud, toe on the part, heel on the 4 cm tread --------------------------
    s = _origin(ctx, "strap")
    tab = s + np.array([TAB_X, 0.0, ST / 2 + 0.015 - 0.0])
    gz = TOP + 0.032
    yield from R.move((tab[0], tab[1], HOV), 2.0, FINGER_OPEN, yaw=0.0)
    yield from R.move((tab[0], tab[1], gz), 1.5, FINGER_OPEN, yaw=0.0)
    yield from R.trim_to_measured((tab[0], tab[1], gz))
    yield from R.grip(FINGER_CLOSED, 0.8)
    carry = TOP + 0.108
    yield from R.move((tab[0], tab[1], carry), 1.5, FINGER_CLOSED, yaw=0.0)
    toff = tab[0] - (_origin(ctx, "strap")[0] - SL / 2)  # TCP x less toe x
    yaw_y = _origin(ctx, "strap")[1] - R.pos[1]
    yield from R.move((0.285 + toff, R.pos[1], carry), 2.0, FINGER_CLOSED, yaw=0.0)
    yield from R.move((0.285 + toff, STUD_XY[1] - yaw_y, carry), 2.0, FINGER_CLOSED, yaw=0.0)
    yield from R.trim_to_measured((0.285 + toff, STUD_XY[1] - yaw_y, carry))
    yield from R.servo(lambda: (0.22 + toff + 0.0, STUD_XY[1] - yaw_y, carry), 8.0, FINGER_CLOSED, max_speed=0.01)
    yield from R.trim_to_measured((0.22 + toff, STUD_XY[1] - yaw_y, carry))
    yield from _down(R, ctx, (0.22 + toff, STUD_XY[1] - yaw_y), carry, TOP + 0.075, lambda: _origin(ctx, "strap")[2] < TOP + 0.0505 + ST / 2, 0.005)
    yield from R.grip(FINGER_OPEN, 0.8)
    yield from R.move((R.pos[0], R.pos[1], TOP + 0.15), 1.5, FINGER_OPEN, yaw=0.0)

    # ---- right arm: run the nut down onto the strap ------------------------------------------------------------------
    hx, hy, hz = STUD_XY[0], STUD_XY[1] - 0.039, TOP + NUT_Z0 + 0.02
    half_open = FINGER_OPEN * 0.45
    yield from R.move((hx + 0.04, hy - 0.08, TOP + 0.19), 2.0, half_open, yaw=math.pi / 2)  # turn the grip well clear of the cutter
    yield from R.move((hx, hy, TOP + 0.19), 1.5, half_open, yaw=math.pi / 2)
    yield from R.move((hx, hy, hz), 1.5, half_open, yaw=math.pi / 2)
    yield from R.trim_to_measured((hx, hy, hz))
    yield from R.grip(FINGER_CLOSED, 0.8)
    yield from _down(R, ctx, (hx, hy), hz, hz - 0.075, lambda: _nut_j(ctx) <= NUT_ON_STRAP + 0.0012, 0.005, yaw=math.pi / 2)
    yield from R.hold(0.5)
    yield from R.grip(FINGER_OPEN, 0.8)
    yield from R.move((R.pos[0], R.pos[1], TOP + 0.19), 1.5, FINGER_OPEN, yaw=math.pi / 2)
    yield from R.home(3.0)
    yield from ctx.wait(1.0)

