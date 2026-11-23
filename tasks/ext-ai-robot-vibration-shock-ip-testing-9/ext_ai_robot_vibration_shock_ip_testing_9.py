"""Vibration/shock qualification fixture: a Carr Lane CLM-8-QAK quick-acting four-prong knob on an M8x1.25 stud and
slotted aluminum crossbar over an edge compute module on a T-slot subplate.

On the station table (TOP = 0.7475 m):
- An aluminum T-slot subplate (160 x 200 x 12 mm, fixed) at (0.260, 0.0, TOP) holding a dark anodized edge compute
  enclosure (90 x 103 x 35 mm) centered at (0.260, 0.0, TOP + 0.012).
- A fixed 6061-T6 aluminum crossbar (180 x 25 x 12 mm) spanning y = -0.090..+0.090 across the module (crossbar top at
  TOP + 0.059), supported on two 35 mm spacer sleeves at y = +0.075 (pre-clamped by a hex nut) and y = -0.075 (open M8
  stud rising 55 mm above the crossbar to TOP + 0.114).
- A loose DIN 6340 M8 washer (8.4 x 23 x 4 mm) lying flat on the crossbar round the open stud.
- The stud's first thread crest, just above the washer inside its bore, rides a mechkit kinematic screw (M8x1.25, right
  hand): it stands for the turn of thread the knob's own half-threads catch. Only the knob's bottom lug reaches it.
- An anodized blue staging block (44 x 44 x 12 mm) with an 18 mm OD, 10 mm pedestal and brass 4.2 mm staging peg at
  (0.200, -0.150, TOP).
- A loose sintered-iron Carr Lane CLM-8-QAK quick-acting four-prong knob (38 mm across the prongs, 17 mm hub, 22 mm
  tall, 41 g) upright on the staging peg. Its relieved bore clears the stud crests only when tilted about 13 deg;
  pushed straight down at <= 5 deg tilt its lugs stop on the top crests. Squared on the washer, its bottom lug lands on
  the first crest, and turning the knob clockwise drives the thread down until its face meets the washer.
"""

from __future__ import annotations

import dataclasses
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from belimits import tilt_deg
import closedloop
from closedloop import placeholder_demo
import mechkit
from mechkit import Mech, box, cone, cyl, on_reset, screw, tube, bore
from yam_arena import ik
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.task_file import TABLE_TOP

TOP = TABLE_TOP["station"]
SUCCESS_HOLD_S = 8.0
ARM_KEEP_OUT = []

# -- dimensions (m) ----------------------------------------------------------------------------------------------------
PLATE_XY = (0.260, 0.0)
PLATE_SIZE = (0.160, 0.200, 0.012)
MODULE_SIZE = (0.090, 0.103, 0.035)
CROSSBAR_SIZE = (0.025, 0.180, 0.012)
CROSSBAR_TOP = TOP + PLATE_SIZE[2] + MODULE_SIZE[2] + CROSSBAR_SIZE[2]  # TOP + 0.059

STUD_Y_OFFSET = 0.075
STUD_XY = (PLATE_XY[0], PLATE_XY[1] - STUD_Y_OFFSET)  # (0.260, -0.075), right arm's own side
STUD_CORE_R = 0.00250
STUD_CREST_R = 0.00400
STUD_LEAD_H = 0.0025
STUD_H = 0.102  # from plate top (TOP + 0.012) to TOP + 0.114 (55 mm exposed above crossbar)
STUD_TOP = TOP + PLATE_SIZE[2] + STUD_H  # TOP + 0.114

STAGE_XY = (0.200, -0.150)
STAGE_SIZE = (0.044, 0.044, 0.012)
STAGE_PED_R = 0.009
STAGE_PED_H = 0.010
STAGE_PEG_R = 0.0021
STAGE_PEG_H = 0.008
STAGE_TOP = TOP + STAGE_SIZE[2] + STAGE_PED_H  # TOP + 0.022

KNOB_R_OUT = 0.0085  # hub radius (catalog B DIA 17 mm)
KNOB_R_IN = 0.0076  # relieved bore: clears the 8 mm crests at the 13 deg tilt (0.5 mm or more)
KNOB_PRONG_R = 0.0190  # four prongs, 38 mm across (catalog A 38 mm, 1-1/2 in OD)
KNOB_PRONG_W = 0.0080
KNOB_PRONG_H = 0.0095  # prongs over the top 9.5 mm; the 12.5 mm hub below them (catalog D 12.5 mm)
KNOB_H = 0.022  # overall height (catalog C 22 mm)
KNOB_MASS = 0.041  # 0.09 lb (All Industrial listing), sintered iron
LUG_LIFT = 0.0002  # bottom lug's lower face above the knob's bottom face
LUG_RI = 0.00325  # lug inner face radius: 0.75 mm inside the 4.0 mm crest radius
KNOB_START = (STAGE_XY[0], STAGE_XY[1], STAGE_TOP + KNOB_H / 2.0)

# -- colours -----------------------------------------------------------------------------------------------------------
AL_PLATE = (0.56, 0.58, 0.62)
T_SLOT = (0.26, 0.28, 0.31)
CHASSIS_DARK = (0.16, 0.18, 0.22)
PCB_GREEN = (0.18, 0.48, 0.26)
HEATSINK_SILVER = (0.72, 0.74, 0.78)
AL_SILVER = (0.82, 0.84, 0.87)
STUD_STEEL = (0.62, 0.64, 0.68)
CREST_DARK = (0.40, 0.42, 0.46)
STAGE_BLUE = (0.22, 0.42, 0.68)
BRASS = (0.82, 0.62, 0.22)
KNOB_BLACK = (0.13, 0.14, 0.17)
KNOB_LUG = (0.18, 0.19, 0.22)
INDEX_GOLD = (0.92, 0.78, 0.20)

# -- scene objects -----------------------------------------------------------------------------------------------------
def _vbox(at, size, color):
    return dataclasses.replace(box(at, size, color), collide=False)


_plate_shapes = [
    box((0.0, 0.0, PLATE_SIZE[2] / 2.0), PLATE_SIZE, AL_PLATE),
    # Visual T-slots along Y
    _vbox((-0.045, 0.0, PLATE_SIZE[2] + 0.0002), (0.008, 0.194, 0.0004), T_SLOT),
    _vbox((0.0, 0.0, PLATE_SIZE[2] + 0.0002), (0.008, 0.194, 0.0004), T_SLOT),
    _vbox((0.045, 0.0, PLATE_SIZE[2] + 0.0002), (0.008, 0.194, 0.0004), T_SLOT),
    # Edge compute enclosure (90 x 103 x 35 mm) resting on subplate
    box((0.0, 0.0, PLATE_SIZE[2] + MODULE_SIZE[2] / 2.0), MODULE_SIZE, CHASSIS_DARK),
    # Visual heatsink ribs and carrier PCB edge stripe for contrast at 320x176
    _vbox((0.032, 0.0, PLATE_SIZE[2] + MODULE_SIZE[2] + 0.0004), (0.016, 0.092, 0.0008), HEATSINK_SILVER),
    _vbox((-0.032, 0.0, PLATE_SIZE[2] + MODULE_SIZE[2] + 0.0004), (0.016, 0.092, 0.0008), HEATSINK_SILVER),
    _vbox((0.0, 0.0518, PLATE_SIZE[2] + 0.012), (0.076, 0.001, 0.006), PCB_GREEN),
]
PLATE = Mech("plate", pos=(PLATE_XY[0], PLATE_XY[1], TOP), mass=None, shapes=_plate_shapes)

STAGE = Mech(
    "stage",
    pos=(STAGE_XY[0], STAGE_XY[1], TOP),
    mass=None,
    shapes=[
        box((0.0, 0.0, STAGE_SIZE[2] / 2.0), STAGE_SIZE, STAGE_BLUE),
        cyl((0.0, 0.0, STAGE_SIZE[2] + STAGE_PED_H / 2.0), STAGE_PED_R, STAGE_PED_H, STAGE_BLUE, contact=0.00025),
        tube((0.0, 0.0, STAGE_SIZE[2] + STAGE_PED_H - 0.00125), 0.0118, 0.0148, 0.0025, STAGE_BLUE, sides=18, contact=0.00025),
        cyl((0.0, 0.0, STAGE_SIZE[2] + STAGE_PED_H + STAGE_PEG_H / 2.0), STAGE_PEG_R, STAGE_PEG_H, BRASS, contact=0.00025),
    ],
)

_cb_z = MODULE_SIZE[2] + CROSSBAR_SIZE[2] / 2.0  # 0.041 above subplate top (world TOP + 0.053)
_cb_top_local = MODULE_SIZE[2] + CROSSBAR_SIZE[2]  # 0.047 above subplate top
_crossbar_shapes = [
    # Spacer sleeves under crossbar ends
    tube((0.0, +STUD_Y_OFFSET, MODULE_SIZE[2] / 2.0), 0.0045, 0.0080, MODULE_SIZE[2], AL_SILVER),
    tube((0.0, -STUD_Y_OFFSET, MODULE_SIZE[2] / 2.0), 0.0045, 0.0080, MODULE_SIZE[2], AL_SILVER),
    # Aluminum crossbar: bored right end (-0.090..-0.060) + main bar (-0.060..+0.090)
    bore((0.0, -STUD_Y_OFFSET, _cb_z), (CROSSBAR_SIZE[0], 0.030, CROSSBAR_SIZE[2]), 0.0045, AL_SILVER,
         lead_in=0.0005, contact=0.00025),
    box((0.0, +0.015, _cb_z), (CROSSBAR_SIZE[0], 0.150, CROSSBAR_SIZE[2]), AL_SILVER),
    # Pre-clamped left stud and hex nut (+0.075)
    cyl((0.0, +STUD_Y_OFFSET, 0.035), 0.0040, 0.070, STUD_STEEL),
    cyl((0.0, +STUD_Y_OFFSET, _cb_top_local + 0.004), 0.0075, 0.008, CREST_DARK, sides=6),
    # Active right M8 stud (-0.075): 5.0 mm core + chamfered tip + 8.0 mm thread-crest collars
    cyl((0.0, -STUD_Y_OFFSET, (STUD_H - STUD_LEAD_H) / 2.0), STUD_CORE_R, STUD_H - STUD_LEAD_H, STUD_STEEL, contact=0.00025),
    cone((0.0, -STUD_Y_OFFSET, STUD_H - STUD_LEAD_H), STUD_CREST_R, 0.0018, STUD_LEAD_H, CREST_DARK, sides=16, contact=0.00025),
    cyl((0.0, -STUD_Y_OFFSET, _cb_top_local + 0.0150), STUD_CREST_R, 0.0028, CREST_DARK, contact=0.00025),
]
for _dz, _ch in ((0.0290, 0.0022), (0.0360, 0.0022), (0.0430, 0.0022), (0.0495, 0.0020)):
    _crossbar_shapes.append(
        cyl((0.0, -STUD_Y_OFFSET, _cb_top_local + _dz), STUD_CREST_R, _ch, CREST_DARK, contact=0.00025)
    )
CROSSBAR = Mech("crossbar", pos=(PLATE_XY[0], PLATE_XY[1], TOP + PLATE_SIZE[2]), mass=None, shapes=_crossbar_shapes)

_PR_C = (KNOB_R_IN + KNOB_PRONG_R) / 2.0
_PR_L = KNOB_PRONG_R - KNOB_R_IN
_PR_Z = KNOB_H / 2.0 - KNOB_PRONG_H / 2.0
_LUG_L = 0.0078 - LUG_RI  # lug from its inner face into the hub wall
_LUG_C = LUG_RI + _LUG_L / 2.0
_LUG_Z = KNOB_H / 2.0 - LUG_LIFT - 0.0011
KNOB = Mech(
    "knob",
    pos=KNOB_START,
    mass=KNOB_MASS,
    friction=0.6,
    shapes=[
        tube((0.0, 0.0, 0.0), KNOB_R_IN, KNOB_R_OUT, KNOB_H, KNOB_BLACK, sides=24, contact=0.00025),
        # Four prongs over the top 9.5 mm (at +-x and +-y)
        box((+_PR_C, 0.0, _PR_Z), (_PR_L, KNOB_PRONG_W, KNOB_PRONG_H), KNOB_BLACK, contact=0.00025),
        box((-_PR_C, 0.0, _PR_Z), (_PR_L, KNOB_PRONG_W, KNOB_PRONG_H), KNOB_BLACK, contact=0.00025),
        box((0.0, +_PR_C, _PR_Z), (KNOB_PRONG_W, _PR_L, KNOB_PRONG_H), KNOB_BLACK, contact=0.00025),
        box((0.0, -_PR_C, _PR_Z), (KNOB_PRONG_W, _PR_L, KNOB_PRONG_H), KNOB_BLACK, contact=0.00025),
        # Opposing half-thread lugs: top at -x, bottom at +x (inner faces 3.25 mm from the axis)
        box((-_LUG_C, 0.0, +_LUG_Z), (_LUG_L, 0.0076, 0.0022), KNOB_LUG, contact=0.00025),
        box((+_LUG_C, 0.0, -_LUG_Z), (_LUG_L, 0.0076, 0.0022), KNOB_LUG, contact=0.00025),
        # Y-side bore guides: 9.6 mm across y, so the knob stays within 0.8 mm of the crests sideways
        box((0.0, +0.0063, 0.0), (0.0060, 0.0030, KNOB_H), KNOB_LUG, contact=0.00025),
        box((0.0, -0.0063, 0.0), (0.0060, 0.0030, KNOB_H), KNOB_LUG, contact=0.00025),
        # Gold index mark on the -x prong (the tilt-relief side)
        _vbox((-0.0150, 0.0, KNOB_H / 2.0 + 0.0004), (0.0060, 0.0050, 0.0008), INDEX_GOLD),
    ],
)

# -- thread engagement: the first thread crest, driven only by the knob's own half-thread lug --------------------------
# The stud's first thread crest just above the washer (8.0 mm crest, 5.4 mm bore round the 5.0 mm core, inside the
# washer's 8.4 mm bore) rides a kinematic screw (M8x1.25, right hand, 1.25 mm lead). It stands for the turn of thread
# the knob's half-threads catch. A squared knob's bottom lug (inner face 3.25 mm from the axis) lands on its top; the
# knob's bore (7.6 mm radius) and face (7.6..8.5 mm) never reach it, so only the lug can drive it. Turning the knob
# clockwise drives the crest down by friction at the lug, a quarter turn (0.31 mm) until the knob face meets the washer;
# past that the thread's hold grows with each turn. Tilted, loose or off the stud, the lug is off the crest.
LEAD = 0.00125  # M8 coarse pitch, m per turn
RUN_TURNS = 0.25  # thread run from where the squared knob's lug catches to face contact on the washer
RUN = LEAD * RUN_TURNS  # 0.3125 mm
SEAT_TRAVEL = -RUN
WASHER_RI, WASHER_RO, WASHER_T = 0.0042, 0.0115, 0.004  # DIN 6340 M8: 8.4 mm bore, 23 mm OD
WASHER_MASS = 0.011
WASHER_TOP_Z = CROSSBAR_TOP + WASHER_T  # the free washer lies flat on the crossbar
SEAT_RISE = 0.0001  # seated crest top above the lug's lower face at face contact: the lug keeps the load
CREST_RI, CREST_RO, CREST_H = 0.0027, 0.0040, 0.0030
CREST_TOP0 = WASHER_T + LUG_LIFT + SEAT_RISE + RUN  # crest top above the crossbar at the start (4.61 mm)
CREST_MASS = 0.020  # kg: 20 g with a 1e4 N/m thread keeps the MuJoCo replay stable at 480 Hz
THREAD_K = 1.0e4
THREAD_HOLD, THREAD_TIGHTEN, THREAD_MAX_HOLD = 0.0008, 0.03, 0.01  # N m running, + per turn past the seat, cap
ZINC = (0.70, 0.71, 0.74)
_carrier, _thread = screw("thread", [tube((0.0, 0.0, -CREST_H / 2.0), CREST_RI, CREST_RO, CREST_H, CREST_DARK,
                                          sides=24, contact=0.00025)],
                          lead=LEAD, at=(0.0, 0.0, CREST_TOP0), low=SEAT_TRAVEL - 0.0003, high=0.0, seat=SEAT_TRAVEL,
                          hold=THREAD_HOLD, tighten=THREAD_TIGHTEN, max_hold=THREAD_MAX_HOLD, mass=CREST_MASS,
                          stiffness=THREAD_K, turn_damping=0.002, friction=1.0)
THREAD = Mech(
    "thread",
    pos=(STUD_XY[0], STUD_XY[1], CROSSBAR_TOP),
    mass=None,
    friction=1.0,
    shapes=[_vbox((0.0, 0.0, 0.0001), (0.0050, 0.0050, 0.0002), CREST_DARK)],  # look-only thread root
    joints=[_carrier, _thread],
)
WASHER = Mech(
    "washer",
    pos=(STUD_XY[0], STUD_XY[1], CROSSBAR_TOP + WASHER_T / 2.0 + 0.0001),
    mass=WASHER_MASS,
    friction=0.6,
    shapes=[tube((0.0, 0.0, 0.0), WASHER_RI, WASHER_RO, WASHER_T, ZINC, sides=24, contact=0.00025)],
)

OBJECTS = [PLATE, STAGE, CROSSBAR, THREAD, WASHER, KNOB]

# -- scoring -----------------------------------------------------------------------------------------------------------
STUD_R_TOL = 0.006  # knob axis within 6 mm of the stud axis counts as "on the stud" while sliding
SLID_DOWN_Z = STUD_TOP - 0.030  # knob bottom at least 3 cm down the exposed stud
ON_AXIS = 0.004  # squared on the washer: axis within 4 mm of the stud
ON_Z = 0.0006  # knob bottom within 0.6 mm of the washer's measured top
ON_TILT = 3.0  # deg
ON_STEPS = 20  # 1 s squared on the washer
ENGAGE_AXIS = 0.0015  # thread engaged: axis within 1.5 mm of the stud ...
ENGAGE_Z = 0.0004  # ... and the bottom lug's face at most 0.4 mm above the crest's measured top ...
ENGAGE_SINK = 0.0002  # ... and at most 0.2 mm into it (upright <= 3 deg, as squared)
FACE_ON = 0.00025  # snug: knob face within 0.25 mm of the washer top
SNUG_TURNS = 1.0 / 24.0  # 15 deg past face contact
DRAW_FULL = RUN - 0.00003
KNOB_TURN_SHARE = 0.8  # the knob's own clockwise turn (world, about the stud) must be >= 0.8 x the thread turn credited
FALL_Z = TOP - 0.05
FALL_STEPS = 3
JAM_TILT_MAX = 5.0
JAM_CONTACT = 0.0003
JAM_Z_LOW = STUD_TOP - 0.008
JAM_STEPS = 10
STILL_V_MAX = 0.002
STILL_STEPS = 30
RELEASE_CLEAR = 0.03

_CB_POSE = (np.eye(3), np.asarray(CROSSBAR.pos, float))
_STUD_TIP_SHAPES = [s for s in CROSSBAR.shapes[6:] if s.collide]  # active stud core, chamfered tip and crests
_KNOB_COLLIDERS = [s for s in KNOB.shapes if s.collide]


def _fresh() -> dict:
    return {"seen": None, "fall": 0, "fell": False, "jam": 0, "jammed": False, "on": 0, "eng_prev": False,
            "law": 0.0, "past": 0.0, "theta": 0.0, "yaw": None, "drawn": 0.0, "past_credit": 0.0, "thread_cw": 0.0,
            "knob_cw": 0.0, "still": 0}


_S = _fresh()


@on_reset
def _reset_latch(env=None) -> None:
    _S.clear()
    _S.update(_fresh())


def _quat_rot(q) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def _pose(st):
    return _quat_rot(st.quat_xyzw), np.asarray(getattr(st, "origin", st.pos), float)


_knob_pose = _pose


def knob_bottom(st) -> float:
    """Lowest point of the knob's 17 mm x 22 mm hub, from its frame origin and attitude (exact for the round rim)."""
    rot, o = _pose(st)
    c = float(np.clip(rot[2, 2], -1.0, 1.0))
    return float(o[2] - (KNOB_H / 2.0) * abs(c) - KNOB_R_OUT * math.sqrt(max(0.0, 1.0 - c * c)))


def knob_yaw(st) -> float:
    """The knob's turn about the vertical (rad, counter-clockwise from above positive)."""
    rot = _pose(st)[0]
    return float(math.atan2(rot[1, 0], rot[0, 0]))


def washer_top(objects) -> float:
    w = objects.get("washer") if hasattr(objects, "get") else None
    if w is None:
        return WASHER_TOP_Z
    return float(_pose(w)[1][2] + WASHER_T / 2.0)


def stud_tip_gap(st) -> float:
    """Collider gap (m) between the knob and the active stud's tip region (core, chamfer, crests)."""
    pose = _pose(st)
    return min(mechkit.gap(a, pose, b, _CB_POSE) for a in _KNOB_COLLIDERS for b in _STUD_TIP_SHAPES)


def thread_state(objects) -> tuple[float, float, float, float]:
    """(measured travel, law travel, turns past the seat, unwrapped turn in rad) of the thread crest; zeros when the
    state has no joint values."""
    t = objects.get("thread") if hasattr(objects, "get") else None
    if t is None:
        return 0.0, 0.0, 0.0, 0.0
    try:
        s = mechkit.screw_states(THREAD, t)["thread"]
        theta = 2.0 * math.pi * s.turns
        law, past = mechkit.screw_law(THREAD, "thread", theta)
        return float(s.travel), float(law), float(past), float(theta)
    except Exception:  # noqa: BLE001  no joint values in this state
        return 0.0, 0.0, 0.0, 0.0


def _axis_r(st) -> float:
    pos = _pose(st)[1]
    return math.hypot(pos[0] - STUD_XY[0], pos[1] - STUD_XY[1])


def riding(st, w_top: float) -> bool:
    """Squared on the washer: bottom within 0.6 mm of its measured top, axis within 4 mm of the stud, tilt <= 3 deg."""
    return abs(knob_bottom(st) - w_top) <= ON_Z and _axis_r(st) <= ON_AXIS and tilt_deg(st) <= ON_TILT


def engaged(st, w_top: float, travel: float) -> bool:
    """Thread engaged: squared on the washer, axis within 1.5 mm of the stud and the bottom lug on the first crest
    (lug face from 0.2 mm into to 0.4 mm above the crest's measured top). A tilted, loose or off-axis knob is not engaged."""
    crest_top = CROSSBAR_TOP + CREST_TOP0 + travel
    return (riding(st, w_top) and _axis_r(st) <= ENGAGE_AXIS
            and -ENGAGE_SINK <= knob_bottom(st) + LUG_LIFT - crest_top <= ENGAGE_Z)


def _fingers_clear(st) -> bool:
    return bool(mechkit.released(st, clearance=RELEASE_CLEAR))


def _snug_now(st, w_top: float, travel: float, past: float) -> bool:
    return (engaged(st, w_top, travel) and knob_bottom(st) - w_top <= FACE_ON
            and _S["drawn"] >= DRAW_FULL and _S["past_credit"] >= SNUG_TURNS and past >= SNUG_TURNS
            and _S["knob_cw"] >= KNOB_TURN_SHARE * _S["thread_cw"] and _S["thread_cw"] > 0.0)


def _track(objects) -> None:
    """Once per control step (success() and score() both run on the same dict each step)."""
    if _S["seen"] is objects:
        return
    _S["seen"] = objects
    st = objects["knob"]
    pos = _pose(st)[1]
    kb, kt = knob_bottom(st), tilt_deg(st)
    v = float(np.linalg.norm(st.lin_vel))

    _S["fall"] = _S["fall"] + 1 if pos[2] < FALL_Z else 0
    _S["fell"] |= _S["fall"] >= FALL_STEPS

    # Straight-push jam: upright, actually touching the stud tip region, and not sliding down, for 0.5 s
    r = _axis_r(st)
    near = kt <= JAM_TILT_MAX and r <= 0.015 and JAM_Z_LOW <= kb <= STUD_TOP + 0.001
    pushing = near and float(st.lin_vel[2]) > -0.005 and stud_tip_gap(st) <= JAM_CONTACT
    _S["jam"] = _S["jam"] + 1 if pushing else 0
    _S["jammed"] |= _S["jam"] >= JAM_STEPS

    w_top = washer_top(objects)
    travel, law, past, theta = thread_state(objects)
    yaw = knob_yaw(st)
    eng = engaged(st, w_top, travel)
    if eng and _S["eng_prev"]:  # thread progress and the knob's own turn count only while the lug engages the thread
        _S["drawn"] += _S["law"] - law  # down positive, unscrewing subtracts
        _S["past_credit"] += past - _S["past"]
        _S["thread_cw"] += _S["theta"] - theta  # clockwise positive
        _S["knob_cw"] -= math.remainder(yaw - _S["yaw"], 2.0 * math.pi)
    _S["law"], _S["past"], _S["theta"], _S["yaw"], _S["eng_prev"] = law, past, theta, yaw, eng
    _S["on"] = _S["on"] + 1 if riding(st, w_top) else 0
    snug = _snug_now(st, w_top, travel, past)
    _S["still"] = _S["still"] + 1 if (snug and v < STILL_V_MAX and _fingers_clear(st)) else 0


def score(objects) -> float:
    """0 if the knob fell off the table or jammed (upright, touching the stud tip, not descending, 0.5 s). Else 0.25 on
    the stud and 3 cm down it; 0.5 squared on the washer for 1 s and still on it; 0.75 when the knob's own lug, engaged
    on the first thread crest, drove the thread its full run until the knob face met the washer and on 15 deg past
    contact, with the knob itself turned clockwise at least 0.8 x as far; 1.0 with that, both grippers 3 cm clear and
    the knob still for 1.5 s."""
    _track(objects)
    if _S["fell"] or _S["jammed"]:
        return 0.0
    st = objects["knob"]
    w_top = washer_top(objects)
    travel, _law, past, _theta = thread_state(objects)
    if riding(st, w_top) and _S["on"] >= ON_STEPS:
        if _snug_now(st, w_top, travel, past):
            return 1.0 if _S["still"] >= STILL_STEPS else 0.75
        return 0.5
    if _axis_r(st) <= STUD_R_TOL and knob_bottom(st) <= SLID_DOWN_Z:
        return 0.25
    return 0.0


def success(objects) -> bool:
    return score(objects) == 1.0


# -- scripted demo (right arm, closed loop) ----------------------------------------------------------------------------
demo = placeholder_demo

TILT_RAD = math.radians(13.0)
GRIP_ROLL = -math.pi / 2.0  # pads close along world x, on the +-x prong ends; the same hand as yaw +90 deg when level
GRIP_OFFSET = 0.0155  # TCP 15.5 mm above knob centre along knob +z: pad tips 2.5 mm above the prongs' underside
FINGER_GRIP = -0.0140  # 28 mm commanded gap across the 38 mm prong ends
HOLD_GAP = 0.030
REGRIP_OPEN = FINGER_OPEN * 0.75  # ~70 mm commanded: pads clear the 38 mm prongs while the open hand turns back
PRESS = 0.0010  # regrasp strokes: hand commanded 1 mm below the knob, a light push of the lug onto the crest
PRESS_HELD = 0.0020  # held first stroke: 2 mm, the face bears toward the washer and the lug loads the crest's run
STROKE = math.pi / 4.0  # regrasp strokes turn the wrist from +45 to -45 deg yaw: one prong pitch, clockwise from above
MAX_STROKES = 8
OFFLINE_STROKES = 3
TILT_TOL = math.radians(1.0)  # live knob tilt servo: stop within 1 deg of the 13 deg slide tilt
PITCH_TRIM_MAX = math.radians(10.0)
SLIDE_TRIES = 3
CENTRE_TOL = 0.0002
LEVEL_TOL_DEG = 0.4  # squaring servo: knob axis within 0.4 deg of vertical before turning
CENTRE_MAX = 0.002  # bound on the live centring of the knob on the stud while turning (m)


def _servo_move(
    arm,
    p1: tuple[float, float, float] | np.ndarray,
    duration: float,
    *,
    pitch: float = 0.0,
    yaw: float = 0.0,
    roll: float = 0.0,
    rot: np.ndarray | None = None,
    finger: float | None = None,
    comp: np.ndarray | None = None,
    max_comp: float = 0.035,
    adapt: bool = True,
):
    """Move TCP along smoothstep(p0 -> p1) while continuously compensating wrist sag from encoder FK."""
    if comp is None:
        comp = np.zeros(3, dtype=float)
    p0 = (arm.pos - comp).copy()
    p1_arr = np.asarray(p1, dtype=float)
    r0 = arm.rot.copy()
    r1 = closedloop.grip_rot(yaw=yaw, pitch=pitch, roll=roll) if rot is None else np.asarray(rot, float)
    f0 = arm.finger
    f1 = arm.finger if finger is None else float(finger)
    steps = max(1, round(float(duration) / arm.dt))
    prev_goal = p0.copy()
    for i in range(1, steps + 1):
        err = prev_goal - arm.measured_pos
        if adapt and not arm.offline:
            dcomp = np.clip(0.28 * err, -0.002, 0.002)
            comp[:] = np.clip(comp + dcomp, -max_comp, max_comp)
        s = ik.smoothstep(i / steps)
        goal = p0 + s * (p1_arr - p0)
        arm._set_cartesian(
            goal + comp,
            ik.slerp_matrix(r0, r1, s),
            f0 + s * (f1 - f0),
        )
        prev_goal = goal
        yield


def _live_thread(ctx):
    """The thread crest's ScrewState from the simulator, or None offline."""
    if ctx.right.offline:
        return None
    try:
        return mechkit.screw_states(THREAD, ctx.obj("thread"))["thread"]
    except Exception:  # noqa: BLE001
        return None


def _live_knob(ctx):
    """The knob's simulator state, or None offline (the dry run's resting states would mislead the servo)."""
    if ctx.right.offline:
        return None
    try:
        return ctx.obj("knob")
    except Exception:  # noqa: BLE001
        return None


def _knob_pitch(st) -> float:
    """The knob axis's lean toward -x (rad), the same sense as the hand's pitch."""
    ax = _pose(st)[0][:, 2]
    return float(math.atan2(-ax[0], ax[2]))


class _Hold:
    """Where the held knob sits in the hand: its centre in the hand frame. Nominal (GRIP_OFFSET along the knob axis)
    until measured live from the knob's simulator pose and the encoder TCP, so a knob that slipped or swung in the pinch
    is still placed by its own centre."""

    def __init__(self) -> None:
        self.off = np.array([0.0, 0.0, -GRIP_OFFSET])  # TCP to knob centre, hand frame: knob 15.5 mm down the approach

    def measure(self, arm, st) -> None:
        if st is not None:
            self.off = arm.rot.T @ (_pose(st)[1] - arm.measured_pos)

    def tcp(self, knob_c, pitch: float) -> np.ndarray:
        return np.asarray(knob_c, float) - closedloop.grip_rot(pitch=pitch, roll=GRIP_ROLL) @ self.off


def demo_script(ctx):
    R = ctx.right
    sx, sy = STAGE_XY
    stx, sty = STUD_XY
    hold = _Hold()

    # 1. Approach the staging peg, pinch the knob across its +-x prong ends (pads along x), lift clear
    pick_tcp = (sx, sy, KNOB_START[2] + GRIP_OFFSET)
    yield from R.move((sx, sy, TOP + 0.120), 3.0, FINGER_OPEN, roll=GRIP_ROLL)
    yield from R.move(pick_tcp, 1.6, FINGER_OPEN, roll=GRIP_ROLL)
    yield from R.trim_to_measured(pick_tcp, tol=0.001, max_duration=1.2)
    yield from R.grip(FINGER_GRIP, 0.8)
    yield from R.hold(0.30)
    if not R.holding(HOLD_GAP):
        yield from R.grip(FINGER_OPEN, 0.5)
        retry_tcp = (sx, sy, pick_tcp[2] - 0.002)
        yield from R.move(retry_tcp, 0.6, FINGER_OPEN, roll=GRIP_ROLL)
        yield from R.trim_to_measured(retry_tcp, tol=0.001, max_duration=1.2)
        yield from R.grip(FINGER_GRIP, 0.8)
        yield from R.hold(0.30)
        if not R.holding(HOLD_GAP):
            yield from R.grip(FINGER_OPEN, 0.5)
            yield from R.home(duration=3.0)
            return
    yield from R.move((sx, sy, STUD_TOP + 0.050), 2.0, roll=GRIP_ROLL)

    # 2. Tilt 13 deg high above the stud. The tilt is about y, across the pad line (x), so the pinch holds it; the hand
    #    pitch is still trimmed until the knob's own read tilt is 13 deg, and the knob is centred over the stud by its
    #    read centre.
    comp = np.zeros(3, dtype=float)
    pitch = TILT_RAD
    high_c = np.array([stx, sty, STUD_TOP + 0.044])
    entry_c = np.array([stx, sty, STUD_TOP + 0.020])
    yield from _servo_move(R, hold.tcp(high_c, pitch), 2.2, pitch=pitch, roll=GRIP_ROLL, comp=comp)
    yield from R.hold(0.3)

    def align(centre):
        nonlocal pitch
        for _ in range(3):
            st = _live_knob(ctx)
            if st is None:
                return
            err = TILT_RAD - _knob_pitch(st)
            hold.measure(R, st)
            if abs(err) > TILT_TOL:
                pitch = float(np.clip(pitch + err, TILT_RAD - PITCH_TRIM_MAX, TILT_RAD + PITCH_TRIM_MAX))
            yield from _servo_move(R, hold.tcp(centre, pitch), 0.6, pitch=pitch, roll=GRIP_ROLL, comp=comp)
            yield from R.hold(0.25)
            st = _live_knob(ctx)
            c = _pose(st)[1]
            if abs(TILT_RAD - _knob_pitch(st)) <= TILT_TOL and math.hypot(c[0] - centre[0], c[1] - centre[1]) <= 0.0006:
                return

    yield from align(high_c)
    yield from _servo_move(R, hold.tcp(entry_c, pitch), 1.0, pitch=pitch, roll=GRIP_ROLL, comp=comp)
    yield from R.hold(0.2)
    yield from align(entry_c)

    # 3. Holding the tilt, slide straight down (-z) to just above the washer. If the knob's read bottom stays more than
    #    4 mm above that (caught on a crest), back up to the entry, re-align the tilt and centre, and slide again.
    dz_touch = (KNOB_H / 2.0) * math.cos(TILT_RAD) + KNOB_R_OUT * math.sin(TILT_RAD)
    bot_c = np.array([stx, sty, WASHER_TOP_Z + dz_touch + 0.0015])
    for _ in range(SLIDE_TRIES):
        yield from _servo_move(R, hold.tcp(bot_c, pitch), 2.6, pitch=pitch, roll=GRIP_ROLL, comp=comp)
        yield from R.hold(0.2)
        st = _live_knob(ctx)
        if st is None or knob_bottom(st) <= WASHER_TOP_Z + 0.0015 + 0.004:
            break
        yield from _servo_move(R, hold.tcp(entry_c, pitch), 1.4, pitch=pitch, roll=GRIP_ROLL, comp=comp)
        yield from R.hold(0.2)
        yield from align(entry_c)

    # 4. Pivot to square: the bottom lug comes down onto the first thread crest (the half-threads catch). The knob's
    #    read tilt and centre are trimmed level and onto the stud axis before turning.
    crest_top0 = CROSSBAR_TOP + CREST_TOP0
    st = _live_knob(ctx)
    hold.measure(R, st)
    mid_pitch = 0.5 * TILT_RAD
    dz_mid = (KNOB_H / 2.0) * math.cos(mid_pitch) + KNOB_R_OUT * math.sin(mid_pitch) + 0.0008
    mid_c = np.array([stx, sty, crest_top0 - LUG_LIFT + dz_mid])
    yield from _servo_move(R, hold.tcp(mid_c, mid_pitch + (pitch - TILT_RAD) * 0.5), 0.9,
                           pitch=mid_pitch + (pitch - TILT_RAD) * 0.5, roll=GRIP_ROLL, comp=comp)
    seat_c = np.array([stx, sty, crest_top0 - LUG_LIFT + KNOB_H / 2.0 + 0.0002])
    sq = pitch - TILT_RAD  # the same in-hand lean offset, now about level
    yield from _servo_move(R, hold.tcp(seat_c, sq), 0.9, pitch=sq, roll=GRIP_ROLL, comp=comp)
    yield from R.hold(0.25)
    # Level the knob's whole axis (the pinch holds it by friction about the pad line too) and centre it on the stud:
    # rotate the hand by the small rotation that takes the knob's read axis to vertical, about the knob's centre.
    seat_rot = closedloop.grip_rot(pitch=sq, roll=GRIP_ROLL)
    for _ in range(6):
        st = _live_knob(ctx)
        if st is None:
            break
        ax = _pose(st)[0][:, 2]
        c = _pose(st)[1]
        lean = math.degrees(math.acos(float(np.clip(ax[2], -1.0, 1.0))))
        if lean <= LEVEL_TOL_DEG and math.hypot(c[0] - stx, c[1] - sty) <= CENTRE_TOL:
            break
        v_world = c - R.measured_pos  # TCP to knob centre
        k = np.cross(ax, [0.0, 0.0, 1.0])
        if np.linalg.norm(k) > 1e-9:
            seat_rot = closedloop.axis_angle_matrix(k / np.linalg.norm(k), math.radians(lean)) @ seat_rot
            v_world = closedloop.axis_angle_matrix(k / np.linalg.norm(k), math.radians(lean)) @ v_world
        yield from _servo_move(R, seat_c - v_world, 0.5, rot=seat_rot, comp=comp)
        yield from R.hold(0.25)
    seat_tcp = R.pos - comp

    def turn_rot(yaw: float, first: bool) -> np.ndarray:
        # first stroke: the seat attitude turned about the vertical stud axis, so the held knob stays level; after a
        # regrasp the knob rests squared on the washer and the hand closes on it level
        if first:
            return closedloop.axis_angle_matrix([0.0, 0.0, 1.0], yaw - math.pi / 2.0) @ seat_rot
        return closedloop.grip_rot(yaw=yaw)

    # 5. Turn strokes, about the knob's own centre on the stud axis. The first turns the held knob 135 deg clockwise
    #    from the +90 deg grasp to -45 deg (the whole run and past snug); each later one opens, turns the open hand back
    #    to +45 deg, closes level on the next prong pair and turns to -45 deg. Live, the hand keeps the knob centred on the stud (the bottom lug overlaps the
    #    crest by only 0.75 mm) and its height follows the read lug-to-crest gap so the lug rides the crest top. It
    #    follows the thread's read travel down and stops once the read thread is 15 deg past its seat plus margin.
    stud = np.array([stx, sty])

    def knob_vec() -> np.ndarray:
        """TCP to knob centre (xy) as read now; zero offline."""
        st = _live_knob(ctx)
        return np.zeros(2) if st is None else (_pose(st)[1][:2] - R.measured_pos[:2])

    cxy = np.zeros(2)

    def goal(z: float, yaw: float, yref: float, v: np.ndarray, dz: float = 0.0) -> np.ndarray:
        rz = closedloop.axis_angle_matrix([0.0, 0.0, 1.0], yaw - yref)[:2, :2]
        return np.array([*(stud - rz @ v + cxy), z + dz])

    def turn(z, press, ya, yb, first, v):
        steps = max(1, round(2.0 * (ya - yb) / (2.0 * STROKE) / R.dt))
        dz = -float(press)
        for i in range(1, steps + 1):
            yaw = ya + (yb - ya) * ik.smoothstep(i / steps)
            st = _live_knob(ctx)  # keep the knob centred on the stud while it turns
            if st is not None:
                cxy[:] = np.clip(cxy + 0.2 * (stud - _pose(st)[1][:2]), -CENTRE_MAX, CENTRE_MAX)
            R._set_cartesian(goal(z, yaw, ya, v, dz) + comp, turn_rot(yaw, first), R.finger)
            yield

    def centre(z, yaw, first, v):
        # Before pressing: hold the knob at the seat height and servo its read centre onto the stud axis (within
        # 0.2 mm for 3 steps, at most 1.5 s), since the bottom lug only overlaps the crest by 0.75 mm.
        if R.offline:
            return
        ok = 0
        for _ in range(round(1.5 / R.dt)):
            st = _live_knob(ctx)
            if st is None:
                return
            e = stud - _pose(st)[1][:2]
            ok = ok + 1 if float(np.hypot(*e)) <= CENTRE_TOL else 0
            if ok >= 3:
                return
            cxy[:] = np.clip(cxy + 0.3 * e, -CENTRE_MAX, CENTRE_MAX)
            R._set_cartesian(goal(z, yaw, yaw, v) + comp, turn_rot(yaw, first), R.finger)
            yield

    yaw0, yaw1 = math.pi / 2.0, -STROKE  # the held first stroke turns 135 deg: the whole run and past snug
    v = knob_vec()
    if R.offline:
        v = stud - seat_tcp[:2]
    for k in range(MAX_STROKES):
        th = _live_thread(ctx)
        z = seat_tcp[2] + (th.travel if th is not None else -min(k * 0.5 * RUN, RUN))
        first = k == 0
        if not first:
            yield from R.grip(REGRIP_OPEN, 0.5)
            st = _live_knob(ctx)
            kc = stud if st is None else _pose(st)[1][:2]  # the open hand comes down centred on the knob as it sits
            open_at = np.array([kc[0], kc[1], z + 0.002])
            yield from _servo_move(R, open_at, 1.2, rot=turn_rot(+STROKE, False), finger=REGRIP_OPEN, comp=comp)
            yield from _servo_move(R, open_at - np.array([0.0, 0.0, 0.002]), 0.4, rot=turn_rot(+STROKE, False),
                                   comp=comp)
            yield from R.grip(FINGER_GRIP, 0.6)
            yield from R.hold(0.2)
            if not R.holding(HOLD_GAP):
                continue
            yaw0, yaw1 = +STROKE, -STROKE
            v = knob_vec() if not R.offline else v
            cxy[:] = 0.0
        press = PRESS_HELD if first else PRESS
        yield from centre(z + (0.0002 if not first else 0.0), yaw0, first, v)
        yield from _servo_move(R, goal(z, yaw0, yaw0, v, -press), 0.4, rot=turn_rot(yaw0, first), comp=comp,
                               adapt=False)
        yield from turn(z, press, yaw0, yaw1, first, v)
        th = _live_thread(ctx)  # unload the push first, to the height the thread has now drawn the knob down to
        z_off = z if th is None else seat_tcp[2] + mechkit.screw_law(THREAD, "thread", 2.0 * math.pi * th.turns)[0]
        yield from _servo_move(R, goal(min(z, z_off), yaw1, yaw0, v), 0.4, rot=turn_rot(yaw1, first), comp=comp,
                               adapt=False)
        th = _live_thread(ctx)
        if th is None:
            if k + 1 >= OFFLINE_STROKES:
                break
        elif mechkit.screw_law(THREAD, "thread", 2.0 * math.pi * th.turns)[1] >= SNUG_TURNS + 0.02:
            break  # snug by the thread law (turn angle), not by the travel the push compresses

    # 6. Open, lift clear above the stud tip, return to READY, and settle >= 1.5 s
    yield from R.grip(FINGER_OPEN, 0.7)
    yield from R.move((stx, sty, STUD_TOP + 0.060), 1.8, FINGER_OPEN, yaw=yaw1)
    yield from R.home(duration=4.0)
    yield from R.hold(2.5)
