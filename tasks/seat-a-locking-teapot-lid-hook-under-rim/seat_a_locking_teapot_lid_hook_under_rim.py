"""On the station table: a dark walnut wood serving tray (22 x 46 cm, fixed) holding an empty glossy white porcelain
teapot (1.25 L size, 740 g, loose) with its spout pointing toward the right arm (-y) and C-handle toward the left arm
(+y). Beside the teapot on the right (-y) end of the tray sits its matching white porcelain locking lid (115 g, loose),
right-side up and turned 90 degrees so its underside locking hook points forward (+x) and its dark charcoal steam vent
on the dome faces the near edge (-x), directly opposite the hook. The lid's underside hook protrudes 12 mm past its
63 mm circular skirt (75 mm overall span across a 70 mm mouth opening), so lowering the lid flat jams the hook on top
of the rim. The teapot must be steadied while the lid is oriented, pitched nose-down to tuck the hidden hook beneath
the spout-side rim ledge, and rocked flat to seat.

Requires -E object_jitter=0: the teapot start mark and tray layout are fixed.

Run it:  inspect-robots run --embodiment yam_arena -E task=seat_a_locking_teapot_lid_hook_under_rim.py -E object_jitter=0 \\
           --instruction "Put the lid on the teapot so its underside hook (opposite the dark vent) locks under the spout-side rim and sits flat." \\
           --scorer task_score ...
"""

from __future__ import annotations

import dataclasses
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np
from belimits import Rect, bottom, is_airborne, tilt_deg
from closedloop import grip_rot, placeholder_demo
import mechkit
from mechkit import Mech, box, cone, cyl, lathe, on_reset, released, ring_profile, tube
from yam_arena.constants import FINGER_OPEN
from yam_arena.task_file import TABLE_TOP

SCENE = "station"
TOP = float(TABLE_TOP[SCENE])  # 0.7475 m
SUCCESS_HOLD_S = 8.0

# -- colours (sRGB) ----------------------------------------------------------------------------------------------------
WALNUT = (0.22, 0.14, 0.09)
TAPE_MARK = (0.55, 0.45, 0.32)
PORCELAIN = (0.95, 0.95, 0.93)
VENT_DARK = (0.10, 0.10, 0.10)

# -- tray geometry (fixed, dark walnut wood serving tray, 22 x 46 cm) --------------------------------------------------
TRAY_XY = (0.270, -0.035)
TRAY_L_X = 0.220  # x: 0.160 .. 0.380 m
TRAY_L_Y = 0.460  # y: -0.265 .. +0.195 m
TRAY_H = 0.010    # base thickness
TRAY_WALL_W = 0.010
TRAY_WALL_H = 0.014
TRAY_TOP = TOP + TRAY_H  # 0.7575 m
TRAY_X = (TRAY_XY[0] - TRAY_L_X / 2.0, TRAY_XY[0] + TRAY_L_X / 2.0)
TRAY_Y = (TRAY_XY[1] - TRAY_L_Y / 2.0, TRAY_XY[1] + TRAY_L_Y / 2.0)
TRAY_RECT = Rect(TRAY_X, TRAY_Y)

# -- teapot geometry (full-scale Seltmann Weiden Trio 1.25 L: 217 x 153 x 143 mm seated) -------------------------------
POT_XY = (0.270, -0.015)
POT_MASS = 0.74
PORCELAIN_MU = 0.45

Z_RIM = 0.109           # rim seat height above teapot base (109 mm; + 34 mm lid top = 143 mm total)
RIM_H = 0.005           # mouth collar thickness
Z_UNDER = Z_RIM - RIM_H  # 0.104 m: underside of overhanging inner rim lip
R_MOUTH = 0.035         # 70 mm circular gallery mouth bore
R_COLLAR_OUT = 0.061    # matches upper belly outer radius

# Spout runs along teapot local -y from belly wall to pouring lip
SPOUT_P0 = np.array([0.0, -0.064, 0.044])
SPOUT_P1 = np.array([0.0, -0.103, Z_RIM - 0.002])
_SPOUT_D = SPOUT_P1 - SPOUT_P0
_SPOUT_L = float(np.linalg.norm(_SPOUT_D))
_SPOUT_MID = tuple(float(v) for v in (SPOUT_P0 + SPOUT_P1) / 2.0)
_SPOUT_TILT = mechkit._quat_z_to(_SPOUT_D / _SPOUT_L)

# C-handle along teapot local +y (18.5 mm clear of widest belly)
HANDLE_GRIP_LOCAL = (0.0, 0.101, 0.0605)

POT_SHAPES = (
    cyl((0.0, 0.0, 0.003), 0.056, 0.006, PORCELAIN),
    ring_profile(
        (0.0, 0.0, 0.0),
        [(0.053, 0.058, 0.006), (0.0715, 0.0765, 0.055), (0.056, 0.061, Z_UNDER)],
        color=PORCELAIN,
        sides=24,
    ),
    tube(
        (0.0, 0.0, Z_UNDER + RIM_H / 2.0),
        r_in=R_MOUTH,
        r_out=R_COLLAR_OUT,
        height=RIM_H,
        color=PORCELAIN,
        sides=24,
        lead_in=0.001,
    ),
    cone(_SPOUT_MID, 0.014, 0.0085, _SPOUT_L, color=PORCELAIN, tilt=_SPOUT_TILT, sides=16),
    box((0.0, 0.080, 0.089), (0.014, 0.046, 0.010), PORCELAIN),
    box((0.0, 0.081, 0.032), (0.014, 0.044, 0.010), PORCELAIN),
    box(HANDLE_GRIP_LOCAL, (0.014, 0.012, 0.067), PORCELAIN),
)

TEAPOT = Mech(
    "teapot",
    pos=(float(POT_XY[0]), float(POT_XY[1]), float(TRAY_TOP)),
    mass=POT_MASS,
    friction=PORCELAIN_MU,
    shapes=POT_SHAPES,
)

# -- lid geometry (origin at flange underside z = 0; local -y = hook, local +y = steam vent) ---------------------------
LID_XY = (0.270, -0.195)
LID_START_YAW = math.pi / 2.0  # +90 deg: hook points +x, dark steam vent faces -x (toward camera)
LID_MASS = 0.115

SKIRT_H = 0.019        # 19 mm skirt depth (14 mm vertical slot between flange underside and hook top)
SKIRT_R_IN = 0.0270
SKIRT_R_OUT = 0.0315   # 63 mm outer skirt (3.5 mm radial clearance in 70 mm mouth)
HOOK_W = 0.015         # 15 mm width along local x
HOOK_L = 0.022         # 22 mm length along local -y: y = -0.0215 .. -0.0435 (12 mm past skirt)
HOOK_H = 0.005         # 5 mm thickness: z = -0.019 .. -0.014 (coplanar bottom with skirt)
HOOK_R_TIP = 0.0435    # radial reach of hook tip (75 mm overall span across skirt + hook vs 70 mm mouth)
HOOK_Z_TOP = -SKIRT_H + HOOK_H  # -0.014 m in lid frame
LID_CAP_R = 0.0405     # 81 mm cap diameter
LID_TOP_H = 0.034      # 34 mm cap + knob above flange (53 mm total lid height)
KNOB_GRASP_Z = 0.0305  # TCP height in lid frame (pad tips 13 mm below at 0.0175, 4.5 mm above dome 0.013)

# Look-only dark charcoal steam vent marker on the lid dome opposite the hook (local +y)
_VENT_SHAPE = dataclasses.replace(
    cyl((0.0, 0.027, 0.0092), 0.0045, 0.0022, VENT_DARK, tilt=mechkit._quat_z_to((0.0, 0.25, 1.0))),
    collide=False,
)

LID_SHAPES = (
    tube((0.0, 0.0, -SKIRT_H / 2.0), r_in=SKIRT_R_IN, r_out=SKIRT_R_OUT, height=SKIRT_H, color=PORCELAIN, sides=24),
    box((0.0, -0.0325, -SKIRT_H + HOOK_H / 2.0), (HOOK_W, HOOK_L, HOOK_H), PORCELAIN),
    tube((0.0, 0.0, 0.002), r_in=SKIRT_R_IN, r_out=LID_CAP_R, height=0.004, color=PORCELAIN, sides=24),
    ring_profile(
        (0.0, 0.0, 0.0),
        [(SKIRT_R_IN, LID_CAP_R, 0.004), (0.0180, 0.0300, 0.0085), (0.0080, 0.0160, 0.0125)],
        color=PORCELAIN,
        sides=24,
    ),
    cyl((0.0, 0.0, 0.0110), 0.012, 0.004, PORCELAIN, sides=24),
    lathe(
        (0.0, 0.0, 0.0),
        [
            (0.0075, 0.0130),
            (0.0075, 0.0185),
            (0.0120, 0.0195),
            (0.0120, 0.0320),
            (0.0095, LID_TOP_H),
        ],
        color=PORCELAIN,
        sides=24,
    ),
    _VENT_SHAPE,
)

LID = Mech(
    "lid",
    pos=(float(LID_XY[0]), float(LID_XY[1]), float(TRAY_TOP + SKIRT_H)),
    yaw=float(LID_START_YAW),
    mass=LID_MASS,
    friction=PORCELAIN_MU,
    shapes=LID_SHAPES,
)

# Serving tray with subtle taped start-mark rings (look-only)
_TRAY_POT_MARK = dataclasses.replace(
    tube((POT_XY[0] - TRAY_XY[0], POT_XY[1] - TRAY_XY[1], TRAY_H + 0.0003), 0.056, 0.060, 0.0006, TAPE_MARK, sides=24),
    collide=False,
)
_TRAY_LID_MARK = dataclasses.replace(
    tube((LID_XY[0] - TRAY_XY[0], LID_XY[1] - TRAY_XY[1], TRAY_H + 0.0003), 0.032, 0.036, 0.0006, TAPE_MARK, sides=24),
    collide=False,
)

TRAY = Mech(
    "tray",
    pos=(float(TRAY_XY[0]), float(TRAY_XY[1]), float(TOP)),
    mass=None,
    friction=0.6,
    shapes=(
        box((0.0, 0.0, TRAY_H / 2.0), (TRAY_L_X, TRAY_L_Y, TRAY_H), WALNUT),
        box((-TRAY_L_X / 2.0 + TRAY_WALL_W / 2.0, 0.0, TRAY_H + TRAY_WALL_H / 2.0),
            (TRAY_WALL_W, TRAY_L_Y, TRAY_WALL_H), WALNUT),
        box((TRAY_L_X / 2.0 - TRAY_WALL_W / 2.0, 0.0, TRAY_H + TRAY_WALL_H / 2.0),
            (TRAY_WALL_W, TRAY_L_Y, TRAY_WALL_H), WALNUT),
        box((0.0, -TRAY_L_Y / 2.0 + TRAY_WALL_W / 2.0, TRAY_H + TRAY_WALL_H / 2.0),
            (TRAY_L_X - 2.0 * TRAY_WALL_W, TRAY_WALL_W, TRAY_WALL_H), WALNUT),
        box((0.0, TRAY_L_Y / 2.0 - TRAY_WALL_W / 2.0, TRAY_H + TRAY_WALL_H / 2.0),
            (TRAY_L_X - 2.0 * TRAY_WALL_W, TRAY_WALL_W, TRAY_WALL_H), WALNUT),
        _TRAY_POT_MARK,
        _TRAY_LID_MARK,
    ),
)

OBJECTS = [TRAY, TEAPOT, LID]

# -- scoring constants and whole-run violation latches -----------------------------------------------------------------
STEP_HZ = 20.0
RELEASE_CLEAR = 0.030    # m: both grippers at least 30 mm from teapot and lid
STILL_SPEED = 0.010      # m/s: linear speed threshold for rest
STILL_SPIN = 0.50        # rad/s: angular speed threshold for rest
STILL_STEPS = 20         # 1.0 s at 20 Hz
VIOL_HOLD_STEPS = 6      # 0.3 s at 20 Hz for tip / off-tray violations
DROP_SPEED = 0.90        # m/s: lid dropped or thrown
DROP_HOLD_STEPS = 2      # 0.1 s at 20 Hz

SEAT_TILT_DEG = 3.0
SEAT_Z_TOL = 0.0025      # 2.5 mm vertical tolerance on rim seat
SEAT_XY_TOL = 0.0040     # 4.0 mm concentricity tolerance
HOOK_RADIAL_MIN = R_MOUTH + 0.005  # 40 mm (>= 5 mm past 35 mm inner mouth rim)
HOOK_TOP_MAX = Z_UNDER - 0.001     # 103 mm in pot frame (>= 1 mm below 104 mm rim underside)
ALIGN_TOL_DEG = 25.0
POT_TILT_DEG = 3.0
POT_TIP_VIOL_DEG = 15.0
POT_MOVE_TOL = 0.030     # 30 mm from start mark on tray

OFFLINE_FINGERS_CLEAR = True  # flipped to False in offline tests to verify held-part rejection

_L = {
    "pot_bad": False,
    "lid_off": False,
    "lid_fast": False,
    "pot_bad_n": 0,
    "lid_off_n": 0,
    "lid_fast_n": 0,
    "still": 0,
    "seen": None,
    "prev_lid": None,
}


@on_reset
def _clear(_env=None):
    _L.update(
        pot_bad=False,
        lid_off=False,
        lid_fast=False,
        pot_bad_n=0,
        lid_off_n=0,
        lid_fast_n=0,
        still=0,
        seen=None,
        prev_lid=None,
    )


def _xy_in_tray(p) -> bool:
    return TRAY_RECT.x0 <= float(p[0]) <= TRAY_RECT.x1 and TRAY_RECT.y0 <= float(p[1]) <= TRAY_RECT.y1


def _fingers_clear(objects) -> bool:
    """True when both arms' finger pads are >= 30 mm clear of both teapot and lid."""
    if mechkit._ENV["env"] is None:
        return bool(OFFLINE_FINGERS_CLEAR)
    return bool(
        released(objects["teapot"], RELEASE_CLEAR) and released(objects["lid"], RELEASE_CLEAR)
    )


def _update(objects) -> None:
    """Update whole-run violation latches and rest counter once per control step."""
    if objects is _L["seen"]:
        return
    _L["seen"] = objects
    pot, lid = objects["teapot"], objects["lid"]

    # Fallback reset detector when run without on_reset (e.g. direct repeated trials)
    lid_origin = np.asarray(lid.origin, dtype=float)
    start_lid = np.asarray(LID.pos, dtype=float)
    d_lid = float(np.linalg.norm(lid_origin - start_lid))
    if (
        _L["prev_lid"] is not None
        and _L["prev_lid"] > 0.03
        and d_lid <= 0.005
        and float(np.linalg.norm(lid.lin_vel)) < STILL_SPEED
    ):
        _clear()
        _L["seen"] = objects
    _L["prev_lid"] = d_lid

    pot_base = pot.to_world((0.0, 0.0, 0.0))
    pot_bad_now = (
        tilt_deg(pot) > POT_TIP_VIOL_DEG
        or bottom(pot, TEAPOT) < TRAY_TOP - 0.005
        or not _xy_in_tray(pot_base)
    )
    _L["pot_bad_n"] = _L["pot_bad_n"] + 1 if pot_bad_now else 0
    if _L["pot_bad_n"] >= VIOL_HOLD_STEPS:
        _L["pot_bad"] = True

    # Lid off tray: outside tray xy footprint OR dropped below tray top onto table/floor
    lid_off_now = (not _xy_in_tray(lid_origin)) or (bottom(lid, LID) < TRAY_TOP - 0.005)
    _L["lid_off_n"] = _L["lid_off_n"] + 1 if lid_off_now else 0
    if _L["lid_off_n"] >= VIOL_HOLD_STEPS:
        _L["lid_off"] = True

    lid_fast_now = float(np.linalg.norm(lid.lin_vel)) > DROP_SPEED
    _L["lid_fast_n"] = _L["lid_fast_n"] + 1 if lid_fast_now else 0
    if _L["lid_fast_n"] >= DROP_HOLD_STEPS:
        _L["lid_fast"] = True

    both_still = (
        float(np.linalg.norm(pot.lin_vel)) < STILL_SPEED
        and float(np.linalg.norm(pot.ang_vel)) < STILL_SPIN
        and float(np.linalg.norm(lid.lin_vel)) < STILL_SPEED
        and float(np.linalg.norm(lid.ang_vel)) < STILL_SPIN
        and _fingers_clear(objects)
    )
    _L["still"] = _L["still"] + 1 if both_still else 0


def pot_ok(pot) -> bool:
    """Teapot upright (<= 3 deg), resting on the tray, base centre within 30 mm of start mark."""
    base = pot.to_world((0.0, 0.0, 0.0))
    return bool(
        tilt_deg(pot) <= POT_TILT_DEG
        and abs(bottom(pot, TEAPOT) - TRAY_TOP) <= 0.005
        and math.hypot(float(base[0]) - POT_XY[0], float(base[1]) - POT_XY[1]) <= POT_MOVE_TOL
        and _xy_in_tray(base)
    )


def lid_seated(pot, lid) -> bool:
    """Lid upright, flat on teapot rim seat, centred, with underside hook tucked under inner rim lip.
    Evaluated in the teapot's live local frame."""
    if tilt_deg(lid) > SEAT_TILT_DEG:
        return False
    r_rel = pot.rotation().T @ lid.rotation()
    if float(r_rel[2, 2]) < math.cos(math.radians(SEAT_TILT_DEG + 1.0)):
        return False
    flange_pot = pot.to_local(lid.to_world((0.0, 0.0, 0.0)))
    if abs(float(flange_pot[2]) - Z_RIM) > SEAT_Z_TOL:
        return False
    if math.hypot(float(flange_pot[0]), float(flange_pot[1])) > SEAT_XY_TOL:
        return False
    hook_tip_c = pot.to_local(lid.to_world((0.0, -HOOK_R_TIP, HOOK_Z_TOP)))
    if math.hypot(float(hook_tip_c[0]), float(hook_tip_c[1])) < HOOK_RADIAL_MIN:
        return False
    for sx in (-HOOK_W / 2.0, 0.0, HOOK_W / 2.0):
        pt_pot = pot.to_local(lid.to_world((sx, -HOOK_R_TIP, HOOK_Z_TOP)))
        if float(pt_pot[2]) > HOOK_TOP_MAX:
            return False
    return True


def hook_aligned(pot, lid) -> bool:
    """Hook heading within 25 deg of the teapot's spout direction (pot local -y)."""
    h_pot = pot.rotation().T @ (lid.rotation() @ np.array([0.0, -1.0, 0.0]))
    ang = abs(math.atan2(float(h_pot[0]), -float(h_pot[1])))
    return bool(math.degrees(ang) <= ALIGN_TOL_DEG)


def score(objects) -> float:
    """0.0 on whole-run violation (pot tipped > 15 deg or off tray for 0.3 s, lid off tray for 0.3 s, or lid speed
    > 0.9 m/s for 0.1 s) or if not released (>= 30 mm clear, not airborne, both parts < 10 mm/s for 1.0 s).
    Otherwise 1.0 if lid seated, aligned within 25 deg of spout, and pot upright on mark; 0.5 if lid seated and pot
    ok with hook misaligned > 25 deg; else 0.0."""
    _update(objects)
    if _L["pot_bad"] or _L["lid_off"] or _L["lid_fast"]:
        return 0.0
    pot, lid = objects["teapot"], objects["lid"]
    pot_rim_z = float(pot.to_world((0.0, 0.0, Z_RIM))[2])
    if is_airborne(pot, TEAPOT, [TRAY_TOP]) or is_airborne(lid, LID, [TRAY_TOP, pot_rim_z - SKIRT_H]):
        return 0.0
    if _L["still"] < STILL_STEPS or not _fingers_clear(objects):
        return 0.0
    if not (pot_ok(pot) and lid_seated(pot, lid)):
        return 0.0
    return 1.0 if hook_aligned(pot, lid) else 0.5


def success(objects) -> bool:
    return score(objects) == 1.0


# -- arm keep-out: padded 15 mm around the protruding porcelain spout --------------------------------------------------
# Spout world AABB at start: x 0.256..0.284, y -0.1253..-0.067, z 0.794..0.870 (TOP+0.047..TOP+0.1225).
SPOUT_KO_X = (0.240, 0.300)
SPOUT_KO_Y = (-0.141, -0.055)
SPOUT_KO_Z_TOP = TOP + 0.1375  # 0.8850 m (15 mm above spout top)

ARM_KEEP_OUT = [
    {"name": "porcelain spout (+15 mm pad)", "x": SPOUT_KO_X, "y": SPOUT_KO_Y, "z_top": SPOUT_KO_Z_TOP},
]

# -- scripted demo (bimanual, closed loop) -----------------------------------------------------------------------------
demo = placeholder_demo

_OPEN_HANDLE = 0.046
_CLOSE_HANDLE = 0.008
_HOLD_HANDLE = 0.011
_OPEN_LID = 0.056
_CLOSE_LID = 0.016
_HOLD_LID = 0.019

_TRANSFER_MPS = 0.072  # peak smoothstep = 1.5 * 0.072 = 0.108 m/s (well below 0.15 m/s limit)
_SLOW_MPS = 0.032
_TUCK_MPS = 0.020


def _finger(gap_m: float) -> float:
    return float(FINGER_OPEN) * float(gap_m) / (2.0 * abs(float(FINGER_OPEN)))


def _rx(deg: float) -> np.ndarray:
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]], dtype=float)


def _rz(rad: float) -> np.ndarray:
    c, s = math.cos(rad), math.sin(rad)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=float)


def _move_lin(arm, pos, speed=_TRANSFER_MPS, min_dur=0.6, finger=None, rot=None):
    pos = np.asarray(pos, dtype=float)
    dist = float(np.linalg.norm(pos - arm.pos))
    dur = max(float(min_dur), dist / float(speed))
    yield from arm.move(pos, duration=dur, finger=finger, rot=rot)


def _yaw_of(rot: np.ndarray) -> float:
    return math.atan2(float(rot[1, 0]), float(rot[0, 0]))


# Dip -> tuck -> rock-flat lid poses in teapot local frame: (pitch_deg, dy, dz_above_rim)
# Every segment verified offline with mechkit.gap() >= +2.2 mm against the 24-sided collar.
INSERT_PATH = (
    (30.0, +0.012, 0.046),   # pre_entry: above mouth, pitched 30 deg nose-down
    (30.0, +0.001, 0.025),   # dip: nose enters mouth, hook drops 4 mm below rim underside
    (28.0, -0.007, 0.022),   # tuck: hook slides 3.8 mm under spout-side rim lip
    (18.0, -0.005, 0.015),   # rock_18: begin rocking rear edge down
    (8.0,  -0.002, 0.008),   # rock_8: continue rocking flat
    (0.0,   0.000, 0.0025),  # release: flat 2.5 mm above rim seat
)


def demo_script(ctx):
    """Left arm steadies the teapot by its C-handle from +y at a 45 deg downward pitch while right arm picks the lid
    by its knob, lifts clear, turns and pitches nose-down, routes forward of the spout keep-out zone, dips the hook
    into the mouth, tucks it under the spout-side rim ledge, rocks flat to seat, and both arms release and park."""
    L, R = ctx.left, ctx.right
    r_g0 = grip_rot(yaw=0.0, pitch=0.0, roll=0.0)
    r_left = grip_rot(yaw=-math.pi / 2.0, pitch=math.pi / 4.0, roll=0.0)
    live = not ctx.offline

    # Read initial object poses (live simulator state when online; fixed layout when offline)
    if live:
        pot0 = ctx.obj("teapot")
        lid0 = ctx.obj("lid")
        handle_w = np.asarray(pot0.to_world(HANDLE_GRIP_LOCAL), dtype=float)
        lid_knob_w = np.asarray(lid0.to_world((0.0, 0.0, KNOB_GRASP_Z)), dtype=float)
        lid_yaw0 = _yaw_of(lid0.rotation())
    else:
        handle_w = np.array([POT_XY[0], POT_XY[1] + HANDLE_GRIP_LOCAL[1], TRAY_TOP + HANDLE_GRIP_LOCAL[2]])
        lid_knob_w = np.array([LID_XY[0], LID_XY[1], TRAY_TOP + SKIRT_H + KNOB_GRASP_Z])
        lid_yaw0 = LID_START_YAW

    r_pick = grip_rot(yaw=lid_yaw0, pitch=0.0, roll=0.0)

    # 1) Bimanual preparation: left steadies handle while right picks and lifts lid
    def left_steady():
        pre = handle_w + np.array([0.0, 0.058, 0.058])
        yield from _move_lin(L, pre, speed=_TRANSFER_MPS, min_dur=2.2, finger=_finger(_OPEN_HANDLE), rot=r_left)
        yield from L.trim_to_measured(pre, tol=0.002)
        yield from _move_lin(L, handle_w, speed=_SLOW_MPS, min_dur=1.6, rot=r_left)
        yield from L.trim_to_measured(handle_w, tol=0.0015)
        for attempt in range(2):
            yield from L.grip(_finger(_CLOSE_HANDLE), 0.7)
            yield from L.hold(0.3)
            if L.holding(_HOLD_HANDLE):
                break
            if attempt == 0:
                yield from L.grip(_finger(_OPEN_HANDLE), 0.5)
                yield from _move_lin(L, L.pos + np.array([0.0, -0.003, -0.003]), speed=0.02, min_dur=0.4, rot=r_left)

    def right_pick_and_lift():
        hover = np.array([lid_knob_w[0], lid_knob_w[1], TRAY_TOP + 0.130])
        lift = np.array([lid_knob_w[0], lid_knob_w[1], TOP + 0.185])
        yield from _move_lin(R, hover, speed=_TRANSFER_MPS, min_dur=2.2, finger=_finger(_OPEN_LID), rot=r_pick)
        yield from R.trim_to_measured(hover, tol=0.002)
        yield from _move_lin(R, lid_knob_w, speed=_SLOW_MPS, min_dur=1.6, rot=r_pick)
        yield from R.trim_to_measured(lid_knob_w, tol=0.0015)
        for attempt in range(2):
            yield from R.grip(_finger(_CLOSE_LID), 0.7)
            yield from R.hold(0.3)
            if R.holding(_HOLD_LID):
                break
            if attempt == 0:
                yield from R.grip(_finger(_OPEN_LID), 0.5)
                yield from _move_lin(R, R.pos - np.array([0.0, 0.0, 0.0025]), speed=0.02, min_dur=0.4, rot=r_pick)
        yield from _move_lin(R, lift, speed=0.055, min_dur=1.8, rot=r_pick)

    yield from ctx.both(left_steady(), right_pick_and_lift())

    # 2) Orient lid hook toward spout (-y) and pre-pitch 28 deg while clear of spout
    r_pre28 = _rx(28.0) @ r_g0
    lift_pt = np.array([lid_knob_w[0], lid_knob_w[1], TOP + 0.185])
    yield from R.move(lift_pt, duration=1.8, rot=r_pre28)
    yield from R.hold(0.2)

    # 3) Route smoothly around and above the spout keep-out zone (x = 0.215 < 0.240, z = TOP + 0.192 > 0.885)
    r_pre30 = _rx(30.0) @ r_g0
    via1 = np.array([0.215, -0.155, TOP + 0.192])
    via2 = np.array([0.215, -0.020, TOP + 0.192])
    yield from _move_lin(R, via1, speed=_TRANSFER_MPS, min_dur=1.2, rot=r_pre28)
    yield from _move_lin(R, via2, speed=_TRANSFER_MPS, min_dur=2.0, rot=r_pre30)

    # Compute nominal gripper-to-lid offset; when live, trim from actual live lid & pot poses!
    t_g2l = r_g0.T @ np.array([0.0, 0.0, -KNOB_GRASP_Z])  # lid origin in gripper frame
    r_g2l = r_g0.T                                         # lid rot in gripper frame (at yaw=0, pitch=0)

    def pot_frame():
        if live:
            p = ctx.obj("teapot")
            return p.rotation(), np.asarray(p.origin, dtype=float)
        return np.eye(3), np.array([POT_XY[0], POT_XY[1], TRAY_TOP], dtype=float)

    def lid_to_tcp(pitch_deg: float, dy: float, dz: float, t_off=None, r_off=None):
        rp, op = pot_frame()
        r_lid_w = rp @ _rx(pitch_deg)
        p_lid_w = op + rp @ np.array([0.0, dy, Z_RIM + dz], dtype=float)
        ro = r_g2l if r_off is None else r_off
        to = t_g2l if t_off is None else t_off
        r_grip = r_lid_w @ ro.T
        p_grip = p_lid_w - r_grip @ to
        return p_grip, r_grip

    # Move to pre_entry over teapot mouth
    p_pre, r_pre = lid_to_tcp(*INSERT_PATH[0])
    yield from _move_lin(R, p_pre, speed=_TRANSFER_MPS, min_dur=1.2, rot=r_pre)
    yield from R.trim_to_measured(p_pre, tol=0.0015)
    yield from R.hold(0.25)

    # Measure actual grip offset and PD wrist sag right above the mouth (live runs)
    if live:
        lid_now = ctx.obj("lid")
        r_m = R.measured_rot
        r_g2l_live = r_m.T @ lid_now.rotation()
        t_g2l_live = r_m.T @ (np.asarray(lid_now.origin, dtype=float) - R.measured_pos)
        sag_ff = R.pos - R.measured_pos
    else:
        r_g2l_live = r_g2l
        t_g2l_live = t_g2l
        sag_ff = np.zeros(3)

    # 4) Execute dip -> tuck -> rock flat -> release with sag compensation
    step_durs = (1.3, 1.3, 1.3, 1.3, 1.4)
    for (pdeg, dy, dz), dur in zip(INSERT_PATH[1:], step_durs, strict=True):
        pg, rg = lid_to_tcp(pdeg, dy, dz, t_g2l_live, r_g2l_live)
        yield from R.move(pg + sag_ff, duration=dur, rot=rg)
        if live and pdeg in (28.0, 0.0):
            yield from R.hold(0.15)
            # Re-check actual lid pose vs desired pose and trim gently if off by > 1 mm
            lid_now = ctx.obj("lid")
            rp, op = pot_frame()
            want_p = op + rp @ np.array([0.0, dy, Z_RIM + dz], dtype=float)
            err = want_p - np.asarray(lid_now.origin, dtype=float)
            err_norm = float(np.linalg.norm(err))
            if err_norm > 0.0012:
                step = err * min(1.0, 0.006 / err_norm)
                yield from R.move(R.pos + step, duration=0.5, rot=rg)

    yield from R.hold(0.3)

    # 5) Release lid and rise clear vertically; then withdraw both arms in parallel and park at READY
    yield from R.grip(_finger(_OPEN_LID), 0.7)
    yield from R.hold(0.4)
    rel_pos = R.pos.copy()
    yield from R.move(rel_pos + np.array([0.0, 0.0, 0.032]), duration=1.0, rot=_rx(14.0) @ r_g0)

    def left_release_home():
        yield from L.grip(_finger(_OPEN_HANDLE), 0.7)
        yield from L.hold(0.25)
        yield from _move_lin(L, L.pos + np.array([0.0, 0.065, 0.065]), speed=_TRANSFER_MPS, min_dur=1.6, rot=r_left)
        yield from L.home(duration=4.5)

    def right_retreat_home():
        yield from R.move(rel_pos + np.array([-0.030, -0.055, 0.075]), duration=1.8, rot=_rx(26.0) @ r_g0)
        yield from R.home(duration=4.5)

    yield from ctx.both(left_release_home(), right_retreat_home())
    yield from ctx.wait(5.0)
