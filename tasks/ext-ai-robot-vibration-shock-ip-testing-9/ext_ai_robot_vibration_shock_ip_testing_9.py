"""Vibration/shock qualification fixture: Carr Lane CLM-8-QKK quick-acting knurled knob on an M8x1.25 stud and
Qualmark-style aluminum crossbar over an edge compute module on a T-slot subplate.

On the station table (TOP = 0.7475 m):
- An aluminum T-slot subplate (160 x 200 x 12 mm, fixed) at (0.260, 0.0, TOP) holding a dark anodized edge compute
  enclosure (90 x 103 x 35 mm) centered at (0.260, 0.0, TOP + 0.012).
- A fixed 6061-T6 aluminum crossbar (180 x 25 x 12 mm) spanning y = -0.090..+0.090 across the module (crossbar top at
  TOP + 0.059), supported on two 35 mm spacer sleeves at y = +0.075 (pre-clamped by a hex nut) and y = -0.075 (open M8
  stud rising 55 mm above the crossbar to TOP + 0.114).
- An anodized blue staging block (44 x 44 x 12 mm) with an 18 mm OD, 10 mm pedestal and brass 5.2 mm staging peg at
  (0.200, -0.150, TOP).
- A loose black-oxide steel Carr Lane CLM-8-QKK quick-acting knurled knob (22 mm OD x 18 mm tall, 35 g) starting
  upright on the staging peg. Its relieved bore clears the M8 stud crests only when tilted ~18 deg off axis; pushing
  straight down at <= 5 deg tilt jams on the top crests. After sliding down onto the crossbar and squaring to <= 3 deg,
  rotating >= 0.5 turns clockwise while seated snugs the crossbar.
"""

from __future__ import annotations

import dataclasses
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from belimits import bottom, tilt_deg
import closedloop
from closedloop import placeholder_demo
import mechkit
from mechkit import Mech, box, cone, cyl, on_reset, released, tube, bore
from yam_arena import ik
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.task_file import TABLE_TOP

TOP = TABLE_TOP["station"]
SUCCESS_HOLD_S = 8.0
ARM_KEEP_OUT = []
OFFLINE_FINGERS_CLEAR = True  # set False in offline check.py to test unreleased scoring (0.75)

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

KNOB_R_OUT = 0.0110
KNOB_R_IN = 0.0090
KNOB_H = 0.018
KNOB_MASS = 0.028  # ~3.1 cm^3 steel at 7.87 g/cm^3 -> ~25-28 g
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
    cyl((0.0, -STUD_Y_OFFSET, _cb_top_local + 0.009), STUD_CREST_R, 0.0028, CREST_DARK, contact=0.00025),
]
for _dz, _ch in ((0.0255, 0.0022), (0.0325, 0.0022), (0.0395, 0.0022), (0.0465, 0.0022), (0.0535, 0.0020)):
    _crossbar_shapes.append(
        cyl((0.0, -STUD_Y_OFFSET, _cb_top_local + _dz), STUD_CREST_R, _ch, CREST_DARK, contact=0.00025)
    )
CROSSBAR = Mech("crossbar", pos=(PLATE_XY[0], PLATE_XY[1], TOP + PLATE_SIZE[2]), mass=None, shapes=_crossbar_shapes)

KNOB = Mech(
    "knob",
    pos=KNOB_START,
    mass=KNOB_MASS,
    friction=0.6,
    shapes=[
        tube((0.0, 0.0, 0.0), KNOB_R_IN, KNOB_R_OUT, KNOB_H, KNOB_BLACK, sides=18, contact=0.00025),
        # Opposing half-thread lugs: top -x (x_inner = -3.25 mm) and bottom +x (x_inner = +3.25 mm)
        box((-0.006125, 0.0, +0.0077), (0.00575, 0.0076, 0.0022), KNOB_LUG, contact=0.00025),
        box((+0.006125, 0.0, -0.0077), (0.00575, 0.0076, 0.0022), KNOB_LUG, contact=0.00025),
        # Y-side bore guide walls keeping 12.8 mm bore width across Y (>= 2.4 mm clearance per side)
        box((0.0, +0.0081, 0.0), (0.0060, 0.0034, KNOB_H), KNOB_LUG, contact=0.00025),
        box((0.0, -0.0081, 0.0), (0.0060, 0.0034, KNOB_H), KNOB_LUG, contact=0.00025),
        # Top alignment indicator mark showing tilt-relief (-x) side
        _vbox((-0.0094, 0.0, KNOB_H / 2.0 + 0.0004), (0.0032, 0.0080, 0.0008), INDEX_GOLD),
    ],
)

OBJECTS = [PLATE, STAGE, CROSSBAR, KNOB]

# -- scoring -----------------------------------------------------------------------------------------------------------
STUD_R_TOL = 0.006
SEAT_Z_TOL = 0.002
SEAT_TILT_MAX = 3.0
TIP_TILT_MAX = 30.0
TIP_Z_MIN = TOP - 0.05
JAM_TILT_MAX = 5.0
JAM_R_TOL = 0.014
JAM_Z_BAND = 0.006
REQUIRED_CW_TURNS = 0.50
STILL_V_MAX = 0.002
STILL_STEPS = 30  # 1.5 s at 20 Hz
SUSTAIN_TIP_STEPS = 3  # 0.15 s at 20 Hz
SUSTAIN_JAM_STEPS = 3  # 0.15 s at 20 Hz
RELEASE_CLEAR = 0.03


class _Latch:
    def __init__(self) -> None:
        self.clear()

    def clear(self) -> None:
        self.seen = None
        self.prev_start_dist: float | None = None
        self.tip_streak = 0
        self.jam_streak = 0
        self.tip_latched = False
        self.jam_latched = False
        self.seated_prev = False
        self.prev_theta: float | None = None
        self.cw_turns = 0.0
        self.still_steps = 0


LATCH = _Latch()


@on_reset
def _reset_latch(env=None) -> None:
    LATCH.clear()


def _knob_yaw(st) -> float:
    x, y, z, w = (float(v) for v in st.quat_xyzw)
    ux = 1.0 - 2.0 * (y * y + z * z)
    uy = 2.0 * (x * y + w * z)
    return math.atan2(uy, ux)


def _wrap_pi(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def _fingers_clear(st) -> bool:
    if mechkit._ENV["env"] is None:
        return bool(OFFLINE_FINGERS_CLEAR)
    return bool(released(st, clearance=RELEASE_CLEAR))


def _update(objects) -> tuple[object, float, float, float]:
    st = objects["knob"]
    pos = np.asarray(getattr(st, "origin", st.pos), float)
    v = float(np.linalg.norm(st.lin_vel))
    d_start = float(np.linalg.norm(pos - np.asarray(KNOB_START, float)))

    if LATCH.seen is objects:
        return st, pos[2], bottom(st, KNOB), tilt_deg(st)

    if LATCH.prev_start_dist is not None and LATCH.prev_start_dist > 0.03 and d_start <= 0.005 and v < 0.01:
        LATCH.clear()
    LATCH.seen = objects
    LATCH.prev_start_dist = d_start

    kb = bottom(st, KNOB)
    kt = tilt_deg(st)
    r_xy = float(math.hypot(pos[0] - STUD_XY[0], pos[1] - STUD_XY[1]))

    # Sustained tip/drop latch
    if pos[2] < TIP_Z_MIN or kt > TIP_TILT_MAX:
        LATCH.tip_streak += 1
        if LATCH.tip_streak >= SUSTAIN_TIP_STEPS:
            LATCH.tip_latched = True
    else:
        LATCH.tip_streak = 0

    # Sustained straight-push jam latch at top threads
    if r_xy <= JAM_R_TOL and abs(kb - STUD_TOP) <= JAM_Z_BAND and kt <= JAM_TILT_MAX:
        LATCH.jam_streak += 1
        if LATCH.jam_streak >= SUSTAIN_JAM_STEPS:
            LATCH.jam_latched = True
    else:
        LATCH.jam_streak = 0

    # Seated & cumulative clockwise rotation tracking
    seated = (r_xy <= STUD_R_TOL) and (abs(kb - CROSSBAR_TOP) <= SEAT_Z_TOL) and (kt <= SEAT_TILT_MAX)
    theta = _knob_yaw(st)
    if seated:
        if LATCH.seated_prev and LATCH.prev_theta is not None:
            d_cw = -_wrap_pi(theta - LATCH.prev_theta) / (2.0 * math.pi)
            LATCH.cw_turns += d_cw
        LATCH.seated_prev = True
        LATCH.prev_theta = theta
    else:
        LATCH.seated_prev = False
        LATCH.prev_theta = None
        # Leave seated (lifted or tilted off threads) -> reset engagement turns
        if r_xy > STUD_R_TOL or kt > 8.0 or kb > CROSSBAR_TOP + 0.006:
            LATCH.cw_turns = 0.0

    rel = _fingers_clear(st)
    if seated and LATCH.cw_turns >= REQUIRED_CW_TURNS and rel and v < STILL_V_MAX:
        LATCH.still_steps += 1
    else:
        LATCH.still_steps = 0

    return st, pos[2], kb, kt


def score(objects) -> float:
    st, _pz, kb, kt = _update(objects)
    if LATCH.tip_latched or LATCH.jam_latched:
        return 0.0
    pos = np.asarray(getattr(st, "origin", st.pos), float)
    r_xy = float(math.hypot(pos[0] - STUD_XY[0], pos[1] - STUD_XY[1]))
    on_stud = r_xy <= STUD_R_TOL and kb >= CROSSBAR_TOP - SEAT_Z_TOL
    slid_down = on_stud and kb <= STUD_TOP - 0.030
    seated = on_stud and abs(kb - CROSSBAR_TOP) <= SEAT_Z_TOL and kt <= SEAT_TILT_MAX
    snugged = seated and LATCH.cw_turns >= REQUIRED_CW_TURNS

    if snugged and _fingers_clear(st) and LATCH.still_steps >= STILL_STEPS:
        return 1.0
    if snugged:
        return 0.75
    if seated:
        return 0.50
    if slid_down:
        return 0.25
    return 0.0


def success(objects) -> bool:
    return score(objects) == 1.0


# -- scripted demo (right arm, closed loop) ----------------------------------------------------------------------------
demo = placeholder_demo

TILT_RAD = math.radians(19.0)
GRIP_OFFSET = 0.0080  # TCP 8.0 mm above knob centre along knob +z (pad tips 4.0 mm above knob bottom)
FINGER_GRIP = -0.0060  # 12 mm commanded gap: ~20 N grip on 22 mm knob without over-pinching
REGRIP_OPEN = FINGER_OPEN * 0.42  # ~39 mm pad gap: ~8.7 mm clear per side during re-index


def _tcp_for_knob(knob_c, pitch: float) -> np.ndarray:
    return np.array(
        [
            float(knob_c[0]) - GRIP_OFFSET * math.sin(pitch),
            float(knob_c[1]),
            float(knob_c[2]) + GRIP_OFFSET * math.cos(pitch),
        ],
        dtype=float,
    )


def _servo_move(
    arm,
    p1: tuple[float, float, float] | np.ndarray,
    duration: float,
    *,
    pitch: float = 0.0,
    yaw: float = 0.0,
    finger: float | None = None,
    comp: np.ndarray | None = None,
    max_comp: float = 0.035,
):
    """Move TCP along smoothstep(p0 -> p1) while continuously compensating wrist sag from encoder FK."""
    if comp is None:
        comp = np.zeros(3, dtype=float)
    p0 = (arm.pos - comp).copy()
    p1_arr = np.asarray(p1, dtype=float)
    r0 = arm.rot.copy()
    r1 = closedloop.grip_rot(yaw=yaw, pitch=pitch)
    f0 = arm.finger
    f1 = arm.finger if finger is None else float(finger)
    steps = max(1, round(float(duration) / arm.dt))
    prev_goal = p0.copy()
    for i in range(1, steps + 1):
        err = prev_goal - arm.measured_pos
        if not arm.offline:
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


def demo_script(ctx):
    R = ctx.right
    sx, sy = STAGE_XY
    stx, sty = STUD_XY

    # 1. Approach staging pedestal, grasp Carr Lane knob across 22 mm OD, lift clear
    pick_tcp = (sx, sy, KNOB_START[2] + GRIP_OFFSET)
    yield from R.move((sx, sy, TOP + 0.120), 3.0, FINGER_OPEN)
    yield from R.move(pick_tcp, 1.6, FINGER_OPEN)
    yield from R.trim_to_measured(pick_tcp, tol=0.001, max_duration=1.2)
    yield from R.grip(FINGER_GRIP, 0.8)
    yield from R.hold(0.30)
    if not R.holding(0.014):
        yield from R.grip(FINGER_OPEN, 0.5)
        retry_tcp = (sx, sy, pick_tcp[2] - 0.002)
        yield from R.move(retry_tcp, 0.6, FINGER_OPEN)
        yield from R.trim_to_measured(retry_tcp, tol=0.001, max_duration=1.2)
        yield from R.grip(FINGER_GRIP, 0.8)
        yield from R.hold(0.30)
        if not R.holding(0.014):
            yield from R.grip(FINGER_OPEN, 0.5)
            yield from R.home(duration=3.0)
            return
    yield from R.move((sx, sy, STUD_TOP + 0.050), 2.0)

    # 2. Tilt +19 deg high above stud, trim wrist sag, then zero out any in-finger XY offset above stud tip
    high_tcp = _tcp_for_knob((stx, sty, STUD_TOP + 0.044), TILT_RAD)
    yield from R.move(high_tcp, 2.2, pitch=TILT_RAD)
    yield from R.trim_to_measured(high_tcp, tol=0.0008, max_duration=1.2)
    comp = (R.pos - high_tcp).copy()

    entry_tcp = _tcp_for_knob((stx, sty, STUD_TOP + 0.020), TILT_RAD)
    yield from _servo_move(R, entry_tcp, 1.0, pitch=TILT_RAD, comp=comp)
    yield from R.hold(0.20)

    grasp_corr = np.zeros(3, dtype=float)
    kpos = ctx.pos("knob")
    if math.hypot(kpos[0] - stx, kpos[1] - sty) < 0.015:
        grasp_corr = np.array(
            [np.clip(stx - kpos[0], -0.006, 0.006), np.clip(sty - kpos[1], -0.006, 0.006), 0.0],
            dtype=float,
        )
        yield from _servo_move(R, entry_tcp + grasp_corr, 0.6, pitch=TILT_RAD, comp=comp)
        yield from R.hold(0.20)
        kpos2 = ctx.pos("knob")
        if math.hypot(kpos2[0] - stx, kpos2[1] - sty) < 0.015:
            grasp_corr[0] = float(np.clip(grasp_corr[0] + 0.7 * (stx - kpos2[0]), -0.008, 0.008))
            grasp_corr[1] = float(np.clip(grasp_corr[1] + 0.7 * (sty - kpos2[1]), -0.008, 0.008))
            yield from _servo_move(R, entry_tcp + grasp_corr, 0.4, pitch=TILT_RAD, comp=comp)

    # 3. Maintaining +19 deg tilt and closed-loop sag tracking, slide straight down (-z) onto crossbar
    dz_touch = (KNOB_H / 2.0) * math.cos(TILT_RAD) + KNOB_R_OUT * math.sin(TILT_RAD)
    bot_tcp = _tcp_for_knob((stx, sty, CROSSBAR_TOP + dz_touch + 0.0015), TILT_RAD) + grasp_corr
    yield from _servo_move(R, bot_tcp, 2.6, pitch=TILT_RAD, comp=comp)

    # 4. Pivot smoothly to 0 deg tilt to square half-threads onto stud groove
    mid_pitch = 0.5 * TILT_RAD
    dz_mid = (KNOB_H / 2.0) * math.cos(mid_pitch) + KNOB_R_OUT * math.sin(mid_pitch) + 0.0005
    mid_tcp = _tcp_for_knob((stx, sty, CROSSBAR_TOP + dz_mid), mid_pitch) + 0.5 * grasp_corr
    yield from _servo_move(R, mid_tcp, 0.9, pitch=mid_pitch, comp=comp)
    seat_tcp = _tcp_for_knob((stx, sty, CROSSBAR_TOP + KNOB_H / 2.0 + 0.0003), 0.0)
    yield from _servo_move(R, seat_tcp, 0.9, pitch=0.0, comp=comp)
    yield from R.hold(0.25)

    # 5. Rotate clockwise >= 0.5 turns in two safe wrist passes (staying well inside joint6 +-120 deg)
    yield from _servo_move(R, seat_tcp, 1.8, pitch=0.0, yaw=-1.10, comp=comp)  # 63 deg CW
    yield from R.grip(REGRIP_OPEN, 0.5)
    yield from _servo_move(R, seat_tcp, 1.8, pitch=0.0, yaw=+1.35, finger=REGRIP_OPEN, comp=comp)
    yield from R.grip(FINGER_GRIP, 0.6)
    yield from R.hold(0.25)
    if not R.holding(0.014):
        yield from R.grip(FINGER_OPEN, 0.5)
        yield from R.move((stx, sty, STUD_TOP + 0.055), 1.5, FINGER_OPEN, yaw=+1.35)
        yield from R.home(duration=3.5)
        return
    yield from _servo_move(R, seat_tcp, 2.5, pitch=0.0, yaw=-1.15, comp=comp)  # +143 deg CW (206 deg = 0.57 turns)

    # 6. Open gripper, lift clear above stud tip, return to READY, and settle >= 1.5 s
    yield from R.grip(FINGER_OPEN, 0.7)
    yield from R.move((stx, sty, STUD_TOP + 0.055), 1.8, FINGER_OPEN, yaw=-1.15)
    yield from R.home(duration=4.0)
    yield from R.hold(2.5)
