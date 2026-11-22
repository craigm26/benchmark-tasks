"""On the station table, at the far end in front of the arms, stands a wall console: a dark-grey base (80 x 160 x 15 mm)
with a light-grey upright panel (16 x 150 x 200 mm). Screwed to the panel's front face, centred at y 0, is a black nylon
Peter Jones Klick Fast Dock 05 (an anti-drop mount: a 46 mm square plate with a U channel open at the top, two front
lips that leave a 20 mm slot, and an amber latching lug inside). A handheld two-way radio (Retevis H-777 style: dark body,
yellow top cap and side button, two silver knobs, a black stubby antenna) hangs in it upright by a Klick Fast Stick-On
stud on its back, and is locked: pulled forward the lips stop the stud's collar; lifted upright the lug stops the
C-shaped collar. Turned 180 degrees about the stud (antenna down) the collar's gate lines up with the lug and the
radio slides straight up and out. A green 90 x 70 x 6 mm pad lies on the table in front of the right arm.

Requires -E object_jitter=0 (the dock and the pad are fixed; the radio starts locked in the dock).

Run it:  inspect-robots run --embodiment yam_arena -E task=undock_klick_fast_shift_radio_invert_180.py -E object_jitter=0 \\
           --instruction "Invert the shift radio 180 deg in the wall dock, slide it straight up free, and stand it
           upright on the green pad." --scorer task_score ...
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402

import mechkit  # noqa: E402
from belimits import Rect  # noqa: E402,F401
from closedloop import grip_rot, placeholder_demo  # noqa: E402
from mechkit import Joint, Mech, box, cyl, lathe, on_reset, released, tube  # noqa: E402,F401
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402
from yam_arena.urdf import axis_angle_matrix  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 8.0  # the demo grips the radio: keep the episode (and the video) running until both hands are parked

GREY_D, GREY_L = (0.22, 0.22, 0.24), (0.78, 0.78, 0.80)
NYLON, AMBER = (0.05, 0.05, 0.06), (1.0, 0.55, 0.05)
GRAPHITE, YELLOW, SILVER, WHITE = (0.16, 0.16, 0.18), (1.0, 0.85, 0.05), (0.80, 0.80, 0.84), (0.95, 0.95, 0.95)
GREEN = (0.10, 0.62, 0.22)

# ---- layout (m, station frame) -------------------------------------------------------------------------------------
XC = 0.328  # console centre x (panel 16 mm thick: front face x 0.320)
ZS = TOP + 0.145  # the dock's axis height (the cradle's centre)
PAD_XY = (0.220, -0.135)
PAD_TOP = TOP + 0.006

# Axial layout of the stud and the dock (x, m). Everything here is a tape-measure number.
X_LIP = (0.3055, 0.3085)  # front lips, 3 mm, leave a 20 mm slot
X_CHAN = (0.3085, 0.3160)  # U channel (walls and cradle), 7.5 mm deep
X_BACK = (0.3160, 0.3200)  # backplate, 4 mm; the panel's front face is x 0.320
X_LUG = (0.3120, 0.3160)  # latching lug, 4 mm, 6 x 6 mm
X_PLATE = (0.3025, 0.3045)  # stud base plate (2 mm), 1 mm in front of the lips
X_NECK = (0.3045, 0.3095)  # neck, r 7 mm, through the 20 mm slot
X_DISC = (0.3095, 0.3105)  # flange disc r 14.5 mm, 1 mm behind the lips
X_RING = (0.3105, 0.3140)  # C collar, r_in 8.5, r_out 15 mm, 3.5 mm
X0 = 0.31225  # the radio Mech's origin: the collar's centre plane (on the stud axis)
Z_REST = ZS - 0.0028  # the collar rests in the cradle 2.8 mm below the dock's axis (3 mm radial play)

BODY = (0.034, 0.056, 0.102)  # radio x (thickness), y (width), z (height)
BODY_X = (0.2685, 0.3025)  # body x range; centre 0.2855
GRIP_X = 0.5 * sum(BODY_X)
GRIP_D = 0.030  # the hands hold the 56 mm walls 30 mm up (right) and 30 mm down (left) from the stud axis


def _lx(x: float) -> float:
    return x - X0


# ---- the console and Dock 05 (fixed) ------------------------------------------------------------------------------
def _cx(x: float) -> float:
    return x - XC


ZL = ZS - TOP
_c = lambda a, b: 0.5 * (a + b)  # noqa: E731
CONSOLE = Mech(
    "console",
    pos=(XC, 0.0, TOP),
    mass=None,
    friction=0.35,
    shapes=[
        box((0, 0, 0.0075), (0.080, 0.160, 0.015), GREY_D),
        box((0, 0, 0.115), (0.016, 0.150, 0.200), GREY_L),
        # Klick Fast Dock 05: backplate, two side walls above the axis, a half-round cradle below it, two front lips, lug
        box((_cx(_c(*X_BACK)), 0, ZL), (X_BACK[1] - X_BACK[0], 0.046, 0.046), NYLON),
        box((_cx(_c(*X_CHAN)), 0.0205, ZL + 0.0115), (X_CHAN[1] - X_CHAN[0], 0.005, 0.023), NYLON),
        box((_cx(_c(*X_CHAN)), -0.0205, ZL + 0.0115), (X_CHAN[1] - X_CHAN[0], 0.005, 0.023), NYLON),
        tube((_cx(_c(*X_CHAN)), 0, ZL), 0.018, 0.023, X_CHAN[1] - X_CHAN[0], NYLON, axis="x", a0=math.pi,
             a1=2 * math.pi),
        box((_cx(_c(*X_LIP)), 0.0165, ZL), (X_LIP[1] - X_LIP[0], 0.013, 0.046), NYLON),
        box((_cx(_c(*X_LIP)), -0.0165, ZL), (X_LIP[1] - X_LIP[0], 0.013, 0.046), NYLON),
        box((_cx(_c(*X_LUG)), 0, ZL), (X_LUG[1] - X_LUG[0], 0.006, 0.006), AMBER),
    ],
)

# ---- the radio and its Stick-On stud (free) -------------------------------------------------------------------------
_GATE_HALF = math.radians(45.0)  # the collar's gate: 90 degrees open at the top (a 12 mm slot at the inner radius)
RADIO_SHAPES = [
    box((_lx(GRIP_X), 0, -0.013), (BODY[0], BODY[1], 0.090), GRAPHITE),
    box((_lx(GRIP_X), 0, 0.038), (BODY[0], BODY[1], 0.012), YELLOW),  # upper 12 mm cap
    box((_lx(BODY_X[0]) - 0.0005, 0, 0.012), (0.002, 0.040, 0.012), WHITE),  # front band: upright from inverted
    box((_lx(GRIP_X), 0.0295, -0.004), (0.014, 0.003, 0.016), YELLOW),  # side PTT button
    cyl((_lx(GRIP_X), 0.008, 0.050), 0.0055, 0.012, SILVER),
    cyl((_lx(GRIP_X), 0.020, 0.050), 0.0055, 0.012, SILVER),
    lathe((_lx(GRIP_X), -0.017, 0.044), [(0.0075, 0.0), (0.0070, 0.004), (0.0050, 0.030), (0.0035, 0.052)], NYLON),
    box((_lx(_c(*X_PLATE)), 0, 0), (X_PLATE[1] - X_PLATE[0], 0.0328, 0.0398), NYLON),
    cyl((_lx(_c(*X_NECK)), 0, 0), 0.007, X_NECK[1] - X_NECK[0], NYLON, axis="x"),
    cyl((_lx(_c(*X_DISC)), 0, 0), 0.0145, X_DISC[1] - X_DISC[0], NYLON, axis="x"),
    tube((_lx(_c(*X_RING)), 0, 0), 0.0085, 0.015, X_RING[1] - X_RING[0], NYLON, axis="x",
         a0=math.pi / 2 + _GATE_HALF, a1=math.pi / 2 - _GATE_HALF + 2 * math.pi),
]
RADIO = Mech("radio", pos=(X0, 0.0, Z_REST), mass=0.132, friction=0.45, shapes=RADIO_SHAPES)
BODY_CENTRE_LOCAL = np.array([_lx(GRIP_X), 0.0, -0.007])  # the body's centre in the radio frame

PAD = Box("handover_pad", pos=(PAD_XY[0], PAD_XY[1], TOP + 0.003), size=(0.090, 0.070, 0.006), color=GREEN)
OBJECTS = [CONSOLE, RADIO, PAD]

# ---- scoring --------------------------------------------------------------------------------------------------------
INV_TOL = 20.0  # deg of the roll about the stud axis counted as inverted
DOCK_YZ = 0.012  # m: the stud within this of the dock's axis counts as seated in it
FREED_LIFT = 0.026  # stud this far above the dock's axis (or pulled clear forward) counts as out
FREED_X = 0.285
ESCAPE_X = 0.304  # the collar in front of the lips' front face: forced out forward
TILT_MAX = 8.0
PAD_XY_TOL = 0.020
PAD_Z_TOL = 0.003
SPEED, SPIN = 0.005, math.radians(3.0)
LATCH_STEPS = 10  # 0.5 s at 20 Hz
STILL_STEPS = 40  # 2.0 s at 20 Hz
DROP_Z = TOP - 0.02

_STATE = {"inv": 0, "was_inv": False, "drop": 0, "escape": 0, "dead": False, "run_pad": 0, "run_free": 0, "last_key": None}


def _clear() -> None:
    _STATE.update(inv=0, was_inv=False, drop=0, escape=0, dead=False, run_pad=0, run_free=0, last_key=None)


@on_reset
def _on_reset(env) -> None:
    _clear()


def roll_deg(s) -> float:
    """Roll of the radio about the world x axis (the stud axis), 0 upright, 180 inverted."""
    r = s.rotation()
    return math.degrees(math.atan2(r[2, 1], r[1, 1]))


def _dev180(deg: float) -> float:
    return abs((deg - 180.0 + 180.0) % 360.0 - 180.0)


def facts(objects) -> dict:
    s = objects["radio"]
    r = s.rotation()
    roll = roll_deg(s)
    o = np.asarray(s.origin, float)
    pts = RADIO.points() @ r.T + o
    centre = s.to_world(BODY_CENTRE_LOCAL)
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(r[2, 2])))))
    bottom = float(pts[:, 2].min())
    inv = _dev180(roll) <= INV_TOL
    seated = bool(abs(o[1]) <= DOCK_YZ and abs(o[2] - ZS) <= DOCK_YZ and o[0] >= 0.309)
    freed = bool(o[2] > ZS + FREED_LIFT or o[0] < FREED_X)
    on_pad = bool(
        tilt <= TILT_MAX and abs(bottom - PAD_TOP) <= PAD_Z_TOL
        and math.hypot(centre[0] - PAD_XY[0], centre[1] - PAD_XY[1]) <= PAD_XY_TOL
    )
    return dict(roll=roll, inv=inv, seated=seated, freed=freed, on_pad=on_pad, origin=o, tilt=tilt, bottom=bottom,
                centre=centre, low_z=float(s.low[2]),
                escaped=bool(ESCAPE_X - 0.02 < o[0] < ESCAPE_X and abs(o[1]) < 0.03 and abs(o[2] - ZS) < 0.024
                             and _dev180(roll) > 30.0))


def _observe(objects, f) -> None:
    """One update per control step (score and success run on the same dict each step)."""
    step = mechkit.step_count()
    key = ("s", step) if step > 0 else ("o", id(objects))
    if _STATE["last_key"] == key:
        return
    _STATE["last_key"] = key
    _STATE["drop"] = _STATE["drop"] + 1 if f["low_z"] < DROP_Z else 0
    _STATE["escape"] = _STATE["escape"] + 1 if f["escaped"] else 0
    if _STATE["drop"] >= LATCH_STEPS or _STATE["escape"] >= LATCH_STEPS:
        _STATE["dead"] = True
    _STATE["inv"] = _STATE["inv"] + 1 if (f["inv"] and f["seated"]) else 0
    if _STATE["inv"] >= LATCH_STEPS:
        _STATE["was_inv"] = True
    s = objects["radio"]
    rel = bool(released(s, clearance=0.03))
    calm = bool(s.resting(SPEED, SPIN))
    free = bool(_STATE["was_inv"] and f["freed"] and rel and calm)
    _STATE["run_free"] = _STATE["run_free"] + 1 if free else 0
    _STATE["run_pad"] = _STATE["run_pad"] + 1 if (free and f["on_pad"]) else 0


def score(objects):
    """Whole-run zeros: the radio fell off the table, or was forced out forward past the lips. Otherwise 0.25 once it
    has been inverted 180 deg (within 20 deg, 0.5 s) with its stud still seated in the dock; 0.50 freed (stud lifted
    out of the dock or pulled clear) and released and still for the last 2 s, anywhere; 0.75 freed and standing
    upright on the pad (tilt <= 8 deg, base within 3 mm of the pad top, centre within 20 mm); 1.00 that, with both
    grippers 3 cm clear and the radio still for the last 2 s. A radio held in the air is never on the pad (its base
    must be on the pad top) and never released."""
    f = facts(objects)
    _observe(objects, f)
    if _STATE["dead"] or not _STATE["was_inv"]:
        return 0.0
    best = 0.25
    if f["freed"]:
        if _STATE["run_free"] >= STILL_STEPS:
            best = 0.5
        if f["on_pad"]:
            best = 0.75
            if _STATE["run_pad"] >= STILL_STEPS:
                best = 1.0
    return best


def success(objects):
    return score(objects) == 1.0


# ---- the robot keeps out of the console's back wall (declared limit) -----------------------------------------------
ARM_KEEP_OUT = [
    {"name": "console back wall and dock plate", "x": (0.316, 0.375), "y": (-0.085, 0.085), "z_top": TOP + 0.2225},
]

# ---- the scripted demo ---------------------------------------------------------------------------------------------
demo = placeholder_demo
RX = lambda a: axis_angle_matrix(np.array([1.0, 0.0, 0.0]), float(a))  # noqa: E731
PITCH = math.radians(70.0)  # both hands point 20 deg below level: the pads close along y, so this only helps reach
HAND = grip_rot(pitch=PITCH)
A_AXIS = np.array([GRIP_X, 0.0, Z_REST])  # the stud axis point the hands turn the radio about
LIFT = 0.045
BACK = 0.055
FINGER_GRIP = -0.0235  # commanded pad gap 47 mm on the 56 mm radio: about 5 mm per finger, 25 N, centres the radio
APPROACH = 0.05  # open hands come in this far in front of the grip along +x
V = 0.05  # m/s typical tool speed (smoothstep peaks at 1.5 x), under the declared 0.15 m/s


def _rot(th, roll_off=0.0):
    """Hand orientation when the radio is turned th about the stud axis (roll_off = pi flips the pad labels)."""
    return RX(th + roll_off) @ HAND


def _grip_pt(centre, th, sign):
    """Where the pads hold the radio at roll th about the axis: 30 mm up (sign +1) or down (-1) the radio's height."""
    return np.asarray(centre, float) + RX(th) @ np.array([0.0, 0.0, sign * GRIP_D])


_BIAS = {}  # per arm: the offset added to the command so the encoder-measured tool point sits on the target


def _follow(arm, pfn, rfn, dur):
    """Move a hand that holds the radio along pfn(s), rfn(s) for s 0..1 over dur seconds, nudging the command by a
    clamped integral of (target - measured tool point) so arm sag never drags the stud against the dock."""
    from yam_arena import ik

    bias = _BIAS.setdefault(arm.side, np.zeros(3))
    steps = max(1, round(dur / arm.dt))
    for i in range(1, steps + 1):
        s = ik.smoothstep(i / steps)
        target = np.asarray(pfn(s), float)
        bias = bias + 0.3 * (target - np.asarray(arm.measured_pos, float))
        n = float(np.linalg.norm(bias))
        if n > 0.035:
            bias = bias * (0.035 / n)
        arm._set_cartesian(target + bias, rfn(s), arm.finger)
        _BIAS[arm.side] = bias
        yield


def _arc(arm, centre, th0, th1, sign, roll_off, dur):
    return _follow(arm, lambda s: _grip_pt(centre, th0 + s * (th1 - th0), sign),
                   lambda s: _rot(th0 + s * (th1 - th0), roll_off), dur)


def _line(arm, p0, p1, th, roll_off, dur):
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    return _follow(arm, lambda s: p0 + s * (p1 - p0), lambda s: _rot(th, roll_off), dur)


def _moveto(arm, pos, th, roll_off, dur, finger=None):
    return arm.move(pos, dur, finger, rot=_rot(th, roll_off))


def _dur(a, b, v=V, floor=0.8):
    return max(floor, 1.6 * float(np.linalg.norm(np.asarray(a, float) - np.asarray(b, float))) / v)


def _take(arm, centre, th, sign, roll_off):
    """Open hand in along +x to the grip, close on the 56 mm walls; True if the pads stopped on the radio. A miss opens,
    backs out and tries once more."""
    g = _grip_pt(centre, th, sign)
    pre = g + np.array([-APPROACH, 0.0, 0.0])
    for _ in range(2):
        _BIAS[arm.side] = np.zeros(3)
        yield from _moveto(arm, pre, th, roll_off, _dur(arm.pos, pre, floor=1.5), FINGER_OPEN)
        yield from _moveto(arm, g, th, roll_off, _dur(pre, g), FINGER_OPEN)
        yield from arm.trim_to_measured(g)
        yield from arm.grip(FINGER_GRIP, 0.8)
        yield from arm.hold(0.4)
        if arm.holding(0.049):
            _BIAS[arm.side] = np.asarray(arm.pos, float) - g
            return True
        yield from arm.grip(FINGER_OPEN, 0.6)
    return False


def _let_go(arm, centre, th, sign, roll_off, back=0.09, lift=0.0, side=0.0, open_s=0.8):
    """Open the hand (slowly if open_s is long, so a set-down part does not lunge), back out along -x, rise and step
    sideways clear of the part, then swing home in joint space."""
    yield from arm.grip(FINGER_OPEN, open_s)
    g = _grip_pt(centre, th, sign)
    yield from _moveto(arm, g + np.array([-back, 0.0, 0.0]), th, roll_off, 2.0, FINGER_OPEN)
    if lift > 0.0 or side != 0.0:
        yield from _moveto(arm, g + np.array([-back, side, lift]), th, roll_off, 2.0, FINGER_OPEN)
    yield from arm.home(3.0)


def demo_script(ctx):
    R, L = ctx.right, ctx.left
    # 1. right hand: hold the upright radio 30 mm above its stud axis by the 56 mm side walls, turn it 90 deg
    via = _grip_pt(A_AXIS, 0.0, +1) + np.array([-APPROACH - 0.03, -0.06, 0.03])
    yield from R.move(via, 3.0, FINGER_OPEN, rot=_rot(0.0))
    if not (yield from _take(R, A_AXIS, 0.0, +1, 0.0)):
        return
    yield from _arc(R, A_AXIS, 0.0, math.pi / 2, +1, 0.0, 4.0)
    # 2. left hand takes the lower end across the walls (now top and bottom); right hand lets go and clears
    viaL = _grip_pt(A_AXIS, math.pi / 2, -1) + np.array([-APPROACH - 0.03, 0.06, 0.03])
    yield from L.move(viaL, 3.0, FINGER_OPEN, rot=_rot(math.pi / 2, -math.pi))
    if not (yield from _take(L, A_AXIS, math.pi / 2, -1, -math.pi)):
        return
    yield from _let_go(R, A_AXIS, math.pi / 2, +1, 0.0, back=0.04, side=-0.08, open_s=1.5)
    # 3. left hand turns it the other 90 deg: the collar's gate meets the lug; then straight up, then back out
    yield from _arc(L, A_AXIS, math.pi / 2, math.pi, -1, -math.pi, 4.0)
    yield from L.hold(0.5)
    top = A_AXIS + np.array([0.0, 0.0, LIFT])
    yield from _line(L, _grip_pt(A_AXIS, math.pi, -1), _grip_pt(top, math.pi, -1), math.pi, -math.pi, 4.0)
    out = top + np.array([-BACK, 0.0, 0.0])
    yield from _line(L, _grip_pt(top, math.pi, -1), _grip_pt(out, math.pi, -1), math.pi, -math.pi, 3.0)
    # 4. turn it back to horizontal, the right hand takes the other end, left lets go, right turns it upright
    yield from _arc(L, out, math.pi, math.pi / 2, -1, -math.pi, 4.0)
    viaR = _grip_pt(out, math.pi / 2, +1) + np.array([-APPROACH - 0.03, -0.06, 0.0])
    yield from R.move(viaR, 3.0, FINGER_OPEN, rot=_rot(math.pi / 2))
    if not (yield from _take(R, out, math.pi / 2, +1, 0.0)):
        return
    yield from _let_go(L, out, math.pi / 2, -1, -math.pi, back=0.04, side=0.08, open_s=1.5)
    yield from _arc(R, out, math.pi / 2, 0.0, +1, 0.0, 4.0)
    # 5. carry it upright to the pad, read where the radio sits in the hand, square it over the pad, set it down
    axis_z = PAD_TOP + 0.058 + 0.0015  # stud axis height with the base 1.5 mm over the pad
    here = _grip_pt(out, 0.0, +1)
    carry = np.array([PAD_XY[0], PAD_XY[1], here[2]])
    yield from _line(R, here, carry, 0.0, 0.0, _dur(here, carry, v=0.06, floor=3.0))
    yield from R.hold(1.0)
    if not R.offline:  # a live run: the radio sits a few mm off the pad's mark in the hand, so move the hand by the miss
        s = ctx.obj("radio")
        c = s.to_world(BODY_CENTRE_LOCAL)
        carry = carry + np.array([PAD_XY[0] - c[0], PAD_XY[1] - c[1], 0.0])
        yield from _line(R, R.measured_pos, carry, 0.0, 0.0, 2.0)
        yield from R.hold(0.5)
    lower = np.array([carry[0], carry[1], axis_z + GRIP_D + 0.010])
    yield from _line(R, carry, lower, 0.0, 0.0, _dur(carry, lower, v=0.06, floor=2.5))
    # lower at 1 cm/s until the radio's base meets the pad (live: from its pose; offline: the nominal depth)
    depth, step = 0.0, 0.01 * R.dt
    from yam_arena import ik
    bias = _BIAS.setdefault(R.side, np.zeros(3))
    while depth < 0.024:
        if not R.offline:
            st = ctx.obj("radio")
            if float((RADIO.points() @ st.rotation().T + st.origin)[:, 2].min()) <= PAD_TOP + 0.0015:
                break
        elif depth >= 0.0115:
            break
        depth += step
        R._set_cartesian(lower - np.array([0.0, 0.0, depth]) + bias, _rot(0.0), R.finger)
        yield
    yield from R.hold(0.5)
    yield from _let_go(R, np.array([lower[0], lower[1], axis_z]), 0.0, +1, 0.0, back=0.07, lift=0.12, side=-0.05, open_s=2.0)
    yield from L.hold(2.5)
