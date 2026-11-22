"""On the station table: a dark green 13 x 13 x 11 in square milk crate centred in front of the arms, holding two layers
of half-pint gable-top milk cartons (white lower half, blue upper half, blue roof and fin, ridges front to back), 5 x 5 per layer. The top
layer's 25 cartons are loose, with the crate's 5 mm of row slack shared out as about 0.8 mm at every seam and wall. To the right of the crate stands an orange probe holster with a between-pack thermometer probe
(black handle, flat stainless blade) standing blade-down in it.

Requires -E object_jitter=0 -E control_hz=20: the scorer measures carton lift and tilt against the fixed layout and times
the reading hold in 20 Hz control steps.

Run it:  inspect-robots run --embodiment yam_arena -E task=milk_crate_between_pack_probe.py -E object_jitter=0 \\
           -E control_hz=20 --instruction "Take this milk delivery's temperature: slide the probe 3 to 7 cm down between two middle cartons, lifting or tipping none, hold it for the reading, then put it back." \\
           --scorer task_score ...
"""

import dataclasses
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402

import mechkit  # noqa: E402
from mechkit import Mech, box, bore, cyl, hull, on_reset  # noqa: E402
from belimits import tilt_deg  # noqa: E402
from closedloop import placeholder_demo  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 8.0
CONTROL_DT = 0.05  # s per control step (-E control_hz=20)

GREEN, DARK_GREEN = (0.10, 0.42, 0.18), (0.05, 0.22, 0.09)
WHITE, BLUE = (0.95, 0.95, 0.93), (0.12, 0.32, 0.80)
BLACK, STEEL, RED = (0.06, 0.06, 0.06), (0.78, 0.80, 0.82), (0.85, 0.10, 0.10)
ORANGE = (0.95, 0.45, 0.05)

# ---- The crate: 16 qt square milk crate, 13 x 13 x 11 in outside, 12 x 12 x 10.75 in inside (HDPE). -----------------
CRATE_X, CRATE_Y = (0.095, 0.425), (-0.165, 0.165)
CRATE_H, WALL, FLOOR = 0.279, 0.0125, 0.008
IN_X = (CRATE_X[0] + WALL, CRATE_X[1] - WALL)  # 0.1075 .. 0.4125 (305 mm)
IN_Y = (CRATE_Y[0] + WALL, CRATE_Y[1] - WALL)  # -0.1525 .. 0.1525
CX, CY = (CRATE_X[0] + CRATE_X[1]) / 2, (CRATE_Y[0] + CRATE_Y[1]) / 2
LOWER_TOP = 0.129  # lower layer of 25 cartons (fixed slab): floor 8 mm + one 121 mm carton
_iw = IN_X[1] - IN_X[0]


def _look(shape):
    """A look-only copy of a shape (no collider)."""
    return dataclasses.replace(shape, collide=False)


CRATE = Mech(
    "milk_crate", pos=(CX, CY, TOP), mass=None, friction=0.40,
    shapes=[
        box((0, 0, FLOOR / 2), (0.330, 0.330, FLOOR), GREEN),
        box((0, 0, (FLOOR + LOWER_TOP) / 2), (_iw, _iw, LOWER_TOP - FLOOR), WHITE),  # lower carton layer, fixed
        box((-(0.165 - WALL / 2), 0, CRATE_H / 2), (WALL, 0.330, CRATE_H), GREEN),
        box((+(0.165 - WALL / 2), 0, CRATE_H / 2), (WALL, 0.330, CRATE_H), GREEN),
        box((0, -(0.165 - WALL / 2), CRATE_H / 2), (0.330 - 2 * WALL, WALL, CRATE_H), GREEN),
        box((0, +(0.165 - WALL / 2), CRATE_H / 2), (0.330 - 2 * WALL, WALL, CRATE_H), GREEN),
    ] + [  # hand slots near the rim on all four sides (look only)
        _look(box((sx * (0.165 + 0.0005), 0, 0.235), (0.002, 0.11, 0.035), DARK_GREEN)) for sx in (-1, 1)
    ] + [
        _look(box((0, sy * (0.165 + 0.0005), 0.235), (0.11, 0.002, 0.035), DARK_GREEN)) for sy in (-1, 1)
    ],
)

# ---- Top layer: 25 loose half-pint gable-top cartons, 60 x 60 mm base, 121 mm to the fin top, 0.25 kg each. ----------
CARTON_W, BODY_H, ROOF_H, FIN_H = 0.060, 0.095, 0.020, 0.006
CHAMFER = 0.002  # rounded shoulder crease
CARTON_MASS, CARTON_MU = 0.25, 0.35
# The crate's 5 mm of row slack (305 mm inside, 5 x 60 mm of cartons) as it settles in a delivered crate: about 0.8 mm
# at each of the 4 seams and 2 walls of a row, front to back and side to side. Every seam is narrower than the 2.0 mm blade.
GAP = (IN_X[1] - IN_X[0] - 5 * CARTON_W) / 6
SEAM = GAP
PITCH = CARTON_W + GAP
ROWS_X = [IN_X[0] + GAP + CARTON_W / 2 + k * PITCH for k in range(5)]  # near (-x) to far
COLS_Y = [IN_Y[0] + GAP + CARTON_W / 2 + k * PITCH for k in range(5)]  # right (-y) to left
BASE_Z = TOP + LOWER_TOP
SHOULDER_Z = BASE_Z + BODY_H  # TOP + 0.224
FIN_TOP_Z = BASE_Z + BODY_H + ROOF_H + FIN_H  # TOP + 0.250


def _carton_shapes():
    """Ridge and fin along x, so the roofs of two side-by-side cartons make a V over the seam between them."""
    h, b = CARTON_W / 2, BODY_H - CHAMFER
    roof = [(sx * h, sy * h, b) for sx in (-1, 1) for sy in (-1, 1)]
    roof += [(sx * h, sy * (h - CHAMFER), BODY_H) for sx in (-1, 1) for sy in (-1, 1)]
    roof += [(sx * h, sy * 0.004, BODY_H + ROOF_H) for sx in (-1, 1) for sy in (-1, 1)]
    return [
        box((0, 0, BODY_H / 4), (CARTON_W, CARTON_W, BODY_H / 2), WHITE),
        box((0, 0, (BODY_H / 2 + b) / 2), (CARTON_W, CARTON_W, b - BODY_H / 2), BLUE),
        hull((0, 0, 0), roof, BLUE),
        box((0, 0, BODY_H + ROOF_H + FIN_H / 2), (CARTON_W, 0.008, FIN_H), WHITE),
    ]


def carton_name(i, j):
    return f"carton_{i}{j}"  # i: row from the near side (x), j: column from the right (y)


CARTONS = {
    (i, j): Mech(carton_name(i, j), pos=(ROWS_X[i], COLS_Y[j], BASE_Z), mass=CARTON_MASS, friction=CARTON_MU,
                 shapes=_carton_shapes())
    for i in range(5) for j in range(5)
}
INNER = [(i, j) for i in (1, 2, 3) for j in (1, 2, 3)]
INNER_PAIRS = [((i, j), (i + 1, j)) for i in (1, 2) for j in (1, 2, 3)] + \
              [((i, j), (i, j + 1)) for i in (1, 2, 3) for j in (1, 2)]

# ---- The probe: ThermoWorks between-pack probe, 5 x 0.15 x 0.08 in flattened stainless blade, rounded tip. -----------
BLADE_L, BLADE_W, BLADE_T = 0.127, 0.0038, 0.0020
TIP_TAPER, TIP_T = 0.010, 0.0006  # the flattened end thins to 0.6 mm over its last 10 mm
HANDLE_R, HANDLE_L = 0.0095, 0.100
PROBE_MASS, PROBE_MU = 0.05, 0.25  # stainless on PE-coated paperboard
GRIP_UP = 0.040  # the hand holds the handle 40 mm above its bottom
TIP_BELOW_GRIP = BLADE_L + GRIP_UP  # 0.167


def _probe_shapes():
    z0 = -BLADE_L + TIP_TAPER
    """Blade flat faces normal to y (thin along y), to go down a seam between two side-by-side cartons."""
    tip = [(sx * BLADE_W / 2, sy * BLADE_T / 2, z0) for sx in (-1, 1) for sy in (-1, 1)]
    r = BLADE_W / 2
    zc = -BLADE_L + r
    for a in np.linspace(0.0, math.pi, 9):
        for sy in (-1, 1):
            tip.append((r * math.cos(a), sy * TIP_T / 2, zc - r * math.sin(a)))
    band = lambda z: _look(box((0, 0, z), (BLADE_W + 0.0006, BLADE_T + 0.0006, 0.003), RED))
    return [
        cyl((0, 0, HANDLE_L / 2), HANDLE_R, HANDLE_L, BLACK, sides=12),
        box((0, 0, z0 / 2), (BLADE_W, BLADE_T, -z0), STEEL),
        hull((0, 0, 0), tip, STEEL),
        band(-BLADE_L + 0.030),
        band(-BLADE_L + 0.070),
    ]


HOLSTER_XY, HOLSTER_H, HOLSTER_W = (0.270, -0.235), 0.135, 0.040
HOLE_R, HOLE_FLOOR = 0.006, 0.005
HOLSTER = Mech(
    "probe_holster", pos=(*HOLSTER_XY, TOP), mass=None,
    shapes=[bore((0, 0, HOLSTER_H / 2), (HOLSTER_W, HOLSTER_W, HOLSTER_H), HOLE_R, ORANGE, lead_in=0.002,
                 floor=HOLE_FLOOR)],
)
PROBE_HOME = (*HOLSTER_XY, TOP + HOLSTER_H)  # probe origin: the handle's bottom centre, on the blade axis
PROBE = Mech("probe", pos=PROBE_HOME, mass=PROBE_MASS, friction=PROBE_MU, shapes=_probe_shapes())

OBJECTS = [CRATE, HOLSTER, PROBE, *CARTONS.values()]

# ---- Scoring -----------------------------------------------------------------------------------------------------------
LIFT_LIMIT, TILT_LIMIT, LATCH_STEPS = 0.005, 10.0, 3
CORNER_IN = 0.010  # tip at least 10 mm in from the corners of the two facing walls
DEPTH_MIN, BAND = 0.015, (0.030, 0.070)
PROBE_TILT = 15.0
READ_S, READ_STILL = 15.0, 0.003
FINAL_STILL_S, FINAL_STILL = 2.0, 0.002
CARTON_SHIFT = 0.010


def _origin(s):
    o = getattr(s, "origin", None)
    return np.asarray(s.pos if o is None else o, float)


def _rot(s):
    x, y, z, w = (float(v) for v in s.quat_xyzw)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def tip_point(s):
    """World position of the probe's blade tip."""
    return _origin(s) + _rot(s) @ np.array([0.0, 0.0, -BLADE_L])


def in_seam(objects, tip):
    """The tip between the facing walls of two cartons of the inner 3 x 3, at least 10 mm in from their corners."""
    for a, b in INNER_PAIRS:
        ca, cb = _origin(objects[carton_name(*a)])[:2], _origin(objects[carton_name(*b)])[:2]
        d = cb - ca
        dist = float(np.hypot(*d))
        if dist < 1e-6:
            continue
        u = d / dist
        v = np.array([-u[1], u[0]])
        rel = tip[:2] - (ca + cb) / 2
        if abs(rel @ u) <= max(dist / 2 - CARTON_W / 2, 0.0) + 0.0015 and abs(rel @ v) <= CARTON_W / 2 - CORNER_IN:
            return True
    return False


def _carton_bad(s, spec):
    """Lifted 5 mm or more, tilted over 10 deg, or its centre outside the crate."""
    o = _origin(s)
    out = not (IN_X[0] < o[0] < IN_X[1] and IN_Y[0] < o[1] < IN_Y[1])
    return o[2] - spec.pos[2] >= LIFT_LIMIT or tilt_deg(s) > TILT_LIMIT or out


_S = {}


@on_reset
def _clear(env=None):
    _S.clear()
    _S.update(last=None, bad=0, failed=False, best=0.0, read_start=None, read_steps=0, read_done=False,
              hist=[])


_clear()


def _probe_in_holster(s):
    tip = tip_point(s)
    return bool(
        np.hypot(tip[0] - HOLSTER_XY[0], tip[1] - HOLSTER_XY[1]) < HOLE_R
        and tip[2] < TOP + HOLSTER_H - 0.05
        and abs(_origin(s)[2] - PROBE_HOME[2]) < 0.005
        and tilt_deg(s) <= 10.0
    )


def _update(objects):
    """Advance the whole-run latches once per control step (score() and success() share one dict per step)."""
    if objects is _S["last"]:
        return
    _S["last"] = objects
    bad = any(_carton_bad(objects[carton_name(*k)], c) for k, c in CARTONS.items())
    _S["bad"] = _S["bad"] + 1 if bad else 0
    if _S["bad"] >= LATCH_STEPS:
        _S["failed"] = True
    p = objects["probe"]
    tip = tip_point(p)
    seam = in_seam(objects, tip)
    depth = SHOULDER_Z - tip[2]
    tier = 0.0
    in_band = seam and BAND[0] <= depth <= BAND[1] and tilt_deg(p) <= PROBE_TILT
    if seam and depth >= DEPTH_MIN:
        tier = 0.25
    if in_band:
        tier = 0.5
        if _S["read_start"] is None or np.linalg.norm(tip - _S["read_start"]) >= READ_STILL:
            _S["read_start"], _S["read_steps"] = tip.copy(), 0
        _S["read_steps"] += 1
        if _S["read_steps"] * CONTROL_DT >= READ_S:
            _S["read_done"] = True
    else:
        _S["read_start"], _S["read_steps"] = None, 0
    if _S["read_done"]:
        tier = 0.75
    _S["best"] = max(_S["best"], tier)
    _S["hist"].append(_origin(p).copy())
    del _S["hist"][: -int(FINAL_STILL_S / CONTROL_DT)]


def _returned(objects):
    p = objects["probe"]
    hist = _S["hist"]
    still = len(hist) >= int(FINAL_STILL_S / CONTROL_DT) and max(np.linalg.norm(h - hist[-1]) for h in hist) < FINAL_STILL
    cartons_home = all(np.hypot(*(_origin(objects[carton_name(*k)])[:2] - np.asarray(c.pos[:2]))) <= CARTON_SHIFT
                       for k, c in CARTONS.items())
    return bool(_probe_in_holster(p) and still and cartons_home and mechkit.released(p, clearance=0.03))


def score(objects):
    """0 once any carton's base has been 5 mm or more above its rest height, or tilted over 10 deg, for 3 steps in a row.
    Otherwise the best of: 0.25 tip in a seam between two inner cartons, 15 mm or more below the shoulders; 0.5 tip in
    such a seam 30 to 70 mm deep, probe within 15 deg of vertical; 0.75 that held still (tip within 3 mm) for 15 s;
    1.0 after the reading, the probe back in its holster, released, still for 2 s, every carton within 10 mm of start."""
    _update(objects)
    if _S["failed"]:
        return 0.0
    if _S["read_done"] and _returned(objects):
        return 1.0
    return float(_S["best"])


def success(objects):
    """Full score, and both hands at least 10 cm from the probe (offline, with no simulator, the hand check passes)."""
    if score(objects) != 1.0:
        return False
    return mechkit._ENV.get("env") is None or mechkit.released(objects["probe"], clearance=0.10)


# The hands stay above the crate: no part of either arm below 1 cm over the rim, anywhere over the crate.
ARM_KEEP_OUT = [{"name": "milk crate and its cartons", "x": CRATE_X, "y": CRATE_Y, "z_top": TOP + CRATE_H + 0.011}]

# ---- Scripted demo (right arm only; the left arm holds READY) ---------------------------------------------------------
LEVEL = math.pi / 2  # pitch: gripper level, pointing +x, pads closing along y on the upright handle
GRIP_Z = PROBE_HOME[2] + GRIP_UP  # TOP + 0.175
CARRY_Z = TOP + 0.470  # tip at TOP + 0.303, clear of the rim (0.279) and fins (0.250)
SEAM_XY = (ROWS_X[2], (COLS_Y[1] + COLS_Y[2]) / 2)  # between the centre carton and the one to its right
SEAM_PAIR = (carton_name(2, 1), carton_name(2, 2))
ENTRY_TIP = SHOULDER_Z + 0.008
READ_DEPTH = 0.042  # aim for the middle of the 30 to 70 mm band
PRE_Z = TOP + 0.265  # open fingers come down either side of the handle (top TOP + 0.235)
VIA = (0.19, -0.26, TOP + 0.30)
DOWN = np.array([0.0, 0.0, -1.0])
HALF_OPEN = FINGER_OPEN * 0.045 / 0.094  # pads 45 mm apart round the 19 mm handle: the fingers stay clear of the crate wall
MAX_PUSH = 0.034  # most extra press lag (m) the demo builds on the blade: about 7 N at this reach
LIFT_GUARD = 0.0015  # a neighbour carton 1.5 mm up while the blade draws out: stop, lower, ease off it

demo = placeholder_demo
TIP_FIX_MAX = 0.020  # a live tip more than 2 cm from where the grip puts it means a bad grasp: no correction


def _probe_yaw(st):
    x, y, z, w = (float(v) for v in st.quat_xyzw)
    a = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return (a + math.pi / 2) % math.pi - math.pi / 2  # the blade is symmetric: fold to -90..+90 deg


def live_tip(ctx, arm):
    """The blade tip: read from the probe's simulator pose; offline (nothing moves there) from the encoder tool point."""
    if ctx.offline:
        return np.asarray(arm.measured_pos, float) - np.array([0.0, 0.0, TIP_BELOW_GRIP])
    return tip_point(ctx.obj("probe"))


def seam_centre_y(ctx):
    """The live middle of the seam between the two cartons the blade goes between (their facing walls)."""
    if ctx.offline:
        return SEAM_XY[1]
    a, b = (_origin(ctx.obj(n))[1] for n in SEAM_PAIR)
    return (a + b) / 2


def carton_lift(ctx):
    """Highest rise of any top-layer carton's base above its rest height (0 offline)."""
    if ctx.offline:
        return 0.0, None
    st = ctx.states()
    best, who = -1.0, None
    for k, c in CARTONS.items():
        d = _origin(st[carton_name(*k)])[2] - c.pos[2]
        if d > best:
            best, who = d, k
    return best, who


def tip_correction(ctx, arm, goal_xy):
    """(dxy, dyaw) that put the live blade tip over goal_xy with its flat faces normal to y, read from the probe's
    simulator pose; None offline (the probe never moves there) or when the tip is not where the grip puts it. The round
    handle can turn and swing a little in the pads, so the demo re-aims the tip from the probe's pose."""
    if ctx.offline:
        return None
    st = ctx.obj("probe")
    tip = tip_point(st)
    if np.hypot(*(tip[:2] - arm.measured_pos[:2])) > TIP_FIX_MAX + 0.015:
        return None
    return np.asarray(goal_xy, float) - tip[:2], -_probe_yaw(st)


def _step(arm, p):
    """Command one control step to p, keeping the hand's orientation."""
    yield from arm.move(p, arm.dt, rot=arm.rot.copy())


def _aim(ctx, R, stage, goal_xy, yaw, tries=4, fine=False, lim=TIP_FIX_MAX):
    """Move the hand until the live tip is over goal_xy (and, for the seam, the blade square to it)."""
    stage = np.asarray(stage, float)
    for _ in range(tries):
        fix = tip_correction(ctx, R, goal_xy)
        if fix is None:
            break
        dxy, dyaw = fix
        if np.hypot(*dxy) < 0.0006 and (abs(dyaw) < 0.02 or not fine):
            break
        if fine:
            dxy = dxy * min(1.0, 0.004 / max(np.hypot(*dxy), 1e-9))
            yaw = float(np.clip(yaw + dyaw, -0.25, 0.25))
        stage[:2] = np.clip(stage[:2] + dxy, np.asarray(goal_xy) - lim, np.asarray(goal_xy) + lim)
        yield from R.move(stage, 1.0, pitch=LEVEL, yaw=yaw)
        yield from R.trim_to_measured(stage, tol=0.001)
        yield from R.hold(0.3)
    return stage, yaw


def _unload(R, lag0, duration=1.0):
    """Walk the command back to where the hand really is plus its free-air sag, so it holds without pushing."""
    goal = np.asarray(R.measured_pos, float) + lag0
    yield from R.move(goal, duration, rot=R.rot.copy())


def demo_script(ctx):
    """Right arm: level grip on the probe handle, lift it out of the holster, aim the blade over the seam between the
    centre carton and the one to its right, feel it down the seam to about 42 mm below the shoulders, take the push
    off, hold 20 s, draw it out slowly (easing off any carton that starts to rise with it), stand it back in the
    holster, let go and back away. Reads the probe's and the cartons' live poses."""
    R = ctx.right
    hx, hy = HOLSTER_XY
    pre = (hx, hy, PRE_Z)
    grip = (hx, hy, GRIP_Z)
    yield from R.move(VIA, 2.5, HALF_OPEN, pitch=LEVEL)
    for attempt in range(2):
        yield from R.move(pre, 2.0, HALF_OPEN, pitch=LEVEL)
        yield from R.trim_to_measured(pre)
        yield from R.move(grip, 1.8, pitch=LEVEL)
        yield from R.trim_to_measured(grip)
        yield from R.grip(FINGER_CLOSED, 0.8)
        yield from R.hold(0.4)
        if R.holding(0.010):
            break
        yield from R.grip(HALF_OPEN, 0.6)
        yield from R.move(pre, 1.2, pitch=LEVEL)
    else:
        return
    # out of the holster, straight up, then over the seam
    yield from R.move((hx, hy, CARRY_Z), 5.5, pitch=LEVEL)
    over = np.array([SEAM_XY[0], SEAM_XY[1], CARRY_Z])
    yield from R.move(over, 3.5, pitch=LEVEL)
    yield from R.trim_to_measured(over)
    over, yaw = yield from _aim(ctx, R, over, SEAM_XY, 0.0, fine=True)
    entry = np.array([over[0], over[1], ENTRY_TIP + TIP_BELOW_GRIP])
    yield from R.move(entry, 3.0, pitch=LEVEL, yaw=yaw)
    yield from R.trim_to_measured(entry, tol=0.001)
    yield from R.hold(0.5)
    entry, yaw = yield from _aim(ctx, R, entry, SEAM_XY, yaw, fine=True)
    lag0 = np.asarray(R.pos, float) - np.asarray(R.measured_pos, float)  # free-air sag with the probe in hand
    # Feel the blade down the seam: lower the command 6 mm/s while the live tip is short of the reading depth, never
    # more than MAX_PUSH ahead of the hand (that caps the push), and keep the tip on the middle of the seam.
    cmd = np.asarray(R.pos, float).copy()
    for _ in range(int(30.0 / R.dt)):
        tip = live_tip(ctx, R)
        depth = SHOULDER_Z - tip[2]
        if depth >= READ_DEPTH:
            break
        push = float((cmd - np.asarray(R.measured_pos) - lag0) @ DOWN)
        if push < MAX_PUSH:
            cmd[2] -= 0.0003
        if depth > 0.004:
            cmd[1] += float(np.clip(0.3 * (seam_centre_y(ctx) - tip[1]), -0.0002, 0.0002))
        cmd[1] = float(np.clip(cmd[1], entry[1] - 0.004, entry[1] + 0.004))
        yield from _step(R, cmd)
    # take the push off so the hand holds the probe still, then the reading
    yield from _unload(R, lag0, 1.0)
    yield from R.hold(20.0)  # the reading: 15 s still is scored; the maker gives under 15 s to a full reading
    # Draw it out at 1 cm/s. If a neighbour starts to rise with the blade, stop, lower it back and ease the blade off
    # that carton by 0.4 mm, then go on.
    cmd = np.asarray(R.pos, float).copy()
    out_z = FIN_TOP_Z + 0.020 + TIP_BELOW_GRIP
    eased = 0
    for _ in range(int(40.0 / R.dt)):
        tip = live_tip(ctx, R)
        if tip[2] > FIN_TOP_Z + 0.010 and cmd[2] >= out_z:
            break
        lift, who = carton_lift(ctx)
        if lift > LIFT_GUARD and eased < 8:
            eased += 1
            side = 1.0 if who is not None and COLS_Y[who[1]] < tip[1] else -1.0  # away from the rising carton
            for _ in range(10):
                cmd[2] -= 0.0004
                yield from _step(R, cmd)
            cmd[1] += side * 0.0004
            yield from _step(R, cmd)
            yield from R.hold(0.5)
            continue
        cmd[2] = min(cmd[2] + 0.0005, out_z)
        yield from _step(R, cmd)
    yield from R.move(over, 2.0, pitch=LEVEL, yaw=yaw)
    # carry it back and stand it in the holster: aim the live tip at the hole, lower it in, feel it seat
    above = np.array([hx, hy, CARRY_Z])
    yield from R.move(above, 3.5, pitch=LEVEL)
    yield from R.trim_to_measured(above)
    above, _ = yield from _aim(ctx, R, above, HOLSTER_XY, 0.0, lim=0.010)
    start = np.array([above[0], above[1], GRIP_Z + 0.040])
    for attempt in range(3):
        yield from R.move(start, 4.0, pitch=LEVEL)
        yield from R.trim_to_measured(start, tol=0.0015)
        start, _ = yield from _aim(ctx, R, start, HOLSTER_XY, 0.0, lim=0.010)
        lag0 = np.asarray(R.pos, float) - np.asarray(R.measured_pos, float)
        cmd = np.asarray(R.pos, float).copy()
        seated = False
        for _ in range(int(12.0 / R.dt)):
            if ctx.offline:
                seated = cmd[2] <= GRIP_Z + 0.001
            else:
                seated = _origin(ctx.obj("probe"))[2] <= PROBE_HOME[2] + 0.0015
            if seated:
                break
            push = float((cmd - np.asarray(R.measured_pos) - lag0) @ DOWN)
            if push < 0.010:
                cmd[2] -= 0.0004
            yield from _step(R, cmd)
        if seated:
            break
        # not seated: lift 3 cm, re-aim and try again
        yield from R.move(np.asarray(R.pos) + [0.0, 0.0, 0.030], 1.5, pitch=LEVEL)
    yield from _unload(R, lag0, 0.8)
    yield from R.hold(0.3)
    yield from R.grip(HALF_OPEN, 2.0)  # slow open: the probe stays standing
    yield from R.hold(0.5)
    yield from R.move((hx, hy, GRIP_Z + 0.130), 3.0, pitch=LEVEL)  # straight up, fingers clear of the handle top
    yield from R.move(VIA, 2.0, pitch=LEVEL)
    yield from R.home(3.0)
    yield from R.hold(3.0)
