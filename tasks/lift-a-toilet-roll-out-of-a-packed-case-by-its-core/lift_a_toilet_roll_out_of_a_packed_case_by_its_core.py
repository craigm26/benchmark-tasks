"""On the station table, in front of the right arm: an open kraft corrugated case holding four white toilet rolls
standing on end, packed wall to wall (1 mm between rolls and to the walls), each showing its brown cardboard core.
Between the case and the robot stands an empty charcoal plastic bin. The rolls are loose; the case and the bin are
fixed. A roll is 114 mm across, wider than the 94 mm the gripper opens, and no finger fits down its side: it comes
out by its 42 mm core.

Requires -E object_jitter=0: the rolls are packed to 1 mm and the scorer measures from the starting layout.

Run it:  inspect-robots run --embodiment yam_arena -E task=lift_a_toilet_roll_out_of_a_packed_case_by_its_core.py \\
           -E object_jitter=0 --instruction "Take one toilet roll out of the case and stand it upright in the grey
           bin. Leave the other rolls in the case." --scorer task_score ...
"""

import dataclasses
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

import mechkit  # noqa: E402
from belimits import bottom, tilt_deg  # noqa: E402
from closedloop import placeholder_demo  # noqa: E402
from mechkit import Mech, box, cyl, on_reset, released, tube  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 8.0  # the demo grips: leave time to let go, rise and park after the first 1.0

KRAFT, FLAP = (0.72, 0.56, 0.38), (0.40, 0.28, 0.16)
PAPER, CORE, SHADOW = (0.95, 0.95, 0.93), (0.45, 0.32, 0.20), (0.25, 0.17, 0.10)
CHARCOAL = (0.18, 0.19, 0.21)

# ---- the toilet rolls: full-size 4.5 x 4 in institutional rolls --------------------------------------------------------
ROLL_R, ROLL_H, BORE_R, ROLL_MASS = 0.057, 0.102, 0.021, 0.15  # 114 mm across, 102 mm tall, 42 mm bore, 150 g
GAP = 0.001  # between rolls, and from each roll to the case walls
PITCH = 2 * ROLL_R + GAP

# ---- the case: a 2 x 2 corner of a one-layer case, single-wall corrugated, fixed ---------------------------------------
WALL, CASE_FLOOR = 0.004, 0.004
CASE_IN = 2 * PITCH + GAP  # 231 mm inside
CASE_OUT = CASE_IN + 2 * WALL  # 239 mm outside
CASE_DEPTH = ROLL_H + 0.001  # the walls stand 1 mm over the rolls
RIM = CASE_FLOOR + CASE_DEPTH  # 107 mm above the table
A_XY = (0.275, -0.180)  # roll_a, the roll nearest the right arm
CASE_XY = (A_XY[0] + PITCH / 2, A_XY[1] + PITCH / 2)
FLAP_T, FLAP_L = 0.002, 0.100


def _open_box(out, floor, rim, color):
    """Floor and four walls of an open-topped square box, in a frame at the centre of its footprint on the table."""
    h, c = rim - floor, out / 2 - WALL / 2
    return [
        box((0, 0, floor / 2), (out, out, floor), color),
        box((-c, 0, floor + h / 2), (WALL, out, h), color),
        box((c, 0, floor + h / 2), (WALL, out, h), color),
        box((0, -c, floor + h / 2), (out - 2 * WALL, WALL, h), color),
        box((0, c, floor + h / 2), (out - 2 * WALL, WALL, h), color),
    ]


def _flaps():
    """The four top flaps, folded down flat against the outside of the walls (look only)."""
    o, z = CASE_OUT / 2 + FLAP_T / 2, RIM - FLAP_L / 2
    shapes = [box((s * o, 0, z), (FLAP_T, CASE_OUT, FLAP_L), FLAP) for s in (-1, 1)]
    shapes += [box((0, s * o, z), (CASE_OUT, FLAP_T, FLAP_L), FLAP) for s in (-1, 1)]
    return [dataclasses.replace(s, collide=False) for s in shapes]


CASE = Mech("case", pos=(float(CASE_XY[0]), float(CASE_XY[1]), float(TOP)), mass=None, friction=0.6,
            shapes=[*_open_box(CASE_OUT, CASE_FLOOR, RIM, KRAFT), *_flaps()])


def _roll(name, x, y):
    return Mech(
        name, pos=(float(x), float(y), float(TOP + CASE_FLOOR + ROLL_H / 2)), mass=ROLL_MASS, friction=0.6,
        shapes=[
            tube((0, 0, 0), BORE_R, ROLL_R, ROLL_H, PAPER, sides=24),
            dataclasses.replace(tube((0, 0, 0), 0.0205, 0.023, ROLL_H + 0.0005, CORE, sides=24), collide=False),
            dataclasses.replace(cyl((0, 0, -ROLL_H / 2 + 0.0005), 0.0205, 0.001, SHADOW), collide=False),
        ],
    )


ROLL_A = _roll("roll_a", A_XY[0], A_XY[1])
ROLL_B = _roll("roll_b", A_XY[0] + PITCH, A_XY[1])
ROLL_C = _roll("roll_c", A_XY[0], A_XY[1] + PITCH)
ROLL_D = _roll("roll_d", A_XY[0] + PITCH, A_XY[1] + PITCH)
ROLLS = (ROLL_A, ROLL_B, ROLL_C, ROLL_D)
SPEC = {r.name: r for r in ROLLS}

# ---- the bin (a cart caddy): charcoal plastic, fixed, 30 mm from the case on the robot's side ---------------------------
BIN_IN, BIN_FLOOR, BIN_RIM = 0.150, 0.004, 0.064
BIN_OUT = BIN_IN + 2 * WALL
BIN_XY = (CASE_XY[0] - CASE_OUT / 2 - 0.030 - BIN_OUT / 2, -0.100)
BIN = Mech("bin", pos=(float(BIN_XY[0]), float(BIN_XY[1]), float(TOP)), mass=None, friction=0.6,
           shapes=_open_box(BIN_OUT, BIN_FLOOR, BIN_RIM, CHARCOAL))

OBJECTS = [CASE, BIN, *ROLLS]

# ---- scoring ----------------------------------------------------------------------------------------------------------
STEP_HZ = 20.0
SETTLE_STEPS = 10  # baselines are read after 0.5 s
V_RISE, V_TILT = 0.005, 10.0  # another roll lifted 5 mm or tipped 10 degrees: V
DROP_SPEED = 0.5  # m/s: D, the taken roll's centre faster than this (2.5 cm between two frames at 20 Hz)
REST_SPEED, REST_STEPS = 0.005, 40  # every roll under 5 mm/s for 2 s
RELEASE_CLEAR = 0.03  # every finger point this far from every roll
SEAT_GAP = 0.006  # resting: lowest point within this of the surface under it
UPRIGHT = 15.0  # degrees from vertical
OFFLINE_FINGERS_CLEAR = True  # what the release test returns with no simulator (the checker flips it for a held roll)

_L = {"n": 0, "base": None, "taken": None, "rise": {}, "tilt": {}, "v": False, "dropped": False, "still": 0, "seen": None, "last": None}


@on_reset
def _clear(_env=None):
    _L.update(n=0, base=None, taken=None, rise={}, tilt={}, v=False, dropped=False, still=0, seen=None, last=None)


def _fingers_clear(objects) -> bool:
    """Both grippers at least 3 cm from every roll. Only the simulator can tell; offline replays have no fingers to
    measure, so they use OFFLINE_FINGERS_CLEAR."""
    if mechkit._ENV["env"] is None:
        return OFFLINE_FINGERS_CLEAR
    return all(released(objects[r.name], RELEASE_CLEAR) for r in ROLLS)


def _update(objects) -> None:
    """Latch what happened this control step. yam_arena calls success() and score() on the same dict each step, so a
    dict already seen changes nothing. A trial reset (every roll back at its start, after any had left) re-arms."""
    if objects is _L["seen"]:
        return
    here = {r.name: np.asarray(objects[r.name].pos, dtype=float) for r in ROLLS}
    home = all(np.linalg.norm(here[r.name] - np.asarray(r.pos)) < 0.004 for r in ROLLS)
    if home and _L["last"] is not None and any(
            np.linalg.norm(_L["last"][r.name] - np.asarray(r.pos)) > 0.03 for r in ROLLS):
        _clear()  # a teleport back to the start layout: a new trial
    _L["seen"] = objects
    last, _L["last"] = _L["last"], here
    _L["n"] += 1
    if _L["base"] is None:
        if _L["n"] >= SETTLE_STEPS:
            _L["base"] = {r.name: float(here[r.name][2]) for r in ROLLS}
        return
    for r in ROLLS:  # R: a roll's lowest point above the rim of the case
        if bottom(objects[r.name], r) > TOP + RIM:
            if _L["taken"] is None:
                _L["taken"] = r.name
            elif _L["taken"] != r.name:
                _L["v"] = True  # a second roll came out
    for r in ROLLS:  # each roll's highest rise and steepest tilt since the baseline
        _L["rise"][r.name] = max(_L["rise"].get(r.name, 0.0), float(here[r.name][2]) - _L["base"][r.name])
        _L["tilt"][r.name] = max(_L["tilt"].get(r.name, 0.0), tilt_deg(objects[r.name]))
    steps = {n: float(np.linalg.norm(here[n] - last[n])) for n in here}
    if _L["taken"] and steps[_L["taken"]] * STEP_HZ > DROP_SPEED:
        _L["dropped"] = True
    _L["still"] = _L["still"] + 1 if max(steps.values()) * STEP_HZ < REST_SPEED else 0


def disturbed() -> bool:
    """V: a second roll came out, or any roll but the one taken ever rose 5 mm or tipped 10 degrees."""
    return bool(_L["v"] or any(_L["rise"].get(r.name, 0.0) > V_RISE or _L["tilt"].get(r.name, 0.0) > V_TILT
                               for r in ROLLS if r.name != _L["taken"]))


def _over(state, xy, half) -> bool:
    return bool(abs(state.pos[0] - xy[0]) < half and abs(state.pos[1] - xy[1]) < half)


def in_bin(state, spec) -> bool:
    """Centre inside the bin's inner walls and lowest point within 6 mm of its floor."""
    return _over(state, BIN_XY, BIN_IN / 2) and abs(bottom(state, spec) - (TOP + BIN_FLOOR)) <= SEAT_GAP


def on_table(state, spec) -> bool:
    """Lowest point within 6 mm of the bare table top, centre outside the case and the bin (so not back in the case,
    not in the bin, not perched on either)."""
    return bool(abs(bottom(state, spec) - TOP) <= SEAT_GAP and not _over(state, CASE_XY, CASE_OUT / 2)
                and not _over(state, BIN_XY, BIN_OUT / 2))


def score(objects):
    """0 if any roll but the first one out rose 5 mm or tipped 10 degrees, or a second roll came out (V). Otherwise,
    once the fingers are 3 cm clear of every roll and the rolls are still: 1.0 with the roll that came out standing
    upright in the bin and never faster than 0.5 m/s; 0.5 in the bin but lying or leaning over, or after moving faster
    than 0.5 m/s; 0.25 at rest on the table. Else 0."""
    _update(objects)
    if _L["taken"] is None or disturbed():
        return 0.0
    if not (_L["still"] >= REST_STEPS and _fingers_clear(objects)):
        return 0.0
    s, spec = objects[_L["taken"]], SPEC[_L["taken"]]
    if in_bin(s, spec):
        return 1.0 if (tilt_deg(s) <= UPRIGHT and not _L["dropped"]) else 0.5
    return 0.25 if on_table(s, spec) else 0.0


def success(objects):
    return score(objects) == 1.0


# ---- the robot keeps off the other rolls ------------------------------------------------------------------------------
# Over the three rolls that stay, no link origin and no point tracked along the links of either arm comes lower than
# 3 cm above the rim: the hand works in roll_a's core and never presses on or hooks a neighbour.
KEEP_OFF_Z = RIM + 0.030
_CASE_X = (CASE_XY[0] - CASE_OUT / 2, CASE_XY[0] + CASE_OUT / 2)
_CASE_Y = (CASE_XY[1] - CASE_OUT / 2, CASE_XY[1] + CASE_OUT / 2)
_SPLIT_X, _SPLIT_Y = A_XY[0] + PITCH / 2, A_XY[1] + PITCH / 2
ARM_KEEP_OUT = [
    {"name": "far rolls (roll_b, roll_d)", "x": (_SPLIT_X, _CASE_X[1]), "y": _CASE_Y, "z_top": TOP + KEEP_OFF_Z},
    {"name": "left near roll (roll_c)", "x": (_CASE_X[0], _SPLIT_X), "y": (_SPLIT_Y, _CASE_Y[1]),
     "z_top": TOP + KEEP_OFF_Z},
]

# ---- the scripted demo ------------------------------------------------------------------------------------------------
TIP_PAST = 0.013  # the closed fingertips reach 13 mm past the tool point
ENTER = 0.019  # fingertips this far into the core: the leaned finger bodies stay off the roll's top
GRIP_BELOW_TOP = ENTER - TIP_PAST  # tool point 6 mm under the roll's top
HANG = ROLL_H / 2 - GRIP_BELOW_TOP  # roll centre this far under the tool point while it hangs on the fingers
Z_ABOVE = RIM + 0.030  # tool point height for the approach, fingertips 17 mm over the roll
LEAN = 0.15  # rad: the hand leans 9 degrees about the pad line (y), which lifts the arm's reach ceiling from 22 to 26 cm
EXPAND_GAP, EXPAND_MAX, STALL = 0.030, 0.046, 0.011  # offline pad gap command; live limit; command minus encoder = bearing
GAP_LOST = 0.006  # an encoder gap under this in the lift means the fingers are no longer on the core wall
CLEAR_RIM, MIN_CLEAR = 0.012, 0.004  # the roll's foot is carried this far over the rim; never under the second
CLEAR_BIN = 0.020  # and this far over the bin's rim on the way down into it
SET_GAP = 0.003  # the roll is let go this far above the bin floor
ENTER_SPEED, LIFT_SPEED, CARRY_SPEED, LOWER_SPEED, EXIT_SPEED = 0.020, 0.025, 0.09, 0.030, 0.040  # m/s
PARK_S = 5.0


def z_cap(x: float) -> float:
    """The highest tool point (above the table) the leaned right arm is commanded to at station x: its reach ceiling
    falls from 26 cm over roll_a to 24 cm over the gap between the case and the bin, less a margin."""
    return float(np.clip(0.210 + 0.5 * (x - 0.10), 0.200, 0.245))


def _finger(gap: float) -> float:
    """Finger joint command for a pad gap in metres."""
    return -min(max(gap, 0.0), 2 * abs(FINGER_OPEN)) / 2


class _Lag:
    """The command offset that cancels the lag the joint encoders report (the load makes the wrist sag), filtered."""

    def __init__(self, arm, gain=0.2, limit=0.035):
        self.arm, self.gain, self.limit = arm, gain, limit
        self.off = arm.pos - np.asarray(arm.measured_pos, dtype=float)

    def to(self, want, finger=None):
        """One control step toward having the measured tool point at ``want``."""
        arm = self.arm
        off = self.off + self.gain * (arm.pos - np.asarray(arm.measured_pos, dtype=float) - self.off)
        n = float(np.linalg.norm(off))
        self.off = off if n <= self.limit else off * (self.limit / n)
        cmd = np.asarray(want, dtype=float) + self.off
        cmd[2] = min(cmd[2], TOP + z_cap(cmd[0]))
        yield from arm.move(cmd, arm.dt, finger, rot=arm.rot.copy())


def _line(arm, lag, start, goal, speed):
    """Move the measured tool point in a straight line from ``start`` to ``goal`` at ``speed`` (m/s)."""
    start, goal = np.asarray(start, dtype=float), np.asarray(goal, dtype=float)
    steps = max(1, round(float(np.linalg.norm(goal - start)) / speed / arm.dt))
    for k in range(1, steps + 1):
        yield from lag.to(start + (goal - start) * k / steps)


def _roll_a(ctx, nominal):
    """(centre, tilt in degrees) of roll_a: from the simulator in a live run; offline, where nothing moves, the
    ``nominal`` centre the step expects."""
    s = ctx.obj("roll_a")
    if ctx.right.offline:
        return np.asarray(nominal, dtype=float), 0.0
    return np.asarray(s.pos, dtype=float), tilt_deg(s)


def _log(msg):
    if os.environ.get("ROLL_DEMO_LOG"):
        print("DEMO " + msg, flush=True)


demo = placeholder_demo


def demo_script(ctx):
    """Right arm only, hand leaned 9 degrees about the pad line. Closed fingers go 19 mm into roll_a's core, open until their backs bear on the core wall, lift
    the roll straight up until its foot is over the rim, carry it over the near wall to the bin, lower it to 3 mm above
    the bin floor, close to let it go, rise out of the core and park. The left arm stays at its ready pose."""
    R = ctx.right
    home_c = np.array(ROLL_A.pos, dtype=float)
    top_z = TOP + CASE_FLOOR + ROLL_H

    # 1) approach: closed fingers over roll_a's core, the hand leaned about the pad line
    over = np.array([A_XY[0], A_XY[1], TOP + Z_ABOVE])
    yield from R.move(over, 4.0, FINGER_CLOSED, pitch=LEAN)
    yield from R.trim_to_measured(over)
    lag = _Lag(R)

    def out_of_core():
        yield from R.grip(FINGER_CLOSED, 1.0)
        here = np.array(R.measured_pos, dtype=float)
        yield from _line(R, _Lag(R), here, (here[0], here[1], TOP + Z_ABOVE), EXIT_SPEED)

    def enter():
        nonlocal lag
        c, _ = _roll_a(ctx, home_c)
        above = np.array([c[0], c[1], TOP + Z_ABOVE])
        yield from _line(R, lag, R.measured_pos, above, 0.02)
        for _ in range(10):
            yield from lag.to(above)
        # 2) enter: straight down the core's axis, fingertips 19 mm in
        grip_at = np.array([c[0], c[1], top_z - GRIP_BELOW_TOP])
        yield from _line(R, lag, above, grip_at, ENTER_SPEED)
        for _ in range(6):
            yield from lag.to(grip_at)
        if R.measured_pos[2] > grip_at[2] + 0.0015:  # held up short: the hand is resting on the roll. Back off 1.5 mm
            grip_at[2] = R.measured_pos[2] + 0.0015
            lag = _Lag(R)
            for _ in range(10):
                yield from lag.to(grip_at)
        # 3) expand: open at 10 mm/s until the encoder gap stops 11 mm short of the command (the finger backs bear on
        # the core wall), holding the tool point on the core's axis
        gap_cmd, bearing = 0.0, R.offline
        while gap_cmd < (EXPAND_GAP if R.offline else EXPAND_MAX):
            gap_cmd += 0.010 * R.dt
            yield from R.move(R.pos.copy(), R.dt, _finger(gap_cmd), rot=R.rot.copy())  # the arm command held still
            if not R.offline and gap_cmd - R.gap() >= STALL:
                bearing = True
                break
        yield from R.hold(0.5)
        # centre: move the hand until the roll is back on the line it stood on, so it presses on no neighbour
        want = R.pos.copy()
        for _ in range(int(3.0 * STEP_HZ)):
            c1, _ = _roll_a(ctx, c)
            err = c[:2] - c1[:2]
            if float(np.linalg.norm(err)) < 0.0003:
                break
            want[:2] += np.clip(0.5 * err, -0.0003, 0.0003)
            yield from R.move(want, R.dt, rot=R.rot.copy())
        c1, _ = _roll_a(ctx, c)
        _log(f"expand cmd_mm={1000 * gap_cmd:.1f} gap_mm={1000 * R.gap():.1f} bearing={bearing} "
             f"off_mm={np.round(1000 * (c1 - c), 2)}")
        return bool(bearing and np.linalg.norm(c1[:2] - c[:2]) < 0.003 and abs(c1[2] - c[2]) < 0.002)

    ok = yield from enter()
    if not ok:  # close, come out, find the axis again and try once more
        yield from out_of_core()
        lag = _Lag(R)
        ok = yield from enter()
    if not ok:  # not in hand: stop rather than guess
        yield from out_of_core()
        yield from R.home(PARK_S)
        return

    # 4) lift: straight up until the roll's foot reads 12 mm over the rim, held on its start line
    lag = _Lag(R)
    want = np.array(R.measured_pos, dtype=float)
    foot_goal = TOP + RIM + CLEAR_RIM
    start_c, _ = _roll_a(ctx, home_c)
    k = 0
    for k in range(int(40 * STEP_HZ)):
        c, tilt = _roll_a(ctx, want - (0, 0, HANG))
        foot = c[2] - ROLL_H / 2
        if foot >= foot_goal or want[2] >= foot_goal + ROLL_H / 2 + HANG + 0.025 or R.gap() < GAP_LOST and not R.offline:
            break
        want[2] += LIFT_SPEED * R.dt * (0.4 if tilt > 3.0 else 1.0)  # slower while it swings
        if foot < TOP + RIM:  # still between its neighbours: keep its axis on the start line
            want[:2] += np.clip(start_c[:2] - c[:2], -0.0002, 0.0002)
        if k % 20 == 0:
            _log(f"lift want={want[2] - TOP:.4f} cmd={R.pos[2] - TOP:.4f} meas={R.measured_pos[2] - TOP:.4f} "
                 f"foot={foot - TOP:.4f} gap={1000 * R.gap():.1f} tilt={tilt:.2f} off_mm={np.round(1000 * (c - start_c)[:2], 2)}")
        yield from lag.to(want)

    for _ in range(int(2.0 * STEP_HZ)):  # let the arm catch up with its command
        c, _ = _roll_a(ctx, want - (0, 0, HANG))
        if c[2] - ROLL_H / 2 >= TOP + RIM + MIN_CLEAR:
            break
        yield from lag.to(want)
    if c[2] - ROLL_H / 2 < TOP + RIM + MIN_CLEAR:  # the foot is not safely over the rim: put the roll back, do not drag it
        _log(f"lift failed foot={c[2] - ROLL_H / 2 - TOP:.4f}")
        yield from _line(R, lag, want, (want[0], want[1], top_z - GRIP_BELOW_TOP + 0.002), LIFT_SPEED)
        yield from out_of_core()
        yield from R.home(PARK_S)
        return

    # 5) carry: over the near wall toward the bin, holding the roll's foot at its height over the rim by its read pose;
    # once the whole roll is past the wall, down to 20 mm over the bin's rim. Then let the swing settle.
    past_wall = _CASE_X[0] - ROLL_R - 0.008  # roll centre x at which its far edge is 8 mm clear of the case
    start_xy, goal_xy = want[:2].copy(), np.array(BIN_XY, dtype=float)
    steps = max(1, round(float(np.linalg.norm(goal_xy - start_xy)) / CARRY_SPEED / R.dt))
    low_goal = TOP + BIN_RIM + CLEAR_BIN
    for k in range(1, steps + int(1.0 * STEP_HZ) + 1):
        want[:2] = start_xy + (goal_xy - start_xy) * min(1.0, k / steps)
        c, tilt = _roll_a(ctx, want - (0, 0, HANG))
        frac = float(np.clip((past_wall - c[0]) / max(past_wall - goal_xy[0], 1e-6), 0.0, 1.0))
        goal = foot_goal + (low_goal - foot_goal) * frac
        want[2] += float(np.clip(goal - (c[2] - ROLL_H / 2), -LOWER_SPEED * R.dt, LIFT_SPEED * R.dt))
        if k % 20 == 0:
            _log(f"carry x={c[0]:.3f} cmd={R.pos[2] - TOP:.4f} meas={R.measured_pos[2] - TOP:.4f} "
                 f"foot={c[2] - ROLL_H / 2 - TOP:.4f} goal={goal - TOP:.4f} gap={1000 * R.gap():.1f} tilt={tilt:.2f}")
        yield from lag.to(want)
    c, _ = _roll_a(ctx, want - (0, 0, HANG))
    fix = np.clip(goal_xy - c[:2], -0.012, 0.012)  # the roll's own axis over the bin's centre
    yield from _line(R, lag, want, want + np.r_[fix, 0.0], 0.02)
    want[:2] += fix

    # 6) set down: lower until the roll's foot reads 3 mm above the bin floor
    floor_z = TOP + BIN_FLOOR + SET_GAP
    for _ in range(int(12 * STEP_HZ)):
        c, _ = _roll_a(ctx, want - (0, 0, HANG))
        if c[2] - ROLL_H / 2 <= floor_z or want[2] <= floor_z + ROLL_H / 2 + HANG - 0.004:
            break
        want[2] -= LOWER_SPEED * R.dt
        yield from lag.to(want)
    _log(f"set down foot={c[2] - ROLL_H / 2 - TOP:.4f} xy_off_mm={np.round(1000 * (c[:2] - goal_xy), 1)}")

    # 7) release: close the fingers, rise straight up out of the core, park
    yield from R.grip(FINGER_CLOSED, 1.0)
    yield from R.hold(0.5)
    lag = _Lag(R)
    here = np.array(R.measured_pos, dtype=float)
    yield from _line(R, lag, here, (here[0], here[1], TOP + 0.185), EXIT_SPEED)
    yield from R.home(PARK_S)
    yield from ctx.wait(2.0)
