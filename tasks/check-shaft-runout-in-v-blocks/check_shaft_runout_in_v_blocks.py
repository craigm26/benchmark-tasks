"""A runout check on an inspection bench. A black granite surface plate (300 x 300 x 50 mm) stands on a low steel
bench stand in front of the arms. On it, two hardened steel V-blocks (50 x 50 x 40 mm, 90 degree V) carry a loose
ground steel shaft, 20 mm diameter and 200 mm long, lying along y with its far end 0.5 mm from a steel end stop. Between
the blocks the shaft has a 24 mm journal turned 0.25 mm off centre (0.50 mm total indicator reading, TIR). A dial
indicator on a magnetic stand rests its sprung plunger (flat contact point) on top of the journal, preloaded 2.0 mm.
The shaft carries two red stripes on its far section and a red bar on its near end face so its turn can be seen.

The job: turn the shaft one full turn in place with repeated regrasps while it stays down in both Vs and against the
end stop, so the indicator sweeps the journal's true runout, then let go.

Requires -E object_jitter=0: the seat is measured from the shaft's own rest height in the first second.

Run it:  inspect-robots run --embodiment yam_arena -E task=check_shaft_runout_in_v_blocks.py -E object_jitter=0 \\
           --instruction "Turn the shaft one full turn in its V-blocks under the dial indicator, keeping it down in
           both Vs and against the end stop, then let go." --scorer task_score ...
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

import mechkit  # noqa: E402
from closedloop import grip_rot, placeholder_demo  # noqa: E402
from dataclasses import replace  # noqa: E402

from mechkit import Joint, Mech, box, cyl, extrude, on_reset, rod, tube  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 8.0  # the demo grips: leave time to back out and park after the first full score

BLACK, STEEL, BRIGHT, DARK = (0.05, 0.05, 0.06), (0.62, 0.64, 0.66), (0.88, 0.89, 0.91), (0.30, 0.31, 0.33)
JOURNAL_C, RED, WHITE, CHROME, STAND_C = (0.42, 0.44, 0.47), (0.85, 0.08, 0.08), (0.96, 0.96, 0.94), (0.80, 0.82, 0.85), (0.20, 0.21, 0.23)

# ---- the bench: stand, granite plate, V-blocks, end stop ---------------------------------------------------------------
RISE = 0.054  # steel bench stand under the plate
PLATE_X, PLATE_Y, PLATE_H = (0.130, 0.430), (-0.095, 0.205), 0.050
PLATE_TOP = TOP + RISE + PLATE_H
STAND = Box("bench_stand", pos=(0.280, 0.055, TOP + RISE / 2), size=(0.260, 0.260, RISE), color=STAND_C)
PLATE = Box("surface_plate", pos=(0.280, 0.055, TOP + RISE + PLATE_H / 2), size=(0.300, 0.300, PLATE_H), color=BLACK)

AXIS_X = 0.280
BLOCK, BLOCK_LEN, V_W, V_D = 0.050, 0.040, 0.036, 0.018
V_NEAR_Y, V_FAR_Y = -0.050, 0.040  # block centres along the shaft
BLOCK_TOP = PLATE_TOP + BLOCK
_V_OUTLINE = [(-BLOCK / 2, 0.0), (BLOCK / 2, 0.0), (BLOCK / 2, BLOCK), (V_W / 2, BLOCK), (0.0, BLOCK - V_D),
              (-V_W / 2, BLOCK), (-BLOCK / 2, BLOCK)]
# extrude's canonical (u, v, w) with w along y maps u to z and v to x; the outline is written (x, z), so swap it.
_V_ZX = [(z, x) for x, z in _V_OUTLINE][::-1]


def _vblock(name: str, y: float) -> Mech:
    return Mech(name, pos=(AXIS_X, float(y), PLATE_TOP), mass=None, friction=0.2,
                shapes=[extrude((0.0, 0.0, 0.0), _V_ZX, BLOCK_LEN, STEEL, axis="y")])


V_NEAR, V_FAR = _vblock("v_block_near", V_NEAR_Y), _vblock("v_block_far", V_FAR_Y)

# ---- the shaft ------------------------------------------------------------------------------------------------------
SHAFT_R, SHAFT_LEN, JOURNAL_R, JOURNAL_LEN, ECC = 0.010, 0.200, 0.012, 0.040, 0.00025
SHAFT_Y = (-0.120, 0.080)
SHAFT_MID_Y = (SHAFT_Y[0] + SHAFT_Y[1]) / 2
JOURNAL_Y = -0.005  # journal centre: 5 mm clear of each V-block
AXIS_Z = BLOCK_TOP - V_D + SHAFT_R * math.sqrt(2.0)  # a 20 mm round in a 90 degree V: centre 14.14 mm above the apex
SHAFT_MASS = 0.54  # steel 7850 kg/m3: 20 x 200 mm bar plus the 24 mm journal's 2 mm wall over 40 mm
SHAFT = Mech("shaft", pos=(AXIS_X, SHAFT_MID_Y, AXIS_Z), mass=SHAFT_MASS, friction=0.2, shapes=[
    cyl((0.0, 0.0, 0.0), SHAFT_R, SHAFT_LEN, BRIGHT, axis="y"),
    cyl((ECC, JOURNAL_Y - SHAFT_MID_Y, 0.0), JOURNAL_R, JOURNAL_LEN, JOURNAL_C, axis="y"),  # axis 0.25 mm off the shaft's: 0.50 mm TIR
    replace(box((0.0, 0.090, SHAFT_R - 0.0002), (0.008, 0.018, 0.0012), RED), collide=False),  # stripe, top
    replace(box((0.0, 0.090, -SHAFT_R + 0.0002), (0.008, 0.018, 0.0012), RED), collide=False),  # stripe, 180 deg on
    replace(box((0.0, -SHAFT_LEN / 2 - 0.0003, 0.0045), (0.008, 0.0012, 0.009), RED), collide=False),  # end-face bar
])
V_LOCAL = (V_NEAR_Y - SHAFT_MID_Y, V_FAR_Y - SHAFT_MID_Y)  # where the shaft axis crosses each V-block's middle

STOP_Y0 = SHAFT_Y[1] + 0.0005  # the end stop's near face: 0.5 mm from the shaft's far end
END_STOP = Box("end_stop", pos=(AXIS_X, STOP_Y0 + 0.010, PLATE_TOP + 0.035), size=(0.030, 0.020, 0.070), color=DARK)

# ---- the dial indicator on its magnetic stand ---------------------------------------------------------------------------
IND_Y = JOURNAL_Y  # over the middle of the journal
JOURNAL_TOP = AXIS_Z + JOURNAL_R  # nominal; the real top moves +-0.25 mm as the shaft turns
BASE_X = 0.385
DIAL_R, DIAL_T = 0.029, 0.025
DIAL_Z = PLATE_TOP + 0.144  # dial centre
ARM_Z = PLATE_TOP + 0.130
POST_TOP = PLATE_TOP + 0.150
PLUNGER_ZERO = JOURNAL_TOP + 0.0005  # contact face at joint value 0: just clear of the journal's highest point
PLUNGER_LOW, PLUNGER_HIGH = -0.004, 0.006  # 10 mm of travel
PLUNGER_MASS = 0.066  # its weight is the 0.65 N gauge force


def _z(z: float) -> float:
    return float(z - TOP)


_ticks = [replace(box((-DIAL_T / 2 - 0.0012, 0.023 * math.sin(a), _z(DIAL_Z) + 0.023 * math.cos(a)), (0.0008, 0.0016, 0.005), BLACK,
                      tilt=(math.cos(-a / 2), math.sin(-a / 2), 0.0, 0.0)), collide=False)
          for a in [2.0 * math.pi * k / 12 for k in range(12)]]
INDICATOR = Mech("dial_indicator", pos=(AXIS_X, IND_Y, TOP), mass=None, friction=0.5, shapes=[
    box((BASE_X - AXIS_X, 0.0, _z(PLATE_TOP) + 0.0275), (0.050, 0.060, 0.055), DARK),  # magnetic base
    rod((BASE_X - AXIS_X, 0.0, _z(PLATE_TOP) + 0.055), (BASE_X - AXIS_X, 0.0, _z(POST_TOP)), 0.008, CHROME),  # post
    rod((BASE_X - AXIS_X, 0.0, _z(ARM_Z)), (DIAL_T / 2, 0.0, _z(ARM_Z)), 0.006, CHROME),  # arm
    box((BASE_X - AXIS_X, 0.0, _z(ARM_Z)), (0.020, 0.020, 0.020), DARK),  # clamp
    cyl((0.0, 0.0, _z(DIAL_Z)), DIAL_R, DIAL_T, DARK, axis="x"),  # dial body, 58 mm
    replace(cyl((-DIAL_T / 2 - 0.0005, 0.0, _z(DIAL_Z)), 0.0265, 0.001, WHITE, axis="x"), collide=False),  # face, toward -x
    replace(tube((-DIAL_T / 2 - 0.0010, 0.0, _z(DIAL_Z)), 0.0262, DIAL_R, 0.003, CHROME, axis="x"), collide=False),  # bezel
    replace(box((-DIAL_T / 2 - 0.0014, 0.0045, _z(DIAL_Z) + 0.008), (0.0008, 0.0016, 0.024), BLACK,
                tilt=(math.cos(-0.25), math.sin(-0.25), 0.0, 0.0)), collide=False),  # pointer (drawn, not driven)
    *_ticks,
    replace(rod((0.0, 0.0, _z(JOURNAL_TOP) + 0.022), (0.0, 0.0, _z(DIAL_Z - DIAL_R) + 0.001), 0.006, CHROME), collide=False),  # stem
], joints=[
    Joint("plunger", "slide", axis="z", at=(0.0, 0.0, _z(PLUNGER_ZERO)), low=PLUNGER_LOW, high=PLUNGER_HIGH, spring=0.0,
          damping=2.0, mass=PLUNGER_MASS, friction=0.05, shapes=[
              cyl((0.0, 0.0, 0.0015), 0.004, 0.003, BRIGHT),  # flat contact point, 8 mm
              rod((0.0, 0.0, 0.003), (0.0, 0.0, 0.028), 0.0025, BRIGHT)]),
])

OBJECTS = [STAND, PLATE, V_NEAR, V_FAR, SHAFT, END_STOP, INDICATOR]

# ---- scoring ----------------------------------------------------------------------------------------------------------
SEAT_TOL = 0.00010  # m: seated means the axis no more than this above its rest height at both V-blocks
LIFT_LATCH = 0.00050  # m: the axis this far above rest at either V at any time scores 0
GAP_OK, GAP_LATCH = 0.002, 0.005  # m: end gap that still counts, and the gap that scores 0
ON_MARGIN = 0.0005  # m: the plunger this far off both stops is riding the journal
TIR_TRUE, TIR_BAND = 2.0 * ECC, (0.00035, 0.00065)
FINGER_KEEP = 0.010  # m: a finger point this close to the plunger or the dial scores 0
RELEASE_CLEAR = 0.03
REST_STEPS = 20  # 1 s at 20 Hz: the rest height is averaged over the first second, and "still" is one second
STILL_POS, STILL_DEG = 0.00005, 0.5
OFFLINE_FINGERS_CLEAR = True  # what the finger tests return with no simulator (the checker flips it)
OFFLINE_FINGER_POINTS = None  # the checker may put a list of finger points here

# The dial body and the plunger with its stem, as two boxes in the station frame, for the finger test.
DIAL_BOX = ((AXIS_X - DIAL_T / 2, IND_Y - DIAL_R, DIAL_Z - DIAL_R), (AXIS_X + DIAL_T / 2, IND_Y + DIAL_R, DIAL_Z + DIAL_R))
PLUNGER_BOX = ((AXIS_X - 0.006, IND_Y - 0.006, JOURNAL_TOP), (AXIS_X + 0.006, IND_Y + 0.006, DIAL_Z - DIAL_R))


def _fresh() -> dict:
    return {"seen": None, "n": 0, "sum": [0.0, 0.0], "rest": None, "L": False, "W": False, "P": False, "net": 0.0,
            "theta": None, "pmin": None, "pmax": None, "still": 0, "anchor": None, "peak_lift": 0.0, "max_gap": 0.0}


_L = _fresh()


@on_reset
def _clear(_env=None):
    _L.clear()
    _L.update(_fresh())


def measure(objects) -> dict:
    """What a real trial measures: the shaft axis height at each V-block, the end gap, the turn angle about the shaft
    axis and the plunger's position."""
    s = objects["shaft"]
    rot = s.rotation()
    zs = [float(s.to_world((0.0, y, 0.0))[2]) for y in V_LOCAL]
    gap = STOP_Y0 - float(s.to_world((0.0, SHAFT_LEN / 2, 0.0))[1])
    theta = math.atan2(-float(rot[2, 0]), float(rot[0, 0]))
    return {"z": zs, "gap": gap, "theta": theta, "plunger": float(objects["dial_indicator"].joints.get("plunger", 0.0))}


def _box_distance(points, lo, hi) -> float:
    p = np.asarray(points, dtype=float).reshape(-1, 3)
    out = np.maximum(np.maximum(np.asarray(lo) - p, p - np.asarray(hi)), 0.0)
    return float(np.linalg.norm(out, axis=1).min())


def _finger_points():
    if OFFLINE_FINGER_POINTS is not None:
        return OFFLINE_FINGER_POINTS
    if mechkit._ENV["env"] is None:
        return None
    try:
        return mechkit.finger_points()
    except Exception:  # noqa: BLE001
        return None


def _fingers_clear(objects) -> bool:
    if OFFLINE_FINGER_POINTS is not None:
        return mechkit.finger_clearance(OFFLINE_FINGER_POINTS, objects["shaft"]) >= RELEASE_CLEAR
    if mechkit._ENV["env"] is None:
        return OFFLINE_FINGERS_CLEAR
    return mechkit.released(objects["shaft"], RELEASE_CLEAR)


def _wrap(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def _update(objects) -> None:
    """Latch what happened this control step. yam_arena calls success() and score() on the same dict each step, so a
    dict already seen changes nothing."""
    if objects is _L["seen"]:
        return
    _L["seen"] = objects
    _L["n"] += 1
    m = measure(objects)
    if _L["rest"] is None:  # the first second: the shaft lies untouched, and its height at each V is the reference
        _L["sum"][0] += m["z"][0]
        _L["sum"][1] += m["z"][1]
        _L["theta"] = m["theta"]
        if _L["n"] >= REST_STEPS:
            _L["rest"] = (_L["sum"][0] / _L["n"], _L["sum"][1] / _L["n"])
        return
    lift = max(m["z"][0] - _L["rest"][0], m["z"][1] - _L["rest"][1])
    _L["peak_lift"], _L["max_gap"] = max(_L["peak_lift"], lift), max(_L["max_gap"], m["gap"])
    if lift >= LIFT_LATCH:
        _L["L"] = True
    if m["gap"] > GAP_LATCH:
        _L["W"] = True
    pts = _finger_points()
    if pts is not None and min(_box_distance(pts, *DIAL_BOX), _box_distance(pts, *PLUNGER_BOX)) < FINGER_KEEP:
        _L["P"] = True
    on = PLUNGER_LOW + ON_MARGIN <= m["plunger"] <= PLUNGER_HIGH - ON_MARGIN
    if on:  # an indicator in TIR hold keeps the highest and lowest reading of the whole run, regrasps included
        _L["pmin"] = m["plunger"] if _L["pmin"] is None else min(_L["pmin"], m["plunger"])
        _L["pmax"] = m["plunger"] if _L["pmax"] is None else max(_L["pmax"], m["plunger"])
    step = _wrap(m["theta"] - _L["theta"])
    _L["theta"] = m["theta"]
    if lift <= SEAT_TOL and on and m["gap"] <= GAP_OK:
        _L["net"] += step  # only turning done seated, against the stop and under the plunger counts
    a = _L["anchor"]
    if a is None or float(np.linalg.norm(np.asarray(objects["shaft"].origin) - a[0])) > STILL_POS or \
            abs(_wrap(m["theta"] - a[1])) > math.radians(STILL_DEG):
        _L["anchor"], _L["still"] = (np.array(objects["shaft"].origin, dtype=float), m["theta"]), 0
    else:
        _L["still"] += 1


def counted_deg() -> float:
    return abs(math.degrees(_L["net"]))


def tir() -> float:
    return 0.0 if _L["pmin"] is None else _L["pmax"] - _L["pmin"]


def score(objects):
    """0 if the shaft axis ever rose 0.5 mm at a V-block (L), the end gap ever passed 5 mm (W) or a finger point ever
    came within 10 mm of the plunger or dial (P). 0 unless, at the end, the fingers are 3 cm clear of the shaft and it
    has lain still for 1 s, seated in both Vs within 2 mm of the stop. Then: 1.0 for 360 degrees or more of counted
    turn with the indicator's max minus min within 0.35 to 0.65 mm; 0.5 for 180 degrees or more; 0.25 for 90 degrees
    or more; else 0."""
    _update(objects)
    if _L["rest"] is None or _L["L"] or _L["W"] or _L["P"]:
        return 0.0
    m = measure(objects)
    lift = max(m["z"][0] - _L["rest"][0], m["z"][1] - _L["rest"][1])
    if lift > SEAT_TOL or m["gap"] > GAP_OK or _L["still"] < REST_STEPS or not _fingers_clear(objects):
        return 0.0
    turn = counted_deg()
    if turn >= 360.0 and TIR_BAND[0] <= tir() <= TIR_BAND[1]:
        return 1.0
    if turn >= 180.0:
        return 0.5
    if turn >= 90.0:
        return 0.25
    return 0.0


def success(objects):
    return score(objects) == 1.0


# ---- the robot keeps off the instrument ---------------------------------------------------------------------------------
IND_X, IND_YR = (0.255, 0.420), (-0.045, 0.035)
ARM_KEEP_OUT = [{"name": "dial indicator, stem and stand arm", "x": IND_X, "y": IND_YR, "z_top": TOP + 0.33}]

# ---- the scripted demo ------------------------------------------------------------------------------------------------
GRIP_Y = -0.105  # on the 50 mm of shaft that overhangs the near V-block
P0, P1 = math.radians(2.0), math.radians(84.0)  # each stroke swings the hand 75 degrees round the shaft axis
GAP_FREE, GAP_CLOSED, GAP_WIDE = 0.040, 0.014, 0.090
STROKE_S, RETURN_S = 9.0, 5.0  # seconds for one gripped swing and for the open swing back
TURN_GOAL_DEG, MAX_STROKES = 364.0, 7
PA = math.radians(30.0)  # the hand comes down leaning 30 degrees forward, in the middle of the wrist's range
PUSH_Y = 0.0015  # the gripped hand leans this far toward the end stop, so the shaft cannot walk away from it

demo = placeholder_demo


def _rot(pitch: float) -> np.ndarray:
    """Gripper over the shaft, pads closing across it; pitch swings it round the shaft axis (y)."""
    return grip_rot(yaw=0.0, pitch=float(pitch), roll=-math.pi / 2)


def demo_script(ctx):
    """Right arm only. Reads the shaft's pose every step (to stay on its axis, to keep it seated and to count its
    own turn) and the plunger joint once per stroke; it never moves or edits a body or joint."""
    A = ctx.right
    dt = ctx.dt

    def axis_point():
        s = ctx.obj("shaft")
        p = s.to_world((0.0, GRIP_Y - SHAFT_MID_Y, 0.0))
        rot = s.rotation()
        return np.array([p[0], GRIP_Y, p[2]], dtype=float), math.atan2(-float(rot[2, 0]), float(rot[0, 0])), \
            STOP_Y0 - float(s.to_world((0.0, SHAFT_LEN / 2, 0.0))[1])

    rest, theta, _ = axis_point()
    off = np.zeros(3)
    table: dict[int, np.ndarray] = {}
    turned = 0.0

    def key(pitch):
        return int(round(math.degrees(pitch)))

    def free_step(target, pitch, gap, learn=True):
        """Hand open: put the encoder-measured tool point on the target by trimming the command."""
        nonlocal off
        if learn:
            err = np.asarray(target, dtype=float) - A.measured_pos
            n = float(np.linalg.norm(err))
            off = np.clip(table.get(key(pitch), off) + 0.25 * (err if n < 0.004 else err * 0.004 / n), -0.03, 0.03)
            table[key(pitch)] = off.copy()
        A._set_cartesian(np.asarray(target, dtype=float) + off, _rot(pitch), -gap / 2.0)

    def smooth(a, b, i, n):
        s = min(max(i / n, 0.0), 1.0)
        return a + (b - a) * s * s * (3.0 - 2.0 * s)

    # 1) approach from above, hand wide open, then down onto the axis and narrow to 36 mm
    above = rest + np.array([0.0, 0.0, 0.07])
    yield from A.move(above, 2.0, -GAP_WIDE / 2.0, rot=_rot(PA))
    for i in range(1, 11):
        free_step(above, PA, GAP_WIDE)
        yield
    n = round(2.0 / dt)
    for i in range(1, n + 1):
        free_step(above + (rest - above) * smooth(0.0, 1.0, i, n), PA, GAP_WIDE)
        yield
    n = round(1.0 / dt)
    for i in range(1, n + 1):
        free_step(axis_point()[0], PA, smooth(GAP_WIDE, GAP_FREE, i, n))
        yield
    # 2) one open pass over the stroke: learn the command trim that keeps the tool point on the axis at every angle
    for a, b, sec in ((PA, P1, 4.0), (P1, P0, 5.0)):
        n = round(sec / dt)
        for i in range(1, n + 1):
            free_step(axis_point()[0], smooth(a, b, i, n), GAP_FREE)
            yield
    # 3) strokes: close gently, swing 75 degrees round the axis, open, swing back
    strokes = 0
    while strokes < MAX_STROKES and math.degrees(abs(turned)) < TURN_GOAL_DEG:
        strokes += 1
        for _ in range(round(0.4 / dt)):
            free_step(axis_point()[0], P0, GAP_FREE)
            yield
        if not (PLUNGER_LOW + ON_MARGIN <= ctx.obj("dial_indicator").joints.get("plunger", -0.0005) <= PLUNGER_HIGH - ON_MARGIN):
            break  # the plunger is not on the journal: stop rather than turn for nothing
        n = round(1.0 / dt)
        for i in range(1, n + 1):  # slow close so neither pad lands hard; the trim freezes as the pads arrive
            g = smooth(GAP_FREE, GAP_CLOSED, i, n)
            free_step(axis_point()[0], P0, g, learn=g > 2 * SHAFT_R + 0.006)
            yield
        yield from A.hold(0.4)
        if not A.holding(0.015):  # the pads did not stop on the 20 mm shaft: open and try again
            yield from A.grip(-GAP_FREE / 2.0, 0.6)
            continue
        # Gripped: the pads centre the shaft across the hand, and it may sit a little along the pads. Swing the
        # encoder-measured tool point on the circle about the shaft's resting axis that keeps that offset, so the
        # hand turns the shaft without carrying it anywhere.
        along = float((A.measured_pos - rest) @ _rot(P0)[:, 2])
        base, trim, push = off.copy(), np.zeros(3), 0.0
        t0 = table.get(key(P0), off)
        n = round(STROKE_S / dt)
        for i in range(1, n + 1):
            pitch = smooth(P0, P1, i, n)
            now, th, gap = axis_point()
            turned += (th - theta + math.pi) % (2.0 * math.pi) - math.pi
            theta = th
            target = rest + along * _rot(pitch)[:, 2]
            err = target - A.measured_pos
            err[1] = 0.0
            trim = np.clip(trim + 0.6 * np.clip(err, -0.003, 0.003), -0.008, 0.008)
            push = float(np.clip(push + 0.2 * (gap - 0.0002), 0.0, 0.003))  # lean on the end stop if a gap opens
            cmd = target + base + (table.get(key(pitch), t0) - t0) + trim
            cmd[1] = rest[1] + base[1] + PUSH_Y + push
            A._set_cartesian(cmd, _rot(pitch), -GAP_CLOSED / 2.0)
            yield
        yield from A.hold(0.3)
        n = round(1.0 / dt)
        for i in range(1, n + 1):
            A._set_cartesian(A.pos.copy(), _rot(P1), -smooth(GAP_CLOSED, GAP_FREE, i, n) / 2.0)
            yield
        now, th, _ = axis_point()
        turned += (th - theta + math.pi) % (2.0 * math.pi) - math.pi
        theta = th
        off = table.get(key(P1), off).copy()
        if math.degrees(abs(turned)) >= TURN_GOAL_DEG:
            break
        n = round(RETURN_S / dt)
        for i in range(1, n + 1):
            free_step(axis_point()[0], smooth(P1, P0, i, n), GAP_FREE)
            yield
    # 4) let go: with the hand leaning forward (where a stroke ends), open, draw it back along its own axis, park
    if abs(float(np.arccos(np.clip(A.rot[:, 2] @ _rot(P1)[:, 2], -1.0, 1.0)))) > 0.05:
        n = round(RETURN_S / dt)
        for i in range(1, n + 1):
            free_step(axis_point()[0], smooth(P0, P1, i, n), GAP_FREE)
            yield
    yield from A.grip(-0.050 / 2.0, 0.6)
    back = A.pos - 0.12 * A.rot[:, 2]
    yield from A.move(back, 1.5, rot=A.rot.copy())
    yield from A.home(3.0)
    yield from ctx.wait(1.5)
