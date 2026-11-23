"""On the station table: a steel equipment access box (200 x 210 x 120 mm) whose lid, hinged along its left top edge,
stands open at 80 degrees on one gas strut in front of the box. The strut's chrome rod has an oil film at the seal (a
worn strut). A technician manikin behind the box rests a forearm on the back rim under the open lid, hand on a pipe
union inside. Right of the box: a lift support clamp (aluminium C body, brass knob, red flag) standing on the table
and a 10 mm combination wrench lying across a two-bar rest. Inside the box, along its right wall, a steel parts tray.

Requires -E object_jitter=0: the strut, clamp and wrench positions are planned from the fixed layout.

Run it:  inspect-robots run --embodiment yam_arena -E task=ext_tornado_safe_room_shelter_latches_33.py -E object_jitter=0 \\
           --instruction "Put the wrench in the parts tray inside the hatch for the technician." --scorer task_score ...
"""

import dataclasses
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402

import mechkit  # noqa: E402
from mechkit import Mech, ball, box, cyl, hull, lathe, on_reset, rod, screw, screw_states, tube  # noqa: E402
from closedloop import grip_rot, placeholder_demo  # noqa: E402
from yam_arena.constants import FINGER_OPEN  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"
SUCCESS_HOLD_S = 8.0

GREEN = (0.20, 0.32, 0.26)  # painted steel enclosure
GREEN_IN = (0.55, 0.62, 0.58)  # lid underside, primer
BLACK = (0.06, 0.06, 0.07)
CHROME = (0.86, 0.87, 0.90)
OIL = (0.30, 0.20, 0.06)
STEEL = (0.55, 0.56, 0.58)
ALU = (0.78, 0.80, 0.83)
BRASS = (0.80, 0.62, 0.18)
RED = (0.88, 0.08, 0.06)
WOOD = (0.45, 0.30, 0.16)
VEST = (1.00, 0.45, 0.05)
SKIN = (0.80, 0.62, 0.48)
HAT = (0.98, 0.82, 0.05)
DENIM = (0.15, 0.22, 0.42)
COPPER = (0.72, 0.42, 0.22)

# ---- the access box: walls 3 mm steel, x 0.20..0.40, y -0.17..+0.04, 120 mm tall --------------------------------------
BX, BY, BH, WALL = (0.24, 0.44), (-0.14, 0.07), 0.12, 0.003
HINGE_Y, LID_DEG = BY[1], 80.0
LID_W, LID_D, LID_T = 0.214, 0.204, 0.003
th = math.radians(LID_DEG)
LID_DIR = np.array([0.0, -math.cos(th), math.sin(th)])  # from the hinge line across the lid
LID_N = np.array([0.0, math.sin(th), math.cos(th)])  # lid outer face normal
_q = (math.cos(-th / 2), math.sin(-th / 2), 0.0, 0.0)  # tilt about x by -55 deg (w, x, y, z)

# ---- the gas strut: 200 mm eye to eye, 22 mm body, 10 mm chrome rod, on ball studs 50 mm off the front face
STRUT_X = 0.17
ALPHA = math.radians(30.0)  # strut rises toward +y at 30 degrees
U = np.array([0.0, math.cos(ALPHA), math.sin(ALPHA)])  # lower eye -> upper eye
UPPER_EYE = np.array([STRUT_X, HINGE_Y, BH]) + 0.04 * LID_DIR
LOWER_EYE = UPPER_EYE - 0.200 * U
BODY_LEN, ROD_R, BODY_R = 0.115, 0.005, 0.011  # body from the lower eye to its end seal (the collar)
ROD_END = 0.190


def strut_point(s: float) -> np.ndarray:
    """Point on the strut axis s metres from the lower eye centre (local = world xy, z above the table)."""
    return LOWER_EYE + s * U


TRAY_X, TRAY_Y, TRAY_FLOOR = (0.255, 0.425), (-0.135, -0.080), 0.095  # inner floor height above the table

HATCH_SHAPES = [
    box(((BX[0] + BX[1]) / 2, (BY[0] + BY[1]) / 2, 0.0015), (BX[1] - BX[0], BY[1] - BY[0], 0.003), GREEN),
    box((BX[0] + WALL / 2, (BY[0] + BY[1]) / 2, BH / 2), (WALL, BY[1] - BY[0], BH), GREEN),
    box((BX[1] - WALL / 2, (BY[0] + BY[1]) / 2, BH / 2), (WALL, BY[1] - BY[0], BH), GREEN),
    box(((BX[0] + BX[1]) / 2, BY[0] + WALL / 2, BH / 2), (BX[1] - BX[0], WALL, BH), GREEN),
    box(((BX[0] + BX[1]) / 2, BY[1] - WALL / 2, BH / 2), (BX[1] - BX[0], WALL, BH), GREEN),
    # lid, primer grey underside shown by a thin look-only skin
    box(tuple(np.array([(BX[0] + BX[1]) / 2, HINGE_Y, BH]) + LID_W / 2 * LID_DIR + LID_T / 2 * LID_N),
        (LID_D, LID_W, LID_T), GREEN, tilt=_q),
    cyl(((BX[0] + BX[1]) / 2, HINGE_Y, BH), 0.006, 0.20, STEEL, axis="x"),  # piano hinge barrel
    # pipe and union the technician is working on
    cyl((0.37, 0.0, 0.05), 0.011, 0.13, COPPER, axis="y"),
    cyl((0.37, 0.0, 0.05), 0.016, 0.022, STEEL, axis="y", sides=6),
    # parts tray hung inside the right wall: 1.5 mm steel, 20 mm deep
    box(((TRAY_X[0] + TRAY_X[1]) / 2, (TRAY_Y[0] + TRAY_Y[1]) / 2, TRAY_FLOOR - 0.001),
        (TRAY_X[1] - TRAY_X[0], TRAY_Y[1] - TRAY_Y[0], 0.002), STEEL),
    *[box((x, (TRAY_Y[0] + TRAY_Y[1]) / 2, TRAY_FLOOR + 0.009), (0.002, TRAY_Y[1] - TRAY_Y[0], 0.020), STEEL)
      for x in (TRAY_X[0] - 0.001, TRAY_X[1] + 0.001)],
    box(((TRAY_X[0] + TRAY_X[1]) / 2, TRAY_Y[1] + 0.001, TRAY_FLOOR + 0.009), (TRAY_X[1] - TRAY_X[0], 0.002, 0.020), STEEL),
    # ball studs on the front wall and on a lug welded to the lid's front edge
    rod((BX[0], LOWER_EYE[1], LOWER_EYE[2]), (STRUT_X - 0.008, LOWER_EYE[1], LOWER_EYE[2]), 0.005, STEEL),
    rod((BX[0] - 0.002, UPPER_EYE[1], UPPER_EYE[2]), (STRUT_X - 0.008, UPPER_EYE[1], UPPER_EYE[2]), 0.005, STEEL),
    box((BX[0] - 0.001, UPPER_EYE[1], UPPER_EYE[2]), (0.004, 0.030, 0.030), GREEN),
    # the strut: end sockets, black body, chrome rod with an oil film at the seal
    ball(tuple(strut_point(0.0)), 0.009, BLACK),
    rod(tuple(strut_point(0.009)), tuple(strut_point(BODY_LEN)), BODY_R, BLACK),
    rod(tuple(strut_point(BODY_LEN)), tuple(strut_point(ROD_END)), ROD_R, CHROME),
    rod(tuple(strut_point(ROD_END)), tuple(strut_point(0.200 - 0.009)), 0.006, BLACK),
    ball(tuple(strut_point(0.200)), 0.009, BLACK),
]

HATCH_SHAPES += [
    dataclasses.replace(rod(tuple(strut_point(BODY_LEN + 0.0005)), tuple(strut_point(BODY_LEN + 0.006)), ROD_R + 0.0004,
                            OIL), collide=False),
    dataclasses.replace(box(tuple(np.array([(BX[0] + BX[1]) / 2, HINGE_Y, BH]) + LID_W / 2 * LID_DIR - 0.0002 * LID_N),
                            (LID_D - 0.004, LID_W - 0.004, 0.0004), GREEN_IN, tilt=_q), collide=False),
]
HATCH = Mech("hatch", pos=(0.0, 0.0, TOP), mass=None, friction=0.45, shapes=HATCH_SHAPES)

# ---- the technician manikin behind the box, forearm on the back rim under the lid --------------------------------------
SHOULDER, ELBOW, WRIST = (0.57, -0.06, 0.24), (0.51, -0.02, 0.19), (0.415, 0.01, 0.12)
TECH = Mech("technician", pos=(0.0, 0.0, TOP), mass=None, shapes=[
    hull((0, 0, 0), [(0.540, -0.110, 0.24), (0.540, 0.090, 0.24), (0.610, -0.110, 0.27), (0.610, 0.090, 0.27),
                     (0.630, -0.100, 0.0), (0.630, 0.100, 0.0), (0.750, -0.100, 0.0), (0.750, 0.100, 0.0)], VEST),
    cyl((0.570, -0.010, 0.275), 0.035, 0.05, SKIN),
    ball((0.540, -0.010, 0.33), 0.075, SKIN),
    lathe((0.540, -0.010, 0.355), [(0.085, 0.0), (0.082, 0.012), (0.07, 0.035), (0.045, 0.058), (0.0, 0.066)], HAT),
    ball((0.472, -0.035, 0.34), 0.008, BLACK), ball((0.472, 0.015, 0.34), 0.008, BLACK),
    box((0.470, -0.010, 0.305), (0.006, 0.03, 0.006), (0.55, 0.20, 0.20)),
    rod(SHOULDER, ELBOW, 0.032, VEST),
    rod(ELBOW, WRIST, 0.022, VEST),
    box((0.375, 0.015, 0.083), (0.075, 0.045, 0.024), SKIN),  # hand round the union
])

# ---- the lift support clamp (Lisle 44870): aluminium C body, brass knob screw, red flag ----------------------------------
# Clamp frame: origin on the slot centre line; x = slot depth (mouth at +x, toward the rod), y = along the rod,
# z = screw axis.
CL_LEN, CL_BACK, CL_MOUTH, SLOT_BOTTOM, SLOT_H, JAW_T, LOW_T = 0.050, -0.012, 0.020, -0.005, 0.018, 0.012, 0.030
# The 30 mm lower jaw puts the centre of mass 12 mm below the rod and over the seat, so the clamp hangs on the rod
# like a hook (a top-heavy body rolled off the rod through its mouth in the MuJoCo replay). A 3 mm lip hangs from the
# upper jaw's mouth edge: once the clamp hangs on the rod, the rod sits behind the lip and cannot leave through the
# mouth unless the clamp is lifted (a hung clamp that only rested on its jaw rolled off the rod in the replay). Carried
# in, the rod passes between the lip and the lower jaw with 2.5 mm each side.
LIP_T, LIP_H = 0.004, 0.003
ROD_SEAT_X = 0.003  # rod centre in the clamp frame when pushed in
KNOB_R, KNOB_H, KNOB_Z0 = 0.0105, 0.012, 0.029
LEAD, SEAT = 0.00127, -0.0003  # 1/4-20 UNC brass screw, right hand; the tip is reset a quarter turn (0.3 mm) above the rod
_carrier, _knob = screw("knob", [
    cyl((0, 0, KNOB_Z0 + KNOB_H / 2), KNOB_R, KNOB_H, BRASS, sides=16),
    dataclasses.replace(cyl((0, 0, (0.009 + KNOB_Z0) / 2), 0.0032, KNOB_Z0 - 0.009, BRASS), collide=False),
], lead=LEAD, axis="z", at=(ROD_SEAT_X, 0.0, 0.0), low=-0.0025, high=0.0, seat=SEAT, hold=0.02, tighten=0.05,
    max_hold=0.15, mass=0.012)
_jaw_h = SLOT_H / 2 + LOW_T
CLAMP_START = (0.17, -0.23, TOP + _jaw_h + 0.0005)
_CLAMP_BODY = [
    box(((CL_MOUTH + CL_BACK) / 2, 0, SLOT_H / 2 + JAW_T / 2), (CL_MOUTH - CL_BACK, CL_LEN, JAW_T), ALU),
    box(((CL_MOUTH + CL_BACK) / 2, 0, -SLOT_H / 2 - LOW_T / 2), (CL_MOUTH - CL_BACK, CL_LEN, LOW_T), ALU),
    box(((SLOT_BOTTOM + CL_BACK) / 2, 0, 0), (SLOT_BOTTOM - CL_BACK, CL_LEN, SLOT_H), ALU),
    box((CL_MOUTH - LIP_T / 2, 0, SLOT_H / 2 - LIP_H / 2), (LIP_T, CL_LEN, LIP_H), ALU),  # hook lip
]
CLAMP_MASS = round(sum(mechkit.shape_volume(b) for b in _CLAMP_BODY) * 2700 + 0.004, 3)  # aluminium + 4 g flag
CLAMP = Mech("clamp", pos=CLAMP_START, mass=CLAMP_MASS, friction=0.45, shapes=[
    box(((CL_MOUTH + CL_BACK) / 2, 0, SLOT_H / 2 + JAW_T / 2), (CL_MOUTH - CL_BACK, CL_LEN, JAW_T), ALU),
    box(((CL_MOUTH + CL_BACK) / 2, 0, -SLOT_H / 2 - LOW_T / 2), (CL_MOUTH - CL_BACK, CL_LEN, LOW_T), ALU),
    box(((SLOT_BOTTOM + CL_BACK) / 2, 0, 0), (SLOT_BOTTOM - CL_BACK, CL_LEN, SLOT_H), ALU),
    box((CL_BACK - 0.0125, 0, -_jaw_h + 0.002), (0.024, 0.040, 0.004), RED),  # warning flag, low on the back
    _CLAMP_BODY[3],
], joints=[_carrier, _knob])

# ---- 10 mm combination wrench, 146 mm, on a two-bar rest -----------------------------------------------------------------
WRENCH_START = (0.33, -0.215, TOP + 0.025 + 0.003)
WRENCH = Mech("wrench", pos=WRENCH_START, mass=0.058, shapes=[
    box((0, 0, 0), (0.100, 0.010, 0.004), CHROME),
    tube((0.062, 0, 0), 0.0060, 0.0110, 0.006, CHROME, sides=12),
    box((-0.0625, 0.0075, 0), (0.021, 0.005, 0.005), CHROME),
    box((-0.0625, -0.0075, 0), (0.021, 0.005, 0.005), CHROME),
    box((-0.052, 0, 0), (0.008, 0.020, 0.005), CHROME),
])
REST = [Box(f"tool_rest_{i}", pos=(x, -0.215, TOP + 0.0125), size=(0.016, 0.05, 0.025), color=WOOD)
        for i, x in enumerate((0.29, 0.37))]

OBJECTS = [HATCH, TECH, CLAMP, WRENCH, *REST]

# ---- scoring -------------------------------------------------------------------------------------------------------------
COLLAR_TOL = 0.005  # clamp's lower end within 5 mm of the body's end seal
TIGHT_TURNS = 0.5  # knob turned at least half a turn past contact
_LATCH = {"violation": False, "tight_off_rod": False, "entered_locked": False, "seated_loose": False}


@on_reset
def _reset(env=None):
    _LATCH.update(violation=False, tight_off_rod=False, entered_locked=False, seated_loose=False)


def clamp_seated(c) -> bool:
    """The rod lies in the clamp's slot at both of the clamp's ends (rod centre at least 6 mm inside the mouth, so the
    whole rod is in the slot, and within the slot height), the clamp is lined up with the rod within 15 degrees, and
    its lower end is within 5 mm of the seal."""
    r = c.rotation()
    if abs(float(r[:, 1] @ U)) < math.cos(math.radians(15)):
        return False
    for end in (-1.0, 1.0):
        q = c.to_world((ROD_SEAT_X, end * CL_LEN / 2, 0.0)) - np.array([0, 0, TOP])
        s = float((q - LOWER_EYE) @ U)
        p = c.to_local(strut_point(s) + np.array([0, 0, TOP]))
        if not (SLOT_BOTTOM <= p[0] <= CL_MOUTH - 0.006 and abs(p[2]) <= SLOT_H / 2 + 0.001):
            return False
    lo_end = c.to_world((ROD_SEAT_X, -CL_LEN / 2 * np.sign(r[:, 1] @ U), 0.0)) - np.array([0, 0, TOP])
    s_end = float((lo_end - LOWER_EYE) @ U)
    return BODY_LEN - 0.002 <= s_end <= BODY_LEN + COLLAR_TOL


def knob_turns_past(c) -> float:
    st = c.screws.get("knob") if getattr(c, "screws", None) else None
    if st is None:
        joints = getattr(c, "joints", None) or {}
        if "knob" not in joints or "knob_travel" not in joints:
            return 0.0
        st = screw_states(CLAMP, c).get("knob")
    return float(st.turns_past) if st is not None and st.seated else 0.0


def locked(objects) -> bool:
    c = objects["clamp"]
    return clamp_seated(c) and knob_turns_past(c) >= TIGHT_TURNS and not _LATCH["tight_off_rod"]


def wrench_over_hatch(w) -> bool:
    """Any part of the wrench over the box opening, at any height."""
    return bool(w.high[0] > BX[0] + WALL and w.low[0] < BX[1] - WALL and w.high[1] > BY[0] + WALL
                and w.low[1] < BY[1] - WALL)


def wrench_in_tray(w) -> bool:
    """Lying in the tray: footprint inside the tray, underside within 1 cm of its floor (not held above it)."""
    return bool(TRAY_X[0] - 0.002 <= w.low[0] and w.high[0] <= TRAY_X[1] + 0.002 and TRAY_Y[0] - 0.002 <= w.low[1]
                and w.high[1] <= TRAY_Y[1] + 0.002 and TOP + TRAY_FLOOR - 0.003 <= w.low[2] <= TOP + TRAY_FLOOR + 0.010)


def update_latches(objects) -> None:
    """Scorer-side record, every control step: was the knob tightened before the clamp was on the rod, and did the
    wrench cross over the opening while the strut was not locked."""
    c = objects["clamp"]
    past, on_rod = knob_turns_past(c), clamp_seated(c)
    if on_rod:
        _LATCH["seated_loose"] = True  # the clamp has been on the rod
    elif past >= 0.05 and not _LATCH["seated_loose"]:
        _LATCH["tight_off_rod"] = True  # the screw went past its seat before the clamp was ever on the rod
    if wrench_over_hatch(objects["wrench"]):
        if locked(objects):
            _LATCH["entered_locked"] = True
        elif not _LATCH["entered_locked"]:
            _LATCH["violation"] = True


def score(objects):
    """0 if the wrench went over the hatch opening before the strut was locked. Otherwise 0.25 for the clamp seated on the
    rod against the seal, 0.25 more for the knob turned at least 0.5 turn past contact with the clamp seated, and 0.5
    more for the wrench lying in the tray while the strut stays locked. The simulator calls score() after every
    control step, so the latches see the whole run."""
    update_latches(objects)
    if _LATCH["violation"]:
        return 0.0
    w = objects["wrench"]
    if wrench_over_hatch(w) and not locked(objects) and not _LATCH["entered_locked"]:
        return 0.0
    c = objects["clamp"]
    s = 0.0
    if clamp_seated(c):
        s += 0.25
        if locked(objects):
            s += 0.25
            if wrench_in_tray(w):
                s += 0.5
    return s


def success(objects):
    return score(objects) == 1.0 and objects["wrench"].resting() and objects["clamp"].resting()


# ---- the robot keeps out of the technician (4 cm margin) and out of the box interior beside the tray ----------------
ARM_KEEP_OUT = [
    {"name": "technician hand", "x": (0.30, 0.46), "y": (-0.05, 0.08), "z_top": TOP + 0.25},
    {"name": "technician forearm and elbow", "x": (0.42, 0.58), "y": (-0.10, 0.06), "z_top": TOP + 0.25},
    {"name": "technician head and torso", "x": (0.42, 0.79), "y": (-0.15, 0.14), "z_top": TOP + 0.25},
]

# ---- scripted demo (right arm only; the left arm holds READY) ------------------------------------------------------------
demo = placeholder_demo
S_CLAMP = BODY_LEN + 0.013 + CL_LEN / 2  # clamp centre along the strut at entry: lower end 13 mm above the seal,
# so the finger below it is over the bare rod; released, the clamp slides down onto the seal (friction 0.45 < tan 30)
GRIP_IN_CLAMP = np.array([0.004, 0.0, 0.022])  # fingertips stay above the rod  # tool point in the clamp frame while carrying it
R_TILT = grip_rot(math.pi / 2, ALPHA, math.pi / 2) @ grip_rot().T  # clamp frame on the strut (about x by +30 deg)
CARRY = dict(yaw=math.pi / 2, pitch=ALPHA, roll=math.pi / 2)
GAP_OPEN_SMALL = -0.015  # finger joint for a 30 mm pad gap
# Grips close to a set opening a few mm under the part, not to FINGER_CLOSED: a pinch whose fingers both sit at the
# 50 N effort cap has no stiffness along the pad line, and the tilted clamp slid 16 mm in the MuJoCo replay.
SQUEEZE_CLAMP = -0.022  # 44 mm opening on the 50 mm clamp ends, about 15 N per finger
# Open only 34 mm round the 21 mm knob so both fingers stay over the 32 mm upper jaw: opened wider, the front finger
# hung past the jaw's front edge and pulled the clamp off the rod on the way up (MuJoCo replay).
GAP_KNOB = -0.017
SQUEEZE_KNOB = -0.0065  # 13 mm opening on the 21 mm knob, about 20 N per finger
SQUEEZE_WRENCH = -0.0025  # 5 mm opening on the 10 mm handle


def _seat_origin() -> np.ndarray:
    return strut_point(S_CLAMP) + np.array([0.0, 0.0, TOP]) - R_TILT @ np.array([ROD_SEAT_X, 0.0, 0.0])


def demo_script(ctx):
    R = ctx.right
    up = np.array([0.0, 0.0, 1.0])
    # 1. pick the clamp off the table by its two end faces
    c0 = np.array(CLAMP_START) + GRIP_IN_CLAMP
    yield from R.move(c0 + 0.08 * up, 2.0, FINGER_OPEN)
    yield from R.move(c0, 1.2, FINGER_OPEN)
    yield from R.grip(SQUEEZE_CLAMP, 0.8)
    yield from R.hold(0.3)
    yield from R.move(c0 + 0.10 * up, 1.0)
    # 2. tilt to the strut's angle in front of the rod, slide the slot sideways onto the rod against the seal
    seat = _seat_origin()
    pre = seat - np.array([0.040, 0.0, 0.0])
    yield from R.move(pre + R_TILT @ GRIP_IN_CLAMP + 0.05 * up, 2.0, **CARRY)
    lift = R_TILT @ np.array([0.0, 0.0, 0.0015])  # aim the rod 1.5 mm below the slot centre line (in-hand droop)
    yield from R.move(pre + R_TILT @ GRIP_IN_CLAMP + lift, 1.0, **CARRY)
    yield from R.trim_to_measured(pre + R_TILT @ GRIP_IN_CLAMP + lift)  # cancel wrist sag under the clamp
    trim = R.pos - (pre + R_TILT @ GRIP_IN_CLAMP + lift)
    yield from R.move(seat + R_TILT @ GRIP_IN_CLAMP + lift + trim, 2.0, **CARRY)
    yield from R.trim_to_measured(seat + R_TILT @ GRIP_IN_CLAMP + lift)  # push the rod fully into the slot
    yield from R.hold(0.3)
    yield from R.move(R.measured_pos.copy(), 0.6, **CARRY)  # unload: command where the hand is, so it does not lunge
    for k in range(8):
        yield from R.grip(SQUEEZE_CLAMP + (FINGER_OPEN * 0.75 - SQUEEZE_CLAMP) * (k + 1) / 8, 0.1)
    approach = R_TILT @ np.array([0.0, 0.0, 1.0])
    yield from R.move(R.pos + 0.04 * approach, 0.8, **CARRY)
    yield from R.hold(0.5)
    # 3. turn the knob down onto the rod: two strokes of 180 degrees, read the clamp's pose before each. Trim the
    # measured tool point onto the knob axis before closing: a pinch closed 7 mm off centre dragged the knob, and the
    # clamp with it, 10 mm toward the slot mouth in the MuJoCo replay.
    for _ in range(2):
        c = ctx.obj("clamp")
        rc = c.rotation()
        knob = c.to_world((ROD_SEAT_X, 0.0, KNOB_Z0 + 0.008))  # tips 3 mm above the jaw
        ax = rc[:, 2]
        base = _grip_about(rc[:, 2])
        rot_a = base @ _rz(-2.5)
        rot_b = base @ _rz(0.64)
        yield from R.move(knob + 0.035 * ax, 1.2, GAP_KNOB, rot=rot_a)
        yield from R.trim_to_measured(knob + 0.035 * ax)
        yield from R.move(knob + (R.pos - knob - 0.035 * ax), 0.8, rot=rot_a)
        yield from R.trim_to_measured(knob)
        trim = R.pos - knob
        yield from R.grip(SQUEEZE_KNOB, 0.6)
        for k in range(4):  # the stroke is wrist roll (joint 6) only: -1.5 to +1.6 rad, inside its 2.09 rad stop
            yield from R.move(knob + trim + 0.003 * rc[:, 0], 0.6, rot=base @ _rz(-2.5 + (0.64 + 2.5) * (k + 1) / 4))
        # unload the lean and the wound-up wrist roll (command where the hand is), then open slowly: a quick open from
        # a loaded pinch past the seat kicked the clamp 10 mm toward its mouth in the MuJoCo replay
        held_rot = R.measured_rot.copy()
        yield from R.move(R.measured_pos.copy(), 0.5, rot=held_rot)
        for k in range(6):
            yield from R.grip(SQUEEZE_KNOB + (GAP_KNOB - SQUEEZE_KNOB) * (k + 1) / 6, 0.15)
        # rise along the knob axis as it is now: rising along a stale axis dragged the open fingers across the knob
        ax = ctx.obj("clamp").rotation()[:, 2]
        yield from R.move(R.pos + 0.035 * ax, 0.8, rot=held_rot)
    yield from R.move(R.pos + 0.05 * ax, 0.8, GAP_KNOB, rot=held_rot)  # straight up the knob axis, clear of the clamp
    # 4. pick the wrench from its rest and lay it in the tray inside the hatch
    w0 = np.array(WRENCH_START) + np.array([0.0, 0.0, 0.008])
    yield from R.move((0.24, -0.20, TOP + 0.17), 1.6, GAP_OPEN_SMALL)
    yield from R.move(w0 + 0.06 * up, 1.4, GAP_OPEN_SMALL)
    yield from R.move(w0, 1.0, GAP_OPEN_SMALL)
    yield from R.grip(SQUEEZE_WRENCH, 0.6)
    yield from R.hold(0.3)
    yield from R.move(w0 + 0.06 * up, 0.8)
    place = np.array([(TRAY_X[0] + TRAY_X[1]) / 2, (TRAY_Y[0] + TRAY_Y[1]) / 2, TOP + TRAY_FLOOR + 0.015])
    yield from R.move((place[0], -0.20, TOP + 0.17), 1.4)
    yield from R.move(place + np.array([0.0, 0.0, 0.06]), 1.0)
    yield from R.move(place, 1.0)
    yield from R.grip(GAP_OPEN_SMALL, 0.6)
    yield from R.move(place + np.array([0.0, -0.03, 0.07]), 1.2)
    yield from R.home(3.0)
    yield from R.hold(1.0)


def _grip_about(axis: np.ndarray) -> np.ndarray:
    """Grip orientation whose approach is down the live knob axis, with its roll reference kept as close as possible to
    the nominal clamp's, so a clamp that has rolled on the rod does not send the wrist to another IK branch."""
    nom = R_TILT @ grip_rot()
    z = -np.asarray(axis, float) / np.linalg.norm(axis)
    x = nom[:, 0] - (nom[:, 0] @ z) * z
    x /= np.linalg.norm(x)
    return np.column_stack([x, np.cross(z, x), z])


def _rz(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])

