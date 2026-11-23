"""On the station table: the drive-roller bay of a small roller conveyor deck, two dark anodised side rails 236 mm
apart (inside faces at y +-0.118) running along x from 0.215 to 0.385, each with an open-top drop-in slot at x 0.30
(13 mm wide, a V seat at the bottom for an 11 mm hex shaft). In front of the bay, on two grey V blocks at x 0.165, lies
a 24 V motorized drive roller (38 mm steel tube, 225 mm long, 11 mm hex stub at each end, 0.70 kg) along y, its hex
stubs flat-up. Its black 6 mm power cable leaves the left (+y) stub along the axis and runs in a short loop to a
connector anchored at the driver card on the left rail's outer face. On the left rail's outer face, a guide pocket at
x 0.30 waits for the stainless anti-rotation torque plate (34 x 70 x 4 mm, hex notch at the bottom), which stands in a
slotted rack at (0.355, 0.215).

Requires -E object_jitter=0: fixed layout.

Run it:
    inspect-robots run --embodiment yam_arena -E task=ext_robot_amr_payload_hitch_conveyor_modules_23.py \\
        -E object_jitter=0 --instruction "Seat the roller's hex ends point-up in the frame slots without pulling
        its cable, then drop the torque plate over the hex." --policy yam_scripted --scorer task_score
"""

from __future__ import annotations

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402

import belimits  # noqa: E402, F401
import closedloop  # noqa: E402, F401
import mechkit  # noqa: E402
import ropekit  # noqa: E402
from closedloop import grip_rot, placeholder_demo  # noqa: E402
from mechkit import Mech, box, cyl, extrude, lathe  # noqa: E402
from ropekit import EndBody, Rope, fillet  # noqa: E402

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402
from yam_arena.urdf import axis_angle_matrix  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 8.0
SCORE_OVER = "end"

# Offline testing override for the hands-clear check (stays None in live simulator runs).
FORCE_RELEASED: list = [None]

STEEL = (0.74, 0.76, 0.80)  # zinc-plated roller tube and shafts
STAINLESS = (0.90, 0.91, 0.93)  # SUS304 torque plate
FRAME = (0.16, 0.20, 0.30)  # dark blue anodised aluminium rails and guide
STAGE = (0.42, 0.43, 0.46)  # grey printed V blocks and plate rack
CABLE = (0.06, 0.06, 0.07)
CARD = (0.10, 0.45, 0.20)  # green driver card housing

# ---- conveyor frame (fixed) ---------------------------------------------------------------------------------------
FX = 0.30  # slot centre x
RAIL_IN, RAIL_T = 0.118, 0.006  # inside face |y| and rail thickness: between-frame width 236 mm
RAIL_OUT = RAIL_IN + RAIL_T
RAIL_X0, RAIL_X1 = 0.215, 0.385
RAIL_H = 0.075
SEAT_H = 0.050  # hex axis height above the table when seated
SEAT_Z = TOP + SEAT_H
HEX_AF = 0.011  # hex across flats
HEX_R = HEX_AF / math.sqrt(3.0)  # across corners / 2 = 6.35 mm
SLOT_HALF = 0.0065  # 13 mm slot: 1 mm per side on the hex flats
LEAD_HALF, LEAD_D = 0.011, 0.005  # entry chamfer at the rail top
TAN30 = math.tan(math.radians(30.0))

# ---- roller (rope end body) ---------------------------------------------------------------------------------------
TUBE_R, TUBE_L = 0.019, 0.225  # 38 mm tube; 236 mm between frames less the 11 mm (5.5 mm a side) end gap
STUB_L = 0.0275  # each hex stub, tube end to shaft end
ROLLER_L = TUBE_L + 2 * STUB_L  # 0.280 overall; shaft ends at y +-0.140 when centred
ROLLER_MASS = 0.70
ROLLER_HALF = ROLLER_L / 2

# ---- cradle and staging -------------------------------------------------------------------------------------------
X0 = 0.165  # roller axis x on the V blocks
V_APEX = 0.012  # V bottom above the table
Z0 = TOP + V_APEX + TUBE_R * math.sqrt(2.0) + 0.0003  # roller axis on two 90 deg V blocks, 0.3 mm above contact
VBLOCK_Y = 0.075
TIP0 = (X0, ROLLER_HALF, Z0)  # left shaft end, where the cable leaves the roller

# ---- torque plate ---------------------------------------------------------------------------------------------------
PL_T, PL_W = 0.004, 0.034
PL_BOT, PL_TOP = -0.010, 0.060  # plate edges relative to its origin (the hex axis point of its notch)
PL_LEAD_HALF, PL_LEAD_D = 0.009, 0.005  # notch mouth flared to 18 mm over its lowest 5 mm: a lead-in onto the hex
PL_GRIP = 0.050  # TCP height in the plate frame (10 mm below its top)
PL_MASS = 0.072  # SUS304, 7.93 g/cm3 x (34 x 70 - notch) x 4 mm, checked in check.py
Y_PLATE = RAIL_OUT + 0.003 + PL_T / 2  # pocket 10 mm deep for a 4 mm plate: 3 mm a side
POCKET_Y1 = RAIL_OUT + 0.010
LEDGE_TOP = TOP + 0.028  # guide floor: 12 mm under a seated plate's bottom
GUIDE_TOP = TOP + 0.070
RACK = (0.355, 0.215)
RACK_FLOOR = 0.004
PLATE0 = (RACK[0], RACK[1], TOP + RACK_FLOOR - PL_BOT)  # plate origin standing in the rack

# ---- cable -----------------------------------------------------------------------------------------------------------
CABLE_R = 0.003
ANCHOR = (0.235, 0.170)
ZC = TOP + CABLE_R + 0.0025  # cable axis lying on the table (the start connector is 10 mm tall)


def _ry(a: float) -> np.ndarray:
    return axis_angle_matrix(np.array([0.0, 1.0, 0.0]), float(a))


def _rail(side: float) -> mechkit.Shape:
    """A side rail with its drop-in hex slot: an x-z outline extruded along y (outline points are (z, x))."""
    a = SEAT_H - HEX_R  # V apex: the hex's lower corner when seated
    v = SEAT_H - HEX_R + SLOT_HALF * TAN30  # V meets the slot walls
    hx = (RAIL_X1 - RAIL_X0) / 2
    xz = [(-hx, 0.0), (hx, 0.0), (hx, RAIL_H), (LEAD_HALF, RAIL_H), (SLOT_HALF, RAIL_H - LEAD_D), (SLOT_HALF, v),
          (0.0, a), (-SLOT_HALF, v), (-SLOT_HALF, RAIL_H - LEAD_D), (-LEAD_HALF, RAIL_H), (-hx, RAIL_H)]
    cx = (RAIL_X0 + RAIL_X1) / 2
    return extrude((cx, side * (RAIL_IN + RAIL_T / 2), 0.0), [(z, x + cx - FX) for x, z in xz], RAIL_T, FRAME,
                   axis="y")


def _vblock(y: float) -> mechkit.Shape:
    h, hx = 0.028, 0.030
    top_half = h - V_APEX
    xz = [(-hx, 0.0), (hx, 0.0), (hx, h), (top_half, h), (0.0, V_APEX), (-top_half, h), (-hx, h)]
    return extrude((X0, y, 0.0), [(z, x) for x, z in xz], 0.016, STAGE, axis="y")


def _frame_shapes() -> list:
    s = [_rail(+1.0), _rail(-1.0)]
    # Cross ties front and back, below the roller
    for x in (RAIL_X0 + 0.006, RAIL_X1 - 0.006):
        s.append(box((x, 0.0, 0.010), (0.012, 2 * RAIL_IN, 0.020), FRAME))
    # Torque plate guide on the left rail's outer face: two posts with front lips, and a floor ledge
    gy0, gy1 = RAIL_OUT, POCKET_Y1 + 0.004
    zc, zh = (LEDGE_TOP - TOP + GUIDE_TOP - TOP) / 2, GUIDE_TOP - LEDGE_TOP
    for sx in (-1.0, 1.0):
        s.append(box((FX + sx * 0.024, (gy0 + gy1) / 2, zc), (0.008, gy1 - gy0, zh), FRAME))
        s.append(box((FX + sx * 0.017, POCKET_Y1 + 0.002, zc), (0.006, 0.004, zh), FRAME))
    s.append(box((FX, (gy0 + gy1) / 2, (LEDGE_TOP - TOP) - 0.004), (0.056, gy1 - gy0, 0.008), FRAME))
    return s


def _staging_shapes() -> list:
    s = [_vblock(+VBLOCK_Y), _vblock(-VBLOCK_Y)]
    rx, ry = RACK
    s.append(box((rx, ry, RACK_FLOOR / 2), (0.052, 0.030, RACK_FLOOR), STAGE))
    for sy in (-1.0, 1.0):  # side walls 1 mm off each plate face, 30 mm tall
        s.append(box((rx, ry + sy * (PL_T / 2 + 0.001 + 0.003), 0.015), (0.052, 0.006, 0.030), STAGE))
    for sx in (-1.0, 1.0):  # end walls 3 mm off each plate edge
        s.append(box((rx + sx * (PL_W / 2 + 0.003 + 0.003), ry, 0.015), (0.006, 2 * (PL_T / 2 + 0.001), 0.030), STAGE))
    return s


def _cable_path(end_y: float = TIP0[1]):
    ax, ay = ANCHOR
    pts = [(ax, ay, ZC), (ax, 0.215, ZC), (X0, 0.215, ZC), (X0, 0.166, Z0), (X0, end_y, Z0)]
    return fillet(pts, 0.02)


def _roller_body() -> EndBody:
    """In the end frame: origin at the left shaft end, +x along the roller (world -y), z up. The hex prisms put a
    corner along the frame's y (world x), so the stubs start flat-up, 30 deg from point-up."""
    shapes = [
        cyl((STUB_L / 2, 0.0, 0.0), HEX_R, STUB_L, STEEL, axis="x", sides=6),
        lathe((STUB_L, 0.0, 0.0), [(0.016, 0.0), (TUBE_R, 0.003), (TUBE_R, TUBE_L - 0.003), (0.016, TUBE_L)], STEEL, axis="x"),
        cyl((ROLLER_L - STUB_L / 2, 0.0, 0.0), HEX_R, STUB_L, STEEL, axis="x", sides=6),
    ]
    return EndBody(shapes, mass=ROLLER_MASS, friction=1.0)


def _connector() -> EndBody:
    return EndBody((box((0.011, 0.0, 0.0), (0.022, 0.014, 0.010), CABLE),), mass=0.012)


def _build_cable() -> Rope:
    """The equal-chord segments cut the path's corners, so the bare tip overshoots the path end: move the path end
    until the tip (where the roller's left shaft end is welded) sits at y = +0.140, the roller centred on y = 0."""
    end_y = TIP0[1]
    for _ in range(6):
        lay = ropekit.segment_layout(_cable_path(end_y), 0.02)
        err = float(lay.knots[-1][1]) - TIP0[1]
        if abs(err) < 2e-5:
            break
        end_y -= err
    return Rope("cable", path=_cable_path(end_y), segment=0.02, radius=CABLE_R, rope_mass=0.012, bend_stiffness=0.01,
                max_bend=1.2, color=CABLE, collision_sides=8, anchor_start=True, start_body=_connector(),
                end_body=_roller_body())


def _plate_shapes() -> list:
    flank = HEX_R - SLOT_HALF * TAN30  # where the notch flanks meet its walls (1 mm off the hex side flats)
    xz = [(-PL_W / 2, PL_BOT), (-PL_LEAD_HALF, PL_BOT), (-SLOT_HALF, PL_BOT + PL_LEAD_D), (-SLOT_HALF, flank),
          (0.0, HEX_R), (SLOT_HALF, flank), (SLOT_HALF, PL_BOT + PL_LEAD_D), (PL_LEAD_HALF, PL_BOT), (PL_W / 2, PL_BOT),
          (PL_W / 2, PL_TOP), (-PL_W / 2, PL_TOP)]
    return [extrude((0.0, 0.0, 0.0), [(z, x) for x, z in xz], PL_T, STAINLESS, axis="y")]


CABLE_ROPE = _build_cable()
CABLE_LEN = CABLE_ROPE.total_length
FRAME_MECH = Mech("conveyor_frame", pos=(0.0, 0.0, TOP), mass=None, shapes=_frame_shapes())
STAGING = Mech("staging", pos=(0.0, 0.0, TOP), mass=None, shapes=_staging_shapes())
CARD_BOX = Mech("driver_card", pos=(0.0, 0.0, TOP), mass=None,
                shapes=[box((ANCHOR[0], (RAIL_OUT + 0.146) / 2, 0.016), (0.060, 0.146 - RAIL_OUT, 0.032), CARD)])
PLATE = Mech("torque_plate", pos=PLATE0, mass=PL_MASS, shapes=_plate_shapes(), friction=0.8)
OBJECTS = [FRAME_MECH, STAGING, CARD_BOX, CABLE_ROPE, PLATE]

# ---- scoring --------------------------------------------------------------------------------------------------------
SEAT_TOL_Z = 0.0015  # both stub axes within 1.5 mm of the seat height
SEAT_TOL_X = 0.002  # and within 2 mm of the slot centre line
SEAT_TOL_Y = 0.006  # the tube ends stop a seated roller 5.5 mm either side of centre: this only rejects a stub out of its slot
CLOCK_TOL_DEG = 5.0  # a hex corner within 5 deg of straight up
PLATE_TOL_Z = 0.0015
PLATE_TOL_X = 0.003
TUG_RATIO = 0.95  # cable straight-line span over 95 % of its length counts as pulled
TUG_STEPS = 5  # held for 5 control steps (0.25 s at 20 Hz)
CATCH_Z = TOP + 0.11  # any cable segment between the rails or in the plate pocket below this is caught


def _quat_matrix(q_xyzw) -> np.ndarray:
    return mechkit._quat_xyzw_matrix(np.asarray(q_xyzw, dtype=float))


def roller_pose(objects):
    """(rotation, left shaft-end point) of the roller from the cable's end pose."""
    o, q = objects["cable"].end_poses["end"]
    return _quat_matrix(q), np.asarray(o, dtype=float)


def stub_axes(objects):
    R, o = roller_pose(objects)
    a = R[:, 0]
    return o + a * (STUB_L / 2), o + a * (ROLLER_L - STUB_L / 2)


def clock_error_deg(objects) -> float:
    """Degrees from the nearest hex corner to straight up, measured about the roller's own axis (0 is point-up)."""
    R, _ = roller_pose(objects)
    a, c = R[:, 0], R[:, 1]  # roller axis and the direction of one hex corner
    u = np.array([0.0, 0.0, 1.0]) - a[2] * a
    if np.linalg.norm(u) < 1e-6:
        return 90.0
    u /= np.linalg.norm(u)
    phi = math.degrees(math.atan2(float(np.cross(u, c) @ a), float(u @ c)))
    return abs(((phi + 30.0) % 60.0) - 30.0)


def roller_seated(objects) -> bool:
    R, _ = roller_pose(objects)
    if abs(float(R[2, 0])) > math.sin(math.radians(3.0)) or abs(float(R[1, 0])) < math.cos(math.radians(3.0)):
        return False  # axis must lie along y within 3 deg
    left, right = stub_axes(objects)
    if abs((left[1] + right[1]) / 2) > SEAT_TOL_Y:
        return False  # centred between the rails
    if left[1] < right[1]:
        left, right = right, left
    ok = True
    for p, side in ((left, 1.0), (right, -1.0)):
        ok &= abs(p[0] - FX) <= SEAT_TOL_X and abs(p[2] - SEAT_Z) <= SEAT_TOL_Z
        ok &= RAIL_IN < side * p[1] < ROLLER_HALF + 0.004  # the stub lies across its rail
    return bool(ok)


def plate_seated(objects) -> bool:
    s = objects["torque_plate"]
    R = _quat_matrix(s.quat_xyzw)
    o = np.asarray(getattr(s, "origin", s.pos), dtype=float)
    upright = float(R[2, 2]) >= math.cos(math.radians(10.0))
    return bool(upright and abs(o[0] - FX) <= PLATE_TOL_X and RAIL_OUT < o[1] < POCKET_Y1
                and abs(o[2] - SEAT_Z) <= PLATE_TOL_Z)


def cable_span(objects) -> float:
    _, o = roller_pose(objects)
    s = objects["cable"]
    a, _ = s.end_poses["start"]
    return float(np.linalg.norm(o - np.asarray(a, dtype=float)))


def cable_caught(objects) -> bool:
    """A cable segment between the rails, in a slot or in the torque plate's pocket (under the plate's path)."""
    for p in np.asarray(objects["cable"].points, dtype=float):
        if RAIL_X0 - 0.01 < p[0] < RAIL_X1 + 0.01 and p[1] < POCKET_Y1 and p[2] < CATCH_Z:
            return True
    return False


_TUG = {"streak": 0, "tugged": False, "last": None}


@mechkit.on_reset
def _clear_tug(env=None):
    _TUG.update(streak=0, tugged=False, last=None)


def _track_tug(objects) -> bool:
    """Latch a pull on the cable: its straight span over 95 % of its length for 5 control steps in a row. score() and
    success() both see each step's states, so a step is counted once."""
    _, o = roller_pose(objects)
    key = (id(objects), tuple(np.round(o, 7)))
    if key != _TUG["last"]:
        _TUG["last"] = key
        if cable_span(objects) > TUG_RATIO * CABLE_LEN:
            _TUG["streak"] += 1
            if _TUG["streak"] >= TUG_STEPS:
                _TUG["tugged"] = True
        else:
            _TUG["streak"] = 0
    return _TUG["tugged"]


def hands_clear(objects) -> bool:
    if FORCE_RELEASED[0] is not None:
        return bool(FORCE_RELEASED[0])
    return bool(mechkit.released(objects["torque_plate"], clearance=0.03)
                and mechkit.released(objects["cable"], clearance=0.03))


def evaluate(objects) -> dict:
    tugged = _track_tug(objects)
    caught = cable_caught(objects)
    seated = roller_seated(objects)
    clocked = seated and clock_error_deg(objects) <= CLOCK_TOL_DEG
    plate = clocked and plate_seated(objects)
    clear = plate and hands_clear(objects)
    if tugged or caught:
        sc = 0.0
    else:
        sc = 0.25 * seated + 0.25 * clocked + 0.25 * plate + 0.25 * clear
    return {"score": sc, "tugged": tugged, "caught": caught, "seated": seated, "clocked": clocked,
            "plate": plate, "clear": clear}


def score(objects) -> float:
    """0 if the cable was ever pulled straight (span over 95 % of its length for 0.25 s) or ends caught between the
    rails, in a slot or in the plate pocket. Otherwise 0.25 roller seated in both slots, +0.25 hex point-up within
    5 deg, +0.25 torque plate seated over the hex, +0.25 both hands 3 cm clear."""
    return float(evaluate(objects)["score"])


def success(objects) -> bool:
    return score(objects) == 1.0 and objects["torque_plate"].resting() and objects["cable"].resting()


# ---- arm keep-out (checked offline by sweep.py) ---------------------------------------------------------------------
# The driver card, its connector and the cable's run beside the left rail: no arm part goes down onto them.
CABLE_KEEP_OUT = {"x": (0.195, 0.275), "y": (0.142, 0.245)}
ARM_KEEP_OUT = [
    {"name": "driver card and cable run", "x": CABLE_KEEP_OUT["x"], "y": CABLE_KEEP_OUT["y"], "z_top": TOP + 0.08},
]

# ---- scripted demo (left arm only) ----------------------------------------------------------------------------------
YAW_X = -math.pi / 2  # pads close along world x, across the roller lying along y
CLOCK_TURN = math.radians(-30.0)  # grasp turned 30 deg about the roller axis, then level: flat-up becomes point-up
CARRY_Z = TOP + 0.125  # roller axis while carried: tube bottom 3 cm over the rail tops
G_LEVEL = grip_rot(yaw=YAW_X)
G_TURNED = _ry(CLOCK_TURN) @ G_LEVEL
G_PLATE = grip_rot(yaw=YAW_X)  # pads close along x on the plate's two 34 mm-apart edges: the pinch centres it in x
GRASP_OPEN = 0.65 * FINGER_OPEN  # about 61 mm: the turned wrist keeps its lower finger 1.5 cm off the table

demo = placeholder_demo


def _trim_pose(arm, target, rot_goal, steps: int = 20, gain: float = 0.5):
    """Servo the measured tool pose (encoder forward kinematics) onto ``target`` and ``rot_goal``: cancels the wrist
    sag a 0.7 kg roller puts on the arm, including its roll about the roller axis. No-op offline."""
    target = np.asarray(target, dtype=float)
    for _ in range(steps):
        dp = target - arm.measured_pos
        R_err = rot_goal @ arm.measured_rot.T
        ang = math.acos(max(-1.0, min(1.0, (np.trace(R_err) - 1.0) / 2.0)))
        if np.linalg.norm(dp) < 0.0015 and ang < math.radians(0.7):
            break
        cmd_pos = arm.pos + gain * np.clip(dp, -0.004, 0.004)
        w = np.array([R_err[2, 1] - R_err[1, 2], R_err[0, 2] - R_err[2, 0], R_err[1, 0] - R_err[0, 1]])
        if np.linalg.norm(w) > 1e-9 and ang > 1e-6:
            dR = axis_angle_matrix(w / np.linalg.norm(w), min(gain * ang, math.radians(1.5)))
            cmd_rot = dR @ arm.rot
        else:
            cmd_rot = arm.rot
        arm._set_cartesian(cmd_pos, cmd_rot, arm.finger)
        yield


def _offset(arm, nominal):
    """Commanded minus nominal position after a trim, carried into the next moves."""
    return np.asarray(arm.pos, dtype=float) - np.asarray(nominal, dtype=float)


def _grasp_roller(L):
    grasp = np.array([X0, 0.0, Z0])
    approach = G_TURNED[:, 2]
    for attempt in range(2):
        pre = grasp - 0.09 * approach
        yield from L.move(pre, 2.0 if attempt == 0 else 1.2, GRASP_OPEN, rot=G_TURNED)
        yield from L.trim_to_measured(pre)
        off = _offset(L, pre)
        yield from L.move(grasp + off, 1.4, rot=G_TURNED)
        yield from L.grip(FINGER_CLOSED, 0.8)
        yield from L.hold(0.4)
        if L.holding(0.030):
            return True
        yield from L.grip(GRASP_OPEN, 0.5)
        yield from L.move(pre + off, 0.8, rot=G_TURNED)
    return False


def _seat_roller(L):
    # Lift clear of the V blocks, level the grip (turning the hex point-up) and carry over the rails.
    yield from L.move((X0, 0.0, Z0 + 0.04), 1.2, rot=G_TURNED)
    yield from L.move((X0 + 0.03, 0.0, CARRY_Z), 1.8, rot=G_LEVEL)
    above = np.array([FX, 0.0, CARRY_Z])
    yield from L.move(above, 2.2, rot=G_LEVEL)
    yield from _trim_pose(L, above, G_LEVEL, steps=30)
    off = _offset(L, above)
    rot = L.rot.copy()
    entry = np.array([FX, 0.0, SEAT_Z + 0.028])  # hex corners just above the rail tops
    yield from L.move(entry + off, 1.6, rot=rot)
    yield from _trim_pose(L, entry, G_LEVEL, steps=24)
    off = _offset(L, entry)
    rot = L.rot.copy()
    yield from L.move(np.array([FX, 0.0, SEAT_Z + 0.008]) + off, 1.6, rot=rot)
    yield from L.press_until(np.array([FX, 0.0, SEAT_Z + 0.008]) + off, max_depth=0.016, err_mm=4.0,
                             min_depth=0.006, speed=0.008, rot=rot)
    yield from L.grip(FINGER_OPEN, 1.2)
    yield from L.hold(0.4)
    yield from L.move(L.pos + np.array([0.0, 0.0, 0.11]), 1.6, rot=L.rot.copy())


def _place_plate(L):
    grasp = np.array([PLATE0[0], PLATE0[1], PLATE0[2] + PL_GRIP])
    for attempt in range(2):
        pre = grasp + np.array([0.0, 0.0, 0.07])
        yield from L.move(pre, 2.2 if attempt == 0 else 1.0, FINGER_OPEN, rot=G_PLATE)
        yield from L.trim_to_measured(pre)
        off = _offset(L, pre)
        yield from L.move(grasp + off, 1.2, rot=G_PLATE)
        yield from L.grip(FINGER_CLOSED, 0.7)
        yield from L.hold(0.4)
        if L.holding(0.028):
            break
        yield from L.grip(FINGER_OPEN, 0.5)
        yield from L.move(pre + off, 0.8, rot=G_PLATE)
    else:
        return
    lift = grasp + np.array([0.0, 0.0, 0.09])
    yield from L.move(lift + off, 1.2, rot=G_PLATE)
    over = np.array([FX, Y_PLATE, TOP + 0.105 + PL_GRIP])  # plate bottom 3.5 cm over the guide tops
    yield from L.move(over + off, 2.2, rot=G_PLATE)
    yield from _trim_pose(L, over, G_PLATE, steps=24)
    yield from L.hold(0.6)  # let the hanging plate stop swinging before it enters the 3 mm-a-side pocket
    off = _offset(L, over)
    rot = L.rot.copy()
    start = np.array([FX, Y_PLATE, SEAT_Z + PL_GRIP + 0.006])
    yield from L.move(start + off, 2.0, rot=rot)
    yield from L.press_until(start + off, max_depth=0.012, err_mm=3.0, min_depth=0.004, speed=0.006, rot=rot)
    yield from L.grip(FINGER_OPEN, 1.0)
    yield from L.hold(0.4)
    yield from L.move(L.pos + np.array([0.0, 0.0, 0.10]), 1.4, rot=L.rot.copy())


def demo_script(ctx):
    """Left arm only (the right arm holds READY). Reads only the fixed layout (object_jitter=0), its joint encoders
    and gripper opening."""
    L = ctx.left
    ok = yield from _grasp_roller(L)
    if not ok:
        yield from L.home()
        return
    yield from _seat_roller(L)
    yield from _place_plate(L)
    yield from L.home(3.0)
    yield from ctx.wait(1.0)
