"""On the station table, 35.5 cm in front of the arms and to the right of centre: a full-size electric car charge
port mock-up standing upright and fixed. A glacial-blue painted fender panel (25.6 x 19.8 cm, 14 mm thick, face at
x 35.5 cm) has a 17.6 x 18.1 cm opening closed by a flush, handleless blue charge port door (17.0 x 17.5 cm, 6 mm
thick) on a vertical hinge at its left edge, with a 3 mm reveal all round. The door is held shut by a push-push latch
(a sprung plunger behind its right-hand edge): press the door's right edge in about 5 mm and let go, and the plunger
pops it about 12 degrees ajar. Behind the door, in a charcoal pocket, sits a CCS1 combo inlet: the round silver
J1772 AC shroud above, and below it a safety-orange DC pin flap (60 x 40 mm) hinged at its bottom edge, with a pull
tab, covering the two copper DC pins. The flap clicks between shut (upright) and open (folded down 100 degrees).

Requires -E object_jitter=0: the fixture is fixed at its layout coordinates.
"""

from __future__ import annotations

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import belimits  # noqa: E402,F401  shared kit (geometry helpers)
import mechkit  # noqa: E402
import numpy as np  # noqa: E402
from closedloop import grip_rot, placeholder_demo  # noqa: E402
from mechkit import Joint, Mech, box, cyl, on_reset, tube  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = float(TABLE_TOP[SCENE])  # 0.7475 m
SCORE_OVER = "end"
SUCCESS_HOLD_S = 8.0

# ---------------------------------------------------------------------------------------------------------------------
# Layout. Mech frame: origin on the fender's outer face, x back into the car body (away from the robot), y left, z up
# from the table top. All sizes in metres, full scale (2017-2021 Chevrolet Bolt EV charge port parts).
FX, FY = 0.355, -0.116

BLUE = (0.24, 0.46, 0.72)  # glacial blue paint
DARK = (0.12, 0.13, 0.15)  # matte charcoal pocket and door backing
BLACK = (0.04, 0.04, 0.05)
GREY = (0.35, 0.36, 0.38)
LIGHT_GREY = (0.70, 0.71, 0.73)
SILVER = (0.80, 0.81, 0.83)
ORANGE = (0.98, 0.42, 0.04)
COPPER = (0.85, 0.52, 0.25)
WHITE = (0.92, 0.92, 0.90)

# Fender panel with the door opening.
PANEL_T = 0.014
PANEL_Y = 0.128  # half width: 25.6 cm
PANEL_H = 0.198
CUT_Y = 0.088  # opening half width: 17.6 cm
CUT_Z0, CUT_Z1 = 0.007, 0.188

# Door: 170 x 175 mm, 6 mm (4 mm painted skin + 2 mm dark inner backing), flush with the fender face when shut.
DOOR_W, DOOR_H, DOOR_T = 0.170, 0.175, 0.006
DOOR_Z0 = 0.010
DOOR_ZC = DOOR_Z0 + DOOR_H / 2  # 0.0975
HINGE_X, HINGE_Y = -0.004, 0.090  # gooseneck hinge axis, 4 mm proud of the face and 2 mm outside the opening
DOOR_EDGE_R = HINGE_Y + DOOR_W / 2  # 0.175: hinge axis to trailing edge

# Push-push latch plunger behind the door's trailing edge.
PLUNGER_Y = -0.064
PLUNGER_R = HINGE_Y - PLUNGER_Y  # 0.154 m from the hinge axis
PLUNGER_X0 = DOOR_T + 0.001  # cap face 1 mm behind the shut door
POP_REST = -0.031  # m: spring rest once popped (door about 12 deg ajar)

# CCS1 inlet in the pocket (centred at y -5 mm).
INLET_Y = -0.005
FLANGE_X0 = 0.038  # inlet face, 38 mm behind the fender face
DC_Z = 0.060  # DC pin centres
AC_Z = DC_Z + 0.057  # J1772 shroud centre
FLAP_HINGE = (0.034, INLET_Y, 0.040)  # bottom edge of the DC flap, axis along y
FLAP_W, FLAP_H, FLAP_T = 0.060, 0.040, 0.005
TAB = (0.016, 0.022, 0.012)  # pull tab: protrudes 16 mm, 22 mm wide, 12 mm tall
TAB_Z = FLAP_H - TAB[2] / 2  # pull tab along the flap's top edge (its top flush with the flap top)

DEG = math.pi / 180.0


def _ghost(shape):
    import dataclasses

    return dataclasses.replace(shape, collide=False)


def _panel_shapes() -> list:
    s = []
    cz = (CUT_Z0 + CUT_Z1) / 2
    xc = PANEL_T / 2
    side_w = PANEL_Y - CUT_Y
    for sgn in (1.0, -1.0):
        s.append(box((xc, sgn * (CUT_Y + side_w / 2), PANEL_H / 2), (PANEL_T, side_w, PANEL_H), BLUE))
    s.append(box((xc, 0.0, (CUT_Z1 + PANEL_H) / 2), (PANEL_T, 2 * CUT_Y, PANEL_H - CUT_Z1), BLUE))
    s.append(box((xc, 0.0, CUT_Z0 / 2), (PANEL_T, 2 * CUT_Y, CUT_Z0), BLUE))
    # rear clamping foot (steel), behind the pocket
    s.append(box((0.090, 0.0, 0.004), (0.090, 2 * PANEL_Y, 0.008), GREY))
    # GM 42780310 pocket: charcoal walls from the fender back face to the back wall
    depth = 0.046 - PANEL_T
    xm = PANEL_T + depth / 2
    for sgn in (1.0, -1.0):
        s.append(box((xm, sgn * (CUT_Y + 0.0015), cz), (depth, 0.003, CUT_Z1 - CUT_Z0 + 0.006), DARK))
    s.append(box((xm, 0.0, CUT_Z1 + 0.0015), (depth, 2 * CUT_Y + 0.006, 0.003), DARK))
    s.append(box((xm, 0.0, CUT_Z0 - 0.0015 + 0.003), (depth, 2 * CUT_Y + 0.006, 0.003), DARK))
    s.append(box((0.0475, 0.0, cz), (0.003, 2 * CUT_Y + 0.006, CUT_Z1 - CUT_Z0 + 0.006), DARK))
    # dark seal strip seen through the 3 mm reveal (look only)
    rz = (DOOR_Z0 + DOOR_Z0 + DOOR_H) / 2
    for sgn in (1.0, -1.0):
        s.append(_ghost(box((0.008, sgn * 0.0865, rz), (0.003, 0.009, DOOR_H + 0.012), BLACK)))
    for zz in (DOOR_Z0 - 0.0015, DOOR_Z0 + DOOR_H + 0.0015):
        s.append(_ghost(box((0.008, 0.0, zz), (0.003, 0.182, 0.009), BLACK)))
    # plunger sleeve (dark grey), fixed to the pocket back wall
    s.append(tube((0.031, PLUNGER_Y, DOOR_ZC), 0.008, 0.013, 0.032, GREY, axis="x"))
    # GM 24062486 CCS1 inlet: flange, J1772 AC shroud with its five pins, DC collar, DC pins in white rings
    s.append(box((FLANGE_X0 + 0.004, INLET_Y, 0.095), (0.008, 0.082, 0.118), DARK))
    s.append(tube((FLANGE_X0 - 0.007, INLET_Y, AC_Z), 0.0175, 0.0219, 0.014, SILVER, axis="x"))
    s.append(_ghost(cyl((FLANGE_X0 - 0.0005, INLET_Y, AC_Z), 0.0175, 0.001, BLACK, axis="x")))
    for dy, dz, r in ((0.0, 0.010, 0.0018), (-0.009, -0.004, 0.0018), (0.009, -0.004, 0.0018),
                      (-0.006, -0.012, 0.0012), (0.006, -0.012, 0.0012)):
        s.append(_ghost(cyl((FLANGE_X0 - 0.006, INLET_Y + dy, AC_Z + dz), r, 0.010, SILVER, axis="x")))
    s.append(_ghost(box((FLANGE_X0 - 0.0005, INLET_Y, DC_Z), (0.001, 0.056, 0.034), DARK)))
    for dy in (-0.016, 0.016):
        s.append(_ghost(tube((FLANGE_X0 + 0.0008, INLET_Y + dy, DC_Z), 0.0045, 0.008, 0.0016, WHITE, axis="x")))
        s.append(_ghost(cyl((FLANGE_X0 + 0.004, INLET_Y + dy, DC_Z), 0.004, 0.008, COPPER, axis="x")))
    return s


# Door body in its joint frame (origin on the hinge axis at mid height).
_DX0 = -HINGE_X  # door outer face, 4 mm behind the hinge axis
DOOR_LO = np.array([_DX0, -(HINGE_Y + DOOR_W / 2), -DOOR_H / 2])  # local box of the door body (for release checks)
DOOR_HI = np.array([_DX0 + DOOR_T, -(HINGE_Y - DOOR_W / 2), DOOR_H / 2])
DOOR_SHAPES = [
    box((_DX0 + 0.002, -HINGE_Y, 0.0), (0.004, DOOR_W, DOOR_H), BLUE),
    box((_DX0 + 0.005, -HINGE_Y, 0.0), (0.002, DOOR_W - 0.004, DOOR_H - 0.004), DARK),
    # look-only: two push ribs on the inner backing and the gooseneck hinge barrel and arm
    _ghost(box((_DX0 + 0.0065, -HINGE_Y, 0.04), (0.001, 0.12, 0.004), BLACK)),
    _ghost(box((_DX0 + 0.0065, -HINGE_Y, -0.04), (0.001, 0.12, 0.004), BLACK)),
    _ghost(cyl((0.0, 0.0, 0.0), 0.004, 0.05, GREY)),
    _ghost(box((_DX0 / 2 + 0.003, -0.006, 0.0), (0.006, 0.012, 0.02), GREY)),
]
OUTER_DOOR = Joint("outer_door", "hinge", axis="z", at=(HINGE_X, HINGE_Y, DOOR_ZC), low=-100 * DEG, high=3 * DEG,
                   spring=0.0, damping=0.05, mass=0.18, hold=0.90, shapes=DOOR_SHAPES, friction=0.9)

PLUNGER = Joint(
    "plunger", "slide", axis="x", at=(PLUNGER_X0, PLUNGER_Y, DOOR_ZC), low=-0.032, high=0.009,
    spring=350.0, damping=14.0, mass=0.02,
    shapes=[cyl((0.002, 0.0, 0.0), 0.0075, 0.004, LIGHT_GREY, axis="x"),
            cyl((0.017, 0.0, 0.0), 0.005, 0.026, GREY, axis="x")],
)

# DC flap in its joint frame (origin on the hinge line at the flap's bottom edge, plate centred on it in x).
FLAP_LO = np.array([-TAB[0] - FLAP_T / 2, -FLAP_W / 2, 0.0])
FLAP_HI = np.array([FLAP_T / 2, FLAP_W / 2, FLAP_H])
TAB_SHAPE = box((-FLAP_T / 2 - TAB[0] / 2, 0.0, TAB_Z), TAB, ORANGE)
DC_FLAP = Joint(
    "dc_flap", "hinge", axis="y", at=FLAP_HINGE, low=-100 * DEG, high=0.0,
    spring=0.14, damping=0.015, mass=0.025, detents=(0.0, -100 * DEG), friction=1.2,
    shapes=[box((0.0, 0.0, FLAP_H / 2), (FLAP_T, FLAP_W, FLAP_H), ORANGE), TAB_SHAPE,
            _ghost(cyl((0.0, 0.0, 0.0), 0.0025, FLAP_W + 0.004, GREY, axis="y"))],
)

CHARGE_PORT = Mech("charge_port", pos=(FX, FY, TOP), mass=None, friction=0.6, shapes=_panel_shapes(),
                   joints=(OUTER_DOOR, PLUNGER, DC_FLAP))

OBJECTS = [CHARGE_PORT]

# ---------------------------------------------------------------------------------------------------------------------
# Push-push latch (scorer side). A real push-push latch is a heart-shaped cam track: press the door in past flush, let
# go, and its spring pops the door ajar; press it shut again and it latches. Here the plunger is a real sprung slide
# that touches the door, and this state machine moves its spring rest and the door hinge's hold:
#   LATCHED  (door hold 0.90 N m, plunger rest 0)  --door >= +2.0 deg and plunger >= 4.0 mm-->  ARMED (hold 0.04 N m)
#   ARMED    --door eases back to <= +1.4 deg-->  POPPED (plunger rest -31 mm: the plunger pushes the door ~12 deg ajar)
#   POPPED   --door pushed back to >= +2.0 deg and plunger >= 4.0 mm-->  RELATCH (plunger rest 0)
#   RELATCH  --door eases to <= +1.0 deg-->  LATCHED (hold 0.90 N m again)
HOLD_LATCHED, HOLD_FREE = 0.90, 0.04
ARM_PHI, ARM_S = 2.0, 0.004
POP_PHI, RELATCH_PHI = 1.4, 1.0

# Scoring thresholds
FAULT_STEPS = 2
POPPED_DEG = -6.0
DOOR_FULL, DOOR_PART = -90.0, -75.0
FLAP_FULL, FLAP_PART = -85.0, -45.0
RELEASE_CLEARANCE = 0.030
STILL_DEG_S, STILL_MM_S = 2.0, 3.0
END_HOLD_S = 1.0
NOMINAL_DT = 0.05

# Offline hooks: check.py and the replay wrapper may set these (never the demo).
PROBES: dict = {}

_L: dict = {}


def _fresh() -> dict:
    return {"state": "LATCHED", "popped": False, "P": False, "fault": None, "fault_run": 0, "still_s": 0.0,
            "prev": None, "seen": None, "writes": [], "last_t": None}


_L.update(_fresh())


def _write(kind: str, joint: str, value: float) -> None:
    _L["writes"].append((kind, joint, value))
    probe = PROBES.get(kind)
    if probe is not None:
        probe(joint, value)
        return
    env = mechkit._ENV.get("env")
    if env is None:
        return
    if kind == "hold":
        mechkit.set_hold(env, CHARGE_PORT.name, joint, value)
    else:
        mechkit.set_rest(env, CHARGE_PORT.name, joint, value)


@on_reset
def _reset(env=None) -> None:
    _L.clear()
    _L.update(_fresh())
    if env is not None:
        try:
            mechkit.set_hold(env, CHARGE_PORT.name, "outer_door", HOLD_LATCHED)
            mechkit.set_rest(env, CHARGE_PORT.name, "plunger", 0.0)
        except Exception as exc:  # noqa: BLE001  never break a reset
            print(f"charge_port reset: {exc!r}", flush=True)
    elif PROBES.get("hold") is not None:
        PROBES["hold"]("outer_door", HOLD_LATCHED)
        PROBES["rest"]("plunger", 0.0)


def _latch_step(phi_deg: float, s_m: float) -> None:
    st = _L["state"]
    if st == "LATCHED" and phi_deg >= ARM_PHI and s_m >= ARM_S:
        _L["state"] = "ARMED"
        _write("hold", "outer_door", HOLD_FREE)
    elif st == "ARMED" and phi_deg <= POP_PHI:
        _L["state"] = "POPPED"
        _L["popped"] = True
        _write("rest", "plunger", POP_REST)
    elif st == "POPPED" and phi_deg >= ARM_PHI and s_m >= ARM_S:
        _L["state"] = "RELATCH"
        _write("rest", "plunger", 0.0)
    elif st == "RELATCH" and phi_deg <= RELATCH_PHI:
        _L["state"] = "LATCHED"
        _write("hold", "outer_door", HOLD_LATCHED)


def _box_clearance(points, rot, org, lo, hi) -> float:
    pts = (np.asarray(points, dtype=float).reshape(-1, 3) - org) @ rot
    out = np.maximum(np.maximum(lo - pts, pts - hi), 0.0)
    return float(np.linalg.norm(out, axis=1).min())


def _hands_clear(st, clearance: float = RELEASE_CLEARANCE) -> bool:
    """Every finger pad point of both arms at least ``clearance`` (30 mm) from the door's and the flap's own boxes.
    False offline."""
    env = mechkit._ENV.get("env")
    if env is None:
        return False
    try:
        pts = mechkit.finger_points(env)
        bodies = mechkit.body_poses(CHARGE_PORT, st)
        for name, lo, hi in (("outer_door", DOOR_LO, DOOR_HI), ("dc_flap", FLAP_LO, FLAP_HI)):
            rot, org = bodies[name]
            if _box_clearance(pts, np.asarray(rot), np.asarray(org), lo, hi) < clearance:
                return False
        return True
    except Exception:  # noqa: BLE001  an unknown grip never counts as released
        return False


def _dt() -> float:
    t = float(mechkit.elapsed())
    prev = _L["last_t"]
    _L["last_t"] = t
    return t - prev if prev is not None and t > prev + 1e-9 else NOMINAL_DT


def _angles(st) -> tuple[float, float, float]:
    j = st.joints
    return math.degrees(float(j["outer_door"])), math.degrees(float(j["dc_flap"])), float(j["plunger"])


def _update(objects) -> None:
    if objects is _L["seen"]:  # success() and score() share one dict per step
        return
    _L["seen"] = objects
    dt = _dt()
    st = objects[CHARGE_PORT.name]
    phi, alpha, s = _angles(st)
    _latch_step(phi, s)
    if _L["popped"] and phi <= POPPED_DEG:
        _L["P"] = True
    bad = None
    if phi > 4.0 or phi < -102.0:
        bad = f"door forced past its stops ({phi:.1f} deg)"
    elif alpha > 2.0 or alpha < -102.0:
        bad = f"DC flap forced past its stops ({alpha:.1f} deg)"
    _L["fault_run"] = _L["fault_run"] + 1 if bad else 0
    if bad and _L["fault_run"] >= FAULT_STEPS and _L["fault"] is None:
        _L["fault"] = bad
    still = False
    if _L["prev"] is not None:
        p0, a0, s0 = _L["prev"]
        still = (abs(phi - p0) / dt < STILL_DEG_S and abs(alpha - a0) / dt < STILL_DEG_S
                 and abs(s - s0) * 1000.0 / dt < STILL_MM_S)
    _L["prev"] = (phi, alpha, s)
    _L["still_s"] = _L["still_s"] + dt if (still and _hands_clear(st)) else 0.0


def credit(phi: float, alpha: float) -> float:
    """Tier for a released, settled end state (angles in degrees, negative = open)."""
    if phi <= DOOR_FULL and alpha <= FLAP_FULL:
        return 1.0
    if (phi <= DOOR_FULL and alpha <= FLAP_PART) or (phi <= DOOR_PART and alpha <= FLAP_FULL):
        return 0.65
    if phi <= DOOR_FULL:
        return 0.45
    if phi <= POPPED_DEG:
        return 0.20
    return 0.0


def score(objects) -> float:
    """0 after a sustained fault, before the door has been push-popped at least 6 deg ajar, or unless both hands have
    been 30 mm clear of the door and flap and everything still for the last 1.0 s. Otherwise the credit() tier."""
    _update(objects)
    if _L["fault"] is not None or not _L["P"] or _L["still_s"] < END_HOLD_S - 1e-6:
        return 0.0
    phi, alpha, _ = _angles(objects[CHARGE_PORT.name])
    return credit(phi, alpha)


PARKED_CLEARANCE = 0.10  # success() (which ends a run early) also waits for both hands 10 cm clear


def success(objects) -> bool:
    return score(objects) >= 1.0 and _hands_clear(objects[CHARGE_PORT.name], PARKED_CLEARANCE)


# ---------------------------------------------------------------------------------------------------------------------
# The robot keeps out of the solid car body behind the charge pocket (sweep.py checks every arm link).
ARM_KEEP_OUT = [
    {"name": "car body behind the charge pocket", "x": (FX + 0.052, FX + 0.140), "y": (FY - PANEL_Y, FY + PANEL_Y),
     "z_top": TOP + 0.25},
]

# ---------------------------------------------------------------------------------------------------------------------
# Scripted demo (right arm only; the left arm holds READY). It reads the live outer_door, plunger and dc_flap joint
# values from the simulator to know when the press has armed the latch, how far the door popped and how far the flap
# has turned, plus its own joint encoders and gripper opening. It never moves an object or writes a joint.

demo = placeholder_demo

PITCH = 80 * DEG  # level approach leaned 10 deg down: reaches the low fender work at 0.1 mm IK error
TIP = 0.0128  # closed pad tips past the tool point
H_W = np.array([FX + HINGE_X, FY + HINGE_Y])  # door hinge axis, world xy
GRIP_R = 0.160  # pinch point: 15 mm in from the trailing edge
GRIP_Z = TOP + DOOR_Z0 + DOOR_H - 0.010  # pads 10 mm below the door's top edge
PINCH_GAP = 0.016
TAB_PINCH_GAP = 0.019  # 3 mm under the 22 mm tab: a firm pinch locks the tab's turn and stalls the arm
FLAP_HW = np.array([FX + FLAP_HINGE[0], FY + FLAP_HINGE[1], TOP + FLAP_HINGE[2]])
XFER = 0.06  # m/s average for transfers (smoothstep peak 1.5x = 0.09 m/s)
SLOW = 0.02  # m/s average near contact


def _finger(gap: float) -> float:
    return FINGER_OPEN * float(np.clip(gap / 0.0939, 0.0, 1.0))


def door_point(phi_deg: float, r: float, depth: float = DOOR_T / 2) -> np.ndarray:
    """World xy of the point on the door's mid plane r from the hinge axis, at door angle phi (deg)."""
    p = phi_deg * DEG
    a, b = _DX0 + depth, -r
    c, s = math.cos(p), math.sin(p)
    return H_W + np.array([c * a - s * b, s * a + c * b])


def _door_yaw(phi_deg: float) -> float:
    return phi_deg * DEG + math.pi / 2  # fingers close along the door normal


def _approach() -> np.ndarray:
    return grip_rot(0.0, PITCH, 0.0)[:, 2]


def _go(arm, pos, finger=None, rot=None, speed=XFER, min_s=0.4):
    dist = float(np.linalg.norm(np.asarray(pos, dtype=float) - arm.pos))
    yield from arm.move(pos, max(min_s, dist / speed), finger, rot=arm.rot.copy() if rot is None else rot)


def _joint_deg(ctx, name: str) -> float:
    return math.degrees(float(ctx.obj(CHARGE_PORT.name).joints[name]))


def _plunger(ctx) -> float:
    return float(ctx.obj(CHARGE_PORT.name).joints["plunger"])


def demo_script(ctx):
    R = ctx.right
    live = not R.offline
    app = _approach()
    rot_press = grip_rot(0.0, PITCH, 0.0)

    # 1. Press the flush door in over its plunger with the closed fingertips, then retract so it pops ajar.
    face = np.array([FX, FY + PLUNGER_Y, TOP + DOOR_ZC])
    start = face - app * (TIP + 0.040)
    yield from R.grip(FINGER_CLOSED, 0.5)
    yield from _go(R, start - np.array([0.03, 0.0, 0.0]), FINGER_CLOSED, rot_press)
    yield from _go(R, start, rot=rot_press, speed=SLOW)
    yield from R.trim_to_measured(start)
    p0 = R.pos.copy()
    depth, armed = 0.0, False
    while depth < 0.040 + 0.016:  # up to 16 mm past the face: wrist compliance takes up the rest
        depth += 0.020 * R.dt
        R._set_cartesian(p0 + app * depth, rot_press, FINGER_CLOSED)
        yield
        if _joint_deg(ctx, "outer_door") >= ARM_PHI + 0.6 and _plunger(ctx) >= ARM_S + 0.0005:
            armed = True
            break
    yield from R.hold(0.2)
    back = p0 - app * 0.010
    yield from R.move(back, 1.0, rot=rot_press)  # steady release: the plunger pops the door after the fingertips
    for _ in range(40):  # up to 2 s for the pop to settle
        if _joint_deg(ctx, "outer_door") <= POPPED_DEG - 1.0:
            break
        yield
    yield from R.hold(0.4)
    phi0 = _joint_deg(ctx, "outer_door") if live and armed else -10.0
    phi0 = min(phi0, -8.0)

    # 2. Pinch the popped door's trailing top corner from above: the open pads come in along the door's plane from
    #    beyond the edge, so the rear pad slips into the gap behind the door without pushing it shut.
    yaw0 = _door_yaw(phi0)
    rot_pinch = grip_rot(yaw0, 0.0, 0.0)
    out = np.r_[door_point(phi0, GRIP_R + 0.050), GRIP_Z]
    yield from _go(R, back + np.array([-0.04, -0.02, 0.06]))
    yield from _go(R, out + np.array([-0.03, 0.0, 0.03]), _finger(PINCH_GAP), rot_pinch)
    yield from _go(R, out, rot=rot_pinch, speed=SLOW)
    yield from R.trim_to_measured(out)
    sag = R.pos - out
    for attempt in range(2):
        phi_now = _joint_deg(ctx, "outer_door") if live else phi0
        phi_now = min(phi_now, -6.0)
        grip = np.r_[door_point(phi_now, GRIP_R), GRIP_Z]
        yield from _go(R, grip + sag, rot=grip_rot(_door_yaw(phi_now), 0.0, 0.0), speed=SLOW)
        yield from R.grip(FINGER_CLOSED, 0.6)
        yield from R.hold(0.3)
        if R.holding(0.003):
            break
        yield from R.grip(_finger(PINCH_GAP), 0.4)
        yield from _go(R, np.r_[door_point(phi_now, GRIP_R + 0.050), GRIP_Z] + sag, speed=SLOW)
    # 3. Swing the door round its hinge to 98 deg, the pads turning with it.
    phi_a = _joint_deg(ctx, "outer_door") if live else phi0
    phi_a = min(phi_a, -6.0)
    target = -98.0
    n = int(round(7.0 / R.dt))
    for i in range(1, n + 1):
        u = i / n
        ph = phi_a + (target - phi_a) * (3 * u * u - 2 * u ** 3)
        R._set_cartesian(np.r_[door_point(ph, GRIP_R), GRIP_Z] + sag, grip_rot(_door_yaw(ph), 0.0, 0.0),
                         FINGER_CLOSED)
        yield
    yield from R.hold(0.5)
    # open the pads, slide them off past the door edge, then lift away
    yield from R.grip(_finger(PINCH_GAP + 0.006), 0.6)
    off = np.r_[door_point(target, GRIP_R + 0.050), GRIP_Z] + sag
    yield from _go(R, off, speed=SLOW)
    yield from _go(R, off + np.array([0.02, -0.06, 0.02]))

    # 4. DC flap: level gripper leaned 10 deg down, pads open across the orange pull tab on the flap's top edge, under
    #    the AC shroud. Squeeze the tab's sides lightly and draw it round the flap hinge, the hand following the arc of
    #    the tab's root (the point 2 mm past the fingertips), so the flap turns about that point between the pads and
    #    the plate below it swings away from the fingertips. Past the detent midpoint (-50 deg) the detent spring takes
    #    over; let go at -80 deg and it folds onto its open stop.
    rot_flap = grip_rot(0.0, PITCH, 0.0)
    pivot = np.array([FX + FLAP_HINGE[0] - FLAP_T / 2, FY + INLET_Y, TOP + FLAP_HINGE[2] + FLAP_H - 0.002])
    tcp0 = pivot - app * (TIP + 0.002)  # pad tips 2 mm short of the flap's face
    pre = tcp0 - np.array([0.045, 0.0, 0.0])
    yield from _go(R, np.array([0.24, FY + INLET_Y - 0.03, TOP + 0.12]), _finger(0.050), rot_flap)
    yield from _go(R, pre, rot=rot_flap)
    yield from R.trim_to_measured(pre)
    fsag = R.pos - pre
    yield from _go(R, tcp0 + fsag, rot=rot_flap, speed=SLOW)
    yield from R.grip(_finger(TAB_PINCH_GAP), 0.6)  # light squeeze: the tab must still turn between the pads
    yield from R.hold(0.3)
    rel = pivot - FLAP_HW

    def flap_tcp(a_deg):
        c, s = math.cos(a_deg * DEG), math.sin(a_deg * DEG)
        p = FLAP_HW + np.array([c * rel[0] + s * rel[2], rel[1], -s * rel[0] + c * rel[2]])
        return p - app * (TIP + 0.002) + fsag

    n = int(round(5.0 / R.dt))
    for i in range(1, n + 1):  # draw the tab round the flap hinge to -80 deg over 5 s
        u = i / n
        R._set_cartesian(flap_tcp(-80.0 * (3 * u * u - 2 * u ** 3)), rot_flap, R.finger)
        yield
    a_cmd = -80.0
    for _ in range(int(round(4.0 / R.dt))):  # if the flap lags (wrist give, pad slip), lead it on to at most -110 deg
        if not live or _joint_deg(ctx, "dc_flap") <= -70.0:
            break
        a_cmd = max(a_cmd - 8.0 * R.dt, -110.0)
        R._set_cartesian(flap_tcp(a_cmd), rot_flap, R.finger)
        yield
    yield from R.grip(FINGER_OPEN, 0.5)
    for _ in range(40):  # the detent spring folds the flap down onto its stop
        if not live or _joint_deg(ctx, "dc_flap") <= -88.0:
            break
        yield
    yield from _go(R, R.pos + np.array([-0.04, 0.0, 0.03]), speed=SLOW)
    # back out to the side of the fender panel before the joint-space swing home, so the open fingertips never sweep
    # the fender face
    yield from _go(R, np.array([0.25, FY + INLET_Y - 0.04, TOP + 0.11]))
    yield from _go(R, np.array([0.25, -0.28, TOP + 0.24]))

    # 5. Park clear with the gripper open and hold still.
    yield from R.home(5.0)
    yield from ctx.wait(10.0)

