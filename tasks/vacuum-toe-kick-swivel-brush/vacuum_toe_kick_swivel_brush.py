"""A kitchen floor: a slate vinyl panel on the table and, at its left, the bottom of a 9 in (229 mm) white base cabinet
with the standard toe-kick recess under it: 101.6 mm (4 in) high, 76 mm (3 in) deep, a matte black kick board at the
back, open at both ends. Ten dried lentils are spilled on the floor: three in the open in front of the cabinet, three
under the overhang and four in the back corner against the kick board, where the top camera cannot see them. A red
low-profile swivel brush tool (Milwaukee 49-90-2027, 495 mm, pivot locked straight) lies on the floor, its head by the
cabinet's far end, turned about 70 deg off square, and its 64 mm hose cuff in front of the right arm. The brush head
swivels freely through 360 deg on its neck, so the wand cannot set its heading.

The robot must take the cuff and steer the head over every lentil, including the ones in the corner. Pushed straight
at the kick board, a turned head wedges on its corner; drawn along the board with light pressure, it turns until an
edge lies flat, and must then be run flat along the board. Then the tool is set down.

Requires -E object_jitter=0 (fixed layout).

Run it:  inspect-robots run --embodiment yam_arena -E task=vacuum_toe_kick_swivel_brush.py -E object_jitter=0 \\
           -E control_hz=20 --instruction "Use the brush tool to vacuum up every spilled lentil, including those
           under the cabinet's toe kick, then set it down." --scorer task_score ...
"""

import dataclasses
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402

import belimits  # noqa: E402
import mechkit  # noqa: E402
from closedloop import grip_rot, placeholder_demo  # noqa: E402
from mechkit import Joint, Mech, box, cyl, hull, lamp, on_reset, on_step, ring_profile, set_lamp  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 8.0  # Isaac ends the episode once success() holds this long; the arm is parked by then
SCORE_OVER = "end"

# Colours (sRGB)
SLATE = (0.30, 0.31, 0.33)  # vinyl floor
CAB_WHITE = (0.93, 0.93, 0.91)
RAIL_GREY = (0.70, 0.70, 0.69)
KICK_BLACK = (0.08, 0.08, 0.08)
TOOL_RED = (0.78, 0.10, 0.10)
RIM_GREY = (0.22, 0.22, 0.23)
BRISTLE = (0.05, 0.05, 0.05)
BUTTON_GREY = (0.45, 0.45, 0.47)
BEAN = (0.45, 0.32, 0.16)  # brown lentil; the lens is drawn lit (emissive), so a mid brown reads saturated


def _look(shape):
    return dataclasses.replace(shape, collide=False)


# ---------------------------------------------------------------------------------------------------------------------
# The kitchen (fixed). Frame: station xy, z from the table top.
PANEL_T = 0.006
FL = TOP + PANEL_T  # floor (vinyl) top
PANEL_X, PANEL_Y = (0.12, 0.44), (-0.35, 0.26)
CAB_X = (0.155, 0.384)  # 9 in (229 mm) cabinet width
DOOR_Y = 0.160  # door face
KICK_Y = 0.236  # kick board face: the recess is 76 mm deep
KICK_T = 0.019  # 3/4 in kick board
BACK_Y = 0.262  # the carcass is cut off here (the rest of the 24 in depth is behind, out of reach)
TOE_H = 0.1016  # 4 in recess height
CAB_TOP = 0.180  # the cabinet is cut off 180 mm above the floor
OVER_Z = FL + TOE_H

LENTIL_R, LENTIL_H = 0.0025, 0.0025
MOUTH_ROW = [(0.185, 0.150), (0.245, 0.150), (0.305, 0.150)]
MID_ROW = [(0.200, 0.195), (0.260, 0.195), (0.320, 0.195)]
CORNER_ROW = [(0.180, 0.225), (0.235, 0.225), (0.290, 0.225), (0.376, 0.225)]  # the last 8 mm from the cabinet end
LENTILS = MOUTH_ROW + MID_ROW + CORNER_ROW
ROW_OF = ["mouth"] * 3 + ["mid"] * 3 + ["corner"] * 4
NAMES = [f"lentil{i}" for i in range(len(LENTILS))]

_cab_mid = (CAB_X[0] + CAB_X[1]) / 2
_cab_w = CAB_X[1] - CAB_X[0]
_z0 = PANEL_T  # floor top, relative to TOP
KITCHEN = Mech(
    "kitchen", pos=(0.0, 0.0, TOP), mass=None, friction=0.4,  # vinyl and painted board against ABS and nylon
    shapes=[
        box(((PANEL_X[0] + PANEL_X[1]) / 2, (PANEL_Y[0] + PANEL_Y[1]) / 2, PANEL_T / 2),
            (PANEL_X[1] - PANEL_X[0], PANEL_Y[1] - PANEL_Y[0], PANEL_T), SLATE),
        # carcass bottom and the lower door, over the recess
        box((_cab_mid, (DOOR_Y + BACK_Y) / 2, _z0 + (TOE_H + CAB_TOP) / 2),
            (_cab_w, BACK_Y - DOOR_Y, CAB_TOP - TOE_H), CAB_WHITE),
        # kick board and the cabinet base behind it
        box((_cab_mid, (KICK_Y + BACK_Y) / 2, _z0 + TOE_H / 2), (_cab_w, BACK_Y - KICK_Y, TOE_H), KICK_BLACK),
        # shaker door: bottom rail groove and the two stile grooves (look only), and the door's bottom reveal
        _look(box((_cab_mid, DOOR_Y - 0.0006, _z0 + TOE_H + 0.068), (_cab_w - 0.12, 0.0016, 0.004), RAIL_GREY)),
        _look(box((CAB_X[0] + 0.060, DOOR_Y - 0.0006, _z0 + TOE_H + 0.120), (0.004, 0.0016, 0.104), RAIL_GREY)),
        _look(box((CAB_X[1] - 0.060, DOOR_Y - 0.0006, _z0 + TOE_H + 0.120), (0.004, 0.0016, 0.104), RAIL_GREY)),
        _look(box((_cab_mid, DOOR_Y - 0.0006, _z0 + TOE_H + 0.0025), (_cab_w, 0.0016, 0.005), RAIL_GREY)),
        *[s for (x, y), n in zip(LENTILS, NAMES)
          for s in lamp(n, (x, y, _z0 + LENTIL_H / 2 + 0.0002), LENTIL_R, {"bean": BEAN}, off=SLATE,
                        height=LENTIL_H)],
    ],
)

# ---------------------------------------------------------------------------------------------------------------------
# The brush tool (free): Milwaukee 49-90-2027 AIR-TIP Low-Profile Pivoting Brush Tool, 495 mm overall, 81 mm head,
# 64 mm cuff, 0.316 kg (0.697 lb). Mech origin: the swivel axis on the floor; the wand runs along local -y.
HEAD_U, HEAD_V, HEAD_H = 0.081, 0.055, 0.024  # head length (across the wand when square), depth, height
MOUTH_U, MOUTH_V = 0.033, 0.022  # the suction mouth under the head, half sizes (inside the bristle rows)
COLLECT_IN = 0.003  # a lentil centre must be this far inside the mouth edge (more than half of it over the mouth)
HEAD0 = math.radians(70.0)  # the head starts turned 70 deg off square (swivel reads 0 there)
CUFF_Y = (-0.4675, -0.4075)
CUFF_R = 0.032
CUFF_Z = 0.032
GRIP_LOCAL = -0.420  # grip centre along the wand, 12.5 mm inboard of the cuff's inner end
TOOL_MASS, HEAD_MASS = 0.316, 0.080
SWIVEL_HOLD = 0.002  # N m: the swivel's own friction (a vertical axis, so no gravity load)


def _head_shapes():
    """The head in its own frame (long axis u along x when square): one convex collider from the bristle tips (z 0)
    to the crown, with a look-only black skirt band and grey rim over it."""
    hu, hv = HEAD_U / 2, HEAD_V / 2
    pts = [(sx * hu, sy * hv, 0.0) for sx in (-1, 1) for sy in (-1, 1)]
    pts += [(sx * hu, sy * hv, 0.018) for sx in (-1, 1) for sy in (-1, 1)]
    pts += [(sx * (hu - 0.006), sy * (hv - 0.008), HEAD_H) for sx in (-1, 1) for sy in (-1, 1)]
    c, s = math.cos(HEAD0 / 2), math.sin(HEAD0 / 2)
    turn = (c, 0.0, 0.0, s)  # the head is built turned HEAD0 about z: swivel 0 is the start heading

    def rz(p):
        ca, sa = math.cos(HEAD0), math.sin(HEAD0)
        return (ca * p[0] - sa * p[1], sa * p[0] + ca * p[1], p[2])

    shapes = [hull((0, 0, 0), pts, TOOL_RED, tilt=turn)]
    # look-only: bristle skirt (4 mm), dark grey rim band, and the swivel collar on top
    shapes.append(_look(box(rz((0, 0, 0.002)), (HEAD_U + 0.001, HEAD_V + 0.001, 0.004), BRISTLE, turn)))
    shapes.append(_look(box(rz((0, 0, 0.007)), (HEAD_U + 0.001, HEAD_V + 0.001, 0.006), RIM_GREY, turn)))
    shapes.append(_look(cyl((0, 0, HEAD_H + 0.001), 0.013, 0.003, RIM_GREY)))
    return shapes


def _wand_shapes():
    """Cuff (stepped bore for 2-1/2, 1-7/8 and 1-1/4 in hoses), the taper into the flat duct, and the locked pivot."""
    L = CUFF_Y[1] - CUFF_Y[0]
    cuff = ring_profile((0, CUFF_Y[0], CUFF_Z),
                        [(0.029, CUFF_R, 0.0), (0.029, CUFF_R, 0.022), (0.0238, CUFF_R, 0.024), (0.0238, CUFF_R, 0.042),
                         (0.0160, CUFF_R, 0.044), (0.0160, CUFF_R, L)], TOOL_RED, axis="y")
    ring = [(0.026 * math.cos(a), CUFF_Y[1] - 0.001, CUFF_Z + 0.026 * math.sin(a))
            for a in np.linspace(0, 2 * math.pi, 12, endpoint=False)]
    flat = [(sx * 0.025, -0.050, 0.036 + sz * 0.011) for sx in (-1, 1) for sz in (-1, 1)]
    duct = hull((0, 0, 0), ring + flat, TOOL_RED)
    pivot = cyl((0, -0.040, 0.036), 0.018, 0.050, RIM_GREY, axis="x")
    button = _look(cyl((0, -0.040, 0.0545), 0.007, 0.003, BUTTON_GREY))
    neck = _look(box((0, -0.020, 0.030), (0.022, 0.040, 0.008), RIM_GREY))
    return [cuff, duct, pivot, button, neck]


TOOL_YAW = math.radians(-27.8)
TOOL_POS = (0.370, 0.150, FL)
BRUSH = Mech(
    "brush_tool", pos=TOOL_POS, yaw=TOOL_YAW, mass=TOOL_MASS - HEAD_MASS, friction=0.4,
    shapes=_wand_shapes(),
    joints=[Joint("swivel", "hinge", axis="z", at=(0.0, 0.0, 0.0), low=-3.1, high=3.1, mass=HEAD_MASS,
                  friction=0.4, hold=SWIVEL_HOLD, damping=0.002, shapes=_head_shapes())],
)

OBJECTS = [KITCHEN, BRUSH]

# ---------------------------------------------------------------------------------------------------------------------
# The robot keeps out of the cabinet: no arm part over the cabinet and its toe-kick recess (grown 4 cm) below 4 cm
# over the cabinet's cut-off top. Only the passive brush head goes under the overhang.
CABINET_KEEP_OUT = {"x": (CAB_X[0] - 0.04, CAB_X[1] + 0.04), "y": (DOOR_Y - 0.04, BACK_Y + 0.04)}
ARM_KEEP_OUT = [
    {"name": "base cabinet and its toe-kick recess, 4 cm round", **CABINET_KEEP_OUT, "z_top": FL + CAB_TOP + 0.04},
]

# ---------------------------------------------------------------------------------------------------------------------
# Scoring (numbers match task.json and README.md)
COLLECT_STEPS = 3  # 0.15 s at 20 Hz with the lentil centre in the mouth
FLAT_DEG = 6.0  # head normal within this of vertical
TIPS_ABOVE = 0.003  # bristle tips (head origin) within this of the floor
HEAD_CLEAR = 0.030  # both grippers this far from the head while it collects
RELEASE_CLEAR = 0.030
STILL = 0.005  # m/s
FINAL_STEPS = 40  # 2 s at 20 Hz
SET_DOWN = 0.010  # the tool's lowest point within 1 cm of the floor

FORCE_RELEASED: list = [None]  # offline test hook for check.py; None in live runs
_ST: dict = {}


def _reset_state(env=None) -> None:
    _ST.update(last=None, step=0, run=[0] * len(NAMES), got=[False] * len(NAMES), zero=False, why="", still_run=0)


_reset_state()


@on_reset
def _on_reset(env=None) -> None:
    _reset_state(env)
    for n in NAMES:
        set_lamp(KITCHEN, n, "bean")


def head_frame(s):
    """(origin, u, v, n): the head centre on its bristle plane and its long axis, depth axis and normal, in the world."""
    rot, org = mechkit.body_poses(BRUSH, s)["swivel"]
    rot = np.asarray(rot, float)
    turn = np.array([[math.cos(HEAD0), -math.sin(HEAD0), 0.0], [math.sin(HEAD0), math.cos(HEAD0), 0.0], [0, 0, 1.0]])
    r = rot @ turn
    return np.asarray(org, float), r[:, 0], r[:, 1], r[:, 2]


def heading_error_deg(s) -> float:
    """How far the head's long axis is off the kick board's line (x), 0 to 90 deg."""
    _, u, _, _ = head_frame(s)
    a = abs(math.degrees(math.atan2(u[1], u[0]))) % 180.0
    return min(a, 180.0 - a)


def head_flat(s) -> bool:
    org, _, _, n = head_frame(s)
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(n[2])))))
    return bool(tilt <= FLAT_DEG and org[2] <= FL + TIPS_ABOVE)


def in_mouth(s, xy) -> bool:
    org, u, v, _ = head_frame(s)
    d = np.array([xy[0], xy[1], FL]) - org
    return bool(abs(float(d @ u)) <= MOUTH_U - COLLECT_IN and abs(float(d @ v)) <= MOUTH_V - COLLECT_IN)


def on_panel(s) -> bool:
    org, _, _, _ = head_frame(s)
    return bool(PANEL_X[0] <= org[0] <= PANEL_X[1] and PANEL_Y[0] <= org[1] <= PANEL_Y[1] and org[2] > FL - 0.01)


def _clear(objects, clearance, lo=None, hi=None) -> bool:
    if FORCE_RELEASED[0] is not None:
        return bool(FORCE_RELEASED[0])
    if mechkit._ENV.get("env") is None:  # offline replays have no Isaac env
        return True
    try:
        return bool(mechkit.released(objects["brush_tool"], clearance=clearance, lo=lo, hi=hi))
    except Exception:
        return False


# the head's swept disc in the tool's own frame (it swivels about the origin), for the gripper clearance
HEAD_LO, HEAD_HI = (-0.05, -0.05, 0.0), (0.05, 0.05, 0.03)


def _update(objects) -> None:
    """Advance the latches once per control step (score() and success() get the same dict each step)."""
    if objects is _ST["last"]:
        return
    _ST["last"] = objects
    _ST["step"] += 1
    s = objects["brush_tool"]
    if not all(np.all(np.isfinite(np.asarray(v, float))) for v in (s.pos, s.quat_xyzw)):
        _ST["zero"], _ST["why"] = True, "nonfinite state"
        return
    if not on_panel(s):
        _ST["zero"], _ST["why"] = True, "brush head left the floor panel"
    collecting = head_flat(s) and _clear(objects, HEAD_CLEAR, HEAD_LO, HEAD_HI)
    for i, xy in enumerate(LENTILS):
        if _ST["got"][i]:
            continue
        _ST["run"][i] = _ST["run"][i] + 1 if collecting and in_mouth(s, xy) else 0
        if _ST["run"][i] >= COLLECT_STEPS:
            _ST["got"][i] = True
    low = float(np.asarray(s.low, float)[2])
    set_down = (low <= FL + SET_DOWN and float(np.linalg.norm(np.asarray(s.lin_vel, float))) < STILL
                and _clear(objects, RELEASE_CLEAR))
    _ST["still_run"] = _ST["still_run"] + 1 if set_down else 0


def counts() -> dict:
    got = _ST["got"]
    out = {"total": sum(got)}
    for row in ("mouth", "mid", "corner"):
        out[row] = sum(g for g, r in zip(got, ROW_OF) if r == row)
    return out


def score(objects) -> float:
    """0 if the head ever leaves the floor panel, or unless the tool is set down (lowest point within 1 cm of the
    floor), still (< 5 mm/s) and both grippers 3 cm clear over the last 2 s. Otherwise by lentils collected:
    1.0 all 10; 0.75 at least 9 with at least 3 corner; 0.5 at least 7 with at least 2 corner; 0.25 at least 4 with at
    least 2 under the overhang (mid or corner); else 0."""
    _update(objects)
    if _ST["zero"] or _ST["still_run"] < FINAL_STEPS:
        return 0.0
    c = counts()
    if c["total"] == len(NAMES):
        return 1.0
    if c["total"] >= 9 and c["corner"] >= 3:
        return 0.75
    if c["total"] >= 7 and c["corner"] >= 2:
        return 0.5
    if c["total"] >= 4 and c["mid"] + c["corner"] >= 2:
        return 0.25
    return 0.0


def success(objects) -> bool:
    """Full score, and both grippers 10 cm clear of the tool."""
    return score(objects) >= 1.0 and _clear(objects, 0.10)


@on_step
def _lamps(env=None) -> None:
    """A collected lentil is gone: its lens turns floor coloured."""
    for i, n in enumerate(NAMES):
        set_lamp(KITCHEN, n, "off" if _ST["got"][i] else "bean")


# ---------------------------------------------------------------------------------------------------------------------
# Scripted demo (closedloop). The demo reads the live pose of the brush tool and its swivel angle from the simulator,
# plus its joint encoders and gripper opening; the lentil positions are the fixed layout (object_jitter=0).
demo = placeholder_demo

GRIP_Z = FL + CUFF_Z  # tool point on the cuff axis: fingertips 19 mm above the floor
HOVER_Z = TOP + 0.12
PARK_Z = TOP + 0.16  # 12 cm over the cuff; TOP+0.22 is out of reach at x 0.355
OPEN_Y = 0.150  # open-floor pass: head centre y (the open-floor lentils)
FLUSH_Y = KICK_Y - HEAD_V / 2  # 0.2085: a square head against the board
X_SQ0, X_NEAR, X_FAR = 0.240, 0.175, 0.355  # squaring stroke from X_SQ0 to X_NEAR, then the board run to X_FAR
REST_Y = 0.125  # the head is left here, on open floor
DRAG = 0.020  # m/s along the board
MOVE = 0.03  # m/s while repositioning the tool
APPROACH = 0.015  # m/s toward the board
PRELOAD = 0.003  # m: the head command leads the measured head into the board by this (about 0.7 N through the arm)
TILT_EASE = 2.0  # deg: if the head starts to tip onto its back bristle row, ease the push
HEADING_TOL = 5.0  # deg


def wand_dir(yaw: float) -> np.ndarray:
    """Unit vector from the cuff to the head for a tool turned by yaw."""
    return np.array([-math.sin(yaw), math.cos(yaw), 0.0])


def grip_point(head_xy, yaw: float) -> np.ndarray:
    d = wand_dir(yaw)
    return np.array([head_xy[0] + GRIP_LOCAL * d[0], head_xy[1] + GRIP_LOCAL * d[1], GRIP_Z])


def hand_yaw(tool_yaw: float) -> float:
    """Gripper yaw that closes the pads across the wand (yaw 0 closes them along world y); +90 deg keeps the wrist in range."""
    return tool_yaw + math.pi / 2


def _tool_yaw(ctx) -> float:
    q = np.asarray(ctx.obj("brush_tool").quat_xyzw, float)
    r = belimits._rot(q)
    return math.atan2(-r[0, 1], r[1, 1])


def _head_xy(ctx) -> np.ndarray:
    org, _, _, _ = head_frame(ctx.obj("brush_tool"))
    return org[:2]


def _carry(arm, path, speed):
    """Move the grip along a path of (head_xy, tool_yaw) keyframes so the head follows the planned lane."""
    for (h0, y0), (h1, y1) in zip(path[:-1], path[1:]):
        p0, p1 = grip_point(h0, y0), grip_point(h1, y1)
        dist = max(float(np.linalg.norm(p1 - p0)), 0.42 * abs(y1 - y0))
        steps = max(1, round(dist / speed / arm.dt))
        for i in range(1, steps + 1):
            t = i / steps
            hx = (1 - t) * np.asarray(h0) + t * np.asarray(h1)
            yw = (1 - t) * y0 + t * y1
            arm._set_cartesian(grip_point(hx, yw), grip_rot(yaw=hand_yaw(yw)), FINGER_CLOSED)
            yield


def _tilt_deg(s) -> float:
    _, _, _, n = head_frame(s)
    return math.degrees(math.acos(max(-1.0, min(1.0, float(n[2])))))


def _squared(s) -> bool:
    """An edge of the head lies along the kick board (long edge, or the short end with the head turned 90 deg)."""
    e = heading_error_deg(s)
    return e <= HEADING_TOL or e >= 90.0 - HEADING_TOL


def _gain(arm) -> float:
    """Integrator gain per step. Off in an offline dry run, where the head never moves and the offset would wind up."""
    return 0.0 if getattr(arm, "offline", False) else 0.05


def _board_drag(ctx, x0, x1, speed, cmd_y):
    """Push the head lightly into the kick board and draw it from x0 to x1 (the head's x). Each step the head's y
    command leads the live head by PRELOAD (rate limited, never past FLUSH_Y + PRELOAD) and eases off if the head
    tips; x waits until the head is on the board. The wand turns a few degrees in the pinch under the board's drag,
    so the grip's x carries an integrated offset that brings the live head, not the hand, to x1. Returns the last
    head y command."""
    R = ctx.right
    rot = grip_rot(yaw=hand_yaw(0.0))
    x, on_board, waited, done = x0, 0, 0, 0
    off = off0 = float(R.pos[0]) - x0
    while True:
        s = ctx.obj("brush_tool")
        h = head_frame(s)[0]
        hx, hy = float(h[0]), float(h[1])
        lead = PRELOAD if _tilt_deg(s) < TILT_EASE else -0.001
        want = min(hy + lead, FLUSH_Y + PRELOAD)
        cmd_y = float(np.clip(want, cmd_y - APPROACH * R.dt, cmd_y + APPROACH * R.dt))
        on_board = on_board + 1 if cmd_y - hy >= PRELOAD - 0.0005 or hy >= FLUSH_Y - 0.0005 else 0
        waited += 1
        if on_board >= 3 or waited > 200:
            step = speed * R.dt
            x = x + max(-step, min(step, x1 - x))
            if _gain(R):
                off = float(np.clip(off + np.clip(_gain(R) * (x - hx), -0.001, 0.001), min(-0.08, off0),
                                    max(0.08, off0)))
        g = grip_point((x, cmd_y), 0.0)
        g[0] += off
        R._set_cartesian(g, rot, FINGER_CLOSED)
        yield
        if abs(x - x1) < 1e-9:
            done += 1
            if abs(hx - x1) < 0.003 or done > 80:
                return cmd_y


def _steer(ctx, pts, speed):
    """Draw the live head along straight legs through pts (head xy) at speed, the wand pointing along +y. The grip
    carries an integrated xy offset so the head itself, not the hand, follows the path."""
    R = ctx.right
    rot = grip_rot(yaw=hand_yaw(0.0))
    tgt = np.asarray(head_frame(ctx.obj("brush_tool"))[0][:2], float)
    off = off0 = np.asarray(R.pos[:2], float) - grip_point(tgt, 0.0)[:2]
    for p in pts:
        p, done = np.asarray(p, float), 0
        while True:
            d = p - tgt
            dist = float(np.linalg.norm(d))
            step = speed * R.dt
            tgt = p.copy() if dist <= step else tgt + d / dist * step
            h = np.asarray(head_frame(ctx.obj("brush_tool"))[0][:2], float)
            if _gain(R):
                off = np.clip(off + np.clip(_gain(R) * (tgt - h), -0.001, 0.001), np.minimum(-0.08, off0),
                              np.maximum(0.08, off0))
            g = grip_point(tgt, 0.0)
            g[:2] += off
            R._set_cartesian(g, rot, FINGER_CLOSED)
            yield
            if dist <= step:
                done += 1
                if float(np.linalg.norm(h - p)) < 0.004 or done > 60:
                    break


def demo_script(ctx):
    R = ctx.right
    yaw0 = TOOL_YAW
    g0 = grip_point(TOOL_POS[:2], yaw0)
    rot0 = grip_rot(yaw=hand_yaw(yaw0))
    # 1) take the cuff from above, pads across it
    yield from R.move((g0[0], g0[1], HOVER_Z), 2.5, FINGER_OPEN, rot=rot0)
    yield from R.trim_to_measured((g0[0], g0[1], HOVER_Z))
    for _attempt in range(2):
        yield from R.move(g0, 2.0, FINGER_OPEN, rot=rot0)
        yield from R.grip(FINGER_CLOSED, 0.8)
        yield from R.hold(0.3)
        if R.holding(0.056):
            break
        yield from R.move((g0[0], g0[1], HOVER_Z), 1.0, FINGER_OPEN, rot=rot0)
    # 2) swing the wand to point straight at the cabinet, the head to the start of the squaring stroke
    yield from _carry(R, [((TOOL_POS[0], TOOL_POS[1]), yaw0), ((X_SQ0, OPEN_Y), 0.0)], MOVE)
    yield from R.hold(0.3)
    # 3) square the head on the kick board: push it in lightly and stroke it toward -x, where the board's friction on
    #    its leading corner turns it flat against the board (pushed straight in, a turned head wedges). Up to 3 tries.
    cmd_y = OPEN_Y
    for _try in range(3):
        cmd_y = yield from _board_drag(ctx, X_SQ0, X_NEAR, DRAG, cmd_y)
        if _squared(ctx.obj("brush_tool")) or _try == 2:
            break
        here = R.pos.copy()  # still turned: back off 15 mm, return to the stroke's start and try again
        yield from R.move((here[0], here[1] - 0.015, here[2]), 0.8, FINGER_CLOSED, rot=R.rot.copy())
        yield from R.move((here[0] + X_SQ0 - X_NEAR, here[1] - 0.015, here[2]), 2.5, FINGER_CLOSED, rot=R.rot.copy())
        cmd_y -= 0.015
    # 4) the board run: the square head drawn along +x flat against the board, through the back corner
    cmd_y = yield from _board_drag(ctx, X_NEAR, X_FAR, DRAG, cmd_y)
    yield from R.hold(0.3)
    # 5) out onto the open floor, back along -x over the lentils in front of the cabinet, and leave the head there
    yield from _steer(ctx, [(X_FAR, OPEN_Y), (X_NEAR, OPEN_Y), (X_NEAR, REST_Y)], MOVE)
    yield from R.hold(0.5)
    yield from R.grip(FINGER_OPEN, 0.8)
    here = R.pos.copy()
    yield from R.move((here[0], here[1], here[2] + 0.04), 1.0, FINGER_OPEN, rot=R.rot.copy())
    yield from R.move((here[0], here[1], PARK_Z), 2.0, FINGER_OPEN, rot=R.rot.copy())
    yield from R.home(3.0)
    yield from ctx.wait(3.0)
