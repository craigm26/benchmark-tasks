"""On the station table: an upright plywood backer at x 0.42 carries a grey 1-gang weatherproof box (Bell 5320-0 size)
fitted with a clear TayMac MM410C while-in-use cover, its face toward the robot. The cover's hinged hood is open
upward; a yellow 12/3 SJTW extension cord is plugged into the lower outlet and runs straight out of the front of the
open cover before dropping to the table and running off to the robot's left. Closing the hood now would land its
front wall on the cord. The cord has to be taken down through the U-shaped exit notch in the hood's bottom wall
(18 mm wide, left of centre) and the hood swung shut over it until its latch clicks.

Requires -E object_jitter=0 (fixed fixture; the cord is never jittered).

Run it:
    inspect-robots run --embodiment yam_arena -E task=ext_jobsite_temporary_power_spider_boxes_25.py -E object_jitter=0 \\
        --instruction "Route the plugged-in cord down through the exit notch in the bottom of the open in-use cover, then close the hood until it latches." \\
        --policy yam_scripted --scorer task_score --max-steps 2400
"""

from __future__ import annotations

import dataclasses
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np

import belimits  # noqa: E402, F401
import closedloop  # noqa: E402, F401
import mechkit  # noqa: E402
import ropekit  # noqa: E402
from closedloop import placeholder_demo  # noqa: E402
from mechkit import Joint, Mech, box, cone, cyl  # noqa: E402
from ropekit import EndBody, Rope, fillet, polyline_length  # noqa: E402

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 8.0
SCORE_OVER = "end"

# Offline testing override for mechkit.released (stays None in live runs)
FORCE_RELEASED: list = [None]

CLEAR = (0.70, 0.84, 0.94)  # clear polycarbonate, drawn light blue so it reads against the table
BASE_CLR = (0.80, 0.86, 0.90)
ALU = (0.52, 0.53, 0.56)  # cast aluminium box
IVORY = (0.93, 0.90, 0.80)  # receptacle face
BLACK = (0.10, 0.10, 0.11)
YELLOW = (0.96, 0.80, 0.06)
PLY = (0.78, 0.62, 0.42)
STEEL = (0.60, 0.60, 0.62)

# ---- Layout (station frame, m). X_F is the front face of the cover's base frame, where the hood's rim seats.
X_F = 0.355
Z_B = TOP + 0.100  # bottom of the cover (hood bottom wall underside) on the backer
COVER_W, COVER_H = 0.102, 0.140  # MM410C: 4 in wide, 5.5 in high
HOOD_D = 0.070  # 2-3/4 in deep hood
WALL = 0.003
BASE_T = 0.010  # base frame thickness between hood rim and box face
BOX_D, BOX_W, BOX_H = 0.051, 0.070, 0.114  # Bell 5320-0: 2 x 2-3/4 x 4-1/2 in
PLY_T, PLY_W, PLY_H = 0.019, 0.30, 0.30  # 3/4 in plywood backer
NOTCH_Y0, NOTCH_Y1 = 0.026, 0.044  # 18 mm exit notch, left of centre (world y)
NOTCH_U0 = -0.052  # notch runs from the hood's back rim (u = 0) forward to here
HINGE_U, HINGE_Z = -0.004, COVER_H  # hood hinge axis (along y) in cover coords (u = x - X_F, z from Z_B)
OPEN = 1.40  # hood authored open 80 deg; joint 0 = open, -OPEN = shut on its rim
HOOD_MASS = 0.09
HOOD_K = 0.5  # N m/rad detent spring (holds it open; snaps it shut past half way)

# Plug in the lower outlet of the duplex receptacle (outlets 38 mm apart about the box centre)
PLUG_Z = Z_B + COVER_H / 2 - 0.019
RECEPT_U = 0.006  # receptacle face, just inside the base frame
PLUG_LEN = 0.040  # moulded NEMA 5-15P body plus strain relief
CORD_R = 0.0055  # 12/3 SJTW, about 11 mm over the jacket
SEG = 0.02
CORD_LEN = 0.40
CORD_MASS = 0.08  # 0.2 kg/m
CORD_Z = TOP + CORD_R + 0.0008

# The hood's interior when shut (world): where the cord may run before it leaves through the notch
IN_X = (X_F - HOOD_D + WALL, X_F)
IN_Y = (-(COVER_W / 2 - WALL), COVER_W / 2 - WALL)
IN_Z = (Z_B + WALL, Z_B + COVER_H - WALL)
NOTCH_X = (X_F + NOTCH_U0, X_F)

LID_SHUT = 0.02  # rad: within this of the rim = latched
LID_NEAR = math.radians(10.0)  # within 10 deg the notch already straddles the cord

ARM_KEEP_OUT = [
    # The live plug and receptacle, grown by 4 cm (the plug spans x 0.321..0.361, y -0.014..0.014)
    {"name": "plug and receptacle, 4 cm around", "x": (0.281, 0.41), "y": (-0.054, 0.054), "z_top": TOP + 0.25},
]


def _rot_y(shape, centre, ang):
    """Rotate a mechkit Shape by ang about a +y axis through centre (x, z) of its own frame."""
    cx, cz = centre
    c, s = math.cos(ang), math.sin(ang)
    dx, dz = shape.at[0] - cx, shape.at[2] - cz
    w1, x1, y1, z1 = math.cos(ang / 2), 0.0, math.sin(ang / 2), 0.0
    w2, x2, y2, z2 = shape.tilt
    q = (w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
         w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2)
    return dataclasses.replace(shape, at=(cx + c * dx + s * dz, shape.at[1], cz - s * dx + c * dz), tilt=q)


def _look(shape):
    """A look-only part (no collider): outlet faces and plug blades, which sit inside each other."""
    return dataclasses.replace(shape, collide=False)


def _bx(u0, u1, y0, y1, z0, z1, color):
    return box(((u0 + u1) / 2, (y0 + y1) / 2, (z0 + z1) / 2), (u1 - u0, y1 - y0, z1 - z0), color)


def _hood_shapes_shut():
    """Hood walls in cover coords with the hood shut: front, top, two sides, and a bottom wall with the exit notch."""
    hw, d, h = COVER_W / 2, HOOD_D, COVER_H
    return [
        _bx(-d, -d + WALL, -hw, hw, 0.0, h, CLEAR),  # front
        _bx(-d, -0.006, -hw, hw, h - WALL, h, CLEAR),  # top (stops short of the hinge knuckle)
        _bx(-d, -0.001, -hw, -hw + WALL, 0.0, h - WALL, CLEAR),  # right side
        _bx(-d, -0.001, hw - WALL, hw, 0.0, h - WALL, CLEAR),  # left side
        _bx(-d + WALL, NOTCH_U0, -hw + WALL, hw - WALL, 0.0, WALL, CLEAR),  # bottom, ahead of the notch
        _bx(NOTCH_U0, -0.001, -hw + WALL, NOTCH_Y0, 0.0, WALL, CLEAR),  # bottom, right of the notch
        _bx(NOTCH_U0, -0.001, NOTCH_Y1, hw - WALL, 0.0, WALL, CLEAR),  # bottom, left of the notch
        _bx(-d - 0.006, -d, -0.010, 0.010, 0.0, 0.016, CLEAR),  # latch / padlock tab on the front bottom
        cyl((HINGE_U, 0.0, h), 0.004, COVER_W - 0.004, BASE_CLR, axis="y", sides=12),  # hinge knuckle
    ]


def _build_cover() -> Mech:
    hw = COVER_W / 2
    win_w, win_z0, win_z1 = BOX_W / 2, 0.013, COVER_H - 0.013
    base = [
        # base frame round the receptacle window
        _bx(0.0, BASE_T, -hw, -win_w, 0.0, COVER_H, BASE_CLR),
        _bx(0.0, BASE_T, win_w, hw, 0.0, COVER_H, BASE_CLR),
        _bx(0.0, BASE_T, -win_w, win_w, 0.0, win_z0, BASE_CLR),
        _bx(0.0, BASE_T, -win_w, win_w, win_z1, COVER_H, BASE_CLR),
        # cast aluminium box behind it
        _bx(BASE_T, BASE_T + BOX_D, -BOX_W / 2, BOX_W / 2, COVER_H / 2 - BOX_H / 2, COVER_H / 2 + BOX_H / 2, ALU),
        # duplex receptacle face in the window, two raised outlet faces
        _bx(RECEPT_U, BASE_T, -0.034, 0.034, win_z0, win_z1, IVORY),
        _look(cyl((RECEPT_U - 0.002, 0.0, COVER_H / 2 + 0.019), 0.017, 0.004, IVORY, axis="x", sides=16)),
        _look(cyl((RECEPT_U - 0.002, 0.0, COVER_H / 2 - 0.019), 0.017, 0.004, IVORY, axis="x", sides=16)),
    ]
    hood = [_rot_y(s, (HINGE_U, HINGE_Z), OPEN) for s in _hood_shapes_shut()]
    hood = [dataclasses.replace(s, at=(s.at[0] - HINGE_U, s.at[1], s.at[2] - HINGE_Z)) for s in hood]
    return Mech(
        "cover",
        pos=(X_F, 0.0, Z_B),
        mass=None,  # screwed to the backer
        shapes=base,
        joints=[
            Joint("hood", "hinge", axis="y", at=(HINGE_U, 0.0, HINGE_Z), low=-OPEN - 0.01, high=0.02,
                  mass=HOOD_MASS, spring=HOOD_K, damping=0.08, detents=(-OPEN, 0.0), shapes=hood),
        ],
    )


def _cord_path():
    x0 = X_F + RECEPT_U - PLUG_LEN  # rope starts at the back of the plug
    pts = [
        (x0, 0.0, PLUG_Z),
        (X_F - HOOD_D - 0.020, 0.0, PLUG_Z),  # straight out past the open hood's front plane
        (0.262, 0.090, CORD_Z),  # drops to the table, left of the cover
        (0.262, 0.175, CORD_Z),
        (0.120, 0.215, CORD_Z),
    ]
    path = fillet(pts, 0.03)
    return path


def _build_cord() -> Rope:
    plug = EndBody(
        mass=0.06,
        shapes=[
            cone((0.006, 0.0, 0.0), 0.0085, 0.011, 0.012, BLACK, axis="x", sides=12),  # strain relief
            box((0.026, 0.0, 0.0), (0.028, 0.028, 0.036), BLACK),  # moulded plug body
            _look(box((0.044, 0.006, 0.0), (0.008, 0.002, 0.006), STEEL)),  # blades, inside the receptacle
            _look(box((0.044, -0.006, 0.0), (0.008, 0.002, 0.006), STEEL)),
        ],
    )
    return Rope("cord", path=_cord_path(), length=CORD_LEN, segment=SEG, radius=CORD_R, rope_mass=CORD_MASS,
                bend_stiffness=0.012, bend_damping=0.002, collision_sides=8, anchor_start=True,
                start_body=plug, color=YELLOW)


COVER = _build_cover()
CORD = _build_cord()
BACKER = Box("backer", pos=(X_F + BASE_T + BOX_D + PLY_T / 2, 0.0, TOP + PLY_H / 2), size=(PLY_T, PLY_W, PLY_H),
             color=PLY)
OBJECTS = [BACKER, COVER, CORD]


# ---------------------------------------------------------------- scoring

def _released(part) -> bool:
    if FORCE_RELEASED[0] is not None:
        return bool(FORCE_RELEASED[0])
    return bool(mechkit.released(part, clearance=0.03))


def hood_angle(objects) -> float:
    """How far the hood is open from its rim (rad, 0 = shut)."""
    return float(OPEN + objects["cover"].joints["hood"])


def _inside(p) -> bool:
    return IN_X[0] <= p[0] <= IN_X[1] and IN_Y[0] <= p[1] <= IN_Y[1] and IN_Z[0] <= p[2] <= IN_Z[1]


def routing(pts) -> dict:
    """Follow the cord from the plug: it must stay inside the shut hood's volume until it first leaves it, leave it
    downward through the bottom plane (z = Z_B) inside the notch, and never come back in."""
    pts = np.asarray(pts, dtype=float)
    out = {"routed": False, "exit": None, "why": ""}
    if not _inside(pts[0]):
        out["why"] = "cord start not inside the cover"
        return out
    k = next((i for i in range(len(pts)) if not _inside(pts[i])), None)
    if k is None:
        out["why"] = "cord never leaves the cover"
        return out
    a, b = pts[k - 1], pts[k]
    if b[2] >= Z_B:
        out["why"] = "cord leaves through the front or a side, not the bottom"
        out["exit"] = tuple(round(float(v), 4) for v in b)
        return out
    t = (a[2] - Z_B) / max(a[2] - b[2], 1e-9)
    c = a + t * (b - a)
    out["exit"] = tuple(round(float(v), 4) for v in c)
    if not (NOTCH_X[0] <= c[0] <= NOTCH_X[1] and NOTCH_Y0 <= c[1] <= NOTCH_Y1):
        out["why"] = "cord crosses the bottom outside the notch"
        return out
    if any(_inside(p) for p in pts[k:]):
        out["why"] = "cord comes back into the cover"
        return out
    out["routed"] = True
    return out


def evaluate(objects) -> dict:
    cord = objects["cord"]
    pts = np.asarray(cord.points, dtype=float)
    r = routing(pts)
    ang = hood_angle(objects)
    intact = float(cord.measured_length()) < 1.05 * CORD_LEN
    return {"routed": r["routed"], "why": r["why"], "exit": r["exit"], "hood_deg": round(math.degrees(ang), 2),
            "shut": ang <= LID_SHUT, "near": ang <= LID_NEAR, "intact": intact}


def score(objects) -> float:
    """1.0: cord through the notch, hood shut on its rim (within 0.02 rad) and both hands 3 cm clear of the cover.
    0.5: cord through the notch with the hood within 10 deg of shut. 0.25: cord through the notch, hood further open,
    hands clear of cord and cover. 0 otherwise (cord out the front or a side, crossing outside the notch, pulled
    apart)."""
    e = evaluate(objects)
    if not (e["routed"] and e["intact"]):
        return 0.0
    if e["shut"] and _released(objects["cover"]):
        return 1.0
    if e["near"]:
        return 0.5
    if _released(objects["cover"]) and _released(objects["cord"]):
        return 0.25
    return 0.0


def success(objects) -> bool:
    return score(objects) == 1.0


# ---------------------------------------------------------------- scripted demo
# The demo reads the cord's live segment positions (to find where to grip it and where it crosses the notch plane)
# and the hood's hinge angle from the simulator.

demo = placeholder_demo

GRIP_S = 0.205  # arclength from the plug where the left hand takes the cord (where it reaches the table)
HOLD = np.array([0.315, 0.135, TOP + 0.030])  # left hand's hold: pulls the cord down-left through the notch
NOTCH_AIM = np.array([X_F - 0.036, 0.5 * (NOTCH_Y0 + NOTCH_Y1)])
PUSH_Y = -0.026  # right hand pushes the hood's right half
TIP = 0.013  # closed fingertips reach this far past the tool point
HINGE_W = np.array([X_F + HINGE_U, Z_B + HINGE_Z])  # hinge axis (x, z) in the world
PUSH_C = np.array([-(HOOD_D + HINGE_U), -0.110])  # push point on the hood front, shut, relative to the hinge


def _cord_pts(ctx):
    try:
        return np.asarray(ctx.obj("cord").points, dtype=float)
    except Exception:  # noqa: BLE001
        return np.asarray(CORD.layout.points if hasattr(CORD.layout, "points") else _cord_path(), dtype=float)


def _point_at(pts, s):
    seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    cum = np.r_[0.0, np.cumsum(seg)]
    s = float(np.clip(s - SEG / 2, 0.0, cum[-1]))  # points are segment centres
    i = int(np.clip(np.searchsorted(cum, s) - 1, 0, len(seg) - 1))
    t = (s - cum[i]) / max(seg[i], 1e-9)
    return pts[i] + t * (pts[i + 1] - pts[i]), (pts[i + 1] - pts[i]) / max(seg[i], 1e-9)


def _hood_open(ctx) -> float:
    try:
        return float(OPEN + ctx.obj("cover").joints["hood"])
    except Exception:  # noqa: BLE001
        return OPEN


def _push_tip(theta, inset):
    """Fingertip target on the hood's front face at hood angle theta, pressed inset metres into it."""
    c, s = math.cos(theta), math.sin(theta)
    p = HINGE_W + np.array([PUSH_C[0] * c + PUSH_C[1] * s, -PUSH_C[0] * s + PUSH_C[1] * c])
    n = np.array([-c, s])  # outward normal of the front face
    return p - inset * n


def _crossing(pts):
    for a, b in zip(pts[:-1], pts[1:]):
        if a[2] >= Z_B > b[2]:
            t = (a[2] - Z_B) / (a[2] - b[2])
            return a + t * (b - a)
    return None


LYAW, LPITCH = -math.pi / 2, 0.45  # left pads close along x, across the cord; hand leaned toward -y keeps the wrist off the hood


def _left(ctx):
    L = ctx.left
    kw = {"yaw": LYAW, "pitch": LPITCH}
    g, _ = _point_at(_cord_pts(ctx), GRIP_S)
    gz = max(float(g[2]) + 0.010, TOP + 0.022)
    yield from L.move((g[0], g[1], TOP + 0.10), 2.0, FINGER_OPEN, **kw)
    yield from L.move((g[0], g[1], gz + 0.03), 0.8, **kw)
    yield from L.move((g[0], g[1], gz), 0.6, **kw)
    yield from L.grip(FINGER_CLOSED, 0.6)
    yield from L.hold(0.3)
    if not L.holding(0.006):  # missed: open, re-read the cord and take it again
        yield from L.move((g[0], g[1], gz + 0.04), 0.6, FINGER_OPEN, **kw)
        g, _ = _point_at(_cord_pts(ctx), GRIP_S)
        gz = max(float(g[2]) + 0.010, TOP + 0.022)
        yield from L.move((g[0], g[1], gz), 0.6, **kw)
        yield from L.grip(FINGER_CLOSED, 0.6)
        yield from L.hold(0.3)
    # Carry the cord back and down: the run from the plug now drops behind the hood's front plane, through the notch
    yield from L.move((g[0], g[1], TOP + 0.05), 0.6, **kw)
    yield from L.move((HOLD[0], HOLD[1], TOP + 0.05), 1.6, **kw)
    yield from L.move(HOLD, 0.8, **kw)
    hold = HOLD.copy()
    # Steer the hold until the cord crosses the notch plane at the notch centre (live cord; offline it never moves)
    for _ in range(0 if L.offline else 4):
        yield from L.hold(0.5)
        c = _crossing(_cord_pts(ctx))
        if c is None:
            break
        ex, ey = NOTCH_AIM[0] - c[0], NOTCH_AIM[1] - c[1]
        if abs(ex) < 0.004 and abs(ey) < 0.003:
            break
        hold = hold + np.array([np.clip(1.5 * ex, -0.012, 0.012), np.clip(-2.0 * ey, -0.012, 0.012), 0.0])
        hold[0] = float(np.clip(hold[0], 0.30, 0.335))
        hold[1] = float(np.clip(hold[1], 0.11, 0.16))
        yield from L.move(hold, 0.6, **kw)
    ctx.left_ready = True
    # Keep the cord in the notch until the hood is shut
    for _ in range(int(14.0 / L.dt)):
        if getattr(ctx, "hood_done", False) or _hood_open(ctx) <= LID_SHUT:
            break
        yield
    yield from L.hold(0.5)
    yield from L.grip(FINGER_OPEN, 0.8)
    yield from L.move((hold[0] - 0.03, hold[1] + 0.04, TOP + 0.16), 1.5, **kw)
    yield from L.home(3.0)


PUSH_PITCH = 0.8  # fingers lean 46 deg toward the cover: the push runs along the hood's swing


def _tcp(tip, dx=0.0, dz=0.0):
    """Tool point that puts the closed fingertips at tip (x, z), with the hand leaned PUSH_PITCH toward +x."""
    a = np.array([math.sin(PUSH_PITCH), -math.cos(PUSH_PITCH)])
    q = np.asarray(tip, dtype=float) - TIP * a
    return (float(q[0] + dx), PUSH_Y, float(q[1] + dz))


def _right(ctx):
    R = ctx.right
    P = PUSH_PITCH
    # Wait until the left hand has the cord in the notch
    for _ in range(int(25.0 / R.dt)):
        if getattr(ctx, "left_ready", False):
            break
        yield
    th = _hood_open(ctx)
    yield from R.move(_tcp(_push_tip(th, -0.04), dx=-0.02), 2.5, FINGER_CLOSED, pitch=P)
    yield from R.move(_tcp(_push_tip(th, -0.005)), 0.8, pitch=P)
    # Swing the hood down by its front face, the tips kept 6 mm into the face, to past half shut
    end = math.radians(25.0)
    n = 40
    for i in range(1, n + 1):
        a = th + (end - th) * i / n
        yield from R.move(_tcp(_push_tip(a, 0.006)), 0.12, pitch=P)
    yield from R.hold(0.4)
    # The detent spring and gravity carry it onto its rim; back away toward the robot
    yield from R.move(_tcp(_push_tip(end, -0.04), dx=-0.02), 0.8, pitch=P)
    for _ in range(int(3.0 / R.dt)):
        if _hood_open(ctx) <= LID_SHUT:
            break
        yield
    if _hood_open(ctx) > LID_SHUT:
        # Not latched: press the hood's front face onto its rim once more
        th = max(_hood_open(ctx), math.radians(8.0))
        yield from R.move(_tcp(_push_tip(th, -0.02)), 1.0, pitch=P)
        low = math.radians(8.0)  # from here the detent spring seats it; the hand stays 4 cm off the plug
        for i in range(1, 16):
            a = th + (low - th) * i / 15
            yield from R.move(_tcp(_push_tip(a, 0.005)), 0.12, pitch=P)
        yield from R.hold(0.5)
        yield from R.move(_tcp(_push_tip(low, -0.04), dx=-0.02), 0.8, pitch=P)
    ctx.hood_done = True
    yield from R.move((0.20, -0.12, TOP + 0.22), 1.2)
    yield from R.home(3.0)


def demo_script(ctx):
    ctx.left_ready = False
    ctx.hood_done = False
    yield from ctx.both(_left(ctx), _right(ctx))
    yield from ctx.wait(1.0)
