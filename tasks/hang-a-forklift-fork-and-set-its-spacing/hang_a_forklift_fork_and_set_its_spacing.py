"""On the station table, in front of the right arm: a half-scale forklift carriage (a grey top bar with a dark notched
rib, a lower bar and a dark back plate) faces the arms. One yellow fork already hangs on it, locked by its red pin in a
notch. A second yellow fork stands loose on the table in front of the carriage: it has a green grip tab on top and a red
lock-pin knob behind the tab. In front of the forks lies a blue pallet with two openings; the hanging fork's tine points
into one of them. The lower bar has a strip above it that is cut away over 7 cm (the removal gap): a fork's bottom hook
only goes down onto the lower bar there. Hung at the gap, the fork slides along the bars; its pin drops into a notch.

Requires -E object_jitter=0.

Run it:  inspect-robots run --embodiment yam_arena -E task=hang_a_forklift_fork_and_set_its_spacing.py \\
           -E object_jitter=0 --instruction "Fit the loose fork to the forklift carriage, spaced for the pallet." \\
           --scorer task_score ...
"""

import os, sys; sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))  # noqa: E402,E702

import math

import numpy as np
from belimits import tilt_deg
from closedloop import grip_rot, placeholder_demo
from mechkit import Joint, Mech, box
from yam_arena.constants import FINGER_OPEN
from yam_arena.task_file import TABLE_TOP

SCENE = "station"
TOP = TABLE_TOP[SCENE]

GREY, DARK, PLATE = (0.62, 0.64, 0.68), (0.10, 0.10, 0.12), (0.26, 0.28, 0.32)
YELLOW, GREEN, RED, PIN_RED = (1.00, 0.84, 0.02), (0.05, 0.62, 0.20), (0.92, 0.16, 0.08), (0.55, 0.05, 0.05)
BLUE = (0.12, 0.30, 0.78)

# --- geometry (metres). Every Mech below is written in station x, y and height above the table: the carriage and the
#     pallet sit at pos (0, 0, TOP); a fork's shapes are written for a fork hung at y 0, and its pos shifts it. --------
Y_GAP, Y_TARGET, Y_REF = -0.08, -0.18, 0.02  # removal gap; the notch the pallet asks for; the fork already fitted
NOTCHES = (-0.18, -0.14, -0.08, -0.02, 0.02)  # notch centres along the rib
NOTCH_W = 0.016
BAR_Y = (-0.22, 0.06)
BAR_X = (0.33, 0.354)  # the top bar; both bars' front face is x 0.33
RAIL = 0.003  # the rib's two outer rails run the whole length; the notches are pockets between them
BLOCK_DROP = 0.0015  # the blocks between the notches stand this much lower than the rails, so a sliding fork rides
#                     on the rails alone and its edge cannot catch a notch wall
RIB_TOP, NOTCH_FLOOR = 0.115, 0.105  # the rib is the top 1 cm of the top bar
STRIP_X1, STRIP_TOP = 0.34, 0.04  # the lower bar's front strip: x 0.33..0.34, up to 4 cm
KEEPER_Z = (0.058, 0.068)  # the strip above the bottom hook, cut away at the gap
GAP_HALF = 0.035
PLATE_X = (0.395, 0.405)  # back plate
PLATE_TOP = 0.125
FORK_HALF_W = 0.015


def _span(x, y, z, color):
    return box(((x[0] + x[1]) / 2, (y[0] + y[1]) / 2, (z[0] + z[1]) / 2), (x[1] - x[0], y[1] - y[0], z[1] - z[0]), color)


def _rib():
    edges, y = [], BAR_Y[0]
    for n in NOTCHES:
        edges.append((y, n - NOTCH_W / 2))
        y = n + NOTCH_W / 2
    edges.append((y, BAR_Y[1]))
    inner = (BAR_X[0] + RAIL, BAR_X[1] - RAIL)
    return [
        _span((BAR_X[0], inner[0]), BAR_Y, (NOTCH_FLOOR, RIB_TOP), DARK),
        _span((inner[1], BAR_X[1]), BAR_Y, (NOTCH_FLOOR, RIB_TOP), DARK),
        *[_span(inner, e, (NOTCH_FLOOR, RIB_TOP - BLOCK_DROP), DARK) for e in edges],
    ]


CARRIAGE = Mech(
    "carriage", pos=(0.0, 0.0, TOP), mass=None, friction=0.2,
    shapes=[
        _span(BAR_X, BAR_Y, (0.085, NOTCH_FLOOR), GREY),  # top bar
        *_rib(),
        _span((BAR_X[0], STRIP_X1), BAR_Y, (0.0, STRIP_TOP), GREY),  # lower bar, front strip
        _span((STRIP_X1, PLATE_X[0]), (BAR_Y[0], Y_GAP - GAP_HALF), KEEPER_Z, GREY),  # keeper strip, right of the gap
        _span((STRIP_X1, PLATE_X[0]), (Y_GAP + GAP_HALF, BAR_Y[1]), KEEPER_Z, GREY),  # keeper strip, left of the gap
        _span(PLATE_X, (BAR_Y[0] - 0.005, BAR_Y[1] + 0.005), (0.0, PLATE_TOP), PLATE),  # back plate
        _span((BAR_X[0], PLATE_X[0]), (BAR_Y[0] - 0.005, BAR_Y[0]), (0.0, PLATE_TOP), PLATE),  # end plates
        _span((BAR_X[0], PLATE_X[0]), (BAR_Y[1], BAR_Y[1] + 0.005), (0.0, PLATE_TOP), PLATE),
    ],
)

# A fork. Hung: its top plate rests on the rib, the back lip hangs behind the top bar, the shank lies against the bars'
# front faces, the bottom hook reaches back over the front strip and under the keeper strip, its lip behind the front strip.
PIN_X = 0.343
PIN_AT = (PIN_X, 0.0, NOTCH_FLOOR + 0.0005)  # pin bottom at joint value 0: half a millimetre above a notch floor
PIN_UP = 0.015  # the pin's upper stop
ON_RIB = RIB_TOP - BLOCK_DROP - PIN_AT[2]  # joint value with the pin standing on the rib between notches (0.008)
TAB_X, TAB_Z = (0.281, 0.299), (0.125, 0.160)
KNOB_Z = (0.030, 0.060)  # in the pin's frame
HOOK = (0.342, 0.0, RIB_TOP)  # the top plate's underside over the rib's centre line
LIP_TOP = (0.357, 0.0, 0.1075)  # front face of the back lip
LIP_BOTTOM = (0.344, 0.0, 0.037)  # front face of the bottom hook's lip
FORK_MASS = 0.15


def fork(name, x, y, z):
    w = (-FORK_HALF_W, FORK_HALF_W)
    return Mech(
        name, pos=(x, y, TOP + z), mass=FORK_MASS, friction=0.3,
        shapes=[
            _span((0.315, 0.329), w, (0.010, 0.125), YELLOW),  # shank
            _span((0.235, 0.315), w, (0.010, 0.025), YELLOW),  # tine
            _span((0.275, 0.362), w, (RIB_TOP, 0.125), YELLOW),  # top plate
            _span((0.357, 0.362), w, (0.100, RIB_TOP), YELLOW),  # back lip, 3 mm behind the top bar
            _span((0.329, 0.349), w, (0.043, 0.055), YELLOW),  # bottom hook: 3 mm over the front strip, 3 mm under the keeper
            _span((0.344, 0.349), w, (0.031, 0.043), YELLOW),  # its lip, 4 mm behind the front strip
            _span(TAB_X, w, TAB_Z, GREEN),  # grip tab
        ],
        joints=[
            Joint("pin", "slide", axis="z", at=PIN_AT, low=0.0, high=PIN_UP, spring=0.0, damping=5.0, mass=0.01, friction=0.8,
                  shapes=[
                      box((0.0, 0.0, 0.015), (0.010, 0.010, 0.030), PIN_RED),
                      box((0.0, 0.0, (KNOB_Z[0] + KNOB_Z[1]) / 2), (0.018, 0.030, KNOB_Z[1] - KNOB_Z[0]), RED),
                  ]),
        ],
    )


START_DX, START_Y, START_Z = -0.035, -0.13, -0.0095  # standing on its tine, 3.5 cm in front of where it would hang
NEW_FORK = fork("new_fork", START_DX, START_Y, START_Z)
REF_FORK = fork("ref_fork", 0.0, Y_REF, 0.0)

# The pallet: a slice of a two-opening pallet lying in front of the tines. Openings 6 cm wide, 4 cm high, centred on the
# reference fork's notch and on the target notch.
PAL_X = (0.15, 0.19)
PAL_BLOCKS = ((-0.25, -0.21), (-0.15, -0.01), (0.05, 0.09))
PALLET = Mech(
    "pallet", pos=(0.0, 0.0, TOP), mass=None,
    shapes=[
        _span(PAL_X, (-0.25, 0.09), (0.0, 0.01), BLUE),
        _span(PAL_X, (-0.25, 0.09), (0.05, 0.06), BLUE),
        *[_span(PAL_X, b, (0.01, 0.05), BLUE) for b in PAL_BLOCKS],
    ],
)

OBJECTS = [CARRIAGE, PALLET, REF_FORK, NEW_FORK]

SUCCESS_HOLD_S = 8.0  # the demo grips: keep the run going so the release and the retreat are on video

# --- scoring ----------------------------------------------------------------------------------------------------------
SEAT_TOL = 0.003  # m: the top plate's underside within this of the rib's top
Y_TOL = 0.004  # m: fork centre within this of a notch centre (the pin has 3 mm of play a side)
PIN_DOWN = 0.003  # m: joint value at or below this means the pin is down in a notch
CAPTIVE = 0.05  # m from the gap centre: the whole bottom hook is under the keeper strip


def _hung(s) -> bool:
    """The fork hangs on the carriage: top plate on the rib (so it is not held up: more than 3 mm above the rib fails),
    back lip behind the top bar, bottom hook's lip behind the front strip, upright, within the bar's length."""
    under = [s.to_world((x, y, RIB_TOP)) for x in (BAR_X[0] + 0.002, BAR_X[1] - 0.002) for y in (-0.013, 0.013)]
    if any(abs(p[2] - (TOP + RIB_TOP)) > SEAT_TOL for p in under):
        return False
    if tilt_deg(s) > 5.0:
        return False
    y = float(s.to_world(HOOK)[1])
    if not BAR_Y[0] + FORK_HALF_W - 0.002 <= y <= BAR_Y[1] - FORK_HALF_W + 0.002:
        return False
    return bool(s.to_world(LIP_TOP)[0] >= BAR_X[1] - 0.0005 and s.to_world(LIP_BOTTOM)[0] >= STRIP_X1 - 0.0005)


def _y(s) -> float:
    return float(s.to_world(HOOK)[1])


def _pin_down(s) -> bool:
    return float(s.joints["pin"]) <= PIN_DOWN


def _locked_at(s, y_notch) -> bool:
    return _hung(s) and _pin_down(s) and abs(_y(s) - y_notch) <= Y_TOL


def score(objects):
    """0 unless the reference fork is still locked in its notch and the new fork hangs on the carriage. Then 1.0 with its
    pin down in the target notch; 0 with its pin down in any other notch but the gap's; 0.5 hung 5 cm or more from the
    gap centre (captive under the keeper strip); else 0.25 (hung at the gap)."""
    new, ref = objects["new_fork"], objects["ref_fork"]
    if not _locked_at(ref, Y_REF) or not _hung(new):
        return 0.0
    y = _y(new)
    if _pin_down(new):
        if abs(y - Y_TARGET) <= Y_TOL:
            return 1.0
        if any(abs(y - n) <= Y_TOL for n in NOTCHES if n not in (Y_TARGET, Y_GAP)):
            return 0.0
    return 0.5 if abs(y - Y_GAP) >= CAPTIVE else 0.25


def success(objects):
    return score(objects) == 1.0 and objects["new_fork"].resting() and objects["ref_fork"].resting()


# The robot itself keeps out of the space behind the back plate, where a truck's mast rails and lift chains run (checked
# offline with the URDF, and by the gate's envelope; the scorer sees only the objects).
ARM_KEEP_OUT = [
    {"name": "behind the carriage's back plate", "x": (PLATE_X[0], 0.45), "y": (BAR_Y[0] - 0.005, BAR_Y[1] + 0.005),
     "z_top": TOP + 0.25},
]

# --- the demo ---------------------------------------------------------------------------------------------------------
demo = placeholder_demo

YAW_TAB = math.pi / 2  # fingers close along x on the tab's 1.8 cm thickness, so the fork cannot swing in the grip
TAB_GRIP = ((TAB_X[0] + TAB_X[1]) / 2, 0.0, 0.145)  # in the fork's frame: pad tips 7 mm above the top plate
TAB_OPEN, TAB_SHUT = -0.017, -0.004  # pads 3.4 cm apart to approach; squeezed 5 mm a side on the 1.8 cm tab
KNOB_GRIP_Z = KNOB_Z[0] + 0.017  # in the pin's frame: pad tips 4 mm above the knob's foot
KNOB_OPEN, KNOB_SHUT = -0.026, -0.012  # pads 5.2 cm apart to approach; squeezed 3 mm a side on the 3 cm knob
UP = TOP + 0.185  # travel height: as high as a straight-down grip reaches here
CARRY_DZ = 0.03  # the fork is carried 3 cm above its hung height: lips 1.5 cm clear of the rib


def _mask(v, axes):
    return np.array([v[i] if i in axes else 0.0 for i in range(3)])


def _offline(ctx) -> bool:
    return ctx.states() is ctx.states()


class Riding:
    """Where a point of a fork is. An offline replay (dry run, arm sweep, gate check) hands back the same fixed start states
    on every read and never moves an object, so there the fork is taken to ride with the hand."""

    def __init__(self, ctx, arm, name, local, nominal=None):
        self.ctx, self.arm, self.name, self.local = ctx, arm, name, local
        self.offline = _offline(ctx)
        self.p0 = np.array(nominal, float) if self.offline and nominal is not None else self._read()
        self.h0 = np.array(arm.pos, float)

    def _read(self):
        return np.array(self.ctx.obj(self.name).to_world(self.local), float)

    def __call__(self):
        return self.p0 + (np.array(self.arm.pos, float) - self.h0) if self.offline else self._read()


def _drive(arm, where, goal, speed, *, tol=0.0006, max_s=8.0, axes=(0, 1, 2), finger=None, yaw=0.0):
    """Move the hand until the measured point ``where()`` is within ``tol`` of ``goal`` on ``axes`` (three steps running)."""
    goal = np.asarray(goal, float)
    ok = [0]
    lead0 = _mask(np.array(arm.pos, float) - where(), axes)

    def target():
        p = where()
        e = _mask(goal - p, axes)
        ok[0] = ok[0] + 1 if np.linalg.norm(e) < tol else 0
        lead = _mask(np.array(arm.pos, float) - p, axes) - lead0
        if np.linalg.norm(lead) > 0.006 and float(lead @ e) > 0.0:
            return np.array(arm.pos, float)  # the part is not following (stalled): do not wind the hand up against it
        return np.array(arm.pos, float) + 0.3 * e

    for _ in arm.servo(target, max_s, finger, yaw=yaw, max_speed=speed):
        if ok[0] >= 3:
            return
        yield


def demo_script(ctx):
    """Right arm only; the left arm stays at its ready pose. Take the loose fork by its tab, lift it 4 cm, carry it in
    front of the bars to the removal gap, move it in over the bars and lower it until its top plate is on the rib and its
    bottom hook is down through the gap; let go. Take the pin's knob, raise the pin to its stop, slide the fork along the
    bars to the notch in line with the pallet's free opening, lower the pin into the notch, let go and park. Every step
    is closed on the fork's measured position."""
    R = ctx.right
    fork0 = ctx.obj("new_fork")
    grip = np.array(fork0.to_world(TAB_GRIP), float)

    # -- take the tab
    yield from R.move((grip[0], grip[1], UP), 3.0, TAB_OPEN, yaw=YAW_TAB)
    yield from R.move(grip + np.array([0.0, 0.0, 0.02]), 1.5, TAB_OPEN, yaw=YAW_TAB)
    yield from R.move(grip, 1.0, TAB_OPEN, yaw=YAW_TAB)
    yield from R.trim_to_measured(grip, tol=0.001, max_duration=2.0)
    yield from R.grip(TAB_SHUT, 0.8)
    yield from ctx.wait(0.3)

    # -- lift, carry in front of the bars to the gap, move in over the bars, lower onto the rib
    hook = Riding(ctx, R, "new_fork", HOOK)
    h = hook()
    front_x = h[0]  # 3.5 cm in front of the hung position
    hung = np.array([HOOK[0] - 0.0015, Y_GAP, TOP + RIB_TOP])  # 1.5 mm proud: shank 2.5 mm off the bars, lips 1.5 mm behind
    high = TOP + RIB_TOP + CARRY_DZ
    yield from _drive(R, hook, (front_x, h[1], high), 0.03, tol=0.0015, finger=TAB_SHUT, yaw=YAW_TAB)
    yield from _drive(R, hook, (front_x, Y_GAP, high), 0.03, tol=0.001, finger=TAB_SHUT, yaw=YAW_TAB)
    yield from _drive(R, hook, (hung[0], Y_GAP, high), 0.02, tol=0.0006, finger=TAB_SHUT, yaw=YAW_TAB)
    yield from ctx.wait(0.4)
    yield from _drive(R, hook, (hung[0], Y_GAP, high), 0.02, tol=0.0006, max_s=3.0, finger=TAB_SHUT, yaw=YAW_TAB)
    # down at 0.5 cm/s, holding x and y, until the plate is within 0.4 mm of the rib (it cannot go lower)
    yield from _drive(R, hook, hung + np.array([0.0, 0.0, 0.0002]), 0.005, tol=0.0004, max_s=9.0, finger=TAB_SHUT, yaw=YAW_TAB)
    if not hook.offline and hook()[2] - hung[2] > 0.0015:  # not seated (a hook caught): lift 1.2 cm, re-centre, lower again
        yield from _drive(R, hook, hung + np.array([0.0, 0.0, 0.012]), 0.01, tol=0.0006, max_s=4.0, finger=TAB_SHUT, yaw=YAW_TAB)
        yield from _drive(R, hook, hung + np.array([0.0, 0.0, 0.0002]), 0.005, tol=0.0004, max_s=6.0, finger=TAB_SHUT, yaw=YAW_TAB)
    yield from R.grip(TAB_OPEN, 0.6)
    yield from R.move(np.array(R.pos, float) + np.array([0.0, 0.0, 0.035]), 1.0, TAB_OPEN, yaw=YAW_TAB)

    # -- take the pin's knob
    def knob():
        if _offline(ctx):  # where the hung fork's knob is by design
            return np.array([hung[0] + PIN_X - HOOK[0], Y_GAP, TOP + PIN_AT[2] + KNOB_GRIP_Z])
        s = ctx.obj("new_fork")
        return np.array(s.to_world((PIN_AT[0], 0.0, PIN_AT[2] + float(s.joints["pin"]) + KNOB_GRIP_Z)), float)

    # A straight-down grip cannot reach high enough to come down onto the knob here, so the hand turns over the tab (fingers
    # still only 3.4 cm apart, clear of the knob), opens along y, comes down astride the tab and moves back onto the knob.
    k = knob()
    over_tab = np.array([R.pos[0], k[1], R.pos[2]])
    yield from R.move(over_tab, 1.5, TAB_OPEN, yaw=0.0)
    yield from R.grip(KNOB_OPEN, 0.5)
    k = knob()
    yield from R.move((over_tab[0], k[1], k[2] + 0.004), 1.0, KNOB_OPEN, yaw=0.0)
    yield from R.move((k[0], k[1], k[2] + 0.004), 1.5, KNOB_OPEN, yaw=0.0)
    yield from R.move(k, 0.6, KNOB_OPEN, yaw=0.0)
    yield from R.trim_to_measured(k, tol=0.001, max_duration=2.0)
    yield from R.grip(KNOB_SHUT, 0.8)
    yield from ctx.wait(0.3)

    # -- raise the pin to just under its stop (the fork itself stays on the rib)
    pin = Riding(ctx, R, "new_fork", (0.0, 0.0, 0.0))  # only its offline flag and hand travel are used

    def pin_up():
        j = float(ctx.obj("new_fork").joints["pin"])
        return float(R.pos[2] - pin.h0[2]) if pin.offline else j

    j0 = pin_up()
    lift = [0.0]

    def raise_target():
        lift[0] = min(lift[0] + 0.005 * ctx.dt, PIN_UP + 0.004)
        return pin.h0 + np.array([0.0, 0.0, lift[0]])

    for _ in R.servo(raise_target, 6.0, KNOB_SHUT, yaw=0.0, max_speed=0.02):
        if pin_up() >= PIN_UP - 0.0015 or lift[0] >= PIN_UP - j0 + 0.004:
            break
        yield

    # -- slide the fork 10 cm along the bars at 1 cm/s to the target notch, on its measured position
    slide = Riding(ctx, R, "new_fork", HOOK, nominal=hung)
    yield from _drive(R, slide, (0.0, Y_TARGET, 0.0), 0.01, tol=0.0005, max_s=16.0, axes=(1,), finger=KNOB_SHUT, yaw=0.0)
    yield from ctx.wait(0.3)
    yield from _drive(R, slide, (0.0, Y_TARGET, 0.0), 0.01, tol=0.0005, max_s=3.0, axes=(1,), finger=KNOB_SHUT, yaw=0.0)

    # -- lower the pin into the notch, checking it went down
    base = np.array(R.pos, float)
    drop = [0.0]

    def lower_target():
        drop[0] = min(drop[0] + 0.005 * ctx.dt, PIN_UP + 0.004)
        return base - np.array([0.0, 0.0, drop[0]])

    for _ in R.servo(lower_target, 6.0, KNOB_SHUT, yaw=0.0, max_speed=0.02):
        j = float(ctx.obj("new_fork").joints["pin"])
        if (not pin.offline and j <= 0.0015) or drop[0] >= PIN_UP + 0.004:
            break
        yield

    # -- let go, rise, park
    yield from R.grip(KNOB_OPEN, 0.6)
    yield from R.move((R.pos[0], R.pos[1], UP), 1.2, KNOB_OPEN, yaw=0.0)
    yield from R.home(3.0)
    yield from ctx.wait(1.0)
