"""On the station table: a panel antenna hangs on a hinge from a grey mast, in front of the right-hand side of the
table. The white panel is swung out at its foot by a blue handle on its left edge and is held at plumb by an orange
lock pin that passes through a red bracket arm on the panel's right edge and the plumb hole of a grey tilt plate.
The plate has four square holes on an arc (plumb, 5, 10 and 15 degrees); the 10 degree hole has a green tick above
it. The pin has a yellow head tab standing on the table.

Requires -E object_jitter=0.

Run it:  inspect-robots run --embodiment yam_arena -E task=repin_the_antenna_downtilt.py -E object_jitter=0 \\
           --instruction "Reset the antenna's downtilt to the hole marked in green on its tilt plate." \\
           --scorer task_score ...
"""

import os, sys; sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))  # noqa: E402,E702

import math

import numpy as np
from belimits import ResetOnJumpTracker
from closedloop import grip_rot, placeholder_demo
from mechkit import Joint, Mech, box, on_reset
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.task_file import TABLE_TOP

SCENE = "station"
TOP = TABLE_TOP[SCENE]

WHITE, GREY, DARK = (0.93, 0.93, 0.95), (0.62, 0.64, 0.70), (0.20, 0.22, 0.26)
BLUE, RED, GREEN = (0.10, 0.28, 0.90), (0.85, 0.10, 0.10), (0.05, 0.75, 0.15)
ORANGE, YELLOW = (0.98, 0.50, 0.05), (0.98, 0.90, 0.30)

# --- the geometry (metres; the Mechs sit at x 0, y 0 on the table top, so their local x, y are station x, y and local
#     z is height above the table) ------------------------------------------------------------------------------------
PX, PZ = 0.315, 0.26  # hinge axis (along y): x, height
ARM_R = 0.20  # the holes sit 20 cm below the hinge axis
HOLE = 0.014  # square holes, 1.4 cm
ROD = 0.008  # the pin's rod, 0.8 cm square: 3 mm of play a side
ANGLES_DEG = (0, 5, 10, 15)
TARGET_DEG = 10
PLATE_Y = (-0.092, -0.082)
ARM_Y = (-0.080, -0.070)
TILT_MAX = 0.30  # rad, the joint's upper stop (17.2 deg)


def hole_xz(angle_rad: float) -> tuple[float, float]:
    """World x and height above the table of the hole at ``angle_rad`` on the arc."""
    return PX - ARM_R * math.sin(angle_rad), PZ - ARM_R * math.cos(angle_rad)


PLATE_HOLES = {a: hole_xz(math.radians(a)) for a in ANGLES_DEG}


def _build_plate():
    """The tilt plate as boxes around four square holes: x 0.24..0.33, height 0..0.10, y -0.092..-0.082."""
    y0, y1 = PLATE_Y
    top_h = 0.10
    shapes = []
    spans = sorted(((hx - HOLE / 2, hx + HOLE / 2, hz, a) for a, (hx, hz) in PLATE_HOLES.items()))
    x_edges = [(0.24, spans[0][0])] + [(spans[i][1], spans[i + 1][0]) for i in range(len(spans) - 1)]
    x_edges.append((spans[-1][1], 0.33))
    for x0, x1 in x_edges:
        shapes.append(box(((x0 + x1) / 2, (y0 + y1) / 2, top_h / 2), (x1 - x0, y1 - y0, top_h), GREY))
    for x0, x1, hz, a in spans:
        col = GREEN if a == TARGET_DEG else GREY
        bot = hz - HOLE / 2
        top = hz + HOLE / 2
        shapes.append(box(((x0 + x1) / 2, (y0 + y1) / 2, bot / 2), (x1 - x0, y1 - y0, bot), col))
        shapes.append(box(((x0 + x1) / 2, (y0 + y1) / 2, (top + top_h) / 2), (x1 - x0, y1 - y0, top_h - top), col))
        # the tick above each hole: a tall green one for the marked hole, short dark ones for the others
        tick = 0.025 if a == TARGET_DEG else 0.010
        shapes.append(box(((x0 + x1) / 2, (y0 + y1) / 2, top_h + tick / 2), (x1 - x0, y1 - y0, tick), col if a == TARGET_DEG else DARK))
    return shapes


MAST_SHAPES = [
    box((0.345, 0.0, 0.15), (0.03, 0.04, 0.30), DARK),  # mast post, x 0.33..0.36
    box((0.323, 0.0, 0.26), (0.014, 0.04, 0.02), DARK),  # pivot bracket, x 0.316..0.33
] + _build_plate()

# The panel and its arm are one hinged body. Joint frame origin = the hinge axis, local z = height - PZ.
def _arm_shapes():
    ay0, ay1 = ARM_Y
    cy, ty = (ay0 + ay1) / 2, ay1 - ay0
    lo, hi = -ARM_R - HOLE / 2, -ARM_R + HOLE / 2  # hole span in local z
    wall = 0.008
    out = [
        box((0.0, cy, (lo - wall + lo) / 2), (0.030, ty, wall), RED),  # below the hole
        box((0.0, cy, (hi + hi + wall) / 2), (0.030, ty, wall), RED),  # above the hole
        box((-(HOLE / 2 + wall / 2), cy, -ARM_R), (wall, ty, HOLE), RED),  # -x of the hole
        box((HOLE / 2 + wall / 2, cy, -ARM_R), (wall, ty, HOLE), RED),  # +x of the hole
        box((0.0, cy, -0.1725), (0.030, ty, 0.025), RED),  # short strip up to the panel's foot (height 7.5 to 10 cm)
    ]
    return out


TILT = Joint(
    "tilt", "hinge", axis="y", at=(PX, 0.0, PZ), low=0.0, high=TILT_MAX, spring=0.0, damping=2.0, mass=0.25,
    shapes=[
        box((-0.01, 0.0, -0.095), (0.02, 0.14, 0.17), WHITE),  # panel x 0.295..0.315, height 0.09..0.25 (see below)
        box((-0.011, 0.0975, -0.155), (0.024, 0.055, 0.035), BLUE),  # handle on the +y edge, x 2.4 cm, y 0.07..0.125, height 0.0875..0.1225
        *_arm_shapes(),
    ],
)
ANTENNA = Mech("antenna", pos=(0.0, 0.0, TOP), mass=None, shapes=MAST_SHAPES, joints=[TILT])

# The lock pin: orange rod 0.8 x 19 x 0.8 cm along y, yellow head tab 1.8 x 1.8 x 3.0 cm at its -y end. The Mech origin is the
# rod's centre. The rod is long so the pin's centre of mass (-8 cm in y) lies between the plate's outer face (-9.2) and the
# arm (-7.0): released, it rests level on the bottoms of the holes with the tab clear of the table.
# Start: rod y -0.13..+0.06, resting on the bottoms of the plumb holes, tip under the panel's foot.
ROD_LEN = 0.19
TAB_Y, TAB_X, TAB_Z = 0.018, 0.018, 0.030
PIN_Z0 = PLATE_HOLES[0][1] - HOLE / 2 + ROD / 2 + 0.0005  # rod centre height at rest on the hole bottoms
PIN_Y0 = -0.035
PIN = Mech(
    "lock_pin", pos=(PX, PIN_Y0, TOP + PIN_Z0), mass=0.05, friction=0.4,
    shapes=[
        box((0.0, 0.0, 0.0), (ROD, ROD_LEN, ROD), ORANGE),
        box((0.0, -ROD_LEN / 2 - TAB_Y / 2, 0.0), (TAB_X, TAB_Y, TAB_Z), YELLOW),
    ],
)
OBJECTS = [ANTENNA, PIN]

SUCCESS_HOLD_S = 8.0  # the demo grips: keep the run going so the release and the retreat are on video

# --- scoring ----------------------------------------------------------------------------------------------------------
TOL = 0.0035  # m: rod axis within this of a hole's centre (the holes allow 3 mm; 0.5 mm for the simulator's contact)
T_LO, T_HI = math.radians(7.5), math.radians(12.0)
CLEAR_Y = PLATE_Y[0] - 0.002  # the rod's tip behind the plate's outer face: out of the plumb hole
ENTER_Y = PLATE_Y[0] + 0.005  # 5 mm into the plate's hole
AIR = 0.010  # the pin's lowest point more than this above the table: held up, not placed
FELL = TOP - 0.03

def _home_centre():
    from mechkit import _local_corners

    c = _local_corners(PIN, {}) + np.array(PIN.pos)
    return (c.min(0) + c.max(0)) / 2


_LATCH = ResetOnJumpTracker({"lock_pin": _home_centre()})
on_reset(lambda env: _LATCH.clear())


def _rod(pin):
    """Rod end points (low y, high y) in the world, from the pin's pose."""
    return pin.to_world((0.0, -ROD_LEN / 2, 0.0)), pin.to_world((0.0, ROD_LEN / 2, 0.0))


def _axis_at(p0, p1, y):
    """World (x, height above table) of the rod's axis at plane y, or None if the rod is more than 37 degrees off y (the hole positions bound the real angle to about 30)."""
    d = p1 - p0
    if abs(d[1]) < 0.8 * np.linalg.norm(d):
        return None
    t = (y - p0[1]) / d[1]
    q = p0 + t * d
    return np.array([q[0], q[2] - TOP])


def _arm_hole(T):
    return np.array(hole_xz(T))


def _through(pin, T, angle_deg) -> bool:
    """The rod passes through the plate's hole at ``angle_deg`` and the arm's hole (at tilt T)."""
    p0, p1 = _rod(pin)
    if p0[1] > p1[1]:
        p0, p1 = p1, p0
    if p0[1] > PLATE_Y[0] - 0.002 or p1[1] < ARM_Y[1] - 0.002:
        return False  # does not span both plates
    a = _axis_at(p0, p1, sum(PLATE_Y) / 2)
    b = _axis_at(p0, p1, sum(ARM_Y) / 2)
    if a is None or b is None:
        return False
    hole = np.array(PLATE_HOLES[angle_deg])
    return bool(np.abs(a - hole).max() <= TOL and np.abs(b - _arm_hole(T)).max() <= TOL)


def _entered(pin, T) -> bool:
    """Tip 5 mm or more into the marked plate hole, rod aimed along it, panel tilted 8 to 12 degrees."""
    p0, p1 = _rod(pin)
    if p0[1] > p1[1]:
        p0, p1 = p1, p0
    if not (T_LO <= T <= T_HI) or p1[1] < ENTER_Y or p0[1] > PLATE_Y[0]:
        return False
    a = _axis_at(p0, p1, sum(PLATE_Y) / 2)
    return a is not None and bool(np.abs(a - np.array(PLATE_HOLES[TARGET_DEG])).max() <= TOL)


def _out(pin) -> bool:
    p0, p1 = _rod(pin)
    return bool(max(p0[1], p1[1]) <= CLEAR_Y or pin.low[2] < FELL)


def _airborne(pin) -> bool:
    """More than 1 cm above every support (the table and the bottom of the marked hole): held up, never counted as placed."""
    hole_bottom = TOP + PLATE_HOLES[TARGET_DEG][1] - HOLE / 2
    return bool(pin.low[2] - TOP > AIR and pin.low[2] - hole_bottom > AIR)


def _flags(objects):
    pin, ant = objects["lock_pin"], objects["antenna"]
    T = float(ant.joints["tilt"])
    peaks = _LATCH.update(objects, {"out": float(_out(pin)), "entered": float(_entered(pin, T))})
    return pin, T, peaks["out"] > 0.5, peaks["entered"] > 0.5


def score(objects):
    """0 if the pin ends through the plumb, 5 or 15 degree hole (a wrong tilt goes live). Otherwise 1.0 for the pin through
    the marked hole and the arm with the panel at 8 to 12 degrees and the pin not held up; 0.5 once the pin was drawn
    out and its tip entered the marked hole; 0.25 once it was drawn out; else 0."""
    pin, T, out, entered = _flags(objects)
    if any(_through(pin, T, a) for a in ANGLES_DEG if a != TARGET_DEG):
        return 0.0
    if out and not _airborne(pin) and _through(pin, T, TARGET_DEG) and T_LO <= T <= T_HI:
        return 1.0
    if out and entered:
        return 0.5
    return 0.25 if out else 0.0


def success(objects):
    return score(objects) == 1.0 and objects["lock_pin"].resting()


# The robot keeps out of the space in front of the antenna's face too (checked offline with the URDF).
ARM_KEEP_OUT = [{"name": "in front of the antenna face", "x": (0.18, 0.25), "y": (-0.07, 0.07), "z_top": TOP + 0.25}]

# --- the demo ---------------------------------------------------------------------------------------------------------
demo = placeholder_demo

FINGER_TAB = -0.016  # pads 3.2 cm apart: clear of the plate while the right hand is at the pin's head
FINGER_HANDLE = -0.032  # pads 6.4 cm apart round the 2.4 cm handle: 2 cm of slack a side
HANDLE_LOCAL = np.array([-0.011, 0.106, -0.155])  # where the pads close
TAB_GRIP_Z = TOP + 0.060  # pad tips 1.3 cm lower, inside the 3 cm tab (height 4.25 to 7.25 cm)
HANDLE_GRIP_Z = TOP + 0.103
YAW_LEFT = math.pi / 2
UP = TOP + 0.20
T_GOAL = math.radians(TARGET_DEG)
PULL = 0.165  # m the pin is drawn out (its tip ends 1.1 cm behind the plate)
SWING_S, INSERT_S = 5.0, 12.0


def _ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def handle_world(T):
    """World position of the handle's centre when the panel is tilted T."""
    return np.array([PX, 0.0, TOP + PZ]) + _ry(T) @ HANDLE_LOCAL


def left_rot(T):
    """Gripper turned with the handle: fingers along x, tilted T about the hinge axis."""
    return _ry(T) @ grip_rot(yaw=YAW_LEFT)


def _rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def _rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


class RodLoop:
    """Closed loop on the pin the right hand carries: trims the hand's position so the rod's axis, extended to the plate's mid
    plane, meets the wanted point, and trims its pitch and yaw so the rod stays along y (the wrist sags, the grip does not slip)."""

    def __init__(self, ctx):
        self.ctx = ctx
        self.a = self.b = 0.0  # commanded extra pitch (about x) and yaw (about z)
        self.off = np.zeros(2)  # extra hand x, z
        self.pit = self.yaw = 0.0
        self.pin0 = np.array(ctx.obj("lock_pin").origin, float)
        self.hand0 = np.array(ctx.right.measured_pos, float)

    def _ends(self):
        p0, p1 = _rod(self.ctx.obj("lock_pin"))
        return (p0, p1) if p0[1] < p1[1] else (p1, p0)

    def step(self, aim, k_rot=0.05, k_pos=0.10):
        """Update the corrections from the measured pin; returns the aim error in metres."""
        tail, tip = self._ends()
        d = tip - tail
        moved = np.linalg.norm(np.array(self.ctx.right.measured_pos, float) - self.hand0)
        if moved > 0.03 and np.linalg.norm(np.array(self.ctx.obj("lock_pin").origin, float) - self.pin0) < 0.001:
            return 0.0  # the pin does not follow the hand (an offline replay with fixed objects): nothing to correct
        self.pit, self.yaw = math.atan2(d[2], d[1]), math.atan2(d[0], d[1])
        self.a = float(np.clip(self.a - k_rot * self.pit, -0.2, 0.2))
        self.b = float(np.clip(self.b + k_rot * self.yaw, -0.2, 0.2))
        q = tail + (sum(PLATE_Y) / 2 - tail[1]) / d[1] * d
        e = aim - np.array([q[0], q[2]])
        self.off = np.clip(self.off + k_pos * e, -0.03, 0.03)
        return float(np.abs(e).max())

    def angle_err(self):
        return max(abs(self.pit), abs(self.yaw))

    def command(self, nominal):
        return np.array([nominal[0] + self.off[0], nominal[1], nominal[2] + self.off[1]]), _rz(self.b) @ _rx(self.a) @ grip_rot(yaw=0.0)


def _smooth(s):
    s = min(1.0, max(0.0, s))
    return s * s * (3 - 2 * s)


def demo_script(ctx):
    """Right hand draws the pin out by its head; left hand swings the panel by its handle to 10 degrees (tilting its
    gripper with the handle) and holds it, correcting from the panel's measured tilt; right hand aims the pin at the marked
    plate hole and pushes it through the plate and the arm, correcting from the pin's measured position; both let go and
    park. Corrections are bounded so the nominal path alone stays inside the reach."""
    L, R, dt = ctx.left, ctx.right, ctx.dt
    pin0 = ctx.obj("lock_pin")
    ox, oy, oz = (float(v) for v in pin0.origin)
    grip = np.array([ox, oy - ROD_LEN / 2 - TAB_Y / 2, TAB_GRIP_Z])  # the tab's centre

    # -- right: take the pin's head, draw the pin 16.5 cm out along -y, keeping the rod level and square on the plumb hole
    yield from R.move((grip[0], grip[1], UP), 3.0, FINGER_TAB, yaw=0.0)
    yield from R.move(grip + np.array([0, 0, 0.03]), 1.5, FINGER_TAB, yaw=0.0)
    yield from R.move(grip, 1.0, FINGER_TAB, yaw=0.0)
    yield from R.trim_to_measured(grip)
    yield from R.grip(FINGER_CLOSED, 0.8)
    yield from ctx.wait(0.3)
    loop = RodLoop(ctx)
    hx0, hz0 = PLATE_HOLES[0]
    aim0 = np.array([hx0, TOP + hz0])
    k0 = [0]

    def pull_target():
        k0[0] += 1
        loop.step(aim0)
        y = grip[1] - PULL * _smooth(k0[0] * dt / 9.0)
        return loop.command(np.array([grip[0], y, grip[2]]))

    yield from R.servo(pull_target, 9.0, FINGER_CLOSED, yaw=0.0, max_speed=0.03)
    yield from ctx.wait(0.5)

    # -- left: take the handle
    h0 = handle_world(0.0)
    hg = np.array([h0[0], h0[1], HANDLE_GRIP_Z])
    yield from L.move((hg[0], hg[1], UP), 3.0, FINGER_HANDLE, rot=left_rot(0.0))
    yield from L.move(hg + np.array([0, 0, 0.03]), 1.5, FINGER_HANDLE, rot=left_rot(0.0))
    yield from L.move(hg, 1.0, FINGER_HANDLE, rot=left_rot(0.0))
    yield from L.trim_to_measured(hg, tol=0.001, max_duration=3.0, max_comp=0.04)
    yield from ctx.wait(0.5)
    yield from L.trim_to_measured(hg, tol=0.001, max_duration=2.0, max_comp=0.04)
    yield from L.grip(FINGER_CLOSED, 0.8)
    yield from ctx.wait(0.3)

    # -- left swings to 10 degrees and holds; right brings the rod onto the marked hole's axis, then pushes it in
    hx, hz = PLATE_HOLES[TARGET_DEG]
    aim = np.array([hx, TOP + hz])
    n = [0]
    dT = [0.0]
    dxz = np.zeros(2)

    def left_target():
        n[0] += 1
        t = n[0] * dt
        T = float(ctx.obj("antenna").joints["tilt"])
        if t > SWING_S - 1.0:  # after the swing, trim the commanded tilt from the panel's measured tilt
            dT[0] = float(np.clip(dT[0] + 0.1 * (T_GOAL - T), -0.05, 0.05))
        Tc = T_GOAL * _smooth(t / (SWING_S - 1.5)) + dT[0]
        p = handle_world(Tc)
        return (np.array([p[0], p[1], HANDLE_GRIP_Z + p[2] - h0[2]]), left_rot(Tc))

    # The rod is carried to the marked hole's axis and pushed in. The hand's own pitch sags under gravity, so the loop works on
    # the rod itself: it holds the rod's axis, extended to the plate's mid plane, on the hole's axis and the rod square to
    # the plate (no pitch, no yaw), and only advances while the aim is within 2 mm.
    start = np.array(R.pos, float)
    m = [0]
    prog = [0.0]
    stall = [0]
    fin = [0]
    forced = [False]
    seq = {"aim": aim0.copy()}

    def right_target():
        m[0] += 1
        t = m[0] * dt
        s = _smooth(t / (SWING_S - 1.5))
        seq["aim"] = aim0 + (aim - aim0) * s
        err = loop.step(seq["aim"])
        if t > SWING_S:
            ok = err < 0.002 and loop.angle_err() < 0.02
            stall[0] = 0 if ok else stall[0] + 1
            if stall[0] > 120:
                forced[0] = True  # the aim did not settle in 6 s: go on rather than hang
            if ok or forced[0]:
                prog[0] = min(PULL + 0.003, prog[0] + (PULL + 0.003) / INSERT_S * dt)
            if prog[0] >= PULL + 0.003:
                fin[0] += 1
        nominal = np.array([start[0] + (seq["aim"][0] - aim0[0]), start[1] + prog[0], start[2] + (seq["aim"][1] - aim0[1])])
        return loop.command(nominal)

    def until_done(gen):
        for _ in gen:
            if fin[0] > 20:  # the pin is home for 1 s: stop, the scripts below let go
                return
            yield

    total = SWING_S + 2 * INSERT_S + 2.0
    yield from ctx.both(
        until_done(L.servo(left_target, total, FINGER_CLOSED, max_speed=0.03)),
        until_done(R.servo(right_target, total, FINGER_CLOSED, yaw=0.0, max_speed=0.03)),
    )

    # -- right lets go (pads part only 3.2 cm) and rises; the left lets go and both park, all inside a few seconds
    yield from R.grip(FINGER_TAB, 0.4)
    yield from R.move(R.pos + np.array([0.0, 0.0, 0.14]), 1.0, FINGER_TAB, yaw=0.0)

    def left_leave():
        yield from L.grip(FINGER_HANDLE, 0.4)
        yield from L.move(L.pos + np.array([0.0, 0.06, 0.10]), 1.0, FINGER_HANDLE, rot=left_rot(0.0))
        yield from L.home(2.0)

    yield from ctx.both(left_leave(), R.home(2.0))
    yield from ctx.wait(1.5)
