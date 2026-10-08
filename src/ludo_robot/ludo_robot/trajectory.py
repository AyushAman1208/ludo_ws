"""Pick-and-place planning and execution (pure Python, no ROS).

A task is a list of *steps*. `MotionExecutor` plays them back at a fixed rate: every call to
`update(dt, measured)` returns the next set of joint position targets. The ROS node only
shuttles messages in and out, so everything here can be tested against the plain simulator.
"""
import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

from ludo_robot import kinematics as K

Vec3 = Tuple[float, float, float]

GRIP_OPEN = 0.0
GRIP_CLOSE = 0.02            # finger target; the piece stops the fingers at ~0.005
GRASP_OK_MAX = 0.012         # fingers closer together than this (<= 0.012) => holding a piece
SPEED_TRAVEL = 0.7           # m/s, moving above the board
SPEED_FINE = 0.25            # m/s, descending / lifting at a cell


# ----------------------------------------------------------------------- steps
@dataclass
class Move:                  # straight line of the pad centre (gripper stays pointing down)
    target: Vec3
    speed: float = SPEED_TRAVEL


@dataclass
class Lift:                  # straight up/down from wherever the arm currently is
    z: float
    speed: float = SPEED_FINE


@dataclass
class Grip:
    value: float
    wait: float = 0.6
    expect_object: Optional[bool] = None    # True: fail if the fingers closed on nothing


@dataclass
class Settle:                # wait until the arm has caught up with the commanded pose
    tol: float = 0.01
    timeout: float = 3.0


@dataclass
class Gaze:                  # point the head at a world point (None = default look)
    target: Optional[Vec3]


@dataclass
class GoHome:                # joint-space move back to the ready pose
    duration: float = 1.5


# -------------------------------------------------------------------- planning
def _smooth(t):              # minimum-jerk 0..1
    return t * t * t * (10 - 15 * t + 6 * t * t)


def pick_and_place_steps(src_xy, dst_xy, go_home=True) -> List:
    sx, sy = src_xy
    dx, dy = dst_xy
    steps = [
        Gaze((sx, sy, K.BRICK_REST_Z)),
        Grip(GRIP_OPEN, wait=0.0),
        Move((sx, sy, K.HOVER_Z)), Settle(),
        Move((sx, sy, K.GRASP_Z), SPEED_FINE), Settle(0.006, 2.0),
        Grip(GRIP_CLOSE, wait=0.8, expect_object=True),
        Move((sx, sy, K.HOVER_Z), SPEED_FINE),
        Gaze((dx, dy, K.BRICK_REST_Z)),
        Move((dx, dy, K.HOVER_Z)), Settle(),
        Move((dx, dy, K.PLACE_Z), SPEED_FINE), Settle(0.006, 2.0),
        Grip(GRIP_OPEN, wait=0.6),
        Lift(K.HOVER_Z),
    ]
    if go_home:
        steps += [Gaze(None), GoHome()]
    return steps


def recovery_steps() -> List:
    return [Grip(GRIP_OPEN, wait=0.5), Lift(K.HOVER_Z, 0.3), Gaze(None), GoHome()]


def park_steps() -> List:
    return [Grip(GRIP_OPEN, wait=0.3), Lift(K.HOVER_Z, 0.3), Gaze(None), GoHome()]


def validate_steps(geo: K.ArmGeometry, steps, q_start) -> Optional[str]:
    """Dry-run every Cartesian move through the IK. Returns an error string, or None if OK."""
    p = geo.fk(q_start)["tcp"]
    for st in steps:
        if isinstance(st, (Move, Lift)):
            tgt = st.target if isinstance(st, Move) else (p[0], p[1], st.z)
            n = max(2, int(math.dist(p, tgt) / 0.02))
            for i in range(n + 1):
                a = i / n
                pt = tuple(p[k] + (tgt[k] - p[k]) * a for k in range(3))
                res = geo.ik(*pt)
                if not res.ok:
                    return f"unreachable at ({pt[0]:.3f}, {pt[1]:.3f}, {pt[2]:.3f}): {res.reason}"
            p = tgt
        elif isinstance(st, GoHome):
            p = geo.fk(geo.home_q())["tcp"]
    return None


# ------------------------------------------------------------------- execution
JOINTS = ("yaw", "shoulder", "elbow", "wrist")


class MotionExecutor:
    def __init__(self, geo: K.ArmGeometry):
        self.geo = geo
        self.home_q = tuple(geo.home_q())
        self.cmd_q = list(self.home_q)                 # last commanded arm joints
        self.cmd_grip = GRIP_OPEN
        self.gaze_target: Optional[Vec3] = None
        self.steps: List = []
        self.idx = 0
        self.t = 0.0
        self.failed = False
        self.message = ""
        self._seg = None                               # per-step working data

    # ------------------------------------------------------------------ API
    @property
    def busy(self):
        return self.idx < len(self.steps) and not self.failed

    def begin(self, steps):
        self.steps, self.idx, self.t, self._seg = list(steps), 0, 0.0, None
        self.failed, self.message = False, ""

    def abort(self, message):
        self.failed, self.message = True, message

    def commands(self):
        pan, tilt = K.HOME_HEAD
        if self.gaze_target is not None:
            pan, tilt = self.geo.gaze(*self.gaze_target)
        out = dict(zip(JOINTS, self.cmd_q))
        out.update(grip=self.cmd_grip, head_pan=pan, head_tilt=tilt)
        return out

    def update(self, dt, measured: Dict[str, float]):
        """Advance by dt seconds. `measured`: joint name -> position. Returns joint targets."""
        guard = 0
        while self.busy and guard < 8:
            guard += 1
            if self._step(dt, measured):          # True => step finished, start the next now
                self.idx += 1
                self.t, self._seg = 0.0, None
                dt = 0.0
            else:
                break
        return self.commands()

    # ---------------------------------------------------------------- steps
    def _step(self, dt, m) -> bool:
        st = self.steps[self.idx]
        first = self._seg is None
        self.t += dt

        if isinstance(st, Gaze):
            self.gaze_target = st.target
            return True

        if isinstance(st, Grip):
            if first:
                self.cmd_grip = st.value
                self._seg = True
            if self.t < st.wait:
                return False
            if st.expect_object is not None:
                width = max(m.get("finger_l", 0.0), m.get("finger_r", 0.0))
                holding = width <= GRASP_OK_MAX
                if st.expect_object and not holding:
                    self.abort("grasp failed: fingers closed on nothing")
                    return False
            return True

        if isinstance(st, Settle):
            if first:
                self._seg = True
            err = max(abs(m.get(j, c) - c) for j, c in zip(JOINTS, self.cmd_q))
            if err < st.tol and self.t >= 0.05:
                return True
            if self.t > st.timeout:
                return True                         # carry on; the verification step will notice
            return False

        if isinstance(st, (Move, Lift)):
            if first:
                p0 = self.geo.fk(self.cmd_q)["tcp"]
                p1 = st.target if isinstance(st, Move) else (p0[0], p0[1], st.z)
                T = max(math.dist(p0, p1) / st.speed, 0.15)
                self._seg = (p0, p1, T)
            p0, p1, T = self._seg
            a = _smooth(min(self.t / T, 1.0))
            pt = tuple(p0[k] + (p1[k] - p0[k]) * a for k in range(3))
            res = self.geo.ik(*pt)
            if not res.ok:
                self.abort(f"IK failed at ({pt[0]:.3f}, {pt[1]:.3f}, {pt[2]:.3f}): {res.reason}")
                return False
            self.cmd_q = list(res.q)
            return self.t >= T

        if isinstance(st, GoHome):
            if first:
                self._seg = (list(self.cmd_q), st.duration)
            q0, T = self._seg
            a = _smooth(min(self.t / T, 1.0))
            self.cmd_q = [q0[i] + (self.home_q[i] - q0[i]) * a for i in range(4)]
            return self.t >= T

        raise TypeError(f"unknown step {st!r}")
