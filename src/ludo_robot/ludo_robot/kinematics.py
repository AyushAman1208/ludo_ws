"""Analytic kinematics of the Ludo robot arm (pure Python, no ROS, no MuJoCo).

Arm layout (all pitch joints rotate about the local +y axis, positive = lean forward/down):

    base (z=0) --yaw--> turret (z=TURRET_Z) --shoulder--> upper arm (z=SHOULDER_Z) --L1--> elbow
          --elbow--> forearm --L2--> wrist --wrist--> hand (tool axis +z, pads 0.035..0.095 m)

The gripper always points straight down (shoulder + elbow + wrist = 180 deg). The yaw joint
sets both the azimuth of the arm and the direction of the gripper's closing axis, so a piece
is always gripped across its short side with its long axis pointing along the arm. Dimensions,
limits, and shared IK equations live in ludo_description.geometry, the workspace's canonical
model. MuJoCo model files are generated from that module.
"""
import math
from dataclasses import dataclass
from typing import Optional, Tuple

from ludo_description.geometry import (
    BASE_DIST,
    BASE_SIGN,
    BRICK_REST_Z,
    COLORS,
    GRASP_Z,
    HEAD_LOCAL_XY,
    HEAD_Z,
    HOME_HEAD,
    HOVER_Z,
    L1,
    L2,
    LIMITS,
    LIMIT_MARGIN,
    PLACE_Z,
    SHOULDER_Z,
    TCP_OFFSET,
    TURRET_Z,
    YARD_CENTER,
    base_pose_for_color,
    home_arm_q,
    solve_arm_ik,
    yard_center_for_color,
)

__all__ = [
    "ArmGeometry",
    "BASE_DIST",
    "BASE_SIGN",
    "BRICK_REST_Z",
    "COLORS",
    "GRASP_Z",
    "HEAD_LOCAL_XY",
    "HEAD_Z",
    "HOME_HEAD",
    "HOVER_Z",
    "IKResult",
    "L1",
    "L2",
    "LIMITS",
    "LIMIT_MARGIN",
    "PLACE_Z",
    "SHOULDER_Z",
    "TCP_OFFSET",
    "TURRET_Z",
    "YARD_CENTER",
]


@dataclass(frozen=True)
class IKResult:
    ok: bool
    q: Optional[Tuple[float, float, float, float]]   # (yaw, shoulder, elbow, wrist)
    reason: str = ""


@dataclass(frozen=True)
class ArmGeometry:
    color: str
    base_x: float
    base_y: float
    base_yaw: float          # world yaw of the robot's +x axis (faces the board centre)

    @classmethod
    def create_for_color(cls, color: str) -> "ArmGeometry":
        return cls(color, *base_pose_for_color(color))

    # ---------------------------------------------------------------- frames
    def to_local(self, x, y):
        dx, dy = x - self.base_x, y - self.base_y
        c, s = math.cos(self.base_yaw), math.sin(self.base_yaw)
        return c * dx + s * dy, -s * dx + c * dy

    def to_world(self, lx, ly):
        c, s = math.cos(self.base_yaw), math.sin(self.base_yaw)
        return self.base_x + c * lx - s * ly, self.base_y + s * lx + c * ly

    # --------------------------------------------------------------- forward
    def fk(self, q):
        """q = (yaw, shoulder, elbow, wrist) -> dict of world points (x, y, z)."""
        yaw, s, e, w = q
        az = self.base_yaw + yaw
        ux, uy = math.cos(az), math.sin(az)

        def pt(r, z):
            return (self.base_x + ux * r, self.base_y + uy * r, z)

        r1, z1 = L1 * math.sin(s), SHOULDER_Z + L1 * math.cos(s)
        r2, z2 = r1 + L2 * math.sin(s + e), z1 + L2 * math.cos(s + e)
        phi = s + e + w
        r3, z3 = r2 + TCP_OFFSET * math.sin(phi), z2 + TCP_OFFSET * math.cos(phi)
        return {"elbow": pt(r1, z1), "wrist": pt(r2, z2), "tcp": pt(r3, z3)}

    # --------------------------------------------------------------- inverse
    def ik(self, x, y, z) -> IKResult:
        """Joint angles that put the pad centre at world (x, y, z), gripper pointing down."""
        q, reason = solve_arm_ik(self.color, x, y, z)
        return IKResult(q is not None, q, reason)

    def reachable(self, x, y, z) -> bool:
        return self.ik(x, y, z).ok

    # ------------------------------------------------------------------ head
    def gaze(self, x, y, z) -> Tuple[float, float]:
        """(pan, tilt) that point the head camera at a world point (tilt > 0 looks down)."""
        lx, ly = self.to_local(x, y)
        dx, dy = lx - HEAD_LOCAL_XY[0], ly - HEAD_LOCAL_XY[1]
        pan = math.atan2(dy, dx)
        tilt = math.atan2(HEAD_Z - z, math.hypot(dx, dy))
        clip = lambda v, n: max(LIMITS[n][0], min(LIMITS[n][1], v))
        return clip(pan, "head_pan"), clip(tilt, "head_tilt")

    # ------------------------------------------------------------------ home
    @property
    def yard_center(self):
        return yard_center_for_color(self.color)

    def home_q(self):
        """Arm 'ready' pose: pads hover above the robot's own yard."""
        return home_arm_q(self.color)
