"""Analytic kinematics of the Ludo robot arm (pure Python, no ROS, no MuJoCo).

The constants here MUST match models/robots/robot_<color>.xml in ludo_sim
(that file is generated from this module, so they stay in sync).

Arm layout (all pitch joints rotate about the local +y axis, positive = lean forward/down):

    base (z=0) --yaw--> turret (z=.17) --shoulder--> upper arm (z=.20) --L1--> elbow
          --elbow--> forearm --L2--> wrist --wrist--> hand (tool axis +z, pads 0.035..0.095 m)

The gripper always points straight down (shoulder + elbow + wrist = 180 deg). The yaw joint
sets both the azimuth of the arm and the direction of the gripper's closing axis, so a piece
is always gripped across its short side with its long axis pointing along the arm.
"""
import math
from dataclasses import dataclass
from typing import Optional, Tuple

COLORS = ("red", "green", "yellow", "blue")

# ---- board / mounting (matches ludo_sim) -----------------------------------
BASE_DIST = 0.46                                     # robot base is at (+-0.46, +-0.46)
BASE_SIGN = {"red": (-1, -1), "green": (-1, 1), "yellow": (1, 1), "blue": (1, -1)}
YARD_CENTER = 0.225                                  # yard centre is at (+-0.225, +-0.225)

# ---- arm ---------------------------------------------------------------------
SHOULDER_Z = 0.20            # shoulder axis height above the floor
L1 = 0.55                    # upper arm length  [m]
L2 = 0.55                    # forearm length    [m]
TCP_OFFSET = 0.065           # wrist -> centre of the finger pads, along the tool axis [m]
HEAD_LOCAL_XY = (0.0, -0.10) # head mast position in the robot base frame
HEAD_Z = 0.32                # height of the head tilt axis

_DEG = math.pi / 180.0
LIMITS = {                   # (min, max) in rad -- must match the joint ranges in the model
    "yaw": (-100 * _DEG, 100 * _DEG),
    "shoulder": (-30 * _DEG, 100 * _DEG),
    "elbow": (0 * _DEG, 165 * _DEG),
    "wrist": (-90 * _DEG, 120 * _DEG),
    "head_pan": (-120 * _DEG, 120 * _DEG),
    "head_tilt": (-20 * _DEG, 80 * _DEG),
}
LIMIT_MARGIN = 1.0 * _DEG    # IK keeps this far away from the joint limits

# ---- task heights (TCP z) ------------------------------------------------------
BRICK_REST_Z = 0.0545        # centre height of a piece standing on the board
GRASP_Z = 0.060              # pad centre while gripping (finger tips stay 4 mm above the board)
PLACE_Z = 0.062              # pad centre when releasing (piece drops ~3 mm)
HOVER_Z = 0.200              # pad centre while travelling / waiting above a cell

HOME_HEAD = (0.0, 0.6)       # head pan, tilt in the ready pose


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
    def for_color(cls, color: str) -> "ArmGeometry":
        sx, sy = BASE_SIGN[color]
        x, y = sx * BASE_DIST, sy * BASE_DIST
        return cls(color, x, y, math.atan2(-y, -x))

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
        lx, ly = self.to_local(x, y)
        r = math.hypot(lx, ly)
        yaw = math.atan2(ly, lx)
        zw = z + TCP_OFFSET                       # wrist sits directly above the pad centre
        dz = zw - SHOULDER_Z
        d = math.hypot(r, dz)
        if d > L1 + L2 - 1e-3:
            return IKResult(False, None, f"out of reach: {d:.3f} m > {L1 + L2:.3f} m")
        cos_e = (d * d - L1 * L1 - L2 * L2) / (2 * L1 * L2)
        e = math.acos(max(-1.0, min(1.0, cos_e)))                      # elbow-up solution
        s = math.atan2(r, dz) - math.atan2(L2 * math.sin(e), L1 + L2 * math.cos(e))
        w = math.pi - s - e                                            # tool axis straight down
        q = (yaw, s, e, w)
        for name, v in zip(("yaw", "shoulder", "elbow", "wrist"), q):
            lo, hi = LIMITS[name]
            if v < lo + LIMIT_MARGIN or v > hi - LIMIT_MARGIN:
                return IKResult(False, None, f"{name} = {v / _DEG:.1f} deg outside "
                                              f"[{lo / _DEG:.0f}, {hi / _DEG:.0f}]")
        return IKResult(True, q)

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
        sx, sy = BASE_SIGN[self.color]
        return sx * YARD_CENTER, sy * YARD_CENTER

    def home_q(self):
        """Arm 'ready' pose: pads hover above the robot's own yard."""
        x, y = self.yard_center
        res = self.ik(x, y, HOVER_Z)
        assert res.ok, res.reason
        return res.q
