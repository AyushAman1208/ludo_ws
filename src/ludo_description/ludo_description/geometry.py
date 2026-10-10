"""Canonical dimensions and analytic arm geometry shared by all runtime packages.

All lengths are metres. Joint ranges are authored in degrees for MuJoCo and exposed
in radians for the Python kinematics API. Robot XML, yard/start-piece geometry, and
the scene's home keyframe are generated from this module.
"""

import math
from typing import Optional, Tuple

COLORS = ("red", "green", "yellow", "blue")
BASE_SIGN = {
    "red": (-1, -1),
    "green": (-1, 1),
    "yellow": (1, 1),
    "blue": (1, -1),
}
BASE_DIST = 0.46
YARD_CENTER = 0.225
YARD_HALF_SIZE = 0.15
YARD_BACKGROUND_HALF_HEIGHT = 0.0015
YARD_INSET_HALF_SIZE = 0.1125
YARD_INSET_HALF_HEIGHT = 0.0005
YARD_BACKGROUND_Z = 0.0215
YARD_INSET_Z = 0.0235
YARD_SLOT_RADIUS = 0.018
YARD_SLOT_HALF_HEIGHT = 0.0005
YARD_SLOT_Z = 0.0245
BOARD_CELL = 0.05

# The shoulder axis is the upper-arm body's origin beneath the turret.
TURRET_Z = 0.17
SHOULDER_OFFSET_Z = 0.03
SHOULDER_Z = TURRET_Z + SHOULDER_OFFSET_Z
L1 = 0.55
L2 = 0.55
TCP_OFFSET = 0.065

# Head pan and tilt axes are in the robot base frame.
HEAD_LOCAL_XY = (0.0, -0.10)
HEAD_PAN_Z = 0.30
HEAD_TILT_OFFSET_Z = 0.02
HEAD_Z = HEAD_PAN_Z + HEAD_TILT_OFFSET_Z

# These details define the robot's physical envelope in MuJoCo.
BASE_FOOT_RADIUS = 0.07
BASE_FOOT_HALF_HEIGHT = 0.02
BASE_FOOT_Z = 0.02
BASE_PEDESTAL_RADIUS = 0.045
BASE_PEDESTAL_HALF_HEIGHT = 0.065
BASE_PEDESTAL_Z = 0.105
TURRET_RADIUS = 0.05
UPPER_RADIUS = 0.032
FOREARM_RADIUS = 0.026
ELBOW_RADIUS = 0.035
WRIST_RADIUS = 0.027
HAND_BOX_HALF_SIZE = (0.04, 0.008, 0.015)
HAND_BOX_Z = 0.02
FINGER_BODY_Y = 0.03
FINGER_BODY_Z = TCP_OFFSET - 0.030
FINGER_BOX_HALF_SIZE = (0.008, 0.005, 0.03)
FINGER_BOX_Z = 0.03
FINGER_RANGE = (0.0, 0.025)
HAND_CAMERA_Z = 0.036
HEAD_POST_START_Z = 0.04
HEAD_POST_END_Z = HEAD_PAN_Z - 0.01
HEAD_POST_RADIUS = 0.011
HEAD_SPHERE_RADIUS = 0.016
HEAD_NECK_BOX_HALF_SIZE = (0.028, 0.035, 0.024)
HEAD_NECK_BOX_X = 0.005
HEAD_LENS_START_X = 0.033
HEAD_LENS_END_X = 0.046
HEAD_LENS_RADIUS = 0.014
HEAD_CAMERA_X = 0.047

ROBOT_RGBA = {
    "red": (0.86, 0.12, 0.12, 1.0),
    "green": (0.10, 0.65, 0.22, 1.0),
    "yellow": (0.96, 0.80, 0.10, 1.0),
    "blue": (0.13, 0.32, 0.85, 1.0),
}
NEUTRAL_RGBA = (0.25, 0.25, 0.28, 1.0)
LENS_RGBA = (0.05, 0.05, 0.08, 1.0)

# The degree values are the canonical limits written to MuJoCo. Python callers use
# LIMITS, the same values converted to radians.
JOINT_LIMITS_DEG = {
    "yaw": (-100.0, 100.0),
    "shoulder": (-30.0, 100.0),
    "elbow": (0.0, 165.0),
    "wrist": (-90.0, 120.0),
    "head_pan": (-120.0, 120.0),
    "head_tilt": (-20.0, 80.0),
}
LIMITS = {
    name: (math.radians(lo), math.radians(hi))
    for name, (lo, hi) in JOINT_LIMITS_DEG.items()
}
LIMIT_MARGIN = math.radians(1.0)

# Controller gains and actuator ranges are simulation tuning, but their ranges must
# stay consistent with the joint limits above.
ACTUATOR_GAINS = {
    "yaw": (400.0, 60.0),
    "shoulder": (900.0, 120.0),
    "elbow": (600.0, 80.0),
    "wrist": (60.0, 6.0),
    "grip": (200.0, 10.0),
    "head_pan": (2.0, 0.2),
    "head_tilt": (2.0, 0.2),
}
ACTUATOR_LIMITS = {
    "yaw": (-1.75, 1.75),
    "shoulder": (-0.5, 1.75),
    "elbow": (0.0, 2.88),
    "wrist": (-1.5, 2.1),
    "grip": FINGER_RANGE,
    "head_pan": (-2.1, 2.1),
    "head_tilt": (-0.35, 1.4),
}
JOINT_DYNAMICS = {
    "yaw": (2.0, 0.01),
    "shoulder": (2.0, 0.01),
    "elbow": (1.0, 0.005),
    "wrist": (0.5, 0.002),
    "finger": (5.0, None),
    "head": (0.05, 0.002),
}

BRICK_REST_Z = 0.0545
BRICK_SIZE = (0.025, 0.02, 0.03)
BRICK_MASS = 0.08
BRICK_FRICTION = (1.2, 0.01, 0.001)
LIFT_THRESHOLD_Z = 0.065
GRASP_Z = 0.060
PLACE_Z = 0.062
HOVER_Z = 0.200
HOME_HEAD = (0.0, 0.6)


def base_pose_for_color(color: str) -> Tuple[float, float, float]:
    """Return world x, y, and yaw for a robot facing the board centre."""
    sx, sy = BASE_SIGN[color]
    x, y = sx * BASE_DIST, sy * BASE_DIST
    return x, y, math.atan2(-y, -x)


def yard_center_for_color(color: str) -> Tuple[float, float]:
    """Return the world centre of a robot's own yard."""
    sx, sy = BASE_SIGN[color]
    return sx * YARD_CENTER, sy * YARD_CENTER


def solve_arm_ik(
    color: str, x: float, y: float, z: float
) -> Tuple[Optional[Tuple[float, float, float, float]], str]:
    """Solve the canonical down-facing arm IK used by planning and model generation."""
    base_x, base_y, base_yaw = base_pose_for_color(color)
    dx, dy = x - base_x, y - base_y
    c, s = math.cos(base_yaw), math.sin(base_yaw)
    lx, ly = c * dx + s * dy, -s * dx + c * dy
    r = math.hypot(lx, ly)
    yaw = math.atan2(ly, lx)

    dz = z + TCP_OFFSET - SHOULDER_Z
    distance = math.hypot(r, dz)
    if distance > L1 + L2 - 1e-3:
        return None, f"out of reach: {distance:.3f} m > {L1 + L2:.3f} m"

    cos_elbow = (distance * distance - L1 * L1 - L2 * L2) / (2 * L1 * L2)
    elbow = math.acos(max(-1.0, min(1.0, cos_elbow)))
    shoulder = math.atan2(r, dz) - math.atan2(
        L2 * math.sin(elbow), L1 + L2 * math.cos(elbow)
    )
    wrist = math.pi - shoulder - elbow
    q = (yaw, shoulder, elbow, wrist)

    for name, angle in zip(("yaw", "shoulder", "elbow", "wrist"), q):
        lo, hi = LIMITS[name]
        if angle < lo + LIMIT_MARGIN or angle > hi - LIMIT_MARGIN:
            return None, (
                f"{name} = {math.degrees(angle):.1f} deg outside "
                f"[{JOINT_LIMITS_DEG[name][0]:.0f}, {JOINT_LIMITS_DEG[name][1]:.0f}]"
            )
    return q, ""


def home_arm_q(color: str) -> Tuple[float, float, float, float]:
    """Return the ready-pose arm angles; fail loudly if the configured pose is invalid."""
    x, y = yard_center_for_color(color)
    q, reason = solve_arm_ik(color, x, y, HOVER_Z)
    if q is None:
        raise ValueError(f"invalid home pose for {color}: {reason}")
    return q
