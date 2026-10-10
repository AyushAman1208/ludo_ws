"""ROS-independent wrapper around the MuJoCo Ludo model.

Everything that touches MuJoCo lives here so it can be tested without ROS.
`mujoco_node.py` is a thin ROS 2 wrapper around `LudoSim`.
"""
import os

import mujoco
import numpy as np
from ludo_description.geometry import COLORS

# Command name (as used in JointState commands) -> actuator suffix(es).
# Actuator names in the model are "<color>_a_<suffix>".
COMMANDS = {
    "yaw": ("yaw",),
    "shoulder": ("shoulder",),
    "elbow": ("elbow",),
    "wrist": ("wrist",),
    "grip": ("grip_l", "grip_r"),      # one value opens/closes both fingers
    "head_pan": ("head_pan",),
    "head_tilt": ("head_tilt",),
}
ALL_ACTUATOR_SUFFIXES = ("yaw", "shoulder", "elbow", "wrist",
                         "grip_l", "grip_r", "head_pan", "head_tilt")
CAMERA_KINDS = ("head_cam", "hand_cam")


def default_model_path():
    """models/ludo.xml from the installed package, else from the source tree."""
    try:
        from ament_index_python.packages import get_package_share_directory
        p = os.path.join(get_package_share_directory("ludo_sim"), "models", "ludo.xml")
        if os.path.exists(p):
            return p
    except Exception:
        pass
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.normpath(os.path.join(here, "..", "models", "ludo.xml"))


class LudoSim:
    def __init__(self, model_path=None):
        self.model_path = model_path or default_model_path()
        self.model = mujoco.MjModel.from_xml_path(self.model_path)
        self.data = mujoco.MjData(self.model)
        self._renderers = {}
        self._lookup()
        self.reset()

    # ------------------------------------------------------------------ setup
    def _lookup(self):
        m = self.model
        self.actuator = {}
        for c in COLORS:
            for suf in ALL_ACTUATOR_SUFFIXES:
                name = f"{c}_a_{suf}"
                aid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
                if aid < 0:
                    raise ValueError(f"actuator '{name}' not found in {self.model_path}")
                self.actuator[(c, suf)] = aid

        self.joint_names, self._qadr, self._dadr = [], [], []
        for j in range(m.njnt):
            if int(m.jnt_type[j]) in (int(mujoco.mjtJoint.mjJNT_HINGE), int(mujoco.mjtJoint.mjJNT_SLIDE)):
                self.joint_names.append(m.joint(j).name)
                self._qadr.append(int(m.jnt_qposadr[j]))
                self._dadr.append(int(m.jnt_dofadr[j]))

        self.brick_body = {c: m.body(f"{c}_brick").id for c in COLORS}
        self.camera_names = [m.camera(i).name for i in range(m.ncam)
                             if m.camera(i).name.endswith(CAMERA_KINDS)]

    def reset(self):
        """Go back to the 'home' keyframe (arms at start pose, bricks in yards)."""
        m, d = self.model, self.data
        key = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_KEY, "home")
        if key >= 0:
            mujoco.mj_resetDataKeyframe(m, d, key)
        else:
            mujoco.mj_resetData(m, d)
        mujoco.mj_forward(m, d)

    # ---------------------------------------------------------------- physics
    @property
    def time(self):
        return float(self.data.time)

    @property
    def timestep(self):
        return float(self.model.opt.timestep)

    def step_to(self, sim_time, max_steps=5000):
        """Step until data.time >= sim_time (bounded). Returns steps taken."""
        n = 0
        while self.data.time < sim_time and n < max_steps:
            mujoco.mj_step(self.model, self.data)
            n += 1
        return n

    # ---------------------------------------------------------------- control
    def set_command(self, color, name, value):
        """Set a position target. Returns False if color/name is unknown."""
        if color not in COLORS or name not in COMMANDS:
            return False
        for suf in COMMANDS[name]:
            aid = self.actuator[(color, suf)]
            lo, hi = self.model.actuator_ctrlrange[aid]
            self.data.ctrl[aid] = float(np.clip(value, lo, hi))
        return True

    # ------------------------------------------------------------------ state
    def joint_state(self):
        """(names, positions, velocities) of every hinge/slide joint."""
        q = self.data.qpos[self._qadr]
        v = self.data.qvel[self._dadr]
        return list(self.joint_names), q.tolist(), v.tolist()

    def brick_pose(self, color):
        """((x, y, z), (qx, qy, qz, qw)) of a piece in the world frame."""
        b = self.brick_body[color]
        w, x, y, z = self.data.xquat[b]
        return tuple(float(v) for v in self.data.xpos[b]), (float(x), float(y), float(z), float(w))

    # ---------------------------------------------------------------- cameras
    def render(self, camera):
        """Render a named camera -> HxWx3 uint8 RGB array."""
        cid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, camera)
        if cid < 0:
            raise ValueError(f"camera '{camera}' not found")
        w, h = (int(v) for v in self.model.cam_resolution[cid])
        if w <= 0 or h <= 0:
            w, h = 640, 480
        r = self._renderers.get((h, w))
        if r is None:
            r = self._renderers[(h, w)] = mujoco.Renderer(self.model, h, w)
        r.update_scene(self.data, camera=camera)
        return r.render()

    def close(self):
        for r in self._renderers.values():
            try:
                r.close()
            except Exception:
                pass
        self._renderers.clear()
