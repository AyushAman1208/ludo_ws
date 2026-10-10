#!/usr/bin/env python3
"""ROS 2 node that runs the MuJoCo Ludo simulation.

Publishes
  /mujoco/joint_states            sensor_msgs/JointState   all hinge/slide joints (full model names)
  /mujoco/brick_poses             geometry_msgs/PoseArray  pieces, order = red, green, yellow, blue
  /<color>/head_cam/image_raw     sensor_msgs/Image        rgb8   (if render_cameras)
  /<color>/hand_cam/image_raw     sensor_msgs/Image        rgb8   (if render_cameras)

Subscribes
  /<color>/joint_command          sensor_msgs/JointState   position targets, names from:
                                  yaw, shoulder, elbow, wrist, grip, head_pan, head_tilt

Services
  /mujoco/reset                   std_srvs/Empty          back to the 'home' keyframe

Parameters
  model_path (str)      path to ludo.xml            [installed models/ludo.xml]
  viewer (bool)         open the interactive viewer [true]
  render_cameras (bool) publish camera images       [true]
  camera_rate (float)   camera Hz                   [10.0]
  state_rate (float)    joint/brick state Hz        [50.0]
  realtime_factor (float)                           [1.0]
"""
import contextlib
import time

import mujoco
import rclpy
from geometry_msgs.msg import Pose, PoseArray
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Int32
from std_srvs.srv import Empty

from ludo_sim.sim_core import COLORS, LudoSim

try:
    from rclpy.executors import ExternalShutdownException
except ImportError:  # older distros
    ExternalShutdownException = KeyboardInterrupt


class MujocoNode(Node):
    def __init__(self):
        super().__init__("mujoco_node")
        self.declare_parameter("model_path", "")
        self.declare_parameter("viewer", True)
        self.declare_parameter("render_cameras", True)
        self.declare_parameter("camera_rate", 10.0)
        self.declare_parameter("state_rate", 50.0)
        self.declare_parameter("realtime_factor", 1.0)

        model_path = self.get_parameter("model_path").value or None
        self.rtf = float(self.get_parameter("realtime_factor").value)
        self.sim = LudoSim(model_path)
        self.get_logger().info(f"loaded {self.sim.model_path}")

        # --- ROS interfaces -------------------------------------------------
        self.joint_pub = self.create_publisher(JointState, "/mujoco/joint_states", 10)
        self.brick_pub = self.create_publisher(PoseArray, "/mujoco/brick_poses", 10)
        self.last_dice_roll = None
        self.create_subscription(Int32, "/ludo/dice", self._on_dice_roll, 10)
        for c in COLORS:
            self.create_subscription(
                JointState, f"/{c}/joint_command",
                lambda msg, color=c: self._on_command(color, msg), 10)
        self.create_service(Empty, "/mujoco/reset", self._on_reset)

        # --- cameras ---------------------------------------------------------
        self.cam_pubs = {}
        if self.get_parameter("render_cameras").value:
            try:
                for cam in self.sim.camera_names:          # e.g. red_head_cam
                    color, kind = cam.split("_", 1)        # red, head_cam
                    self.cam_pubs[cam] = self.create_publisher(
                        Image, f"/{color}/{kind}/image_raw", 1)
                self.sim.render(self.sim.camera_names[0])  # fail early if no GL
                rate = float(self.get_parameter("camera_rate").value)
                self.create_timer(1.0 / rate, self._publish_cameras)
            except Exception as e:  # no OpenGL available, etc.
                self.get_logger().warn(
                    f"camera rendering disabled ({e}). On a headless machine try: "
                    "export MUJOCO_GL=egl")
                self.cam_pubs = {}

        # --- viewer ----------------------------------------------------------
        self.viewer = None
        if self.get_parameter("viewer").value:
            try:
                import mujoco.viewer
                self.viewer = mujoco.viewer.launch_passive(self.sim.model, self.sim.data)
            except Exception as e:
                self.get_logger().warn(f"viewer disabled ({e})")

        # --- timers ----------------------------------------------------------
        self._wall0 = time.monotonic() - self.sim.time / self.rtf
        self.create_timer(0.01, self._tick)                                  # physics, 100 Hz
        self.create_timer(1.0 / float(self.get_parameter("state_rate").value),
                          self._publish_state)

    # ------------------------------------------------------------------ utils
    def _lock(self):
        return self.viewer.lock() if self.viewer is not None else contextlib.nullcontext()

    # -------------------------------------------------------------- callbacks
    def _on_command(self, color, msg):
        with self._lock():
            for name, value in zip(msg.name, msg.position):
                if not self.sim.set_command(color, name, value):
                    self.get_logger().warn(
                        f"[{color}] unknown command '{name}'", throttle_duration_sec=5.0)

    def _on_reset(self, request, response):
        with self._lock():
            self.sim.reset()
        self._wall0 = time.monotonic()
        self.get_logger().info("simulation reset")
        return response

    def _on_dice_roll(self, msg):
        self.last_dice_roll = msg.data

    def _tick(self):
        if self.viewer is not None and not self.viewer.is_running():
            self.get_logger().info("viewer closed, shutting down")
            rclpy.shutdown()
            return
        now = time.monotonic()
        target = (now - self._wall0) * self.rtf
        if self.sim.time < target - 0.5:      # can't keep up: drop the backlog
            self._wall0 = now - self.sim.time / self.rtf
            target = self.sim.time
        with self._lock():
            self.sim.step_to(target)
        if self.viewer is not None:
            roll = self.last_dice_roll if self.last_dice_roll is not None else "-"
            self.viewer.set_texts(
                (None, mujoco.mjtGridPos.mjGRID_TOPRIGHT, "Ludo dice", f"[ {roll} ]"))
            self.viewer.sync()

    def _publish_state(self):
        stamp = self.get_clock().now().to_msg()
        with self._lock():
            names, pos, vel = self.sim.joint_state()
            poses = [self.sim.brick_pose(c) for c in COLORS]

        js = JointState()
        js.header.stamp = stamp
        js.name, js.position, js.velocity = names, pos, vel
        self.joint_pub.publish(js)

        pa = PoseArray()
        pa.header.stamp = stamp
        pa.header.frame_id = "world"
        for (x, y, z), (qx, qy, qz, qw) in poses:
            p = Pose()
            p.position.x, p.position.y, p.position.z = x, y, z
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = qx, qy, qz, qw
            pa.poses.append(p)
        self.brick_pub.publish(pa)

    def _publish_cameras(self):
        stamp = self.get_clock().now().to_msg()
        for cam, pub in self.cam_pubs.items():
            with self._lock():
                img = self.sim.render(cam)
            msg = Image()
            msg.header.stamp = stamp
            msg.header.frame_id = cam
            msg.height, msg.width = int(img.shape[0]), int(img.shape[1])
            msg.encoding = "rgb8"
            msg.is_bigendian = 0
            msg.step = msg.width * 3
            msg.data = img.tobytes()
            pub.publish(msg)

    def shutdown(self):
        if self.viewer is not None:
            try:
                self.viewer.close()
            except Exception:
                pass
        self.sim.close()


def main(args=None):
    rclpy.init(args=args)
    node = MujocoNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
