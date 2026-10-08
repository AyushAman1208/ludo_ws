#!/usr/bin/env python3
"""Turns the raw simulator output into per-player state.

Subscribes
  /mujoco/joint_states   sensor_msgs/JointState
  /mujoco/brick_poses    geometry_msgs/PoseArray

Publishes
  /<color>/joint_states  sensor_msgs/JointState   names without the colour prefix
                                                  (yaw, shoulder, elbow, wrist, finger_l, ...)
  /<color>/brick_pose    geometry_msgs/PoseStamped
  /ludo/state            std_msgs/String          JSON: where each piece is, lifted or not
  /tf                    world -> <color>_brick   (dynamic)
  /tf_static             world -> <color>_base    (robot mounting points)
"""
import json
import math

import rclpy
from geometry_msgs.msg import PoseArray, PoseStamped, TransformStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from ludo_sim.sim_core import COLORS

try:
    from rclpy.executors import ExternalShutdownException
except ImportError:
    ExternalShutdownException = KeyboardInterrupt

# Must match models/robots/*.xml and models/board.xml
BASE_DIST = 0.46
SIGN = {"red": (-1, -1), "green": (-1, 1), "yellow": (1, 1), "blue": (1, -1)}   # (sx, sy)
YARD_MIN, YARD_MAX = 0.075, 0.375          # yard spans 0.075..0.375 m from centre on each axis
LIFT_Z = 0.065                             # resting piece centre is at z = 0.054


def in_yard(color, x, y):
    sx, sy = SIGN[color]
    return (YARD_MIN <= sx * x <= YARD_MAX) and (YARD_MIN <= sy * y <= YARD_MAX)


class StatePublisher(Node):
    def __init__(self):
        super().__init__("state_publisher")
        self.declare_parameter("state_topic_rate", 5.0)

        self.joint_pubs = {c: self.create_publisher(JointState, f"/{c}/joint_states", 10) for c in COLORS}
        self.pose_pubs = {c: self.create_publisher(PoseStamped, f"/{c}/brick_pose", 10) for c in COLORS}
        self.state_pub = self.create_publisher(String, "/ludo/state", 10)
        self.tf = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)

        self.last_poses = None
        self.create_subscription(JointState, "/mujoco/joint_states", self._on_joints, 10)
        self.create_subscription(PoseArray, "/mujoco/brick_poses", self._on_bricks, 10)
        self.create_timer(1.0 / float(self.get_parameter("state_topic_rate").value), self._publish_state)

        self._publish_static()

    # ------------------------------------------------------------------ static
    def _publish_static(self):
        stamp = self.get_clock().now().to_msg()
        msgs = []
        for c in COLORS:
            sx, sy = SIGN[c]
            x, y = sx * BASE_DIST, sy * BASE_DIST
            yaw = math.atan2(-y, -x)                      # robot +x faces the board centre
            t = TransformStamped()
            t.header.stamp, t.header.frame_id, t.child_frame_id = stamp, "world", f"{c}_base"
            t.transform.translation.x, t.transform.translation.y = x, y
            t.transform.rotation.z, t.transform.rotation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            msgs.append(t)
        self.static_tf.sendTransform(msgs)

    # ----------------------------------------------------------------- inputs
    def _on_joints(self, msg):
        for c in COLORS:
            prefix = c + "_"
            out = JointState()
            out.header = msg.header
            for i, name in enumerate(msg.name):
                if name.startswith(prefix):
                    out.name.append(name[len(prefix):])
                    out.position.append(msg.position[i])
                    if i < len(msg.velocity):
                        out.velocity.append(msg.velocity[i])
            self.joint_pubs[c].publish(out)

    def _on_bricks(self, msg):
        self.last_poses = msg
        tfs = []
        for c, pose in zip(COLORS, msg.poses):
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose = pose
            self.pose_pubs[c].publish(ps)

            t = TransformStamped()
            t.header = msg.header
            t.child_frame_id = f"{c}_brick"
            t.transform.translation.x = pose.position.x
            t.transform.translation.y = pose.position.y
            t.transform.translation.z = pose.position.z
            t.transform.rotation = pose.orientation
            tfs.append(t)
        self.tf.sendTransform(tfs)

    def _publish_state(self):
        if self.last_poses is None:
            return
        pieces = {}
        for c, pose in zip(COLORS, self.last_poses.poses):
            x, y, z = pose.position.x, pose.position.y, pose.position.z
            pieces[c] = {
                "x": round(x, 4), "y": round(y, 4), "z": round(z, 4),
                "in_own_yard": in_yard(c, x, y),
                "lifted": z > LIFT_Z,
            }
        self.state_pub.publish(String(data=json.dumps({"pieces": pieces})))


def main(args=None):
    rclpy.init(args=args)
    node = StatePublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
