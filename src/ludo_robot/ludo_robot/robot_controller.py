#!/usr/bin/env python3
"""Turn-time controller for ONE robot arm. Run one instance per colour (see launch/robots.launch.py).

Subscribes
  /<color>/task            std_msgs/String       JSON task, see below
  /<color>/joint_states    sensor_msgs/JointState  (from ludo_sim state_publisher)
  /<color>/brick_pose      geometry_msgs/PoseStamped  actual position of this robot's piece

Publishes
  /<color>/joint_command   sensor_msgs/JointState  position targets for ludo_sim's mujoco_node
  /<color>/task_status     std_msgs/String         JSON {"id", "state", "detail"}
                           state: accepted | running | done | failed

Tasks (JSON on /<color>/task)
  {"id": 7, "type": "move_piece", "to": [x, y]}                 pick up own piece, put it at (x, y)
  {"id": 7, "type": "move_piece", "from": [x, y], "to": [x, y]} same, but pick up from (x, y)
  {"id": 8, "type": "park"}                                     go back to the ready pose
  {"id": 9, "type": "look_at", "at": [x, y]}                    turn the head towards (x, y)

If "from" is omitted the piece is picked up from where the simulator says it really is.

Parameters
  color (str)        red | green | yellow | blue
  rate (float)       control loop Hz                             [50]
  time_scale (float) simulation speed relative to wall clock     [1.0]  (= mujoco_node realtime_factor)
"""
import json
import time
from collections import deque

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from ludo_robot import kinematics as K
from ludo_robot import trajectory as T

try:
    from rclpy.executors import ExternalShutdownException
except ImportError:
    ExternalShutdownException = KeyboardInterrupt

COMMAND_NAMES = ["yaw", "shoulder", "elbow", "wrist", "grip", "head_pan", "head_tilt"]


class RobotController(Node):
    def __init__(self):
        super().__init__("robot_controller")
        self.declare_parameter("color", "red")
        self.declare_parameter("rate", 50.0)
        self.declare_parameter("time_scale", 1.0)
        self.color = self.get_parameter("color").value
        if self.color not in K.COLORS:
            raise ValueError(f"color must be one of {K.COLORS}, got '{self.color}'")
        self.time_scale = float(self.get_parameter("time_scale").value)
        rate = float(self.get_parameter("rate").value)

        self.geo = K.ArmGeometry.create_for_color(self.color)
        self.exec = T.MotionExecutor(self.geo)
        self.measured = {}
        self.brick_xy = None
        self.queue = deque()
        self.current = None          # task dict being executed
        self.recovering = False
        self.failure = ""
        self._last = time.monotonic()

        c = self.color
        self.cmd_pub = self.create_publisher(JointState, f"/{c}/joint_command", 10)
        self.status_pub = self.create_publisher(String, f"/{c}/task_status", 10)
        self.create_subscription(String, f"/{c}/task", self._on_task, 10)
        self.create_subscription(JointState, f"/{c}/joint_states", self._on_joints, 10)
        self.create_subscription(PoseStamped, f"/{c}/brick_pose", self._on_brick, 10)

        self._publish_command()      # make sure the actuators hold the ready pose
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(f"[{c}] ready, base at ({self.geo.base_x:+.2f}, {self.geo.base_y:+.2f})")

    # --------------------------------------------------------------- inputs
    def _on_joints(self, msg):
        self.measured = dict(zip(msg.name, msg.position))

    def _on_brick(self, msg):
        self.brick_xy = (msg.pose.position.x, msg.pose.position.y)

    def _on_task(self, msg):
        try:
            task = json.loads(msg.data)
            task["id"] = task.get("id", -1)
            if task.get("type") not in ("move_piece", "park", "look_at"):
                raise ValueError(f"unknown task type '{task.get('type')}'")
        except Exception as e:
            self._status(-1, "failed", f"bad task: {e}")
            return
        self.queue.append(task)
        self._status(task["id"], "accepted")

    # ----------------------------------------------------------------- loop
    def _tick(self):
        now = time.monotonic()
        dt = min(now - self._last, 0.2) * self.time_scale
        self._last = now

        if not self.exec.busy:
            if self.current is not None:
                self._finish_current()
            if self.queue:
                self._start(self.queue.popleft())
        if self.exec.busy:
            self.exec.update(dt, self.measured)
            self._publish_command()
        elif self.current is not None:
            self._finish_current()

    def _start(self, task):
        self.current, self.recovering, self.failure = task, False, ""
        kind = task["type"]
        if kind == "move_piece":
            src = task.get("from") or self.brick_xy
            if src is None or "to" not in task:
                return self._reject(task, "need 'to' and a known piece position")
            steps = T.pick_and_place_steps(tuple(src), tuple(task["to"]))
        elif kind == "park":
            steps = T.park_steps()
        else:   # look_at
            x, y = task["at"]
            steps = [T.Gaze((x, y, K.BRICK_REST_Z)), T.Settle(0.05, 1.0)]
        q_now = tuple(self.exec.cmd_q)
        err = T.validate_steps(self.geo, steps, q_now)
        if err:
            return self._reject(task, err)
        self.exec.begin(steps)
        self._status(task["id"], "running")

    def _reject(self, task, why):
        self._status(task["id"], "failed", why)
        self.current = None

    def _finish_current(self):
        task = self.current
        if self.exec.failed and not self.recovering:
            self.failure = self.exec.message
            self.get_logger().warn(f"[{self.color}] task {task['id']} failed: {self.failure}; recovering")
            self.recovering = True
            self.exec.begin(T.recovery_steps())
            return
        if self.recovering:
            self._status(task["id"], "failed", self.failure)
        else:
            self._status(task["id"], "done")
        self.current, self.recovering = None, False

    # -------------------------------------------------------------- outputs
    def _publish_command(self):
        c = self.exec.commands()
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(COMMAND_NAMES)
        msg.position = [float(c[n]) for n in COMMAND_NAMES]
        self.cmd_pub.publish(msg)

    def _status(self, task_id, state, detail=""):
        self.status_pub.publish(String(data=json.dumps({"id": task_id, "state": state, "detail": detail})))


def main(args=None):
    rclpy.init(args=args)
    node = RobotController()
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
