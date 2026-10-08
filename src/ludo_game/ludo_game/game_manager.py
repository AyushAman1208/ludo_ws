#!/usr/bin/env python3
"""Orchestrates a Ludo game between the four robot arms.

For every turn: roll the die -> let the rules decide the move -> (if it captures) make the
victim's robot put its piece back in its yard -> make the mover's robot carry its piece to the
new cell -> check with the simulator that the piece really is there -> next player.

Subscribes
  /<color>/task_status   std_msgs/String           from ludo_robot
  /<color>/brick_pose    geometry_msgs/PoseStamped  where each piece really is
Publishes
  /<color>/task          std_msgs/String           JSON tasks for ludo_robot
  /ludo/dice             std_msgs/Int32            every roll
  /ludo/game_state       std_msgs/String           JSON: phase, turn, roll, progress, winner, ...
Services
  /ludo/start            std_srvs/Trigger          start (or restart the turn loop) when ready
  /ludo/new_game         std_srvs/Trigger          reset the simulator and the game

Parameters
  auto_start (bool)      start as soon as all four robots are up        [true]
  start_delay (float)    seconds to wait before the first roll          [2.0]
  turn_delay (float)     pause between turns                            [1.0]
  dice_seed (int)        -1 = random                                    [-1]
  dice_sequence (int[])  fixed first rolls (e.g. for demos / tests)     []
  first_player (str)     which colour starts                            [red]
  verify_tol (float)     max distance (m) between a piece and its cell  [0.03]
  max_retries (int)      re-tries of a robot task that failed           [2]
  task_timeout (float)   wall seconds to wait for a robot                [120.0]
"""
import json
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import Int32, String
from std_srvs.srv import Empty, Trigger

from ludo_game import board_map as B
from ludo_game.rules import Dice, Game

try:
    from rclpy.executors import ExternalShutdownException
except ImportError:
    ExternalShutdownException = KeyboardInterrupt


class GameManager(Node):
    def __init__(self):
        super().__init__("game_manager")
        P = self.declare_parameter
        P("auto_start", True)
        P("start_delay", 2.0)
        P("turn_delay", 1.0)
        P("dice_seed", -1)
        P("dice_sequence", [0])              # [0] = "none" (empty arrays can't be declared)
        P("first_player", "red")
        P("verify_tol", 0.03)
        P("max_retries", 2)
        P("task_timeout", 120.0)
        g = lambda k: self.get_parameter(k).value
        self.auto_start, self.start_delay, self.turn_delay = g("auto_start"), g("start_delay"), g("turn_delay")
        self.tol, self.max_retries, self.task_timeout = g("verify_tol"), g("max_retries"), g("task_timeout")
        self.first = g("first_player")
        seed = g("dice_seed")
        seq = [int(v) for v in g("dice_sequence") if int(v) > 0]
        self.dice = Dice(None if seed < 0 else seed, seq)

        self.game = Game(first=self.first)
        self.phase = "waiting_for_robots"
        self.last_roll = 0
        self.last_event = ""
        self.error = ""
        self.move_count = 0
        self.actual = {}                     # colour -> (x, y, z) from the simulator
        self.task_pubs, self.statuses = {}, {}
        self.next_id = 1
        self.job = None                      # the robot task currently being waited for
        self.plan = []                       # remaining jobs of this turn
        self.pending_move = None
        self.wait_until = 0.0
        self._started_at = time.monotonic()
        self._ready_since = None

        for c in B.COLORS:
            self.task_pubs[c] = self.create_publisher(String, f"/{c}/task", 10)
            self.create_subscription(String, f"/{c}/task_status",
                                     lambda m, color=c: self._on_status(color, m), 10)
            self.create_subscription(PoseStamped, f"/{c}/brick_pose",
                                     lambda m, color=c: self._on_pose(color, m), 10)
        self.dice_pub = self.create_publisher(Int32, "/ludo/dice", 10)
        self.state_pub = self.create_publisher(String, "/ludo/game_state", 10)
        self.create_service(Trigger, "/ludo/start", self._srv_start)
        self.create_service(Trigger, "/ludo/new_game", self._srv_new_game)
        self.reset_client = self.create_client(Empty, "/mujoco/reset")

        self.create_timer(0.1, self._tick)
        self.create_timer(0.5, self._publish_state)
        self.get_logger().info("game manager up; waiting for the four robots and the simulator")

    # ------------------------------------------------------------- callbacks
    def _on_pose(self, color, msg):
        p = msg.pose.position
        self.actual[color] = (p.x, p.y, p.z)

    def _on_status(self, color, msg):
        try:
            s = json.loads(msg.data)
        except ValueError:
            return
        self.statuses[(color, s.get("id"))] = s

    def _srv_start(self, request, response):
        if self.phase in ("ready", "waiting_for_robots", "paused"):
            self.phase, self.wait_until = "ready", 0.0
            self.auto_start = True
            response.success, response.message = True, "starting"
        else:
            response.success, response.message = False, f"game is in phase '{self.phase}'"
        return response

    def _srv_new_game(self, request, response):
        if self.reset_client.service_is_ready():
            self.reset_client.call_async(Empty.Request())
        self.game = Game(first=self.first)
        self.plan, self.job, self.pending_move, self.error = [], None, None, ""
        self.move_count, self.last_roll = 0, 0
        self.phase, self.wait_until = "ready", time.monotonic() + 3.0
        response.success, response.message = True, "new game"
        return response

    # -------------------------------------------------------------- helpers
    def _robots_up(self):
        return (len(self.actual) == 4 and
                all(p.get_subscription_count() > 0 for p in self.task_pubs.values()))

    def _send(self, color, task):
        task = dict(task, id=self.next_id)
        self.next_id += 1
        self.task_pubs[color].publish(String(data=json.dumps(task)))
        return task["id"]

    def _near(self, color, xy):
        a = self.actual.get(color)
        return a is not None and ((a[0] - xy[0]) ** 2 + (a[1] - xy[1]) ** 2) ** 0.5 <= self.tol

    def _publish_state(self):
        self.state_pub.publish(String(data=json.dumps({
            "phase": self.phase, "turn": self.game.current, "last_roll": self.last_roll,
            "progress": self.game.progress, "winner": self.game.winner,
            "moves": self.move_count, "event": self.last_event, "error": self.error,
        })))

    # ---------------------------------------------------------- state machine
    def _tick(self):
        now = time.monotonic()

        if self.phase == "waiting_for_robots":
            if self._robots_up():
                self._ready_since = self._ready_since or now
                if now - self._ready_since >= self.start_delay:
                    self.phase = "ready"
                    self.get_logger().info("all robots up")
            return

        if self.phase == "ready":
            if self.auto_start and now >= self.wait_until:
                self.phase = "rolling"
            return

        if self.phase == "rolling":
            if now < self.wait_until:
                return
            self._roll()
            return

        if self.phase == "executing":
            self._run_plan(now)
            return
        # paused / finished / error: nothing to do

    def _roll(self):
        color = self.game.current
        roll = self.dice.roll()
        self.last_roll = roll
        self.dice_pub.publish(Int32(data=roll))
        move = self.game.propose(color, roll)
        if move is None:
            self.last_event = f"{color} rolled {roll}: no move"
            self.get_logger().info(self.last_event)
            self._finish_turn(roll)
            return
        self.last_event = (f"{color} rolled {roll}: progress {move.old_progress} -> {move.new_progress}"
                           + (f", captures {move.captured}" if move.captured else ""))
        self.get_logger().info(self.last_event)
        self.pending_move = move

        # plan: victim goes home first, then the mover walks to its new cell
        self.plan = []
        if move.captured:
            self.plan.append({"color": move.captured,
                              "to": B.YARD_XY[move.captured], "label": f"{move.captured} back to yard"})
        self.plan.append({"color": color, "to": B.position_xy(color, move.new_progress),
                          "label": f"{color} to progress {move.new_progress}"})
        self.job, self.phase = None, "executing"

    def _run_plan(self, now):
        if self.job is None:
            if not self.plan:
                return self._complete_move()
            job = self.plan[0]
            src = self.actual.get(job["color"])
            tid = self._send(job["color"], {"type": "move_piece", "to": list(job["to"]),
                                            **({"from": [src[0], src[1]]} if src else {})})
            self.job = dict(job, id=tid, sent=now, tries=job.get("tries", 0))
            self.get_logger().info(f"-> {job['label']} (task {tid})")
            return

        st = self.statuses.get((self.job["color"], self.job["id"]))
        if st and st["state"] in ("done", "failed"):
            ok = st["state"] == "done" and self._near(self.job["color"], self.job["to"])
            if ok:
                self.plan.pop(0)
                self.job = None
                return
            reason = st.get("detail") or "piece is not where it should be"
            self._retry_or_stop(reason)
        elif now - self.job["sent"] > self.task_timeout:
            self._retry_or_stop("timeout waiting for the robot")

    def _retry_or_stop(self, reason):
        job = self.job
        job["tries"] += 1
        if job["tries"] > self.max_retries:
            self.phase = "error"
            self.error = f"{job['label']}: {reason}"
            self.get_logger().error(f"giving up: {self.error}")
            return
        self.get_logger().warn(f"{job['label']} failed ({reason}); retry {job['tries']}/{self.max_retries}")
        self.plan[0] = {k: job[k] for k in ("color", "to", "label")} | {"tries": job["tries"]}
        self.job = None

    def _complete_move(self):
        move = self.pending_move
        self.game.apply(move)
        self.move_count += 1
        self.pending_move = None
        if self.game.winner:
            self.phase = "finished"
            self.last_event = f"{self.game.winner} wins!"
            self.get_logger().info(self.last_event)
            return
        self._finish_turn(move.roll)

    def _finish_turn(self, roll):
        self.game.end_turn(roll)
        self.phase = "rolling"
        self.wait_until = time.monotonic() + self.turn_delay


def main(args=None):
    rclpy.init(args=args)
    node = GameManager()
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
