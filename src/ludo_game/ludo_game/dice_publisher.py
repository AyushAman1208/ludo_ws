#!/usr/bin/env python3
"""Publish one random dice roll for each explicit request."""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, Int32

from ludo_game.rules import Dice

try:
    from rclpy.executors import ExternalShutdownException
except ImportError:
    ExternalShutdownException = KeyboardInterrupt


class DicePublisher(Node):
    def __init__(self):
        super().__init__("dice_publisher")
        self.declare_parameter("dice_seed", -1)

        seed = int(self.get_parameter("dice_seed").value)
        self.dice = Dice(None if seed < 0 else seed)
        self.dice_pub = self.create_publisher(Int32, "/ludo/dice", 10)
        self.create_subscription(Empty, "/ludo/roll_request", self._on_roll_request, 10)
        self.get_logger().info("waiting for requests on /ludo/roll_request")

    def _on_roll_request(self, _request):
        roll = self.dice.roll()
        self.dice_pub.publish(Int32(data=roll))
        self.get_logger().info(f"published dice roll: {roll}")


def main(args=None):
    rclpy.init(args=args)
    node = DicePublisher()
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
