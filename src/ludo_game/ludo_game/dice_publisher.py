#!/usr/bin/env python3
"""Publish random dice rolls for a player node to consume."""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

from ludo_game.rules import Dice

try:
    from rclpy.executors import ExternalShutdownException
except ImportError:
    ExternalShutdownException = KeyboardInterrupt


class DicePublisher(Node):
    def __init__(self):
        super().__init__("dice_publisher")
        self.declare_parameter("dice_seed", -1)
        self.declare_parameter("dice_rate", 0.2)

        seed = int(self.get_parameter("dice_seed").value)
        rate = float(self.get_parameter("dice_rate").value)
        if rate <= 0.0:
            raise ValueError(f"dice_rate must be positive, got {rate}")

        self.dice = Dice(None if seed < 0 else seed)
        self.dice_pub = self.create_publisher(Int32, "/ludo/dice", 10)
        self.create_timer(1.0 / rate, self._publish_roll)
        self.get_logger().info(f"publishing random dice rolls at {rate:g} Hz")

    def _publish_roll(self):
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
