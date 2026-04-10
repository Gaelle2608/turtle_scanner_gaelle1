#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose


class TurtleScannerNode(Node):

    def __init__(self):
        super().__init__('turtle_scanner_node')

        # positions
        self.pose_scanner = None
        self.pose_target = None

        # subscriber turtle1
        self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.scanner_callback,
            10
        )

        # subscriber turtle_target
        self.create_subscription(
            Pose,
            '/turtle_target/pose',
            self.target_callback,
            10
        )

    def scanner_callback(self, msg):
        self.pose_scanner = msg
        self.get_logger().info(
            f"Scanner: x={msg.x:.2f}, y={msg.y:.2f}"
        )

    def target_callback(self, msg):
        self.pose_target = msg
        self.get_logger().info(
            f"Target: x={msg.x:.2f}, y={msg.y:.2f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = TurtleScannerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
