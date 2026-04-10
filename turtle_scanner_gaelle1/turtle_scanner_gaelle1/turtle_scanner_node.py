#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import math


class TurtleScannerNode(Node):

    def __init__(self):
        super().__init__('turtle_scanner_node')

        # positions
        self.pose_scanner = None
        self.pose_target = None

        # publisher cmd_vel
        self.cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # subscriber scanner
        self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.scanner_callback,
            10
        )

        # serpentin
        self.waypoints = self.generate_serpentin()
        self.index = 0

        # timer 20 Hz
        self.timer = self.create_timer(0.05, self.scan_step)

        # gains
        self.Kp_lin = 1.0
        self.Kp_ang = 4.0
        self.tolerance = 0.3

        self.linear_speed_max = 2.0

    # CALLBACK POSE
    def scanner_callback(self, msg):
        self.pose_scanner = msg

    # SERPENTIN WAYPOINTS
    def generate_serpentin(self):
        waypoints = []

        nb_lignes = 5
        y_start = 1.0
        y_step = 2.0
        x_min = 1.0
        x_max = 10.0

        for i in range(nb_lignes):
            y = y_start + i * y_step

            if i % 2 == 0:
                waypoints.append((x_max, y))
                waypoints.append((x_min, y))
            else:
                waypoints.append((x_min, y))
                waypoints.append((x_max, y))

        return waypoints

    # DISTANCE
    def compute_distance(self, A, B):
        return math.sqrt((B[0] - A.x)**2 + (B[1] - A.y)**2)

    # ANGLE
    def compute_angle(self, A, B):
        return math.atan2(B[1] - A.y, B[0] - A.x)

    # SCAN STEP
    def scan_step(self):

        if self.pose_scanner is None or self.index >= len(self.waypoints):
            if self.index >= len(self.waypoints):
                self.stop()
                self.get_logger().info("Balayage terminé")
            return

        target = self.waypoints[self.index]

        distance = self.compute_distance(self.pose_scanner, target)
        angle_desired = self.compute_angle(self.pose_scanner, target)

        error_angle = math.atan(math.tan((angle_desired - self.pose_scanner.theta) / 2))

        cmd = Twist()

        if distance < self.tolerance:
            self.index += 1
        else:
            cmd.linear.x = min(self.Kp_lin * distance, self.linear_speed_max)
            cmd.angular.z = self.Kp_ang * error_angle

        self.cmd_pub.publish(cmd)

    def stop(self):
        cmd = Twist()
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = TurtleScannerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
