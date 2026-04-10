#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import math


class TurtleScannerNode(Node):

    def __init__(self):
        super().__init__('turtle_scanner_node')

        self.pose_scanner = None
        self.pose_target = None

        self.cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.scanner_callback,
            10
        )

        self.waypoints = self.generate_serpentin()
        self.index = 0

        self.timer = self.create_timer(0.05, self.scan_step)

        self.Kp_lin = 1.0
        self.Kp_ang = 4.0
        self.tolerance = 0.3
        self.linear_speed_max = 2.0

        self.detection_radius = 1.5

        self.detect_pub = self.create_publisher(
            Bool,
            '/target_detected',
            10
        )

        self.target_detected = False

    #CALLBACK
    def scanner_callback(self, msg):
        self.pose_scanner = msg

    #SERPENTIN
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

    #DISTANCE
    def compute_distance(self, A, B):
        return math.sqrt((B[0] - A.x)**2 + (B[1] - A.y)**2)

    #ANGLE
    def compute_angle(self, A, B):
        return math.atan2(B[1] - A.y, B[0] - A.x)

    # STOP
    def stop(self):
        cmd = Twist()
        self.cmd_pub.publish(cmd)

    # MAIN LOOP 
    def scan_step(self):

        if self.pose_scanner is None:
            return

        # DETECTION CIBLE 
        if self.pose_target is not None:

            dist_target = math.sqrt(
                (self.pose_target.x - self.pose_scanner.x) ** 2 +
                (self.pose_target.y - self.pose_scanner.y) ** 2
            )

            if dist_target < self.detection_radius:

                self.stop()

                msg = Bool()
                msg.data = True
                self.detect_pub.publish(msg)

                self.get_logger().info(
                    f"Cible détectée à ({self.pose_target.x:.2f}, {self.pose_target.y:.2f}) !"
                )
                return

        # pas détecté
        msg = Bool()
        msg.data = False
        self.detect_pub.publish(msg)

        # ===== BALAYAGE =====
        if self.index >= len(self.waypoints):
            self.stop()
            self.get_logger().info("Balayage terminé")
            return

        target = self.waypoints[self.index]

        distance = self.compute_distance(self.pose_scanner, target)
        angle_desired = self.compute_angle(self.pose_scanner, target)

        error_angle = math.atan(
            math.tan((angle_desired - self.pose_scanner.theta) / 2)
        )

        cmd = Twist()

        if distance < self.tolerance:
            self.index += 1
        else:
            cmd.linear.x = min(self.Kp_lin * distance, self.linear_speed_max)
            cmd.angular.z = self.Kp_ang * error_angle

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = TurtleScannerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()