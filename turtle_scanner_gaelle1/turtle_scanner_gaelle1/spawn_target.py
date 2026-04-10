#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn
import random


class SpawnTarget(Node):

    def __init__(self):
        super().__init__('spawn_target')

        # Création du client pour le service /spawn
        self.client = self.create_client(Spawn, '/spawn')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service /spawn non disponible, attente...')

        # Générer des coordonnées aléatoires
        x = random.uniform(1.0, 10.0)
        y = random.uniform(1.0, 10.0)
        theta = random.uniform(0.0, 6.28)

        # Créer la requête
        self.req = Spawn.Request()
        self.req.x = x
        self.req.y = y
        self.req.theta = theta
        self.req.name = 'turtle_target'

        # Appel du service
        self.future = self.client.call_async(self.req)
        self.future.add_done_callback(self.callback)

    def callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(
                f"Tortue spawnée : {response.name} aux coordonnées x={self.req.x:.2f}, y={self.req.y:.2f}"
            )
        except Exception as e:
            self.get_logger().error(f"Erreur lors du spawn : {e}")


def main(args=None):
    rclpy.init(args=args)
    node = SpawnTarget()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
