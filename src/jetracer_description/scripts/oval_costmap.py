#!/usr/bin/env python3

import copy
import numpy as np
import rclpy

from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid, Path


# Parametry kosztów
MAX_PENALTY = 65
PENALTY_DISTANCE = 0.60  # [m]


class OvalCostmap(Node):

    def __init__(self):
        super().__init__('oval_costmap')

        self.map_msg = None
        self.track_msg = None
        self.generated = False

        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        self.create_subscription(
            OccupancyGrid, '/map',
            self.map_callback, map_qos
        )

        self.create_subscription(
            Path, '/track_path',
            self.track_callback, 10
        )

        self.publisher = self.create_publisher(
            OccupancyGrid, '/track_cost_map', map_qos
        )

    def map_callback(self, msg):
        self.map_msg = msg
        self.generate()

    def track_callback(self, msg):
        self.track_msg = msg
        self.generate()

    def generate(self):

        if self.generated:
            return

        if self.map_msg is None or self.track_msg is None:
            return

        m = self.map_msg
        width = m.info.width
        height = m.info.height
        res = m.info.resolution

        # Współrzędne środków komórek mapy.
        # Nasza mapa ma orientację origin równą 0.
        xs = m.info.origin.position.x + (
            np.arange(width) + 0.5
        ) * res

        ys = m.info.origin.position.y + (
            np.arange(height) + 0.5
        ) * res

        gx, gy = np.meshgrid(xs, ys)

        distance_sq = np.full(
            (height, width), np.inf
        )

        # Pobranie geometrii owalu.
        # Co trzeci punkt wystarcza przy rozdzielczości 3 cm.
        points = [
            (p.pose.position.x, p.pose.position.y)
            for p in self.track_msg.poses[::3]
        ]

        if len(points) < 3:
            return

        # Zamknięcie owalu
        points.append(points[0])

        # Odległość każdej komórki od najbliższego
        # odcinka trajektorii referencyjnej.
        for (ax, ay), (bx, by) in zip(
            points[:-1], points[1:]
        ):

            dx = bx - ax
            dy = by - ay
            length_sq = dx * dx + dy * dy

            if length_sq < 1e-12:
                continue

            t = np.clip(
                ((gx - ax) * dx + (gy - ay) * dy)
                / length_sq,
                0.0, 1.0
            )

            px = ax + t * dx
            py = ay + t * dy

            d2 = (gx - px)**2 + (gy - py)**2

            np.minimum(
                distance_sq, d2, out=distance_sq
            )

        distance = np.sqrt(distance_sq)

        # Koszt wzrasta wraz z odległością od owalu.
        penalty = np.clip(
            distance / PENALTY_DISTANCE,
            0.0, 1.0
        ) * MAX_PENALTY

        penalty = penalty.astype(np.int16)

        # Zachowanie oryginalnych przeszkód
        # i nieznanych obszarów.
        original = np.asarray(
            m.data, dtype=np.int16
        ).reshape(height, width)

        result = original.copy()

        known = original >= 0

        result[known] = np.maximum(
            original[known],
            penalty[known]
        )

        output = copy.deepcopy(m)
        output.header.stamp = self.get_clock().now().to_msg()
        output.data = result.flatten().tolist()

        self.publisher.publish(output)
        self.generated = True

        self.get_logger().info(
            'Opublikowano /track_cost_map'
        )


def main():

    rclpy.init()
    node = OvalCostmap()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
