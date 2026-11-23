#!/usr/bin/env python3

import math
import rclpy

from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


# Geometria owalu [m]
R = 0.715
L = 0.98129

# Położenie na mapie [m]
CX = 1.1074
CY = 0.6770

# Obrót owalu
ANGLE = math.radians(-0.69)


class TrackPath(Node):

    def __init__(self):
        super().__init__('track_path')

        self.publisher = self.create_publisher(
            Path, '/track_path', 10
        )

        self.path = Path()
        self.path.header.frame_id = 'map'

        self.generate_path()

        self.timer = self.create_timer(
            1.0, self.publish_path
        )

    def add_point(self, x, y, yaw):

        # Obrót i przesunięcie na mapę
        mx = CX + x * math.cos(ANGLE) - y * math.sin(ANGLE)
        my = CY + x * math.sin(ANGLE) + y * math.cos(ANGLE)

        yaw += ANGLE

        pose = PoseStamped()
        pose.header.frame_id = 'map'

        pose.pose.position.x = mx
        pose.pose.position.y = my

        pose.pose.orientation.z = math.sin(yaw / 2)
        pose.pose.orientation.w = math.cos(yaw / 2)

        self.path.poses.append(pose)

    def generate_path(self):

        # Dolny odcinek prosty
        for i in range(61):
            x = -L / 2 + L * i / 60
            self.add_point(x, -R, 0)

        # Prawy półokrąg
        for i in range(1, 91):
            a = -math.pi / 2 + math.pi * i / 90

            x = L / 2 + R * math.cos(a)
            y = R * math.sin(a)

            self.add_point(x, y, a + math.pi / 2)

        # Górny odcinek prosty
        for i in range(1, 61):
            x = L / 2 - L * i / 60
            self.add_point(x, R, math.pi)

        # Lewy półokrąg
        for i in range(1, 91):
            a = math.pi / 2 + math.pi * i / 90

            x = -L / 2 + R * math.cos(a)
            y = R * math.sin(a)

            self.add_point(x, y, a + math.pi / 2)

    def publish_path(self):

        stamp = self.get_clock().now().to_msg()

        self.path.header.stamp = stamp

        for pose in self.path.poses:
            pose.header.stamp = stamp

        self.publisher.publish(self.path)


def main():
    rclpy.init()
    node = TrackPath()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
