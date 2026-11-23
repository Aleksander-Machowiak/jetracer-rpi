#!/usr/bin/env python3

import time
import rclpy

from rclpy.node import Node
from rclpy.action import ActionClient
from nav_msgs.msg import Path
from nav2_msgs.action import SmoothPath


# Fragment owalu do przetestowania



def main():
    rclpy.init()
    node = Node("test_constrained_smoother")

    received = {"path": None}

    def path_callback(msg):
        received["path"] = msg

    node.create_subscription(
        Path, "/track_path", path_callback, 10
    )

    print("Oczekiwanie na /track_path...")

    start_time = time.monotonic()

    while received["path"] is None:
        rclpy.spin_once(node, timeout_sec=0.2)

        if time.monotonic() - start_time > 10:
            print("BLAD: Nie otrzymano /track_path")
            node.destroy_node()
            rclpy.shutdown()
            return

    original = received["path"]

    # Wybieramy fragment owalu
    test_path = Path()
    test_path.header = original.header
    test_path.poses = original.poses[:]

    print(f"Liczba punktow: {len(test_path.poses)}")

    # Klient akcji SmoothPath
    client = ActionClient(
        node, SmoothPath, "/smooth_path"
    )

    if not client.wait_for_server(timeout_sec=10.0):
        print("BLAD: Smoother Server niedostepny")
        node.destroy_node()
        rclpy.shutdown()
        return

    goal = SmoothPath.Goal()
    goal.path = test_path
    goal.smoother_id = "Constrained"
    goal.max_smoothing_duration.sec = 30
    goal.check_for_collisions = False

    print("Wysylanie sciezki do Constrained Smoother...")

    future = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, future)

    handle = future.result()

    if handle is None or not handle.accepted:
        print("BLAD: Cel zostal odrzucony")
        node.destroy_node()
        rclpy.shutdown()
        return

    result_future = handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future)

    result = result_future.result().result

    print(f"Kod wyniku: {result.error_code}")
    print(f"Optymalizacja zakonczona: {result.was_completed}")

    smoothed = result.path

    if not smoothed.poses:
        print("BLAD: Brak zmodyfikowanej sciezki")
        node.destroy_node()
        rclpy.shutdown()
        return

    print(f"Punktow po wygladzeniu: {len(smoothed.poses)}")

    # Publikacja do RViz
    pub_original = node.create_publisher(
        Path, "/test_oval_original", 10
    )

    pub_smoothed = node.create_publisher(
        Path, "/test_oval_smoothed", 10
    )

    def publish_paths():
        pub_original.publish(test_path)
        pub_smoothed.publish(smoothed)

    node.create_timer(1.0, publish_paths)

    print("Publikowanie obu sciezek. Ctrl+C konczy program.")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
