#!/usr/bin/env python3

import math
import time
from pathlib import Path as FilePath

import numpy as np
import rclpy

from nav_msgs.msg import OccupancyGrid, Path as RosPath
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


# ============================================================
# PARAMETRY
# ============================================================

MAX_PENALTY = 65
PENALTY_DISTANCE = 0.30
MAX_DEVIATION = 0.30

MAP_TOPIC = "/map"
PATH_TOPIC = "/track_path"

OUTPUT_DIR = FilePath(__file__).resolve().parents[1] / "maps"


# ============================================================
# ODBIERANIE DANYCH Z ROS 2
# ============================================================

def receive_data():

    rclpy.init()
    node = rclpy.create_node("generate_oval_map")

    received = {
        "map": None,
        "path": None
    }

    map_qos = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL
    )

    map_sub = node.create_subscription(
        OccupancyGrid,
        MAP_TOPIC,
        lambda msg: received.__setitem__("map", msg),
        map_qos
    )

    path_sub = node.create_subscription(
        RosPath,
        PATH_TOPIC,
        lambda msg: received.__setitem__("path", msg),
        10
    )

    print("Oczekiwanie na /map oraz /track_path...")

    deadline = time.monotonic() + 15.0

    while time.monotonic() < deadline:

        if received["map"] is not None and received["path"] is not None:
            break

        rclpy.spin_once(node, timeout_sec=0.5)

    grid = received["map"]
    path = received["path"]

    node.destroy_node()
    rclpy.shutdown()

    if grid is None:
        raise RuntimeError("Nie otrzymano mapy /map")

    if path is None:
        raise RuntimeError("Nie otrzymano trajektorii /track_path")

    if grid.header.frame_id != path.header.frame_id:
        raise RuntimeError(
            "Mapa i trajektoria mają różne układy współrzędnych"
        )

    print("Odebrano mapę i trajektorię.")

    return grid, path


# ============================================================
# GENEROWANIE MAPY KOSZTÓW
# ============================================================

def generate_cost_map(grid, path):

    width = grid.info.width
    height = grid.info.height
    resolution = grid.info.resolution

    original = np.asarray(
        grid.data, dtype=np.int16
    ).reshape(height, width)

    # Punkty owalu pobrane bezpośrednio z /track_path
    points = np.array([
        [pose.pose.position.x, pose.pose.position.y]
        for pose in path.poses
    ], dtype=np.float64)

    if len(points) < 2:
        raise RuntimeError("Trajektoria zawiera za mało punktów")

    # Ograniczenie liczby odcinków
    sampled = points[::3]

    if not np.allclose(sampled[-1], points[-1]):
        sampled = np.vstack((sampled, points[-1]))

    # Zamknięcie owalu
    if not np.allclose(sampled[-1], sampled[0]):
        sampled = np.vstack((sampled, sampled[0]))

    # --------------------------------------------------------
    # Współrzędne komórek mapy w układzie globalnym
    # --------------------------------------------------------

    rows, cols = np.indices((height, width))

    origin = grid.info.origin
    q = origin.orientation

    yaw = math.atan2(
        2 * (q.w * q.z + q.x * q.y),
        1 - 2 * (q.y**2 + q.z**2)
    )

    local_x = (cols + 0.5) * resolution
    local_y = (rows + 0.5) * resolution

    x = (
        origin.position.x
        + local_x * math.cos(yaw)
        - local_y * math.sin(yaw)
    )

    y = (
        origin.position.y
        + local_x * math.sin(yaw)
        + local_y * math.cos(yaw)
    )

    # --------------------------------------------------------
    # Odległość każdej komórki od owalu
    # --------------------------------------------------------

    distance_squared = np.full(
        (height, width), np.inf
    )

    for a, b in zip(sampled[:-1], sampled[1:]):

        dx = b[0] - a[0]
        dy = b[1] - a[1]

        length_squared = dx**2 + dy**2

        if length_squared == 0:
            continue

        # Rzut punktu na odcinek trajektorii
        t = np.clip(
            ((x - a[0]) * dx + (y - a[1]) * dy)
            / length_squared,
            0.0,
            1.0
        )

        nearest_x = a[0] + t * dx
        nearest_y = a[1] + t * dy

        d2 = (
            (x - nearest_x)**2
            + (y - nearest_y)**2
        )

        distance_squared = np.minimum(
            distance_squared, d2
        )

    distance = np.sqrt(distance_squared)

    # --------------------------------------------------------
    # Wyznaczenie dodatkowych kosztów
    # --------------------------------------------------------

    penalty = np.clip(
        distance / PENALTY_DISTANCE * MAX_PENALTY,
        0,
        MAX_PENALTY
    ).astype(np.int16)

    # Zachowujemy przeszkody i obszary nieznane.
    # Dodatkowy koszt nie może zmniejszyć kosztu oryginalnego.

    result = np.where(
        original == -1,
        -1,
        np.maximum(original, penalty)
    )
    # Obszar poza korytarzem MAX_DEVIATION jest nieprzejezdny
    result[distance > MAX_DEVIATION] = 100

    return result, yaw


# ============================================================
# ZAPIS MAPY DO PLIKU
# ============================================================

def save_pgm(filename, array):

    height, width = array.shape

    # Obraz PGM ma początek w lewym górnym rogu,
    # natomiast OccupancyGrid w lewym dolnym.
    image = np.flipud(array).astype(np.uint8)

    header = f"P5\n{width} {height}\n255\n"

    with open(filename, "wb") as file:
        file.write(header.encode("ascii"))
        file.write(image.tobytes())


def save_maps(grid, cost_map, yaw):

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    # --------------------------------------------------------
    # Mapa RAW do późniejszego wczytania przez Nav2
    # --------------------------------------------------------

    # W trybie RAW:
    # 0-100 = wartości komórek
    # 255 = obszar nieznany

    raw_image = np.where(
        cost_map == -1,
        255,
        cost_map
    ).astype(np.uint8)

    save_pgm(
        OUTPUT_DIR / "map03_oval_raw.pgm",
        raw_image
    )

    # --------------------------------------------------------
    # Mapa poglądowa
    # --------------------------------------------------------

    # Biały = mały koszt
    # Czarny = przeszkoda
    # Szary = koszt pośredni
    # Obszary nieznane = jasnoszary

    preview = np.where(
        cost_map == -1,
        205,
        254 - np.clip(cost_map, 0, 100) * 2.54
    ).astype(np.uint8)

    save_pgm(
        OUTPUT_DIR / "map03_oval_preview.pgm",
        preview
    )

    # --------------------------------------------------------
    # Plik YAML dla Map Server
    # --------------------------------------------------------

    origin = grid.info.origin.position

    yaml_content = (
        "image: map03_oval_raw.pgm\n"
        "mode: raw\n"
        f"resolution: {grid.info.resolution}\n"
        f"origin: [{origin.x}, {origin.y}, {yaw}]\n"
        "negate: 0\n"
        "occupied_thresh: 0.65\n"
        "free_thresh: 0.196\n"
    )

    (OUTPUT_DIR / "map03_oval.yaml").write_text(
        yaml_content
    )


# ============================================================
# MAIN
# ============================================================

def main():

    grid, path = receive_data()

    print("Generowanie mapy kosztów...")

    cost_map, yaw = generate_cost_map(grid, path)

    save_maps(grid, cost_map, yaw)

    print("\nZakończono generowanie.")
    print("Rozdzielczość:", grid.info.resolution, "m")
    print("Rozmiar:", grid.info.width, "x", grid.info.height)
    print("Punkty owalu:", len(path.poses))

    print(
        "Komórki z kosztami 1-99:",
        np.count_nonzero((cost_map > 0) & (cost_map < 100))
    )

    print("Zapisano pliki w:", OUTPUT_DIR)


if __name__ == "__main__":
    main()
