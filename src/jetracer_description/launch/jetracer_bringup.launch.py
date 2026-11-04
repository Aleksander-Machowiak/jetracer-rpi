from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def include_launch(package, filename, arguments=None):
    """Uruchamia istniejący plik launch z wybranego pakietu."""
    path = Path(get_package_share_directory(package)) / "launch" / filename

    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(path)),
        launch_arguments=(arguments or {}).items(),
    )


def generate_launch_description():

    # 1. Intel RealSense T265 - odometria
    t265 = include_launch(
        "realsense2_camera",
        "rs_launch.py",
        {
            "enable_pose": "true",
            "enable_fisheye1": "false",
            "enable_fisheye2": "false",
        },
    )

    # 2. RPLIDAR A1M8-R5 - skanowanie laserowe
    lidar = include_launch(
        "sllidar_ros2",
        "sllidar_a1_launch.py"
    )

    # 3. Model URDF/Xacro JetRacera
    robot_model = include_launch(
        "jetracer_description",
        "jetracer_model.launch.py"
    )

    # 4. Transformacja TF: camera_pose_frame -> base_link
    camera_to_base_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="camera_pose_to_base_link_tf",
        output="screen",
        arguments=[
            "--x", "-0.21595",
            "--y", "-0.00910",
            "--z", "-0.02225",
            "--roll", "0",
            "--pitch", "0",
            "--yaw", "0",
            "--frame-id", "camera_pose_frame",
            "--child-frame-id", "base_link",
        ],
    )

    # Uruchomienie wszystkich elementów
    return LaunchDescription([
        t265,
        lidar,
        robot_model,
        camera_to_base_tf
    ])
