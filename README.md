# JetRacer - Raspberry Pi 4

## Hardware
- Raspberry Pi 4 (8 GB RAM)
- RPLIDAR A1M8-R5

## Software
- Ubuntu 24.04 Server (ARM64)
- ROS 2 Jazzy
- Slamtec sllidar_ros2

## Running LiDAR

source ~/ros2_ws/install/setup.bash

ros2 launch sllidar_ros2 sllidar_a1_launch.py

## Diagnostics

ros2 topic list
ros2 topic hz /scan
