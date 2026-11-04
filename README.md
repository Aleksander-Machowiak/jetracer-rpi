# JetRacer - Raspberry Pi 4

ROS 2 environment for the JetRacer autonomous vehicle, responsible for sensor integration, odometry, robot visualization, and future mapping and navigation.

## Hardware
- Raspberry Pi 4 (8 GB RAM)
- RPLIDAR A1M8-R5
- Intel RealSense T265
- NVIDIA Jetson Nano (planned integration)

## Software
- Ubuntu 24.04 Server (ARM64)
- ROS 2 Jazzy
- Slamtec sllidar_ros2
- Intel RealSense SDK (librealsense)
- realsense-ros
- RViz2 (Ubuntu laptop)

## ROS 2 Workspaces

- `~/ros2_ws` - RPLIDAR driver and JetRacer description
- `~/t265_ws` - Intel RealSense T265 ROS 2 driver

## Project Structure

```text
ros2_ws/
├── src/
│   ├── sllidar_ros2/
│   └── jetracer_description/
│       ├── launch/
│       │   ├── jetracer_bringup.launch.py
│       │   └── jetracer_model.launch.py
│       ├── urdf/
│       │   └── jetracer.urdf.xacro
│       └── meshes/
│           └── *.stl
└── README.md
```

## Running JetRacer

### Raspberry Pi 4 (SSH)

Connect from the laptop:

```bash
ssh rp4
```

Verify Intel RealSense T265 detection:

```bash
rs-enumerate-devices
lsusb
```

Source ROS 2 environments:

```bash
source /opt/ros/jazzy/setup.bash
source ~/t265_ws/install/setup.bash
source ~/ros2_ws/install/setup.bash
```

Launch the system:

```bash
ros2 launch jetracer_description jetracer_bringup.launch.py
```

The bringup launch file starts:
- Intel RealSense T265 (odometry)
- RPLIDAR A1 (laser scans)
- JetRacer URDF model (robot_state_publisher)
- Static TF transformation (`camera_pose_frame` → `base_link`)

### Laptop (RViz2)

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
rviz2
```

Load the saved RViz2 configuration.

Set `Fixed Frame` to `odom_frame`.

## ROS 2 Topics and TF

Main topics:
- `/scan` - RPLIDAR LaserScan
- `/camera/pose/sample` - T265 odometry
- `/robot_description` - JetRacer URDF model

TF structure:

```text
odom_frame
└── camera_pose_frame
    └── base_link
        ├── laser
        ├── t265_tracking_center
        ├── front_left_wheel
        ├── front_right_wheel
        ├── rear_left_wheel
        └── rear_right_wheel
```

## Diagnostics

List ROS 2 topics:

```bash
ros2 topic list
```

Check LiDAR scan frequency:

```bash
ros2 topic hz /scan
```

Verify odometry transformation:

```bash
ros2 run tf2_ros tf2_echo odom_frame base_link
```

## Current Status (04.11.2026)

- [x] RPLIDAR integration with ROS 2
- [x] Intel RealSense T265 odometry
- [x] JetRacer URDF/Xacro model
- [x] TF tree configuration
- [x] RViz2 visualization
- [x] Unified bringup launch file
- [x] Successful startup test after Raspberry Pi reboot
- [ ] SLAM Toolbox integration
- [ ] Autonomous navigation (Nav2)
- [ ] Jetson Nano and Raspberry Pi communication
- [ ] Autonomous obstacle avoidance

## Shutdown

Stop ROS 2 processes using `Ctrl+C`.

Safely shut down Raspberry Pi:

```bash
sudo poweroff
```
