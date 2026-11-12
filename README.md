# JetRacer - Raspberry Pi 4

ROS 2 environment for the JetRacer autonomous vehicle, responsible for sensor integration, odometry, robot visualization, SLAM, path planning, and future autonomous navigation.

The system uses a Raspberry Pi 4 running ROS 2 Jazzy alongside an NVIDIA Jetson Nano responsible for vision processing and vehicle control.

## Hardware

- Raspberry Pi 4 (8 GB RAM)
- NVIDIA Jetson Nano (4 GB RAM)
- RPLIDAR A1M8-R5
- Intel RealSense T265
- Waveshare JetRacer platform

## Software

- Ubuntu 24.04 Server (ARM64) - Raspberry Pi 4
- ROS 2 Jazzy
- Slamtec sllidar_ros2
- Intel RealSense SDK (librealsense)
- realsense-ros (original direct-connection configuration)
- SLAM Toolbox
- Nav2 (Navigation2)
- RViz2 (Ubuntu laptop)
- Python 3
- UDP communication

## System Architecture

The Raspberry Pi 4 handles sensor integration, mapping, localization, path planning, and future navigation control.

The Jetson Nano handles vision processing, communication with the Intel RealSense T265, and vehicle actuation.

Current odometry communication:

```text
Intel RealSense T265
        |
        v
   NVIDIA Jetson Nano
        |
        | UDP
        v
    Raspberry Pi 4
        |
        v
   ROS 2 Odometry
        |
        v
   SLAM / Navigation
```

The target architecture also includes transferring obstacle information generated from stereo disparity to the Raspberry Pi.

## ROS 2 Workspaces

- `~/ros2_ws` - Main JetRacer ROS 2 workspace
- `~/t265_ws` - Intel RealSense T265 ROS 2 driver (original direct-connection configuration)

The main workspace contains the LiDAR driver, robot description, configuration files, maps, and experimental scripts.

## Project Structure

```text
ros2_ws/
├── src/
│   ├── sllidar_ros2/
│   │
│   └── jetracer_description/
│       ├── config/
│       │   ├── nav2_params.yaml
│       │   ├── slam_no_loop.yaml
│       │   └── ...
│       │
│       ├── launch/
│       │   ├── jetracer_bringup.launch.py
│       │   └── jetracer_model.launch.py
│       │
│       ├── maps/
│       │   └── map03_1.*
│       │
│       ├── scripts/
│       │   └── ...
│       │
│       ├── urdf/
│       │   └── jetracer.urdf.xacro
│       │
│       └── meshes/
│           └── *.stl
│
├── README.md
├── .gitignore
└── .gitmodules
```

ROS 2 bag recordings are stored separately in `slam_bags/` and are not intended to be tracked in Git.

## Running JetRacer

### Raspberry Pi 4 (SSH)

Connect from the laptop:

```bash
ssh rp4
```

Source ROS 2 environments:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
```

For the original directly connected T265 configuration, also source:

```bash
source ~/t265_ws/install/setup.bash
```

Verify Intel RealSense T265 detection when needed:

```bash
rs-enumerate-devices
lsusb
```

These commands should be executed on the computer physically connected to the T265. They are diagnostic checks and are not required for normal ROS 2 operation.

### Robot Bringup

The existing base launch command is:

```bash
ros2 launch jetracer_description jetracer_bringup.launch.py
```

The original bringup configuration includes:

- Intel RealSense T265 ROS 2 driver
- RPLIDAR A1 laser scans
- JetRacer URDF model
- Robot TF transformations

The newer architecture receives T265 odometry over UDP from the Jetson Nano. Integration of the UDP receiver into the unified bringup procedure is part of the ongoing work.

### Laptop (RViz2)

```bash
source /opt/ros/jazzy/setup.bash
rviz2
```

Load the saved RViz2 configuration.

Use the appropriate `Fixed Frame`:

- `odom_frame` - Original odometry visualization configuration
- `map` - SLAM and Nav2 map visualization

## ROS 2 Topics and TF

Main topics used in the project:

| Topic | Description |
|---|---|
| `/scan` | RPLIDAR laser scans |
| `/camera/pose/sample` | T265 odometry in the original direct-connection configuration |
| `/robot_description` | JetRacer URDF model |
| `/map` | Occupancy grid map |
| `/global_costmap/costmap` | Nav2 global costmap |
| `/global_costmap/published_footprint` | Robot footprint |
| `/plan` | Generated path |

The UDP odometry receiver provides the connection between Jetson Nano and ROS 2 on the Raspberry Pi.

### TF Configuration

The original odometry TF structure:

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

The navigation system requires a consistent localization transform chain between `map`, `odom`, and `base_link`.

During initial Nav2 planning tests, a temporary static `map → base_link` transform was used. This is a test configuration, not the final localization solution.

## SLAM Toolbox

SLAM Toolbox has been integrated and tested using RPLIDAR scans and T265 odometry.

### Configuration

An experimental configuration is stored in:

```text
src/jetracer_description/config/slam_no_loop.yaml
```

Loop closing was disabled:

```yaml
do_loop_closing: false
```

This resolved the observed sudden map displacement problem during testing.

### Generated Maps

The environment was mapped and saved for further navigation experiments.

Example:

```text
src/jetracer_description/maps/map03_1.yaml
```

Map resolution:

```text
0.03 m/cell
```

The saved map has been successfully loaded and visualized in RViz2.

ROS 2 bag recordings have also been used for testing and analysis of mapping data.

## Nav2 - Path Planning

Initial Nav2 configuration and path planning tests have been completed.

Configuration file:

```text
src/jetracer_description/config/nav2_params.yaml
```

### Planner Configuration

Selected global planner:

**Smac Hybrid-A\***

Motion model:

**Dubins**

Main parameters:

| Parameter | Value |
|---|---|
| Minimum turning radius | 0.45 m |
| Robot footprint | 0.275 × 0.20 m |
| Map resolution | 0.03 m/cell |

The planner accounts for vehicle dimensions, obstacles, and minimum turning radius.

### Completed Tests

- Loading a previously generated occupancy grid map
- Configuring and activating Nav2 Map Server
- Configuring the global costmap
- Configuring Nav2 Planner Server
- Generating paths with Smac Hybrid-A*
- Visualizing generated paths in RViz2
- Testing paths with different target points
- Experimenting with custom costmaps and track paths

The current system can generate collision-aware paths on a static map.

Some waypoint sequences still require improvements because of the vehicle's nonholonomic motion constraints.

**Important:** Successful path generation does not yet mean that the JetRacer can physically follow the generated path.

## Planned Navigation and Control

The next development stage focuses on integrating perception, path planning, and vehicle control.

### Obstacle Detection and Sensor Integration

- Generate disparity maps from T265 stereo images
- Convert disparity information into 3D point clouds
- Transfer point cloud data from Jetson Nano to Raspberry Pi 4
- Publish received data as ROS 2 `PointCloud2` messages
- Integrate LiDAR and stereo obstacle data into Nav2 costmaps

### Path Following

- Configure a Nav2 path-following controller
- Evaluate Regulated Pure Pursuit for trajectory tracking
- Implement a simplified Ackermann/bicycle kinematic model
- Convert velocity commands into `throttle` and `steering`
- Calibrate steering limits and vehicle velocity
- Send control commands from Raspberry Pi 4 to Jetson Nano

### System Integration

- Complete communication between Jetson Nano and Raspberry Pi 4
- Synchronize system clocks using chrony
- Integrate mapping, localization, planning, and control
- Implement communication watchdog and emergency stopping
- Test autonomous navigation and obstacle avoidance

## Diagnostics

List ROS 2 topics:

```bash
ros2 topic list
```

Check LiDAR scan frequency:

```bash
ros2 topic hz /scan
```

Check LiDAR data:

```bash
ros2 topic echo /scan --once
```

Check the published map:

```bash
ros2 topic echo /map --once
```

Verify odometry transformation in the original TF configuration:

```bash
ros2 run tf2_ros tf2_echo odom_frame base_link
```

## Current Status (12.11.2026)

### Completed

- [x] Raspberry Pi 4 Ubuntu Server configuration
- [x] ROS 2 Jazzy installation
- [x] RPLIDAR integration with ROS 2
- [x] Intel RealSense T265 odometry
- [x] UDP odometry transfer from Jetson Nano to Raspberry Pi 4
- [x] ROS 2 odometry receiver implementation
- [x] JetRacer URDF/Xacro model
- [x] Sensor TF configuration
- [x] RViz2 visualization
- [x] Initial unified bringup launch file
- [x] SLAM Toolbox integration and mapping tests
- [x] SLAM loop-closing issue resolved for experimental mapping
- [x] Saving and loading occupancy grid maps
- [x] Initial Nav2 configuration
- [x] Smac Hybrid-A* path planning
- [x] Robot footprint and turning radius configuration
- [x] Path visualization in RViz2

### In Progress / Planned

- [ ] Disparity-based obstacle detection
- [ ] Point cloud transmission over UDP
- [ ] Integration of LiDAR and stereo obstacle information
- [ ] Improved localization integration for Nav2
- [ ] Full Nav2 navigation and path following
- [ ] Ackermann steering control implementation
- [ ] Conversion from ROS 2 velocity commands to `throttle` and `steering`
- [ ] Raspberry Pi → Jetson control communication
- [ ] Clock synchronization between computers
- [ ] Autonomous obstacle avoidance
- [ ] End-to-end autonomous driving tests

## Shutdown

Stop ROS 2 processes using `Ctrl+C`.

Safely shut down Raspberry Pi:

```bash
sudo poweroff
```
