# 🤖 Autonomous Maze Solving Robot

A ROS 2 (Jazzy) project that simulates an autonomous differential-drive robot capable of navigating and solving mazes of varying difficulty using LiDAR-based wall-following, real-time SLAM mapping, and Gazebo simulation.

---

## 📋 Table of Contents

- [Overview](#overview)
- [System Architecture](#system-architecture)
- [Packages](#packages)
- [Robot Model](#robot-model)
- [Maze Worlds](#maze-worlds)
- [Prerequisites](#prerequisites)
- [Installation & Build](#installation--build)
- [Running the Simulation](#running-the-simulation)
- [ROS 2 Topics](#ros-2-topics)
- [Project Structure](#project-structure)
- [License](#license)

---

## Overview

This project implements a fully autonomous maze-solving robot in **ROS 2 Jazzy** with **Gazebo (Harmonic)**. The robot uses a 360° LiDAR sensor to perceive its environment, processes the scan data to detect obstacles in four cardinal directions, and applies a wall-following algorithm to navigate through maze corridors. Simultaneously, `slam_toolbox` builds a 2D occupancy map that is visualized live in **RViz2**.

Three difficulty levels are supported — **easy**, **medium**, and **hard** — each corresponding to a different maze world and robot spawn position.

---

## System Architecture

```
┌─────────────────────────────────────────────────────────┐
│                     Gazebo Simulation                   │
│  ┌──────────────┐   /scan    ┌──────────────────────┐   │
│  │  LiDAR Sensor│──────────▶│  perception_pkg      │   │
│  │  (GPU Ray)   │           │  lidar_processor_node│   │
│  └──────────────┘           └──────────┬───────────┘   │
│                                        │ /obstacle_data │
│  ┌──────────────┐   /cmd_vel           ▼                │
│  │  DiffDrive   │◀──────────┌──────────────────────┐   │
│  │  Plugin      │           │  control_pkg          │   │
│  └──────────────┘           │  wall_follower_node   │   │
│         │                   └──────────────────────┘   │
│         │ /odom                        ▲                │
│         └──────────────────────────────┘                │
│                                                         │
│  ┌──────────────┐   /scan + /odom                       │
│  │  slam_toolbox│◀──────────────────────────────────── │
│  │  (SLAM)      │──▶ /map ──▶ RViz2                    │
│  └──────────────┘                                       │
└─────────────────────────────────────────────────────────┘
```

---

## Packages

This workspace contains **four ROS 2 packages**:

### 1. `robot_simulation`
The core simulation package. Hosts the robot URDF/Xacro model, Gazebo world files, and launch infrastructure.

- **URDF model**: TurtleBot3 Waffle Pi-based differential drive robot with LiDAR and RGB camera
- **Gazebo plugins**: Differential drive, GPU LiDAR, camera sensor
- **Launch files**: `bring_up.launch.py`, `robot_state_publisher.launch.py`, `spawn_entity.launch.py`

### 2. `perception_pkg`
Processes raw LiDAR data from Gazebo into directional obstacle distances.

- **Node**: `lidar_processor_node`
- Subscribes to `/scan` (`sensor_msgs/LaserScan`)
- Computes minimum distances in **front, right, left, back** sectors (20° FOV each)
- Publishes to `/obstacle_data` (`std_msgs/Float32MultiArray`)

### 3. `control_pkg`
Implements the wall-following maze navigation algorithm.

- **Node**: `wall_follower_node`
- Subscribes to `/obstacle_data` and `/odom`
- Implements a **state machine** with states: `GO`, `TURN_LEFT`, `TURN_RIGHT`, `STOP`
- Publishes velocity commands to `/cmd_vel` (`geometry_msgs/Twist`)
- Includes heading correction to keep the robot aligned to 90° grid angles

### 4. `mapping_pkg`
Handles SLAM-based map building and RViz2 visualization.

- Uses **slam_toolbox** in `online_async` mode
- Launches RViz2 with a preconfigured layout showing the live occupancy map
- Config files: `slam.yaml`, `rviz_config.rviz`

---

## Robot Model

The robot is based on the **TurtleBot3 Waffle Pi** platform:

| Component | Specification |
|-----------|--------------|
| Drive | Differential drive (2 active wheels + 2 casters) |
| Wheel separation | 0.288 m |
| Wheel radius | 0.033 m |
| LiDAR | 360° GPU ray, 0.12–10 m range, 10 Hz, Gaussian noise |
| Camera | RGB 640×480, 30 Hz, 80° FOV, Gaussian noise |
| IMU | Fixed link at base |
| Odometry publish rate | 50 Hz |

---

## Maze Worlds

Three SDF world files with increasing complexity:

| Difficulty | Spawn Position (x, y) | Yaw |
|------------|----------------------|-----|
| `easy`     | (0.0, -3.5)          | 90° |
| `medium`   | (0.28, -7.8)         | 90° |
| `hard`     | (0.0, -8.0)          | 90° |

---

## Prerequisites

- **OS**: Ubuntu 24.04
- **ROS 2**: Jazzy Jalisco
- **Gazebo**: Harmonic
- **Required ROS packages**:

```bash
sudo apt install \
  ros-jazzy-ros-gz \
  ros-jazzy-slam-toolbox \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-tf2-ros \
  ros-jazzy-tf2-geometry-msgs \
  ros-jazzy-rviz2
```

---

## Installation & Build

```bash
# Clone into your ROS 2 workspace
cd ~/ros2_ws/src
git clone <repository-url> maze_solving_robot

# Install dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# Build all packages
colcon build --packages-select robot_simulation perception_pkg control_pkg mapping_pkg

# Source the workspace
source install/setup.bash
```

---

## Running the Simulation

### Easy difficulty (default)
```bash
ros2 launch robot_simulation bring_up.launch.py
```

### Medium difficulty
```bash
ros2 launch robot_simulation bring_up.launch.py difficulty:=medium
```

### Hard difficulty
```bash
ros2 launch robot_simulation bring_up.launch.py difficulty:=hard
```

The launch file will start:
1. **Gazebo server** — physics simulation
2. **Gazebo client** — GUI visualizer
3. **Robot State Publisher** — TF tree from URDF
4. **Spawn Entity** — places the robot at the correct maze start
5. **LiDAR Processor** — perception pipeline
6. **SLAM Toolbox + RViz2** — live map building and visualization
7. **Wall Follower** — autonomous navigation controller

---

## ROS 2 Topics

| Topic | Type | Publisher | Subscriber |
|-------|------|-----------|------------|
| `/scan` | `sensor_msgs/LaserScan` | Gazebo LiDAR plugin | `lidar_processor_node` |
| `/obstacle_data` | `std_msgs/Float32MultiArray` | `lidar_processor_node` | `wall_follower_node` |
| `/cmd_vel` | `geometry_msgs/Twist` | `wall_follower_node` | Gazebo DiffDrive plugin |
| `/odom` | `nav_msgs/Odometry` | Gazebo DiffDrive plugin | `wall_follower_node` |
| `/map` | `nav_msgs/OccupancyGrid` | `slam_toolbox` | RViz2 |
| `/camera/image_raw` | `sensor_msgs/Image` | Gazebo camera plugin | — |
| `/robot_description` | `std_msgs/String` | `robot_state_publisher` | — |

---

## Project Structure

```
maze_solving_robot/
├── robot_simulation/           # Simulation & robot model package
│   ├── launch/
│   │   ├── bring_up.launch.py          # Main entry point
│   │   ├── robot_state_publisher.launch.py
│   │   └── spawn_entity.launch.py
│   ├── urdf/
│   │   ├── robot.urdf.xacro            # Robot kinematic model
│   │   └── robot.gazebo.xacro          # Gazebo plugins & sensors
│   ├── worlds/
│   │   ├── easy.sdf
│   │   ├── medium.sdf
│   │   └── hard.sdf
│   ├── models/robot/meshes/            # STL mesh files
│   ├── CMakeLists.txt
│   └── package.xml
│
├── perception_pkg/             # LiDAR processing package
│   ├── src/
│   │   └── lidar_processor_node.cpp
│   ├── CMakeLists.txt
│   └── package.xml
│
├── control_pkg/                # Navigation & control package
│   ├── src/
│   │   └── wall_follower_node.cpp
│   ├── CMakeLists.txt
│   └── package.xml
│
├── mapping_pkg/                # SLAM & visualization package
│   ├── launch/
│   │   └── map_visulator.launch.py
│   ├── config/
│   │   ├── slam.yaml
│   │   └── rviz_config.rviz
│   ├── CMakeLists.txt
│   └── package.xml
│
└── README.md
```

---
