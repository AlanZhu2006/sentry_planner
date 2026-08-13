# RoboMaster Sentry Autonomous Navigation System

<div align="center">

[![简体中文](https://img.shields.io/badge/%E7%AE%80%E4%BD%93%E4%B8%AD%E6%96%87-Switch-2ea44f?style=for-the-badge)](README_LIDAR.md)
[![English](https://img.shields.io/badge/English-Current-0969da?style=for-the-badge)](README_LIDAR_EN.md)

[![ROS 2 Humble](https://img.shields.io/badge/ROS%202-Humble-22314E?logo=ros&logoColor=white)](https://docs.ros.org/en/humble/)
[![Ubuntu 22.04](https://img.shields.io/badge/Ubuntu-22.04-E95420?logo=ubuntu&logoColor=white)](https://ubuntu.com/)
[![License: MIT](https://img.shields.io/badge/License-MIT-2ea44f.svg)](LICENSE)

**LiDAR SLAM and Nav2 autonomous navigation for a RoboMaster sentry robot**

</div>

Last updated: 2026-04-11

## 0. Documentation Scope

| Document | Responsibility |
|---|---|
| [README_EN.md](README_EN.md) | Central index: overview, architecture, build order, and shortest startup path |
| [README_COMMUNICATION_EN.md](README_COMMUNICATION_EN.md) | `sentry_bridge`, PTYs, `serial_sender`, packet formats, and ROS 2 communication |
| [README_BEHAVIOR_TREE_FLOW_EN.md](README_BEHAVIOR_TREE_FLOW_EN.md) | Behavior-tree XML, `SendGoal`, `RobotControl`, Groot2, and debugging |
| **This document** | Mid-360, FAST-LIO, Nav2, mapping, Gazebo Sim2Real, performance, and localization troubleshooting |
| [README_COMMANDS_EN.md](README_COMMANDS_EN.md) | Commands, environment variables, map files, point selection, and staged integration |

Recommended reading:

- Gazebo, RMUL2026, `bringup_sim`, and Sim2Real: §6.4 plus [mid360 command.txt](mid360%20command.txt).
- Point clouds, FAST-LIO, Nav2 parameters, costmaps, and TF: the remaining sections here.
- `MAP_FILE`, PCD/PGM/YAML, and terminal order: [README_COMMANDS_EN.md](README_COMMANDS_EN.md).
- Serial, bridge, and PTYs: [README_COMMUNICATION_EN.md](README_COMMUNICATION_EN.md).

### Recommended Sim2Real Path

First stabilize costmaps, velocity, `SendGoal`, and the `center_attack_simple` approach/hold/return branches in Gazebo RMUL2026 + Nav2. Move to the Mid-360 + FAST-LIO robot only after that loop is repeatable.

| Stage | Entry Point |
|---|---|
| Gazebo startup and referee fixtures | [mid360 command.txt](mid360%20command.txt) |
| Simulation parameters and real-robot differences | §6.4 |
| Behavior tree and Groot2 | [README_BEHAVIOR_TREE_FLOW_EN.md](README_BEHAVIOR_TREE_FLOW_EN.md) |
| Bridge, PTYs, and real-robot terminals | [README_COMMUNICATION_EN.md](README_COMMUNICATION_EN.md), [README_COMMANDS_EN.md](README_COMMANDS_EN.md) |

## Contents

1. [Overview](#1-overview)
2. [Architecture](#2-architecture)
3. [Hardware](#3-hardware)
4. [Software Stack](#4-software-stack)
5. [Installation](#5-installation)
6. [Quick Start](#6-quick-start)
7. [Configuration](#7-configuration)
8. [Performance](#8-performance)
9. [Troubleshooting](#9-troubleshooting)
10. [Development Notes](#10-development-notes)
11. [Roadmap](#11-roadmap)
12. [References](#12-references)

## 1. Overview

### 1.1 Introduction

This system combines 3D LiDAR odometry with Nav2 to provide real-time localization, mapping, path planning, dynamic obstacle avoidance, and chassis velocity output for a RoboMaster sentry robot. Livox Mid-360 is the current production sensor; Unitree L2 remains an experimental option.

### 1.2 Capabilities

| Area | Capability |
|---|---|
| SLAM | Real-time 3D LiDAR odometry with FAST-LIO or Point-LIO |
| Mapping | PCD attitude correction, 2D projection, and PGM map saving |
| Navigation | Nav2 global/local planning and DWB control |
| Obstacle avoidance | Static and dynamic obstacles through `/scan` and costmaps |
| Decision integration | Behavior-tree `SendGoal`, `navigate_to_pose`, and waypoint management |
| Chassis integration | `/cmd_vel` → frame transform → BT merge → sender/bridge → MCU |

### 1.3 Recorded Indoor Baseline

These 5×5 m indoor results are regression references, not competition-field guarantees.

| Metric | Result |
|---|---|
| Cruise speed | 0.20–0.26 m/s |
| Obstacle clearance | ≥ 0.3 m |
| Final position error | ≤ 0.2 m |
| Success rate over 20 trials | ≥ 90% |
| Consecutive collision-free runs | ≥ 5 |

## 2. Architecture

### 2.1 System Diagram

```text
Mid-360 / Unitree L2
          |
          v
   LiDAR driver + IMU
          |
          v
 FAST-LIO / Point-LIO ---> odometry + TF + /cloud_registered
          |
          v
 pointcloud_to_laserscan ---> /scan
          |
          v
 Nav2: localization + planner + controller + costmaps
          |
          v
 /cmd_vel ---> fake_vel_transform ---> /cmd_vel_chassis
          |
          v
 bt_comm_adapter ---> /cmd_vel_chassis_bt
          |
          v
 serial_sender ---> Radar PTY ---> sentry_bridge ---> MCU ---> chassis
```

### 2.2 TF Tree

```text
map
 `- odom
     `- camera_init
         `- body / aft_mapped
             `- base_link
                 `- base_footprint
```

The runtime tree must provide a continuous, timely `map -> odom -> base_link` path. Point clouds and `/scan` must use the intended target frame. For the current tilted mount, a static `body -> base_link` pitch rotation performs the frame correction.

### 2.3 Data Flow

```text
LiDAR 10 Hz -> SLAM -> /cloud_registered -> /scan -> Nav2 -> /cmd_vel
                   `-> /Odometry -> TF: odom -> base_link
```

## 3. Hardware

### 3.1 Compute Platforms

| Component | NUC 12 Pro, primary | Jetson Orin Nano, backup |
|---|---|---|
| CPU | Intel i7-1260P | ARM Cortex-A78AE |
| Memory | 16 GB DDR4 | 8 GB LPDDR5 |
| Storage | 512 GB NVMe SSD | 128 GB eMMC |
| OS | Ubuntu 22.04 LTS | Ubuntu 22.04 / JetPack 6 |
| Suggested use | Full navigation and decision stack | Edge computing or mapping experiments |

### 3.2 LiDAR Sensors

| Specification | Livox Mid-360 | Unitree L2 |
|---|---|---|
| Status | Primary / real robot | Experimental |
| Field of view | 360° × 59° | 360° × 90° |
| Range | 40 m | 30 m |
| Point rate | 200,000 points/s | 43,200 points/s |
| IMU | Stable in the current setup | Noise and drift on the gimbal mount |
| Interface | Ethernet / UDP | USB serial or Ethernet |
| Data | Livox CustomMsg | PointCloud2 |
| Time synchronization | Strong | Requires further tuning |
| Tilted mount | Supported by current solution | Remaining IMU-drift risk |

Default Mid-360 network:

- LiDAR IP: `192.168.1.182`
- Host IP: `192.168.1.2`
- UDP ports: 56101–56501

### 3.3 Chassis Controller

The RoboMaster STM32 C board controls the chassis. The current path does not let navigation open `/dev/ttyACM0` directly:

```text
/cmd_vel_chassis_bt -> serial_sender -> Radar PTY
                    -> sentry_bridge -> MCU -> chassis
```

Only `sentry_bridge` owns the physical USB CDC device. See [README_COMMUNICATION_EN.md](README_COMMUNICATION_EN.md) for velocity, A3, SX/ST, and CRC details.

### 3.4 Remote Access

The robot is commonly maintained through SSH and VNC. Addresses and ports are deployment configuration and should not be hard-coded into general-purpose launch scripts:

```bash
ssh <user>@<host> -p <ssh-port>
```

Use a local display, X11 forwarding, or VNC for RViz, Groot2, and `map_point_picker.py`.

## 4. Software Stack

### 4.1 Core Dependencies

| Package | Version / Branch | Purpose |
|---|---|---|
| ROS 2 | Humble | Middleware |
| Nav2 | Humble | Navigation framework |
| FAST-LIO | Current ROS 2 working revision | Mid-360 SLAM |
| Point-LIO | ROS 2 fork | Experimental Unitree L2 SLAM |
| `livox_ros_driver2` | Current working revision | Mid-360 driver |
| `unitree_lidar_ros2` | Current working revision | Unitree L2 driver |
| `pcd2pgm` | Current working revision | PCD to 2D occupancy grid |
| `pointcloud_to_laserscan` | ROS 2 | 3D cloud to `/scan` |

### 4.2 Workspace Layout

```text
~/nav_ws/
|-- src/
|   |-- FAST_LIO/
|   |   |-- config/mid360.yaml
|   |   `-- PCD/scans.pcd
|   |-- point_lio_ros2/
|   |   `-- config/unilidar_l2.yaml
|   |-- livox_ros_driver2/
|   |   `-- config/MID360_config.json
|   |-- unitree_lidar_ros2/
|   |-- pcd2pgm/
|   `-- pointcloud_to_laserscan/
|-- install/
|-- my_nav2_params.yaml
|-- my_nav2_params_test.yaml
`-- start_robot.sh
```

This repository's `rm_navigation_ws` additionally contains `rm_nav_bringup`, simulation worlds, navigation packages, and perception adapters.

### 4.3 Key ROS 2 Topics

| Topic | Type | Typical rate | Purpose |
|---|---|---|---|
| `/livox/lidar` | CustomMsg | 10 Hz | Raw cloud |
| `/livox/imu` | Imu | 200 Hz | IMU |
| `/Odometry` | Odometry | 10 Hz | SLAM pose |
| `/cloud_registered` | PointCloud2 | 10 Hz | Registered cloud |
| `/scan` | LaserScan | About 10 Hz | Nav2 2D scan |
| `/cmd_vel` | Twist | 20 Hz | Nav2 velocity |
| `/map` | OccupancyGrid | Static | 2D navigation map |

## 5. Installation

### 5.1 Prerequisites

```bash
sudo apt update
sudo apt install -y ros-humble-desktop
sudo apt install -y ros-humble-navigation2 ros-humble-nav2-bringup
sudo apt install -y \
  ros-humble-pcl-ros \
  ros-humble-tf2-tools \
  ros-humble-pointcloud-to-laserscan \
  python3-serial \
  libpcl-dev
```

### 5.2 Clone and Build

```bash
mkdir -p ~/nav_ws/src
cd ~/nav_ws/src
git clone https://github.com/Livox-SDK/livox_ros_driver2.git
git clone https://github.com/hku-mars/FAST_LIO.git
git clone https://github.com/LihanChen2004/pcd2pgm.git

cd ~/nav_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

Treat these commands as dependency provenance. Use the revisions, forks, and patches pinned in the team's working `nav_ws` for a reproducible robot build.

### 5.3 Mid-360 Network

```bash
sudo nmcli con mod "Wired connection 1" ipv4.addresses 192.168.1.2/24
sudo nmcli con mod "Wired connection 1" ipv4.method manual
sudo nmcli con up "Wired connection 1"

sudo ufw disable
sudo sysctl -w net.core.rmem_max=26214400
sudo sysctl -w net.core.rmem_default=26214400
```

Before disabling a firewall on a managed or competition network, follow team network policy. At minimum, allow the required Livox UDP ports.

## 6. Quick Start

### 6.1 One-Command Launch

For the integrated real-robot path, start from the repository root:

```bash
cd /path/to/nyush_rm_sentry
./start_robot.sh
```

It can launch the Mid-360 driver, static TF, FAST-LIO, point-cloud-to-scan conversion, Nav2, `bt_comm_adapter`, the behavior tree, and an optional `serial_sender`. Start the bridge separately first.

For navigation-only development, the external workspace also provides:

```bash
cd ~/nav_ws
./start_robot.sh
```

The two launchers do not share every default. Read their headers and [README_COMMANDS_EN.md](README_COMMANDS_EN.md) before combining examples.

### 6.2 Manual Startup

Step 1 — Mid-360 driver:

```bash
cd ~/nav_ws
source install/setup.bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

Step 2 — FAST-LIO:

```bash
export LD_PRELOAD=/lib/x86_64-linux-gnu/libusb-1.0.so.0
ros2 launch fast_lio mapping.launch.py config_file:=mid360.yaml
```

Step 3 — static TF:

```bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom camera_init
ros2 run tf2_ros static_transform_publisher 0 0 0 0 -0.873 0 body base_link
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link base_footprint
```

Step 4 — PointCloud to LaserScan:

```bash
ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node --ros-args \
  -p target_frame:=base_link \
  -p min_height:=-0.4 -p max_height:=1.0 \
  -p range_min:=0.1 -p range_max:=20.0 \
  -r cloud_in:=/cloud_registered -r scan:=/scan
```

Step 5 — Nav2:

```bash
ros2 launch nav2_bringup bringup_launch.py \
  use_sim_time:=False \
  map:=/home/nyu/Desktop/map/my_map.yaml \
  params_file:=/home/nyu/nav_ws/my_nav2_params.yaml
```

Step 6 — chassis communication: do not let the sender compete for the real port. Start the bridge, then point the sender to the Radar PTY. See [README_COMMUNICATION_EN.md](README_COMMUNICATION_EN.md).

### 6.3 Mapping

```bash
# 1. Run SLAM and collect the cloud
ros2 launch fast_lio mapping.launch.py config_file:=mid360.yaml

# 2. Stopping saves FAST_LIO/PCD/scans.pcd

# 3. Rotate the PCD for a tilted mount
python3 rotate_pcd.py

# 4. Project 3D PCD to a 2D grid
ros2 launch pcd2pgm pcd2pgm_launch.py

# 5. Save the map
ros2 run nav2_map_server map_saver_cli \
  -f /home/nyu/Desktop/map/my_map
```

See [README_COMMANDS_EN.md](README_COMMANDS_EN.md) for PCD/PGM/YAML ownership and waypoint recalibration.

<a id="nyush-gazebo-sim2real"></a>

### 6.4 Gazebo RMUL2026 + Nav2: First Sim2Real Step

Use ROS 2 Humble with Gazebo Classic 11. Simulation uses Gazebo odometry and AMCL and does not require a physical Mid-360. It is the preferred place to stabilize Nav2 costmaps, velocity, `SendGoal`, and BT branches.

Terminal 1 — world, Nav2, and RViz:

```bash
cd /path/to/nyush_rm_sentry/rm_navigation_ws
source ~/nav_ws/install/setup.bash
source install/setup.bash
ros2 launch rm_nav_bringup bringup_sim.launch.py \
  world:=RMUL2026 \
  mode:=nav \
  localization:=amcl \
  use_gazebo_odom:=true \
  nav_rviz:=True
```

Terminal 2 — persistent BT debugging session:

```bash
bash /path/to/nyush_rm_sentry/scripts/run_center_attack_debug_session.sh
```

The script defaults to `USE_SIM_TIME=True` and keeps `bt_comm_adapter`, `rm_behavior_tree`, and `watch_center_attack_state.py` alive. Enable `enable_groot:=true` for Groot2 monitoring on the default port 1667.

Terminal 3 — copy the §4–§8 `ros2 topic pub` examples from [mid360 command.txt](mid360%20command.txt) to test `APPROACH_CENTER`, `CENTER_HOLD_ATTACK`, and `HOME_RECOVER`.

| Item | Gazebo | Real robot |
|---|---|---|
| Localization | `use_gazebo_odom:=true` + AMCL | FAST-LIO / `bringup_real`, etc. |
| Time | `use_sim_time:=true` | Usually `false` |
| MCU output | Usually ROS-only; Radar PTY + sender is optional | Bridge + `serial_sender` |

Optional Groot2:

```bash
cd ~/Desktop
./Groot2-v1.9.0-x86_64.AppImage
```

Open `rm_decision_ws/rm_behavior_tree/config/Project.btproj` and connect Monitor to `127.0.0.1:1667`. Match the command to the installed AppImage version.

## 7. Configuration

### 7.1 Nav2 Parameters

File: `~/nav_ws/my_nav2_params.yaml`

| Parameter | Current value | Meaning |
|---|---|---|
| `max_vel_x` | 0.26 m/s | Maximum forward velocity |
| `max_vel_y` | 0.26 m/s | Maximum lateral velocity |
| `max_vel_theta` | 0.0 rad/s | Direct Nav2 rotation disabled |
| `acc_lim_x/y` | 2.5 m/s² | Acceleration limit |
| `min_speed_xy` | 0.05 m/s | Translation deadband |
| `controller_frequency` | 10.0 Hz | Control-loop frequency |

`max_vel_theta=0` matches the current policy in which the behavior-tree `chassis_spin_vel` controls tactical spin separately.

### 7.2 FAST-LIO Parameters

File: `~/nav_ws/src/FAST_LIO/config/mid360.yaml`

```yaml
lidar_type: 2
scan_line: 4
scan_rate: 10
point_filter_num: 3
```

### 7.3 Mid-360 Network Configuration

File: `~/nav_ws/src/livox_ros_driver2/config/MID360_config.json`

```json
{
  "host_net_info": {
    "cmd_data_ip": "192.168.1.2",
    "point_data_ip": "192.168.1.2",
    "imu_data_ip": "192.168.1.2"
  },
  "lidar_configs": [{
    "ip": "192.168.1.182"
  }]
}
```

## 8. Performance

### 8.1 Resource Baseline

Platform: NUC 12 Pro, i7-1260P, 16 GB RAM.

| Metric | Recorded value |
|---|---|
| CPU peak | About 40% |
| CPU average | About 36% |
| Memory | About 6.3 GB |
| Network receive | About 3.15 MB/s |

### 8.2 Topic Rates

| Topic | Measured | Expected | Status |
|---|---|---|---|
| `/livox/lidar` | 10 Hz | 10 Hz | Normal |
| `/livox/imu` | 200 Hz | 200 Hz | Normal |
| `/Odometry` | 10 Hz | 10–100 Hz | Evaluate against configuration |
| `/scan` | 7–9 Hz | 10 Hz | Optimization candidate |
| `/cmd_vel` | 20 Hz | 20 Hz | Normal |

### 8.3 Latency

| Path | Recorded | Target |
|---|---|---|
| `/cmd_vel` → sender | 0.34 ms median | < 1 ms |
| Point cloud → LaserScan | About 15 ms/frame | < 5 ms ideal, < 20 ms acceptable |
| End-to-end control | About 20 ms | < 50 ms |

### 8.4 Diagnostic Commands

```bash
ros2 topic hz /scan /Odometry /cmd_vel
python3 measure_pointcloud_latency.py \
  --cloud /cloud_registered --scan /scan --duration 30
./monitor_resources.sh --interval 1 --duration 60
ros2 run tf2_tools view_frames
```

## 9. Troubleshooting

### 9.1 No Mid-360 Data

```bash
ping 192.168.1.182
ros2 topic list | grep livox
ip address
```

Check the host static IP, LiDAR JSON, interface name, UDP buffers, and firewall rules.

### 9.2 TF Failure

Typical message: `Transform from map to base_link failed`.

```bash
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo map base_link
```

Check for duplicate publishers, consistent time sources, and a continuous `map -> odom -> base_link` path.

### 9.3 AMCL Message-Filter Drops

- Increase AMCL `queue_size`.
- Increase `transform_tolerance`.
- Check `/scan` rate and frame ID.
- Check clock synchronization and `use_sim_time`.

### 9.4 Serial or Chassis Does Not Respond

```bash
lsof /dev/ttyACM0
ls -l /tmp/nyush-rm-sentry-*
ros2 topic echo /cmd_vel_chassis_bt --once
ros2 topic echo /robot_control --once
```

Only `sentry_bridge.py` should own the physical port. Do not run a legacy direct-serial test alongside the bridge. See [README_COMMUNICATION_EN.md](README_COMMUNICATION_EN.md).

### 9.5 Localization Fails During Gimbal Rotation

Possible causes include IMU saturation, incorrect extrinsics, clock drift, and mechanical vibration.

- Reduce gimbal rotation speed.
- Warm up the IMU and inspect bias.
- Recheck extrinsics and static TF.
- Verify cloud and IMU timestamps.
- Use a more stable or higher-range IMU when required.

## 10. Development Notes

### 10.1 LiDAR Selection

| Aspect | Mid-360 | Unitree L2 |
|---|---|---|
| Time sync | CustomMsg `offset_time`, stable in current tests | PointCloud2 path still needs tuning |
| IMU | Stable in the current setup | Noisy on a gimbal mount |
| Tilted mount | Static TF + offline PCD rotation works | Gravity alignment and drift issues |
| Current status | Primary sensor | Experimental |

### 10.2 Unitree L2 Known Issues

Connection configuration is in `unilidar_sdk2/unitree_lidar_ros2/launch/launch.py`:

| Connection | Configuration |
|---|---|
| USB serial | `serial_port: '/dev/ttyACM0'`, `initialize_type: 2` |
| Ethernet UDP | `lidar_ip: '10.10.10.10'`, `initialize_type: 1` |

The L2 and MCU CDC may both appear as `/dev/ttyACM*`. Identify devices by USB attributes or udev rules, not only by their assigned number.

Observed issues:

1. The cloud may rotate or diverge during initialization; increasing `cloud_scan_num` to 72 has limited effect.
2. Normal `acc_norm` is about 10.2 m/s²; jumps toward 14 m/s² can destabilize the cloud.
3. Gimbal or tilted installation amplifies balance and IMU gravity-alignment problems.

Official serial example:

```bash
cd ~/nav_ws/src/unilidar_sdk2/unitree_lidar_sdk/build
sudo chmod 777 /dev/ttyACM0
../bin/example_lidar_serial
```

Verify that the selected ACM device is the L2, not the MCU, before running it.

### 10.3 Point-LIO Configuration

Adapted repository: [dfloreaa/point_lio_ros2](https://github.com/dfloreaa/point_lio_ros2). File: `~/nav_ws/src/point_lio_ros2/config/unilidar.yaml`.

| Parameter | Current value | Purpose |
|---|---|---|
| `start_in_aggressive_motion` | `true` | Use preset gravity and reduce initialization divergence |
| `gravity_init` | `[0.0, 0.0, -9.810]` | Preset gravity |
| `extrinsic_est_en` | `false` | Disable online extrinsic estimation in aggressive-motion mode |
| `acc_norm` | `10.2` | Expected acceleration norm |
| `b_acc_cov` / `b_gyr_cov` | `0.0001` | Bias covariance |
| `imu_meas_acc_cov` | `0.1` | Accelerometer covariance |
| `imu_meas_omg_cov` | `0.1` | Gyroscope covariance |

Do not rely only on Point-LIO `extrinsic_R` to correct a large mechanical tilt. It can rotate the cloud but destabilize IMU/gravity alignment during motion.

Suggested `mapping_unilidar_l2.launch.py` baselines:

| Parameter | NUC | Jetson |
|---|---|---|
| `point_filter_num` | 3 | 1 |
| `filter_size_surf` | 0.5 | 0.3 |
| `filter_size_map` | 0.5 | 0.3 |

Tune to available CPU capacity to avoid timestamp backlog and `the queue is full`.

### 10.4 Mid-360 Details

Full `MID360_config.json` example:

```json
{
  "host_net_info": {
    "cmd_data_ip": "192.168.1.2",
    "cmd_data_port": 56101,
    "push_msg_ip": "192.168.1.2",
    "push_msg_port": 56201,
    "point_data_ip": "192.168.1.2",
    "point_data_port": 56301,
    "imu_data_ip": "192.168.1.2",
    "imu_data_port": 56401,
    "log_data_ip": "",
    "log_data_port": 56501
  },
  "lidar_configs": [{
    "ip": "192.168.1.182",
    "pcl_data_type": 0,
    "pattern_mode": 0,
    "extrinsic_parameter": {
      "roll": 0.0,
      "pitch": 0.0,
      "yaw": 0.0,
      "x": 0,
      "y": 0,
      "z": 0
    }
  }]
}
```

Recommended in `msg_MID360_launch.py`:

```python
xfer_format = 1  # Livox CustomMsg
```

CustomMsg exposes timing fields such as `offset_time`. PointCloud2 mode may run FAST-LIO but can emit warnings about missing Livox-specific timing information.

### 10.5 Tilted-Mount Solution

The current mechanical design tilts the LiDAR for greater coverage. The recommended approach is:

1. Publish a static `body -> base_link` pitch rotation.
2. Rotate the saved PCD with `rotate_pcd.py` before 2D projection.
3. Do not replace both steps with a large Point-LIO `extrinsic_R`.

```bash
ros2 run tf2_ros static_transform_publisher \
  0 0 0 0 -0.873 0 body base_link

python3 rotate_pcd.py
```

`-0.873 rad` is approximately `-50°`. Measure again whenever the physical mount changes.

### 10.6 Mapping Workflow

After enabling PCD saving in FAST-LIO or Point-LIO:

```bash
ros2 launch fast_lio mapping.launch.py config_file:=mid360.yaml

cd ~/nav_ws/src/FAST_LIO/PCD
pcl_viewer scans.pcd
python3 rotate_pcd.py

ros2 launch pcd2pgm pcd2pgm_launch.py

cd ~/Desktop/map
ros2 run nav2_map_server map_saver_cli -f my_map
```

### 10.7 Current Navigation Results

- Point-to-point navigation in a 5×5 m indoor area.
- Fixed-obstacle bypass and real-time dynamic avoidance.
- Recorded cruise speed up to 0.26 m/s.
- Final position error ≤ 0.2 m.
- At least 90% success over 20 trials and five consecutive collision-free runs.

These are stage results. Revalidate localization, costmaps, and tuning on the competition field.

### 10.8 NYUSH Real-Robot Navigation Checklist

Working communication does not imply a closed navigation loop. The physical environment must match the selected 2D map, such as competition `RMUL2026` or local `11_map`.

This path must remain stable:

```text
map -> odom -> base_link
```

Minimum checks:

- `ros2 action list | grep navigate_to_pose` returns the action; otherwise BT `SendGoal` cannot close the loop.
- The robot pose approximately matches the map in RViz.
- The local costmap does not remain abnormally inflated.
- Home and center goals lie in free occupancy-grid cells.
- Map origin and orientation match the physical field.

Map mismatch often looks like “a path exists but the robot moves in the wrong direction” or “the planner rejects the goal.” Do not immediately blame Nav2.

Validate localization and short motion on a bench or small area before testing approach-center, center-hold, and return-home on the full field. Without a matching environment, communication and BT can be validated, but real-robot navigation cannot be declared complete.

## 11. Roadmap

- [ ] Unitree L2 automatic initialization and stability evaluation
- [ ] IMU calibration procedure
- [ ] Improved dynamic obstacle avoidance
- [ ] ROS battery monitoring
- [ ] Automatic recovery behaviors
- [ ] New performance baseline from real competition-field data
- [ ] Continued evaluation of sentry and radar-station sensors

## 12. References

### Official Documentation

- [Livox Mid-360](https://www.livoxtech.com/mid-360/downloads)
- [FAST-LIO](https://github.com/hku-mars/FAST_LIO)
- [Nav2](https://docs.nav2.org/)
- [ROS 2 Humble](https://docs.ros.org/en/humble/)

### Related Projects

- [Livox ROS Driver 2](https://github.com/Livox-SDK/livox_ros_driver2)
- [Point-LIO](https://github.com/hku-mars/Point-LIO)
- [Point-LIO ROS 2, Unitree-adapted](https://github.com/dfloreaa/point_lio_ros2)
- [pcd2pgm](https://github.com/LihanChen2004/pcd2pgm)

### Useful Tools

- `pcl_viewer`: point-cloud inspection.
- Foxglove: ROS 2 visualization.
- PlotJuggler: real-time plots.
- RViz2: maps, TF, LaserScan, and Nav2 debugging.

## Contributing and License

Use GitHub issues for reproducible bug reports and pull requests for code or documentation improvements.

The repository root uses the [MIT License](LICENSE). FAST-LIO, Nav2, Livox SDK, Unitree SDK, and other third-party components retain their respective licenses; comply with each when redistributing or deploying the complete stack.
