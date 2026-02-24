# RoboMaster Sentry Autonomous Navigation System
# RoboMaster 哨兵自主导航系统

[![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![Ubuntu 22.04](https://img.shields.io/badge/Ubuntu-22.04-orange)](https://ubuntu.com/)
[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)

> **Autonomous navigation solution for RoboMaster Sentry robot using LiDAR SLAM and Nav2**  
> **基于激光雷达SLAM与Nav2的RoboMaster哨兵机器人自主导航解决方案**

---

## Table of Contents / 目录

| Section | 章节 |
|---------|------|
| [1. Project Overview](#1-project-overview--项目概述) | 项目概述 |
| [2. System Architecture](#2-system-architecture--系统架构) | 系统架构 |
| [3. Hardware Configuration](#3-hardware-configuration--硬件配置) | 硬件配置 |
| [4. Software Stack](#4-software-stack--软件栈) | 软件栈 |
| [5. Installation Guide](#5-installation-guide--安装指南) | 安装指南 |
| [6. Quick Start](#6-quick-start--快速启动) | 快速启动 |
| [7. Configuration](#7-configuration--配置说明) | 配置说明 |
| [8. Performance Metrics](#8-performance-metrics--性能指标) | 性能指标 |
| [9. Troubleshooting](#9-troubleshooting--故障排除) | 故障排除 |
| [10. Development Notes](#10-development-notes--开发笔记) | 开发笔记 |
| [11. Future Plans](#11-future-plans--未来计划) | 未来计划 |
| [12. References](#12-references--参考资料) | 参考资料 |

---

## 1. Project Overview / 项目概述

### 1.1 Introduction / 简介

This project implements a complete autonomous navigation system for the RoboMaster Sentry robot. The system integrates LiDAR-based SLAM for real-time localization and mapping, with Nav2 for path planning and obstacle avoidance.

本项目为RoboMaster哨兵机器人实现完整的自主导航系统。系统集成基于激光雷达的SLAM实时定位建图，以及Nav2路径规划与避障功能。

### 1.2 Key Features / 核心功能

| Feature | Description | 功能描述 |
|---------|-------------|----------|
| **SLAM** | Real-time 3D LiDAR odometry with FastLIO2/Point-LIO | 基于FastLIO2/Point-LIO的实时3D激光里程计 |
| **Mapping** | 3D PCD to 2D PGM map conversion | 3D点云到2D栅格地图转换 |
| **Navigation** | Nav2 global/local planning with DWB controller | Nav2全局/局部路径规划，DWB控制器 |
| **Obstacle Avoidance** | Real-time static and dynamic obstacle detection | 实时静态/动态障碍物检测避障 |
| **Chassis Control** | Serial communication with STM32 C-board | 串口通信控制STM32 C板底盘 |

### 1.3 Performance Achieved / 已实现性能

**Indoor Navigation (5×5m场地):**

| Metric | Value | 指标 | 数值 |
|--------|-------|------|------|
| Cruise Speed | 0.2-0.26 m/s | 巡航速度 | 0.2-0.26 m/s |
| Obstacle Clearance | ≥ 0.3 m | 避障距离 | ≥ 0.3 m |
| Position Error | ≤ 0.2 m | 终点误差 | ≤ 0.2 m |
| Success Rate | ≥ 90% (20 trials) | 成功率 | ≥ 90% (20次) |
| Collision-free | Yes (5+ consecutive runs) | 无碰撞 | 是 (连续5次+) |

---

## 2. System Architecture / 系统架构

### 2.1 System Diagram / 系统框图

```
┌─────────────────────────────────────────────────────────────────────────┐
│                    SENTRY NAVIGATION SYSTEM                              │
│                        哨兵导航系统                                        │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                          │
│  ┌──────────────┐    ┌──────────────┐    ┌──────────────────────────┐   │
│  │   LiDAR      │───▶│  SLAM Node   │───▶│   /Odometry              │   │
│  │ Mid-360/L2   │    │ FastLIO/PLIO │    │   /cloud_registered      │   │
│  └──────────────┘    └──────────────┘    └──────────────────────────┘   │
│         │                   │                        │                   │
│         ▼                   ▼                        ▼                   │
│  ┌──────────────┐    ┌──────────────┐    ┌──────────────────────────┐   │
│  │  IMU Data    │    │   TF Tree    │    │  pointcloud_to_laserscan │   │
│  │  200 Hz      │    │  Transforms  │    │     /scan (10 Hz)        │   │
│  └──────────────┘    └──────────────┘    └──────────────────────────┘   │
│                             │                        │                   │
│                             ▼                        ▼                   │
│  ┌─────────────────────────────────────────────────────────────────┐    │
│  │                         NAV2 STACK                               │    │
│  │  ┌───────────┐  ┌─────────────┐  ┌───────────┐  ┌───────────┐   │    │
│  │  │  AMCL     │  │   Planner   │  │Controller │  │ Costmap2D │   │    │
│  │  │Localization│  │  NavFn/A*  │  │   DWB     │  │  Layers   │   │    │
│  │  └───────────┘  └─────────────┘  └───────────┘  └───────────┘   │    │
│  └─────────────────────────────────────────────────────────────────┘    │
│                                    │                                     │
│                                    ▼                                     │
│  ┌─────────────────────────────────────────────────────────────────┐    │
│  │                    /cmd_vel (20 Hz)                              │    │
│  │              Twist: linear.x, linear.y, angular.z                │    │
│  └─────────────────────────────────────────────────────────────────┘    │
│                                    │                                     │
│                                    ▼                                     │
│  ┌──────────────────────────────────────────────────────────────────┐   │
│  │                    serial_sender.py                               │   │
│  │     ROS2 → UART (115200 baud) → STM32 C-Board → Chassis          │   │
│  └──────────────────────────────────────────────────────────────────┘   │
│                                                                          │
└─────────────────────────────────────────────────────────────────────────┘
```

### 2.2 TF Tree Structure / TF坐标树结构

```
                    map
                     │
                     ▼
                   odom ─────────────────────────┐
                     │                            │
                     ▼                            │ (static: 0,0,0)
               camera_init                        │
                     │                            │
                     ▼                            │
            body / aft_mapped                     │
                     │                            │
                     ▼                            │
               base_link ◀───────────────────────┘
                     │         (static: pitch -50°)
                     ▼
              base_footprint
```

### 2.3 Data Flow / 数据流向

```
LiDAR (10Hz) ──▶ SLAM ──▶ /cloud_registered ──▶ /scan ──▶ Nav2 ──▶ /cmd_vel ──▶ Chassis
                  │
                  ▼
              /Odometry (10Hz)
                  │
                  ▼
          TF: odom → base_link
```

---

## 3. Hardware Configuration / 硬件配置

### 3.1 Computing Platform / 计算平台

| Component | NUC 12 Pro (Primary) | Jetson Orin Nano (Backup) |
|-----------|---------------------|---------------------------|
| **组件** | **NUC 12 Pro (主力)** | **Jetson Orin Nano (备用)** |
| CPU | Intel i7-1260P (16核) | ARM Cortex-A78AE (6核) |
| RAM | 16 GB DDR4 | 8 GB LPDDR5 |
| Storage | 512 GB NVMe SSD | 128 GB eMMC |
| OS | Ubuntu 22.04 LTS | Ubuntu 22.04 (JetPack 6) |
| Power | 19V DC | 9-20V DC |
| Suitability | Full stack (recommended) | Edge computing, mapping only |
| 适用场景 | 全栈运行 (推荐) | 边缘计算，仅建图 |

### 3.2 LiDAR Sensors / 激光雷达

| Specification | Livox Mid-360 ✅ | Unitree L2 ⚠️ |
|---------------|------------------|---------------|
| **规格** | **Livox Mid-360 (推荐)** | **Unitree L2 (实验)** |
| FOV | 360° × 59° | 360° × 90° |
| Range | 40 m | 30 m |
| Points/sec | 200,000 | 43,200 |
| IMU | Built-in (stable) | Built-in (noisy on gimbal) |
| Interface | Ethernet (UDP) | USB Serial (ttyACM0) |
| Data Format | CustomMsg (offset_time) | PointCloud2 |
| Time Sync | ✅ Excellent | ⚠️ Weak |
| Tilted Mount | ✅ Supported | ⚠️ IMU drift issue |
| Status | **Production Ready** | **Experimental** |

**Mid-360 Network Configuration / Mid-360网络配置:**
- LiDAR IP: `192.168.1.182`
- Host IP: `192.168.1.2`
- UDP Ports: 56101-56501

### 3.3 Chassis Controller / 底盘控制器

| Item | Specification | 规格 |
|------|---------------|------|
| Controller | STM32 C-Board (RoboMaster) | STM32 C板 |
| Interface | UART via USB | USB转串口 |
| Baud Rate | 115200 | 波特率 115200 |
| Protocol | Binary (15 bytes) | 二进制协议 (15字节) |
| Frame Format | `0xA5 0x5A [vx:4B] [vy:4B] [wz:4B] [CRC8]` | 帧格式 |
| Port | `/dev/ttyACM0` | 设备端口 |

### 3.4 Remote Access / 远程访问

| Service | Port | Address | 用途 |
|---------|------|---------|------|
| SSH | 9913 | 42.192.208.124:9913 | Terminal access |
| VNC | 9914 | 42.192.208.124:9914 | Desktop view |

```bash
# SSH login / SSH登录
ssh nyu@42.192.208.124 -p 9913

# VNC viewer configuration / VNC配置
VNC Viewer → 42.192.208.124:9914
```

---

## 4. Software Stack / 软件栈

### 4.1 Core Dependencies / 核心依赖

| Package | Version | Purpose | 用途 |
|---------|---------|---------|------|
| ROS 2 | Humble | Middleware | 中间件 |
| Nav2 | Humble | Navigation | 导航框架 |
| FastLIO2 | Latest | SLAM (Mid-360) | 激光SLAM |
| Point-LIO | ROS2 fork | SLAM (Unitree L2) | 激光SLAM |
| livox_ros_driver2 | 1.1.2 | Mid-360 driver | Mid-360驱动 |
| unitree_lidar_ros2 | Latest | Unitree L2 driver | L2驱动 |
| pcd2pgm | Latest | Map conversion | 地图转换 |
| pointcloud_to_laserscan | Latest | 3D→2D scan | 点云转激光 |

### 4.2 Workspace Structure / 工作空间结构

```
~/nav_ws/
├── src/
│   ├── FAST_LIO/                # FastLIO2 SLAM
│   │   ├── config/mid360.yaml   # LiDAR config
│   │   └── PCD/scans.pcd        # Saved point cloud
│   ├── point_lio_ros2/          # Point-LIO SLAM
│   │   └── config/unilidar_l2.yaml
│   ├── livox_ros_driver2/       # Mid-360 driver
│   │   └── config/MID360_config.json
│   ├── unitree_lidar_ros2/      # Unitree L2 driver
│   ├── pcd2pgm/                 # PCD → PGM converter
│   └── pointcloud_to_laserscan/ # 3D → 2D converter
├── install/                     # Compiled packages
├── my_nav2_params.yaml          # Nav2 parameters (production)
├── my_nav2_params_test.yaml     # Nav2 parameters (testing)
├── start_robot.sh               # One-click launch script
└── serial_sender.py             # Chassis control bridge
```

### 4.3 Key ROS2 Topics / 核心话题

| Topic | Type | Frequency | Description |
|-------|------|-----------|-------------|
| `/livox/lidar` | CustomMsg | 10 Hz | Raw point cloud |
| `/livox/imu` | Imu | 200 Hz | IMU data |
| `/Odometry` | Odometry | 10 Hz | Pose from SLAM |
| `/cloud_registered` | PointCloud2 | 10 Hz | Registered cloud |
| `/scan` | LaserScan | 10 Hz | 2D laser scan |
| `/cmd_vel` | Twist | 20 Hz | Velocity commands |
| `/map` | OccupancyGrid | Static | Navigation map |

---

## 5. Installation Guide / 安装指南

### 5.1 Prerequisites / 前置条件

```bash
# Install ROS 2 Humble (if not installed)
# 安装ROS 2 Humble (如未安装)
sudo apt update && sudo apt install -y ros-humble-desktop

# Install Nav2
# 安装Nav2
sudo apt install -y ros-humble-navigation2 ros-humble-nav2-bringup

# Install dependencies
# 安装依赖
sudo apt install -y \
  ros-humble-pcl-ros \
  ros-humble-tf2-tools \
  ros-humble-pointcloud-to-laserscan \
  python3-serial \
  libpcl-dev
```

### 5.2 Clone and Build / 克隆并编译

```bash
# Create workspace / 创建工作空间
mkdir -p ~/nav_ws/src && cd ~/nav_ws/src

# Clone repositories / 克隆仓库
git clone https://github.com/Livox-SDK/livox_ros_driver2.git
git clone https://github.com/hku-mars/FAST_LIO.git
git clone https://github.com/LihanChen2004/pcd2pgm.git

# Build / 编译
cd ~/nav_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

### 5.3 Network Setup (Mid-360) / 网络配置

```bash
# Set static IP / 设置静态IP
sudo nmcli con mod "Wired connection 1" ipv4.addresses 192.168.1.2/24
sudo nmcli con mod "Wired connection 1" ipv4.method manual
sudo nmcli con up "Wired connection 1"

# Disable firewall / 关闭防火墙
sudo ufw disable

# Increase UDP buffer / 增大UDP缓冲区
sudo sysctl -w net.core.rmem_max=26214400
sudo sysctl -w net.core.rmem_default=26214400
```

---

## 6. Quick Start / 快速启动

### 6.1 One-Click Launch (Recommended) / 一键启动 (推荐)

```bash
cd ~/nav_ws
./start_robot.sh
```

This script automatically: / 该脚本自动执行:
1. Sources workspace / 加载工作空间
2. Launches LiDAR driver / 启动雷达驱动
3. Starts SLAM node / 启动SLAM节点
4. Publishes TF transforms / 发布TF变换
5. Converts point cloud to laser scan / 点云转激光扫描
6. Launches Nav2 / 启动Nav2
7. Opens RViz2 / 打开RViz2

### 6.2 Manual Launch Steps / 手动启动步骤

**Step 1: LiDAR Driver / 雷达驱动**
```bash
cd ~/nav_ws && source install/setup.bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

**Step 2: SLAM / 激光SLAM**
```bash
export LD_PRELOAD=/lib/x86_64-linux-gnu/libusb-1.0.so.0
ros2 launch fast_lio mapping.launch.py config_file:=mid360.yaml
```

**Step 3: TF Transforms / TF变换**
```bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom camera_init &
ros2 run tf2_ros static_transform_publisher 0 0 0 0 -0.873 0 body base_link &
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link base_footprint &
```

**Step 4: Point Cloud to LaserScan / 点云转激光**
```bash
ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node --ros-args \
  -p target_frame:=base_link \
  -p min_height:=-0.4 -p max_height:=1.0 \
  -p range_min:=0.1 -p range_max:=20.0 \
  -r cloud_in:=/cloud_registered -r scan:=/scan &
```

**Step 5: Nav2 / 导航**
```bash
ros2 launch nav2_bringup bringup_launch.py \
    use_sim_time:=False \
    map:=/home/nyu/Desktop/map/my_map.yaml \
    params_file:=/home/nyu/nav_ws/my_nav2_params.yaml &
```

**Step 6: Chassis Control / 底盘控制**
```bash
sudo chmod 777 /dev/ttyACM0
python3 serial_sender.py --port /dev/ttyACM0 --ros2
```

### 6.3 Creating a Map / 建图流程

```bash
# 1. Run SLAM to collect point cloud / 运行SLAM采集点云
ros2 launch fast_lio mapping.launch.py config_file:=mid360.yaml

# 2. Save point cloud (automatic when stopping) / 保存点云 (停止时自动保存)
# Location: ~/nav_ws/src/FAST_LIO/PCD/scans.pcd

# 3. Rotate if tilted mount (optional) / 倾斜安装时旋转 (可选)
python3 rotate_pcd.py

# 4. Convert 3D PCD to 2D PGM / 3D点云转2D栅格
ros2 launch pcd2pgm pcd2pgm_launch.py

# 5. Save map / 保存地图
ros2 run nav2_map_server map_saver_cli -f /home/nyu/Desktop/map/my_map
```

---

## 7. Configuration / 配置说明

### 7.1 Nav2 Parameters / Nav2参数

Location / 路径: `~/nav_ws/my_nav2_params.yaml`

| Parameter | Value | Description | 说明 |
|-----------|-------|-------------|------|
| `max_vel_x` | 0.26 m/s | Max forward speed | 最大前进速度 |
| `max_vel_y` | 0.26 m/s | Max lateral speed | 最大横移速度 |
| `max_vel_theta` | 0.0 rad/s | Rotation disabled | 禁用旋转 |
| `acc_lim_x/y` | 2.5 m/s² | Acceleration limit | 加速度限制 |
| `min_speed_xy` | 0.05 m/s | Min speed (deadband) | 最小速度 (死区) |
| `controller_frequency` | 10.0 Hz | Control loop rate | 控制频率 |

### 7.2 FastLIO Parameters / FastLIO参数

Location / 路径: `~/nav_ws/src/FAST_LIO/config/mid360.yaml`

```yaml
lidar_type: 2          # 2 = Livox CustomMsg
scan_line: 4           # Number of scan lines
scan_rate: 10          # Hz
point_filter_num: 3    # Point decimation factor
```

### 7.3 Mid-360 Network Config / Mid-360网络配置

Location / 路径: `~/nav_ws/src/livox_ros_driver2/config/MID360_config.json`

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

---

## 8. Performance Metrics / 性能指标

### 8.1 System Resource Usage / 系统资源占用

**Platform: NUC 12 Pro (i7-1260P, 16GB RAM)**

| Metric | Value | 指标 | 数值 |
|--------|-------|------|------|
| CPU Peak | ~40% | CPU峰值 | ~40% |
| CPU Average | ~36% | CPU均值 | ~36% |
| Memory | ~6.3 GB | 内存 | ~6.3 GB |
| Network RX | ~3.15 MB/s | 网络接收 | ~3.15 MB/s |

### 8.2 Topic Frequencies / 话题频率

| Topic | Measured | Expected | Status |
|-------|----------|----------|--------|
| `/livox/lidar` | 10 Hz | 10 Hz | ✅ |
| `/livox/imu` | 200 Hz | 200 Hz | ✅ |
| `/Odometry` | 10 Hz | 10-100 Hz | ⚠️ |
| `/scan` | 7-9 Hz | 10 Hz | ⚠️ |
| `/cmd_vel` | 20 Hz | 20 Hz | ✅ |

### 8.3 Latency Measurements / 延迟测量

| Pipeline | Latency | Target | Status |
|----------|---------|--------|--------|
| `/cmd_vel` → Serial | 0.34 ms (median) | < 1 ms | ✅ |
| Point cloud → LaserScan | 15 ms/frame | < 5 ms | ⚠️ |
| End-to-end control | ~20 ms | < 50 ms | ✅ |

### 8.4 Performance Benchmarks / 性能基准

| Metric | Good | Acceptable | Needs Work |
|--------|------|------------|------------|
| **指标** | **良好** | **可接受** | **需优化** |
| Control latency | < 1 ms | 1-5 ms | > 5 ms |
| Cloud→Scan | < 5 ms | 5-20 ms | > 20 ms |
| Frame drop rate | < 5% | 5-20% | > 20% |
| CPU peak | < 50% | 50-80% | > 80% |

### 8.5 Debugging Commands / 调试命令

```bash
# Monitor topic frequencies / 监控话题频率
ros2 topic hz /scan /Odometry /cmd_vel

# Measure latency / 测量延迟
python3 measure_pointcloud_latency.py --cloud /cloud_registered --scan /scan --duration 30

# System resources / 系统资源
./monitor_resources.sh --interval 1 --duration 60

# TF tree / TF树
ros2 run tf2_tools view_frames
```

---

## 9. Troubleshooting / 故障排除

### 9.1 Common Issues / 常见问题

#### Issue 1: No LiDAR Data / 无雷达数据
**Symptoms / 症状:** `❌ 未检测到雷达数据`

```bash
# Check network / 检查网络
ping 192.168.1.182

# Check topics / 检查话题
ros2 topic list | grep livox

# Disable firewall / 关闭防火墙
sudo ufw disable
```

#### Issue 2: TF Transform Errors / TF变换错误
**Symptoms / 症状:** `Transform from map to base_link failed`

```bash
# View TF tree / 查看TF树
ros2 run tf2_tools view_frames

# Check specific transform / 检查特定变换
ros2 run tf2_ros tf2_echo map base_link
```

#### Issue 3: AMCL Message Filter Dropping / AMCL消息丢弃
**Symptoms / 症状:** `Message Filter dropping message`

**Solutions / 解决方案:**
- Increase `queue_size` in AMCL config / 增大队列大小
- Increase `transform_tolerance` / 增大变换容忍度
- Reduce `/scan` frequency / 降低扫描频率
- Check system time sync / 检查系统时间同步

#### Issue 4: Serial Port Failed / 串口通信失败
**Symptoms / 症状:** `Failed to open serial port`

```bash
# Grant permission / 授权
sudo chmod 777 /dev/ttyACM0

# Check if occupied / 检查占用
lsof /dev/ttyACM0

# Test connection / 测试连接
python3 serial_sender.py --port /dev/ttyACM0 --vx 0.1 --duration 1.0
```

#### Issue 5: Localization Loss on Gimbal Rotation / 云台旋转时定位丢失
**Causes / 原因:**
- IMU saturation / IMU量程饱和
- Extrinsic calibration error / 外参标定误差
- Time synchronization drift / 时间同步偏差

**Solutions / 解决方案:**
- Reduce gimbal rotation speed / 降低云台转速
- Use higher range IMU / 使用更大量程IMU
- Recalibrate extrinsics / 重新标定外参
- Preheat IMU, calibrate bias / 预热IMU，校准偏置

---

## 10. Development Notes / 开发笔记

> 本章节记录开发过程中遇到的问题与解决方法，供后续开发参考。

### 10.1 LiDAR Selection Summary / 雷达选型总结

| Aspect | Mid-360 | Unitree L2 |
|--------|---------|------------|
| Time sync | ✅ CustomMsg with offset_time | ⚠️ Standard PointCloud2 |
| IMU stability | ✅ Stable | ⚠️ Noisy on gimbal |
| Tilted mount | ✅ Works with TF rotation | ⚠️ IMU drift issues |
| Production ready | ✅ Yes | ⚠️ Experimental |

**Conclusion / 结论:** Mid-360在时间同步、IMU稳定性等方面表现更优，**目前采用Mid-360作为主力雷达**。宇树L2需要进一步研究后再投入使用。

---

### 10.2 Unitree L2 LiDAR Issues / 宇树L2雷达问题

#### 10.2.1 Connection Types / 连接方式

**Config File / 配置文件:** `unilidar_sdk2/unitree_lidar_ros2/launch/launch.py`

| Type | Configuration | 配置说明 |
|------|---------------|----------|
| **USB Serial** | `serial_port: '/dev/ttyACM0'`, `initialize_type: 2` | USB串口连接 |
| **Ethernet UDP** | `lidar_ip: '10.10.10.10'`, `initialize_type: 1` | 网线连接，需配置静态IP |

```bash
# Check USB port / 检查USB端口
ls /dev/ttyACM*
```

#### 10.2.2 Known Issues / 已知问题

1. **Dynamic Balance & IMU Issues / 动平衡与IMU问题:**
   - 宇树雷达的动平衡以及IMU存在问题
   - 点云在初始化时可能旋转或飞走
   - 根据宇树官方手册可增大 `cloud_scan_num` 到72（用更多scan来自身定位，但效果不明显）
   - 需要进一步调试

2. **IMU Acceleration Instability / 加速度不稳定:**
   - 正常条件下应稳定在 ~9.8 m/s² (`acc_norm: 10.2`)
   - **一旦加速度突变会导致点云飞走**
   - 安装到云台或底盘按墙壁时加速度突变到 ~14 m/s²
   - 这是倾斜安装的主要障碍

3. **Point Cloud Spinning / 点云旋转:**
   - 动平衡不足导致
   - 增大 `cloud_scan_num` 效果有限

#### 10.2.3 Debugging Commands / 调试命令

```bash
# Official example for reset/debug / 官方调试示例
# 当点云消失或驱动无法加载时可用此方法调试
cd ~/nav_ws/src/unilidar_sdk2/unitree_lidar_sdk/build
sudo chmod 777 /dev/ttyACM0
../bin/example_lidar_serial
```

---

### 10.3 Point-LIO Configuration / Point-LIO配置详解

**Why Point-LIO? / 为什么用Point-LIO?**  

宇树雷达与目前大部分的state estimation算法不是十分适配，而 [dfloreaa/point_lio_ros2](https://github.com/dfloreaa/point_lio_ros2) 根据宇树雷达做了适配。本质来说Point-LIO和FastLIO2的性能基本相同。

**Config File / 配置文件:** `~/nav_ws/src/point_lio_ros2/config/unilidar.yaml`

| Parameter | Value | Description | 说明 |
|-----------|-------|-------------|------|
| `start_in_aggressive_motion` | `true` | 建议设为true，使用预设重力方向，避免使用IMU导致飞走 | Use preset gravity |
| `gravity_init` | `[0.0, 0.0, -9.810]` | 预设重力方向 | Preset gravity |
| `extrinsic_est_en` | `false` | 用于aggressive motion时设为false | Disable for aggressive motion |
| `acc_norm` | `10.2` | `ros2 topic echo /unilidar/imu` linear acceleration应稳定在的读数 (m/s²) | Expected acceleration |
| `b_acc_cov` / `b_gyr_cov` | `0.0001` | 偏置协方差，可适当调高但效果不明显 | Bias covariance |
| `imu_meas_acc_cov` | `0.1` | IMU加速度测量协方差 | IMU measurement covariance |
| `imu_meas_omg_cov` | `0.1` | IMU角速度测量协方差 | IMU measurement covariance |

**⚠️ Extrinsic Rotation Warning / 外参旋转警告:**
```yaml
# 虽然网上通常以此方法作为倾斜安装的解决方案，但不推荐使用！
extrinsic_T: [0.007698, 0.014655, -0.00667]
extrinsic_R: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
# 经测验：在倾斜安装情况下使用旋转矩阵可以将点云旋转
# 但问题在于会导致IMU与重力对齐极度不稳，移动旋转时点云飞走
```

**Launch File Tuning / Launch文件调参:**  
`~/nav_ws/src/point_lio_ros2/launch/mapping_unilidar_l2.launch.py`

| Parameter | NUC (推荐) | Jetson | Description |
|-----------|------------|--------|-------------|
| `point_filter_num` | 3 | 1 | 点云降采样 |
| `filter_size_surf` | 0.5 | 0.3 | 表面滤波尺寸 |
| `filter_size_map` | 0.5 | 0.3 | 地图滤波尺寸 |

> 根据CPU处理能力调整，避免时间戳问题和 "the queue is full" 错误。NUC可保持默认配置，Jetson建议降低。

---

### 10.4 Mid-360 Configuration / Mid-360配置详解

**Config File / 配置文件:** `~/nav_ws/src/livox_ros_driver2/config/MID360_config.json`

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
      "roll": 0.0, "pitch": 0.0, "yaw": 0.0,
      "x": 0, "y": 0, "z": 0
    }
  }]
}
```

> ⚠️ 由于目前Mid-360统一通过网线连接，host和lidar的config均要基于自己的实际配置进行调整

**Launch File / Launch文件:** `~/nav_ws/src/livox_ros_driver2/launch_ROS2/msg_MID360_launch.py`

```python
xfer_format = 1   # 0-Pointcloud2(PointXYZRTL), 1-customized pointcloud format
```

> ✅ **推荐使用 `xfer_format = 1` (CustomMsg)**  
> 
> Mid-360的优势在于不同于一般雷达的PointCloud2，其使用了customized pointcloud format，提供了offset_time等参数的时间戳同步。
> 
> 经测试如果在这里使用PointCloud2可选项，虽然可以跑通FastLIO，但会出现missing message报错——这是Mid-360 launch正在寻找其他Mid-360提供的参数的提示。推荐使用customized pointcloud以充分发挥优势。

---

### 10.5 Tilted LiDAR Mount Solution / 倾斜安装解决方案

机械组采用了多数战队使用的**云台倾斜安装雷达**以保证更完整的激光扫描，但这导致 `camera_init` 点云倾斜，影响建图效果。

**Solution / 解决方案:**

| Step | Action | 说明 |
|------|--------|------|
| 1 | TF rotation: `body` → `base_link` pitch -50° | 静态变换旋转坐标 |
| 2 | Use `rotate_pcd.py` to rotate saved PCD | 用rotate将PCD旋转到对应角度 |
| 3 | Do NOT use extrinsic rotation in Point-LIO | 不要用Point-LIO的外参旋转 |

```bash
# TF rotation for tilted mount / 倾斜安装的TF旋转
ros2 run tf2_ros static_transform_publisher 0 0 0 0 -0.873 0 body base_link
# Note: -0.873 rad ≈ -50°

# Rotate PCD before map conversion / 地图转换前旋转PCD
python3 rotate_pcd.py
```

> 这确保了Nav2的建图垂直并且scan正常

---

### 10.6 Mapping Workflow / 建图流程详解

使用FastLIO/Point-LIO自带的 `PCD_save=True` 配置，每次运行后自动保存 `scans.pcd`。

为满足Nav2的2D地图导航需求，使用 [pcd2pgm](https://github.com/LihanChen2004/pcd2pgm) 将PCD转为PGM。

```bash
# Complete mapping workflow / 完整建图流程

# 1. Run SLAM / 运行SLAM
ros2 launch fast_lio mapping.launch.py config_file:=mid360.yaml

# 2. View saved PCD / 查看保存的点云
cd ~/nav_ws/src/FAST_LIO/PCD/
pcl_viewer scans.pcd

# 3. Rotate PCD if tilted / 倾斜安装时旋转
python3 rotate_pcd.py

# 4. Convert 3D→2D / 3D转2D
ros2 launch pcd2pgm pcd2pgm_launch.py

# 5. Save map / 保存地图
cd ~/Desktop/map
ros2 run nav2_map_server map_saver_cli -f my_map
```

详细命令见 [mid360_command.txt](mid360_command.txt)

---

### 10.7 Navigation Status / 导航现状

**Current Achievement / 目前成果:**

使用Nav2架构，详细参数见 `my_nav2_params.yaml`。目前实现了：

✅ **基础导航：**
- 在5×5m室内场地内，机器人从起点A建图并自主导航至目标点B
- 绕过1个固定障碍物
- 实时避开动态障碍物
- 全程保持避障距离 ≥ 0.3m
- 巡航速度 0.2m/s
- 终点位置误差 ≤ 0.2m
- 连续多次运行无碰撞

✅ **动态避障：**
- 避障距离 ≥ 0.5m
- 巡航速度 0.26m/s
- 连续5次运行无碰撞
- 20次尝试成功率 ≥ 90%

> 📋 中期考核已完成，该文档会持续更新

---

## 11. Future Plans / 未来计划

- [ ] Automatic initialization for Unitree L2 / L2自动初始化
- [ ] IMU calibration procedure / IMU标定流程
- [ ] Dynamic obstacle avoidance improvement / 动态避障改进
- [ ] Battery monitoring via ROS / ROS电池监控
- [ ] Multi-floor navigation / 多楼层导航
- [ ] Web-based monitoring dashboard / Web监控面板
- [ ] Automatic recovery behaviors / 自动恢复行为
- [ ] Further LiDAR research for radar station / 根据雷达站规划进一步更换雷达

---

## 12. References / 参考资料

### Official Documentation / 官方文档
- [Livox Mid-360 Manual](https://www.livoxtech.com/mid-360/downloads)
- [FAST-LIO GitHub](https://github.com/hku-mars/FAST_LIO)
- [Nav2 Documentation](https://docs.nav2.org/)
- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/)

### Related Projects / 相关项目
- [Sentry Chassis Control](https://github.com/NYUSH-Robotics-Club/robomaster-control/tree/alan_sentry_radar)
- [Livox ROS Driver 2](https://github.com/Livox-SDK/livox_ros_driver2)
- [Point-LIO](https://github.com/hku-mars/Point-LIO)
- [Point-LIO ROS2 (Unitree adapted)](https://github.com/dfloreaa/point_lio_ros2)
- [pcd2pgm](https://github.com/LihanChen2004/pcd2pgm)

### Useful Tools / 实用工具
- **pcl_viewer:** Point cloud visualization / 点云可视化
- **foxglove:** Advanced ROS2 visualization / 高级ROS2可视化
- **plotjuggler:** Real-time data plotting / 实时数据绘图

---

## Contributing / 贡献指南

If you find bugs or have suggestions: / 如发现问题或有建议:

1. Create an issue in the repository / 在仓库中创建issue
2. Fork and create a pull request / Fork并创建PR
3. Contact: Yanheng Zhu (yz11502@nyu.edu)

---

## License / 许可证

This project integrates multiple open-source components: / 本项目集成多个开源组件:

- **FAST-LIO:** HKU Mars Lab (GPLv2)
- **Nav2:** ROS 2 Navigation Team (Apache 2.0)
- **Livox SDK:** Livox Technology
- **Unitree SDK:** Unitree Robotics

**Special Thanks / 特别感谢:**
- NYU Shanghai Robotics Club
- All contributors and testers

---

**Last Updated / 最后更新:** January 2025  
**Maintained by / 维护者:** Yanheng Zhu  
**Contact / 联系方式:** yz11502@nyu.edu
