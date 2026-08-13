# RoboMaster 哨兵自主导航系统

<div align="center">

[![简体中文](https://img.shields.io/badge/%E7%AE%80%E4%BD%93%E4%B8%AD%E6%96%87-%E5%BD%93%E5%89%8D-0969da?style=for-the-badge)](README_LIDAR.md)
[![English](https://img.shields.io/badge/English-Switch-2ea44f?style=for-the-badge)](README_LIDAR_EN.md)

[![ROS 2 Humble](https://img.shields.io/badge/ROS%202-Humble-22314E?logo=ros&logoColor=white)](https://docs.ros.org/en/humble/)
[![Ubuntu 22.04](https://img.shields.io/badge/Ubuntu-22.04-E95420?logo=ubuntu&logoColor=white)](https://ubuntu.com/)
[![License: MIT](https://img.shields.io/badge/License-MIT-2ea44f.svg)](LICENSE)

**基于激光雷达 SLAM 与 Nav2 的 RoboMaster 哨兵自主导航方案**

</div>

最后更新：2026-04-11

## 0. 文档定位

| 文档 | 职责 |
|---|---|
| [README.md](README.md) | 中央索引：项目总览、架构、编译顺序与最短启动路径 |
| [README_COMMUNICATION.md](README_COMMUNICATION.md) | `sentry_bridge`、PTY、`serial_sender`、协议帧与 ROS 2 通信链路 |
| [README_BEHAVIOR_TREE_FLOW.md](README_BEHAVIOR_TREE_FLOW.md) | 行为树 XML、`SendGoal`、`RobotControl`、Groot2 与调试脚本 |
| **本文** | Mid-360、FAST-LIO、Nav2、建图、Gazebo Sim2Real、性能与定位排障 |
| [README_COMMANDS.md](README_COMMANDS.md) | 可执行命令、环境变量、地图文件、坐标选择与分阶段实机联调 |

阅读建议：

- Gazebo、RMUL2026、`bringup_sim` 与 Sim2Real：本文 §6.4 + [mid360 command.txt](mid360%20command.txt)。
- 点云、FAST-LIO、Nav2 参数、代价地图与 TF：本文其余章节。
- `MAP_FILE`、PCD/PGM/YAML 和实机终端顺序：[README_COMMANDS.md](README_COMMANDS.md)。
- 串口、bridge 与 PTY：[README_COMMUNICATION.md](README_COMMUNICATION.md)。

### 推荐 Sim2Real 路线

队内推荐先在 Gazebo RMUL2026 + Nav2 中跑通代价地图、速度、`SendGoal` 与 `center_attack_simple` 的去中心 / 守中 / 回家分支，再切换到 Mid-360 + FAST-LIO 实车。

| 阶段 | 入口 |
|---|---|
| Gazebo 启动与裁判话题测试 | [mid360 command.txt](mid360%20command.txt) |
| 仿真参数与实车差异 | 本文 §6.4 |
| 行为树与 Groot2 | [README_BEHAVIOR_TREE_FLOW.md](README_BEHAVIOR_TREE_FLOW.md) |
| bridge、PTY 与实机多终端 | [README_COMMUNICATION.md](README_COMMUNICATION.md)、[README_COMMANDS.md](README_COMMANDS.md) |

## 目录

1. [项目概述](#1-项目概述)
2. [系统架构](#2-系统架构)
3. [硬件配置](#3-硬件配置)
4. [软件栈](#4-软件栈)
5. [安装指南](#5-安装指南)
6. [快速启动](#6-快速启动)
7. [配置说明](#7-配置说明)
8. [性能指标](#8-性能指标)
9. [故障排除](#9-故障排除)
10. [开发笔记](#10-开发笔记)
11. [后续计划](#11-后续计划)
12. [参考资料](#12-参考资料)

## 1. 项目概述

### 1.1 简介

本系统将 3D 激光雷达里程计与 Nav2 组合，为 RoboMaster 哨兵提供实时定位、建图、路径规划、动态避障和底盘速度输出。当前实车主力传感器为 Livox Mid-360，Unitree L2 保留为实验方案。

### 1.2 核心能力

| 模块 | 能力 |
|---|---|
| SLAM | FastLIO2 / Point-LIO 实时 3D 激光里程计 |
| 建图 | 3D PCD 姿态修正、投影与 2D PGM 地图保存 |
| 导航 | Nav2 全局 / 局部规划与 DWB 控制器 |
| 避障 | 基于 `/scan` 与 costmap 的静态、动态障碍检测 |
| 决策衔接 | 行为树 `SendGoal`、`navigate_to_pose` 与目标点管理 |
| 底盘衔接 | `/cmd_vel` → 坐标变换 → BT 合成 → sender / bridge → MCU |

### 1.3 已记录的室内基线

以下数据来自 5×5 m 室内测试，用于回归参考，不等同于正式赛场指标。

| 指标 | 结果 |
|---|---|
| 巡航速度 | 0.20–0.26 m/s |
| 避障距离 | ≥ 0.3 m |
| 终点误差 | ≤ 0.2 m |
| 20 次测试成功率 | ≥ 90% |
| 连续无碰撞运行 | ≥ 5 次 |

## 2. 系统架构

### 2.1 系统框图

```text
Mid-360 / Unitree L2
          │
          ▼
   LiDAR driver + IMU
          │
          ▼
 FAST-LIO / Point-LIO ──► odometry + TF + /cloud_registered
          │
          ▼
 pointcloud_to_laserscan ──► /scan
          │
          ▼
 Nav2: localization + planner + controller + costmaps
          │
          ▼
 /cmd_vel ──► fake_vel_transform ──► /cmd_vel_chassis
          │
          ▼
 bt_comm_adapter ──► /cmd_vel_chassis_bt
          │
          ▼
 serial_sender ──► Radar PTY ──► sentry_bridge ──► MCU ──► chassis
```

### 2.2 TF 树

```text
map
 └─ odom
     └─ camera_init
         └─ body / aft_mapped
             └─ base_link
                 └─ base_footprint
```

实际运行时必须保证 `map -> odom -> base_link` 连续、时间戳有效，并让点云和 `/scan` 使用正确的目标坐标系。倾斜安装时，当前方案通过 `body -> base_link` 的静态 pitch 旋转校正。

### 2.3 数据流

```text
LiDAR 10 Hz -> SLAM -> /cloud_registered -> /scan -> Nav2 -> /cmd_vel
                   └-> /Odometry -> TF: odom -> base_link
```

## 3. 硬件配置

### 3.1 计算平台

| 组件 | NUC 12 Pro（主力） | Jetson Orin Nano（备用） |
|---|---|---|
| CPU | Intel i7-1260P | ARM Cortex-A78AE |
| 内存 | 16 GB DDR4 | 8 GB LPDDR5 |
| 存储 | 512 GB NVMe SSD | 128 GB eMMC |
| 系统 | Ubuntu 22.04 LTS | Ubuntu 22.04 / JetPack 6 |
| 使用建议 | 完整导航与决策栈 | 边缘计算或建图实验 |

### 3.2 激光雷达

| 规格 | Livox Mid-360 | Unitree L2 |
|---|---|---|
| 状态 | 主力 / 实机 | 实验 |
| 视场 | 360° × 59° | 360° × 90° |
| 量程 | 40 m | 30 m |
| 点频 | 200,000 points/s | 43,200 points/s |
| IMU | 内置，当前较稳定 | 云台安装下噪声与漂移明显 |
| 接口 | Ethernet / UDP | USB Serial 或 Ethernet |
| 数据 | Livox CustomMsg | PointCloud2 |
| 时间同步 | 较好 | 需进一步调试 |
| 倾斜安装 | 已有方案 | 仍有 IMU 漂移风险 |

Mid-360 默认网络：

- 雷达 IP：`192.168.1.182`
- 主机 IP：`192.168.1.2`
- UDP 端口：56101–56501

### 3.3 底盘控制器

底盘由 RoboMaster STM32 C 板控制。当前推荐通信不是让导航进程直接打开 `/dev/ttyACM0`，而是：

```text
/cmd_vel_chassis_bt -> serial_sender -> Radar PTY
                    -> sentry_bridge -> MCU -> chassis
```

真实 USB CDC 只由 `sentry_bridge` 占用；速度帧、A3、SX/ST 与 CRC 细节见 [README_COMMUNICATION.md](README_COMMUNICATION.md)。

### 3.4 远程访问

实车常通过 SSH 和 VNC 维护。地址与端口属于部署配置，不应硬编码进通用启动脚本：

```bash
ssh <user>@<host> -p <ssh-port>
```

需要运行 RViz、Groot2 或 `map_point_picker.py` 时，使用本地显示器、X11 转发或 VNC 会话。

## 4. 软件栈

### 4.1 核心依赖

| 软件包 | 版本 / 分支 | 用途 |
|---|---|---|
| ROS 2 | Humble | 中间件 |
| Nav2 | Humble | 导航框架 |
| FAST-LIO | 当前 ROS 2 工作版本 | Mid-360 SLAM |
| Point-LIO | ROS 2 fork | Unitree L2 实验 SLAM |
| `livox_ros_driver2` | 当前工作版本 | Mid-360 驱动 |
| `unitree_lidar_ros2` | 当前工作版本 | Unitree L2 驱动 |
| `pcd2pgm` | 当前工作版本 | PCD 转二维占用栅格 |
| `pointcloud_to_laserscan` | ROS 2 | 3D 点云转 `/scan` |

### 4.2 工作空间结构

```text
~/nav_ws/
├── src/
│   ├── FAST_LIO/
│   │   ├── config/mid360.yaml
│   │   └── PCD/scans.pcd
│   ├── point_lio_ros2/
│   │   └── config/unilidar_l2.yaml
│   ├── livox_ros_driver2/
│   │   └── config/MID360_config.json
│   ├── unitree_lidar_ros2/
│   ├── pcd2pgm/
│   └── pointcloud_to_laserscan/
├── install/
├── my_nav2_params.yaml
├── my_nav2_params_test.yaml
└── start_robot.sh
```

本仓库的 `rm_navigation_ws` 还包含 `rm_nav_bringup`、仿真世界、导航包与感知适配包。

### 4.3 核心 ROS 2 话题

| 话题 | 类型 | 典型频率 | 用途 |
|---|---|---|---|
| `/livox/lidar` | CustomMsg | 10 Hz | 原始点云 |
| `/livox/imu` | Imu | 200 Hz | IMU |
| `/Odometry` | Odometry | 10 Hz | SLAM 位姿 |
| `/cloud_registered` | PointCloud2 | 10 Hz | 配准点云 |
| `/scan` | LaserScan | 约 10 Hz | Nav2 二维激光 |
| `/cmd_vel` | Twist | 20 Hz | Nav2 速度 |
| `/map` | OccupancyGrid | 静态 | 二维导航地图 |

## 5. 安装指南

### 5.1 前置依赖

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

### 5.2 克隆与编译

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

项目实际使用的 fork、分支与补丁以本机 `nav_ws` 和仓库锁定版本为准；上面的命令用于说明依赖来源。

### 5.3 Mid-360 网络

```bash
sudo nmcli con mod "Wired connection 1" ipv4.addresses 192.168.1.2/24
sudo nmcli con mod "Wired connection 1" ipv4.method manual
sudo nmcli con up "Wired connection 1"

sudo ufw disable
sudo sysctl -w net.core.rmem_max=26214400
sudo sysctl -w net.core.rmem_default=26214400
```

在比赛网络或受管设备上关闭防火墙前，应确认队内网络策略；至少保证 Livox UDP 端口可达。

## 6. 快速启动

### 6.1 一键启动

完整实车链推荐从仓库根目录启动：

```bash
cd /path/to/nyush_rm_sentry
./start_robot.sh
```

它可统一启动 Mid-360 驱动、静态 TF、FAST-LIO、点云转 `/scan`、Nav2、`bt_comm_adapter`、行为树，以及可选的 `serial_sender`。bridge 必须提前单独启动。

只调导航工作空间时也可使用：

```bash
cd ~/nav_ws
./start_robot.sh
```

两份脚本默认值并不完全相同，运行前应查看文件头部与 [README_COMMANDS.md](README_COMMANDS.md) 的环境变量说明。

### 6.2 手动启动

步骤 1：Mid-360 驱动。

```bash
cd ~/nav_ws
source install/setup.bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

步骤 2：FAST-LIO。

```bash
export LD_PRELOAD=/lib/x86_64-linux-gnu/libusb-1.0.so.0
ros2 launch fast_lio mapping.launch.py config_file:=mid360.yaml
```

步骤 3：静态 TF。

```bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom camera_init
ros2 run tf2_ros static_transform_publisher 0 0 0 0 -0.873 0 body base_link
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link base_footprint
```

步骤 4：点云转 LaserScan。

```bash
ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node --ros-args \
  -p target_frame:=base_link \
  -p min_height:=-0.4 -p max_height:=1.0 \
  -p range_min:=0.1 -p range_max:=20.0 \
  -r cloud_in:=/cloud_registered -r scan:=/scan
```

步骤 5：Nav2。

```bash
ros2 launch nav2_bringup bringup_launch.py \
  use_sim_time:=False \
  map:=/home/nyu/Desktop/map/my_map.yaml \
  params_file:=/home/nyu/nav_ws/my_nav2_params.yaml
```

步骤 6：底盘通信。不要让 sender 直接抢真实串口；先运行 bridge，再让 sender 指向 Radar PTY。完整命令见 [README_COMMUNICATION.md](README_COMMUNICATION.md)。

### 6.3 建图流程

```bash
# 1. 运行 SLAM 采集点云
ros2 launch fast_lio mapping.launch.py config_file:=mid360.yaml

# 2. 停止时自动保存到 FAST_LIO/PCD/scans.pcd

# 3. 倾斜安装时旋转 PCD
python3 rotate_pcd.py

# 4. 3D PCD 转 2D 栅格
ros2 launch pcd2pgm pcd2pgm_launch.py

# 5. 保存地图
ros2 run nav2_map_server map_saver_cli \
  -f /home/nyu/Desktop/map/my_map
```

PCD、PGM、YAML 的职责和目标点重标定见 [README_COMMANDS.md](README_COMMANDS.md)。

<a id="nyush-gazebo-sim2real"></a>

### 6.4 Gazebo RMUL2026 + Nav2：Sim2Real 第一步

环境为 ROS 2 Humble + Gazebo Classic 11。仿真使用 Gazebo 里程计和 AMCL，无需连接真实 Mid-360，适合先稳定 Nav2 代价地图、速度、`SendGoal` 与行为树分支。

终端 1：世界、Nav2 和 RViz。

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

终端 2：长期行为树调试会话。

```bash
bash /path/to/nyush_rm_sentry/scripts/run_center_attack_debug_session.sh
```

脚本默认 `USE_SIM_TIME=True`，并保活 `bt_comm_adapter`、`rm_behavior_tree` 与 `watch_center_attack_state.py`。Groot2 监控需启用 `enable_groot:=true`，默认端口为 1667。

终端 3：使用 [mid360 command.txt](mid360%20command.txt) §4–§8 的 `ros2 topic pub` 示例伪造裁判状态，验证 `APPROACH_CENTER`、`CENTER_HOLD_ATTACK` 与 `HOME_RECOVER`。

| 项目 | Gazebo | 实车 |
|---|---|---|
| 定位 | `use_gazebo_odom:=true` + AMCL | FAST-LIO / `bringup_real` 等 |
| 时间 | `use_sim_time:=true` | 通常 `false` |
| MCU 输出 | 通常只验证 ROS 链；可选 Radar PTY + sender | bridge + `serial_sender` |

可选 Groot2：

```bash
cd ~/Desktop
./Groot2-v1.9.0-x86_64.AppImage
```

打开 `rm_decision_ws/rm_behavior_tree/config/Project.btproj`，Monitor 连接 `127.0.0.1:1667`。AppImage 名称以实际下载版本为准。

## 7. 配置说明

### 7.1 Nav2 参数

路径：`~/nav_ws/my_nav2_params.yaml`

| 参数 | 当前值 | 含义 |
|---|---|---|
| `max_vel_x` | 0.26 m/s | 最大前进速度 |
| `max_vel_y` | 0.26 m/s | 最大横移速度 |
| `max_vel_theta` | 0.0 rad/s | Nav2 直接旋转关闭 |
| `acc_lim_x/y` | 2.5 m/s² | 加速度限制 |
| `min_speed_xy` | 0.05 m/s | 平移死区 |
| `controller_frequency` | 10.0 Hz | 控制循环频率 |

`max_vel_theta=0` 与当前由行为树 `chassis_spin_vel` 独立控制小陀螺的策略一致。

### 7.2 FAST-LIO 参数

路径：`~/nav_ws/src/FAST_LIO/config/mid360.yaml`

```yaml
lidar_type: 2
scan_line: 4
scan_rate: 10
point_filter_num: 3
```

### 7.3 Mid-360 网络配置

路径：`~/nav_ws/src/livox_ros_driver2/config/MID360_config.json`

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

## 8. 性能指标

### 8.1 资源占用基线

平台：NUC 12 Pro，i7-1260P，16 GB RAM。

| 指标 | 记录值 |
|---|---|
| CPU 峰值 | 约 40% |
| CPU 平均值 | 约 36% |
| 内存 | 约 6.3 GB |
| 网络接收 | 约 3.15 MB/s |

### 8.2 话题频率

| 话题 | 实测 | 期望 | 状态 |
|---|---|---|---|
| `/livox/lidar` | 10 Hz | 10 Hz | 正常 |
| `/livox/imu` | 200 Hz | 200 Hz | 正常 |
| `/Odometry` | 10 Hz | 10–100 Hz | 需结合配置观察 |
| `/scan` | 7–9 Hz | 10 Hz | 可优化 |
| `/cmd_vel` | 20 Hz | 20 Hz | 正常 |

### 8.3 延迟

| 链路 | 记录值 | 目标 |
|---|---|---|
| `/cmd_vel` → sender | 中位数 0.34 ms | < 1 ms |
| 点云 → LaserScan | 约 15 ms / 帧 | < 5 ms 理想，< 20 ms 可接受 |
| 端到端控制 | 约 20 ms | < 50 ms |

### 8.4 调试命令

```bash
ros2 topic hz /scan /Odometry /cmd_vel
python3 measure_pointcloud_latency.py \
  --cloud /cloud_registered --scan /scan --duration 30
./monitor_resources.sh --interval 1 --duration 60
ros2 run tf2_tools view_frames
```

## 9. 故障排除

### 9.1 无 Mid-360 数据

```bash
ping 192.168.1.182
ros2 topic list | grep livox
ip address
```

核对主机静态 IP、雷达 JSON、网卡名称、UDP 缓冲区和防火墙规则。

### 9.2 TF 变换失败

典型报错：`Transform from map to base_link failed`。

```bash
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo map base_link
```

确认不存在重复 TF 发布者，时间源一致，`map -> odom -> base_link` 连续。

### 9.3 AMCL Message Filter 丢包

- 增大 AMCL `queue_size`。
- 增大 `transform_tolerance`。
- 检查 `/scan` 频率与 frame ID。
- 检查系统时间和 `use_sim_time`。

### 9.4 串口或底盘无响应

```bash
lsof /dev/ttyACM0
ls -l /tmp/nyush-rm-sentry-*
ros2 topic echo /cmd_vel_chassis_bt --once
ros2 topic echo /robot_control --once
```

理想状态是只有 `sentry_bridge.py` 占用真实口。不要使用旧的直接串口测试命令与 bridge 同时抢口；详细排障见 [README_COMMUNICATION.md](README_COMMUNICATION.md)。

### 9.5 云台旋转时定位丢失

可能原因：IMU 饱和、外参错误、时间同步漂移或机械振动。

- 降低云台转速。
- 预热 IMU 并检查 bias。
- 重新核对外参和静态 TF。
- 检查点云时间戳与 IMU 时间轴。
- 必要时使用更大量程、更稳定的 IMU。

## 10. 开发笔记

### 10.1 雷达选型结论

| 维度 | Mid-360 | Unitree L2 |
|---|---|---|
| 时间同步 | CustomMsg 提供 `offset_time`，当前稳定 | 标准 PointCloud2 链仍需调试 |
| IMU | 当前实测稳定 | 云台安装下噪声明显 |
| 倾斜安装 | 静态 TF + 离线 PCD 旋转可用 | 有重力对齐与漂移问题 |
| 当前状态 | 实机主力 | 实验 |

### 10.2 Unitree L2 已知问题

连接配置位于 `unilidar_sdk2/unitree_lidar_ros2/launch/launch.py`：

| 方式 | 配置 |
|---|---|
| USB Serial | `serial_port: '/dev/ttyACM0'`, `initialize_type: 2` |
| Ethernet UDP | `lidar_ip: '10.10.10.10'`, `initialize_type: 1` |

注意：L2 的 `/dev/ttyACM0` 与 MCU CDC 可能发生设备名冲突，必须按 USB 标识或 udev 规则区分，不能仅凭设备号判断。

已观察问题：

1. 初始化时点云可能旋转或飞走；提高 `cloud_scan_num` 到 72 改善有限。
2. 正常 `acc_norm` 约为 10.2 m/s²；突变到约 14 m/s² 时容易导致点云失稳。
3. 云台或倾斜安装放大动平衡与 IMU 重力对齐问题。

官方串口示例：

```bash
cd ~/nav_ws/src/unilidar_sdk2/unitree_lidar_sdk/build
sudo chmod 777 /dev/ttyACM0
../bin/example_lidar_serial
```

执行前先确认这个 ACM 设备确实是 L2，而不是 MCU。

### 10.3 Point-LIO 配置

适配仓库：[dfloreaa/point_lio_ros2](https://github.com/dfloreaa/point_lio_ros2)。配置文件：`~/nav_ws/src/point_lio_ros2/config/unilidar.yaml`。

| 参数 | 当前值 | 说明 |
|---|---|---|
| `start_in_aggressive_motion` | `true` | 使用预设重力方向，降低初始化飞走概率 |
| `gravity_init` | `[0.0, 0.0, -9.810]` | 预设重力 |
| `extrinsic_est_en` | `false` | aggressive motion 下关闭在线外参估计 |
| `acc_norm` | `10.2` | 期望 IMU 加速度模长 |
| `b_acc_cov` / `b_gyr_cov` | `0.0001` | 偏置协方差 |
| `imu_meas_acc_cov` | `0.1` | 加速度测量协方差 |
| `imu_meas_omg_cov` | `0.1` | 角速度测量协方差 |

不要仅靠 Point-LIO `extrinsic_R` 修正机械大倾角。实测中它虽能旋转点云，却会让 IMU 与重力对齐变得不稳定，运动时可能导致点云飞走。

`mapping_unilidar_l2.launch.py` 的建议基线：

| 参数 | NUC | Jetson |
|---|---|---|
| `point_filter_num` | 3 | 1 |
| `filter_size_surf` | 0.5 | 0.3 |
| `filter_size_map` | 0.5 | 0.3 |

按 CPU 能力调节，避免时间戳积压和 `the queue is full`。

### 10.4 Mid-360 配置细节

`MID360_config.json` 完整结构示例：

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

在 `msg_MID360_launch.py` 中推荐：

```python
xfer_format = 1  # Livox CustomMsg
```

CustomMsg 提供 `offset_time` 等时间同步字段。PointCloud2 模式即使能运行 FAST-LIO，也可能触发缺少 Livox 专用字段的提示。

### 10.5 倾斜安装方案

当前机械方案倾斜安装雷达以获得更完整视场。建议：

1. 使用 `body -> base_link` 的静态 pitch 旋转。
2. 建图后使用 `rotate_pcd.py` 修正保存的 PCD。
3. 不在 Point-LIO 中用大角度 `extrinsic_R` 代替上述两步。

```bash
ros2 run tf2_ros static_transform_publisher \
  0 0 0 0 -0.873 0 body base_link

python3 rotate_pcd.py
```

`-0.873 rad` 约为 `-50°`。安装角改变后必须重新测量，不能沿用固定值。

### 10.6 建图工作流

启用 FAST-LIO / Point-LIO 的 PCD 保存后：

```bash
ros2 launch fast_lio mapping.launch.py config_file:=mid360.yaml

cd ~/nav_ws/src/FAST_LIO/PCD
pcl_viewer scans.pcd
python3 rotate_pcd.py

ros2 launch pcd2pgm pcd2pgm_launch.py

cd ~/Desktop/map
ros2 run nav2_map_server map_saver_cli -f my_map
```

### 10.7 当前导航成果

- 5×5 m 室内起点到目标点导航。
- 绕过固定障碍并实时避开动态障碍。
- 巡航速度最高记录为 0.26 m/s。
- 终点误差 ≤ 0.2 m。
- 20 次测试成功率 ≥ 90%，连续 5 次无碰撞。

这些是阶段性测试结果，正式赛场仍需重新验证定位、代价地图与参数。

### 10.8 NYUSH 实机导航联调要点

通信打通不等于导航闭环。实机还要求环境与加载的二维地图一致，例如正式场地 `RMUL2026` 或自建 `11_map`。

必须稳定：

```text
map -> odom -> base_link
```

至少检查：

- `ros2 action list | grep navigate_to_pose` 能看到 action，否则 BT `SendGoal` 无法闭环。
- RViz 中机器人位姿与地图大致重合。
- 局部 costmap 没有持续异常膨胀。
- Home 与中心目标点位于占用栅格自由空间。
- 地图原点、方向和真实场地一致。

地图坐标不一致常表现为“能规划但车往错方向走”或“规划器拒绝目标”，不应立刻归因于 Nav2 本身。

先在台架与小场地验证定位和短距离导航，再进入完整场地测试去中心、守中和回家。没有匹配环境时，可以验证通信和 BT，但不能据此认定实机导航完整可用。

## 11. 后续计划

- [ ] Unitree L2 自动初始化与稳定性评估
- [ ] IMU 标定流程
- [ ] 动态避障优化
- [ ] ROS 电池监控
- [ ] 自动恢复行为
- [ ] 基于真实赛场数据重新建立性能基线
- [ ] 雷达站与哨兵传感器方案继续评估

## 12. 参考资料

### 官方文档

- [Livox Mid-360](https://www.livoxtech.com/mid-360/downloads)
- [FAST-LIO](https://github.com/hku-mars/FAST_LIO)
- [Nav2](https://docs.nav2.org/)
- [ROS 2 Humble](https://docs.ros.org/en/humble/)

### 相关项目

- [Livox ROS Driver 2](https://github.com/Livox-SDK/livox_ros_driver2)
- [Point-LIO](https://github.com/hku-mars/Point-LIO)
- [Point-LIO ROS 2（Unitree 适配）](https://github.com/dfloreaa/point_lio_ros2)
- [pcd2pgm](https://github.com/LihanChen2004/pcd2pgm)

### 常用工具

- `pcl_viewer`：点云查看。
- Foxglove：ROS 2 可视化。
- PlotJuggler：实时数据绘图。
- RViz2：地图、TF、LaserScan 与 Nav2 调试。

## 贡献与许可证

发现问题时请优先提交 GitHub issue；代码或文档改进可通过 pull request 贡献。

仓库根目录采用 [MIT License](LICENSE)。FAST-LIO、Nav2、Livox SDK、Unitree SDK 等第三方组件保留各自许可证，分发与部署时应分别遵守。
