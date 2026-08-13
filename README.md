<div align="center">

# NYUSH RM Sentry

**面向 RoboMaster 哨兵机器人的自主导航、行为树决策与跨系统通信栈**

![Ubuntu 22.04](https://img.shields.io/badge/Ubuntu-22.04-E95420?logo=ubuntu&logoColor=white)
![ROS 2 Humble](https://img.shields.io/badge/ROS%202-Humble-22314E?logo=ros&logoColor=white)
[![License: MIT](https://img.shields.io/badge/License-MIT-2ea44f.svg)](LICENSE)

[![简体中文](https://img.shields.io/badge/%E7%AE%80%E4%BD%93%E4%B8%AD%E6%96%87-%E5%BD%93%E5%89%8D-0969da?style=for-the-badge)](README.md)
[![English](https://img.shields.io/badge/English-Switch-2ea44f?style=for-the-badge)](README_EN.md)

</div>

## 项目概览

`nyush_rm_sentry`（原 `sentry_planner`）是 **NYUSH Robotics** 的哨兵上位机规划与系统集成仓库。项目基于深圳北理莫斯科大学北极熊战队开源栈改造，将激光雷达定位、Nav2 导航、BehaviorTree.CPP 决策，以及 `nyush-rm-control` / `nyush-rm-vision` 的桥接与串口通信整合为一套完整工作流。

### 核心能力

| 模块 | 能力 |
|---|---|
| 定位与感知 | Livox Mid-360、FAST-LIO、点云转 LaserScan、地图与 TF 管理 |
| 自主导航 | Nav2 路径规划、速度指令转换、目标点与返航流程 |
| 战术决策 | BehaviorTree.CPP、可视化 Groot2 工程、可切换战术树 |
| 系统通信 | 单 USB CDC 桥接、Radar / Vision PTY、SP 与 SX/ST 协议链路 |
| 仿真验证 | Gazebo Classic 11、RMUL2026 场景、Sim2Real 调试流程 |

## 系统架构

```text
        Mid-360 ──► livox_ros_driver2 ──► FAST-LIO ──► 点云与里程计
                         │
                         └──► pointcloud_to_laserscan ──► /scan ──► Nav2
                                       │
Nav2 /cmd_vel ──► fake_vel_transform ──► /cmd_vel_chassis ──► bt_comm_adapter
                                                                            │
行为树 /goal_pose ──► Nav2       行为树 /robot_control ──► serial_sender ────┤
                                                                            ▼
视觉 ◄──► Vision PTY ◄──► sentry_bridge ◄──► /dev/ttyACM0 ◄──► 控制板 MCU
雷达/导航 ◄──► Radar PTY ◄───────┘
```

> [!IMPORTANT]
> 真实 USB 串口只能由 `sentry_bridge` 占用。视觉与雷达/导航进程应分别连接桥接程序创建的 Vision PTY 和 Radar PTY。

当前平面旋转由 `/robot_control.chassis_spin_vel` 经 `bt_comm_adapter` 写入 `/cmd_vel_chassis_bt.angular.z`，不使用 Nav2 输出的 `angular.z`。完整数据链与模块职责见 [README_COMMANDS.md §1](README_COMMANDS.md#1-完整数据链路与职责划分)。

## 文档导航

根 README 只提供项目总览和最短上手路径；协议、调参和排障细节集中在以下专题文档中。

| 文档 | 内容 |
|---|---|
| [README_COMMUNICATION.md](README_COMMUNICATION.md) | 通信架构、`sentry_bridge`、PTY、SP/SX/ST、串口帧、ROS 2 话题与实机通信摘要 |
| [README_BEHAVIOR_TREE_FLOW.md](README_BEHAVIOR_TREE_FLOW.md) | 战术树、节点与插件、`RobotControl`、Nav2 衔接、Groot2 与调试脚本 |
| [README_LIDAR.md](README_LIDAR.md) | Mid-360、FAST-LIO、SLAM、Nav2、Gazebo RMUL2026、参数与定位排障 |
| [README_COMMANDS.md](README_COMMANDS.md) | 完整数据流、启动命令、环境变量、地图、建图与分阶段实机联调 |

常用命令备忘：

- [communication command.txt](communication%20command.txt)：通信相关命令片段；原理与参数以 `README_COMMANDS.md` 为准。
- [mid360 command.txt](mid360%20command.txt)：Gazebo、Nav2、行为树与 Groot2 的逐步命令。

## 仓库与工作空间

| 路径 / 仓库 | 职责 |
|---|---|
| **本仓库 `nyush_rm_sentry`** | 启动编排、导航工作空间、行为树工作空间、通信适配脚本与调试工具 |
| **`~/nav_ws`** | Livox 驱动、FAST-LIO、Nav2 参数等外部导航依赖；由 `start_robot.sh` source |
| [**nyush-rm-control**](https://github.com/NYUSH-Robotics-Club/nyush-rm-control)（团队仓库，需权限） | STM32 固件、底盘与云台控制、`sentry_bridge` |
| [**nyush-rm-vision**](https://github.com/NYUSH-Robotics-Club/nyush-rm-vision)（团队仓库，需权限） | 装甲板识别、跟踪、自瞄与视觉侧串口链路 |

## 环境要求

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Classic 11（仅仿真需要）
- Livox Mid-360 与对应驱动（实机定位需要）

## 编译顺序

工作空间之间存在依赖，推荐按以下顺序编译：

1. `nav_ws`（含 `livox_ros_driver2`）
2. `nyush_rm_sentry/rm_vision_ws`
3. `nyush_rm_sentry/rm_decision_ws`（先 source `rm_vision_ws`）
4. `nyush_rm_sentry/rm_navigation_ws`（先 source `nav_ws`）

具体命令、依赖与常见错误见 [README_COMMANDS.md](README_COMMANDS.md) 和各工作空间 README。

## 实机快速启动

1. 在 `nyush-rm-control` 中启动下位机桥接；它会独占 USB CDC，并输出 Radar / Vision PTY：

   ```bash
   just sentry-bridge
   ```

2. 在本仓库启动导航、行为树和雷达侧串口发送：

   ```bash
   START_SERIAL_SENDER=1 RADAR_PTY=<Radar_PTY> ./start_robot.sh
   ```

3. 如需视觉，在 `nyush-rm-vision` 的 `sentry.yaml` 中将 `com_port` 指向 Vision PTY，再按视觉仓库流程启动。

`start_robot.sh` 不会自动启动 `sentry_bridge`。端口选择、环境变量和完整检查清单见 [README_COMMANDS.md §4](README_COMMANDS.md#4-nav_wsstart_robotsh调试用环境变量详解) 与 [§12](README_COMMANDS.md#12-实机联调)。

## 推荐验证流程

建议按 **Gazebo RMUL2026 + Nav2 参数验证 → 台架通信与行为树 → 小场地低速定位和返航 → 全场联调** 的顺序推进。仿真入口见 [README_LIDAR.md §6.4](README_LIDAR.md#nyush-gazebo-sim2real)，逐步命令见 [mid360 command.txt](mid360%20command.txt)。

## 致谢与许可证

本项目基于 [SMBU-POLARBEAR/RM2024_SMBU_auto_sentry_ws](https://gitee.com/SMBU-POLARBEAR/RM2024_SMBU_auto_sentry_ws) 及其相关开源组件进行适配和扩展，感谢原作者与社区贡献者。

项目采用 [MIT License](LICENSE)。NYUSH 特有的协议与底盘改动（包括 19 字节雷达帧、`ref_yaw` 与哨兵舵轮适配）以 `README_COMMUNICATION.md` 和下位机仓库实现为准。

---

<div align="center">

根 README 维护稳定总览；实现细节以专题文档与当前代码为准。

</div>
