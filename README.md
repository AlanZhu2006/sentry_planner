# RM2025 Auto Sentry — NYUSH Robotics

基于深圳北理莫斯科大学北极熊战队开源栈改造，面向 **NYUSH Robotics** 哨兵：激光雷达定位、Nav2、**BehaviorTree** 决策，以及与 **nyush-rm-control / nyush-rm-vision** 的 **bridge + 串口** 通讯。

**运行环境：** Ubuntu 22.04 · ROS 2 Humble ·（可选）Gazebo Classic 11  

---

## 文档怎么读（中央索引）

本文件只做**总览与入口**；**细节、协议、调参、排障、实机联调**在下面四份专题里展开（**五文分工表在每份专题文首 §0 一致列出**，便于交叉引用）。

| 文档 | 职责（详细说明在对应文件内） |
|------|------------------------------|
| [**README_COMMUNICATION.md**](README_COMMUNICATION.md) | **通讯与协议全文**：单端口、`sentry_bridge`、SP/SX/ST、帧、`serial_sender`、`bt_comm_adapter`、话题；**§15.3** 实机通讯摘要 |
| [**README_BEHAVIOR_TREE_FLOW.md**](README_BEHAVIOR_TREE_FLOW.md) | **行为树全文**：默认 `center_attack_simple`、XML/节点/插件、`RobotControl` 与 Nav2 衔接、Groot2、调试脚本；**§5.1** 目标点与 watcher 对齐 |
| [**README_LIDAR.md**](README_LIDAR.md) | **激光雷达 + SLAM + Nav2**：**§6.4 Gazebo RMUL2026（Sim2Real 推荐）**、Mid360、FAST-LIO、参数、建图、TF、排障；**§10.8** 实机定位前提 |
| [**README_COMMANDS.md**](README_COMMANDS.md) | **命令与数据流全书**：**§1** 四路径、[§4](README_COMMANDS.md#4-nav_wsstart_robotsh调试用环境变量详解) 环境变量、**§4.4** 地图、**§7** 建图、[§12 实机](README_COMMANDS.md#12-实机联调)、[**§13** 该读哪份专题](README_COMMANDS.md#13-四专题分工速查我该打开哪份-readme) |

**其它常用附录：**

| 文件 | 用途 |
|------|------|
| [communication command.txt](communication%20command.txt) | 命令片段备忘（可复制）；**原理与参数说明以 README_COMMANDS.md 为准** |
| [mid360 command.txt](mid360%20command.txt) | **Gazebo + Nav2 + BT** 逐步命令（去中心/守中/回家）、Groot2 AppImage；**原理见 README_LIDAR §6.4** |

---

## 一条线看懂架构

```text
        Mid360 ──► livox_ros_driver2 ──► FAST-LIO ──► /cloud_registered、里程计
                        │
                        └──► pointcloud_to_laserscan ──► /scan ──► Nav2
                                      │
Nav2 /cmd_vel ──► fake_vel_transform ──► /cmd_vel_chassis ──► bt_comm_adapter ──► /cmd_vel_chassis_bt
                                                                              │
行为树 /goal_pose ──► Nav2          行为树 /robot_control ──► serial_sender ────┤
                                                                              ▼
视觉 ◄──► Vision PTY ◄──► sentry_bridge ◄──► /dev/ttyACM0 ◄──► nyush-rm-control (MCU)
雷达侧 ◄──► Radar PTY ◄──┘
```

- **真实 USB 串口只给 `sentry_bridge`**；视觉与导航各连桥接出来的 **PTY**（详见通讯文档）。  
- **平面旋转**：当前由 **`/robot_control.chassis_spin_vel`** 经 `bt_comm_adapter` 写入 `/cmd_vel_chassis_bt.angular.z`，**不**使用 Nav2 输出的 `angular.z`（实现见 `scripts/bt_comm_adapter.py`）。  
- **端到端数据链与职责表**（四条主路径 A–D、SX 与 SP 分工）：[README_COMMANDS.md §1](README_COMMANDS.md#1-完整数据链路与职责划分)。

---

## 代码仓库角色

| 路径 / 仓库 | 角色 |
|-------------|------|
| **本仓库 `sentry_planner`** | `start_robot.sh`、`rm_navigation_ws`、`rm_decision_ws`、`scripts/bt_comm_adapter.py` 等 |
| **`~/nav_ws`**（或你本机的 nav 工作空间） | Livox 驱动、FAST-LIO、`my_nav2_params.yaml` 等；`start_robot.sh` 会 source |
| **`nyush-rm-control`** | STM32 固件、`just sentry-bridge` |
| **`nyush-rm-vision`** | 视觉、`serial_sender.py --ros2` |

---

## 编译顺序（摘要）

完整命令与排障见各专题文档；顺序一般为：

1. `nav_ws`（含 `livox_ros_driver2`）  
2. `sentry_planner/rm_vision_ws`  
3. `sentry_planner/rm_decision_ws`（常需先 source `rm_vision_ws`）  
4. `sentry_planner/rm_navigation_ws`（**必须先 source `nav_ws`** 再 `colcon build`）

---

## 实机最短启动（摘要）

1. **下位机桥接**（占 USB CDC，常见为 `/dev/ttyACM0`）：`nyush-rm-control` 里 **`just sentry-bridge`**（自动选口）或显式 `--port`，记下 **Radar / Vision PTY**（优先用稳定软链，见通讯文档）。  
2. **本仓库**：`START_SERIAL_SENDER=1 RADAR_PTY=<Radar_PTY> ./start_robot.sh`（**`start_robot.sh` 不会自动起 bridge**；细节见 [README_COMMANDS.md §4](README_COMMANDS.md#4-nav_wsstart_robotsh调试用环境变量详解)）。  
3. **视觉**（若需要）：`sentry.yaml` 里 **`com_port` 指向 Vision PTY**，再按 `nyush-rm-vision` 流程启动。

**联调顺序**：**推荐 Sim2Real**——先在 **Gazebo RMUL2026 + Nav2** 调参数与 `center_attack_simple`（**[README_LIDAR.md §6.4](README_LIDAR.md#nyush-gazebo-sim2real)**，逐步命令 **[mid360 command.txt](mid360%20command.txt)**），再上实车。实机不必一上来就占完整赛场：**台架通通讯与 BT → 小场地低速定位与回家 → 全场**。步骤与检查清单见 **[README_COMMANDS.md §12](README_COMMANDS.md#12-实机联调)**。

**不要用** `scripts/start_fullstack_sequence.sh` 除非你希望 **gnome-terminal 自动开很多窗口**；日常推荐多终端手动起（见 `README_COMMUNICATION.md`）。

---

## 变更与参考

NYUSH 相对原栈的协议与底盘改动（19 字节雷达帧、`ref_yaw`、哨兵舵轮等）在 **README_COMMUNICATION.md** 与下位机仓库中为准；北极熊原始栈见 Gitee SMBU-POLARBEAR。

---

*中央 README 仅维护索引与共识；细节以四份专题 README 为准。*
