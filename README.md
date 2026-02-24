# RM2025 Auto Sentry - NYUSH Robotics

基于深圳北理莫斯科大学北极熊战队开源代码修改，适配 NYUSH Robotics 哨兵机器人

**环境要求:** Ubuntu 22.04 + ROS2 Humble + Gazebo Classic 11

---

## 目录

| 章节 | 内容 |
|------|------|
| [〇](#〇nyush-适配与变更记录--change-log) | NYUSH 适配与变更记录 |
| [一](#一系统架构总览) | 系统架构总览 |
| [二](#二项目结构) | 项目结构 |
| [三](#三行为树决策详解) | 行为树决策详解 |
| [四](#四上下位机通信协议) | 上下位机通信协议 |
| [五](#五快速开始) | 快速开始 (含 build 顺序、start_robot、无雷达测试) |
| [五.7](#57-决策系统调试指南) | 决策系统调试指南 (Todo + 必修问题) |
| [五.9](#59-navigation-仿真进度与配置) | Navigation 仿真进度与配置 (2025-02) |
| [五.10](#510-下一步-todo) | 下一步 TODO |
| [六](#六实车部署) | 实车部署 |
| [七](#七常见问题) | 常见问题 |
| [八](#八参考资料) | 参考资料 |

---

## 〇、NYUSH 适配与变更记录 / Change Log

> 本节记录 NYUSH Robotics 对原始 SMBU 框架所做的全部修改及当前架构。

### 0.1 固件层 (nyush-rm-control) 变更

| 模块 | 变更内容 |
|------|----------|
| **radar_comm** | 雷达协议从 15 字节扩展为 **19 字节**：`[0xA5][0x5A][vx:4B][vy:4B][wz:4B][yaw_deg:4B][CRC8:1B]`，新增 `yaw_deg` 用于雷达 IMU 固定正方向 |
| **Chassis_Ctrl_Cmd_s** | 新增 `ref_yaw_deg`、`ref_yaw_valid` 字段，雷达有效时传递世界系下底盘 yaw |
| **robot_cmd.c** | 雷达分支解析 `radar_data->yaw_deg` 并写入 `chassis_cmd_send.ref_yaw_deg` |
| **sentry_controller.c** | 当 `ref_yaw_valid` 时，使用 `-ref_yaw_deg` 做世界→底盘速度变换，替代 `offset_angle` |

### 0.2 上位机脚本变更

| 文件 | 用途 |
|------|------|
| `nyush-rm-control/scripts/cmd_vel_keyboard_fixed.py` | 键盘控制 + 19 字节雷达协议，`(vx,vy)` 逆时针旋转 90° 以适配雷达系，`yaw_deg=0`（键盘模式无 IMU） |
| `sentry_planner/scripts/fake_sensors_for_test.py` | 发布假 `/scan`、`/odom`、`map→odom→base_link` TF，用于无雷达测试 Nav2 + 决策 |
| `sentry_planner/scripts/test_no_lidar.sh` | 无雷达测试流程说明脚本 |

### 0.3 sentry_planner 变更

| 项目 | 变更内容 |
|------|----------|
| **start_robot.sh** | 在 Nav2 之后增加决策行为树启动，依次 source `nav_ws` → `rm_vision_ws` → `rm_decision_ws`，`style:=retreat_attack_left` |
| **rm_navigation_ws install/setup.bash** | 移除对 `rm_decision_ws`、`rm_vision_ws` 的链式依赖，避免 "not found" 报错 |
| **rm_decision_ws** | 修改 `is_detect_enemy.hpp` 去掉 `armors__struct.hpp` 依赖；CMakeLists 增加 `auto_aim_interfaces` 依赖 |
| **rm_navigation_ws 构建** | 需先 `source ~/nav_ws/install/setup.bash` 再 `colcon build`，否则 `ros2_livox_simulation` 找不到 `livox_ros_driver2` |
| **rm_nav_bringup 依赖** | 需构建 `rm_navigation`、`fake_vel_transform`，否则 launch 报 `PackageNotFoundError` |
| **nav2_params_sim.yaml** | 对齐 my_nav2_params：min_y_velocity_threshold 0.001、AMCL 粒子数、costmap 参数；保留旋转 (max_vel_theta) |
| **仿真轻量化** | Gazebo 100Hz、Livox 点云降密度、Fast-LIO/segmentation 参数调低；新增 headless 模式 |

---

## 一、系统架构总览

### 1.1 整体数据流

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              感知层 (Perception)                             │
├─────────────────────────────────┬───────────────────────────────────────────┤
│     Mid360 激光雷达              │         HIK相机 + 装甲板检测               │
│  ┌──────────────────────┐       │      ┌──────────────────────┐             │
│  │ 点云数据 /livox/lidar │       │      │ /detector/armors     │             │
│  └──────────┬───────────┘       │      │ (检测到的装甲板信息)  │             │
│             │                   │      └──────────┬───────────┘             │
│             ▼                   │                 │                          │
│  ┌──────────────────────┐       │                 │                          │
│  │ FAST_LIO (定位建图)   │       │                 │                          │
│  │ 输出: /Odometry      │       │                 │                          │
│  └──────────┬───────────┘       │                 │                          │
└─────────────┼───────────────────┴─────────────────┼──────────────────────────┘
              │                                     │
              ▼                                     ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                              决策层 (Decision)                               │
│  ┌──────────────────────────────────────────────────────────────────────┐   │
│  │                      Behavior Tree 行为树                             │   │
│  │  订阅: /all_robot_hp, /robot_status, /game_status, /detector/armors  │   │
│  │  发布: /goal_pose (导航目标), /robot_control (云台/底盘控制)          │   │
│  └──────────────────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────────────┘
              │                                     │
              ▼                                     ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                              执行层 (Execution)                              │
│  ┌──────────────────────────────────────────────────────────────────────┐   │
│  │                        Navigation2 (Nav2)                            │   │
│  │  输入: /goal_pose, /Odometry, /scan                                  │   │
│  │  处理: 全局路径规划 → 局部路径规划 → 动态避障                           │   │
│  │  输出: /cmd_vel ──► serial_sender / rm_serial_driver ──► C板执行       │   │
│  └──────────────────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 1.2 实车完整数据流

```
LiDAR → livox_ros_driver2 → Fast-LIO → /cloud_registered, /Odometry
                              │
                              ▼
         pointcloud_to_laserscan → /scan ──┐
                                           │
         TF: odom→body→base_link ──────────┼──► Nav2 ──► /cmd_vel
                                           │         │
         /goal_pose (来自决策树) ────────────┘         │
                                                      ▼
         serial_sender.py / rm_serial_driver ──► USB ──► C板 radar_comm
                                                      │
                                                      ▼
         STM32 radar_comm ──► robot_cmd ──► chassis_cmd ──► sentry_controller
         (19字节: vx,vy,wz,yaw_deg)         (ref_yaw)      (舵轮底盘控制)
```

### 1.3 工作空间与依赖关系

```
                    /opt/ros/humble
                           │
        ┌──────────────────┼──────────────────┐
        ▼                  ▼                  ▼
   ~/nav_ws           rm_vision_ws        rm_decision_ws
   (livox_ros_driver2,  (auto_aim_        (rm_behavior_tree,
    Fast-LIO, Nav2...)   interfaces)        rm_decision_interfaces)
        │                  │                  │
        └──────────────────┴────────┬──────────┘
                                   ▼
                           rm_navigation_ws
                    (需先 source nav_ws 再 build)
                    (Gazebo 仿真、rm_nav_bringup)
```

**构建顺序建议：** ROS → nav_ws → rm_vision_ws → rm_decision_ws → rm_navigation_ws

---

## 二、项目结构

### 2.1 NYUSH 完整目录结构

```
sentry_planner/                    # 哨兵决策与仿真主目录
├── start_robot.sh                 # 一键启动 (雷达+Fast-LIO+Nav2+决策)
├── README.md
├── scripts/
│   ├── fake_sensors_for_test.py   # 无雷达假传感器 (/scan, /odom, TF)
│   └── test_no_lidar.sh           # 无雷达测试流程
│
├── rm_decision_ws/                # 决策模块 (行为树)
│   └── src/
│       ├── rm_behavior_tree/       # 行为树实现
│       │   ├── config/            # 行为树 XML 配置
│       │   │   ├── attack_left.xml
│       │   │   ├── retreat_attack_left.xml
│       │   │   └── ...
│       │   ├── plugins/            # 行为树节点插件
│       │   └── src/
│       └── rm_decision_interfaces/ # 消息定义
│
├── rm_vision_ws/                  # 视觉模块
│   └── src/
│       ├── auto_aim_interfaces/   # 装甲板检测消息 (决策依赖)
│       ├── rm_serial_driver/      # 串口通信 (与 C 板)
│       └── ros2_hik_camera/       # 海康相机
│
├── rm_navigation_ws/              # 导航与仿真
│   └── src/
│       ├── rm_nav_bringup/        # 仿真/实车启动配置
│       ├── rm_navigation/         # Nav2 + TEB (需单独 build)
│       │   ├── rm_navigation/      # launch、params
│       │   └── fake_vel_transform/ # 云台旋转时速度变换
│       ├── rm_localization/        # ICP 等
│       ├── rm_perception/         # 点云处理
│       └── rm_simulation/          # Gazebo + ros2_livox_simulation
│
~/nav_ws/                          # 实车雷达栈 (独立工作空间)
├── src/
│   ├── livox_ros_driver2/         # Mid-360 驱动 (rm_navigation_ws 构建依赖)
│   ├── FAST_LIO/
│   ├── pointcloud_to_laserscan/
│   └── ...
├── my_nav2_params.yaml
├── start_robot.sh                 # 或使用 sentry_planner/start_robot.sh
└── serial_sender.py               # cmd_vel → 串口 (可选)

nyush-rm-control/                  # STM32 固件 (C 板)
├── application/
│   ├── chassis/sentry_controller.c # 舵轮底盘，使用 ref_yaw 做速度变换
│   ├── cmd/robot_cmd.c            # 雷达分支传递 ref_yaw
│   └── robot_def.h                # Chassis_Ctrl_Cmd_s 定义
├── modules/
│   └── radar_comm/                # 19 字节雷达协议解析
└── scripts/
    └── cmd_vel_keyboard_fixed.py  # 键盘控制 + 19 字节协议
```

---

## 三、行为树决策详解

### 3.1 行为树节点类型

| 类型 | 节点名 | 功能 | 输入/输出 |
|------|--------|------|-----------|
| **Subscriber** | `SubAllRobotHP` | 订阅所有机器人血量 | → `{robot_hp}` |
| | `SubRobotStatus` | 订阅本机状态 | → `{robot_status}` |
| | `SubGameStatus` | 订阅比赛状态 | → `{game_status}` |
| | `SubArmors` | 订阅装甲板检测结果 | → `{armors}` |
| **Condition** | `IsGameTime` | 判断比赛阶段和剩余时间 | `game_progress`, `lower/higher_remain_time` |
| | `IsStatusOK` | 判断血量/热量是否健康 | `hp_threshold`, `heat_threshold` |
| | `IsDetectEnemy` | 是否检测到敌人装甲板 | `{armors}` |
| | `IsAttacked` | 是否正在被攻击 | `{robot_status}.is_attacked` |
| | `IsFriendOK` | 友方机器人是否存活 | `{robot_hp}`, `friend_color` |
| | `IsOutpostOK` | 前哨站是否存活 | `{robot_hp}` |
| **Action** | `SendGoal` | 发送导航目标点到Nav2 | `goal_pose="x;y;z; qx;qy;qz;qw"` |
| | `RobotControl` | 控制云台扫描/底盘旋转 | `stop_gimbal_scan`, `chassis_spin_vel` |
| | `MoveAround` | 躲避动作(受攻击时) | `expected_nearby_goal_count`, `expected_dis` |
| | `GetCurrentLocation` | 获取当前位置 | → `{current_location}` |
| **Decorator** | `RateController` | 限制执行频率 | `hz` |

### 3.2 决策逻辑示例 (attack_left.xml)

```
比赛进行中 (game_progress=4)?
├─ 是 → 
│   ├─ 检测到敌人?
│   │   ├─ 是 → RobotControl(stop_gimbal_scan=True, chassis_spin_vel=0.5)
│   │   └─ 否 → RobotControl(stop_gimbal_scan=False, chassis_spin_vel=0.5)
│   │
│   ├─ 检测到敌人 AND 状态OK (HP>400, Heat<350)?
│   │   ├─ 被攻击中? → 躲避动作 (MoveAround)
│   │   └─ 血量<250? → 撤退到补给区 (-2.5, 4.07)
│   │
│   ├─ 状态OK?
│   │   ├─ 时间在 3:20-4:05? → 占领中心 (3.0, 0.4)
│   │   ├─ 友方OK? → 进攻左路 (5.1, 1.9)
│   │   └─ 友方不OK? → 守中心 (3, 1)
│   │
│   └─ 状态不OK → 撤退到补给区
│
└─ 否 → 待机 (等待比赛开始)
```

### 3.3 自定义行为树

编辑 `rm_decision_ws/rm_behavior_tree/config/` 下的 XML 文件:

```xml
<!-- 示例: 简单巡逻 -->
<BehaviorTree ID="SimplePatrol">
  <ReactiveSequence>
    <SubGameStatus topic_name="game_status" game_status="{game_status}"/>
    <WhileDoElse>
      <IsGameTime message="{game_status}" game_progress="4" 
                  lower_remain_time="0" higher_remain_time="300"/>
      <!-- 比赛中: 循环巡逻 -->
      <Sequence>
        <SendGoal goal_pose="1;1;0; 0;0;0;1" action_name="navigate_to_pose"/>
        <SendGoal goal_pose="5;1;0; 0;0;0;1" action_name="navigate_to_pose"/>
        <SendGoal goal_pose="5;5;0; 0;0;0;1" action_name="navigate_to_pose"/>
        <SendGoal goal_pose="1;5;0; 0;0;0;1" action_name="navigate_to_pose"/>
      </Sequence>
      <!-- 比赛未开始: 待机 -->
      <AlwaysSuccess/>
    </WhileDoElse>
  </ReactiveSequence>
</BehaviorTree>
```

---

## 四、上下位机通信协议

### 4.1 通信架构

```
┌─────────────────┐    USB/UART    ┌─────────────────┐
│   上位机 (NUC)  │ ◄────────────► │   C板 (STM32)   │
│ serial_sender / │   115200 baud  │  radar_comm +   │
│ rm_serial_driver│                │  master_process │
└─────────────────┘                └─────────────────┘
```

### 4.2 导航速度协议 (上位机 → C板, 19 字节)

**NYUSH 扩展协议：** 在原有 15 字节基础上增加 `yaw_deg` 字段，用于雷达 IMU 固定正方向。

| 偏移 | 长度 | 字段 | 说明 |
|------|------|------|------|
| 0 | 2B | `0xA5 0x5A` | 帧头 |
| 2 | 4B | `vx` | 线速度 x (m/s), float32 小端 |
| 6 | 4B | `vy` | 线速度 y (m/s) |
| 10 | 4B | `wz` | 角速度 (rad/s) |
| 14 | 4B | `yaw_deg` | 底盘在雷达/世界系下的 yaw（度），键盘模式传 0 |
| 18 | 1B | CRC8 | 多项式 0x07，对前 18 字节计算 |

**键盘控制脚本：** `nyush-rm-control/scripts/cmd_vel_keyboard_fixed.py` 会做 90° 旋转 `(vx,vy)→(-vy,vx)` 以适配雷达坐标系，`yaw_deg=0`。

### 4.4 C板 → 上位机 (裁判系统数据上传)

| 包类型 | Header | 数据内容 | 校验 | 总长度 |
|--------|--------|----------|------|--------|
| 所有机器人血量 | `0x5B` | 16×uint16 | CRC16 | 35B |
| 比赛状态 | `0x5C` | u8 + u16 | CRC16 | 6B |
| 机器人状态 | `0x5D` | u8+u16+u16+u8+u8 | CRC16 | 10B |

**AllRobotHP (0x5B):**
```
[0x5B][red1_hp:2B][red2_hp:2B]...[blue_base_hp:2B][CRC16:2B]
```

**GameStatus (0x5C):**
```
[0x5C][game_progress:1B][stage_remain_time:2B][CRC16:2B]
game_progress: 0=未开始, 1=准备, 2=自检, 3=倒计时, 4=比赛中, 5=结算
```

**RobotStatus (0x5D):**
```
[0x5D][robot_id:1B][current_hp:2B][shooter_heat:2B][team_color:1B][is_attacked:1B][CRC16:2B]
```

#### 上位机 → C板 (控制命令下发)

| 包类型 | Header | 数据内容 | 校验 | 总长度 |
|--------|--------|----------|------|--------|
| 导航速度 (NYUSH) | `0xA5 0x5A` | vx + vy + wz + yaw_deg (4×float) | CRC8 | **19B** |
| 机器人控制 | `0xA3` | stop_scan + spin_vel | CRC16 | 8B |

**NavCmd (0xA5 0x5A) - 19 字节：**
```
[0xA5][0x5A][vx:4B][vy:4B][wz:4B][yaw_deg:4B][CRC8:1B]
vx, vy: m/s (底盘线速度，雷达系)
wz: rad/s (底盘角速度)
yaw_deg: 底盘在雷达/世界系下的 yaw（度），雷达 IMU 时有效
CRC8 polynomial: 0x07
```

**RobotControl (0xA3):**
```
[0xA3][stop_gimbal_scan:1B][chassis_spin_vel:4B][CRC16:2B]
stop_gimbal_scan: 0=继续扫描, 1=停止扫描
chassis_spin_vel: 底盘自旋速度 (rad/s)
```

### 4.5 ROS2 Topic 对应关系

| C板数据 | ROS2 Topic | 消息类型 |
|---------|------------|----------|
| 所有机器人血量 | `/all_robot_hp` | `rm_decision_interfaces/msg/AllRobotHP` |
| 比赛状态 | `/game_status` | `rm_decision_interfaces/msg/GameStatus` |
| 机器人状态 | `/robot_status` | `rm_decision_interfaces/msg/RobotStatus` |
| 导航速度命令 | `/cmd_vel_chassis` | `geometry_msgs/msg/Twist` |
| 机器人控制 | `/robot_control` | `rm_decision_interfaces/msg/RobotControl` |

---

## 五、快速开始

### 5.1 安装依赖

```bash
# 基础依赖
sudo apt update
sudo apt install -y ros-humble-gazebo-ros-pkgs \
                    ros-humble-navigation2 \
                    ros-humble-nav2-bringup \
                    ros-humble-tf2-tools \
                    ros-humble-robot-state-publisher \
                    ros-humble-xacro

# Livox SDK2
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd Livox-SDK2 && mkdir build && cd build
cmake .. && make -j$(nproc)
sudo make install
```

### 5.2 编译项目 (重要：顺序与 source 依赖)

**⚠️ rm_navigation_ws 必须在其依赖已 source 的环境下构建：** `livox_ros_driver2` 在 `nav_ws` 中，需先 source `nav_ws` 再 build，否则 `ros2_livox_simulation` 报 `PackageNotFoundError`。

```bash
# 1. 编译 nav_ws (含 livox_ros_driver2, Fast-LIO 等)
cd ~/nav_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

# 2. 编译 rm_vision_ws
cd ~/sentry_planner/rm_vision_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash

# 3. 编译 rm_decision_ws
cd ~/sentry_planner/rm_decision_ws
source /opt/ros/humble/setup.bash
source ~/sentry_planner/rm_vision_ws/install/setup.bash
colcon build --symlink-install
source install/setup.bash

# 4. 编译 rm_navigation_ws (关键：先 source nav_ws)
cd ~/sentry_planner/rm_navigation_ws
source /opt/ros/humble/setup.bash
source ~/nav_ws/install/setup.bash          # 提供 livox_ros_driver2
colcon build --symlink-install
source install/setup.bash
```

**若 build 时出现 `package 'rm_navigation' not found`：** 需构建 `rm_navigation` 和 `fake_vel_transform`，使用 `colcon build --symlink-install` 全量构建即可。

### 5.3 实车一键启动 (start_robot.sh)

`sentry_planner/start_robot.sh` 自动完成：清理环境 → 雷达驱动 → TF → Fast-LIO → pointcloud_to_laserscan → Nav2 → 决策行为树 → RViz。

```bash
cd ~/sentry_planner
./start_robot.sh
```

**脚本内 source 顺序：** `nav_ws` → (决策启动时) `rm_vision_ws` → `rm_decision_ws`  
**地图配置：** 实机用 `nav_ws/start_robot.sh` 时地图在脚本内配置；仿真用 RMUL；无雷达测试用 `run_test_a` 时固定 RMUL。

### 5.5 运行仿真

**导航模式**（推荐，使用已有 RMUL 地图 + AMCL 定位）：

```bash
# 终端1: 启动仿真 (需先 source nav_ws 再 source rm_navigation_ws)
cd ~/sentry_planner/rm_navigation_ws
source ~/nav_ws/install/setup.bash
source install/setup.bash
ros2 launch rm_nav_bringup bringup_sim.launch.py \
  world:=RMUL mode:=nav localization:=amcl lio:=fastlio nav_rviz:=True
```

启动后在 RViz 中点击 **2D Pose Estimate** 设置初始位姿，再通过 **Nav2 Goal** 发送目标。卡顿时可加 `headless:=True`。

**建图模式**（需先建图再切 nav）：

```bash
ros2 launch rm_nav_bringup bringup_sim.launch.py \
  world:=RMUL mode:=mapping lio:=fastlio nav_rviz:=True
```

**配合决策树完整联调**：

```bash
# 终端2: 决策树
cd ~/sentry_planner/rm_decision_ws
source ~/sentry_planner/rm_vision_ws/install/setup.bash
source install/setup.bash
ros2 launch rm_behavior_tree rm_behavior_tree.launch.py \
  style:=retreat_attack_left use_sim_time:=True

# 终端3: 模拟裁判系统
ros2 topic pub /game_status rm_decision_interfaces/msg/GameStatus \
  "{game_progress: 4, stage_remain_time: 180}"
```

详细配置与排障见 [5.9 Navigation 仿真进度与配置](#59-navigation-仿真进度与配置)。

### 5.6 无雷达测试 (假传感器)

无实车雷达时，可用假传感器测试 Nav2 + 决策。**推荐：无需仿真、无需实机。**

```bash
# 一键启动 (假传感器 + Nav2 + 决策 + game_status)，固定使用 RMUL 地图
cd ~/sentry_planner
unset MAP_YAML    # 避免环境变量覆盖为 11_map
./scripts/run_test_a_headless.sh   # 无 RViz
# 或
./scripts/run_test_a.sh            # 含 RViz
```

手动分步启动见 `scripts/test_no_lidar.sh`。

### 5.7 决策系统调试指南

**不一定要跑仿真。** 决策树可在以下三种环境中测试，任选其一：

| 方案 | 说明 | 依赖 |
|------|------|------|
| **A. 无雷达测试 (run_test_a)** | 假传感器 + Nav2 + 决策树 | 不需要仿真、不需要实机 |
| **B. 仿真 (bringup_sim)** | Gazebo + 仿真机器人 + Nav2 + 决策树 | 需 GPU |
| **C. 实机** | 真雷达 + FAST-LIO + Nav2 + 决策树 | 需实机 + 串口 |

#### 决策调好 Todo List

**1. Nav2 正常启动**
- [ ] map_server 激活成功（无 `Failed to change state for node: map_server`）
- [ ] 地图使用 **RMUL**（非 11_map）
- [ ] `/map`、`/scan` 话题有数据

**2. 决策树行为树配置**
- [ ] **goal_pose 格式**：错误 `Can't convert string to PoseStamped` → 检查 XML/YAML 中 `goal_pose` 格式，需符合 `geometry_msgs/PoseStamped`
- [ ] **Missing required input [message]**：检查 BT 节点端口定义

**3. 决策与 Nav2 衔接**
- [ ] `navigate_to_pose` 可用（无 `Action server is not reachable`）
- [ ] 决策树需在 Nav2 完全启动后再启，或脚本中增加等待逻辑

**4. 适配自定义配置**
- [ ] 决策树中的目标点改为 RMUL 地图坐标
- [ ] 实机时确认 `/cmd_vel` → 串口转发正确
- [ ] 若用裁判系统，确认 `robot_hp` / `robot_status` / `game_status` 话题正确

#### 推荐顺序

1. 先用 **run_test_a** 跑通（无仿真、无实机）
2. 修掉 map_server、goal_pose、Missing input 等问题
3. 确认决策树能正常调用 `navigate_to_pose`
4. 再切到仿真或实机完整联调

#### 快速自检命令

```bash
# 1. 确保用 RMUL 地图（取消可能存在的 MAP_YAML 覆盖）
unset MAP_YAML
./scripts/run_test_a_headless.sh

# 2. 检查 Nav2 是否就绪
ros2 action list | grep navigate_to_pose
ros2 topic list | grep -E "map|scan|cmd_vel"

# 3. 行为树配置位置
ls ~/sentry_planner/rm_decision_ws/src/rm_behavior_tree/config/
```

#### 详细排查清单（run_test_a 报错时）

| 报错 | 原因 | 处理 |
|------|------|------|
| **地图仍是 11_map** | 环境变量 `MAP_YAML` 覆盖 | 运行前执行 `unset MAP_YAML`；脚本已改为固定 RMUL |
| **Failed to change state for node: map_server** | map_server 生命周期激活失败 | ① `cd /tmp` 导致环境异常：尝试去掉 `cd /tmp` 在同一 shell 启动<br>② 地图路径错误：确认 RMUL.yaml 存在<br>③ nav2_bringup 与 my_nav2_params 不兼容 |
| **Action server 'navigate_to_pose' is not reachable** | Nav2 未完全启动 | map_server 失败导致整链失败；先修 map_server，再增加 sleep 或等待 action 就绪 |
| **Can't convert string [ 0] to double** | `goal_pose` 格式含前导空格 | `bt_conversions.hpp` 中 `parts[3]` 为 `" 0"`，需 trim；或改 XML 为 `0;0;0;0;0;0;1`（无空格） |
| **Missing required input [message]** | `robot_status` / `robot_hp` 为空 | 无雷达测试无 C 板，不发布这些话题；需在 fake_sensors 中增加假 `robot_hp`、`robot_status` 发布 |
| **SubAllRobotHP topic_name** | XML 为 `robot_hp`，C 板为 `/all_robot_hp` | 确认 topic 名一致，必要时改 XML 或加 remap |

**run_test_a 启动顺序：** 假传感器(1) → Nav2(2) → 决策树(3) → game_status(4)。Nav2 需约 8s 才能就绪，决策树过早启动会报 `navigate_to_pose` 不可达。

### 5.8 键盘控制测试 (固件调试)

**ROS2 键盘：**
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**直接串口 19 字节协议 (nyush-rm-control)：**
```bash
cd ~/path/to/nyush-rm-control/scripts
python3 cmd_vel_keyboard_fixed.py --port /dev/ttyACM0 --speed 0.3 --keyboard
```

---

## 5.9 Navigation 仿真进度与配置

> **当前状态：Navigation 仿真已跑通**（2025-02）

### 5.9.1 推荐启动命令

```bash
cd ~/sentry_planner/rm_navigation_ws
source ~/nav_ws/install/setup.bash
source install/setup.bash

ros2 launch rm_nav_bringup bringup_sim.launch.py \
  world:=RMUL mode:=nav localization:=amcl lio:=fastlio nav_rviz:=True
```

**使用步骤：**
1. 启动后在 RViz 中点击 **2D Pose Estimate**，在地图上拖出机器人初始位姿（AMCL 需此才会发布 map 坐标系）
2. 点击 **Nav2 Goal** 发送导航目标
3. 若仍卡顿，可加 `headless:=True` 关闭 Gazebo GUI

### 5.9.2 已完成的配置与优化

#### Nav2 参数 (nav2_params_sim.yaml)

| 类别 | 关键配置 | 说明 |
|------|----------|------|
| **全向轮** | min_y_velocity_threshold: 0.001 | 0.1 会截断小 y 速度，全向轮必须 0.001 |
| **AMCL** | max_particles 5000, recovery_alpha_fast 0.1 | 支持全局定位与丢失恢复 |
| **Controller** | controller_frequency 10 Hz | 平衡响应与 CPU 负载 |
| **DWB** | vx_samples 15, vy_samples 5, sim_time 1.5 | 支持旋转 (max_vel_theta 1.0) |
| **Costmap** | local 3×3m res 0.1, global res 0.1 | 参考 my_nav2_params |
| **velocity_smoother** | max_velocity [0.5, 0.5, 5.0] | 限速 0.5 m/s，保留角速度 |

参考文件：`sentry_planner/my_nav2_params.yaml`

#### 轻量化配置（降低仿真卡顿）

| 模块 | 优化项 |
|------|--------|
| **Gazebo** | 物理 100 Hz、关闭阴影 |
| **Livox 仿真** | 点云 30000→8000、update_rate 10→5 Hz、downsample 2 |
| **Fast-LIO** | point_filter_num 5、max_iteration 1、map_en false |
| **Ground Segmentation** | n_bins 80、n_segments 180、n_threads 2 |
| **headless 模式** | `headless:=True` 可完全关闭 Gazebo GUI |

详细参数见：`rm_navigation_ws/src/rm_nav_bringup/config/simulation/`

#### base_link_fake 与 fake_vel_transform

- **用途**：云台与雷达共 yaw 轴时，使 Nav2 规划方向与云台/枪管朝向解耦
- **流程**：Nav2 在 base_link_fake（路径朝向）下发 cmd_vel → fake_vel_transform 转换到 base_link → cmd_vel_chassis 给底盘
- **仿真**：云台不转时 base_link_fake≈base_link，保留架构以便与实机一致

### 5.9.3 仿真排障

详见 `rm_navigation_ws/docs/仿真启动排障.md`。

---

## 5.10 下一步 TODO

| 优先级 | 任务 | 说明 |
|--------|------|------|
| **P0** | 实机 Nav2 联调 | 使用 real launch、确认 my_nav2_params 在实机上的表现 |
| **P0** | 决策树与仿真完整联调 | bringup_sim + rm_behavior_tree + game_status，验证攻击/撤退/占点逻辑 |
| **P1** | 地图与初始位姿自动化 | 启动时自动设置 AMCL 初始位姿，减少手动 2D Pose Estimate |
| **P1** | 裁判系统仿真/假数据 | 完善 fake_sensors 或增加裁判协议模拟，供决策树测试 |
| **P1** | 实机串口与 cmd_vel 验证 | 确认 /cmd_vel → serial_sender/rm_serial_driver → C 板 19 字节协议正确 |
| **P2** | Nav2 参数精细调优 | 根据 RMUL 场地和实际机器人尺寸微调 costmap、DWB、velocity_smoother |
| **P2** | 云台小陀螺模式验证 | 实机或仿真中验证 fake_vel_transform 在云台旋转时的轨迹跟踪效果 |
| **P2** | 建图流程文档化 | mapping 模式建图、保存、转 localization 的完整步骤 |

---

## 六、实车部署

### 6.1 硬件要求

| 硬件 | 型号 | 用途 |
|------|------|------|
| 激光雷达 | Livox Mid360 | 定位、建图、避障 |
| 工控机 | Intel NUC / Jetson | 运行ROS2 |
| 相机 | HIK工业相机 | 装甲板检测 |
| 主控板 | STM32F407 (C板) | 底层控制 |

### 6.2 启动实车

```bash
# 启动导航
ros2 launch rm_nav_bringup bringup_real.launch.py \
  world:=YOUR_MAP_NAME \
  mode:=nav \
  lio:=fastlio \
  localization:=slam_toolbox

# 启动视觉
ros2 launch rm_vision_bringup vision_bringup.launch.py

# 启动决策
ros2 launch rm_behavior_tree rm_behavior_tree.launch.py
```

---

## 七、常见问题

| 问题 | 解决方案 |
|------|----------|
| Gazebo黑屏 | `export LIBGL_ALWAYS_SOFTWARE=1` |
| rosdep找不到包 | `sudo rosdep init && rosdep update` |
| 串口权限 | `sudo chmod 666 /dev/ttyACM0` 或加入dialout组 |
| Nav2路径规划失败 | 检查地图是否正确加载，costmap是否清除 |
| 行为树不执行 | 检查 `/game_status` 是否发布，`game_progress` 是否为4 |
| **Action server 'navigate_to_pose' is not reachable** | Nav2 未完全启动或 map_server 激活失败；决策树需在 Nav2 就绪后启动 |
| **Can't convert string to PoseStamped** | 行为树 `goal_pose` 格式错误，检查 XML/YAML 中 `x;y;z; qx;qy;qz;qw` 是否符合 BT 要求 |
| **Failed to change state for node: map_server** | map_server 生命周期激活失败；run_test_a 中 `cd /tmp` 可能影响环境，可尝试不切换目录启动 |
| **rm_navigation_ws: package 'rm_navigation' not found** | 需 build `rm_navigation`、`fake_vel_transform`，执行 `colcon build --symlink-install` 全量构建 |
| **rm_navigation_ws: livox_ros_driver2 not found** | 构建前先 `source ~/nav_ws/install/setup.bash` |
| **source install/setup.bash 报 "not found" 或 "local_setup.bash"** | install 下的 setup 被污染了链式依赖，可 `rm -rf build install log` 后干净环境重建，或手动编辑 install/setup.bash 移除对 decision/vision 的 chain |

---

## 八、参考资料

- [BehaviorTree.CPP 官方文档](https://www.behaviortree.dev/)
- [Navigation2 官方文档](https://navigation.ros.org/)
- [FAST_LIO GitHub](https://github.com/hku-mars/FAST_LIO)
- [Livox ROS Driver 2](https://github.com/Livox-SDK/livox_ros_driver2)
- [SMBU 原始仓库](https://gitee.com/SMBU-POLARBEAR)
- [NYUSH-RM C板代码 (nyush-rm-control)](https://github.com/NYUSH-Robotics-Club/nyush-rm-control)

**补充文档：** `README(3).md` 含更详细的雷达选型、Nav2 参数、建图流程、性能指标等。

---

## 九、贡献者

- 原始代码: 深圳北理莫斯科大学 北极熊战队
- 适配修改: NYUSH Robotics Club
