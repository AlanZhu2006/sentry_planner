# RM2025 Auto Sentry - NYUSH Robotics

基于深圳北理莫斯科大学北极熊战队开源代码修改，适配 NYUSH Robotics 哨兵机器人

**环境要求:** Ubuntu 22.04 + ROS2 Humble + Gazebo Classic 11

---

## 一、系统架构总览

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
│  │  输出: /cmd_vel_chassis ──────────────► C板执行                       │   │
│  └──────────────────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## 二、项目结构

```
RM2024_SMBU_auto_sentry_ws/
├── rm_decision_ws/          # 决策模块 (行为树)
│   ├── rm_behavior_tree/    # 行为树实现
│   │   ├── config/          # 行为树XML配置
│   │   │   ├── attack_left.xml
│   │   │   ├── attack_right.xml
│   │   │   ├── protect_supply.xml
│   │   │   └── retreat_attack_left.xml
│   │   ├── plugins/         # 行为树节点插件
│   │   │   ├── action/      # 动作节点
│   │   │   ├── condition/   # 条件节点
│   │   │   └── decorator/   # 装饰节点
│   │   └── src/             # 主程序
│   └── rm_decision_interfaces/  # 消息定义
│
├── rm_navigation_ws/        # 导航模块
│   └── src/
│       ├── rm_nav_bringup/  # 启动文件和配置
│       ├── rm_localization/ # 定位 (FAST_LIO, Point_LIO, ICP)
│       ├── rm_navigation/   # Navigation2 + TEB
│       ├── rm_perception/   # 点云处理
│       └── rm_simulation/   # Gazebo仿真
│
└── rm_vision_ws/            # 视觉模块
    └── src/
        ├── rm_auto_aim/     # 自瞄算法
        ├── rm_serial_driver/ # 串口通信 (与C板)
        └── ros2_hik_camera/ # 海康相机驱动
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
│  rm_serial_driver│               │  master_process │
└─────────────────┘                └─────────────────┘
```

### 4.2 数据包格式

#### C板 → 上位机 (裁判系统数据上传)

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
| 导航速度 | `0xA5 0x5A` | vx + vy + wz (float) | CRC8 | 15B |
| 机器人控制 | `0xA3` | stop_scan + spin_vel | CRC16 | 8B |

**NavCmd / Twist (0xA5 0x5A):**
```
[0xA5][0x5A][vx:4B][vy:4B][wz:4B][CRC8:1B]
vx, vy: m/s (底盘线速度)
wz: rad/s (底盘角速度)
CRC8 polynomial: 0x07
```

**RobotControl (0xA3):**
```
[0xA3][stop_gimbal_scan:1B][chassis_spin_vel:4B][CRC16:2B]
stop_gimbal_scan: 0=继续扫描, 1=停止扫描
chassis_spin_vel: 底盘自旋速度 (rad/s)
```

### 4.3 ROS2 Topic 对应关系

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

### 5.2 编译项目

```bash
# 编译导航模块
cd rm_navigation_ws
rosdep install -r --from-paths src --ignore-src --rosdistro humble -y
colcon build --symlink-install
source install/setup.bash

# 编译决策模块
cd ../rm_decision_ws
rosdep install -r --from-paths src --ignore-src --rosdistro humble -y
colcon build --symlink-install
source install/setup.bash

# 编译视觉模块 (可选)
cd ../rm_vision_ws
rosdep install -r --from-paths src --ignore-src --rosdistro humble -y
colcon build --symlink-install
source install/setup.bash
```

### 5.3 运行仿真

```bash
# 终端1: 启动导航仿真
cd rm_navigation_ws && source install/setup.bash
ros2 launch rm_nav_bringup bringup_sim.launch.py \
  world:=RMUL \
  mode:=mapping \
  lio:=fastlio \
  nav_rviz:=True

# 终端2: 启动决策树
cd rm_decision_ws && source install/setup.bash
ros2 launch rm_behavior_tree rm_behavior_tree.launch.py \
  style:=./rm_behavior_tree/config/attack_left.xml

# 终端3: 模拟裁判系统数据 (测试用)
ros2 topic pub /game_status rm_decision_interfaces/msg/GameStatus \
  "{game_progress: 4, stage_remain_time: 180}"
```

### 5.4 键盘控制测试

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

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

---

## 八、参考资料

- [BehaviorTree.CPP 官方文档](https://www.behaviortree.dev/)
- [Navigation2 官方文档](https://navigation.ros.org/)
- [FAST_LIO GitHub](https://github.com/hku-mars/FAST_LIO)
- [SMBU 原始仓库](https://gitee.com/SMBU-POLARBEAR)
- [NYUSH-RM C板代码](https://github.com/NYUSH-Robotics-Club/nyush-rm-control)

---

## 九、贡献者

- 原始代码: 深圳北理莫斯科大学 北极熊战队
- 适配修改: NYUSH Robotics Club
