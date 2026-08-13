# NYUSH 哨兵：常用命令与数据流说明

<div align="center">

[![简体中文](https://img.shields.io/badge/%E7%AE%80%E4%BD%93%E4%B8%AD%E6%96%87-%E5%BD%93%E5%89%8D-0969da?style=for-the-badge)](README_COMMANDS.md)
[![English](https://img.shields.io/badge/English-Switch-2ea44f?style=for-the-badge)](README_COMMANDS_EN.md)

</div>

最后更新：2026-04-11  

本文与 [`communication command.txt`](communication%20command.txt) 配合使用：**这里写清原理、数据流与各参数含义**；`communication command.txt` 侧重**可复制命令片段**。协议细节仍以 [`README_COMMUNICATION.md`](README_COMMUNICATION.md) 为准。

---

## 0. 文档定位（五文分工）

| 文档 | 职责 |
|------|------|
| [README.md](README.md) | **中央索引**：一页总览、架构简图、编译顺序、最短启动 |
| [README_COMMUNICATION.md](README_COMMUNICATION.md) | **通讯与协议全文**：帧、PTY、`serial_sender`、`bt_comm_adapter`（**深度**见该文 **§5～§15**） |
| [README_BEHAVIOR_TREE_FLOW.md](README_BEHAVIOR_TREE_FLOW.md) | **行为树全文**：XML、节点、`RobotControl` 语义、Groot2、**§5.1** 坐标与 watcher |
| [README_LIDAR.md](README_LIDAR.md) | **激光雷达 + Nav2**：Mid360、FAST-LIO、参数、**§10.8** 实机定位前提 |
| **本文** | **命令与数据流全书**：可执行命令、环境变量、脚本对照、端到端链路摘要 |

**如何选读：** 要**复制命令**、理解 **`MAP_FILE`/起终点**、**实机分阶段步骤** → **本文**；协议位域与 MCU 帧 → [README_COMMUNICATION.md](README_COMMUNICATION.md)；**树怎么切分支、XML 里写什么** → [README_BEHAVIOR_TREE_FLOW.md](README_BEHAVIOR_TREE_FLOW.md)；雷达与 costmap 现象 → [README_LIDAR.md](README_LIDAR.md)。

**本文章节索引（便于跳转）：**

| 章节 | 内容 |
|------|------|
| **§1** | MCU / LiDAR / 视觉 / BT / 上下位机，四条路径 A–D |
| **§2** | Bridge 自动选口与 PTY |
| **§3** | 视觉 Web、与 BT/`RobotControl` 分工 |
| **§4** | `nav_ws/start_robot.sh` 环境变量；**§4.4** `11_map` / `RMUL2026`、PCD·PGM·YAML、换图与 BT 起终点 |
| **§5～§6** | 裁判话题、`watch_*` 与热键 |
| **§7** | `rotate_pcd` → `pcd2pgm` → `map_saver_cli`；**§7.4** `map_point_picker.py` |
| **§8～§11** | `autostart`、脚本对照表、最小终端组合、`communication command.txt` 提示 |
| **§12** | **实机联调**：台架→小场地→全场、安全、终端分工、检查清单、伪造裁判 |
| **§13** | **四专题分工速查表**（问题 → 打开哪份 README） |

**与另文的边界：** 本文 **§1** 给**总链路**；**SP/SX/ST 字节级定义**以 [README_COMMUNICATION.md](README_COMMUNICATION.md) 为准。**§4.4、§7** 给地图与建图**命令**；**Gazebo RMUL2026、Sim2Real 第一步、Groot2 与仿真的关系**以 [README_LIDAR.md §6.4](README_LIDAR.md#nyush-gazebo-sim2real) 为准（逐步命令另见 [mid360 command.txt](mid360%20command.txt)）。**倾斜雷达 TF、性能指标**见 [README_LIDAR.md](README_LIDAR.md)。

---

## 1. 完整数据链路与职责划分

### 1.1 物理分层：谁在哪个「电脑」上跑

| 层级 | 硬件/进程 | 说明 |
|------|-----------|------|
| **上位机（工控机 NUC 等）** | ROS 2：雷达驱动、FAST-LIO、Nav2、行为树、`serial_sender`、`bt_comm_adapter` | 定位、规划、决策；把**速度 + 模式字**打成串口侧协议 |
| **桥接（一般跑在上位机）** | `sentry_bridge.py` | **唯一**占用 USB CDC；拆成 **Vision PTY** 与 **Radar PTY** |
| **下位机** | STM32（`nyush-rm-control`） | 解析 **SX/ST、SP**；融合命令；驱动底盘/云台/射击；读裁判 |

**LiDAR（Mid360）**：挂在车上，**网线**进工控机，**不经过 MCU USB**。点云/里程计以 ROS **话题与 TF** 存在，供 Nav2 与 BT 使用。

---

### 1.2 四条主数据路径（从传感器到执行）

```text
【A — 定位与导航 / 底盘平移】
Mid360 ─(以太网)→ livox_ros_driver2 → FAST-LIO → /cloud_registered、里程计、TF
       → pointcloud_to_laserscan → /scan → Nav2 → /cmd_vel
       → fake_vel_transform → /cmd_vel_chassis
       → bt_comm_adapter（并入 chassis_spin_vel）→ /cmd_vel_chassis_bt
       → serial_sender → Radar PTY → bridge → MCU
       → SX(vx,vy,wz,…) → 底盘执行（MCU 内 swerve 等解算）

【B — 行为树决策 / 模式与底盘旋转意图】
BT 订阅 /game_status、/robot_status（等）→ 逻辑判断
BT 发布 /goal_pose → Nav2（只影响路径 A，不经串口）
BT 发布 /robot_control → serial_sender → A3 功能帧 → Radar PTY → bridge → MCU
       → SX.control_flags、scan_*、allow_vision_control、…
       → robot_cmd.c：是否采纳视觉、BT 扫描、与 vx/vy/wz 一起参与整机状态机

【C — 视觉伺服 / 云台角与开火】
相机 → nyush-rm-vision → SP(VisionToGimbal) → Vision PTY → bridge → MCU
     → 在「允许视觉接管」等标志有效时，用 SP 的 yaw/pitch/mode 做跟瞄、开火
MCU → SP(GimbalToVision) → 视觉闭环（姿态、弹速等）

【D — 裁判与机载状态回上位机】
裁判 → MCU → ST → bridge → Radar PTY 侧 0x5C/0x5D
     → serial_sender → /game_status、/robot_status → BT 订阅
```

A 与 B 在 **Radar PTY** 合并为**速度帧 + A3**；C 独立走 **Vision PTY**；D 与 A/B **同经 Radar 回传**，在 sender 里解成 ROS。

---

### 1.3 各模块负责什么（速查）

| 模块 | 位置 | 负责内容 | 主要对外接口 |
|------|------|----------|----------------|
| **LiDAR + 驱动** | 车上 + 工控机 | 点云 | `/livox/lidar` 等 |
| **FAST-LIO** | 工控机 | 里程计、点云配准 | `/cloud_registered`、odom、TF |
| **pointcloud_to_laserscan** | 工控机 | 2D 激光 | `/scan` |
| **Nav2** | 工控机 | 规划、避障 | `/cmd_vel` |
| **fake_vel_transform** | 工控机 | 速度系变换 | `/cmd_vel_chassis` |
| **bt_comm_adapter** | 工控机 | 速度 + 小陀螺合并 | `/cmd_vel_chassis_bt` |
| **rm_behavior_tree** | 工控机 | 高层战术、发点、发 RobotControl | `/goal_pose`、`/robot_control` |
| **serial_sender --ros2** | 工控机 | ROS ↔ 雷达侧 PTY；状态回灌 | 19B 速度 + A3 |
| **sentry_bridge** | 工控机 | 单 USB ↔ 双 PTY；帧转换 | SP / SX / ST |
| **nyush-rm-vision** | 工控机 | 检测、跟踪、瞄准 | SP |
| **MCU** | C 板 | 执行与融合 | SX+SP+RC→底盘/云台 |

---

### 1.4 RobotControl（SX）与 SP 的分工

| 通道 | 典型内容 | 谁产生 |
|------|----------|--------|
| **SX / RobotControl** | 是否允许视觉接管、扫描参数、`chassis_spin_vel`、标志位 | BT → sender |
| **SP** | 目标 yaw/pitch、自瞄/开火 mode | Vision |

MCU 用 **SX 标志** 决定 **`vision_enabled`** 等；为真时才把 **SP 跟瞄**接到云台环（见 `nyush-rm-control/application/cmd/robot_cmd.c`）。

---

### 1.5 简化逻辑图（与 §1.2 对照）

```text
                    ┌───────────── BT (rm_behavior_tree)
                    │  订阅: /game_status, /robot_status, …
                    │  发布: /goal_pose, /robot_control
                    └──────┬───────────────────────┬──────────────┐
                           │                       │
                           ▼                       ▼
                    ┌──────────────┐      ┌─────────────────────────┐
                    │    Nav2      │      │ bt_comm_adapter.py      │
                    │  /cmd_vel    │      │ /cmd_vel_chassis +      │
                    └──────┬───────┘      │ /robot_control →        │
                           │              │ /cmd_vel_chassis_bt     │
                           ▼              └────────────┬────────────┘
                    ┌──────────────┐                 │
                    │fake_vel_     │                 │
                    │transform     │                 │
                    └──────┬───────┘                 │
                           │                         │
                           └────────────┬────────────┘
                                        ▼
                              serial_sender.py --ros2
                                        │
                                        ▼
                              Radar PTY → sentry_bridge → MCU (SX)
                                        ▲
MCU 裁判/状态 ◄── ST ── bridge ──► 0x5C/0x5D ──► serial_sender ──► /game_status, /robot_status

视觉: nyush-rm-vision ◄── SP ──► Vision PTY ──► bridge ──► MCU
```

---

## 2. 启动 Bridge（不要写死 `/dev/ttyACM0`）

### 2.1 推荐命令

```bash
cd /path/to/nyush-rm-control
just sentry-bridge
```

等价于调用 `sentry_bridge.py` **不传 `--port`**。

### 2.2 内部行为（自动选口）

`sentry_bridge.py` 中 `--port` 的 help 写明：**默认自动检测**。实现上会枚举 USB 串口并按优先级挑选 MCU CDC（见脚本内 `resolve_serial_port` / `port_priority`）。  

USB 重新插拔后设备名可能从 `ttyACM0` 变成 `ttyACM1` 等，**自动选口**可避免改命令。

### 2.3 何时仍需要手写 `--port`

- 多台串口设备同时插入，自动选错时：  
  `just sentry-bridge --port /dev/ttyACM1`  
- 调试时指定固定设备。

### 2.4 Bridge 启动后你要记下的东西

终端会打印 **MCU serial**（实际打开的口）、**Vision PTY**、**Radar PTY**。  
若未加 `--no-links`，通常还有稳定软链接（如 `/tmp/nyush-rm-sentry-radar`），**导航侧 `RADAR_PTY` 优先用软链接**，避免 `/dev/pts/N` 每次变。

---

## 3. 视觉：`just test detect --web --send`

### 3.1 命令与 Web

```bash
cd /path/to/nyush-rm-vision
just test detect --web --send
```

- **`--web`**：在本地起 HTTP 服务，**默认** `http://127.0.0.1:8888`（以当前 vision 仓库 justfile 为准），用于看检测画面、调试参数。  
- **`--send`**：把视觉结果通过串口协议发给下位机；**串口必须是 Vision PTY**（在视觉配置里把 `com_port` 指到 bridge 打印的 Vision 链，或稳定软链接）。

### 3.2 和 BT 怎么互动（概念上）：**有关系，但默认树不读装甲板话题**

先分清两件事：

1. **「识别到目标 / 自瞄」** —— 主要由 **视觉程序 → Vision PTY → SP → 下位机** 完成；检测、跟踪、发给 MCU 的云台量在这条链上。  
2. **「行为树要不要根据『有没有敌人』切分支」** —— 这才涉及 BT 是否订阅 **`/detector/armors`**（或其它视觉 ROS 话题）。

**`center_attack_simple.xml` 里只有 `SubRobotStatus` 与 `SubGameStatus`，没有 `SubArmors`，也没有 `IsDetectEnemy`。** 因此：

- **不是没有「识别目标」**，而是 **不把「是否看到装甲」写进这棵树的判断条件**。  
- 到中心驻守时，BT 通过 **`RobotControl`** 设置例如 **`allow_vision_control=True`、`stop_gimbal_scan=True`** 等，相当于下发 **「允许视觉自瞄接管云台」的模式位**；**真正看到目标并跟瞄**仍由 **视觉 + SP** 完成，**不依赖** BT 订阅装甲板。  
- 行进阶段若 **`allow_vision_control=False`**，更偏 **扫描**；视觉仍可运行、网页仍可看检测，但是否按队内逻辑接管由 **MCU + 这些标志** 共同决定（以固件为准）。

**装甲板话题和 BT 何时强相关？**  
在使用 **`SubArmors` + `IsDetectEnemy`** 的旧树（如 **`retreat_attack_left.xml`**）时：BT **订阅** `/detector/armors`，用「列表是否为空」等做 **见敌 / 不见敌** 的战术分支。

**可选第三条路径：** `bt_comm_adapter.py` 可把 **`auto_aim_target_pos`** 转成 **`/detector/armors`**，供需要该话题的树使用；与 **SP 直连 MCU** 是不同通道。

**小结：**  
- **视觉与 BT 有关系**：BT 用 **`/robot_control`** 管 **扫描、是否允许自瞄接管、小陀螺** 等 **高层模式**；视觉管 **感知与 SP 控制量**。  
- **默认简单树不订阅装甲板**：不是「视觉没用」，而是 **故意不把见敌写进 BT**，把 **目标级闭环** 放在 **视觉–MCU**，BT 侧重 **赛况、血量、到点、回家**。

| 路径 | 作用 |
|------|------|
| 视觉 → MCU | **SP**：云台与开火相关控制，**不经过** BT 节点。 |
| BT → MCU（经 `serial_sender`） | **SX** / **RobotControl**：`scan_*`、`allow_vision_control` 等 **模式与权限**。 |
| BT ← `/detector/armors` | **仅旧树**等 XML 显式写了 `SubArmors` / 见敌条件时。 |

---

## 4. `~/nav_ws/start_robot.sh`：调试用环境变量详解

以下命令是你常用的「**bridge +（可选）vision 已开** → **自建图 + 起 Nav + BT**」形态：

```bash
cd ~/nav_ws
MAP_FILE="/home/nyu/Desktop/map/11_map.yaml" \
BT_STYLE=center_attack_fullstack \
BT_START_GOAL="0.407;0.130;0; 0;0;0;1" \
BT_END_GOAL="1.083;0.767;0; 0;0;0;1" \
START_SERIAL_SENDER=1 \
SERIAL_SENDER_DISABLE_STATUS_PUB=0 \
START_BT=1 \
RADAR_PTY=/tmp/nyush-rm-sentry-radar \
START_FAKE_VEL_TRANSFORM=1 \
./start_robot.sh
```

### 4.1 各变量含义

| 变量 | 含义 |
|------|------|
| **`MAP_FILE`** | Nav2 `map_server` 加载的 **地图入口 yaml**（内含对 **pgm** 的引用）。**日常自建图**见下表 **`11_map`**；**赛场**见 **`RMUL2026.yaml`**。详见 **§4.4**。 |
| **`BT_STYLE`** | 行为树 XML **不含 `.xml`**。`center_attack_fullstack` 为队内全栈树；`center_attack_simple` 为简化树（在 `sentry_planner` 的 `start_robot.sh` 默认更常见）。 |
| **`BT_START_GOAL` / `BT_END_GOAL`** | 传给 `rm_behavior_tree` launch 的 **`start_goal_pose` / `end_goal_pose`**，写入黑板；格式 `x;y;z; qx;qy;qz;qw`。**换图后常与 XML 里写死的 `SendGoal` 一起核对**，见 **§4.4**。 |
| **`START_SERIAL_SENDER=1`** | 在后台启动 **`serial_sender.py --ros2`**，把 ROS 速度与控制写到 **`RADAR_PTY`**。 |
| **`RADAR_PTY`** | **必须与当前 bridge 的 Radar 侧一致**（推荐 `/tmp/nyush-rm-sentry-radar`）。脚本内会赋给 `SERIAL_SENDER_PORT`。 |
| **`START_BT=1`** | 启动 **`bt_comm_adapter.py`** + **`rm_behavior_tree`**。且脚本会强制把 **`SERIAL_SENDER_TOPIC` 改为 `/cmd_vel_chassis_bt`**（若原来是 `/cmd_vel_chassis`），以便合并 BT 的 `chassis_spin_vel`。 |
| **`SERIAL_SENDER_DISABLE_STATUS_PUB`** | 传给 sender 的 **`--disable-status-pub`**：`0` = **发布** `/game_status`、`/robot_status`（来自 bridge 回传的 0x5C/0x5D）；`1` = 不发布，避免与 **热键调试** 或 **其它伪造裁判** 抢话题。 |
| **`START_FAKE_VEL_TRANSFORM=1`** | 启动 **`fake_vel_transform`**：`/cmd_vel` → `/cmd_vel_chassis`。Nav2 与底盘坐标/云台补偿在此对齐。 |

### 4.2 脚本内还会 source 的工作空间

默认会 source **`~/nav_ws/install`**、**`sentry_planner/install`**（若存在）、**`rm_vision_ws`**、**`rm_decision_ws`**，以便 `rm_behavior_tree`、`fake_vel_transform`、`serial_sender` 依赖的消息与包可用。

### 4.3 与 `sentry_planner/start_robot.sh` 的区别

- **`nav_ws/start_robot.sh`**：你当前 **ICP 未接**时的主力脚本，Nav2 用 **AMCL + bringup_launch**，环境变量如上。  
- **`sentry_planner/start_robot.sh`**：队内另一套 **Mid360 + Fast-LIO + 可选 ICP** 长脚本，变量名部分重叠但默认值不同（见该文件头部）。

### 4.4 地图文件两套来源；`PCD` / `PGM` / `YAML` 各在哪一步起效；换图与 BT 起终点

#### 4.4.1 你常用的两套地图（路径约定）

| 场景 | `MAP_FILE` 典型取值 | 说明 |
|------|---------------------|------|
| **自建图 / 实验室** | `$HOME/Desktop/map/11_map.yaml` | 建图流程（§7）里用 `map_saver_cli -f 11_map` 保存到 **`~/Desktop/map/`**，得到 **`11_map.yaml` + `11_map.pgm`** 一对文件。启动 Nav 时 **`MAP_FILE` 指向 yaml**。 |
| **正式赛场 RMUL 2026** | `…/sentry_planner/rm_navigation_ws/src/rm_nav_bringup/map/RMUL2026.yaml` | 与赛场地形一致的固定图；同级有 **`RMUL2026.pgm`**。场上统一把 **`MAP_FILE`** 指到该 yaml。 |

`yaml` 里的 **`image:`** 字段写相对路径时，**pgm 与 yaml 同目录**即可；不要只拷 yaml 不拷 pgm。

#### 4.4.2 `PCD`、`PGM`、`YAML` 在链路里分别干什么

| 文件类型 | 典型位置 | **谁在运行时读它** |
|----------|----------|---------------------|
| **PCD** | 如 `~/nav_ws/src/FAST_LIO/PCD/scans.pcd` | **仅离线建图管道**：`rotate_pcd` → **`pcd2pgm`** 的输入。**Nav2 / `map_server` 启动后不会加载 PCD**。 |
| **PGM** | 与地图 yaml 同目录，由 yaml 的 `image:` 指向 | **`map_server` 通过 yaml 载入栅格**；全局代价地图、规划器、`SendGoal` 是否在自由空间，都基于这份 **2D 占用栅格**。 |
| **YAML**（地图元数据） | 传给环境变量 **`MAP_FILE`** 的那个文件 | **Nav2 地图服务的唯一入口**：分辨率、原点、朝向、`free_thresh` / `occupied_thresh`、以及 **指向哪张 pgm**。换图 = **换 `MAP_FILE`**（并确保对应 pgm 存在且路径正确）。 |

一句话：**运行时 Nav2 只认「地图 yaml + 其引用的 pgm」**；**PCD 只参与生成这份 pgm 之前的那几步**。

#### 4.4.3 行为树起点 / 终点：换图必须重标定

当前使用的树里，**部分 `SendGoal` 或占位点可能在 XML 里写死世界坐标**。换 **`11_map` ↔ `RMUL2026`**（或任意新图）后，**同一组数字在栅格上可能变成障碍或根本不在场内**，必须重新测定并：

- **改 `start_robot.sh` 前的环境变量**：**`BT_START_GOAL`**、**`BT_END_GOAL`**（格式 `x;y;z; qx;qy;qz;qw`），以及  
- **核对 / 修改 XML 内硬编码坐标**（以你实际选用的 **`BT_STYLE`** 为准——黑板参数与 XML 可同时存在，**以树里节点实际引用为准**）。

**坐标从哪里来：** 保存好地图 yaml 后，在有 **图形界面** 的环境（实车工控机常通过 **VNC**）运行 **`map_point_picker.py`**，在弹出的地图上点击读 **`x, y`**（相对地图原点），再填入上述参数或 XML。命令与注意见 **§7.4**。

---

## 5. 裁判数据是否进到 ROS：`/game_status`、`/robot_status`

```bash
source /opt/ros/humble/setup.bash   # 或 setup.zsh
source /path/to/sentry_planner/rm_decision_ws/install/setup.bash
ros2 topic echo /game_status
ros2 topic echo /robot_status
```

### 5.1 数据从哪来

1. MCU 经 **ST** 把精简裁判字段交给 **bridge**。  
2. Bridge 在 Radar PTY 侧发 **0x5C / 0x5D** 帧。  
3. **`serial_sender.py --ros2`** 解析后 **发布** `GameStatus`、`RobotStatus`。

若 **echo 有刷新且字段合理**，说明 **上位机已收到裁判链路**（至少到 sender）。若全 0 或无输出：检查 bridge、sender 是否运行、`SERIAL_SENDER_DISABLE_STATUS_PUB` 是否为 1。

---

## 6. 无真实裁判时调试 BT：`watch` + `hotkey`

### 6.1 `watch_center_attack_state.py`（只读观察）

```bash
source /opt/ros/humble/setup.bash
source /path/to/sentry_planner/rm_decision_ws/install/setup.bash
python3 /path/to/sentry_planner/scripts/watch_center_attack_state.py
```

- **角色**：**订阅** `/amcl_pose`、`/game_status`、`/robot_status`、`/robot_control`、`/cmd_vel_chassis_bt`，定时打印一行状态。  
- **对应关系**：**BT + Nav（位姿）+ 通讯输出** 的「监视器」，**不发布**话题。  
- 适合与下面热键脚本同开：一个改状态，一个看结果。

### 6.2 `bt_hotkey_debug.py`（伪造裁判输入）

```bash
source /opt/ros/humble/setup.bash
source /path/to/sentry_planner/rm_decision_ws/install/setup.bash
python3 /path/to/sentry_planner/scripts/bt_hotkey_debug.py
```

- **角色**：**持续发布** `/game_status`、`/robot_status`（及某些预设下的 `/detector/armors`），用键盘切换「未开赛 / 比赛中 / 低血 / 高热 / attacked」等。  
- **对应关系**：**行为树调试**；会 **覆盖** 真实裁判（若同时开着 `serial_sender` 发状态，二者会抢同一话题——调试时通常 **`SERIAL_SENDER_DISABLE_STATUS_PUB=1`** 关 sender 的状态发布）。  
- 按键与预设名见脚本内 `PRESETS`（含 `0/1/2/…` 等）。

---

## 7. Fast-LIO 建图后：旋转 PCD → 2D PGM → 保存地图（Nav2）

流程对应你三条命令：

### 7.1 旋转点云

```bash
cd ~/nav_ws/src/FAST_LIO/PCD
python3 rotate_pcd.py
```

- **作用**：对建图得到的 **`scans.pcd`（或脚本配置的路径）** 做姿态修正，使地面与地图轴向和 Nav2 期望一致。  
- **注意**：`pcd2pgm` 的 launch 里若写死了输入 PCD 路径，需与旋转输出路径一致。

### 7.2 PCD 转 2D 栅格（pgm）

```bash
export LD_PRELOAD=/lib/x86_64-linux-gnu/libusb-1.0.so.0
cd ~/nav_ws
source install/setup.zsh   # 或 setup.bash
ros2 launch pcd2pgm pcd2pgm_launch.py
```

- **作用**：把 **3D 点云** 投影成 **2D 占用栅格**，供 `map_server` 使用。  
- **`LD_PRELOAD`**：缓解部分环境下 PCL/Open3D 与 **libusb** 的兼容问题（与是否插雷达无关时常保留也无妨）。

### 7.3 保存 Nav2 地图

```bash
cd ~/Desktop/map
ros2 run nav2_map_server map_saver_cli -f 11_map
```

- **作用**：把当前 **`/map`** 保存为 **`11_map.yaml` + `11_map.pgm`**（前缀 `-f`）。  
- 之后在 **`MAP_FILE`** 里指向该 yaml 即可被 `start_robot.sh` 使用。  
- **PCD / PGM / YAML 分工、赛场 `RMUL2026`、换图时要改 BT 起终点**：见 **§4.4**。

### 7.4 用 `map_point_picker.py` 在图上点选坐标（需 GUI / VNC）

在 **`map_saver_cli` 保存完** `11_map.yaml`（或任意已就绪的地图 yaml）之后，**需要显示器或 VNC**（脚本会打开图像窗口）：

```bash
python3 ~/nav_ws/map_point_picker.py ~/Desktop/map/11_map.yaml
```

- **作用**：打开对应栅格图，**在 figure 上点击**即可得到该点相对**地图原点**的坐标（用于填写 **`BT_START_GOAL` / `BT_END_GOAL`** 中的 **`x;y`**，或修改 XML 里写死的 `SendGoal`）。  
- **赛场图**可把路径换成 **`…/rm_nav_bringup/map/RMUL2026.yaml`**，在同一套坐标系下标定。  
- 若 SSH 无转发图形，请在 **本机桌面或 VNC 会话**里执行。

---

## 8. 开机自启：`autostart_fullstack.sh`

```bash
bash /home/nyu/sentry_planner/scripts/autostart_fullstack.sh
```

### 8.1 做什么

1. 清理旧进程与 PTY 软链（可选）。  
2. **`systemctl --user restart sentry_bridge.service`**（bridge 在 systemd 里，**仍应用自动选口或你在 service 里写的参数**）。  
3. 等待 **`/tmp/nyush-rm-sentry-vision`** 出现。  
4. 启动 **`just test detect --web --send`**（有图形用 xterm，无则 nohup 写日志）。  
5. 清理 Nav/LIO 残留后，在 **`~/nav_ws`** 用一组固定环境变量跑 **`./start_robot.sh`**（含 `MAP_FILE`、`BT_STYLE=center_attack_fullstack`、`RADAR_PTY=/tmp/nyush-rm-sentry-radar` 等，与脚本内硬编码一致）。

### 8.2 日志

- `/home/nyu/sentry_planner/logs/autostart/` 下 **`vision_detect.log`**、**`nav_bt.log`** 等。

### 8.3 场上用途

无笔记本时 **一条命令拉起 bridge（经 systemd）+ 视觉 + Nav + BT + sender**，与手动多终端等价。

---

## 9. 脚本与节点功能对照表

| 组件 | 路径或命令 | 通讯 | 导航 | 视觉 | BT |
|------|------------|------|------|------|-----|
| `sentry_bridge.py` | `just sentry-bridge` | **核心**：占 MCU 口，拆 Vision/Radar PTY | — | PTY | — |
| `serial_sender.py --ros2` | nyush-rm-vision | Radar PTY ↔ ROS 话题 | 消费 `/cmd_vel_chassis_bt` 等 | — | 消费 `/robot_control` |
| `bt_comm_adapter.py` | sentry_planner/scripts | 产出 `/cmd_vel_chassis_bt` | 衔接 `/cmd_vel_chassis` | 可选 armors 适配 | 衔接 `/robot_control` |
| `rm_behavior_tree` | rm_decision_ws | — | `SendGoal` | 可选订阅 armors | **决策核心** |
| `fake_vel_transform` | nav_ws 包 | — | `/cmd_vel`→`/cmd_vel_chassis` | — | — |
| `watch_center_attack_state.py` | scripts | 监视 `cmd_vel_chassis_bt`、`robot_control` | amcl 位姿 | — | 监视裁判类输入 |
| `bt_hotkey_debug.py` | scripts | — | — | — | **伪造** `game`/`robot` 状态 |
| `autostart_fullstack.sh` | scripts | 间接启动 bridge 服务 + sender | 启动 `nav_ws/start_robot.sh` | 启动 detect | 启动 BT |

---

## 10. 速查：最小手动终端组合

| 终端 | 内容 |
|------|------|
| 1 | `just sentry-bridge`（自动选口） |
| 2（可选） | `just test detect --web --send`（Vision PTY） |
| 3 | `RADAR_PTY=/tmp/nyush-rm-sentry-radar … ./start_robot.sh`（nav_ws 或 sentry_planner，按你现场脚本） |

---

## 11. `communication command.txt` 里易混点

- **Radar mock**：`just radar --port /tmp/nyush-rm-sentry-radar` 测的是 **PTY**，不是强制 ACM0。  
- **串口权限**：`nav_ws/start_robot.sh` 里仍有对 **`/dev/ttyACM0`** 的 `chmod`；若自动选口选到 **ACM1**，需自行 `chmod` 或 udev 规则，与 bridge **选口策略**是两件事。

---

## 12. 实机联调

### 12.1 分阶段策略（结论）

不必一开始就占完整比赛场：

| 阶段 | 目标 | 是否强依赖「与地图一致的场地」 |
|------|------|----------------------------------|
| **A — 台架** | bridge、PTY、`serial_sender`、底盘速度响应、BT 分支、`/robot_control` 与 `/cmd_vel_chassis_bt` 是否有输出 | **否** |
| **B — 小范围低速地面** | 定位稳定、短距离移动、`Home` 类点是否在自由空间 | **是**（环境需与所用地图大体一致） |
| **C — 真实场地** | 去中心、守中、低血回家；评价 Nav2 参数与战术 | **是**（赛场图如 **`RMUL2026`**） |

没有匹配环境时，可以调通讯与 BT 逻辑，**不要轻易下结论「实机导航已完全 OK」**。定位链、代价地图与 `navigate_to_pose` 前提见 [README_LIDAR.md](README_LIDAR.md) **§10.8**。

### 12.2 上实机前安全（最低限度）

- 枪口安全，必要时先断发射；首轮联调关闭不必要的自动开火。  
- 底盘首次测试先架空或**限速**，场边有人监护，急停手段明确。  

### 12.3 `start_robot.sh` 与 bridge、sender

- **`start_robot.sh`（`nav_ws` 或 `sentry_planner`）默认不会启动 `sentry_bridge`**；bridge 仍建议**单独终端**或 systemd。  
- 要把 **`/cmd_vel_chassis_bt`** 与 **`/robot_control`** 写进下位机链路，需 **`START_SERIAL_SENDER=1 RADAR_PTY=<Radar PTY>`**（与 **§4** 一致）。  
- 当前实机主线里，**`/cmd_vel_chassis_bt`** 由 Nav2 底盘速度与 **`RobotControl.chassis_spin_vel`** 经 **`bt_comm_adapter`** 合成，再经 **sender → Radar PTY → MCU**；**`/robot_control`** 的 **`scan_*`、`allow_vision_control`** 等经 **A3 → MCU** 已可落到云台状态机（协议细节见 [README_COMMUNICATION.md](README_COMMUNICATION.md)）。

### 12.4 推荐终端分工（手动多终端）

路径按你本机仓库位置改写。

| 终端 | 内容 |
|------|------|
| **1 — 导航/定位** | 例：`ros2 launch rm_nav_bringup bringup_real.launch.py world:=<地图前缀> mode:=nav …`（`world` 与 **`MAP_FILE`/地图包**一致；定位方式按你现场 `lio` / `localization` 参数） |
| **2 — bridge** | `cd nyush-rm-control && just sentry-bridge`（或 `--port /dev/ttyACM0`）；记下 **Vision / Radar PTY**，优先用 **`/tmp/nyush-rm-sentry-radar`** 等软链 |
| **3 — sender** | `source /opt/ros/humble/setup.bash` 后 `python3 …/nyush-rm-vision/serial_sender.py --port <Radar PTY> --ros2 --topic /cmd_vel_chassis_bt`；若已用 **§4** 的 **`START_SERIAL_SENDER=1`** 则可不单独开 |
| **4 — BT 调试** | `bash sentry_planner/scripts/run_center_attack_debug_session.sh`（保活 **`bt_comm_adapter`**、**`rm_behavior_tree`**、**`watch_center_attack_state.py`**）；**须先**有 Nav2 且 **`navigate_to_pose` action 在线** |
| **5 — 视觉（可选）** | `nyush-rm-vision` 启动 sentry；**`configs/sentry.yaml` 的 `com_port` 必须指向当前 Vision PTY** |

### 12.5 最小检查清单

上电后、起 BT 前：

```bash
ros2 action list | grep navigate_to_pose
ros2 topic echo /robot_control --once
ros2 topic echo /cmd_vel_chassis_bt --once
```

若 **`navigate_to_pose`** 不在线，先不要指望 **`SendGoal`** 能闭环。

### 12.6 伪造裁判话题（调 BT 分支）

（需已 `source rm_decision_ws/install/setup.bash`。）

**比赛中 + 正常血量 → 期望 watcher 侧接近 `APPROACH_CENTER`，`/cmd_vel_chassis_bt` 有输出：**

```bash
ros2 topic pub -r 1 /game_status rm_decision_interfaces/msg/GameStatus \
  "{game_progress: 4, stage_remain_time: 220}"
ros2 topic pub -r 10 /robot_status rm_decision_interfaces/msg/RobotStatus \
  "{robot_id: 7, current_hp: 600, shooter_heat: 0, team_color: false, is_attacked: false}"
```

**低血 → 期望 `HOME_RECOVER`、回 `Home`：**

```bash
ros2 topic pub -r 10 /robot_status rm_decision_interfaces/msg/RobotStatus \
  "{robot_id: 7, current_hp: 200, shooter_heat: 0, team_color: false, is_attacked: false}"
```

**未开赛 → 期望 `HOME_STANDBY`、不主动冲中心：**

```bash
ros2 topic pub -r 1 /game_status rm_decision_interfaces/msg/GameStatus \
  "{game_progress: 0, stage_remain_time: 220}"
```

若同时开着 **`serial_sender` 发布真实裁判**，会与上述 **抢同一话题**；调试时可 **`SERIAL_SENDER_DISABLE_STATUS_PUB=1`**（见 **§5**）。

### 12.7 何时算「可以去完整场地」

至少：**bridge / sender 稳定**、**实机定位稳定**、**`navigate_to_pose` 正常**、**`APPROACH_CENTER` / `HOME_RECOVER` / `HOME_STANDBY` 分支能切对**、**`Home` / 中心点在所用地图上处于自由空间**（换图与坐标测定见 **§4.4**、**§7.4**）。

### 12.8 已知易错点

- **PTY 每次 bridge 重启可能变**：勿把旧的 **`/dev/pts/N`** 写死忘改；见 [README_COMMUNICATION.md](README_COMMUNICATION.md) **§15.2**。  
- **`watch_center_attack_state.py` 默认 `--home-x/y`、`--center-x/y` 可能与 XML 中 `SendGoal` 坐标不一致**，实机请看 [README_BEHAVIOR_TREE_FLOW.md](README_BEHAVIOR_TREE_FLOW.md) **§5.1**，用命令行参数与树内点对齐。

---

## 13. 四专题分工速查（我该打开哪份 README？）

| 你的问题 | 优先打开的文档 |
|----------|----------------|
| 串口谁占、PTY 是啥、19 字节 / A3 / SP、话题从哪来 | [README_COMMUNICATION.md](README_COMMUNICATION.md) |
| 行为树跑哪棵 XML、`SendGoal`/`RobotControl`、Groot2、热键 | [README_BEHAVIOR_TREE_FLOW.md](README_BEHAVIOR_TREE_FLOW.md) |
| **Gazebo RMUL2026、Sim2Real、Groot AppImage 与 `bringup_sim`** | [README_LIDAR.md §6.4](README_LIDAR.md#nyush-gazebo-sim2real) + [mid360 command.txt](mid360%20command.txt) |
| Mid360、FAST-LIO、Nav2 参数、点云、代价地图、TF | [README_LIDAR.md](README_LIDAR.md) |
| 一键命令、`MAP_FILE`、建图、`start_robot` 变量、实机终端顺序 | **本文**（§1、§4、§7、§12） |
| 仍不确定 | [README.md](README.md) 中央索引 |

---

如需把某条命令嵌进 systemd 或改 `autostart_fullstack.sh` 内 `MAP_FILE` / BT 目标点，直接改脚本内对应环境变量块即可。
