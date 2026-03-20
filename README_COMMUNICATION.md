# NYUSH 哨兵通讯链路与协议说明

最后更新：2026-03-20

本文专门整理 NYUSH 哨兵当前的通讯链路、协议格式、桥接方式、启动顺序和排障方法。

这份文档覆盖 4 个代码源：

- 当前仓库：`/home/nyu/sentry_planner`
- 下位机电控：`/home/nyu/Codespace/nyush-rm-control`
- 上位机视觉：`/home/nyu/Codespace/nyush-rm-vision`
- 旧雷达串口发送脚本：`/home/nyu/Codespace/nyush-rm-vision/serial_sender.py` 与 `/home/nyu/Desktop/serial_sender.py`

## 1. 先说结论

当前最推荐、也最清晰的结构是：

```text
nyush-rm-vision <-> Vision PTY <-> sentry_bridge.py <-> /dev/ttyACM0 <-> nyush-rm-control
雷达/导航侧     <-> Radar  PTY <-> sentry_bridge.py <-> /dev/ttyACM0 <-> nyush-rm-control
```

要点只有 4 个：

1. 真实硬件口 `/dev/ttyACM0` 只能被一个进程占用。
2. 这个唯一占口的进程应该是 `nyush-rm-control/scripts/sentry_bridge.py`。
3. 视觉程序不再直连 `/dev/ttyACM0`，而是连桥接脚本打印出来的 `Vision PTY`。
4. 雷达/导航程序也不再直连 `/dev/ttyACM0`，而是连桥接脚本打印出来的 `Radar PTY`。

如果多个进程同时直接打开 `/dev/ttyACM0`，结果通常就是：

- 视觉和雷达互相抢串口
- 帧头交叉污染
- 一个程序能收，另一个程序完全超时
- 偶发能跑，重启后就不行

## 2. 角色对照

### 2.1 谁是谁

- `nyush-rm-control`
  - 下位机电控
  - 运行在 STM32 C 板
  - 负责底盘、云台、射击、裁判系统、本地状态机
- `nyush-rm-vision`
  - 上位机视觉
  - 运行在小电脑
  - 负责装甲板识别、跟踪、瞄准、发射决策、部分 ROS2 预留接口
- `sentry_planner`
  - 导航 + 行为树决策 + 旧 ROS2 串口链路
  - 负责 Nav2、BehaviorTree、激光雷达定位、旧 `rm_serial_driver`
- `serial_sender.py`
  - 旧雷达/导航串口转发脚本
  - 本质是 “ROS2 `/cmd_vel` 或键盘 -> 串口帧”
  - 不属于视觉主链本身

### 2.2 当前最重要的边界

- 视觉和雷达都不应该直接占用真实 MCU 口。
- 桥接脚本是唯一串口所有者。
- 下位机看到的是一根口，但桥接脚本在逻辑上拆成了两条链：
  - 视觉链：`SP`
  - 哨兵扩展链：`SX/ST`

## 3. 一图看懂

### 3.1 当前建议链路

```text
                      +---------------------------+
                      |   nyush-rm-control MCU    |
                      |  SP + SX/ST on one CDC    |
                      +-------------+-------------+
                                    ^
                                    |
                              /dev/ttyACM0
                                    |
                     +--------------+--------------+
                     |         sentry_bridge.py     |
                     |  own real port, split PTYs   |
                     +--------------+--------------+
                                    |
                 +------------------+------------------+
                 |                                     |
            Vision PTY                            Radar PTY
                 |                                     |
     +-----------+-----------+             +-----------+-----------+
     |     nyush-rm-vision   |             |  radar/nav side tool  |
     |   SP send/recv only   |             |  legacy radar frames  |
     +-----------------------+             +-----------------------+
```

### 3.2 ROS2 侧链路

```text
Nav2 -> /cmd_vel -> fake_vel_transform -> /cmd_vel_chassis -> 雷达侧发送器 -> Radar PTY

视觉检测 -> /detector/armors -> 行为树
行为树 -> /goal_pose -> Nav2
行为树 -> /robot_control -> 串口驱动/下位机
```

## 4. 推荐阅读顺序

如果你要读源码，建议按这个顺序读：

1. `nyush-rm-control/scripts/sentry_bridge.py`
   - 先看桥接脚本如何拆 Vision/Radar 两个 PTY
2. `nyush-rm-control/modules/master_machine/master_process.h`
   - 先看协议结构体定义和长度
3. `nyush-rm-control/modules/master_machine/master_process.c`
   - 再看 MCU 侧如何解包 `SP` / `SX`，以及如何回发 `SP` / `ST`
4. `nyush-rm-vision/io/gimbal/gimbal.hpp`
   - 看视觉侧串口收发结构体
5. `nyush-rm-vision/io/gimbal/gimbal.cpp`
   - 看视觉如何读 `com_port`、如何发 `SP`
6. `sentry_planner/rm_navigation_ws/src/rm_navigation/fake_vel_transform/src/fake_vel_transform.cpp`
   - 看 Nav2 的 `/cmd_vel` 如何变成 `/cmd_vel_chassis`
7. `sentry_planner/rm_decision_ws/rm_behavior_tree/plugins/action/sub_armors.cpp`
   - 看行为树如何吃视觉检测
8. `sentry_planner/rm_decision_ws/rm_behavior_tree/plugins/condition/is_detect_enemy.cpp`
   - 看当前决策侧实际只用了“是否看到敌人”
9. `nyush-rm-vision/tasks/omniperception/decider.cpp`
   - 看视觉发给导航的目标信息长什么样
10. `nyush-rm-vision/io/ros2/publish2nav.cpp`
   - 看视觉 ROS2 发布端到底发了什么 topic

## 5. 真实口与虚拟口的所有权

### 5.0 命令在哪个仓库执行

这套哨兵链路会同时用到 `nyush-rm-control` 和 `nyush-rm-vision` 两个仓库，它们的 `justfile` 不是同一个，所以命令不能混着跑。

- 在 `nyush-rm-control` 里执行：
  - `just bridge` / `just sentry-bridge`
  - `just logger`
  - `just logger-cli`
  - `just vision`
- 在 `nyush-rm-vision` 里执行：
  - `just test detect --web --send`
  - 其他 `just test ...`

如果仓库跑错了，看到下面这种报错是正常现象：

```text
error: Justfile does not contain recipe `logger`
error: Justfile does not contain recipe `test`
```

不要先怀疑命令本身，先确认当前目录是不是对的。

### 5.1 唯一原则

真实口 `/dev/ttyACM0` 只能被下面这一个进程占用：

```bash
cd /home/nyu/Codespace/nyush-rm-control
just sentry-bridge --port /dev/ttyACM0
```

桥启动后会打印类似：

```text
MCU serial : /dev/ttyACM0
Vision PTY : /dev/pts/3
Radar PTY  : /dev/pts/4
Press Ctrl+C to stop.
```

此后：

- 视觉程序连 `/dev/pts/3`
- 雷达/导航程序连 `/dev/pts/4`

### 5.2 为什么一定要桥接

因为下位机现在已经支持在一根 CDC 上同时收发两类消息：

- 视觉 `SP`
- 哨兵扩展 `SX/ST`

而桥接脚本做的事情就是：

- 视觉侧透传 `SP`
- 雷达侧旧协议转成 `SX`
- MCU 回发 `ST` 再转回雷达侧旧遥测帧

## 6. 协议总览

### 6.1 基本约定

- 多字节数值默认按当前代码中的小端序处理
- 雷达侧旧协议使用 CRC8
- `SP` / `SX` / `ST` 使用当前代码实现的 CRC16
- 如果你要自己写外部工具，不要手猜 CRC，直接复用现有实现

推荐直接参考：

- `nyush-rm-control/scripts/sentry_bridge.py`
- `nyush-rm-control/modules/master_machine/master_process.h`
- `nyush-rm-vision/io/gimbal/gimbal.hpp`

## 7. SP 视觉协议

`SP` 是视觉和下位机之间的主协议。

### 7.1 下位机 -> 视觉：`GimbalToVision`，43 字节

定义来源：

- `nyush-rm-control/modules/master_machine/master_process.h`
- `nyush-rm-vision/io/gimbal/gimbal.hpp`

帧结构：

```text
head[2] = 'S','P'
mode        uint8
q[4]        float[4]
yaw         float   rad
yaw_vel     float   rad/s
pitch       float   rad
pitch_vel   float   rad/s
bullet_speed float  m/s
bullet_count uint16
crc16       uint16
```

字段说明：

- `mode`
  - 0: IDLE
  - 1: AUTO_AIM
  - 2: SMALL_BUFF
  - 3: BIG_BUFF
- `q[4]`
  - 四元数，顺序 `w x y z`
- `yaw/pitch`
  - 当前云台姿态
- `bullet_speed`
  - 当前弹速
- `bullet_count`
  - 当前累计发弹计数

### 7.2 视觉 -> 下位机：`VisionToGimbal`，29 字节

帧结构：

```text
head[2] = 'S','P'
mode         uint8
yaw          float   rad
yaw_vel      float   rad/s
yaw_acc      float   rad/s^2
pitch        float   rad
pitch_vel    float   rad/s
pitch_acc    float   rad/s^2
crc16        uint16
```

字段说明：

- `mode`
  - 0: 不控制
  - 1: 控制云台，不开火
  - 2: 控制云台，且允许开火

重要说明：

- 这条 `SP` 上行包当前只包含云台控制量和开火模式。
- 它不直接携带“目标类别”。
- 下位机内部虽然有 `target_type` 字段定义，但当前 `ApplyVisionPacket()` 明确写的是“视觉端未提供此数据”，会把它置成 `NO_TARGET_NUM`。

也就是说：

- 当前串口主链上，视觉到下位机传的是“怎么转、要不要开火”
- 不是“我现在看到的是步兵还是哨兵”

## 8. SX / ST 哨兵扩展协议

这是桥接雷达侧和哨兵底盘控制的关键。

### 8.1 雷达侧/桥 -> MCU：`SX`，33 字节

定义来源：

- `nyush-rm-control/modules/master_machine/master_process.h`
- `nyush-rm-control/modules/master_machine/master_process.c`
- `nyush-rm-control/scripts/sentry_bridge.py`

帧结构：

```text
head[2] = 'S','X'
vx                  float
vy                  float
wz                  float
gimbal_yaw_delta    float
gimbal_pitch_delta  float
control_flags       uint8
scan_yaw_rate_deg_s float
search_pitch_deg    float
crc16               uint16
```

其中 `control_flags` 当前使用：

- bit0: `scan_control_valid`
- bit1: `stop_gimbal_scan`（旧兼容位）
- bit2: `scan_enabled`
- bit3: `allow_vision_control`
- bit4: `search_when_target_lost`

MCU 收到后会落到：

- `sentry_ext.vx`
- `sentry_ext.vy`
- `sentry_ext.wz`
- `sentry_ext.gimbal_yaw_delta`
- `sentry_ext.gimbal_pitch_delta`
- `sentry_ext.control_flags`
- `sentry_ext.scan_yaw_rate_deg_s`
- `sentry_ext.search_pitch_deg`

随后在 `robot_cmd.c` 中被应用到：

- `chassis_cmd_send.vx`
- `chassis_cmd_send.vy`
- `chassis_cmd_send.wz`
- 云台扫描 / 自瞄接管 / 丢目标回扫状态机

### 8.2 MCU -> 雷达侧/桥：`ST`，27 字节

帧结构：

```text
head[2]            = 'S','T'
cmd_vx             float
cmd_vy             float
cmd_wz             float
robot_status       uint8
game_status        uint8
stage_remain_time  uint16
robot_id           uint8
current_hp         uint16
shooter_heat       uint16
team_color         uint8
is_attacked        uint8
crc16              uint16
```

它表示 MCU 当前实际采用的底盘命令以及一组精简裁判系统状态：

- `robot_status` 仍然是裁判系统电源管理输出位压成的 bitfield
- `game_status / stage_remain_time / robot_id / current_hp / shooter_heat / team_color / is_attacked` 现在也随 `ST` 一起从 MCU 回来

其中 `is_attacked` 是下位机根据 HP 下降做的短时锁存位，不是裁判系统原始字段原样透传。

### 8.3 当前桥接脚本对 `SX` 的实际用法

桥接脚本现在是这样构造 `SX` 的：

- `vx/vy/wz` 来自雷达侧速度输入
- `gimbal_yaw_delta = 0.0`
- `gimbal_pitch_delta = 0.0`
- `control_flags / scan_yaw_rate_deg_s / search_pitch_deg` 来自 `A3 RobotControl` 功能帧

也就是说，桥现在同时解决的是：

- 底盘速度透传
- 云台扫描 / 自瞄接管功能量透传

当前仍然没有做的是：

- 雷达侧云台微调透传

## 9. 雷达侧旧协议

桥接脚本对雷达侧暴露的是一套“旧雷达工具友好”的帧。

### 9.1 雷达侧命令帧：19 字节

桥接脚本当前期待的雷达输入格式是：

```text
[0xA5][0x5A][vx:4][vy:4][wz:4][yaw_deg:4][CRC8:1]
```

总长度：19 字节

用途：

- `vx/vy/wz` 会被桥转成 `SX`
- `yaw_deg` 当前桥脚本收了，但没有继续映射到 `SX` 的云台增量字段

### 9.2 雷达侧遥测帧：19 字节 + 状态包

桥接脚本回给 Radar PTY 的现在不只是一种帧，而是三类：

```text
A6 6A + vx + vy + wz + reserved0 + crc8
5C    + game_progress + stage_remain_time + crc16
5D    + robot_id + current_hp + shooter_heat + team_color + is_attacked + crc16
```

其中：

- `A6 6A` 仍然是旧雷达工具兼容的速度遥测
- `0x5C` 和 `0x5D` 是桥根据 MCU 的 `ST` 新补发的状态包
- `serial_sender.py --ros2` 现在会消费这两个状态包，并发布 `/game_status` 和 `/robot_status`

## 10. `serial_sender.py` 与桥接雷达协议的对齐状态

现在 `nyush-rm-vision/serial_sender.py` 会同时发送两类桥接帧：

- 19 字节雷达速度帧
- 16 字节 `A3 RobotControl` 功能帧

### 10.1 当前 `serial_sender.py` 的速度帧仍是 19 字节

桥接脚本的 `RADAR_FRAME_SIZE` 现在是 19，并按：

```text
[0xA5][0x5A][vx:4][vy:4][wz:4][yaw_deg:4][CRC8:1]
```

来解包；`serial_sender.py` 现在也按这一格式编码。

### 10.2 字段说明

- `vx/vy/wz`
  - 雷达侧给底盘的速度指令
- `yaw_deg`
  - 当前作为兼容字段保留
  - 发送端会把旧接口里的 `gimbal_yaw_rad` 转成角度后写入这里
  - 桥接脚本目前仍然只真正使用 `vx/vy/wz`
- `gimbal_pitch_rad`
  - 不在 19 字节协议里
  - `serial_sender.py` 为兼容旧函数签名仍保留该参数，但发送时会忽略

### 10.3 结论

- `nyush-rm-vision/serial_sender.py` 现在可以直接接到 `Radar PTY`
- `/home/nyu/Desktop/serial_sender.py` 如果是从旧版本单独拷出来的，也要和仓库版本保持同步
- 桥接思路现在已经打通：
  - `nyush-rm-vision` 走 `Vision PTY`
  - 雷达/导航发送器走 `Radar PTY`
  - 真实硬件口仍然只由 `sentry_bridge.py` 占用

当前已经变化的是：

- 桥接脚本除了把 `vx/vy/wz` 写入 `SX`，还会把 `A3 RobotControl` 里的 `scan_enabled / allow_vision_control / search_when_target_lost / scan_yaw_rate_deg_s / search_pitch_deg` 一起写入 `SX`
- `serial_sender.py --ros2` 会同时订阅 `/cmd_vel_chassis_bt` 和 `/robot_control`
- `serial_sender.py --ros2` 还会把 bridge 回来的 `0x5C/0x5D` 状态包发布成 `/game_status` 和 `/robot_status`
- `stop_gimbal_scan` 仍然兼容，但已经降级成旧树兼容位
- 云台增量微调 `yaw_delta/pitch_delta` 这条雷达侧直通链仍未启用

## 11. ROS2 话题链路

### 11.1 导航到底盘

当前 `sentry_planner` 的速度链路是：

```text
Nav2 -> /cmd_vel -> fake_vel_transform -> /cmd_vel_chassis -> bt_comm_adapter.py -> /cmd_vel_chassis_bt
```

其中：

- `/cmd_vel`
  - Nav2 原始速度输出
- `/cmd_vel_chassis`
  - 经过坐标系转换后的底盘速度
- `/cmd_vel_chassis_bt`
  - `bt_comm_adapter.py` 合并 `/cmd_vel_chassis` 与 `/robot_control.chassis_spin_vel` 后的输出

如果要接桥接雷达口，当前最合适的输入就是 `/cmd_vel_chassis_bt`。

当前默认还额外做了一件事：

- `fake_vel_transform` 默认忽略 Nav2 原始 `wz`
- `bt_comm_adapter.py` 默认也忽略 `/cmd_vel_chassis` 自带的 `angular.z`
- 也就是说，当前链路默认先只走 `vx/vy`
- 如果后面需要恢复 Nav2 角速度，再把 `use_nav_wz` / `use_nav_cmd_wz` 打开

### 11.2 决策到视觉

当前行为树订阅：

- `/detector/armors`
- `/game_status`
- `/robot_status`
- `/all_robot_hp`

其中：

- `/game_status` 和 `/robot_status` 在当前实机主线下，默认由 `serial_sender.py --ros2` 根据 bridge 回传的真实裁判系统状态发布
- `bt_hotkey_debug.py` 仍然可以人工持续发布这两个话题做调试，但那是覆盖/伪造输入，不是默认上游
- 和视觉最直接相关的仍然是 `/detector/armors`

当前 `IsDetectEnemy` 的逻辑很简单，只看：

- `armors` 数组是不是空

也就是说当前决策侧并没有按“英雄 / 工程 / 步兵 / 哨兵 / 前哨站”做细分分流。

### 11.3 视觉到导航/决策的预留口

`nyush-rm-vision` 里已经预留了一条 ROS2 发布：

- topic: `auto_aim_target_pos`
- 类型：`std_msgs/String`
- 内容格式：`x,y,z,target_id`

其中第四个值 `target_id` 来自目标类别，当前代码里是：

- `armor.name + 1`

这意味着：

- 视觉侧已经具备“把目标类别发出去”的基础
- 但当前 `sentry_planner` 还没有实际消费这个 topic

## 12. 启动方式

本节分成 3 套：

- 桥接自检
- 视觉链自检
- 当前可落地的实机启动顺序

### 12.1 桥接自检

终端 1：启动桥接

```bash
cd /home/nyu/Codespace/nyush-rm-control
just py-bootstrap
just sentry-bridge --self-test
just sentry-bridge --port /dev/ttyACM0
```

看到类似输出：

```text
MCU serial : /dev/ttyACM0
Vision PTY : /dev/pts/3
Radar PTY  : /dev/pts/4
```

记下两个 PTY。

#### 下位机 dashboard

如果你想同时看 C 板 dashboard，而不是只看桥接日志，可以在 `nyush-rm-control` 再开一个终端：

```bash
cd /home/nyu/Codespace/nyush-rm-control
just logger
```

然后访问：

```text
http://127.0.0.1:8080
```

注意这页是下位机 RTT dashboard，不是视觉检测网页。

### 12.2 视觉协议自检

终端 2：直接用交互工具连 `Vision PTY`

```bash
cd /home/nyu/Codespace/nyush-rm-control
just vision --port /dev/pts/3
```

这个工具可以：

- 读 MCU 下发的 `SP`
- 手动发 `VisionToGimbal`

它非常适合在视觉程序没跑起来前先确认：

- 桥接没问题
- MCU 在正常回帧
- CRC、帧长、模式位都是通的

### 12.3 雷达协议自检

终端 3：用 mock 工具连 `Radar PTY`

```bash
cd /home/nyu/Codespace/nyush-rm-control
just radar --port /dev/pts/4
```

进入交互后可用：

```text
set vx vy wz [yaw_deg]
stop
rate hz
state
pulse vx vy wz sec
```

用途：

- 验证桥能否把雷达侧帧转成 `SX`
- 验证 MCU 是否回 `ST`
- 验证底盘命令是否被真正应用

### 12.4 视觉主线接桥启动

先确保视觉程序连到桥接脚本给出的 `Vision PTY`。

推荐优先使用桥创建的稳定软链接；如果你现场看到的是动态 `PTY`，就把对应路径写进 vision 配置。

当前实机联调最常用、也最推荐的视觉命令是：

```bash
cd /home/nyu/Codespace/nyush-rm-vision
just test detect --web --send
```

注意：

- 这条命令要在 `nyush-rm-vision` 里执行，不在 `nyush-rm-control` 里执行。
- 它会启动视觉检测测试，并把控制量通过当前 `com_port` 发给下位机。
- 默认视觉网页是：

```text
http://127.0.0.1:8888
```

- 如果要真正进入视觉接管，遥控器左边开关需要拨到中间。
- 当前 `detect` 主线是“看到目标才跟随”，不是完整自动搜索主程序。

如果你后面确实要跑视觉仓库里的完整 `sentry` 主程序，再单独确认：

- `sentry` target 已经编出来
- `configs/sentry.yaml` 的 `com_port` 指向当前 `Vision PTY`

但对当前这套 bridge + BT + 自瞄联调来说，不再建议把 `./build/sentry configs/sentry.yaml` 当作默认主线。

### 12.5 完整实机启动流程（当前推荐）

本节默认你要跑的是这条完整链路：

- `nyush-rm-control` 负责 bridge 和可选 dashboard
- `nyush-rm-vision` 负责 `detect --web --send`
- `sentry_planner` 负责 Mid360 + FAST-LIO + Nav2 + 行为树 + `bt_comm_adapter.py`
- `serial_sender.py` 通过 `Radar PTY` 把 `/cmd_vel_chassis_bt` 和 `/robot_control` 发到下位机
- 同一个 `serial_sender.py` 进程还会把 bridge 回传的真实裁判系统状态发布成 `/game_status` 和 `/robot_status`

开始之前，先确认：

- 下位机已经刷入最新哨兵固件
- 不要再启动 `rm_serial_driver` 抢 `/dev/ttyACM0`
- 如果你当前 shell 是 `bash`，就统一使用 `*.bash` 环境脚本
- `start_robot.sh` 默认不自动开 RViz；只有在本机有图形界面时再加 `ENABLE_RVIZ=1`

推荐直接按下面 5 个终端启动。

终端 1：桥接下位机

```bash
cd /home/nyu/Codespace/nyush-rm-control
just sentry-bridge --port /dev/ttyACM0
```

你需要记住桥打印出来的：

```text
Vision PTY : /dev/pts/X
Radar PTY  : /dev/pts/Y
```

如果桥同时创建了稳定软链接，优先用：

```text
/tmp/nyush-rm-sentry-vision
/tmp/nyush-rm-sentry-radar
```

终端 1B：可选，下位机 dashboard

```bash
cd /home/nyu/Codespace/nyush-rm-control
just logger
```

浏览器访问：

```text
http://127.0.0.1:8080
```

终端 2：视觉自瞄主线

```bash
cd /home/nyu/Codespace/nyush-rm-vision
just test detect --web --send
```

浏览器访问：

```text
http://127.0.0.1:8888
```

终端 3：导航 + 行为树 + 串口发送

```bash
bash
cd /home/nyu/sentry_planner
START_SERIAL_SENDER=1 RADAR_PTY=/tmp/nyush-rm-sentry-radar ./start_robot.sh
```

如果你的 bridge 没有创建稳定软链接，就把 `RADAR_PTY` 换成桥刚打印出来的 `Radar PTY`，例如：

```bash
START_SERIAL_SENDER=1 RADAR_PTY=/dev/pts/4 ./start_robot.sh
```

这条脚本默认使用：`/home/nyu/sentry_planner/rm_navigation_ws/src/rm_nav_bringup/map/RMUL2026.yaml`。如果你现场临时要切别的图，再手动覆盖 `MAP_FILE=...`。

这条脚本现在会自动拉起：

- `livox_ros_driver2`
- 静态 TF
- `fast_lio`
- `pointcloud_to_laserscan`
- `nav2_bringup`
- `bt_comm_adapter.py`
- `rm_behavior_tree`
- 可选的 `serial_sender.py`

如果你本机有显示器并且想看 Nav2 RViz，用：

```bash
ENABLE_RVIZ=1 START_SERIAL_SENDER=1 RADAR_PTY=/tmp/nyush-rm-sentry-radar ./start_robot.sh
```

终端 4：可选，键盘一键切行为树分支

```bash
bash
source /opt/ros/humble/setup.bash
source /home/nyu/sentry_planner/rm_decision_ws/install/setup.bash
python3 /home/nyu/sentry_planner/scripts/bt_hotkey_debug.py
```

可用按键：

- `0`：`HOME_STANDBY`
- `1`：`APPROACH_CENTER`
- `2`：低血量恢复
- `3`：高热量恢复
- `4`：被攻击分支
- `p`：打印当前预设
- `h`：帮助
- `q`：退出

注意：这个热键脚本能切游戏/机器人状态，但 `CENTER_HOLD_ATTACK` 这类“已经到中点”的分支仍然取决于真实 `map -> base_link` 位姿，不是单靠热键伪造。

终端 5：可选，Groot2 观察树状态

```bash
cd ~/Desktop
./Groot2-v1.9.0-x86_64.AppImage
```

然后：

- 打开 `/home/nyu/sentry_planner/rm_decision_ws/rm_behavior_tree/config/Project.btproj`
- 连接 `127.0.0.1:1667`

### 12.6 实机联调时最常看的检查点

最小检查清单：

```bash
lsof /dev/ttyACM0
ros2 topic echo /robot_control --once
ros2 topic echo /cmd_vel_chassis_bt --once
ros2 topic echo /game_status --once
ros2 topic echo /robot_status --once
ros2 topic echo /detector/armors --once
```

如果你现在就是要确认真实裁判系统是否已经进 ROS，再额外看：

```bash
ros2 topic hz /game_status
ros2 topic hz /robot_status
```

理想状态：

- `/dev/ttyACM0` 只有 `sentry_bridge.py` 占用
- `/robot_control` 能看到 `scan_enabled / allow_vision_control / search_when_target_lost` 等功能字段
- `/cmd_vel_chassis_bt` 能看到底盘导航速度和小陀螺角速度
- `/game_status` 和 `/robot_status` 在裁判系统在线时不再需要手工 `ros2 topic pub`
- `/detector/armors` 在视觉看到目标时非空

车上操作建议：

- 上电后先确认右拨杆在上位，整车处于 `READY`
- 真正要允许视觉接管时，把左拨杆拨到中位
- 如果你只是验证 BT/导航链路，可以先不进视觉接管
- 当前 `detect --web --send` 不是完整自动搜索主程序；它主要负责“检测到目标后跟随”

### 12.7 ROS2 Topic 手动调试（不依赖整套行为树）

如果你现在只是想验证 `serial_sender -> bridge -> MCU`，不一定要先启动 `start_robot.sh`。

最小前置条件：

- `nyush-rm-control` 里先跑 `just sentry-bridge --port /dev/ttyACM0`
- 单独跑仓库版 `serial_sender.py --ros2 --topic /cmd_vel_chassis_bt`
- 机器人不在急停，右拨杆不要放中位自转档

最小底盘接管测试：

终端 1，持续发布 BT 功能接管：

```bash
source /opt/ros/humble/setup.zsh
source /home/nyu/sentry_planner/rm_decision_ws/install/setup.zsh
ros2 topic pub -r 20 /robot_control rm_decision_interfaces/msg/RobotControl \
"{stop_gimbal_scan: false, chassis_spin_vel: 0.0, scan_enabled: true, allow_vision_control: false, search_when_target_lost: false, scan_yaw_rate_deg_s: 90.0, search_pitch_deg: 0.0}"
```

终端 2，持续发布底盘速度：

```bash
source /opt/ros/humble/setup.zsh
ros2 topic pub -r 20 /cmd_vel_chassis_bt geometry_msgs/msg/Twist \
"{linear: {x: 0.20, y: 0.00, z: 0.00}, angular: {x: 0.00, y: 0.00, z: 0.00}}"
```

注意：

- 单独发 `/cmd_vel_chassis` 或单独发 `/cmd_vel_chassis_bt` 都不够；当前哨兵底盘链还要求 `/robot_control` 在持续刷新，MCU 才会把 BT 当成有效接管方
- 如果你是用 `start_robot.sh` 起的 sender，它默认监听的是 `/cmd_vel_chassis_bt`，不是 `/cmd_vel_chassis`
- `serial_sender.py --ros2` 收到速度话题后，终端会打印类似 `[NAV2 -> STM32] vx=... vy=... wz=...`，这是判断速度有没有真的送进 sender 的最快方法

纯云台扫描测试：

```bash
source /opt/ros/humble/setup.zsh
source /home/nyu/sentry_planner/rm_decision_ws/install/setup.zsh
ros2 topic pub -r 20 /robot_control rm_decision_interfaces/msg/RobotControl \
"{stop_gimbal_scan: false, chassis_spin_vel: 0.0, scan_enabled: true, allow_vision_control: false, search_when_target_lost: false, scan_yaw_rate_deg_s: 120.0, search_pitch_deg: -6.0}"
```

这条走的是 BT functional scan：

- yaw 匀速转
- pitch 固定到 `search_pitch_deg`
- 不会上下摆

如果你想测“像左拨杆中位那样，没目标先搜索，有目标再跟随”的语义，要发下面这条，不是上面那条：

```bash
source /opt/ros/humble/setup.zsh
source /home/nyu/sentry_planner/rm_decision_ws/install/setup.zsh
ros2 topic pub -r 20 /robot_control rm_decision_interfaces/msg/RobotControl \
"{stop_gimbal_scan: true, chassis_spin_vel: 0.0, scan_enabled: true, allow_vision_control: true, search_when_target_lost: true, scan_yaw_rate_deg_s: 120.0, search_pitch_deg: -6.0}"
```

这条现在和电控里“左拨杆中位自瞄”语义对齐：

- 没目标时：yaw 转 + pitch 上下扫
- 识别到目标后：视觉接管云台开始跟随

配套前置条件：

```bash
cd /home/nyu/Codespace/nyush-rm-vision
just test detect --web --send
```

浏览器页面：

```text
http://127.0.0.1:8888
```

当前这条 `detect --web --send` 测试链路里：

- 识别到目标时才会发 `AUTO_AIM` 控制云台
- 默认不会自动开火
- 日志里 `sent_ctl=true` 说明视觉已经开始接管

现场最容易踩的坑：

- `/robot_control` 一停，BT 接管会超时，底盘/云台就会退回本地逻辑
- 右拨杆中位的哨兵自转模式会压掉 BT/视觉这条云台链
- 云台“纯扫描”和“自瞄搜索”是两套不同语义：前者 pitch 固定，后者 pitch 才会上下摆
- 如果开着视觉又想测纯扫描，视觉偶尔发来的控制会打断扫描，建议临时先关 `just test detect --web --send`

## 13. 当前代码中的有效链路和预留链路

### 13.1 已经打通或基本打通

- MCU 单口同时支持视觉 `SP` 和哨兵扩展 `SX/ST`
- 桥接脚本单口拆双 PTY
- 视觉侧 `io::Gimbal` 能直接对接 `Vision PTY`
- `sentry_planner` 中 Nav2 -> `/cmd_vel_chassis` 链路是清楚的
- `bt_comm_adapter.py` 已把 `/cmd_vel_chassis + /robot_control.chassis_spin_vel` 合成 `/cmd_vel_chassis_bt`
- 行为树血量话题已统一到 `/all_robot_hp`

### 13.2 已预留但还没完全接上

- `auto_aim_target_pos` 这条视觉 -> 导航/决策的 ROS2 通路

`/robot_control` 的功能话题已经接到当前 bridge/MCU 链路；这里还没接上的主要是更高层的视觉 -> 决策反馈闭环。

### 13.3 不建议混用

- `rm_serial_driver`
- `sentry_bridge.py`

这两个都想占真实串口。

如果下位机已经走 `nyush-rm-control + sentry_bridge.py` 路线，就不要再让 `rm_serial_driver` 打开同一个 `/dev/ttyACM0`。

## 14. 现场排障命令

### 14.1 看谁占了真实口

```bash
lsof /dev/ttyACM0
```

理想情况：

- 只有 `sentry_bridge.py` 占用

### 14.2 看桥有没有把 PTY 打出来

桥启动后终端里应有：

```text
Vision PTY : /dev/pts/X
Radar PTY  : /dev/pts/Y
```

### 14.3 看导航速度有没有出来

```bash
source /opt/ros/humble/setup.bash
source /home/nyu/sentry_planner/install/setup.bash
ros2 topic hz /cmd_vel /cmd_vel_chassis
```

### 14.4 看视觉检测有没有出来

```bash
ros2 topic echo /detector/armors --once
```

### 14.5 看行为树有没有在吃视觉

当前行为树只要 `armors` 非空就会认为“检测到敌人”。

可以先用假数据：

```bash
cd /home/nyu/sentry_planner
source rm_decision_ws/install/setup.bash
source rm_vision_ws/install/setup.bash
./rm_decision_ws/rm_decision_interfaces/publish_script.sh
```

### 14.6 看桥接雷达侧有没有收发

```bash
cd /home/nyu/Codespace/nyush-rm-control
just radar --port <Radar PTY>
```

如果能看到回传的 telemetry，说明：

- `Radar PTY -> bridge -> MCU -> ST -> bridge -> Radar PTY`

这一圈至少在协议层已经闭环。

## 15. 已知风险与注意事项

### 15.1 最大风险：协议漂移

目前至少存在两种“雷达侧上位机帧”：

- 桥接脚本当前使用的 19 字节格式
- 旧 `serial_sender.py` 使用的 23 字节格式

这不是设计思想冲突，而是代码尚未完全收敛。

### 15.2 PTY 不是稳定设备名

每次桥重启，`/dev/pts/X` 可能变化。

推荐做法：

- 用软链接
- 或桥启动后立即把 PTY 写入你们自己的启动脚本/环境变量

### 15.3 视觉到决策的“目标类别”链并未真正接入行为树

当前行为树主要消费的是 `/detector/armors` 的“非空”信息。

视觉虽然能发 `auto_aim_target_pos`，而且第四个量里已经带了目标类别，但当前 `sentry_planner` 还没有真正订阅并使用它。

### 15.4 `rm_serial_driver` 和桥接版不要同时上

两者的定位不同：

- `rm_serial_driver` 是旧 ROS2 串口驱动栈
- `sentry_bridge.py` 是新单口双路桥接栈

同时运行只会互抢串口。

## 16. 推荐的下一步收敛方向

如果要把整条通讯链进一步收敛成“稳定调决策”的状态，建议优先做这两件事：

1. 继续把 `/robot_control` 的功能话题用到更完整的比赛状态机里
   - 目前 `scan_enabled / allow_vision_control / search_when_target_lost / scan_yaw_rate_deg_s / search_pitch_deg` 已经能落到 MCU
   - 下一步更值得做的是把这些功能量系统地铺到更多行为树节点，而不是再回退成“模拟 RC 挡位”
2. 继续把真实裁判系统状态扩到更多决策输入
   - 当前 bridge 主线已经能把真实 `/game_status`、`/robot_status` 带到 ROS
   - 下一步如果要进一步减少 fallback，可继续补 `all_robot_hp` 等更完整裁判字段

---

如果只想记一句最关键的话，那就是：

```text
真实 /dev/ttyACM0 只给 sentry_bridge.py；
视觉改连 Vision PTY；
雷达/导航改连 Radar PTY；
当前仓库版 serial_sender.py 已经和 bridge 协议对齐。
```
