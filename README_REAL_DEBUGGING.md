# 实机调试须知

最后更新：2026-03-19

这份文档专门给当前 NYUSH 哨兵的实机联调用，重点覆盖：

- 上实机前到底要准备什么
- 是不是必须先有完整比赛场地
- 现在这套 `RMUL2026 + center_attack_simple + bridge` 链路应该怎么上车
- 哪些功能已经真正打通，哪些还只是 ROS 侧逻辑

## 1. 先说结论

当前最合理的实机联调顺序不是“直接去完整场地跑全流程”，而是分 3 段：

1. 台架联调
   - 先测通讯、底盘响应、行为树分支切换
   - 不强依赖完整比赛场地
2. 小范围低速地面联调
   - 找一块安全平地，先测定位、去点、回家
   - 这一步已经需要环境和地图大体一致
3. 真实场地联调
   - 最后再测完整“去中心 -> 守中 -> 低血回家”

所以答案是：

- 不是一开始就必须有完整比赛场地
- 但只要你开始测导航闭环，就必须有和地图足够一致的真实环境

## 2. 当前这套实机假设

本文按你现在仓库里的当前配置写：

- 行为树：
  - `center_attack_simple.xml`
- 地图：
  - `RMUL2026.yaml`
- 当前点位：
  - `Home = (0.8, 7.8)`
  - `Center2026 = (6.33, 4.32)`

对应代码：

- 行为树目标点：
  - `rm_decision_ws/rm_behavior_tree/config/center_attack_simple.xml`
- 仿真/实机调试 watcher 默认点：
  - `scripts/watch_center_attack_state.py`

如果你后面改了地图、原点或场地方向，这两个点也必须一起改。

## 3. 上实机前必须确认的 6 件事

### 3.1 安全措施

至少要有这些：

- 枪口安全，必要时先断射击
- 底盘第一次测试先架空，或者限制在很小速度
- 场边有人看车，不要单人直接放地跑
- 急停手段准备好
- 首轮联调先关闭一切不必要的自动攻击动作

### 3.2 地图与场地坐标一致

这是最容易被忽略、也最容易让你误判“Nav2 不行”的地方。

必须确认：

- 实机场地和 `RMUL2026.yaml` 对应的是同一套布局
- 原点方向一致
- `Home(0.8, 7.8)` 确实在你想要的左侧角内侧自由区域
- `Center2026(6.33, 4.32)` 确实是你想要的守中点

如果地图方向、原点、障碍摆位和实物不一致，那么：

- 路径会看起来“有规划”
- 但车会走错方向，或者根本不愿意走

### 3.3 定位链稳定

实机上真正需要稳定的是：

- `map -> odom -> base_link`
- 激光雷达输入
- 定位结果不乱跳

至少要满足：

- `navigate_to_pose` action 在线
- RViz 中机器人位姿稳定
- 障碍和地图大体重合
- 局部 costmap 不持续炸开

### 3.4 通讯链清楚

当前推荐链路仍然是：

```text
vision <-> Vision PTY <-> sentry_bridge.py <-> /dev/ttyACM0 <-> nyush-rm-control
planner/nav <-> Radar  PTY <-> sentry_bridge.py <-> /dev/ttyACM0 <-> nyush-rm-control
```

要点：

- `/dev/ttyACM0` 只能给 `sentry_bridge.py`
- 雷达/导航侧只连 `Radar PTY`
- 视觉侧只连 `Vision PTY`

详细协议和命令见：

- `README_COMMUNICATION.md`
- `communication command.txt`

### 3.5 别直接依赖 `start_robot.sh`

当前要特别注意：

- `start_robot.sh` 仍然默认启动旧树 `retreat_attack_left`
- 它不是你现在这条 `center_attack_simple` 主线的最佳入口

如果你现在调的是：

- 开赛去中心
- 到点守中
- 低血回家

那建议按本文的“手动分终端启动”走，不要直接把 `start_robot.sh` 当唯一入口。

### 3.6 明确当前已打通和未打通的量

当前实机链路里，和行为树最相关的两个输出是：

- `chassis_spin_vel`
  - 已经能通过 `bt_comm_adapter.py` 合成进 `/cmd_vel_chassis_bt`
  - 并继续经 `serial_sender -> Radar PTY -> bridge -> MCU`
- `stop_gimbal_scan`
  - 当前仍主要停留在 ROS 侧 `/robot_control`
  - 还不是完整直通到 bridge/MCU 的独立控制量

这点非常重要：

- 你现在能比较稳地测“去中心 / 回家 / 小陀螺速度”
- 但不要默认“停扫/切自瞄”已经全链路落到下位机

## 4. 是不是必须先有实际场地

### 不需要完整比赛场地也能做的

- 串口桥接与 PTY 分流
- `serial_sender` 到下位机底盘速度链
- 行为树分支切换
- `/robot_control` 和 `/cmd_vel_chassis_bt` 是否输出正确
- 低血回家 / 开赛去中心这些逻辑是否切对

### 必须有真实且匹配环境才能做的

- `go home`
- `go center`
- `到点后切换守中`
- Nav2 是否真能绕障
- 实际位置和地图是否一致

所以：

- 没有完整比赛场地，不代表不能上实机
- 但没有真实且匹配的环境，就不要过早下结论说“导航已经调好了”

## 5. 推荐实机联调顺序

### 阶段 A：台架联调

目标：

- 先测通讯
- 先测底盘速度响应
- 先测行为树分支切换
- 不急着测完整导航

建议做的事：

1. 启动 bridge
2. 确认 `Vision PTY / Radar PTY`
3. 用 `serial_sender` 做 one-shot 或低速话题写口
4. 确认底盘能收到速度
5. 单独看 `/robot_control`、`/cmd_vel_chassis_bt`

这一阶段更像“全链路通断测试”，不是正式导航测试。

### 阶段 B：小范围低速地面联调

目标：

- 先测定位是否稳定
- 先测起点附近小范围移动
- 先测 `Home` 点是否合理

建议：

- 先低速
- 先只跑短距离
- 先确认 `Home` 点不会刷到边界或障碍

### 阶段 C：真实场地联调

目标：

- 去中心
- 到点守中
- 低血回家

只有这一阶段，才适合评价：

- 你现在的 `Home / Center2026`
- 你的实机 Nav2 参数
- 你的真实比赛行为逻辑

## 6. 推荐终端分工

这是当前最推荐的实机联调分法。

### 终端 1：实机导航与定位

```bash
cd ~/sentry_planner/rm_navigation_ws
source ~/nav_ws/install/setup.zsh
source install/setup.zsh
ros2 launch rm_nav_bringup bringup_real.launch.py \
  world:=YOUR_MAP_NAME \
  mode:=nav \
  lio:=fastlio \
  localization:=slam_toolbox \
  nav_rviz:=True
```

说明：

- `YOUR_MAP_NAME` 应换成你实机实际使用的地图前缀
- 如果你不是 `slam_toolbox`，就换成你当前在用的定位方式

### 终端 2：bridge

```bash
python3 /home/nyu/Codespace/nyush-rm-control/scripts/sentry_bridge.py --port /dev/ttyACM0
```

记下它打印出来的：

- `Vision PTY`
- `Radar PTY`

### 终端 3：底盘串口发送器

```bash
source /opt/ros/humble/setup.zsh
python3 /home/nyu/Codespace/nyush-rm-vision/serial_sender.py \
  --port <Radar PTY> \
  --ros2 \
  --topic /cmd_vel_chassis_bt
```

### 终端 4：行为树调试会话

```bash
bash ~/sentry_planner/scripts/run_center_attack_debug_session.sh
```

这个脚本会保活：

- `bt_comm_adapter.py`
- `center_attack_simple`
- `watch_center_attack_state.py`

### 终端 5：视觉

如果你要带真视觉一起测，再开：

```bash
cd /home/nyu/Codespace/nyush-rm-vision
./build/sentry configs/sentry.yaml
```

注意：

- `sentry.yaml` 里的 `com_port` 必须指向 bridge 当前打印出来的 `Vision PTY`

## 7. 当前最小实机检查清单

### 7.1 上电后先查

```bash
ros2 action list | grep navigate_to_pose
ros2 topic echo /robot_control --once
ros2 topic echo /cmd_vel_chassis_bt --once
```

如果 `navigate_to_pose` 不在线，就先不要起行为树。

### 7.2 开赛去中心

```bash
source /opt/ros/humble/setup.bash
source ~/sentry_planner/rm_decision_ws/install/setup.bash

ros2 topic pub -r 1 /game_status rm_decision_interfaces/msg/GameStatus \
  "{game_progress: 4, stage_remain_time: 220}"

ros2 topic pub -r 10 /robot_status rm_decision_interfaces/msg/RobotStatus \
  "{robot_id: 7, current_hp: 600, shooter_heat: 0, team_color: false, is_attacked: false}"
```

期望：

- watcher 显示 `APPROACH_CENTER`
- `/cmd_vel_chassis_bt` 有输出
- 机器人开始向 `Center2026` 运动

### 7.3 低血回家

```bash
ros2 topic pub -r 10 /robot_status rm_decision_interfaces/msg/RobotStatus \
  "{robot_id: 7, current_hp: 200, shooter_heat: 0, team_color: false, is_attacked: false}"
```

期望：

- watcher 显示 `HOME_RECOVER`
- 机器人开始回 `Home`

### 7.4 未开赛待机

```bash
ros2 topic pub -r 1 /game_status rm_decision_interfaces/msg/GameStatus \
  "{game_progress: 0, stage_remain_time: 220}"
```

期望：

- watcher 显示 `HOME_STANDBY`
- 不主动去中心

## 8. 什么时候算“可以去场地了”

至少满足下面这些，再去完整场地会比较省时间：

- bridge 稳定
- `serial_sender` 能稳定驱动底盘
- 实机定位稳定
- `navigate_to_pose` 正常
- `APPROACH_CENTER / HOME_RECOVER / HOME_STANDBY` 三个分支都能切对
- `Home` 点和 `Center2026` 点已经确认在自由区域

如果这些都还没稳，去完整场地很容易浪费时间在基础问题上。

## 9. 当前已知坑

### 9.1 `start_robot.sh` 不是当前主线入口

它仍然默认：

- 旧树 `retreat_attack_left`
- 旧实机流程

所以当前建议以手动分终端启动为准。

### 9.2 `stop_gimbal_scan` 还不是完整下位机闭环

当前真正稳地落到底盘链上的，是：

- `Nav2 /cmd_vel_chassis`
- `RobotControl.chassis_spin_vel`

而：

- `RobotControl.stop_gimbal_scan`

目前仍主要停留在 ROS 侧。

### 9.3 PTY 会变

bridge 每次重启：

- `Vision PTY`
- `Radar PTY`

都可能变化。不要把旧的 `/dev/pts/N` 写死后忘了改。

### 9.4 没有真实环境时，别过早评价导航

如果你只是台架或小区域联调：

- 可以评价通讯
- 可以评价分支逻辑
- 不能轻易下结论说“实机导航已经完全 OK”

## 10. 推荐阅读

- `README_COMMUNICATION.md`
- `README_BEHAVIOR_TREE_FLOW.md`
- `communication command.txt`
- `mid360 command.txt`

