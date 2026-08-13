# NYUSH Sentry Commands and Data Flow

<div align="center">

[![简体中文](https://img.shields.io/badge/%E7%AE%80%E4%BD%93%E4%B8%AD%E6%96%87-Switch-2ea44f?style=for-the-badge)](README_COMMANDS.md)
[![English](https://img.shields.io/badge/English-Current-0969da?style=for-the-badge)](README_COMMANDS_EN.md)

</div>

Last updated: 2026-04-11

Use this guide together with [communication command.txt](communication%20command.txt). This document explains the architecture, data flow, and parameter semantics; the text file provides copy-ready command snippets. Packet-level details remain authoritative in [README_COMMUNICATION_EN.md](README_COMMUNICATION_EN.md).

## 0. Documentation Scope

| Document | Responsibility |
|---|---|
| [README_EN.md](README_EN.md) | Central index: overview, architecture, build order, and shortest startup path |
| [README_COMMUNICATION_EN.md](README_COMMUNICATION_EN.md) | Frames, PTYs, `serial_sender`, `bt_comm_adapter`, and MCU communication |
| [README_BEHAVIOR_TREE_FLOW_EN.md](README_BEHAVIOR_TREE_FLOW_EN.md) | XML, nodes, `RobotControl`, Groot2, coordinates, and watchers |
| [README_LIDAR_EN.md](README_LIDAR_EN.md) | Mid-360, FAST-LIO, Nav2, parameters, simulation, and localization |
| **This document** | Executable commands, environment variables, script index, end-to-end paths, maps, and real-robot bring-up |

Section map:

| Section | Content |
|---|---|
| §1 | Four paths across MCU, LiDAR, vision, behavior tree, and host computer |
| §2 | Bridge auto-detection and PTYs |
| §3 | Vision web UI and division of responsibility with BT/`RobotControl` |
| §4 | `nav_ws/start_robot.sh` variables; maps, PCD/PGM/YAML, and BT goals |
| §5–§6 | Referee topics, watcher, and hotkeys |
| §7 | `rotate_pcd` → `pcd2pgm` → `map_saver_cli`; `map_point_picker.py` |
| §8–§11 | Autostart, component matrix, minimum terminals, and quick-reference notes |
| §12 | Real-robot integration from bench to small area to full field |
| §13 | Problem-to-document lookup table |

This document owns the system-level path and operational commands. SP/SX/ST byte layouts belong to the communication guide. Gazebo RMUL2026 and the first Sim2Real step belong to the LiDAR guide; [mid360 command.txt](mid360%20command.txt) contains the corresponding step-by-step commands.

## 1. End-to-End Data Paths and Responsibilities

### 1.1 Physical Layers

| Layer | Hardware / Process | Responsibility |
|---|---|---|
| Host computer, such as the NUC | ROS 2 LiDAR driver, FAST-LIO, Nav2, behavior tree, `serial_sender`, `bt_comm_adapter` | Localization, planning, decisions, and encoding velocity plus mode fields |
| Bridge, normally on the host | `sentry_bridge.py` | Sole owner of USB CDC; splits it into Vision and Radar PTYs |
| Lower controller | STM32 in `nyush-rm-control` | Decodes SX/ST and SP, merges commands, drives chassis/gimbal/shooter, and reads referee data |

The Mid-360 connects to the host over Ethernet. It never passes through the MCU USB port. Point clouds, odometry, topics, and TF are consumed directly by Nav2 and the behavior tree.

### 1.2 Four Main Paths

```text
[A — localization, navigation, and chassis translation]
Mid-360 --Ethernet--> livox_ros_driver2 -> FAST-LIO
        -> /cloud_registered + odometry + TF
        -> pointcloud_to_laserscan -> /scan -> Nav2 -> /cmd_vel
        -> fake_vel_transform -> /cmd_vel_chassis
        -> bt_comm_adapter (+ chassis_spin_vel) -> /cmd_vel_chassis_bt
        -> serial_sender -> Radar PTY -> bridge -> MCU
        -> SX(vx, vy, wz, ...) -> chassis execution

[B — behavior-tree decisions, modes, and chassis-spin intent]
BT subscribes to /game_status, /robot_status, ...
BT publishes /goal_pose -> Nav2, affecting path A without a serial packet
BT publishes /robot_control -> serial_sender -> A3 -> Radar PTY -> bridge -> MCU
        -> SX.control_flags + scan_* + allow_vision_control + ...
        -> robot_cmd.c state machine

[C — vision servo, gimbal angles, and firing]
camera -> nyush-rm-vision -> SP(VisionToGimbal) -> Vision PTY -> bridge -> MCU
       -> when takeover is permitted, apply SP yaw/pitch/mode for tracking and firing
MCU -> SP(GimbalToVision) -> vision feedback: attitude, projectile speed, ...

[D — referee and onboard state back to ROS]
referee -> MCU -> ST -> bridge -> Radar-side 0x5C/0x5D
        -> serial_sender -> /game_status + /robot_status -> BT
```

Paths A and B share the Radar PTY as a velocity frame plus A3. Path C uses the Vision PTY. Path D returns through the Radar side and is decoded into ROS messages by the sender.

### 1.3 Component Matrix

| Component | Location | Responsibility | Main interface |
|---|---|---|---|
| LiDAR and driver | Robot + host | Point clouds | `/livox/lidar`, related topics |
| FAST-LIO | Host | Odometry and registered cloud | `/cloud_registered`, odom, TF |
| `pointcloud_to_laserscan` | Host | 2D laser scan | `/scan` |
| Nav2 | Host | Planning and obstacle avoidance | `/cmd_vel` |
| `fake_vel_transform` | Host | Velocity-frame conversion | `/cmd_vel_chassis` |
| `bt_comm_adapter` | Host | Merge translation and tactical spin | `/cmd_vel_chassis_bt` |
| `rm_behavior_tree` | Host | High-level tactics and goals | `/goal_pose`, `/robot_control` |
| `serial_sender --ros2` | Host | ROS ↔ Radar PTY and status feedback | 19-byte velocity + A3 |
| `sentry_bridge` | Host | One USB ↔ two PTYs and frame conversion | SP / SX / ST |
| `nyush-rm-vision` | Host | Detection, tracking, and aiming | SP |
| MCU | C board | Command fusion and execution | SX + SP + RC |

### 1.4 `RobotControl`/SX Versus SP

| Channel | Typical content | Producer |
|---|---|---|
| SX / `RobotControl` | Vision-takeover permission, scan parameters, `chassis_spin_vel`, mode flags | BT → sender |
| SP | Target yaw/pitch and auto-aim/firing mode | Vision |

The MCU uses SX flags to decide whether vision is enabled; only then is SP tracking connected to the gimbal control loop in `nyush-rm-control/application/cmd/robot_cmd.c`.

### 1.5 Simplified Diagram

```text
                    +------------- BT (rm_behavior_tree)
                    | subscribes: /game_status, /robot_status, ...
                    | publishes : /goal_pose, /robot_control
                    +------+----------------------+---------------+
                           |                      |
                           v                      v
                    +-------------+      +------------------------+
                    |    Nav2     |      | bt_comm_adapter.py     |
                    |  /cmd_vel   |      | /cmd_vel_chassis +     |
                    +------+------+      | /robot_control ->       |
                           |             | /cmd_vel_chassis_bt    |
                           v             +-----------+------------+
                    +-------------+                  |
                    | fake_vel_   |                  |
                    | transform   |                  |
                    +------+------+                  |
                           +------------+------------+
                                        v
                              serial_sender.py --ros2
                                        |
                                        v
                              Radar PTY -> bridge -> MCU (SX)
                                        ^
MCU referee/state <- ST -> bridge -> 0x5C/0x5D -> sender -> /game_status, /robot_status

vision: nyush-rm-vision <- SP -> Vision PTY -> bridge -> MCU
```

## 2. Start the Bridge Without Hard-Coding `/dev/ttyACM0`

### 2.1 Recommended Command

```bash
cd /path/to/nyush-rm-control
just sentry-bridge
```

This invokes `sentry_bridge.py` without `--port`.

### 2.2 Automatic Port Selection

The script enumerates USB serial ports and selects the MCU CDC device using `resolve_serial_port` and `port_priority`. Replugging USB may change `ttyACM0` to `ttyACM1`; automatic selection avoids editing commands after every reconnect.

### 2.3 When to Pass `--port`

- Several serial devices are attached and automatic selection chooses the wrong one.
- A fixed device is required for a controlled test.

Example:

```bash
just sentry-bridge --port /dev/ttyACM1
```

### 2.4 Record the Bridge Output

The terminal prints the actual MCU serial device plus the Vision and Radar PTYs. Unless `--no-links` is used, stable symlinks such as `/tmp/nyush-rm-sentry-radar` are normally created. Prefer the symlink for `RADAR_PTY` because `/dev/pts/N` changes after bridge restarts.

## 3. Vision: `just test detect --web --send`

### 3.1 Command and Web UI

```bash
cd /path/to/nyush-rm-vision
just test detect --web --send
```

- `--web`: starts the local web UI, normally at `http://127.0.0.1:8888`; verify against the current vision `justfile`.
- `--send`: writes vision commands through the serial protocol. Vision `com_port` must point to the bridge's Vision PTY or stable symlink.

### 3.2 Interaction with the Behavior Tree

Separate two responsibilities:

1. Target detection and auto-aim are primarily camera → vision → Vision PTY → SP → MCU.
2. Branching a tactic on “enemy visible” requires the XML to subscribe to `/detector/armors` or another vision topic.

`center_attack_simple.xml` contains `SubRobotStatus` and `SubGameStatus`, but no `SubArmors` or `IsDetectEnemy`.

- Vision remains active even though target visibility is not a BT condition.
- At center hold, BT sets `allow_vision_control=True`, `stop_gimbal_scan=True`, and related fields to permit takeover. Vision and SP still perform the actual tracking.
- While moving, `allow_vision_control=False` favors scanning. Firmware and the SX flags decide whether vision may take control.
- Legacy trees such as `retreat_attack_left.xml` instantiate `SubArmors` and branch on whether the armor array is empty.
- As an optional, separate path, `bt_comm_adapter.py` can adapt `auto_aim_target_pos` into `/detector/armors` for trees that need that topic. This does not replace the direct SP path.

| Path | Purpose |
|---|---|
| Vision → MCU | SP gimbal and firing control, without passing through a BT node |
| BT → MCU through sender | SX/`RobotControl` scan, takeover permission, and tactical spin |
| BT ← `/detector/armors` | Only when the selected XML explicitly uses `SubArmors` |

## 4. `~/nav_ws/start_robot.sh` Environment Variables

A common “bridge and optional vision already running → localization + Nav2 + BT” invocation is:

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

### 4.1 Variable Reference

| Variable | Meaning |
|---|---|
| `MAP_FILE` | Map-server YAML entry point. It references the PGM. Use `11_map.yaml` for a locally built map or `RMUL2026.yaml` for the competition field. |
| `BT_STYLE` | Behavior-tree XML name without `.xml`. `center_attack_fullstack` is the team full-stack tree; `center_attack_simple` is the reduced tree and the common default in this repository's launcher. |
| `BT_START_GOAL` / `BT_END_GOAL` | `start_goal_pose` / `end_goal_pose` blackboard strings in `x;y;z; qx;qy;qz;qw` format. Revalidate them together with hard-coded XML goals after changing maps. |
| `START_SERIAL_SENDER=1` | Starts `serial_sender.py --ros2` and writes ROS velocity and control to `RADAR_PTY`. |
| `RADAR_PTY` | Must match the current bridge Radar side. Prefer `/tmp/nyush-rm-sentry-radar`; the script maps it to `SERIAL_SENDER_PORT`. |
| `START_BT=1` | Starts `bt_comm_adapter.py` and `rm_behavior_tree`. It also changes the sender topic to `/cmd_vel_chassis_bt` so tactical spin can be merged. |
| `SERIAL_SENDER_DISABLE_STATUS_PUB` | Sender `--disable-status-pub`: `0` publishes bridge-derived `/game_status` and `/robot_status`; `1` suppresses them to avoid competing with hotkey or fixture publishers. |
| `START_FAKE_VEL_TRANSFORM=1` | Starts `/cmd_vel` → `/cmd_vel_chassis` conversion. |

### 4.2 Sourced Workspaces

The script normally sources `~/nav_ws/install`, the repository install overlay when present, `rm_vision_ws`, and `rm_decision_ws`, making behavior-tree, velocity-transform, sender, and message dependencies available.

### 4.3 Two `start_robot.sh` Variants

- `nav_ws/start_robot.sh`: primary path while ICP is not connected; uses AMCL and Nav2 `bringup_launch` with the variables above.
- `nyush_rm_sentry/start_robot.sh`: longer Mid-360 + FAST-LIO + optional ICP launcher. Some variable names overlap, but defaults differ; read its header before combining examples.

### 4.4 Map Sources, PCD/PGM/YAML, and BT Goals

#### 4.4.1 Common Maps

| Scenario | Typical `MAP_FILE` | Notes |
|---|---|---|
| Local/lab map | `$HOME/Desktop/map/11_map.yaml` | `map_saver_cli -f 11_map` creates `11_map.yaml` and `11_map.pgm` in `~/Desktop/map/`. |
| RMUL 2026 field | `.../rm_nav_bringup/map/RMUL2026.yaml` | Fixed field map with `RMUL2026.pgm` in the same directory. |

If YAML `image:` uses a relative path, keep the PGM beside it. Never copy only the YAML.

#### 4.4.2 File Roles

| Type | Typical location | Runtime consumer |
|---|---|---|
| PCD | `~/nav_ws/src/FAST_LIO/PCD/scans.pcd` | Offline mapping only: input to `rotate_pcd` and `pcd2pgm`; Nav2 does not load it at runtime. |
| PGM | Beside the map YAML and referenced by `image:` | Loaded through `map_server`; supplies the 2D occupancy grid used by global costmap, planner, and goal validity. |
| Map YAML | File passed as `MAP_FILE` | Nav2 map-service entry point: resolution, origin, thresholds, and PGM path. |

At runtime, Nav2 consumes the map YAML plus its referenced PGM. PCD participates only before the PGM is generated.

#### 4.4.3 Recalibrate Goals After Changing Maps

Some `SendGoal` values are hard-coded world coordinates in XML. The same numbers may become obstacles or leave the field after switching between `11_map`, `RMUL2026`, or any new map.

After changing maps:

- Update `BT_START_GOAL` and `BT_END_GOAL` as needed.
- Inspect all hard-coded XML coordinates for the selected `BT_STYLE`.
- Follow the values actually referenced by the active tree; blackboard parameters and XML constants may coexist.

Use `map_point_picker.py` with a graphical desktop or VNC to click the saved map and read map-frame `x, y` values.

## 5. Referee Data in ROS

Inspect `/game_status` and `/robot_status`:

```bash
source /opt/ros/humble/setup.bash
source /path/to/sentry_planner/rm_decision_ws/install/setup.bash
ros2 topic echo /game_status
ros2 topic echo /robot_status
```

### 5.1 Data Source

1. MCU sends compact referee fields in ST.
2. Bridge emits `0x5C` and `0x5D` on the Radar PTY.
3. `serial_sender.py --ros2` parses them and publishes `GameStatus` and `RobotStatus`.

Refreshing, plausible values confirm the referee path reaches the sender. No output or all zeros suggests checking bridge, sender, and whether `SERIAL_SENDER_DISABLE_STATUS_PUB=1`.

## 6. Debug BT Without Live Referee Data

### 6.1 `watch_center_attack_state.py`

```bash
source /opt/ros/humble/setup.bash
source /path/to/sentry_planner/rm_decision_ws/install/setup.bash
python3 /path/to/sentry_planner/scripts/watch_center_attack_state.py
```

This read-only monitor subscribes to `/amcl_pose`, `/game_status`, `/robot_status`, `/robot_control`, and `/cmd_vel_chassis_bt`, then prints a compact state line. It does not publish anything.

### 6.2 `bt_hotkey_debug.py`

```bash
source /opt/ros/humble/setup.bash
source /path/to/sentry_planner/rm_decision_ws/install/setup.bash
python3 /path/to/sentry_planner/scripts/bt_hotkey_debug.py
```

The hotkey tool continuously publishes `/game_status`, `/robot_status`, and armor fixtures for some presets. It can select not-started, in-match, low-HP, high-heat, attacked, and related states.

It competes with live sender-derived referee topics. Set `SERIAL_SENDER_DISABLE_STATUS_PUB=1` during fixture-driven tests. See `PRESETS` in the script for the current key map.

## 7. Mapping: Rotate PCD, Create PGM, Save Nav2 Map

### 7.1 Rotate the Point Cloud

```bash
cd ~/nav_ws/src/FAST_LIO/PCD
python3 rotate_pcd.py
```

This corrects the attitude of `scans.pcd`, or the path configured by the script, so the ground and axes match Nav2 expectations. If the `pcd2pgm` launch file hard-codes its input, keep it aligned with the rotated output.

### 7.2 Project PCD to a 2D Grid

```bash
export LD_PRELOAD=/lib/x86_64-linux-gnu/libusb-1.0.so.0
cd ~/nav_ws
source install/setup.zsh
ros2 launch pcd2pgm pcd2pgm_launch.py
```

This projects a 3D cloud into a 2D occupancy grid. `LD_PRELOAD` works around PCL/Open3D and libusb compatibility issues in some environments.

### 7.3 Save the Nav2 Map

```bash
cd ~/Desktop/map
ros2 run nav2_map_server map_saver_cli -f 11_map
```

This writes `11_map.yaml` and `11_map.pgm`. Pass the YAML as `MAP_FILE`.

### 7.4 Select Coordinates with `map_point_picker.py`

After saving the map, run this from a desktop or VNC session:

```bash
python3 ~/nav_ws/map_point_picker.py ~/Desktop/map/11_map.yaml
```

Click the figure to read map-frame coordinates for `BT_START_GOAL`, `BT_END_GOAL`, or hard-coded XML `SendGoal` values. Pass the RMUL2026 YAML instead to select points in the competition-field frame. The script opens a graphical window and will not work in a plain SSH session without display forwarding.

## 8. Boot-Time Startup: `autostart_fullstack.sh`

```bash
bash /home/nyu/sentry_planner/scripts/autostart_fullstack.sh
```

### 8.1 Behavior

1. Optionally cleans stale processes and PTY symlinks.
2. Runs `systemctl --user restart sentry_bridge.service`.
3. Waits for `/tmp/nyush-rm-sentry-vision`.
4. Starts `just test detect --web --send`, using a terminal when graphical or logging through `nohup` when headless.
5. Cleans Nav/LIO leftovers and runs `~/nav_ws/start_robot.sh` with the map, `center_attack_fullstack`, Radar PTY, and other values hard-coded in the script.

### 8.2 Logs

Logs such as `vision_detect.log` and `nav_bt.log` are written under `/home/nyu/sentry_planner/logs/autostart/`.

### 8.3 Field Use

This starts the bridge through systemd plus vision, navigation, BT, and sender with one command when no laptop is available. It is operationally equivalent to the manual multi-terminal setup.

## 9. Script and Node Matrix

| Component | Path / Command | Communication | Navigation | Vision | BT |
|---|---|---|---|---|---|
| `sentry_bridge.py` | `just sentry-bridge` | Owns MCU port and splits PTYs | — | Vision PTY | — |
| `serial_sender.py --ros2` | `nyush-rm-vision` | Radar PTY ↔ ROS | Consumes `/cmd_vel_chassis_bt` | — | Consumes `/robot_control` |
| `bt_comm_adapter.py` | `scripts/` | Produces `/cmd_vel_chassis_bt` | Adapts `/cmd_vel_chassis` | Optional armor adapter | Adapts `/robot_control` |
| `rm_behavior_tree` | `rm_decision_ws` | — | `SendGoal` | Optional armor subscription | Decision core |
| `fake_vel_transform` | Navigation package | — | `/cmd_vel` → `/cmd_vel_chassis` | — | — |
| `watch_center_attack_state.py` | `scripts/` | Watches output topics | Watches AMCL pose | — | Watches referee inputs |
| `bt_hotkey_debug.py` | `scripts/` | — | — | — | Publishes fixture game/robot state |
| `autostart_fullstack.sh` | `scripts/` | Starts bridge service and sender | Starts `nav_ws/start_robot.sh` | Starts detect | Starts BT |

## 10. Minimum Manual Terminals

| Terminal | Command |
|---|---|
| 1 | `just sentry-bridge` with automatic port selection |
| 2, optional | `just test detect --web --send` against the Vision PTY |
| 3 | `RADAR_PTY=/tmp/nyush-rm-sentry-radar ... ./start_robot.sh` using the appropriate launcher |

## 11. Common Quick-Reference Confusion

- `just radar --port /tmp/nyush-rm-sentry-radar` tests the PTY; it does not force direct use of ACM0.
- `nav_ws/start_robot.sh` still contains a `chmod` for `/dev/ttyACM0`. If automatic selection chooses ACM1, provide permissions through the matching command or a udev rule. Permission setup and bridge port selection are separate concerns.

## 12. Real-Robot Integration

### 12.1 Stage the Work

| Stage | Goal | Requires an environment matching the map? |
|---|---|---|
| A — bench | Bridge, PTYs, sender, chassis response, BT branches, `/robot_control`, and `/cmd_vel_chassis_bt` | No |
| B — small, low-speed area | Stable localization, short motion, and free-space Home goal | Yes, approximately |
| C — real field | Approach center, hold center, return on low HP, tune Nav2 and tactics | Yes, use a field map such as RMUL2026 |

Communication and BT logic can be tested without the full field, but do not claim navigation is complete until localization, costmaps, and `navigate_to_pose` are validated in a matching environment.

### 12.2 Minimum Safety

- Keep the muzzle safe and disable firing when it is not under test.
- For the first chassis test, raise the robot or enforce a low speed, use a spotter, and keep an explicit emergency-stop method available.

### 12.3 Launcher, Bridge, and Sender

- Neither `nav_ws/start_robot.sh` nor this repository's launcher starts `sentry_bridge` by default. Run it in a separate terminal or through systemd.
- To send `/cmd_vel_chassis_bt` and `/robot_control` to the MCU, use `START_SERIAL_SENDER=1 RADAR_PTY=<Radar PTY>`.
- `bt_comm_adapter` merges Nav2 translation with `RobotControl.chassis_spin_vel`. Sender and A3 carry velocity plus `scan_*` and `allow_vision_control` through the Radar PTY to the MCU.

### 12.4 Recommended Terminal Assignment

Adapt paths to the current machine.

| Terminal | Responsibility |
|---|---|
| 1 — localization/navigation | `ros2 launch rm_nav_bringup bringup_real.launch.py world:=<map-prefix> mode:=nav ...`; keep `world`, `MAP_FILE`, and localization mode consistent |
| 2 — bridge | `cd nyush-rm-control && just sentry-bridge`; record the PTYs and prefer stable `/tmp/nyush-rm-sentry-*` links |
| 3 — sender | Source ROS, then run `serial_sender.py --port <Radar PTY> --ros2 --topic /cmd_vel_chassis_bt`; omit if the launcher already starts it |
| 4 — BT debugging | `bash scripts/run_center_attack_debug_session.sh`; requires Nav2 and an active `navigate_to_pose` action |
| 5 — optional vision | Start vision with `configs/sentry.yaml` `com_port` pointing to the current Vision PTY |

### 12.5 Minimum Checklist

After power-up and before BT:

```bash
ros2 action list | grep navigate_to_pose
ros2 topic echo /robot_control --once
ros2 topic echo /cmd_vel_chassis_bt --once
```

If `navigate_to_pose` is unavailable, `SendGoal` cannot close the loop.

### 12.6 Publish Referee Fixtures

Source `rm_decision_ws/install/setup.bash` first.

In-match and healthy — expect `APPROACH_CENTER` and chassis output:

```bash
ros2 topic pub -r 1 /game_status rm_decision_interfaces/msg/GameStatus \
  "{game_progress: 4, stage_remain_time: 220}"
ros2 topic pub -r 10 /robot_status rm_decision_interfaces/msg/RobotStatus \
  "{robot_id: 7, current_hp: 600, shooter_heat: 0, team_color: false, is_attacked: false}"
```

Low HP — expect `HOME_RECOVER`:

```bash
ros2 topic pub -r 10 /robot_status rm_decision_interfaces/msg/RobotStatus \
  "{robot_id: 7, current_hp: 200, shooter_heat: 0, team_color: false, is_attacked: false}"
```

Match not started — expect `HOME_STANDBY`:

```bash
ros2 topic pub -r 1 /game_status rm_decision_interfaces/msg/GameStatus \
  "{game_progress: 0, stage_remain_time: 220}"
```

Fixture publishers compete with live sender status. Set `SERIAL_SENDER_DISABLE_STATUS_PUB=1` during this test.

### 12.7 Ready for the Full Field

Proceed only after:

- Bridge and sender are stable.
- Real-robot localization is stable.
- `navigate_to_pose` works.
- `APPROACH_CENTER`, `HOME_RECOVER`, and `HOME_STANDBY` switch correctly.
- Home and center goals are in free space on the selected map.

### 12.8 Known Pitfalls

- PTY numbers may change after every bridge restart. Do not retain an old `/dev/pts/N`; use the stable symlink when possible.
- Watcher default Home/Center values may differ from XML. Pass explicit coordinates matching the selected tree.

## 13. Which Guide Should I Open?

| Question | Document |
|---|---|
| Who owns serial, what is a PTY, what are 19-byte/A3/SP packets, where do topics originate? | [README_COMMUNICATION_EN.md](README_COMMUNICATION_EN.md) |
| Which XML runs, how do `SendGoal` and `RobotControl` work, how do I use Groot2 and hotkeys? | [README_BEHAVIOR_TREE_FLOW_EN.md](README_BEHAVIOR_TREE_FLOW_EN.md) |
| How do Gazebo RMUL2026, Sim2Real, Groot2, and `bringup_sim` fit together? | [README_LIDAR_EN.md](README_LIDAR_EN.md) and [mid360 command.txt](mid360%20command.txt) |
| How do Mid-360, FAST-LIO, Nav2 parameters, clouds, costmaps, and TF work? | [README_LIDAR_EN.md](README_LIDAR_EN.md) |
| What command, map, environment variable, mapping step, or terminal order should I use? | This document, especially §1, §4, §7, and §12 |
| Still unsure | [README_EN.md](README_EN.md) |

To embed a command in systemd or change `MAP_FILE` and BT goals in `autostart_fullstack.sh`, edit the corresponding environment block in that script.
