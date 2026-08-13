# NYUSH Sentry Communication Links and Protocols

<div align="center">

[![简体中文](https://img.shields.io/badge/%E7%AE%80%E4%BD%93%E4%B8%AD%E6%96%87-Switch-2ea44f?style=for-the-badge)](README_COMMUNICATION.md)
[![English](https://img.shields.io/badge/English-Current-0969da?style=for-the-badge)](README_COMMUNICATION_EN.md)

</div>

Last updated: 2026-04-11

This document describes the current communication topology, wire formats, bridge design, startup order, and troubleshooting workflow for the NYUSH sentry robot.

## 0. Documentation Scope

| Document | Responsibility |
|---|---|
| [README_EN.md](README_EN.md) | Central index: one-page overview, architecture, build order, and shortest startup path |
| **This document** | Complete communication reference: `sentry_bridge`, SP/SX/ST, PTYs, frames, `serial_sender`, `bt_comm_adapter`, ROS 2 topics, and troubleshooting |
| [README_BEHAVIOR_TREE_FLOW_EN.md](README_BEHAVIOR_TREE_FLOW_EN.md) | Behavior trees, XML, nodes, `RobotControl` semantics, and tactics |
| [README_LIDAR_EN.md](README_LIDAR_EN.md) | LiDAR, SLAM, Nav2, Gazebo Sim2Real, tuning, and real-robot localization |
| [README_COMMANDS_EN.md](README_COMMANDS_EN.md) | End-to-end data paths, bridge port selection, environment variables, maps, and staged integration |

Use this document when you need frame layouts, CRC rules, PTY ownership, or serial-topic mapping. Use the behavior-tree guide for how tactics produce `RobotControl`; use the commands guide for terminal-by-terminal bring-up; use the LiDAR guide for localization and simulation.

Frame formats, CRC behavior, and the `0x5C` / `0x5D` packets are authoritative here. The implementation-level description of how `bt_comm_adapter` merges `chassis_spin_vel` into `/cmd_vel_chassis_bt` is also maintained here.

The quick-reference files [communication command.txt](communication%20command.txt) and [mid360 command.txt](mid360%20command.txt) remain useful at the robot, but [README_COMMANDS_EN.md](README_COMMANDS_EN.md) is the source of truth for command behavior and parameters.

This guide covers four code sources:

- Current repository: `/home/nyu/sentry_planner`
- MCU and control: `/home/nyu/Codespace/nyush-rm-control`
- Vision: `/home/nyu/Codespace/nyush-rm-vision`
- Legacy radar serial senders: `/home/nyu/Codespace/nyush-rm-vision/serial_sender.py` and `/home/nyu/Desktop/serial_sender.py`

## 1. Key Takeaway

The recommended topology is:

```text
nyush-rm-vision <-> Vision PTY <-> sentry_bridge.py <-> /dev/ttyACM0 <-> nyush-rm-control
radar/navigation <-> Radar PTY <-> sentry_bridge.py <-> /dev/ttyACM0 <-> nyush-rm-control
```

Four rules matter most:

1. Only one process may open the physical port, normally `/dev/ttyACM0`.
2. That process must be `nyush-rm-control/scripts/sentry_bridge.py`.
3. Vision connects to the Vision PTY printed by the bridge, never directly to `/dev/ttyACM0`.
4. Radar/navigation connects to the Radar PTY printed by the bridge.

Opening the physical port from multiple processes commonly causes interleaved headers, timeouts on one side, nondeterministic startup behavior, and failures that disappear until the next reboot.

## 2. Component Roles

### 2.1 Components

- `nyush-rm-control`
  - Runs on the STM32 C board.
  - Controls the chassis, gimbal, shooter, referee-system interface, and local state machine.
- `nyush-rm-vision`
  - Runs on the onboard computer.
  - Performs armor detection, tracking, aiming, firing decisions, and exposes reserved ROS 2 interfaces.
- `nyush_rm_sentry` / legacy local name `sentry_planner`
  - Provides navigation, behavior-tree decisions, LiDAR localization, and the legacy ROS 2 serial path.
- `serial_sender.py`
  - Adapts ROS 2 `/cmd_vel`-family topics or keyboard input to the radar-side serial frames.
  - It is not part of the main vision path.

### 2.2 Ownership Boundary

- Vision and radar must not own the real MCU port.
- The bridge is the sole serial-port owner.
- The MCU still sees one CDC connection, while the bridge presents two logical paths:
  - Vision path: `SP`
  - Sentry extension path: `SX/ST`

## 3. Architecture at a Glance

### 3.1 Recommended Serial Topology

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
                     |  owns real port, splits PTYs |
                     +--------------+--------------+
                                    |
                 +------------------+------------------+
                 |                                     |
            Vision PTY                            Radar PTY
                 |                                     |
     +-----------+-----------+             +-----------+-----------+
     |     nyush-rm-vision   |             |  radar/nav-side tool  |
     |   SP send/receive     |             | legacy radar frames   |
     +-----------------------+             +-----------------------+
```

### 3.2 ROS 2 Path

```text
Nav2 -> /cmd_vel -> fake_vel_transform -> /cmd_vel_chassis
     -> bt_comm_adapter -> /cmd_vel_chassis_bt -> serial_sender -> Radar PTY

vision detection -> /detector/armors -> behavior tree
behavior tree -> /goal_pose -> Nav2
behavior tree -> /robot_control -> serial_sender -> bridge -> MCU
```

## 4. Recommended Source-Reading Order

1. `nyush-rm-control/scripts/sentry_bridge.py`: PTY creation and bidirectional conversion.
2. `nyush-rm-control/modules/master_machine/master_process.h`: packet structures and sizes.
3. `nyush-rm-control/modules/master_machine/master_process.c`: MCU-side SP/SX decoding and SP/ST replies.
4. `nyush-rm-vision/io/gimbal/gimbal.hpp`: vision-side structures.
5. `nyush-rm-vision/io/gimbal/gimbal.cpp`: `com_port` handling and SP transmission.
6. `rm_navigation_ws/src/rm_navigation/fake_vel_transform/src/fake_vel_transform.cpp`: `/cmd_vel` conversion.
7. `rm_decision_ws/rm_behavior_tree/plugins/action/sub_armors.cpp`: vision input to the behavior tree.
8. `rm_decision_ws/rm_behavior_tree/plugins/condition/is_detect_enemy.cpp`: current enemy-detection condition.
9. `nyush-rm-vision/tasks/omniperception/decider.cpp`: target information exported by vision.
10. `nyush-rm-vision/io/ros2/publish2nav.cpp`: vision-side ROS 2 publisher.

## 5. Physical and Virtual Port Ownership

### 5.0 Run Commands in the Correct Repository

The control and vision repositories have different `justfile` recipes.

- Run these from `nyush-rm-control`:
  - `just bridge` / `just sentry-bridge`
  - `just logger`
  - `just logger-cli`
  - `just vision`
- Run these from `nyush-rm-vision`:
  - `just test detect --web --send`
  - Other `just test ...` recipes

Errors such as the following usually mean the command was run in the wrong repository:

```text
error: Justfile does not contain recipe `logger`
error: Justfile does not contain recipe `test`
```

### 5.1 Single-Owner Rule

The physical port must be owned only by:

```bash
cd /home/nyu/Codespace/nyush-rm-control
just sentry-bridge --port /dev/ttyACM0
```

The bridge prints paths similar to:

```text
MCU serial : /dev/ttyACM0
Vision PTY : /dev/pts/3
Radar PTY  : /dev/pts/4
Press Ctrl+C to stop.
```

Vision then uses `/dev/pts/3`, and the radar/navigation sender uses `/dev/pts/4`.

### 5.2 Why the Bridge Is Required

The MCU supports two message families over one CDC link:

- Vision packets: `SP`
- Sentry extension packets: `SX/ST`

The bridge forwards SP packets for vision, converts legacy radar commands into SX, and converts MCU ST replies back into legacy radar telemetry plus ROS-facing status packets.

## 6. Protocol Overview

### 6.1 Conventions

- Multi-byte values follow the little-endian behavior of the current implementation.
- Legacy radar packets use CRC8.
- SP, SX, and ST use the current CRC16 implementation.
- External tools should reuse the implementation instead of reimplementing CRC from memory.

Reference implementations:

- `nyush-rm-control/scripts/sentry_bridge.py`
- `nyush-rm-control/modules/master_machine/master_process.h`
- `nyush-rm-vision/io/gimbal/gimbal.hpp`

## 7. SP Vision Protocol

SP is the primary vision-to-MCU protocol.

### 7.1 MCU to Vision: `GimbalToVision`, 43 Bytes

Defined in `master_process.h` and `gimbal.hpp`:

```text
head[2]      = 'S','P'
mode         uint8
q[4]         float[4]
yaw          float   rad
yaw_vel      float   rad/s
pitch        float   rad
pitch_vel    float   rad/s
bullet_speed float   m/s
bullet_count uint16
crc16        uint16
```

Fields:

- `mode`: `0` IDLE, `1` AUTO_AIM, `2` SMALL_BUFF, `3` BIG_BUFF.
- `q[4]`: quaternion in `w x y z` order.
- `yaw` / `pitch`: current gimbal attitude.
- `bullet_speed`: current projectile speed.
- `bullet_count`: cumulative shot count.

### 7.2 Vision to MCU: `VisionToGimbal`, 29 Bytes

```text
head[2] = 'S','P'
mode      uint8
yaw       float   rad
yaw_vel   float   rad/s
yaw_acc   float   rad/s^2
pitch     float   rad
pitch_vel float   rad/s
pitch_acc float   rad/s^2
crc16     uint16
```

`mode` values:

- `0`: no control.
- `1`: control the gimbal without firing.
- `2`: control the gimbal and allow firing.

This upstream SP packet carries gimbal control and firing mode, not a target class. Although the MCU has a `target_type` field, `ApplyVisionPacket()` explicitly sets it to `NO_TARGET_NUM` because vision does not provide it in this packet. In other words, SP answers “how should the gimbal move, and may it fire?” rather than “which robot class is being tracked?”

## 8. SX/ST Sentry Extension Protocol

### 8.1 Radar/Bridge to MCU: `SX`, 33 Bytes

```text
head[2]             = 'S','X'
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

Current `control_flags` bits:

- bit 0: `scan_control_valid`
- bit 1: `stop_gimbal_scan` (legacy compatibility)
- bit 2: `scan_enabled`
- bit 3: `allow_vision_control`
- bit 4: `search_when_target_lost`

The MCU stores the packet in `sentry_ext` and applies it to chassis `vx/vy/wz` plus the gimbal scan, vision takeover, and target-lost search state machine in `robot_cmd.c`.

### 8.2 MCU to Radar/Bridge: `ST`, 27 Bytes

```text
head[2]           = 'S','T'
cmd_vx            float
cmd_vy            float
cmd_wz            float
robot_status      uint8
game_status       uint8
stage_remain_time uint16
robot_id          uint8
current_hp        uint16
shooter_heat      uint16
team_color        uint8
is_attacked       uint8
crc16             uint16
```

ST reports the chassis command actually accepted by the MCU and a compact referee-system state. `robot_status` remains a bitfield derived from referee power-management outputs. `is_attacked` is a short MCU-side latch inferred from an HP decrease, not an unmodified referee field.

### 8.3 How the Bridge Currently Builds SX

- `vx/vy/wz` come from radar-side velocity input.
- `gimbal_yaw_delta` and `gimbal_pitch_delta` are currently `0.0`.
- `control_flags`, `scan_yaw_rate_deg_s`, and `search_pitch_deg` come from the `A3 RobotControl` function frame.

The bridge therefore carries both chassis velocity and gimbal scan/vision-takeover controls. Direct radar-side gimbal trim is still not enabled.

## 9. Legacy Radar-Side Protocol

### 9.1 Command Frame: 19 Bytes

```text
[0xA5][0x5A][vx:4][vy:4][wz:4][yaw_deg:4][CRC8:1]
```

- Total length: 19 bytes.
- `vx/vy/wz` are converted to SX.
- `yaw_deg` is parsed for compatibility but is not mapped to the SX gimbal-delta fields.

### 9.2 Telemetry and Status Packets

The bridge writes three packet types to the Radar PTY:

```text
A6 6A + vx + vy + wz + reserved0 + crc8
5C    + game_progress + stage_remain_time + crc16
5D    + robot_id + current_hp + shooter_heat + team_color + is_attacked + crc16
```

- `A6 6A` preserves compatibility with the legacy radar tool.
- `0x5C` and `0x5D` are generated from MCU ST data.
- `serial_sender.py --ros2` consumes the two status packets and publishes `/game_status` and `/robot_status`.

## 10. `serial_sender.py` Alignment

The repository version of `nyush-rm-vision/serial_sender.py` sends both:

- A 19-byte radar velocity frame.
- A 16-byte `A3 RobotControl` function frame.

### 10.1 Velocity Frame

Both bridge and sender now use:

```text
[0xA5][0x5A][vx:4][vy:4][wz:4][yaw_deg:4][CRC8:1]
```

### 10.2 Field Behavior

- `vx/vy/wz`: chassis commands.
- `yaw_deg`: compatibility field; the sender converts legacy `gimbal_yaw_rad` to degrees, while the bridge currently ignores it beyond parsing.
- `gimbal_pitch_rad`: retained in the legacy function signature but not transmitted in the 19-byte packet.

### 10.3 Current Result

- The repository `serial_sender.py` can connect directly to the Radar PTY.
- A separately copied `/home/nyu/Desktop/serial_sender.py` must be kept synchronized.
- Vision uses the Vision PTY; navigation uses the Radar PTY; only the bridge uses the real port.
- The bridge maps `RobotControl` scan and takeover fields into SX.
- ROS mode subscribes to `/cmd_vel_chassis_bt` and `/robot_control` and publishes ST-derived `/game_status` and `/robot_status`.
- `stop_gimbal_scan` remains only as a legacy compatibility flag.
- Direct `yaw_delta/pitch_delta` passthrough remains disabled.

## 11. ROS 2 Topic Paths

### 11.1 Navigation to Chassis

```text
Nav2 -> /cmd_vel -> fake_vel_transform -> /cmd_vel_chassis
     -> bt_comm_adapter.py -> /cmd_vel_chassis_bt
```

- `/cmd_vel`: raw Nav2 velocity.
- `/cmd_vel_chassis`: velocity after frame conversion.
- `/cmd_vel_chassis_bt`: result of merging chassis translation with `/robot_control.chassis_spin_vel`.

The Radar PTY sender should consume `/cmd_vel_chassis_bt`.

Angular-velocity policy since April 2026:

- `fake_vel_transform` may process Nav2 `wz` according to navigation configuration.
- `bt_comm_adapter.py` deliberately does not forward `/cmd_vel_chassis.angular.z`.
- Output `angular.z` comes only from `/robot_control.chassis_spin_vel`, preventing Nav2 rotation from competing with the sentry policy of translating without spin and spinning only while holding position.
- Re-enabling direct Nav2 yaw control requires changing the Python adapter logic, not only a launch parameter.

### 11.2 Decision Inputs and Vision

The behavior-tree executable registers subscribers for:

- `/detector/armors`
- `/game_status`
- `/robot_status`
- `/all_robot_hp`

Whether a specific tree uses a topic depends on its XML. The default `center_attack_simple` reads only `/game_status` and `/robot_status`; legacy trees such as `retreat_attack_left` also use armor detections and all-robot HP.

In the real-robot path, `serial_sender.py --ros2` publishes real `/game_status` and `/robot_status` from bridge telemetry. `bt_hotkey_debug.py` can override those topics for testing. `IsDetectEnemy` currently checks only whether the armor array is empty; it does not branch by hero, engineer, infantry, sentry, or outpost class.

### 11.3 Reserved Vision-to-Decision Path

Vision publishes a reserved topic:

- Topic: `auto_aim_target_pos`
- Type: `std_msgs/String`
- Payload: `x,y,z,target_id`
- `target_id`: currently `armor.name + 1`

Vision can therefore export a target class, but `nyush_rm_sentry` does not yet consume this topic.

## 12. Startup and Validation

### 12.1 Bridge Self-Test

```bash
cd /home/nyu/Codespace/nyush-rm-control
just py-bootstrap
just sentry-bridge --self-test
just sentry-bridge --port /dev/ttyACM0
```

Record both printed PTYs. For the optional MCU RTT dashboard, open another control-repository terminal:

```bash
just logger
```

Then visit `http://127.0.0.1:8080`. This is the MCU dashboard, not the vision detection page.

### 12.2 Vision Protocol Self-Test

Use the interactive tool against the Vision PTY before starting the full vision stack:

```bash
cd /home/nyu/Codespace/nyush-rm-control
just vision --port /dev/pts/3
```

It reads MCU SP packets and can manually transmit `VisionToGimbal`, making it useful for checking the bridge, MCU replies, frame length, CRC, and mode bits independently of vision.

### 12.3 Radar Protocol Self-Test

```bash
cd /home/nyu/Codespace/nyush-rm-control
just radar --port /dev/pts/4
```

Interactive commands:

```text
set vx vy wz [yaw_deg]
stop
rate hz
state
pulse vx vy wz sec
```

Use this to verify radar-frame-to-SX conversion, ST replies, and whether the MCU actually applies chassis commands.

### 12.4 Vision Through the Bridge

Point vision's `com_port` to the bridge's stable Vision symlink or current dynamic PTY, then run:

```bash
cd /home/nyu/Codespace/nyush-rm-vision
just test detect --web --send
```

- Vision web UI: `http://127.0.0.1:8888`.
- Set the left remote-control switch to the middle position to permit vision takeover.
- The current `detect` path follows a detected target; it is not the complete autonomous-search application.
- For bridge + BT + auto-aim integration, this command is preferred over treating `./build/sentry configs/sentry.yaml` as the default path.

### 12.5 Recommended Full Real-Robot Startup

Before starting:

- Flash the latest sentry firmware.
- Do not run `rm_serial_driver` against `/dev/ttyACM0`.
- Use one shell family consistently (`setup.bash` in Bash, `setup.zsh` in Zsh).
- RViz is disabled by default; enable it only with a graphical session.

Terminal 1 — MCU bridge:

```bash
cd /home/nyu/Codespace/nyush-rm-control
just sentry-bridge --port /dev/ttyACM0
```

Prefer stable symlinks if the bridge creates them:

```text
/tmp/nyush-rm-sentry-vision
/tmp/nyush-rm-sentry-radar
```

Terminal 1B — optional MCU dashboard:

```bash
cd /home/nyu/Codespace/nyush-rm-control
just logger
```

Terminal 2 — vision and auto-aim:

```bash
cd /home/nyu/Codespace/nyush-rm-vision
just test detect --web --send
```

Terminal 3 — LiDAR, navigation, behavior tree, and serial sender:

```bash
bash
cd /home/nyu/sentry_planner
START_SERIAL_SENDER=1 RADAR_PTY=/tmp/nyush-rm-sentry-radar ./start_robot.sh
```

If no stable symlink exists, replace `RADAR_PTY` with the printed PTY, for example `/dev/pts/4`. The default map is `rm_navigation_ws/src/rm_nav_bringup/map/RMUL2026.yaml`; override `MAP_FILE` only when intentionally changing maps.

`start_robot.sh` launches:

- `livox_ros_driver2`
- Static TF
- FAST-LIO
- `pointcloud_to_laserscan`
- `nav2_bringup`
- `bt_comm_adapter.py`
- `rm_behavior_tree`
- Optional `serial_sender.py`

Enable RViz when a display is available:

```bash
ENABLE_RVIZ=1 START_SERIAL_SENDER=1 \
  RADAR_PTY=/tmp/nyush-rm-sentry-radar ./start_robot.sh
```

Terminal 4 — optional behavior-tree hotkeys:

```bash
bash
source /opt/ros/humble/setup.bash
source /home/nyu/sentry_planner/rm_decision_ws/install/setup.bash
python3 /home/nyu/sentry_planner/scripts/bt_hotkey_debug.py
```

Keys: `0` home/standby, `1` approach center, `2` low-HP recovery, `3` high-heat recovery, `4` attacked branch, `p` print preset, `h` help, `q` quit. Reaching `CENTER_HOLD_ATTACK` still requires a real `map -> base_link` pose near the center goal.

Terminal 5 — optional Groot2 monitor:

```bash
cd ~/Desktop
./Groot2-v1.9.0-x86_64.AppImage
```

Open `rm_decision_ws/rm_behavior_tree/config/Project.btproj` and connect Monitor to `127.0.0.1:1667` with `enable_groot:=true`. Match the command to the AppImage version actually installed.

### 12.6 High-Value Integration Checks

```bash
lsof /dev/ttyACM0
ros2 topic echo /robot_control --once
ros2 topic echo /cmd_vel_chassis_bt --once
ros2 topic echo /game_status --once
ros2 topic echo /robot_status --once
ros2 topic echo /detector/armors --once
```

For live referee input:

```bash
ros2 topic hz /game_status
ros2 topic hz /robot_status
```

Expected state:

- Only `sentry_bridge.py` owns `/dev/ttyACM0`.
- `/robot_control` contains scan, vision-takeover, and spin fields.
- `/cmd_vel_chassis_bt` contains navigation translation and tactical spin.
- Live referee data supplies `/game_status` and `/robot_status` without manual publishers.
- `/detector/armors` becomes non-empty when vision detects a target.

Operational notes:

- Start with the right switch up so the robot is `READY`.
- Move the left switch to the middle only when vision takeover should be enabled.
- Navigation and BT can be tested before enabling vision takeover.
- `detect --web --send` follows detected targets but is not a full autonomous-search program.

### 12.7 Manual ROS 2 Testing Without the Full Tree

Prerequisites:

- Run `just sentry-bridge --port /dev/ttyACM0` first.
- Run the repository `serial_sender.py --ros2 --topic /cmd_vel_chassis_bt` against the Radar PTY.
- Release emergency stop and avoid the right-switch local-spin position.

Publish BT functional control continuously:

```bash
source /opt/ros/humble/setup.zsh
source /home/nyu/sentry_planner/rm_decision_ws/install/setup.zsh
ros2 topic pub -r 20 /robot_control rm_decision_interfaces/msg/RobotControl \
"{stop_gimbal_scan: false, chassis_spin_vel: 0.0, scan_enabled: true, allow_vision_control: false, search_when_target_lost: false, scan_yaw_rate_deg_s: 90.0, search_pitch_deg: 0.0}"
```

Publish chassis velocity from another terminal:

```bash
source /opt/ros/humble/setup.zsh
ros2 topic pub -r 20 /cmd_vel_chassis_bt geometry_msgs/msg/Twist \
"{linear: {x: 0.20, y: 0.00, z: 0.00}, angular: {x: 0.00, y: 0.00, z: 0.00}}"
```

Both topics must refresh continuously: the MCU accepts bridge chassis control only while the BT control flag is valid. A sender started by `start_robot.sh` listens to `/cmd_vel_chassis_bt`, not `/cmd_vel_chassis`. The sender log line `[NAV2 -> STM32] vx=...` is the quickest confirmation that velocity reached the sender.

Pure gimbal scan, with fixed pitch:

```bash
ros2 topic pub -r 20 /robot_control rm_decision_interfaces/msg/RobotControl \
"{stop_gimbal_scan: false, chassis_spin_vel: 0.0, scan_enabled: true, allow_vision_control: false, search_when_target_lost: false, scan_yaw_rate_deg_s: 120.0, search_pitch_deg: -6.0}"
```

Search when no target, then allow vision takeover:

```bash
ros2 topic pub -r 20 /robot_control rm_decision_interfaces/msg/RobotControl \
"{stop_gimbal_scan: true, chassis_spin_vel: 0.0, scan_enabled: true, allow_vision_control: true, search_when_target_lost: true, scan_yaw_rate_deg_s: 120.0, search_pitch_deg: -6.0}"
```

The second form matches the MCU auto-aim semantics: yaw and pitch search while no target is present, followed by vision tracking after detection. It requires `just test detect --web --send`. The test path does not fire by default; `sent_ctl=true` in its log means vision has taken control.

Common field mistakes:

- Stopping `/robot_control` causes BT takeover to time out.
- The right-switch local sentry-spin mode overrides the BT/vision gimbal path.
- Pure scan fixes pitch; auto-aim search moves pitch.
- Vision control packets may interrupt a pure-scan test, so stop vision for an isolated scan test.

### 12.8 Missing `/robot_control` Subscriber in ROS Mode

A frequent symptom is that the bridge and sender run and `/cmd_vel_chassis` has a subscriber, yet `/robot_control` has none and the chassis does not move.

The usual cause is sourcing only `/opt/ros/humble/setup.zsh` without the sentry workspace overlay. The process can import base ROS messages but not `rm_decision_interfaces/msg/RobotControl`, so it silently degrades to Twist-only operation. Because the MCU requires a valid BT/bridge control flag, velocity alone is insufficient.

Check both topics:

```bash
source /opt/ros/humble/setup.zsh
source /home/nyu/sentry_planner/install/setup.zsh
ros2 topic info /cmd_vel_chassis -v
ros2 topic info /robot_control -v
```

Both subscription counts should be at least one. If `/robot_control` is zero, restart the sender after sourcing the overlay:

```bash
source /opt/ros/humble/setup.zsh
source /home/nyu/sentry_planner/install/setup.zsh
python3 /home/nyu/Codespace/nyush-rm-vision/serial_sender.py \
  --port /tmp/nyush-rm-sentry-radar \
  --ros2 \
  --topic /cmd_vel_chassis \
  --robot-control-topic /robot_control
```

Legacy `--keyboard` mode sends only the velocity frame and no BT control flag, so it normally cannot drive the current bridge/MCU path. Prefer `--ros2` for bridge diagnosis.

## 13. Active, Reserved, and Conflicting Paths

### 13.1 Active or Functionally Complete

- One MCU port carries SP and SX/ST.
- The bridge splits one physical port into two PTYs.
- Vision `io::Gimbal` connects directly to the Vision PTY.
- Nav2 produces `/cmd_vel_chassis` through an explicit conversion path.
- `bt_comm_adapter.py` produces `/cmd_vel_chassis_bt` from translation and tactical spin.
- All-robot HP uses the unified `/all_robot_hp` topic.
- `/robot_control`, `/game_status`, and `/robot_status` are carried through the current bridge path.

### 13.2 Reserved but Not End-to-End

- Vision-to-decision `auto_aim_target_pos` and its target-class feedback.

### 13.3 Do Not Combine

Do not run `rm_serial_driver` and `sentry_bridge.py` against the same physical port. If the MCU uses the new bridge path, the legacy ROS 2 serial driver must not open `/dev/ttyACM0`.

## 14. Field Troubleshooting

### 14.1 Physical-Port Owner

```bash
lsof /dev/ttyACM0
```

Only `sentry_bridge.py` should appear.

### 14.2 Bridge PTYs

The bridge must print both `Vision PTY` and `Radar PTY`. Their `/dev/pts/X` numbers may change after every restart.

### 14.3 Navigation Velocity

```bash
source /opt/ros/humble/setup.bash
source /home/nyu/sentry_planner/install/setup.bash
ros2 topic hz /cmd_vel /cmd_vel_chassis
```

### 14.4 Vision Detection

```bash
ros2 topic echo /detector/armors --once
```

### 14.5 Behavior-Tree Vision Input

The current `IsDetectEnemy` condition treats any non-empty armor array as an enemy. Publish fixture data with:

```bash
cd /home/nyu/sentry_planner
source rm_decision_ws/install/setup.bash
source rm_vision_ws/install/setup.bash
./rm_decision_ws/rm_decision_interfaces/publish_script.sh
```

### 14.6 Radar-Side Round Trip

```bash
cd /home/nyu/Codespace/nyush-rm-control
just radar --port <Radar PTY>
```

Receiving telemetry verifies the protocol loop `Radar PTY -> bridge -> MCU -> ST -> bridge -> Radar PTY`.

## 15. Known Risks and Constraints

### 15.1 Protocol Drift

At least two historical radar-host frame formats exist: the current 19-byte bridge format and an older 23-byte sender format. Keep copied senders synchronized with the repository implementation.

### 15.2 PTY Names Are Not Stable

`/dev/pts/X` can change whenever the bridge restarts. Prefer bridge-created stable symlinks or export the printed PTYs into the launch environment immediately.

### 15.3 Real-Robot Communication Summary

- Keep the bridge as the only real USB owner.
- Point `serial_sender` to the current Radar PTY, optionally through `START_SERIAL_SENDER=1` and `RADAR_PTY=...`.
- `/cmd_vel_chassis_bt` and `/robot_control` reach the MCU through sender/A3 and SX.
- Vision `com_port` must point to the Vision PTY.
- `start_robot.sh` does not launch the bridge.

### 15.4 Target Class Is Not Yet Used by the Behavior Tree

The tree mainly consumes the non-empty state of `/detector/armors`. Although vision exports `target_id` through `auto_aim_target_pos`, the planner does not subscribe to or branch on it yet.

### 15.5 Do Not Run the Legacy and Bridge Serial Stacks Together

`rm_serial_driver` is the old ROS 2 serial stack; `sentry_bridge.py` is the new single-port, dual-path bridge. Running both only creates ownership conflicts.

### 15.6 Related Documentation

| Topic | Document |
|---|---|
| Behavior-tree XML, `SendGoal`, `RobotControl`, and tactics | [README_BEHAVIOR_TREE_FLOW_EN.md](README_BEHAVIOR_TREE_FLOW_EN.md) |
| End-to-end paths, environment variables, maps, and real-robot terminals | [README_COMMANDS_EN.md](README_COMMANDS_EN.md) |
| Gazebo RMUL2026 and Sim2Real | [README_LIDAR_EN.md](README_LIDAR_EN.md) and [mid360 command.txt](mid360%20command.txt) |
| Mid-360, FAST-LIO, Nav2, and localization behavior | [README_LIDAR_EN.md](README_LIDAR_EN.md) |
| One-page project overview | [README_EN.md](README_EN.md) |

## 16. Recommended Convergence Work

1. Extend `/robot_control` fields across the full competition state machine. The scan, vision-takeover, target-lost search, yaw-rate, and pitch fields already reach the MCU; they should be applied systematically by more behavior-tree branches instead of emulating remote-control positions.
2. Feed more real referee data into decisions. `/game_status` and `/robot_status` already reach ROS through the bridge; adding fields such as `all_robot_hp` would reduce remaining test fallbacks.

The single most important rule is:

```text
Give the physical /dev/ttyACM0 only to sentry_bridge.py.
Connect vision to the Vision PTY.
Connect radar/navigation to the Radar PTY.
Use the repository serial_sender.py, which is aligned with the bridge protocol.
```
