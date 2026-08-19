# Jetson ROS 2 full-stack reproduction

This directory is the source of truth for the sentry's Jetson stack. It keeps
the project-owned ROS 2 packages beside the STM32 firmware, pins every external
source dependency to a commit, records the required FAST-LIO changes as a
reviewable patch, and includes the latest validated `arena` map set.

The setup has no localization fallback: navigation shuts down if FAST-LIO or
its odometry adapter exits. ICP is the sole owner of `map -> odom`; FAST-LIO is
the sole navigation source for `odom -> base_link`. `/wheel/odom` remains a
diagnostic topic until dynamic wheel-odometry validation is complete.

## Contents

- `dependencies.repos`: exact FAST-LIO, Livox, and pcd2pgm Git commits
- `patches/fast_lio_mid360.patch`: mandatory density and latency changes
- `setup_workspace.sh`: non-destructive source workspace bootstrap/check
- `src/mid360_nav_bringup`: launches, configuration, bridge, WASD, RViz
- `src/icp_probe`: guarded full-map ICP/GICP initial localization
- `src/mid360_nav_bringup/maps/arena.*`: validated matching PCD/PGM/YAML set
- `system/`: MID-360 Ethernet and persistent TigerVNC host configuration
- `TESTED_ENVIRONMENT.md`: software/toolchain versions used for validation

## Tested robot assumptions

| Item | Value |
| --- | --- |
| Chassis | four-module omnidirectional swerve |
| Body envelope | `0.55 x 0.55 x 0.50 m` |
| ROS convention | `+x` forward, `+y` left, `+z` up, positive yaw CCW |
| MID-360 pose in `base_link` | `xyz = 0.0, -0.11, 0.35 m`, zero RPY |
| FAST-LIO IMU-to-LiDAR translation | `[-0.011, -0.02329, 0.04412] m` |
| Jetson LiDAR NIC | `enP8p1s0`, `192.168.1.50/24`, no default route |
| MID-360 | `192.168.1.182` |
| ROS domain / RMW | `42` / Cyclone DDS |
| STM32 command link | USB CDC, CRC16, 20 Hz telemetry |
| VNC | TigerVNC system display `:1`, TCP 5901 |

Re-measure the chassis, sensor offset, sensor orientation, wheel radius,
module geometry, motor directions, and steering zero encoders before applying
this configuration to another robot. Matching the MID-360 model is not enough.

## 1. Host packages

Start from Ubuntu 22.04 and ROS 2 Humble desktop. Install the build/runtime
dependencies (rosdep may add transitive packages):

```bash
sudo apt update
sudo apt install \
  build-essential cmake git libapr1-dev libeigen3-dev libpcl-dev \
  python3-colcon-common-extensions python3-rosdep python3-vcstool \
  python3-serial \
  ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-nav2-mppi-controller ros-humble-nav2-smac-planner \
  ros-humble-pcl-conversions ros-humble-pointcloud-to-laserscan \
  ros-humble-rmw-cyclonedds-cpp ros-humble-robot-localization \
  ros-humble-slam-toolbox ros-humble-twist-mux ros-humble-xacro
sudo rosdep init  # omit this line if rosdep was initialized already
rosdep update
```

Configure the dedicated Ethernet link and persistent VNC session by following
[`system/README.md`](system/README.md). Do not commit `~/.vnc/passwd` or Wi-Fi
credentials.

## 2. Recreate the source workspace

From the firmware repository root:

```bash
./ros2_ws/setup_workspace.sh
```

The default target is `~/robot_ws`; set `ROBOT_WS=/absolute/path` to use a
different workspace. The script imports the exact commits, links the two
project packages from this repository, applies the mandatory FAST-LIO patch,
and refuses to overwrite or silently switch an existing checkout.

The resulting source layout is:

```text
robot_ws/src/
├── FAST_LIO_ROS2        pinned upstream + recorded patch
├── Livox-SDK2           pinned upstream SDK
├── livox_ros_driver2    pinned upstream ROS 2 driver
├── pcd2pgm              pinned upstream converter
├── icp_probe            symlink to this repository
└── mid360_nav_bringup   symlink to this repository
```

## 3. Build Livox and ROS 2

Livox-SDK2 installs a system library required by `livox_ros_driver2`:

```bash
cd "${ROBOT_WS:-$HOME/robot_ws}/src/Livox-SDK2"
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
sudo cmake --install build
sudo ldconfig
```

Build the workspace:

```bash
cd "${ROBOT_WS:-$HOME/robot_ws}"
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

The FAST-LIO patch is required, not optional. It uses every MID-360 point
(`point_filter_num=1`), changes both voxel filters from 0.50 to 0.35 m, changes
the LiDAR subscription to latest-sample sensor QoS, and avoids constructing
large point-cloud/path messages when no subscriber exists. On this Jetson it
reduced the observed navigation data delay from roughly 2.1 s to about
0.12-0.19 s while retaining enough geometry for ICP.

## 4. Build and flash the STM32 firmware

The robot-specific firmware selection is `sentry_swerve`:

```bash
printf 'sentry_swerve\n' > .robot_type
just rebuild
just flash jlink
```

Flashing is a deliberate hardware action; it is never performed by bootstrap
or validation scripts. `just flash` auto-detects a transport, while
`just flash jlink`, `just flash pyocd`, and `just flash dfu` select one
explicitly. Stop the RTT logger before sharing a debug probe with flashing.

The controller consumes body-frame `vx`, `vy`, and `wz`, computes continuous
four-module swerve vectors, applies 180-degree flip hysteresis, waits for common
steering alignment, ramps commands, brakes briefly at zero, and stops on stale
USB commands or missing motor feedback. Configuration is centralized in
`application/robot_configs/robot_sentry_swerve.h`.

## 5. Daily commands

All daily entry points live in the repository Justfile. They use
`${ROBOT_WS:-$HOME/robot_ws}` and directly execute the version-controlled
package scripts; no untracked wrapper or shell alias is required.

```bash
just map arena          # MID-360 + FAST-LIO mapping + RViz
just wasd               # browser teleop, higher priority than Nav2
just map-save arena     # save matching PCD + filtered PGM/YAML
just nav arena          # FAST-LIO + guarded ICP + Nav2 MPPI + RViz
just logger             # RTT dashboard at http://JETSON_IP:8765/
```

The stable WASD URL `http://JETSON_IP:8088/` has no access token by default,
matching the current LAN workflow. Any host that can reach that port can request
control after pressing the page's enable button. On a shared network, pass a
fresh `--token`, bind to localhost and tunnel over SSH, or restrict port 8088 in
the firewall. The RTT dashboard on 8765 is also unauthenticated and should be
treated as trusted-LAN diagnostics.

`just nav` deliberately requires a map name. There is no hidden fallback to
`/home/nyu/b1.*`. A custom pair may be selected with `NAV_MAP_YAML` and
`NAV_PCD_MAP`; the PCD and YAML/PGM must originate from the same mapping run.

Mapping and navigation share an exclusive lock because only one process may
own the MID-360, FAST-LIO frames, chassis bridge, and command mux. Stop the
active stack with Ctrl-C before switching modes. Start mapping/navigation
before starting a separate `just wasd`; WASD then reuses the active bridge.

The checked-in `arena` set can be verified with:

```bash
cd ros2_ws/src/mid360_nav_bringup/maps
sha256sum -c SHA256SUMS
```

## 6. Runtime data flow and TF ownership

```text
MID-360 CustomMsg + IMU
        │
        ▼
FAST-LIO ── /Odometry + /cloud_registered_body
        │                         │
        │                         ├── pointcloud_to_laserscan ── /scan
        │                         └── guarded ICP vs saved PCD
        ▼                                      │
fastlio_odom_adapter                           └── map -> odom
        │
        └── /odom + odom -> base_link
                         │
map server + Nav2 MPPI ──┴── /cmd_vel_nav_smoothed
                                      │ priority 10
browser WASD ── /cmd_vel_teleop ──────┤ priority 100
emergency stop ────────────────────────┤ priority 255
                                      ▼
                                  twist_mux
                                      ▼ /cmd_vel
                              sentry_nav2_bridge
                              USB CDC ⇄ STM32
                                      └── /wheel/odom (diagnostic)
```

Expected TF chain:

```text
map -> odom -> base_link -> livox_frame
```

Do not run AMCL or another localization publisher with `navigation_icp`; two
`map -> odom` publishers make the robot jump. Do not enable EKF TF publication
beside the FAST-LIO adapter; two `odom -> base_link` publishers cause the same
failure. The legacy 2D SLAM/AMCL launch files remain diagnostic alternatives,
not automatic fallbacks.

## 7. Localization and navigation behavior

ICP uses the motion-compensated FAST-LIO body cloud and the full raw PCD. The
current startup search spans `+/-2.25 m` in X/Y and all headings: 1225 rough
candidates, 0.40 m rough voxels, then 0.10 m GICP refinement. Acceptance gates
are fitness `<=0.10`, overlap `>=0.80`, inlier RMSE `<=0.15 m`, and bounded
correction from the seed. The full search may take roughly 50 seconds. Keep
the robot still; use RViz `2D Pose Estimate` as a new search seed when needed.

After acceptance, ICP keeps broadcasting the fixed initial `map -> odom`
alignment while FAST-LIO continuously tracks local motion. This is initial
global localization plus continuous LiDAR-inertial odometry, not continuous
map-to-scan relocalization. Re-seed if global drift becomes unacceptable.

Nav2 uses an Omni MPPI motion model, 20 Hz controller, 0.45 m/s autonomous
translation limit, low path-following yaw authority, a 0.55 m square footprint,
voxel obstacles from `/scan`, and a 0.60 m inflation radius. Browser WASD may
command up to the firmware's 0.80 m/s and 2.40 rad/s hard limits. Review the
costmap and stopping distance before increasing autonomous speed.

## 8. Health checks

With navigation running:

```bash
ros2 run mid360_nav_bringup check_navigation.sh
ros2 topic hz /livox/lidar
ros2 topic hz /Odometry
ros2 topic hz /scan
ros2 run tf2_ros tf2_echo map base_link
ros2 topic echo /icp_probe/pose --once
```

For the lower controller, run `just logger`, open
`http://JETSON_IP:8765/`, and inspect steering target/actual/error, steering
speed/current, drive target/actual RPM, command freshness, online bitmaps,
flip state, and common drive-alignment scale.

The RViz profile intentionally renders at 1 FPS for VNC. A delayed green/blue
visual marker can therefore be a display delay even when `/Odometry` and
control are current; diagnose topic timestamps before changing control code.

## 9. Safety and calibration checklist

Before first powered motion on a restored machine:

1. Confirm all four steering zero ECD values and motor/CAN assignments.
2. Lift the chassis and test W/A/S/D/Q/E at low speed.
3. Confirm `+x` forward, `+y` left, and positive yaw counter-clockwise.
4. Verify each target/actual steering trace and drive RPM sign in the dashboard.
5. Confirm releasing input stops commands within the 0.30 s watchdog window.
6. Confirm emergency-stop lock prevents all `/cmd_vel` output.
7. Verify sensor IP, mounting translation, mounting yaw, and map checksums.
8. Validate TF ownership before sending a Nav2 goal.
9. Start in open space; validate footprint, inflation, and stopping distance.

The planned small-gyro mode is intentionally not enabled in this snapshot. Its
migration design and staged acceptance tests are documented in
[`../docs/sentry-swerve-spin-mode.md`](../docs/sentry-swerve-spin-mode.md).
