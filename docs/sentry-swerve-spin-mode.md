# Swerve small-gyro mode migration design

Status: design only. This full-stack snapshot does not enable small-gyro mode
and does not change the command protocol for it.

## 1. Decision

Small gyro must remain a chassis command mode that combines planar translation
with deliberate yaw. It must not become a hard-coded four-wheel pose, and the
STM32 must not secretly inject yaw into a Nav2 command. The final commanded
`vx`, `vy`, and `wz` must be visible in ROS, recorded in telemetry, constrained
with the real module geometry, and reflected in the localization feedback.

The recommended delivery has two stages:

1. A low-speed ROS overlay validates mechanics, localization, saturation, and
   command arbitration in open space.
2. Competition navigation moves the spin objective into a Nav2-aware controller
   or MPPI extension so its rollout model predicts the same yaw the robot runs.

Stage 1 is not considered safe for close-obstacle competition navigation.

## 2. What the legacy sentry actually did

The reference is `NYUSH-Robotics-Club/nyush-rm-control` commit `2b7ca72`:

- `application/robot_def.h` defined `CHASSIS_ROTATE`.
- `application/cmd/robot_cmd.c` used keyboard Q to toggle rotate mode and
  restored the last non-rotate mode when Q was pressed again.
- `application/chassis/chassis.c` preserved `vx`/`vy` and replaced `wz` with
  `CHASSIS_ROTATE_SPEED` before projecting translation into the chassis frame.
- The comment explicitly described self-spin with omnidirectional translation
  and left irregular spin-speed modulation as future work.

Only that control abstraction is reusable. The old `4000.0f` constant used the
mecanum controller's internal units, not rad/s. Its `SwerveCalculate()` simply
called `MecanumCalculate()`, so it provides no valid steering-module kinematics
for this robot.

## 3. Current chassis facts

The current firmware already accepts SI-derived body commands through USB:

- ROS axes: `+x` forward, `+y` left, positive yaw counter-clockwise
- firmware payload: mm/s, mm/s, mrad/s
- translation hard limit: 0.80 m/s
- yaw hard limit: 2.40 rad/s
- drive-motor limit: 2500 RPM
- module rectangle: 0.350 m wheelbase by 0.300 m track width
- wheel radius: 0.060 m; drive reduction: 19:1
- command watchdog: 0.30 s

For module position `(x_i, y_i)`, the physical module vector is:

```text
v_ix = vx - wz * y_i
v_iy = vy + wz * x_i
```

`CalculateSwerveTargets()` already converts the ROS left-positive convention to
the motor mounting convention, selects the shorter steering solution using a
180-degree drive reversal with hysteresis, preserves all four wheel-speed ratios,
and applies one common drive-alignment scale. Small gyro needs no new wheel
equation and no fixed steering pose table.

The module radius is:

```text
sqrt(0.175^2 + 0.150^2) = 0.2305 m
```

Pure spin therefore consumes about 0.2305 m/s of wheel ground speed per rad/s,
or about 697 motor RPM per rad/s. The 2500 RPM drive limit corresponds to about
0.827 m/s at the ground. Examples before vector-dependent saturation:

| Translation | Spin | Worst-case wheel speed upper bound |
| --- | --- | --- |
| 0.00 m/s | 0.50 rad/s | 0.115 m/s (~349 RPM) |
| 0.45 m/s | 1.00 rad/s | 0.681 m/s (~2060 RPM) |
| 0.45 m/s | 1.50 rad/s | 0.796 m/s (~2408 RPM) |
| 0.80 m/s | 1.00 rad/s | 1.031 m/s (must be limited) |

The upper bound assumes translation and tangential velocity align at one module;
the exact four vectors must still be calculated for every command.

## 4. Why firmware-only yaw injection is rejected

If STM32 silently adds a spin rate, Nav2 publishes one Twist but the robot runs
another. This breaks controller prediction, recorded command truth, acceleration
limits, saturation analysis, and debugging. It is especially unsafe here because:

- MPPI currently samples `wz_max=0.10 rad/s` for path following.
- `TwirlingCritic` actively penalizes rotation.
- the 0.55 m square footprint rotates relative to nearby obstacles; the
  circumscribed body radius is about 0.389 m.
- FAST-LIO must track the real rapid yaw for the next controller cycle.

A hidden override would make MPPI evaluate collision and path costs for a nearly
non-rotating rollout while the physical footprint spins.

## 5. Stage-1 ROS integration

Add a package-owned node, tentatively `spin_command_overlay.py`, after Nav2's
velocity smoother and before `twist_mux`:

```text
Nav2 controller -> velocity_smoother -> /cmd_vel_nav_smoothed
                                           │
                                           ▼
                                  spin_command_overlay
                                           │ /cmd_vel_nav_spin
                                           ▼
browser WASD --------------------------> twist_mux -> /cmd_vel -> USB bridge
emergency stop --------------------------->  ▲
```

Change only the navigation input in `twist_mux.yaml` to
`/cmd_vel_nav_spin`. WASD retains priority 100 and emergency stop retains 255.
The overlay must never sit after `twist_mux`, because doing so could modify a
manual or emergency-stop result.

Suggested public interface without new message types:

| Interface | Purpose |
| --- | --- |
| subscriber `/cmd_vel_nav_smoothed` | translation/yaw selected by Nav2 |
| subscriber `/odom` | actual yaw rate and freshness gate |
| service `/small_gyro/set_enabled` (`std_srvs/SetBool`) | explicit mode latch |
| parameter `spin_rate` | signed target yaw rate |
| publisher `/cmd_vel_nav_spin` | complete command sent to arbitration |
| publisher `/small_gyro/active` (`std_msgs/Bool`) | operator-visible state |
| diagnostics | stale input, limiting reason, requested/applied rate |

The node state machine is `OFF -> RAMP_UP -> ACTIVE -> RAMP_DOWN -> OFF`, with
a transition from every state to `FAULT`. `FAULT` publishes zero until a fresh
command, fresh odometry, valid parameters, and a new explicit enable request are
present. Process exit must also cause `twist_mux` timeout to produce zero.

## 6. Stage-1 command algorithm

For every input Twist:

1. Reject non-finite values and stale command or odometry timestamps.
2. Preserve Nav2's current body-frame `vx` and `vy`; do not rotate them again.
   Nav2 recomputes body-frame translation from the latest pose each cycle.
3. Ramp the signed spin target using a monotonic clock.
4. Combine the selected spin rate with the input command.
5. Evaluate all four module vectors with the exact firmware geometry.
6. Apply the configured saturation policy before publishing.
7. Publish status with requested/applied Twist and the limiting module.

Two saturation policies should be explicit configuration, never implicit:

- `balanced`: multiply `vx`, `vy`, and `wz` by one scale. This preserves the
  requested instantaneous center of rotation and is the safest default.
- `spin_priority`: preserve `wz` and reduce translation until every wheel fits.
  This preserves defensive spin but can slow or stop path progress.

A translation-priority mode is not initially recommended because silently
losing defensive yaw defeats the purpose of small gyro.

Initial conservative Stage-1 parameters:

| Parameter | Initial value |
| --- | --- |
| `spin_rate` | `+0.50 rad/s` |
| `angular_acceleration` | `1.5 rad/s^2` |
| `command_timeout` | `0.30 s` |
| `odom_timeout` | `0.20 s` |
| `max_wheel_ground_speed` | `0.80 m/s` (below the 0.827 m/s theoretical cap) |
| saturation policy | `balanced` |
| minimum obstacle clearance for first tests | `1.0 m` |

Do not begin at the firmware ceiling. Increase by no more than 0.25 rad/s per
validated test step.

## 7. Stage-2 Nav2-aware implementation

The Stage-1 overlay changes the executed yaw after MPPI has evaluated its
rollouts. Low-rate open-space testing is useful, but the model mismatch remains.
Competition implementation should make the spin rate part of the controller's
predicted command sequence. Candidate approaches, in preference order:

1. Extend/fork the Humble MPPI controller so the nominal control sequence
   includes a bounded spin reference and every rollout propagates it.
2. Add a custom MPPI critic/control constraint that rewards the requested yaw
   rate while the mode is active and remove/disable `TwirlingCritic` then only.
3. Implement a holonomic path controller with translation/yaw decoupling and
   full footprint collision checking.

The selected controller must still publish the complete final Twist through the
normal velocity smoother and `twist_mux`. `yaw_goal_tolerance=6.28` already
allows goal completion independent of final body heading, but controller
progress and recovery behavior must be retested under continuous spin.

## 8. Localization requirements

Small gyro depends on FAST-LIO; wheel odometry is not an acceptable fallback.
The MID-360 IMU lets FAST-LIO estimate motion and bias while deskewing the cloud,
but continuous rotation increases sensitivity to time synchronization,
mechanical LiDAR movement, self-points, CPU backlog, and weak geometry.

Required gates before navigation spin is enabled:

- `/Odometry` and `/scan` remain fresh at their expected rates.
- map-frame pose has no discontinuity during several stationary turns.
- the LiDAR mounting yaw and `base_link -> livox_frame` transform are fixed and
  physically repeatable; a loose sensor cannot be solved in software.
- ICP has already accepted the correct global alignment.
- VNC/RViz rendering load does not increase sensor-message delay.

## 9. Implementation sequence

1. Add the overlay node, configuration, launch wiring, status, and unit-level
   command/saturation tests; leave it disabled by default.
2. Add a browser/CLI mode toggle with clear active indication and automatic
   disable on page disconnect.
3. Run raised-wheel tests and compare ROS requested/applied commands against
   STM32 dashboard module targets.
4. Run low-rate floor spin tests without Nav2.
5. Run wide open-space Nav2 routes using Stage 1 and quantify model mismatch.
6. Implement the Stage-2 controller integration before close-obstacle use.
7. Add the mode only to the `sentry_swerve` configuration; do not change other
   robot types or reusable motor modules.

## 10. Acceptance criteria

Raised chassis:

- Pure spin and spin plus W/A/S/D produce the expected four module vectors.
- No module has sustained steering oscillation, flip chatter, current limit,
  or opposite drive sign.
- Disable, stale command, stale odometry, USB loss, and emergency stop all
  produce zero drive command within the defined timeout.

Floor, open space:

- At 0.50 rad/s pure spin, chassis-center drift stays below 0.15 m over 10 s.
- Commanded versus FAST-LIO measured yaw rate has no sustained error above 10%.
- `/Odometry` has no jump above 0.10 m or 5 degrees between samples.
- Steering error stays inside the full-drive alignment band in steady spin.
- USB command freshness and all eight motor-online bits remain valid.

Navigation:

- Wide outbound and return routes each complete at least five consecutive runs.
- No progress-checker oscillation, recovery loop, unexplained stop/go cycle, or
  path departure is attributable to spin.
- Costmap/footprint collision checks use the executed yaw trajectory.
- Disabling spin during motion ramps cleanly back to ordinary Nav2 control.

Only after all criteria pass should spin rate or translation speed be raised.
