# RTT Dashboard

This dashboard streams binary telemetry from STM32 over SEGGER RTT channel 1 and renders a live text view in terminal.

## Firmware protocol

Frame definition is implemented in `application/dashboard/dashboard.h`.

- Magic: `0x4452`
- Current version: `6` (host tools accept `3`, `4`, `5`, `6`)
- Header: `<HBBI` (`magic`, `version`, `payload_len`, `seq`)
- `payload_len` stores the low 8 bits of payload size. Version `6` uses fixed-size payload `269`, so header `payload_len=13`.
- Payload v6: `<Iff4f4f3f3f11fBHBBHHHHH6B8hBH3B5HfII7fBB15fH`
- CRC16: firmware `crc_16()` from `modules/algorithm/crc16.c`, reflected polynomial `0x8408`, init `0xFFFF`, over `header + payload`

Payload fields:

- `timestamp_ms`: MCU tick (`HAL_GetTick`, ms)
- `chassis_power`, `chassis_volt`: referee power telemetry (`W`, `V`)
- `motor_target_rpm[4]`: chassis wheel target speed `LF, RF, LB, RB` (`rpm`)
- `motor_rpm[4]`: chassis wheel measured speed `LF, RF, LB, RB` (`rpm`)
- `imu_angle_deg[3]`: IMU absolute `Pitch, Yaw, Roll` (`deg`)
- `imu_gyro_deg_s[3]`: IMU angular velocity `X, Y, Z` (`deg/s`)
- `gimbal_yaw_target_deg`, `gimbal_yaw_actual_deg`: yaw target/actual angle (`deg`)
- `gimbal_yaw_target_deg_s`, `gimbal_yaw_actual_deg_s`: yaw target/actual angular speed (`deg/s`)
- `gimbal_pitch_target_deg`, `gimbal_pitch_actual_deg`: pitch target/actual angle (`deg`)
- `gimbal_pitch_target_deg_s`, `gimbal_pitch_actual_deg_s`: pitch target/actual angular speed (`deg/s`)
- `gimbal_cmd_yaw_deg`, `gimbal_cmd_pitch_deg`, `gimbal_cmd_chassis_rotate_wz`: gimbal cmd snapshot
- Referee game/robot status:
  - `referee_game_state`: raw packed byte (`bit[3:0]=game_type`, `bit[7:4]=game_progress`)
  - `referee_stage_remain_time`
  - `referee_robot_id`, `referee_robot_level`
  - `referee_current_hp`, `referee_maximum_hp`
  - `referee_shooter_barrel_cooling_value`
  - `referee_shooter_barrel_heat_limit`, `referee_chassis_power_limit`
  - `referee_power_management_flags`: `bit0=gimbal`, `bit1=chassis`, `bit2=shooter`
  - `referee_chassis_current`, `referee_buffer_energy`
  - `referee_shooter_17mm_1_barrel_heat`, `referee_shooter_17mm_2_barrel_heat`, `referee_shooter_42mm_barrel_heat`
  - `referee_shoot_bullet_speed_mps`: measured shot speed from referee `0x0207` (`m/s`)
- `remote_packed`: `bit[3:0]=rc_switches`, `bit[5:4]=remote_protocol`, `bit[7:6]=remote_mode_switch(3 means unavailable)`
- `motor_online_bitmap`: `bit0..bit3 = LF, RF, LB, RB` motor online flags
- `link_bitmap_packed`: `bit[1:0]=gimbal_motor_online`, `bit[4:2]=shoot_motor_online`, `bit[6:5]=can_link`
- `status_flags`: system status bitfield:
  - `bit0`: referee online
  - `bit1`: remote online
  - `bit2`: IMU online
  - `bit3`: gimbal feed updated
  - `bit4`: chassis feed updated
  - `bit5`: gimbal cmd updated
  - `bit6`: chassis cmd updated
  - `bit7`: shoot feed updated
- `remote_state_flags`:
  - `bit[4:0]`: remote buttons (`pause`, `custom_left`, `custom_right`, `trigger`, `mouse_middle`)
  - `bit[7:5]`: remote control flags (`vision`, `keyboard/mouse`, `emergency stop`)
- RC detail:
  - `rc_rocker_l_x`, `rc_rocker_l_y`, `rc_rocker_r_x`, `rc_rocker_r_y`, `rc_dial`
  - `mouse_x`, `mouse_y`, `mouse_z`, `mouse_buttons(bit0=left, bit1=right)`
  - `key_pressed_bits`
- `mode_packed_low`: `chassis_mode`, `gimbal_mode`, `shoot_mode`, `shoot_load_mode`
- `mode_packed_high`: `shoot_lid_mode`, `shoot_friction_mode`, `shoot_bullet_speed_code`
- `shoot_rest_heat`, `shoot_rate`
- `telemetry_drop_count`: total dropped dashboard frames on MCU side
- `free_heap_bytes`: current FreeRTOS free heap size in bytes
- `chassis_cmd_vx`, `chassis_cmd_vy`, `chassis_cmd_wz`: current chassis cmd velocities
- `shoot_loader_speed_aps`, `shoot_friction_l_speed_aps`, `shoot_friction_r_speed_aps`: shoot motor speed feedback
- `vision_meta_flags`: `bit0=recv new_data`, `bit1=recv snapshot valid`, `bit2=send snapshot valid`
- `vision_sp_mode_packed`: `bit[3:0]=recv mode`, `bit[7:4]=send mode`

Behavior when some buses/devices are disconnected:

- RTT frame stream still works; Python should not crash.
- Missing CAN feedback usually shows `motor_online_bitmap=0x00` and motor values become `0` or stale.
- Missing remote/referee is reflected by `status_flags` online bits cleared.
- Dashboard UI keeps rendering with placeholders instead of blank panels while waiting for valid frames.
- Terminal/Web dashboard will switch remote display automatically according to `remote_protocol`.

## Build firmware

```bash
just build infantry 12
```

## Host setup (Jetson)

```bash
just py-bootstrap
```

`py-bootstrap` will auto-create `.venv` (if missing) and install `scripts/requirement.txt`.

## Run dashboard

```bash
just logger-cli
```

Defaults already built into script:

- `--device STM32H723VG`
- `--channel 1`
- `--speed 4000`
- `--connect normal`

You can still append extra options when needed, for example:

```bash
just logger-cli --serial 69655005 --refresh-ms 80
```

By default, backend selection is automatic (`--backend auto`): prefer pyOCD and
fall back to direct `pylink` RTT mode when needed. RTT telemetry channel is also
auto-detected unless `--no-auto-channel` is provided.

Useful options:

- `--serial <probe-serial>`
- `--backend auto|pyocd|jlink`
- `--connect under-reset`
- `--poll-ms 20`
- `--refresh-ms 100`
- `--mode dashboard|split`
- `--log-channel 0` (used in `split` mode)
- `NO_COLOR=1` to disable ANSI colors

## Split mode (left dashboard + right realtime logs)

```bash
just logger-cli --mode split --channel 1 --log-channel 0
```

- Left pane: telemetry dashboard (decoded binary frames).
- Right pane: realtime text logs from `--log-channel`.
- Recommended: keep telemetry and logs on separate RTT channels (for example `1` + `0`).

## Acceptance checks

1. Zero intrusion
   - Disconnect probe while robot is running.
   - Control loop continues (RTT channel 1 uses `SEGGER_RTT_MODE_NO_BLOCK_SKIP`).
2. Latency
   - Observe `latency=...ms` in top status bar.
   - Keep below 100 ms in normal operation.
3. Integrity
   - `crc_err` remains near zero.
   - `sync_drop` does not grow continuously.
4. Headless
   - Run directly in SSH terminal without X11.
