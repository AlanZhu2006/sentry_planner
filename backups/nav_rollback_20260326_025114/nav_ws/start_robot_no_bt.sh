#!/bin/bash

set -eo pipefail

# 无 BT 版本：保留当前导航、FAST-LIO、fake_vel_transform、serial_sender 通讯链，
# 只是不启动 bt_comm_adapter / rm_behavior_tree。
# 注意：当前下位机只有在收到 /robot_control 功能标志后，
# 才会接管外部导航速度；所以 no-BT 默认也要保留 keepalive。

export START_BT="${START_BT:-0}"
export START_FAKE_VEL_TRANSFORM="${START_FAKE_VEL_TRANSFORM:-1}"
export START_GIMBAL_CLOUD_COMPENSATION="${START_GIMBAL_CLOUD_COMPENSATION:-1}"
export GIMBAL_COMP_STATE_SOURCE="${GIMBAL_COMP_STATE_SOURCE:-constant_rate}"
export GIMBAL_COMP_CONSTANT_YAW_RATE_DEG_S="${GIMBAL_COMP_CONSTANT_YAW_RATE_DEG_S:-120.0}"
export GIMBAL_COMP_INITIAL_YAW_DEG="${GIMBAL_COMP_INITIAL_YAW_DEG:-0.0}"
export START_SERIAL_SENDER="${START_SERIAL_SENDER:-1}"
export START_ROBOT_CONTROL_KEEPALIVE="${START_ROBOT_CONTROL_KEEPALIVE:-1}"
export RADAR_PTY="${RADAR_PTY:-/tmp/nyush-rm-sentry-radar}"
export SERIAL_SENDER_TOPIC="${SERIAL_SENDER_TOPIC:-/cmd_vel_chassis}"

exec "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/start_robot.sh"
