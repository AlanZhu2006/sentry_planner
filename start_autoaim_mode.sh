#!/usr/bin/env bash

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SENTRY_ROOT="$SCRIPT_DIR"
VISION_ROOT="${VISION_ROOT:-$HOME/Codespace/nyush-rm-vision}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
DECISION_SETUP="${DECISION_SETUP:-$SENTRY_ROOT/rm_decision_ws/install/setup.bash}"
PLANNER_SETUP="${PLANNER_SETUP:-$SENTRY_ROOT/install/setup.bash}"
ROBOT_CONTROL_TOPIC="${ROBOT_CONTROL_TOPIC:-/robot_control}"
ROBOT_CONTROL_RATE="${ROBOT_CONTROL_RATE:-20}"
VISION_PTY_PATH="${VISION_PTY_PATH:-/tmp/nyush-rm-sentry-vision}"
RADAR_PTY_PATH="${RADAR_PTY_PATH:-/tmp/nyush-rm-sentry-radar}"
REQUIRE_ROBOT_CONTROL_SUBSCRIBER="${REQUIRE_ROBOT_CONTROL_SUBSCRIBER:-1}"
AUTO_START_SERIAL_SENDER="${AUTO_START_SERIAL_SENDER:-1}"
SERIAL_SENDER_SCRIPT="${SERIAL_SENDER_SCRIPT:-$VISION_ROOT/serial_sender.py}"
SERIAL_SENDER_TOPIC="${SERIAL_SENDER_TOPIC:-/cmd_vel_chassis_bt}"
SERIAL_SENDER_WAIT_S="${SERIAL_SENDER_WAIT_S:-5}"
SERIAL_SENDER_EXTRA_ARGS="${SERIAL_SENDER_EXTRA_ARGS:-}"
AUTOAIM_MODE="${AUTOAIM_MODE:-autoaim}"
AUTOAIM_SCAN_YAW_RATE_DEG_S="${AUTOAIM_SCAN_YAW_RATE_DEG_S:-120.0}"
AUTOAIM_SEARCH_PITCH_DEG="${AUTOAIM_SEARCH_PITCH_DEG:--6.0}"
AUTOAIM_CHASSIS_SPIN_VEL="${AUTOAIM_CHASSIS_SPIN_VEL:-0.0}"
START_VISION="${START_VISION:-1}"
VISION_WEB_PORT="${VISION_WEB_PORT:-8888}"
VISION_EXTRA_ARGS="${VISION_EXTRA_ARGS:-}"

cleanup() {
    kill $(jobs -p) 2>/dev/null || true
}
trap cleanup EXIT INT TERM

if [ ! -f "$ROS_SETUP" ]; then
    echo "❌ ROS setup not found: $ROS_SETUP"
    exit 1
fi

if [ ! -f "$DECISION_SETUP" ]; then
    echo "❌ rm_decision_ws setup not found: $DECISION_SETUP"
    exit 1
fi

source "$ROS_SETUP"
source "$DECISION_SETUP"
if [ -f "$PLANNER_SETUP" ]; then
    source "$PLANNER_SETUP"
fi

if [ ! -e "$VISION_PTY_PATH" ]; then
    echo "❌ Vision PTY not found: $VISION_PTY_PATH"
    echo "   先确认 sentry_bridge 正在运行，并且已经创建 Vision PTY。"
    exit 1
fi

get_robot_control_info() {
    ros2 topic info "$ROBOT_CONTROL_TOPIC" 2>&1 || true
}

get_robot_control_subscribers() {
    local info="$1"
    local count
    count="$(printf '%s\n' "$info" | awk '/Subscription count:/ {print $3; exit}')"
    printf '%s\n' "${count:-0}"
}

wait_for_robot_control_subscriber() {
    local timeout_s="$1"
    local start_ts
    local elapsed
    local info
    local subscribers

    start_ts="$(date +%s)"
    while true; do
        info="$(get_robot_control_info)"
        subscribers="$(get_robot_control_subscribers "$info")"
        if [ "$subscribers" != "0" ]; then
            printf '%s\n' "$info"
            return 0
        fi

        elapsed=$(( $(date +%s) - start_ts ))
        if [ "$elapsed" -ge "$timeout_s" ]; then
            printf '%s\n' "$info"
            return 1
        fi
        sleep 0.5
    done
}

start_serial_sender() {
    if [ ! -f "$SERIAL_SENDER_SCRIPT" ]; then
        echo "❌ serial_sender script not found: $SERIAL_SENDER_SCRIPT"
        exit 1
    fi

    if [ ! -e "$RADAR_PTY_PATH" ]; then
        echo "❌ Radar PTY not found: $RADAR_PTY_PATH"
        echo "   先确认 sentry_bridge 正在运行，并且已经创建 Radar PTY。"
        exit 1
    fi

    echo ""
    echo ">>> 自动启动 serial_sender..."
    echo "    port              : $RADAR_PTY_PATH"
    echo "    topic             : $SERIAL_SENDER_TOPIC"
    echo "    robot_control     : $ROBOT_CONTROL_TOPIC"

    sender_args=(
        "$SERIAL_SENDER_SCRIPT"
        --port "$RADAR_PTY_PATH"
        --ros2
        --topic "$SERIAL_SENDER_TOPIC"
        --robot-control-topic "$ROBOT_CONTROL_TOPIC"
    )
    if [ -n "$SERIAL_SENDER_EXTRA_ARGS" ]; then
        # shellcheck disable=SC2206
        extra_args=($SERIAL_SENDER_EXTRA_ARGS)
        sender_args+=("${extra_args[@]}")
    fi

    python3 "${sender_args[@]}" &
}

case "$AUTOAIM_MODE" in
    autoaim)
        STOP_GIMBAL_SCAN=true
        SCAN_ENABLED=true
        ALLOW_VISION_CONTROL=true
        SEARCH_WHEN_TARGET_LOST=true
        MODE_DESC="视觉接管 + 丢目标回搜索"
        ;;
    scan)
        STOP_GIMBAL_SCAN=false
        SCAN_ENABLED=true
        ALLOW_VISION_CONTROL=false
        SEARCH_WHEN_TARGET_LOST=false
        MODE_DESC="纯扫描"
        ;;
    idle)
        STOP_GIMBAL_SCAN=true
        SCAN_ENABLED=false
        ALLOW_VISION_CONTROL=false
        SEARCH_WHEN_TARGET_LOST=false
        AUTOAIM_SCAN_YAW_RATE_DEG_S=0.0
        AUTOAIM_SEARCH_PITCH_DEG=0.0
        MODE_DESC="空闲保持"
        ;;
    *)
        echo "❌ Unsupported AUTOAIM_MODE: $AUTOAIM_MODE"
        echo "   Supported: autoaim | scan | idle"
        exit 1
        ;;
esac

echo ">>> 启动自瞄模式 keepalive..."
echo "    mode              : $AUTOAIM_MODE ($MODE_DESC)"
echo "    robot_control     : $ROBOT_CONTROL_TOPIC @ ${ROBOT_CONTROL_RATE}Hz"
echo "    scan_yaw_rate     : ${AUTOAIM_SCAN_YAW_RATE_DEG_S} deg/s"
echo "    search_pitch      : ${AUTOAIM_SEARCH_PITCH_DEG} deg"
echo "    chassis_spin_vel  : ${AUTOAIM_CHASSIS_SPIN_VEL} rad/s"
echo ""
echo ">>> 前置条件提醒："
echo "    1. bridge 已启动"
echo "    2. 右拨杆不要放在本地自转档"
echo "    3. 如果 /robot_control 还没人订阅，脚本会自动补起 serial_sender"

ros2 topic pub -r "$ROBOT_CONTROL_RATE" "$ROBOT_CONTROL_TOPIC" rm_decision_interfaces/msg/RobotControl \
    "{stop_gimbal_scan: $STOP_GIMBAL_SCAN, chassis_spin_vel: $AUTOAIM_CHASSIS_SPIN_VEL, scan_enabled: $SCAN_ENABLED, allow_vision_control: $ALLOW_VISION_CONTROL, search_when_target_lost: $SEARCH_WHEN_TARGET_LOST, scan_yaw_rate_deg_s: $AUTOAIM_SCAN_YAW_RATE_DEG_S, search_pitch_deg: $AUTOAIM_SEARCH_PITCH_DEG}" &

sleep 1
echo ""
echo ">>> 话题检查："
robot_control_info="$(get_robot_control_info)"
printf '%s\n' "$robot_control_info"
robot_control_subscribers="$(get_robot_control_subscribers "$robot_control_info")"

if [ "$AUTO_START_SERIAL_SENDER" = "1" ] && [ "$robot_control_subscribers" = "0" ]; then
    start_serial_sender
    sleep 1
    echo ""
    echo ">>> 等待 /robot_control 订阅者上线..."
    robot_control_info="$(wait_for_robot_control_subscriber "$SERIAL_SENDER_WAIT_S" || true)"
    printf '%s\n' "$robot_control_info"
    robot_control_subscribers="$(get_robot_control_subscribers "$robot_control_info")"
fi

if [ "$REQUIRE_ROBOT_CONTROL_SUBSCRIBER" = "1" ] && [ "$robot_control_subscribers" = "0" ]; then
    echo ""
    echo "❌ $ROBOT_CONTROL_TOPIC 当前没有订阅者。"
    echo "   这意味着 allow_vision_control / search_when_target_lost 不会被转发到下位机。"
    echo "   已尝试自动启动 serial_sender，但订阅仍未建立。"
    echo "   请检查 bridge、Radar PTY、以及 serial_sender 依赖环境。"
    exit 1
fi

if [ "$START_VISION" = "1" ]; then
    if [ ! -d "$VISION_ROOT" ]; then
        echo "❌ Vision repo not found: $VISION_ROOT"
        exit 1
    fi

    echo ""
    echo ">>> 启动视觉跟随：just test detect --web --send"
    echo "    web: http://127.0.0.1:${VISION_WEB_PORT}"

    vision_args=(test detect --web --send "--web-port=${VISION_WEB_PORT}")
    if [ -n "$VISION_EXTRA_ARGS" ]; then
        # shellcheck disable=SC2206
        extra_args=($VISION_EXTRA_ARGS)
        vision_args+=("${extra_args[@]}")
    fi

    cd "$VISION_ROOT"
    just "${vision_args[@]}"
else
    echo ""
    echo ">>> START_VISION=0，仅保持自瞄模式话题发布。按 Ctrl+C 退出。"
    wait
fi
