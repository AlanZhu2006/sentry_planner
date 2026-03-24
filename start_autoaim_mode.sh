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
echo "    2. 已有进程在订阅 /robot_control 并通过 Radar PTY 转给 bridge"
echo "       - 最省事：直接用 start_robot.sh"
echo "       - 或者单独起 serial_sender --ros2 --robot-control-topic /robot_control"
echo "    3. 右拨杆不要放在本地自转档"

ros2 topic pub -r "$ROBOT_CONTROL_RATE" "$ROBOT_CONTROL_TOPIC" rm_decision_interfaces/msg/RobotControl \
    "{stop_gimbal_scan: $STOP_GIMBAL_SCAN, chassis_spin_vel: $AUTOAIM_CHASSIS_SPIN_VEL, scan_enabled: $SCAN_ENABLED, allow_vision_control: $ALLOW_VISION_CONTROL, search_when_target_lost: $SEARCH_WHEN_TARGET_LOST, scan_yaw_rate_deg_s: $AUTOAIM_SCAN_YAW_RATE_DEG_S, search_pitch_deg: $AUTOAIM_SEARCH_PITCH_DEG}" &

sleep 1
echo ""
echo ">>> 话题检查："
ros2 topic info "$ROBOT_CONTROL_TOPIC" || true

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
