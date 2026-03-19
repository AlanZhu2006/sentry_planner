#!/usr/bin/env bash

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SENTRY_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
BT_STYLE="${BT_STYLE:-center_attack_simple}"
USE_SIM_TIME="${USE_SIM_TIME:-True}"
BT_RESPAWN="${BT_RESPAWN:-False}"
ENABLE_GROOT="${ENABLE_GROOT:-False}"
GROOT_PORT="${GROOT_PORT:-1667}"

cleanup() {
  kill $(jobs -p) 2>/dev/null || true
}
trap cleanup EXIT INT TERM

if [ ! -w "$HOME/.ros/log" ] 2>/dev/null; then
  export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/ros_logs_behavior_chain}"
  mkdir -p "$ROS_LOG_DIR"
fi

source /opt/ros/humble/setup.bash
source "$SENTRY_ROOT/rm_vision_ws/install/setup.bash"
source "$SENTRY_ROOT/rm_decision_ws/install/setup.bash"

echo ">>> 清理旧的测试残留进程..."
pkill -f "$SENTRY_ROOT/scripts/bt_comm_adapter.py" 2>/dev/null || true
pkill -f "$SENTRY_ROOT/rm_decision_ws/install/rm_behavior_tree/lib/rm_behavior_tree/rm_behavior_tree" 2>/dev/null || true
sleep 1

for arg in "$@"; do
  if [ "$arg" = "--help" ] || [ "$arg" = "-h" ]; then
    python3 "$SCRIPT_DIR/test_behavior_chain.py" "$@"
    exit 0
  fi
done

echo ">>> 启动 bt_comm_adapter..."
python3 "$SENTRY_ROOT/scripts/bt_comm_adapter.py" &
sleep 2

echo ">>> 等待 navigate_to_pose action..."
ACTION_READY=0
for _ in $(seq 1 15); do
  if ros2 action list 2>/dev/null | grep -q "navigate_to_pose"; then
    ACTION_READY=1
    break
  fi
  sleep 1
done

if [ "$ACTION_READY" -ne 1 ]; then
  echo ">>> 警告: 15 秒内未看到 navigate_to_pose，继续启动行为树，测试里会再次检查"
fi

echo ">>> 启动 rm_behavior_tree ($BT_STYLE)..."
ros2 launch rm_behavior_tree rm_behavior_tree.launch.py \
  style:="$BT_STYLE" \
  use_sim_time:="$USE_SIM_TIME" \
  respawn:="$BT_RESPAWN" \
  enable_groot:="$ENABLE_GROOT" \
  groot_port:="$GROOT_PORT" &
sleep 3

echo ">>> 运行行为链测试..."
python3 "$SCRIPT_DIR/test_behavior_chain.py" "$@"
