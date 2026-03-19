#!/usr/bin/env bash

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SENTRY_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
BT_STYLE="${BT_STYLE:-center_attack_simple}"
USE_SIM_TIME="${USE_SIM_TIME:-True}"
WATCH_STATE="${WATCH_STATE:-1}"
BT_RESPAWN="${BT_RESPAWN:-False}"
ENABLE_GROOT="${ENABLE_GROOT:-False}"
GROOT_PORT="${GROOT_PORT:-1667}"

cleanup() {
  echo ">>> 停止 debug session..."
  kill $(jobs -p) 2>/dev/null || true
}
trap cleanup EXIT INT TERM

if [ ! -w "$HOME/.ros/log" ] 2>/dev/null; then
  export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/ros_logs_center_attack_debug}"
  mkdir -p "$ROS_LOG_DIR"
fi

source /opt/ros/humble/setup.bash
source "$SENTRY_ROOT/rm_vision_ws/install/setup.bash"
source "$SENTRY_ROOT/rm_decision_ws/install/setup.bash"

echo ">>> 清理旧的 debug 残留进程..."
pkill -f "$SENTRY_ROOT/scripts/watch_center_attack_state.py" 2>/dev/null || true
pkill -f "$SENTRY_ROOT/scripts/bt_comm_adapter.py" 2>/dev/null || true
pkill -f "$SENTRY_ROOT/rm_decision_ws/install/rm_behavior_tree/lib/rm_behavior_tree/rm_behavior_tree" 2>/dev/null || true
sleep 1

echo ">>> 启动 bt_comm_adapter..."
python3 "$SENTRY_ROOT/scripts/bt_comm_adapter.py" &
sleep 2

echo ">>> 检查 navigate_to_pose action..."
if ros2 action list 2>/dev/null | grep -q "navigate_to_pose"; then
  echo ">>> navigate_to_pose 已在线"
else
  echo ">>> 警告: 当前还没看到 navigate_to_pose"
  echo ">>> 先确认 Gazebo/Nav2 那个终端已经启动完成"
fi

echo ">>> 启动 rm_behavior_tree ($BT_STYLE)..."
ros2 launch rm_behavior_tree rm_behavior_tree.launch.py \
  style:="$BT_STYLE" \
  use_sim_time:="$USE_SIM_TIME" \
  respawn:="$BT_RESPAWN" \
  enable_groot:="$ENABLE_GROOT" \
  groot_port:="$GROOT_PORT" &
sleep 3

if [ "$WATCH_STATE" = "1" ]; then
  echo ">>> 启动中心/回家状态 watcher..."
  python3 "$SENTRY_ROOT/scripts/watch_center_attack_state.py" &
  sleep 1
fi

cat <<'EOF'

>>> Debug session 已启动，可以直接手动发话题调分支：

source /opt/ros/humble/setup.bash
source ~/sentry_planner/rm_decision_ws/install/setup.bash

# 比赛中 + 正常血量：去中心
ros2 topic pub -r 1 /game_status rm_decision_interfaces/msg/GameStatus \
  "{game_progress: 4, stage_remain_time: 220}"
ros2 topic pub -r 10 /robot_status rm_decision_interfaces/msg/RobotStatus \
  "{robot_id: 7, current_hp: 600, shooter_heat: 0, team_color: false, is_attacked: false}"

# 低血：回家
ros2 topic pub -r 10 /robot_status rm_decision_interfaces/msg/RobotStatus \
  "{robot_id: 7, current_hp: 200, shooter_heat: 0, team_color: false, is_attacked: false}"

# 未开赛：回家待机
ros2 topic pub -r 1 /game_status rm_decision_interfaces/msg/GameStatus \
  "{game_progress: 0, stage_remain_time: 220}"

按 Ctrl+C 结束这个 debug session。
EOF

wait
