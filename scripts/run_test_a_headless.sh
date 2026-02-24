#!/bin/bash
# 方案 A：无雷达测试 - 无界面版（无 RViz，适用于 headless/CI）
# 会开 3 个后台进程：假传感器、Nav2、决策、game_status
# 若 ~/.ros 无写权限，自动使用 /tmp 作为日志目录

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SENTRY_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
# 固定使用 RMUL 场地地图（高校联盟赛），不读取 MAP_YAML 环境变量
MAP_YAML="$SENTRY_ROOT/rm_navigation_ws/src/rm_nav_bringup/map/RMUL.yaml"
NAV_PARAMS="${NAV_PARAMS:-/home/nyu/nav_ws/my_nav2_params.yaml}"
ROS_LOG_DIR="${ROS_LOG_DIR:-}"

trap 'echo ">>> 停止所有..."; kill $(jobs -p) 2>/dev/null; exit' SIGINT

echo "=========================================="
echo "  方案 A：无雷达测试 (headless)"
echo "  地图: $MAP_YAML"
echo "=========================================="

# 若 ~/.ros/log 无写权限，使用 /tmp
if [ ! -w "$HOME/.ros/log" ] 2>/dev/null; then
    ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/ros_log_$$}"
    mkdir -p "$ROS_LOG_DIR"
    export ROS_LOG_DIR
    export ROS_HOME="$ROS_LOG_DIR"
    echo "⚠️  ~/.ros/log 无写权限，使用 $ROS_LOG_DIR"
    echo "   建议执行: sudo chown -R \$(whoami):\$(whoami) ~/.ros"
fi

# 检查地图是否存在
if [ ! -f "$MAP_YAML" ]; then
    echo "❌ 地图不存在: $MAP_YAML"
    exit 1
fi

echo ">>> [1/4] 启动假传感器..."
(cd /tmp && unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH && \
 source /opt/ros/humble/setup.bash && \
 python3 "$SCRIPT_DIR/fake_sensors_for_test.py") &
sleep 2

echo ">>> [2/4] 启动 Nav2..."
(cd /tmp && unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH && \
 source /opt/ros/humble/setup.bash && \
 source ~/nav_ws/install/setup.bash && \
 ros2 launch nav2_bringup bringup_launch.py \
   use_sim_time:=False map:="$MAP_YAML" params_file:="$NAV_PARAMS") &
sleep 8

echo ">>> [3/4] 启动决策行为树..."
(cd /tmp && unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH && \
 source /opt/ros/humble/setup.bash && \
 source "$SENTRY_ROOT/rm_vision_ws/install/setup.bash" 2>/dev/null; \
 source "$SENTRY_ROOT/rm_decision_ws/install/setup.bash" && \
 ros2 launch rm_behavior_tree rm_behavior_tree.launch.py \
   style:=retreat_attack_left use_sim_time:=False) &
sleep 3

echo ">>> [4/4] 发布 game_status..."
(cd /tmp && unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH && \
 source /opt/ros/humble/setup.bash && \
 source "$SENTRY_ROOT/rm_decision_ws/install/setup.bash" 2>/dev/null; \
 ros2 topic pub -r 1 /game_status rm_decision_interfaces/msg/GameStatus \
  '{game_progress: 4, stage_remain_time: 180}') &
sleep 1

echo ""
echo "✅ 全部启动完成 (无 RViz)"
echo "   可用 ros2 topic list / ros2 node list 检查"
echo "   按 Ctrl+C 停止所有节点"
echo ""

wait
