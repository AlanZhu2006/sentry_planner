#!/bin/bash
# 方案 A：无雷达测试 - 一键启动脚本
# 会开 6 个后台进程：假传感器、Nav2、RViz、通讯适配层、决策、game_status
# 按 Ctrl+C 可停止所有

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SENTRY_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
# 固定使用 RMUL 场地地图（高校联盟赛），不读取 MAP_YAML 环境变量
MAP_YAML="$SENTRY_ROOT/rm_navigation_ws/src/rm_nav_bringup/map/RMUL.yaml"
NAV_PARAMS="${NAV_PARAMS:-/home/nyu/nav_ws/my_nav2_params.yaml}"
BT_STYLE="${BT_STYLE:-center_attack_simple}"

trap 'echo ">>> 停止所有..."; kill $(jobs -p) 2>/dev/null; exit' SIGINT

echo "=========================================="
echo "  方案 A：无雷达测试"
echo "  地图: $MAP_YAML"
echo "  行为树: $BT_STYLE"
echo "=========================================="

# 检查 .ros 权限
if [ ! -w "$HOME/.ros/log" ] 2>/dev/null; then
    echo "⚠️  ~/.ros/log 无写权限，请先执行："
    echo "   sudo chown -R \$(whoami):\$(whoami) ~/.ros"
    exit 1
fi

# 检查地图是否存在
if [ ! -f "$MAP_YAML" ]; then
    echo "❌ 地图不存在: $MAP_YAML"
    echo "   请修改 MAP_YAML 或创建地图"
    exit 1
fi

echo ">>> [1/4] 启动假传感器..."
(cd /tmp && unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH && \
 source /opt/ros/humble/setup.bash && \
 python3 "$SCRIPT_DIR/fake_sensors_for_test.py") &
sleep 2

echo ">>> [2/5] 启动 Nav2..."
(cd /tmp && unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH && \
 source /opt/ros/humble/setup.bash && \
 source ~/nav_ws/install/setup.bash && \
 ros2 launch nav2_bringup bringup_launch.py \
   use_sim_time:=False map:="$MAP_YAML" params_file:="$NAV_PARAMS") &
sleep 8

echo ">>> [3/5] 启动 RViz (bringup_launch 默认不包含 RViz)..."
(cd /tmp && unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH && \
 source /opt/ros/humble/setup.bash && \
 source ~/nav_ws/install/setup.bash && \
 ros2 run rviz2 rviz2 -d $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/rviz/nav2_default_view.rviz) &
sleep 2

echo ">>> [4/6] 启动行为树通讯适配层..."
(cd /tmp && unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH && \
 source /opt/ros/humble/setup.bash && \
 source "$SENTRY_ROOT/rm_vision_ws/install/setup.bash" && \
 source "$SENTRY_ROOT/rm_decision_ws/install/setup.bash" && \
 python3 "$SENTRY_ROOT/scripts/bt_comm_adapter.py") &
sleep 2

echo ">>> [5/6] 启动决策行为树..."
(cd /tmp && unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH && \
 source /opt/ros/humble/setup.bash && \
 source "$SENTRY_ROOT/rm_vision_ws/install/setup.bash" && \
 source "$SENTRY_ROOT/rm_decision_ws/install/setup.bash" && \
 ros2 launch rm_behavior_tree rm_behavior_tree.launch.py \
   style:="$BT_STYLE" use_sim_time:=False) &
sleep 3

echo ">>> [6/6] 发布 game_status..."
(cd /tmp && unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH && \
 source /opt/ros/humble/setup.bash && \
 source "$SENTRY_ROOT/rm_decision_ws/install/setup.bash" 2>/dev/null || true && \
 ros2 topic pub -r 1 /game_status rm_decision_interfaces/msg/GameStatus \
  '{game_progress: 4, stage_remain_time: 180}') &
sleep 1

echo ""
echo "✅ 全部启动完成！"
echo "   - RViz 应已打开，在 map 上点击 'Nav2 Goal' 发送目标"
echo "   - 决策树会收到 game_progress=4，可执行导航逻辑"
echo "   - 通讯适配层已启动，默认只用 vx/vy，忽略 Nav2 自带 wz"
echo "   - 按 Ctrl+C 停止所有节点"
echo ""

wait
