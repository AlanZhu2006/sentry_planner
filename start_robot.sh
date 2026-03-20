#!/usr/bin/env bash

set -eo pipefail

# --- 脚本功能：一键启动 激光雷达 + FAST-LIO + Nav2 + 当前主线行为树 ---

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SENTRY_ROOT="$SCRIPT_DIR"
NAV_WS_ROOT="${NAV_WS_ROOT:-$HOME/nav_ws}"
RM_VISION_WS_ROOT="${RM_VISION_WS_ROOT:-$SENTRY_ROOT/rm_vision_ws}"
RM_DECISION_WS_ROOT="${RM_DECISION_WS_ROOT:-$SENTRY_ROOT/rm_decision_ws}"
BT_STYLE="${BT_STYLE:-center_attack_simple}"
USE_SIM_TIME="${USE_SIM_TIME:-False}"
ENABLE_RVIZ="${ENABLE_RVIZ:-0}"
RVIZ_CONFIG="${RVIZ_CONFIG:-}"
START_SERIAL_SENDER="${START_SERIAL_SENDER:-0}"
RADAR_PTY="${RADAR_PTY:-}"
SERIAL_SENDER_TOPIC="${SERIAL_SENDER_TOPIC:-/cmd_vel_chassis_bt}"
SERIAL_SENDER_PORT="${SERIAL_SENDER_PORT:-$RADAR_PTY}"
MAP_FILE="${MAP_FILE:-$HOME/sentry_planner/rm_navigation_ws/src/rm_nav_bringup/map/RMUL2026.yaml}"
NAV2_PARAMS_FILE="${NAV2_PARAMS_FILE:-/home/nyu/nav_ws/my_nav2_params.yaml}"

cleanup() {
    echo "正在关闭所有节点..."
    kill $(jobs -p) 2>/dev/null || true
}
trap cleanup EXIT INT TERM

source /opt/ros/humble/setup.bash
source "$NAV_WS_ROOT/install/setup.bash"
source "$RM_VISION_WS_ROOT/install/setup.bash"
source "$RM_DECISION_WS_ROOT/install/setup.bash"

if [ -z "$RVIZ_CONFIG" ]; then
    RVIZ_CONFIG="$(ros2 pkg prefix nav2_bringup 2>/dev/null || true)/share/nav2_bringup/rviz/nav2_default_view.rviz"
fi

if [ ! -w "$HOME/.ros/log" ] 2>/dev/null; then
    export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/ros_logs_start_robot}"
    mkdir -p "$ROS_LOG_DIR"
fi

if ! command -v python3 >/dev/null 2>&1; then
    echo "❌ 错误：未找到 python3"
    exit 1
fi

echo ">>> [0/11] 清理环境..."
rm -f /dev/shm/fastrtps_*
ros2 daemon stop || true
ros2 daemon start

echo "   正在停止旧的 ROS2 / 导航 / 行为树残留进程..."
pkill -9 -f fast_lio_mapping 2>/dev/null || true
pkill -9 -f livox_ros_driver2 2>/dev/null || true
pkill -9 -f pointcloud_to_laserscan 2>/dev/null || true
pkill -9 -f nav2 2>/dev/null || true
pkill -9 -f rm_behavior_tree 2>/dev/null || true
pkill -9 -f "$SENTRY_ROOT/scripts/bt_comm_adapter.py" 2>/dev/null || true
if [ "$START_SERIAL_SENDER" = "1" ] && [ -n "$SERIAL_SENDER_PORT" ]; then
    pkill -9 -f "/home/nyu/Codespace/nyush-rm-vision/serial_sender.py --port $SERIAL_SENDER_PORT" 2>/dev/null || true
fi
sleep 2

echo ">>> [1/11] 初始化环境..."
echo "   BT_STYLE=$BT_STYLE"
echo "   USE_SIM_TIME=$USE_SIM_TIME"
echo "   ENABLE_RVIZ=$ENABLE_RVIZ"
if [ "$START_SERIAL_SENDER" = "1" ]; then
    echo "   START_SERIAL_SENDER=1 ($SERIAL_SENDER_PORT <- $SERIAL_SENDER_TOPIC)"
fi

echo ">>> [2/11] 设置串口权限 (如果卡在这里，请输入密码)..."
if [ -e /dev/ttyACM0 ]; then
    sudo chmod 777 /dev/ttyACM0
else
    echo "警告: 未检测到 /dev/ttyACM0，跳过权限设置"
fi

echo ">>> [3/11] 启动 MI360 激光雷达驱动..."
ros2 launch livox_ros_driver2 msg_MID360_launch.py &
DRIVER_PID=$!
echo ">>> 等待雷达驱动启动 (5秒)..."
sleep 5

if timeout 2 ros2 topic echo /livox/lidar --once > /dev/null 2>&1; then
    echo "✅ 雷达驱动已启动并发送数据"
else
    echo "❌ 警告：未检测到雷达数据！请检查网线连接或防火墙。"
fi

echo ">>> [4/11] 发布静态 TF 变换..."
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom camera_init &
ros2 run tf2_ros static_transform_publisher 0 0 0 0 -0.873 0 body base_link &
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link base_footprint &

echo ">>> [5/11] 启动 FAST-LIO 里程计..."
export LD_PRELOAD=/lib/x86_64-linux-gnu/libusb-1.0.so.0
ros2 launch fast_lio mapping.launch.py config_file:=mid360.yaml &
sleep 5

echo ">>> [6/11] 启动 Pointcloud 转 LaserScan..."
ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node --ros-args -p target_frame:=base_link -p transform_tolerance:=0.01 -p min_height:=-0.4 -p max_height:=1.0 -p angle_min:=-3.1415 -p angle_max:=3.1415 -p range_min:=0.1 -p range_max:=20.0 -p use_inf:=true -p qos_overrides./cloud_in.reliability:=best_effort -r cloud_in:=/cloud_registered -r scan:=/scan &

echo ">>> [7/11] 启动 Nav2 导航..."
ros2 launch nav2_bringup bringup_launch.py     use_sim_time:="$USE_SIM_TIME"     map:="$MAP_FILE"     params_file:="$NAV2_PARAMS_FILE" &

echo ">>> [8/11] 等待 Nav2 启动 (8秒)..."
sleep 8

echo ">>> [9/11] 启动行为树通讯适配层..."
python3 "$SENTRY_ROOT/scripts/bt_comm_adapter.py" &
sleep 2

echo ">>> [10/11] 启动决策行为树 ($BT_STYLE)..."
ros2 launch rm_behavior_tree rm_behavior_tree.launch.py     style:="$BT_STYLE"     use_sim_time:="$USE_SIM_TIME" &
sleep 2

if [ "$START_SERIAL_SENDER" = "1" ]; then
    if [ -z "$SERIAL_SENDER_PORT" ]; then
        echo "❌ 错误：START_SERIAL_SENDER=1 但没有提供 RADAR_PTY / SERIAL_SENDER_PORT"
        exit 1
    fi
    echo ">>> [11/11] 启动 serial_sender ($SERIAL_SENDER_PORT <- $SERIAL_SENDER_TOPIC)..."
    python3 /home/nyu/Codespace/nyush-rm-vision/serial_sender.py         --port "$SERIAL_SENDER_PORT"         --ros2         --topic "$SERIAL_SENDER_TOPIC" &
else
    echo ">>> [11/11] 启动完成！"
fi

echo "-----------------------------------------------------"
echo ">>> 当前主线节点已启动（Nav2 + bt_comm_adapter + rm_behavior_tree）"
echo ">>> 默认行为树：$BT_STYLE"
echo ">>> bridge 仍需单独在 nyush-rm-control 终端启动"
echo ">>> 视觉程序仍需单独连 Vision PTY 启动"
echo ">>> 如需自动写 Radar PTY，可这样运行："
echo "    START_SERIAL_SENDER=1 RADAR_PTY=/tmp/nyush-rm-sentry-radar ./start_robot.sh"
echo ""
echo ">>> 常用实机配套终端："
echo "    1. nyush-rm-control: just sentry-bridge --port /dev/ttyACM0"
echo "    2. nyush-rm-vision : just test detect --web --send"
echo "    3. sentry_planner : START_SERIAL_SENDER=1 RADAR_PTY=/tmp/nyush-rm-sentry-radar ./start_robot.sh"
echo ""
echo ">>> 关键话题检查："
echo "    ros2 topic echo /robot_control --once"
echo "    ros2 topic echo /cmd_vel_chassis_bt --once"
echo "-----------------------------------------------------"

if [ "$ENABLE_RVIZ" = "1" ]; then
    if [ -n "$RVIZ_CONFIG" ] && [ -f "$RVIZ_CONFIG" ]; then
        ros2 run rviz2 rviz2 -d "$RVIZ_CONFIG"
    else
        ros2 run rviz2 rviz2
    fi
else
    wait
fi
