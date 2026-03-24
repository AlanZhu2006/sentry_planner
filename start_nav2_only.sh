#!/usr/bin/env bash

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SENTRY_ROOT="$SCRIPT_DIR"
NAV_WS_ROOT="${NAV_WS_ROOT:-$HOME/nav_ws}"
RM_VISION_WS_ROOT="${RM_VISION_WS_ROOT:-$SENTRY_ROOT/rm_vision_ws}"
USE_SIM_TIME="${USE_SIM_TIME:-False}"
ENABLE_RVIZ="${ENABLE_RVIZ:-1}"
RVIZ_CONFIG="${RVIZ_CONFIG:-}"
RESET_FASTRTPS_SHM="${RESET_FASTRTPS_SHM:-0}"
MAP_FILE="${MAP_FILE:-$HOME/sentry_planner/rm_navigation_ws/src/rm_nav_bringup/map/RMUL2026.yaml}"
NAV2_PARAMS_FILE="${NAV2_PARAMS_FILE:-/home/nyu/nav_ws/my_nav2_params.yaml}"
PUBLISH_NAV2_INITIAL_POSE="${PUBLISH_NAV2_INITIAL_POSE:-0}"
NAV2_INITIAL_POSE_X="${NAV2_INITIAL_POSE_X:-0.8}"
NAV2_INITIAL_POSE_Y="${NAV2_INITIAL_POSE_Y:-7.8}"
NAV2_INITIAL_POSE_YAW="${NAV2_INITIAL_POSE_YAW:-0.0}"
WAIT_MANUAL_INITIAL_POSE="${WAIT_MANUAL_INITIAL_POSE:-1}"
WAIT_MANUAL_INITIAL_POSE_TIMEOUT="${WAIT_MANUAL_INITIAL_POSE_TIMEOUT:-600}"
START_SERIAL_SENDER="${START_SERIAL_SENDER:-1}"
RADAR_PTY="${RADAR_PTY:-}"
SERIAL_SENDER_PORT="${SERIAL_SENDER_PORT:-$RADAR_PTY}"
SERIAL_SENDER_TOPIC="${SERIAL_SENDER_TOPIC:-/cmd_vel_chassis}"
START_ROBOT_CONTROL_KEEPALIVE="${START_ROBOT_CONTROL_KEEPALIVE:-1}"
FAKE_VEL_NAV_BASE_FRAME="${FAKE_VEL_NAV_BASE_FRAME:-base_link}"
FAKE_VEL_USE_PATH_HEADING_COMPENSATION="${FAKE_VEL_USE_PATH_HEADING_COMPENSATION:-false}"
LEGACY_FAKE_VEL_CHASSIS_X_FROM_NAV_Y_SIGN="${FAKE_VEL_CHASSIS_X_FROM_NAV_Y_SIGN:-}"
LEGACY_FAKE_VEL_CHASSIS_Y_FROM_NAV_X_SIGN="${FAKE_VEL_CHASSIS_Y_FROM_NAV_X_SIGN:-}"
if [ -z "${FAKE_VEL_SWAP_NAV_XY+x}" ] && { [ -n "$LEGACY_FAKE_VEL_CHASSIS_X_FROM_NAV_Y_SIGN" ] || [ -n "$LEGACY_FAKE_VEL_CHASSIS_Y_FROM_NAV_X_SIGN" ]; }; then
    FAKE_VEL_SWAP_NAV_XY=true
fi
FAKE_VEL_SWAP_NAV_XY="${FAKE_VEL_SWAP_NAV_XY:-false}"
FAKE_VEL_CHASSIS_X_SIGN="${FAKE_VEL_CHASSIS_X_SIGN:-${LEGACY_FAKE_VEL_CHASSIS_X_FROM_NAV_Y_SIGN:-1.0}}"
FAKE_VEL_CHASSIS_Y_SIGN="${FAKE_VEL_CHASSIS_Y_SIGN:-${LEGACY_FAKE_VEL_CHASSIS_Y_FROM_NAV_X_SIGN:-1.0}}"
RVIZ_STARTED=0
LIDAR_SCAN_PARENT_FRAME="${LIDAR_SCAN_PARENT_FRAME:-body}"
LIDAR_SCAN_YAW="${LIDAR_SCAN_YAW:-0.0}"
LIDAR_SCAN_PITCH="${LIDAR_SCAN_PITCH:--0.873}"
LIDAR_SCAN_ROLL="${LIDAR_SCAN_ROLL:-0.0}"

start_scan_bridge() {
    if ! wait_for_tf base_footprint "$LIDAR_SCAN_PARENT_FRAME" 10; then
        echo "Error: base_footprint -> $LIDAR_SCAN_PARENT_FRAME TF did not appear."
        echo "The lidar/gimbal compensation frame is missing, so scan alignment would be wrong."
        exit 1
    fi
    echo ">>> Starting pointcloud_to_laserscan..."
    ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node --ros-args \
        -p target_frame:=base_link \
        -p transform_tolerance:=0.01 \
        -p min_height:=-0.4 \
        -p max_height:=1.0 \
        -p angle_min:=-3.1415 \
        -p angle_max:=3.1415 \
        -p range_min:=0.1 \
        -p range_max:=20.0 \
        -p use_inf:=true \
        -p qos_overrides./cloud_in.reliability:=best_effort \
        -r cloud_in:=/cloud_registered \
        -r scan:=/scan &
    sleep 2
}

launch_rviz_background() {
    if [ "$ENABLE_RVIZ" != "1" ] || [ "$RVIZ_STARTED" = "1" ]; then
        return
    fi
    echo ">>> Launching RViz..."
    if [ -n "$RVIZ_CONFIG" ] && [ -f "$RVIZ_CONFIG" ]; then
        ros2 run rviz2 rviz2 -d "$RVIZ_CONFIG" &
    else
        ros2 run rviz2 rviz2 &
    fi
    RVIZ_STARTED=1
    sleep 2
}

wait_for_tf() {
    local target_frame="$1"
    local source_frame="$2"
    local timeout_secs="$3"
    python3 - "$target_frame" "$source_frame" "$timeout_secs" <<'PYWAITTF'
import sys
import time

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

target_frame = sys.argv[1]
source_frame = sys.argv[2]
timeout_secs = float(sys.argv[3])

rclpy.init()
node = Node("wait_for_tf_helper")
buffer = Buffer()
listener = TransformListener(buffer, node, spin_thread=False)

deadline = time.monotonic() + timeout_secs
success = False
try:
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        try:
            buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time())
            success = True
            break
        except Exception:
            pass
finally:
    node.destroy_node()
    rclpy.shutdown()

sys.exit(0 if success else 1)
PYWAITTF
}

wait_for_initial_pose() {
    local timeout_secs="$1"
    python3 - "$timeout_secs" <<'PYWAITPOSE'
import sys
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node

timeout_secs = float(sys.argv[1])

rclpy.init()
node = Node("wait_for_initial_pose_helper")
received = {"ok": False}

def cb(_msg):
    received["ok"] = True

sub = node.create_subscription(PoseWithCovarianceStamped, "/initialpose", cb, 10)

deadline = time.monotonic() + timeout_secs
try:
    while time.monotonic() < deadline and not received["ok"]:
        rclpy.spin_once(node, timeout_sec=0.1)
finally:
    node.destroy_subscription(sub)
    node.destroy_node()
    rclpy.shutdown()

sys.exit(0 if received["ok"] else 1)
PYWAITPOSE
}

source /opt/ros/humble/setup.bash
source "$NAV_WS_ROOT/install/setup.bash"
source "$RM_VISION_WS_ROOT/install/setup.bash"

if [ -z "$RVIZ_CONFIG" ]; then
    RVIZ_CONFIG="$(ros2 pkg prefix nav2_bringup 2>/dev/null || true)/share/nav2_bringup/rviz/nav2_default_view.rviz"
fi

if [ ! -w "$HOME/.ros/log" ] 2>/dev/null; then
    export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/ros_logs_start_nav2_only}"
    mkdir -p "$ROS_LOG_DIR"
fi

echo ">>> [0/8] Resetting ROS 2 graph..."
if [ "$RESET_FASTRTPS_SHM" = "1" ]; then
    echo "    RESET_FASTRTPS_SHM=1, removing /dev/shm/fastrtps_* ..."
    rm -f /dev/shm/fastrtps_* 2>/dev/null || true
fi
ros2 daemon stop || true
ros2 daemon start

echo ">>> [1/8] Stopping conflicting processes..."
pkill -9 -f fast_lio_mapping 2>/dev/null || true
pkill -9 -f livox_ros_driver2 2>/dev/null || true
pkill -9 -f pointcloud_to_laserscan 2>/dev/null || true
pkill -9 -f rviz2 2>/dev/null || true
# Avoid killing this script itself: broad `pkill -f nav2` matches start_nav2_only.sh.
pkill -9 -f "nav2_bringup" 2>/dev/null || true
pkill -9 -f "bringup_rm_navigation.py" 2>/dev/null || true
pkill -9 -f "planner_server" 2>/dev/null || true
pkill -9 -f "controller_server" 2>/dev/null || true
pkill -9 -f "bt_navigator" 2>/dev/null || true
pkill -9 -f "lifecycle_manager_navigation" 2>/dev/null || true
pkill -9 -f "lifecycle_manager_localization" 2>/dev/null || true
pkill -9 -f fake_vel_transform 2>/dev/null || true
pkill -9 -f rm_behavior_tree 2>/dev/null || true
pkill -9 -f "$SENTRY_ROOT/scripts/bt_comm_adapter.py" 2>/dev/null || true
pkill -9 -f "ros2 topic pub -r 10 /robot_control rm_decision_interfaces/msg/RobotControl" 2>/dev/null || true
if [ "$START_SERIAL_SENDER" = "1" ] && [ -n "$SERIAL_SENDER_PORT" ]; then
    pkill -9 -f "/home/nyu/Codespace/nyush-rm-vision/serial_sender.py --port $SERIAL_SENDER_PORT" 2>/dev/null || true
fi
sleep 2

echo ">>> [2/8] Configuration"
echo "   MAP_FILE=$MAP_FILE"
echo "   NAV2_PARAMS_FILE=$NAV2_PARAMS_FILE"
echo "   ENABLE_RVIZ=$ENABLE_RVIZ"
echo "   WAIT_MANUAL_INITIAL_POSE=$WAIT_MANUAL_INITIAL_POSE"
echo "   START_SERIAL_SENDER=$START_SERIAL_SENDER"
echo "   START_ROBOT_CONTROL_KEEPALIVE=$START_ROBOT_CONTROL_KEEPALIVE"
echo "   FAKE_VEL_NAV_BASE_FRAME=$FAKE_VEL_NAV_BASE_FRAME"
echo "   FAKE_VEL_USE_PATH_HEADING_COMPENSATION=$FAKE_VEL_USE_PATH_HEADING_COMPENSATION"
echo "   FAKE_VEL_SWAP_NAV_XY=$FAKE_VEL_SWAP_NAV_XY"
echo "   FAKE_VEL_CHASSIS_X_SIGN=$FAKE_VEL_CHASSIS_X_SIGN"
echo "   FAKE_VEL_CHASSIS_Y_SIGN=$FAKE_VEL_CHASSIS_Y_SIGN"
if [ "$START_SERIAL_SENDER" = "1" ]; then
    echo "   SERIAL_SENDER_PORT=$SERIAL_SENDER_PORT"
    echo "   SERIAL_SENDER_TOPIC=$SERIAL_SENDER_TOPIC"
fi

echo ">>> [3/8] Starting Livox driver..."
ros2 launch livox_ros_driver2 msg_MID360_launch.py &
sleep 5

echo ">>> [4/8] Starting TF + FAST-LIO + LaserScan bridge..."
ros2 run tf2_ros static_transform_publisher \
    --x 0 --y 0 --z 0 \
    --yaw 0 --pitch 0 --roll 0 \
    --frame-id odom --child-frame-id camera_init &
ros2 run tf2_ros static_transform_publisher \
    --x 0 --y 0 --z 0 \
    --yaw "$LIDAR_SCAN_YAW" --pitch "$LIDAR_SCAN_PITCH" --roll "$LIDAR_SCAN_ROLL" \
    --frame-id "$LIDAR_SCAN_PARENT_FRAME" --child-frame-id base_link &
export LD_PRELOAD=/lib/x86_64-linux-gnu/libusb-1.0.so.0
ros2 launch fast_lio mapping.launch.py config_file:=mid360.yaml &
sleep 5

echo ">>> [5/8] Starting Nav2 + AMCL..."
ros2 launch nav2_bringup bringup_launch.py use_sim_time:="$USE_SIM_TIME" map:="$MAP_FILE" params_file:="$NAV2_PARAMS_FILE" &
sleep 8

if [ "$PUBLISH_NAV2_INITIAL_POSE" = "1" ]; then
    echo ">>> [5.5/8] Publishing initial pose..."
    export NAV2_INITIAL_POSE_X NAV2_INITIAL_POSE_Y NAV2_INITIAL_POSE_YAW
    INITIAL_POSE_MSG="$(python3 - <<'PYPOSE'
import math
import os

x = float(os.environ.get("NAV2_INITIAL_POSE_X", "0.8"))
y = float(os.environ.get("NAV2_INITIAL_POSE_Y", "7.8"))
yaw = float(os.environ.get("NAV2_INITIAL_POSE_YAW", "0.0"))
qz = math.sin(yaw * 0.5)
qw = math.cos(yaw * 0.5)
cov = [0.0] * 36
cov[0] = 0.25
cov[7] = 0.25
cov[35] = 0.06853891945200942
print(
    "{header: {frame_id: 'map'}, pose: {pose: {position: {x: %.4f, y: %.4f, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: %.8f, w: %.8f}}, covariance: [%s]}}"
    % (x, y, qz, qw, ", ".join(str(v) for v in cov))
)
PYPOSE
)"
    ros2 topic pub -1 /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "$INITIAL_POSE_MSG" > /dev/null
    sleep 1
fi

if [ "$WAIT_MANUAL_INITIAL_POSE" = "1" ]; then
    launch_rviz_background
    echo ">>> [5.6/8] Waiting for manual /initialpose..."
    echo "    Use RViz '2D Pose Estimate', then startup will continue."
    if ! wait_for_initial_pose "$WAIT_MANUAL_INITIAL_POSE_TIMEOUT"; then
        echo "Error: Timed out waiting for manual /initialpose."
        exit 1
    fi
fi

echo ">>> [5.7/8] Starting scan bridge after initial pose..."
start_scan_bridge

echo ">>> [5.8/8] Waiting for map -> base_link TF..."
if ! wait_for_tf map base_link 60; then
    echo "Error: map -> base_link TF did not appear after manual initial pose."
    exit 1
fi

echo ">>> [6/8] Starting fake_vel_transform..."
ros2 run fake_vel_transform fake_vel_transform_node --ros-args \
    -p use_sim_time:="$USE_SIM_TIME" \
    -p nav_base_frame:="$FAKE_VEL_NAV_BASE_FRAME" \
    -p use_path_heading_compensation:="$FAKE_VEL_USE_PATH_HEADING_COMPENSATION" \
    -p use_nav_wz:=false \
    -p swap_nav_xy:="$FAKE_VEL_SWAP_NAV_XY" \
    -p chassis_x_sign:="$FAKE_VEL_CHASSIS_X_SIGN" \
    -p chassis_y_sign:="$FAKE_VEL_CHASSIS_Y_SIGN" &
sleep 2

if [ "$START_SERIAL_SENDER" = "1" ]; then
    if [ -z "$SERIAL_SENDER_PORT" ]; then
        echo "Error: START_SERIAL_SENDER=1 requires RADAR_PTY or SERIAL_SENDER_PORT."
        exit 1
    fi
    echo ">>> [7/8] Starting serial_sender..."
    python3 /home/nyu/Codespace/nyush-rm-vision/serial_sender.py \
        --port "$SERIAL_SENDER_PORT" \
        --ros2 \
        --topic "$SERIAL_SENDER_TOPIC" &
fi

if [ "$START_ROBOT_CONTROL_KEEPALIVE" = "1" ]; then
    echo ">>> [7.5/8] Starting RobotControl keepalive..."
    ros2 topic pub -r 10 /robot_control rm_decision_interfaces/msg/RobotControl \
        "{stop_gimbal_scan: true, chassis_spin_vel: 0.0, scan_enabled: false, allow_vision_control: false, search_when_target_lost: false, scan_yaw_rate_deg_s: 0.0, search_pitch_deg: 0.0}" &
fi

echo ">>> [8/8] Nav2-only stack is ready."
echo "    No bt_comm_adapter. No rm_behavior_tree."
echo "    Use RViz 'Nav2 Goal' to send goals manually."
echo "    Useful checks:"
echo "      ros2 topic echo /cmd_vel --once"
echo "      ros2 topic echo /cmd_vel_chassis --once"
if [ "$START_SERIAL_SENDER" = "1" ]; then
    echo "      ros2 topic echo $SERIAL_SENDER_TOPIC --once"
fi

if [ "$ENABLE_RVIZ" = "1" ]; then
    if [ "$RVIZ_STARTED" = "1" ]; then
        wait
    elif [ -n "$RVIZ_CONFIG" ] && [ -f "$RVIZ_CONFIG" ]; then
        ros2 run rviz2 rviz2 -d "$RVIZ_CONFIG"
    else
        ros2 run rviz2 rviz2
    fi
else
    wait
fi
