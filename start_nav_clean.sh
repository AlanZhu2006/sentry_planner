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
NAV2_PARAMS_FILE="${NAV2_PARAMS_FILE:-$HOME/nav_ws/my_nav2_params.yaml}"
NAV2_EFFECTIVE_PARAMS_FILE="${NAV2_EFFECTIVE_PARAMS_FILE:-/tmp/start_nav_clean_nav2_params.yaml}"
NAV_BASE_FRAME="${NAV_BASE_FRAME:-base_footprint}"
NAV_ODOM_TOPIC="${NAV_ODOM_TOPIC:-/chassis_odom}"
LIDAR_BODY_FRAME="${LIDAR_BODY_FRAME:-body}"
LIDAR_SCAN_FRAME="${LIDAR_SCAN_FRAME:-base_link}"
FASTLIO_POINTCLOUD_TOPIC="${FASTLIO_POINTCLOUD_TOPIC:-/cloud_registered_body}"
LIDAR_SCAN_PARENT_FRAME="${LIDAR_SCAN_PARENT_FRAME:-body}"
LIDAR_SCAN_X="${LIDAR_SCAN_X:-0.0}"
LIDAR_SCAN_Y="${LIDAR_SCAN_Y:-0.0}"
LIDAR_SCAN_Z="${LIDAR_SCAN_Z:-0.0}"
LIDAR_SCAN_YAW="${LIDAR_SCAN_YAW:-0.0}"
LIDAR_SCAN_PITCH="${LIDAR_SCAN_PITCH:--0.873}"
LIDAR_SCAN_ROLL="${LIDAR_SCAN_ROLL:-0.0}"
PUBLISH_NAV2_INITIAL_POSE="${PUBLISH_NAV2_INITIAL_POSE:-0}"
NAV2_INITIAL_POSE_X="${NAV2_INITIAL_POSE_X:-0.8}"
NAV2_INITIAL_POSE_Y="${NAV2_INITIAL_POSE_Y:-7.8}"
NAV2_INITIAL_POSE_YAW="${NAV2_INITIAL_POSE_YAW:-0.0}"
WAIT_MANUAL_INITIAL_POSE="${WAIT_MANUAL_INITIAL_POSE:-1}"
WAIT_MANUAL_INITIAL_POSE_TIMEOUT="${WAIT_MANUAL_INITIAL_POSE_TIMEOUT:-600}"
START_SERIAL_SENDER="${START_SERIAL_SENDER:-1}"
START_ROBOT_CONTROL_KEEPALIVE="${START_ROBOT_CONTROL_KEEPALIVE:-1}"
RADAR_PTY="${RADAR_PTY:-}"
SERIAL_SENDER_PORT="${SERIAL_SENDER_PORT:-$RADAR_PTY}"
SERIAL_SENDER_TOPIC="${SERIAL_SENDER_TOPIC:-/cmd_vel_chassis}"
FAKE_VEL_NAV_BASE_FRAME="${FAKE_VEL_NAV_BASE_FRAME:-$NAV_BASE_FRAME}"
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

start_scan_bridge() {
    if ! wait_for_tf "$NAV_BASE_FRAME" "$LIDAR_SCAN_PARENT_FRAME" 10; then
        echo "Error: $NAV_BASE_FRAME -> $LIDAR_SCAN_PARENT_FRAME TF did not appear."
        echo "The lidar/gimbal compensation frame is missing, so scan alignment would be wrong."
        exit 1
    fi
    echo ">>> Starting pointcloud_to_laserscan..."
    ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node --ros-args \
        -p target_frame:="$LIDAR_SCAN_FRAME" \
        -p transform_tolerance:=0.05 \
        -p min_height:=-0.4 \
        -p max_height:=1.0 \
        -p angle_min:=-3.1415 \
        -p angle_max:=3.1415 \
        -p range_min:=0.1 \
        -p range_max:=20.0 \
        -p use_inf:=true \
        -p qos_overrides./cloud_in.reliability:=best_effort \
        -r cloud_in:="$FASTLIO_POINTCLOUD_TOPIC" \
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

generate_nav2_params() {
    local src_file="$1"
    local dst_file="$2"
    local base_frame="$3"
    local odom_topic="$4"
    python3 - "$src_file" "$dst_file" "$base_frame" "$odom_topic" <<'PYPARAMS'
import re
import sys
from pathlib import Path

src = Path(sys.argv[1])
dst = Path(sys.argv[2])
base_frame = sys.argv[3]
odom_topic = sys.argv[4]

text = src.read_text()
text = re.sub(r'(^\s*base_frame_id:\s*)"[^"]+"(\s*$)', rf'\1"{base_frame}"\2', text, flags=re.MULTILINE)
text = re.sub(r'(^\s*base_frame_id:\s*)\S+(\s*$)', rf'\1{base_frame}\2', text, flags=re.MULTILINE)
text = re.sub(r'(^\s*robot_base_frame:\s*)"[^"]+"(\s*$)', rf'\1"{base_frame}"\2', text, flags=re.MULTILINE)
text = re.sub(r'(^\s*robot_base_frame:\s*)\S+(\s*$)', rf'\1{base_frame}\2', text, flags=re.MULTILINE)
text = re.sub(r'(^\s*odom_topic:\s*)"[^"]+"(\s*$)', rf'\1"{odom_topic}"\2', text, flags=re.MULTILINE)
text = re.sub(r'(^\s*odom_topic:\s*)\S+(\s*$)', rf'\1{odom_topic}\2', text, flags=re.MULTILINE)
controller_marker = "controller_server:\n  ros__parameters:\n"
if controller_marker in text:
    controller_start = text.index(controller_marker)
    next_section = re.search(r'(?m)^[A-Za-z0-9_]+:\n', text[controller_start + len(controller_marker):])
    controller_end = controller_start + len(controller_marker) + (next_section.start() if next_section else len(text))
    controller_block = text[controller_start:controller_end]
    if "odom_topic:" not in controller_block:
        text = text.replace(
            controller_marker,
            controller_marker + f"    odom_topic: {odom_topic}\n",
            1,
        )
dst.write_text(text)
print(dst)
PYPARAMS
}

source /opt/ros/humble/setup.bash
source "$NAV_WS_ROOT/install/setup.bash"
source "$RM_VISION_WS_ROOT/install/setup.bash"

if [ -z "$RVIZ_CONFIG" ]; then
    RVIZ_CONFIG="$(ros2 pkg prefix nav2_bringup 2>/dev/null || true)/share/nav2_bringup/rviz/nav2_default_view.rviz"
fi

if [ ! -w "$HOME/.ros/log" ] 2>/dev/null; then
    export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/ros_logs_start_nav_clean}"
    mkdir -p "$ROS_LOG_DIR"
fi

echo ">>> [0/9] Resetting ROS 2 graph..."
if [ "$RESET_FASTRTPS_SHM" = "1" ]; then
    echo "    RESET_FASTRTPS_SHM=1, removing /dev/shm/fastrtps_* ..."
    rm -f /dev/shm/fastrtps_* 2>/dev/null || true
fi
ros2 daemon stop || true
ros2 daemon start

echo ">>> [1/9] Stopping conflicting processes..."
pkill -9 -f fast_lio_mapping 2>/dev/null || true
pkill -9 -f "ros2 launch fast_lio mapping.launch.py" 2>/dev/null || true
pkill -9 -f livox_ros_driver2 2>/dev/null || true
pkill -9 -f "ros2 launch livox_ros_driver2 msg_MID360_launch.py" 2>/dev/null || true
pkill -9 -f pointcloud_to_laserscan 2>/dev/null || true
pkill -9 -f static_transform_publisher 2>/dev/null || true
pkill -9 -f rviz2 2>/dev/null || true
pkill -9 -f nav2 2>/dev/null || true
pkill -9 -f fake_vel_transform 2>/dev/null || true
pkill -9 -f icp_registration 2>/dev/null || true
pkill -9 -f odom_to_tf.py 2>/dev/null || true
pkill -9 -f rm_behavior_tree 2>/dev/null || true
pkill -9 -f "$SENTRY_ROOT/scripts/bt_comm_adapter.py" 2>/dev/null || true
pkill -9 -f "ros2 topic pub -r 10 /robot_control rm_decision_interfaces/msg/RobotControl" 2>/dev/null || true
if [ "$START_SERIAL_SENDER" = "1" ] && [ -n "$SERIAL_SENDER_PORT" ]; then
    pkill -9 -f "/home/nyu/Codespace/nyush-rm-vision/serial_sender.py --port $SERIAL_SENDER_PORT" 2>/dev/null || true
fi
sleep 2

echo ">>> [2/9] Configuration"
echo "   MAP_FILE=$MAP_FILE"
echo "   NAV2_PARAMS_FILE=$NAV2_PARAMS_FILE"
echo "   NAV2_EFFECTIVE_PARAMS_FILE=$NAV2_EFFECTIVE_PARAMS_FILE"
echo "   NAV_BASE_FRAME=$NAV_BASE_FRAME"
echo "   NAV_ODOM_TOPIC=$NAV_ODOM_TOPIC"
echo "   FASTLIO_POINTCLOUD_TOPIC=$FASTLIO_POINTCLOUD_TOPIC"
echo "   LIDAR_BODY_FRAME=$LIDAR_BODY_FRAME"
echo "   LIDAR_SCAN_PARENT_FRAME=$LIDAR_SCAN_PARENT_FRAME"
echo "   LIDAR_SCAN_FRAME=$LIDAR_SCAN_FRAME"
echo "   LIDAR_SCAN_XYZ=($LIDAR_SCAN_X, $LIDAR_SCAN_Y, $LIDAR_SCAN_Z)"
echo "   LIDAR_SCAN_YPR=($LIDAR_SCAN_YAW, $LIDAR_SCAN_PITCH, $LIDAR_SCAN_ROLL)"
echo "   START_SERIAL_SENDER=$START_SERIAL_SENDER"
echo "   START_ROBOT_CONTROL_KEEPALIVE=$START_ROBOT_CONTROL_KEEPALIVE"
echo "   FAKE_VEL_NAV_BASE_FRAME=$FAKE_VEL_NAV_BASE_FRAME"
echo "   FAKE_VEL_USE_PATH_HEADING_COMPENSATION=$FAKE_VEL_USE_PATH_HEADING_COMPENSATION"
echo "   FAKE_VEL_SWAP_NAV_XY=$FAKE_VEL_SWAP_NAV_XY"
echo "   FAKE_VEL_CHASSIS_X_SIGN=$FAKE_VEL_CHASSIS_X_SIGN"
echo "   FAKE_VEL_CHASSIS_Y_SIGN=$FAKE_VEL_CHASSIS_Y_SIGN"

echo ">>> [3/9] Starting Livox driver..."
ros2 launch livox_ros_driver2 msg_MID360_launch.py &
sleep 5

echo ">>> [4/9] Starting FAST-LIO and scan bridge..."
ros2 run tf2_ros static_transform_publisher \
    --x 0 --y 0 --z 0 \
    --yaw 0 --pitch 0 --roll 0 \
    --frame-id odom --child-frame-id camera_init &
ros2 run tf2_ros static_transform_publisher \
    --x "$LIDAR_SCAN_X" --y "$LIDAR_SCAN_Y" --z "$LIDAR_SCAN_Z" \
    --yaw "$LIDAR_SCAN_YAW" --pitch "$LIDAR_SCAN_PITCH" --roll "$LIDAR_SCAN_ROLL" \
    --frame-id "$LIDAR_SCAN_PARENT_FRAME" --child-frame-id "$LIDAR_SCAN_FRAME" &
export LD_PRELOAD=/lib/x86_64-linux-gnu/libusb-1.0.so.0
ros2 launch fast_lio mapping.launch.py config_file:=mid360.yaml &
sleep 5

echo ">>> [4.5/9] Waiting for chassis odom..."
if ! timeout 20 ros2 topic echo "$NAV_ODOM_TOPIC" --once >/dev/null 2>&1; then
    echo "Error: $NAV_ODOM_TOPIC did not appear."
    echo "Start sentry_bridge first and make sure it can publish ROS odom."
    exit 1
fi

echo ">>> [4.8/9] Starting odom_to_tf + fake_vel_transform..."
python3 "$SENTRY_ROOT/rm_navigation_ws/src/rm_nav_bringup/scripts/odom_to_tf.py" --ros-args \
    -p odom_topic:="$NAV_ODOM_TOPIC" &
ros2 run fake_vel_transform fake_vel_transform_node --ros-args \
    -p use_sim_time:="$USE_SIM_TIME" \
    -p nav_base_frame:="$FAKE_VEL_NAV_BASE_FRAME" \
    -p use_path_heading_compensation:="$FAKE_VEL_USE_PATH_HEADING_COMPENSATION" \
    -p use_nav_wz:=false \
    -p swap_nav_xy:="$FAKE_VEL_SWAP_NAV_XY" \
    -p chassis_x_sign:="$FAKE_VEL_CHASSIS_X_SIGN" \
    -p chassis_y_sign:="$FAKE_VEL_CHASSIS_Y_SIGN" &
sleep 2

echo ">>> [4.9/9] Waiting for odom -> $NAV_BASE_FRAME TF..."
if ! wait_for_tf odom "$NAV_BASE_FRAME" 20; then
    echo "Error: odom -> $NAV_BASE_FRAME TF did not appear."
    exit 1
fi

echo ">>> [5/9] Preparing Nav2 params..."
generate_nav2_params "$NAV2_PARAMS_FILE" "$NAV2_EFFECTIVE_PARAMS_FILE" "$NAV_BASE_FRAME" "$NAV_ODOM_TOPIC" >/dev/null
ros2 launch nav2_bringup localization_launch.py use_sim_time:="$USE_SIM_TIME" map:="$MAP_FILE" params_file:="$NAV2_EFFECTIVE_PARAMS_FILE" &
sleep 6

if [ "$PUBLISH_NAV2_INITIAL_POSE" = "1" ]; then
    echo ">>> [5.5/9] Publishing initial pose..."
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
    echo ">>> [5.6/9] Waiting for manual /initialpose..."
    echo "    Use RViz '2D Pose Estimate', then startup will continue."
    if ! wait_for_initial_pose "$WAIT_MANUAL_INITIAL_POSE_TIMEOUT"; then
        echo "Error: Timed out waiting for manual /initialpose."
        exit 1
    fi
fi

echo ">>> [5.7/9] Starting scan bridge after initial pose..."
start_scan_bridge

echo ">>> [5.8/9] Waiting for map -> $NAV_BASE_FRAME TF..."
if ! wait_for_tf map "$NAV_BASE_FRAME" 60; then
    echo "Error: map -> $NAV_BASE_FRAME TF did not appear after initial pose."
    exit 1
fi

echo ">>> [6/9] Starting Nav2 navigation nodes..."
ros2 launch nav2_bringup navigation_launch.py use_sim_time:="$USE_SIM_TIME" params_file:="$NAV2_EFFECTIVE_PARAMS_FILE" &
sleep 6

if [ "$START_SERIAL_SENDER" = "1" ]; then
    if [ -z "$SERIAL_SENDER_PORT" ]; then
        echo "Error: START_SERIAL_SENDER=1 requires RADAR_PTY or SERIAL_SENDER_PORT."
        exit 1
    fi
    echo ">>> [7/9] Starting serial_sender..."
    python3 /home/nyu/Codespace/nyush-rm-vision/serial_sender.py \
        --port "$SERIAL_SENDER_PORT" \
        --ros2 \
        --topic "$SERIAL_SENDER_TOPIC" \
        --robot-control-topic /robot_control &
fi

if [ "$START_ROBOT_CONTROL_KEEPALIVE" = "1" ]; then
    echo ">>> [7.5/9] Starting RobotControl keepalive..."
    ros2 topic pub -r 10 /robot_control rm_decision_interfaces/msg/RobotControl \
        "{stop_gimbal_scan: true, chassis_spin_vel: 0.0, scan_enabled: false, allow_vision_control: false, search_when_target_lost: false, scan_yaw_rate_deg_s: 0.0, search_pitch_deg: 0.0}" &
fi

echo ">>> [8/9] Nav stack ready."
echo "    Use RViz 'Nav2 Goal' to send goals manually."
echo "    Useful checks:"
echo "      ros2 topic echo $NAV_ODOM_TOPIC --once"
echo "      ros2 run tf2_ros tf2_echo $NAV_BASE_FRAME $LIDAR_SCAN_FRAME"
echo "      ros2 topic echo /scan --once"
echo "      ros2 topic echo /cmd_vel --once"
echo "      ros2 topic echo /cmd_vel_chassis --once"

if [ "$ENABLE_RVIZ" = "1" ]; then
    launch_rviz_background
fi

wait
