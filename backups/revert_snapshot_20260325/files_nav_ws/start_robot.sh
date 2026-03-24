#!/bin/bash

# --- 脚本功能：一键启动 激光雷达 + FAST-LIO + Nav2 ---
# 兼容当前 bridge + Radar PTY 通讯链，但不改变原有导航技术栈。

set -eo pipefail

RESET_FASTRTPS_SHM="${RESET_FASTRTPS_SHM:-0}"
ENABLE_RVIZ="${ENABLE_RVIZ:-1}"
MAP_FILE="${MAP_FILE:-/home/nyu/Desktop/map/11_map.yaml}"
NAV2_PARAMS_FILE="${NAV2_PARAMS_FILE:-/home/nyu/nav_ws/my_nav2_params.yaml}"
SENTRY_PLANNER_ROOT="${SENTRY_PLANNER_ROOT:-/home/nyu/sentry_planner}"
START_SERIAL_SENDER="${START_SERIAL_SENDER:-0}"
START_ROBOT_CONTROL_KEEPALIVE="${START_ROBOT_CONTROL_KEEPALIVE:-0}"
START_FAKE_VEL_TRANSFORM="${START_FAKE_VEL_TRANSFORM:-1}"
RADAR_PTY="${RADAR_PTY:-/tmp/nyush-rm-sentry-radar}"
SERIAL_SENDER_PORT="${SERIAL_SENDER_PORT:-$RADAR_PTY}"
SERIAL_SENDER_TOPIC="${SERIAL_SENDER_TOPIC:-/cmd_vel_chassis}"
ROBOT_CONTROL_TOPIC="${ROBOT_CONTROL_TOPIC:-/robot_control}"
SERIAL_SENDER_SCRIPT="${SERIAL_SENDER_SCRIPT:-/home/nyu/Codespace/nyush-rm-vision/serial_sender.py}"
FAKE_VEL_NAV_BASE_FRAME="${FAKE_VEL_NAV_BASE_FRAME:-base_link}"
FAKE_VEL_USE_PATH_HEADING_COMPENSATION="${FAKE_VEL_USE_PATH_HEADING_COMPENSATION:-false}"
FAKE_VEL_USE_NAV_WZ="${FAKE_VEL_USE_NAV_WZ:-false}"
FAKE_VEL_SWAP_NAV_XY="${FAKE_VEL_SWAP_NAV_XY:-false}"
FAKE_VEL_CHASSIS_X_SIGN="${FAKE_VEL_CHASSIS_X_SIGN:-1.0}"
FAKE_VEL_CHASSIS_Y_SIGN="${FAKE_VEL_CHASSIS_Y_SIGN:-1.0}"
FASTLIO_POINTCLOUD_TOPIC="${FASTLIO_POINTCLOUD_TOPIC:-/cloud_registered}"
FASTLIO_BODY_POINTCLOUD_TOPIC="${FASTLIO_BODY_POINTCLOUD_TOPIC:-/cloud_registered_body}"
LIDAR_BODY_PITCH="${LIDAR_BODY_PITCH:--0.873}"
ENABLE_DYNAMIC_GIMBAL_SCAN="${ENABLE_DYNAMIC_GIMBAL_SCAN:-1}"
GIMBAL_DYNAMIC_FRAME="${GIMBAL_DYNAMIC_FRAME:-body_feedback}"
GIMBAL_SCAN_FRAME="${GIMBAL_SCAN_FRAME:-body_scan}"
GIMBAL_SCAN_POINTCLOUD_TOPIC="${GIMBAL_SCAN_POINTCLOUD_TOPIC:-/cloud_registered_gimbal}"
CLOUD_FRAME_REWRITER_SCRIPT="${CLOUD_FRAME_REWRITER_SCRIPT:-$SENTRY_PLANNER_ROOT/scripts/reframe_pointcloud.py}"
GIMBAL_SCAN_PITCH="${GIMBAL_SCAN_PITCH:-$(python3 - "$LIDAR_BODY_PITCH" <<'PY'
import sys
print(f"{-float(sys.argv[1]):.6f}")
PY
)}"
POINTCLOUD_TO_LASERSCAN_QUEUE_SIZE="${POINTCLOUD_TO_LASERSCAN_QUEUE_SIZE:-64}"
POINTCLOUD_TO_LASERSCAN_TF_TOLERANCE="${POINTCLOUD_TO_LASERSCAN_TF_TOLERANCE:-0.1}"
PUBLISH_NAV2_INITIAL_POSE="${PUBLISH_NAV2_INITIAL_POSE:-0}"
NAV2_INITIAL_POSE_X="${NAV2_INITIAL_POSE_X:-0.8}"
NAV2_INITIAL_POSE_Y="${NAV2_INITIAL_POSE_Y:-7.8}"
NAV2_INITIAL_POSE_YAW="${NAV2_INITIAL_POSE_YAW:-0.0}"
WAIT_MANUAL_INITIAL_POSE="${WAIT_MANUAL_INITIAL_POSE:-1}"
WAIT_MANUAL_INITIAL_POSE_TIMEOUT="${WAIT_MANUAL_INITIAL_POSE_TIMEOUT:-600}"
RVIZ_STARTED=0

to_ros_bool() {
    case "${1,,}" in
        1|true|yes|on) echo "true" ;;
        0|false|no|off|"") echo "false" ;;
        *)
            echo "警告: 无法识别布尔值 '$1'，按 false 处理" >&2
            echo "false"
            ;;
    esac
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

launch_rviz_background() {
    if [ "$ENABLE_RVIZ" != "1" ] || [ "$RVIZ_STARTED" = "1" ]; then
        return
    fi
    echo ">>> Launching RViz..."
    ros2 run rviz2 rviz2 -d "$(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/rviz/nav2_default_view.rviz" &
    RVIZ_STARTED=1
    sleep 2
}

cleanup() {
    echo "正在关闭所有节点..."
    kill $(jobs -p) 2>/dev/null || true
}

trap 'cleanup; exit' SIGINT SIGTERM

echo ">>> [0/9] 清理环境..."
# 清理 FastRTPS 共享内存（默认关闭，避免把已运行的 bridge DDS 通信打断）
if [ "$RESET_FASTRTPS_SHM" = "1" ]; then
    rm -f /dev/shm/fastrtps_*
fi
# 重启 ROS2 daemon
ros2 daemon stop
ros2 daemon start
# 停止所有 Fast-LIO 相关进程（确保干净启动）
echo "   正在停止所有 Fast-LIO 相关进程..."
pkill -9 -f fastlio_mapping 2>/dev/null || true
pkill -9 -f livox_ros_driver2 2>/dev/null || true
pkill -9 -f pointcloud_to_laserscan 2>/dev/null || true
pkill -9 -f nav2 2>/dev/null || true
pkill -9 -f fake_vel_transform 2>/dev/null || true
pkill -9 -f static_transform_publisher 2>/dev/null || true
pkill -9 -f "$CLOUD_FRAME_REWRITER_SCRIPT --input-topic $FASTLIO_BODY_POINTCLOUD_TOPIC" 2>/dev/null || true
pkill -9 -f "$SERIAL_SENDER_SCRIPT --port $SERIAL_SENDER_PORT" 2>/dev/null || true
pkill -9 -f "ros2 topic pub -r 10 $ROBOT_CONTROL_TOPIC rm_decision_interfaces/msg/RobotControl" 2>/dev/null || true
# 等待进程完全退出
sleep 2

echo ">>> [1/9] 初始化环境..."
source /opt/ros/humble/setup.bash
# 假设你的工作空间在这里，根据实际情况修改
source ~/nav_ws/install/setup.bash 
if [ -f "$SENTRY_PLANNER_ROOT/install/setup.bash" ]; then
    source "$SENTRY_PLANNER_ROOT/install/setup.bash"
fi

echo ">>> [2/9] 设置串口权限 (如果卡在这里，请输入密码)..."
# 建议永久解决权限问题，避免每次都 sudo
if [ -e /dev/ttyACM0 ]; then
    sudo chmod 777 /dev/ttyACM0
else
    echo "警告: 未检测到 /dev/ttyACM0，跳过权限设置"
fi

echo ">>> [3/9] 启动 MID360 激光雷达驱动..."

# 单雷达：msg_MID360_launch.py；双雷达：dual_lidar_merge_launch.py（需先配置网络与路由）
ros2 launch livox_ros_driver2 msg_MID360_launch.py &
DRIVER_PID=$!
echo ">>> 等待雷达驱动启动 (5秒)..."
sleep 5

# 检查驱动数据
if timeout 2 ros2 topic echo /livox/lidar --once > /dev/null 2>&1; then
    echo "✅ 雷达驱动已启动并发送数据"
else
    echo "❌ 错误：未检测到雷达数据！请检查网线连接或防火墙。"
    # exit 1  # 你可以选择在这里退出，或者继续尝试
fi

echo ">>> [4/9] 发布底盘 / 雷达 TF 变换..."
ros2 run tf2_ros static_transform_publisher \
    --x 0 --y 0 --z 0 \
    --yaw 0 --pitch 0 --roll 0 \
    --frame-id odom --child-frame-id camera_init &
ros2 run tf2_ros static_transform_publisher \
    --x 0 --y 0 --z 0 \
    --yaw 0 --pitch 0 --roll 0 \
    --frame-id base_link --child-frame-id base_footprint &
ros2 run tf2_ros static_transform_publisher \
    --x 0 --y 0 --z 0 \
    --yaw 0 --pitch "$LIDAR_BODY_PITCH" --roll 0 \
    --frame-id body --child-frame-id base_link &
if [ "$ENABLE_DYNAMIC_GIMBAL_SCAN" = "1" ]; then
    ros2 run tf2_ros static_transform_publisher \
        --x 0 --y 0 --z 0 \
        --yaw 0 --pitch "$GIMBAL_SCAN_PITCH" --roll 0 \
        --frame-id "$GIMBAL_DYNAMIC_FRAME" --child-frame-id "$GIMBAL_SCAN_FRAME" &
fi

echo ">>> [5/9] 启动 FAST-LIO 里程计..."
# 设置 libusb 环境变量（修复 PCL 兼容性问题）
export LD_PRELOAD=/lib/x86_64-linux-gnu/libusb-1.0.so.0
ros2 launch fast_lio mapping.launch.py config_file:=mid360_single.yaml &

sleep 5

echo ">>> [6/9] 启动 Pointcloud 转 LaserScan..."
POINTCLOUD_TO_SCAN_INPUT_TOPIC="$FASTLIO_POINTCLOUD_TOPIC"
POINTCLOUD_TO_SCAN_SOURCE_DESC="$FASTLIO_POINTCLOUD_TOPIC"
if [ "$ENABLE_DYNAMIC_GIMBAL_SCAN" = "1" ] && [ -f "$CLOUD_FRAME_REWRITER_SCRIPT" ]; then
    echo ">>> [5.8/9] 检查动态云台 TF..."
    if wait_for_tf base_link "$GIMBAL_DYNAMIC_FRAME" 10; then
        echo ">>> [5.9/9] 启动云台补偿点云重映射..."
        python3 "$CLOUD_FRAME_REWRITER_SCRIPT" \
            --input-topic "$FASTLIO_BODY_POINTCLOUD_TOPIC" \
            --output-topic "$GIMBAL_SCAN_POINTCLOUD_TOPIC" \
            --output-frame "$GIMBAL_SCAN_FRAME" &
        sleep 2
        if timeout 10 ros2 topic echo "$GIMBAL_SCAN_POINTCLOUD_TOPIC" --once >/dev/null 2>&1; then
            POINTCLOUD_TO_SCAN_INPUT_TOPIC="$GIMBAL_SCAN_POINTCLOUD_TOPIC"
            POINTCLOUD_TO_SCAN_SOURCE_DESC="$FASTLIO_BODY_POINTCLOUD_TOPIC -> $GIMBAL_SCAN_POINTCLOUD_TOPIC(frame=$GIMBAL_SCAN_FRAME)"
            echo "✅ 已启用动态云台补偿 scan 链"
        else
            echo "⚠️  动态云台补偿点云未及时输出，回退到 $FASTLIO_POINTCLOUD_TOPIC"
        fi
    else
        echo "⚠️  未检测到 $GIMBAL_DYNAMIC_FRAME TF，回退到 $FASTLIO_POINTCLOUD_TOPIC"
    fi
fi

# 默认保持 FAST-LIO 全局点云链稳定；若动态云台补偿链就绪，则自动切到补偿后的 body 点云。
ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node --ros-args \
    -p target_frame:=base_link \
    -p transform_tolerance:="$POINTCLOUD_TO_LASERSCAN_TF_TOLERANCE" \
    -p queue_size:="$POINTCLOUD_TO_LASERSCAN_QUEUE_SIZE" \
    -p min_height:=-0.4 \
    -p max_height:=1.0 \
    -p angle_min:=-3.1415 \
    -p angle_max:=3.1415 \
    -p range_min:=0.1 \
    -p range_max:=20.0 \
    -p use_inf:=true \
    -p qos_overrides./cloud_in.reliability:=best_effort \
    -r cloud_in:="$POINTCLOUD_TO_SCAN_INPUT_TOPIC" \
    -r scan:=/scan &

echo ">>> [6.3/9] 等待 odom -> base_link TF..."
if ! wait_for_tf odom base_link 20; then
    echo "❌ 错误：未等到 odom -> base_link TF。"
    echo "   请检查 FAST-LIO 与静态 TF 是否正常启动。"
    exit 1
fi

echo ">>> [6.6/9] 等待 /scan 数据..."
if ! timeout 20 ros2 topic echo /scan --once >/dev/null 2>&1; then
    echo "❌ 错误：未检测到 /scan 数据。"
    echo "   请检查 pointcloud_to_laserscan 日志，以及 TF 链是否连通。"
    exit 1
fi

if rg -q 'odom_topic:[[:space:]]*"?/chassis_odom"?' "$NAV2_PARAMS_FILE"; then
    echo ">>> [6.8/9] 检查 /chassis_odom..."
    if ! timeout 10 ros2 topic echo /chassis_odom --once >/dev/null 2>&1; then
        echo "⚠️  警告：当前参数文件使用 /chassis_odom，但暂时还没收到消息。"
        echo "   localization 仍会继续启动，但请确认 sentry_bridge 已经在发 odom。"
    fi
fi

if [ "$PUBLISH_NAV2_INITIAL_POSE" != "1" ] && [ "$WAIT_MANUAL_INITIAL_POSE" != "1" ]; then
    echo "❌ 错误：当前启动顺序需要 initial pose。"
    echo "   请设置 PUBLISH_NAV2_INITIAL_POSE=1，或保持 WAIT_MANUAL_INITIAL_POSE=1。"
    exit 1
fi

echo ">>> [7/9] 启动 Nav2 localization..."
ros2 launch nav2_bringup localization_launch.py \
    use_sim_time:=False \
    map:="$MAP_FILE" \
    params_file:="$NAV2_PARAMS_FILE" &
sleep 6

if [ "$PUBLISH_NAV2_INITIAL_POSE" = "1" ]; then
    echo ">>> [7.3/9] 发布 initial pose..."
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
    ros2 topic pub -1 /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "$INITIAL_POSE_MSG" >/dev/null
    sleep 1
fi

if [ "$WAIT_MANUAL_INITIAL_POSE" = "1" ] && [ "$PUBLISH_NAV2_INITIAL_POSE" != "1" ]; then
    launch_rviz_background
    echo ">>> [7.6/9] 等待手动 /initialpose..."
    echo "    请在 RViz 中使用 '2D Pose Estimate'。"
    if ! wait_for_initial_pose "$WAIT_MANUAL_INITIAL_POSE_TIMEOUT"; then
        echo "❌ 错误：等待手动 /initialpose 超时。"
        exit 1
    fi
fi

echo ">>> [7.8/9] 等待 map -> base_link TF..."
if ! wait_for_tf map base_link 60; then
    echo "❌ 错误：未等到 map -> base_link TF。"
    echo "   请检查 AMCL 是否已经收到 /scan 与 /initialpose。"
    exit 1
fi

echo ">>> [8/9] 启动 Nav2 navigation..."
ros2 launch nav2_bringup navigation_launch.py \
    use_sim_time:=False \
    params_file:="$NAV2_PARAMS_FILE" &
sleep 6

if [ "$START_FAKE_VEL_TRANSFORM" = "1" ]; then
    echo ">>> [8.3/9] 启动 fake_vel_transform (/cmd_vel -> /cmd_vel_chassis)..."
    ROS_FAKE_VEL_USE_PATH_HEADING_COMPENSATION="$(to_ros_bool "$FAKE_VEL_USE_PATH_HEADING_COMPENSATION")"
    ROS_FAKE_VEL_USE_NAV_WZ="$(to_ros_bool "$FAKE_VEL_USE_NAV_WZ")"
    ROS_FAKE_VEL_SWAP_NAV_XY="$(to_ros_bool "$FAKE_VEL_SWAP_NAV_XY")"
    ros2 run fake_vel_transform fake_vel_transform_node --ros-args \
        -p nav_base_frame:="$FAKE_VEL_NAV_BASE_FRAME" \
        -p use_path_heading_compensation:="$ROS_FAKE_VEL_USE_PATH_HEADING_COMPENSATION" \
        -p use_nav_wz:="$ROS_FAKE_VEL_USE_NAV_WZ" \
        -p swap_nav_xy:="$ROS_FAKE_VEL_SWAP_NAV_XY" \
        -p chassis_x_sign:="$FAKE_VEL_CHASSIS_X_SIGN" \
        -p chassis_y_sign:="$FAKE_VEL_CHASSIS_Y_SIGN" &
    sleep 2
    if ! timeout 5 ros2 topic info /cmd_vel_chassis >/dev/null 2>&1; then
        echo "❌ 错误：fake_vel_transform 未成功发布 /cmd_vel_chassis"
        echo "   请检查 fake_vel_transform 的启动日志。"
        exit 1
    fi
fi

if [ "$START_SERIAL_SENDER" = "1" ]; then
    if [ -z "$SERIAL_SENDER_PORT" ]; then
        echo "❌ 错误：START_SERIAL_SENDER=1 但没有设置 SERIAL_SENDER_PORT / RADAR_PTY"
        exit 1
    fi
    if [ ! -e "$SERIAL_SENDER_PORT" ]; then
        echo "❌ 错误：未检测到通讯端口 $SERIAL_SENDER_PORT"
        echo "   请先启动 sentry_bridge，例如：just sentry-bridge --port /dev/ttyACM0"
        exit 1
    fi
    if [ ! -f "$SERIAL_SENDER_SCRIPT" ]; then
        echo "❌ 错误：未找到 bridge-compatible serial_sender: $SERIAL_SENDER_SCRIPT"
        exit 1
    fi
    echo ">>> [8.5/9] 启动 bridge-compatible serial_sender..."
    python3 "$SERIAL_SENDER_SCRIPT" \
        --port "$SERIAL_SENDER_PORT" \
        --ros2 \
        --topic "$SERIAL_SENDER_TOPIC" \
        --robot-control-topic "$ROBOT_CONTROL_TOPIC" &
fi

if [ "$START_ROBOT_CONTROL_KEEPALIVE" = "1" ]; then
    echo ">>> [8.6/9] 启动 RobotControl keepalive..."
    ros2 topic pub -r 10 "$ROBOT_CONTROL_TOPIC" rm_decision_interfaces/msg/RobotControl \
        "{stop_gimbal_scan: true, chassis_spin_vel: 0.0, scan_enabled: false, allow_vision_control: false, search_when_target_lost: false, scan_yaw_rate_deg_s: 0.0, search_pitch_deg: 0.0}" &
fi


echo ">>> [9/9] 启动完成！"
echo "-----------------------------------------------------"
echo ">>> 机器人端所有节点已启动！"
if [ "$ENABLE_RVIZ" = "1" ]; then
    echo ">>> RViz 会在本机自动启动"
else
    echo ">>> 当前未自动启动 RViz；如需手动打点/发目标，请自行运行 rviz2"
fi
echo ">>> 按 Ctrl+C 可以停止脚本并关闭所有节点"
echo ""
echo "💡 提示："
echo "   - Point-LIO 会在启动时进行 IMU 初始化（约 10-12 秒）"
echo "   - 初始化时请确保设备完全静止，处于你希望使用的姿态"
echo "   - 如果需要在不同姿态下使用，请重新启动脚本"
echo "   - 如果点云旋转，请检查配置中的 start_in_aggressive_motion 是否为 true"
echo "   - 当前雷达链: odom -> camera_init (静态) + camera_init -> body (FAST-LIO) + body -> base_link (固定安装外参)"
if [ "$ENABLE_DYNAMIC_GIMBAL_SCAN" = "1" ]; then
    echo "   - 动态云台链: base_link -> base_footprint -> $GIMBAL_DYNAMIC_FRAME -> $GIMBAL_SCAN_FRAME"
fi
echo "   - 当前 scan 链: $POINTCLOUD_TO_SCAN_SOURCE_DESC -> pointcloud_to_laserscan(target_frame=base_link) -> /scan"
if [ "$START_SERIAL_SENDER" = "1" ]; then
    echo "   - 当前通讯链: $SERIAL_SENDER_TOPIC -> $SERIAL_SENDER_PORT -> sentry_bridge -> MCU"
    if [ "$START_FAKE_VEL_TRANSFORM" = "1" ]; then
        echo "   - fake_vel_transform: /cmd_vel -> /cmd_vel_chassis (默认屏蔽 Nav2 angular.z)"
    fi
    echo "   - 可检查: ros2 topic echo $SERIAL_SENDER_TOPIC --once"
    echo "             ros2 topic echo $ROBOT_CONTROL_TOPIC --once"
fi
echo "-----------------------------------------------------"

if [ "$ENABLE_RVIZ" = "1" ]; then
    launch_rviz_background
fi

wait
