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
pkill -9 -f fast_lio_mapping 2>/dev/null || true
pkill -9 -f livox_ros_driver2 2>/dev/null || true
pkill -9 -f pointcloud_to_laserscan 2>/dev/null || true
pkill -9 -f nav2 2>/dev/null || true
pkill -9 -f fake_vel_transform 2>/dev/null || true
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

echo ">>> [4/9] 发布静态 TF 变换..."
# 保持你原来的 TF 树配置
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom camera_init &
ros2 run tf2_ros static_transform_publisher 0 0 0 0 -0.873 0 body base_link &
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link base_footprint &

echo ">>> [5/9] 启动 FAST-LIO 里程计..."
# 设置 libusb 环境变量（修复 PCL 兼容性问题）
export LD_PRELOAD=/lib/x86_64-linux-gnu/libusb-1.0.so.0
ros2 launch fast_lio mapping.launch.py config_file:=mid360_single.yaml &

sleep 5

echo ">>> [6/9] 启动 Pointcloud 转 LaserScan..."
# 关键修改：
# 1. 输入话题改回 /cloud_registered (因为它是标准的 PointCloud2)
# 2. 增加 qos_overrides (强制让节点能听到 Best Effort 的数据)
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

echo ">>> [7/9] 启动 Nav2 导航..."
ros2 launch nav2_bringup bringup_launch.py \
    use_sim_time:=False \
    map:="$MAP_FILE" \
    params_file:="$NAV2_PARAMS_FILE" &

echo ">>> [8/9] 等待 Nav2 启动 (8秒)..."
sleep 8

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
echo ">>> 请在【你的笔记本电脑】上打开终端运行: rviz2"
echo ">>> 按 Ctrl+C 可以停止脚本并关闭所有节点"
echo ""
echo "💡 提示："
echo "   - Point-LIO 会在启动时进行 IMU 初始化（约 10-12 秒）"
echo "   - 初始化时请确保设备完全静止，处于你希望使用的姿态"
echo "   - 如果需要在不同姿态下使用，请重新启动脚本"
echo "   - 如果点云旋转，请检查配置中的 start_in_aggressive_motion 是否为 true"
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
    # 这是一个技巧：让脚本不退出，挂起等待，直到你按 Ctrl+C
    ros2 run rviz2 rviz2 -d $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/rviz/nav2_default_view.rviz
else
    wait
fi
