#!/bin/bash

# --- 脚本功能：一键启动 激光雷达 + FAST-LIO + Nav2 ---
# 兼容当前 bridge + Radar PTY 通讯链，但不改变原有导航技术栈。

set -eo pipefail

RESET_FASTRTPS_SHM="${RESET_FASTRTPS_SHM:-0}"
ENABLE_RVIZ="${ENABLE_RVIZ:-1}"
MAP_FILE="${MAP_FILE:-/home/nyu/Desktop/map/11_map.yaml}"
NAV2_PARAMS_FILE="${NAV2_PARAMS_FILE:-/home/nyu/nav_ws/my_nav2_params.yaml}"
SENTRY_PLANNER_ROOT="${SENTRY_PLANNER_ROOT:-/home/nyu/sentry_planner}"
RM_VISION_WS_ROOT="${RM_VISION_WS_ROOT:-$SENTRY_PLANNER_ROOT/rm_vision_ws}"
RM_DECISION_WS_ROOT="${RM_DECISION_WS_ROOT:-$SENTRY_PLANNER_ROOT/rm_decision_ws}"
START_BT="${START_BT:-1}"
BT_STYLE="${BT_STYLE:-center_attack_fullstack}"
BT_USE_SIM_TIME="${BT_USE_SIM_TIME:-False}"
BT_RESPAWN="${BT_RESPAWN:-False}"
ENABLE_GROOT="${ENABLE_GROOT:-False}"
GROOT_PORT="${GROOT_PORT:-1667}"
START_SERIAL_SENDER="${START_SERIAL_SENDER:-0}"
START_ROBOT_CONTROL_KEEPALIVE="${START_ROBOT_CONTROL_KEEPALIVE:-0}"
START_FAKE_VEL_TRANSFORM="${START_FAKE_VEL_TRANSFORM:-1}"
START_GIMBAL_CLOUD_COMPENSATION="${START_GIMBAL_CLOUD_COMPENSATION:-0}"
RADAR_PTY="${RADAR_PTY:-/tmp/nyush-rm-sentry-radar}"
SERIAL_SENDER_PORT="${SERIAL_SENDER_PORT:-$RADAR_PTY}"
SERIAL_SENDER_TOPIC="${SERIAL_SENDER_TOPIC:-/cmd_vel_chassis}"
ROBOT_CONTROL_TOPIC="${ROBOT_CONTROL_TOPIC:-/robot_control}"
SERIAL_SENDER_SCRIPT="${SERIAL_SENDER_SCRIPT:-/home/nyu/Codespace/nyush-rm-vision/serial_sender.py}"
BT_COMM_ADAPTER_SCRIPT="${BT_COMM_ADAPTER_SCRIPT:-$SENTRY_PLANNER_ROOT/scripts/bt_comm_adapter.py}"
GIMBAL_CLOUD_COMPENSATOR_SCRIPT="${GIMBAL_CLOUD_COMPENSATOR_SCRIPT:-$SENTRY_PLANNER_ROOT/scripts/compensate_gimbal_pointcloud.py}"
GIMBAL_STATE_TOPIC="${GIMBAL_STATE_TOPIC:-/gimbal_joint_states}"
CLOUD_BODY_TOPIC="${CLOUD_BODY_TOPIC:-/cloud_registered_body}"
COMPENSATED_CLOUD_TOPIC="${COMPENSATED_CLOUD_TOPIC:-/cloud_registered_body_compensated}"
COMPENSATED_CLOUD_FRAME="${COMPENSATED_CLOUD_FRAME:-base_link}"
BODY_TO_BASE_PITCH_RAD="${BODY_TO_BASE_PITCH_RAD:--0.873}"
GIMBAL_COMP_STATE_SOURCE="${GIMBAL_COMP_STATE_SOURCE:-joint_state}"
GIMBAL_COMP_MAX_STATE_AGE="${GIMBAL_COMP_MAX_STATE_AGE:-0.5}"
GIMBAL_COMP_MAX_EXTRAPOLATION="${GIMBAL_COMP_MAX_EXTRAPOLATION:-0.15}"
GIMBAL_COMP_YAW_SIGN="${GIMBAL_COMP_YAW_SIGN:-1.0}"
GIMBAL_COMP_CONSTANT_YAW_RATE_DEG_S="${GIMBAL_COMP_CONSTANT_YAW_RATE_DEG_S:-120.0}"
GIMBAL_COMP_INITIAL_YAW_DEG="${GIMBAL_COMP_INITIAL_YAW_DEG:-0.0}"
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

wait_for_action_server() {
    local action_name="$1"
    local timeout_sec="${2:-20}"
    local elapsed=0

    while [ "$elapsed" -lt "$timeout_sec" ]; do
        if ros2 action list 2>/dev/null | grep -qE "^/?${action_name}$"; then
            return 0
        fi
        sleep 1
        elapsed=$((elapsed + 1))
    done

    return 1
}

wait_for_topic_once() {
    local topic_name="$1"
    local timeout_sec="${2:-8}"

    if timeout "$timeout_sec" ros2 topic echo "$topic_name" --once > /dev/null 2>&1; then
        return 0
    fi

    return 1
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
pkill -9 -f "$BT_COMM_ADAPTER_SCRIPT" 2>/dev/null || true
pkill -9 -f rm_behavior_tree 2>/dev/null || true
pkill -9 -f "$SERIAL_SENDER_SCRIPT --port $SERIAL_SENDER_PORT" 2>/dev/null || true
pkill -9 -f "ros2 topic pub -r 10 $ROBOT_CONTROL_TOPIC rm_decision_interfaces/msg/RobotControl" 2>/dev/null || true
pkill -9 -f "$GIMBAL_CLOUD_COMPENSATOR_SCRIPT" 2>/dev/null || true
# 等待进程完全退出
sleep 2

echo ">>> [1/9] 初始化环境..."
source /opt/ros/humble/setup.bash
# 假设你的工作空间在这里，根据实际情况修改
source ~/nav_ws/install/setup.bash 
if [ -f "$SENTRY_PLANNER_ROOT/install/setup.bash" ]; then
    source "$SENTRY_PLANNER_ROOT/install/setup.bash"
fi
if [ -f "$RM_VISION_WS_ROOT/install/setup.bash" ]; then
    source "$RM_VISION_WS_ROOT/install/setup.bash"
fi
if [ -f "$RM_DECISION_WS_ROOT/install/setup.bash" ]; then
    source "$RM_DECISION_WS_ROOT/install/setup.bash"
fi

if [ "$START_BT" = "1" ]; then
    if [ ! -f "$BT_COMM_ADAPTER_SCRIPT" ]; then
        echo "❌ 错误：未找到 bt_comm_adapter 脚本: $BT_COMM_ADAPTER_SCRIPT"
        exit 1
    fi
    if ! ros2 pkg prefix rm_behavior_tree >/dev/null 2>&1; then
        echo "❌ 错误：当前环境里没有 rm_behavior_tree，请先构建/安装 rm_decision_ws"
        exit 1
    fi
    if [ "$START_ROBOT_CONTROL_KEEPALIVE" = "1" ]; then
        echo ">>> 检测到 START_BT=1，自动关闭固定 keepalive，避免覆盖 BT 的 /robot_control"
        START_ROBOT_CONTROL_KEEPALIVE=0
    fi
    if [ "$SERIAL_SENDER_TOPIC" != "/cmd_vel_chassis_bt" ]; then
        echo ">>> 检测到 START_BT=1，自动将 SERIAL_SENDER_TOPIC 切到 /cmd_vel_chassis_bt"
        SERIAL_SENDER_TOPIC="/cmd_vel_chassis_bt"
    fi
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
ros2 run tf2_ros static_transform_publisher 0 0 0 0 "$BODY_TO_BASE_PITCH_RAD" 0 body base_link &
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link base_footprint &
if [ "$START_GIMBAL_CLOUD_COMPENSATION" = "1" ]; then
    :
fi

echo ">>> [5/9] 启动 FAST-LIO 里程计..."
# 设置 libusb 环境变量（修复 PCL 兼容性问题）
export LD_PRELOAD=/lib/x86_64-linux-gnu/libusb-1.0.so.0
ros2 launch fast_lio mapping.launch.py config_file:=mid360_single.yaml &

sleep 5

echo ">>> [6/9] 启动 Pointcloud 转 LaserScan..."
POINTCLOUD_INPUT_TOPIC="/cloud_registered"
if [ "$START_GIMBAL_CLOUD_COMPENSATION" = "1" ]; then
    if [ ! -f "$GIMBAL_CLOUD_COMPENSATOR_SCRIPT" ]; then
        echo "❌ 错误：未找到点云补偿脚本: $GIMBAL_CLOUD_COMPENSATOR_SCRIPT"
        exit 1
    fi
    echo ">>> [6.1/9] 启动云台点云补偿 (/cloud_registered_body -> $COMPENSATED_CLOUD_TOPIC)..."
    python3 "$GIMBAL_CLOUD_COMPENSATOR_SCRIPT" \
        --cloud-topic "$CLOUD_BODY_TOPIC" \
        --state-topic "$GIMBAL_STATE_TOPIC" \
        --output-topic "$COMPENSATED_CLOUD_TOPIC" \
        --output-frame "$COMPENSATED_CLOUD_FRAME" \
        --state-source "$GIMBAL_COMP_STATE_SOURCE" \
        --max-state-age "$GIMBAL_COMP_MAX_STATE_AGE" \
        --max-extrapolation "$GIMBAL_COMP_MAX_EXTRAPOLATION" \
        --yaw-sign "$GIMBAL_COMP_YAW_SIGN" \
        --constant-yaw-rate-deg-s "$GIMBAL_COMP_CONSTANT_YAW_RATE_DEG_S" \
        --initial-yaw-deg "$GIMBAL_COMP_INITIAL_YAW_DEG" \
        --compensate-pitch \
        --pitch-sign 1.0 \
        --pitch-offset-rad "$BODY_TO_BASE_PITCH_RAD" &
    if [ "$GIMBAL_COMP_STATE_SOURCE" = "constant_rate" ]; then
        echo ">>> [6.2/9] 固定角速度补偿模式已启用，LaserScan 输入强制切到 $COMPENSATED_CLOUD_TOPIC"
        POINTCLOUD_INPUT_TOPIC="$COMPENSATED_CLOUD_TOPIC"
    elif wait_for_topic_once "$COMPENSATED_CLOUD_TOPIC" 8; then
        echo ">>> [6.2/9] 云台点云补偿已生效，LaserScan 输入切到 $COMPENSATED_CLOUD_TOPIC"
        POINTCLOUD_INPUT_TOPIC="$COMPENSATED_CLOUD_TOPIC"
    else
        echo ">>> [6.2/9] 云台点云补偿未成功发布，回退到 /cloud_registered"
    fi
fi
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
    -r cloud_in:="$POINTCLOUD_INPUT_TOPIC" \
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

if [ "$START_BT" = "1" ]; then
    echo ">>> [8.4/9] 等待 navigate_to_pose action server..."
    if ! wait_for_action_server "navigate_to_pose" 20; then
        echo "❌ 错误：20 秒内未检测到 /navigate_to_pose action server"
        echo "   当前不启动 BT，请先确认 Nav2 已完全起来。"
        exit 1
    fi

    echo ">>> [8.5/9] 启动 bt_comm_adapter..."
    python3 "$BT_COMM_ADAPTER_SCRIPT" &
    sleep 2

    echo ">>> [8.6/9] 启动决策行为树 ($BT_STYLE)..."
    ros2 launch rm_behavior_tree rm_behavior_tree.launch.py \
        style:="$BT_STYLE" \
        use_sim_time:="$BT_USE_SIM_TIME" \
        respawn:="$BT_RESPAWN" \
        enable_groot:="$ENABLE_GROOT" \
        groot_port:="$GROOT_PORT" &
    sleep 2
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
    echo ">>> [8.7/9] 启动 bridge-compatible serial_sender..."
    python3 "$SERIAL_SENDER_SCRIPT" \
        --port "$SERIAL_SENDER_PORT" \
        --ros2 \
        --topic "$SERIAL_SENDER_TOPIC" \
        --robot-control-topic "$ROBOT_CONTROL_TOPIC" &
fi

if [ "$START_ROBOT_CONTROL_KEEPALIVE" = "1" ]; then
    echo ">>> [8.8/9] 启动 RobotControl keepalive..."
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
    if [ "$START_BT" = "1" ]; then
        echo "   - BT 链路: /cmd_vel_chassis + /robot_control -> bt_comm_adapter -> /cmd_vel_chassis_bt"
        echo "   - 当前行为树: $BT_STYLE"
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
