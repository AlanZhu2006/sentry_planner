#!/bin/bash

# --- 脚本功能：一键启动 激光雷达 + FAST-LIO + Nav2 + 决策行为树 ---

# 捕捉 Ctrl+C 信号，退出时自动清理所有后台进程
trap 'echo "正在关闭所有节点..."; kill $(jobs -p); exit' SIGINT

echo ">>> [0/10] 清理环境..."
# 清理 FastRTPS 共享内存
rm -f /dev/shm/fastrtps_*
# 重启 ROS2 daemon
ros2 daemon stop
ros2 daemon start
# 停止所有 Fast-LIO 相关进程（确保干净启动）
echo "   正在停止所有 Fast-LIO 相关进程..."
pkill -9 -f fast_lio_mapping 2>/dev/null
pkill -9 -f livox_ros_driver2 2>/dev/null
pkill -9 -f pointcloud_to_laserscan 2>/dev/null
pkill -9 -f nav2 2>/dev/null
pkill -9 -f rm_behavior_tree 2>/dev/null
# 等待进程完全退出
sleep 2

echo ">>> [1/10] 初始化环境..."
source /opt/ros/humble/setup.bash
# 假设你的工作空间在这里，根据实际情况修改
source ~/nav_ws/install/setup.bash 

echo ">>> [2/10] 设置串口权限 (如果卡在这里，请输入密码)..."
# 建议永久解决权限问题，避免每次都 sudo
if [ -e /dev/ttyACM0 ]; then
    sudo chmod 777 /dev/ttyACM0
else
    echo "警告: 未检测到 /dev/ttyACM0，跳过权限设置"
fi

echo ">>> [3/10] 启动 MI360 激光雷达驱动..."

# 启动驱动
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

echo ">>> [4/10] 发布静态 TF 变换..."
# 保持你原来的 TF 树配置
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom camera_init &
ros2 run tf2_ros static_transform_publisher 0 0 0 0 -0.873 0 body base_link &
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link base_footprint &

echo ">>> [5/10] 启动 FAST-LIO 里程计..."
# 设置 libusb 环境变量（修复 PCL 兼容性问题）
export LD_PRELOAD=/lib/x86_64-linux-gnu/libusb-1.0.so.0
ros2 launch fast_lio mapping.launch.py config_file:=mid360.yaml &

sleep 5

echo ">>> [6/10] 启动 Pointcloud 转 LaserScan..."
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

echo ">>> [7/10] 启动 Nav2 导航..."
# 使用项目自带的 RMUL 场地地图（高校联盟赛）
ros2 launch nav2_bringup bringup_launch.py \
    use_sim_time:=False \
    map:=$HOME/sentry_planner/rm_navigation_ws/src/rm_nav_bringup/map/RMUL.yaml \
    params_file:=/home/nyu/nav_ws/my_nav2_params.yaml &

echo ">>> [8/10] 等待 Nav2 启动 (8秒)..."
sleep 8

echo ">>> [9/10] 启动决策行为树..."
source ~/sentry_planner/rm_vision_ws/install/setup.bash
source ~/sentry_planner/rm_decision_ws/install/setup.bash
ros2 launch rm_behavior_tree rm_behavior_tree.launch.py \
    style:=retreat_attack_left \
    use_sim_time:=False &

sleep 2

echo ">>> [10/10] 启动完成！"
echo "-----------------------------------------------------"
echo ">>> 机器人端所有节点已启动！（含决策行为树）"
echo ">>> 请在【你的笔记本电脑】上打开终端运行: rviz2"
echo ">>> 按 Ctrl+C 可以停止脚本并关闭所有节点"
echo ""
echo "💡 提示："
echo "   - Point-LIO 会在启动时进行 IMU 初始化（约 10-12 秒）"
echo "   - 初始化时请确保设备完全静止，处于你希望使用的姿态"
echo "   - 如果需要在不同姿态下使用，请重新启动脚本"
echo "   - 如果点云旋转，请检查配置中的 start_in_aggressive_motion 是否为 true"
echo "-----------------------------------------------------"

# 这是一个技巧：让脚本不退出，挂起等待，直到你按 Ctrl+C
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/rviz/nav2_default_view.rviz
