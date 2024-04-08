#!/bin/zsh

# Start navigation
source install/setup.sh
ros2 launch rm_bringup bringup_real.launch.py \
    world:=RMUL \
    mode:=nav \
    localization:=slam_toolbox \
    nav_rviz:=False
