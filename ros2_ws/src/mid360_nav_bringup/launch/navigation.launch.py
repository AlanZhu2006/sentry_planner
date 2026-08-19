from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetRemap


def generate_launch_description():
    share = Path(get_package_share_directory('mid360_nav_bringup'))
    nav2_share = Path(get_package_share_directory('nav2_bringup'))

    common_robot_args = {
        name: LaunchConfiguration(name) for name in (
            'use_sim_time', 'use_ekf', 'chassis_port', 'livox_config', 'wheel_odom_topic',
            'base_length', 'base_width', 'base_height', 'lidar_x', 'lidar_y',
            'lidar_z', 'lidar_roll', 'lidar_pitch', 'lidar_yaw')
    }

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_ekf', default_value='true'),
        DeclareLaunchArgument('chassis_port', default_value=''),
        DeclareLaunchArgument('livox_config', default_value=str(share / 'config' / 'mid360_config.json')),
        DeclareLaunchArgument('wheel_odom_topic', default_value='/wheel/odom'),
        DeclareLaunchArgument(
            'map', default_value=str(share / 'maps' / 'fastlio_map.yaml')),
        DeclareLaunchArgument('params_file', default_value=str(share / 'config' / 'nav2_params.yaml')),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument(
            'rviz_config', default_value=str(share / 'rviz' / 'nav2_vnc.rviz')),
        DeclareLaunchArgument('base_length', default_value='0.55'),
        DeclareLaunchArgument('base_width', default_value='0.55'),
        DeclareLaunchArgument('base_height', default_value='0.50'),
        DeclareLaunchArgument('lidar_x', default_value='0.0'),
        DeclareLaunchArgument('lidar_y', default_value='-0.11'),
        DeclareLaunchArgument('lidar_z', default_value='0.35'),
        DeclareLaunchArgument('lidar_roll', default_value='0.0'),
        DeclareLaunchArgument('lidar_pitch', default_value='0.0'),
        DeclareLaunchArgument('lidar_yaw', default_value='0.0'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / 'launch' / 'robot.launch.py')),
            launch_arguments=common_robot_args.items(),
        ),
        GroupAction([
            SetRemap(src='/cmd_vel', dst='/cmd_vel_nav'),
            SetRemap(src='cmd_vel_smoothed', dst='/cmd_vel_nav_smoothed'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(nav2_share / 'launch' / 'bringup_launch.py')),
                launch_arguments={
                    'slam': 'False',
                    'map': LaunchConfiguration('map'),
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'params_file': LaunchConfiguration('params_file'),
                    'autostart': LaunchConfiguration('autostart'),
                }.items(),
            ),
        ]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(nav2_share / 'launch' / 'rviz_launch.py')),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'rviz_config': LaunchConfiguration('rviz_config'),
            }.items(),
            condition=IfCondition(LaunchConfiguration('use_rviz')),
        ),
    ])
