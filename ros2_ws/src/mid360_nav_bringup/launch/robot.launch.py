from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, Shutdown
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('mid360_nav_bringup'))
    sensors_launch = share / 'launch' / 'sensors.launch.py'

    forwarded_sensor_args = {
        name: LaunchConfiguration(name) for name in (
            'use_sim_time', 'livox_config', 'xfer_format', 'use_scan',
            'scan_cloud_topic', 'base_length', 'base_width',
            'base_height', 'lidar_x', 'lidar_y', 'lidar_z', 'lidar_roll',
            'lidar_pitch', 'lidar_yaw')
    }

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_ekf', default_value='true'),
        DeclareLaunchArgument('chassis_port', default_value=''),
        DeclareLaunchArgument('livox_config', default_value=str(share / 'config' / 'mid360_config.json')),
        DeclareLaunchArgument('xfer_format', default_value='0'),
        DeclareLaunchArgument('use_scan', default_value='true'),
        DeclareLaunchArgument('scan_cloud_topic', default_value='/livox/lidar'),
        DeclareLaunchArgument('wheel_odom_topic', default_value='/wheel/odom'),
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
            PythonLaunchDescriptionSource(str(sensors_launch)),
            launch_arguments=forwarded_sensor_args.items(),
        ),
        Node(
            package='mid360_nav_bringup',
            executable='sentry_nav2_bridge.py',
            name='sentry_nav2_bridge',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('chassis_port'),
                'command_topic': '/cmd_vel',
                'odom_topic': LaunchConfiguration('wheel_odom_topic'),
                'max_translation_m_s': 0.80,
                'max_angular_rad_s': 2.40,
            }],
            on_exit=Shutdown(
                reason='chassis bridge exited; no wheel odometry feedback'),
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_ekf')),
            parameters=[str(share / 'config' / 'ekf.yaml'),
                        {'use_sim_time': LaunchConfiguration('use_sim_time')}],
            remappings=[
                ('/wheel/odom', LaunchConfiguration('wheel_odom_topic')),
                ('odometry/filtered', '/odom'),
            ],
        ),
        Node(
            package='twist_mux',
            executable='twist_mux',
            name='twist_mux',
            output='screen',
            parameters=[str(share / 'config' / 'twist_mux.yaml')],
            remappings=[('cmd_vel_out', '/cmd_vel')],
        ),
    ])
