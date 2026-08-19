from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory('mid360_nav_bringup'))

    return LaunchDescription([
        DeclareLaunchArgument('chassis_port', default_value=''),
        DeclareLaunchArgument('wheel_odom_topic', default_value='/wheel/odom'),
        DeclareLaunchArgument('start_bridge', default_value='true'),
        DeclareLaunchArgument('start_mux', default_value='true'),
        DeclareLaunchArgument('max_translation_m_s', default_value='0.80'),
        DeclareLaunchArgument('max_angular_rad_s', default_value='2.40'),

        Node(
            package='mid360_nav_bringup',
            executable='sentry_nav2_bridge.py',
            name='sentry_nav2_bridge',
            output='screen',
            condition=IfCondition(LaunchConfiguration('start_bridge')),
            parameters=[{
                'port': LaunchConfiguration('chassis_port'),
                'command_topic': '/cmd_vel',
                'odom_topic': LaunchConfiguration('wheel_odom_topic'),
                'max_translation_m_s': ParameterValue(
                    LaunchConfiguration('max_translation_m_s'), value_type=float),
                'max_angular_rad_s': ParameterValue(
                    LaunchConfiguration('max_angular_rad_s'), value_type=float),
            }],
            on_exit=Shutdown(
                reason='chassis bridge exited; teleoperation is unavailable'),
        ),
        Node(
            package='twist_mux',
            executable='twist_mux',
            name='twist_mux',
            output='screen',
            condition=IfCondition(LaunchConfiguration('start_mux')),
            parameters=[str(share / 'config' / 'twist_mux.yaml')],
            remappings=[('cmd_vel_out', '/cmd_vel')],
        ),
    ])
