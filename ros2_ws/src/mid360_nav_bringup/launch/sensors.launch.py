from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory('mid360_nav_bringup'))
    xacro_file = share / 'urdf' / 'robot.urdf.xacro'
    default_livox_config = share / 'config' / 'mid360_config.json'
    scan_config = share / 'config' / 'pointcloud_to_laserscan.yaml'

    use_sim_time = LaunchConfiguration('use_sim_time')
    livox_config = LaunchConfiguration('livox_config')

    robot_description = ParameterValue(
        Command([
            'xacro ', str(xacro_file),
            ' base_length:=', LaunchConfiguration('base_length'),
            ' base_width:=', LaunchConfiguration('base_width'),
            ' base_height:=', LaunchConfiguration('base_height'),
            ' lidar_x:=', LaunchConfiguration('lidar_x'),
            ' lidar_y:=', LaunchConfiguration('lidar_y'),
            ' lidar_z:=', LaunchConfiguration('lidar_z'),
            ' lidar_roll:=', LaunchConfiguration('lidar_roll'),
            ' lidar_pitch:=', LaunchConfiguration('lidar_pitch'),
            ' lidar_yaw:=', LaunchConfiguration('lidar_yaw'),
        ]),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('livox_config', default_value=str(default_livox_config)),
        DeclareLaunchArgument('xfer_format', default_value='0'),
        DeclareLaunchArgument('use_scan', default_value='true'),
        DeclareLaunchArgument('scan_cloud_topic', default_value='/livox/lidar'),
        DeclareLaunchArgument('base_length', default_value='0.55'),
        DeclareLaunchArgument('base_width', default_value='0.55'),
        DeclareLaunchArgument('base_height', default_value='0.50'),
        DeclareLaunchArgument('lidar_x', default_value='0.0'),
        DeclareLaunchArgument('lidar_y', default_value='-0.11'),
        DeclareLaunchArgument('lidar_z', default_value='0.35'),
        DeclareLaunchArgument('lidar_roll', default_value='0.0'),
        DeclareLaunchArgument('lidar_pitch', default_value='0.0'),
        DeclareLaunchArgument('lidar_yaw', default_value='0.0'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description,
                         'use_sim_time': use_sim_time}],
        ),
        Node(
            package='livox_ros_driver2',
            executable='livox_ros_driver2_node',
            name='livox_lidar_publisher',
            output='screen',
            parameters=[{
                'xfer_format': ParameterValue(
                    LaunchConfiguration('xfer_format'), value_type=int),
                'multi_topic': 0,
                'data_src': 0,
                'publish_freq': 10.0,
                'output_data_type': 0,
                'frame_id': 'livox_frame',
                'lvx_file_path': '',
                'user_config_path': livox_config,
                'cmdline_input_bd_code': 'livox0000000001',
                'use_sim_time': use_sim_time,
            }],
        ),
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_scan')),
            parameters=[str(scan_config), {'use_sim_time': use_sim_time}],
            remappings=[
                ('cloud_in', LaunchConfiguration('scan_cloud_topic')),
                ('scan', '/scan'),
            ],
        ),
    ])
