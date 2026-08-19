from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory('mid360_nav_bringup'))
    fast_lio_share = Path(get_package_share_directory('fast_lio'))

    robot_args = {
        name: LaunchConfiguration(name) for name in (
            'use_sim_time', 'chassis_port', 'livox_config', 'wheel_odom_topic',
            'base_length', 'base_width', 'base_height', 'lidar_x', 'lidar_y',
            'lidar_z', 'lidar_roll', 'lidar_pitch', 'lidar_yaw')
    }
    robot_args.update({
        'use_ekf': 'false',
        'xfer_format': '1',
        'use_scan': 'false',
    })

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('chassis_port', default_value=''),
        DeclareLaunchArgument(
            'livox_config',
            default_value=str(share / 'config' / 'mid360_config.json')),
        DeclareLaunchArgument('wheel_odom_topic', default_value='/wheel/odom'),
        DeclareLaunchArgument(
            'fastlio_config',
            default_value=str(fast_lio_share / 'config' / 'mid360.yaml')),
        DeclareLaunchArgument(
            'pcd_path', default_value=str(share / 'maps' / 'fastlio_map.pcd')),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=str(fast_lio_share / 'rviz' / 'fastlio.rviz')),
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
            PythonLaunchDescriptionSource(
                str(share / 'launch' / 'robot.launch.py')),
            launch_arguments=robot_args.items(),
        ),
        Node(
            package='fast_lio',
            executable='fastlio_mapping',
            name='laser_mapping',
            output='screen',
            parameters=[
                LaunchConfiguration('fastlio_config'),
                {
                    'use_sim_time': ParameterValue(
                        LaunchConfiguration('use_sim_time'), value_type=bool),
                    'map_file_path': LaunchConfiguration('pcd_path'),
                    'common.lid_topic': '/livox/lidar',
                    'common.imu_topic': '/livox/imu',
                    'preprocess.lidar_type': 1,
                    'preprocess.timestamp_unit': 3,
                    'mapping.extrinsic_est_en': False,
                    'pcd_save.pcd_save_en': True,
                    'pcd_save.interval': -1,
                },
            ],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='fastlio_rviz',
            output='screen',
            arguments=['-d', LaunchConfiguration('rviz_config')],
            parameters=[{
                'use_sim_time': ParameterValue(
                    LaunchConfiguration('use_sim_time'), value_type=bool),
            }],
            condition=IfCondition(LaunchConfiguration('use_rviz')),
        ),
    ])
