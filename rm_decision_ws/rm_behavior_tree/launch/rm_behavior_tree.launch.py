import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    bt_config_dir = os.path.join(get_package_share_directory('rm_behavior_tree'), 'config')
    
    style = LaunchConfiguration('style', default='full')
    use_sim_time = LaunchConfiguration('use_sim_time', default='False')
    respawn = LaunchConfiguration('respawn', default='True')
    enable_groot = LaunchConfiguration('enable_groot', default='True')
    groot_port = LaunchConfiguration('groot_port', default='1667')

    bt_xml_dir = PathJoinSubstitution([bt_config_dir, style]), ".xml"

    declare_style = DeclareLaunchArgument('style', default_value='full')
    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='False')
    declare_respawn = DeclareLaunchArgument('respawn', default_value='True')
    declare_enable_groot = DeclareLaunchArgument('enable_groot', default_value='True')
    declare_groot_port = DeclareLaunchArgument('groot_port', default_value='1667')

    rm_behavior_tree_node = Node(
        package='rm_behavior_tree',
        executable='rm_behavior_tree',
        respawn=respawn,
        respawn_delay=3,
        parameters=[
            {
              'style': bt_xml_dir,
              'use_sim_time': use_sim_time,
              'enable_groot': enable_groot,
              'groot_port': groot_port,
            }
        ]
    )

    return LaunchDescription([
        declare_style,
        declare_use_sim_time,
        declare_respawn,
        declare_enable_groot,
        declare_groot_port,
        rm_behavior_tree_node,
    ])
