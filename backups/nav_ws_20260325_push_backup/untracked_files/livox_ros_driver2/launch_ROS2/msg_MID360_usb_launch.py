# 仅 USB 网口单雷达启动（雷达 192.168.1.114）
import os
from launch import LaunchDescription
from launch_ros.actions import Node

cur_path = os.path.split(os.path.realpath(__file__))[0] + '/'
cur_config_path = cur_path + '../config'
user_config_path = os.path.join(cur_config_path, 'MID360_usb_config.json')

livox_ros2_params = [
    {"xfer_format": 1},
    {"multi_topic": 0},
    {"data_src": 0},
    {"publish_freq": 10.0},
    {"output_data_type": 0},
    {"frame_id": "livox_frame"},
    {"lvx_file_path": "/home/livox/livox_test.lvx"},
    {"user_config_path": user_config_path},
    {"cmdline_input_bd_code": "livox0000000001"}
]


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='livox_ros_driver2',
            executable='livox_ros_driver2_node',
            name='livox_lidar_publisher',
            output='screen',
            parameters=livox_ros2_params
        )
    ])
