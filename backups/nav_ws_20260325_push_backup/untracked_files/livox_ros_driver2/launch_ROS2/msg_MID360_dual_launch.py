# 双 MID360 雷达启动：使用 MID360_dual_config.json（两台 MID360 合并到 /livox/lidar、/livox/imu）
# 与 start_robot.sh 兼容：FAST-LIO 和后续节点无需改话题
import os
from launch import LaunchDescription
from launch_ros.actions import Node

################### user configure parameters for ros2 start ###################
xfer_format   = 1
multi_topic   = 0    # 0=合并到同一话题，与单雷达一致
data_src      = 0
publish_freq  = 10.0
output_type   = 0
frame_id      = 'livox_frame'
lvx_file_path = '/home/livox/livox_test.lvx'
cmdline_bd_code = 'livox0000000001'

cur_path = os.path.split(os.path.realpath(__file__))[0] + '/'
cur_config_path = cur_path + '../config'
# 使用 swap 配置（114 第一项）确保两台都能连，与 MID360_dual_config_swap.json 内容一致
user_config_path = os.path.join(cur_config_path, 'MID360_dual_config_swap.json')
################### user configure parameters for ros2 end #####################

livox_ros2_params = [
    {"xfer_format": xfer_format},
    {"multi_topic": multi_topic},
    {"data_src": data_src},
    {"publish_freq": publish_freq},
    {"output_data_type": output_type},
    {"frame_id": frame_id},
    {"lvx_file_path": lvx_file_path},
    {"user_config_path": user_config_path},
    {"cmdline_input_bd_code": cmdline_bd_code}
]


def generate_launch_description():
    livox_driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=livox_ros2_params
    )
    return LaunchDescription([livox_driver])
