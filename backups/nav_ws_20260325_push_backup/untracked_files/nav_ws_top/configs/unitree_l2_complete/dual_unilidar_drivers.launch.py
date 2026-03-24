"""
双 Unilidar 雷达驱动启动
- 主雷达：/dev/ttyACM0 -> unilidar/cloud, unilidar/imu（供 Point-LIO 使用）
- 副雷达：可配置串口或网络 -> unilidar2/cloud, unilidar2/imu
请根据实际硬件修改副雷达的 serial_port 或 lidar_ip/local_port。
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 副雷达串口（若用网络方式请改下面 Node 的 parameters）
    serial_port_2_arg = DeclareLaunchArgument(
        'serial_port_2',
        default_value='/dev/ttyUSB0',
        description='Second lidar serial port (e.g. /dev/ttyUSB0)',
    )

    # 主雷达（与现有单雷达配置一致，供 Point-LIO 使用）
    driver_main = Node(
        package='unitree_lidar_ros2',
        executable='unitree_lidar_ros2_node',
        name='unitree_lidar_ros2_node',
        output='screen',
        parameters=[{
            'initialize_type': 2,
            'work_mode': 0,
            'use_system_timestamp': True,
            'range_min': 0.5,
            'range_max': 100.0,
            'cloud_scan_num': 18,
            'serial_port': '/dev/ttyACM0',
            'baudrate': 4608000,
            'lidar_port': 6101,
            'lidar_ip': '192.168.1.62',
            'local_port': 6201,
            'local_ip': '192.168.1.2',
            'cloud_frame': 'unilidar_lidar',
            'cloud_topic': 'unilidar/cloud',
            'imu_frame': 'unilidar_imu',
            'imu_topic': 'unilidar/imu',
        }],
    )

    # 副雷达（不同 topic/frame，不同串口或网络）
    # 若副雷达走网络：改 serial_port 为从机无效，需用另一组 lidar_ip/local_port
    driver_second = Node(
        package='unitree_lidar_ros2',
        executable='unitree_lidar_ros2_node',
        name='unitree_lidar_ros2_node_2',
        output='screen',
        parameters=[{
            'initialize_type': 2,
            'work_mode': 0,
            'use_system_timestamp': True,
            'range_min': 0.5,
            'range_max': 100.0,
            'cloud_scan_num': 18,
            'serial_port': LaunchConfiguration('serial_port_2'),
            'baudrate': 4608000,
            # 若用网络：为副雷达设不同 IP/端口，例如：
            # 'lidar_port': 6102,
            # 'lidar_ip': '192.168.1.63',
            # 'local_port': 6202,
            # 'local_ip': '192.168.1.2',
            'lidar_port': 6101,
            'lidar_ip': '192.168.1.62',
            'local_port': 6201,
            'local_ip': '192.168.1.2',
            'cloud_frame': 'unilidar2_lidar',
            'cloud_topic': 'unilidar2/cloud',
            'imu_frame': 'unilidar2_imu',
            'imu_topic': 'unilidar2/imu',
        }],
    )

    # 副雷达相对 base_link 的静态 TF（按实际安装改 x y z yaw pitch roll）
    tf_base_to_lidar2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'unilidar2_lidar'],
        name='tf_base_unilidar2',
    )

    return LaunchDescription([
        serial_port_2_arg,
        driver_main,
        driver_second,
        tf_base_to_lidar2,
    ])
