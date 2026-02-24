#!/usr/bin/env python3
"""
无雷达测试：发布假的 /scan, /odom, TF，让 Nav2 + 决策能跑起来。
使用前需有预存地图 (map + yaml)。

用法:
  python3 fake_sensors_for_test.py
  配合: Nav2 + 决策 + 模拟 game_status
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math


class FakeSensorsNode(Node):
    def __init__(self):
        super().__init__('fake_sensors')

        # 发布假 /scan（空或简单障碍，避免 Nav2 报错）
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.timer = self.create_timer(0.1, self.tick)  # 10 Hz
        self.t = 0.0

    def tick(self):
        self.t += 0.1

        # 假的 LaserScan：360 度，range 5m，无障碍
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'base_link'
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = math.pi / 180.0
        scan.time_increment = 0.0  # 同一次扫描内无时间差
        scan.scan_time = 0.1       # 10 Hz
        scan.range_min = 0.1
        scan.range_max = 10.0
        scan.ranges = [5.0] * 360  # 全是 5m，表示无障碍
        self.scan_pub.publish(scan)

        # 假的 Odometry
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = 0.0
        odom.pose.pose.position.y = 0.0
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.w = 1.0
        self.odom_pub.publish(odom)

        # TF: odom -> base_link
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)

        # TF: map -> odom (静态，机器人假设在原点)
        t2 = TransformStamped()
        t2.header.stamp = self.get_clock().now().to_msg()
        t2.header.frame_id = 'map'
        t2.child_frame_id = 'odom'
        t2.transform.translation.x = 0.0
        t2.transform.translation.y = 0.0
        t2.transform.translation.z = 0.0
        t2.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t2)


def main():
    rclpy.init()
    node = FakeSensorsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
