#!/usr/bin/env bash
set -euo pipefail

# Use one local rclpy node for the whole health check.  The ROS 2 CLI daemon on
# Humble can remain in a failed rclpy.ok() state after a command is interrupted,
# which made the previous sequence of `ros2 topic echo` calls report false
# failures even while every topic was healthy.
python3 - <<'PY'
import sys
import time

import rclpy
from livox_ros_driver2.msg import CustomMsg
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from sensor_msgs.msg import Imu, LaserScan, PointCloud2
from tf2_ros import Buffer, TransformListener


EXPECTED_TOPICS = (
    ('/livox/lidar', CustomMsg, 'MID-360 point cloud'),
    ('/livox/imu', Imu, 'MID-360 IMU'),
    ('/Odometry', Odometry, 'FAST-LIO body odometry'),
    ('/cloud_registered_body', PointCloud2,
     'FAST-LIO motion-compensated body cloud'),
    ('/scan', LaserScan, '2D laser scan'),
    ('/odom', Odometry, 'Nav2 FAST-LIO odometry'),
    ('/map', OccupancyGrid, 'Nav2 occupancy map'),
)
EXPECTED_TRANSFORMS = (
    ('base_link', 'livox_frame'),
    ('odom', 'base_link'),
    ('map', 'odom'),
)


class NavigationHealthCheck(Node):
    def __init__(self):
        super().__init__('navigation_health_check')
        self.received = set()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        for topic, msg_type, _ in EXPECTED_TOPICS:
            qos = map_qos if topic == '/map' else qos_profile_sensor_data
            self.create_subscription(
                msg_type,
                topic,
                lambda _msg, name=topic: self.received.add(name),
                qos,
            )

    def transform_ready(self, parent, child):
        return self.tf_buffer.can_transform(
            parent, child, Time(), timeout=Duration(seconds=0.0))


rclpy.init()
node = NavigationHealthCheck()
deadline = time.monotonic() + 12.0
graph_topics = set()

while time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.1)
    graph_topics = {name for name, _types in node.get_topic_names_and_types()}
    topics_ready = all(topic in node.received for topic, _, _ in EXPECTED_TOPICS)
    transforms_ready = all(
        node.transform_ready(parent, child)
        for parent, child in EXPECTED_TRANSFORMS
    )
    action_ready = '/navigate_to_pose/_action/status' in graph_topics
    if topics_ready and transforms_ready and action_ready:
        break

failed = False
for topic, _msg_type, label in EXPECTED_TOPICS:
    if topic in node.received:
        print(f'[OK]   {label} ({topic})')
    else:
        print(f'[FAIL] {label} ({topic})')
        failed = True

for parent, child in EXPECTED_TRANSFORMS:
    if node.transform_ready(parent, child):
        print(f'[OK]   TF {parent} -> {child}')
    else:
        print(f'[FAIL] TF {parent} -> {child}')
        failed = True

if '/navigate_to_pose/_action/status' in graph_topics:
    print('[OK]   Nav2 navigate_to_pose action')
else:
    print('[FAIL] Nav2 navigate_to_pose action')
    failed = True

node.destroy_node()
rclpy.shutdown()
sys.exit(1 if failed else 0)
PY
