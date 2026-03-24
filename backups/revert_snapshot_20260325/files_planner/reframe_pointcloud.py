#!/usr/bin/env python3

import argparse

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2


def sensor_qos(depth: int) -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


class PointCloudFrameRewriter(Node):
    def __init__(self, input_topic: str, output_topic: str, output_frame: str, depth: int) -> None:
        super().__init__("pointcloud_frame_rewriter")
        self.output_frame = output_frame
        qos = sensor_qos(depth)
        self.pub = self.create_publisher(PointCloud2, output_topic, qos)
        self.sub = self.create_subscription(PointCloud2, input_topic, self.on_cloud, qos)
        self._count = 0
        self.get_logger().info(
            f"Reframing point cloud: {input_topic} -> {output_topic}, frame={output_frame}"
        )

    def on_cloud(self, msg: PointCloud2) -> None:
        out = PointCloud2()
        out.header = msg.header
        out.header.frame_id = self.output_frame
        out.height = msg.height
        out.width = msg.width
        out.fields = list(msg.fields)
        out.is_bigendian = msg.is_bigendian
        out.point_step = msg.point_step
        out.row_step = msg.row_step
        out.data = bytes(msg.data)
        out.is_dense = msg.is_dense
        self.pub.publish(out)
        self._count += 1
        if self._count == 1:
            self.get_logger().info(
                f"Published first reframed cloud with stamp={msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}"
            )


def main() -> int:
    parser = argparse.ArgumentParser(description="Rewrite PointCloud2 header.frame_id")
    parser.add_argument("--input-topic", required=True)
    parser.add_argument("--output-topic", required=True)
    parser.add_argument("--output-frame", required=True)
    parser.add_argument("--depth", type=int, default=20)
    args = parser.parse_args()

    rclpy.init()
    node = PointCloudFrameRewriter(
        input_topic=args.input_topic,
        output_topic=args.output_topic,
        output_frame=args.output_frame,
        depth=max(1, args.depth),
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
