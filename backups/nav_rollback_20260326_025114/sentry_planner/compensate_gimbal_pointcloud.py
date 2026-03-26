#!/usr/bin/env python3

import argparse
import array
import math
import threading

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState, PointCloud2
from sensor_msgs_py import point_cloud2


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def stamp_to_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class GimbalCloudCompensator(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("gimbal_cloud_compensator")
        self.cloud_topic = args.cloud_topic
        self.state_topic = args.state_topic
        self.output_topic = args.output_topic
        self.output_frame = args.output_frame
        self.state_source = args.state_source
        self.max_state_age = float(args.max_state_age)
        self.max_extrapolation = float(args.max_extrapolation)
        self.yaw_sign = float(args.yaw_sign)
        self.compensate_pitch = bool(args.compensate_pitch)
        self.pitch_sign = float(args.pitch_sign)
        self.pitch_offset_rad = float(args.pitch_offset_rad)
        self.status_period = float(args.status_period)
        self.constant_yaw_rate_rad_s = math.radians(float(args.constant_yaw_rate_deg_s))
        self.constant_initial_yaw_rad = math.radians(float(args.initial_yaw_deg))

        self._state_lock = threading.Lock()
        self._state_stamp_sec: float | None = None
        self._yaw_rad = 0.0
        self._pitch_rad = 0.0
        self._yaw_vel_rad_s = 0.0
        self._pitch_vel_rad_s = 0.0
        self._constant_ref_stamp_sec: float | None = None
        self._warned_missing_state = False
        self._processed_count = 0
        self._last_status_time = self.get_clock().now()

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.state_sub = None
        if self.state_source == "joint_state":
            self.state_sub = self.create_subscription(JointState, self.state_topic, self.on_state, sensor_qos)
        self.cloud_sub = self.create_subscription(PointCloud2, self.cloud_topic, self.on_cloud, sensor_qos)
        self.cloud_pub = self.create_publisher(PointCloud2, self.output_topic, pub_qos)

        if self.state_source == "joint_state":
            self.get_logger().info(
                f"gimbal cloud compensation enabled: {self.cloud_topic} -> {self.output_topic}, "
                f"state={self.state_topic}, out_frame={self.output_frame}, yaw_sign={self.yaw_sign:+.1f}, "
                f"pitch_offset={math.degrees(self.pitch_offset_rad):+.1f}deg"
            )
        else:
            self.get_logger().info(
                f"gimbal cloud compensation enabled: {self.cloud_topic} -> {self.output_topic}, "
                f"source=constant_rate({math.degrees(self.constant_yaw_rate_rad_s):+.1f}deg/s), "
                f"initial_yaw={math.degrees(self.constant_initial_yaw_rad):+.1f}deg, "
                f"out_frame={self.output_frame}, yaw_sign={self.yaw_sign:+.1f}, "
                f"pitch_offset={math.degrees(self.pitch_offset_rad):+.1f}deg"
            )

    def on_state(self, msg: JointState) -> None:
        yaw_idx = 0
        pitch_idx = 1
        if msg.name:
            try:
                yaw_idx = msg.name.index("gimbal_yaw")
                pitch_idx = msg.name.index("gimbal_pitch")
            except ValueError:
                return
        if len(msg.position) <= max(yaw_idx, pitch_idx):
            return

        yaw_vel = 0.0
        pitch_vel = 0.0
        if len(msg.velocity) > yaw_idx:
            yaw_vel = float(msg.velocity[yaw_idx])
        if len(msg.velocity) > pitch_idx:
            pitch_vel = float(msg.velocity[pitch_idx])

        with self._state_lock:
            self._state_stamp_sec = stamp_to_sec(msg.header.stamp)
            self._yaw_rad = float(msg.position[yaw_idx])
            self._pitch_rad = float(msg.position[pitch_idx])
            self._yaw_vel_rad_s = yaw_vel
            self._pitch_vel_rad_s = pitch_vel

    def estimate_angles(self, stamp_sec: float) -> tuple[float, float] | None:
        if self.state_source == "constant_rate":
            if self._constant_ref_stamp_sec is None:
                self._constant_ref_stamp_sec = stamp_sec
            dt = stamp_sec - self._constant_ref_stamp_sec
            yaw_hat = wrap_angle(self.constant_initial_yaw_rad + self.constant_yaw_rate_rad_s * dt)
            return yaw_hat, 0.0

        with self._state_lock:
            if self._state_stamp_sec is None:
                return None
            state_stamp_sec = self._state_stamp_sec
            yaw_rad = self._yaw_rad
            pitch_rad = self._pitch_rad
            yaw_vel_rad_s = self._yaw_vel_rad_s
            pitch_vel_rad_s = self._pitch_vel_rad_s

        age = stamp_sec - state_stamp_sec
        if abs(age) > self.max_state_age:
            return None

        dt = clamp(age, -self.max_extrapolation, self.max_extrapolation)
        yaw_hat = wrap_angle(yaw_rad + yaw_vel_rad_s * dt)
        pitch_hat = wrap_angle(pitch_rad + pitch_vel_rad_s * dt)
        return yaw_hat, pitch_hat

    def on_cloud(self, msg: PointCloud2) -> None:
        stamp_sec = stamp_to_sec(msg.header.stamp)
        angles = self.estimate_angles(stamp_sec)
        if angles is None:
            if not self._warned_missing_state:
                self.get_logger().warning(
                    "no fresh gimbal state yet; publishing uncompensated cloud until state arrives"
                )
                self._warned_missing_state = True
            yaw_hat = 0.0
            pitch_hat = 0.0
        else:
            self._warned_missing_state = False
            yaw_hat, pitch_hat = angles

        points = point_cloud2.read_points(msg)
        points_out = np.array(points, copy=True)
        if points_out.dtype.names is None or "x" not in points_out.dtype.names or "y" not in points_out.dtype.names:
            self.get_logger().error("point cloud does not contain x/y fields; skipping frame")
            return

        x = points_out["x"].astype(np.float32, copy=True)
        y = points_out["y"].astype(np.float32, copy=True)
        yaw_angle = self.yaw_sign * yaw_hat
        cos_yaw = math.cos(yaw_angle)
        sin_yaw = math.sin(yaw_angle)
        points_out["x"] = cos_yaw * x - sin_yaw * y
        points_out["y"] = sin_yaw * x + cos_yaw * y

        if self.compensate_pitch and "z" in points_out.dtype.names:
            x2 = points_out["x"].astype(np.float32, copy=True)
            z = points_out["z"].astype(np.float32, copy=True)
            pitch_angle = self.pitch_offset_rad + self.pitch_sign * pitch_hat
            cos_pitch = math.cos(pitch_angle)
            sin_pitch = math.sin(pitch_angle)
            points_out["x"] = cos_pitch * x2 + sin_pitch * z
            points_out["z"] = -sin_pitch * x2 + cos_pitch * z

        out_msg = PointCloud2()
        out_msg.header = msg.header
        out_msg.header.frame_id = self.output_frame
        out_msg.height = msg.height
        out_msg.width = msg.width
        out_msg.fields = msg.fields
        out_msg.is_bigendian = msg.is_bigendian
        out_msg.point_step = msg.point_step
        out_msg.row_step = msg.row_step
        out_msg.is_dense = msg.is_dense
        out_msg.data = array.array("B", points_out.tobytes())
        self.cloud_pub.publish(out_msg)

        self._processed_count += 1
        now = self.get_clock().now()
        if (now - self._last_status_time).nanoseconds >= int(self.status_period * 1e9):
            self._last_status_time = now
            self.get_logger().info(
                f"processed={self._processed_count} yaw={math.degrees(yaw_hat):+.1f}deg "
                f"pitch={math.degrees(pitch_hat):+.1f}deg"
            )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Compensate rotating gimbal yaw on FAST-LIO body cloud.")
    parser.add_argument("--cloud-topic", default="/cloud_registered_body")
    parser.add_argument("--state-topic", default="/gimbal_joint_states")
    parser.add_argument("--output-topic", default="/cloud_registered_body_compensated")
    parser.add_argument("--output-frame", default="base_link")
    parser.add_argument("--state-source", choices=["joint_state", "constant_rate"], default="joint_state")
    parser.add_argument("--max-state-age", type=float, default=0.5)
    parser.add_argument("--max-extrapolation", type=float, default=0.15)
    parser.add_argument("--yaw-sign", type=float, default=1.0)
    parser.add_argument("--constant-yaw-rate-deg-s", type=float, default=120.0)
    parser.add_argument("--initial-yaw-deg", type=float, default=0.0)
    parser.add_argument("--compensate-pitch", action="store_true")
    parser.add_argument("--pitch-sign", type=float, default=1.0)
    parser.add_argument("--pitch-offset-rad", type=float, default=0.0)
    parser.add_argument("--status-period", type=float, default=5.0)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    rclpy.init(args=None)
    node = GimbalCloudCompensator(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
