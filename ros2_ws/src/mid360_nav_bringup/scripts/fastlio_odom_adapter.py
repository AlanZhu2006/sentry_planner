#!/usr/bin/env python3
"""Expose FAST-LIO body odometry as the Nav2 odom -> base_link chain."""

import math
import time

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


def normalize_quaternion(q: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    norm = math.sqrt(sum(value * value for value in q))
    if norm <= 1.0e-12:
        raise ValueError("zero-length quaternion")
    return tuple(value / norm for value in q)


def quaternion_multiply(
    first: tuple[float, float, float, float],
    second: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    ax, ay, az, aw = first
    bx, by, bz, bw = second
    return normalize_quaternion((
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ))


def quaternion_conjugate(
    q: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    return (-q[0], -q[1], -q[2], q[3])


def quaternion_from_rpy(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    cr = math.cos(0.5 * roll)
    sr = math.sin(0.5 * roll)
    cp = math.cos(0.5 * pitch)
    sp = math.sin(0.5 * pitch)
    cy = math.cos(0.5 * yaw)
    sy = math.sin(0.5 * yaw)
    return normalize_quaternion((
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ))


def rotate_vector(
    q: tuple[float, float, float, float],
    vector: tuple[float, float, float],
) -> tuple[float, float, float]:
    x, y, z, w = q
    vx, vy, vz = vector
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + y * tz - z * ty,
        vy + w * ty + z * tx - x * tz,
        vz + w * tz + x * ty - y * tx,
    )


def compose_transform(
    first_translation: tuple[float, float, float],
    first_rotation: tuple[float, float, float, float],
    second_translation: tuple[float, float, float],
    second_rotation: tuple[float, float, float, float],
) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
    rotated = rotate_vector(first_rotation, second_translation)
    translation = tuple(a + b for a, b in zip(first_translation, rotated))
    return translation, quaternion_multiply(first_rotation, second_rotation)


def invert_transform(
    translation: tuple[float, float, float],
    rotation: tuple[float, float, float, float],
) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
    inverse_rotation = quaternion_conjugate(normalize_quaternion(rotation))
    inverse_translation = rotate_vector(
        inverse_rotation, tuple(-value for value in translation)
    )
    return inverse_translation, inverse_rotation


def quaternion_yaw(q: tuple[float, float, float, float]) -> float:
    x, y, z, w = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class FastlioOdomAdapter(Node):
    """Convert FAST-LIO's camera_init -> body pose into odom -> base_link."""

    def __init__(self) -> None:
        super().__init__("fastlio_odom_adapter")
        self.declare_parameter("source_odom_topic", "/Odometry")
        self.declare_parameter("output_odom_topic", "/odom")
        self.declare_parameter("source_world_frame", "camera_init")
        self.declare_parameter("source_body_frame", "body")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("base_to_lidar_x", 0.0)
        self.declare_parameter("base_to_lidar_y", -0.11)
        self.declare_parameter("base_to_lidar_z", 0.35)
        self.declare_parameter("base_to_lidar_roll", 0.0)
        self.declare_parameter("base_to_lidar_pitch", 0.0)
        self.declare_parameter("base_to_lidar_yaw", 0.0)
        # FAST-LIO mid360.yaml defines the LiDAR pose in the IMU body frame.
        self.declare_parameter("body_to_lidar_x", -0.011)
        self.declare_parameter("body_to_lidar_y", -0.02329)
        self.declare_parameter("body_to_lidar_z", 0.04412)
        self.declare_parameter("body_to_lidar_roll", 0.0)
        self.declare_parameter("body_to_lidar_pitch", 0.0)
        self.declare_parameter("body_to_lidar_yaw", 0.0)
        self.declare_parameter("startup_timeout_s", 20.0)
        self.declare_parameter("stale_timeout_s", 2.0)
        self.declare_parameter("velocity_filter_alpha", 0.35)

        self._source_world_frame = str(self.get_parameter("source_world_frame").value)
        self._source_body_frame = str(self.get_parameter("source_body_frame").value)
        self._odom_frame = str(self.get_parameter("odom_frame").value)
        self._base_frame = str(self.get_parameter("base_frame").value)

        base_to_lidar_translation = self._translation_parameter("base_to_lidar")
        base_to_lidar_rotation = self._rotation_parameter("base_to_lidar")
        body_to_lidar_translation = self._translation_parameter("body_to_lidar")
        body_to_lidar_rotation = self._rotation_parameter("body_to_lidar")
        lidar_to_body = invert_transform(body_to_lidar_translation, body_to_lidar_rotation)
        self._base_to_body = compose_transform(
            base_to_lidar_translation,
            base_to_lidar_rotation,
            lidar_to_body[0],
            lidar_to_body[1],
        )
        self._body_to_base = invert_transform(*self._base_to_body)

        output_topic = str(self.get_parameter("output_odom_topic").value)
        source_topic = str(self.get_parameter("source_odom_topic").value)
        self._publisher = self.create_publisher(Odometry, output_topic, 20)
        self._tf_broadcaster = TransformBroadcaster(self)
        self._static_broadcaster = StaticTransformBroadcaster(self)
        self._publish_base_to_body_static_transform()
        self._subscription = self.create_subscription(
            Odometry, source_topic, self._odom_callback, 20
        )

        self._started_at = time.monotonic()
        self._last_source_at: float | None = None
        self._previous_stamp: float | None = None
        self._previous_translation: tuple[float, float, float] | None = None
        self._previous_rotation: tuple[float, float, float, float] | None = None
        self._filtered_twist = [0.0, 0.0, 0.0]
        self.create_timer(0.25, self._check_source)

        translation = self._base_to_body[0]
        self.get_logger().info(
            "FAST-LIO is the authoritative Nav2 pose/TF source: "
            f"{source_topic} ({self._source_world_frame}->{self._source_body_frame}) "
            f"-> {output_topic} ({self._odom_frame}->{self._base_frame}); "
            "wheel odometry is diagnostics only until dynamically validated; "
            f"base->body xyz=({translation[0]:.5f}, {translation[1]:.5f}, "
            f"{translation[2]:.5f})"
        )

    def _translation_parameter(self, prefix: str) -> tuple[float, float, float]:
        return tuple(
            float(self.get_parameter(f"{prefix}_{axis}").value)
            for axis in ("x", "y", "z")
        )

    def _rotation_parameter(self, prefix: str) -> tuple[float, float, float, float]:
        return quaternion_from_rpy(*(
            float(self.get_parameter(f"{prefix}_{axis}").value)
            for axis in ("roll", "pitch", "yaw")
        ))

    def _publish_base_to_body_static_transform(self) -> None:
        translation, rotation = self._base_to_body
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self._base_frame
        transform.child_frame_id = self._source_body_frame
        transform.transform.translation.x = translation[0]
        transform.transform.translation.y = translation[1]
        transform.transform.translation.z = translation[2]
        transform.transform.rotation.x = rotation[0]
        transform.transform.rotation.y = rotation[1]
        transform.transform.rotation.z = rotation[2]
        transform.transform.rotation.w = rotation[3]
        self._static_broadcaster.sendTransform(transform)

    def _odom_callback(self, source: Odometry) -> None:
        if (
            source.header.frame_id != self._source_world_frame
            or source.child_frame_id != self._source_body_frame
        ):
            raise RuntimeError(
                "Unexpected FAST-LIO frames: "
                f"{source.header.frame_id}->{source.child_frame_id}; expected "
                f"{self._source_world_frame}->{self._source_body_frame}"
            )

        source_translation = (
            source.pose.pose.position.x,
            source.pose.pose.position.y,
            source.pose.pose.position.z,
        )
        source_rotation = normalize_quaternion((
            source.pose.pose.orientation.x,
            source.pose.pose.orientation.y,
            source.pose.pose.orientation.z,
            source.pose.pose.orientation.w,
        ))
        base_translation, base_rotation = compose_transform(
            source_translation,
            source_rotation,
            self._body_to_base[0],
            self._body_to_base[1],
        )

        output = Odometry()
        output.header.stamp = source.header.stamp
        output.header.frame_id = self._odom_frame
        output.child_frame_id = self._base_frame
        output.pose.pose.position.x = base_translation[0]
        output.pose.pose.position.y = base_translation[1]
        output.pose.pose.position.z = base_translation[2]
        output.pose.pose.orientation.x = base_rotation[0]
        output.pose.pose.orientation.y = base_rotation[1]
        output.pose.pose.orientation.z = base_rotation[2]
        output.pose.pose.orientation.w = base_rotation[3]
        output.pose.covariance = source.pose.covariance

        stamp = source.header.stamp.sec + source.header.stamp.nanosec * 1.0e-9
        self._update_velocity(stamp, base_translation, base_rotation)
        output.twist.twist.linear.x = self._filtered_twist[0]
        output.twist.twist.linear.y = self._filtered_twist[1]
        output.twist.twist.angular.z = self._filtered_twist[2]
        output.twist.covariance[0] = 0.04
        output.twist.covariance[7] = 0.04
        output.twist.covariance[35] = 0.03

        self._publisher.publish(output)
        transform = TransformStamped()
        transform.header = output.header
        transform.child_frame_id = self._base_frame
        transform.transform.translation.x = base_translation[0]
        transform.transform.translation.y = base_translation[1]
        transform.transform.translation.z = base_translation[2]
        transform.transform.rotation = output.pose.pose.orientation
        self._tf_broadcaster.sendTransform(transform)
        self._last_source_at = time.monotonic()

    def _update_velocity(
        self,
        stamp: float,
        translation: tuple[float, float, float],
        rotation: tuple[float, float, float, float],
    ) -> None:
        if (
            self._previous_stamp is not None
            and self._previous_translation is not None
            and self._previous_rotation is not None
        ):
            dt = stamp - self._previous_stamp
            if 0.02 <= dt <= 0.5:
                world_velocity = tuple(
                    (current - previous) / dt
                    for current, previous in zip(translation, self._previous_translation)
                )
                body_velocity = rotate_vector(quaternion_conjugate(rotation), world_velocity)
                angular_z = wrap_angle(
                    quaternion_yaw(rotation) - quaternion_yaw(self._previous_rotation)
                ) / dt
                alpha = float(self.get_parameter("velocity_filter_alpha").value)
                alpha = max(0.0, min(1.0, alpha))
                measurements = (body_velocity[0], body_velocity[1], angular_z)
                self._filtered_twist = [
                    alpha * measurement + (1.0 - alpha) * previous
                    for measurement, previous in zip(measurements, self._filtered_twist)
                ]
        self._previous_stamp = stamp
        self._previous_translation = translation
        self._previous_rotation = rotation

    def _check_source(self) -> None:
        now = time.monotonic()
        if self._last_source_at is None:
            timeout = float(self.get_parameter("startup_timeout_s").value)
            if now - self._started_at > timeout:
                raise RuntimeError(
                    f"No FAST-LIO odometry received within {timeout:.1f} seconds"
                )
            return
        timeout = float(self.get_parameter("stale_timeout_s").value)
        if now - self._last_source_at > timeout:
            raise RuntimeError(
                f"FAST-LIO odometry has been stale for more than {timeout:.1f} seconds"
            )


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node: FastlioOdomAdapter | None = None
    try:
        node = FastlioOdomAdapter()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as error:
        if node is not None:
            node.get_logger().fatal(str(error))
        raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
