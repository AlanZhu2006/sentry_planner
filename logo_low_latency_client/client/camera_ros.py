from __future__ import annotations

from dataclasses import dataclass
import logging
from threading import Event, Lock, Thread
from typing import Optional

from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import rclpy
from sensor_msgs.msg import CameraInfo, Image

from client.config import ClientConfig
from client.latest_frame_buffer import LatestFrameBuffer
from client.metrics import now_ms


LOGGER = logging.getLogger(__name__)


def _stamp_to_ns(message_stamp) -> int:
    return int(message_stamp.sec) * 1_000_000_000 + int(message_stamp.nanosec)


@dataclass(frozen=True)
class RawFrameBundle:
    frame_id: int
    capture_ts_ms: float
    rgb_msg_ts_ms: float
    depth_msg_ts_ms: float
    rgb_msg: Image
    depth_msg: Image
    rgb_info_msg: CameraInfo
    depth_info_msg: CameraInfo


class RosRgbdCaptureNode(Node):
    def __init__(self, config: ClientConfig, frame_buffer: LatestFrameBuffer[RawFrameBundle]) -> None:
        super().__init__("logo_low_latency_rgbd_client")
        self._config = config
        self._frame_buffer = frame_buffer
        self._lock = Lock()

        self._frame_id = 0
        self._last_pair_key: tuple[int, int] | None = None
        self._rgb_info: Optional[CameraInfo] = None
        self._depth_info: Optional[CameraInfo] = None
        self._rgb_msg: Optional[Image] = None
        self._depth_msg: Optional[Image] = None
        self._rgb_recv_ts_ms: float = 0.0
        self._depth_recv_ts_ms: float = 0.0

        self.create_subscription(Image, config.rgb_topic, self._on_rgb, qos_profile_sensor_data)
        self.create_subscription(Image, config.depth_topic, self._on_depth, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, config.rgb_info_topic, self._on_rgb_info, qos_profile_sensor_data)
        self.create_subscription(
            CameraInfo, config.depth_info_topic, self._on_depth_info, qos_profile_sensor_data
        )

        self._warned_waiting_for_info = False
        self._logged_first_pair = False
        LOGGER.info(
            "ROS capture node subscribed: rgb=%s depth=%s rgb_info=%s depth_info=%s",
            config.rgb_topic,
            config.depth_topic,
            config.rgb_info_topic,
            config.depth_info_topic,
        )

    def _on_rgb_info(self, msg: CameraInfo) -> None:
        with self._lock:
            self._rgb_info = msg
        LOGGER.debug("Received RGB camera_info width=%s height=%s", msg.width, msg.height)

    def _on_depth_info(self, msg: CameraInfo) -> None:
        with self._lock:
            self._depth_info = msg
        LOGGER.debug("Received depth camera_info width=%s height=%s", msg.width, msg.height)

    def _on_rgb(self, msg: Image) -> None:
        frame = None
        with self._lock:
            self._rgb_msg = msg
            self._rgb_recv_ts_ms = now_ms()
            frame = self._try_build_frame_locked()
        if frame is not None:
            self._frame_buffer.put(frame)

    def _on_depth(self, msg: Image) -> None:
        frame = None
        with self._lock:
            self._depth_msg = msg
            self._depth_recv_ts_ms = now_ms()
            frame = self._try_build_frame_locked()
        if frame is not None:
            self._frame_buffer.put(frame)

    def _try_build_frame_locked(self) -> Optional[RawFrameBundle]:
        if self._rgb_msg is None or self._depth_msg is None:
            return None
        if self._rgb_info is None or self._depth_info is None:
            if not self._warned_waiting_for_info:
                LOGGER.info("Waiting for both RGB and depth camera_info topics before sending reset.")
                self._warned_waiting_for_info = True
            return None

        rgb_stamp_ns = _stamp_to_ns(self._rgb_msg.header.stamp) or int(self._rgb_recv_ts_ms * 1_000_000.0)
        depth_stamp_ns = _stamp_to_ns(self._depth_msg.header.stamp) or int(
            self._depth_recv_ts_ms * 1_000_000.0
        )

        if abs(rgb_stamp_ns - depth_stamp_ns) > int(self._config.sync_tolerance_ms * 1_000_000.0):
            return None

        pair_key = (rgb_stamp_ns, depth_stamp_ns)
        if pair_key == self._last_pair_key:
            return None

        self._last_pair_key = pair_key
        self._frame_id += 1

        frame = RawFrameBundle(
            frame_id=self._frame_id,
            capture_ts_ms=max(self._rgb_recv_ts_ms, self._depth_recv_ts_ms),
            rgb_msg_ts_ms=rgb_stamp_ns / 1_000_000.0,
            depth_msg_ts_ms=depth_stamp_ns / 1_000_000.0,
            rgb_msg=self._rgb_msg,
            depth_msg=self._depth_msg,
            rgb_info_msg=self._rgb_info,
            depth_info_msg=self._depth_info,
        )

        if not self._logged_first_pair:
            self._logged_first_pair = True
            LOGGER.info(
                "First paired RGB-D frame captured: frame_id=%s rgb_stamp_ms=%.3f depth_stamp_ms=%.3f",
                frame.frame_id,
                frame.rgb_msg_ts_ms,
                frame.depth_msg_ts_ms,
            )
        return frame


class RosRgbdCapture:
    def __init__(self, config: ClientConfig, frame_buffer: LatestFrameBuffer[RawFrameBundle]) -> None:
        self._started = False
        self._stop_event = Event()

        if not rclpy.ok():
            rclpy.init(args=None)
        self._node = RosRgbdCaptureNode(config, frame_buffer)
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._thread = Thread(target=self._spin, name="ros_capture", daemon=True)

    def start(self) -> None:
        if self._started:
            return
        self._started = True
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        if self._executor is not None:
            self._executor.shutdown()
        if self._node is not None:
            self._node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if self._thread.is_alive():
            self._thread.join(timeout=2.0)

    def _spin(self) -> None:
        LOGGER.info("ROS capture thread started.")
        try:
            while not self._stop_event.is_set():
                self._executor.spin_once(timeout_sec=0.1)
        except ExternalShutdownException:
            LOGGER.debug("ROS executor shutdown requested.")
        LOGGER.info("ROS capture thread stopped.")
