from __future__ import annotations

from dataclasses import dataclass
import sys

import cv2
import numpy as np
from sensor_msgs.msg import CameraInfo, Image

from client.camera_ros import RawFrameBundle
from client.config import ClientConfig
from client.metrics import now_ms


@dataclass(frozen=True)
class EncodedFrame:
    frame_id: int
    capture_ts_ms: float
    rgb_msg_ts_ms: float
    depth_msg_ts_ms: float
    encode_start_ts_ms: float
    intrinsic: list[list[float]]
    image_jpeg: bytes
    depth_png: bytes
    image_load_or_capture_ms: float
    resize_ms: float
    encode_ms: float
    rgb_source_shape: tuple[int, int, int]
    depth_source_shape: tuple[int, int]
    rgb_encoded_shape: tuple[int, int, int]
    depth_encoded_shape: tuple[int, int]


class FrameEncoder:
    def __init__(self, config: ClientConfig) -> None:
        self._config = config

    def encode(self, frame: RawFrameBundle) -> EncodedFrame:
        encode_start_ts_ms = now_ms()

        rgb_image = self._decode_color_image(frame.rgb_msg)
        depth_image = self._decode_depth_image(frame.depth_msg)
        image_load_or_capture_ms = now_ms() - encode_start_ts_ms

        resize_start_ms = now_ms()
        resized_rgb = cv2.resize(
            rgb_image,
            (self._config.rgb_width, self._config.rgb_height),
            interpolation=cv2.INTER_AREA,
        )
        resized_depth = cv2.resize(
            depth_image,
            (self._config.depth_width, self._config.depth_height),
            interpolation=cv2.INTER_NEAREST,
        )
        resize_ms = now_ms() - resize_start_ms

        intrinsic_info = frame.rgb_info_msg if self._config.intrinsic_source == "rgb" else frame.depth_info_msg
        intrinsic_source_shape = rgb_image.shape[:2] if self._config.intrinsic_source == "rgb" else depth_image.shape[:2]
        target_shape = (
            self._config.rgb_width,
            self._config.rgb_height,
        ) if self._config.intrinsic_source == "rgb" else (
            self._config.depth_width,
            self._config.depth_height,
        )
        intrinsic = self._scale_intrinsic(
            intrinsic_info,
            source_width=intrinsic_source_shape[1],
            source_height=intrinsic_source_shape[0],
            target_width=target_shape[0],
            target_height=target_shape[1],
        )

        encode_start = now_ms()
        image_jpeg = self._encode_jpeg(resized_rgb)
        depth_png = self._encode_depth_png(resized_depth)
        encode_ms = now_ms() - encode_start

        return EncodedFrame(
            frame_id=frame.frame_id,
            capture_ts_ms=frame.capture_ts_ms,
            rgb_msg_ts_ms=frame.rgb_msg_ts_ms,
            depth_msg_ts_ms=frame.depth_msg_ts_ms,
            encode_start_ts_ms=encode_start_ts_ms,
            intrinsic=intrinsic,
            image_jpeg=image_jpeg,
            depth_png=depth_png,
            image_load_or_capture_ms=image_load_or_capture_ms,
            resize_ms=resize_ms,
            encode_ms=encode_ms,
            rgb_source_shape=tuple(int(v) for v in rgb_image.shape),
            depth_source_shape=(int(depth_image.shape[0]), int(depth_image.shape[1])),
            rgb_encoded_shape=tuple(int(v) for v in resized_rgb.shape),
            depth_encoded_shape=(int(resized_depth.shape[0]), int(resized_depth.shape[1])),
        )

    def _decode_color_image(self, msg: Image) -> np.ndarray:
        encoding = msg.encoding.lower()
        if encoding in {"rgb8", "bgr8", "rgba8", "bgra8"}:
            channels = 4 if "a8" in encoding else 3
            image = self._reshape_image(msg, np.uint8, channels)
            if encoding == "rgb8":
                return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            if encoding == "rgba8":
                return cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)
            if encoding == "bgra8":
                return cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
            return image
        if encoding in {"mono8", "8uc1"}:
            gray = self._reshape_image(msg, np.uint8, 1)
            return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        raise ValueError(f"Unsupported RGB image encoding: {msg.encoding}")

    def _decode_depth_image(self, msg: Image) -> np.ndarray:
        encoding = msg.encoding.lower()
        if encoding in {"16uc1", "mono16", "16sc1"}:
            image = self._reshape_image(msg, np.uint16 if encoding != "16sc1" else np.int16, 1)
            image = np.asarray(image)
            if image.dtype == np.int16:
                image = np.clip(image, 0, np.iinfo(np.uint16).max).astype(np.uint16)
            return image
        if encoding in {"32fc1", "64fc1"}:
            dtype = np.float32 if encoding == "32fc1" else np.float64
            image = self._reshape_image(msg, dtype, 1)
            image = np.nan_to_num(image, nan=0.0, posinf=0.0, neginf=0.0)
            image = np.clip(image, 0.0, np.iinfo(np.uint16).max / max(self._config.depth_float_scale, 1.0))
            image = np.rint(image * self._config.depth_float_scale).astype(np.uint16)
            return image
        raise ValueError(
            f"Unsupported depth image encoding: {msg.encoding}. "
            "Expected 16UC1/mono16 or 32FC1/64FC1."
        )

    def _reshape_image(self, msg: Image, dtype, channels: int) -> np.ndarray:
        itemsize = np.dtype(dtype).itemsize
        step_items = int(msg.step) // itemsize
        try:
            flat = np.frombuffer(msg.data, dtype=dtype)
        except TypeError:
            flat = np.asarray(msg.data, dtype=dtype)

        required_items = int(msg.height) * step_items
        if flat.size < required_items:
            raise ValueError(
                f"Image buffer too small for encoding={msg.encoding}: "
                f"got={flat.size} required={required_items}"
            )

        flat = flat[:required_items]
        if channels == 1:
            image = flat.reshape(int(msg.height), step_items)[:, : int(msg.width)]
        else:
            row_width = int(msg.width) * channels
            image = flat.reshape(int(msg.height), step_items)[:, :row_width]
            image = image.reshape(int(msg.height), int(msg.width), channels)

        if itemsize > 1 and bool(msg.is_bigendian) != (sys.byteorder == "big"):
            image = image.byteswap().newbyteorder()
        return image

    def _scale_intrinsic(
        self,
        camera_info: CameraInfo,
        *,
        source_width: int,
        source_height: int,
        target_width: int,
        target_height: int,
    ) -> list[list[float]]:
        if len(camera_info.k) < 9:
            raise ValueError("camera_info.k must contain a 3x3 intrinsic matrix.")

        info_width = int(camera_info.width) or int(source_width)
        info_height = int(camera_info.height) or int(source_height)
        scale_x = float(target_width) / max(1.0, float(info_width))
        scale_y = float(target_height) / max(1.0, float(info_height))
        fx = float(camera_info.k[0]) * scale_x
        fy = float(camera_info.k[4]) * scale_y
        cx = float(camera_info.k[2]) * scale_x
        cy = float(camera_info.k[5]) * scale_y
        return [[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]]

    def _encode_jpeg(self, image: np.ndarray) -> bytes:
        ok, encoded = cv2.imencode(
            ".jpg", image, [int(cv2.IMWRITE_JPEG_QUALITY), int(self._config.jpeg_quality)]
        )
        if not ok:
            raise RuntimeError("Failed to JPEG-encode RGB image.")
        return encoded.tobytes()

    def _encode_depth_png(self, depth_image: np.ndarray) -> bytes:
        ok, encoded = cv2.imencode(
            ".png",
            depth_image,
            [int(cv2.IMWRITE_PNG_COMPRESSION), int(self._config.png_compression)],
        )
        if not ok:
            raise RuntimeError("Failed to PNG-encode depth image.")
        return encoded.tobytes()
