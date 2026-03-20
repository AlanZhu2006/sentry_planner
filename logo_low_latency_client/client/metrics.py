from __future__ import annotations

from collections import deque
from dataclasses import asdict, dataclass
from statistics import mean
from typing import Any
import time


def now_ms() -> float:
    return time.time_ns() / 1_000_000.0


@dataclass
class FrameMetrics:
    capture_ts_ms: float
    encode_start_ts_ms: float
    send_start_ts_ms: float
    recv_ts_ms: float
    image_load_or_capture_ms: float
    resize_ms: float
    encode_ms: float
    request_total_ms: float
    roundtrip_ms: float
    end_to_end_ms: float

    def to_dict(self) -> dict[str, float]:
        return asdict(self)


class RollingStats:
    def __init__(self, window_size: int = 50) -> None:
        self._window_size = window_size
        self._values: dict[str, deque[float]] = {
            "end_to_end_ms": deque(maxlen=window_size),
            "request_total_ms": deque(maxlen=window_size),
            "encode_ms": deque(maxlen=window_size),
        }

    def add(self, metrics: FrameMetrics) -> None:
        for key in self._values:
            self._values[key].append(getattr(metrics, key))

    def average(self, key: str) -> float | None:
        values = self._values.get(key)
        if not values:
            return None
        return mean(values) if values else None


def extract_server_timing(response: dict[str, Any]) -> dict[str, float]:
    timing = response.get("timing")
    if not isinstance(timing, dict):
        return {}
    result: dict[str, float] = {}
    for key in ("server_parse_ms", "server_decode_ms", "server_inference_ms", "server_total_ms"):
        value = timing.get(key)
        if isinstance(value, (int, float)):
            result[key] = float(value)
    return result


def trajectory_point_count(response: dict[str, Any]) -> int:
    trajectory = response.get("trajectory")
    if isinstance(trajectory, list):
        return len(trajectory)
    return 0


def format_stats_line(
    frame_id: int,
    transport: str,
    metrics: FrameMetrics,
    response: dict[str, Any],
    rolling_stats: RollingStats,
) -> str:
    server_timing = extract_server_timing(response)
    trajectory_count = trajectory_point_count(response)
    avg_e2e = rolling_stats.average("end_to_end_ms")
    avg_request = rolling_stats.average("request_total_ms")

    parts = [
        f"frame={frame_id}",
        f"transport={transport}",
        f"end_to_end={metrics.end_to_end_ms:.1f}ms",
        f"request={metrics.request_total_ms:.1f}ms",
        f"encode={metrics.encode_ms:.1f}ms",
        f"traj_points={trajectory_count}",
    ]

    if avg_e2e is not None:
        parts.append(f"avg_end_to_end={avg_e2e:.1f}ms")
    if avg_request is not None:
        parts.append(f"avg_request={avg_request:.1f}ms")
    if "server_total_ms" in server_timing:
        parts.append(f"server_total={server_timing['server_total_ms']:.1f}ms")
    if "server_inference_ms" in server_timing:
        parts.append(f"server_infer={server_timing['server_inference_ms']:.1f}ms")
    return " | ".join(parts)

