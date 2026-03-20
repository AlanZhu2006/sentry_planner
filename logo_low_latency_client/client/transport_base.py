from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Any


class TransportClient(ABC):
    name: str

    @abstractmethod
    def reset(self, intrinsic: list[list[float]], stop_threshold: float, batch_size: int) -> dict[str, Any]:
        raise NotImplementedError

    @abstractmethod
    def step(
        self,
        *,
        frame_id: int,
        capture_ts_ms: float,
        send_ts_ms: float,
        intrinsic: list[list[float]],
        goal_x: float,
        goal_y: float,
        image_jpeg: bytes,
        depth_png: bytes,
    ) -> dict[str, Any]:
        raise NotImplementedError

    def close(self) -> None:
        return None

