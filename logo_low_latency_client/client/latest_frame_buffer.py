from __future__ import annotations

from dataclasses import dataclass
from threading import Condition
from typing import Generic, Optional, TypeVar


T = TypeVar("T")


@dataclass(frozen=True)
class VersionedFrame(Generic[T]):
    version: int
    frame: T


class LatestFrameBuffer(Generic[T]):
    """Single-slot latest-frame-only buffer."""

    def __init__(self) -> None:
        self._condition = Condition()
        self._version = 0
        self._frame: Optional[T] = None

    def put(self, frame: T) -> int:
        with self._condition:
            self._version += 1
            self._frame = frame
            self._condition.notify_all()
            return self._version

    def get_latest(self) -> Optional[VersionedFrame[T]]:
        with self._condition:
            if self._frame is None:
                return None
            return VersionedFrame(version=self._version, frame=self._frame)

    def wait_for_new(self, last_version: int, timeout: float | None = None) -> Optional[VersionedFrame[T]]:
        with self._condition:
            self._condition.wait_for(lambda: self._version > last_version, timeout=timeout)
            if self._version <= last_version or self._frame is None:
                return None
            return VersionedFrame(version=self._version, frame=self._frame)

