from __future__ import annotations

import json
import time
from typing import Any

import msgpack
import zmq

from client.transport_base import TransportClient


class ZmqTransport(TransportClient):
    """Two-socket PUSH/PULL transport reserved for a future bridge server."""

    name = "zmq"

    def __init__(self, push_endpoint: str, pull_endpoint: str, poll_timeout_ms: int) -> None:
        self._context = zmq.Context.instance()
        self._push = self._context.socket(zmq.PUSH)
        self._pull = self._context.socket(zmq.PULL)

        self._configure_socket(self._push, send_socket=True)
        self._configure_socket(self._pull, send_socket=False)

        self._push.connect(push_endpoint)
        self._pull.connect(pull_endpoint)

        self._poller = zmq.Poller()
        self._poller.register(self._pull, zmq.POLLIN)
        self._poll_timeout_ms = int(poll_timeout_ms)

    def reset(self, intrinsic: list[list[float]], stop_threshold: float, batch_size: int) -> dict[str, Any]:
        payload = {
            "type": "navigator_reset",
            "frame_id": 0,
            "send_ts_ms": 0.0,
            "intrinsic": intrinsic,
            "stop_threshold": [float(stop_threshold)],
            "batch_size": int(batch_size),
        }
        self._push.send(msgpack.packb(payload, use_bin_type=True))
        reply = self._recv_reply(expected_frame_id=0)
        reply.setdefault("transport", self.name)
        return reply

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
        payload = {
            "type": "pointgoal_step_fast",
            "frame_id": int(frame_id),
            "capture_ts_ms": float(capture_ts_ms),
            "send_ts_ms": float(send_ts_ms),
            "intrinsic": intrinsic,
            "goal_x": [float(goal_x)],
            "goal_y": [float(goal_y)],
            "image_jpeg": image_jpeg,
            "depth_png": depth_png,
        }
        self._push.send(msgpack.packb(payload, use_bin_type=True))
        reply = self._recv_reply(expected_frame_id=frame_id)
        reply.setdefault("transport", self.name)
        return reply

    def close(self) -> None:
        self._poller.unregister(self._pull)
        self._push.close(linger=0)
        self._pull.close(linger=0)

    def _recv_reply(self, expected_frame_id: int) -> dict[str, Any]:
        deadline = time.monotonic() + (self._poll_timeout_ms / 1000.0)
        while True:
            remaining_ms = int((deadline - time.monotonic()) * 1000.0)
            if remaining_ms <= 0:
                raise TimeoutError(f"ZMQ reply timeout for frame_id={expected_frame_id}")
            events = dict(self._poller.poll(max(1, remaining_ms)))
            if self._pull not in events:
                continue
            raw = self._pull.recv()
            reply = self._decode_reply(raw)
            frame_id = reply.get("frame_id")
            if isinstance(frame_id, int) and frame_id not in (0, expected_frame_id):
                if frame_id < expected_frame_id:
                    continue
            return reply

    @staticmethod
    def _configure_socket(socket: zmq.Socket, *, send_socket: bool) -> None:
        socket.setsockopt(zmq.LINGER, 0)
        socket.setsockopt(zmq.SNDHWM, 1)
        socket.setsockopt(zmq.RCVHWM, 1)
        if hasattr(zmq, "IMMEDIATE"):
            socket.setsockopt(zmq.IMMEDIATE, 1)
        if send_socket:
            socket.setsockopt(zmq.SNDTIMEO, 5000)
        else:
            socket.setsockopt(zmq.RCVTIMEO, 5000)

    @staticmethod
    def _decode_reply(raw: bytes) -> dict[str, Any]:
        try:
            data = msgpack.unpackb(raw, raw=False)
            if isinstance(data, dict):
                return data
            return {"raw_response": data}
        except Exception as exc:
            try:
                decoded = json.loads(raw.decode("utf-8"))
                if isinstance(decoded, dict):
                    return decoded
                return {"raw_response": decoded}
            except Exception:
                return {"decode_error": str(exc)}
