from __future__ import annotations

from typing import Any

import msgpack
import requests
from requests.adapters import HTTPAdapter

from client.transport_base import TransportClient


class HttpFastTransport(TransportClient):
    name = "http_fast"

    def __init__(self, server_url: str, request_timeout: float) -> None:
        self._base_url = server_url.rstrip("/")
        self._timeout = request_timeout
        self._session = requests.Session()
        adapter = HTTPAdapter(pool_connections=1, pool_maxsize=1, max_retries=0)
        self._session.mount("http://", adapter)
        self._session.mount("https://", adapter)
        self._session.headers.update({"Connection": "keep-alive"})

    def reset(self, intrinsic: list[list[float]], stop_threshold: float, batch_size: int) -> dict[str, Any]:
        url = f"{self._base_url}/navigator_reset"
        payload = {
            "intrinsic": intrinsic,
            "stop_threshold": [float(stop_threshold)],
            "batch_size": int(batch_size),
        }
        response = self._session.post(url, json=payload, timeout=self._timeout)
        response.raise_for_status()
        data = self._decode_response(response)
        if isinstance(data, dict):
            data.setdefault("transport", self.name)
        return data

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
        del frame_id, capture_ts_ms, send_ts_ms, intrinsic
        payload = {
            "image_jpeg": image_jpeg,
            "depth_png": depth_png,
            "goal_x": [float(goal_x)],
            "goal_y": [float(goal_y)],
        }
        packed = msgpack.packb(payload, use_bin_type=True)
        response = self._session.post(
            f"{self._base_url}/pointgoal_step_fast",
            data=packed,
            headers={"Content-Type": "application/octet-stream"},
            timeout=self._timeout,
        )
        response.raise_for_status()
        data = self._decode_response(response)
        if isinstance(data, dict):
            data.setdefault("transport", self.name)
            return data
        return {"transport": self.name, "raw_response": data}

    def close(self) -> None:
        self._session.close()

    @staticmethod
    def _decode_response(response: requests.Response) -> Any:
        content_type = (response.headers.get("Content-Type") or "").lower()
        if "msgpack" in content_type or "octet-stream" in content_type:
            try:
                return msgpack.unpackb(response.content, raw=False)
            except Exception:
                pass
        try:
            return response.json()
        except ValueError:
            try:
                return msgpack.unpackb(response.content, raw=False)
            except Exception:
                return {"text": response.text}

