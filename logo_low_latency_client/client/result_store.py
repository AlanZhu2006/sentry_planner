from __future__ import annotations

import json
from pathlib import Path
from threading import Lock
from typing import Any


class ResultStore:
    def __init__(self, latest_json: Path, results_dir: Path, save_json: bool) -> None:
        self._latest_json = latest_json
        self._results_dir = results_dir
        self._save_json = save_json
        self._lock = Lock()
        self.latest_result: dict[str, Any] | None = None

    def save(self, result: dict[str, Any]) -> None:
        serializable = self._to_jsonable(result)
        with self._lock:
            self.latest_result = serializable
            self._write_json(self._latest_json, serializable)
            if self._save_json:
                frame_id = int(serializable.get("frame_id", 0))
                recv_ts_ms = int(float(serializable.get("recv_ts_ms", 0.0)))
                filename = self._results_dir / f"frame_{frame_id:06d}_{recv_ts_ms}.json"
                self._write_json(filename, serializable)

    def _write_json(self, path: Path, data: dict[str, Any]) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        tmp_path = path.with_suffix(path.suffix + ".tmp")
        with tmp_path.open("w", encoding="utf-8") as file_obj:
            json.dump(data, file_obj, ensure_ascii=False, indent=2, sort_keys=True)
        tmp_path.replace(path)

    def _to_jsonable(self, value: Any) -> Any:
        if isinstance(value, dict):
            return {str(key): self._to_jsonable(item) for key, item in value.items()}
        if isinstance(value, (list, tuple)):
            return [self._to_jsonable(item) for item in value]
        if isinstance(value, bytes):
            return {"bytes_len": len(value)}
        if isinstance(value, Path):
            return str(value)
        if hasattr(value, "item") and callable(getattr(value, "item")):
            try:
                return value.item()
            except Exception:
                pass
        return value

