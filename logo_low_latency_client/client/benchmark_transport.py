from __future__ import annotations

import argparse
from dataclasses import dataclass
import logging
from pathlib import Path
import signal
from statistics import mean
import time
from typing import Any

import cv2
import numpy as np

from client.metrics import FrameMetrics, RollingStats, extract_server_timing, format_stats_line, now_ms
from client.result_store import ResultStore
from client.transport_base import TransportClient
from client.transport_http_fast import HttpFastTransport
from client.transport_zmq import ZmqTransport


LOGGER = logging.getLogger(__name__)


@dataclass(frozen=True)
class BenchmarkConfig:
    server_url: str
    transport: str
    goal_x: float
    goal_y: float
    rgb_width: int
    rgb_height: int
    depth_width: int
    depth_height: int
    jpeg_quality: int
    png_compression: int
    request_timeout: float
    stop_threshold: float
    batch_size: int
    iterations: int
    warmup: int
    send_fps: float
    show_stats: bool
    save_json: bool
    latest_json: Path
    results_dir: Path
    log_level: str
    zmq_push_endpoint: str
    zmq_pull_endpoint: str
    zmq_poll_timeout_ms: int
    reencode_each_frame: bool
    intrinsic_fx: float
    intrinsic_fy: float
    intrinsic_cx: float
    intrinsic_cy: float
    goal_sleep_s: float


@dataclass(frozen=True)
class PreparedPayload:
    intrinsic: list[list[float]]
    image_jpeg: bytes
    depth_png: bytes
    rgb_shape: tuple[int, int, int]
    depth_shape: tuple[int, int]
    encode_ms: float


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Benchmark the LoGoPlanner transport path without ROS or a physical depth camera."
    )
    parser.add_argument("--server-url", default="http://10.224.36.118:19999")
    parser.add_argument("--transport", choices=("http_fast", "zmq"), default="http_fast")
    parser.add_argument("--goal-x", type=float, default=3.0)
    parser.add_argument("--goal-y", type=float, default=0.5)
    parser.add_argument("--rgb-width", type=int, default=320)
    parser.add_argument("--rgb-height", type=int, default=240)
    parser.add_argument("--depth-width", type=int, default=320)
    parser.add_argument("--depth-height", type=int, default=240)
    parser.add_argument("--jpeg-quality", type=int, default=60)
    parser.add_argument("--png-compression", type=int, default=1)
    parser.add_argument("--request-timeout", type=float, default=5.0)
    parser.add_argument("--stop-threshold", type=float, default=-1.0)
    parser.add_argument("--batch-size", type=int, default=1)
    parser.add_argument("--iterations", type=int, default=20)
    parser.add_argument("--warmup", type=int, default=3)
    parser.add_argument("--send-fps", type=float, default=5.0)
    parser.add_argument("--show-stats", action="store_true")
    parser.add_argument("--save-json", action="store_true")
    parser.add_argument("--latest-json", type=Path, default=Path("results/benchmark_latest_result.json"))
    parser.add_argument("--results-dir", type=Path, default=Path("results"))
    parser.add_argument("--log-level", default="INFO")
    parser.add_argument("--zmq-push-endpoint", default="tcp://127.0.0.1:5555")
    parser.add_argument("--zmq-pull-endpoint", default="tcp://127.0.0.1:5556")
    parser.add_argument("--zmq-poll-timeout-ms", type=int, default=5000)
    parser.add_argument(
        "--reencode-each-frame",
        action="store_true",
        help="Recreate JPEG and PNG every frame so encode_ms is included in end-to-end timing.",
    )
    parser.add_argument("--intrinsic-fx", type=float, default=320.0)
    parser.add_argument("--intrinsic-fy", type=float, default=320.0)
    parser.add_argument("--intrinsic-cx", type=float, default=160.0)
    parser.add_argument("--intrinsic-cy", type=float, default=120.0)
    return parser


def parse_args(argv: list[str] | None = None) -> BenchmarkConfig:
    args = build_parser().parse_args(argv)
    send_fps = max(0.0, float(args.send_fps))
    return BenchmarkConfig(
        server_url=args.server_url.rstrip("/"),
        transport=args.transport,
        goal_x=float(args.goal_x),
        goal_y=float(args.goal_y),
        rgb_width=max(1, int(args.rgb_width)),
        rgb_height=max(1, int(args.rgb_height)),
        depth_width=max(1, int(args.depth_width)),
        depth_height=max(1, int(args.depth_height)),
        jpeg_quality=max(1, min(100, int(args.jpeg_quality))),
        png_compression=max(0, min(9, int(args.png_compression))),
        request_timeout=max(0.1, float(args.request_timeout)),
        stop_threshold=float(args.stop_threshold),
        batch_size=max(1, int(args.batch_size)),
        iterations=max(1, int(args.iterations)),
        warmup=max(0, int(args.warmup)),
        send_fps=send_fps,
        show_stats=bool(args.show_stats),
        save_json=bool(args.save_json),
        latest_json=args.latest_json,
        results_dir=args.results_dir,
        log_level=args.log_level.upper(),
        zmq_push_endpoint=args.zmq_push_endpoint,
        zmq_pull_endpoint=args.zmq_pull_endpoint,
        zmq_poll_timeout_ms=max(1, int(args.zmq_poll_timeout_ms)),
        reencode_each_frame=bool(args.reencode_each_frame),
        intrinsic_fx=float(args.intrinsic_fx),
        intrinsic_fy=float(args.intrinsic_fy),
        intrinsic_cx=float(args.intrinsic_cx),
        intrinsic_cy=float(args.intrinsic_cy),
        goal_sleep_s=0.0 if send_fps <= 0.0 else (1.0 / send_fps),
    )


def configure_logging(log_level: str) -> None:
    logging.basicConfig(
        level=getattr(logging, log_level.upper(), logging.INFO),
        format="%(asctime)s | %(levelname)s | %(name)s | %(message)s",
    )


def build_transport(config: BenchmarkConfig) -> TransportClient:
    if config.transport == "http_fast":
        return HttpFastTransport(config.server_url, config.request_timeout)
    return ZmqTransport(
        push_endpoint=config.zmq_push_endpoint,
        pull_endpoint=config.zmq_pull_endpoint,
        poll_timeout_ms=config.zmq_poll_timeout_ms,
    )


def build_intrinsic(config: BenchmarkConfig) -> list[list[float]]:
    return [
        [float(config.intrinsic_fx), 0.0, float(config.intrinsic_cx)],
        [0.0, float(config.intrinsic_fy), float(config.intrinsic_cy)],
        [0.0, 0.0, 1.0],
    ]


def generate_rgb_image(config: BenchmarkConfig, frame_id: int) -> np.ndarray:
    width = config.rgb_width
    height = config.rgb_height
    x = np.linspace(0, 255, width, dtype=np.uint8)
    y = np.linspace(0, 255, height, dtype=np.uint8)
    xv = np.tile(x, (height, 1))
    yv = np.tile(y[:, None], (1, width))
    phase = np.uint8(frame_id % 255)
    image = np.stack(
        [
            (xv + phase).astype(np.uint8),
            (yv + (phase * 2)).astype(np.uint8),
            ((xv // 2 + yv // 2 + phase * 3) % 255).astype(np.uint8),
        ],
        axis=2,
    )
    return image


def generate_depth_image(config: BenchmarkConfig, frame_id: int) -> np.ndarray:
    width = config.depth_width
    height = config.depth_height
    x = np.linspace(600, 2200, width, dtype=np.float32)
    y = np.linspace(0, 400, height, dtype=np.float32)
    depth = np.tile(x, (height, 1)) + np.tile(y[:, None], (1, width))
    depth += float(frame_id % 50)
    return np.clip(depth, 0, np.iinfo(np.uint16).max).astype(np.uint16)


def encode_payload(config: BenchmarkConfig, frame_id: int) -> PreparedPayload:
    encode_start_ms = now_ms()
    rgb_image = generate_rgb_image(config, frame_id)
    depth_image = generate_depth_image(config, frame_id)

    ok_rgb, encoded_rgb = cv2.imencode(
        ".jpg",
        rgb_image,
        [int(cv2.IMWRITE_JPEG_QUALITY), int(config.jpeg_quality)],
    )
    if not ok_rgb:
        raise RuntimeError("Failed to encode synthetic RGB JPEG.")

    ok_depth, encoded_depth = cv2.imencode(
        ".png",
        depth_image,
        [int(cv2.IMWRITE_PNG_COMPRESSION), int(config.png_compression)],
    )
    if not ok_depth:
        raise RuntimeError("Failed to encode synthetic depth PNG.")

    return PreparedPayload(
        intrinsic=build_intrinsic(config),
        image_jpeg=encoded_rgb.tobytes(),
        depth_png=encoded_depth.tobytes(),
        rgb_shape=tuple(int(v) for v in rgb_image.shape),
        depth_shape=(int(depth_image.shape[0]), int(depth_image.shape[1])),
        encode_ms=now_ms() - encode_start_ms,
    )


def percentile(values: list[float], fraction: float) -> float | None:
    if not values:
        return None
    sorted_values = sorted(values)
    index = int(round((len(sorted_values) - 1) * fraction))
    index = max(0, min(len(sorted_values) - 1, index))
    return float(sorted_values[index])


def build_result(
    *,
    config: BenchmarkConfig,
    payload: PreparedPayload,
    frame_id: int,
    metrics: FrameMetrics,
    response: dict[str, Any],
) -> dict[str, Any]:
    result: dict[str, Any] = dict(response)
    result["frame_id"] = frame_id
    result["transport"] = str(result.get("transport", config.transport))
    result["capture_ts_ms"] = metrics.capture_ts_ms
    result["encode_start_ts_ms"] = metrics.encode_start_ts_ms
    result["send_start_ts_ms"] = metrics.send_start_ts_ms
    result["recv_ts_ms"] = metrics.recv_ts_ms
    result["image_load_or_capture_ms"] = metrics.image_load_or_capture_ms
    result["resize_ms"] = metrics.resize_ms
    result["encode_ms"] = metrics.encode_ms
    result["request_total_ms"] = metrics.request_total_ms
    result["roundtrip_ms"] = metrics.roundtrip_ms
    result["end_to_end_ms"] = metrics.end_to_end_ms
    result["goal_x"] = [float(config.goal_x)]
    result["goal_y"] = [float(config.goal_y)]
    result["intrinsic"] = payload.intrinsic
    result["rgb_encoded_shape"] = list(payload.rgb_shape)
    result["depth_encoded_shape"] = list(payload.depth_shape)
    result["image_jpeg_bytes"] = len(payload.image_jpeg)
    result["depth_png_bytes"] = len(payload.depth_png)
    result["benchmark_mode"] = True
    return result


def main(argv: list[str] | None = None) -> None:
    config = parse_args(argv)
    configure_logging(config.log_level)

    stop_event = False

    def _handle_signal(signum, _frame) -> None:
        nonlocal stop_event
        LOGGER.info("Received signal %s, shutting down benchmark.", signum)
        stop_event = True

    signal.signal(signal.SIGINT, _handle_signal)
    signal.signal(signal.SIGTERM, _handle_signal)

    transport = build_transport(config)
    result_store = ResultStore(config.latest_json, config.results_dir, config.save_json)
    rolling_stats = RollingStats(window_size=50)

    measured_metrics: list[FrameMetrics] = []
    server_totals: list[float] = []
    payload = encode_payload(config, frame_id=0)

    LOGGER.info(
        "Benchmark started: transport=%s iterations=%s warmup=%s send_fps=%.2f reencode_each_frame=%s "
        "jpeg_bytes=%s depth_bytes=%s",
        config.transport,
        config.iterations,
        config.warmup,
        config.send_fps,
        config.reencode_each_frame,
        len(payload.image_jpeg),
        len(payload.depth_png),
    )

    try:
        reset_response = transport.reset(payload.intrinsic, config.stop_threshold, config.batch_size)
        LOGGER.info(
            "navigator_reset ok | transport=%s | reply_keys=%s",
            transport.name,
            sorted(reset_response.keys()) if isinstance(reset_response, dict) else "n/a",
        )

        total_frames = config.warmup + config.iterations
        for frame_id in range(1, total_frames + 1):
            if stop_event:
                break

            capture_ts_ms = now_ms()
            if config.reencode_each_frame:
                encode_start_ts_ms = now_ms()
                payload = encode_payload(config, frame_id=frame_id)
                encode_ms = payload.encode_ms
                image_load_or_capture_ms = encode_ms
            else:
                encode_start_ts_ms = capture_ts_ms
                encode_ms = 0.0
                image_load_or_capture_ms = 0.0

            send_start_ts_ms = now_ms()
            response = transport.step(
                frame_id=frame_id,
                capture_ts_ms=capture_ts_ms,
                send_ts_ms=send_start_ts_ms,
                intrinsic=payload.intrinsic,
                goal_x=config.goal_x,
                goal_y=config.goal_y,
                image_jpeg=payload.image_jpeg,
                depth_png=payload.depth_png,
            )
            recv_ts_ms = now_ms()

            metrics = FrameMetrics(
                capture_ts_ms=capture_ts_ms,
                encode_start_ts_ms=encode_start_ts_ms,
                send_start_ts_ms=send_start_ts_ms,
                recv_ts_ms=recv_ts_ms,
                image_load_or_capture_ms=image_load_or_capture_ms,
                resize_ms=0.0,
                encode_ms=encode_ms,
                request_total_ms=recv_ts_ms - send_start_ts_ms,
                roundtrip_ms=recv_ts_ms - send_start_ts_ms,
                end_to_end_ms=recv_ts_ms - capture_ts_ms,
            )

            result = build_result(
                config=config,
                payload=payload,
                frame_id=frame_id,
                metrics=metrics,
                response=response,
            )
            result_store.save(result)

            if frame_id > config.warmup:
                measured_metrics.append(metrics)
                rolling_stats.add(metrics)
                server_timing = extract_server_timing(result)
                if "server_total_ms" in server_timing:
                    server_totals.append(server_timing["server_total_ms"])

            phase = "warmup" if frame_id <= config.warmup else "measure"
            if config.show_stats:
                LOGGER.info("%s | %s", phase, format_stats_line(frame_id, transport.name, metrics, result, rolling_stats))
            else:
                LOGGER.info(
                    "%s | frame=%s | transport=%s | request=%.1fms | end_to_end=%.1fms",
                    phase,
                    frame_id,
                    transport.name,
                    metrics.request_total_ms,
                    metrics.end_to_end_ms,
                )

            if config.goal_sleep_s > 0.0 and frame_id < total_frames:
                time.sleep(config.goal_sleep_s)

        request_values = [item.request_total_ms for item in measured_metrics]
        end_to_end_values = [item.end_to_end_ms for item in measured_metrics]
        encode_values = [item.encode_ms for item in measured_metrics]

        if request_values:
            LOGGER.info(
                "Summary | frames=%s | request_avg=%.1fms | request_p50=%.1fms | request_p95=%.1fms | "
                "end_to_end_avg=%.1fms | end_to_end_p95=%.1fms | encode_avg=%.1fms%s",
                len(request_values),
                mean(request_values),
                percentile(request_values, 0.50) or 0.0,
                percentile(request_values, 0.95) or 0.0,
                mean(end_to_end_values),
                percentile(end_to_end_values, 0.95) or 0.0,
                mean(encode_values),
                f" | server_total_avg={mean(server_totals):.1f}ms" if server_totals else "",
            )
        else:
            LOGGER.warning("Benchmark finished before any measured frames were collected.")
    finally:
        transport.close()


if __name__ == "__main__":
    main()
