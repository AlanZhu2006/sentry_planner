from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class ClientConfig:
    server_url: str
    transport: str
    goal_x: float
    goal_y: float
    rgb_topic: str
    depth_topic: str
    rgb_info_topic: str
    depth_info_topic: str
    rgb_width: int
    rgb_height: int
    depth_width: int
    depth_height: int
    jpeg_quality: int
    png_compression: int
    request_timeout: float
    save_json: bool
    latest_json: Path
    results_dir: Path
    send_fps: float
    show_stats: bool
    stop_threshold: float
    batch_size: int
    sync_tolerance_ms: float
    log_level: str
    intrinsic_source: str
    depth_float_scale: float
    zmq_push_endpoint: str
    zmq_pull_endpoint: str
    zmq_poll_timeout_ms: int


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Low-latency RGB-D client for LoGoPlanner pointgoal inference."
    )
    parser.add_argument("--server-url", default="http://10.224.36.118:19999")
    parser.add_argument("--transport", choices=("http_fast", "zmq"), default="http_fast")
    parser.add_argument("--goal-x", type=float, default=3.0)
    parser.add_argument("--goal-y", type=float, default=0.5)

    parser.add_argument("--rgb-topic", default="/usb_cam_color/image_raw")
    parser.add_argument("--depth-topic", default="/camera/depth/image_raw")
    parser.add_argument("--rgb-info-topic", default="/usb_cam_color/camera_info")
    parser.add_argument("--depth-info-topic", default="/camera/depth/camera_info")

    parser.add_argument("--rgb-width", type=int, default=320)
    parser.add_argument("--rgb-height", type=int, default=240)
    parser.add_argument("--depth-width", type=int, default=320)
    parser.add_argument("--depth-height", type=int, default=240)
    parser.add_argument("--jpeg-quality", type=int, default=60)
    parser.add_argument(
        "--png-compression",
        type=int,
        default=1,
        help="OpenCV PNG compression level for depth encoding (0-9). Lower is faster.",
    )

    parser.add_argument("--request-timeout", type=float, default=5.0)
    parser.add_argument(
        "--save-json",
        action="store_true",
        help="Save every response as an individual JSON file.",
    )
    parser.add_argument("--latest-json", type=Path, default=Path("results/latest_result.json"))
    parser.add_argument("--results-dir", type=Path, default=Path("results"))
    parser.add_argument("--send-fps", type=float, default=5.0)
    parser.add_argument("--show-stats", action="store_true")

    parser.add_argument("--stop-threshold", type=float, default=-1.0)
    parser.add_argument("--batch-size", type=int, default=1)
    parser.add_argument(
        "--sync-tolerance-ms",
        type=float,
        default=50.0,
        help="Maximum RGB-depth stamp delta accepted as one frame pair.",
    )
    parser.add_argument(
        "--intrinsic-source",
        choices=("rgb", "depth"),
        default="rgb",
        help="Which camera_info topic provides the intrinsic matrix sent to the server.",
    )
    parser.add_argument(
        "--depth-float-scale",
        type=float,
        default=1000.0,
        help="Scale applied when converting float depth images to uint16 before PNG encoding.",
    )

    parser.add_argument("--zmq-push-endpoint", default="tcp://127.0.0.1:5555")
    parser.add_argument("--zmq-pull-endpoint", default="tcp://127.0.0.1:5556")
    parser.add_argument("--zmq-poll-timeout-ms", type=int, default=5000)
    parser.add_argument("--log-level", default="INFO")
    return parser


def parse_args(argv: list[str] | None = None) -> ClientConfig:
    args = build_parser().parse_args(argv)
    return ClientConfig(
        server_url=args.server_url.rstrip("/"),
        transport=args.transport,
        goal_x=args.goal_x,
        goal_y=args.goal_y,
        rgb_topic=args.rgb_topic,
        depth_topic=args.depth_topic,
        rgb_info_topic=args.rgb_info_topic,
        depth_info_topic=args.depth_info_topic,
        rgb_width=args.rgb_width,
        rgb_height=args.rgb_height,
        depth_width=args.depth_width,
        depth_height=args.depth_height,
        jpeg_quality=max(1, min(100, args.jpeg_quality)),
        png_compression=max(0, min(9, args.png_compression)),
        request_timeout=max(0.1, args.request_timeout),
        save_json=bool(args.save_json),
        latest_json=args.latest_json,
        results_dir=args.results_dir,
        send_fps=max(0.0, args.send_fps),
        show_stats=bool(args.show_stats),
        stop_threshold=args.stop_threshold,
        batch_size=max(1, args.batch_size),
        sync_tolerance_ms=max(1.0, args.sync_tolerance_ms),
        log_level=args.log_level.upper(),
        intrinsic_source=args.intrinsic_source,
        depth_float_scale=max(0.0, args.depth_float_scale),
        zmq_push_endpoint=args.zmq_push_endpoint,
        zmq_pull_endpoint=args.zmq_pull_endpoint,
        zmq_poll_timeout_ms=max(1, args.zmq_poll_timeout_ms),
    )

