from __future__ import annotations

import logging
import signal
from threading import Event, Thread
import time
from typing import Any

from client.camera_ros import RawFrameBundle, RosRgbdCapture
from client.config import ClientConfig, parse_args
from client.encoder import EncodedFrame, FrameEncoder
from client.latest_frame_buffer import LatestFrameBuffer, VersionedFrame
from client.metrics import FrameMetrics, RollingStats, format_stats_line, now_ms, trajectory_point_count
from client.result_store import ResultStore
from client.transport_base import TransportClient
from client.transport_http_fast import HttpFastTransport
from client.transport_zmq import ZmqTransport


LOGGER = logging.getLogger(__name__)


def build_transport(config: ClientConfig) -> TransportClient:
    if config.transport == "http_fast":
        return HttpFastTransport(config.server_url, config.request_timeout)
    if config.transport == "zmq":
        return ZmqTransport(
            push_endpoint=config.zmq_push_endpoint,
            pull_endpoint=config.zmq_pull_endpoint,
            poll_timeout_ms=config.zmq_poll_timeout_ms,
        )
    raise ValueError(f"Unsupported transport: {config.transport}")


def configure_logging(log_level: str) -> None:
    logging.basicConfig(
        level=getattr(logging, log_level.upper(), logging.INFO),
        format="%(asctime)s | %(levelname)s | %(name)s | %(message)s",
    )


def build_result(
    *,
    config: ClientConfig,
    encoded: EncodedFrame,
    metrics: FrameMetrics,
    response: dict[str, Any],
) -> dict[str, Any]:
    result: dict[str, Any] = dict(response)
    result["frame_id"] = encoded.frame_id
    result["transport"] = str(result.get("transport", config.transport))
    result["capture_ts_ms"] = encoded.capture_ts_ms
    result["rgb_msg_ts_ms"] = encoded.rgb_msg_ts_ms
    result["depth_msg_ts_ms"] = encoded.depth_msg_ts_ms
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
    result["intrinsic"] = encoded.intrinsic
    result["client_transport"] = config.transport
    result["rgb_source_shape"] = list(encoded.rgb_source_shape)
    result["depth_source_shape"] = list(encoded.depth_source_shape)
    result["rgb_encoded_shape"] = list(encoded.rgb_encoded_shape)
    result["depth_encoded_shape"] = list(encoded.depth_encoded_shape)
    result["trajectory_point_count"] = trajectory_point_count(result)
    return result


def process_frame(
    *,
    config: ClientConfig,
    transport: TransportClient,
    encoder: FrameEncoder,
    versioned_frame: VersionedFrame[RawFrameBundle],
    needs_reset: bool,
    last_intrinsic_signature: tuple[float, ...] | None,
) -> tuple[tuple[float, ...], dict[str, Any], FrameMetrics]:
    encoded = encoder.encode(versioned_frame.frame)
    intrinsic_signature = tuple(round(value, 6) for row in encoded.intrinsic for value in row)

    if needs_reset or intrinsic_signature != last_intrinsic_signature:
        reset_response = transport.reset(encoded.intrinsic, config.stop_threshold, config.batch_size)
        LOGGER.info(
            "navigator_reset ok | transport=%s | intrinsic=%s | reply_keys=%s",
            transport.name,
            intrinsic_signature,
            sorted(reset_response.keys()) if isinstance(reset_response, dict) else "n/a",
        )

    send_start_ts_ms = now_ms()
    response = transport.step(
        frame_id=encoded.frame_id,
        capture_ts_ms=encoded.capture_ts_ms,
        send_ts_ms=send_start_ts_ms,
        intrinsic=encoded.intrinsic,
        goal_x=config.goal_x,
        goal_y=config.goal_y,
        image_jpeg=encoded.image_jpeg,
        depth_png=encoded.depth_png,
    )
    recv_ts_ms = now_ms()

    metrics = FrameMetrics(
        capture_ts_ms=encoded.capture_ts_ms,
        encode_start_ts_ms=encoded.encode_start_ts_ms,
        send_start_ts_ms=send_start_ts_ms,
        recv_ts_ms=recv_ts_ms,
        image_load_or_capture_ms=encoded.image_load_or_capture_ms,
        resize_ms=encoded.resize_ms,
        encode_ms=encoded.encode_ms,
        request_total_ms=recv_ts_ms - send_start_ts_ms,
        roundtrip_ms=recv_ts_ms - send_start_ts_ms,
        end_to_end_ms=recv_ts_ms - encoded.capture_ts_ms,
    )
    return intrinsic_signature, build_result(config=config, encoded=encoded, metrics=metrics, response=response), metrics


def sender_loop(
    *,
    config: ClientConfig,
    stop_event: Event,
    frame_buffer: LatestFrameBuffer[RawFrameBundle],
    transport: TransportClient,
    encoder: FrameEncoder,
    result_store: ResultStore,
) -> None:
    rolling_stats = RollingStats(window_size=50)
    last_version = 0
    last_send_monotonic = 0.0
    min_send_interval = 0.0 if config.send_fps <= 0.0 else (1.0 / config.send_fps)
    last_intrinsic_signature: tuple[float, ...] | None = None
    needs_reset = True

    LOGGER.info(
        "Sender loop started: transport=%s send_fps=%.2f latest_frame_only=true queue_size=1",
        config.transport,
        config.send_fps,
    )

    while not stop_event.is_set():
        versioned_frame = frame_buffer.wait_for_new(last_version, timeout=0.2)
        if versioned_frame is None:
            continue

        if min_send_interval > 0.0 and last_send_monotonic > 0.0:
            next_ready_time = last_send_monotonic + min_send_interval
            remaining = next_ready_time - time.monotonic()
            if remaining > 0.0:
                stop_event.wait(remaining)
                if stop_event.is_set():
                    break
                latest = frame_buffer.get_latest()
                if latest is not None and latest.version >= versioned_frame.version:
                    versioned_frame = latest

        try:
            last_send_monotonic = time.monotonic()
            last_intrinsic_signature, result, metrics = process_frame(
                config=config,
                transport=transport,
                encoder=encoder,
                versioned_frame=versioned_frame,
                needs_reset=needs_reset,
                last_intrinsic_signature=last_intrinsic_signature,
            )
            needs_reset = False

            result_store.save(result)
            rolling_stats.add(metrics)
            if config.show_stats:
                LOGGER.info(
                    format_stats_line(
                        frame_id=int(result["frame_id"]),
                        transport=str(result["transport"]),
                        metrics=metrics,
                        response=result,
                        rolling_stats=rolling_stats,
                    )
                )
            else:
                LOGGER.info(
                    "frame=%s | transport=%s | end_to_end=%.1fms | traj_points=%s",
                    result["frame_id"],
                    result["transport"],
                    result["end_to_end_ms"],
                    result["trajectory_point_count"],
                )
            last_version = versioned_frame.version
        except Exception as exc:
            needs_reset = True
            last_version = versioned_frame.version
            LOGGER.exception(
                "Failed to process frame_id=%s version=%s: %s",
                versioned_frame.frame.frame_id,
                versioned_frame.version,
                exc,
            )

    LOGGER.info("Sender loop stopped.")


def main(argv: list[str] | None = None) -> None:
    config = parse_args(argv)
    configure_logging(config.log_level)

    stop_event = Event()

    def _handle_signal(signum, _frame) -> None:
        LOGGER.info("Received signal %s, shutting down.", signum)
        stop_event.set()

    signal.signal(signal.SIGINT, _handle_signal)
    signal.signal(signal.SIGTERM, _handle_signal)

    frame_buffer: LatestFrameBuffer[RawFrameBundle] = LatestFrameBuffer()
    transport = build_transport(config)
    encoder = FrameEncoder(config)
    result_store = ResultStore(config.latest_json, config.results_dir, config.save_json)
    capture = RosRgbdCapture(config, frame_buffer)

    sender_thread = Thread(
        target=sender_loop,
        kwargs={
            "config": config,
            "stop_event": stop_event,
            "frame_buffer": frame_buffer,
            "transport": transport,
            "encoder": encoder,
            "result_store": result_store,
        },
        name="sender",
        daemon=True,
    )

    try:
        capture.start()
        sender_thread.start()
        while not stop_event.is_set():
            time.sleep(0.5)
    except KeyboardInterrupt:
        stop_event.set()
    finally:
        stop_event.set()
        capture.stop()
        transport.close()
        if sender_thread.is_alive():
            sender_thread.join(timeout=2.0)


if __name__ == "__main__":
    main()
