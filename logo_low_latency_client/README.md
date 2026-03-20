# LoGoPlanner Low-Latency RGB-D Client

This is a standalone Python 3 client for sending ROS RGB-D frames to a remote LoGoPlanner inference server with a latest-frame-only policy.

## Features

- ROS topic input only, no RealSense SDK dependency
- HTTP fast transport for `/navigator_reset` and `/pointgoal_step_fast`
- ZMQ transport abstraction reserved for a future bridge server
- Resize before send, JPEG for RGB, 16-bit PNG for depth
- Single-slot overwrite buffer to avoid stale-frame buildup
- Capture thread and sender thread separated
- Per-frame latency metrics and JSON result persistence

## Project Layout

```text
logo_low_latency_client/
├── README.md
├── requirements.txt
└── client/
    ├── __init__.py
    ├── __main__.py
    ├── camera_ros.py
    ├── config.py
    ├── encoder.py
    ├── latest_frame_buffer.py
    ├── main.py
    ├── metrics.py
    ├── result_store.py
    ├── transport_base.py
    ├── transport_http_fast.py
    └── transport_zmq.py
```

## Install

Source ROS 2 first so that `rclpy` and `sensor_msgs` are available. If you are using `zsh`, use `setup.zsh`:

```bash
source /opt/ros/humble/setup.zsh
cd /home/nyu/sentry_planner/logo_low_latency_client
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

For the two-computer RGB-D -> GPU workflow, see `command.txt` in this project for a concise command checklist.

## HTTP Fast Run

```bash
source /opt/ros/humble/setup.zsh
cd /home/nyu/sentry_planner/logo_low_latency_client
source .venv/bin/activate
python3 -m client.main \
  --transport http_fast \
  --server-url http://10.224.36.118:19999 \
  --goal-x 3.0 \
  --goal-y 0.5 \
  --rgb-topic /usb_cam_color/image_raw \
  --depth-topic /camera/depth/image_raw \
  --rgb-info-topic /usb_cam_color/camera_info \
  --depth-info-topic /camera/depth/camera_info \
  --rgb-width 320 \
  --rgb-height 240 \
  --depth-width 320 \
  --depth-height 240 \
  --jpeg-quality 60 \
  --request-timeout 5.0 \
  --send-fps 5 \
  --show-stats \
  --save-json \
  --latest-json results/latest_result.json
```

## Transport Benchmark

If you want to measure the transport path first, without ROS topics or a physical depth camera, use the synthetic benchmark sender below. It still calls `/navigator_reset` and `/pointgoal_step_fast`, but it generates a 320x240 RGB JPEG and 320x240 depth PNG locally.

Request path only, with reused payload:

```bash
cd /home/nyu/sentry_planner/logo_low_latency_client
source .venv/bin/activate
python3 -m client.benchmark_transport \
  --transport http_fast \
  --server-url http://10.224.36.118:19999 \
  --iterations 20 \
  --warmup 3 \
  --send-fps 5 \
  --show-stats
```

Request path plus per-frame encode cost:

```bash
cd /home/nyu/sentry_planner/logo_low_latency_client
source .venv/bin/activate
python3 -m client.benchmark_transport \
  --transport http_fast \
  --server-url http://10.224.36.118:19999 \
  --iterations 20 \
  --warmup 3 \
  --send-fps 5 \
  --reencode-each-frame \
  --show-stats
```

## ZMQ Run

The ZMQ mode is implemented as a future-facing PUSH/PULL bridge contract. It sends msgpack payloads with `frame_id`, timestamps, `intrinsic`, goals, and compressed image bytes.

```bash
source /opt/ros/humble/setup.zsh
cd /home/nyu/sentry_planner/logo_low_latency_client
source .venv/bin/activate
python3 -m client.main \
  --transport zmq \
  --zmq-push-endpoint tcp://127.0.0.1:5555 \
  --zmq-pull-endpoint tcp://127.0.0.1:5556 \
  --goal-x 3.0 \
  --goal-y 0.5 \
  --show-stats
```

## Core Parameters

- `--server-url`: HTTP server base URL
- `--transport`: `http_fast` or `zmq`
- `--goal-x`, `--goal-y`: pointgoal input
- `--rgb-topic`, `--depth-topic`: RGB and depth image topics
- `--rgb-info-topic`, `--depth-info-topic`: camera info topics
- `--rgb-width`, `--rgb-height`: RGB resize target
- `--depth-width`, `--depth-height`: depth resize target
- `--jpeg-quality`: JPEG quality for RGB
- `--request-timeout`: transport timeout in seconds
- `--save-json`: save every response JSON
- `--latest-json`: latest result JSON path
- `--send-fps`: maximum outbound request rate
- `--show-stats`: print per-frame timing summary
- `--intrinsic-source`: which camera info is sent to the server
- `--depth-float-scale`: scale for `32FC1` or `64FC1` depth to `uint16`

## Metrics

Each response includes client-side fields:

- `capture_ts_ms`
- `encode_start_ts_ms`
- `send_start_ts_ms`
- `recv_ts_ms`
- `image_load_or_capture_ms`
- `resize_ms`
- `encode_ms`
- `request_total_ms`
- `roundtrip_ms`
- `end_to_end_ms`

If the server returns `timing`, the client prints and stores:

- `server_parse_ms`
- `server_decode_ms`
- `server_inference_ms`
- `server_total_ms`

## Latest Result

The newest response is always written to:

```text
results/latest_result.json
```

If `--save-json` is enabled, every response is also stored under `results/frame_<frame_id>_<recv_ts_ms>.json`.

## Latency Validation

1. Run the client with `--show-stats`.
2. Watch `end_to_end`, `request`, `encode`, and `server_total`.
3. Compare these numbers against an older sender path that used raw images or multipart upload.
4. The transport win should show up mainly in lower `request_total_ms` and lower `end_to_end_ms`.

## Protocol Gaps To Confirm

The current server contract is enough for the HTTP fast path, but these details should be nailed down for long-term stability:

1. Depth unit is not explicit. If the source topic is `32FC1`, the client assumes meters and converts to `uint16` with `--depth-float-scale=1000`.
2. The reset API only accepts one `intrinsic`, so it implicitly assumes RGB and depth are already aligned.
3. ZMQ reset and reply schemas are not finalized yet. This client currently uses a practical msgpack contract with `type`, `frame_id`, timestamps, `intrinsic`, goals, and compressed payload bytes.
