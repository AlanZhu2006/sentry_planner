#!/usr/bin/env python3
import argparse
import os
import sys
import time
from pathlib import Path

ROOT_DIR = Path(__file__).resolve().parents[2]
if str(ROOT_DIR) not in sys.path:
    sys.path.insert(0, str(ROOT_DIR))

from scripts.rtt_common.rtt_transport import RTTTransport, RTTTransportConfig


DEFAULT_DEVICE = "STM32H723VG"
DEFAULT_IFACE = "SWD"
DEFAULT_SPEED = 4000
DEFAULT_CHANNEL = 0
DEFAULT_ELF = os.path.join("build", "nyush_rm_control_h723.elf")


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="RTT logger for NYUSH RoboMaster control firmware.",
    )
    parser.add_argument(
        "--backend",
        choices=["auto", "jlink", "pyocd"],
        default="jlink",
        help="RTT backend to use.",
    )
    parser.add_argument("--device", default=DEFAULT_DEVICE, help="Target device name.")
    parser.add_argument("--iface", default=DEFAULT_IFACE, help="Target interface.")
    parser.add_argument("--speed", type=int, default=DEFAULT_SPEED, help="SWD speed.")
    parser.add_argument(
        "--serial",
        default=None,
        help="J-Link or DAP serial number (optional).",
    )
    parser.add_argument(
        "--elf",
        default=DEFAULT_ELF,
        help="ELF file for RTT symbol lookup (J-Link mode).",
    )
    parser.add_argument(
        "--channel",
        type=int,
        default=DEFAULT_CHANNEL,
        help="RTT channel index.",
    )
    parser.add_argument(
        "--poll-ms",
        type=int,
        default=50,
        help="Polling interval in milliseconds.",
    )
    parser.add_argument(
        "--out",
        default=None,
        help="Optional output file (append).",
    )
    parser.add_argument(
        "--connect",
        choices=["normal", "halt", "under-reset"],
        default="normal",
        help="pyOCD connect mode.",
    )
    return parser.parse_args()


def _open_output(path: str | None):
    if not path:
        return None
    return open(path, "a", encoding="utf-8")


def _write_out(handle, data: bytes) -> None:
    if not data:
        return
    text = data.decode("utf-8", errors="replace")
    sys.stdout.write(text)
    sys.stdout.flush()
    if handle:
        handle.write(text)
        handle.flush()


def main() -> int:
    args = _parse_args()
    transport = RTTTransport(
        RTTTransportConfig(
            device=args.device,
            serial=args.serial,
            speed_khz=args.speed,
            connect_mode=args.connect,
            backend=args.backend,
            iface=args.iface,
            elf=args.elf,
        )
    )

    out_handle = None
    try:
        transport.open()
        out_handle = _open_output(args.out)
        print(
            f"RTT logging started ({transport.backend_in_use}). "
            f"channel={args.channel} channels={transport.channel_info or '<unknown>'}. "
            "Press Ctrl+C to stop."
        )
        while True:
            data = transport.read(args.channel, 4096)
            _write_out(out_handle, data)
            time.sleep(max(args.poll_ms, 1) / 1000.0)
    except KeyboardInterrupt:
        print("\nStopped.")
        return 0
    except Exception as exc:
        print(str(exc))
        print("Tip: try --backend jlink/pyocd, --connect under-reset, or lower --speed.")
        return 2
    finally:
        if out_handle:
            out_handle.close()
        transport.close()


if __name__ == "__main__":
    raise SystemExit(main())
