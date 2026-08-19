#!/usr/bin/env python3
"""Send held WASD driving directions to the sentry C board over USB CDC."""

import argparse
import glob
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
import secrets
import select
import struct
import sys
import threading
import time

import serial
from serial.tools import list_ports


COMMAND_NAMES = {
    "w": "forward",
    "a": "left",
    "s": "backward",
    "d": "right",
    "q": "forward-left",
    "e": "forward-right",
    "z": "backward-left",
    "c": "backward-right",
    "!": "disabled",
}

EV_KEY = 0x01
KEY_W = 17
KEY_A = 30
KEY_S = 31
KEY_D = 32
KEY_Q = 16
KEY_SPACE = 57
INPUT_EVENT = struct.Struct("llHHI")
KEY_COMMANDS = {
    KEY_W: "w",
    KEY_A: "a",
    KEY_S: "s",
    KEY_D: "d",
}

WEB_PAGE = """<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Sentry WASD Control</title>
  <style>
    body { font-family: system-ui, sans-serif; max-width: 42rem; margin: 3rem auto;
           padding: 0 1rem; text-align: center; background: #111827; color: #f9fafb; }
    .panel { padding: 2rem; border: 2px solid #374151; border-radius: 1rem; outline: none; }
    .panel:focus { border-color: #60a5fa; }
    .keys { display: grid; grid-template-columns: repeat(3, 4rem); gap: .6rem;
            justify-content: center; margin: 2rem 0; }
    .key { padding: 1.2rem 0; border-radius: .5rem; background: #374151; font-weight: 700; }
    .key.active { background: #2563eb; }
    #state { font-size: 1.4rem; font-weight: 700; }
    #link { color: #fbbf24; }
    .warning { color: #fca5a5; }
  </style>
</head>
<body>
  <main id="panel" class="panel" tabindex="0">
    <h1>Sentry WASD Control</h1>
    <p id="link">正在连接……</p>
    <div class="keys">
      <span></span><span class="key" data-key="w">W</span><span></span>
      <span class="key" data-key="a">A</span><span class="key" data-key="s">S</span>
      <span class="key" data-key="d">D</span>
    </div>
    <p id="state">已停止</p>
    <p>按住 W/A/S/D 行驶，松开停止；空格紧急停止。</p>
    <p class="warning">切换窗口、页面失焦或 SSH 断开都会自动停止。</p>
  </main>
  <script>
    const token = "__TOKEN__";
    const endpoint = `/command?token=${encodeURIComponent(token)}`;
    const panel = document.getElementById("panel");
    const state = document.getElementById("state");
    const link = document.getElementById("link");
    const held = [];
    const keyCommands = {KeyW: "w", KeyA: "a", KeyS: "s", KeyD: "d"};
    const commandLabels = {
      w: "W / 前", a: "A / 左", s: "S / 后", d: "D / 右",
      q: "W+A / 前左", e: "W+D / 前右", z: "S+A / 后左", c: "S+D / 后右"
    };
    const client = crypto.randomUUID();
    let sequence = 0;
    let current = "!";

    function activeCommand() {
      const forward = Number(held.includes("w")) - Number(held.includes("s"));
      const right = Number(held.includes("d")) - Number(held.includes("a"));
      if (forward > 0) return right < 0 ? "q" : right > 0 ? "e" : "w";
      if (forward < 0) return right < 0 ? "z" : right > 0 ? "c" : "s";
      if (right < 0) return "a";
      if (right > 0) return "d";
      return "!";
    }
    function render() {
      document.querySelectorAll(".key").forEach(
        el => el.classList.toggle("active", held.includes(el.dataset.key)));
      state.textContent = current === "!" ? "已停止" : `当前方向：${commandLabels[current]}`;
    }
    function transmit(command, beacon = false) {
      current = command;
      render();
      const body = JSON.stringify({command, client, sequence: ++sequence});
      if (beacon) {
        navigator.sendBeacon(endpoint, new Blob([body], {type: "application/json"}));
        return;
      }
      fetch(endpoint, {method: "POST", headers: {"Content-Type": "application/json"},
                       body, cache: "no-store"})
        .then(response => {
          if (!response.ok) throw new Error(`HTTP ${response.status}`);
          link.textContent = "控制链路正常";
          link.style.color = "#86efac";
        })
        .catch(() => {
          link.textContent = "控制链路中断——主控将自动停止";
          link.style.color = "#fca5a5";
        });
    }
    function stop(beacon = false) {
      held.length = 0;
      transmit("!", beacon);
    }
    window.addEventListener("keydown", event => {
      const key = keyCommands[event.code];
      if (key) {
        event.preventDefault();
        if (!event.repeat) {
          const old = held.indexOf(key);
          if (old >= 0) held.splice(old, 1);
          held.push(key);
          transmit(activeCommand());
        }
      } else if (event.code === "Space") {
        event.preventDefault();
        stop();
      }
    });
    window.addEventListener("keyup", event => {
      const key = keyCommands[event.code];
      if (!key) return;
      const index = held.indexOf(key);
      if (index >= 0) {
        event.preventDefault();
        held.splice(index, 1);
        transmit(activeCommand());
      }
    });
    window.addEventListener("blur", () => stop());
    document.addEventListener("visibilitychange", () => {
      if (document.hidden) stop(true);
    });
    window.addEventListener("pagehide", () => stop(true));
    setInterval(() => transmit(activeCommand()), 100);
    panel.focus();
    transmit("!");
  </script>
</body>
</html>
"""


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", help="USB CDC port, for example /dev/ttyACM0")
    parser.add_argument("--input", help="Linux keyboard event device, for example /dev/input/event4")
    parser.add_argument("--rate", type=float, default=20.0, help="heartbeat rate in Hz")
    parser.add_argument("--web", action="store_true", help="serve browser controls for use through SSH forwarding")
    parser.add_argument("--bind", default="127.0.0.1", help="web bind address (default: loopback only)")
    parser.add_argument("--web-port", type=int, default=8765, help="web control port")
    return parser.parse_args()


def choose_port(requested: str | None) -> str:
    if requested:
        return requested

    candidates = [port.device for port in list_ports.comports() if "ttyACM" in port.device]
    if len(candidates) == 1:
        return candidates[0]
    if not candidates:
        raise RuntimeError("No /dev/ttyACM* USB CDC device found; pass --port explicitly.")
    raise RuntimeError(f"Multiple USB CDC devices found: {', '.join(candidates)}; pass --port.")


def send_stop(ser: serial.Serial) -> None:
    for _ in range(3):
        ser.write(b"!")
        ser.flush()
        time.sleep(0.02)


def held_direction_command(held: list[str]) -> str | None:
    forward = int("w" in held) - int("s" in held)
    right = int("d" in held) - int("a" in held)
    if forward > 0:
        return "q" if right < 0 else "e" if right > 0 else "w"
    if forward < 0:
        return "z" if right < 0 else "c" if right > 0 else "s"
    if right < 0:
        return "a"
    if right > 0:
        return "d"
    return None


def choose_input_device(requested: str | None) -> str:
    if requested:
        return requested

    candidates = sorted(glob.glob("/dev/input/by-id/*-event-kbd"))
    if not candidates:
        try:
            with open("/proc/bus/input/devices", encoding="utf-8") as devices:
                for block in devices.read().split("\n\n"):
                    lines = block.splitlines()
                    name = next((line for line in lines if line.startswith("N: Name=")), "")
                    handler_line = next((line for line in lines if line.startswith("H: Handlers=")), "")
                    handlers = handler_line.partition("=")[2].split()
                    if "kbd" not in handlers or 'Name="gpio-keys"' in name:
                        continue
                    candidates.extend(f"/dev/input/{handler}" for handler in handlers if handler.startswith("event"))
        except OSError:
            pass

    candidates = sorted(set(candidates))
    if len(candidates) == 1:
        return candidates[0]
    if not candidates:
        raise RuntimeError("No Linux keyboard event device found; pass --input /dev/input/eventX explicitly.")
    raise RuntimeError(f"Multiple keyboards found: {', '.join(candidates)}; pass --input.")


def run(ser: serial.Serial, input_path: str, rate_hz: float) -> None:
    if rate_hz <= 0.0:
        raise ValueError("--rate must be positive")

    active_command: str | None = None
    held_commands: list[str] = []
    period_s = 1.0 / rate_hz

    print("Hold W/A/S/D: drive | Release: stop | Space: stop | Q: quit")
    print("Current command: disabled")
    with open(input_path, "rb", buffering=0) as input_device:
        while True:
            readable, _, _ = select.select([input_device], [], [], period_s)
            if readable:
                data = os.read(input_device.fileno(), INPUT_EVENT.size * 32)
                for offset in range(0, len(data) - INPUT_EVENT.size + 1, INPUT_EVENT.size):
                    _, _, event_type, code, value = INPUT_EVENT.unpack_from(data, offset)
                    if event_type != EV_KEY:
                        continue
                    if code == KEY_Q and value == 1:
                        return
                    if code == KEY_SPACE and value == 1:
                        held_commands.clear()
                    elif code in KEY_COMMANDS:
                        command = KEY_COMMANDS[code]
                        if value == 1:
                            if command in held_commands:
                                held_commands.remove(command)
                            held_commands.append(command)
                        elif value == 0 and command in held_commands:
                            held_commands.remove(command)

                    next_command = held_direction_command(held_commands)
                    if next_command != active_command:
                        active_command = next_command
                        wire_command = active_command if active_command is not None else "!"
                        ser.write(wire_command.encode("ascii"))
                        ser.flush()
                        print(f"\rCurrent command: {COMMAND_NAMES[wire_command]:<8}", end="", flush=True)

            if active_command is not None:
                ser.write(active_command.encode("ascii"))
                ser.flush()


class WebControlState:
    def __init__(self, ser: serial.Serial, rate_hz: float) -> None:
        self.ser = ser
        self.period_s = 1.0 / rate_hz
        self.lock = threading.Lock()
        self.command = "!"
        self.last_update = 0.0
        self.client = ""
        self.sequence = -1
        self.stop_event = threading.Event()

    def update(self, command: str, client: str, sequence: int) -> bool:
        with self.lock:
            if client == self.client and sequence <= self.sequence:
                return False
            previous = self.command
            self.client = client
            self.sequence = sequence
            self.command = command
            self.last_update = time.monotonic()
            if command == "!" and previous != "!":
                self.ser.write(b"!")
                self.ser.flush()
            if command != previous:
                print(f"\nBrowser command: {COMMAND_NAMES[command]}", flush=True)
        return True

    def heartbeat(self) -> None:
        while not self.stop_event.wait(self.period_s):
            with self.lock:
                if time.monotonic() - self.last_update > 0.3:
                    if self.command != "!":
                        self.command = "!"
                        self.ser.write(b"!")
                        self.ser.flush()
                    continue
                if self.command != "!":
                    self.ser.write(self.command.encode("ascii"))
                    self.ser.flush()


def make_web_handler(state: WebControlState, token: str) -> type[BaseHTTPRequestHandler]:
    page = WEB_PAGE.replace("__TOKEN__", token).encode("utf-8")

    class WebHandler(BaseHTTPRequestHandler):
        def do_GET(self) -> None:
            if self.path != f"/?token={token}":
                self.send_error(403)
                return
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(page)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(page)

        def do_POST(self) -> None:
            if self.path != f"/command?token={token}":
                self.send_error(403)
                return
            try:
                length = int(self.headers.get("Content-Length", "0"))
                if length <= 0 or length > 1024:
                    raise ValueError("invalid body length")
                payload = json.loads(self.rfile.read(length))
                command = payload["command"]
                client = str(payload["client"])
                sequence = int(payload["sequence"])
                if command not in COMMAND_NAMES or not client:
                    raise ValueError("invalid command")
            except (KeyError, TypeError, ValueError, json.JSONDecodeError):
                self.send_error(400)
                return

            accepted = state.update(command, client, sequence)
            self.send_response(204 if accepted else 409)
            self.send_header("Cache-Control", "no-store")
            self.end_headers()

        def log_message(self, format: str, *args: object) -> None:
            return

    return WebHandler


def run_web(ser: serial.Serial, bind: str, web_port: int, rate_hz: float) -> None:
    if rate_hz <= 0.0:
        raise ValueError("--rate must be positive")
    if not 1 <= web_port <= 65535:
        raise ValueError("--web-port must be between 1 and 65535")

    token = secrets.token_urlsafe(18)
    state = WebControlState(ser, rate_hz)
    server = ThreadingHTTPServer((bind, web_port), make_web_handler(state, token))
    heartbeat_thread = threading.Thread(target=state.heartbeat, daemon=True)
    send_stop(ser)
    heartbeat_thread.start()
    print(f"Web control listening on http://{bind}:{web_port}/?token={token}")
    print(f"From your laptop, run: ssh -N -L {web_port}:127.0.0.1:{web_port} nyu@<JETSON_IP>")
    print(f"Then open: http://127.0.0.1:{web_port}/?token={token}")
    try:
        server.serve_forever(poll_interval=0.1)
    finally:
        server.shutdown()
        server.server_close()
        state.stop_event.set()
        heartbeat_thread.join(timeout=1.0)


def main() -> int:
    args = parse_args()
    try:
        port = choose_port(args.port)
        with serial.Serial(port, 115200, timeout=0, write_timeout=0.2) as ser:
            try:
                if args.web:
                    print(f"Connected to {port}")
                    run_web(ser, args.bind, args.web_port, args.rate)
                else:
                    input_path = choose_input_device(args.input)
                    print(f"Connected to {port}; reading {input_path}")
                    run(ser, input_path, args.rate)
            finally:
                send_stop(ser)
                print("\nDrive output disabled.")
    except (RuntimeError, ValueError, OSError, serial.SerialException) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        return 130
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
