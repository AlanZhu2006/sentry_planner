#!/usr/bin/env python3
"""Interactive vision communication tool for SP protocol.

Communicates with the STM32 MCU over UART/VCP using SP protocol. Supports:
  - Read mode:  display MCU data (quaternion, yaw/pitch, bullet speed, etc.)
  - Write mode: send vision commands to MCU interactively

    python3 vision_tool.py
"""
import math
import struct
import sys
from datetime import datetime

try:
    import serial
    from serial.tools.list_ports import comports
except ImportError:
    print("pyserial is required.  pip install pyserial", file=sys.stderr)
    sys.exit(1)

# ---------------------------------------------------------------------------
# CRC16 (matches firmware crc16.c table)
# ---------------------------------------------------------------------------
CRC16_TABLE = [
    0x0000, 0x1189, 0x2312, 0x329B, 0x4624, 0x57AD, 0x6536, 0x74BF, 0x8C48, 0x9DC1, 0xAF5A, 0xBED3,
    0xCA6C, 0xDBE5, 0xE97E, 0xF8F7, 0x1081, 0x0108, 0x3393, 0x221A, 0x56A5, 0x472C, 0x75B7, 0x643E,
    0x9CC9, 0x8D40, 0xBFDB, 0xAE52, 0xDAED, 0xCB64, 0xF9FF, 0xE876, 0x2102, 0x308B, 0x0210, 0x1399,
    0x6726, 0x76AF, 0x4434, 0x55BD, 0xAD4A, 0xBCC3, 0x8E58, 0x9FD1, 0xEB6E, 0xFAE7, 0xC87C, 0xD9F5,
    0x3183, 0x200A, 0x1291, 0x0318, 0x77A7, 0x662E, 0x54B5, 0x453C, 0xBDCB, 0xAC42, 0x9ED9, 0x8F50,
    0xFBEF, 0xEA66, 0xD8FD, 0xC974, 0x4204, 0x538D, 0x6116, 0x709F, 0x0420, 0x15A9, 0x2732, 0x36BB,
    0xCE4C, 0xDFC5, 0xED5E, 0xFCD7, 0x8868, 0x99E1, 0xAB7A, 0xBAF3, 0x5285, 0x430C, 0x7197, 0x601E,
    0x14A1, 0x0528, 0x37B3, 0x263A, 0xDECD, 0xCF44, 0xFDDF, 0xEC56, 0x98E9, 0x8960, 0xBBFB, 0xAA72,
    0x6306, 0x728F, 0x4014, 0x519D, 0x2522, 0x34AB, 0x0630, 0x17B9, 0xEF4E, 0xFEC7, 0xCC5C, 0xDDD5,
    0xA96A, 0xB8E3, 0x8A78, 0x9BF1, 0x7387, 0x620E, 0x5095, 0x411C, 0x35A3, 0x242A, 0x16B1, 0x0738,
    0xFFCF, 0xEE46, 0xDCDD, 0xCD54, 0xB9EB, 0xA862, 0x9AF9, 0x8B70, 0x8408, 0x9581, 0xA71A, 0xB693,
    0xC22C, 0xD3A5, 0xE13E, 0xF0B7, 0x0840, 0x19C9, 0x2B52, 0x3ADB, 0x4E64, 0x5FED, 0x6D76, 0x7CFF,
    0x9489, 0x8500, 0xB79B, 0xA612, 0xD2AD, 0xC324, 0xF1BF, 0xE036, 0x18C1, 0x0948, 0x3BD3, 0x2A5A,
    0x5EE5, 0x4F6C, 0x7DF7, 0x6C7E, 0xA50A, 0xB483, 0x8618, 0x9791, 0xE32E, 0xF2A7, 0xC03C, 0xD1B5,
    0x2942, 0x38CB, 0x0A50, 0x1BD9, 0x6F66, 0x7EEF, 0x4C74, 0x5DFD, 0xB58B, 0xA402, 0x9699, 0x8710,
    0xF3AF, 0xE226, 0xD0BD, 0xC134, 0x39C3, 0x284A, 0x1AD1, 0x0B58, 0x7FE7, 0x6E6E, 0x5CF5, 0x4D7C,
    0xC60C, 0xD785, 0xE51E, 0xF497, 0x8028, 0x91A1, 0xA33A, 0xB2B3, 0x4A44, 0x5BCD, 0x6956, 0x78DF,
    0x0C60, 0x1DE9, 0x2F72, 0x3EFB, 0xD68D, 0xC704, 0xF59F, 0xE416, 0x90A9, 0x8120, 0xB3BB, 0xA232,
    0x5AC5, 0x4B4C, 0x79D7, 0x685E, 0x1CE1, 0x0D68, 0x3FF3, 0x2E7A, 0xE70E, 0xF687, 0xC41C, 0xD595,
    0xA12A, 0xB0A3, 0x8238, 0x93B1, 0x6B46, 0x7ACF, 0x4854, 0x59DD, 0x2D62, 0x3CEB, 0x0E70, 0x1FF9,
    0xF78F, 0xE606, 0xD49D, 0xC514, 0xB1AB, 0xA022, 0x92B9, 0x8330, 0x7BC7, 0x6A4E, 0x58D5, 0x495C,
    0x3DE3, 0x2C6A, 0x1EF1, 0x0F78
]

def crc16(data: bytes) -> int:
    """Calculate CRC16 with initial value 0xFFFF."""
    crc = 0xFFFF
    for b in data:
        crc = (crc >> 8) ^ CRC16_TABLE[(crc ^ b) & 0x00FF]
    return crc & 0xFFFF

# ---------------------------------------------------------------------------
# SP Protocol structures
# ---------------------------------------------------------------------------
SP_HEADER_1 = ord('S')
SP_HEADER_2 = ord('P')

# GimbalToVision: 43/44 bytes (云台 -> 视觉)
# struct: head[2] + mode(1) + (pad?) + q[4](16) + yaw(4) + yaw_vel(4) + pitch(4) + pitch_vel(4) + bullet_speed(4) + bullet_count(2) + crc16(2)
# Packed size = 43 bytes, default alignment size = 44 bytes
GIMBAL_TO_VISION_MIN_SIZE = 43
GIMBAL_TO_VISION_MAX_SIZE = 44
GIMBAL_TO_VISION_FMT_43 = '<2sB9f2H'   # head(2) + mode + 9 floats + 2 uint16
GIMBAL_TO_VISION_FMT_44 = '<2sBx9f2H'  # head(2) + mode + pad + 9 floats + 2 uint16

# VisionToGimbal: 29 bytes (视觉 -> 云台)
# struct: head[2] + mode(1) + yaw(4) + yaw_vel(4) + yaw_acc(4) + pitch(4) + pitch_vel(4) + pitch_acc(4) + crc16(2)
VISION_TO_GIMBAL_SIZE = 29
VISION_TO_GIMBAL_FMT = '<2sB ffffff H'  # Little-endian

# Enum labels
GIMBAL_MODE = {0: "IDLE", 1: "AUTO_AIM", 2: "SMALL_BUFF", 3: "BIG_BUFF"}
VISION_MODE = {0: "NO_CONTROL", 1: "CONTROL_GIMBAL", 2: "CONTROL_GIMBAL_FIRE"}

# ---------------------------------------------------------------------------
# ANSI colors
# ---------------------------------------------------------------------------
BOLD = "\033[1m"
DIM = "\033[2m"
GREEN = "\033[32m"
CYAN = "\033[36m"
YELLOW = "\033[33m"
RED = "\033[31m"
RESET = "\033[0m"

# ---------------------------------------------------------------------------
# SP Protocol packet building/parsing
# ---------------------------------------------------------------------------
def build_vision_to_gimbal(mode: int, yaw: float, yaw_vel: float, yaw_acc: float,
                           pitch: float, pitch_vel: float, pitch_acc: float) -> bytes:
    """Build VisionToGimbal packet (29 bytes)."""
    head = bytes([SP_HEADER_1, SP_HEADER_2])

    # Pack data (without CRC16)
    data = struct.pack('<B ffffff', mode, yaw, yaw_vel, yaw_acc, pitch, pitch_vel, pitch_acc)
    packet = head + data

    # Calculate and append CRC16
    crc = crc16(packet)
    packet += struct.pack('<H', crc)

    return packet

def parse_gimbal_to_vision(data: bytes) -> dict:
    """Parse GimbalToVision packet (43 or 44 bytes)."""
    if len(data) not in (GIMBAL_TO_VISION_MIN_SIZE, GIMBAL_TO_VISION_MAX_SIZE):
        raise ValueError(
            f"Invalid packet size: {len(data)} (expected {GIMBAL_TO_VISION_MIN_SIZE} or {GIMBAL_TO_VISION_MAX_SIZE})"
        )

    # Verify header
    if data[0] != SP_HEADER_1 or data[1] != SP_HEADER_2:
        raise ValueError(f"Invalid header: {data[0]:02X} {data[1]:02X}")

    # Verify CRC16
    calc_crc = crc16(data[:-2])
    recv_crc = struct.unpack('<H', data[-2:])[0]
    if calc_crc != recv_crc:
        raise ValueError(f"CRC mismatch: calc={calc_crc:04X}, recv={recv_crc:04X}")

    # Unpack data
    # Format: '<3B9f2H' -> [head[0], head[1], mode, q[0], q[1], q[2], q[3], yaw, yaw_vel, pitch, pitch_vel, bullet_speed, bullet_count, crc16]
    fmt = GIMBAL_TO_VISION_FMT_44 if len(data) == GIMBAL_TO_VISION_MAX_SIZE else GIMBAL_TO_VISION_FMT_43
    unpacked = struct.unpack(fmt, data)

    return {
        'mode': unpacked[1],           # index 1 (after head)
        'q': unpacked[2:6],            # indices 2-5 (quaternion [w, x, y, z])
        'yaw': unpacked[6],            # index 6
        'yaw_vel': unpacked[7],        # index 7
        'pitch': unpacked[8],          # index 8
        'pitch_vel': unpacked[9],      # index 9
        'bullet_speed': unpacked[10],  # index 10
        'bullet_count': unpacked[11],  # index 11
        # crc16 is at index 12, not extracted
    }

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
def prompt(msg: str, default: str = "") -> str:
    try:
        hint = f" {DIM}[{default}]{RESET}" if default else ""
        val = input(f"  {msg}{hint}: ").strip()
        return val if val else default
    except (EOFError, KeyboardInterrupt):
        print()
        return ""

def pick_port() -> str:
    ports = sorted(comports(), key=lambda p: p.device)
    if not ports:
        print(f"  {YELLOW}No serial ports detected.{RESET}")
        return prompt("Enter port manually", "/dev/ttyACM0")

    print(f"  {BOLD}Available ports:{RESET}")
    for i, p in enumerate(ports, 1):
        desc = f" {DIM}- {p.description}{RESET}" if p.description and p.description != "n/a" else ""
        print(f"    {CYAN}[{i}]{RESET} {p.device}{desc}")
    print()

    choice = prompt("Select port number or type path", "1")
    if choice.isdigit():
        idx = int(choice) - 1
        if 0 <= idx < len(ports):
            return ports[idx].device
    return choice

def separator():
    print(f"  {DIM}{'─' * 70}{RESET}")

def quat_to_euler(q):
    """Convert quaternion [w,x,y,z] to Euler angles [yaw, pitch, roll] in degrees."""
    w, x, y, z = q

    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    pitch = math.asin(max(-1, min(1, sinp)))

    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return [yaw * 57.295779513, pitch * 57.295779513, roll * 57.295779513]

# ---------------------------------------------------------------------------
# Read mode
# ---------------------------------------------------------------------------
def fmt_packet(info: dict) -> str:
    """Format GimbalToVision packet for display."""
    ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]

    mode_str = GIMBAL_MODE.get(info['mode'], f"UNKNOWN({info['mode']})")

    # Convert quaternion to Euler angles for display
    ypr = quat_to_euler(info['q'])

    return (
        f"  {DIM}[{ts}]{RESET}\n"
        f"    mode={CYAN}{mode_str:<11}{RESET}  "
        f"bullet={YELLOW}{info['bullet_speed']:.1f}m/s{RESET}  "
        f"count={info['bullet_count']}\n"
        f"    yaw={GREEN}{info['yaw']:+7.3f}rad{RESET} ({ypr[0]:+7.2f}°)  "
        f"vel={GREEN}{info['yaw_vel']:+7.3f}rad/s{RESET}\n"
        f"    pitch={GREEN}{info['pitch']:+7.3f}rad{RESET} ({ypr[1]:+7.2f}°)  "
        f"vel={GREEN}{info['pitch_vel']:+7.3f}rad/s{RESET}"
    )

def run_read(ser: serial.Serial):
    separator()
    print(f"  {BOLD}Read Mode{RESET} - receiving GimbalToVision packets")
    print(
        f"  {DIM}Packet size: {GIMBAL_TO_VISION_MIN_SIZE}/{GIMBAL_TO_VISION_MAX_SIZE} bytes (auto){RESET}"
    )
    print(f"  {DIM}Press Ctrl+C to stop.{RESET}")
    separator()
    print()

    buf = bytearray()
    try:
        while True:
            chunk = ser.read(max(ser.in_waiting, 1))
            if not chunk:
                continue
            buf.extend(chunk)

            # Look for SP header
            while len(buf) >= 2:
                idx = -1
                for i in range(len(buf) - 1):
                    if buf[i] == SP_HEADER_1 and buf[i+1] == SP_HEADER_2:
                        idx = i
                        break

                if idx < 0:
                    # No header found, keep last byte
                    buf = buf[-1:]
                    break

                if idx > 0:
                    buf = buf[idx:]

                if len(buf) < GIMBAL_TO_VISION_MIN_SIZE:
                    break

                # Try to parse packet (packed or padded)
                packet = bytes(buf[:GIMBAL_TO_VISION_MIN_SIZE])
                try:
                    info = parse_gimbal_to_vision(packet)
                    print(fmt_packet(info))
                    buf = buf[GIMBAL_TO_VISION_MIN_SIZE:]
                except ValueError as e:
                    if "CRC mismatch" in str(e) and len(buf) >= GIMBAL_TO_VISION_MAX_SIZE:
                        packet = bytes(buf[:GIMBAL_TO_VISION_MAX_SIZE])
                        try:
                            info = parse_gimbal_to_vision(packet)
                            print(fmt_packet(info))
                            buf = buf[GIMBAL_TO_VISION_MAX_SIZE:]
                            continue
                        except ValueError:
                            pass
                    print(f"  {RED}Parse error: {e}{RESET}")
                    buf = buf[2:]  # Skip header and continue

    except KeyboardInterrupt:
        print(f"\n  {DIM}Stopped.{RESET}\n")

# ---------------------------------------------------------------------------
# Write mode
# ---------------------------------------------------------------------------
FIELDS = [
    # key,            label,                      type, enum_map, default
    ("mode",          "Control mode",             int,  VISION_MODE, 0),
    ("yaw",           "Yaw target (rad)",         float, None, 0.0),
    ("yaw_vel",       "Yaw velocity (rad/s)",     float, None, 0.0),
    ("yaw_acc",       "Yaw acceleration (rad/s²)", float, None, 0.0),
    ("pitch",         "Pitch target (rad)",       float, None, 0.0),
    ("pitch_vel",     "Pitch velocity (rad/s)",   float, None, 0.0),
    ("pitch_acc",     "Pitch accel (rad/s²)",     float, None, 0.0),
]

def show_state(s: dict):
    print()
    for i, (key, label, _, emap, _) in enumerate(FIELDS, 1):
        v = s[key]
        if emap:
            name = emap.get(v, "?")
            opts = " | ".join(f"{k}={n}" for k, n in emap.items())
            print(f"    {CYAN}[{i}]{RESET} {label:28s}:  {BOLD}{v}{RESET}  ({name})")
            print(f"        {DIM}{opts}{RESET}")
        else:
            print(f"    {CYAN}[{i}]{RESET} {label:28s}:  {BOLD}{v}{RESET}")

def do_send(ser: serial.Serial, s: dict):
    packet = build_vision_to_gimbal(
        s["mode"], s["yaw"], s["yaw_vel"], s["yaw_acc"],
        s["pitch"], s["pitch_vel"], s["pitch_acc"]
    )
    ser.write(packet)
    ser.flush()
    print(f"  {GREEN}-> Sent {len(packet)} bytes{RESET}  {DIM}[{packet.hex()}]{RESET}")

def run_write(ser: serial.Serial):
    state = {key: default for key, _, _, _, default in FIELDS}

    separator()
    print(f"  {BOLD}Write Mode{RESET} - send VisionToGimbal packets to MCU")
    separator()
    print()
    print(f"  {YELLOW}IMPORTANT - Control modes:{RESET}")
    print(f"    {BOLD}0{RESET} = NO_CONTROL     - Manual control only")
    print(f"    {BOLD}1{RESET} = CONTROL_GIMBAL - Vision controls gimbal (no fire)")
    print(f"    {BOLD}2{RESET} = CONTROL_GIMBAL_FIRE - Vision controls gimbal + auto fire")
    print()
    print(f"  {DIM}Angles are in radians. Velocities and accelerations are optional.")
    print(f"  {DIM}Set mode=1 or mode=2 to enable vision control.{RESET}")
    separator()
    print()
    print(f"  Commands:  {CYAN}1-7{RESET} edit field   "
          f"{CYAN}s{RESET} send   {CYAN}q{RESET} quit")

    while True:
        show_state(state)
        cmd = prompt(f"\n  {BOLD}>{RESET} ")
        if not cmd:
            continue
        if cmd == "q":
            break
        elif cmd == "s":
            do_send(ser, state)
        elif cmd.isdigit():
            idx = int(cmd) - 1
            if 0 <= idx < len(FIELDS):
                key, label, typ, _, _ = FIELDS[idx]
                raw = prompt(f"  New value for {label}", str(state[key]))
                if raw:
                    try:
                        state[key] = typ(raw)
                    except ValueError:
                        print(f"    {RED}Invalid input, value unchanged.{RESET}")
            else:
                print(f"    {RED}Invalid field number.{RESET}")
        else:
            print(f"    {DIM}Unknown command. Use 1-7, s, or q.{RESET}")
    print()

# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    print()
    print(f"  {BOLD}╔════════════════════════════════════════════════╗{RESET}")
    print(f"  {BOLD}║   Vision Tool  -  SP protocol v2.0             ║{RESET}")
    print(f"  {BOLD}╚════════════════════════════════════════════════╝{RESET}")
    print()

    port = pick_port()
    if not port:
        return
    baud = 115200  # Adjust if needed

    print()
    print(f"  {BOLD}Select mode:{RESET}")
    print(f"    {CYAN}[1]{RESET} Read  - receive & display GimbalToVision packets")
    print(f"    {CYAN}[2]{RESET} Write - send VisionToGimbal commands to MCU")
    mode = prompt("Mode", "1")

    try:
        ser = serial.Serial(port, baud, timeout=0.5)
    except serial.SerialException as e:
        print(f"\n  {RED}Failed to open {port}: {e}{RESET}\n")
        return

    with ser:
        print(f"\n  {GREEN}Opened {port} @ {baud}{RESET}")
        if mode == "2":
            run_write(ser)
        else:
            run_read(ser)

    print(f"  {DIM}Port closed. Bye!{RESET}")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print(f"\n  {DIM}Interrupted. Bye!{RESET}")
