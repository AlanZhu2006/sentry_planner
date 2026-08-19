#!/usr/bin/env python3
"""Publish safe WASD body-velocity commands for the current ROS 2 bridge."""

import argparse
import math
import os
import select
import sys
import termios
import time
import tty

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node


KEY_VECTORS = {
    "w": (1.0, 0.0, 0.0),
    "s": (-1.0, 0.0, 0.0),
    "a": (0.0, 1.0, 0.0),
    "d": (0.0, -1.0, 0.0),
    "q": (0.0, 0.0, 1.0),
    "e": (0.0, 0.0, -1.0),
}
OPPOSITE_KEYS = {
    "w": "s",
    "s": "w",
    "a": "d",
    "d": "a",
    "q": "e",
    "e": "q",
}
STOP_KEYS = {" ", "x", "k"}


def print_controls() -> None:
    print(
        "\nW/S: forward/backward | A/D: strafe left/right | "
        "Q/E: rotate left/right\n"
        "Combine recent movement keys for diagonal motion | "
        "Space/X/K: stop | H: help | Esc/Ctrl-C: stop and exit\n"
        "This terminal cannot report key-release events. Hold a key so the "
        "terminal repeats it; output stops automatically when repeats stop.\n"
    )


class WasdTeleop(Node):
    """Publish teleoperation commands without bypassing twist_mux or the bridge."""

    def __init__(
        self,
        topic: str,
        linear_speed: float,
        angular_speed: float,
        key_timeout: float,
        publish_rate: float,
    ) -> None:
        super().__init__("wasd_teleop")
        if not topic:
            raise ValueError("topic must not be empty")
        if linear_speed <= 0.0 or angular_speed <= 0.0:
            raise ValueError("linear and angular speeds must be positive")
        if key_timeout <= 0.0 or publish_rate <= 0.0:
            raise ValueError("timeout and publish rate must be positive")

        self._publisher = self.create_publisher(Twist, topic, 10)
        self._linear_speed = linear_speed
        self._angular_speed = angular_speed
        self._key_timeout = key_timeout
        self._active_keys: dict[str, float] = {}
        self._motion_active = False
        self._last_reported: tuple[float, float, float] | None = None
        self._last_subscriber_warning = 0.0
        self.create_timer(1.0 / publish_rate, self._publish_tick)
        self.get_logger().info(
            f"WASD teleop ready: topic={topic}, linear={linear_speed:.2f} m/s, "
            f"angular={angular_speed:.2f} rad/s, deadman={key_timeout:.2f} s"
        )

    def handle_key(self, key: str) -> bool:
        """Handle one terminal character; return false when the user requests exit."""
        if key == "\x1b":
            self.stop(force=True)
            return False

        key = key.lower()
        if key in STOP_KEYS:
            self.stop(force=True)
            return True
        if key == "h" or key == "?":
            self.stop(force=True)
            print_controls()
            return True
        if key not in KEY_VECTORS:
            self.stop(force=False)
            return True

        now = time.monotonic()
        self._active_keys.pop(OPPOSITE_KEYS[key], None)
        self._active_keys[key] = now
        self._publish_motion(now)
        return True

    def stop(self, force: bool = False) -> None:
        """Publish an immediate zero and stop refreshing the teleop topic."""
        self._active_keys.clear()
        if self._motion_active or force:
            self._publisher.publish(Twist())
        self._motion_active = False
        self._report_command(0.0, 0.0, 0.0)

    def shutdown(self) -> None:
        """Send several zero frames so the downstream bridge stops immediately."""
        self._active_keys.clear()
        zero = Twist()
        for _ in range(3):
            self._publisher.publish(zero)
            time.sleep(0.02)
        self._motion_active = False

    def _publish_tick(self) -> None:
        self._publish_motion(time.monotonic())

    def _publish_motion(self, now: float) -> None:
        expired = [
            key
            for key, timestamp in self._active_keys.items()
            if now - timestamp > self._key_timeout
        ]
        for key in expired:
            self._active_keys.pop(key, None)

        if not self._active_keys:
            if self._motion_active:
                self.stop()
            return

        vx_scale = sum(KEY_VECTORS[key][0] for key in self._active_keys)
        vy_scale = sum(KEY_VECTORS[key][1] for key in self._active_keys)
        wz_scale = sum(KEY_VECTORS[key][2] for key in self._active_keys)
        translation_norm = math.hypot(vx_scale, vy_scale)
        if translation_norm > 1.0:
            vx_scale /= translation_norm
            vy_scale /= translation_norm
        wz_scale = max(-1.0, min(1.0, wz_scale))

        vx = vx_scale * self._linear_speed
        vy = vy_scale * self._linear_speed
        wz = wz_scale * self._angular_speed
        message = Twist()
        message.linear.x = vx
        message.linear.y = vy
        message.angular.z = wz
        self._publisher.publish(message)
        self._motion_active = True
        self._report_command(vx, vy, wz)

        if self._publisher.get_subscription_count() == 0:
            if now - self._last_subscriber_warning > 3.0:
                self.get_logger().warning(
                    "No subscriber on the teleop topic. Run this command via "
                    "nav-wasd or just wasd so the base stack starts automatically."
                )
                self._last_subscriber_warning = now

    def _report_command(self, vx: float, vy: float, wz: float) -> None:
        command = (round(vx, 3), round(vy, 3), round(wz, 3))
        if command == self._last_reported:
            return
        self._last_reported = command
        print(
            f"\rcommand vx={vx:+.2f} m/s  vy={vy:+.2f} m/s  "
            f"wz={wz:+.2f} rad/s   ",
            end="",
            flush=True,
        )


def parse_arguments() -> tuple[argparse.Namespace, list[str]]:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", default="/cmd_vel_teleop")
    parser.add_argument(
        "--speed", type=float, default=0.80, help="translation speed in m/s"
    )
    parser.add_argument("--turn", type=float, default=2.40, help="yaw speed in rad/s")
    parser.add_argument(
        "--timeout",
        type=float,
        default=0.45,
        help="stop this many seconds after keyboard repeats cease",
    )
    parser.add_argument(
        "--rate", type=float, default=20.0, help="publish rate in Hz"
    )
    return parser.parse_known_args()


def main() -> int:
    arguments, ros_arguments = parse_arguments()
    if not sys.stdin.isatty():
        print("Error: WASD teleop requires an interactive terminal.", file=sys.stderr)
        return 2

    rclpy.init(args=ros_arguments)
    node: WasdTeleop | None = None
    terminal_settings = termios.tcgetattr(sys.stdin)
    try:
        node = WasdTeleop(
            topic=arguments.topic,
            linear_speed=arguments.speed,
            angular_speed=arguments.turn,
            key_timeout=arguments.timeout,
            publish_rate=arguments.rate,
        )
        print_controls()
        tty.setcbreak(sys.stdin.fileno())
        running = True
        while running and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
            readable, _, _ = select.select([sys.stdin], [], [], 0.0)
            if readable:
                data = os.read(sys.stdin.fileno(), 32).decode("utf-8", errors="ignore")
                for key in data:
                    if not node.handle_key(key):
                        running = False
                        break
    except (KeyboardInterrupt, ValueError) as error:
        if isinstance(error, ValueError):
            print(f"Error: {error}", file=sys.stderr)
            return 2
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, terminal_settings)
        if node is not None:
            node.shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("\nTeleop stopped; zero velocity sent.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
