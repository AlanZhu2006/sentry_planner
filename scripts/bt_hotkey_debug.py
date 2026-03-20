#!/usr/bin/env python3
"""Keyboard-driven debug publisher for center_attack_simple on real robot.

This tool continuously publishes /game_status and /robot_status so you can
switch BT branches with single-key presets while watching Groot2 and the
watcher scripts.
"""

from __future__ import annotations

import argparse
import select
import sys
import termios
import tty
from dataclasses import dataclass

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rm_decision_interfaces.msg import GameStatus, RobotStatus


@dataclass
class DebugPreset:
    name: str
    description: str
    game_progress: int
    stage_remain_time: int
    current_hp: int
    shooter_heat: int
    is_attacked: bool = False


PRESETS: dict[str, DebugPreset] = {
    "0": DebugPreset(
        name="HOME_STANDBY",
        description="未开赛，回家待机",
        game_progress=0,
        stage_remain_time=220,
        current_hp=600,
        shooter_heat=0,
    ),
    "1": DebugPreset(
        name="APPROACH_CENTER",
        description="比赛中 + 正常血量，去中心",
        game_progress=4,
        stage_remain_time=220,
        current_hp=600,
        shooter_heat=0,
    ),
    "2": DebugPreset(
        name="HOME_RECOVER_HP",
        description="比赛中 + 低血，回家恢复",
        game_progress=4,
        stage_remain_time=220,
        current_hp=200,
        shooter_heat=0,
    ),
    "3": DebugPreset(
        name="HOME_RECOVER_HEAT",
        description="比赛中 + 高热，回家恢复",
        game_progress=4,
        stage_remain_time=220,
        current_hp=600,
        shooter_heat=400,
    ),
    "4": DebugPreset(
        name="APPROACH_CENTER_ATTACKED",
        description="比赛中 + 正常血量 + attacked 标志",
        game_progress=4,
        stage_remain_time=220,
        current_hp=600,
        shooter_heat=0,
        is_attacked=True,
    ),
}


def format_preset(preset: DebugPreset) -> str:
    return (
        f"{preset.name}: game={preset.game_progress} remain={preset.stage_remain_time} "
        f"hp={preset.current_hp} heat={preset.shooter_heat} attacked={preset.is_attacked}"
    )


class HotkeyDebugPublisher(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("bt_hotkey_debug")
        self.args = args
        self.game_pub = self.create_publisher(GameStatus, args.game_topic, 10)
        self.robot_pub = self.create_publisher(RobotStatus, args.robot_topic, 10)
        self.timer = self.create_timer(1.0 / max(1.0, args.rate_hz), self.publish_current)
        self.current_key = args.default_key
        self.current_preset = PRESETS[self.current_key]
        self.current_team_color = bool(args.team_color)
        self.current_robot_id = int(args.robot_id)
        self._print_help()
        self._print_current("startup")

    def set_preset(self, key: str) -> None:
        if key not in PRESETS:
            return
        self.current_key = key
        self.current_preset = PRESETS[key]
        self._print_current(f"switch:{key}")

    def publish_current(self) -> None:
        game_msg = GameStatus()
        game_msg.game_progress = int(self.current_preset.game_progress)
        game_msg.stage_remain_time = int(self.current_preset.stage_remain_time)
        self.game_pub.publish(game_msg)

        robot_msg = RobotStatus()
        robot_msg.robot_id = int(self.current_robot_id)
        robot_msg.current_hp = int(self.current_preset.current_hp)
        robot_msg.shooter_heat = int(self.current_preset.shooter_heat)
        robot_msg.team_color = bool(self.current_team_color)
        robot_msg.is_attacked = bool(self.current_preset.is_attacked)
        self.robot_pub.publish(robot_msg)

    def _print_help(self) -> None:
        print("[BT HOTKEY] 可用按键：", flush=True)
        for key, preset in PRESETS.items():
            print(f"  {key} -> {preset.name:<20} {preset.description}", flush=True)
        print("  p -> 打印当前预设", flush=True)
        print("  h -> 打印帮助", flush=True)
        print("  q -> 退出", flush=True)
        print(
            "[BT HOTKEY] 注意：CENTER_HOLD_ATTACK 仍然取决于真实 map->base_link 是否到达中心点；\n"
            "           这个工具主要用来一键切 game_status / robot_status 分支。",
            flush=True,
        )

    def _print_current(self, reason: str) -> None:
        print(f"[BT HOTKEY] {reason} -> {format_preset(self.current_preset)}", flush=True)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Keyboard presets for BT debug on real robot")
    parser.add_argument("--rate-hz", type=float, default=10.0)
    parser.add_argument("--game-topic", default="/game_status")
    parser.add_argument("--robot-topic", default="/robot_status")
    parser.add_argument("--robot-id", type=int, default=7)
    parser.add_argument("--team-color", type=int, default=0, choices=[0, 1])
    parser.add_argument("--default-key", default="0", choices=sorted(PRESETS.keys()))
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = HotkeyDebugPublisher(args)

    stdin_fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(stdin_fd)
    tty.setcbreak(stdin_fd)

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            readable, _, _ = select.select([sys.stdin], [], [], 0.0)
            if not readable:
                continue
            key = sys.stdin.read(1)
            if key in PRESETS:
                node.set_preset(key)
            elif key == "p":
                node._print_current("manual")
            elif key == "h":
                node._print_help()
            elif key == "q":
                break
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        pass
    finally:
        termios.tcsetattr(stdin_fd, termios.TCSADRAIN, old_settings)
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
