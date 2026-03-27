#!/usr/bin/env python3
"""Live watcher for the current center_attack_simple behavior tree."""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rm_decision_interfaces.msg import GameStatus, RobotControl, RobotStatus


@dataclass
class StateSnapshot:
    x: Optional[float] = None
    y: Optional[float] = None
    game_progress: Optional[int] = None
    stage_remain_time: Optional[int] = None
    current_hp: Optional[int] = None
    shooter_heat: Optional[int] = None
    stop_gimbal_scan: Optional[bool] = None
    chassis_spin_vel: Optional[float] = None
    scan_enabled: Optional[bool] = None
    allow_vision_control: Optional[bool] = None
    search_when_target_lost: Optional[bool] = None
    scan_yaw_rate_deg_s: Optional[float] = None
    search_pitch_deg: Optional[float] = None
    vx: Optional[float] = None
    vy: Optional[float] = None
    wz: Optional[float] = None


def fmt_float(value: Optional[float], digits: int = 2) -> str:
    if value is None:
        return "None"
    return f"{value:.{digits}f}"


def fmt_int(value: Optional[int]) -> str:
    if value is None:
        return "None"
    return str(value)


def fmt_bool(value: Optional[bool]) -> str:
    if value is None:
        return "None"
    return "True" if value else "False"


class CenterAttackWatcher(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("center_attack_state_watcher")
        self.args = args
        self.state = StateSnapshot()

        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self.on_amcl_pose, 10)
        self.create_subscription(GameStatus, "/game_status", self.on_game_status, 10)
        self.create_subscription(RobotStatus, "/robot_status", self.on_robot_status, 10)
        self.create_subscription(RobotControl, "/robot_control", self.on_robot_control, 10)
        self.create_subscription(Twist, "/cmd_vel_chassis_bt", self.on_cmd_vel_chassis_bt, 10)

        self.create_timer(1.0 / max(0.5, args.rate_hz), self.print_status)

    def on_amcl_pose(self, msg: PoseWithCovarianceStamped) -> None:
        self.state.x = float(msg.pose.pose.position.x)
        self.state.y = float(msg.pose.pose.position.y)

    def on_game_status(self, msg: GameStatus) -> None:
        self.state.game_progress = int(msg.game_progress)
        self.state.stage_remain_time = int(msg.stage_remain_time)

    def on_robot_status(self, msg: RobotStatus) -> None:
        self.state.current_hp = int(msg.current_hp)
        self.state.shooter_heat = int(msg.shooter_heat)

    def on_robot_control(self, msg: RobotControl) -> None:
        self.state.stop_gimbal_scan = bool(msg.stop_gimbal_scan)
        self.state.chassis_spin_vel = float(msg.chassis_spin_vel)
        self.state.scan_enabled = bool(getattr(msg, "scan_enabled", False))
        self.state.allow_vision_control = bool(getattr(msg, "allow_vision_control", False))
        self.state.search_when_target_lost = bool(getattr(msg, "search_when_target_lost", False))
        self.state.scan_yaw_rate_deg_s = float(getattr(msg, "scan_yaw_rate_deg_s", 0.0))
        self.state.search_pitch_deg = float(getattr(msg, "search_pitch_deg", 0.0))

    def on_cmd_vel_chassis_bt(self, msg: Twist) -> None:
        self.state.vx = float(msg.linear.x)
        self.state.vy = float(msg.linear.y)
        self.state.wz = float(msg.angular.z)

    def dist_to(self, target_x: float, target_y: float) -> Optional[float]:
        if self.state.x is None or self.state.y is None:
            return None
        return math.hypot(self.state.x - target_x, self.state.y - target_y)

    def inferred_mode(self) -> str:
        if self.state.game_progress is None or self.state.current_hp is None or self.state.shooter_heat is None:
            return "WAIT_INPUT"

        if self.state.game_progress == 3:
            if self.state.current_hp < self.args.hp_threshold or self.state.shooter_heat > self.args.heat_threshold:
                return "HOME_RECOVER"
            return "PRESTART_SCAN"

        if self.state.game_progress != 4:
            return "IDLE"

        if self.state.current_hp < self.args.hp_threshold or self.state.shooter_heat > self.args.heat_threshold:
            return "HOME_RECOVER"

        if self.state.stop_gimbal_scan and (self.state.chassis_spin_vel or 0.0) > 0.1:
            return "CENTER_HOLD_ATTACK"

        return "APPROACH_CENTER"

    def print_status(self) -> None:
        dist_home = self.dist_to(self.args.home_x, self.args.home_y)
        dist_center = self.dist_to(self.args.center_x, self.args.center_y)

        print(
            "[BT WATCH] "
            f"mode={self.inferred_mode()} "
            f"pose=({fmt_float(self.state.x)}, {fmt_float(self.state.y)}) "
            f"d_home={fmt_float(dist_home)} "
            f"d_center={fmt_float(dist_center)} "
            f"game={fmt_int(self.state.game_progress)} "
            f"remain={fmt_int(self.state.stage_remain_time)} "
            f"hp={fmt_int(self.state.current_hp)} "
            f"heat={fmt_int(self.state.shooter_heat)} "
            f"stop_scan={fmt_bool(self.state.stop_gimbal_scan)} "
            f"scan={fmt_bool(self.state.scan_enabled)} "
            f"vision={fmt_bool(self.state.allow_vision_control)} "
            f"lost_search={fmt_bool(self.state.search_when_target_lost)} "
            f"spin={fmt_float(self.state.chassis_spin_vel)} "
            f"scan_rate={fmt_float(self.state.scan_yaw_rate_deg_s)} "
            f"search_pitch={fmt_float(self.state.search_pitch_deg)} "
            f"cmd=({fmt_float(self.state.vx)}, {fmt_float(self.state.vy)}, {fmt_float(self.state.wz)})",
            flush=True,
        )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Watch center_attack_simple branch state")
    parser.add_argument("--rate-hz", type=float, default=2.0)
    parser.add_argument("--home-x", type=float, default=0.0)
    parser.add_argument("--home-y", type=float, default=0.0)
    parser.add_argument("--center-x", type=float, default=-5.431)
    parser.add_argument("--center-y", type=float, default=3.342)
    parser.add_argument("--hp-threshold", type=int, default=250)
    parser.add_argument("--heat-threshold", type=int, default=350)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = CenterAttackWatcher(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
