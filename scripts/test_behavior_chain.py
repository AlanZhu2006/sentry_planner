#!/usr/bin/env python3
"""End-to-end behavior-chain smoke test for the current sentry stack.

This script is meant to be run after Nav2, bt_comm_adapter, and rm_behavior_tree
are already up. It publishes referee inputs, subscribes to the main behavior
outputs, and prints a small pass/fail report for the current simplified tree.
"""

from __future__ import annotations

import argparse
import math
import time
from dataclasses import dataclass
from typing import Callable, Optional

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rm_decision_interfaces.msg import GameStatus, RobotControl, RobotStatus


@dataclass
class TimedValue:
    stamp_sec: float
    value: object


def bool_text(value: bool) -> str:
    return "True" if value else "False"


def approx_zero(value: float, tol: float = 0.05) -> bool:
    return abs(value) <= tol


def format_robot_control(msg: Optional[RobotControl]) -> str:
    if msg is None:
        return "None"
    return (
        "RobotControl("
        f"stop_gimbal_scan={bool_text(bool(msg.stop_gimbal_scan))}, "
        f"chassis_spin_vel={float(msg.chassis_spin_vel):.3f}, "
        f"scan_enabled={bool_text(bool(getattr(msg, "scan_enabled", False)))}, "
        f"allow_vision_control={bool_text(bool(getattr(msg, "allow_vision_control", False)))}, "
        f"search_when_target_lost={bool_text(bool(getattr(msg, "search_when_target_lost", False)))}, "
        f"scan_yaw_rate_deg_s={float(getattr(msg, "scan_yaw_rate_deg_s", 0.0)):.1f}, "
        f"search_pitch_deg={float(getattr(msg, "search_pitch_deg", 0.0)):.1f})"
    )


def format_twist(msg: Optional[Twist]) -> str:
    if msg is None:
        return "None"
    return (
        "Twist("
        f"vx={float(msg.linear.x):.3f}, "
        f"vy={float(msg.linear.y):.3f}, "
        f"wz={float(msg.angular.z):.3f})"
    )


def format_pose(msg: Optional[PoseStamped]) -> str:
    if msg is None:
        return "None"
    return (
        "PoseStamped("
        f"x={float(msg.pose.position.x):.3f}, "
        f"y={float(msg.pose.position.y):.3f}, "
        f"z={float(msg.pose.position.z):.3f})"
    )


class BehaviorChainTester(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("behavior_chain_tester")
        self.args = args

        self.game_status_pub = self.create_publisher(GameStatus, "/game_status", 10)
        self.robot_status_pub = self.create_publisher(RobotStatus, "/robot_status", 10)

        self.latest_robot_control: Optional[TimedValue] = None
        self.latest_cmd_vel: Optional[TimedValue] = None
        self.latest_cmd_vel_chassis: Optional[TimedValue] = None
        self.latest_cmd_vel_chassis_bt: Optional[TimedValue] = None
        self.latest_goal_pose: Optional[TimedValue] = None

        self.create_subscription(RobotControl, "/robot_control", self.on_robot_control, 10)
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd_vel, 10)
        self.create_subscription(Twist, "/cmd_vel_chassis", self.on_cmd_vel_chassis, 10)
        self.create_subscription(Twist, "/cmd_vel_chassis_bt", self.on_cmd_vel_chassis_bt, 10)
        self.create_subscription(PoseStamped, "/goal_pose", self.on_goal_pose, 10)

        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        self.active_game_status = GameStatus()
        self.active_game_status.game_progress = 0
        self.active_game_status.stage_remain_time = int(args.stage_remain_time)

        self.active_robot_status = RobotStatus()
        self.active_robot_status.robot_id = int(args.robot_id)
        self.active_robot_status.current_hp = int(args.current_hp)
        self.active_robot_status.shooter_heat = int(args.shooter_heat)
        self.active_robot_status.team_color = bool(args.team_color)
        self.active_robot_status.is_attacked = bool(args.is_attacked)

        self.publish_timer = self.create_timer(1.0 / max(1.0, args.publish_rate_hz), self.publish_inputs)

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def on_robot_control(self, msg: RobotControl) -> None:
        self.latest_robot_control = TimedValue(self.now_sec(), msg)

    def on_cmd_vel(self, msg: Twist) -> None:
        self.latest_cmd_vel = TimedValue(self.now_sec(), msg)

    def on_cmd_vel_chassis(self, msg: Twist) -> None:
        self.latest_cmd_vel_chassis = TimedValue(self.now_sec(), msg)

    def on_cmd_vel_chassis_bt(self, msg: Twist) -> None:
        self.latest_cmd_vel_chassis_bt = TimedValue(self.now_sec(), msg)

    def on_goal_pose(self, msg: PoseStamped) -> None:
        self.latest_goal_pose = TimedValue(self.now_sec(), msg)

    def publish_inputs(self) -> None:
        self.active_game_status.stage_remain_time = int(self.args.stage_remain_time)
        self.game_status_pub.publish(self.active_game_status)
        self.robot_status_pub.publish(self.active_robot_status)

    def set_inputs(
        self,
        *,
        game_progress: int,
        current_hp: int,
        shooter_heat: int,
        is_attacked: bool = False,
    ) -> None:
        self.active_game_status.game_progress = int(game_progress)
        self.active_robot_status.current_hp = int(current_hp)
        self.active_robot_status.shooter_heat = int(shooter_heat)
        self.active_robot_status.is_attacked = bool(is_attacked)

    def is_fresh(self, timed_value: Optional[TimedValue], timeout_sec: float = 1.0) -> bool:
        return timed_value is not None and (self.now_sec() - timed_value.stamp_sec) <= timeout_sec

    def current_robot_control(self) -> Optional[RobotControl]:
        return None if self.latest_robot_control is None else self.latest_robot_control.value

    def current_cmd_vel(self) -> Optional[Twist]:
        return None if self.latest_cmd_vel is None else self.latest_cmd_vel.value

    def current_cmd_vel_chassis(self) -> Optional[Twist]:
        return None if self.latest_cmd_vel_chassis is None else self.latest_cmd_vel_chassis.value

    def current_cmd_vel_chassis_bt(self) -> Optional[Twist]:
        return None if self.latest_cmd_vel_chassis_bt is None else self.latest_cmd_vel_chassis_bt.value

    def current_goal_pose(self) -> Optional[PoseStamped]:
        return None if self.latest_goal_pose is None else self.latest_goal_pose.value

    def wait_until(self, predicate: Callable[[], bool], timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if predicate():
                return True
        return predicate()

    def wait_for_quiet_settle(self, seconds: float = 0.8) -> None:
        deadline = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

    def is_scan_mode(self) -> bool:
        msg = self.current_robot_control()
        if msg is None:
            return False
        return bool(getattr(msg, "scan_enabled", False)) and (not bool(getattr(msg, "allow_vision_control", False)))

    def is_attack_mode(self) -> bool:
        msg = self.current_robot_control()
        if msg is None:
            return False
        return (
            bool(getattr(msg, "scan_enabled", False)) and
            bool(getattr(msg, "allow_vision_control", False)) and
            bool(getattr(msg, "search_when_target_lost", False)) and
            float(msg.chassis_spin_vel) > 0.1
        )

    def saw_motion_topic(self) -> bool:
        twists = [
            self.current_cmd_vel(),
            self.current_cmd_vel_chassis(),
            self.current_cmd_vel_chassis_bt(),
        ]
        for msg in twists:
            if msg is None:
                continue
            if abs(msg.linear.x) > 1e-3 or abs(msg.linear.y) > 1e-3 or abs(msg.angular.z) > 1e-3:
                return True
        return False

    def graph_summary(self) -> str:
        return (
            f"publishers: "
            f"/robot_control={self.count_publishers('/robot_control')}, "
            f"/cmd_vel={self.count_publishers('/cmd_vel')}, "
            f"/cmd_vel_chassis={self.count_publishers('/cmd_vel_chassis')}, "
            f"/cmd_vel_chassis_bt={self.count_publishers('/cmd_vel_chassis_bt')}, "
            f"/goal_pose={self.count_publishers('/goal_pose')}"
        )

    def latest_outputs_summary(self) -> str:
        return "\n".join(
            [
                f"  /robot_control      : {format_robot_control(self.current_robot_control())}",
                f"  /cmd_vel            : {format_twist(self.current_cmd_vel())}",
                f"  /cmd_vel_chassis    : {format_twist(self.current_cmd_vel_chassis())}",
                f"  /cmd_vel_chassis_bt : {format_twist(self.current_cmd_vel_chassis_bt())}",
                f"  /goal_pose          : {format_pose(self.current_goal_pose())}",
            ]
        )


def print_result(title: str, passed: bool, detail: str) -> None:
    status = "PASS" if passed else "FAIL"
    print(f"[{status}] {title}")
    print(f"  {detail}")


def run_suite(node: BehaviorChainTester, args: argparse.Namespace) -> int:
    failures = 0

    print("=== Behavior Chain Smoke Test ===")
    print(node.graph_summary())

    bt_inputs_ready = node.wait_until(
        lambda: node.count_subscribers("/game_status") > 0 and node.count_subscribers("/robot_status") > 0,
        args.topology_timeout,
    )
    print_result(
        "behavior tree input subscribers",
        bt_inputs_ready,
        (
            f"/game_status subscribers={node.count_subscribers('/game_status')}, "
            f"/robot_status subscribers={node.count_subscribers('/robot_status')}"
        ),
    )
    if not bt_inputs_ready:
        failures += 1

    adapter_ready = node.wait_until(
        lambda: node.count_subscribers("/robot_control") > 0 or node.count_publishers("/cmd_vel_chassis_bt") > 0,
        args.topology_timeout,
    )
    print_result(
        "bt_comm_adapter topology",
        adapter_ready,
        (
            f"/robot_control subscribers={node.count_subscribers('/robot_control')}, "
            f"/cmd_vel_chassis_bt publishers={node.count_publishers('/cmd_vel_chassis_bt')}"
        ),
    )
    if not adapter_ready:
        failures += 1

    nav_ready = node.nav_client.wait_for_server(timeout_sec=args.server_timeout)
    print_result(
        "navigate_to_pose action server",
        nav_ready,
        "reachable" if nav_ready else "not reachable yet",
    )
    if not nav_ready:
        failures += 1

    node.wait_for_quiet_settle()

    node.set_inputs(game_progress=4, current_hp=args.low_hp, shooter_heat=0, is_attacked=False)
    retreat_ok = node.wait_until(node.is_scan_mode, args.short_timeout)
    print_result(
        "low-HP retreat branch",
        retreat_ok,
        format_robot_control(node.current_robot_control()),
    )
    if not retreat_ok:
        failures += 1

    node.set_inputs(game_progress=0, current_hp=args.current_hp, shooter_heat=0, is_attacked=False)
    standby_ok = node.wait_until(node.is_scan_mode, args.short_timeout)
    print_result(
        "pre-game standby branch",
        standby_ok,
        format_robot_control(node.current_robot_control()),
    )
    if not standby_ok:
        failures += 1

    node.set_inputs(game_progress=4, current_hp=args.current_hp, shooter_heat=0, is_attacked=False)
    move_or_attack_ok = node.wait_until(
        lambda: node.is_scan_mode() or node.is_attack_mode(),
        args.short_timeout,
    )
    print_result(
        "healthy attack-chain branch entered",
        move_or_attack_ok,
        format_robot_control(node.current_robot_control()),
    )
    if not move_or_attack_ok:
        failures += 1

    motion_seen = node.wait_until(
        lambda: node.saw_motion_topic() or node.is_attack_mode(),
        args.motion_timeout,
    )
    print_result(
        "navigation or attack output observed",
        motion_seen,
        (
            "saw motion topic or already in attack mode\n"
            + node.latest_outputs_summary()
        ),
    )
    if not motion_seen:
        failures += 1

    if args.require_center:
        center_ok = node.wait_until(node.is_attack_mode, args.center_timeout)
        print_result(
            "arrived near center and switched to attack mode",
            center_ok,
            format_robot_control(node.current_robot_control()),
        )
        if not center_ok:
            failures += 1

        node.set_inputs(game_progress=4, current_hp=args.low_hp, shooter_heat=0, is_attacked=False)
        retreat_after_attack_ok = node.wait_until(node.is_scan_mode, args.short_timeout)
        print_result(
            "low-HP return after attack mode",
            retreat_after_attack_ok,
            format_robot_control(node.current_robot_control()),
        )
        if not retreat_after_attack_ok:
            failures += 1

    print("\n=== Latest Outputs ===")
    print(node.latest_outputs_summary())
    print("\n=== Summary ===")
    if failures == 0:
        print("ALL PASS")
        return 0

    print(f"{failures} CHECK(S) FAILED")
    return 1


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Test the current sentry behavior chain")
    parser.add_argument("--topology-timeout", type=float, default=3.0)
    parser.add_argument("--server-timeout", type=float, default=8.0)
    parser.add_argument("--short-timeout", type=float, default=3.0)
    parser.add_argument("--motion-timeout", type=float, default=10.0)
    parser.add_argument("--center-timeout", type=float, default=45.0)
    parser.add_argument("--publish-rate-hz", type=float, default=10.0)
    parser.add_argument("--stage-remain-time", type=int, default=180)
    parser.add_argument("--robot-id", type=int, default=7)
    parser.add_argument("--current-hp", type=int, default=600)
    parser.add_argument("--low-hp", type=int, default=200)
    parser.add_argument("--shooter-heat", type=int, default=0)
    parser.add_argument("--team-color", type=int, default=0)
    parser.add_argument("--is-attacked", action="store_true")
    parser.add_argument(
        "--require-center",
        action="store_true",
        help="Wait until the robot reaches center and switches to attack mode",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    rclpy.init()
    node = BehaviorChainTester(args)
    try:
        return run_suite(node, args)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
