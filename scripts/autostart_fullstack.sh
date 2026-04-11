#!/usr/bin/env bash
set -euo pipefail

LOG_DIR="/home/nyu/sentry_planner/logs/autostart"
mkdir -p "$LOG_DIR"

export PATH="/home/nyu/.local/bin:/usr/local/bin:/usr/bin:/bin:$PATH"
unset BASH_ENV || true
unset ZDOTDIR || true

LOG_BRIDGE="$LOG_DIR/sentry_bridge.log"
LOG_VISION="$LOG_DIR/vision_detect.log"
LOG_NAV="$LOG_DIR/nav_bt.log"

 : >"$LOG_VISION"
 : >"$LOG_NAV"

echo "[autostart] Cleaning previous processes..."
pkill -f auto_aim_camera_test >/dev/null 2>&1 || true
pkill -f "just test detect --web --send" >/dev/null 2>&1 || true
pkill -f start_robot.sh >/dev/null 2>&1 || true
rm -f /tmp/nyush-rm-sentry-vision /tmp/nyush-rm-sentry-radar >/dev/null 2>&1 || true
sleep 1

ROS_SETUP=". /opt/ros/humble/setup.bash"
PLANNER_SETUP=". /home/nyu/sentry_planner/install/setup.bash"
DECISION_SETUP=". /home/nyu/sentry_planner/rm_decision_ws/install/setup.bash"

wait_for_vision_link() {
  local link_path="/tmp/nyush-rm-sentry-vision"
  local timeout_s=10
  local waited=0
  while [ ! -e "$link_path" ] && [ "$waited" -lt "$timeout_s" ]; do
    sleep 1
    waited=$((waited + 1))
  done
}

cleanup_nav_bt_processes() {
  echo "[autostart] Cleaning stale Nav/LIO processes before nav + BT..."
  pkill -9 -f fastlio_mapping >/dev/null 2>&1 || true
  pkill -9 -f livox_ros_driver2 >/dev/null 2>&1 || true
  pkill -9 -f pointcloud_to_laserscan >/dev/null 2>&1 || true
  pkill -9 -f static_transform_publisher >/dev/null 2>&1 || true
  pkill -9 -f "ros2 launch fast_lio" >/dev/null 2>&1 || true
  pkill -9 -f "ros2 launch livox_ros_driver2" >/dev/null 2>&1 || true
  sleep 1
}

if command -v xterm >/dev/null 2>&1 && [ -n "${DISPLAY:-}" ] && [ -f "${XAUTHORITY:-$HOME/.Xauthority}" ]; then
  echo "[autostart] Restarting sentry_bridge.service..."
  systemctl --user restart sentry_bridge.service

  wait_for_vision_link

  echo "[autostart] Starting vision detect (web+send) in xterm..."
  xterm -T "vision_detect" -e bash -c "$ROS_SETUP; $PLANNER_SETUP; cd /home/nyu/Codespace/nyush-rm-vision && just test detect --web --send; read -r -p 'Press Enter to close...'" &

  sleep 2

  cleanup_nav_bt_processes

  echo "[autostart] Starting nav + BT in xterm..."
  xterm -T "nav_bt" -e bash -c "$ROS_SETUP; $PLANNER_SETUP; $DECISION_SETUP; cd /home/nyu/nav_ws && \
MAP_FILE=\"/home/nyu/sentry_planner/rm_navigation_ws/src/rm_nav_bringup/map/RMUL2026.yaml\" \
BT_STYLE=center_attack_fullstack \
BT_START_GOAL=\"-0.655;0.543;0; 0;0;0;1\" \
BT_END_GOAL=\"-5.472;3.571;0; 0;0;0;1\" \
START_SERIAL_SENDER=1 \
SERIAL_SENDER_DISABLE_STATUS_PUB=0 \
START_BT=1 \
RADAR_PTY=/tmp/nyush-rm-sentry-radar \
START_FAKE_VEL_TRANSFORM=1 \
./start_robot.sh; read -r -p 'Press Enter to close...'" &
else
  echo "[autostart] xterm not found; falling back to background logs."
  echo "[autostart] Restarting sentry_bridge.service..."
  systemctl --user restart sentry_bridge.service

  wait_for_vision_link

  echo "[autostart] Starting vision detect (web+send)..."
  nohup env -u BASH_ENV -u ZDOTDIR bash -c "$ROS_SETUP; $PLANNER_SETUP; echo \"[autostart] ROS_DISTRO=\${ROS_DISTRO:-}\"; command -v just; cd /home/nyu/Codespace/nyush-rm-vision && just test detect --web --send" \
    >"$LOG_VISION" 2>&1 &

  sleep 2

  cleanup_nav_bt_processes

  echo "[autostart] Starting nav + BT..."
  nohup env -u BASH_ENV -u ZDOTDIR bash -c "$ROS_SETUP; $PLANNER_SETUP; $DECISION_SETUP; echo \"[autostart] ROS_DISTRO=\${ROS_DISTRO:-}\"; command -v just; cd /home/nyu/nav_ws && \
MAP_FILE=\"/home/nyu/sentry_planner/rm_navigation_ws/src/rm_nav_bringup/map/RMUL2026.yaml\" \
BT_STYLE=center_attack_fullstack \
BT_START_GOAL=\"-0.655;0.543;0; 0;0;0;1\" \
BT_END_GOAL=\"-5.472;3.571;0; 0;0;0;1\" \
START_SERIAL_SENDER=1 \
SERIAL_SENDER_DISABLE_STATUS_PUB=0 \
START_BT=1 \
RADAR_PTY=/tmp/nyush-rm-sentry-radar \
START_FAKE_VEL_TRANSFORM=1 \
./start_robot.sh" \
    >"$LOG_NAV" 2>&1 &
fi

echo "[autostart] Done. Logs: $LOG_DIR"
