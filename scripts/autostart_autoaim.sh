#!/usr/bin/env bash
set -euo pipefail

LOG_DIR="/home/nyu/sentry_planner/logs/autostart"
mkdir -p "$LOG_DIR"

export PATH="/home/nyu/.local/bin:/usr/local/bin:/usr/bin:/bin:$PATH"
unset BASH_ENV || true
unset ZDOTDIR || true

LOG_BRIDGE="$LOG_DIR/sentry_bridge.log"
LOG_VISION="$LOG_DIR/vision_detect.log"
LOG_AUTOAIM="$LOG_DIR/autoaim_keepalive.log"

: >"$LOG_BRIDGE"
: >"$LOG_VISION"
: >"$LOG_AUTOAIM"

ROS_SETUP=". /opt/ros/humble/setup.bash"
PLANNER_SETUP=". /home/nyu/sentry_planner/install/setup.bash"

AUTOAIM_MODE="${AUTOAIM_MODE:-autoaim}"
AUTOAIM_SCAN_YAW_RATE_DEG_S="${AUTOAIM_SCAN_YAW_RATE_DEG_S:-120.0}"
AUTOAIM_SEARCH_PITCH_DEG="${AUTOAIM_SEARCH_PITCH_DEG:--6.0}"
AUTOAIM_CHASSIS_SPIN_VEL="${AUTOAIM_CHASSIS_SPIN_VEL:-0.0}"

echo "[autostart] Cleaning previous processes..."
pkill -f sentry_bridge.py >/dev/null 2>&1 || true
pkill -f auto_aim_camera_test >/dev/null 2>&1 || true
pkill -f "just test detect --web --send" >/dev/null 2>&1 || true
pkill -f start_autoaim_mode.sh >/dev/null 2>&1 || true
rm -f /tmp/nyush-rm-sentry-bridge-ttyACM*.lock >/dev/null 2>&1 || true
rm -f /tmp/nyush-rm-sentry-vision /tmp/nyush-rm-sentry-radar >/dev/null 2>&1 || true
sleep 1

wait_for_vision_link() {
  local link_path="/tmp/nyush-rm-sentry-vision"
  local timeout_s=10
  local waited=0
  while [ ! -e "$link_path" ] && [ "$waited" -lt "$timeout_s" ]; do
    sleep 1
    waited=$((waited + 1))
  done
}

if command -v xterm >/dev/null 2>&1 && [ -n "${DISPLAY:-}" ] && [ -f "${XAUTHORITY:-$HOME/.Xauthority}" ]; then
  echo "[autostart] Starting sentry_bridge in xterm..."
  xterm -T "sentry_bridge" -e bash -c "$ROS_SETUP; $PLANNER_SETUP; cd /home/nyu/Codespace/nyush-rm-control && just sentry-bridge; read -r -p 'Press Enter to close...'" &

  wait_for_vision_link

  echo "[autostart] Starting vision detect (web+send) in xterm..."
  xterm -T "vision_detect" -e bash -c "$ROS_SETUP; $PLANNER_SETUP; cd /home/nyu/Codespace/nyush-rm-vision && just test detect --web --send; read -r -p 'Press Enter to close...'" &

  sleep 2

  echo "[autostart] Starting autoaim keepalive in xterm..."
  xterm -T "autoaim_keepalive" -e bash -c "$ROS_SETUP; $PLANNER_SETUP; cd /home/nyu/sentry_planner && \
AUTOAIM_MODE=$AUTOAIM_MODE \
AUTOAIM_SCAN_YAW_RATE_DEG_S=$AUTOAIM_SCAN_YAW_RATE_DEG_S \
AUTOAIM_SEARCH_PITCH_DEG=$AUTOAIM_SEARCH_PITCH_DEG \
AUTOAIM_CHASSIS_SPIN_VEL=$AUTOAIM_CHASSIS_SPIN_VEL \
./start_autoaim_mode.sh; read -r -p 'Press Enter to close...'" &
else
  echo "[autostart] xterm not found; falling back to background logs."
  echo "[autostart] Starting sentry_bridge..."
  nohup env -u BASH_ENV -u ZDOTDIR bash -c "$ROS_SETUP; $PLANNER_SETUP; echo \"[autostart] ROS_DISTRO=\${ROS_DISTRO:-}\"; command -v just; cd /home/nyu/Codespace/nyush-rm-control && just sentry-bridge" \
    >"$LOG_BRIDGE" 2>&1 &

  wait_for_vision_link

  echo "[autostart] Starting vision detect (web+send)..."
  nohup env -u BASH_ENV -u ZDOTDIR bash -c "$ROS_SETUP; $PLANNER_SETUP; echo \"[autostart] ROS_DISTRO=\${ROS_DISTRO:-}\"; command -v just; cd /home/nyu/Codespace/nyush-rm-vision && just test detect --web --send" \
    >"$LOG_VISION" 2>&1 &

  sleep 2

  echo "[autostart] Starting autoaim keepalive..."
  nohup env -u BASH_ENV -u ZDOTDIR bash -c "$ROS_SETUP; $PLANNER_SETUP; cd /home/nyu/sentry_planner && \
AUTOAIM_MODE=$AUTOAIM_MODE \
AUTOAIM_SCAN_YAW_RATE_DEG_S=$AUTOAIM_SCAN_YAW_RATE_DEG_S \
AUTOAIM_SEARCH_PITCH_DEG=$AUTOAIM_SEARCH_PITCH_DEG \
AUTOAIM_CHASSIS_SPIN_VEL=$AUTOAIM_CHASSIS_SPIN_VEL \
./start_autoaim_mode.sh" \
    >"$LOG_AUTOAIM" 2>&1 &
fi

echo "[autostart] Done. Logs: $LOG_DIR"
