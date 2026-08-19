#!/usr/bin/env bash
set -euo pipefail

workspace_root="${ROBOT_WS:-${HOME}/robot_ws}"
ros_setup="${ROS_SETUP:-/opt/ros/${ROS_DISTRO:-humble}/setup.bash}"
bringup_pid=""
bringup_log=""

source_relaxed() {
  local setup_file="$1"
  local restore_nounset=0
  case $- in
    *u*) restore_nounset=1; set +u ;;
  esac
  # shellcheck disable=SC1090
  source "$setup_file"
  if [ "$restore_nounset" -eq 1 ]; then
    set -u
  fi
}

cleanup() {
  if [ -n "$bringup_pid" ] && kill -0 "$bringup_pid" 2>/dev/null; then
    kill -INT "$bringup_pid" 2>/dev/null || true
    for _ in {1..20}; do
      if ! kill -0 "$bringup_pid" 2>/dev/null; then
        break
      fi
      sleep 0.1
    done
    if kill -0 "$bringup_pid" 2>/dev/null; then
      kill -TERM "$bringup_pid" 2>/dev/null || true
    fi
    wait "$bringup_pid" 2>/dev/null || true
  fi
  if [ -n "$bringup_log" ] && [ -f "$bringup_log" ]; then
    rm -f "$bringup_log"
  fi
}

node_is_present() {
  local node_name="$1"
  local node_list="$2"
  grep -Fxq "$node_name" <<<"$node_list"
}

if [ ! -r "$ros_setup" ] || [ ! -r "$workspace_root/install/setup.bash" ]; then
  echo "Error: ROS 2 or workspace setup is missing. See ros2_ws/README.md." >&2
  exit 2
fi
source_relaxed "$ros_setup"
source_relaxed "$workspace_root/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"

node_list="$(timeout 3 ros2 node list --no-daemon --spin-time 0.2 2>/dev/null || true)"
start_bridge=true
start_mux=true
if node_is_present /sentry_nav2_bridge "$node_list"; then
  start_bridge=false
fi
if node_is_present /twist_mux "$node_list"; then
  start_mux=false
fi

if [ "$start_bridge" = false ] && [ "$start_mux" = false ]; then
  echo ">>> Reusing the running chassis bridge and twist_mux"
else
  bringup_log="$(mktemp --tmpdir="${XDG_RUNTIME_DIR:-/tmp}" nav_wasd_bringup.XXXXXX.log)"
  ros2 launch mid360_nav_bringup teleop_base.launch.py \
    start_bridge:="$start_bridge" \
    start_mux:="$start_mux" \
    >"$bringup_log" 2>&1 &
  bringup_pid=$!
  trap cleanup EXIT

  bridge_ready=false
  mux_ready=false
  for _ in {1..40}; do
    if ! kill -0 "$bringup_pid" 2>/dev/null; then
      echo "Error: standalone chassis stack exited during startup:" >&2
      sed -n '1,160p' "$bringup_log" >&2
      exit 1
    fi
    if [ "$start_bridge" = true ] && \
       grep -Eq 'sentry_nav2_bridge.*process has died' "$bringup_log"; then
      echo "Error: chassis bridge exited during startup:" >&2
      sed -n '1,160p' "$bringup_log" >&2
      exit 1
    fi
    node_list="$(timeout 2 ros2 node list --no-daemon --spin-time 0.2 2>/dev/null || true)"
    if node_is_present /sentry_nav2_bridge "$node_list"; then
      bridge_ready=true
    fi
    if node_is_present /twist_mux "$node_list"; then
      mux_ready=true
    fi
    if [ "$bridge_ready" = true ] && [ "$mux_ready" = true ]; then
      break
    fi
    sleep 0.25
  done

  if [ "$bridge_ready" != true ] || [ "$mux_ready" != true ]; then
    echo "Error: timed out waiting for the standalone chassis stack:" >&2
    sed -n '1,160p' "$bringup_log" >&2
    exit 1
  fi
  echo ">>> Started standalone chassis bridge and twist_mux"
fi

echo ">>> Browser WASD limits: 0.80 m/s translation, 2.40 rad/s rotation"
echo ">>> Keep this terminal open; press Ctrl-C here to stop the controller"
ros2 run mid360_nav_bringup wasd_web_teleop.py "$@"
