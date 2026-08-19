#!/usr/bin/env bash
set -euo pipefail

workspace_root="${ROBOT_WS:-${HOME}/robot_ws}"
maps_dir="${MID360_MAPS_DIR:-$workspace_root/src/mid360_nav_bringup/maps}"
ros_setup="${ROS_SETUP:-/opt/ros/${ROS_DISTRO:-humble}/setup.bash}"
stack_lock="${MID360_STACK_LOCK:-/tmp/nyu_mid360_mapping_navigation.lock}"

# The MID-360/FAST-LIO/TF chain is single-owner.  Sharing this lock with the
# navigation launcher prevents duplicate drivers and coordinate-frame trees.
exec 9>"$stack_lock"
if ! flock -n 9; then
  echo "Error: another MID-360 mapping/navigation stack is already running." >&2
  echo "Stop its terminal with Ctrl-C before starting a new stack." >&2
  exit 1
fi

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

configure_rviz_display() {
  if [ -n "${NAV_RVIZ_DISPLAY:-}" ]; then
    export DISPLAY="$NAV_RVIZ_DISPLAY"
    if [[ "$DISPLAY" == :* ]]; then
      export XAUTHORITY="${NAV_RVIZ_XAUTHORITY:-${HOME}/.Xauthority}"
    fi
    return
  fi

  if [ -n "${SSH_CONNECTION:-}" ]; then
    if [ -S /tmp/.X11-unix/X1 ]; then
      export DISPLAY=:1
      export XAUTHORITY="${HOME}/.Xauthority"
    else
      echo "Warning: SSH detected but VNC display :1 is unavailable; RViz may not start." >&2
    fi
  fi
}

if [ ! -r "$ros_setup" ] || [ ! -r "$workspace_root/install/setup.bash" ]; then
  echo "Error: ROS 2 or workspace setup is missing. See ros2_ws/README.md." >&2
  exit 2
fi
source_relaxed "$ros_setup"
source_relaxed "$workspace_root/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
configure_rviz_display

node_list="$(timeout 3 ros2 node list --no-daemon --spin-time 0.2 2>/dev/null || true)"
if grep -Eq '^/(sentry_nav2_bridge|twist_mux)$' <<<"$node_list"; then
  echo "Error: a standalone chassis bridge or twist_mux is already running." >&2
  echo "Stop the current 'just wasd' terminal, start mapping, then run 'just wasd' again." >&2
  echo "The restarted WASD controller will reuse the mapping stack's bridge." >&2
  exit 1
fi

map_name="${FASTLIO_MAP_NAME:-fastlio_map}"
if [ "$#" -gt 0 ] && [[ "$1" != -* ]] && [[ "$1" != *":="* ]]; then
  map_name="$1"
  shift
fi
if [[ ! "$map_name" =~ ^[A-Za-z0-9._-]+$ ]]; then
  echo "Error: map name may contain only letters, digits, dot, underscore and dash." >&2
  exit 1
fi

pcd_path="$maps_dir/$map_name.pcd"
mkdir -p "$maps_dir"
echo ">>> Starting MID-360 + FAST-LIO mapping"
echo "    PCD output: $pcd_path"
echo "    RViz display: ${DISPLAY:-not set}"
echo "    Drive with /cmd_vel_teleop; Nav2 is not active in mapping mode."
echo "    When finished, run in another terminal: just map-save $map_name"

exec ros2 launch mid360_nav_bringup fastlio_mapping.launch.py \
  pcd_path:="$pcd_path" \
  "$@"
