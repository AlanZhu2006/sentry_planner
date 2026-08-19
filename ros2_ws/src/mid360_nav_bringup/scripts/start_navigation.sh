#!/usr/bin/env bash
set -euo pipefail

workspace_root="${ROBOT_WS:-${HOME}/robot_ws}"
maps_dir="${MID360_MAPS_DIR:-$workspace_root/src/mid360_nav_bringup/maps}"
ros_setup="${ROS_SETUP:-/opt/ros/${ROS_DISTRO:-humble}/setup.bash}"
stack_lock="${MID360_STACK_LOCK:-/tmp/nyu_mid360_mapping_navigation.lock}"

# Mapping and navigation own the MID-360, FAST-LIO frames and chassis command
# path exclusively.  Never allow a second stack to create duplicate drivers,
# odometry publishers or map->odom transforms.
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

# Navigation must own the bridge so losing a standalone WASD terminal cannot
# silently remove the controller velocity feedback halfway through a route.
node_list="$(timeout 3 ros2 node list --no-daemon --spin-time 0.2 2>/dev/null || true)"
if grep -Eq '^/(sentry_nav2_bridge|twist_mux)$' <<<"$node_list"; then
  echo "Error: a standalone chassis bridge or twist_mux is already running." >&2
  echo "Stop the current 'just wasd' terminal, start navigation, then run 'just wasd' again." >&2
  echo "The restarted WASD controller will reuse Nav2's bridge and retain higher priority." >&2
  exit 1
fi

map_yaml=""
pcd_map=""

if [ -n "${NAV_MAP_YAML:-}" ]; then
  map_yaml="$(realpath -m "$NAV_MAP_YAML")"
  pcd_map="$(realpath -m "${NAV_PCD_MAP:-${map_yaml%.yaml}.pcd}")"
elif [ -n "${NAV_PCD_MAP:-}" ]; then
  echo "Error: NAV_PCD_MAP requires NAV_MAP_YAML." >&2
  exit 2
elif [ "$#" -gt 0 ] && [[ "$1" != -* ]] && [[ "$1" != *":="* ]]; then
  map_selection="$1"
  shift
  if [[ "$map_selection" == *.yaml ]] || [[ "$map_selection" == */* ]]; then
    map_yaml="$(realpath -m "$map_selection")"
    pcd_map="${map_yaml%.yaml}.pcd"
  else
    if [[ ! "$map_selection" =~ ^[A-Za-z0-9._-]+$ ]]; then
      echo "Error: invalid map name: $map_selection" >&2
      exit 2
    fi
    map_yaml="$maps_dir/$map_selection.yaml"
    pcd_map="$maps_dir/$map_selection.pcd"
  fi
else
  echo "Error: a map name is required (example: just nav arena)." >&2
  echo "Alternatively set NAV_MAP_YAML and optionally NAV_PCD_MAP." >&2
  exit 2
fi

if [ ! -f "$map_yaml" ]; then
  echo "Error: Nav2 map YAML not found: $map_yaml" >&2
  exit 1
fi
if [ ! -f "$pcd_map" ]; then
  echo "Error: ICP PCD map not found: $pcd_map" >&2
  echo "The YAML/PGM and PCD must come from the same mapping coordinate frame." >&2
  exit 1
fi

echo ">>> Starting MID-360 + FAST-LIO + ICP localization + Nav2"
echo "    Nav2 map: $map_yaml"
echo "    ICP map:  $pcd_map"
echo "    RViz display: ${DISPLAY:-not set}"
echo "    Pose/TF: FAST-LIO only (no wheel/IMU localization fallback)"
echo "    Controller velocity feedback: FAST-LIO /odom"
echo "    /wheel/odom: diagnostics only until dynamic validation passes"
echo "    Keep the robot still while FAST-LIO initializes and full-map ICP runs."
echo "    ICP searches 1225 coarse poses across +/-2.25 m and all headings."
echo "    If the robot is farther away, seed it with RViz 2D Pose Estimate."

exec ros2 launch mid360_nav_bringup navigation_icp.launch.py \
  map:="$map_yaml" \
  pcd_map:="$pcd_map" \
  "$@"
