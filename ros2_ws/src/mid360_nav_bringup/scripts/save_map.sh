#!/usr/bin/env bash
set -euo pipefail

workspace_root="${ROBOT_WS:-${HOME}/robot_ws}"
maps_dir="${MID360_MAPS_DIR:-$workspace_root/src/mid360_nav_bringup/maps}"
ros_setup="${ROS_SETUP:-/opt/ros/${ROS_DISTRO:-humble}/setup.bash}"

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

if [ ! -r "$ros_setup" ] || [ ! -r "$workspace_root/install/setup.bash" ]; then
  echo "Error: ROS 2 or workspace setup is missing. See ros2_ws/README.md." >&2
  exit 2
fi
source_relaxed "$ros_setup"
source_relaxed "$workspace_root/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"

map_name="${FASTLIO_MAP_NAME:-fastlio_map}"
if [ "$#" -gt 0 ] && [[ "$1" != -* ]]; then
  map_name="$1"
  shift
fi
if [[ ! "$map_name" =~ ^[A-Za-z0-9._-]+$ ]]; then
  echo "Error: invalid map name: $map_name" >&2
  exit 1
fi

map_base="$maps_dir/$map_name"
mkdir -p "$maps_dir"
pcd_path="$map_base.pcd"
pcd2pgm_params="$workspace_root/src/mid360_nav_bringup/config/pcd2pgm.yaml"
map_topic="/pcd2pgm_map_save"

if [ "$#" -ne 0 ]; then
  echo "Error: unexpected arguments: $*" >&2
  echo "pcd2pgm parameters are configured in $pcd2pgm_params" >&2
  exit 2
fi

if ! ros2 service list 2>/dev/null | grep -qx /map_save; then
  echo "Error: FAST-LIO /map_save service is not available." >&2
  echo "Start mapping first with: just map $map_name" >&2
  exit 1
fi
pcd_save_state="$(ros2 param get /laser_mapping pcd_save.pcd_save_en 2>/dev/null || true)"
if [[ "$pcd_save_state" != *"True"* ]]; then
  echo "Error: FAST-LIO is running with PCD saving disabled." >&2
  echo "Stop navigation and start mapping first with: just map $map_name" >&2
  exit 1
fi

echo ">>> Saving FAST-LIO PCD"
map_save_response="$(ros2 service call /map_save std_srvs/srv/Trigger '{}')"
printf '%s\n' "$map_save_response"
if [[ "$map_save_response" != *"success=True"* ]]; then
  echo "Error: FAST-LIO rejected the PCD save request." >&2
  exit 1
fi
if [ ! -s "$pcd_path" ]; then
  echo "Error: FAST-LIO did not create $pcd_path" >&2
  exit 1
fi

if ! ros2 pkg prefix pcd2pgm >/dev/null 2>&1; then
  echo "Error: upstream pcd2pgm is not installed in $workspace_root/install." >&2
  echo "Build $workspace_root/src/pcd2pgm before saving a map." >&2
  exit 1
fi
if [ ! -r "$pcd2pgm_params" ]; then
  echo "Error: pcd2pgm parameter file is missing: $pcd2pgm_params" >&2
  exit 1
fi

pcd2pgm_pid=""
stop_pcd2pgm() {
  if [ -n "$pcd2pgm_pid" ] && kill -0 "$pcd2pgm_pid" 2>/dev/null; then
    kill "$pcd2pgm_pid" 2>/dev/null || true
    wait "$pcd2pgm_pid" 2>/dev/null || true
  fi
}
trap stop_pcd2pgm EXIT INT TERM

echo ">>> Filtering PCD and publishing the occupancy grid with upstream pcd2pgm"
ros2 run pcd2pgm pcd2pgm_node --ros-args \
  --params-file "$pcd2pgm_params" \
  -p pcd_file:="$pcd_path" \
  -p map_topic_name:="$map_topic" &
pcd2pgm_pid=$!

# pcd2pgm performs its PCL filters during node construction and publishes the
# resulting transient-local map once per second. Wait for the first complete
# map so map_saver_cli never races node startup on a large PCD.
if ! timeout 60 ros2 topic echo "$map_topic" nav_msgs/msg/OccupancyGrid \
    --once \
    --qos-reliability reliable \
    --qos-durability transient_local >/dev/null; then
  if ! kill -0 "$pcd2pgm_pid" 2>/dev/null; then
    wait "$pcd2pgm_pid" || true
  fi
  echo "Error: pcd2pgm did not publish $map_topic within 60 seconds." >&2
  exit 1
fi

echo ">>> Saving the filtered occupancy grid as Nav2 PGM/YAML"
ros2 run nav2_map_server map_saver_cli \
  -t "$map_topic" \
  -f "$map_base" \
  --fmt pgm \
  --mode trinary \
  --ros-args -p save_map_timeout:=30.0

stop_pcd2pgm
pcd2pgm_pid=""
trap - EXIT INT TERM

if [ ! -s "$map_base.pgm" ] || [ ! -s "$map_base.yaml" ]; then
  echo "Error: map_saver_cli did not create $map_base.pgm and $map_base.yaml" >&2
  exit 1
fi

echo ">>> Map set is ready"
echo "    $pcd_path"
echo "    $map_base.pgm"
echo "    $map_base.yaml"
echo "Navigate with: just nav $map_name"
