#!/usr/bin/env bash
# DEPRECATED COPY — 正式入口请使用仓库顶层：
#   services/geometry/ros2_ws/src/GO2_RTABNAV2/go2.sh
#
# Unitree GO2: mapping / localization / exploration launcher.
#
# Usage:
#   ./go2.sh --mapping [map.db]        # SLAM + Nav2 + 2D goal (default: ~/.ros/rtabmap.db)
#   ./go2.sh --mapping-only [map.db]   # SLAM only, faster (no Nav2 / obstacle_grid)
#   ./go2.sh --localization <map.db>   # Load map, localize only, Nav2 (db required, read-only source)
#   ./go2.sh --explore [map.db]        # SLAM + Nav2 + m-explore (default: ~/.ros/rtabmap.db)
#   ./go2.sh --fresh --mapping         # Force restart_map:=true (new SLAM session)
#
# Environment overrides (optional):
#   GO2_WS=~/huang_grok/ws          Workspace overlay path
#   GO2_ROBOT_IFACE=enx00e04c123598  Unitree Ethernet interface (auto-detected if unset)
#   GO2_SKIP_KILL=1                  Skip killing stale ROS nodes before launch
#   GO2_CLOUD_MODE=deskewed          Use /utlidar/cloud_deskewed (default, motion-compensated)
#   GO2_CLOUD_MODE=base              Fallback: /utlidar/cloud_base passthrough

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MODE=""
DB_ARG=""
FRESH_MAP="false"

if [[ -z "${GO2_ROS_DISTRO:-}" ]]; then
  if [[ -f /opt/ros/foxy/setup.bash ]]; then
    GO2_ROS_DISTRO=foxy
  elif [[ -f /opt/ros/jazzy/setup.bash ]]; then
    GO2_ROS_DISTRO=jazzy
  else
    GO2_ROS_DISTRO="${ROS_DISTRO:-foxy}"
  fi
fi
export GO2_ROS_DISTRO
GO2_WS="${GO2_WS:-${SCRIPT_DIR}/../../ws}"
GO2_WS="$(cd "${GO2_WS}" && pwd)"
if [[ -z "${GO2_UNITREE_SETUP:-}" ]]; then
  for _u in \
    "${HOME}/unitree_ros2/install/setup.bash" \
    "${HOME}/unitree_ros2/cyclonedds_ws/install/setup.bash" \
    "${HOME}/cyclonedds_ws/install/setup.bash"; do
    if [[ -f "${_u}" ]]; then
      GO2_UNITREE_SETUP="${_u}"
      break
    fi
  done
fi
export GO2_UNITREE_SETUP="${GO2_UNITREE_SETUP:-}"

usage() {
  sed -n '2,15p' "$0" | sed 's/^# \{0,1\}//'
}

die() {
  echo "go2.sh: $*" >&2
  exit 1
}

warn() {
  echo "go2.sh: warning: $*" >&2
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --fresh)
      FRESH_MAP="true"
      shift
      ;;
    --mapping|--mapping-only|--localization|--explore)
      [[ -z "$MODE" ]] || die "only one mode flag allowed"
      MODE="${1#--}"
      shift
      if [[ $# -gt 0 && "$1" != --* ]]; then
        DB_ARG="$1"
        shift
      fi
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      die "unknown argument: $1 (try --help)"
      ;;
  esac
done

[[ -n "$MODE" ]] || die "missing mode: --mapping | --mapping-only | --localization | --explore"

if [[ "$MODE" == "localization" ]]; then
  [[ -n "$DB_ARG" ]] || die "--localization requires a database path"
fi

DEFAULT_DB="${HOME}/.ros/rtabmap.db"
TARGET_DB="${DB_ARG:-$DEFAULT_DB}"

if [[ "$TARGET_DB" != /* ]]; then
  TARGET_DB="$(pwd)/${TARGET_DB}"
fi
mkdir -p "$(dirname "$TARGET_DB")"

find_source_go2() {
  local candidate
  for candidate in \
    "${GO2_WS}/source_go2.bash" \
    "${SCRIPT_DIR}/source_go2.bash" \
    "${HOME}/huang_grok/ws/source_go2.bash"; do
    if [[ -f "$candidate" ]]; then
      echo "$candidate"
      return 0
    fi
  done
  return 1
}

detect_robot_iface() {
  if [[ -n "${GO2_ROBOT_IFACE:-}" ]]; then
    echo "${GO2_ROBOT_IFACE}"
    return 0
  fi
  local iface
  iface="$(ip -4 -br addr 2>/dev/null | awk '$3 ~ /^192\.168\.123\./ {print $1; exit}')"
  if [[ -n "$iface" ]]; then
    echo "$iface"
    return 0
  fi
  iface="$(ip -br link 2>/dev/null | awk '/^enx/ && $2=="UP" {print $1; exit}')"
  if [[ -n "$iface" ]]; then
    echo "$iface"
    return 0
  fi
  return 1
}

setup_cyclonedds() {
  local iface="$1"
  export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
  export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"${iface}\" priority=\"default\" multicast=\"default\" /></Interfaces></General></Domain></CycloneDDS>"
}

check_iface_up() {
  local iface="$1"
  ip -br link show "$iface" 2>/dev/null | grep -q '\<UP\>'
}

preflight_network() {
  local iface
  if ! iface="$(detect_robot_iface)"; then
    warn "no Unitree robot Ethernet detected (192.168.123.x / enx*)."
    warn "Connect GO2 and set GO2_ROBOT_IFACE if needed."
    return 0
  fi
  if ! check_iface_up "$iface"; then
    die "robot interface ${iface} is not UP — check USB/Ethernet cable"
  fi
  setup_cyclonedds "$iface"
  echo "==> DDS interface: ${iface}"
}

source_ros() {
  local source_script
  if ! source_script="$(find_source_go2)"; then
    die "source_go2.bash not found — expected under ${GO2_WS}/source_go2.bash"
  fi

  set +u
  # shellcheck disable=SC1090
  source "$source_script" || die "failed to source ${source_script}"
  set -u

  export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
  preflight_network

  echo "==> ROS: ${GO2_ROS_DISTRO}"
  echo "==> Workspace: ${GO2_WS}"
  echo "==> RMW: ${RMW_IMPLEMENTATION:-unknown}"
}

kill_other_go2_shells() {
  local pid
  while read -r pid; do
    [[ -z "$pid" || "$pid" == "$$" || "$pid" == "$PPID" ]] && continue
    kill -9 "$pid" 2>/dev/null || true
  done < <(pgrep -f '[/ ]go2\.sh' 2>/dev/null || true)
}

kill_stale() {
  [[ "${GO2_SKIP_KILL:-0}" == "1" ]] && return 0
  echo "==> Stopping stale GO2/ROS nodes..."
  kill_other_go2_shells
  local patterns=(
    'ros2 launch go2_slam_nav'
    'lib/rtabmap_slam/rtabmap'
    'lib/rtabmap_odom/icp_odometry'
    'lib/go2_slam_nav/sensor_stamp_relay'
    'lib/go2_slam_nav/obstacle_grid_projector'
    'lib/go2_slam_nav/nav_loop_guard'
    'rtabmap_progress_monitor'
    'nav_progress_monitor'
    'nav_loop_guard'
    'map_robot_pose_publisher'
    'lib/go2_slam_nav/go2_tf_relay'
    'lib/go2_cmd_processor/sport_ctrl'
    'explore_node'
    'explore_preroll'
    'lib/nav2_bt_navigator/bt_navigator'
    'lib/nav2_controller/controller_server'
    'lib/nav2_planner/planner_server'
    'lib/nav2_recoveries/recoveries_server'
    'lib/nav2_waypoint_follower/waypoint_follower'
    'nav2_container'
    'component_container_isolated'
    'lifecycle_manager'
    'rviz2'
  )
  local pattern pid
  for pattern in "${patterns[@]}"; do
    while read -r pid; do
      [[ -z "$pid" || "$pid" == "$$" || "$pid" == "$PPID" ]] && continue
      kill -9 "$pid" 2>/dev/null || true
    done < <(pgrep -f "$pattern" 2>/dev/null || true)
  done
  sleep 1
}

setup_database() {
  local restart_map="false"
  local localize_only="false"
  local active_db=""

  case "$MODE" in
    mapping|mapping-only|explore)
      if [[ "$FRESH_MAP" == "true" ]]; then
        restart_map="true"
        echo "==> Fresh map session (restart_map=true): ${TARGET_DB}"
      elif [[ -f "$TARGET_DB" ]]; then
        restart_map="false"
        echo "==> Using existing map DB: ${TARGET_DB}"
      else
        restart_map="true"
        echo "==> Creating new map DB: ${TARGET_DB}"
      fi
      active_db="$TARGET_DB"
      localize_only="false"
      if [[ "$MODE" == "explore" ]]; then
        echo "==> Global LiDAR relocalization enabled (ProximityGlobalScanMap)."
        echo "    Drive slowly through mapped areas until /rtabmap/info proximity_detection_id > 0."
      fi
      ;;
    localization)
      [[ -f "$TARGET_DB" ]] || die "database not found: ${TARGET_DB}"
      active_db="${HOME}/.ros/rtabmap_localization_session.db"
      local target_real="" session_real=""
      target_real="$(realpath "$TARGET_DB")"
      if [[ -f "$active_db" ]]; then
        session_real="$(realpath "$active_db")"
      fi
      restart_map="false"
      localize_only="true"
      if [[ -n "$session_real" && "$target_real" == "$session_real" ]]; then
        echo "==> Localization session DB: ${active_db}"
        echo "    (same as ${TARGET_DB}; skip copy)"
      else
        cp -f "$TARGET_DB" "$active_db"
        echo "==> Localization from: ${TARGET_DB}"
        echo "    (session copy: ${active_db}; source DB will not be written)"
      fi
      echo "==> Global LiDAR relocalization enabled (ProximityGlobalScanMap)."
      echo "    Drive slowly through mapped areas until /rtabmap/info proximity_detection_id > 0."
      ;;
  esac

  mkdir -p "${HOME}/.ros"
  local default_db="${HOME}/.ros/rtabmap.db"
  local active_real default_real
  active_real="$(realpath -m "$active_db")"
  default_real="$(realpath -m "$default_db")"
  if [[ "$active_real" != "$default_real" ]]; then
    ln -sfn "$active_real" "$default_db"
    echo "==> RTAB-Map DB link: ${default_db} -> ${active_real}"
  else
    echo "==> RTAB-Map DB: ${default_real}"
  fi

  export GO2_RESTART_MAP="$restart_map"
  export GO2_LOCALIZE_ONLY="$localize_only"
  export GO2_ACTIVE_DB="$active_db"
  export GO2_SOURCE_DB="$TARGET_DB"
}

launch_stack() {
  local launch_file args=()

  case "$MODE" in
    mapping|localization)
      launch_file="nav_go2.launch.py"
      ;;
    mapping-only)
      launch_file="mapping_go2.launch.py"
      ;;
    explore)
      launch_file="explore_go2.launch.py"
      ;;
  esac

  args=(
    "localize_only:=${GO2_LOCALIZE_ONLY}"
    "restart_map:=${GO2_RESTART_MAP}"
  )
  if [[ "$MODE" == "mapping-only" ]]; then
    args+=("use_obstacle_grid:=false")
  fi

  if [[ "$launch_file" == "nav_go2.launch.py" ]]; then
    args+=("nav2_startup_delay:=10.0")
  fi
  if [[ "$launch_file" == "explore_go2.launch.py" ]]; then
    args+=("nav2_startup_delay:=15.0")
  fi

  case "${GO2_CLOUD_MODE:-deskewed}" in
    base)
      args+=(
        "cloud_in:=/utlidar/cloud_base"
        "cloud_frame_mode:=base"
      )
      ;;
    deskewed|*)
      args+=(
        "cloud_in:=/utlidar/cloud_deskewed"
        "cloud_frame_mode:=deskewed"
      )
      ;;
  esac

  echo "==> Mode: ${MODE}"
  echo "==> Launch: go2_slam_nav/${launch_file}"
  echo "    localize_only=${GO2_LOCALIZE_ONLY}  restart_map=${GO2_RESTART_MAP}"
  echo "    active db: ${GO2_ACTIVE_DB}"

  exec ros2 launch go2_slam_nav "$launch_file" "${args[@]}"
}

kill_stale
source_ros
setup_database
launch_stack