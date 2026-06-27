#!/usr/bin/env bash
# Unitree GO2: mapping / localization / exploration launcher.
#
# Usage:
#   ./go2.sh --mapping [map.db]        # SLAM + Nav2 + 2D goal (default: NUWAX_RTABMAP_DATABASE_ROOT/live_rtabmap.db)
#   ./go2.sh --mapping-only [map.db]   # SLAM only, faster (no Nav2 / obstacle_grid)
#   ./go2.sh --localization <map.db>   # Load map, localize only, Nav2 (db required, read-only source)
#   ./go2.sh --explore [map.db]        # SLAM + Nav2 + m-explore (default: ~/.ros/rtabmap.db)
#   ./go2.sh --fresh --mapping         # Force restart_map:=true (new SLAM session)
#
# Environment overrides (optional):
#   GO2_WS=~/GO2_RTABNAV2/ws          Workspace overlay path
#   GO2_ROBOT_IFACE=enx00e04c123598  Unitree Ethernet interface (auto-detected if unset)
#   GO2_SKIP_KILL=1                  Skip killing stale ROS nodes before launch
#   GO2_USE_RVIZ=0                   Headless (no rviz2; saves ~300–500MB RAM on Jetson)
#   GO2_CLOUD_MODE=deskewed          Use /utlidar/cloud_deskewed (default, motion-compensated)
#   GO2_CLOUD_MODE=base              Fallback: /utlidar/cloud_base passthrough
#
# Native test: stop nuwax gateway first (stop_mapping / Ctrl+C start_gateway.sh) to avoid dual-stack OOM.
#
# Examples:
#   ./go2.sh --mapping
#   ./go2.sh --mapping ~/maps/room1.db
#   ./go2.sh --localization ~/maps/room1.db
#   ./go2.sh --explore ~/maps/room1.db

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
GO2_WS="${GO2_WS:-${SCRIPT_DIR}/ws}"
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
  sed -n '2,20p' "$0" | sed 's/^# \{0,1\}//'
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

source_platform_env() {
  set +u
  # shellcheck disable=SC1091
  source "${SCRIPT_DIR}/scripts/env.sh"
  set -u
}

resolve_default_db() {
  local root="${NUWAX_RTABMAP_DATABASE_ROOT:-${NUWAX_BACKEND_DATABASE_ROOT:-}}"
  if [[ -n "$root" ]]; then
    mkdir -p "$root"
    echo "${root%/}/live_rtabmap.db"
  else
    echo "${HOME}/.ros/rtabmap.db"
  fi
}

source_platform_env
DEFAULT_DB="$(resolve_default_db)"
TARGET_DB="${DB_ARG:-$DEFAULT_DB}"

if [[ "$TARGET_DB" != /* ]]; then
  TARGET_DB="$(pwd)/${TARGET_DB}"
fi
mkdir -p "$(dirname "$TARGET_DB")"

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

assert_ros_ready() {
  export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
  preflight_network
  echo "==> ROS: ${GO2_ROS_DISTRO}"
  echo "==> Workspace: ${GO2_WS}"
  echo "==> RMW: ${RMW_IMPLEMENTATION:-unknown}"
  echo "==> DB root: ${NUWAX_RTABMAP_DATABASE_ROOT:-${NUWAX_BACKEND_DATABASE_ROOT:-(unset)}}"
  echo "==> obstacle_grid: live_cloud=${GO2_OBSTACLE_GRID_LIVE_CLOUD_ENABLED:-false} publish_on_cloud=${GO2_OBSTACLE_GRID_PUBLISH_ON_CLOUD:-false}"
}

kill_other_go2_shells() {
  # 等价于「pkill -f go2.sh」，但必须排除当前脚本 ($$)，否则会自杀。
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
  # Do NOT match bare 'rtabmap' — it hits go2.sh argv (e.g. .../rtabmap.db).
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

preflight_memory() {
  local avail_kb
  avail_kb="$(awk '/^MemAvailable:/ {print $2}' /proc/meminfo 2>/dev/null || echo 0)"
  if [[ -z "$avail_kb" || "$avail_kb" -lt 1 ]]; then
    return 0
  fi
  local avail_mb=$((avail_kb / 1024))
  echo "==> MemAvailable: ${avail_mb} MB"
  if [[ "$avail_mb" -lt 600 ]]; then
    die "可用内存仅 ${avail_mb} MB，启动 rviz2+RTAB+Nav2 极易被 OOM Killed。" \
      "请先停止 nuwax gateway（stop_mapping），或 GO2_USE_RVIZ=0 ./go2.sh ..."
  fi
  if [[ "$avail_mb" -lt 1200 && "${GO2_USE_RVIZ:-1}" != "0" ]]; then
    warn "可用内存 ${avail_mb} MB 偏低；建议 GO2_USE_RVIZ=0 关闭 RViz 避免 Killed。"
  fi
}

preflight_conflicting_stack() {
  if pgrep -f 'start_gateway\.sh|nuwax_robot_bridge.*gateway' >/dev/null 2>&1; then
    warn "检测到 nuwax gateway 可能在运行；其与原生 go2.sh 会争抢内存/ROS 栈。"
    warn "原生测试前请先 stop_mapping 或停止 start_gateway.sh，否则易出现 Killed / 双栈。"
  fi
  if pgrep -f 'ros2 launch go2_slam_nav' >/dev/null 2>&1; then
    warn "仍有 go2_slam_nav launch 存活；执行二次清理..."
    kill_stale
    sleep 1
    if pgrep -f 'ros2 launch go2_slam_nav' >/dev/null 2>&1; then
      die "无法清理已有 go2_slam_nav（可能被 gateway 自动拉起）。请先停止 gateway 再测。"
    fi
  fi
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
      active_db="$TARGET_DB"
      restart_map="false"
      localize_only="true"
      echo "==> Localization (read-only source DB): ${active_db}"
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
  case "$MODE" in
    localization) args+=("slam_mode:=localization") ;;
    explore) args+=("slam_mode:=explore") ;;
    *) args+=("slam_mode:=mapping") ;;
  esac
  if [[ "$MODE" == "mapping-only" ]]; then
    args+=("use_obstacle_grid:=false")
  fi

  if [[ "$launch_file" == "nav_go2.launch.py" ]]; then
    args+=("nav2_startup_delay:=15.0")
  fi
  if [[ "$launch_file" == "explore_go2.launch.py" ]]; then
    args+=("nav2_startup_delay:=20.0")
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

  if [[ "${GO2_USE_RVIZ:-1}" == "0" ]]; then
    args+=("use_rviz:=false")
    echo "==> RViz disabled (GO2_USE_RVIZ=0)"
  fi

  echo "==> Mode: ${MODE}"
  echo "==> Launch: go2_slam_nav/${launch_file}"
  echo "    localize_only=${GO2_LOCALIZE_ONLY}  restart_map=${GO2_RESTART_MAP}"
  echo "    active db: ${GO2_ACTIVE_DB}"

  exec ros2 launch go2_slam_nav "$launch_file" "${args[@]}"
}

kill_stale
preflight_conflicting_stack
preflight_memory
assert_ros_ready
setup_database
launch_stack