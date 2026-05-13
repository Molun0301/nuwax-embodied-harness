#!/usr/bin/env bash
# 用法:
#   ./start_go2_nav2_localization.sh                                # 自动使用 maps/ 下最新地图和 output/ 下最新 PCD
#   ./start_go2_nav2_localization.sh maps/my_map.yaml               # 指定 2D 地图，自动使用最新 PCD
#   ./start_go2_nav2_localization.sh maps/my_map.yaml output/a.pcd  # 同时指定 2D 地图和 PCD

set -eo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_SETUP="$ROOT_DIR/install/setup.bash"
GEOMETRY_RUNTIME_ENV_SCRIPT="$ROOT_DIR/../../../source_runtime_env.sh"
LAUNCH_PID=""

detect_ros_setup() {
  local candidates=()
  if [[ -n "${NUWAX_ROS_SETUP_SCRIPT:-}" ]]; then
    candidates+=("${NUWAX_ROS_SETUP_SCRIPT}")
  fi
  if [[ -n "${ROS_DISTRO:-}" ]]; then
    candidates+=("/opt/ros/${ROS_DISTRO}/setup.bash")
  fi
  candidates+=(
    "/opt/ros/jazzy/setup.bash"
    "/opt/ros/humble/setup.bash"
    "/opt/ros/iron/setup.bash"
    "/opt/ros/foxy/setup.bash"
  )
  local candidate
  for candidate in "${candidates[@]}"; do
    if [[ -f "${candidate}" ]]; then
      echo "${candidate}"
      return 0
    fi
  done
  return 1
}

resolve_nav2_rviz_config() {
  local ros_setup="$1"
  local ros_prefix="${ros_setup%/setup.bash}"
  echo "${ros_prefix}/share/nav2_bringup/rviz/nav2_default_view.rviz"
}

ROS_SETUP="$(detect_ros_setup || true)"
RVIZ_CONFIG=""

cleanup() {
  if [[ -n "${LAUNCH_PID}" ]] && kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    kill "${LAUNCH_PID}" 2>/dev/null || true
    wait "${LAUNCH_PID}" 2>/dev/null || true
  fi
}

trap cleanup EXIT INT TERM

workspace_stale_source() {
  local setup_file="$1"
  local source_file=""
  local search_roots=()
  [[ -d "${ROOT_DIR}/src" ]] && search_roots+=("${ROOT_DIR}/src")
  [[ -d "${ROOT_DIR}/m-explore-ros2" ]] && search_roots+=("${ROOT_DIR}/m-explore-ros2")
  if [[ "${#search_roots[@]}" -eq 0 ]]; then
    return 1
  fi
  while IFS= read -r -d '' source_file; do
    if [[ "${source_file}" -nt "${setup_file}" ]]; then
      echo "${source_file}"
      return 0
    fi
  done < <(find "${search_roots[@]}" -type f \
    \( -name 'package.xml' -o -name 'CMakeLists.txt' -o -name '*.launch.py' -o -name '*.py' -o -name '*.cpp' -o -name '*.hpp' -o -name '*.h' -o -name '*.yaml' -o -name '*.xml' \) \
    -print0 2>/dev/null)
  return 1
}

assert_workspace_current() {
  local setup_file="$1"
  local stale_source=""
  stale_source="$(workspace_stale_source "${setup_file}" || true)"
  if [[ -n "${stale_source}" ]]; then
    echo "Workspace install is older than source: ${stale_source}" >&2
    echo "Rebuild first: cd ${ROOT_DIR} && source ${ROS_SETUP} && colcon build --symlink-install" >&2
    exit 1
  fi
}

if [[ -z "${ROS_SETUP}" || ! -f "${ROS_SETUP}" ]]; then
  echo "Missing ROS setup. Set NUWAX_ROS_SETUP_SCRIPT or install ROS2 Jazzy/Humble/Iron/Foxy." >&2
  exit 1
fi
RVIZ_CONFIG="$(resolve_nav2_rviz_config "${ROS_SETUP}")"

if [[ ! -f "${WS_SETUP}" ]]; then
  echo "Missing workspace setup: ${WS_SETUP}" >&2
  echo "Build the workspace first: colcon build" >&2
  exit 1
fi

assert_workspace_current "${WS_SETUP}"

if [[ -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" ]]; then
  echo "No GUI session detected. RViz needs DISPLAY or WAYLAND_DISPLAY." >&2
  exit 1
fi

if [[ -n "${1:-}" ]]; then
  MAP_YAML="$1"
else
  MAP_YAML="$(ls -t "${ROOT_DIR}/maps/"*.yaml 2>/dev/null | head -n1)"
fi

if [[ -z "${MAP_YAML}" || ! -f "${MAP_YAML}" ]]; then
  echo "No map found. Save a map first, or specify one: $0 <path/to/map.yaml>" >&2
  exit 1
fi

if [[ -n "${2:-}" ]]; then
  PCD_MAP="$2"
else
  PCD_MAP="$(ls -t "${ROOT_DIR}/output/"*.pcd 2>/dev/null | head -n1)"
fi

PCD_INITIAL_POSE_ARGS=(use_pcd_initial_pose:=false)
if [[ -n "${PCD_MAP}" && -f "${PCD_MAP}" ]]; then
  PCD_INITIAL_POSE_ARGS=(
    use_pcd_initial_pose:=true
    pcd_map:="${PCD_MAP}"
    pcd_initial_pose_cloud_topic:=/utlidar/cloud_deskewed
    auto_spin_after_initial_pose:=true
  )
fi

source "${ROS_SETUP}"
source "${WS_SETUP}"
export NUWAX_GEOMETRY_RUNTIME_SOURCE_ROS_ENV=false
export NUWAX_GEOMETRY_RUNTIME_FORCE_CYCLONEDDS=true
source "${GEOMETRY_RUNTIME_ENV_SCRIPT}"

set -u

echo "Starting Go2 Nav2 with existing map: ${MAP_YAML}"
if [[ "${PCD_INITIAL_POSE_ARGS[0]}" == "use_pcd_initial_pose:=true" ]]; then
  echo "Using PCD map for automatic initial pose: ${PCD_MAP}"
else
  echo "No PCD map found. Use RViz 2D Pose Estimate if AMCL needs an initial pose."
fi
ros2 launch go2_navigation2 go2_nav2.launch.py localization:=amcl map:="${MAP_YAML}" use_rviz:=false "${PCD_INITIAL_POSE_ARGS[@]}" &
LAUNCH_PID=$!

sleep 4

if ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
  echo "Navigation launch exited before RViz started. Check the terminal output above." >&2
  exit 1
fi

echo "Starting RViz with Nav2 costmap view..."
rviz2 -d "${RVIZ_CONFIG}"
