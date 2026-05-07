#!/usr/bin/env bash
# 用法:
#   ./start_go2_nav2_localization.sh                                # 自动使用 maps/ 下最新地图和 output/ 下最新 PCD
#   ./start_go2_nav2_localization.sh maps/my_map.yaml               # 指定 2D 地图，自动使用最新 PCD
#   ./start_go2_nav2_localization.sh maps/my_map.yaml output/a.pcd  # 同时指定 2D 地图和 PCD

set -eo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_SETUP="/opt/ros/jazzy/setup.bash"
WS_SETUP="$ROOT_DIR/install/setup.bash"
RVIZ_CONFIG="/opt/ros/jazzy/share/nav2_bringup/rviz/nav2_default_view.rviz"
LAUNCH_PID=""

cleanup() {
  if [[ -n "${LAUNCH_PID}" ]] && kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    kill "${LAUNCH_PID}" 2>/dev/null || true
    wait "${LAUNCH_PID}" 2>/dev/null || true
  fi
}

trap cleanup EXIT INT TERM

configure_cyclonedds_participant_limit() {
  local max_auto_participant_index=100

  if [[ "${RMW_IMPLEMENTATION:-}" != "rmw_cyclonedds_cpp" && -z "${CYCLONEDDS_URI:-}" ]]; then
    return
  fi

  if [[ -z "${CYCLONEDDS_URI:-}" ]]; then
    export CYCLONEDDS_URI="<CycloneDDS><Domain><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>${max_auto_participant_index}</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>"
    return
  fi

  if [[ "${CYCLONEDDS_URI}" == *"MaxAutoParticipantIndex"* ]]; then
    return
  fi

  if [[ "${CYCLONEDDS_URI}" == "<CycloneDDS"* && "${CYCLONEDDS_URI}" == *"</Discovery>"* ]]; then
    local discovery_close="</Discovery>"
    local discovery_with_limit="<ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>${max_auto_participant_index}</MaxAutoParticipantIndex></Discovery>"
    export CYCLONEDDS_URI="${CYCLONEDDS_URI/$discovery_close/$discovery_with_limit}"
    echo "Configured CycloneDDS MaxAutoParticipantIndex=${max_auto_participant_index}"
    return
  fi

  echo "Warning: CYCLONEDDS_URI is set but could not be patched automatically. If nodes fail with participant index errors, add MaxAutoParticipantIndex=${max_auto_participant_index} to the CycloneDDS config." >&2
}

if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "Missing ROS setup: ${ROS_SETUP}" >&2
  exit 1
fi

if [[ ! -f "${WS_SETUP}" ]]; then
  echo "Missing workspace setup: ${WS_SETUP}" >&2
  echo "Build the workspace first: colcon build" >&2
  exit 1
fi

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
configure_cyclonedds_participant_limit

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
