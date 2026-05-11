#!/usr/bin/env bash

set -eo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MAP_DIR="${ROOT_DIR}/maps"

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

ROS_SETUP="$(detect_ros_setup || true)"
if [[ -z "${ROS_SETUP}" || ! -f "${ROS_SETUP}" ]]; then
  echo "Missing ROS setup. Set NUWAX_ROS_SETUP_SCRIPT or install ROS2 Jazzy/Humble/Iron/Foxy." >&2
  exit 1
fi
if [[ ! -f "${ROOT_DIR}/install/setup.bash" ]]; then
  echo "Missing workspace setup: ${ROOT_DIR}/install/setup.bash" >&2
  echo "Build the workspace first: colcon build --symlink-install" >&2
  exit 1
fi
source "${ROS_SETUP}"
source "${ROOT_DIR}/install/setup.bash"

mkdir -p "${MAP_DIR}"

echo "Saving /map to ${MAP_DIR}/map.yaml and ${MAP_DIR}/map.pgm"

ros2 run nav2_map_server map_saver_cli -f "${MAP_DIR}/map"
