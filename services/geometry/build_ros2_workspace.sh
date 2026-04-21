#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
WS_DIR="${REPO_ROOT}/services/geometry/ros2_ws"

source_without_nounset() {
  local target="$1"
  local had_nounset=0
  if [[ $- == *u* ]]; then
    had_nounset=1
    set +u
  fi
  # shellcheck disable=SC1090
  source "${target}"
  if [[ "${had_nounset}" -eq 1 ]]; then
    set -u
  fi
}

detect_ros_setup() {
  local candidates=(
    "/opt/ros/humble/setup.bash"
    "/opt/ros/jazzy/setup.bash"
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
if [[ -z "${ROS_SETUP}" ]]; then
  echo "未找到 ROS2 安装，请先安装 ROS2 Humble 或兼容发行版。" >&2
  exit 1
fi

source_without_nounset "${ROS_SETUP}"
cd "${WS_DIR}"

DEFAULT_BUILD_PACKAGES=(
  elevation_mapping_cupy
)

BUILD_PACKAGES=("${DEFAULT_BUILD_PACKAGES[@]}")
if [[ -n "${NUWAX_GEOMETRY_ROS2_BUILD_PACKAGES:-}" ]]; then
  read -r -a BUILD_PACKAGES <<<"${NUWAX_GEOMETRY_ROS2_BUILD_PACKAGES}"
fi

echo "使用 ROS2 环境: ${ROS_SETUP}"
echo "开始构建平台几何侧车工作空间: ${WS_DIR}"
echo "构建包集合: ${BUILD_PACKAGES[*]}"

colcon build \
  --symlink-install \
  --base-paths src \
  --packages-up-to "${BUILD_PACKAGES[@]}"

echo "构建完成。请执行: source ${WS_DIR}/install/setup.bash"
