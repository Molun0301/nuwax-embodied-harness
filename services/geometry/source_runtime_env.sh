#!/usr/bin/env bash

GEOMETRY_RUNTIME_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
GEOMETRY_RUNTIME_REPO_ROOT="$(cd "${GEOMETRY_RUNTIME_SCRIPT_DIR}/../.." && pwd)"
GEOMETRY_RUNTIME_WS_DIR="${GEOMETRY_RUNTIME_REPO_ROOT}/services/geometry/ros2_ws"
NUWAX_GEOMETRY_RUNTIME_SOURCE_ROS_ENV="${NUWAX_GEOMETRY_RUNTIME_SOURCE_ROS_ENV:-false}"

geometry_source_without_nounset() {
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

geometry_detect_ros_setup() {
  local candidates=()
  if [[ -n "${NUWAX_ROS_SETUP_SCRIPT:-}" ]]; then
    candidates+=("${NUWAX_ROS_SETUP_SCRIPT}")
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
      printf '%s' "${candidate}"
      return 0
    fi
  done
  return 1
}

geometry_has_cyclonedds_lib() {
  local candidate="$1"
  [[ -e "${candidate}/libddsc.so" ]] || compgen -G "${candidate}/libddsc.so.*" >/dev/null 2>&1
}

geometry_resolve_cyclonedds_lib_dir() {
  local candidate=""
  local home_dir="${HOME:-/home/unitree}"
  local candidates=()
  if [[ -n "${NUWAX_GEOMETRY_CYCLONEDDS_LIB_DIR:-}" ]]; then
    candidates+=("${NUWAX_GEOMETRY_CYCLONEDDS_LIB_DIR}")
  fi
  candidates+=(
    "${home_dir}/cyclonedds_ws/install/cyclonedds/lib"
    "${home_dir}/cyclonedds/install/lib"
    "/usr/local/lib"
  )
  for candidate in "${candidates[@]}"; do
    if [[ -d "${candidate}" ]] && geometry_has_cyclonedds_lib "${candidate}"; then
      printf '%s' "${candidate}"
      return 0
    fi
  done
  return 1
}

geometry_prepend_ld_library_path() {
  local target="$1"
  local current="${LD_LIBRARY_PATH:-}"
  case ":${current}:" in
    *":${target}:"*)
      return 0
      ;;
  esac
  if [[ -n "${current}" ]]; then
    export LD_LIBRARY_PATH="${target}:${current}"
    return 0
  fi
  export LD_LIBRARY_PATH="${target}"
}

if [[ "${NUWAX_GEOMETRY_RUNTIME_SOURCE_ROS_ENV,,}" == "1" || "${NUWAX_GEOMETRY_RUNTIME_SOURCE_ROS_ENV,,}" == "true" || "${NUWAX_GEOMETRY_RUNTIME_SOURCE_ROS_ENV,,}" == "yes" || "${NUWAX_GEOMETRY_RUNTIME_SOURCE_ROS_ENV,,}" == "on" ]]; then
  GEOMETRY_RUNTIME_ROS_SETUP="$(geometry_detect_ros_setup || true)"
  if [[ -n "${GEOMETRY_RUNTIME_ROS_SETUP}" ]]; then
    geometry_source_without_nounset "${GEOMETRY_RUNTIME_ROS_SETUP}"
  fi

  if [[ -f "${GEOMETRY_RUNTIME_WS_DIR}/install/setup.bash" ]]; then
    geometry_source_without_nounset "${GEOMETRY_RUNTIME_WS_DIR}/install/setup.bash"
  fi
fi

GEOMETRY_RUNTIME_CYCLONEDDS_LIB_DIR="$(geometry_resolve_cyclonedds_lib_dir || true)"
if [[ -n "${GEOMETRY_RUNTIME_CYCLONEDDS_LIB_DIR}" ]]; then
  geometry_prepend_ld_library_path "${GEOMETRY_RUNTIME_CYCLONEDDS_LIB_DIR}"
fi

unset GEOMETRY_RUNTIME_ROS_SETUP
unset GEOMETRY_RUNTIME_CYCLONEDDS_LIB_DIR
unset NUWAX_GEOMETRY_RUNTIME_SOURCE_ROS_ENV
