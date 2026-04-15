#!/usr/bin/env bash

GO2_RUNTIME_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
GO2_RUNTIME_REPO_ROOT="$(cd "${GO2_RUNTIME_SCRIPT_DIR}/../../.." && pwd)"
GO2_RUNTIME_WS_DIR="${GO2_RUNTIME_REPO_ROOT}/drivers/go2/ros2_ws"
GO2_RUNTIME_SOURCE_ROS_ENV="${GO2_RUNTIME_SOURCE_ROS_ENV:-false}"

go2_source_without_nounset() {
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

go2_detect_ros_setup() {
  local candidates=(
    "/opt/ros/humble/setup.bash"
    "/opt/ros/jazzy/setup.bash"
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

go2_has_cyclonedds_lib() {
  local candidate="$1"
  [[ -e "${candidate}/libddsc.so" ]] || compgen -G "${candidate}/libddsc.so.*" >/dev/null 2>&1
}

go2_resolve_cyclonedds_lib_dir() {
  local candidate=""
  local home_dir="${HOME:-/home/unitree}"
  local candidates=()
  if [[ -n "${GO2_CYCLONEDDS_LIB_DIR:-}" ]]; then
    candidates+=("${GO2_CYCLONEDDS_LIB_DIR}")
  fi
  candidates+=(
    "${home_dir}/cyclonedds_ws/install/cyclonedds/lib"
    "${home_dir}/cyclonedds/install/lib"
    "/usr/local/lib"
  )
  for candidate in "${candidates[@]}"; do
    if [[ -d "${candidate}" ]] && go2_has_cyclonedds_lib "${candidate}"; then
      printf '%s' "${candidate}"
      return 0
    fi
  done
  return 1
}

go2_prepend_ld_library_path() {
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

if [[ "${GO2_RUNTIME_SOURCE_ROS_ENV,,}" == "1" || "${GO2_RUNTIME_SOURCE_ROS_ENV,,}" == "true" || "${GO2_RUNTIME_SOURCE_ROS_ENV,,}" == "yes" || "${GO2_RUNTIME_SOURCE_ROS_ENV,,}" == "on" ]]; then
  GO2_RUNTIME_ROS_SETUP="$(go2_detect_ros_setup || true)"
  if [[ -n "${GO2_RUNTIME_ROS_SETUP}" ]]; then
    go2_source_without_nounset "${GO2_RUNTIME_ROS_SETUP}"
  fi

  if [[ -f "${GO2_RUNTIME_WS_DIR}/install/setup.bash" ]]; then
    go2_source_without_nounset "${GO2_RUNTIME_WS_DIR}/install/setup.bash"
  fi
fi

GO2_RUNTIME_CYCLONEDDS_LIB_DIR="$(go2_resolve_cyclonedds_lib_dir || true)"
if [[ -n "${GO2_RUNTIME_CYCLONEDDS_LIB_DIR}" ]]; then
  go2_prepend_ld_library_path "${GO2_RUNTIME_CYCLONEDDS_LIB_DIR}"
fi

unset GO2_RUNTIME_ROS_SETUP
unset GO2_RUNTIME_CYCLONEDDS_LIB_DIR
unset GO2_RUNTIME_SOURCE_ROS_ENV
