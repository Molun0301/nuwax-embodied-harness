#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
WS_DIR="${REPO_ROOT}/drivers/go2/ros2_ws"

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

trim_whitespace() {
  local value="$1"
  value="${value#"${value%%[![:space:]]*}"}"
  value="${value%"${value##*[![:space:]]}"}"
  printf '%s' "${value}"
}

load_env_file() {
  local target="$1"
  local raw_line=""
  local line=""
  local key=""
  local value=""
  while IFS= read -r raw_line || [[ -n "${raw_line}" ]]; do
    line="$(trim_whitespace "${raw_line}")"
    if [[ -z "${line}" || "${line}" == \#* || "${line}" != *=* ]]; then
      continue
    fi
    key="$(trim_whitespace "${line%%=*}")"
    value="$(trim_whitespace "${line#*=}")"
    if [[ -z "${key}" ]]; then
      continue
    fi
    if [[ ${#value} -ge 2 ]]; then
      if [[ "${value:0:1}" == '"' && "${value: -1}" == '"' ]]; then
        value="${value:1:${#value}-2}"
      elif [[ "${value:0:1}" == "'" && "${value: -1}" == "'" ]]; then
        value="${value:1:${#value}-2}"
      fi
    fi
    export "${key}=${value}"
  done < "${target}"
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

if [[ -f "${REPO_ROOT}/drivers/go2/data_plane.env" ]]; then
  load_env_file "${REPO_ROOT}/drivers/go2/data_plane.env"
fi

export GO2_RUNTIME_SOURCE_ROS_ENV=true
source_without_nounset "${SCRIPT_DIR}/source_runtime_env.sh"

cd "${REPO_ROOT}"

if [[ $# -eq 0 ]]; then
  cat <<'EOF' >&2
launch_ros2_stack.sh 只负责为独立 ROS2 侧车进程准备环境。
根据宇树官方约束，Unitree SDK2 与 ROS2 不能在同一进程中同时初始化，
因此本脚本不再默认启动 Go2 SDK2 数据面入口。
请显式传入需要启动的 ROS2 命令，例如：
  ./drivers/go2/launch_ros2_stack.sh ros2 topic list
EOF
  exit 64
fi

exec "$@"
