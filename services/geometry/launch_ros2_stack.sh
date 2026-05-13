#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

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

if [[ -f "${REPO_ROOT}/.env" ]]; then
  load_env_file "${REPO_ROOT}/.env"
fi

export NUWAX_GEOMETRY_RUNTIME_SOURCE_ROS_ENV=true
export NUWAX_GEOMETRY_RUNTIME_FORCE_CYCLONEDDS=true
source_without_nounset "${SCRIPT_DIR}/source_runtime_env.sh"

cd "${REPO_ROOT}"

if [[ $# -eq 0 ]]; then
  cat <<'EOF' >&2
launch_ros2_stack.sh 只负责为平台几何 ROS2 侧车进程准备环境。
它不会默认启动任何 RTAB-Map / Nav2 launch，请显式传入要执行的命令，例如：
  ./services/geometry/launch_ros2_stack.sh ros2 topic list
EOF
  exit 64
fi

exec "$@"
