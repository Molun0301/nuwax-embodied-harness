#!/usr/bin/env bash
# nav2_action_proxy_launch.sh：拉起独立 ROS 进程 nav2_action_proxy.py。
#
# 设计要点（openspec/changes/navigation-and-exploration-boundary/design.md 决策 D1 + D3）：
# - 参照 services/geometry/launch_ros2_stack.sh 风格：env 加载 + RMW 注入 + exec 入口。
# - source /opt/ros/humble/setup.bash + export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp。
# - 跨 RMW 隔离：与平台 services 层（rclpy=fastrtps）解耦，单独走 cyclonedds。
# - 不引入新 lock 文件：与 gateway 共用 runtime_data/.gateway.lock
#   （参照 memory start-gateway-shared-lock-path.md）。
# - 入口固定为 services/navigation/nav2_action_proxy.py。
#
# 用法：
#   ./services/navigation/nav2_action_proxy_launch.sh
#
# 退出码：
#   0  正常
#   64 参数缺失（USAGE）
#   非 0 启动失败

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

# 与 start_gateway.sh 共用 runtime_data/.gateway.lock（installer 共享锁封装已统一）
LOCK_FILE="${NUWAX_NAV2_PROXY_LOCK_FILE:-${REPO_ROOT}/runtime_data/.gateway.lock}"
LOCK_DIR="$(dirname "${LOCK_FILE}")"
mkdir -p "${LOCK_DIR}"
exec 9>"${LOCK_FILE}"
if ! flock -n 9; then
  echo "another nuwax process is running (lock: ${LOCK_FILE})." >&2
  exit 1
fi

# ROS 环境（与 launch_ros2_stack.sh 同源；本进程只 source 一次避免污染平台 services）
if [[ -f "/opt/ros/humble/setup.bash" ]]; then
  source_without_nounset "/opt/ros/humble/setup.bash"
fi

# 跨 RMW 隔离：cyclonedds（platform services 层 rclpy=fastrtps，本进程独立 cyclonedds）
export RMW_IMPLEMENTATION="${NAV_PROXY_RMW:-rmw_cyclonedds_cpp}"
export NUWAX_NAV2_PROXY_INDEPENDENT_PROCESS=true

# 端口 / 队列上限 / action namespace 默认值（与 core/settings.py NAV_PROXY_* 对齐）
export NAV_PROXY_GRPC_PORT="${NAV_PROXY_GRPC_PORT:-50061}"
export NAV_PROXY_PENDING_QUEUE_MAX_SIZE="${NAV_PROXY_PENDING_QUEUE_MAX_SIZE:-4}"
export NAV_ACTION_NAMESPACE="${NAV_ACTION_NAMESPACE:-/navigate_to_pose}"
export PYTHONUNBUFFERED=1

cd "${REPO_ROOT}"

# 入口：直接 exec python3，避免 bash 进程残留
exec python3 "${SCRIPT_DIR}/nav2_action_proxy.py" \
  --grpc-port "${NAV_PROXY_GRPC_PORT}" \
  --pending-queue-max-size "${NAV_PROXY_PENDING_QUEUE_MAX_SIZE}" \
  --action-namespace "${NAV_ACTION_NAMESPACE}" \
  --rmw "${RMW_IMPLEMENTATION}"
