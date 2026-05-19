#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
ENV_FILE="${NUWAX_BOOTSTRAP_ENV_FILE:-${REPO_ROOT}/.env}"
PYTHON_BIN="${NUWAX_PYTHON_BIN:-python3}"
RESTART_POLICY="${NUWAX_GATEWAY_RESTART:-on-failure}"
RESTART_SEC="${NUWAX_GATEWAY_RESTART_SEC:-3}"
RESTART_MAX_ATTEMPTS="${NUWAX_GATEWAY_RESTART_MAX_ATTEMPTS:-0}"
ACTION="${1:-start}"
PID_FILE=""
GATEWAY_PID=""
VIDEO_STREAMER_PID=""
ROS_BRIDGE_PID=""
STOP_REQUESTED=0

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

is_truthy() {
  local value="$(printf '%s' "$1" | tr '[:upper:]' '[:lower:]')"
  [[ "${value}" == "1" || "${value}" == "true" || "${value}" == "yes" || "${value}" == "on" ]]
}

source_runtime_script() {
  local target="$1"
  if [[ -f "${target}" ]]; then
    # shellcheck disable=SC1090
    source "${target}"
  fi
}

is_restart_enabled() {
  local value="$(printf '%s' "$1" | tr '[:upper:]' '[:lower:]')"
  [[ "${value}" == "1" || "${value}" == "true" || "${value}" == "yes" || "${value}" == "on" || "${value}" == "always" || "${value}" == "on-failure" ]]
}

should_restart() {
  local exit_code="$1"
  local policy="$(printf '%s' "${RESTART_POLICY}" | tr '[:upper:]' '[:lower:]')"
  if [[ "${STOP_REQUESTED}" -eq 1 ]]; then
    return 1
  fi
  case "${policy}" in
    always|1|true|yes|on)
      return 0
      ;;
    on-failure)
      [[ "${exit_code}" -ne 0 ]]
      return
      ;;
    *)
      return 1
      ;;
  esac
}

read_pid_file() {
  if [[ -f "${PID_FILE}" ]]; then
    tr -d '[:space:]' < "${PID_FILE}"
  fi
}

is_pid_alive() {
  local pid="$1"
  [[ -n "${pid}" ]] && kill -0 "${pid}" >/dev/null 2>&1
}

pid_pgid() {
  local pid="$1"
  ps -o pgid= -p "${pid}" 2>/dev/null | tr -d '[:space:]' || true
}

send_signal_to_process_scope() {
  local signal="$1"
  local pid="$2"
  local pgid=""
  local self_pgid=""

  if ! is_pid_alive "${pid}"; then
    return 0
  fi

  pgid="$(pid_pgid "${pid}")"
  self_pgid="$(pid_pgid "$$")"
  if [[ -n "${pgid}" && "${pgid}" != "${self_pgid}" ]]; then
    kill "-${signal}" -- "-${pgid}" >/dev/null 2>&1 || true
  fi
  kill "-${signal}" -- "${pid}" >/dev/null 2>&1 || true
}

wait_for_pid_exit() {
  local pid="$1"
  local timeout_sec="$2"
  local waited=0

  while is_pid_alive "${pid}"; do
    if [[ "${waited}" -ge "${timeout_sec}" ]]; then
      return 1
    fi
    sleep 1
    waited=$((waited + 1))
  done
  return 0
}

terminate_process_scope() {
  local pid="$1"
  local label="${2:-process}"
  local stop_timeout="${NUWAX_GATEWAY_STOP_TIMEOUT_SEC:-20}"
  local kill_timeout="${NUWAX_GATEWAY_KILL_TIMEOUT_SEC:-5}"

  if ! is_pid_alive "${pid}"; then
    return 0
  fi

  echo "stopping ${label} pid ${pid}..."
  send_signal_to_process_scope TERM "${pid}"
  send_signal_to_process_scope CONT "${pid}"
  if wait_for_pid_exit "${pid}" "${stop_timeout}"; then
    return 0
  fi

  echo "${label} pid ${pid} did not stop after ${stop_timeout}s; sending SIGKILL." >&2
  send_signal_to_process_scope KILL "${pid}"
  wait_for_pid_exit "${pid}" "${kill_timeout}" || true
}

cleanup_stale_gateway_processes() {
  local pid=""
  local args=""

  while read -r pid args; do
    if [[ -z "${pid}" || "${pid}" == "$$" || "${pid}" == "${BASHPID}" ]]; then
      continue
    fi
    if [[ "${args}" == *"uvicorn gateways.app:create_gateway_app_from_env"* && "${args}" == *"--port ${PORT}"* ]]; then
      terminate_process_scope "${pid}" "stale gateway worker"
    fi
  done < <(ps -eo pid=,args=)
}

cleanup_external_geometry_processes() {
  local geometry_root="${REPO_ROOT}/services/geometry/ros2_ws/src/go2slam"
  local pid=""
  local args=""
  local pgid=""
  declare -A seen_pgids=()

  if ! is_truthy "${NUWAX_GATEWAY_CLEANUP_EXTERNAL_GEOMETRY:-true}"; then
    return 0
  fi

  while read -r pid args; do
    if [[ -z "${pid}" || "${pid}" == "$$" || "${pid}" == "${BASHPID}" ]]; then
      continue
    fi
    if [[ "${args}" != *"${geometry_root}"* ]]; then
      continue
    fi
    pgid="$(pid_pgid "${pid}")"
    if [[ -z "${pgid}" || -n "${seen_pgids[${pgid}]:-}" ]]; then
      continue
    fi
    seen_pgids["${pgid}"]=1
    terminate_process_scope "${pid}" "external geometry process group ${pgid}"
  done < <(ps -eo pid=,args=)
}

remove_pid_file() {
  local pid=""
  pid="$(read_pid_file || true)"
  if [[ -z "${pid}" || "${pid}" == "$$" ]]; then
    rm -f "${PID_FILE}"
  fi
}

write_pid_file() {
  mkdir -p "$(dirname "${PID_FILE}")"
  printf '%s\n' "$$" > "${PID_FILE}"
}

stop_existing_gateway() {
  local pid=""

  pid="$(read_pid_file || true)"
  if [[ -z "${pid}" ]]; then
    echo "gateway pid file not found: ${PID_FILE}"
    cleanup_stale_gateway_processes
    cleanup_external_geometry_processes
    return 0
  fi

  if ! is_pid_alive "${pid}"; then
    echo "gateway pid ${pid} is not running; removing stale pid file."
    rm -f "${PID_FILE}"
    cleanup_stale_gateway_processes
    cleanup_external_geometry_processes
    return 0
  fi

  terminate_process_scope "${pid}" "gateway supervisor"
  rm -f "${PID_FILE}"
  cleanup_stale_gateway_processes
  cleanup_external_geometry_processes
}

status_gateway() {
  local pid=""
  pid="$(read_pid_file || true)"
  if [[ -n "${pid}" ]] && is_pid_alive "${pid}"; then
    echo "gateway running pid=${pid}"
    return 0
  fi
  echo "gateway not running"
  return 3
}

should_start_ros_bridge() {
  local binary="${REPO_ROOT}/services/geometry/nuwax_ros_bridge/target/release/nuwax-ros-bridge"
  [[ -x "${binary}" ]] && is_truthy "${NUWAX_ROS_BRIDGE_ENABLED:-true}"
}

should_start_go2_video_streamer() {
  is_truthy "${GO2_VIDEO_STREAM_ENABLED:-true}" \
    && is_truthy "${GO2_DATA_PLANE_ENABLED:-false}" \
    && [[ "$(printf '%s' "${NUWAX_ACTIVE_ROBOT:-}" | tr '[:upper:]' '[:lower:]')" == "go2" ]]
}

start_ros_bridge() {
  if [[ -n "${ROS_BRIDGE_PID}" ]] && is_pid_alive "${ROS_BRIDGE_PID}"; then
    return 0
  fi
  if ! should_start_ros_bridge; then
    return 0
  fi
  local binary="${REPO_ROOT}/services/geometry/nuwax_ros_bridge/target/release/nuwax-ros-bridge"
  echo "starting Rust ros-bridge: ${binary}"
  (source /opt/ros/foxy/setup.bash && exec "${binary}") &
  ROS_BRIDGE_PID="$!"
  # 等待 UDS socket 出现
  local waited=0
  while [[ ! -S /tmp/nuwax_video.sock ]] && [[ "${waited}" -lt 5 ]]; do
    sleep 1
    waited=$((waited + 1))
  done
  if [[ -S /tmp/nuwax_video.sock ]]; then
    echo "Rust ros-bridge ready (pid ${ROS_BRIDGE_PID})"
  else
    echo "WARNING: Rust ros-bridge UDS not ready after 5s" >&2
  fi
}

start_go2_video_streamer() {
  if [[ -n "${VIDEO_STREAMER_PID}" ]] && is_pid_alive "${VIDEO_STREAMER_PID}"; then
    return 0
  fi
  if ! should_start_go2_video_streamer; then
    return 0
  fi
  echo "starting Go2 side video streamer -> UDS /tmp/nuwax_video.sock"
  "${PYTHON_BIN}" -m drivers.go2.video_streamer &
  VIDEO_STREAMER_PID="$!"
}

stop_ros_bridge() {
  if [[ -n "${ROS_BRIDGE_PID}" ]] && is_pid_alive "${ROS_BRIDGE_PID}"; then
    terminate_process_scope "${ROS_BRIDGE_PID}" "rust ros-bridge"
    wait "${ROS_BRIDGE_PID}" >/dev/null 2>&1 || true
  fi
  ROS_BRIDGE_PID=""
  rm -f /tmp/nuwax_video.sock /tmp/nuwax_video.shm
}

stop_go2_video_streamer() {
  if [[ -n "${VIDEO_STREAMER_PID}" ]] && is_pid_alive "${VIDEO_STREAMER_PID}"; then
    terminate_process_scope "${VIDEO_STREAMER_PID}" "go2 video streamer"
    wait "${VIDEO_STREAMER_PID}" >/dev/null 2>&1 || true
  fi
  VIDEO_STREAMER_PID=""
}

stop_gateway() {
  STOP_REQUESTED=1
  stop_go2_video_streamer
  stop_ros_bridge
  if [[ -n "${GATEWAY_PID}" ]] && kill -0 "${GATEWAY_PID}" >/dev/null 2>&1; then
    terminate_process_scope "${GATEWAY_PID}" "gateway worker"
    wait "${GATEWAY_PID}" >/dev/null 2>&1 || true
  fi
  cleanup_external_geometry_processes
  exit 0
}

if [[ -f "${ENV_FILE}" ]]; then
  load_env_file "${ENV_FILE}"
fi
GO2_DATA_PLANE_ENV_FILE="${GO2_DATA_PLANE_ENV_FILE:-${REPO_ROOT}/drivers/go2/data_plane.env}"
if [[ -f "${GO2_DATA_PLANE_ENV_FILE}" ]]; then
  load_env_file "${GO2_DATA_PLANE_ENV_FILE}"
fi

PYTHON_BIN="${NUWAX_PYTHON_BIN:-${PYTHON_BIN:-python3}}"
PID_FILE="${NUWAX_GATEWAY_PID_FILE:-${REPO_ROOT}/runtime_data/gateway/gateway.pid}"
HOST="${NUWAX_GATEWAY_HOST:-0.0.0.0}"
PORT="${NUWAX_GATEWAY_PORT:-8766}"

case "${ACTION}" in
  start)
    if [[ -n "$(read_pid_file || true)" ]] && is_pid_alive "$(read_pid_file || true)"; then
      echo "gateway is already running pid=$(read_pid_file). Use: $0 restart" >&2
      exit 1
    fi
    rm -f "${PID_FILE}"
    cleanup_stale_gateway_processes
    cleanup_external_geometry_processes
    ;;
  stop)
    stop_existing_gateway
    exit 0
    ;;
  restart)
    stop_existing_gateway
    ;;
  status)
    status_gateway
    exit $?
    ;;
  *)
    echo "usage: $0 [start|stop|restart|status]" >&2
    exit 2
    ;;
esac

if is_truthy "${NUWAX_GEOMETRY_ENABLED:-true}"; then
  if is_truthy "${NUWAX_GEOMETRY_REQUIRE_ROS2:-true}"; then
    export NUWAX_GEOMETRY_RUNTIME_SOURCE_ROS_ENV="${NUWAX_GEOMETRY_RUNTIME_SOURCE_ROS_ENV:-true}"
  fi
  source_runtime_script "${REPO_ROOT}/services/geometry/source_runtime_env.sh"
fi

export PYTHONUNBUFFERED=1
export GO2_VIDEO_STREAM_GATEWAY_URL="${GO2_VIDEO_STREAM_GATEWAY_URL:-ws://127.0.0.1:${PORT}/streams/video.ingest}"
GATEWAY_AGENT_TOKEN_DEFAULT="${NUWAX_GATEWAY_AGENT_TOKENS:-}"
export GO2_VIDEO_STREAM_ACCESS_TOKEN="${GO2_VIDEO_STREAM_ACCESS_TOKEN:-${GATEWAY_AGENT_TOKEN_DEFAULT%%,*}}"

cd "${REPO_ROOT}"
write_pid_file
trap remove_pid_file EXIT
trap stop_gateway INT TERM

if ! is_restart_enabled "${RESTART_POLICY}"; then
  set +e
  "${PYTHON_BIN}" -m uvicorn gateways.app:create_gateway_app_from_env --factory --host "${HOST}" --port "${PORT}" &
  GATEWAY_PID="$!"
  start_ros_bridge
  start_go2_video_streamer
  wait "${GATEWAY_PID}"
  exit_code="$?"
  stop_go2_video_streamer
  exit "${exit_code}"
fi

attempt=0
while true; do
  set +e
  "${PYTHON_BIN}" -m uvicorn gateways.app:create_gateway_app_from_env --factory --host "${HOST}" --port "${PORT}" &
  GATEWAY_PID="$!"
  start_ros_bridge
  start_go2_video_streamer
  wait "${GATEWAY_PID}"
  exit_code="$?"
  stop_go2_video_streamer
  GATEWAY_PID=""
  set -e

  if ! should_restart "${exit_code}"; then
    exit "${exit_code}"
  fi

  attempt=$((attempt + 1))
  if [[ "${RESTART_MAX_ATTEMPTS}" != "0" && "${attempt}" -ge "${RESTART_MAX_ATTEMPTS}" ]]; then
    echo "gateway exited with code ${exit_code}; restart attempts exhausted (${attempt})." >&2
    exit "${exit_code}"
  fi

  echo "gateway exited with code ${exit_code}; restarting in ${RESTART_SEC}s (attempt ${attempt})." >&2
  sleep "${RESTART_SEC}"
done
