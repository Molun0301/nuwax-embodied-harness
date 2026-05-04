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
  local waited=0

  pid="$(read_pid_file || true)"
  if [[ -z "${pid}" ]]; then
    echo "gateway pid file not found: ${PID_FILE}"
    return 0
  fi

  if ! is_pid_alive "${pid}"; then
    echo "gateway pid ${pid} is not running; removing stale pid file."
    rm -f "${PID_FILE}"
    return 0
  fi

  echo "stopping gateway pid ${pid}..."
  kill "${pid}" >/dev/null 2>&1 || true
  while is_pid_alive "${pid}"; do
    waited=$((waited + 1))
    if [[ "${waited}" -ge 20 ]]; then
      echo "gateway pid ${pid} did not stop after ${waited}s; sending SIGKILL." >&2
      kill -9 "${pid}" >/dev/null 2>&1 || true
      break
    fi
    sleep 1
  done
  rm -f "${PID_FILE}"
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

stop_gateway() {
  STOP_REQUESTED=1
  if [[ -n "${GATEWAY_PID}" ]] && kill -0 "${GATEWAY_PID}" >/dev/null 2>&1; then
    kill "${GATEWAY_PID}" >/dev/null 2>&1 || true
    wait "${GATEWAY_PID}" >/dev/null 2>&1 || true
  fi
  exit 0
}

if [[ -f "${ENV_FILE}" ]]; then
  load_env_file "${ENV_FILE}"
fi

PID_FILE="${NUWAX_GATEWAY_PID_FILE:-${REPO_ROOT}/runtime_data/gateway/gateway.pid}"

case "${ACTION}" in
  start)
    if [[ -n "$(read_pid_file || true)" ]] && is_pid_alive "$(read_pid_file || true)"; then
      echo "gateway is already running pid=$(read_pid_file). Use: $0 restart" >&2
      exit 1
    fi
    rm -f "${PID_FILE}"
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

HOST="${NUWAX_GATEWAY_HOST:-0.0.0.0}"
PORT="${NUWAX_GATEWAY_PORT:-8766}"
export PYTHONUNBUFFERED=1

cd "${REPO_ROOT}"
write_pid_file
trap remove_pid_file EXIT
trap stop_gateway INT TERM

if ! is_restart_enabled "${RESTART_POLICY}"; then
  exec "${PYTHON_BIN}" -m uvicorn gateways.app:create_gateway_app_from_env --factory --host "${HOST}" --port "${PORT}"
fi

attempt=0
while true; do
  set +e
  "${PYTHON_BIN}" -m uvicorn gateways.app:create_gateway_app_from_env --factory --host "${HOST}" --port "${PORT}" &
  GATEWAY_PID="$!"
  wait "${GATEWAY_PID}"
  exit_code="$?"
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
