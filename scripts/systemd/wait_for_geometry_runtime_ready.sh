#!/usr/bin/env bash
set -euo pipefail

ACTION="${1:-wait}"
READY_FILE="${NUWAX_GEOMETRY_RUNTIME_READY_FILE:-/run/nuwax/geometry/nuwax_geometry_runtime.ready}"
EXPECTED_PORT="${NUWAX_GEOMETRY_RUNTIME_PORT:-50051}"
TIMEOUT_SEC="${NUWAX_GEOMETRY_RUNTIME_HEALTH_TIMEOUT:-30}"

check_geometry_ready() {
  local pid=""
  local port=""
  local key=""
  local value=""
  local seen_pid=0
  local seen_port=0

  [[ -f "${READY_FILE}" && ! -L "${READY_FILE}" ]] || return 1
  while IFS='=' read -r key value || [[ -n "${key}${value}" ]]; do
    case "${key}" in
      pid)
        [[ "${seen_pid}" -eq 0 ]] || return 1
        pid="${value}"
        seen_pid=1
        ;;
      port)
        [[ "${seen_port}" -eq 0 ]] || return 1
        port="${value}"
        seen_port=1
        ;;
      *)
        return 1
        ;;
    esac
  done < "${READY_FILE}"

  [[ "${seen_pid}${seen_port}" == "11" ]] || return 1
  [[ "${pid}" =~ ^[1-9][0-9]*$ ]] || return 1
  [[ "${port}" =~ ^[1-9][0-9]*$ && "${port}" -le 65535 ]] || return 1
  [[ "${EXPECTED_PORT}" =~ ^[1-9][0-9]*$ && "${EXPECTED_PORT}" -le 65535 ]] || return 1
  [[ "${port}" == "${EXPECTED_PORT}" ]] || return 1
  [[ -d "/proc/${pid}" ]]
}

case "${ACTION}" in
  check)
    check_geometry_ready
    ;;
  wait)
    case "${TIMEOUT_SEC}" in
      ''|*[!0-9]*)
        echo "invalid NUWAX_GEOMETRY_RUNTIME_HEALTH_TIMEOUT=${TIMEOUT_SEC}" >&2
        exit 2
        ;;
    esac
    attempts=$((TIMEOUT_SEC * 10 + 1))
    for ((attempt = 0; attempt < attempts; attempt++)); do
      check_geometry_ready && exit 0
      sleep 0.1
    done
    echo "nuwax-geometry-runtime ready file missing or stale after ${TIMEOUT_SEC}s: ${READY_FILE}" >&2
    exit 1
    ;;
  *)
    echo "usage: $0 [check|wait]" >&2
    exit 2
    ;;
esac
