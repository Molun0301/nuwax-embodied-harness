#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
IDENTITY_HELPER="${SCRIPT_DIR}/process_identity.sh"
RUNTIME_BINARY="${NUWAX_GEOMETRY_RUNTIME_BINARY:-${REPO_ROOT}/services/geometry/nuwax_geometry_runtime/target/release/nuwax-geometry-runtime}"
ACTION="${1:-wait}"
READY_FILE="${NUWAX_GEOMETRY_RUNTIME_READY_FILE:-/run/nuwax/geometry/nuwax_geometry_runtime.ready}"
EXPECTED_PORT="${NUWAX_GEOMETRY_RUNTIME_PORT:-50051}"
TIMEOUT_SEC="${NUWAX_GEOMETRY_RUNTIME_HEALTH_TIMEOUT:-30}"

[[ -r "${IDENTITY_HELPER}" ]] || {
  echo "process identity helper is unavailable: ${IDENTITY_HELPER}" >&2
  exit 1
}
# shellcheck source=scripts/systemd/process_identity.sh
source "${IDENTITY_HELPER}"

check_geometry_ready() {
  local pid=""
  local boot_id=""
  local starttime=""
  local port=""
  local health_ready=""
  local runtime_binary_sha256=""
  local current_binary_sha256=""
  local current_boot_id=""
  local current_starttime=""
  local key=""
  local value=""
  local seen_pid=0
  local seen_boot_id=0
  local seen_starttime=0
  local seen_port=0
  local seen_health_ready=0
  local seen_runtime_binary_sha256=0

  [[ -x "${RUNTIME_BINARY}" && ! -L "${RUNTIME_BINARY}" ]] || return 1
  [[ -f "${READY_FILE}" && ! -L "${READY_FILE}" ]] || return 1
  while IFS='=' read -r key value || [[ -n "${key}${value}" ]]; do
    case "${key}" in
      pid)
        [[ "${seen_pid}" -eq 0 ]] || return 1
        pid="${value}"
        seen_pid=1
        ;;
      boot_id)
        [[ "${seen_boot_id}" -eq 0 ]] || return 1
        boot_id="${value,,}"
        seen_boot_id=1
        ;;
      starttime)
        [[ "${seen_starttime}" -eq 0 ]] || return 1
        starttime="${value}"
        seen_starttime=1
        ;;
      port)
        [[ "${seen_port}" -eq 0 ]] || return 1
        port="${value}"
        seen_port=1
        ;;
      health_ready)
        [[ "${seen_health_ready}" -eq 0 ]] || return 1
        health_ready="${value}"
        seen_health_ready=1
        ;;
      runtime_binary_sha256)
        [[ "${seen_runtime_binary_sha256}" -eq 0 ]] || return 1
        runtime_binary_sha256="${value}"
        seen_runtime_binary_sha256=1
        ;;
      *)
        return 1
        ;;
    esac
  done < "${READY_FILE}"

  [[ "${seen_pid}${seen_boot_id}${seen_starttime}${seen_port}${seen_health_ready}${seen_runtime_binary_sha256}" == "111111" ]] \
    || return 1
  [[ "${pid}" =~ ^[1-9][0-9]*$ ]] || return 1
  [[ "${starttime}" =~ ^[1-9][0-9]*$ ]] || return 1
  [[ "${port}" =~ ^[1-9][0-9]*$ && "${port}" -le 65535 ]] || return 1
  [[ "${EXPECTED_PORT}" =~ ^[1-9][0-9]*$ && "${EXPECTED_PORT}" -le 65535 ]] || return 1
  [[ "${port}" == "${EXPECTED_PORT}" && "${health_ready}" == "1" ]] || return 1
  [[ "${runtime_binary_sha256}" =~ ^[0-9a-f]{64}$ ]] || return 1
  current_binary_sha256="$(sha256sum "${RUNTIME_BINARY}" | cut -d' ' -f1)" || return 1
  [[ "${runtime_binary_sha256}" == "${current_binary_sha256}" ]] || return 1
  current_boot_id="$(nuwax_read_boot_id)" || return 1
  [[ "${boot_id}" == "${current_boot_id}" ]] || return 1
  current_starttime="$(nuwax_read_proc_starttime "${pid}")" || return 1
  [[ "${starttime}" == "${current_starttime}" ]] || return 1
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
