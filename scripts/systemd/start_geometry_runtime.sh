#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
IDENTITY_HELPER="${SCRIPT_DIR}/process_identity.sh"
BINARY="${NUWAX_GEOMETRY_RUNTIME_BINARY:-${REPO_ROOT}/services/geometry/nuwax_geometry_runtime/target/release/nuwax-geometry-runtime}"
READY_FILE="${NUWAX_GEOMETRY_RUNTIME_READY_FILE:-${REPO_ROOT}/runtime_data/geometry/nuwax_geometry_runtime.ready}"
INTERNAL_READY_FILE="${READY_FILE}.runtime"
PORT="${NUWAX_GEOMETRY_RUNTIME_PORT:-50051}"
HEALTH_TIMEOUT="${NUWAX_GEOMETRY_RUNTIME_HEALTH_TIMEOUT:-30}"
CHILD_PID=""

[[ -r "${IDENTITY_HELPER}" ]] || {
  echo "nuwax-geometry-runtime launcher: process identity helper is unavailable: ${IDENTITY_HELPER}" >&2
  exit 1
}
# shellcheck source=scripts/systemd/process_identity.sh
source "${IDENTITY_HELPER}"

fail() {
  echo "nuwax-geometry-runtime launcher: $*" >&2
  exit 1
}

stop_geometry_child() {
  [[ -n "${CHILD_PID}" ]] || return 0
  if kill -0 "${CHILD_PID}" >/dev/null 2>&1; then
    kill -INT "${CHILD_PID}" >/dev/null 2>&1 || true
    for ((step = 0; step < 20; step++)); do
      kill -0 "${CHILD_PID}" >/dev/null 2>&1 || break
      sleep 0.1
    done
  fi
  if kill -0 "${CHILD_PID}" >/dev/null 2>&1; then
    kill -TERM "${CHILD_PID}" >/dev/null 2>&1 || true
    for ((step = 0; step < 20; step++)); do
      kill -0 "${CHILD_PID}" >/dev/null 2>&1 || break
      sleep 0.1
    done
  fi
  if kill -0 "${CHILD_PID}" >/dev/null 2>&1; then
    kill -KILL "${CHILD_PID}" >/dev/null 2>&1 || true
  fi
  wait "${CHILD_PID}" >/dev/null 2>&1 || true
}

cleanup() {
  local status=$?
  trap - EXIT
  rm -f "${READY_FILE}" "${READY_FILE}.tmp" \
    "${INTERNAL_READY_FILE}" "${INTERNAL_READY_FILE}.tmp" 2>/dev/null || true
  stop_geometry_child
  exit "${status}"
}

forward_shutdown() {
  stop_geometry_child
  exit 0
}

geometry_internal_health_ready() {
  local internal_port=""
  [[ -f "${INTERNAL_READY_FILE}" && ! -L "${INTERNAL_READY_FILE}" ]] || return 1
  internal_port="$(< "${INTERNAL_READY_FILE}")" || return 1
  [[ "${internal_port}" =~ ^[1-9][0-9]*$ ]] || return 1
  [[ "${internal_port}" -le 65535 && "${internal_port}" == "${PORT}" ]]
}

case "${PORT}" in
  ''|*[!0-9]*)
    fail "NUWAX_GEOMETRY_RUNTIME_PORT must be an integer in [1, 65535]"
    ;;
esac
[[ "${PORT}" -ge 1 && "${PORT}" -le 65535 ]] \
  || fail "NUWAX_GEOMETRY_RUNTIME_PORT must be an integer in [1, 65535]"
case "${HEALTH_TIMEOUT}" in
  ''|*[!0-9]*)
    fail "NUWAX_GEOMETRY_RUNTIME_HEALTH_TIMEOUT must be a positive integer"
    ;;
esac
[[ "${HEALTH_TIMEOUT}" -gt 0 ]] \
  || fail "NUWAX_GEOMETRY_RUNTIME_HEALTH_TIMEOUT must be greater than zero"
[[ -x "${BINARY}" ]] || fail "binary is not executable: ${BINARY}"

mkdir -p "$(dirname "${READY_FILE}")"
rm -f "${READY_FILE}" "${READY_FILE}.tmp" \
  "${INTERNAL_READY_FILE}" "${INTERNAL_READY_FILE}.tmp"
trap cleanup EXIT
trap forward_shutdown INT TERM

NUWAX_GEOMETRY_RUNTIME_READY_FILE="${INTERNAL_READY_FILE}" \
  NUWAX_GEOMETRY_RUNTIME_PORT="${PORT}" \
  "${BINARY}" &
CHILD_PID="$!"

attempts=$((HEALTH_TIMEOUT * 10 + 1))
for ((attempt = 0; attempt < attempts; attempt++)); do
  if ! kill -0 "${CHILD_PID}" >/dev/null 2>&1; then
    set +e
    wait "${CHILD_PID}"
    status=$?
    set -e
    fail "runtime exited before gRPC health readiness with status ${status}"
  fi
  geometry_internal_health_ready && break
  sleep 0.1
done

geometry_internal_health_ready \
  || fail "runtime did not publish the expected healthy port within ${HEALTH_TIMEOUT}s"
BOOT_ID="$(nuwax_read_boot_id)" || fail "cannot read current boot identity"
CHILD_STARTTIME="$(nuwax_read_proc_starttime "${CHILD_PID}")" \
  || fail "cannot read child process starttime from /proc/${CHILD_PID}/stat"

printf 'pid=%s\nboot_id=%s\nstarttime=%s\nport=%s\nhealth_ready=1\n' \
  "${CHILD_PID}" "${BOOT_ID}" "${CHILD_STARTTIME}" "${PORT}" > "${READY_FILE}.tmp"
mv -f "${READY_FILE}.tmp" "${READY_FILE}"

set +e
wait "${CHILD_PID}"
status=$?
set -e
CHILD_PID=""
exit "${status}"
