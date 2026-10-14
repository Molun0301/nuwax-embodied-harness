#!/usr/bin/env bash
set -euo pipefail

ACTION="${1:-wait}"
SOCKET_PATH="${NUWAX_MOTION_GUARD_SOCKET:-/run/nuwax/motion-guard.sock}"
TIMEOUT_SEC="${NUWAX_MOTION_GUARD_START_TIMEOUT_SEC:-15}"
EXPECTED_USER="${2:-nuwax}"
EXPECTED_GROUP="${3:-nuwax}"

case "${TIMEOUT_SEC}" in
  ''|*[!0-9]*)
    echo "invalid NUWAX_MOTION_GUARD_START_TIMEOUT_SEC=${TIMEOUT_SEC}" >&2
    exit 2
    ;;
esac

check_socket() {
  [[ -S "${SOCKET_PATH}" ]] || return 1
  [[ "$(stat -c '%U' "${SOCKET_PATH}")" == "${EXPECTED_USER}" ]] || return 1
  [[ "$(stat -c '%G' "${SOCKET_PATH}")" == "${EXPECTED_GROUP}" ]] || return 1
  [[ "$(stat -c '%a' "${SOCKET_PATH}")" == "660" ]] || return 1
  [[ -r "${SOCKET_PATH}" && -w "${SOCKET_PATH}" ]] || return 1
}

render_permissions() {
  local runtime_dir
  runtime_dir="$(dirname "${SOCKET_PATH}")"
  chgrp "${EXPECTED_GROUP}" "${runtime_dir}" "${SOCKET_PATH}"
  chmod 0750 "${runtime_dir}"
  chmod 0660 "${SOCKET_PATH}"
}

case "${ACTION}" in
  check)
    if check_socket; then
      exit 0
    fi
    echo "motion guard transport is not ready: ${SOCKET_PATH}" >&2
    exit 1
    ;;
  wait)
    ;;
  *)
    echo "usage: $0 [wait|check] [expected-user expected-group]" >&2
    exit 2
    ;;
esac

attempts=$((TIMEOUT_SEC * 10 + 1))
for ((attempt = 0; attempt < attempts; attempt++)); do
  if [[ -S "${SOCKET_PATH}" ]]; then
    render_permissions
    if check_socket; then
      exit 0
    fi
  fi
  sleep 0.1
done

echo "motion guard did not expose a secured socket within ${TIMEOUT_SEC}s: ${SOCKET_PATH}" >&2
exit 1
