#!/usr/bin/env bash
set -euo pipefail

ACTION="${1:-wait}"
READY_FILE="${NUWAX_NAV_STACK_READY_FILE:-/run/nuwax/navigation/nuwax_nav_stack.ready}"
TIMEOUT_SEC="${NUWAX_NAV_STACK_START_TIMEOUT_SEC:-15}"

check_ready() {
  local pid=""
  local profile=""
  [[ -f "${READY_FILE}" && ! -L "${READY_FILE}" ]] || return 1
  pid="$(sed -n 's/^pid=//p' "${READY_FILE}" | head -n 1)"
  profile="$(sed -n 's/^profile=//p' "${READY_FILE}" | head -n 1)"
  [[ "${pid}" =~ ^[0-9]+$ ]] || return 1
  [[ "${profile}" == "simple-2d" || "${profile}" == "replanning-astar" ]] || return 1
  kill -0 "${pid}" >/dev/null 2>&1
}

case "${ACTION}" in
  check)
    check_ready
    ;;
  wait)
    case "${TIMEOUT_SEC}" in
      ''|*[!0-9]*)
        echo "invalid NUWAX_NAV_STACK_START_TIMEOUT_SEC=${TIMEOUT_SEC}" >&2
        exit 2
        ;;
    esac
    attempts=$((TIMEOUT_SEC * 10 + 1))
    for ((attempt = 0; attempt < attempts; attempt++)); do
      if check_ready; then
        exit 0
      fi
      sleep 0.1
    done
    echo "nuwax-nav-stack ready file missing or stale after ${TIMEOUT_SEC}s: ${READY_FILE}" >&2
    exit 1
    ;;
  *)
    echo "usage: $0 [check|wait]" >&2
    exit 2
    ;;
esac
