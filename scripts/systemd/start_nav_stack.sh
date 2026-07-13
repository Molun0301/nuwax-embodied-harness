#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
BINARY="${NUWAX_NAV_STACK_BINARY:-${REPO_ROOT}/services/nuwax-workspace/target/release/nuwax-nav-stack}"
PROFILE="${NUWAX_NAV_PROFILE:-simple-2d}"
READY_FILE="${NUWAX_NAV_STACK_READY_FILE:-${REPO_ROOT}/runtime_data/navigation/nuwax_nav_stack.ready}"
WARMUP_STEPS="${NUWAX_NAV_STACK_WARMUP_STEPS:-10}"
CHILD_PID=""

fail() {
  echo "nuwax-nav-stack launcher: $*" >&2
  exit 1
}

cleanup() {
  rm -f "${READY_FILE}" "${READY_FILE}.tmp" 2>/dev/null || true
}

forward_shutdown() {
  if [[ -n "${CHILD_PID}" ]] && kill -0 "${CHILD_PID}" >/dev/null 2>&1; then
    # The Rust CLI owns SIGINT through tokio::signal::ctrl_c().
    kill -INT "${CHILD_PID}" >/dev/null 2>&1 || true
    for ((step = 0; step < 20; step++)); do
      if ! kill -0 "${CHILD_PID}" >/dev/null 2>&1; then
        break
      fi
      sleep 0.1
    done
    if kill -0 "${CHILD_PID}" >/dev/null 2>&1; then
      kill -TERM "${CHILD_PID}" >/dev/null 2>&1 || true
    fi
    for ((step = 0; step < 20; step++)); do
      if ! kill -0 "${CHILD_PID}" >/dev/null 2>&1; then
        break
      fi
      sleep 0.1
    done
    if kill -0 "${CHILD_PID}" >/dev/null 2>&1; then
      kill -KILL "${CHILD_PID}" >/dev/null 2>&1 || true
    fi
    wait "${CHILD_PID}" >/dev/null 2>&1 || true
  fi
  exit 0
}

case "${PROFILE}" in
  simple-2d|replanning-astar)
    ;;
  *)
    fail "unsupported production profile '${PROFILE}'; expected simple-2d or replanning-astar"
    ;;
esac

case "${WARMUP_STEPS}" in
  ''|*[!0-9]*)
    fail "NUWAX_NAV_STACK_WARMUP_STEPS must be a positive integer"
    ;;
esac
[[ "${WARMUP_STEPS}" -gt 0 ]] || fail "NUWAX_NAV_STACK_WARMUP_STEPS must be greater than zero"
[[ -x "${BINARY}" ]] || fail "binary is not executable: ${BINARY}"

mkdir -p "$(dirname "${READY_FILE}")"
cleanup
trap cleanup EXIT
trap forward_shutdown INT TERM

"${BINARY}" --profile="${PROFILE}" &
CHILD_PID="$!"

for ((step = 0; step < WARMUP_STEPS; step++)); do
  if ! kill -0 "${CHILD_PID}" >/dev/null 2>&1; then
    set +e
    wait "${CHILD_PID}"
    status="$?"
    set -e
    fail "runtime exited during ownership/health warmup with status ${status}; build it via scripts/build_nav_stack.sh"
  fi
  sleep 0.1
done

if ! kill -0 "${CHILD_PID}" >/dev/null 2>&1; then
  set +e
  wait "${CHILD_PID}"
  status="$?"
  set -e
  fail "runtime exited before readiness publication with status ${status}"
fi

printf 'pid=%s\nprofile=%s\n' "${CHILD_PID}" "${PROFILE}" > "${READY_FILE}.tmp"
mv -f "${READY_FILE}.tmp" "${READY_FILE}"

set +e
wait "${CHILD_PID}"
status="$?"
set -e
exit "${status}"
