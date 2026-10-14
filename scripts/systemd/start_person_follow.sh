#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
# 先解析共同配置并统一旧通道别名，再物化启动参数。
source "${REPO_ROOT}/services/geometry/source_runtime_env.sh"
# 严格真实通信启动须在启动业务子进程之前确认本机/外部选路条件。
case "${NUWAX_REAL_LCM_REQUIRED:-false}" in
  1|true|TRUE|yes|YES|on|ON) bash "${SCRIPT_DIR}/check_lcm_multicast.sh" ;;
esac

IDENTITY_HELPER="${SCRIPT_DIR}/process_identity.sh"
BINARY="${NUWAX_PERSON_FOLLOW_BINARY:-${REPO_ROOT}/services/nuwax-workspace/target/release/nuwax-person-follow}"
READY_FILE="${NUWAX_PERSON_FOLLOW_READY_FILE:-/run/nuwax/person-follow/nuwax_person_follow.ready}"
INTERNAL_READY_FILE="${READY_FILE}.runtime"
WARMUP_STEPS="${NUWAX_PERSON_FOLLOW_WARMUP_STEPS:-150}"
CHILD_PID=""

[[ -r "${IDENTITY_HELPER}" ]] || {
  echo "nuwax-person-follow launcher: process identity helper is unavailable: ${IDENTITY_HELPER}" >&2
  exit 1
}
# shellcheck source=scripts/systemd/process_identity.sh
source "${IDENTITY_HELPER}"

fail() {
  echo "nuwax-person-follow launcher: $*" >&2
  exit 1
}

stop_person_follow_child() {
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
  stop_person_follow_child
  exit "${status}"
}

forward_shutdown() {
  stop_person_follow_child
  exit 0
}

person_follow_internal_ready() {
  local marker_pid=""
  local marker_ready=""
  local key=""
  local value=""
  local seen_pid=0
  local seen_ready=0

  [[ -f "${INTERNAL_READY_FILE}" && ! -L "${INTERNAL_READY_FILE}" ]] || return 1
  while IFS='=' read -r key value || [[ -n "${key}${value}" ]]; do
    case "${key}" in
      pid)
        [[ "${seen_pid}" -eq 0 ]] || return 1
        marker_pid="${value}"
        seen_pid=1
        ;;
      ready)
        [[ "${seen_ready}" -eq 0 ]] || return 1
        marker_ready="${value}"
        seen_ready=1
        ;;
      *)
        return 1
        ;;
    esac
  done < "${INTERNAL_READY_FILE}"

  [[ "${seen_pid}${seen_ready}" == "11" \
    && "${marker_pid}" == "${CHILD_PID}" \
    && "${marker_ready}" == "1" ]]
}

case "${WARMUP_STEPS}" in
  ''|*[!0-9]*)
    fail "NUWAX_PERSON_FOLLOW_WARMUP_STEPS must be a positive integer"
    ;;
esac
[[ "${WARMUP_STEPS}" -gt 0 ]] \
  || fail "NUWAX_PERSON_FOLLOW_WARMUP_STEPS must be greater than zero"
[[ -x "${BINARY}" ]] || fail "binary is not executable: ${BINARY}"

mkdir -p "$(dirname "${READY_FILE}")"
rm -f "${READY_FILE}" "${READY_FILE}.tmp" \
  "${INTERNAL_READY_FILE}" "${INTERNAL_READY_FILE}.tmp"
trap cleanup EXIT
trap forward_shutdown INT TERM

NUWAX_PERSON_FOLLOW_INTERNAL_READY_FILE="${INTERNAL_READY_FILE}" \
  "${BINARY}" &
CHILD_PID="$!"
for ((step = 0; step < WARMUP_STEPS; step++)); do
  if ! kill -0 "${CHILD_PID}" >/dev/null 2>&1; then
    set +e
    wait "${CHILD_PID}"
    status=$?
    set -e
    fail "runtime exited during process readiness warmup with status ${status}"
  fi
  person_follow_internal_ready && break
  sleep 0.1
done
kill -0 "${CHILD_PID}" >/dev/null 2>&1 \
  || fail "runtime exited before process readiness publication"
person_follow_internal_ready \
  || fail "runtime did not prove typed subscription readiness within ${WARMUP_STEPS} checks"

BOOT_ID="$(nuwax_read_boot_id)" || fail "cannot read current boot identity"
CHILD_STARTTIME="$(nuwax_read_proc_starttime "${CHILD_PID}")" \
  || fail "cannot read child process starttime from /proc/${CHILD_PID}/stat"
printf 'pid=%s\nboot_id=%s\nstarttime=%s\nsubscriptions_ready=1\n' \
  "${CHILD_PID}" "${BOOT_ID}" "${CHILD_STARTTIME}" > "${READY_FILE}.tmp"
mv -f "${READY_FILE}.tmp" "${READY_FILE}"

set +e
wait "${CHILD_PID}"
status=$?
set -e
CHILD_PID=""
exit "${status}"
