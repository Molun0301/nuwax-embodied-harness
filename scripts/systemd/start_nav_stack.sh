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

# 模型/手填设置只在启动时解析，两个进程消费同一机体坐标外形。
NUWAX_ROBOT_GEOMETRY_JSON="$("${NUWAX_PYTHON_BIN:-python3}" "${REPO_ROOT}/scripts/resolve_robot_geometry.py")"
export NUWAX_ROBOT_GEOMETRY_JSON
IDENTITY_HELPER="${SCRIPT_DIR}/process_identity.sh"
BINARY="${NUWAX_NAV_STACK_BINARY:-${REPO_ROOT}/services/nuwax-workspace/target/release/nuwax-nav-stack}"
PROFILE="${NUWAX_NAV_PROFILE}"
READY_FILE="${NUWAX_NAV_STACK_READY_FILE:-${REPO_ROOT}/runtime_data/navigation/nuwax_nav_stack.ready}"
INTERNAL_READY_FILE="${READY_FILE}.runtime"
MOTION_GUARD_SOCKET="${NUWAX_MOTION_GUARD_SOCKET:-/run/nuwax/motion-guard.sock}"
WARMUP_STEPS="${NUWAX_NAV_STACK_WARMUP_STEPS:-150}"
EXPECTED_LAUNCHER_CONTRACT="nuwax-nav-stack-internal-ready-v1"
CHILD_PID=""

[[ -r "${IDENTITY_HELPER}" ]] || {
  echo "nuwax-nav-stack launcher: process identity helper is unavailable: ${IDENTITY_HELPER}" >&2
  exit 1
}
# shellcheck source=scripts/systemd/process_identity.sh
source "${IDENTITY_HELPER}"

fail() {
  echo "nuwax-nav-stack launcher: $*" >&2
  exit 1
}

stop_nav_child() {
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
}

cleanup_ready_files() {
  rm -f "${READY_FILE}" "${READY_FILE}.tmp" "${INTERNAL_READY_FILE}" 2>/dev/null || true
}

cleanup() {
  local status=$?
  trap - EXIT
  cleanup_ready_files
  stop_nav_child
  exit "${status}"
}

forward_shutdown() {
  stop_nav_child
  exit 0
}

internal_scheduler_ready() {
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

case "${PROFILE}" in
  simple-2d|pct-scan-3d)
    ;;
  *)
    fail "生产导航配置必须由二维或三维执行链确定：${PROFILE}"
    ;;
esac

case "${WARMUP_STEPS}" in
  ''|*[!0-9]*)
    fail "NUWAX_NAV_STACK_WARMUP_STEPS must be a positive integer"
    ;;
esac
[[ "${WARMUP_STEPS}" -gt 0 ]] || fail "NUWAX_NAV_STACK_WARMUP_STEPS must be greater than zero"
[[ -x "${BINARY}" ]] || fail "binary is not executable: ${BINARY}"
if ! ACTUAL_LAUNCHER_CONTRACT="$(timeout 2s "${BINARY}" --launcher-contract 2>/dev/null)" \
  || [[ "${ACTUAL_LAUNCHER_CONTRACT}" != "${EXPECTED_LAUNCHER_CONTRACT}" ]]; then
  fail "binary does not support the internal readiness protocol: ${BINARY}; rebuild it via scripts/build_nav_stack.sh"
fi

mkdir -p "$(dirname "${READY_FILE}")"
cleanup_ready_files
trap cleanup EXIT
trap forward_shutdown INT TERM

NUWAX_NAV_STACK_INTERNAL_READY_FILE="${INTERNAL_READY_FILE}" \
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
  if internal_scheduler_ready; then
    break
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

internal_scheduler_ready || \
  fail "runtime did not prove guard-backed scheduler readiness within ${WARMUP_STEPS} checks"

BOOT_ID="$(nuwax_read_boot_id)" || fail "cannot read current boot identity"
CHILD_STARTTIME="$(nuwax_read_proc_starttime "${CHILD_PID}")" \
  || fail "cannot read child process starttime from /proc/${CHILD_PID}/stat"

printf 'pid=%s\nboot_id=%s\nstarttime=%s\nprofile=%s\nscheduler_ready=1\n' \
  "${CHILD_PID}" "${BOOT_ID}" "${CHILD_STARTTIME}" "${PROFILE}" > "${READY_FILE}.tmp"
mv -f "${READY_FILE}.tmp" "${READY_FILE}"

set +e
wait "${CHILD_PID}"
status="$?"
set -e
exit "${status}"
