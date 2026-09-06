#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
# 先解析共同配置并统一旧通道别名，再物化启动参数。
source "${REPO_ROOT}/services/geometry/source_runtime_env.sh"
RUNTIME_BINARY="${NUWAX_GEOMETRY_RUNTIME_BINARY:-${REPO_ROOT}/services/geometry/nuwax_geometry_runtime/target/release/nuwax-geometry-runtime}"
RELEASE_CHECKER="${REPO_ROOT}/services/geometry/release_manifest.py"
RELEASE_MANIFEST="${NUWAX_GEOMETRY_RELEASE_MANIFEST:-}"
READY_FILE="${NUWAX_GEOMETRY_RUNTIME_READY_FILE:-${REPO_ROOT}/runtime_data/geometry/nuwax_geometry_runtime.ready}"
INTERNAL_READY_FILE="${READY_FILE}.runtime"
EXEC_PROOF_FILE="${READY_FILE}.release"
PORT="${NUWAX_GEOMETRY_RUNTIME_PORT:-50051}"
HEALTH_TIMEOUT="${NUWAX_GEOMETRY_RUNTIME_HEALTH_TIMEOUT:-30}"
LIO_BACKEND="${NUWAX_GEOMETRY_LIO_BACKEND:-device}"
MAPPING_BACKEND="${NUWAX_GEOMETRY_MAPPING_BACKEND:-voxel}"
CORRECTION_BACKEND="${NUWAX_GEOMETRY_CORRECTION_BACKEND:-none}"
CHILD_PID=""
READY_MONITOR_PID=""

fail() {
  echo "nuwax-geometry-runtime launcher: $*" >&2
  exit 1
}

stop_geometry_child() {
  local child_status=0
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
  wait "${CHILD_PID}" >/dev/null 2>&1 || child_status=$?
  return "${child_status}"
}

stop_ready_monitor() {
  [[ -n "${READY_MONITOR_PID}" ]] || return 0
  kill "${READY_MONITOR_PID}" >/dev/null 2>&1 || true
  wait "${READY_MONITOR_PID}" >/dev/null 2>&1 || true
  READY_MONITOR_PID=""
}

cleanup() {
  local status=$?
  trap - EXIT
  stop_ready_monitor
  rm -f "${READY_FILE}" "${READY_FILE}.tmp" \
    "${INTERNAL_READY_FILE}" "${INTERNAL_READY_FILE}.tmp" \
    "${EXEC_PROOF_FILE}" 2>/dev/null || true
  stop_geometry_child || true
  exit "${status}"
}

forward_shutdown() {
  local status=0
  # 重复信号不得中断 wait，或让退出清理再次抢先关闭同一子进程。
  trap '' INT TERM
  stop_ready_monitor
  rm -f "${READY_FILE}" "${READY_FILE}.tmp" 2>/dev/null || true
  if [[ -n "${CHILD_PID}" ]]; then
    # 根拥有方负责依序回收；服务管理器沿用外层 20 秒最终关闭期限。
    kill -TERM "${CHILD_PID}" >/dev/null 2>&1 || true
    wait "${CHILD_PID}" || status=$?
    CHILD_PID=""
  fi
  exit "${status}"
}

geometry_internal_health_ready() {
  local internal_port=""
  [[ -f "${INTERNAL_READY_FILE}" && ! -L "${INTERNAL_READY_FILE}" ]] || return 1
  internal_port="$(< "${INTERNAL_READY_FILE}")" || return 1
  [[ "${internal_port}" =~ ^[1-9][0-9]*$ ]] || return 1
  [[ "${internal_port}" -le 65535 && "${internal_port}" == "${PORT}" ]]
}

monitor_geometry_ready_lease() {
  while kill -0 "${CHILD_PID}" >/dev/null 2>&1; do
    if ! geometry_internal_health_ready; then
      rm -f "${READY_FILE}" "${READY_FILE}.tmp" 2>/dev/null || true
    fi
    sleep 0.1
  done
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
case "${LIO_BACKEND}" in
  device|fastlio2|pointlio) ;;
  *) fail "NUWAX_GEOMETRY_LIO_BACKEND must be device, fastlio2, or pointlio" ;;
esac
case "${MAPPING_BACKEND}" in
  voxel|raytracing) ;;
  *) fail "NUWAX_GEOMETRY_MAPPING_BACKEND must be voxel or raytracing" ;;
esac
case "${CORRECTION_BACKEND}" in
  none|library_pgo|native_pgo|relocalization) ;;
  *) fail "NUWAX_GEOMETRY_CORRECTION_BACKEND must be none, library_pgo, native_pgo, or relocalization" ;;
esac
mkdir -p "$(dirname "${READY_FILE}")"
[[ -x "${RUNTIME_BINARY}" && ! -L "${RUNTIME_BINARY}" ]] \
  || fail "Geometry runtime binary is not an executable regular file: ${RUNTIME_BINARY}"
rm -f "${READY_FILE}" "${READY_FILE}.tmp" \
  "${INTERNAL_READY_FILE}" "${INTERNAL_READY_FILE}.tmp" "${EXEC_PROOF_FILE}"
trap cleanup EXIT
trap forward_shutdown INT TERM

export NUWAX_GEOMETRY_LIO_BACKEND="${LIO_BACKEND}"
export NUWAX_GEOMETRY_MAPPING_BACKEND="${MAPPING_BACKEND}"
export NUWAX_GEOMETRY_CORRECTION_BACKEND="${CORRECTION_BACKEND}"
# 开发启动使用项目数据目录；systemd（系统服务管理器）仍可显式指定持久化路径。
export NUWAX_GEOMETRY_SEQUENCE_STATE_PATH="${NUWAX_GEOMETRY_SEQUENCE_STATE_PATH:-${REPO_ROOT}/runtime_data/geometry/sequence_state.json}"
export NUWAX_GEOMETRY_EXPLORATION_RECONCILIATION_JOURNAL_PATH="${NUWAX_GEOMETRY_EXPLORATION_RECONCILIATION_JOURNAL_PATH:-${REPO_ROOT}/runtime_data/geometry/exploration_ownership_journal.json}"
printf 'nuwax-geometry-runtime launcher: owner profile lio=%s mapping=%s correction=%s\n' \
  "${LIO_BACKEND}" "${MAPPING_BACKEND}" "${CORRECTION_BACKEND}"

if [[ -n "${RELEASE_MANIFEST}" ]]; then
  NUWAX_GEOMETRY_RUNTIME_READY_FILE="${INTERNAL_READY_FILE}" \
    NUWAX_GEOMETRY_RUNTIME_PORT="${PORT}" \
    PYTHONPATH="${REPO_ROOT}" python3 "${RELEASE_CHECKER}" exec \
      --manifest "${RELEASE_MANIFEST}" --proof-output "${EXEC_PROOF_FILE}" &
else
  # 开发构建直接运行；只有显式选择发布清单时才进行发布制品验证。
  NUWAX_GEOMETRY_RUNTIME_READY_FILE="${INTERNAL_READY_FILE}" \
    NUWAX_GEOMETRY_RUNTIME_PORT="${PORT}" "${RUNTIME_BINARY}" &
fi
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
if [[ -n "${RELEASE_MANIFEST}" ]]; then
  [[ -f "${EXEC_PROOF_FILE}" && ! -L "${EXEC_PROOF_FILE}" ]] \
    || fail "verified runtime did not publish its immutable release proof"
  python3 "${RELEASE_CHECKER}" exec-proof \
    --manifest "${RELEASE_MANIFEST}" \
    --proof-output "${EXEC_PROOF_FILE}" \
    --pid "${CHILD_PID}" \
    || fail "verified runtime proof does not match the running release"
fi
printf 'pid=%s\nport=%s\n' "${CHILD_PID}" "${PORT}" > "${READY_FILE}.tmp"
mv -f "${READY_FILE}.tmp" "${READY_FILE}"

monitor_geometry_ready_lease &
READY_MONITOR_PID="$!"

set +e
wait "${CHILD_PID}"
status=$?
set -e
stop_ready_monitor
CHILD_PID=""
exit "${status}"
