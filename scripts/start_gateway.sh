#!/usr/bin/env bash
set -euo pipefail

if [[ "${NUWAX_GATEWAY_SUPERVISOR_TITLED:-0}" != "1" && -n "${BASH:-}" ]]; then
  export NUWAX_GATEWAY_SUPERVISOR_TITLED=1
  exec -a nuwax-gateway-supervisor "${BASH}" "$0" "$@"
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
ENV_FILE="${NUWAX_BOOTSTRAP_ENV_FILE:-${REPO_ROOT}/.env}"
PYTHON_BIN="${NUWAX_PYTHON_BIN:-python3}"
RESTART_POLICY="${NUWAX_GATEWAY_RESTART:-on-failure}"
RESTART_SEC="${NUWAX_GATEWAY_RESTART_SEC:-3}"
RESTART_MAX_ATTEMPTS="${NUWAX_GATEWAY_RESTART_MAX_ATTEMPTS:-0}"
ACTION="${1:-start}"
PID_FILE=""
LOCK_FILE=""
LOCK_FD=""
GATEWAY_PID=""
VIDEO_STREAMER_PID=""
NAV_STACK_PID=""
GEOMETRY_RUNTIME_PID=""
PERSON_FOLLOW_PID=""
NAV_STACK_MONITOR_PID=""
GEOMETRY_MONITOR_PID=""
PERSON_FOLLOW_MONITOR_PID=""
RUNTIME_ACTIVATION_MONITOR_PID=""
RUNTIME_ACTIVATION_LEVEL=""
RUNTIME_ACTIVATION_CAPABILITY=""
STOP_REQUESTED=0

PERSON_FOLLOW_TARGET_CHANNEL="/person_follow_target"
PERSON_FOLLOW_POINTCLOUD_CHANNEL="/global_map"
PERSON_FOLLOW_TF_CHANNEL="/tf"
PERSON_FOLLOW_COMMAND_CHANNEL="/gateway/person_follow_cmd_vel"

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
    if [[ "${value}" == *[[:space:]]\#* ]]; then
      value="$(trim_whitespace "${value%%#*}")"
    fi
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
    if [[ -z "${!key+x}" ]]; then
      export "${key}=${value}"
    fi
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

geometry_runtime_managed_by_systemd() {
  is_truthy "${NUWAX_GEOMETRY_RUNTIME_MANAGED_BY_SYSTEMD:-false}"
}

nav_stack_managed_by_systemd() {
  is_truthy "${NUWAX_NAV_STACK_MANAGED_BY_SYSTEMD:-false}"
}

person_follow_managed_by_systemd() {
  is_truthy "${NUWAX_PERSON_FOLLOW_MANAGED_BY_SYSTEMD:-false}"
}

go2_video_streamer_managed_by_systemd() {
  is_truthy "${NUWAX_GO2_VIDEO_STREAMER_MANAGED_BY_SYSTEMD:-false}"
}

person_follow_autostart_enabled() {
  is_truthy "${NUWAX_PERSON_FOLLOW_AUTOSTART:-true}"
}

default_gateway_lock_file() {
  if [[ -d /run/nuwax/gateway ]]; then
    printf '%s' /run/nuwax/gateway/start_gateway.lock
    return
  fi
  printf '%s' "${REPO_ROOT}/runtime_data/gateway/start_gateway.lock"
}

validate_person_follow_monitor_interval() {
  local interval="${NUWAX_PERSON_FOLLOW_MONITOR_INTERVAL_SEC:-0.5}"
  if [[ ! "${interval}" =~ ^[0-9]+([.][0-9]+)?$ || "${interval}" =~ ^0+([.]0+)?$ ]]; then
    echo "invalid NUWAX_PERSON_FOLLOW_MONITOR_INTERVAL_SEC=${interval}" >&2
    return 1
  fi
}

person_follow_binary() {
  printf '%s' "${NUWAX_PERSON_FOLLOW_BINARY:-${REPO_ROOT}/services/nuwax-workspace/target/release/nuwax-person-follow}"
}

person_follow_is_alive() {
  local pid="$1"
  local state=""
  if ! is_pid_alive "${pid}"; then
    return 1
  fi
  state="$(ps -o stat= -p "${pid}" 2>/dev/null | tr -d '[:space:]' || true)"
  [[ -n "${state}" && "${state:0:1}" != "Z" ]]
}

wait_for_motion_guard_transport() {
  local helper="${REPO_ROOT}/scripts/systemd/wait_for_motion_guard_socket.sh"
  local timeout_sec="${NUWAX_MOTION_GUARD_START_TIMEOUT_SEC:-15}"
  local attempts=0
  case "${timeout_sec}" in
    ''|*[!0-9]*)
      echo "invalid NUWAX_MOTION_GUARD_START_TIMEOUT_SEC=${timeout_sec}" >&2
      return 1
      ;;
  esac
  attempts=$((timeout_sec * 10 + 1))

  for ((attempt = 0; attempt < attempts; attempt++)); do
    if bash "${helper}" check >/dev/null 2>&1; then
      echo "motion guard transport ready: ${NUWAX_MOTION_GUARD_SOCKET:-/run/nuwax/motion-guard.sock}"
      return 0
    fi
    sleep 0.1
  done

  bash "${helper}" check
  echo "refusing to start the motion-capable navigation stack without motion guard transport" >&2
  return 1
}

wait_for_geometry_runtime_ready() {
  local ready_file="${NUWAX_GEOMETRY_RUNTIME_READY_FILE:-${REPO_ROOT}/runtime_data/geometry/nuwax_geometry_runtime.ready}"
  NUWAX_GEOMETRY_RUNTIME_READY_FILE="${ready_file}" \
    bash "${REPO_ROOT}/scripts/systemd/wait_for_geometry_runtime_ready.sh" wait
  echo "nuwax-geometry-runtime ready: ${ready_file}"
}

wait_for_nav_stack_ready() {
  local helper="${REPO_ROOT}/scripts/systemd/wait_for_nav_stack_ready.sh"
  local ready_file="${NUWAX_NAV_STACK_READY_FILE:-${REPO_ROOT}/runtime_data/navigation/nuwax_nav_stack.ready}"
  NUWAX_NAV_STACK_READY_FILE="${ready_file}" \
    bash "${helper}" wait
  echo "nuwax-nav-stack ready: ${ready_file}"
}

wait_for_person_follow_ready() {
  local ready_file="${NUWAX_PERSON_FOLLOW_READY_FILE:-${REPO_ROOT}/runtime_data/person-follow/nuwax_person_follow.ready}"
  NUWAX_PERSON_FOLLOW_READY_FILE="${ready_file}" \
    bash "${REPO_ROOT}/scripts/systemd/wait_for_person_follow_ready.sh" wait
  echo "nuwax-person-follow ready: ${ready_file}"
}

run_with_title() {
  local process_title="$1"
  shift
  if [[ -n "${BASH_VERSION:-}" ]]; then
    exec -a "${process_title}" "$@"
  fi
  exec "$@"
}

acquire_script_lock() {
  local lock_path="$1"
  mkdir -p "$(dirname "${lock_path}")"
  # shellcheck disable=SC3045
  exec {LOCK_FD}>"${lock_path}"
  if ! flock -n "${LOCK_FD}"; then
    echo "another start_gateway.sh instance is running (lock: ${lock_path})." >&2
    echo "please stop existing one before running '${ACTION}' again." >&2
    exit 1
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

pid_pgid() {
  local pid="$1"
  ps -o pgid= -p "${pid}" 2>/dev/null | tr -d '[:space:]' || true
}

send_signal_to_process_scope() {
  local signal="$1"
  local pid="$2"
  local pgid=""
  local self_pgid=""

  if ! is_pid_alive "${pid}"; then
    return 0
  fi

  pgid="$(pid_pgid "${pid}")"
  self_pgid="$(pid_pgid "$$")"
  if [[ -n "${pgid}" && "${pgid}" != "${self_pgid}" ]]; then
    kill "-${signal}" -- "-${pgid}" >/dev/null 2>&1 || true
  fi
  kill "-${signal}" -- "${pid}" >/dev/null 2>&1 || true
}

wait_for_pid_exit() {
  local pid="$1"
  local timeout_sec="$2"
  local waited=0

  while is_pid_alive "${pid}"; do
    if [[ "${waited}" -ge "${timeout_sec}" ]]; then
      return 1
    fi
    sleep 1
    waited=$((waited + 1))
  done
  return 0
}

terminate_process_scope() {
  local pid="$1"
  local label="${2:-process}"
  local stop_timeout="${NUWAX_GATEWAY_STOP_TIMEOUT_SEC:-20}"
  local kill_timeout="${NUWAX_GATEWAY_KILL_TIMEOUT_SEC:-5}"

  if ! is_pid_alive "${pid}"; then
    return 0
  fi

  echo "stopping ${label} pid ${pid}..."
  send_signal_to_process_scope TERM "${pid}"
  send_signal_to_process_scope CONT "${pid}"
  if wait_for_pid_exit "${pid}" "${stop_timeout}"; then
    return 0
  fi

  echo "${label} pid ${pid} did not stop after ${stop_timeout}s; sending SIGKILL." >&2
  send_signal_to_process_scope KILL "${pid}"
  wait_for_pid_exit "${pid}" "${kill_timeout}" || true
}

terminate_person_follow_scope() {
  local pid="$1"
  local stop_timeout="${NUWAX_GATEWAY_STOP_TIMEOUT_SEC:-20}"
  local kill_timeout="${NUWAX_GATEWAY_KILL_TIMEOUT_SEC:-5}"

  if ! person_follow_is_alive "${pid}"; then
    return 0
  fi

  echo "stopping nuwax-person-follow pid ${pid}..."
  # The Rust owner handles SIGINT and publishes a final typed zero during teardown.
  send_signal_to_process_scope INT "${pid}"
  send_signal_to_process_scope CONT "${pid}"
  if wait_for_pid_exit "${pid}" "${stop_timeout}"; then
    return 0
  fi

  echo "nuwax-person-follow pid ${pid} did not stop after ${stop_timeout}s; sending SIGTERM." >&2
  send_signal_to_process_scope TERM "${pid}"
  if wait_for_pid_exit "${pid}" "${kill_timeout}"; then
    return 0
  fi

  echo "nuwax-person-follow pid ${pid} ignored SIGTERM; sending SIGKILL." >&2
  send_signal_to_process_scope KILL "${pid}"
  wait_for_pid_exit "${pid}" "${kill_timeout}" || true
}

cleanup_stale_gateway_processes() {
  local pid=""
  local args=""

  while read -r pid args; do
    if [[ -z "${pid}" || "${pid}" == "$$" || "${pid}" == "${BASHPID}" ]]; then
      continue
    fi
    if [[ "${args}" == *"uvicorn gateways.app:create_gateway_app_from_env"* && "${args}" == *"--port ${PORT}"* ]]; then
      terminate_process_scope "${pid}" "stale gateway worker"
    fi
  done < <(ps -eo pid=,args=)
}

cleanup_stale_geometry_runtime_processes() {
  if geometry_runtime_managed_by_systemd; then
    return 0
  fi

  local binary="${REPO_ROOT}/services/geometry/nuwax_geometry_runtime/target/release/nuwax-geometry-runtime"
  local pid=""
  local args=""

  while read -r pid args; do
    if [[ -z "${pid}" || "${pid}" == "$$" || "${pid}" == "${BASHPID}" ]]; then
      continue
    fi
    if [[ "${args}" == *"nuwax-geometry-runtime"* ]]; then
      terminate_process_scope "${pid}" "stale geometry runtime"
    fi
  done < <(ps -eo pid=,args=)

  # 兜底清理：nuwax-geometry-runtime 已被 pgrep 杀过但 gRPC listen socket 仍处
  # TIME_WAIT，或被 SIGKILL 后 socket 未立即释放。下次 spawn 会因 port-in-use 失败，
  # 平台侧 GeometryRuntimeProvider 端口探测会误报"端口已开"导致 attach 到僵尸。
  local runtime_port="${NUWAX_GEOMETRY_RUNTIME_PORT:-50051}"
  if command -v ss >/dev/null 2>&1; then
    if ss -ltn 2>/dev/null | awk '{print $4}' | grep -qE "(^|:)${runtime_port}$"; then
      if ! pgrep -f "nuwax-geometry-runtime" >/dev/null 2>&1; then
        echo "==> port ${runtime_port} still LISTEN but no nuwax-geometry-runtime process; clearing residual socket"
        if command -v fuser >/dev/null 2>&1; then
          fuser -KILL "${runtime_port}/tcp" >/dev/null 2>&1 || true
        else
          if command -v lsof >/dev/null 2>&1; then
            local residual_pid
            residual_pid="$(lsof -ti tcp:${runtime_port} 2>/dev/null || true)"
            if [[ -n "${residual_pid}" ]]; then
              kill -KILL ${residual_pid} >/dev/null 2>&1 || true
            fi
          else
            echo "WARNING: fuser/lsof 都不可用，端口 ${runtime_port} 残留可能让 spawn 失败" >&2
          fi
        fi
        sleep 0.5
      fi
    fi
  fi

  rm -f /tmp/nuwax_video.sock /tmp/nuwax_video.shm 2>/dev/null || true
}

cleanup_stale_person_follow_processes() {
  if person_follow_managed_by_systemd; then
    return 0
  fi

  local pid=""
  local args=""
  while read -r pid args; do
    if [[ -z "${pid}" || "${pid}" == "$$" || "${pid}" == "${BASHPID}" ]]; then
      continue
    fi
    if [[ "${args}" == *"nuwax-person-follow"* ]]; then
      terminate_person_follow_scope "${pid}"
    fi
  done < <(ps -eo pid=,args=)
}

cleanup_stale_go2_video_streamer() {
  if go2_video_streamer_managed_by_systemd; then
    return 0
  fi

  # 终止任何残留的 go2 视频推流进程（无论由本脚本、systemd 还是手动启动），
  # 保证网关成为唯一管理者，避免"网关停了 streamer 还在"以及双拉起。
  local pid=""
  local args=""

  while read -r pid args; do
    if [[ -z "${pid}" || "${pid}" == "$$" || "${pid}" == "${BASHPID}" ]]; then
      continue
    fi
    if [[ "${args}" == *"drivers.go2.video_streamer"* ]]; then
      terminate_process_scope "${pid}" "stale go2 video streamer"
    fi
  done < <(ps -eo pid=,args=)
}

cleanup_external_geometry_processes() {
  local pid=""
  local args=""
  local lcm_patterns=(
    "nuwax-geometry-runtime"
    "nuwax-lcm-runtime"
    "nuwax-nav-stack"
    "nuwax-video-streamer"
  )

  if ! is_truthy "${NUWAX_GATEWAY_CLEANUP_EXTERNAL_GEOMETRY:-true}"; then
    return 0
  fi

  while read -r pid args; do
    if [[ -z "${pid}" || "${pid}" == "$$" || "${pid}" == "${BASHPID}" ]]; then
      continue
    fi
    for pattern in "${lcm_patterns[@]}"; do
      if [[ "${args}" == *"${pattern}"* ]]; then
        terminate_process_scope "${pid}" "external LCM/native geometry process (${pattern})"
        break
      fi
    done
  done < <(ps -eo pid=,args=)

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

  pid="$(read_pid_file || true)"
  if [[ -z "${pid}" ]]; then
    echo "gateway pid file not found: ${PID_FILE}"
    cleanup_stale_gateway_processes
    cleanup_stale_person_follow_processes
    cleanup_stale_geometry_runtime_processes
    cleanup_stale_go2_video_streamer
    cleanup_external_geometry_processes
    return 0
  fi

  if ! is_pid_alive "${pid}"; then
    echo "gateway pid ${pid} is not running; removing stale pid file."
    rm -f "${PID_FILE}"
    cleanup_stale_gateway_processes
    cleanup_stale_person_follow_processes
    cleanup_stale_geometry_runtime_processes
    cleanup_stale_go2_video_streamer
    cleanup_external_geometry_processes
    return 0
  fi

  terminate_process_scope "${pid}" "gateway supervisor"
  rm -f "${PID_FILE}"
  # 顺序：先杀网关 worker（切断 geometry/perception 客户端调用）→
  # 再停 ros-bridge（避免重启窗口期 GetFrameTree 抛 UNAVAILABLE 刷日志）→
  # 最后清理外部几何进程。
  cleanup_stale_gateway_processes
  cleanup_stale_person_follow_processes
  cleanup_stale_geometry_runtime_processes
  cleanup_external_geometry_processes
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

should_start_geometry_runtime() {
  local binary="${NUWAX_GEOMETRY_RUNTIME_BINARY:-${REPO_ROOT}/services/geometry/nuwax_geometry_runtime/target/release/nuwax-geometry-runtime}"
  [[ -x "${binary}" ]] && is_truthy "${NUWAX_GEOMETRY_EXTERNAL_ENABLED:-true}"
}

should_start_nav_stack() {
  local binary="${NUWAX_NAV_STACK_BINARY:-${REPO_ROOT}/services/nuwax-workspace/target/release/nuwax-nav-stack}"
  [[ -x "${binary}" ]] && is_truthy "${NUWAX_NAV_AUTOSTART:-true}"
}

should_start_person_follow() {
  local binary=""
  binary="$(person_follow_binary)"
  [[ -x "${binary}" ]] && person_follow_autostart_enabled
}

should_start_go2_video_streamer() {
  is_truthy "${GO2_VIDEO_STREAM_ENABLED:-true}" \
    && [[ "$(printf '%s' "${NUWAX_ACTIVE_ROBOT:-}" | tr '[:upper:]' '[:lower:]')" == "go2" ]]
}

start_nav_stack() {
  if [[ -n "${NAV_STACK_PID}" ]] && is_pid_alive "${NAV_STACK_PID}"; then
    return 0
  fi
  if nav_stack_managed_by_systemd; then
    wait_for_nav_stack_ready
    return
  fi
  if ! is_truthy "${NUWAX_NAV_AUTOSTART:-true}"; then
    echo "nuwax-nav-stack disabled by NUWAX_NAV_AUTOSTART"
    return 0
  fi
  if ! should_start_nav_stack; then
    echo "nuwax-nav-stack is enabled but its production binary is not executable; run scripts/build_nav_stack.sh" >&2
    return 1
  fi

  local launcher="${REPO_ROOT}/scripts/systemd/start_nav_stack.sh"
  export NUWAX_NAV_STACK_READY_FILE="${NUWAX_NAV_STACK_READY_FILE:-${REPO_ROOT}/runtime_data/navigation/nuwax_nav_stack.ready}"
  export NUWAX_LCM_URL="${NUWAX_LCM_URL:-udpm://239.255.76.67:7667?ttl=1}"
  echo "starting nuwax-nav-stack profile=${NUWAX_NAV_PROFILE:-simple-2d}"
  (run_with_title "nuwax-nav-stack-launcher" bash "${launcher}") &
  NAV_STACK_PID="$!"

  if wait_for_nav_stack_ready; then
    echo "nuwax-nav-stack healthy (launcher pid ${NAV_STACK_PID})"
    return 0
  fi
  terminate_process_scope "${NAV_STACK_PID}" "unready nuwax-nav-stack"
  wait "${NAV_STACK_PID}" >/dev/null 2>&1 || true
  NAV_STACK_PID=""
  return 1
}

start_geometry_runtime() {
  # 通过通用 env 加载器启动原生几何运行时。
  if [[ -n "${GEOMETRY_RUNTIME_PID}" ]] && is_pid_alive "${GEOMETRY_RUNTIME_PID}"; then
    return 0
  fi
  if geometry_runtime_managed_by_systemd; then
    wait_for_geometry_runtime_ready
    return
  fi
  if ! is_truthy "${NUWAX_GEOMETRY_EXTERNAL_ENABLED:-true}"; then
    echo "nuwax-geometry-runtime disabled by NUWAX_GEOMETRY_EXTERNAL_ENABLED"
    return 0
  fi
  if ! should_start_geometry_runtime; then
    echo "nuwax-geometry-runtime is enabled but its binary is not executable" >&2
    return 1
  fi
  local binary="${NUWAX_GEOMETRY_RUNTIME_BINARY:-${REPO_ROOT}/services/geometry/nuwax_geometry_runtime/target/release/nuwax-geometry-runtime}"
  local launcher="${REPO_ROOT}/scripts/systemd/start_geometry_runtime.sh"
  echo "starting nuwax-geometry-runtime: ${binary}"
  (
    set +u
    # shellcheck disable=SC1091
    source "${REPO_ROOT}/services/geometry/source_runtime_env.sh"
    export NUWAX_GEOMETRY_RUNTIME_PORT="${NUWAX_GEOMETRY_RUNTIME_PORT:-50051}"
    export NUWAX_GEOMETRY_RUNTIME_READY_FILE="${NUWAX_GEOMETRY_RUNTIME_READY_FILE:-${REPO_ROOT}/runtime_data/geometry/nuwax_geometry_runtime.ready}"
    export NUWAX_LCM_URL="${NUWAX_LCM_URL:-udpm://239.255.76.67:7667?ttl=1}"
    export NUWAX_GEOMETRY_RUNTIME_POINTCLOUD_CHANNEL="${NUWAX_GEOMETRY_RUNTIME_POINTCLOUD_CHANNEL:-${NUWAX_LCM_CHANNEL_POINTCLOUD:-/go2/lidar/points#sensor_msgs.PointCloud2}}"
    export NUWAX_GEOMETRY_RUNTIME_CLOUDMAP_CHANNEL="${NUWAX_GEOMETRY_RUNTIME_CLOUDMAP_CHANNEL:-${NUWAX_LCM_CHANNEL_CLOUD_MAP:-/cloud_map#sensor_msgs.PointCloud2}}"
    export NUWAX_GEOMETRY_RUNTIME_NAV_GOAL_CHANNEL="/nav/goal#nuwax_msgs.GoalEnvelope"
    export NUWAX_LCM_CHANNEL_GOAL_POSE="/nav/goal#nuwax_msgs.GoalEnvelope"
    export NUWAX_GEOMETRY_RUNTIME_NAV_PATH_CHANNEL="/nav/path#nuwax_msgs.PathEnvelope"
    export NUWAX_LCM_CHANNEL_NAV_PATH="/nav/path#nuwax_msgs.PathEnvelope"
    export NUWAX_GEOMETRY_RUNTIME_BINARY="${binary}"
    run_with_title "nuwax-geometry-runtime-launcher" bash "${launcher}"
  ) &
  GEOMETRY_RUNTIME_PID="$!"
  if wait_for_geometry_runtime_ready; then
    echo "nuwax-geometry-runtime healthy (launcher pid ${GEOMETRY_RUNTIME_PID})"
    return 0
  fi
  terminate_process_scope "${GEOMETRY_RUNTIME_PID}" "unready nuwax-geometry-runtime"
  wait "${GEOMETRY_RUNTIME_PID}" >/dev/null 2>&1 || true
  GEOMETRY_RUNTIME_PID=""
  return 1
}

start_person_follow() {
  local binary=""
  local status=1
  local launcher="${REPO_ROOT}/scripts/systemd/start_person_follow.sh"

  if [[ -n "${PERSON_FOLLOW_PID}" ]] && person_follow_is_alive "${PERSON_FOLLOW_PID}"; then
    return 0
  fi
  if person_follow_managed_by_systemd; then
    return 0
  fi
  if ! person_follow_autostart_enabled; then
    echo "nuwax-person-follow disabled by NUWAX_PERSON_FOLLOW_AUTOSTART"
    return 0
  fi
  if ! should_start_person_follow; then
    echo "nuwax-person-follow is enabled but its production binary is not executable: $(person_follow_binary)" >&2
    return 1
  fi
  if ! validate_person_follow_monitor_interval; then
    return 1
  fi
  binary="$(person_follow_binary)"
  echo "starting nuwax-person-follow: ${binary}"
  (
    # These unsuffixed channels are frozen. The Rust transport appends the LCM type.
    export NUWAX_PERSON_FOLLOW_TARGET_CHANNEL="${PERSON_FOLLOW_TARGET_CHANNEL}"
    export NUWAX_PERSON_FOLLOW_POINTCLOUD_CHANNEL="${PERSON_FOLLOW_POINTCLOUD_CHANNEL}"
    export NUWAX_PERSON_FOLLOW_TF_CHANNEL="${PERSON_FOLLOW_TF_CHANNEL}"
    export NUWAX_PERSON_FOLLOW_COMMAND_CHANNEL="${PERSON_FOLLOW_COMMAND_CHANNEL}"
    export NUWAX_PERSON_FOLLOW_READY_FILE="${NUWAX_PERSON_FOLLOW_READY_FILE:-${REPO_ROOT}/runtime_data/person-follow/nuwax_person_follow.ready}"
    export NUWAX_PERSON_FOLLOW_BINARY="${binary}"
    export NUWAX_LCM_URL="${NUWAX_LCM_URL:-udpm://239.255.76.67:7667?ttl=1}"
    run_with_title "nuwax-person-follow-launcher" bash "${launcher}"
  ) &
  PERSON_FOLLOW_PID="$!"

  if wait_for_person_follow_ready; then
    echo "nuwax-person-follow healthy (launcher pid ${PERSON_FOLLOW_PID})"
    return 0
  fi

  terminate_person_follow_scope "${PERSON_FOLLOW_PID}"
  if wait "${PERSON_FOLLOW_PID}"; then
    status=0
  else
    status="$?"
  fi
  echo "nuwax-person-follow failed subscription readiness with status ${status}" >&2
  PERSON_FOLLOW_PID=""
  return 1
}

abort_person_follow_startup() {
  echo "nuwax-person-follow failed to start; refusing to start Gateway" >&2
  STOP_REQUESTED=1
  stop_person_follow_monitor
  stop_runtime_monitors
  stop_person_follow
  stop_geometry_runtime
  stop_nav_stack
  exit 1
}

start_go2_video_streamer() {
  if go2_video_streamer_managed_by_systemd; then
    return 0
  fi
  if [[ -n "${VIDEO_STREAMER_PID}" ]] && is_pid_alive "${VIDEO_STREAMER_PID}"; then
    return 0
  fi
  if ! should_start_go2_video_streamer; then
    return 0
  fi
  if ! geometry_runtime_managed_by_systemd; then
    start_geometry_runtime
  fi
  echo "starting Go2 side video streamer -> UDS /tmp/nuwax_video.sock"
  (run_with_title "nuwax-go2-video-streamer" "${PYTHON_BIN}" -m drivers.go2.video_streamer) &
  VIDEO_STREAMER_PID="$!"
}

stop_geometry_runtime() {
  if geometry_runtime_managed_by_systemd; then
    return 0
  fi
  if [[ -n "${GEOMETRY_RUNTIME_PID}" ]] && is_pid_alive "${GEOMETRY_RUNTIME_PID}"; then
    terminate_process_scope "${GEOMETRY_RUNTIME_PID}" "nuwax-geometry-runtime"
    wait "${GEOMETRY_RUNTIME_PID}" >/dev/null 2>&1 || true
  fi
  GEOMETRY_RUNTIME_PID=""
  rm -f /tmp/nuwax_video.sock /tmp/nuwax_video.shm 2>/dev/null || true
  rm -f "${NUWAX_GEOMETRY_RUNTIME_READY_FILE:-${REPO_ROOT}/runtime_data/geometry/nuwax_geometry_runtime.ready}" 2>/dev/null || true
}

stop_nav_stack() {
  if nav_stack_managed_by_systemd; then
    return 0
  fi
  if [[ -n "${NAV_STACK_PID}" ]] && is_pid_alive "${NAV_STACK_PID}"; then
    terminate_process_scope "${NAV_STACK_PID}" "nuwax-nav-stack"
    wait "${NAV_STACK_PID}" >/dev/null 2>&1 || true
  fi
  NAV_STACK_PID=""
  rm -f "${NUWAX_NAV_STACK_READY_FILE:-${REPO_ROOT}/runtime_data/navigation/nuwax_nav_stack.ready}" 2>/dev/null || true
}

stop_person_follow() {
  if person_follow_managed_by_systemd; then
    return 0
  fi
  if [[ -n "${PERSON_FOLLOW_PID}" ]]; then
    if person_follow_is_alive "${PERSON_FOLLOW_PID}"; then
      terminate_person_follow_scope "${PERSON_FOLLOW_PID}"
    fi
    wait "${PERSON_FOLLOW_PID}" >/dev/null 2>&1 || true
  fi
  PERSON_FOLLOW_PID=""
}

monitor_geometry_runtime() {
  if geometry_runtime_managed_by_systemd; then
    return 0
  fi
  while [[ "${STOP_REQUESTED}" -eq 0 ]]; do
    sleep 5
    if [[ -n "${GEOMETRY_RUNTIME_PID}" ]] && ! is_pid_alive "${GEOMETRY_RUNTIME_PID}"; then
      echo "nuwax-geometry-runtime (pid ${GEOMETRY_RUNTIME_PID}) crashed, restarting..." >&2
      rm -f /tmp/nuwax_video.sock /tmp/nuwax_video.shm 2>/dev/null || true
      GEOMETRY_RUNTIME_PID=""
      start_geometry_runtime
    fi
  done
}

monitor_nav_stack() {
  if nav_stack_managed_by_systemd || ! is_truthy "${NUWAX_NAV_AUTOSTART:-true}"; then
    return 0
  fi
  while [[ "${STOP_REQUESTED}" -eq 0 ]]; do
    sleep 5
    if [[ -n "${NAV_STACK_PID}" ]] && ! is_pid_alive "${NAV_STACK_PID}"; then
      echo "nuwax-nav-stack launcher (pid ${NAV_STACK_PID}) crashed, restarting..." >&2
      wait "${NAV_STACK_PID}" >/dev/null 2>&1 || true
      NAV_STACK_PID=""
      if ! start_nav_stack; then
        echo "nuwax-nav-stack restart failed; stopping gateway worker" >&2
        if [[ -n "${GATEWAY_PID}" ]] && is_pid_alive "${GATEWAY_PID}"; then
          send_signal_to_process_scope TERM "${GATEWAY_PID}"
        fi
        return 1
      fi
    fi
  done
}

monitor_person_follow() {
  local watched_person_follow_pid="${PERSON_FOLLOW_PID}"
  local watched_gateway_pid="${GATEWAY_PID}"
  local interval="${NUWAX_PERSON_FOLLOW_MONITOR_INTERVAL_SEC:-0.5}"

  if person_follow_managed_by_systemd || ! person_follow_autostart_enabled; then
    return 0
  fi
  if [[ -z "${watched_person_follow_pid}" ]]; then
    return 0
  fi

  while [[ "${STOP_REQUESTED}" -eq 0 ]]; do
    sleep "${interval}"
    if ! person_follow_is_alive "${watched_person_follow_pid}"; then
      echo "nuwax-person-follow (pid ${watched_person_follow_pid}) crashed; stopping gateway worker" >&2
      if [[ -n "${watched_gateway_pid}" ]] && is_pid_alive "${watched_gateway_pid}"; then
        send_signal_to_process_scope TERM "${watched_gateway_pid}"
      fi
      return 1
    fi
  done
}

start_person_follow_monitor() {
  if [[ -n "${PERSON_FOLLOW_MONITOR_PID}" ]] && is_pid_alive "${PERSON_FOLLOW_MONITOR_PID}"; then
    return 0
  fi
  monitor_person_follow &
  PERSON_FOLLOW_MONITOR_PID="$!"
}

stop_person_follow_monitor() {
  if [[ -n "${PERSON_FOLLOW_MONITOR_PID}" ]] && is_pid_alive "${PERSON_FOLLOW_MONITOR_PID}"; then
    kill -TERM "${PERSON_FOLLOW_MONITOR_PID}" >/dev/null 2>&1 || true
  fi
  if [[ -n "${PERSON_FOLLOW_MONITOR_PID}" ]]; then
    wait "${PERSON_FOLLOW_MONITOR_PID}" >/dev/null 2>&1 || true
  fi
  PERSON_FOLLOW_MONITOR_PID=""
}

start_runtime_monitors() {
  if [[ -z "${NAV_STACK_MONITOR_PID}" ]] || ! is_pid_alive "${NAV_STACK_MONITOR_PID}"; then
    monitor_nav_stack &
    NAV_STACK_MONITOR_PID="$!"
  fi
  if [[ -z "${GEOMETRY_MONITOR_PID}" ]] || ! is_pid_alive "${GEOMETRY_MONITOR_PID}"; then
    monitor_geometry_runtime &
    GEOMETRY_MONITOR_PID="$!"
  fi
}

write_runtime_activation_result() {
  local state="$1"
  local stage="$2"
  local message="$3"
  local result_file="${NUWAX_RUNTIME_ACTIVATION_RESULT}"
  local temporary="${result_file}.$$"
  STATE="${state}" STAGE="${stage}" MESSAGE="${message}" \
    ACTIVATION_LEVEL="${RUNTIME_ACTIVATION_LEVEL:-unknown}" \
    CAPABILITY_NAME="${RUNTIME_ACTIVATION_CAPABILITY:-unknown}" \
    "${PYTHON_BIN}" -c 'import json, os; print(json.dumps({"state": os.environ["STATE"], "stage": os.environ["STAGE"], "message": os.environ["MESSAGE"], "activation_level": os.environ["ACTIVATION_LEVEL"], "capability_name": os.environ["CAPABILITY_NAME"]}, ensure_ascii=True, sort_keys=True))' \
    > "${temporary}"
  chmod 0644 "${temporary}"
  mv -f "${temporary}" "${result_file}"
}

read_runtime_activation_request() {
  local request_file="$1"
  local parsed=""
  parsed="$("${PYTHON_BIN}" - "${request_file}" <<'PY'
import json
import pathlib
import sys

payload = json.loads(pathlib.Path(sys.argv[1]).read_text(encoding="utf-8"))
level = str(payload.get("activation_level") or "motion").strip().lower()
if level not in {"guard", "geometry", "motion", "person_follow"}:
    raise SystemExit("invalid activation_level: %s" % level)
capability = str(payload.get("capability_name") or "unknown").strip() or "unknown"
print("%s\t%s" % (level, capability))
PY
  )" || return 1
  IFS=$'\t' read -r RUNTIME_ACTIVATION_LEVEL RUNTIME_ACTIVATION_CAPABILITY <<< "${parsed}"
  [[ -n "${RUNTIME_ACTIVATION_LEVEL}" && -n "${RUNTIME_ACTIVATION_CAPABILITY}" ]]
}

monitor_runtime_activation_requests() {
  local request_file="${NUWAX_RUNTIME_ACTIVATION_REQUEST}"
  if geometry_runtime_managed_by_systemd && nav_stack_managed_by_systemd; then
    return 0
  fi
  while [[ "${STOP_REQUESTED}" -eq 0 ]]; do
    if [[ ! -f "${request_file}" ]]; then
      sleep 0.1
      continue
    fi
    rm -f "${NUWAX_RUNTIME_ACTIVATION_RESULT}"
    if ! read_runtime_activation_request "${request_file}"; then
      RUNTIME_ACTIVATION_LEVEL="invalid"
      RUNTIME_ACTIVATION_CAPABILITY="unknown"
      write_runtime_activation_result failed request "invalid runtime activation request"
      rm -f "${request_file}"
      continue
    fi
    if [[ "${RUNTIME_ACTIVATION_LEVEL}" == "guard" \
      || "${RUNTIME_ACTIVATION_LEVEL}" == "motion" \
      || "${RUNTIME_ACTIVATION_LEVEL}" == "person_follow" ]]; then
      write_runtime_activation_result activating motion_guard "waiting for motion guard component"
      if ! wait_for_motion_guard_transport; then
        write_runtime_activation_result failed motion_guard "motion guard is unavailable"
        rm -f "${request_file}"
        continue
      fi
    fi
    if [[ "${RUNTIME_ACTIVATION_LEVEL}" == "motion" \
      || "${RUNTIME_ACTIVATION_LEVEL}" == "person_follow" ]]; then
      write_runtime_activation_result activating navigation "starting navigation component"
      if ! start_nav_stack; then
        write_runtime_activation_result failed navigation "navigation stack failed to start"
        rm -f "${request_file}"
        continue
      fi
    fi
    if [[ "${RUNTIME_ACTIVATION_LEVEL}" == "geometry" \
      || "${RUNTIME_ACTIVATION_LEVEL}" == "motion" \
      || "${RUNTIME_ACTIVATION_LEVEL}" == "person_follow" ]]; then
      write_runtime_activation_result activating starting "starting geometry component"
      if ! start_geometry_runtime; then
        write_runtime_activation_result failed geometry "geometry runtime failed to start"
        rm -f "${request_file}"
        continue
      fi
      start_runtime_monitors
    fi
    if [[ "${RUNTIME_ACTIVATION_LEVEL}" == "person_follow" ]]; then
      write_runtime_activation_result activating person_follow "starting person-follow component"
      if ! start_person_follow; then
        write_runtime_activation_result failed person_follow "person-follow failed to start"
        rm -f "${request_file}"
        continue
      fi
      if person_follow_managed_by_systemd && ! wait_for_person_follow_ready; then
        write_runtime_activation_result failed person_follow "person-follow is not ready"
        rm -f "${request_file}"
        continue
      fi
      start_person_follow_monitor
    fi
    write_runtime_activation_result ready ready "requested runtime components are ready"
    rm -f "${request_file}"
  done
}

start_runtime_activation_monitor() {
  if geometry_runtime_managed_by_systemd && nav_stack_managed_by_systemd; then
    return 0
  fi
  monitor_runtime_activation_requests &
  RUNTIME_ACTIVATION_MONITOR_PID="$!"
}

stop_runtime_activation_monitor() {
  if [[ -n "${RUNTIME_ACTIVATION_MONITOR_PID}" ]] && is_pid_alive "${RUNTIME_ACTIVATION_MONITOR_PID}"; then
    kill -TERM "${RUNTIME_ACTIVATION_MONITOR_PID}" >/dev/null 2>&1 || true
    wait "${RUNTIME_ACTIVATION_MONITOR_PID}" >/dev/null 2>&1 || true
  fi
  RUNTIME_ACTIVATION_MONITOR_PID=""
}

stop_runtime_monitors() {
  local monitor_pid=""
  for monitor_pid in "${NAV_STACK_MONITOR_PID}" "${GEOMETRY_MONITOR_PID}"; do
    if [[ -n "${monitor_pid}" ]] && is_pid_alive "${monitor_pid}"; then
      kill -TERM "${monitor_pid}" >/dev/null 2>&1 || true
      wait "${monitor_pid}" >/dev/null 2>&1 || true
    fi
  done
  NAV_STACK_MONITOR_PID=""
  GEOMETRY_MONITOR_PID=""
}

stop_go2_video_streamer() {
  if go2_video_streamer_managed_by_systemd; then
    VIDEO_STREAMER_PID=""
    return 0
  fi
  if [[ -n "${VIDEO_STREAMER_PID}" ]] && is_pid_alive "${VIDEO_STREAMER_PID}"; then
    terminate_process_scope "${VIDEO_STREAMER_PID}" "go2 video streamer"
    wait "${VIDEO_STREAMER_PID}" >/dev/null 2>&1 || true
  fi
  VIDEO_STREAMER_PID=""
  # 兜底：清理任何未被本脚本跟踪的残留 streamer（如 systemd/手动启动的），
  # 确保网关停止后不会留下推流进程。
  cleanup_stale_go2_video_streamer
}

stop_gateway() {
  STOP_REQUESTED=1
  # 顺序：先杀网关 worker → person-follow → geometry runtime → nav-stack → video streamer。
  # 关键是先切断网关（geometry / perception 客户端），否则 geometry runtime 一停，
  # 客户端在重启窗口期 GetFrameTree 立即抛 UNAVAILABLE，日志刷一长串
  # "failed to connect to all addresses; last error: UNKNOWN: ipv4:127.0.0.1:50051"。
  if [[ -n "${GATEWAY_PID}" ]] && kill -0 "${GATEWAY_PID}" >/dev/null 2>&1; then
    terminate_process_scope "${GATEWAY_PID}" "gateway worker"
    wait "${GATEWAY_PID}" >/dev/null 2>&1 || true
  fi
  stop_runtime_activation_monitor
  stop_person_follow_monitor
  stop_runtime_monitors
  stop_person_follow
  stop_geometry_runtime
  stop_nav_stack
  stop_go2_video_streamer
  cleanup_external_geometry_processes
  exit 0
}


if [[ -f "${ENV_FILE}" ]]; then
  load_env_file "${ENV_FILE}"
fi
GO2_DATA_PLANE_ENV_FILE="${GO2_DATA_PLANE_ENV_FILE:-${REPO_ROOT}/drivers/go2/data_plane.env}"
if [[ -f "${GO2_DATA_PLANE_ENV_FILE}" ]]; then
  load_env_file "${GO2_DATA_PLANE_ENV_FILE}"
fi

PYTHON_BIN="${NUWAX_PYTHON_BIN:-${PYTHON_BIN:-python3}}"
PID_FILE="${NUWAX_GATEWAY_PID_FILE:-${REPO_ROOT}/runtime_data/gateway/gateway.pid}"
LOCK_FILE="${NUWAX_GATEWAY_LOCK_FILE:-$(default_gateway_lock_file)}"
HOST="${NUWAX_GATEWAY_HOST:-0.0.0.0}"
PORT="${NUWAX_GATEWAY_PORT:-8766}"
acquire_script_lock "${LOCK_FILE}"

case "${ACTION}" in
  start)
    if [[ -n "$(read_pid_file || true)" ]] && is_pid_alive "$(read_pid_file || true)"; then
      echo "gateway is already running pid=$(read_pid_file). Use: $0 restart" >&2
      exit 1
    fi
    rm -f "${PID_FILE}"
    cleanup_stale_gateway_processes
    cleanup_stale_person_follow_processes
    cleanup_stale_geometry_runtime_processes
    cleanup_stale_go2_video_streamer
    cleanup_external_geometry_processes
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
  source_runtime_script "${REPO_ROOT}/services/geometry/source_runtime_env.sh"
fi

export PYTHONUNBUFFERED=1
export NUWAX_PERSON_FOLLOW_TARGET_CHANNEL="${PERSON_FOLLOW_TARGET_CHANNEL}"
export NUWAX_PERSON_FOLLOW_POINTCLOUD_CHANNEL="${PERSON_FOLLOW_POINTCLOUD_CHANNEL}"
export NUWAX_PERSON_FOLLOW_TF_CHANNEL="${PERSON_FOLLOW_TF_CHANNEL}"
export NUWAX_PERSON_FOLLOW_COMMAND_CHANNEL="${PERSON_FOLLOW_COMMAND_CHANNEL}"
export NUWAX_GEOMETRY_RUNTIME_NAV_GOAL_CHANNEL="/nav/goal#nuwax_msgs.GoalEnvelope"
export NUWAX_LCM_CHANNEL_GOAL_POSE="/nav/goal#nuwax_msgs.GoalEnvelope"
export NUWAX_GEOMETRY_RUNTIME_NAV_PATH_CHANNEL="/nav/path#nuwax_msgs.PathEnvelope"
export NUWAX_LCM_CHANNEL_NAV_PATH="/nav/path#nuwax_msgs.PathEnvelope"
export GO2_VIDEO_STREAM_GATEWAY_URL="${GO2_VIDEO_STREAM_GATEWAY_URL:-ws://127.0.0.1:${PORT}/streams/video.ingest}"
export NUWAX_RUNTIME_ACTIVATION_REQUEST="${NUWAX_RUNTIME_ACTIVATION_REQUEST:-${REPO_ROOT}/runtime_data/gateway/geometry.request}"
export NUWAX_RUNTIME_ACTIVATION_RESULT="${NUWAX_RUNTIME_ACTIVATION_RESULT:-${REPO_ROOT}/runtime_data/gateway/geometry.result.json}"
mkdir -p "$(dirname "${NUWAX_RUNTIME_ACTIVATION_REQUEST}")"
# 个人机器狗：网关不鉴权，视频推流令牌默认留空（如需可单独设置 GO2_VIDEO_STREAM_ACCESS_TOKEN）。
export GO2_VIDEO_STREAM_ACCESS_TOKEN="${GO2_VIDEO_STREAM_ACCESS_TOKEN:-}"

cd "${REPO_ROOT}"
write_pid_file
trap remove_pid_file EXIT
trap stop_gateway INT TERM

if ! is_restart_enabled "${RESTART_POLICY}"; then
  set +e
  (run_with_title "nuwax-gateway-web" "${PYTHON_BIN}" -m uvicorn gateways.app:create_gateway_app_from_env --factory --host "${HOST}" --port "${PORT}") &
  GATEWAY_PID="$!"
  start_go2_video_streamer
  start_runtime_activation_monitor
  wait "${GATEWAY_PID}"
  exit_code="$?"
  STOP_REQUESTED=1
  stop_runtime_activation_monitor
  stop_runtime_monitors
  stop_person_follow
  stop_go2_video_streamer
  stop_geometry_runtime
  stop_nav_stack
  exit "${exit_code}"
fi

attempt=0
while true; do
  set +e
  (run_with_title "nuwax-gateway-web" "${PYTHON_BIN}" -m uvicorn gateways.app:create_gateway_app_from_env --factory --host "${HOST}" --port "${PORT}") &
  GATEWAY_PID="$!"
  start_go2_video_streamer
  start_runtime_activation_monitor
  wait "${GATEWAY_PID}"
  exit_code="$?"
  stop_go2_video_streamer
  stop_runtime_activation_monitor
  GATEWAY_PID=""
  set -e

  if ! should_restart "${exit_code}"; then
    STOP_REQUESTED=1
    stop_runtime_monitors
    stop_person_follow
    stop_geometry_runtime
    stop_nav_stack
    exit "${exit_code}"
  fi

  attempt=$((attempt + 1))
  if [[ "${RESTART_MAX_ATTEMPTS}" != "0" && "${attempt}" -ge "${RESTART_MAX_ATTEMPTS}" ]]; then
    echo "gateway exited with code ${exit_code}; restart attempts exhausted (${attempt})." >&2
    STOP_REQUESTED=1
    stop_runtime_monitors
    stop_person_follow
    stop_geometry_runtime
    stop_nav_stack
    exit "${exit_code}"
  fi

  echo "gateway exited with code ${exit_code}; restarting in ${RESTART_SEC}s (attempt ${attempt})." >&2
  sleep "${RESTART_SEC}"
done
