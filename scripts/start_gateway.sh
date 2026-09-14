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

# 与 Python 设置、Rust 启动入口共用解析口径。
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/lib/config_env.sh"


default_gateway_web_config_env_file() {
  if [[ -n "${STATE_DIRECTORY:-}" ]]; then
    printf '%s/%s' "${STATE_DIRECTORY%%:*}" pointcloud3d.env
    return
  fi
  printf '%s' "${REPO_ROOT}/.env"
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
  [[ -x "${binary}" && ! -L "${binary}" ]] \
    && is_truthy "${NUWAX_GEOMETRY_EXTERNAL_ENABLED:-true}"
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
  echo "starting nuwax-nav-stack（导航配置由建图执行链确定）"
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
    export NUWAX_GEOMETRY_RUNTIME_NAV_GOAL_CHANNEL="${NUWAX_GEOMETRY_RUNTIME_NAV_GOAL_CHANNEL:-${NUWAX_LCM_CHANNEL_GOAL_POSE:-/nav/goal#nuwax_msgs.GoalEnvelope}}"
    export NUWAX_LCM_CHANNEL_GOAL_POSE="${NUWAX_LCM_CHANNEL_GOAL_POSE:-${NUWAX_GEOMETRY_RUNTIME_NAV_GOAL_CHANNEL}}"
    export NUWAX_GEOMETRY_RUNTIME_NAV_PATH_CHANNEL="${NUWAX_GEOMETRY_RUNTIME_NAV_PATH_CHANNEL:-${NUWAX_LCM_CHANNEL_NAV_PATH:-/nav/path#nuwax_msgs.PathEnvelope}}"
    export NUWAX_LCM_CHANNEL_NAV_PATH="${NUWAX_LCM_CHANNEL_NAV_PATH:-${NUWAX_GEOMETRY_RUNTIME_NAV_PATH_CHANNEL}}"
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
  local error_code="${4:-}"
  local result_file="${NUWAX_RUNTIME_ACTIVATION_RESULT}"
  local temporary="${result_file}.$$"
  STATE="${state}" STAGE="${stage}" MESSAGE="${message}" \
    REQUEST_ID="${RUNTIME_ACTIVATION_REQUEST_ID:-}" \
    MOTION_STOP_GENERATION="${RUNTIME_ACTIVATION_GENERATION:-0}" ERROR_CODE="${error_code}" \
    ACTIVATION_LEVEL="${RUNTIME_ACTIVATION_LEVEL:-unknown}" \
    CAPABILITY_NAME="${RUNTIME_ACTIVATION_CAPABILITY:-unknown}" \
    "${PYTHON_BIN}" -c 'import json, os; payload = {"state": os.environ["STATE"], "stage": os.environ["STAGE"], "message": os.environ["MESSAGE"], "request_id": os.environ["REQUEST_ID"], "motion_stop_generation": int(os.environ["MOTION_STOP_GENERATION"]), "activation_level": os.environ["ACTIVATION_LEVEL"], "capability_name": os.environ["CAPABILITY_NAME"]}; payload.update({"error_code": os.environ["ERROR_CODE"]} if os.environ["ERROR_CODE"] else {}); print(json.dumps(payload, ensure_ascii=True, sort_keys=True))' \
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
request_id = str(payload.get("request_id") or "-").strip()
raw_generation = payload.get("motion_stop_generation")
generation = 0 if raw_generation is None else int(raw_generation)
if generation < 0 or any(char.isspace() for char in request_id + capability):
    raise SystemExit("invalid activation request identity")
if request_id != "-" and raw_generation is None and level != "geometry":
    raise SystemExit("missing motion_stop_generation")
print("%s\t%s\t%s\t%s" % (level, capability, request_id, generation))
PY
  )" || return 1
  IFS=$'\t' read -r RUNTIME_ACTIVATION_LEVEL RUNTIME_ACTIVATION_CAPABILITY RUNTIME_ACTIVATION_REQUEST_ID RUNTIME_ACTIVATION_GENERATION <<< "${parsed}"
  [[ -n "${RUNTIME_ACTIVATION_LEVEL}" && -n "${RUNTIME_ACTIVATION_CAPABILITY}" ]]
}

runtime_activation_protocol_state() {
  local phase="${1:-starting}"
  "${PYTHON_BIN}" - "${NUWAX_RUNTIME_ACTIVATION_REQUEST}" \
    "${RUNTIME_ACTIVATION_REQUEST_ID}" "${RUNTIME_ACTIVATION_GENERATION}" "${phase}" <<'PY'
import fcntl
import json
import os
from pathlib import Path
import sys

request, request_id, generation, phase = sys.argv[1:]
generation = int(generation)
try:
    descriptor = os.open(request + ".lock", os.O_RDONLY | os.O_CREAT | os.O_EXCL, 0o644)
except FileExistsError:
    descriptor = os.open(request + ".lock", os.O_RDONLY)
else:
    os.fchmod(descriptor, 0o644)
protocol_lock = os.fdopen(descriptor, "r")
fcntl.flock(protocol_lock, fcntl.LOCK_SH)
def read(path):
    try:
        value = json.loads(Path(path).read_text(encoding="utf-8"))
        return value if isinstance(value, dict) else {}
    except (OSError, ValueError):
        return {}
def matches(value, cancel=False):
    try:
        marker_generation = int(value.get("motion_stop_generation") or 0)
    except (ValueError, TypeError):
        return False
    return value.get("request_id") == request_id and (
        marker_generation >= generation if cancel else marker_generation == generation
    )
cancel = read(os.environ.get("NUWAX_RUNTIME_ACTIVATION_CANCEL", request + ".cancel.json"))
ack = read(os.environ.get("NUWAX_RUNTIME_ACTIVATION_ACK", request + ".ack.json"))
current = read(request)
if matches(cancel, cancel=True):
    print("cancelled")
elif not Path(request).exists():
    # 提交由网关移除本请求表示；其它请求身份不能冒充提交确认。
    print("committed" if phase == "ready" else "cancelled")
elif current.get("request_id") != request_id:
    print("cancelled")
elif phase == "prepared" and matches(ack) and ack.get("state") == "acknowledged":
    print("acknowledged")
else:
    print("pending")
PY
}

cleanup_runtime_activation_protocol() {
  # 与网关写入共用短文件锁，在同一临界区检查身份并删除。
  "${PYTHON_BIN}" - "${NUWAX_RUNTIME_ACTIVATION_REQUEST}" "${RUNTIME_ACTIVATION_REQUEST_ID}" <<'PY'
import fcntl
import json
import os
from pathlib import Path
import sys
request, request_id = sys.argv[1:]
try:
    descriptor = os.open(request + ".lock", os.O_RDONLY | os.O_CREAT | os.O_EXCL, 0o644)
except FileExistsError:
    descriptor = os.open(request + ".lock", os.O_RDONLY)
else:
    os.fchmod(descriptor, 0o644)
protocol_lock = os.fdopen(descriptor, "r")
fcntl.flock(protocol_lock, fcntl.LOCK_EX)
for value in (request, os.environ.get("NUWAX_RUNTIME_ACTIVATION_CANCEL", request + ".cancel.json"),
              os.environ.get("NUWAX_RUNTIME_ACTIVATION_ACK", request + ".ack.json")):
    path = Path(value)
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
        if str(payload.get("request_id") or "-") == (request_id or "-"):
            path.unlink()
    except (OSError, ValueError, AttributeError):
        pass
PY
}

rollback_runtime_activation() {
  local index component
  for ((index = ${#RUNTIME_ACTIVATION_STARTED[@]} - 1; index >= 0; index--)); do
    component="${RUNTIME_ACTIVATION_STARTED[index]}"
    "stop_${component}"
  done
  RUNTIME_ACTIVATION_STARTED=()
}

fail_runtime_activation() {
  local state="$1" stage="$2" message="$3" error_code="$4"
  rollback_runtime_activation
  write_runtime_activation_result "${state}" "${stage}" "${message}" "${error_code}"
  cleanup_runtime_activation_protocol
  RUNTIME_ACTIVATION_REQUEST_ID=""
  return 1
}

check_runtime_activation_cancelled() {
  if [[ "$(runtime_activation_protocol_state)" == "cancelled" ]]; then
    fail_runtime_activation cancelled "${RUNTIME_ACTIVATION_STAGE}" "本次运行组件激活已取消。" activation_cancelled
    return 1
  fi
}

start_runtime_activation_component() {
  local component="$1" pid_variable="$2" manager="$3"
  local previous_pid="${!pid_variable}" result=0
  RUNTIME_ACTIVATION_STAGE="${component}"
  check_runtime_activation_cancelled || return 1
  write_runtime_activation_result activating "${component}" "正在启动所需运行组件。"
  "start_${component}" || result=$?
  if ! "${manager}" && [[ -n "${!pid_variable}" && "${!pid_variable}" != "${previous_pid}" ]]; then
    RUNTIME_ACTIVATION_STARTED+=("${component}")
  fi
  check_runtime_activation_cancelled || return 1
  if [[ "${result}" -ne 0 ]] || { ! "${manager}" && [[ -z "${!pid_variable}" ]]; }; then
    fail_runtime_activation failed "${component}" "所需运行组件未启动或已被禁用。" runtime_activation_failed
    return 1
  fi
}

wait_runtime_activation_handshake() {
  local phase="$1" expected="$2" timeout_sec="$3" attempts attempt state
  attempts="$("${PYTHON_BIN}" - "${timeout_sec}" <<'PY'
import math
import sys
try:
    value = float(sys.argv[1])
    value = max(0.1, value) if math.isfinite(value) else 10.0
except ValueError:
    value = 10.0
print(max(1, math.ceil(value * 10)))
PY
)"
  for ((attempt = 0; attempt < attempts; attempt++)); do
    state="$(runtime_activation_protocol_state "${phase}")"
    if [[ "${state}" == "${expected}" ]]; then
      return 0
    fi
    if [[ "${state}" == "cancelled" ]]; then
      fail_runtime_activation cancelled "${phase}" "网关已取消本次运行组件激活。" activation_cancelled
      return 1
    fi
    sleep 0.1
  done
  local error_code="runtime_activation_ack_timeout"
  [[ "${phase}" != "ready" ]] || error_code="runtime_activation_commit_timeout"
  fail_runtime_activation failed "${phase}" "等待网关确认运行组件激活超时。" "${error_code}"
}

activate_requested_runtime_components() {
    RUNTIME_ACTIVATION_STAGE="request"
    RUNTIME_ACTIVATION_STARTED=()
    check_runtime_activation_cancelled || return 1
    if [[ "${RUNTIME_ACTIVATION_LEVEL}" == "guard" \
      || "${RUNTIME_ACTIVATION_LEVEL}" == "motion" \
      || "${RUNTIME_ACTIVATION_LEVEL}" == "person_follow" ]]; then
      write_runtime_activation_result activating motion_guard "waiting for motion guard component"
      if ! wait_for_motion_guard_transport; then
        fail_runtime_activation failed motion_guard "运动守卫传输不可用。" runtime_activation_failed
        return 1
      fi
      check_runtime_activation_cancelled || return 1
    fi
    if [[ "${RUNTIME_ACTIVATION_LEVEL}" == "motion" \
      || "${RUNTIME_ACTIVATION_LEVEL}" == "person_follow" ]]; then
      start_runtime_activation_component nav_stack NAV_STACK_PID nav_stack_managed_by_systemd || return 1
    fi
    if [[ "${RUNTIME_ACTIVATION_LEVEL}" == "geometry" \
      || "${RUNTIME_ACTIVATION_LEVEL}" == "motion" \
      || "${RUNTIME_ACTIVATION_LEVEL}" == "person_follow" ]]; then
      start_runtime_activation_component geometry_runtime GEOMETRY_RUNTIME_PID geometry_runtime_managed_by_systemd || return 1
    fi
    if [[ "${RUNTIME_ACTIVATION_LEVEL}" == "person_follow" ]]; then
      start_runtime_activation_component person_follow PERSON_FOLLOW_PID person_follow_managed_by_systemd || return 1
      if person_follow_managed_by_systemd && ! wait_for_person_follow_ready; then
        fail_runtime_activation failed person_follow "人物跟随组件未就绪。" runtime_activation_failed
        return 1
      fi
    fi
    check_runtime_activation_cancelled || return 1
    write_runtime_activation_result prepared prepared "运行组件已准备，等待网关确认。"
    wait_runtime_activation_handshake prepared acknowledged "${NUWAX_RUNTIME_ACTIVATION_ACK_TIMEOUT_SEC:-10}" || return 1
    write_runtime_activation_result ready ready "requested runtime components are ready"
    wait_runtime_activation_handshake ready committed "${NUWAX_RUNTIME_ACTIVATION_COMMIT_TIMEOUT_SEC:-10}" || return 1
    RUNTIME_ACTIVATION_STARTED=()
    if [[ "${RUNTIME_ACTIVATION_LEVEL}" == "person_follow" ]]; then
      start_person_follow_monitor
    fi
    cleanup_runtime_activation_protocol
    write_runtime_activation_result committed committed "网关提交已确认，本次激活事务已收尾。"
    RUNTIME_ACTIVATION_REQUEST_ID=""
}

cleanup_runtime_activation_owner() {
  trap - INT TERM HUP
  if [[ -n "${RUNTIME_ACTIVATION_REQUEST_ID:-}" ]]; then
    fail_runtime_activation cancelled "${RUNTIME_ACTIVATION_STAGE:-starting}" "本机启动拥有方正在关闭。" activation_cancelled || true
  fi
  # 子 shell 创建的进程只由该拥有方回收；父启动器原有的视频/几何仍归父方。
  stop_person_follow_monitor
  [[ "${PERSON_FOLLOW_PID}" == "${initial_follow_pid}" ]] || stop_person_follow
  [[ "${GEOMETRY_RUNTIME_PID}" == "${initial_geometry_pid}" ]] || stop_geometry_runtime
  [[ "${NAV_STACK_PID}" == "${initial_nav_pid}" ]] || stop_nav_stack
  exit 0
}

monitor_runtime_activation_requests() {
  local request_file="${NUWAX_RUNTIME_ACTIVATION_REQUEST}"
  local initial_geometry_pid="${GEOMETRY_RUNTIME_PID}" initial_nav_pid="${NAV_STACK_PID}" initial_follow_pid="${PERSON_FOLLOW_PID}"
  local RUNTIME_ACTIVATION_REQUEST_ID="" RUNTIME_ACTIVATION_GENERATION=0 RUNTIME_ACTIVATION_STAGE="request"
  local -a RUNTIME_ACTIVATION_STARTED=()
  local next_health_check=$((SECONDS + 5))
  trap cleanup_runtime_activation_owner INT TERM HUP
  while [[ "${STOP_REQUESTED}" -eq 0 ]]; do
    if [[ -f "${request_file}" ]]; then
      RUNTIME_ACTIVATION_REQUEST_ID=""
      if read_runtime_activation_request "${request_file}" && [[ "${RUNTIME_ACTIVATION_REQUEST_ID}" != "-" ]]; then
        activate_requested_runtime_components || true
      else
        write_runtime_activation_result failed request "激活请求缺少合法的请求身份。" runtime_activation_request_invalid
        cleanup_runtime_activation_protocol
        RUNTIME_ACTIVATION_REQUEST_ID=""
      fi
    fi
    # 监督与启动留在同一拥有方，重启后的 PID 不再遗失在另一个后台 shell 中。
    if ((SECONDS >= next_health_check)); then
      next_health_check=$((SECONDS + 5))
      if [[ -n "${NAV_STACK_PID}" && "${NAV_STACK_PID}" != "${initial_nav_pid}" ]] && ! is_pid_alive "${NAV_STACK_PID}"; then
        wait "${NAV_STACK_PID}" >/dev/null 2>&1 || true
        NAV_STACK_PID=""
        if ! start_nav_stack; then
          send_signal_to_process_scope TERM "${GATEWAY_PID}"
          break
        fi
      fi
      if [[ -n "${GEOMETRY_RUNTIME_PID}" && "${GEOMETRY_RUNTIME_PID}" != "${initial_geometry_pid}" ]] && ! is_pid_alive "${GEOMETRY_RUNTIME_PID}"; then
        wait "${GEOMETRY_RUNTIME_PID}" >/dev/null 2>&1 || true
        GEOMETRY_RUNTIME_PID=""
        if ! start_geometry_runtime; then
          send_signal_to_process_scope TERM "${GATEWAY_PID}"
          break
        fi
      fi
    fi
    sleep 0.1
  done
  cleanup_runtime_activation_owner
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


GO2_DATA_PLANE_ENV_FILE="${GO2_DATA_PLANE_ENV_FILE:-${REPO_ROOT}/drivers/go2/data_plane.env}"
load_env_file "${GO2_DATA_PLANE_ENV_FILE}"
load_env_file "${REPO_ROOT}/.config"
load_env_file "${ENV_FILE}"
GATEWAY_WEB_CONFIG_ENV_FILE="${NUWAX_GATEWAY_WEB_CONFIG_ENV_FILE:-$(default_gateway_web_config_env_file)}"
export NUWAX_GATEWAY_WEB_CONFIG_ENV_FILE="${GATEWAY_WEB_CONFIG_ENV_FILE}"
if [[ -f "${GATEWAY_WEB_CONFIG_ENV_FILE}" ]]; then
  # Web 覆盖文件只能影响无安全敏感性的点云显示项。
  load_env_file "${GATEWAY_WEB_CONFIG_ENV_FILE}" "NUWAX_GATEWAY_POINTCLOUD3D_"
fi

if [[ -n "${NUWAX_USER_CONFIG_ENV_FILE:-}" ]]; then
  load_env_file "${NUWAX_USER_CONFIG_ENV_FILE}"
fi
RESTART_POLICY="${NUWAX_GATEWAY_RESTART:-on-failure}"
RESTART_SEC="${NUWAX_GATEWAY_RESTART_SEC:-3}"
RESTART_MAX_ATTEMPTS="${NUWAX_GATEWAY_RESTART_MAX_ATTEMPTS:-0}"
PERSON_FOLLOW_TARGET_CHANNEL="${NUWAX_PERSON_FOLLOW_TARGET_CHANNEL:-${PERSON_FOLLOW_TARGET_CHANNEL}}"
PERSON_FOLLOW_POINTCLOUD_CHANNEL="${NUWAX_PERSON_FOLLOW_POINTCLOUD_CHANNEL:-${PERSON_FOLLOW_POINTCLOUD_CHANNEL}}"
PERSON_FOLLOW_TF_CHANNEL="${NUWAX_PERSON_FOLLOW_TF_CHANNEL:-${PERSON_FOLLOW_TF_CHANNEL}}"
PERSON_FOLLOW_COMMAND_CHANNEL="${NUWAX_PERSON_FOLLOW_COMMAND_CHANNEL:-${PERSON_FOLLOW_COMMAND_CHANNEL}}"
PYTHON_BIN="${NUWAX_PYTHON_BIN:-${PYTHON_BIN:-python3}}"
PID_FILE="${NUWAX_GATEWAY_PID_FILE:-${REPO_ROOT}/runtime_data/gateway/gateway.pid}"
LOCK_FILE="${NUWAX_GATEWAY_LOCK_FILE:-$(default_gateway_lock_file)}"
normalize_config_aliases
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
export NUWAX_GEOMETRY_RUNTIME_NAV_GOAL_CHANNEL="${NUWAX_GEOMETRY_RUNTIME_NAV_GOAL_CHANNEL:-${NUWAX_LCM_CHANNEL_GOAL_POSE:-/nav/goal#nuwax_msgs.GoalEnvelope}}"
export NUWAX_LCM_CHANNEL_GOAL_POSE="${NUWAX_LCM_CHANNEL_GOAL_POSE:-${NUWAX_GEOMETRY_RUNTIME_NAV_GOAL_CHANNEL}}"
export NUWAX_GEOMETRY_RUNTIME_NAV_PATH_CHANNEL="${NUWAX_GEOMETRY_RUNTIME_NAV_PATH_CHANNEL:-${NUWAX_LCM_CHANNEL_NAV_PATH:-/nav/path#nuwax_msgs.PathEnvelope}}"
export NUWAX_LCM_CHANNEL_NAV_PATH="${NUWAX_LCM_CHANNEL_NAV_PATH:-${NUWAX_GEOMETRY_RUNTIME_NAV_PATH_CHANNEL}}"
export NUWAX_RUNTIME_ACTIVATION_REQUEST="${NUWAX_RUNTIME_ACTIVATION_REQUEST:-${REPO_ROOT}/runtime_data/gateway/geometry.request}"
export NUWAX_RUNTIME_ACTIVATION_RESULT="${NUWAX_RUNTIME_ACTIVATION_RESULT:-${REPO_ROOT}/runtime_data/gateway/geometry.result.json}"
mkdir -p "$(dirname "${NUWAX_RUNTIME_ACTIVATION_REQUEST}")"

cd "${REPO_ROOT}"
write_pid_file
trap remove_pid_file EXIT
trap stop_gateway INT TERM

# 先有界排空HTTP长连接，再进入应用生命周期释放运行时资源；外层关闭预算保持不变。
if ! is_restart_enabled "${RESTART_POLICY}"; then
  set +e
  # 实时流不在网页事件循环内同步压缩，避免地图发送阻塞视频与位姿。
  (run_with_title "nuwax-gateway-web" "${PYTHON_BIN}" -m uvicorn gateways.app:create_gateway_app_from_env --factory --host "${HOST}" --port "${PORT}" --ws-per-message-deflate false --timeout-graceful-shutdown 5) &
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
  (run_with_title "nuwax-gateway-web" "${PYTHON_BIN}" -m uvicorn gateways.app:create_gateway_app_from_env --factory --host "${HOST}" --port "${PORT}" --ws-per-message-deflate false --timeout-graceful-shutdown 5) &
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
