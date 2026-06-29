#!/usr/bin/env bash
set -euo pipefail

if [[ "${NUWAX_GATEWAY_SUPERVISOR_TITLED:-0}" != "1" && -n "${BASH:-}" ]]; then
  export NUWAX_GATEWAY_SUPERVISOR_TITLED=1
  export NUWAX_GATEWAY_SUPERVISOR_DEPTH=$((${NUWAX_GATEWAY_SUPERVISOR_DEPTH:-0}+1))
  if [[ "${NUWAX_GATEWAY_SUPERVISOR_DEPTH}" -gt "2" ]]; then
    echo "[v8 FATAL] start_gateway.sh self-exec depth=${NUWAX_GATEWAY_SUPERVISOR_DEPTH} > 2; abort to prevent supervisor fork storm" >&2
    exit 1
  fi
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
ROS_BRIDGE_PID=""
VIDEO_RELAY_PID=""
STOP_REQUESTED=0

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

wait_for_port_listen() {
  local port="$1"
  local timeout_sec="$2"
  local waited=0
  if ! command -v ss >/dev/null 2>&1; then
    return 0
  fi
  while [[ "${waited}" -lt "${timeout_sec}" ]]; do
    if ss -ltn 2>/dev/null | awk '{print $4}' | grep -qE "(^|:)${port}$"; then
      return 0
    fi
    sleep 1
    waited=$((waited + 1))
  done
  return 1
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

cleanup_stale_ros_bridge_processes() {
  local binary="${REPO_ROOT}/services/geometry/nuwax_ros_bridge/target/release/nuwax-ros-bridge"
  local pid=""
  local args=""

  while read -r pid args; do
    if [[ -z "${pid}" || "${pid}" == "$$" || "${pid}" == "${BASHPID}" ]]; then
      continue
    fi
    if [[ "${args}" == *"nuwax-ros-bridge"* || "${args}" == *"nuwax_video_relay.py"* ]]; then
      terminate_process_scope "${pid}" "stale video/ros bridge"
    fi
  done < <(ps -eo pid=,args=)

  # 兜底清理：nuwax-ros-bridge 可能已被 pgrep 杀过但 listen socket 仍处 TIME_WAIT，
  # 或被 SIGKILL 后 socket 未立即释放。下次 spawn 会因 port-in-use 失败，
  # 而 Python 端 RosBridgeSubprocess 端口探测会误报"端口已开"导致 attach 到僵尸。
  # 这里先看端口状态：若 50051 仍 LISTEN 且 pgrep 已无匹配 → 主动 fuser 杀端口持有者。
  # 顺序约束：必须等 gateway worker 已停（本函数仅在 cleanup_stale_gateway_processes 之后
  # 调用），否则会误杀在跑 bridge 的 listen socket。当前所有调用点均已满足。
  local bridge_port="${NUWAX_ROS_BRIDGE_PORT:-50051}"
  if command -v ss >/dev/null 2>&1; then
    if ss -ltn 2>/dev/null | awk '{print $4}' | grep -qE "(^|:)${bridge_port}$"; then
      if ! pgrep -f "nuwax-ros-bridge" >/dev/null 2>&1; then
        echo "==> port ${bridge_port} still LISTEN but no nuwax-ros-bridge process; clearing residual socket"
        if command -v fuser >/dev/null 2>&1; then
          fuser -KILL "${bridge_port}/tcp" >/dev/null 2>&1 || true
        else
          # 退化路径：用 lsof 找持有者再 kill；缺失时记 warning 不阻塞。
          if command -v lsof >/dev/null 2>&1; then
            local residual_pid
            residual_pid="$(lsof -ti tcp:${bridge_port} 2>/dev/null || true)"
            if [[ -n "${residual_pid}" ]]; then
              kill -KILL ${residual_pid} >/dev/null 2>&1 || true
            fi
          else
            echo "WARNING: fuser/lsof 都不可用，端口 ${bridge_port} 残留可能让 spawn 失败" >&2
          fi
        fi
        sleep 0.5
      fi
    fi
  fi

  rm -f /tmp/nuwax_video.sock /tmp/nuwax_video.shm /tmp/nuwax_pointcloud.shm 2>/dev/null || true
}

cleanup_stale_video_streamer() {
  # 终止任何残留的端侧视频推流进程（无论由本脚本、systemd 还是手动启动），
  # 保证网关成为唯一管理者，避免"网关停了 streamer 还在"以及双拉起。
  local pid=""
  local args=""

  while read -r pid args; do
    if [[ -z "${pid}" || "${pid}" == "$$" || "${pid}" == "${BASHPID}" ]]; then
      continue
    fi
    if [[ "${args}" == *"drivers.go2.video_streamer"* || "${args}" == *"drivers.g1.video_streamer"* ]]; then
      terminate_process_scope "${pid}" "stale video streamer"
    fi
  done < <(ps -eo pid=,args=)
}

cleanup_external_geometry_processes() {
  local geometry_root="${REPO_ROOT}/services/geometry/ros2_ws/src/GO2_RTABNAV2"
  local pid=""
  local args=""
  local pgid=""
  local pattern=""
  local go2_patterns=(
    "ros2 launch go2_slam_nav"
    "lib/rtabmap_slam/rtabmap"
    "lib/rtabmap_odom/icp_odometry"
    "lib/go2_slam_nav/sensor_stamp_relay"
    "lib/go2_slam_nav/obstacle_grid_projector"
    "lib/go2_slam_nav/go2_tf_relay"
    "lib/go2_cmd_processor/sport_ctrl"
    "explore_node"
    "explore_preroll"
    "nav2_container"
    "component_container_isolated"
    "lifecycle_manager"
    "/nav2_controller/controller_server"
    "/nav2_planner/planner_server"
    "/nav2_recoveries/recoveries_server"
    "/nav2_bt_navigator/bt_navigator"
  )
  declare -A seen_pgids=()

  if ! is_truthy "${NUWAX_GATEWAY_CLEANUP_EXTERNAL_GEOMETRY:-true}"; then
    return 0
  fi

  while read -r pid args; do
    if [[ -z "${pid}" || "${pid}" == "$$" || "${pid}" == "${BASHPID}" ]]; then
      continue
    fi
    if [[ "${args}" != *"${geometry_root}"* ]]; then
      continue
    fi
    pgid="$(pid_pgid "${pid}")"
    if [[ -z "${pgid}" || -n "${seen_pgids[${pgid}]:-}" ]]; then
      continue
    fi
    seen_pgids["${pgid}"]=1
    terminate_process_scope "${pid}" "external geometry process group ${pgid}"
  done < <(ps -eo pid=,args=)

  for pattern in "${go2_patterns[@]}"; do
    pkill -TERM -f "${pattern}" 2>/dev/null || true
  done
  sleep 1
  for pattern in "${go2_patterns[@]}"; do
    pkill -KILL -f "${pattern}" 2>/dev/null || true
  done
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
    cleanup_stale_ros_bridge_processes
    cleanup_stale_video_streamer
    cleanup_external_geometry_processes
    return 0
  fi

  if ! is_pid_alive "${pid}"; then
    echo "gateway pid ${pid} is not running; removing stale pid file."
    rm -f "${PID_FILE}"
    cleanup_stale_gateway_processes
    cleanup_stale_ros_bridge_processes
    cleanup_stale_video_streamer
    cleanup_external_geometry_processes
    return 0
  fi

  terminate_process_scope "${pid}" "gateway supervisor"
  rm -f "${PID_FILE}"
  # 顺序：先杀网关 worker（切断 geometry/perception 客户端调用）→
  # 再停 ros-bridge（避免重启窗口期 GetFrameTree 抛 UNAVAILABLE 刷日志）→
  # 最后清理外部几何进程。
  cleanup_stale_gateway_processes
  cleanup_stale_ros_bridge_processes
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

should_start_ros2_bridge() {
  local binary="${REPO_ROOT}/services/geometry/nuwax_ros_bridge/target/release/nuwax-ros-bridge"
  is_truthy "${NUWAX_ROS2_BRIDGE_ENABLED:-false}" && [[ -x "${binary}" ]]
}

should_start_video_relay() {
  should_start_go2_video_streamer || should_start_g1_video_streamer
}

should_start_ros_bridge() {
  should_start_ros2_bridge || should_start_video_relay
}

active_robot_name() {
  printf '%s' "${NUWAX_ACTIVE_ROBOT:-}" | tr '[:upper:]' '[:lower:]'
}

should_start_go2_video_streamer() {
  is_truthy "${GO2_VIDEO_STREAM_ENABLED:-true}" \
    && is_truthy "${GO2_DATA_PLANE_ENABLED:-false}" \
    && [[ "$(active_robot_name)" == "go2" ]]
}

should_start_g1_video_streamer() {
  is_truthy "${G1_VIDEO_STREAM_ENABLED:-true}" \
    && is_truthy "${G1_DATA_PLANE_ENABLED:-false}" \
    && [[ "$(active_robot_name)" == "g1" ]]
}

geometry_cyclonedds_uri() {
  local candidate="${NUWAX_GEOMETRY_CYCLONEDDS_URI:-}"
  if [[ -z "${candidate}" && -f "${HOME}/cyclonedds_ws/cyclonedds.xml" ]]; then
    candidate="${HOME}/cyclonedds_ws/cyclonedds.xml"
  fi
  if [[ -z "${candidate}" && -f "${HOME}/unitree_ros2/cyclonedds_ws/cyclonedds.xml" ]]; then
    candidate="${HOME}/unitree_ros2/cyclonedds_ws/cyclonedds.xml"
  fi
  printf '%s' "${candidate}"
}

wait_for_video_uds_socket() {
  local uds_path="${NUWAX_VIDEO_UDS_PATH:-/tmp/nuwax_video.sock}"
  local timeout_sec="${NUWAX_VIDEO_UDS_READY_TIMEOUT_SEC:-30}"
  local waited=0
  while [[ "${waited}" -lt "${timeout_sec}" ]]; do
    if [[ -S "${uds_path}" ]]; then
      echo "video UDS ready (${uds_path})"
      return 0
    fi
    sleep 1
    waited=$((waited + 1))
  done
  echo "WARNING: video UDS not ready after ${timeout_sec}s (${uds_path})" >&2
  return 1
}

prepare_robot_video_backend() {
  if ! should_start_video_streamer; then
    return 0
  fi
  local robot_name
  robot_name="$(active_robot_name)"
  echo "preparing SDK video backend for robot=${robot_name}..."
  if ! "${PYTHON_BIN}" "${REPO_ROOT}/scripts/ensure_robot_video_backend.py" --robot "${robot_name}"; then
    echo "WARNING: SDK 视频后端尚未就绪，video streamer 将继续重试。" >&2
  fi
}

wait_for_video_shm_frame() {
  local shm_path="${NUWAX_VIDEO_SHM_PATH:-/tmp/nuwax_video.shm}"
  local timeout_sec="${NUWAX_VIDEO_SHM_READY_TIMEOUT_SEC:-${G1_VIDEO_SHM_READY_TIMEOUT_SEC:-90}}"
  local waited=0
  while [[ "${waited}" -lt "${timeout_sec}" ]]; do
    if [[ -f "${shm_path}" ]] && "${PYTHON_BIN}" -c "
from services.geometry.video_shm_reader import VideoShmReader
reader = VideoShmReader('${shm_path}')
try:
    raise SystemExit(0 if reader.has_new_frame() else 1)
finally:
    reader.close()
" 2>/dev/null; then
      echo "video SHM ready (${shm_path})"
      return 0
    fi
    sleep 2
    waited=$((waited + 2))
  done
  echo "WARNING: video SHM has no frames after ${timeout_sec}s (debug 面板将等待首帧)" >&2
}

should_start_video_streamer() {
  should_start_video_relay
}

start_python_video_relay() {
  if [[ -n "${VIDEO_RELAY_PID}" ]] && is_pid_alive "${VIDEO_RELAY_PID}"; then
    return 0
  fi
  if ! should_start_video_relay; then
    return 0
  fi
  echo "starting Python video relay (UDS -> SHM)"
  (
    run_with_title "nuwax-video-relay" "${PYTHON_BIN}" "${REPO_ROOT}/scripts/nuwax_video_relay.py"
  ) &
  VIDEO_RELAY_PID="$!"
  wait_for_video_uds_socket || echo "WARNING: Python video relay UDS not ready (pid ${VIDEO_RELAY_PID})" >&2
}

start_video_relay() {
  if should_start_ros2_bridge; then
    start_ros_bridge
    return 0
  fi
  start_python_video_relay
}

start_ros_bridge() {
  if [[ -n "${ROS_BRIDGE_PID}" ]] && is_pid_alive "${ROS_BRIDGE_PID}"; then
    return 0
  fi
  if ! should_start_ros2_bridge; then
    return 0
  fi
  local binary="${REPO_ROOT}/services/geometry/nuwax_ros_bridge/target/release/nuwax-ros-bridge"
  local bridge_env_script="${NUWAX_ROS_BRIDGE_ENV_SCRIPT:-}"
  if [[ -z "${bridge_env_script}" && -n "${NUWAX_GEOMETRY_EXTERNAL_SCRIPT:-}" ]]; then
    bridge_env_script="$(dirname "${NUWAX_GEOMETRY_EXTERNAL_SCRIPT}")/scripts/env.sh"
  fi
  local bridge_map_topic="${NUWAX_MAP_TOPIC:-${NUWAX_GEOMETRY_OCCUPANCY_TOPIC:-/map_obstacles}}"
  local bridge_tf_topic="${NUWAX_TF_TOPIC:-${NUWAX_GEOMETRY_TF_TOPIC:-/tf}}"
  local bridge_nav_action="${NUWAX_NAV_ACTION:-${NUWAX_NAV2_ACTION_NAME:-/navigate_to_pose}}"
  local bridge_odom_source="${NUWAX_GO2_ODOM_SOURCE:-${NUWAX_GEOMETRY_ODOM_TOPIC:-/utlidar/robot_odom}}"
  local bridge_cloud_source="${NUWAX_GO2_POINTCLOUD_SOURCE:-${NUWAX_GEOMETRY_POINT_CLOUD_TOPIC:-/utlidar/cloud}}"
  local geometry_cyclonedds
  geometry_cyclonedds="$(geometry_cyclonedds_uri)"
  local cyclonedds_lib_dir="${NUWAX_GEOMETRY_CYCLONEDDS_LIB_DIR:-${G1_CYCLONEDDS_LIB_DIR:-${GO2_CYCLONEDDS_LIB_DIR:-}}}"
  echo "starting Rust ros-bridge: ${binary}"
  (
    set +u
    export NUWAX_MAP_TOPIC="${bridge_map_topic}"
    export NUWAX_TF_TOPIC="${bridge_tf_topic}"
    export NUWAX_NAV_ACTION="${bridge_nav_action}"
    export NUWAX_GO2_ODOM_SOURCE="${bridge_odom_source}"
    export NUWAX_GO2_POINTCLOUD_SOURCE="${bridge_cloud_source}"
    if [[ -n "${geometry_cyclonedds}" ]]; then
      export CYCLONEDDS_URI="${geometry_cyclonedds}"
    else
      unset CYCLONEDDS_URI
    fi
    if [[ -f "${bridge_env_script}" ]]; then
      source "${bridge_env_script}"
    elif [[ -f "${REPO_ROOT}/services/geometry/source_runtime_env.sh" ]]; then
      export NUWAX_GEOMETRY_RUNTIME_SOURCE_ROS_ENV="${NUWAX_GEOMETRY_RUNTIME_SOURCE_ROS_ENV:-true}"
      # shellcheck disable=SC1091
      source "${REPO_ROOT}/services/geometry/source_runtime_env.sh"
    else
      source /opt/ros/foxy/setup.bash
      export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    fi
    if [[ -n "${cyclonedds_lib_dir}" && -d "${cyclonedds_lib_dir}" ]]; then
      case ":${LD_LIBRARY_PATH:-}:" in
        *":${cyclonedds_lib_dir}:"*) ;;
        *)
          if [[ -n "${LD_LIBRARY_PATH:-}" ]]; then
            export LD_LIBRARY_PATH="${cyclonedds_lib_dir}:${LD_LIBRARY_PATH}"
          else
            export LD_LIBRARY_PATH="${cyclonedds_lib_dir}"
          fi
          ;;
      esac
    fi
    set -u
    run_with_title "nuwax-ros-bridge" "${binary}"
  ) &
  ROS_BRIDGE_PID="$!"
  wait_for_video_uds_socket || echo "WARNING: Rust ros-bridge UDS not ready (pid ${ROS_BRIDGE_PID})" >&2
}

start_go2_video_streamer() {
  if [[ -n "${VIDEO_STREAMER_PID}" ]] && is_pid_alive "${VIDEO_STREAMER_PID}"; then
    return 0
  fi
  if ! should_start_go2_video_streamer; then
    return 0
  fi
  echo "starting Go2 side video streamer (unitree_sdk2py) -> UDS /tmp/nuwax_video.sock"
  (
    if [[ -f "${GO2_DATA_PLANE_ENV_FILE}" ]]; then
      set +u
      # shellcheck disable=SC1090
      source "${GO2_DATA_PLANE_ENV_FILE}"
      set -u
    fi
    run_with_title "nuwax-go2-video-streamer" "${PYTHON_BIN}" -m drivers.go2.video_streamer
  ) &
  VIDEO_STREAMER_PID="$!"
}

start_g1_video_streamer() {
  if [[ -n "${VIDEO_STREAMER_PID}" ]] && is_pid_alive "${VIDEO_STREAMER_PID}"; then
    return 0
  fi
  if ! should_start_g1_video_streamer; then
    return 0
  fi
  echo "starting G1 side video streamer (unitree_sdk2py) -> UDS /tmp/nuwax_video.sock"
  (
    if [[ -f "${G1_DATA_PLANE_ENV_FILE}" ]]; then
      set +u
      # shellcheck disable=SC1090
      source "${G1_DATA_PLANE_ENV_FILE}"
      set -u
    fi
    run_with_title "nuwax-g1-video-streamer" "${PYTHON_BIN}" -m drivers.g1.video_streamer
  ) &
  VIDEO_STREAMER_PID="$!"
}

start_video_streamer() {
  wait_for_video_uds_socket || true
  prepare_robot_video_backend
  case "$(active_robot_name)" in
    go2)
      start_go2_video_streamer
      ;;
    g1)
      start_g1_video_streamer
      ;;
    *)
      if should_start_go2_video_streamer; then
        start_go2_video_streamer
      elif should_start_g1_video_streamer; then
        start_g1_video_streamer
      fi
      ;;
  esac
}

monitor_video_streamer() {
  while [[ "${STOP_REQUESTED}" -eq 0 ]]; do
    sleep 5
    if [[ -z "${VIDEO_STREAMER_PID}" ]] || ! is_pid_alive "${VIDEO_STREAMER_PID}"; then
      if should_start_video_streamer; then
        echo "video streamer (pid ${VIDEO_STREAMER_PID:-none}) exited, restarting..." >&2
        VIDEO_STREAMER_PID=""
        start_video_streamer
      fi
    fi
  done
}

stop_video_relay() {
  if [[ -n "${ROS_BRIDGE_PID}" ]] && is_pid_alive "${ROS_BRIDGE_PID}"; then
    terminate_process_scope "${ROS_BRIDGE_PID}" "rust ros-bridge"
    wait "${ROS_BRIDGE_PID}" >/dev/null 2>&1 || true
  fi
  ROS_BRIDGE_PID=""
  if [[ -n "${VIDEO_RELAY_PID}" ]] && is_pid_alive "${VIDEO_RELAY_PID}"; then
    terminate_process_scope "${VIDEO_RELAY_PID}" "python video relay"
    wait "${VIDEO_RELAY_PID}" >/dev/null 2>&1 || true
  fi
  VIDEO_RELAY_PID=""
  rm -f /tmp/nuwax_video.sock /tmp/nuwax_video.shm /tmp/nuwax_pointcloud.shm 2>/dev/null || true
}

stop_ros_bridge() {
  stop_video_relay
}

monitor_video_relay() {
  while [[ "${STOP_REQUESTED}" -eq 0 ]]; do
    sleep 5
    if should_start_ros2_bridge; then
      if [[ -n "${ROS_BRIDGE_PID}" ]] && ! is_pid_alive "${ROS_BRIDGE_PID}"; then
        echo "Rust ros-bridge (pid ${ROS_BRIDGE_PID}) crashed, restarting..." >&2
        rm -f /tmp/nuwax_video.sock /tmp/nuwax_video.shm /tmp/nuwax_pointcloud.shm 2>/dev/null || true
        ROS_BRIDGE_PID=""
        start_ros_bridge
      fi
      continue
    fi
    if [[ -n "${VIDEO_RELAY_PID}" ]] && ! is_pid_alive "${VIDEO_RELAY_PID}"; then
      echo "Python video relay (pid ${VIDEO_RELAY_PID}) exited, restarting..." >&2
      rm -f /tmp/nuwax_video.sock /tmp/nuwax_video.shm 2>/dev/null || true
      VIDEO_RELAY_PID=""
      start_python_video_relay
    fi
  done
}

monitor_ros_bridge() {
  monitor_video_relay
}

stop_video_streamer() {
  if [[ -n "${VIDEO_STREAMER_PID}" ]] && is_pid_alive "${VIDEO_STREAMER_PID}"; then
    terminate_process_scope "${VIDEO_STREAMER_PID}" "video streamer"
    wait "${VIDEO_STREAMER_PID}" >/dev/null 2>&1 || true
  fi
  VIDEO_STREAMER_PID=""
  # 兜底：清理任何未被本脚本跟踪的残留 streamer（如 systemd/手动启动的），
  # 确保网关停止后不会留下推流进程。
  cleanup_stale_video_streamer
}

stop_gateway() {
  STOP_REQUESTED=1
  # 顺序：先杀网关 worker → 再停 ros-bridge → 最后停 video streamer。
  # 关键是先切断网关（geometry / perception 客户端），否则 ros-bridge 一停，
  # 客户端在重启窗口期 GetFrameTree 立即抛 UNAVAILABLE，日志刷一长串
  # "failed to connect to all addresses; last error: UNKNOWN: ipv4:127.0.0.1:50051"。
  if [[ -n "${GATEWAY_PID}" ]] && kill -0 "${GATEWAY_PID}" >/dev/null 2>&1; then
    terminate_process_scope "${GATEWAY_PID}" "gateway worker"
    wait "${GATEWAY_PID}" >/dev/null 2>&1 || true
  fi
  stop_ros_bridge
  stop_video_streamer
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
G1_DATA_PLANE_ENV_FILE="${G1_DATA_PLANE_ENV_FILE:-${REPO_ROOT}/drivers/g1/data_plane.env}"
if [[ -f "${G1_DATA_PLANE_ENV_FILE}" ]]; then
  load_env_file "${G1_DATA_PLANE_ENV_FILE}"
fi

# 当 NUWAX_ACTIVE_ROBOT=g1 时，从项目根 .env.G1_example 回填空键。
# v6 修订（OpenSpec g1-rtabnav2-adapter）：
#   - load_env_file 对已 export 变量不会覆盖（G1 回填时不能 source 强覆盖，会破坏 .env 已配项）；
#   - v5 的 set -a source 强制 export 是错误的：用户手动配的 GO2 默认值会被 G1 默认值强行覆盖；
#   - 正确语义："只填空键，不覆盖已有值"（含未定义 + 已定义但值为空两种情况）。
#   - load_env_file_fill_empty 函数：本函数在 load_env_file 基础上，对每行判断目标键是否已 export
#     且非空；只有"未定义 或 空字符串"才 export 模板里的值；已 export 非空值保持不变。
load_env_file_fill_empty() {
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
    if [[ ${#value} -ge 2 ]]; then
      if [[ "${value:0:1}" == '"' && "${value: -1}" == '"' ]]; then
        value="${value:1:${#value}-2}"
      elif [[ "${value:0:1}" == "'" && "${value: -1}" == "'" ]]; then
        value="${value:1:${#value}-2}"
      fi
    fi
    if [[ -z "${key}" ]]; then
      continue
    fi
    # 关键：只在变量未定义 或 已定义但值为空字符串时填入模板值
    if [[ -z "${!key+x}" || -z "${!key:-}" ]]; then
      export "${key}=${value}"
    fi
  done < "${target}"
}

G1_ROBOT_ENV_FILE="${G1_ROBOT_ENV_FILE:-${REPO_ROOT}/.env.G1_example}"
if [[ "$(printf '%s' "${NUWAX_ACTIVE_ROBOT:-}" | tr '[:upper:]' '[:lower:]')" == "g1" \
   && -f "${G1_ROBOT_ENV_FILE}" ]]; then
  load_env_file_fill_empty "${G1_ROBOT_ENV_FILE}"
fi

unset RMW_IMPLEMENTATION
if grep -q '^CYCLONEDDS_URI=' "${ENV_FILE}" 2>/dev/null; then
  CYCLONEDDS_URI="$(grep '^CYCLONEDDS_URI=' "${ENV_FILE}" | head -1)"
  CYCLONEDDS_URI="${CYCLONEDDS_URI#CYCLONEDDS_URI=}"
  export CYCLONEDDS_URI
fi

PYTHON_BIN="${NUWAX_PYTHON_BIN:-${PYTHON_BIN:-python3}}"
PID_FILE="${NUWAX_GATEWAY_PID_FILE:-${REPO_ROOT}/runtime_data/gateway/gateway.pid}"
LOCK_FILE="${NUWAX_GATEWAY_LOCK_FILE:-${REPO_ROOT}/runtime_data/gateway/start_gateway.lock}"
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
    cleanup_stale_ros_bridge_processes
    cleanup_stale_video_streamer
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
  if is_truthy "${NUWAX_GEOMETRY_REQUIRE_ROS2:-true}"; then
    export NUWAX_GEOMETRY_RUNTIME_SOURCE_ROS_ENV="${NUWAX_GEOMETRY_RUNTIME_SOURCE_ROS_ENV:-true}"
  fi
  source_runtime_script "${REPO_ROOT}/services/geometry/source_runtime_env.sh"
fi

if is_truthy "${NUWAX_GEOMETRY_ENABLED:-true}" && is_truthy "${NUWAX_GEOMETRY_REQUIRE_ROS2:-true}"; then
  export NUWAX_ROS2_BRIDGE_ENABLED="${NUWAX_ROS2_BRIDGE_ENABLED:-true}"
else
  export NUWAX_ROS2_BRIDGE_ENABLED="${NUWAX_ROS2_BRIDGE_ENABLED:-false}"
fi

export PYTHONUNBUFFERED=1
export GO2_VIDEO_STREAM_GATEWAY_URL="${GO2_VIDEO_STREAM_GATEWAY_URL:-ws://127.0.0.1:${PORT}/streams/video.ingest}"
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
  # v8 防护：uvicorn 5s 健康检查（监听 8766）
  if ! wait_for_port_listen "${PORT}" 5; then
    echo "[v8 FATAL] uvicorn gateway not listening on port ${PORT} after 5s; killing to prevent restart loop" >&2
    kill -TERM "${GATEWAY_PID}" 2>/dev/null || true
    exit 1
  fi
  start_video_relay
  start_video_streamer
  monitor_video_relay &
  monitor_video_streamer &
  wait_for_video_shm_frame &
  wait "${GATEWAY_PID}"
  exit_code="$?"
  stop_video_streamer
  stop_ros_bridge
  exit "${exit_code}"
fi

attempt=0
while true; do
  set +e
  (run_with_title "nuwax-gateway-web" "${PYTHON_BIN}" -m uvicorn gateways.app:create_gateway_app_from_env --factory --host "${HOST}" --port "${PORT}") &
  GATEWAY_PID="$!"
  # v8 防护：uvicorn 5s 健康检查（监听 8766）
  if ! wait_for_port_listen "${PORT}" 5; then
    echo "[v8 FATAL] uvicorn gateway not listening on port ${PORT} after 5s; killing to prevent restart loop" >&2
    kill -TERM "${GATEWAY_PID}" 2>/dev/null || true
    exit 1
  fi
  start_video_relay
  start_video_streamer
  monitor_video_relay &
  monitor_video_streamer &
  wait_for_video_shm_frame &
  wait "${GATEWAY_PID}"
  exit_code="$?"
  stop_video_streamer
  GATEWAY_PID=""
  set -e

  if ! should_restart "${exit_code}"; then
    exit "${exit_code}"
  fi

  attempt=$((attempt + 1))
  if [[ "${RESTART_MAX_ATTEMPTS}" != "0" && "${attempt}" -ge "${RESTART_MAX_ATTEMPTS}" ]]; then
    echo "gateway exited with code ${exit_code}; restart attempts exhausted (${attempt})." >&2
    exit "${exit_code}"
  fi

  echo "gateway exited with code ${exit_code}; restarting in ${RESTART_SEC}s (attempt ${attempt})." >&2
  sleep "${RESTART_SEC}"
done
