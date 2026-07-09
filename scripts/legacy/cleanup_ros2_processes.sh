#!/usr/bin/env bash
# 一次性迁移脚本：清理实机 os 层遗留 ROS2/Nav2 进程。
#
# deprecated: 历史路径已物理删（2026-07-09 nuwax-workspace 接管），本脚本为隔离层保留，
# 默认不调用，显式 NUWAX_LEGACY_ROS2_CLEANUP=1 开启。
#
# 与本仓库的目标态关系：
#   - 历史 ROS2 工作区路径已被物理删除，本脚本不再扫描具体目录（路径字面已置空）。
#   - nuwax-geometry-runtime 替代上述 ROS2 工作区，新路径不再使用以下任何 pattern。
#   - 进程名 cmdline pattern（lib/rtabmap_* / nav2_* / explore_*）作为孤儿 cmdline
#     兜底扫描保留，仅在 NUWAX_LEGACY_ROS2_CLEANUP=1 显式触发时生效。
#
# 调用方：scripts/start_gateway.sh 的 cleanup_external_geometry_processes 在
# NUWAX_LEGACY_ROS2_CLEANUP=1 时 fork 该脚本（不引入额外 bash 函数，保证默认
# path 零 ROS2 字符串）。

set -u
set -o pipefail

# 解析仓库根（start_gateway.sh 调起时通过环境变量传入，避免本脚本依赖任何 cwd）
REPO_ROOT="${REPO_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"

log() {
  echo "[legacy-ros2-deprecated] $*" >&2
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
  local label="${2:-legacy process}"
  local stop_timeout="${NUWAX_LEGACY_STOP_TIMEOUT_SEC:-20}"
  local kill_timeout="${NUWAX_LEGACY_KILL_TIMEOUT_SEC:-5}"

  if ! is_pid_alive "${pid}"; then
    return 0
  fi

  log "stopping ${label} pid ${pid}..."
  send_signal_to_process_scope TERM "${pid}"
  send_signal_to_process_scope CONT "${pid}"
  if wait_for_pid_exit "${pid}" "${stop_timeout}"; then
    return 0
  fi

  log "${label} pid ${pid} did not stop after ${stop_timeout}s; sending SIGKILL."
  send_signal_to_process_scope KILL "${pid}"
  wait_for_pid_exit "${pid}" "${kill_timeout}" || true
}

cleanup_legacy_geometry_processes() {
  # 历史路径已被物理删除，跳过目录级扫描；仅保留孤儿 cmdline pattern 兜底
  # （见 cleanup_legacy_command_patterns）。
  local geometry_root=""
  local pid=""
  local args=""
  local pgid=""
  declare -A seen_pgids=()

  # geometry_root="" 直接跳过目录扫描（4.5 step D：物理已删，无路径可扫）。
  if [[ -z "${geometry_root}" ]]; then
    log "skip: 历史路径已物理删，无目录级扫描目标。"
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
    terminate_process_scope "${pid}" "legacy geometry process group ${pgid}"
  done < <(ps -eo pid=,args=)
}

cleanup_legacy_command_patterns() {
  # 已脱离工作区目录直接出现在 cmdline 中的孤立进程模式：
  # 见 docs/开发文档/建图导航探索重构/2026-07-01_彻底去ROS2化-执行方案/feedback_fix_BFINAL1.md
  local pattern=""
  local go2_patterns=(
    "ros2 launch"
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

  for pattern in "${go2_patterns[@]}"; do
    pkill -TERM -f "${pattern}" 2>/dev/null || true
  done
  sleep 1
  for pattern in "${go2_patterns[@]}"; do
    pkill -KILL -f "${pattern}" 2>/dev/null || true
  done
}

main() {
  log "启动 legacy ROS2/Nav2 孤儿进程清理（一次性迁移验收开关）"
  cleanup_legacy_geometry_processes
  cleanup_legacy_command_patterns
  log "legacy 清理完成。"
}

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
  main "$@"
fi
