#!/usr/bin/env bash
# services/geometry/scripts/start_nuwax_geometry_runtime.sh
#
# 启动 nuwax-geometry-runtime Rust 二进制，承载 LCM-first 建图 / 定位 / 探索 / 导航运行时
#（2026-07-03 drop-ros2 后替代 ROS2 工作区 + colcon build + NuwaxSLAM launch 链路）。
#
# 不变量：
#   - 不 source /opt/ros/*/setup.bash
#   - 不调用 ros2 launch
#   - 不依赖 colcon / rosdep / libddsc.so / CycloneDDS
#   - 仅启动 nuwax-geometry-runtime 并等待 HealthCheck ready
#
# 退出码：
#   0  - runtime 启动并 ready
#   1  - 缺少二进制 / 必需 env 缺失 / 启动失败

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# scripts/ 位于 services/geometry/scripts，所以向上 3 层到 repo 根：
#   scripts -> geometry -> services -> REPO_ROOT
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"

# --- 1. 加载通用 env（去 ROS2 化后的 source_runtime_env.sh）---
# shellcheck disable=SC1091
source "${REPO_ROOT}/services/geometry/source_runtime_env.sh"

# --- 2. 解析 binary / port / ready_file ---
# Phase 0 收敛：env-only 启动契约。Rust main.rs 只读 env，不再传 CLI args，
# 所以必需 env 必须在本脚本显式兜底，否则在 CI / 容器 / 开发机没有 web/.env 时
# Rust 端只能拿到空字符串并启动失败。
#
# env-only 契约兜底规则：
#   - 仅在 env 未设置时设置（`: "${VAR:=default}"` 不会覆盖 .env 已 export 的值）
#   - binary / port / ready_file / LCM URL/backend/channel 全部走默认值
#   - 与 installer 写出的 web/.env / .env / runtime_data/.env 兼容
export NUWAX_GEOMETRY_RUNTIME_BINARY="${NUWAX_GEOMETRY_RUNTIME_BINARY:-${REPO_ROOT}/services/geometry/nuwax_geometry_runtime/target/release/nuwax-geometry-runtime}"
export NUWAX_GEOMETRY_RUNTIME_PORT="${NUWAX_GEOMETRY_RUNTIME_PORT:-50051}"
export NUWAX_GEOMETRY_RUNTIME_READY_FILE="${NUWAX_GEOMETRY_RUNTIME_READY_FILE:-${REPO_ROOT}/runtime_data/geometry/nuwax_geometry_runtime.ready}"
export NUWAX_GEOMETRY_RUNTIME_HEALTH_TIMEOUT="${NUWAX_GEOMETRY_RUNTIME_HEALTH_TIMEOUT:-30}"

# LCM URL/backend 兜底（与 settings.py::build_default_lcm_url / fake-lcm 默认对齐）
export NUWAX_LCM_URL="${NUWAX_LCM_URL:-udp://239.255.76.67:7667?ttl=1}"
export NUWAX_LCM_BACKEND="${NUWAX_LCM_BACKEND:-fake-lcm}"

# Phase 6 typed channel 兜底（与 lcm_runtime 默认 channel registry 对齐）
export NUWAX_LCM_CHANNEL_ODOM="${NUWAX_LCM_CHANNEL_ODOM:-/utlidar/robot_odom}"
export NUWAX_LCM_CHANNEL_TF="${NUWAX_LCM_CHANNEL_TF:-/tf}"
export NUWAX_LCM_CHANNEL_POINTCLOUD="${NUWAX_LCM_CHANNEL_POINTCLOUD:-/utlidar/cloud_deskewed}"
export NUWAX_LCM_CHANNEL_OCCUPANCY="${NUWAX_LCM_CHANNEL_OCCUPANCY:-/map_obstacles}"
export NUWAX_LCM_CHANNEL_CMD_VEL="${NUWAX_LCM_CHANNEL_CMD_VEL:-/cmd_vel}"

# --- 2.5 Dry-run 模式（必须在 binary 检查之前）：当 `NUWAX_DRY_RUN=1` 时只 echo
# 启动命令与解析后的 env，不实际 nohup 启动 binary。用于 CI / 容器 / 开发机
# sanity-check env 是否齐全（无需真实 cargo build 产物）。
#
# Phase 6 (codex-round3 R3-2)：dry-run 模式下不允许做文件系统操作。
# dev 机的 home 是 /home/ml（不是 /home/unitree），LOG_DIR 默认会落到
# /home/unitree/nuwax-robot-bridge，mkdir 会在 dev 上失败并 exit 1。
# 把 mkdir 全部移到 dry-run 判读之后——dry-run 应该是纯 echo + env dump。
echo "==> 启动 nuwax-geometry-runtime: ${NUWAX_GEOMETRY_RUNTIME_BINARY}"
echo "    gRPC port=${NUWAX_GEOMETRY_RUNTIME_PORT} ready_file=${NUWAX_GEOMETRY_RUNTIME_READY_FILE}"
echo "    LCM_URL=${NUWAX_LCM_URL:-<unset>} LCM_BACKEND=${NUWAX_LCM_BACKEND:-<unset>}"

if [[ "${NUWAX_DRY_RUN:-0}" == "1" ]]; then
  echo "==> NUWAX_DRY_RUN=1：仅 echo 启动命令，不实际 nohup binary，不创建任何目录。"
  LOG_DIR="${NUWAX_LOG_DIR:-${REPO_ROOT}}"
  LOG_FILE="${LOG_DIR}/nuwax_geometry_runtime.log"
  echo "    binary=${NUWAX_GEOMETRY_RUNTIME_BINARY}"
  echo "    log=${LOG_FILE}（dry-run 下不创建）"
  echo "    全部 env:"
  env | grep -E '^NUWAX_(GEOMETRY|LCM)' | sort || true
  exit 0
fi

# 真实启动才创建 dir（dry-run 跳过，避免 dev 机上 mkdir 失败）
mkdir -p "$(dirname "${NUWAX_GEOMETRY_RUNTIME_READY_FILE}")"
rm -f "${NUWAX_GEOMETRY_RUNTIME_READY_FILE}"

LOG_DIR="${NUWAX_LOG_DIR:-${REPO_ROOT}}"
LOG_FILE="${LOG_DIR}/nuwax_geometry_runtime.log"
mkdir -p "$(dirname "${LOG_FILE}")"

# --- 3. 启动 runtime ---
# Phase 0 收敛：env-only 启动契约。Rust main.rs 只读 env，不再传 CLI args。
# 旧 `--port/--ready-file/--lcm-url` 已被 nuwax-geometry-runtime main.rs:49-83 拒绝
# （main.rs 当前只接受 env 解析），继续传只会让 binary 报 unknown arg。
if [[ ! -x "${NUWAX_GEOMETRY_RUNTIME_BINARY}" ]]; then
  echo "ERROR: nuwax-geometry-runtime 二进制不存在或不可执行: ${NUWAX_GEOMETRY_RUNTIME_BINARY}" >&2
  echo "       请先在 ${REPO_ROOT}/services/geometry/nuwax_geometry_runtime/ 执行:" >&2
  echo "         cargo build --release --locked" >&2
  exit 1
fi
nohup "${NUWAX_GEOMETRY_RUNTIME_BINARY}" \
  >> "${LOG_FILE}" 2>&1 &
RUNTIME_PID=$!
echo "==> runtime PID=${RUNTIME_PID}, 日志=${LOG_FILE}"

# --- 4. 等待 HealthCheck ready ---
deadline=$(( $(date +%s) + NUWAX_GEOMETRY_RUNTIME_HEALTH_TIMEOUT ))
while [[ $(date +%s) -lt ${deadline} ]]; do
  if [[ -f "${NUWAX_GEOMETRY_RUNTIME_READY_FILE}" ]]; then
    echo "==> nuwax-geometry-runtime ready (pid=${RUNTIME_PID})"
    exit 0
  fi
  if ! kill -0 "${RUNTIME_PID}" 2>/dev/null; then
    echo "ERROR: nuwax-geometry-runtime 进程已退出，请查日志 ${LOG_FILE}" >&2
    tail -n 50 "${LOG_FILE}" >&2 || true
    exit 1
  fi
  sleep 0.5
done

echo "ERROR: nuwax-geometry-runtime 在 ${NUWAX_GEOMETRY_RUNTIME_HEALTH_TIMEOUT}s 内未 ready" >&2
tail -n 50 "${LOG_FILE}" >&2 || true
kill "${RUNTIME_PID}" 2>/dev/null || true
exit 1