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
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

# --- 1. 加载通用 env（去 ROS2 化后的 source_runtime_env.sh）---
# shellcheck disable=SC1091
source "${REPO_ROOT}/services/geometry/source_runtime_env.sh"

# --- 2. 解析 binary / port / ready_file ---
NUWAX_GEOMETRY_RUNTIME_BINARY="${NUWAX_GEOMETRY_RUNTIME_BINARY:-${REPO_ROOT}/services/geometry/nuwax_geometry_runtime/target/release/nuwax-geometry-runtime}"
NUWAX_GEOMETRY_RUNTIME_PORT="${NUWAX_GEOMETRY_RUNTIME_PORT:-50051}"
NUWAX_GEOMETRY_RUNTIME_READY_FILE="${NUWAX_GEOMETRY_RUNTIME_READY_FILE:-${REPO_ROOT}/runtime_data/geometry/nuwax_geometry_runtime.ready}"
NUWAX_GEOMETRY_RUNTIME_HEALTH_TIMEOUT="${NUWAX_GEOMETRY_RUNTIME_HEALTH_TIMEOUT:-30}"

if [[ ! -x "${NUWAX_GEOMETRY_RUNTIME_BINARY}" ]]; then
  echo "ERROR: nuwax-geometry-runtime 二进制不存在或不可执行: ${NUWAX_GEOMETRY_RUNTIME_BINARY}" >&2
  echo "       请先在 ${REPO_ROOT}/services/geometry/nuwax_geometry_runtime/ 执行:" >&2
  echo "         cargo build --release --locked" >&2
  exit 1
fi

mkdir -p "$(dirname "${NUWAX_GEOMETRY_RUNTIME_READY_FILE}")"
rm -f "${NUWAX_GEOMETRY_RUNTIME_READY_FILE}"

# --- 3. 启动 runtime ---
LOG_DIR="${NUWAX_LOG_DIR:-${REPO_ROOT}}"
LOG_FILE="${LOG_DIR}/nuwax_geometry_runtime.log"
mkdir -p "$(dirname "${LOG_FILE}")"

echo "==> 启动 nuwax-geometry-runtime: ${NUWAX_GEOMETRY_RUNTIME_BINARY}"
echo "    gRPC port=${NUWAX_GEOMETRY_RUNTIME_PORT} ready_file=${NUWAX_GEOMETRY_RUNTIME_READY_FILE}"
echo "    LCM_URL=${NUWAX_LCM_URL:-<unset>}"
nohup "${NUWAX_GEOMETRY_RUNTIME_BINARY}" \
  --port "${NUWAX_GEOMETRY_RUNTIME_PORT}" \
  --ready-file "${NUWAX_GEOMETRY_RUNTIME_READY_FILE}" \
  --lcm-url "${NUWAX_LCM_URL:-udpm://239.255.76.67:7667?ttl=0}" \
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