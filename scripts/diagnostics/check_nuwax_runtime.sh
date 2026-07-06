#!/usr/bin/env bash
# scripts/diagnostics/check_nuwax_runtime.sh
#
# 一页 evidence bundle —— 不依赖 grpcurl / lcm-logger / 外网。
#
# 用法:
#   bash scripts/diagnostics/check_nuwax_runtime.sh                # 默认 127.0.0.1:50051
#   NUWAX_GEOMETRY_RUNTIME_PORT=50051 bash scripts/diagnostics/check_nuwax_runtime.sh
#
# 输出段:
#   1. 守门 — no_ros2_residual
#   2. 二进制 — nuwax-geometry-runtime 可执行/未执行
#   3. 端口 — 50051 监听状态
#   4. ready file — runtime_data/geometry/nuwax_geometry_runtime.ready
#   5. SHM refs — /tmp/nuwax_video.{sock,shm} /tmp/nuwax_pointcloud.shm
#   6. gRPC status — python3 + grpc 调用 GetStatus / HealthCheck /
#                    GetNavigationReadiness / GetExplorationState(grpc 缺失则 NOT RUN)
#   7. env canonical — 抓 backend/profile 关键 env
#   8. 日志尾部 — nuwax_geometry_runtime.log 最近 20 行
#
# 退出码:0  evidence 全部产出(允许若干段 NOT READY)
#
# Phase 7 一次性脚本:运行结果归档到
# docs/开发文档/建图导航探索重构/2026-07-06_.../70_installer_systemd_实机验收_phase7.md

set -uo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "${REPO_ROOT}"

GATEWAY_RUNTIME_PORT="${NUWAX_GEOMETRY_RUNTIME_PORT:-50051}"
GATEWAY_RUNTIME_READY_FILE="${NUWAX_GEOMETRY_RUNTIME_READY_FILE:-${REPO_ROOT}/runtime_data/geometry/nuwax_geometry_runtime.ready}"
GATEWAY_LOG_FILE="${NUWAX_LOG_DIR:-${REPO_ROOT}}/nuwax_geometry_runtime.log"

PYTHON_BIN="${PYTHON_BIN:-python3}"

bar() { printf '\n========== %s ==========\n' "$1"; }
ok()  { printf '  [OK]      %s\n' "$1"; }
miss(){ printf '  [MISSING] %s\n' "$1"; }
nr()  { printf '  [NOT RUN] %s\n' "$1"; }

bar "1. 守门:scripts/ci/no_ros2_residual.sh"
if bash scripts/ci/no_ros2_residual.sh >/tmp/check_nuwax_runtime.gate.log 2>&1; then
  ok "no_ros2_residual EXIT 0"
else
  miss "no_ros2_residual EXIT non-zero (见 /tmp/check_nuwax_runtime.gate.log)"
  tail -n 20 /tmp/check_nuwax_runtime.gate.log
fi

bar "2. 二进制:nuwax-geometry-runtime"
RUNTIME_BIN="${REPO_ROOT}/services/geometry/nuwax_geometry_runtime/target/release/nuwax-geometry-runtime"
if [[ -x "${RUNTIME_BIN}" ]]; then
  ok "executable: ${RUNTIME_BIN}"
else
  miss "二进制不存在或不可执行: ${RUNTIME_BIN}"
  echo "         Phase 7 不强制 build,需实机 cargo build --release --locked"
fi

bar "3. 端口:${GATEWAY_RUNTIME_PORT} 监听"
if command -v ss >/dev/null 2>&1; then
  if ss -ltn 2>/dev/null | awk '{print $4}' | grep -qE "(^|:)${GATEWAY_RUNTIME_PORT}$"; then
    ok "端口 ${GATEWAY_RUNTIME_PORT} LISTEN"
  else
    nr "端口 ${GATEWAY_RUNTIME_PORT} 未 LISTEN(runtime 未启/已退出)"
  fi
elif command -v netstat >/dev/null 2>&1; then
  if netstat -ltn 2>/dev/null | awk '{print $4}' | grep -qE "(^|:)${GATEWAY_RUNTIME_PORT}$"; then
    ok "端口 ${GATEWAY_RUNTIME_PORT} LISTEN"
  else
    nr "端口 ${GATEWAY_RUNTIME_PORT} 未 LISTEN"
  fi
else
  nr "ss/netstat 都不可用"
fi

bar "4. ready file:${GATEWAY_RUNTIME_READY_FILE}"
if [[ -f "${GATEWAY_RUNTIME_READY_FILE}" ]]; then
  ok "ready file 存在,内容=$(cat "${GATEWAY_RUNTIME_READY_FILE}" 2>/dev/null || echo '<empty>')"
else
  nr "ready file 不存在(runtime 未启/已退出)"
fi

bar "5. SHM refs"
for f in /tmp/nuwax_video.sock /tmp/nuwax_video.shm /tmp/nuwax_pointcloud.shm; do
  if [[ -e "${f}" ]]; then
    ok "存在: ${f}"
  else
    nr "缺失: ${f}(未启用/未启动对应组件)"
  fi
done

bar "6. gRPC status (python3 + grpcio)"
GRPC_OUT="$(
  PYTHONPATH="${REPO_ROOT}/services/geometry/proto_gen" \
    "${PYTHON_BIN}" - <<'PY' 2>/dev/null
import sys, os
try:
    import grpc  # noqa: F401
    import geometry_runtime_pb2 as pb
    import geometry_runtime_pb2_grpc as pbg
except Exception as e:
    print(f"GRPC_NOT_AVAILABLE: {e} (无需 grpcio 系统 python3,装: pip install grpcio+protobuf)")
    sys.exit(0)
port = int(os.environ.get("NUWAX_GEOMETRY_RUNTIME_PORT", "50051"))
target = f"127.0.0.1:{port}"
try:
    ch = grpc.insecure_channel(target)
    stub = pbg.GeometryRuntimeStub(ch)
    s = stub.GetStatus(pb.Empty(), timeout=3.0)
    print(f"phase={s.phase} uptime_ms={s.uptime_ms} rss_kb={s.rss_kb} lcm_backend={s.lcm_backend}")
    h = stub.HealthCheck(pb.HealthCheckRequest(), timeout=3.0)
    print(f"healthy={h.healthy} phase={h.phase} lcm_backend={h.lcm_backend}")
    try:
        r = stub.GetNavigationReadiness(pb.Empty(), timeout=3.0)
        print(f"nav.ready={r.ready} nav.reason={r.reason}")
    except grpc.RpcError as e:
        print(f"nav.readiness=RPC_ERROR({e.code().name})")
    try:
        f = stub.GetExplorationState(pb.Empty(), timeout=3.0)
        print(f"explore.status={f.status} explore.message={f.message}")
    except grpc.RpcError as e:
        print(f"explore.state=RPC_ERROR({e.code().name})")
except grpc.RpcError as e:
    print(f"RPC_ERROR({e.code().name}): {e.details()}")
except Exception as e:
    print(f"ERROR: {e}")
PY
)"
if [[ -z "${GRPC_OUT}" ]]; then
  nr "gRPC 调用无输出"
else
  echo "${GRPC_OUT}" | sed 's/^/  /'
fi

bar "7. env canonical"
for k in NUWAX_LCM_BACKEND NUWAX_LCM_URL NUWAX_NAV_PROFILE \
         NUWAX_EXPLORATION_LEGACY NUWAX_EXPLORE_ARBITRATION \
         NUWAX_REAL_LCM_REQUIRED NUWAX_GEOMETRY_RUNTIME_PORT \
         NUWAX_GEOMETRY_RUNTIME_ENABLED; do
  v="${!k:-<unset>}"
  printf '  %-32s = %s\n' "${k}" "${v}"
done

bar "8. 日志尾部 (last 20 lines)"
if [[ -f "${GATEWAY_LOG_FILE}" ]]; then
  tail -n 20 "${GATEWAY_LOG_FILE}" | sed 's/^/  /'
else
  nr "日志文件不存在: ${GATEWAY_LOG_FILE}"
fi

echo
echo "==> evidence bundle 完成 (Phase 7 / 2026-07-06)"
exit 0
