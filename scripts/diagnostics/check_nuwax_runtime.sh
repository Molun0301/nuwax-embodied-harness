#!/usr/bin/env bash
# scripts/diagnostics/check_nuwax_runtime.sh
#
# 一页 evidence bundle —— 不依赖 grpcurl / lcm-logger / 外网,但 Phase 8 起
# 必须主动跑 lcm roundtrip + 抓 channel counters,不再用"不依赖 lcm-logger"
# 作为完成证据（§5 Phase 8 任务 4 + §12.6）。
#
# 用法:
#   bash scripts/diagnostics/check_nuwax_runtime.sh                # 默认 127.0.0.1:50051
#   NUWAX_GEOMETRY_RUNTIME_PORT=50051 bash scripts/diagnostics/check_nuwax_runtime.sh
#
# 输出段:
#   1. 守门 — no_ros2_residual
#   2. motion guard — production module / UDS / durable record permissions
#   3. 二进制 — nuwax-geometry-runtime 可执行/未执行
#   3. 端口 — 50051 监听状态
#   4. ready file — runtime_data/geometry/nuwax_geometry_runtime.ready
#   5. SHM refs — /tmp/nuwax_video.{sock,shm} /tmp/nuwax_pointcloud.shm
#   6. gRPC status — python3 + grpc 调用 GetStatus / HealthCheck /
#                    GetNavigationReadiness / GetExplorationState(grpc 缺失则 NOT RUN)
#   7. env canonical — 抓 backend/profile 关键 env
#   8. 日志尾部 — nuwax_geometry_runtime.log 最近 20 行
#   9. LCM roundtrip — Python publisher+subscriber 跨进程互通（liblcm/udpm 不可用 → NOT RUN）
#  10. Channel counters — 解析 nuwax_geometry_runtime.log 中 typed channel pub/sub 计数
#  11. systemd unit — 检查 production unit 是否 hardcode NUWAX_LCM_BACKEND=real-lcm
#
# 退出码:0  evidence 全部产出(允许若干段 NOT READY)
#
# Phase 8 一次性脚本:运行结果归档到
# docs/开发文档/建图导航探索重构/2026-07-07_建图导航探索库真接入与real-lcm通信实施方案/

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

bar "2. motion guard:module / UDS / durable records"
GUARD_MODULE="${REPO_ROOT}/services/navigation/motion_safety_guard/__main__.py"
GUARD_SOCKET="${NUWAX_MOTION_GUARD_SOCKET:-/run/nuwax/motion-guard.sock}"
GUARD_LOCK="${NUWAX_MOTION_GUARD_LOCK:-/run/nuwax/motion-guard.lock}"
GUARD_STATE_DIR="${NUWAX_MOTION_GUARD_STATE_DIR:-/var/lib/nuwax/motion-guard}"
ACTUATOR_SYMBOLS_FILE="${NUWAX_ACTUATOR_SYMBOLS_FILE:-/usr/share/nuwax/actuator-symbols.json}"
if [[ -f "${GUARD_MODULE}" ]]; then
  ok "production module: services.navigation.motion_safety_guard"
else
  miss "production module missing: ${GUARD_MODULE}"
fi
if [[ -S "${GUARD_SOCKET}" ]]; then
  socket_meta="$(stat -c '%U:%G %a' "${GUARD_SOCKET}" 2>/dev/null || echo '<stat-failed>')"
  if [[ "${socket_meta}" == "nuwax-motion-guard:nuwax-motion-clients 660" ]]; then
    ok "guard UDS secured: ${GUARD_SOCKET} (${socket_meta})"
  else
    miss "guard UDS metadata drift: ${GUARD_SOCKET} (${socket_meta})"
  fi
else
  nr "guard UDS absent: ${GUARD_SOCKET}"
fi
if [[ -f "${GUARD_LOCK}" && ! -L "${GUARD_LOCK}" ]]; then
  lock_meta="$(stat -c '%U:%G %a' "${GUARD_LOCK}" 2>/dev/null || echo '<stat-failed>')"
  if [[ "${lock_meta}" == "nuwax-motion-guard:nuwax-motion-guard 600" ]]; then
    ok "guard lock secured: ${GUARD_LOCK} (${lock_meta})"
  else
    miss "guard lock metadata drift: ${GUARD_LOCK} (${lock_meta})"
  fi
else
  nr "guard lock absent/non-regular/symlink: ${GUARD_LOCK}"
fi
for record_spec in "provisioned.json:400" "epoch.json:600"; do
  record_name="${record_spec%%:*}"
  expected_mode="${record_spec##*:}"
  record_path="${GUARD_STATE_DIR}/${record_name}"
  if [[ -s "${record_path}" ]]; then
    record_meta="$(stat -c '%U:%G %a' "${record_path}" 2>/dev/null || echo '<stat-failed>')"
    if [[ "${record_meta}" == "nuwax-motion-guard:nuwax-motion-guard ${expected_mode}" ]]; then
      ok "durable record: ${record_path} (${record_meta})"
    else
      miss "durable record metadata drift: ${record_path} (${record_meta})"
    fi
  else
    miss "durable record missing or empty: ${record_path}"
  fi
done
if [[ -s "${ACTUATOR_SYMBOLS_FILE}" ]]; then
  manifest_meta="$(stat -c '%U:%G %a' "${ACTUATOR_SYMBOLS_FILE}" 2>/dev/null || echo '<stat-failed>')"
  if [[ "${manifest_meta}" == "root:root 444" ]]; then
    ok "actuator symbols manifest: ${ACTUATOR_SYMBOLS_FILE} (${manifest_meta})"
  else
    miss "actuator symbols manifest metadata drift: ${ACTUATOR_SYMBOLS_FILE} (${manifest_meta})"
  fi
else
  miss "actuator symbols manifest missing or empty: ${ACTUATOR_SYMBOLS_FILE}"
fi

bar "3. 二进制:nuwax-geometry-runtime"
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

bar "9. LCM roundtrip (Phase 8 §12.6 硬约束)"
if ! command -v pkg-config >/dev/null 2>&1 || ! pkg-config --modversion lcm >/dev/null 2>&1; then
  nr "liblcm-dev 不可用,跳过 roundtrip（CI 镜像需 apt install -y liblcm-dev pkg-config python3-lcm）"
elif ! python3 -c 'import lcm' >/dev/null 2>&1; then
  nr "python3-lcm 不可用,跳过 roundtrip（CI 镜像需 apt install -y python3-lcm）"
else
  RT_LOG="$(mktemp)"
  if bash "${REPO_ROOT}/scripts/ci/smoke_real_lcm_roundtrip.sh" >"${RT_LOG}" 2>&1; then
    ok "LCM 跨进程 roundtrip PASS"
  else
    rc=$?
    if [[ "${rc}" -eq 2 ]]; then
      nr "LCM roundtrip NOT_RUN（udpm multicast 在本网络被阻断,见 ${RT_LOG}）"
    else
      nr "LCM roundtrip FAIL rc=${rc}（详见 ${RT_LOG}）"
    fi
  fi
  tail -n 10 "${RT_LOG}" | sed 's/^/    /'
  rm -f "${RT_LOG}"
fi

bar "10. Channel counters (Phase 8 §12.6 / §13.5 关键 evidence)"
if [[ -f "${GATEWAY_LOG_FILE}" ]]; then
  # 解析最近 5000 行日志中 typed channel pub/sub 计数（不依赖 lcm-logger）
  declare -A CHAN_PUB_COUNT=() CHAN_SUB_COUNT=()
  recent_lines="$(wc -l < "${GATEWAY_LOG_FILE}")"
  scan_start=$(( recent_lines > 5000 ? recent_lines - 5000 : 0 ))
  while IFS= read -r line; do
    if [[ "${line}" =~ publish[[:space:]]+channel=([A-Za-z0-9_/.\\-]+) ]]; then
      ch="${BASH_REMATCH[1]}"
      CHAN_PUB_COUNT[$ch]=$(( ${CHAN_PUB_COUNT[$ch]:-0} + 1 ))
    fi
    if [[ "${line}" =~ subscribe[[:space:]]+channel=([A-Za-z0-9_/.\\-]+) ]]; then
      ch="${BASH_REMATCH[1]}"
      CHAN_SUB_COUNT[$ch]=$(( ${CHAN_SUB_COUNT[$ch]:-0} + 1 ))
    fi
  done < <(tail -n +"$((scan_start+1))" "${GATEWAY_LOG_FILE}" 2>/dev/null)
  if [[ ${#CHAN_PUB_COUNT[@]} -eq 0 && ${#CHAN_SUB_COUNT[@]} -eq 0 ]]; then
    nr "日志中无 typed channel pub/sub 事件（runtime 未启动 / 走 fake backend / 未运行 Phase 5/6 bridge）"
  else
    printf '  %-40s %10s %10s\n' "channel" "publish" "subscribe"
    all_channels="$(printf '%s\n' "${!CHAN_PUB_COUNT[@]}" "${!CHAN_SUB_COUNT[@]}" | sort -u)"
    while IFS= read -r ch; do
      [[ -z "${ch}" ]] && continue
      printf '  %-40s %10s %10s\n' "${ch}" "${CHAN_PUB_COUNT[$ch]:-0}" "${CHAN_SUB_COUNT[$ch]:-0}"
    done <<< "${all_channels}"
  fi
else
  nr "日志不存在,跳过 channel counters"
fi

bar "11. systemd units (production env + guard ordering)"
SERVICE_FILE="${REPO_ROOT}/deploy/systemd/nuwax-geometry-runtime.service"
GUARD_SERVICE_FILE="${REPO_ROOT}/deploy/systemd/nuwax-motion-safety-guard.service"
GATEWAY_SERVICE_FILE="${REPO_ROOT}/deploy/systemd/nuwax-host-gateway.service"
if [[ -f "${GUARD_SERVICE_FILE}" ]]; then
  if grep -qF 'ExecStart=/usr/bin/python3 -m services.navigation.motion_safety_guard' "${GUARD_SERVICE_FILE}"; then
    ok "guard unit references production Python owner"
  else
    miss "guard unit production ExecStart drift"
  fi
  if grep -qF 'Environment=NUWAX_MOTION_GUARD_SOCKET=/run/nuwax/motion-guard.sock' "${GUARD_SERVICE_FILE}"; then
    ok "guard unit UDS path frozen"
  else
    miss "guard unit UDS path drift"
  fi
  if grep -qF 'Environment=NUWAX_ACTUATOR_SYMBOLS_FILE=/usr/share/nuwax/actuator-symbols.json' "${GUARD_SERVICE_FILE}"; then
    ok "guard unit actuator manifest path frozen"
  else
    miss "guard unit actuator manifest path drift"
  fi
else
  miss "guard service file missing: ${GUARD_SERVICE_FILE}"
fi
if [[ -f "${SERVICE_FILE}" ]]; then
  if grep -qE '^Environment=NUWAX_LCM_BACKEND=real-lcm' "${SERVICE_FILE}"; then
    ok "service hardcode NUWAX_LCM_BACKEND=real-lcm"
  else
    miss "service 缺 NUWAX_LCM_BACKEND=real-lcm"
  fi
  if grep -qE '^Environment=NUWAX_REAL_LCM_REQUIRED=true' "${SERVICE_FILE}"; then
    ok "service hardcode NUWAX_REAL_LCM_REQUIRED=true"
  else
    miss "service 缺 NUWAX_REAL_LCM_REQUIRED=true"
  fi
  if grep -qE '^SuccessExitStatus=0 4' "${SERVICE_FILE}"; then
    miss "service 仍把 real-lcm fail-fast code(4) 当 SuccessExitStatus（违反 §13.2）"
  else
    ok "service SuccessExitStatus 仅 0"
  fi
  if grep -qF 'Requires=nuwax-motion-safety-guard.service' "${SERVICE_FILE}"; then
    ok "geometry runtime requires motion guard"
  else
    miss "geometry runtime does not require motion guard"
  fi
else
  nr "service 文件不存在: ${SERVICE_FILE}"
fi
if [[ -f "${GATEWAY_SERVICE_FILE}" ]] \
  && grep -qF 'Requires=nuwax-motion-safety-guard.service nuwax-geometry-runtime.service' "${GATEWAY_SERVICE_FILE}"; then
  ok "Gateway requires guard and geometry runtime"
else
  miss "Gateway systemd dependency chain drift"
fi

echo
echo "==> evidence bundle 完成 (Phase 8 / 2026-07-07)"
exit 0
