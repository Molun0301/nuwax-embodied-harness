#!/usr/bin/env bash
# scripts/diagnostics/check_nuwax_runtime.sh
#
# 一页 evidence bundle —— 不依赖 grpcurl / lcm-logger / 外网；通过 multicast
# route/interface 预检与真实跨进程 roundtrip 证明 transport 可用。
#
# 用法:
#   bash scripts/diagnostics/check_nuwax_runtime.sh                # 默认 127.0.0.1:50051
#   NUWAX_GEOMETRY_RUNTIME_PORT=50051 bash scripts/diagnostics/check_nuwax_runtime.sh
#
# 输出段:
#   1. 诊断输入 — 不依赖仅供开发使用且未部署的 CI 脚本
#   2. motion guard — production module / UDS / durable record permissions
#   3. 二进制/端口 — nuwax-geometry-runtime 可执行与监听状态
#   4. ready file — /run/nuwax/geometry/nuwax_geometry_runtime.ready
#   5. SHM refs — video + cloudmap + occupancy 的真实生产路径
#   6. gRPC status — 部署解释器 + grpc 调用 GetStatus / HealthCheck /
#                    GetNavigationReadiness / GetExplorationState(grpc 缺失则 NOT RUN)
#   7. env canonical — 抓 backend/profile 关键 env
#   8. 日志尾部 — systemd journal 最近 20 行
#   9. LCM roundtrip — Python publisher+subscriber 跨进程互通（liblcm/udpm 不可用 → NOT RUN）
#  10. systemd unit/profile — 核对 production env 与懒启动契约
#
# 退出码:0  evidence 全部产出(允许若干段 NOT READY)
#
# Phase 8 一次性脚本:运行结果归档到
# docs/开发文档/建图导航探索重构/2026-07-07_建图导航探索库真接入与real-lcm通信实施方案/

set -uo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "${REPO_ROOT}"

GATEWAY_RUNTIME_PORT="${NUWAX_GEOMETRY_RUNTIME_PORT:-50051}"
GATEWAY_RUNTIME_READY_FILE="${NUWAX_GEOMETRY_RUNTIME_READY_FILE:-/run/nuwax/geometry/nuwax_geometry_runtime.ready}"
GEOMETRY_DATA_DIR="${NUWAX_GEOMETRY_DATA_DIR:-/var/lib/nuwax_robot_bridge}"
PROFILE_FILE="${NUWAX_DEPLOYMENT_PROFILE:-${REPO_ROOT}/deploy/profiles/go2-jetson.json}"
ACTIVATION_SCRIPT="${REPO_ROOT}/scripts/systemd/activate_geometry_runtime.sh"

PYTHON_BIN="${NUWAX_PYTHON_BIN:-${REPO_ROOT}/.venv/bin/python}"
if [[ -z "${NUWAX_PYTHON_BIN:-}" && ! -x "${PYTHON_BIN}" ]]; then
  PYTHON_BIN=/usr/bin/python3
fi

bar() { printf '\n========== %s ==========\n' "$1"; }
ok()  { printf '  [OK]      %s\n' "$1"; }
miss(){ printf '  [MISSING] %s\n' "$1"; }
nr()  { printf '  [NOT RUN] %s\n' "$1"; }

bar "1. 诊断输入"
ok "部署后诊断不依赖未跟踪的 scripts/ci/no_ros2_residual.sh"
printf '  deployment profile = %s\n' "${PROFILE_FILE}"

bar "2. motion guard:module / UDS / durable records"
GUARD_MODULE="${REPO_ROOT}/services/navigation/motion_safety_guard/__main__.py"
GUARD_SOCKET="${NUWAX_MOTION_GUARD_SOCKET:-/run/nuwax/motion-guard.sock}"
GUARD_LOCK="${NUWAX_MOTION_GUARD_LOCK:-/run/nuwax/motion-guard.lock}"
GUARD_STATE_DIR="${NUWAX_MOTION_GUARD_STATE_DIR:-/var/lib/nuwax/motion-guard}"
ACTUATOR_SYMBOLS_FILE="${NUWAX_ACTUATOR_SYMBOLS_FILE:-/etc/nuwax/actuator/actuator-symbols.json}"
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
  if [[ "${manifest_meta}" == "root:nuwax-motion-clients 640" ]]; then
    ok "actuator symbols manifest: ${ACTUATOR_SYMBOLS_FILE} (${manifest_meta})"
  else
    miss "actuator symbols manifest metadata drift: ${ACTUATOR_SYMBOLS_FILE} (${manifest_meta})"
  fi
else
  miss "actuator symbols manifest missing or empty: ${ACTUATOR_SYMBOLS_FILE}"
fi

bar "3. 二进制:nuwax-geometry-runtime"
RUNTIME_BIN="${NUWAX_GEOMETRY_RUNTIME_BINARY:-${REPO_ROOT}/services/geometry/nuwax_geometry_runtime/target/release/nuwax-geometry-runtime}"
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
for f in /tmp/nuwax_video.sock /tmp/nuwax_video.shm \
         "${GEOMETRY_DATA_DIR}/nuwax_cloudmap.mmap" \
         "${GEOMETRY_DATA_DIR}/nuwax_occupancy.mmap"; do
  if [[ -e "${f}" ]]; then
    ok "存在: ${f}"
  else
    nr "缺失: ${f}(未启用/未启动对应组件)"
  fi
done

bar "6. gRPC status (deployment Python + grpcio)"
printf '  selected interpreter = %s\n' "${PYTHON_BIN}"
GRPC_OUT="$(
  PYTHONPATH="${REPO_ROOT}" \
    "${PYTHON_BIN}" - <<'PY' 2>&1
import sys, os
print(f"sys.executable={sys.executable}")
try:
    import grpc  # noqa: F401
    from services.geometry.proto_gen import geometry_runtime_pb2 as pb
    from services.geometry.proto_gen import geometry_runtime_pb2_grpc as pbg
except Exception as e:
    print(f"GRPC_NOT_AVAILABLE: {e} (install grpcio==1.70.0 and protobuf==5.29.6 in this interpreter)")
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
         NUWAX_EXPLORE_ARBITRATION NUWAX_NAV_STACK_READY_FILE \
         NUWAX_REAL_LCM_REQUIRED NUWAX_GEOMETRY_RUNTIME_PORT \
         NUWAX_GEOMETRY_RUNTIME_READY_FILE NUWAX_GEOMETRY_EXTERNAL_ENABLED \
         NUWAX_GEOMETRY_RUNTIME_MANAGED_BY_SYSTEMD \
         NUWAX_NAV_STACK_MANAGED_BY_SYSTEMD \
         NUWAX_PERSON_FOLLOW_MANAGED_BY_SYSTEMD; do
  v="${!k:-<unset>}"
  printf '  %-32s = %s\n' "${k}" "${v}"
done

bar "8. systemd journal (last 20 lines)"
if command -v journalctl >/dev/null 2>&1; then
  journalctl -u nuwax-geometry-runtime.service -n 20 --no-pager 2>&1 | sed 's/^/  /'
else
  nr "journalctl 不可用"
fi

bar "9. LCM roundtrip (Phase 8 §12.6 硬约束)"
LCM_URL="${NUWAX_LCM_URL:-udpm://239.255.76.67:7667?ttl=1}"
if ! bash "${REPO_ROOT}/scripts/systemd/check_lcm_multicast.sh" "${LCM_URL}"; then
  nr "multicast route/interface 未就绪"
elif ! command -v pkg-config >/dev/null 2>&1 || ! pkg-config --modversion lcm >/dev/null 2>&1; then
  nr "liblcm-dev/pkg-config 不可用,跳过 roundtrip"
elif ! "${PYTHON_BIN}" -c 'import lcm' >/dev/null 2>&1; then
  nr "Python LCM 不可用: interpreter=${PYTHON_BIN}（安装: ${PYTHON_BIN} -m pip install lcm==1.5.2）"
else
  RT_LOG="$(mktemp)"
  RT_CHANNEL="/nuwax/diagnostics/lcm_roundtrip"
  RT_TOKEN="nuwax-lcm-roundtrip-$$"
  "${PYTHON_BIN}" - "${LCM_URL}" "${RT_CHANNEL}" "${RT_TOKEN}" \
      >"${RT_LOG}" 2>&1 <<'PY' &
import sys
import time

import lcm

url, channel, token = sys.argv[1:4]
client = lcm.LCM(url)
received = []


def on_message(observed_channel, payload):
    if observed_channel == channel and payload == token.encode("utf-8"):
        received.append(True)


subscription = client.subscribe(channel, on_message)
deadline = time.monotonic() + 5.0
while time.monotonic() < deadline and not received:
    client.handle_timeout(100)
try:
    client.unsubscribe(subscription)
except TypeError:
    client.unsubscribe(channel)
if received:
    print("subscriber received exact diagnostic token")
    sys.exit(0)
print("subscriber timed out waiting for exact diagnostic token", file=sys.stderr)
sys.exit(4)
PY
  RT_SUBSCRIBER_PID=$!
  sleep 0.5
  RT_PUBLISH_OUT="$("${PYTHON_BIN}" - "${LCM_URL}" "${RT_CHANNEL}" "${RT_TOKEN}" <<'PY' 2>&1
import sys
import time

import lcm

url, channel, token = sys.argv[1:4]
client = lcm.LCM(url)
payload = token.encode("utf-8")
for _ in range(20):
    client.publish(channel, payload)
    time.sleep(0.1)
print("publisher sent diagnostic token")
PY
)"
  wait "${RT_SUBSCRIBER_PID}"
  RT_SUBSCRIBER_RC=$?
  printf '%s\n' "${RT_PUBLISH_OUT}" | sed 's/^/    pub: /'
  sed 's/^/    sub: /' "${RT_LOG}"
  if [[ "${RT_SUBSCRIBER_RC}" -eq 0 ]]; then
    ok "LCM 跨进程 roundtrip PASS: interpreter=${PYTHON_BIN}"
  else
    miss "LCM 跨进程 roundtrip FAIL: subscriber_rc=${RT_SUBSCRIBER_RC} interpreter=${PYTHON_BIN}"
  fi
  rm -f "${RT_LOG}"
fi

bar "10. systemd units (production env + lazy activation contract)"
SERVICE_FILE="${REPO_ROOT}/deploy/systemd/nuwax-geometry-runtime.service"
GUARD_SERVICE_FILE="${REPO_ROOT}/deploy/systemd/nuwax-motion-safety-guard.service"
GATEWAY_SERVICE_FILE="${REPO_ROOT}/deploy/systemd/nuwax-host-gateway.service"
ACTIVATION_SERVICE_FILE="${REPO_ROOT}/deploy/systemd/nuwax-runtime-activation.service"
ACTIVATION_PATH_FILE="${REPO_ROOT}/deploy/systemd/nuwax-runtime-activation.path"
SYSTEMD_LIVE=false
if command -v systemctl >/dev/null 2>&1 && [[ -d /run/systemd/system ]]; then
  SYSTEMD_LIVE=true
  ok "检查 systemd 当前加载的 unit（包含 drop-in）"
else
  nr "systemd manager 不可用；仅执行源码模板离线检查"
fi
load_unit_source() {
  local unit_name="$1"
  local fallback_path="$2"
  if [[ "${SYSTEMD_LIVE}" == "true" ]]; then
    systemctl cat --no-pager "${unit_name}" 2>/dev/null
  else
    cat "${fallback_path}" 2>/dev/null
  fi
}
GUARD_UNIT_TEXT="$(load_unit_source nuwax-motion-safety-guard.service "${GUARD_SERVICE_FILE}")"
GEOMETRY_UNIT_TEXT="$(load_unit_source nuwax-geometry-runtime.service "${SERVICE_FILE}")"
GATEWAY_UNIT_TEXT="$(load_unit_source nuwax-host-gateway.service "${GATEWAY_SERVICE_FILE}")"
ACTIVATION_SERVICE_TEXT="$(load_unit_source nuwax-runtime-activation.service "${ACTIVATION_SERVICE_FILE}")"
ACTIVATION_PATH_TEXT="$(load_unit_source nuwax-runtime-activation.path "${ACTIVATION_PATH_FILE}")"
if [[ "${SYSTEMD_LIVE}" == "true" ]]; then
  for unit_name in nuwax-motion-safety-guard.service nuwax-nav-stack.service \
                   nuwax-geometry-runtime.service nuwax-person-follow.service \
                   nuwax-runtime-activation.service nuwax-runtime-activation.path \
                   nuwax-host-gateway.service nuwax-go2-video-streamer.service; do
    unit_details="$(systemctl show -p FragmentPath -p DropInPaths -p UnitFileState "${unit_name}" 2>&1)"
    if [[ $? -eq 0 && "${unit_details}" == *"FragmentPath="* ]]; then
      printf '  %s\n' "${unit_name}"
      printf '%s\n' "${unit_details}" | sed 's/^/    /'
    else
      miss "systemd effective unit unavailable: ${unit_name}: ${unit_details:-<empty>}"
    fi
  done
  if printf '%s\n' "${GUARD_UNIT_TEXT}" "${GEOMETRY_UNIT_TEXT}" \
      "${GATEWAY_UNIT_TEXT}" "${ACTIVATION_SERVICE_TEXT}" \
      "${ACTIVATION_PATH_TEXT}" | grep -q '@NUWAX_'; then
    miss "systemd effective unit 仍含未渲染 @NUWAX_* 占位符"
  else
    ok "systemd effective unit 不含未渲染模板占位符"
  fi
fi
if [[ -n "${GUARD_UNIT_TEXT}" ]]; then
  if grep -qE '^ExecStart=/usr/bin/env .* (@NUWAX_PYTHON_BIN@|/[^[:space:]]+) -m services\.navigation\.motion_safety_guard$' <<<"${GUARD_UNIT_TEXT}"; then
    ok "guard unit ExecStart uses rendered deployment Python"
  else
    miss "guard unit production ExecStart drift: expected /usr/bin/env ... <deployment Python> -m services.navigation.motion_safety_guard"
  fi
  if grep -qF 'Environment=NUWAX_MOTION_GUARD_SOCKET=/run/nuwax/motion-guard.sock' <<<"${GUARD_UNIT_TEXT}"; then
    ok "guard unit UDS path frozen"
  else
    miss "guard unit UDS path drift"
  fi
  if grep -qE '^Environment=NUWAX_ACTUATOR_SYMBOLS_FILE=.*/actuator/actuator-symbols.json$' <<<"${GUARD_UNIT_TEXT}"; then
    ok "guard unit actuator manifest path frozen"
  else
    miss "guard unit actuator manifest path drift"
  fi
else
  miss "guard effective unit unavailable"
fi
if [[ -n "${GEOMETRY_UNIT_TEXT}" ]]; then
  if grep -qE '^Environment=NUWAX_LCM_BACKEND=real-lcm' <<<"${GEOMETRY_UNIT_TEXT}"; then
    ok "service hardcode NUWAX_LCM_BACKEND=real-lcm"
  else
    miss "service 缺 NUWAX_LCM_BACKEND=real-lcm"
  fi
  if grep -qE '^Environment=NUWAX_REAL_LCM_REQUIRED=true' <<<"${GEOMETRY_UNIT_TEXT}"; then
    ok "service hardcode NUWAX_REAL_LCM_REQUIRED=true"
  else
    miss "service 缺 NUWAX_REAL_LCM_REQUIRED=true"
  fi
  if grep -qE '^SuccessExitStatus=0 4' <<<"${GEOMETRY_UNIT_TEXT}"; then
    miss "service 仍把 real-lcm fail-fast code(4) 当 SuccessExitStatus（违反 §13.2）"
  else
    ok "service SuccessExitStatus 仅 0"
  fi
  if grep -qF 'Requires=nuwax-motion-safety-guard.service' <<<"${GEOMETRY_UNIT_TEXT}"; then
    miss "geometry runtime 不应 eager Requires motion guard"
  else
    ok "geometry runtime 保持独立生命周期"
  fi
else
  nr "geometry effective unit unavailable"
fi
if [[ -n "${GATEWAY_UNIT_TEXT}" ]]; then
  if grep -qE '^Requires=.*nuwax-(motion-safety-guard|nav-stack|geometry-runtime|person-follow)\.service' <<<"${GATEWAY_UNIT_TEXT}"; then
    miss "Gateway 不应 eager Requires 按需运行时"
  else
    ok "Gateway 与按需运行时保持解耦"
  fi
else
  miss "Gateway effective unit unavailable"
fi

PROFILE_CONTRACT_OUT="$("${PYTHON_BIN}" - "${PROFILE_FILE}" <<'PY' 2>&1
import json
import sys

profile_path = sys.argv[1]
expected_always_on = {
    "nuwax-runtime-activation.path",
    "nuwax-host-gateway.service",
    "nuwax-go2-video-streamer.service",
}
expected_on_demand = {
    "nuwax-motion-safety-guard.service",
    "nuwax-nav-stack.service",
    "nuwax-geometry-runtime.service",
    "nuwax-person-follow.service",
    "nuwax-runtime-activation.service",
}
try:
    with open(profile_path, "r", encoding="utf-8") as stream:
        profile = json.load(stream)
    metadata = profile["metadata"]
    always_on = set(metadata["always_on_services"])
    on_demand = set(metadata["on_demand_services"])
    services = set(profile["services"])
    if always_on != expected_always_on:
        raise ValueError("always_on_services={}".format(sorted(always_on)))
    if on_demand != expected_on_demand:
        raise ValueError("on_demand_services={}".format(sorted(on_demand)))
    if always_on & on_demand or always_on | on_demand != services:
        raise ValueError("activation sets do not cover profile.services exactly")
    print("PROFILE_CONTRACT_OK")
except Exception as exc:
    print("PROFILE_CONTRACT_ERROR: {}".format(exc))
PY
)"
if [[ "${PROFILE_CONTRACT_OUT}" == "PROFILE_CONTRACT_OK" ]]; then
  ok "profile 常驻/按需服务契约完整"
else
  miss "profile 懒启动契约 drift: ${PROFILE_CONTRACT_OUT}"
fi

activation_lines=()
for unit_name in nuwax-motion-safety-guard.service nuwax-nav-stack.service \
                 nuwax-geometry-runtime.service nuwax-person-follow.service; do
  activation_line="$(grep -nF "start_unit_if_needed ${unit_name}" "${ACTIVATION_SCRIPT}" 2>/dev/null | cut -d: -f1 | head -n 1)"
  if [[ -z "${activation_line}" ]]; then
    activation_lines=()
    break
  fi
  activation_lines+=("${activation_line}")
done
if [[ "${#activation_lines[@]}" -eq 4 \
      && "${activation_lines[0]}" -lt "${activation_lines[1]}" \
      && "${activation_lines[1]}" -lt "${activation_lines[2]}" \
      && "${activation_lines[2]}" -lt "${activation_lines[3]}" ]]; then
  ok "activation 顺序: Guard -> Nav -> Geometry -> Person Follow"
else
  miss "activation 启动顺序 drift: ${activation_lines[*]:-<missing>}"
fi
if [[ -n "${ACTIVATION_SERVICE_TEXT}" && -n "${ACTIVATION_PATH_TEXT}" ]] \
    && ! grep -q '^\[Install\]$' <<<"${ACTIVATION_SERVICE_TEXT}" \
    && grep -q '^Unit=nuwax-runtime-activation.service$' <<<"${ACTIVATION_PATH_TEXT}" \
    && grep -q '^WantedBy=multi-user.target$' <<<"${ACTIVATION_PATH_TEXT}"; then
  ok "activation service 保持 static，path 保持可启用"
else
  miss "activation service/path 懒启动拓扑 drift"
fi

echo
echo "==> evidence bundle 完成 (Phase 8 / 2026-07-07)"
exit 0
